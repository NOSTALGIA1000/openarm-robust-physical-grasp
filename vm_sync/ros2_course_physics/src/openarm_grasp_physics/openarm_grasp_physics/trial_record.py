"""Per-trial data products: raw samples, machine summary, human summary."""

import csv
import dataclasses
import json
import os
from dataclasses import dataclass
from typing import Dict, List

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import FailureCategory, TrialObservations


@dataclass(frozen=True)
class Sample:
    sim_time: float
    stage: str
    left_contact: bool
    right_contact: bool
    left_force: float
    right_force: float
    object_x: float
    object_y: float
    object_z: float
    object_qw: float
    object_qx: float
    object_qy: float
    object_qz: float
    gripper_x: float
    gripper_y: float
    gripper_z: float
    elevation: float
    drift: float


class TrialRecorder:
    def __init__(self, output_dir: str, trial_id: str, config: GraspConfig):
        self._trial_dir = os.path.join(output_dir, trial_id)
        self._trial_id = trial_id
        self._config = config
        self._samples: List[Sample] = []
        self._stage_times: Dict[str, float] = {}
        self._measurements: Dict[str, float] = {}

    def add_sample(self, sample: Sample) -> None:
        self._samples.append(sample)

    def record_measurement(self, **values: float) -> None:
        """Persist the optical measurements the correction is computed from.

        Until this existed the camera-frame readings appeared only in the
        node's log, so a finished batch could not be re-analysed for anything
        the log had not already been asked to print - the whole method rests on
        these two vectors and none of the 470 recorded trials contains them.
        """
        self._measurements.update({k: float(v) for k, v in values.items()})

    def mark_stage(self, stage: str, sim_time: float) -> None:
        self._stage_times[stage] = float(sim_time)

    def finish(
        self, observations: TrialObservations, category: FailureCategory
    ) -> dict:
        os.makedirs(self._trial_dir, exist_ok=True)
        self._write_samples()

        summary = {
            "trial_id": self._trial_id,
            "mode": "physics_mode",
            "category": category.value,
            "success": category is FailureCategory.SUCCESS,
            "sample_count": len(self._samples),
            "stage_times": self._stage_times,
            "observations": dataclasses.asdict(observations),
            "measurements": self._measurements,
            "config": self._config.snapshot(),
        }

        with open(os.path.join(self._trial_dir, "trial.json"), "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)

        self._write_report(summary)
        return summary

    def _write_samples(self) -> None:
        fields = [field.name for field in dataclasses.fields(Sample)]
        path = os.path.join(self._trial_dir, "samples.csv")
        with open(path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for sample in self._samples:
                writer.writerow(dataclasses.asdict(sample))

    def _write_report(self, summary: dict) -> None:
        observations = summary["observations"]
        lines = [
            "# Physics-mode grasp trial {}".format(self._trial_id),
            "",
            "- Result: `{}`".format(summary["category"]),
            "- Success: `{}`".format(summary["success"]),
            "- Samples: `{}`".format(summary["sample_count"]),
            "- Max elevation: `{:.4f} m` (threshold `{:.3f} m`)".format(
                observations["max_elevation_m"], self._config.lift_height_m
            ),
            "- Hold bilateral ratio: `{:.3f}` (threshold `{:.2f}`)".format(
                observations["hold_bilateral_ratio"], self._config.min_bilateral_ratio
            ),
            "- Max relative drift: `{:.4f} m` (threshold `{:.3f} m`)".format(
                observations["max_drift_m"], self._config.max_drift_m
            ),
            "",
            "## Stage times (simulation seconds)",
            "",
        ]
        for stage, sim_time in sorted(
            summary["stage_times"].items(), key=lambda item: item[1]
        ):
            lines.append("- `{}`: `{:.3f}`".format(stage, sim_time))
        lines.append("")
        lines.append(
            "No software attachment and no object pose command exist in this mode; "
            "object motion is produced only by contact."
        )
        lines.append("")

        with open(os.path.join(self._trial_dir, "report.md"), "w") as handle:
            handle.write("\n".join(lines))
