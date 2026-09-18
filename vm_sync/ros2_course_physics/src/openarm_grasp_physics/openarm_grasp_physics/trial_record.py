"""Per-trial data products: raw samples, machine summary, human summary."""

import csv
import dataclasses
import hashlib
import json
import os
import platform
import subprocess
import sys
import xml.etree.ElementTree as ElementTree
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
    # Midpoint of the two finger collision geoms - where the jaws meet. The
    # gripper_* above is the link origin, whose offset from the jaws depends on
    # the whole arm pose: measured at zero bias it is +7.65mm in x for the
    # open-loop pipeline and +21.45mm for the corrected one, so an x offset
    # taken from it cannot be compared across pipelines, arms or approach
    # poses. NaN on trials recorded before 2026-09-16 or when the telemetry
    # could not find the finger geoms.
    jaw_x: float
    jaw_y: float
    jaw_z: float
    elevation: float
    drift: float
    # Every contact with the object that is not one of the two fingers, with
    # the name and force of the strongest. The table is normally one of them.
    # Zero and empty on trials recorded before 2026-09-18, where a link passing
    # through the object left no trace in the record at all.
    other_contacts: int = 0
    other_geom: str = ""
    other_force: float = 0.0


def trial_seed(trial_id: str) -> int:
    """A seed that is actually a function of the trial id.

    Python salts string hashing per process unless PYTHONHASHSEED is set, so
    the previous seed - hash(trial_id) - took a different value on every run
    while reading as though it were determined by the trial. None of the
    recorded noise realisations can be reproduced; from here they can.
    """
    return int.from_bytes(hashlib.sha256(trial_id.encode()).digest()[:4], "big")


def _sha256(path: str) -> str:
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


SCENE_DEFAULT = "table_scene_physics.xml"


def _object_half_extents(scene_path: str, geom_name: str):
    """Half-extents of the graspable object, read from the scene that ran.

    Needed because trial.json recorded the object only by geom NAME, and
    detector_z_bias - the one numeric clue - is half the object's HEIGHT plus
    the detector's 5.5mm surface offset. Height fixes the width only for a
    cube: the 30x30x44mm cuboid the scene carried before 2026-09-11 has a 22mm
    half-height and a 30mm jaw-axis width. So the jaw-axis clearance, which
    the 2026-09-13 criterion is stated against, could not be computed from the
    record and had to come from a lookup table keyed on detector_z_bias.
    """
    root = ElementTree.parse(scene_path).getroot()
    for geom in root.iter("geom"):
        if geom.get("name") != geom_name:
            continue
        size = geom.get("size")
        if not size:
            return None
        return [float(v) for v in size.split()]
    return None


def describe_environment(object_geom: str = "") -> dict:
    """What was running when the trial was produced.

    Only one result document in this project ever recorded an environment -
    the 2026-08-22 stability screen, which stored a scene checksum and made
    itself the sole batch whose inputs can be confirmed after the fact. Every
    ROS2-phase batch since has been reconstructed from prose. The commit comes
    from OPENARM_COMMIT when the runner sets it, else from git if the source
    tree is a checkout; the model checksum from OPENARM_MODEL. Each part is
    optional, because a missing checksum is worth less than a refused trial.
    """
    env = {"python": platform.python_version(), "platform": platform.platform()}

    commit = os.environ.get("OPENARM_COMMIT", "").strip()
    if not commit:
        try:
            commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=os.path.dirname(os.path.abspath(__file__)),
                capture_output=True, text=True, timeout=5,
            ).stdout.strip()
        except Exception:
            commit = ""
    if commit:
        env["commit"] = commit

    try:
        import mujoco
        env["mujoco"] = mujoco.__version__
    except Exception:
        pass

    model_dir = os.environ.get("OPENARM_MODEL", "")
    if model_dir:
        digests = {}
        # The scene that actually ran, not a fixed name. PHYSICS_SCENE selects
        # a generated variant, and hashing table_scene_physics.xml regardless
        # left every variant batch - stage52_cube20, stage53_noise20,
        # stage51_offseterror - with a checksum for a file it did not use. That
        # is worse than no checksum, because it reads as if the scene were
        # pinned when the scene in play was unhashed.
        scene = os.environ.get("PHYSICS_SCENE", SCENE_DEFAULT)
        env["scene"] = scene
        for name in ("openarm_bimanual_physics.xml", scene):
            path = os.path.join(model_dir, name)
            try:
                digests[name] = _sha256(path)
            except OSError:
                pass
        if digests:
            env["model_sha256"] = digests

        if object_geom:
            try:
                extents = _object_half_extents(
                    os.path.join(model_dir, scene), object_geom)
            except (OSError, ElementTree.ParseError, ValueError):
                extents = None
            if extents:
                env["object_half_extents_m"] = extents

    # The hand-eye error is injected into the world->camera_base_link static
    # transform by the launch file, which reads these same two variables. It is
    # therefore an environment fact, not a picker parameter, and until now it
    # was recorded nowhere: config.calib_bias_x/y stay 0.0 for every trial run
    # this way, so the ceiling batches (240/288/320/384mm) carry their own
    # independent variable only in the directory name. A reanalysis that trusts
    # the config gets 0mm for all of them and groups them together, silently.
    for var, key in (("CAM_BIAS_X", "cam_bias_x"), ("CAM_BIAS_Y", "cam_bias_y")):
        raw = os.environ.get(var)
        if raw is None:
            continue
        try:
            env[key] = float(raw)
        except ValueError:
            env[key] = raw
    return env


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
            "environment": describe_environment(self._config.object_geom),
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
