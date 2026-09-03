import csv
import json
import os

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import FailureCategory, TrialObservations
from openarm_grasp_physics.trial_record import Sample, TrialRecorder

CONFIG = GraspConfig()

OBSERVATIONS = TrialObservations(
    planning_ok=True,
    gripper_ok=True,
    timed_out=False,
    any_contact=True,
    bilateral_confirmed=True,
    contact_lost_during_lift=False,
    max_elevation_m=0.024,
    hold_completed=True,
    hold_bilateral_ratio=0.93,
    max_drift_m=0.003,
)


def make_sample(sim_time, stage="hold"):
    return Sample(
        sim_time=sim_time,
        stage=stage,
        left_contact=True,
        right_contact=True,
        left_force=4.0,
        right_force=3.5,
        object_x=0.30,
        object_y=0.0,
        object_z=0.469,
        object_qw=1.0,
        object_qx=0.0,
        object_qy=0.0,
        object_qz=0.0,
        gripper_x=0.30,
        gripper_y=0.0,
        gripper_z=0.55,
        elevation=0.024,
        drift=0.003,
    )


def test_finish_writes_all_three_products(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_001", CONFIG)
    recorder.mark_stage("close", 1.0)
    recorder.add_sample(make_sample(1.02))
    recorder.add_sample(make_sample(1.04))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    trial_dir = os.path.join(str(tmp_path), "trial_001")
    assert sorted(os.listdir(trial_dir)) == ["report.md", "samples.csv", "trial.json"]
    assert summary["success"] is True
    assert summary["category"] == "success"


def test_samples_csv_has_one_row_per_sample(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_002", CONFIG)
    recorder.add_sample(make_sample(0.5))
    recorder.add_sample(make_sample(0.6))
    recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    with open(os.path.join(str(tmp_path), "trial_002", "samples.csv")) as handle:
        rows = list(csv.DictReader(handle))

    assert len(rows) == 2
    assert rows[0]["sim_time"] == "0.5"
    assert rows[1]["stage"] == "hold"


def test_trial_json_embeds_the_config_snapshot_and_stage_times(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_003", CONFIG)
    recorder.mark_stage("lift", 3.25)
    recorder.add_sample(make_sample(3.3))
    recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    with open(os.path.join(str(tmp_path), "trial_003", "trial.json")) as handle:
        payload = json.load(handle)

    assert payload["config"]["lift_height_m"] == 0.020
    assert payload["stage_times"]["lift"] == 3.25
    assert payload["observations"]["max_elevation_m"] == 0.024
    assert payload["sample_count"] == 1


def test_failure_is_recorded_as_not_successful(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_004", CONFIG)
    recorder.add_sample(make_sample(0.1))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.EXCESSIVE_SLIP)

    assert summary["success"] is False
    assert summary["category"] == "excessive_slip"


def test_report_markdown_names_the_category(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_005", CONFIG)
    recorder.add_sample(make_sample(0.1))
    recorder.finish(OBSERVATIONS, FailureCategory.INSUFFICIENT_LIFT)

    with open(os.path.join(str(tmp_path), "trial_005", "report.md")) as handle:
        report = handle.read()

    assert "insufficient_lift" in report
    assert "trial_005" in report
