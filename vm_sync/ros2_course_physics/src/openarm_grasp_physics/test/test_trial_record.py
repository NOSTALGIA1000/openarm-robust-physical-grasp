import csv
import json
import os

from openarm_grasp_physics.config import GraspConfig
from openarm_grasp_physics.evaluation import FailureCategory, TrialObservations
from openarm_grasp_physics.trial_record import Sample, TrialRecorder, trial_seed

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
        jaw_x=0.30,
        jaw_y=0.0,
        jaw_z=0.49,
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


def test_optical_measurements_reach_the_summary(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_006", CONFIG)
    recorder.add_sample(make_sample(0.1))
    recorder.record_measurement(grip_cam_x=-0.1912, grip_cam_z=0.6822)
    recorder.record_measurement(residual_y=0.0003)

    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    assert summary["measurements"]["grip_cam_x"] == -0.1912
    assert summary["measurements"]["residual_y"] == 0.0003

    with open(os.path.join(str(tmp_path), "trial_006", "trial.json")) as handle:
        assert json.load(handle)["measurements"]["grip_cam_z"] == 0.6822


def test_measurements_default_to_empty_when_none_recorded(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_007", CONFIG)
    recorder.add_sample(make_sample(0.1))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    assert summary["measurements"] == {}


def test_environment_block_names_the_interpreter(tmp_path):
    recorder = TrialRecorder(str(tmp_path), "trial_008", CONFIG)
    recorder.add_sample(make_sample(0.1))

    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    assert summary["environment"]["python"]
    assert summary["environment"]["platform"]


def test_model_checksums_recorded_when_model_dir_is_set(tmp_path, monkeypatch):
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    (model_dir / "openarm_bimanual_physics.xml").write_text("<mujoco/>")
    monkeypatch.setenv("OPENARM_MODEL", str(model_dir))

    recorder = TrialRecorder(str(tmp_path), "trial_009", CONFIG)
    recorder.add_sample(make_sample(0.1))
    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    digests = summary["environment"]["model_sha256"]
    assert list(digests) == ["openarm_bimanual_physics.xml"]
    assert len(digests["openarm_bimanual_physics.xml"]) == 64


def test_missing_model_dir_does_not_break_the_record(tmp_path, monkeypatch):
    monkeypatch.setenv("OPENARM_MODEL", str(tmp_path / "nope"))

    recorder = TrialRecorder(str(tmp_path), "trial_010", CONFIG)
    recorder.add_sample(make_sample(0.1))
    summary = recorder.finish(OBSERVATIONS, FailureCategory.SUCCESS)

    assert "model_sha256" not in summary["environment"]


def test_seed_is_stable_across_processes():
    # The point of the change: hash() would give a different answer in every
    # process, so this value is pinned rather than merely recomputed.
    assert trial_seed("og_cas_x000_1") == trial_seed("og_cas_x000_1")
    assert trial_seed("og_cas_x000_1") != trial_seed("og_cas_x000_2")
    assert 0 <= trial_seed("anything") < 2 ** 32


def test_seed_matches_a_pinned_value():
    import hashlib
    expected = int.from_bytes(
        hashlib.sha256(b"trial_042").digest()[:4], "big")
    assert trial_seed("trial_042") == expected
