import pytest

from openarm_grasp_physics.config import GraspConfig


def test_defaults_match_the_declared_stage_1_thresholds():
    config = GraspConfig()

    assert config.lift_height_m == 0.020
    assert config.hold_duration_s == 2.0
    assert config.min_bilateral_ratio == 0.80
    assert config.max_drift_m == 0.010
    assert config.min_normal_force == 0.5
    assert config.confirm_window_s == 0.30


def test_geom_names_match_the_physics_model():
    config = GraspConfig()

    assert config.object_geom == "physics_object_geom"
    assert config.left_finger_geom == "openarm_left_left_finger_collision"
    assert config.right_finger_geom == "openarm_left_right_finger_collision"


def test_config_is_immutable():
    config = GraspConfig()

    with pytest.raises(Exception):
        config.lift_height_m = 0.5


def test_snapshot_is_a_plain_dict_for_trial_records():
    snapshot = GraspConfig().snapshot()

    assert isinstance(snapshot, dict)
    assert snapshot["lift_height_m"] == 0.020


def test_servo_baseline_is_off_by_default():
    config = GraspConfig()

    assert config.servo_baseline is False
    assert config.relative_correction is False


def test_servo_defaults_are_a_usable_loop():
    config = GraspConfig()

    # A tolerance under the open-loop cliff, a gain that converges rather than
    # oscillates, and a probe long enough to move the reading measurably
    # against the detector's own noise of about 2mm.
    assert 0.0 < config.servo_tol_m < 0.024
    assert 0.0 < config.servo_gain <= 1.0
    assert config.servo_probe_m > 0.004
    assert config.servo_max_iters >= 3
    assert config.servo_max_step_m > config.servo_probe_m


def test_lift_command_is_separate_from_the_acceptance_threshold():
    config = GraspConfig()
    # The command reproduces the old lift_height_m + 0.01 so past batches are
    # unchanged, but the two can now move independently.
    assert config.lift_command_m == 0.030
    assert config.lift_height_m == 0.020
    assert config.lift_command_m != config.lift_height_m
