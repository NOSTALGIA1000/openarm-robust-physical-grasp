import pytest

from openarm_grasp_physics.config import GraspConfig, arm_names


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


def test_arm_names_cover_every_arm_dependent_name():
    """The names a right-arm run needs, checked against the MuJoCo model.

    These were module constants in physics_picker.py naming the left arm. A
    right-arm run would have planned for left_arm, commanded the left gripper
    and filtered for left joint states while recording itself as the right,
    which is the defect class this project keeps hitting: a constant valid for
    one configuration applied to another.
    """
    left = arm_names("left")
    right = arm_names("right")

    assert left["group"] == "left_arm"
    assert left["ee_link"] == "openarm_left_hand_tcp"
    assert left["gripper_action"] == "/left_hand_controller/gripper_cmd"
    assert left["joint_prefix"] == "openarm_left_joint"
    assert left["gripper_body"] == "openarm_left_hand"
    assert left["left_finger_geom"] == "openarm_left_left_finger_collision"
    assert left["right_finger_geom"] == "openarm_left_right_finger_collision"
    assert left["branch_joint"] == "openarm_left_joint5"

    # The second word is the finger's side within the hand, not the arm, so
    # the right arm's pair is right_left_finger and right_right_finger. Getting
    # this wrong yields a geom that does not exist and a telemetry stream that
    # reports jaw_centre_valid false for every frame.
    assert right["left_finger_geom"] == "openarm_right_left_finger_collision"
    assert right["right_finger_geom"] == "openarm_right_right_finger_collision"
    assert right["group"] == "right_arm"
    assert right["ee_link"] == "openarm_right_hand_tcp"
    assert right["gripper_action"] == "/right_hand_controller/gripper_cmd"

    # No key may be shared between the two: anything that does not change with
    # the arm does not belong in here.
    assert not set(left.values()) & set(right.values())


def test_arm_names_refuses_anything_else():
    for bad in ("Left", "both", "", None):
        with pytest.raises(ValueError):
            arm_names(bad)


def test_arm_is_recorded_in_the_snapshot():
    assert GraspConfig().arm == "left"
    assert "arm" in GraspConfig().__dataclass_fields__
