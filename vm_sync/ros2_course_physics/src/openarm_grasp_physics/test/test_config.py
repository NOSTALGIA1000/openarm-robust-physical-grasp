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
