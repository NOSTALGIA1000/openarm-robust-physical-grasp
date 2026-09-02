import math

from openarm_grasp_physics.geometry import distance, elevation, relative_position

IDENTITY = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))


def assert_vec_close(actual, expected, abs_tol=1e-9):
    for a, e in zip(actual, expected):
        assert math.isclose(a, e, abs_tol=abs_tol), f"{actual} != {expected}"


def test_identity_rotation_is_a_plain_offset():
    result = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)

    # 0.5 - 0.6 is not exactly representable in binary64 (it lands near
    # -0.09999999999999998, not the literal -0.1), so this compares with a
    # tolerance rather than "==" — the geometry, not raw bit patterns, is
    # what this test locks in.
    assert_vec_close(result, (0.0, 0.0, -0.1))


def test_gripper_rotation_is_applied():
    result = relative_position((1.0, 0.0, 0.0), (0.0, 0.0, 0.0), YAW_90)

    # sqrt(0.5) is irrational; its rotation-matrix terms carry a residual on
    # the order of 1e-16 even for the "should be exactly zero" components, so
    # this compares with a tolerance rather than "==".
    assert_vec_close(result, (0.0, -1.0, 0.0))


def test_distance_is_euclidean():
    assert distance((0.0, 0.0, 0.0), (3.0, 4.0, 0.0)) == 5.0


def test_zero_drift_when_relative_pose_is_unchanged():
    grasp = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)
    lifted = relative_position((0.3, 0.0, 0.55), (0.3, 0.0, 0.65), IDENTITY)

    assert math.isclose(distance(grasp, lifted), 0.0, abs_tol=1e-9)


def test_slip_shows_up_as_relative_drift():
    grasp = relative_position((0.3, 0.0, 0.5), (0.3, 0.0, 0.6), IDENTITY)
    slipped = relative_position((0.3, 0.0, 0.49), (0.3, 0.0, 0.65), IDENTITY)

    assert math.isclose(distance(grasp, slipped), 0.06, rel_tol=1e-9)


def test_elevation_is_measured_from_the_settled_baseline():
    assert math.isclose(elevation(0.465, 0.445), 0.020, rel_tol=1e-9)
