import math

from openarm_grasp_physics.geometry import distance, elevation, relative_position, shrink_radially, jacobian_from_probes, broyden_update, servo_step

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


def test_radial_shrink_is_a_no_op_without_gain():
    point = (-0.19, 0.021, 0.68)
    assert shrink_radially(point, 0.0) == point


def test_radial_shrink_pulls_towards_the_axis_by_gain_times_sine():
    # 3-4-5: lateral 0.03, depth 0.04, so sin(theta) = 0.6.
    x, y, z = shrink_radially((0.03, 0.0, 0.04), 0.01)
    assert z == 0.04
    assert y == 0.0
    assert abs(x - (0.03 - 0.01 * 0.6)) < 1e-12


def test_radial_shrink_leaves_a_reading_on_the_axis_alone():
    assert shrink_radially((0.0, 0.0, 0.68), 0.05) == (0.0, 0.0, 0.68)


def test_radial_shrink_never_crosses_the_axis():
    # The shrink is gain*sin(theta), and sin(theta) shrinks with the lateral
    # offset too, so overshooting needs a gain larger than the depth.
    x, y, _ = shrink_radially((0.001, 0.0, 0.68), 1.0)
    assert x == 0.0 and y == 0.0


def test_radial_shrink_of_a_near_axis_reading_stays_small():
    x, _, _ = shrink_radially((0.001, 0.0, 0.68), 0.05)
    assert 0.0 < x < 0.001


def test_jacobian_recovers_a_known_mapping():
    # The scene's own mapping: world x to camera x, world y flipped.
    true = ((1.0, 0.0), (0.0, -1.0))
    apply = lambda dp: (true[0][0] * dp[0] + true[0][1] * dp[1],
                        true[1][0] * dp[0] + true[1][1] * dp[1])
    a, b = (0.02, 0.0), (0.0, 0.02)
    jac = jacobian_from_probes(a, apply(a), b, apply(b))
    for i in range(2):
        for j in range(2):
            assert abs(jac[i][j] - true[i][j]) < 1e-9


def test_parallel_probes_are_refused():
    import pytest
    with pytest.raises(ValueError):
        jacobian_from_probes((0.02, 0.0), (0.02, 0.0),
                             (0.04, 0.0), (0.04, 0.0))


def test_broyden_absorbs_an_observation_it_mispredicted():
    jac = ((1.0, 0.0), (0.0, 1.0))
    dp, de = (0.02, 0.0), (0.03, 0.0)          #真实增益是 1.5，不是 1
    updated = broyden_update(jac, dp, de)
    assert abs(updated[0][0] - 1.5) < 1e-9
    assert abs(updated[1][1] - 1.0) < 1e-9     # 未被探测的方向不动


def test_broyden_ignores_a_zero_move():
    jac = ((1.0, 0.0), (0.0, 1.0))
    assert broyden_update(jac, (0.0, 0.0), (0.01, 0.01)) == jac


def test_servo_step_cancels_the_error_under_a_perfect_jacobian():
    jac = ((1.0, 0.0), (0.0, -1.0))
    dx, dy = servo_step(jac, (0.010, -0.006), gain=1.0, max_step=1.0)
    assert abs(dx - 0.010) < 1e-9
    assert abs(dy - 0.006) < 1e-9


def test_servo_step_is_clipped():
    jac = ((1.0, 0.0), (0.0, 1.0))
    dx, dy = servo_step(jac, (1.0, 0.0), gain=1.0, max_step=0.05)
    assert abs(math.hypot(dx, dy) - 0.05) < 1e-9


def test_singular_jacobian_is_refused():
    import pytest
    with pytest.raises(ValueError):
        servo_step(((1.0, 2.0), (2.0, 4.0)), (0.01, 0.01), 1.0, 0.05)
