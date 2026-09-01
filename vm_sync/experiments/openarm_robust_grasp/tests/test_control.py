from pathlib import Path

import numpy as np
import pytest

from openarm_robust_grasp.control import (
    end_effector_rotation,
    end_effector_target_for_finger_center,
    execute_joint_move,
    finger_collision_center,
    fingertip_midpoint,
    forward_kinematics,
    solve_ik,
    solve_pose_ik,
)
from openarm_robust_grasp.model import load_scene


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")


def test_ik_reaches_current_end_effector_position():
    ctx = load_scene(str(SCENE))
    q0 = ctx.data.qpos[ctx.left.joint_qpos_ids].copy()
    target = forward_kinematics(ctx, ctx.left, q0)

    result = solve_ik(ctx, ctx.left, target, seed_q=q0)

    assert result.success
    assert result.error_m < 1e-3
    assert np.all(result.q >= ctx.left.lower_limits)
    assert np.all(result.q <= ctx.left.upper_limits)


def test_execute_joint_move_records_motion():
    ctx = load_scene(str(SCENE))
    q0 = ctx.data.qpos[ctx.left.joint_qpos_ids].copy()
    q_goal = np.clip(q0 + 0.02, ctx.left.lower_limits, ctx.left.upper_limits)

    log = execute_joint_move(ctx, ctx.left, q_goal, steps=5)

    assert log.q.shape == (5, 7)
    assert log.ee_pos.shape == (5, 3)
    assert log.final_error_rad >= 0.0


def test_execute_joint_move_calls_step_observer_after_each_mj_step():
    ctx = load_scene(str(SCENE))
    q0 = ctx.data.qpos[ctx.left.joint_qpos_ids].copy()
    q_goal = np.clip(q0 + 0.02, ctx.left.lower_limits, ctx.left.upper_limits)
    observed_times: list[float] = []

    def observer(observed_ctx):
        observed_times.append(float(observed_ctx.data.time))

    execute_joint_move(ctx, ctx.left, q_goal, steps=5, step_observer=observer)

    assert len(observed_times) == 5
    np.testing.assert_allclose(
        np.diff(observed_times),
        np.full(4, float(ctx.model.opt.timestep)),
    )
    np.testing.assert_allclose(
        observed_times,
        np.arange(1, 6, dtype=float) * float(ctx.model.opt.timestep),
    )
    assert observed_times[-1] == pytest.approx(float(ctx.data.time))


def test_execute_joint_move_noop_step_observer_preserves_trajectory():
    ctx_without_observer = load_scene(str(SCENE))
    ctx_with_noop_observer = load_scene(str(SCENE))
    q0_without_observer = ctx_without_observer.data.qpos[ctx_without_observer.left.joint_qpos_ids].copy()
    q0_with_noop_observer = ctx_with_noop_observer.data.qpos[ctx_with_noop_observer.left.joint_qpos_ids].copy()
    q_goal = np.clip(
        q0_without_observer + 0.02,
        ctx_without_observer.left.lower_limits,
        ctx_without_observer.left.upper_limits,
    )

    np.testing.assert_allclose(q0_without_observer, q0_with_noop_observer)

    log_without_observer = execute_joint_move(
        ctx_without_observer,
        ctx_without_observer.left,
        q_goal,
        steps=5,
    )
    log_with_noop_observer = execute_joint_move(
        ctx_with_noop_observer,
        ctx_with_noop_observer.left,
        q_goal,
        steps=5,
        step_observer=lambda observed_ctx: None,
    )

    np.testing.assert_allclose(log_without_observer.q, log_with_noop_observer.q)
    np.testing.assert_allclose(log_without_observer.ee_pos, log_with_noop_observer.ee_pos)
    np.testing.assert_allclose(log_without_observer.ctrl, log_with_noop_observer.ctrl)
    assert log_without_observer.final_error_rad == pytest.approx(log_with_noop_observer.final_error_rad)
    assert float(ctx_without_observer.data.time) == pytest.approx(float(ctx_with_noop_observer.data.time))
    np.testing.assert_allclose(ctx_without_observer.data.qpos, ctx_with_noop_observer.data.qpos)


def test_pose_ik_preserves_current_orientation_nearby():
    ctx = load_scene(str(SCENE))
    q0 = ctx.data.qpos[ctx.left.joint_qpos_ids].copy()
    target_pos = forward_kinematics(ctx, ctx.left, q0) + np.array([0.0, 0.0, 0.01])
    target_rot = end_effector_rotation(ctx, ctx.left)

    result = solve_pose_ik(ctx, ctx.left, target_pos, target_rot, seed_q=q0)

    assert result.success
    assert result.error_m < 2e-3


def test_fingertip_midpoint_is_between_fingers():
    ctx = load_scene(str(SCENE))

    midpoint = fingertip_midpoint(ctx, ctx.left)

    assert midpoint.shape == (3,)
    assert np.isfinite(midpoint).all()


def test_finger_collision_center_is_offset_from_ee_base():
    ctx = load_scene(str(SCENE))

    center = finger_collision_center(ctx, ctx.left)
    ee_pos = forward_kinematics(ctx, ctx.left, ctx.data.qpos[ctx.left.joint_qpos_ids])

    assert center.shape == (3,)
    assert np.isfinite(center).all()
    assert np.linalg.norm(center - ee_pos) > 0.05


def test_end_effector_target_for_finger_center_compensates_current_offset():
    ctx = load_scene(str(SCENE))
    desired_center = finger_collision_center(ctx, ctx.left) + np.array([0.01, -0.02, 0.03])

    target_ee = end_effector_target_for_finger_center(ctx, ctx.left, desired_center)

    current_offset = finger_collision_center(ctx, ctx.left) - forward_kinematics(
        ctx, ctx.left, ctx.data.qpos[ctx.left.joint_qpos_ids]
    )
    assert np.allclose(target_ee + current_offset, desired_center)
