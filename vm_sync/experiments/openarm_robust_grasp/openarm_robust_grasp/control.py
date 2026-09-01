from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import mujoco
import numpy as np

from openarm_robust_grasp.model import ArmIndex, SceneContext


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    error_m: float
    iterations: int


@dataclass(frozen=True)
class MotionLog:
    q: np.ndarray
    ee_pos: np.ndarray
    ctrl: np.ndarray
    final_error_rad: float


def _set_arm_qpos(ctx: SceneContext, arm: ArmIndex, q: np.ndarray) -> None:
    ctx.data.qpos[arm.joint_qpos_ids] = np.clip(q, arm.lower_limits, arm.upper_limits)
    mujoco.mj_forward(ctx.model, ctx.data)


def get_arm_qpos(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    return ctx.data.qpos[arm.joint_qpos_ids].copy()


def get_arm_qvel(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    return ctx.data.qvel[arm.joint_dof_ids].copy()


def forward_kinematics(ctx: SceneContext, arm: ArmIndex, q: np.ndarray) -> np.ndarray:
    original = get_arm_qpos(ctx, arm)
    _set_arm_qpos(ctx, arm, q)
    pos = ctx.data.xpos[arm.ee_body_id].copy()
    _set_arm_qpos(ctx, arm, original)
    return pos


def end_effector_position(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    mujoco.mj_forward(ctx.model, ctx.data)
    return ctx.data.xpos[arm.ee_body_id].copy()


def end_effector_rotation(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    mujoco.mj_forward(ctx.model, ctx.data)
    return ctx.data.xmat[arm.ee_body_id].reshape(3, 3).copy()


def fingertip_midpoint(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    inner_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{arm.prefix}_ee_inner_finger"
    )
    outer_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, f"openarm_{arm.prefix}_ee_outer_finger"
    )
    if inner_id < 0 or outer_id < 0:
        raise ValueError(f"Finger bodies not found for arm prefix: {arm.prefix}")
    mujoco.mj_forward(ctx.model, ctx.data)
    return 0.5 * (ctx.data.xpos[inner_id] + ctx.data.xpos[outer_id])


def _finger_collision_geom_ids(ctx: SceneContext, arm: ArmIndex) -> list[int]:
    geom_ids: list[int] = []
    for geom_id in range(ctx.model.ngeom):
        name = mujoco.mj_id2name(ctx.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith(f"finger_inner_{arm.prefix}_collision") or name.startswith(
            f"finger_outer_{arm.prefix}_collision"
        ):
            geom_ids.append(geom_id)
    if not geom_ids:
        raise ValueError(f"Finger collision geoms not found for arm prefix: {arm.prefix}")
    return geom_ids


def finger_collision_center(ctx: SceneContext, arm: ArmIndex) -> np.ndarray:
    geom_ids = _finger_collision_geom_ids(ctx, arm)
    mujoco.mj_forward(ctx.model, ctx.data)
    return np.mean([ctx.data.geom_xpos[geom_id] for geom_id in geom_ids], axis=0)


def end_effector_target_for_finger_center(
    ctx: SceneContext, arm: ArmIndex, desired_center: np.ndarray
) -> np.ndarray:
    mujoco.mj_forward(ctx.model, ctx.data)
    ee_pos = ctx.data.xpos[arm.ee_body_id].copy()
    current_center = finger_collision_center(ctx, arm)
    return np.asarray(desired_center, dtype=float) - (current_center - ee_pos)


def _rotation_error(target_rot: np.ndarray, current_rot: np.ndarray) -> np.ndarray:
    target_quat = np.zeros(4)
    current_quat = np.zeros(4)
    current_conj = np.zeros(4)
    error_quat = np.zeros(4)
    error_vel = np.zeros(3)
    mujoco.mju_mat2Quat(target_quat, np.asarray(target_rot, dtype=float).reshape(9))
    mujoco.mju_mat2Quat(current_quat, np.asarray(current_rot, dtype=float).reshape(9))
    mujoco.mju_negQuat(current_conj, current_quat)
    mujoco.mju_mulQuat(error_quat, target_quat, current_conj)
    mujoco.mju_quat2Vel(error_vel, error_quat, 1.0)
    return error_vel


def solve_ik(
    ctx: SceneContext,
    arm: ArmIndex,
    target_pos: np.ndarray,
    seed_q: np.ndarray | None = None,
    max_iters: int = 1000,
    tol: float = 1e-3,
    damping: float = 0.08,
) -> IKResult:
    original = get_arm_qpos(ctx, arm)
    q = original.copy() if seed_q is None else np.asarray(seed_q, dtype=float).copy()
    q = np.clip(q, arm.lower_limits, arm.upper_limits)
    jacp = np.zeros((3, ctx.model.nv))
    jacr = np.zeros((3, ctx.model.nv))
    best_q = q.copy()
    best_error = float("inf")

    for iteration in range(1, max_iters + 1):
        _set_arm_qpos(ctx, arm, q)
        err = np.asarray(target_pos, dtype=float) - ctx.data.xpos[arm.ee_body_id]
        error_m = float(np.linalg.norm(err))
        if error_m < best_error:
            best_error = error_m
            best_q = q.copy()
        if error_m < tol:
            _set_arm_qpos(ctx, arm, original)
            return IKResult(True, q.copy(), error_m, iteration)

        mujoco.mj_jacBody(ctx.model, ctx.data, jacp, jacr, arm.ee_body_id)
        j_pos = jacp[:, arm.joint_dof_ids]
        lhs = j_pos.T @ j_pos + damping * np.eye(len(arm.joint_dof_ids))
        rhs = j_pos.T @ err
        dq = np.linalg.solve(lhs, rhs)
        if float(np.linalg.norm(dq)) < 1e-10:
            break
        q = np.clip(q + np.clip(dq, -0.25, 0.25), arm.lower_limits, arm.upper_limits)

    _set_arm_qpos(ctx, arm, original)
    return IKResult(False, best_q, best_error, max_iters)


def solve_pose_ik(
    ctx: SceneContext,
    arm: ArmIndex,
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    seed_q: np.ndarray | None = None,
    max_iters: int = 1200,
    pos_tol: float = 2e-3,
    rot_tol: float = 3e-2,
    damping: float = 0.08,
    rot_weight: float = 0.35,
) -> IKResult:
    original = get_arm_qpos(ctx, arm)
    q = original.copy() if seed_q is None else np.asarray(seed_q, dtype=float).copy()
    q = np.clip(q, arm.lower_limits, arm.upper_limits)
    jacp = np.zeros((3, ctx.model.nv))
    jacr = np.zeros((3, ctx.model.nv))
    best_q = q.copy()
    best_error = float("inf")

    for iteration in range(1, max_iters + 1):
        _set_arm_qpos(ctx, arm, q)
        pos_err = np.asarray(target_pos, dtype=float) - ctx.data.xpos[arm.ee_body_id]
        rot_err = _rotation_error(target_rot, ctx.data.xmat[arm.ee_body_id].reshape(3, 3))
        pos_error_m = float(np.linalg.norm(pos_err))
        rot_error = float(np.linalg.norm(rot_err))
        combined_error = pos_error_m + rot_weight * rot_error
        if combined_error < best_error:
            best_error = combined_error
            best_q = q.copy()
        if pos_error_m < pos_tol and rot_error < rot_tol:
            _set_arm_qpos(ctx, arm, original)
            return IKResult(True, q.copy(), pos_error_m, iteration)

        mujoco.mj_jacBody(ctx.model, ctx.data, jacp, jacr, arm.ee_body_id)
        j_pos = jacp[:, arm.joint_dof_ids]
        j_rot = jacr[:, arm.joint_dof_ids]
        jac = np.vstack((j_pos, rot_weight * j_rot))
        err = np.concatenate((pos_err, rot_weight * rot_err))
        lhs = jac.T @ jac + damping * np.eye(len(arm.joint_dof_ids))
        rhs = jac.T @ err
        dq = np.linalg.solve(lhs, rhs)
        if float(np.linalg.norm(dq)) < 1e-10:
            break
        q = np.clip(q + np.clip(dq, -0.2, 0.2), arm.lower_limits, arm.upper_limits)

    _set_arm_qpos(ctx, arm, original)
    return IKResult(False, best_q, best_error, max_iters)


def set_arm_command(ctx: SceneContext, arm: ArmIndex, q_cmd: np.ndarray) -> None:
    ctrl = ctx.data.ctrl.copy()
    ctrl[arm.actuator_ids] = np.clip(q_cmd, arm.lower_limits, arm.upper_limits)
    ctx.data.ctrl[:] = ctrl


def set_gripper(ctx: SceneContext, arm: ArmIndex, value: float) -> None:
    ctx.data.ctrl[arm.gripper_actuator_id] = value


def execute_joint_move(
    ctx: SceneContext,
    arm: ArmIndex,
    q_goal: np.ndarray,
    steps: int = 80,
    kp: float = 12.0,
    kd: float = 0.3,
    step_observer: Callable[[SceneContext], None] | None = None,
) -> MotionLog:
    q_start = get_arm_qpos(ctx, arm)
    q_goal = np.clip(np.asarray(q_goal, dtype=float), arm.lower_limits, arm.upper_limits)
    q_log = np.zeros((steps, len(arm.joint_qpos_ids)))
    ee_log = np.zeros((steps, 3))
    ctrl_log = np.zeros((steps, len(arm.actuator_ids)))

    for step in range(steps):
        phase = step / max(steps - 1, 1)
        smooth = 0.5 - 0.5 * np.cos(np.pi * phase)
        q_des = q_start + smooth * (q_goal - q_start)
        q = get_arm_qpos(ctx, arm)
        qvel = get_arm_qvel(ctx, arm)
        q_cmd = np.clip(q + kp * (q_des - q) - kd * qvel, arm.lower_limits, arm.upper_limits)
        set_arm_command(ctx, arm, q_cmd)
        mujoco.mj_step(ctx.model, ctx.data)
        if step_observer is not None:
            step_observer(ctx)
        q_log[step] = get_arm_qpos(ctx, arm)
        ee_log[step] = end_effector_position(ctx, arm)
        ctrl_log[step] = ctx.data.ctrl[arm.actuator_ids]

    return MotionLog(
        q=q_log,
        ee_pos=ee_log,
        ctrl=ctrl_log,
        final_error_rad=float(np.linalg.norm(get_arm_qpos(ctx, arm) - q_goal)),
    )
