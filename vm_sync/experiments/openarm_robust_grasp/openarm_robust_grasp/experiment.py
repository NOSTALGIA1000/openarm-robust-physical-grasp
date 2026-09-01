from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import lru_cache

import mujoco
import numpy as np

from openarm_robust_grasp.control import (
    end_effector_position,
    end_effector_target_for_finger_center,
    end_effector_rotation,
    execute_joint_move,
    get_arm_qpos,
    set_gripper,
    solve_ik,
    solve_pose_ik,
)
from openarm_robust_grasp.grasp_diagnostics import (
    GripperCalibration,
    infer_gripper_calibration,
    run_gripper_sweep,
)
from openarm_robust_grasp.model import load_scene


CONTACT_POLICIES = {
    "contact_push",
    "ee_base_contact_push",
    "finger_center_contact_push",
    "pre_grasp_finger_center_correction",
    "cube_lift",
}
ALLOWED_POLICIES = {"open_loop", "pre_grasp_correction"} | CONTACT_POLICIES


@dataclass(frozen=True)
class NoiseConfig:
    translation_std_m: float = 0.0
    fixed_bias_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    command_noise_std_rad: float = 0.0


@dataclass(frozen=True)
class ContactFrame:
    contact_pairs: int
    left_finger_contact: bool
    right_finger_contact: bool
    any_finger_contact: bool
    two_finger_contact: bool


@dataclass
class StageContactStats:
    sampled_frames: int = 0
    contact_frames: int = 0
    two_finger_frames: int = 0

    def observe(self, frame: ContactFrame) -> None:
        self.sampled_frames += 1
        if frame.any_finger_contact:
            self.contact_frames += 1
        if frame.two_finger_contact:
            self.two_finger_frames += 1

    @property
    def contact_frame_rate(self) -> float:
        if self.sampled_frames == 0:
            return 0.0
        return self.contact_frames / self.sampled_frames

    @property
    def two_finger_contact_frame_rate(self) -> float:
        if self.sampled_frames == 0:
            return 0.0
        return self.two_finger_frames / self.sampled_frames


@dataclass(frozen=True)
class TrialResult:
    policy: str
    seed: int
    translation_std_m: float
    fixed_bias_x_m: float
    fixed_bias_y_m: float
    fixed_bias_z_m: float
    command_noise_std_rad: float
    ik_success: bool
    proxy_success: bool
    contact_success: bool
    grasp_success: bool
    final_ee_error_m: float
    min_ee_object_distance_m: float
    final_ee_object_distance_m: float
    object_motion_m: float
    object_lift_m: float
    finger_object_contacts: int
    approach_finger_object_contacts: int
    grasp_finger_object_contacts: int
    lift_finger_object_contacts: int
    approach_sampled_frames: int
    grasp_sampled_frames: int
    lift_sampled_frames: int
    approach_contact_frames: int
    grasp_contact_frames: int
    lift_contact_frames: int
    approach_two_finger_contact_frames: int
    grasp_two_finger_contact_frames: int
    lift_two_finger_contact_frames: int
    approach_contact_frame_rate: float
    grasp_contact_frame_rate: float
    lift_contact_frame_rate: float
    grasp_two_finger_contact_success: bool
    lift_two_finger_contact_success: bool
    sustained_lift_contact_success: bool
    post_grasp_contact_success: bool
    lift_contact_success: bool
    lift_height_m: float
    steps: int

    def to_row(self) -> dict[str, object]:
        return asdict(self)


def _object_pos(ctx, object_name: str) -> np.ndarray:
    return ctx.data.xpos[ctx.objects[object_name]].copy()


def _geom_name(ctx, geom_id: int) -> str:
    return mujoco.mj_id2name(ctx.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""


def _object_geom_ids(ctx, object_name: str) -> set[int]:
    body_id = ctx.objects[object_name]
    root_id = ctx.model.body_rootid[body_id]
    geom_ids: set[int] = set()
    for geom_id in range(ctx.model.ngeom):
        geom_body = int(ctx.model.geom_bodyid[geom_id])
        if geom_body == body_id or int(ctx.model.body_rootid[geom_body]) == root_id:
            geom_ids.add(geom_id)
    return geom_ids


def _finger_side_geom_ids(ctx, arm_prefix: str) -> tuple[set[int], set[int]]:
    left_prefix = f"finger_inner_{arm_prefix}_collision"
    right_prefix = f"finger_outer_{arm_prefix}_collision"
    left_ids: set[int] = set()
    right_ids: set[int] = set()
    ambiguous_names: list[str] = []

    for geom_id in range(ctx.model.ngeom):
        name = _geom_name(ctx, geom_id)
        if arm_prefix not in name or "finger" not in name or "collision" not in name:
            continue
        if name.startswith(left_prefix):
            left_ids.add(geom_id)
            continue
        if name.startswith(right_prefix):
            right_ids.add(geom_id)
            continue
        ambiguous_names.append(name)

    if ambiguous_names:
        raise ValueError(
            f"Ambiguous finger collision geom grouping for arm prefix '{arm_prefix}': {ambiguous_names}"
        )
    if not left_ids:
        raise ValueError(f"Missing inner finger collision geoms for arm prefix '{arm_prefix}'")
    if not right_ids:
        raise ValueError(f"Missing outer finger collision geoms for arm prefix '{arm_prefix}'")

    return left_ids, right_ids


def _validated_finger_side_geom_ids(
    finger_side_geom_ids: tuple[set[int], set[int]],
) -> tuple[set[int], set[int]]:
    left_geom_ids = set(finger_side_geom_ids[0])
    right_geom_ids = set(finger_side_geom_ids[1])
    if not left_geom_ids:
        raise ValueError("Missing inner finger side geom ids")
    if not right_geom_ids:
        raise ValueError("Missing outer finger side geom ids")
    overlap = left_geom_ids & right_geom_ids
    if overlap:
        raise ValueError(f"Overlapping finger side geom ids: {sorted(overlap)}")
    return left_geom_ids, right_geom_ids


def _sample_contact_frame(
    ctx,
    finger_side_geom_ids: tuple[set[int], set[int]],
    object_geom_ids: set[int],
) -> ContactFrame:
    left_geom_ids, right_geom_ids = _validated_finger_side_geom_ids(finger_side_geom_ids)
    contact_pairs: set[tuple[int, int]] = set()
    left_finger_contact = False
    right_finger_contact = False

    for contact_id in range(ctx.data.ncon):
        contact = ctx.data.contact[contact_id]
        g1 = int(contact.geom1)
        g2 = int(contact.geom2)
        if g1 in object_geom_ids:
            object_geom_id = g1
            finger_geom_id = g2
        elif g2 in object_geom_ids:
            object_geom_id = g2
            finger_geom_id = g1
        else:
            continue

        if finger_geom_id in left_geom_ids:
            left_finger_contact = True
        elif finger_geom_id in right_geom_ids:
            right_finger_contact = True
        else:
            continue

        contact_pairs.add(tuple(sorted((object_geom_id, finger_geom_id))))

    any_finger_contact = left_finger_contact or right_finger_contact
    two_finger_contact = left_finger_contact and right_finger_contact
    return ContactFrame(
        contact_pairs=len(contact_pairs),
        left_finger_contact=left_finger_contact,
        right_finger_contact=right_finger_contact,
        any_finger_contact=any_finger_contact,
        two_finger_contact=two_finger_contact,
    )


def _finger_object_contact_count(ctx, arm_prefix: str, object_geom_ids: set[int]) -> int:
    pairs: set[tuple[int, int]] = set()
    for contact_id in range(ctx.data.ncon):
        contact = ctx.data.contact[contact_id]
        g1 = int(contact.geom1)
        g2 = int(contact.geom2)
        if g1 in object_geom_ids:
            other = g2
            obj = g1
        elif g2 in object_geom_ids:
            other = g1
            obj = g2
        else:
            continue
        other_name = _geom_name(ctx, other)
        if "finger" in other_name and arm_prefix in other_name:
            pairs.add(tuple(sorted((obj, other))))
    return len(pairs)


def _perceived_position(true_pos: np.ndarray, noise: NoiseConfig, rng: np.random.Generator) -> np.ndarray:
    return (
        true_pos
        + np.array(noise.fixed_bias_m, dtype=float)
        + rng.normal(0.0, noise.translation_std_m, size=3)
    )


def _apply_command_noise(q: np.ndarray, std_rad: float, rng: np.random.Generator) -> np.ndarray:
    if std_rad <= 0:
        return q
    return q + rng.normal(0.0, std_rad, size=q.shape)


def _finger_center_contact_waypoints(ctx, arm, perceived: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # The OpenArm EE base is offset from the actual finger collision geometry.
    # For contact experiments, target the mean finger collision center rather
    # than the EE base, then convert back to the EE target used by IK.
    grasp_center = np.asarray(perceived, dtype=float) + np.array([0.0, 0.0, 0.02])
    approach_center = grasp_center + np.array([0.0, 0.0, 0.08])
    lift_center = grasp_center + np.array([0.0, 0.0, 0.16])
    return (
        end_effector_target_for_finger_center(ctx, arm, approach_center),
        end_effector_target_for_finger_center(ctx, arm, grasp_center),
        end_effector_target_for_finger_center(ctx, arm, lift_center),
    )


def _ee_base_contact_waypoints(perceived: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Legacy baseline: Cartesian offsets are applied directly to the OpenArm EE
    # base. This is intentionally kept for comparison against finger-center
    # planning, because the EE base is not the physical contact point.
    contact_offset = np.array([-0.04, 0.03, 0.02])
    approach = np.asarray(perceived, dtype=float) + contact_offset + np.array([0.0, 0.0, 0.08])
    grasp = np.asarray(perceived, dtype=float) + contact_offset
    lift = np.asarray(perceived, dtype=float) + contact_offset + np.array([0.0, 0.0, 0.16])
    return approach, grasp, lift


def _move_to(
    ctx,
    arm,
    target_pos: np.ndarray,
    rng: np.random.Generator,
    noise: NoiseConfig,
    steps: int = 80,
    step_observer=None,
) -> tuple[bool, float, int]:
    q_seed = get_arm_qpos(ctx, arm)
    ik = solve_ik(ctx, arm, target_pos, seed_q=q_seed)
    if not ik.success:
        return False, ik.error_m, 0
    q_goal = _apply_command_noise(ik.q, noise.command_noise_std_rad, rng)
    log = execute_joint_move(ctx, arm, q_goal, steps=steps, step_observer=step_observer)
    return True, log.final_error_rad, len(log.q)


def _move_to_pose(
    ctx,
    arm,
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    rng: np.random.Generator,
    noise: NoiseConfig,
    steps: int = 120,
    step_observer=None,
) -> tuple[bool, float, int]:
    q_seed = get_arm_qpos(ctx, arm)
    ik = solve_pose_ik(ctx, arm, target_pos, target_rot, seed_q=q_seed)
    if not ik.success:
        return False, ik.error_m, 0
    q_goal = _apply_command_noise(ik.q, noise.command_noise_std_rad, rng)
    log = execute_joint_move(ctx, arm, q_goal, steps=steps, step_observer=step_observer)
    return True, log.final_error_rad, len(log.q)


@lru_cache(maxsize=None)
def _gripper_calibration_for_scene(scene_path: str, arm_name: str) -> GripperCalibration:
    return infer_gripper_calibration(run_gripper_sweep(scene_path, arm_name=arm_name))


def _calibrated_gripper_control_endpoints(ctx, arm) -> tuple[float, float]:
    ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[arm.gripper_actuator_id]
    calibration = _gripper_calibration_for_scene(ctx.scene_path, arm.prefix)
    if calibration.command_increases_aperture:
        return float(ctrl_min), float(ctrl_max)
    return float(ctrl_max), float(ctrl_min)


def run_trial(
    scene_path: str,
    policy: str,
    noise: NoiseConfig,
    seed: int,
    object_name: str = "bottle",
) -> TrialResult:
    if policy not in ALLOWED_POLICIES:
        raise ValueError(f"Unknown policy: {policy}")

    rng = np.random.default_rng(seed)
    ctx = load_scene(scene_path)
    arm = ctx.left
    mujoco.mj_forward(ctx.model, ctx.data)

    true_object = _object_pos(ctx, object_name)
    initial_object = true_object.copy()
    object_geoms = _object_geom_ids(ctx, object_name)
    finger_side_geom_ids = _finger_side_geom_ids(ctx, arm.prefix)
    perceived = _perceived_position(true_object, noise, rng)
    target_rot = end_effector_rotation(ctx, arm)

    if policy == "ee_base_contact_push":
        approach, grasp, lift = _ee_base_contact_waypoints(perceived)
    elif policy in {
        "contact_push",
        "finger_center_contact_push",
        "pre_grasp_finger_center_correction",
        "cube_lift",
    }:
        approach, grasp, lift = _finger_center_contact_waypoints(ctx, arm, perceived)
    else:
        # Conservative waypoints for the current OpenArm bottle scene. The grasp
        # point is above the object center because the bottle body origin is near
        # the table contact frame.
        approach = perceived + np.array([0.0, 0.0, 0.18])
        grasp = perceived + np.array([0.0, 0.0, 0.08])
        lift = perceived + np.array([0.0, 0.0, 0.24])

    min_dist = float("inf")
    max_object_motion = 0.0
    max_object_lift = 0.0
    finger_object_contacts = 0
    approach_finger_object_contacts = 0
    grasp_finger_object_contacts = 0
    lift_finger_object_contacts = 0
    approach_contact_stats = StageContactStats()
    grasp_contact_stats = StageContactStats()
    lift_contact_stats = StageContactStats()
    steps = 0
    ik_success = True
    final_ik_error = 0.0
    closed_gripper_command, open_gripper_command = _calibrated_gripper_control_endpoints(ctx, arm)

    def stage_observer(stage_stats: StageContactStats):
        def observe_step(observed_ctx) -> None:
            stage_stats.observe(
                _sample_contact_frame(observed_ctx, finger_side_geom_ids, object_geoms)
            )

        return observe_step

    set_gripper(ctx, arm, open_gripper_command)
    for _ in range(100):
        mujoco.mj_step(ctx.model, ctx.data)
        steps += 1

    for target in (approach,):
        ok, err, used_steps = _move_to(
            ctx,
            arm,
            target,
            rng,
            noise,
            step_observer=stage_observer(approach_contact_stats),
        )
        steps += used_steps
        final_ik_error = max(final_ik_error, err)
        ik_success = ik_success and ok
        min_dist = min(min_dist, float(np.linalg.norm(end_effector_position(ctx, arm) - true_object)))
        current_object = _object_pos(ctx, object_name)
        max_object_motion = max(max_object_motion, float(np.linalg.norm(current_object - initial_object)))
        max_object_lift = max(max_object_lift, float(current_object[2] - initial_object[2]))
        contact_count = _finger_object_contact_count(ctx, arm.prefix, object_geoms)
        approach_finger_object_contacts += contact_count
        finger_object_contacts += contact_count
        if not ok:
            break

    if ik_success and policy == "pre_grasp_correction":
        perceived = _perceived_position(_object_pos(ctx, object_name), noise, rng)
        grasp = perceived + np.array([0.0, 0.0, 0.08])
        lift = perceived + np.array([0.0, 0.0, 0.24])

    if ik_success and policy == "pre_grasp_finger_center_correction":
        perceived = _perceived_position(_object_pos(ctx, object_name), noise, rng)
        _, grasp, lift = _finger_center_contact_waypoints(ctx, arm, perceived)

    if ik_success:
        for stage_name, target in (("grasp", grasp), ("lift", lift)):
            contact_stats = grasp_contact_stats if stage_name == "grasp" else lift_contact_stats
            if policy == "cube_lift" and stage_name == "lift":
                target_rot = end_effector_rotation(ctx, arm)
                ok, err, used_steps = _move_to_pose(
                    ctx,
                    arm,
                    target,
                    target_rot,
                    rng,
                    noise,
                    step_observer=stage_observer(contact_stats),
                )
            else:
                ok, err, used_steps = _move_to(
                    ctx,
                    arm,
                    target,
                    rng,
                    noise,
                    step_observer=stage_observer(contact_stats),
                )
            steps += used_steps
            final_ik_error = max(final_ik_error, err)
            min_dist = min(min_dist, float(np.linalg.norm(end_effector_position(ctx, arm) - true_object)))
            current_object = _object_pos(ctx, object_name)
            max_object_motion = max(max_object_motion, float(np.linalg.norm(current_object - initial_object)))
            max_object_lift = max(max_object_lift, float(current_object[2] - initial_object[2]))
            contact_count = _finger_object_contact_count(ctx, arm.prefix, object_geoms)
            if stage_name == "grasp":
                grasp_finger_object_contacts += contact_count
            else:
                lift_finger_object_contacts += contact_count
            finger_object_contacts += contact_count
            ik_success = ik_success and ok
            if stage_name == "grasp":
                set_gripper(ctx, arm, closed_gripper_command)
                for _ in range(40):
                    mujoco.mj_step(ctx.model, ctx.data)
                    grasp_contact_stats.observe(
                        _sample_contact_frame(ctx, finger_side_geom_ids, object_geoms)
                    )
                    steps += 1
                    current_object = _object_pos(ctx, object_name)
                    max_object_motion = max(max_object_motion, float(np.linalg.norm(current_object - initial_object)))
                    max_object_lift = max(max_object_lift, float(current_object[2] - initial_object[2]))
                    contact_count = _finger_object_contact_count(ctx, arm.prefix, object_geoms)
                    grasp_finger_object_contacts += contact_count
                    finger_object_contacts += contact_count
            if not ok:
                break

    final_ee = end_effector_position(ctx, arm)
    final_dist = float(np.linalg.norm(final_ee - true_object))
    lift_height = float(final_ee[2] - true_object[2])
    # Proxy success for the first benchmark: the OpenArm EE base is not the
    # fingertip contact point, so this checks whether the commanded grasp came
    # near the object and then reached a lift pose. A later contact benchmark
    # should replace this with object motion/contact conditions.
    proxy_success = bool(ik_success and min_dist < 0.16 and lift_height > 0.12)
    contact_success = bool(ik_success and finger_object_contacts > 0)
    post_grasp_contact_success = bool(ik_success and (grasp_finger_object_contacts + lift_finger_object_contacts) > 0)
    lift_contact_success = bool(ik_success and lift_finger_object_contacts > 0)
    grasp_success = bool(contact_success and max_object_lift > 0.02)
    grasp_two_finger_contact_success = bool(ik_success and grasp_contact_stats.two_finger_frames > 0)
    lift_two_finger_contact_success = bool(ik_success and lift_contact_stats.two_finger_frames > 0)
    sustained_lift_contact_success = bool(
        ik_success
        and lift_contact_stats.sampled_frames > 0
        and (lift_contact_stats.two_finger_frames / lift_contact_stats.sampled_frames) >= 0.5
    )

    return TrialResult(
        policy=policy,
        seed=seed,
        translation_std_m=noise.translation_std_m,
        fixed_bias_x_m=noise.fixed_bias_m[0],
        fixed_bias_y_m=noise.fixed_bias_m[1],
        fixed_bias_z_m=noise.fixed_bias_m[2],
        command_noise_std_rad=noise.command_noise_std_rad,
        ik_success=bool(ik_success),
        proxy_success=proxy_success,
        contact_success=contact_success,
        grasp_success=grasp_success,
        final_ee_error_m=float(final_ik_error),
        min_ee_object_distance_m=min_dist,
        final_ee_object_distance_m=final_dist,
        object_motion_m=float(max_object_motion),
        object_lift_m=float(max_object_lift),
        finger_object_contacts=finger_object_contacts,
        approach_finger_object_contacts=approach_finger_object_contacts,
        grasp_finger_object_contacts=grasp_finger_object_contacts,
        lift_finger_object_contacts=lift_finger_object_contacts,
        approach_sampled_frames=approach_contact_stats.sampled_frames,
        grasp_sampled_frames=grasp_contact_stats.sampled_frames,
        lift_sampled_frames=lift_contact_stats.sampled_frames,
        approach_contact_frames=approach_contact_stats.contact_frames,
        grasp_contact_frames=grasp_contact_stats.contact_frames,
        lift_contact_frames=lift_contact_stats.contact_frames,
        approach_two_finger_contact_frames=approach_contact_stats.two_finger_frames,
        grasp_two_finger_contact_frames=grasp_contact_stats.two_finger_frames,
        lift_two_finger_contact_frames=lift_contact_stats.two_finger_frames,
        approach_contact_frame_rate=approach_contact_stats.contact_frame_rate,
        grasp_contact_frame_rate=grasp_contact_stats.contact_frame_rate,
        lift_contact_frame_rate=lift_contact_stats.contact_frame_rate,
        grasp_two_finger_contact_success=grasp_two_finger_contact_success,
        lift_two_finger_contact_success=lift_two_finger_contact_success,
        sustained_lift_contact_success=sustained_lift_contact_success,
        post_grasp_contact_success=post_grasp_contact_success,
        lift_contact_success=lift_contact_success,
        lift_height_m=lift_height,
        steps=steps,
    )
