from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, field
from numbers import Integral
from pathlib import Path

import mujoco
import numpy as np

from openarm_robust_grasp.control import finger_collision_center, set_gripper
from openarm_robust_grasp.model import ArmIndex, SceneContext, load_scene


@dataclass(frozen=True)
class GripperSweepSample:
    command: float
    joint_qpos: float
    inner_outer_center_distance_m: float
    min_opposing_geom_center_distance_m: float
    actuator_force: float

    def to_row(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class GripperCalibration:
    closed_command: float
    open_command: float
    closed_distance_m: float
    open_distance_m: float
    aperture_span_m: float
    command_increases_aperture: bool
    distance_monotonic: bool
    geometric_motion_meaningful: bool


@dataclass(frozen=True)
class StabilityConfig:
    close_fraction: float
    cube_width_m: float
    sliding_friction: float
    seed: int = 0
    arm_name: str = "left"
    object_name: str = "orange_cube"
    cube_mass_kg: float = 0.015
    lift_height_m: float = 0.08
    open_settle_steps: int = 100
    close_settle_steps: int = 100
    approach_steps: int = 80
    grasp_steps: int = 80
    lift_steps: int = 120
    pre_disturb_hold_steps: int = 500
    disturb_steps: int = 100
    post_disturb_hold_steps: int = 250
    disturbance_force_n: tuple[float, float, float] = (0.05, 0.0, 0.0)

    def __post_init__(self) -> None:
        finite_fields = (
            "close_fraction",
            "cube_width_m",
            "sliding_friction",
            "cube_mass_kg",
            "lift_height_m",
        )
        for field_name in finite_fields:
            if not np.isfinite(float(getattr(self, field_name))):
                raise ValueError(f"{field_name} must be finite")
        if not 0.0 <= self.close_fraction <= 1.0:
            raise ValueError("close_fraction must be within [0, 1]")
        for field_name in (
            "cube_width_m",
            "sliding_friction",
            "cube_mass_kg",
            "lift_height_m",
        ):
            if getattr(self, field_name) <= 0.0:
                raise ValueError(f"{field_name} must be positive")

        if not isinstance(self.seed, Integral) or isinstance(self.seed, bool) or self.seed < 0:
            raise ValueError("seed must be a non-negative integer")
        for field_name in (
            "open_settle_steps",
            "close_settle_steps",
            "approach_steps",
            "grasp_steps",
            "lift_steps",
            "pre_disturb_hold_steps",
            "disturb_steps",
            "post_disturb_hold_steps",
        ):
            value = getattr(self, field_name)
            if not isinstance(value, Integral) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{field_name} must be a non-negative integer")

        force = np.asarray(self.disturbance_force_n, dtype=float)
        if force.shape != (3,) or not np.isfinite(force).all():
            raise ValueError("disturbance_force_n must be a finite 3-vector")
        object.__setattr__(self, "disturbance_force_n", tuple(float(value) for value in force))


@dataclass(frozen=True)
class StabilityResult:
    close_fraction: float
    resolved_close_command: float
    cube_width_m: float
    sliding_friction: float
    cube_mass_kg: float
    seed: int
    ik_success: bool
    close_sampled_frames: int
    lift_sampled_frames: int
    pre_disturb_hold_sampled_frames: int
    disturbance_sampled_frames: int
    post_disturb_hold_sampled_frames: int
    achieved_lift_height_m: float
    final_lift_height_m: float
    pre_disturb_elevation_m: float
    close_two_finger_contact_fraction: float
    lift_two_finger_contact_fraction: float
    pre_disturb_hold_two_finger_contact_fraction: float
    disturbance_two_finger_contact_fraction: float
    post_disturb_hold_two_finger_contact_fraction: float
    max_object_gripper_displacement_m: float
    final_object_gripper_displacement_m: float
    disturbance_drop_m: float
    total_steps: int
    final_elevation_success: bool = field(init=False)
    hold_contact_success: bool = field(init=False)
    disturbance_eligible: bool = field(init=False)
    disturbance_survival_success: bool | None = field(init=False)
    stable_grasp_success: bool = field(init=False)

    def __post_init__(self) -> None:
        final_elevation_success = bool(self.final_lift_height_m >= 0.05)
        hold_contact_success = bool(
            self.pre_disturb_hold_two_finger_contact_fraction >= 0.5
            and self.post_disturb_hold_two_finger_contact_fraction >= 0.5
        )
        disturbance_eligible = bool(
            self.pre_disturb_elevation_m >= 0.05
            and self.pre_disturb_hold_two_finger_contact_fraction >= 0.5
        )
        disturbance_survival_success = (
            bool(self.disturbance_drop_m <= 0.02)
            if disturbance_eligible
            else None
        )
        object.__setattr__(self, "final_elevation_success", final_elevation_success)
        object.__setattr__(self, "hold_contact_success", hold_contact_success)
        object.__setattr__(self, "disturbance_eligible", disturbance_eligible)
        object.__setattr__(
            self, "disturbance_survival_success", disturbance_survival_success
        )
        object.__setattr__(
            self,
            "stable_grasp_success",
            bool(
                final_elevation_success
                and hold_contact_success
                and disturbance_survival_success is True
            ),
        )

    def to_row(self) -> dict[str, object]:
        return asdict(self)


def gripper_command_grid(ctx: SceneContext, arm: ArmIndex, points: int = 9) -> np.ndarray:
    if points < 2:
        raise ValueError("points must be at least 2")
    ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[arm.gripper_actuator_id]
    return np.linspace(float(ctrl_min), float(ctrl_max), points)


def _arm_from_name(ctx: SceneContext, arm_name: str) -> ArmIndex:
    try:
        arm = getattr(ctx, arm_name)
    except AttributeError as exc:
        raise ValueError(f"Unknown arm name: {arm_name}") from exc
    if not isinstance(arm, ArmIndex):
        raise ValueError(f"Unknown arm name: {arm_name}")
    return arm


def _named_collision_geoms(ctx: SceneContext, arm: ArmIndex) -> tuple[list[int], list[int]]:
    inner: list[int] = []
    outer: list[int] = []
    inner_prefix = f"finger_inner_{arm.prefix}_collision"
    outer_prefix = f"finger_outer_{arm.prefix}_collision"
    for geom_id in range(ctx.model.ngeom):
        name = mujoco.mj_id2name(ctx.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith(inner_prefix):
            inner.append(geom_id)
        if name.startswith(outer_prefix):
            outer.append(geom_id)
    if not inner or not outer:
        raise ValueError(
            f"Named inner/outer finger collision geoms not found for arm prefix: {arm.prefix}"
        )
    return inner, outer


def _sample_scene(ctx: SceneContext, arm: ArmIndex, command: float, settle_steps: int) -> GripperSweepSample:
    inner_ids, outer_ids = _named_collision_geoms(ctx, arm)
    ctx.data.ctrl[arm.gripper_actuator_id] = command
    for _ in range(settle_steps):
        mujoco.mj_step(ctx.model, ctx.data)

    inner_centers = ctx.data.geom_xpos[inner_ids]
    outer_centers = ctx.data.geom_xpos[outer_ids]
    inner_center = np.mean(inner_centers, axis=0)
    outer_center = np.mean(outer_centers, axis=0)
    all_distances = np.linalg.norm(
        inner_centers[:, None, :] - outer_centers[None, :, :], axis=2
    )
    joint_id = int(ctx.model.actuator_trnid[arm.gripper_actuator_id, 0])
    if joint_id < 0:
        raise ValueError(f"Gripper actuator has no driven joint: {arm.gripper_actuator_name}")
    joint_qpos = float(ctx.data.qpos[ctx.model.jnt_qposadr[joint_id]])
    distances = (
        float(np.linalg.norm(inner_center - outer_center)),
        float(np.min(all_distances)),
    )
    values = (distances[0], distances[1], float(ctx.data.actuator_force[arm.gripper_actuator_id]))
    if not np.isfinite(values).all() or min(values[:2]) < 0.0:
        raise ValueError("Gripper sweep produced non-finite or negative diagnostic values")
    return GripperSweepSample(
        command=float(command),
        joint_qpos=joint_qpos,
        inner_outer_center_distance_m=values[0],
        min_opposing_geom_center_distance_m=values[1],
        actuator_force=values[2],
    )


def run_gripper_sweep(
    scene_path: str, arm_name: str = "left", points: int = 9, settle_steps: int = 100
) -> list[GripperSweepSample]:
    if settle_steps < 0:
        raise ValueError("settle_steps must be non-negative")
    initial_ctx = load_scene(scene_path)
    arm = _arm_from_name(initial_ctx, arm_name)
    commands = gripper_command_grid(initial_ctx, arm, points=points)
    samples: list[GripperSweepSample] = []
    for command in commands:
        ctx = load_scene(scene_path)
        samples.append(_sample_scene(ctx, _arm_from_name(ctx, arm_name), float(command), settle_steps))
    return samples


def _monotonic_increasing(values: list[float]) -> bool:
    return bool(values) and all(right > left for left, right in zip(values, values[1:]))


def _monotonic_nonincreasing(values: list[float]) -> bool:
    return bool(values) and all(right <= left for left, right in zip(values, values[1:]))


def _monotonic_nondecreasing(values: list[float]) -> bool:
    return bool(values) and all(right >= left for left, right in zip(values, values[1:]))


def infer_gripper_calibration(
    samples: list[GripperSweepSample], min_aperture_span_m: float = 0.005
) -> GripperCalibration:
    if len(samples) < 2:
        raise ValueError("Gripper calibration requires at least two samples")
    if not np.isfinite(min_aperture_span_m) or min_aperture_span_m < 0.0:
        raise ValueError("min_aperture_span_m must be finite and non-negative")

    commands = [float(sample.command) for sample in samples]
    distances = [float(sample.min_opposing_geom_center_distance_m) for sample in samples]
    for sample in samples:
        values = (
            sample.command,
            sample.joint_qpos,
            sample.inner_outer_center_distance_m,
            sample.min_opposing_geom_center_distance_m,
            sample.actuator_force,
        )
        if not np.isfinite(values).all():
            raise ValueError("Gripper calibration samples contain non-finite values")
        if (
            sample.inner_outer_center_distance_m < 0.0
            or sample.min_opposing_geom_center_distance_m < 0.0
        ):
            raise ValueError("Gripper calibration samples contain negative distance")
    if any(right <= left for left, right in zip(commands, commands[1:])):
        raise ValueError("Gripper calibration commands must be strictly increasing")

    nondecreasing = _monotonic_nondecreasing(distances)
    nonincreasing = _monotonic_nonincreasing(distances)
    distance_monotonic = nondecreasing or nonincreasing
    closed_distance = min(distances)
    open_distance = max(distances)
    aperture_span = open_distance - closed_distance

    if nondecreasing and not nonincreasing:
        command_increases_aperture = True
    elif nonincreasing and not nondecreasing:
        command_increases_aperture = False
    else:
        command_increases_aperture = commands[distances.index(open_distance)] > commands[
            distances.index(closed_distance)
        ]

    closed_candidates = [
        command for command, distance in zip(commands, distances) if abs(distance - closed_distance) <= 1e-9
    ]
    open_candidates = [
        command for command, distance in zip(commands, distances) if abs(distance - open_distance) <= 1e-9
    ]
    if command_increases_aperture:
        closed_command = max(closed_candidates)
        open_command = min(open_candidates)
    else:
        closed_command = min(closed_candidates)
        open_command = max(open_candidates)

    return GripperCalibration(
        closed_command=closed_command,
        open_command=open_command,
        closed_distance_m=closed_distance,
        open_distance_m=open_distance,
        aperture_span_m=aperture_span,
        command_increases_aperture=command_increases_aperture,
        distance_monotonic=distance_monotonic,
        geometric_motion_meaningful=bool(
            distance_monotonic and aperture_span >= min_aperture_span_m
        ),
    )


def gripper_command_for_close_fraction(
    calibration: GripperCalibration, close_fraction: float
) -> float:
    if not 0.0 <= close_fraction <= 1.0:
        raise ValueError("close_fraction must be within [0, 1]")
    return float(
        calibration.open_command
        + close_fraction * (calibration.closed_command - calibration.open_command)
    )


def _configure_runtime_cube(
    ctx: SceneContext, config: StabilityConfig
) -> tuple[int, int]:
    body_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, config.object_name
    )
    if body_id < 0:
        raise ValueError(f"MuJoCo body not found: {config.object_name}")
    geom_name = f"{config.object_name}_geom"
    geom_id = mujoco.mj_name2id(ctx.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0:
        raise ValueError(f"MuJoCo geom not found: {geom_name}")
    if int(ctx.model.geom_bodyid[geom_id]) != body_id:
        raise ValueError(f"Geom {geom_name} does not belong to body {config.object_name}")

    half_width = config.cube_width_m / 2.0
    ctx.model.geom_size[geom_id, :3] = half_width
    ctx.model.geom_friction[geom_id, 0] = config.sliding_friction
    ctx.model.body_mass[body_id] = config.cube_mass_kg
    diagonal_inertia = config.cube_mass_kg * config.cube_width_m**2 / 6.0
    ctx.model.body_inertia[body_id, :3] = diagonal_inertia

    qpos = ctx.data.qpos.copy()
    qvel = ctx.data.qvel.copy()
    ctrl = ctx.data.ctrl.copy()
    act = ctx.data.act.copy()
    time = float(ctx.data.time)
    mujoco.mj_setConst(ctx.model, ctx.data)
    ctx.data.qpos[:] = qpos
    ctx.data.qvel[:] = qvel
    ctx.data.ctrl[:] = ctrl
    if ctx.data.act.size:
        ctx.data.act[:] = act
    ctx.data.time = time
    mujoco.mj_forward(ctx.model, ctx.data)
    return body_id, geom_id


def run_stability_trial(scene_path: str, config: StabilityConfig) -> StabilityResult:
    # Lazy import avoids the existing experiment -> diagnostics dependency cycle.
    import openarm_robust_grasp.experiment as experiment_module

    ctx = load_scene(scene_path)
    object_body_id, _ = _configure_runtime_cube(ctx, config)
    arm = _arm_from_name(ctx, config.arm_name)
    calibration = experiment_module._gripper_calibration_for_scene(
        scene_path, config.arm_name
    )
    close_command = gripper_command_for_close_fraction(
        calibration, config.close_fraction
    )
    rng = np.random.default_rng(config.seed)
    noise = experiment_module.NoiseConfig()
    total_steps = 0
    close_stats = experiment_module.StageContactStats()

    def failed_result() -> StabilityResult:
        return StabilityResult(
            close_fraction=config.close_fraction,
            resolved_close_command=close_command,
            cube_width_m=config.cube_width_m,
            sliding_friction=config.sliding_friction,
            cube_mass_kg=config.cube_mass_kg,
            seed=config.seed,
            ik_success=False,
            close_sampled_frames=close_stats.sampled_frames,
            lift_sampled_frames=0,
            pre_disturb_hold_sampled_frames=0,
            disturbance_sampled_frames=0,
            post_disturb_hold_sampled_frames=0,
            achieved_lift_height_m=0.0,
            final_lift_height_m=0.0,
            pre_disturb_elevation_m=0.0,
            close_two_finger_contact_fraction=close_stats.two_finger_contact_frame_rate,
            lift_two_finger_contact_fraction=0.0,
            pre_disturb_hold_two_finger_contact_fraction=0.0,
            disturbance_two_finger_contact_fraction=0.0,
            post_disturb_hold_two_finger_contact_fraction=0.0,
            max_object_gripper_displacement_m=0.0,
            final_object_gripper_displacement_m=0.0,
            disturbance_drop_m=0.0,
            total_steps=total_steps,
        )

    set_gripper(ctx, arm, calibration.open_command)
    for _ in range(config.open_settle_steps):
        mujoco.mj_step(ctx.model, ctx.data)
        total_steps += 1
    mujoco.mj_forward(ctx.model, ctx.data)
    settled_object_z = float(ctx.data.xpos[object_body_id, 2])

    object_position = ctx.data.xpos[object_body_id].copy()
    approach_target, grasp_target, _ = experiment_module._finger_center_contact_waypoints(
        ctx, arm, object_position
    )
    lift_target = grasp_target + np.array([0.0, 0.0, config.lift_height_m])

    ok, _, used_steps = experiment_module._move_to(
        ctx,
        arm,
        approach_target,
        rng,
        noise,
        steps=config.approach_steps,
    )
    total_steps += used_steps
    if not ok:
        return failed_result()

    ok, _, used_steps = experiment_module._move_to(
        ctx,
        arm,
        grasp_target,
        rng,
        noise,
        steps=config.grasp_steps,
    )
    total_steps += used_steps
    if not ok:
        return failed_result()

    object_geom_ids = experiment_module._object_geom_ids(ctx, config.object_name)
    finger_side_geom_ids = experiment_module._finger_side_geom_ids(ctx, arm.prefix)
    set_gripper(ctx, arm, close_command)
    for _ in range(config.close_settle_steps):
        mujoco.mj_step(ctx.model, ctx.data)
        total_steps += 1
        close_stats.observe(
            experiment_module._sample_contact_frame(
                ctx, finger_side_geom_ids, object_geom_ids
            )
        )

    phase_stats = {
        "lift": experiment_module.StageContactStats(),
        "pre_hold": experiment_module.StageContactStats(),
        "disturbance": experiment_module.StageContactStats(),
        "post_hold": experiment_module.StageContactStats(),
    }
    phase_object_z: dict[str, list[float]] = {
        phase_name: [] for phase_name in phase_stats
    }
    object_gripper_displacements: list[float] = []

    def observe(phase_name: str) -> None:
        phase_stats[phase_name].observe(
            experiment_module._sample_contact_frame(
                ctx, finger_side_geom_ids, object_geom_ids
            )
        )
        object_position_now = ctx.data.xpos[object_body_id].copy()
        phase_object_z[phase_name].append(float(object_position_now[2]))
        object_gripper_displacements.append(
            float(np.linalg.norm(object_position_now - finger_collision_center(ctx, arm)))
        )

    target_rotation = experiment_module.end_effector_rotation(ctx, arm)
    ok, _, used_steps = experiment_module._move_to_pose(
        ctx,
        arm,
        lift_target,
        target_rotation,
        rng,
        noise,
        steps=config.lift_steps,
        step_observer=lambda _ctx: observe("lift"),
    )
    total_steps += used_steps
    if not ok:
        return failed_result()

    for _ in range(config.pre_disturb_hold_steps):
        mujoco.mj_step(ctx.model, ctx.data)
        total_steps += 1
        observe("pre_hold")

    pre_disturb_object_z = float(ctx.data.xpos[object_body_id, 2])
    pre_disturb_elevation = pre_disturb_object_z - settled_object_z
    minimum_disturbance_window_z = pre_disturb_object_z
    ctx.data.xfrc_applied[object_body_id, :3] = np.asarray(
        config.disturbance_force_n, dtype=float
    )
    try:
        for _ in range(config.disturb_steps):
            mujoco.mj_step(ctx.model, ctx.data)
            total_steps += 1
            observe("disturbance")
            minimum_disturbance_window_z = min(
                minimum_disturbance_window_z,
                phase_object_z["disturbance"][-1],
            )
    finally:
        ctx.data.xfrc_applied[object_body_id, :3] = 0.0

    for _ in range(config.post_disturb_hold_steps):
        mujoco.mj_step(ctx.model, ctx.data)
        total_steps += 1
        observe("post_hold")
        minimum_disturbance_window_z = min(
            minimum_disturbance_window_z,
            phase_object_z["post_hold"][-1],
        )

    lift_z = phase_object_z["lift"]
    achieved_lift_height = (
        max(0.0, max(lift_z) - settled_object_z) if lift_z else 0.0
    )
    final_lift_height = float(ctx.data.xpos[object_body_id, 2] - settled_object_z)
    max_displacement = (
        max(object_gripper_displacements) if object_gripper_displacements else 0.0
    )
    final_displacement = (
        object_gripper_displacements[-1] if object_gripper_displacements else 0.0
    )

    return StabilityResult(
        close_fraction=config.close_fraction,
        resolved_close_command=close_command,
        cube_width_m=config.cube_width_m,
        sliding_friction=config.sliding_friction,
        cube_mass_kg=config.cube_mass_kg,
        seed=config.seed,
        ik_success=True,
        close_sampled_frames=close_stats.sampled_frames,
        lift_sampled_frames=phase_stats["lift"].sampled_frames,
        pre_disturb_hold_sampled_frames=phase_stats["pre_hold"].sampled_frames,
        disturbance_sampled_frames=phase_stats["disturbance"].sampled_frames,
        post_disturb_hold_sampled_frames=phase_stats["post_hold"].sampled_frames,
        achieved_lift_height_m=achieved_lift_height,
        final_lift_height_m=final_lift_height,
        pre_disturb_elevation_m=pre_disturb_elevation,
        close_two_finger_contact_fraction=close_stats.two_finger_contact_frame_rate,
        lift_two_finger_contact_fraction=phase_stats[
            "lift"
        ].two_finger_contact_frame_rate,
        pre_disturb_hold_two_finger_contact_fraction=phase_stats[
            "pre_hold"
        ].two_finger_contact_frame_rate,
        disturbance_two_finger_contact_fraction=phase_stats[
            "disturbance"
        ].two_finger_contact_frame_rate,
        post_disturb_hold_two_finger_contact_fraction=phase_stats[
            "post_hold"
        ].two_finger_contact_frame_rate,
        max_object_gripper_displacement_m=max_displacement,
        final_object_gripper_displacement_m=final_displacement,
        disturbance_drop_m=max(
            0.0, pre_disturb_object_z - minimum_disturbance_window_z
        ),
        total_steps=total_steps,
    )


def write_gripper_sweep_report(
    samples: list[GripperSweepSample],
    output_dir: str | Path,
    scene_path: str,
    arm_name: str,
    settle_steps: int,
) -> None:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = [sample.to_row() for sample in samples]
    fieldnames = list(GripperSweepSample.__dataclass_fields__)
    with (output / "results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    commands = [sample.command for sample in samples]
    inner_outer = [sample.inner_outer_center_distance_m for sample in samples]
    opposing = [sample.min_opposing_geom_center_distance_m for sample in samples]
    calibration = infer_gripper_calibration(samples) if samples else None
    calibration_fields = (
        asdict(calibration)
        if calibration is not None
        else {
            "closed_command": None,
            "open_command": None,
            "closed_distance_m": None,
            "open_distance_m": None,
            "aperture_span_m": None,
            "command_increases_aperture": None,
            "distance_monotonic": False,
            "geometric_motion_meaningful": False,
        }
    )
    summary = {
        "scene_path": scene_path,
        "arm_name": arm_name,
        "points": len(samples),
        "settle_steps": settle_steps,
        "command_monotonic_increasing": _monotonic_increasing(commands),
        "inner_outer_distance_monotonic_nonincreasing": _monotonic_nonincreasing(inner_outer),
        "min_opposing_geom_center_distance_monotonic_nonincreasing": _monotonic_nonincreasing(opposing),
        "inner_outer_distance_reduction_m": (inner_outer[0] - inner_outer[-1]) if samples else 0.0,
        "min_opposing_geom_center_distance_reduction_m": (opposing[0] - opposing[-1]) if samples else 0.0,
        **calibration_fields,
        "geometric_closure_gate_passed": bool(
            calibration is not None and calibration.geometric_motion_meaningful
        ),
    }
    with (output / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
