from __future__ import annotations

from dataclasses import dataclass
import csv
import hashlib
import io
import json
from math import isfinite
import os
from pathlib import Path

import mujoco
import numpy as np

from openarm_robust_grasp.contact_diagnostics import (
    ContactEvidence,
    REQUIRED_CONSECUTIVE_FRAMES,
    _geom_name,
    _gripper_joint_qpos,
    _named_finger_side_geom_ids,
    _place_fixed_probe,
    _validated_side_ids,
    sample_contact_evidence,
)
from openarm_robust_grasp.model import load_scene


SHAPE_CONDITIONS = (
    ("sphere", (0.021,)),
    ("aligned_cuboid", (0.021, 0.010, 0.015)),
    ("world_cube", (0.021, 0.021, 0.021)),
)

SHAPE_DECISIONS = {
    (False, False, False): "linkage_stroke_or_contact_configuration",
    (True, False, False): "face_orientation_or_mesh_normal",
    (True, False, True): "face_orientation_or_mesh_normal",
    (False, True, False): "aligned_surface_mechanism_pass",
    (False, True, True): "aligned_surface_mechanism_pass",
    (True, True, False): "world_orientation_or_placement",
    (True, True, True): "prior_centroid_world_pose_failure",
    (False, False, True): "contradictory_shape_controls",
}

INSTRUMENT_CONTROL_NAMES = (
    "positive_segment_consistency",
    "geom_order_swap",
    "set_order_invariance",
)


@dataclass(frozen=True)
class NearestSurfacePair:
    inner_geom_id: int
    inner_geom_name: str
    outer_geom_id: int
    outer_geom_name: str
    signed_distance_m: float
    inner_point: tuple[float, float, float] | None
    outer_point: tuple[float, float, float] | None


@dataclass(frozen=True)
class InstrumentControlResult:
    name: str
    passed: bool
    signed_distance_m: float
    error_m: float

    def __post_init__(self) -> None:
        if self.name not in INSTRUMENT_CONTROL_NAMES:
            raise ValueError("unknown instrument control")
        if type(self.passed) is not bool:
            raise ValueError("instrument control passed must be boolean")
        if not isfinite(self.signed_distance_m) or not isfinite(self.error_m):
            raise ValueError("instrument control values must be finite")
        if self.error_m < 0.0:
            raise ValueError("instrument control error must be non-negative")


def select_shape_decision(sphere: bool, aligned: bool, world: bool) -> str:
    values = (sphere, aligned, world)
    if any(type(value) is not bool for value in values):
        raise ValueError("shape decisions require boolean outcomes")
    return SHAPE_DECISIONS[values]


@dataclass(frozen=True)
class ClosingFrame:
    origin: tuple[float, float, float]
    e_close: tuple[float, float, float]
    e_tangent: tuple[float, float, float]
    e_vertical: tuple[float, float, float]

    def __post_init__(self) -> None:
        rotation = self.rotation_matrix
        if not np.all(np.isfinite(rotation)) or not np.all(
            np.isfinite(np.asarray(self.origin, dtype=float))
        ):
            raise ValueError("closing frame values must be finite")
        gram = rotation.T @ rotation
        if not np.allclose(gram, np.eye(3), atol=1e-9, rtol=0.0):
            raise ValueError("closing frame axes must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0.0):
            raise ValueError("closing frame axes must be right-handed")

    @property
    def rotation_matrix(self) -> np.ndarray:
        return np.column_stack(
            (
                np.asarray(self.e_close, dtype=float),
                np.asarray(self.e_tangent, dtype=float),
                np.asarray(self.e_vertical, dtype=float),
            )
        )


@dataclass(frozen=True)
class LinkageConfig:
    command_levels: int = 101
    steps_per_level: int = 20
    open_command: float = 0.58905
    closed_command: float = 0.0

    def __post_init__(self) -> None:
        if type(self.command_levels) is not int or self.command_levels < 2:
            raise ValueError("command_levels must be an integer at least 2")
        if type(self.steps_per_level) is not int or self.steps_per_level < 1:
            raise ValueError("steps_per_level must be a positive integer")
        if not isfinite(self.open_command) or not isfinite(self.closed_command):
            raise ValueError("command endpoints must be finite")
        if self.open_command == self.closed_command:
            raise ValueError("command endpoints must be distinct")


@dataclass(frozen=True)
class LinkageMetrics:
    midpoint_world_drift_m: float | None
    midpoint_local_drift_xyz: tuple[float, float, float] | None
    closing_axis_angular_drift_rad: float | None
    inner_surface_displacement_m: float | None
    outer_surface_displacement_m: float | None
    symmetry_residual_m: float | None


@dataclass(frozen=True)
class LinkageFrame:
    command_index: int
    settle_index: int
    command: float
    close_fraction: float
    joint_qpos: float
    actuator_force: float
    inner_geom_id: int
    inner_geom_name: str
    outer_geom_id: int
    outer_geom_name: str
    signed_distance_m: float
    inner_point: tuple[float, float, float] | None
    outer_point: tuple[float, float, float] | None
    midpoint_world_drift_m: float | None
    midpoint_local_drift_xyz: tuple[float, float, float] | None
    closing_axis_angular_drift_rad: float | None
    inner_surface_displacement_m: float | None
    outer_surface_displacement_m: float | None
    symmetry_residual_m: float | None
    finger_finger_contact_count: int

    def __post_init__(self) -> None:
        if type(self.command_index) is not int or self.command_index < 0:
            raise ValueError("command_index must be a non-negative integer")
        if type(self.settle_index) is not int or self.settle_index < 0:
            raise ValueError("settle_index must be a non-negative integer")
        if not all(
            isfinite(value)
            for value in (
                self.command,
                self.close_fraction,
                self.joint_qpos,
                self.actuator_force,
                self.signed_distance_m,
            )
        ):
            raise ValueError("linkage frame scalar values must be finite")
        if not 0.0 <= self.close_fraction <= 1.0:
            raise ValueError("close_fraction must be within [0, 1]")
        if (
            type(self.finger_finger_contact_count) is not int
            or self.finger_finger_contact_count < 0
        ):
            raise ValueError("finger_finger_contact_count must be non-negative")
        optional_values = (
            self.midpoint_world_drift_m,
            self.midpoint_local_drift_xyz,
            self.closing_axis_angular_drift_rad,
            self.inner_surface_displacement_m,
            self.outer_surface_displacement_m,
            self.symmetry_residual_m,
        )
        if self.signed_distance_m <= 0.0:
            if self.inner_point is not None or self.outer_point is not None:
                raise ValueError("non-positive gap must not expose nearest points")
            if any(value is not None for value in optional_values):
                raise ValueError("non-positive gap must have null derived metrics")
        elif any(value is None for value in optional_values):
            raise ValueError("positive gap requires all derived metrics")


@dataclass(frozen=True)
class LinkageResult:
    config: LinkageConfig
    ik_success: bool
    frames: tuple[LinkageFrame, ...]
    valid_point_frames: int
    pair_switch_count: int
    minimum_probe_gap_m: float

    def __post_init__(self) -> None:
        expected_count = self.config.command_levels * self.config.steps_per_level
        if len(self.frames) != expected_count:
            raise ValueError("linkage frame count does not match config")
        expected_order = tuple(
            (command_index, settle_index)
            for command_index in range(self.config.command_levels)
            for settle_index in range(self.config.steps_per_level)
        )
        if tuple(
            (frame.command_index, frame.settle_index) for frame in self.frames
        ) != expected_order:
            raise ValueError("linkage frame order does not match config")
        measured_valid = sum(
            frame.midpoint_world_drift_m is not None for frame in self.frames
        )
        if self.valid_point_frames != measured_valid:
            raise ValueError("valid_point_frames does not match frames")
        if (
            type(self.pair_switch_count) is not int
            or not 0 <= self.pair_switch_count <= max(len(self.frames) - 1, 0)
        ):
            raise ValueError("pair_switch_count is invalid")
        if not isfinite(self.minimum_probe_gap_m) or self.minimum_probe_gap_m < 0.0:
            raise ValueError("minimum_probe_gap_m must be finite and non-negative")

    @property
    def valid_point_fraction(self) -> float:
        return self.valid_point_frames / len(self.frames)


@dataclass(frozen=True)
class ShapeContactPoint:
    side: str
    finger_geom_id: int
    finger_geom_name: str
    object_geom_id: int
    object_geom_name: str
    position_xyz: tuple[float, float, float]
    finger_to_object_normal_xyz: tuple[float, float, float]
    normal_force_n: float
    absolute_closing_axis_alignment: float


@dataclass(frozen=True)
class ShapeFrame:
    condition: str
    command_index: int
    settle_index: int
    command: float
    close_fraction: float
    joint_qpos: float
    actuator_force: float
    evidence: ContactEvidence
    finger_pair_signed_distance_m: float
    finger_pair_inner_geom_name: str
    finger_pair_outer_geom_name: str
    inner_finger_surface_point_object_xyz: tuple[float, float, float] | None
    inner_object_surface_point_object_xyz: tuple[float, float, float] | None
    outer_finger_surface_point_object_xyz: tuple[float, float, float] | None
    outer_object_surface_point_object_xyz: tuple[float, float, float] | None
    contacts: tuple[ShapeContactPoint, ...]
    opposing_normal_dot_product: float | None
    inner_normal_closing_alignment: float | None
    outer_normal_closing_alignment: float | None
    contact_point_separation_local_xyz: tuple[float, float, float] | None

    def __post_init__(self) -> None:
        if self.condition not in dict(SHAPE_CONDITIONS):
            raise ValueError("unknown shape frame condition")
        if type(self.command_index) is not int or self.command_index < 0:
            raise ValueError("shape command_index must be non-negative")
        if type(self.settle_index) is not int or self.settle_index < 0:
            raise ValueError("shape settle_index must be non-negative")
        required_scalars = (
            self.command,
            self.close_fraction,
            self.joint_qpos,
            self.actuator_force,
            self.finger_pair_signed_distance_m,
        )
        optional_scalars = (
            self.opposing_normal_dot_product,
            self.inner_normal_closing_alignment,
            self.outer_normal_closing_alignment,
        )
        if not all(isfinite(value) for value in required_scalars) or not all(
            value is None or isfinite(value) for value in optional_scalars
        ):
            raise ValueError("shape frame scalar values must be finite")
        if not 0.0 <= self.close_fraction <= 1.0:
            raise ValueError("shape close_fraction must be within [0, 1]")
        if self.opposing_normal_dot_product is not None and not (
            -1.0 - 1e-9 <= self.opposing_normal_dot_product <= 1.0 + 1e-9
        ):
            raise ValueError("opposing normal dot product is outside [-1, 1]")
        for alignment in (
            self.inner_normal_closing_alignment,
            self.outer_normal_closing_alignment,
        ):
            if alignment is not None and not 0.0 <= alignment <= 1.0 + 1e-9:
                raise ValueError("normal alignment is outside [0, 1]")
        vectors = (
            self.inner_finger_surface_point_object_xyz,
            self.inner_object_surface_point_object_xyz,
            self.outer_finger_surface_point_object_xyz,
            self.outer_object_surface_point_object_xyz,
            self.contact_point_separation_local_xyz,
        )
        for vector in vectors:
            if vector is not None and (
                len(vector) != 3 or not all(isfinite(value) for value in vector)
            ):
                raise ValueError("shape frame vectors must be finite three-vectors")


@dataclass(frozen=True)
class ShapeConditionResult:
    condition: str
    dimensions: tuple[float, ...]
    config: LinkageConfig
    ik_success: bool
    open_pair: NearestSurfacePair
    open_frame: ClosingFrame
    object_position_xyz: tuple[float, float, float]
    object_quaternion_wxyz: tuple[float, float, float, float]
    object_rotation_matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    frames: tuple[ShapeFrame, ...]
    gate_pass: bool
    first_passing_command_index: int | None
    stop_reason: str

    def __post_init__(self) -> None:
        dimensions_by_name = dict(SHAPE_CONDITIONS)
        if self.condition not in dimensions_by_name:
            raise ValueError("unknown shape condition result")
        if self.dimensions != dimensions_by_name[self.condition]:
            raise ValueError("shape dimensions do not match condition")
        first_passing = first_passing_shape_level(self.frames)
        if self.first_passing_command_index != first_passing:
            raise ValueError("first passing command does not match frames")
        if self.gate_pass is not (first_passing is not None):
            raise ValueError("shape gate does not match frames")
        expected_reason = (
            "bilateral_gate_passed"
            if self.gate_pass
            else "full_closure_without_bilateral_gate"
        )
        if self.stop_reason != expected_reason:
            raise ValueError("shape stop reason does not match gate")
        expected_count = (
            (first_passing + 1) * self.config.steps_per_level
            if first_passing is not None
            else self.config.command_levels * self.config.steps_per_level
        )
        if len(self.frames) != expected_count:
            raise ValueError("shape frame count does not match stop semantics")
        if any(frame.condition != self.condition for frame in self.frames):
            raise ValueError("shape frame condition does not match result")
        expected_order = tuple(
            (command_index, settle_index)
            for command_index in range(
                first_passing + 1
                if first_passing is not None
                else self.config.command_levels
            )
            for settle_index in range(self.config.steps_per_level)
        )
        if tuple(
            (frame.command_index, frame.settle_index) for frame in self.frames
        ) != expected_order:
            raise ValueError("shape frame order does not match protocol")


def minimum_opposing_surface_pair(
    ctx, inner_ids: set[int], outer_ids: set[int]
) -> NearestSurfacePair:
    inner, outer = _validated_side_ids((set(inner_ids), set(outer_ids)))
    candidates = []
    for inner_id in sorted(inner):
        for outer_id in sorted(outer):
            fromto = np.zeros(6, dtype=float)
            distance = float(
                mujoco.mj_geomDistance(
                    ctx.model, ctx.data, inner_id, outer_id, 1.0, fromto
                )
            )
            if not np.isfinite(distance):
                raise ValueError("opposing surface distance must be finite")
            points = None
            if distance > 0.0:
                segment_length = float(np.linalg.norm(fromto[3:] - fromto[:3]))
                if not np.isclose(segment_length, distance, atol=1e-9, rtol=0.0):
                    raise ValueError(
                        "nearest-point segment length does not match distance"
                    )
                points = (
                    tuple(float(value) for value in fromto[:3]),
                    tuple(float(value) for value in fromto[3:]),
                )
            candidates.append((distance, inner_id, outer_id, points))

    distance, inner_id, outer_id, points = min(
        candidates, key=lambda row: row[:3]
    )
    return NearestSurfacePair(
        inner_geom_id=inner_id,
        inner_geom_name=_geom_name(ctx, inner_id),
        outer_geom_id=outer_id,
        outer_geom_name=_geom_name(ctx, outer_id),
        signed_distance_m=distance,
        inner_point=None if points is None else points[0],
        outer_point=None if points is None else points[1],
    )


def run_surface_instrument_controls(
    scene_path: str,
) -> tuple[InstrumentControlResult, ...]:
    from openarm_robust_grasp.control import set_gripper

    ctx = load_scene(scene_path)
    set_gripper(ctx, ctx.left, LinkageConfig().open_command)
    for _ in range(100):
        mujoco.mj_step(ctx.model, ctx.data)
    inner_ids, outer_ids = _named_finger_side_geom_ids(ctx, ctx.left.prefix)
    pair = minimum_opposing_surface_pair(ctx, inner_ids, outer_ids)
    if pair.inner_point is None or pair.outer_point is None:
        raise RuntimeError("instrument control requires positive surface separation")

    segment_error = abs(
        float(
            np.linalg.norm(
                np.asarray(pair.outer_point) - np.asarray(pair.inner_point)
            )
        )
        - pair.signed_distance_m
    )
    reverse_fromto = np.zeros(6, dtype=float)
    reverse_distance = float(
        mujoco.mj_geomDistance(
            ctx.model,
            ctx.data,
            pair.outer_geom_id,
            pair.inner_geom_id,
            1.0,
            reverse_fromto,
        )
    )
    swap_error = max(
        abs(reverse_distance - pair.signed_distance_m),
        float(
            np.max(
                np.abs(reverse_fromto[:3] - np.asarray(pair.outer_point))
            )
        ),
        float(
            np.max(
                np.abs(reverse_fromto[3:] - np.asarray(pair.inner_point))
            )
        ),
    )
    reordered = minimum_opposing_surface_pair(
        ctx,
        set(reversed(sorted(inner_ids))),
        set(reversed(sorted(outer_ids))),
    )
    order_error = max(
        abs(reordered.signed_distance_m - pair.signed_distance_m),
        float(
            np.max(
                np.abs(np.asarray(reordered.inner_point) - pair.inner_point)
            )
        ),
        float(
            np.max(
                np.abs(np.asarray(reordered.outer_point) - pair.outer_point)
            )
        ),
    )
    controls = (
        InstrumentControlResult(
            "positive_segment_consistency",
            segment_error <= 1e-9,
            pair.signed_distance_m,
            segment_error,
        ),
        InstrumentControlResult(
            "geom_order_swap",
            swap_error <= 1e-9,
            pair.signed_distance_m,
            swap_error,
        ),
        InstrumentControlResult(
            "set_order_invariance",
            order_error <= 1e-9
            and reordered.inner_geom_id == pair.inner_geom_id
            and reordered.outer_geom_id == pair.outer_geom_id,
            pair.signed_distance_m,
            order_error,
        ),
    )
    return controls


def closing_frame_from_surface_pair(pair: NearestSurfacePair) -> ClosingFrame:
    if (
        pair.inner_point is None
        or pair.outer_point is None
        or not np.isfinite(pair.signed_distance_m)
        or pair.signed_distance_m <= 1e-6
    ):
        raise ValueError("surface pair must contain positive separated points")

    inner = np.asarray(pair.inner_point, dtype=float)
    outer = np.asarray(pair.outer_point, dtype=float)
    if inner.shape != (3,) or outer.shape != (3,) or not np.all(
        np.isfinite(np.concatenate((inner, outer)))
    ):
        raise ValueError("surface pair points must be finite three-vectors")

    displacement = outer - inner
    length = float(np.linalg.norm(displacement))
    if not np.isfinite(length) or length <= 1e-6:
        raise ValueError("surface pair points must have positive separation")
    e_close = displacement / length
    world_z = np.array((0.0, 0.0, 1.0), dtype=float)
    if abs(float(np.dot(e_close, world_z))) >= 0.95:
        raise ValueError("closing axis must not be within cosine 0.95 of world Z")

    vertical = world_z - float(np.dot(world_z, e_close)) * e_close
    e_vertical = vertical / np.linalg.norm(vertical)
    e_tangent = np.cross(e_vertical, e_close)
    origin = 0.5 * (inner + outer)
    return ClosingFrame(
        origin=tuple(float(value) for value in origin),
        e_close=tuple(float(value) for value in e_close),
        e_tangent=tuple(float(value) for value in e_tangent),
        e_vertical=tuple(float(value) for value in e_vertical),
    )


def linkage_metrics_from_pair(
    pair: NearestSurfacePair,
    reference: ClosingFrame,
    reference_pair: NearestSurfacePair,
) -> LinkageMetrics:
    if pair.signed_distance_m <= 0.0:
        return LinkageMetrics(None, None, None, None, None, None)
    if reference_pair.inner_point is None or reference_pair.outer_point is None:
        raise ValueError("reference surface pair must contain positive points")

    current = closing_frame_from_surface_pair(pair)
    reference_rotation = reference.rotation_matrix
    delta = np.asarray(current.origin) - np.asarray(reference.origin)
    local_delta = reference_rotation.T @ delta
    axis_dot = float(np.dot(current.e_close, reference.e_close))
    axis_angle = float(np.arccos(np.clip(axis_dot, -1.0, 1.0)))
    inner_projection = float(
        np.dot(
            np.asarray(pair.inner_point) - np.asarray(reference_pair.inner_point),
            reference.e_close,
        )
    )
    outer_projection = float(
        np.dot(
            np.asarray(pair.outer_point) - np.asarray(reference_pair.outer_point),
            reference.e_close,
        )
    )
    return LinkageMetrics(
        midpoint_world_drift_m=float(np.linalg.norm(delta)),
        midpoint_local_drift_xyz=tuple(float(value) for value in local_delta),
        closing_axis_angular_drift_rad=axis_angle,
        inner_surface_displacement_m=inner_projection,
        outer_surface_displacement_m=outer_projection,
        symmetry_residual_m=abs(inner_projection + outer_projection),
    )


def _finger_finger_contact_count(
    ctx, inner_ids: set[int], outer_ids: set[int]
) -> int:
    pairs: set[tuple[int, int]] = set()
    for contact_index in range(int(ctx.data.ncon)):
        contact = ctx.data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        if (geom1 in inner_ids and geom2 in outer_ids) or (
            geom2 in inner_ids and geom1 in outer_ids
        ):
            pairs.add(tuple(sorted((geom1, geom2))))
    return len(pairs)


def _minimum_probe_gap(
    ctx, finger_ids: set[int], probe_geom_id: int
) -> float:
    distances = [
        float(
            mujoco.mj_geomDistance(
                ctx.model, ctx.data, finger_id, probe_geom_id, 10.0, None
            )
        )
        for finger_id in sorted(finger_ids)
    ]
    if not distances or not np.isfinite(distances).all():
        raise ValueError("probe gaps must be finite")
    return min(distances)


def run_unloaded_linkage_baseline(
    scene_path: str, config: LinkageConfig = LinkageConfig()
) -> LinkageResult:
    from openarm_robust_grasp.control import set_gripper
    import openarm_robust_grasp.experiment as experiment_module

    ctx = load_scene(scene_path)
    arm = ctx.left
    probe_body_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, "contact_probe"
    )
    probe_geom_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_probe_geom"
    )
    if probe_body_id < 0 or probe_geom_id < 0:
        raise ValueError("fixed contact probe body/geom not found")

    set_gripper(ctx, arm, config.open_command)
    initial_probe_position = ctx.data.xpos[probe_body_id].copy()
    approach_target, grasp_target, _ = experiment_module._finger_center_contact_waypoints(
        ctx, arm, initial_probe_position
    )
    rng = np.random.default_rng(0)
    noise = experiment_module.NoiseConfig()
    ok, _, _ = experiment_module._move_to(
        ctx, arm, approach_target, rng, noise, steps=80
    )
    if ok:
        ok, _, _ = experiment_module._move_to(
            ctx, arm, grasp_target, rng, noise, steps=80
        )
    if not ok:
        raise RuntimeError("unloaded linkage approach or grasp IK failed")

    finger_side_ids = _named_finger_side_geom_ids(ctx, arm.prefix)
    inner_ids, outer_ids = finger_side_ids
    all_finger_ids = inner_ids | outer_ids
    _place_fixed_probe(
        ctx,
        probe_body_id,
        probe_geom_id,
        np.array((0.0, 0.0, 3.0), dtype=float),
        0.005,
    )
    minimum_probe_gap_m = _minimum_probe_gap(ctx, all_finger_ids, probe_geom_id)
    if minimum_probe_gap_m < 0.20:
        raise RuntimeError("unloaded probe clearance must be at least 0.20 m")

    set_gripper(ctx, arm, config.open_command)
    mujoco.mj_forward(ctx.model, ctx.data)
    reference_pair = minimum_opposing_surface_pair(ctx, inner_ids, outer_ids)
    reference_frame = closing_frame_from_surface_pair(reference_pair)

    commands = np.linspace(
        config.open_command, config.closed_command, config.command_levels
    )
    frames: list[LinkageFrame] = []
    previous_pair_ids: tuple[int, int] | None = None
    pair_switch_count = 0
    for command_index, command in enumerate(commands):
        set_gripper(ctx, arm, float(command))
        for settle_index in range(config.steps_per_level):
            mujoco.mj_step(ctx.model, ctx.data)
            pair = minimum_opposing_surface_pair(ctx, inner_ids, outer_ids)
            pair_ids = (pair.inner_geom_id, pair.outer_geom_id)
            if previous_pair_ids is not None and pair_ids != previous_pair_ids:
                pair_switch_count += 1
            previous_pair_ids = pair_ids
            metrics = linkage_metrics_from_pair(
                pair, reference_frame, reference_pair
            )
            frames.append(
                LinkageFrame(
                    command_index=command_index,
                    settle_index=settle_index,
                    command=float(command),
                    close_fraction=command_index / (len(commands) - 1),
                    joint_qpos=_gripper_joint_qpos(ctx, arm),
                    actuator_force=float(
                        ctx.data.actuator_force[arm.gripper_actuator_id]
                    ),
                    inner_geom_id=pair.inner_geom_id,
                    inner_geom_name=pair.inner_geom_name,
                    outer_geom_id=pair.outer_geom_id,
                    outer_geom_name=pair.outer_geom_name,
                    signed_distance_m=pair.signed_distance_m,
                    inner_point=pair.inner_point,
                    outer_point=pair.outer_point,
                    midpoint_world_drift_m=metrics.midpoint_world_drift_m,
                    midpoint_local_drift_xyz=metrics.midpoint_local_drift_xyz,
                    closing_axis_angular_drift_rad=(
                        metrics.closing_axis_angular_drift_rad
                    ),
                    inner_surface_displacement_m=(
                        metrics.inner_surface_displacement_m
                    ),
                    outer_surface_displacement_m=(
                        metrics.outer_surface_displacement_m
                    ),
                    symmetry_residual_m=metrics.symmetry_residual_m,
                    finger_finger_contact_count=_finger_finger_contact_count(
                        ctx, inner_ids, outer_ids
                    ),
                )
            )

    valid_point_frames = sum(
        frame.midpoint_world_drift_m is not None for frame in frames
    )
    return LinkageResult(
        config=config,
        ik_success=True,
        frames=tuple(frames),
        valid_point_frames=valid_point_frames,
        pair_switch_count=pair_switch_count,
        minimum_probe_gap_m=minimum_probe_gap_m,
    )


def configure_surface_probe(
    ctx, condition: str, frame: ClosingFrame
) -> tuple[int, int]:
    dimensions_by_name = dict(SHAPE_CONDITIONS)
    if condition not in dimensions_by_name:
        raise ValueError("unknown shape condition")
    body_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, "surface_probe"
    )
    geom_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "surface_probe_geom"
    )
    if body_id < 0 or geom_id < 0:
        raise ValueError("surface probe body/geom not found")

    expected_type = (
        int(mujoco.mjtGeom.mjGEOM_SPHERE)
        if condition == "sphere"
        else int(mujoco.mjtGeom.mjGEOM_BOX)
    )
    if int(ctx.model.geom_type[geom_id]) != expected_type:
        raise ValueError("surface probe geom type does not match condition")

    qpos = ctx.data.qpos.copy()
    qvel = ctx.data.qvel.copy()
    ctrl = ctx.data.ctrl.copy()
    act = ctx.data.act.copy()
    time = float(ctx.data.time)
    ctx.model.body_pos[body_id, :3] = np.asarray(frame.origin, dtype=float)
    if condition == "aligned_cuboid":
        quaternion = np.zeros(4, dtype=float)
        mujoco.mju_mat2Quat(quaternion, frame.rotation_matrix.reshape(9))
        ctx.model.body_quat[body_id, :4] = quaternion
    else:
        ctx.model.body_quat[body_id, :4] = (1.0, 0.0, 0.0, 0.0)
    ctx.model.geom_size[geom_id, :3] = 0.0
    dimensions = dimensions_by_name[condition]
    ctx.model.geom_size[geom_id, : len(dimensions)] = dimensions

    mujoco.mj_setConst(ctx.model, ctx.data)
    ctx.data.qpos[:] = qpos
    ctx.data.qvel[:] = qvel
    ctx.data.ctrl[:] = ctrl
    if ctx.data.act.size:
        ctx.data.act[:] = act
    ctx.data.time = time
    mujoco.mj_forward(ctx.model, ctx.data)
    return body_id, geom_id


def shape_level_gate_passes(frames) -> bool:
    items = tuple(frames)
    if len(items) != REQUIRED_CONSECUTIVE_FRAMES:
        return False
    command_index = items[0].command_index
    if tuple(frame.command_index for frame in items) != (
        command_index,
    ) * REQUIRED_CONSECUTIVE_FRAMES:
        return False
    if tuple(frame.settle_index for frame in items) != tuple(
        range(REQUIRED_CONSECUTIVE_FRAMES)
    ):
        return False
    return all(frame.evidence.bilateral_positive_force for frame in items)


def first_passing_shape_level(frames) -> int | None:
    items = tuple(frames)
    start = 0
    while start < len(items):
        command_index = items[start].command_index
        stop = start + 1
        while stop < len(items) and items[stop].command_index == command_index:
            stop += 1
        if shape_level_gate_passes(items[start:stop]):
            return int(command_index)
        start = stop
    return None


def _relocate_surface_probe(ctx, body_id: int, center) -> None:
    qpos = ctx.data.qpos.copy()
    qvel = ctx.data.qvel.copy()
    ctrl = ctx.data.ctrl.copy()
    act = ctx.data.act.copy()
    time = float(ctx.data.time)
    ctx.model.body_pos[body_id, :3] = np.asarray(center, dtype=float)
    mujoco.mj_setConst(ctx.model, ctx.data)
    ctx.data.qpos[:] = qpos
    ctx.data.qvel[:] = qvel
    ctx.data.ctrl[:] = ctrl
    if ctx.data.act.size:
        ctx.data.act[:] = act
    ctx.data.time = time
    mujoco.mj_forward(ctx.model, ctx.data)


def _minimum_side_object_surface_points(
    ctx, side_ids: set[int], object_geom_id: int
) -> tuple[
    float,
    tuple[float, float, float] | None,
    tuple[float, float, float] | None,
]:
    candidates = []
    for finger_id in sorted(side_ids):
        fromto = np.zeros(6, dtype=float)
        distance = float(
            mujoco.mj_geomDistance(
                ctx.model,
                ctx.data,
                finger_id,
                object_geom_id,
                1.0,
                fromto,
            )
        )
        if not np.isfinite(distance):
            raise ValueError("finger-object surface distance must be finite")
        points = None
        if distance > 0.0:
            if not np.isclose(
                np.linalg.norm(fromto[3:] - fromto[:3]),
                distance,
                atol=1e-9,
                rtol=0.0,
            ):
                raise ValueError("finger-object nearest-point segment mismatch")
            points = (fromto[:3].copy(), fromto[3:].copy())
        candidates.append((distance, finger_id, points))
    distance, _, points = min(candidates, key=lambda row: row[:2])
    if points is None:
        return distance, None, None
    return (
        distance,
        tuple(float(value) for value in points[0]),
        tuple(float(value) for value in points[1]),
    )


def _object_local_point(
    point: tuple[float, float, float] | None,
    object_position: np.ndarray,
    object_rotation: np.ndarray,
) -> tuple[float, float, float] | None:
    if point is None:
        return None
    local = object_rotation.T @ (np.asarray(point) - object_position)
    return tuple(float(value) for value in local)


def _shape_contact_diagnostics(
    ctx,
    inner_ids: set[int],
    outer_ids: set[int],
    object_geom_id: int,
    open_frame: ClosingFrame,
) -> tuple[
    tuple[ShapeContactPoint, ...],
    float | None,
    float | None,
    float | None,
    tuple[float, float, float] | None,
]:
    rows: list[ShapeContactPoint] = []
    closing_axis = np.asarray(open_frame.e_close)
    for contact_index in range(int(ctx.data.ncon)):
        contact = ctx.data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        if geom1 == object_geom_id and geom2 in inner_ids | outer_ids:
            finger_id = geom2
            normal = -np.asarray(contact.frame[:3], dtype=float)
        elif geom2 == object_geom_id and geom1 in inner_ids | outer_ids:
            finger_id = geom1
            normal = np.asarray(contact.frame[:3], dtype=float)
        else:
            continue
        side = "inner" if finger_id in inner_ids else "outer"
        force_torque = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(ctx.model, ctx.data, contact_index, force_torque)
        if not np.isfinite(force_torque).all() or not np.isfinite(normal).all():
            raise ValueError("shape contact diagnostic must be finite")
        normal_norm = float(np.linalg.norm(normal))
        if not np.isclose(normal_norm, 1.0, atol=1e-9, rtol=0.0):
            raise ValueError("shape contact normal must be unit length")
        rows.append(
            ShapeContactPoint(
                side=side,
                finger_geom_id=finger_id,
                finger_geom_name=_geom_name(ctx, finger_id),
                object_geom_id=object_geom_id,
                object_geom_name=_geom_name(ctx, object_geom_id),
                position_xyz=tuple(float(value) for value in contact.pos),
                finger_to_object_normal_xyz=tuple(float(value) for value in normal),
                normal_force_n=max(0.0, float(force_torque[0])),
                absolute_closing_axis_alignment=abs(
                    float(np.dot(normal, closing_axis))
                ),
            )
        )

    strongest = {}
    for side in ("inner", "outer"):
        side_rows = [row for row in rows if row.side == side]
        if side_rows:
            strongest[side] = max(
                side_rows,
                key=lambda row: (
                    row.normal_force_n,
                    -row.finger_geom_id,
                    row.position_xyz,
                ),
            )
    if set(strongest) != {"inner", "outer"}:
        return tuple(rows), None, None, None, None

    inner = strongest["inner"]
    outer = strongest["outer"]
    inner_normal = np.asarray(inner.finger_to_object_normal_xyz)
    outer_normal = np.asarray(outer.finger_to_object_normal_xyz)
    separation_world = np.asarray(outer.position_xyz) - np.asarray(inner.position_xyz)
    separation_local = open_frame.rotation_matrix.T @ separation_world
    return (
        tuple(rows),
        float(np.dot(inner_normal, outer_normal)),
        inner.absolute_closing_axis_alignment,
        outer.absolute_closing_axis_alignment,
        tuple(float(value) for value in separation_local),
    )


def run_shape_ladder(
    sphere_scene_path: str,
    cuboid_scene_path: str,
    config: LinkageConfig = LinkageConfig(),
) -> tuple[ShapeConditionResult, ...]:
    from openarm_robust_grasp.control import set_gripper
    import openarm_robust_grasp.experiment as experiment_module

    if config.steps_per_level != REQUIRED_CONSECUTIVE_FRAMES:
        raise ValueError("shape ladder requires exactly 20 steps per command level")
    results: list[ShapeConditionResult] = []
    for condition, dimensions in SHAPE_CONDITIONS:
        scene_path = sphere_scene_path if condition == "sphere" else cuboid_scene_path
        ctx = load_scene(scene_path)
        arm = ctx.left
        probe_body_id = mujoco.mj_name2id(
            ctx.model, mujoco.mjtObj.mjOBJ_BODY, "surface_probe"
        )
        probe_geom_id = mujoco.mj_name2id(
            ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "surface_probe_geom"
        )
        if probe_body_id < 0 or probe_geom_id < 0:
            raise ValueError("surface probe body/geom not found")
        target_position = ctx.data.xpos[probe_body_id].copy()
        _relocate_surface_probe(ctx, probe_body_id, (0.0, 0.0, 3.0))
        set_gripper(ctx, arm, config.open_command)
        approach_target, grasp_target, _ = (
            experiment_module._finger_center_contact_waypoints(
                ctx, arm, target_position
            )
        )
        rng = np.random.default_rng(0)
        noise = experiment_module.NoiseConfig()
        ok, _, _ = experiment_module._move_to(
            ctx, arm, approach_target, rng, noise, steps=80
        )
        if ok:
            ok, _, _ = experiment_module._move_to(
                ctx, arm, grasp_target, rng, noise, steps=80
            )
        if not ok:
            raise RuntimeError(
                f"shape ladder approach or grasp IK failed for {condition}"
            )

        inner_ids, outer_ids = _named_finger_side_geom_ids(ctx, arm.prefix)
        set_gripper(ctx, arm, config.open_command)
        mujoco.mj_forward(ctx.model, ctx.data)
        open_pair = minimum_opposing_surface_pair(ctx, inner_ids, outer_ids)
        open_frame = closing_frame_from_surface_pair(open_pair)
        probe_body_id, probe_geom_id = configure_surface_probe(
            ctx, condition, open_frame
        )
        object_position = ctx.data.xpos[probe_body_id].copy()
        object_quaternion = ctx.data.xquat[probe_body_id].copy()
        object_rotation = ctx.data.xmat[probe_body_id].reshape(3, 3).copy()

        frames: list[ShapeFrame] = []
        commands = np.linspace(
            config.open_command, config.closed_command, config.command_levels
        )
        for command_index, command in enumerate(commands):
            set_gripper(ctx, arm, float(command))
            for settle_index in range(config.steps_per_level):
                mujoco.mj_step(ctx.model, ctx.data)
                evidence = sample_contact_evidence(
                    ctx, (inner_ids, outer_ids), {probe_geom_id}
                )
                finger_pair = minimum_opposing_surface_pair(
                    ctx, inner_ids, outer_ids
                )
                _, inner_finger_point, inner_object_point = (
                    _minimum_side_object_surface_points(
                        ctx, inner_ids, probe_geom_id
                    )
                )
                _, outer_finger_point, outer_object_point = (
                    _minimum_side_object_surface_points(
                        ctx, outer_ids, probe_geom_id
                    )
                )
                contacts, normal_dot, inner_alignment, outer_alignment, separation = (
                    _shape_contact_diagnostics(
                        ctx,
                        inner_ids,
                        outer_ids,
                        probe_geom_id,
                        open_frame,
                    )
                )
                frames.append(
                    ShapeFrame(
                        condition=condition,
                        command_index=command_index,
                        settle_index=settle_index,
                        command=float(command),
                        close_fraction=command_index / (len(commands) - 1),
                        joint_qpos=_gripper_joint_qpos(ctx, arm),
                        actuator_force=float(
                            ctx.data.actuator_force[arm.gripper_actuator_id]
                        ),
                        evidence=evidence,
                        finger_pair_signed_distance_m=(
                            finger_pair.signed_distance_m
                        ),
                        finger_pair_inner_geom_name=finger_pair.inner_geom_name,
                        finger_pair_outer_geom_name=finger_pair.outer_geom_name,
                        inner_finger_surface_point_object_xyz=_object_local_point(
                            inner_finger_point, object_position, object_rotation
                        ),
                        inner_object_surface_point_object_xyz=_object_local_point(
                            inner_object_point, object_position, object_rotation
                        ),
                        outer_finger_surface_point_object_xyz=_object_local_point(
                            outer_finger_point, object_position, object_rotation
                        ),
                        outer_object_surface_point_object_xyz=_object_local_point(
                            outer_object_point, object_position, object_rotation
                        ),
                        contacts=contacts,
                        opposing_normal_dot_product=normal_dot,
                        inner_normal_closing_alignment=inner_alignment,
                        outer_normal_closing_alignment=outer_alignment,
                        contact_point_separation_local_xyz=separation,
                    )
                )
            if shape_level_gate_passes(frames[-config.steps_per_level :]):
                break

        first_passing = first_passing_shape_level(frames)
        gate_pass = first_passing is not None
        results.append(
            ShapeConditionResult(
                condition=condition,
                dimensions=dimensions,
                config=config,
                ik_success=True,
                open_pair=open_pair,
                open_frame=open_frame,
                object_position_xyz=tuple(float(value) for value in object_position),
                object_quaternion_wxyz=tuple(
                    float(value) for value in object_quaternion
                ),
                object_rotation_matrix=tuple(
                    tuple(float(value) for value in row) for row in object_rotation
                ),
                frames=tuple(frames),
                gate_pass=gate_pass,
                first_passing_command_index=first_passing,
                stop_reason=(
                    "bilateral_gate_passed"
                    if gate_pass
                    else "full_closure_without_bilateral_gate"
                ),
            )
        )
    return tuple(results)


def _csv_content(fieldnames: list[str], rows: list[dict[str, object]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _json_cell(value) -> str:
    return "" if value is None else json.dumps(value, separators=(",", ":"))


def _surface_plot_bytes(
    linkage: LinkageResult,
    shapes: tuple[ShapeConditionResult, ...],
) -> tuple[bytes, bytes]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis_left = plt.subplots(figsize=(8, 4.5))
    valid = [
        frame for frame in linkage.frames if frame.midpoint_world_drift_m is not None
    ]
    axis_left.plot(
        [frame.close_fraction for frame in valid],
        [frame.midpoint_world_drift_m for frame in valid],
        label="midpoint drift",
        color="tab:blue",
    )
    axis_left.set_xlabel("Calibrated close fraction")
    axis_left.set_ylabel("Midpoint drift (m)", color="tab:blue")
    axis_right = axis_left.twinx()
    axis_right.plot(
        [frame.close_fraction for frame in valid],
        [frame.closing_axis_angular_drift_rad for frame in valid],
        label="axis drift",
        color="tab:orange",
    )
    axis_right.set_ylabel("Closing-axis drift (rad)", color="tab:orange")
    axis_left.grid(alpha=0.25)
    figure.tight_layout()
    midpoint_stream = io.BytesIO()
    figure.savefig(
        midpoint_stream,
        format="png",
        dpi=140,
        metadata={"Software": "openarm_robust_grasp"},
    )
    plt.close(figure)

    figure, (gap_axis, force_axis) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for result in shapes:
        x_values = [frame.close_fraction for frame in result.frames]
        gap_axis.plot(
            x_values,
            [frame.evidence.inner_signed_gap_m for frame in result.frames],
            label=f"{result.condition} inner",
        )
        gap_axis.plot(
            x_values,
            [frame.evidence.outer_signed_gap_m for frame in result.frames],
            linestyle="--",
            label=f"{result.condition} outer",
        )
        force_axis.plot(
            x_values,
            [frame.evidence.inner_normal_force_n for frame in result.frames],
            label=f"{result.condition} inner",
        )
        force_axis.plot(
            x_values,
            [frame.evidence.outer_normal_force_n for frame in result.frames],
            linestyle="--",
            label=f"{result.condition} outer",
        )
    gap_axis.axhline(0.0, color="black", linewidth=0.8)
    gap_axis.set_ylabel("Signed gap (m)")
    gap_axis.grid(alpha=0.25)
    force_axis.axhline(1e-6, color="black", linewidth=0.8, label="gate >1e-6 N")
    force_axis.set_ylabel("Compressive normal force (N)")
    force_axis.set_xlabel("Calibrated close fraction")
    force_axis.set_yscale("symlog", linthresh=1e-6)
    force_axis.grid(alpha=0.25)
    gap_axis.legend(fontsize=7, ncol=2)
    force_axis.legend(fontsize=7, ncol=2)
    figure.tight_layout()
    shape_stream = io.BytesIO()
    figure.savefig(
        shape_stream,
        format="png",
        dpi=140,
        metadata={"Software": "openarm_robust_grasp"},
    )
    plt.close(figure)
    return midpoint_stream.getvalue(), shape_stream.getvalue()


def write_surface_geometry_report(
    instrument_controls: tuple[InstrumentControlResult, ...],
    linkage: LinkageResult,
    shapes: tuple[ShapeConditionResult, ...],
    decision: str,
    output_dir: str | Path,
) -> dict[str, Path]:
    expected_controls = INSTRUMENT_CONTROL_NAMES
    if tuple(row.name for row in instrument_controls) != expected_controls:
        raise ValueError("instrument controls must have exact order")
    if not all(row.passed for row in instrument_controls):
        raise ValueError("instrument gate must pass before reportable physical stages")
    expected_shapes = tuple(name for name, _ in SHAPE_CONDITIONS)
    if tuple(row.condition for row in shapes) != expected_shapes:
        raise ValueError("shape condition order must be exact")
    expected_decision = select_shape_decision(
        shapes[0].gate_pass, shapes[1].gate_pass, shapes[2].gate_pass
    )
    if decision != expected_decision:
        raise ValueError("decision does not match shape outcomes")

    instrument_fields = ["name", "passed", "signed_distance_m", "error_m"]
    instrument_rows = [
        {
            "name": row.name,
            "passed": row.passed,
            "signed_distance_m": row.signed_distance_m,
            "error_m": row.error_m,
        }
        for row in instrument_controls
    ]
    linkage_fields = [
        "command_index", "settle_index", "command", "close_fraction",
        "joint_qpos", "actuator_force", "inner_geom_id", "inner_geom_name",
        "outer_geom_id", "outer_geom_name", "signed_distance_m",
        "inner_point", "outer_point", "midpoint_world_drift_m",
        "midpoint_local_drift_xyz", "closing_axis_angular_drift_rad",
        "inner_surface_displacement_m", "outer_surface_displacement_m",
        "symmetry_residual_m", "finger_finger_contact_count",
    ]
    linkage_rows = [
        {
            "command_index": frame.command_index,
            "settle_index": frame.settle_index,
            "command": frame.command,
            "close_fraction": frame.close_fraction,
            "joint_qpos": frame.joint_qpos,
            "actuator_force": frame.actuator_force,
            "inner_geom_id": frame.inner_geom_id,
            "inner_geom_name": frame.inner_geom_name,
            "outer_geom_id": frame.outer_geom_id,
            "outer_geom_name": frame.outer_geom_name,
            "signed_distance_m": frame.signed_distance_m,
            "inner_point": _json_cell(frame.inner_point),
            "outer_point": _json_cell(frame.outer_point),
            "midpoint_world_drift_m": frame.midpoint_world_drift_m,
            "midpoint_local_drift_xyz": _json_cell(frame.midpoint_local_drift_xyz),
            "closing_axis_angular_drift_rad": frame.closing_axis_angular_drift_rad,
            "inner_surface_displacement_m": frame.inner_surface_displacement_m,
            "outer_surface_displacement_m": frame.outer_surface_displacement_m,
            "symmetry_residual_m": frame.symmetry_residual_m,
            "finger_finger_contact_count": frame.finger_finger_contact_count,
        }
        for frame in linkage.frames
    ]
    shape_fields = [
        "condition", "command_index", "settle_index", "command", "close_fraction",
        "joint_qpos", "actuator_force", "inner_contact", "outer_contact",
        "bilateral_positive_force", "inner_normal_force_n", "outer_normal_force_n",
        "inner_signed_gap_m", "outer_signed_gap_m", "finger_pair_signed_distance_m",
        "finger_pair_inner_geom_name", "finger_pair_outer_geom_name",
        "inner_finger_surface_point_object_xyz", "inner_object_surface_point_object_xyz",
        "outer_finger_surface_point_object_xyz", "outer_object_surface_point_object_xyz",
        "contacts_json", "opposing_normal_dot_product",
        "inner_normal_closing_alignment", "outer_normal_closing_alignment",
        "contact_point_separation_local_xyz",
    ]
    shape_rows = []
    for result in shapes:
        for frame in result.frames:
            shape_rows.append(
                {
                    "condition": frame.condition,
                    "command_index": frame.command_index,
                    "settle_index": frame.settle_index,
                    "command": frame.command,
                    "close_fraction": frame.close_fraction,
                    "joint_qpos": frame.joint_qpos,
                    "actuator_force": frame.actuator_force,
                    "inner_contact": frame.evidence.inner_contact,
                    "outer_contact": frame.evidence.outer_contact,
                    "bilateral_positive_force": frame.evidence.bilateral_positive_force,
                    "inner_normal_force_n": frame.evidence.inner_normal_force_n,
                    "outer_normal_force_n": frame.evidence.outer_normal_force_n,
                    "inner_signed_gap_m": frame.evidence.inner_signed_gap_m,
                    "outer_signed_gap_m": frame.evidence.outer_signed_gap_m,
                    "finger_pair_signed_distance_m": frame.finger_pair_signed_distance_m,
                    "finger_pair_inner_geom_name": frame.finger_pair_inner_geom_name,
                    "finger_pair_outer_geom_name": frame.finger_pair_outer_geom_name,
                    "inner_finger_surface_point_object_xyz": _json_cell(
                        frame.inner_finger_surface_point_object_xyz
                    ),
                    "inner_object_surface_point_object_xyz": _json_cell(
                        frame.inner_object_surface_point_object_xyz
                    ),
                    "outer_finger_surface_point_object_xyz": _json_cell(
                        frame.outer_finger_surface_point_object_xyz
                    ),
                    "outer_object_surface_point_object_xyz": _json_cell(
                        frame.outer_object_surface_point_object_xyz
                    ),
                    "contacts_json": json.dumps(
                        [
                            {
                                "side": contact.side,
                                "finger_geom_name": contact.finger_geom_name,
                                "object_geom_name": contact.object_geom_name,
                                "position_xyz": contact.position_xyz,
                                "finger_to_object_normal_xyz": (
                                    contact.finger_to_object_normal_xyz
                                ),
                                "normal_force_n": contact.normal_force_n,
                                "absolute_closing_axis_alignment": (
                                    contact.absolute_closing_axis_alignment
                                ),
                            }
                            for contact in frame.contacts
                        ],
                        separators=(",", ":"),
                    ),
                    "opposing_normal_dot_product": frame.opposing_normal_dot_product,
                    "inner_normal_closing_alignment": (
                        frame.inner_normal_closing_alignment
                    ),
                    "outer_normal_closing_alignment": (
                        frame.outer_normal_closing_alignment
                    ),
                    "contact_point_separation_local_xyz": _json_cell(
                        frame.contact_point_separation_local_xyz
                    ),
                }
            )

    csv_streams = {
        "instrument_controls.csv": _csv_content(instrument_fields, instrument_rows),
        "linkage_kinematics.csv": _csv_content(linkage_fields, linkage_rows),
        "shape_ladder.csv": _csv_content(shape_fields, shape_rows),
    }
    valid_frames = [
        frame for frame in linkage.frames if frame.midpoint_world_drift_m is not None
    ]

    def shape_summary(result: ShapeConditionResult) -> dict[str, object]:
        normal_frames = [
            frame
            for frame in result.frames
            if frame.opposing_normal_dot_product is not None
        ]
        return {
            "condition": result.condition,
            "dimensions_m": result.dimensions,
            "sampled_frames": len(result.frames),
            "gate_pass": result.gate_pass,
            "first_passing_command_index": result.first_passing_command_index,
            "stop_reason": result.stop_reason,
            "max_inner_normal_force_n": max(
                frame.evidence.inner_normal_force_n for frame in result.frames
            ),
            "max_outer_normal_force_n": max(
                frame.evidence.outer_normal_force_n for frame in result.frames
            ),
            "min_inner_signed_gap_m": min(
                frame.evidence.inner_signed_gap_m for frame in result.frames
            ),
            "min_outer_signed_gap_m": min(
                frame.evidence.outer_signed_gap_m for frame in result.frames
            ),
            "bilateral_contact_normal_diagnostic_frames": len(normal_frames),
            "min_opposing_normal_dot_product": min(
                (
                    frame.opposing_normal_dot_product
                    for frame in normal_frames
                ),
                default=None,
            ),
            "max_opposing_normal_dot_product": max(
                (
                    frame.opposing_normal_dot_product
                    for frame in normal_frames
                ),
                default=None,
            ),
            "max_inner_normal_closing_alignment": max(
                (
                    frame.inner_normal_closing_alignment
                    for frame in normal_frames
                ),
                default=None,
            ),
            "max_outer_normal_closing_alignment": max(
                (
                    frame.outer_normal_closing_alignment
                    for frame in normal_frames
                ),
                default=None,
            ),
        }

    summary = {
        "protocol": "contact_surface_geometry_v1",
        "instrument_gate_pass": True,
        "protocol_constants": {
            "command_levels": linkage.config.command_levels,
            "steps_per_level": linkage.config.steps_per_level,
            "open_command": linkage.config.open_command,
            "closed_command": linkage.config.closed_command,
            "positive_force_threshold_n": 1e-6,
            "required_consecutive_frames": REQUIRED_CONSECUTIVE_FRAMES,
            "shape_conditions": [
                {"condition": name, "dimensions_m": dimensions}
                for name, dimensions in SHAPE_CONDITIONS
            ],
        },
        "linkage": {
            "sampled_frames": len(linkage.frames),
            "valid_point_frames": linkage.valid_point_frames,
            "valid_point_fraction": linkage.valid_point_fraction,
            "pair_switch_count": linkage.pair_switch_count,
            "minimum_probe_gap_m": linkage.minimum_probe_gap_m,
            "min_signed_distance_m": min(
                frame.signed_distance_m for frame in linkage.frames
            ),
            "max_finger_finger_contact_count": max(
                frame.finger_finger_contact_count for frame in linkage.frames
            ),
            "max_midpoint_world_drift_m": max(
                (frame.midpoint_world_drift_m for frame in valid_frames), default=None
            ),
            "max_closing_axis_angular_drift_rad": max(
                (
                    frame.closing_axis_angular_drift_rad
                    for frame in valid_frames
                ),
                default=None,
            ),
            "max_symmetry_residual_m": max(
                (frame.symmetry_residual_m for frame in valid_frames), default=None
            ),
        },
        "shape_conditions": [shape_summary(result) for result in shapes],
        "decision": decision,
        "csv_sha256": {
            name: hashlib.sha256(content).hexdigest()
            for name, content in csv_streams.items()
        },
    }
    midpoint_png, shape_png = _surface_plot_bytes(linkage, shapes)
    streams = dict(csv_streams)
    streams["summary.json"] = (
        json.dumps(summary, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    streams["midpoint_axis_drift.png"] = midpoint_png
    streams["shape_gap_force.png"] = shape_png

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    paths = {name: output_path / name for name in streams}
    staged = {name: output_path / f".{name}.stage" for name in streams}
    backups = {name: output_path / f".{name}.backup" for name in streams}
    replaced: list[str] = []
    backed_up: list[str] = []
    try:
        for name, content in streams.items():
            staged[name].write_bytes(content)
        for name, path in paths.items():
            if path.exists():
                os.replace(path, backups[name])
                backed_up.append(name)
        for name, path in paths.items():
            os.replace(staged[name], path)
            replaced.append(name)
    except Exception:
        for name in reversed(replaced):
            if paths[name].exists():
                paths[name].unlink()
        for name in reversed(backed_up):
            if backups[name].exists():
                os.replace(backups[name], paths[name])
        raise
    finally:
        for path in (*staged.values(), *backups.values()):
            if path.exists():
                path.unlink()
    return paths
