from __future__ import annotations

from dataclasses import dataclass, field
import csv
import hashlib
import io
import json
from math import isfinite
import os
from pathlib import Path
from typing import Iterable

import mujoco
import numpy as np

from openarm_robust_grasp.model import load_scene


POSITIVE_FORCE_N = 1e-6
REQUIRED_CONSECUTIVE_FRAMES = 20
CONTACT_CONTROL_NAMES = ("negative", "inner_only", "outer_only", "bilateral")


@dataclass(frozen=True)
class ContactPairEvidence:
    object_geom_id: int
    object_geom_name: str
    finger_geom_id: int
    finger_geom_name: str
    side: str
    normal_force_n: float

    def __post_init__(self) -> None:
        if self.side not in {"inner", "outer"}:
            raise ValueError("side must be inner or outer")
        if not isfinite(self.normal_force_n) or self.normal_force_n < 0.0:
            raise ValueError("normal_force_n must be finite and non-negative")


@dataclass(frozen=True)
class ContactEvidence:
    inner_contact: bool
    outer_contact: bool
    raw_pairs: tuple[ContactPairEvidence, ...]
    inner_normal_force_n: float
    outer_normal_force_n: float
    inner_signed_gap_m: float
    outer_signed_gap_m: float

    def __post_init__(self) -> None:
        values = (
            self.inner_normal_force_n,
            self.outer_normal_force_n,
            self.inner_signed_gap_m,
            self.outer_signed_gap_m,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("contact evidence values must be finite")
        if self.inner_normal_force_n < 0.0 or self.outer_normal_force_n < 0.0:
            raise ValueError("normal force sums must be non-negative")

    @property
    def bilateral_positive_force(self) -> bool:
        return bool(
            self.inner_contact
            and self.outer_contact
            and self.inner_normal_force_n > POSITIVE_FORCE_N
            and self.outer_normal_force_n > POSITIVE_FORCE_N
        )


@dataclass(frozen=True)
class ContactSequenceStats:
    sampled_frames: int
    inner_contact_frames: int
    outer_contact_frames: int
    bilateral_frames: int
    max_consecutive_bilateral_positive_force_frames: int
    max_inner_normal_force_n: float
    max_outer_normal_force_n: float

    @classmethod
    def from_frames(cls, frames: Iterable[ContactEvidence]) -> ContactSequenceStats:
        items = tuple(frames)
        current = 0
        maximum = 0
        for frame in items:
            current = current + 1 if frame.bilateral_positive_force else 0
            maximum = max(maximum, current)
        return cls(
            sampled_frames=len(items),
            inner_contact_frames=sum(frame.inner_contact for frame in items),
            outer_contact_frames=sum(frame.outer_contact for frame in items),
            bilateral_frames=sum(
                frame.inner_contact and frame.outer_contact for frame in items
            ),
            max_consecutive_bilateral_positive_force_frames=maximum,
            max_inner_normal_force_n=max(
                (frame.inner_normal_force_n for frame in items), default=0.0
            ),
            max_outer_normal_force_n=max(
                (frame.outer_normal_force_n for frame in items), default=0.0
            ),
        )


@dataclass(frozen=True)
class ContactControlResult:
    repeat_index: int
    expected_class: str
    observed_class: str
    geometry_precondition_pass: bool
    evidence: ContactEvidence
    pass_control: bool

    def __post_init__(self) -> None:
        if type(self.repeat_index) is not int or self.repeat_index < 0:
            raise ValueError("repeat_index must be a non-negative integer")
        if self.expected_class not in CONTACT_CONTROL_NAMES:
            raise ValueError("unknown expected contact class")
        if self.observed_class not in CONTACT_CONTROL_NAMES:
            raise ValueError("unknown observed contact class")
        expected_pass = bool(
            self.geometry_precondition_pass
            and self.observed_class == self.expected_class
        )
        if self.pass_control is not expected_pass:
            raise ValueError("pass_control does not match control evidence")


@dataclass(frozen=True)
class ClosureConfig:
    command_levels: int = 101
    steps_per_level: int = 20
    cube_width_m: float = 0.042
    placement_names: tuple[str, str] = field(
        default=("current_waypoint", "geometric_center"), init=False
    )
    profile_names: tuple[str, str] = field(
        default=("raw_full", "calibrated"), init=False
    )

    def __post_init__(self) -> None:
        if type(self.command_levels) is not int or self.command_levels < 2:
            raise ValueError("command_levels must be an integer at least 2")
        if type(self.steps_per_level) is not int or self.steps_per_level < 0:
            raise ValueError("steps_per_level must be a non-negative integer")
        if not isfinite(self.cube_width_m) or self.cube_width_m <= 0.0:
            raise ValueError("cube_width_m must be finite and positive")


@dataclass(frozen=True)
class ClosureFrame:
    placement: str
    profile: str
    command_index: int
    settle_index: int
    command: float
    close_fraction: float
    joint_qpos: float
    actuator_force: float
    evidence: ContactEvidence
    cube_position_xyz: tuple[float, float, float]
    cube_quaternion_wxyz: tuple[float, float, float, float]
    gripper_position_xyz: tuple[float, float, float]
    gripper_quaternion_wxyz: tuple[float, float, float, float]
    relative_translation_xyz: tuple[float, float, float]
    relative_rotation_rad: float

    def __post_init__(self) -> None:
        scalars = (
            self.command,
            self.close_fraction,
            self.joint_qpos,
            self.actuator_force,
            self.relative_rotation_rad,
            *self.cube_position_xyz,
            *self.cube_quaternion_wxyz,
            *self.gripper_position_xyz,
            *self.gripper_quaternion_wxyz,
            *self.relative_translation_xyz,
        )
        if not all(isfinite(value) for value in scalars):
            raise ValueError("closure frame values must be finite")


@dataclass(frozen=True)
class ClosureResult:
    placement: str
    profile: str
    ik_success: bool
    frames: tuple[ClosureFrame, ...]
    stats: ContactSequenceStats
    max_cube_displacement_m: float
    mechanism_gate_pass: bool
    failure_reason: str | None

    def __post_init__(self) -> None:
        if self.stats.sampled_frames != len(self.frames):
            raise ValueError("closure stats/frame count mismatch")
        expected_gate = bool(
            self.placement == "geometric_center"
            and self.ik_success
            and closure_gate_passes(self.stats)
        )
        if self.mechanism_gate_pass is not expected_gate:
            raise ValueError("mechanism gate does not match closure evidence")


@dataclass(frozen=True)
class DiagnosticRunResult:
    controls: tuple[ContactControlResult, ...]
    fixed_closures: tuple[ClosureResult, ...]
    free_closures: tuple[object, ...]
    conditional_lifts: tuple[object, ...]
    failure_branch: str
    free_closure_not_run_reason: str | None
    conditional_lift_not_run_reason: str | None

    def __post_init__(self) -> None:
        expected_controls = tuple(
            (repeat_index, name)
            for repeat_index in range(2)
            for name in CONTACT_CONTROL_NAMES
        )
        observed_controls = tuple(
            (row.repeat_index, row.expected_class) for row in self.controls
        )
        if observed_controls != expected_controls:
            raise ValueError("controls must contain exact two-repeat condition order")
        if not all(row.pass_control for row in self.controls):
            raise ValueError("measurement controls must pass before fixed closure")
        expected_fixed = (
            ("current_waypoint", "raw_full"),
            ("current_waypoint", "calibrated"),
            ("geometric_center", "raw_full"),
            ("geometric_center", "calibrated"),
        )
        if tuple((row.placement, row.profile) for row in self.fixed_closures) != expected_fixed:
            raise ValueError("fixed closures must contain exact condition order")
        fixed_gate = any(row.mechanism_gate_pass for row in self.fixed_closures)
        if self.failure_branch == "fixed_mechanism_gate_failed":
            if fixed_gate or self.free_closures or self.conditional_lifts:
                raise ValueError("failed fixed gate requires empty downstream stages")
            if self.free_closure_not_run_reason != "fixed_mechanism_gate_failed":
                raise ValueError("free closure skip reason must match fixed gate failure")


def _csv_bytes(fieldnames: list[str], rows: list[dict[str, object]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def write_contact_diagnostic_report(
    run: DiagnosticRunResult, output_dir: str | Path
) -> dict[str, Path]:
    output_path = Path(output_dir)
    control_fields = [
        "repeat_index", "expected_class", "observed_class",
        "geometry_precondition_pass", "pass_control",
        "inner_signed_gap_m", "outer_signed_gap_m",
        "inner_normal_force_n", "outer_normal_force_n", "raw_pairs_json",
    ]
    control_rows = []
    for row in run.controls:
        control_rows.append({
            "repeat_index": row.repeat_index,
            "expected_class": row.expected_class,
            "observed_class": row.observed_class,
            "geometry_precondition_pass": row.geometry_precondition_pass,
            "pass_control": row.pass_control,
            "inner_signed_gap_m": row.evidence.inner_signed_gap_m,
            "outer_signed_gap_m": row.evidence.outer_signed_gap_m,
            "inner_normal_force_n": row.evidence.inner_normal_force_n,
            "outer_normal_force_n": row.evidence.outer_normal_force_n,
            "raw_pairs_json": json.dumps(
                [
                    {
                        "object": pair.object_geom_name,
                        "finger": pair.finger_geom_name,
                        "side": pair.side,
                        "normal_force_n": pair.normal_force_n,
                    }
                    for pair in row.evidence.raw_pairs
                ],
                separators=(",", ":"),
            ),
        })

    fixed_fields = [
        "placement", "profile", "command_index", "settle_index", "command",
        "close_fraction", "joint_qpos", "actuator_force",
        "inner_contact", "outer_contact", "bilateral_positive_force",
        "inner_signed_gap_m", "outer_signed_gap_m",
        "inner_normal_force_n", "outer_normal_force_n",
        "cube_position_xyz", "cube_quaternion_wxyz", "gripper_position_xyz",
        "gripper_quaternion_wxyz", "relative_translation_xyz",
        "relative_rotation_rad", "raw_pairs_json",
    ]
    fixed_rows = []
    for result in run.fixed_closures:
        for frame in result.frames:
            fixed_rows.append({
                "placement": frame.placement,
                "profile": frame.profile,
                "command_index": frame.command_index,
                "settle_index": frame.settle_index,
                "command": frame.command,
                "close_fraction": frame.close_fraction,
                "joint_qpos": frame.joint_qpos,
                "actuator_force": frame.actuator_force,
                "inner_contact": frame.evidence.inner_contact,
                "outer_contact": frame.evidence.outer_contact,
                "bilateral_positive_force": frame.evidence.bilateral_positive_force,
                "inner_signed_gap_m": frame.evidence.inner_signed_gap_m,
                "outer_signed_gap_m": frame.evidence.outer_signed_gap_m,
                "inner_normal_force_n": frame.evidence.inner_normal_force_n,
                "outer_normal_force_n": frame.evidence.outer_normal_force_n,
                "cube_position_xyz": json.dumps(frame.cube_position_xyz),
                "cube_quaternion_wxyz": json.dumps(frame.cube_quaternion_wxyz),
                "gripper_position_xyz": json.dumps(frame.gripper_position_xyz),
                "gripper_quaternion_wxyz": json.dumps(frame.gripper_quaternion_wxyz),
                "relative_translation_xyz": json.dumps(frame.relative_translation_xyz),
                "relative_rotation_rad": frame.relative_rotation_rad,
                "raw_pairs_json": json.dumps(
                    [
                        {"object": pair.object_geom_name,
                         "finger": pair.finger_geom_name,
                         "side": pair.side,
                         "normal_force_n": pair.normal_force_n}
                        for pair in frame.evidence.raw_pairs
                    ], separators=(",", ":")
                ),
            })

    streams = {
        "contact_controls.csv": _csv_bytes(control_fields, control_rows),
        "fixed_closure.csv": _csv_bytes(fixed_fields, fixed_rows),
        "free_closure.csv": _csv_bytes(["not_run_reason"], []),
        "conditional_lift.csv": _csv_bytes(["not_run_reason"], []),
    }
    summary = {
        "protocol": "contact_formation_diagnostics_v1",
        "measurement_gate_pass": all(row.pass_control for row in run.controls),
        "fixed_mechanism_gate_pass": any(
            row.mechanism_gate_pass for row in run.fixed_closures
        ),
        "failure_branch": run.failure_branch,
        "free_closure_not_run_reason": run.free_closure_not_run_reason,
        "conditional_lift_not_run_reason": run.conditional_lift_not_run_reason,
        "control_count": len(run.controls),
        "fixed_condition_count": len(run.fixed_closures),
        "fixed_frame_count": len(fixed_rows),
        "fixed_conditions": [
            {
                "placement": row.placement,
                "profile": row.profile,
                "ik_success": row.ik_success,
                "sampled_frames": row.stats.sampled_frames,
                "bilateral_frames": row.stats.bilateral_frames,
                "max_consecutive_bilateral_positive_force_frames":
                    row.stats.max_consecutive_bilateral_positive_force_frames,
                "max_inner_normal_force_n": row.stats.max_inner_normal_force_n,
                "max_outer_normal_force_n": row.stats.max_outer_normal_force_n,
                "mechanism_gate_pass": row.mechanism_gate_pass,
            }
            for row in run.fixed_closures
        ],
        "csv_sha256": {
            name: hashlib.sha256(content).hexdigest()
            for name, content in streams.items()
        },
    }
    streams["summary.json"] = (
        json.dumps(summary, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")

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


def contact_class(frame: ContactEvidence) -> str:
    inner = frame.inner_contact and frame.inner_normal_force_n > POSITIVE_FORCE_N
    outer = frame.outer_contact and frame.outer_normal_force_n > POSITIVE_FORCE_N
    return {
        (False, False): "negative",
        (True, False): "inner_only",
        (False, True): "outer_only",
        (True, True): "bilateral",
    }[(inner, outer)]


def command_profile(
    ctx,
    arm,
    calibration,
    profile: str,
    command_levels: int = 101,
) -> np.ndarray:
    if type(command_levels) is not int or command_levels < 2:
        raise ValueError("command_levels must be an integer at least 2")
    if profile not in {"raw_full", "calibrated"}:
        raise ValueError("profile must be raw_full or calibrated")
    if profile == "calibrated":
        open_command = float(calibration.open_command)
        closed_command = float(calibration.closed_command)
    else:
        ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[
            arm.gripper_actuator_id
        ]
        if calibration.command_increases_aperture:
            open_command, closed_command = float(ctrl_max), float(ctrl_min)
        else:
            open_command, closed_command = float(ctrl_min), float(ctrl_max)
    values = np.linspace(open_command, closed_command, command_levels)
    if not np.isfinite(values).all():
        raise ValueError("command profile must be finite")
    return values


def _gripper_joint_qpos(ctx, arm) -> float:
    actuator_id = int(arm.gripper_actuator_id)
    transmission = int(ctx.model.actuator_trntype[actuator_id])
    if transmission != int(mujoco.mjtTrn.mjTRN_JOINT):
        raise ValueError(
            f"Gripper actuator must use joint transmission: {arm.gripper_actuator_name}"
        )
    joint_id = int(ctx.model.actuator_trnid[actuator_id, 0])
    if joint_id < 0:
        raise ValueError(f"Gripper actuator has no driven joint: {arm.gripper_actuator_name}")
    return float(ctx.data.qpos[int(ctx.model.jnt_qposadr[joint_id])])


def _matrix_quaternion(matrix: np.ndarray) -> np.ndarray:
    quaternion = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, np.asarray(matrix, dtype=float).reshape(9))
    return quaternion


def _relative_rotation_rad(first: np.ndarray, second: np.ndarray) -> float:
    dot = float(abs(np.dot(first, second)))
    return float(2.0 * np.arccos(np.clip(dot, 0.0, 1.0)))


def run_fixed_cube_closure(
    scene_path: str, config: ClosureConfig = ClosureConfig()
) -> list[ClosureResult]:
    from openarm_robust_grasp.control import set_gripper
    import openarm_robust_grasp.experiment as experiment_module

    calibration = experiment_module._gripper_calibration_for_scene(
        scene_path, "left"
    )
    results: list[ClosureResult] = []
    for placement in config.placement_names:
        for profile in config.profile_names:
            ctx = load_scene(scene_path)
            arm = ctx.left
            body_id = mujoco.mj_name2id(
                ctx.model, mujoco.mjtObj.mjOBJ_BODY, "contact_probe"
            )
            geom_id = mujoco.mj_name2id(
                ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_probe_geom"
            )
            if body_id < 0 or geom_id < 0:
                raise ValueError("fixed contact probe body/geom not found")
            _place_fixed_probe(
                ctx,
                body_id,
                geom_id,
                ctx.data.xpos[body_id].copy(),
                config.cube_width_m / 2.0,
            )
            _gripper_joint_qpos(ctx, arm)
            set_gripper(ctx, arm, calibration.open_command)

            object_position = ctx.data.xpos[body_id].copy()
            approach_target, grasp_target, _ = (
                experiment_module._finger_center_contact_waypoints(
                    ctx, arm, object_position
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
                empty_stats = ContactSequenceStats.from_frames(())
                results.append(
                    ClosureResult(
                        placement=placement,
                        profile=profile,
                        ik_success=False,
                        frames=(),
                        stats=empty_stats,
                        max_cube_displacement_m=0.0,
                        mechanism_gate_pass=False,
                        failure_reason="approach_or_grasp_ik_failed",
                    )
                )
                continue

            finger_side_ids = _named_finger_side_geom_ids(ctx, arm.prefix)
            if placement == "geometric_center":
                inner_centroid = np.mean(
                    ctx.data.geom_xpos[sorted(finger_side_ids[0])], axis=0
                )
                outer_centroid = np.mean(
                    ctx.data.geom_xpos[sorted(finger_side_ids[1])], axis=0
                )
                midpoint = 0.5 * (inner_centroid + outer_centroid)
                centered_position = ctx.data.xpos[body_id].copy()
                centered_position[:2] = midpoint[:2]
                _place_fixed_probe(
                    ctx,
                    body_id,
                    geom_id,
                    centered_position,
                    config.cube_width_m / 2.0,
                )

            initial_cube_position = ctx.data.xpos[body_id].copy()
            commands = command_profile(
                ctx, arm, calibration, profile, config.command_levels
            )
            frames: list[ClosureFrame] = []
            for command_index, command in enumerate(commands):
                set_gripper(ctx, arm, float(command))
                for settle_index in range(config.steps_per_level):
                    mujoco.mj_step(ctx.model, ctx.data)
                    evidence = sample_contact_evidence(
                        ctx, finger_side_ids, {geom_id}
                    )
                    cube_position = ctx.data.xpos[body_id].copy()
                    cube_quaternion = ctx.data.xquat[body_id].copy()
                    all_finger_ids = sorted(
                        finger_side_ids[0] | finger_side_ids[1]
                    )
                    gripper_position = np.mean(
                        ctx.data.geom_xpos[all_finger_ids], axis=0
                    )
                    gripper_quaternion = _matrix_quaternion(
                        ctx.data.xmat[arm.ee_body_id]
                    )
                    frames.append(
                        ClosureFrame(
                            placement=placement,
                            profile=profile,
                            command_index=command_index,
                            settle_index=settle_index,
                            command=float(command),
                            close_fraction=command_index / (len(commands) - 1),
                            joint_qpos=_gripper_joint_qpos(ctx, arm),
                            actuator_force=float(
                                ctx.data.actuator_force[arm.gripper_actuator_id]
                            ),
                            evidence=evidence,
                            cube_position_xyz=tuple(cube_position),
                            cube_quaternion_wxyz=tuple(cube_quaternion),
                            gripper_position_xyz=tuple(gripper_position),
                            gripper_quaternion_wxyz=tuple(gripper_quaternion),
                            relative_translation_xyz=tuple(
                                cube_position - gripper_position
                            ),
                            relative_rotation_rad=_relative_rotation_rad(
                                cube_quaternion, gripper_quaternion
                            ),
                        )
                    )
            stats = ContactSequenceStats.from_frames(
                frame.evidence for frame in frames
            )
            max_displacement = max(
                (
                    float(
                        np.linalg.norm(
                            np.asarray(frame.cube_position_xyz)
                            - initial_cube_position
                        )
                    )
                    for frame in frames
                ),
                default=0.0,
            )
            gate_pass = bool(
                placement == "geometric_center" and closure_gate_passes(stats)
            )
            results.append(
                ClosureResult(
                    placement=placement,
                    profile=profile,
                    ik_success=True,
                    frames=tuple(frames),
                    stats=stats,
                    max_cube_displacement_m=max_displacement,
                    mechanism_gate_pass=gate_pass,
                    failure_reason=None,
                )
            )
    return results


def closure_gate_passes(stats: ContactSequenceStats) -> bool:
    return (
        stats.max_consecutive_bilateral_positive_force_frames
        >= REQUIRED_CONSECUTIVE_FRAMES
    )


def _validated_side_ids(
    finger_side_geom_ids: tuple[set[int], set[int]],
) -> tuple[set[int], set[int]]:
    inner_ids = {int(value) for value in finger_side_geom_ids[0]}
    outer_ids = {int(value) for value in finger_side_geom_ids[1]}
    if not inner_ids:
        raise ValueError("inner finger geom ids must not be empty")
    if not outer_ids:
        raise ValueError("outer finger geom ids must not be empty")
    overlap = inner_ids & outer_ids
    if overlap:
        raise ValueError(f"overlapping finger geom ids: {sorted(overlap)}")
    return inner_ids, outer_ids


def _geom_name(ctx, geom_id: int) -> str:
    return (
        mujoco.mj_id2name(ctx.model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""
    )


def sample_contact_evidence(
    ctx,
    finger_side_geom_ids: tuple[set[int], set[int]],
    object_geom_ids: set[int],
) -> ContactEvidence:
    inner_ids, outer_ids = _validated_side_ids(finger_side_geom_ids)
    object_ids = {int(value) for value in object_geom_ids}
    if not object_ids:
        raise ValueError("object geom ids must not be empty")

    pair_rows: list[ContactPairEvidence] = []
    side_forces = {"inner": 0.0, "outer": 0.0}
    side_contacts = {"inner": False, "outer": False}
    for contact_id in range(int(ctx.data.ncon)):
        contact = ctx.data.contact[contact_id]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        if geom1 in object_ids:
            object_id, finger_id = geom1, geom2
        elif geom2 in object_ids:
            object_id, finger_id = geom2, geom1
        else:
            continue

        side = (
            "inner"
            if finger_id in inner_ids
            else "outer"
            if finger_id in outer_ids
            else None
        )
        if side is None:
            continue

        force_torque = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(
            ctx.model, ctx.data, contact_id, force_torque
        )
        if not np.isfinite(force_torque).all():
            raise ValueError("contact force must be finite")
        normal_force = max(0.0, float(force_torque[0]))
        side_contacts[side] = True
        side_forces[side] += normal_force
        pair_rows.append(
            ContactPairEvidence(
                object_geom_id=object_id,
                object_geom_name=_geom_name(ctx, object_id),
                finger_geom_id=finger_id,
                finger_geom_name=_geom_name(ctx, finger_id),
                side=side,
                normal_force_n=normal_force,
            )
        )

    def minimum_gap(side_ids: set[int]) -> float:
        distances: list[float] = []
        for finger_id in side_ids:
            for object_id in object_ids:
                value = float(
                    mujoco.mj_geomDistance(
                        ctx.model,
                        ctx.data,
                        finger_id,
                        object_id,
                        1.0,
                        None,
                    )
                )
                if not isfinite(value):
                    raise ValueError("signed geom distance must be finite")
                distances.append(value)
        return min(distances)

    return ContactEvidence(
        inner_contact=side_contacts["inner"],
        outer_contact=side_contacts["outer"],
        raw_pairs=tuple(pair_rows),
        inner_normal_force_n=side_forces["inner"],
        outer_normal_force_n=side_forces["outer"],
        inner_signed_gap_m=minimum_gap(inner_ids),
        outer_signed_gap_m=minimum_gap(outer_ids),
    )


def _named_finger_side_geom_ids(ctx, arm_prefix: str) -> tuple[set[int], set[int]]:
    inner_prefix = f"finger_inner_{arm_prefix}_collision"
    outer_prefix = f"finger_outer_{arm_prefix}_collision"
    inner_ids: set[int] = set()
    outer_ids: set[int] = set()
    ambiguous: list[str] = []
    for geom_id in range(ctx.model.ngeom):
        name = _geom_name(ctx, geom_id)
        if arm_prefix not in name or "finger" not in name or "collision" not in name:
            continue
        if name.startswith(inner_prefix):
            inner_ids.add(geom_id)
        elif name.startswith(outer_prefix):
            outer_ids.add(geom_id)
        else:
            ambiguous.append(name)
    if ambiguous:
        raise ValueError(f"ambiguous finger collision geoms: {ambiguous}")
    return _validated_side_ids((inner_ids, outer_ids))


def _closest_finger_geom_pair(
    ctx, inner_ids: set[int], outer_ids: set[int]
) -> tuple[int, int]:
    return min(
        ((inner_id, outer_id) for inner_id in inner_ids for outer_id in outer_ids),
        key=lambda pair: float(
            np.linalg.norm(
                ctx.data.geom_xpos[pair[0]] - ctx.data.geom_xpos[pair[1]]
            )
        ),
    )


def _place_fixed_probe(
    ctx, body_id: int, geom_id: int, center: np.ndarray, half_size: float
) -> None:
    center_array = np.asarray(center, dtype=float)
    if center_array.shape != (3,) or not np.isfinite(center_array).all():
        raise ValueError("probe center must be a finite 3-vector")
    if not isfinite(half_size) or half_size <= 0.0:
        raise ValueError("probe half size must be finite and positive")
    qpos = ctx.data.qpos.copy()
    qvel = ctx.data.qvel.copy()
    ctrl = ctx.data.ctrl.copy()
    act = ctx.data.act.copy()
    time = float(ctx.data.time)
    ctx.model.body_pos[body_id, :3] = center_array
    ctx.model.geom_size[geom_id, :3] = half_size
    mujoco.mj_setConst(ctx.model, ctx.data)
    ctx.data.qpos[:] = qpos
    ctx.data.qvel[:] = qvel
    ctx.data.ctrl[:] = ctrl
    if ctx.data.act.size:
        ctx.data.act[:] = act
    ctx.data.time = time
    mujoco.mj_forward(ctx.model, ctx.data)


def _geometry_precondition(expected_class: str, frame: ContactEvidence) -> bool:
    if expected_class == "negative":
        return bool(
            frame.inner_signed_gap_m >= 0.020
            and frame.outer_signed_gap_m >= 0.020
        )
    if expected_class == "inner_only":
        return bool(
            frame.inner_signed_gap_m <= 0.0 and frame.outer_signed_gap_m > 0.0
        )
    if expected_class == "outer_only":
        return bool(
            frame.outer_signed_gap_m <= 0.0 and frame.inner_signed_gap_m > 0.0
        )
    return bool(
        frame.inner_signed_gap_m <= 0.0 and frame.outer_signed_gap_m <= 0.0
    )


def run_contact_controls(
    scene_path: str, repeats: int = 2
) -> list[ContactControlResult]:
    if type(repeats) is not int or repeats <= 0:
        raise ValueError("repeats must be a positive integer")

    results: list[ContactControlResult] = []
    for repeat_index in range(repeats):
        for expected_class in CONTACT_CONTROL_NAMES:
            ctx = load_scene(scene_path)
            finger_side_ids = _named_finger_side_geom_ids(ctx, "left")
            inner_ids, outer_ids = finger_side_ids
            inner_id, outer_id = _closest_finger_geom_pair(
                ctx, inner_ids, outer_ids
            )
            probe_body_id = mujoco.mj_name2id(
                ctx.model, mujoco.mjtObj.mjOBJ_BODY, "contact_probe"
            )
            probe_geom_id = mujoco.mj_name2id(
                ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_probe_geom"
            )
            if probe_body_id < 0 or probe_geom_id < 0:
                raise ValueError("fixed contact probe body/geom not found")
            object_ids = {int(probe_geom_id)}

            inner_center = ctx.data.geom_xpos[inner_id].copy()
            outer_center = ctx.data.geom_xpos[outer_id].copy()
            closing_axis = outer_center - inner_center
            closing_axis /= np.linalg.norm(closing_axis)
            if expected_class == "negative":
                finger_centers = ctx.data.geom_xpos[
                    sorted(inner_ids | outer_ids)
                ]
                base_center = np.max(finger_centers, axis=0)
                evidence = None
                for z_offset in (0.200, 0.400, 0.800):
                    center = base_center + np.array([0.0, 0.0, z_offset])
                    _place_fixed_probe(
                        ctx, probe_body_id, probe_geom_id, center, 0.001
                    )
                    evidence = sample_contact_evidence(
                        ctx, finger_side_ids, object_ids
                    )
                    if _geometry_precondition(expected_class, evidence):
                        break
                if evidence is None:
                    raise RuntimeError("negative control construction did not run")
            elif expected_class == "inner_only":
                _place_fixed_probe(
                    ctx,
                    probe_body_id,
                    probe_geom_id,
                    inner_center - 0.004 * closing_axis,
                    0.001,
                )
            elif expected_class == "outer_only":
                _place_fixed_probe(
                    ctx,
                    probe_body_id,
                    probe_geom_id,
                    outer_center + 0.004 * closing_axis,
                    0.001,
                )
            else:
                midpoint = 0.5 * (inner_center + outer_center)
                half_size = (
                    0.5 * float(np.linalg.norm(inner_center - outer_center))
                    + 0.001
                )
                _place_fixed_probe(
                    ctx, probe_body_id, probe_geom_id, midpoint, half_size
                )

            evidence = sample_contact_evidence(ctx, finger_side_ids, object_ids)
            observed_class = contact_class(evidence)
            geometry_pass = _geometry_precondition(expected_class, evidence)
            pass_control = bool(
                geometry_pass and observed_class == expected_class
            )
            results.append(
                ContactControlResult(
                    repeat_index=repeat_index,
                    expected_class=expected_class,
                    observed_class=observed_class,
                    geometry_precondition_pass=geometry_pass,
                    evidence=evidence,
                    pass_control=pass_control,
                )
            )
    return results
