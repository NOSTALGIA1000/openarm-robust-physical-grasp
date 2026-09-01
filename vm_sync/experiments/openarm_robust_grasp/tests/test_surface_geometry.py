from dataclasses import FrozenInstanceError, replace
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest

from openarm_robust_grasp.contact_diagnostics import ContactEvidence
from openarm_robust_grasp.surface_geometry import (
    ClosingFrame,
    InstrumentControlResult,
    LinkageConfig,
    LinkageFrame,
    LinkageResult,
    NearestSurfacePair,
    SHAPE_CONDITIONS,
    ShapeConditionResult,
    ShapeFrame,
    closing_frame_from_surface_pair,
    configure_surface_probe,
    first_passing_shape_level,
    linkage_metrics_from_pair,
    minimum_opposing_surface_pair,
    run_unloaded_linkage_baseline,
    run_shape_ladder,
    run_surface_instrument_controls,
    select_shape_decision,
    shape_level_gate_passes,
    write_surface_geometry_report,
)


def _fake_context(names: dict[int, str]) -> SimpleNamespace:
    return SimpleNamespace(model=SimpleNamespace(names=names), data=SimpleNamespace())


def _install_distances(
    monkeypatch: pytest.MonkeyPatch,
    distances: dict[tuple[int, int], float],
) -> None:
    def fake_geom_distance(model, data, geom1, geom2, distmax, fromto) -> float:
        key = (int(geom1), int(geom2))
        if key not in distances:
            reverse = (key[1], key[0])
            value = distances[reverse]
            direction = -1.0
        else:
            value = distances[key]
            direction = 1.0
        if np.isfinite(value):
            segment = max(float(value), 0.0)
            fromto[:] = (0.0, 0.0, 0.0, direction * segment, 0.0, 0.0)
        return float(value)

    def fake_id_to_name(model, object_type, geom_id) -> str:
        return model.names[int(geom_id)]

    monkeypatch.setattr("mujoco.mj_geomDistance", fake_geom_distance)
    monkeypatch.setattr("mujoco.mj_id2name", fake_id_to_name)


def test_minimum_pair_uses_distance_then_geom_id_tie_break(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_context({1: "inner1", 2: "inner2", 8: "outer8", 9: "outer9"})
    _install_distances(
        monkeypatch,
        {(1, 8): 0.02, (1, 9): 0.01, (2, 8): 0.01, (2, 9): 0.03},
    )

    pair = minimum_opposing_surface_pair(ctx, {2, 1}, {9, 8})

    assert (pair.inner_geom_id, pair.outer_geom_id) == (1, 9)
    assert (pair.inner_geom_name, pair.outer_geom_name) == ("inner1", "outer9")
    assert pair.signed_distance_m == pytest.approx(0.01)
    assert np.linalg.norm(np.asarray(pair.outer_point) - pair.inner_point) == pytest.approx(0.01)


def test_minimum_pair_refuses_penetration_points(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_context({1: "inner", 2: "outer"})
    _install_distances(monkeypatch, {(1, 2): -0.001})

    pair = minimum_opposing_surface_pair(ctx, {1}, {2})

    assert pair.signed_distance_m == pytest.approx(-0.001)
    assert pair.inner_point is None
    assert pair.outer_point is None


def test_minimum_pair_is_invariant_to_set_iteration_order(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_context({1: "inner1", 2: "inner2", 8: "outer8", 9: "outer9"})
    _install_distances(
        monkeypatch,
        {(1, 8): 0.03, (1, 9): 0.02, (2, 8): 0.01, (2, 9): 0.04},
    )

    forward = minimum_opposing_surface_pair(ctx, {1, 2}, {8, 9})
    reverse = minimum_opposing_surface_pair(ctx, {2, 1}, {9, 8})

    assert forward == reverse


def test_minimum_pair_preserves_inner_outer_point_ownership_when_inner_id_is_larger(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_context({8: "inner", 1: "outer"})
    _install_distances(monkeypatch, {(8, 1): 0.04})

    pair = minimum_opposing_surface_pair(ctx, {8}, {1})

    assert pair.inner_point == pytest.approx((0.0, 0.0, 0.0))
    assert pair.outer_point == pytest.approx((0.04, 0.0, 0.0))


@pytest.mark.parametrize(
    ("inner_ids", "outer_ids", "message"),
    [
        (set(), {2}, "inner"),
        ({1}, set(), "outer"),
        ({1, 2}, {2}, "overlap"),
    ],
)
def test_minimum_pair_rejects_invalid_side_sets(
    inner_ids: set[int], outer_ids: set[int], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        minimum_opposing_surface_pair(_fake_context({}), inner_ids, outer_ids)


@pytest.mark.parametrize("bad_distance", [np.nan, np.inf, -np.inf])
def test_minimum_pair_rejects_nonfinite_distance(
    monkeypatch: pytest.MonkeyPatch, bad_distance: float
) -> None:
    ctx = _fake_context({1: "inner", 2: "outer"})
    _install_distances(monkeypatch, {(1, 2): bad_distance})

    with pytest.raises(ValueError, match="distance.*finite"):
        minimum_opposing_surface_pair(ctx, {1}, {2})


def test_minimum_pair_rejects_positive_distance_segment_mismatch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_context({1: "inner", 2: "outer"})

    def inconsistent_distance(model, data, geom1, geom2, distmax, fromto) -> float:
        fromto[:] = (0.0, 0.0, 0.0, 0.02, 0.0, 0.0)
        return 0.01

    monkeypatch.setattr("mujoco.mj_geomDistance", inconsistent_distance)
    monkeypatch.setattr(
        "mujoco.mj_id2name", lambda model, object_type, geom_id: model.names[geom_id]
    )

    with pytest.raises(ValueError, match="segment length"):
        minimum_opposing_surface_pair(ctx, {1}, {2})


def test_nearest_surface_pair_is_frozen() -> None:
    pair = NearestSurfacePair(1, "inner", 2, "outer", 0.01, (0.0, 0.0, 0.0), (0.01, 0.0, 0.0))
    with pytest.raises(FrozenInstanceError):
        pair.signed_distance_m = 0.02  # type: ignore[misc]


def _surface_pair(
    inner_point: tuple[float, float, float] | None,
    outer_point: tuple[float, float, float] | None,
    distance: float = 0.04,
) -> NearestSurfacePair:
    return NearestSurfacePair(
        1, "inner", 2, "outer", distance, inner_point, outer_point
    )


def test_closing_frame_is_right_handed_and_orthonormal() -> None:
    frame = closing_frame_from_surface_pair(
        _surface_pair((0.0, 0.0, 0.0), (0.0, 0.04, 0.0))
    )
    rotation = frame.rotation_matrix

    assert frame.origin == pytest.approx((0.0, 0.02, 0.0))
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-6)
    assert frame.e_close == pytest.approx((0.0, 1.0, 0.0))
    assert frame.e_vertical == pytest.approx((0.0, 0.0, 1.0))


@pytest.mark.parametrize(
    "pair",
    [
        _surface_pair(None, (0.0, 0.04, 0.0)),
        _surface_pair((0.0, 0.0, 0.0), None),
        _surface_pair((0.0, 0.0, 0.0), (0.0, 0.04, 0.0), 1e-6),
        _surface_pair((np.nan, 0.0, 0.0), (0.0, 0.04, 0.0)),
    ],
)
def test_closing_frame_rejects_unusable_surface_pair(pair: NearestSurfacePair) -> None:
    with pytest.raises(ValueError, match="surface pair"):
        closing_frame_from_surface_pair(pair)


def test_closing_frame_rejects_closing_axis_near_world_vertical() -> None:
    with pytest.raises(ValueError, match="world Z"):
        closing_frame_from_surface_pair(
            _surface_pair((0.0, 0.0, 0.0), (0.0, 0.001, 0.04))
        )


def test_closing_frame_record_rejects_left_handed_axes() -> None:
    with pytest.raises(ValueError, match="right-handed"):
        ClosingFrame(
            origin=(0.0, 0.0, 0.0),
            e_close=(1.0, 0.0, 0.0),
            e_tangent=(0.0, 1.0, 0.0),
            e_vertical=(0.0, 0.0, -1.0),
        )


def test_linkage_config_has_exact_protocol() -> None:
    config = LinkageConfig()

    assert (config.command_levels, config.steps_per_level) == (101, 20)
    assert (config.open_command, config.closed_command) == pytest.approx(
        (0.58905, 0.0)
    )


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"command_levels": 1}, "command_levels"),
        ({"command_levels": True}, "command_levels"),
        ({"steps_per_level": 0}, "steps_per_level"),
        ({"open_command": np.nan}, "command"),
        ({"open_command": 0.0, "closed_command": 0.0}, "distinct"),
    ],
)
def test_linkage_config_rejects_invalid_protocol(kwargs, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        LinkageConfig(**kwargs)


def test_linkage_metrics_measure_world_local_drift_and_surface_symmetry() -> None:
    reference_pair = _surface_pair((0.0, 0.0, 0.0), (0.0, 0.04, 0.0))
    reference = closing_frame_from_surface_pair(reference_pair)
    shifted = _surface_pair((0.0, 0.001, 0.0), (0.0, 0.041, 0.0))

    metrics = linkage_metrics_from_pair(shifted, reference, reference_pair)

    assert metrics.midpoint_world_drift_m == pytest.approx(0.001)
    assert metrics.midpoint_local_drift_xyz == pytest.approx((0.001, 0.0, 0.0))
    assert metrics.closing_axis_angular_drift_rad == pytest.approx(0.0)
    assert metrics.inner_surface_displacement_m == pytest.approx(0.001)
    assert metrics.outer_surface_displacement_m == pytest.approx(0.001)
    assert metrics.symmetry_residual_m == pytest.approx(0.002)


def test_linkage_metrics_clip_axis_angle_roundoff() -> None:
    reference = ClosingFrame(
        origin=(0.0, 0.0, 0.0),
        e_close=(1.0, 0.0, 0.0),
        e_tangent=(0.0, 1.0, 0.0),
        e_vertical=(0.0, 0.0, 1.0),
    )
    pair = _surface_pair((0.0, 0.0, 0.0), (0.04, 0.0, 0.0))

    metrics = linkage_metrics_from_pair(pair, reference, pair)

    assert metrics.closing_axis_angular_drift_rad == pytest.approx(0.0)


def test_linkage_metrics_are_null_after_nonpositive_gap() -> None:
    reference_pair = _surface_pair((0.0, 0.0, 0.0), (0.0, 0.04, 0.0))
    reference = closing_frame_from_surface_pair(reference_pair)
    penetration = _surface_pair(None, None, -0.001)

    metrics = linkage_metrics_from_pair(penetration, reference, reference_pair)

    assert metrics.midpoint_world_drift_m is None
    assert metrics.midpoint_local_drift_xyz is None
    assert metrics.closing_axis_angular_drift_rad is None
    assert metrics.inner_surface_displacement_m is None
    assert metrics.outer_surface_displacement_m is None
    assert metrics.symmetry_residual_m is None


def _linkage_frame(
    command_index: int,
    settle_index: int,
    *,
    metrics_valid: bool = True,
) -> LinkageFrame:
    return LinkageFrame(
        command_index=command_index,
        settle_index=settle_index,
        command=0.58905 - 0.58905 * command_index,
        close_fraction=float(command_index),
        joint_qpos=0.1,
        actuator_force=0.0,
        inner_geom_id=1,
        inner_geom_name="inner",
        outer_geom_id=2,
        outer_geom_name="outer",
        signed_distance_m=0.04 if metrics_valid else -0.001,
        inner_point=(0.0, 0.0, 0.0) if metrics_valid else None,
        outer_point=(0.0, 0.04, 0.0) if metrics_valid else None,
        midpoint_world_drift_m=0.0 if metrics_valid else None,
        midpoint_local_drift_xyz=(0.0, 0.0, 0.0) if metrics_valid else None,
        closing_axis_angular_drift_rad=0.0 if metrics_valid else None,
        inner_surface_displacement_m=-0.02 if metrics_valid else None,
        outer_surface_displacement_m=0.02 if metrics_valid else None,
        symmetry_residual_m=0.0 if metrics_valid else None,
        finger_finger_contact_count=0,
    )


def test_linkage_result_validates_frame_count_and_valid_denominator() -> None:
    config = LinkageConfig(command_levels=2, steps_per_level=1)
    frames = (_linkage_frame(0, 0), _linkage_frame(1, 0, metrics_valid=False))

    result = LinkageResult(
        config=config,
        ik_success=True,
        frames=frames,
        valid_point_frames=1,
        pair_switch_count=0,
        minimum_probe_gap_m=0.2,
    )

    assert result.valid_point_fraction == pytest.approx(0.5)
    with pytest.raises(ValueError, match="frame count"):
        LinkageResult(config, True, frames[:1], 1, 0, 0.2)
    with pytest.raises(ValueError, match="valid_point_frames"):
        LinkageResult(config, True, frames, 2, 0, 0.2)


def test_linkage_records_are_frozen() -> None:
    config = LinkageConfig(command_levels=2, steps_per_level=1)
    with pytest.raises(FrozenInstanceError):
        config.command_levels = 3  # type: ignore[misc]


def test_unloaded_linkage_runner_uses_exact_order_endpoints_and_fresh_state() -> None:
    root = __import__("pathlib").Path(__file__).parents[1]
    scene_path = str(root / "assets" / "cube_contact_fixed_scene.xml")
    config = LinkageConfig(command_levels=2, steps_per_level=3)

    first = run_unloaded_linkage_baseline(scene_path, config)
    second = run_unloaded_linkage_baseline(scene_path, config)

    assert first == second
    assert first.ik_success
    assert first.minimum_probe_gap_m >= 0.20
    assert len(first.frames) == 6
    assert [(frame.command_index, frame.settle_index) for frame in first.frames] == [
        (0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)
    ]
    assert [first.frames[0].command, first.frames[-1].command] == pytest.approx(
        [0.58905, 0.0]
    )
    assert [first.frames[0].close_fraction, first.frames[-1].close_fraction] == [
        0.0,
        1.0,
    ]
    assert first.valid_point_frames <= 6
    assert 0 <= first.pair_switch_count <= 5


def test_shape_conditions_are_exact_and_ordered() -> None:
    assert SHAPE_CONDITIONS == (
        ("sphere", (0.021,)),
        ("aligned_cuboid", (0.021, 0.010, 0.015)),
        ("world_cube", (0.021, 0.021, 0.021)),
    )


def _test_closing_frame() -> ClosingFrame:
    return closing_frame_from_surface_pair(
        _surface_pair((0.40, -0.02, 1.05), (0.40, 0.02, 1.05))
    )


@pytest.mark.parametrize(
    ("asset_name", "condition", "expected_size"),
    [
        ("contact_surface_sphere_scene.xml", "sphere", (0.021,)),
        (
            "contact_surface_cuboid_scene.xml",
            "aligned_cuboid",
            (0.021, 0.010, 0.015),
        ),
        (
            "contact_surface_cuboid_scene.xml",
            "world_cube",
            (0.021, 0.021, 0.021),
        ),
    ],
)
def test_configure_surface_probe_sets_exact_pose_and_preserves_dynamic_state(
    asset_name: str, condition: str, expected_size: tuple[float, ...]
) -> None:
    import mujoco

    from openarm_robust_grasp.model import load_scene

    root = __import__("pathlib").Path(__file__).parents[1]
    ctx = load_scene(str(root / "assets" / asset_name))
    body_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, "surface_probe"
    )
    geom_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "surface_probe_geom"
    )
    qpos_before = ctx.data.qpos.copy()
    qvel_before = ctx.data.qvel.copy()
    ctrl_before = ctx.data.ctrl.copy()
    act_before = ctx.data.act.copy()
    time_before = float(ctx.data.time)
    frame = _test_closing_frame()

    configure_surface_probe(ctx, condition, frame)

    assert ctx.model.geom_size[geom_id, : len(expected_size)] == pytest.approx(
        expected_size
    )
    assert ctx.data.xpos[body_id] == pytest.approx(frame.origin)
    expected_rotation = (
        frame.rotation_matrix if condition == "aligned_cuboid" else np.eye(3)
    )
    assert ctx.data.xmat[body_id].reshape(3, 3) == pytest.approx(
        expected_rotation, abs=1e-9
    )
    assert ctx.data.qpos == pytest.approx(qpos_before)
    assert ctx.data.qvel == pytest.approx(qvel_before)
    assert ctx.data.ctrl == pytest.approx(ctrl_before)
    assert ctx.data.act == pytest.approx(act_before)
    assert ctx.data.time == pytest.approx(time_before)


def test_configure_surface_probe_rejects_condition_scene_type_mismatch() -> None:
    from openarm_robust_grasp.model import load_scene

    root = __import__("pathlib").Path(__file__).parents[1]
    sphere = load_scene(str(root / "assets" / "contact_surface_sphere_scene.xml"))

    with pytest.raises(ValueError, match="geom type"):
        configure_surface_probe(sphere, "aligned_cuboid", _test_closing_frame())


def _shape_gate_frame(
    command_index: int,
    settle_index: int,
    inner_force: float = 2e-6,
    outer_force: float = 2e-6,
) -> SimpleNamespace:
    evidence = ContactEvidence(
        inner_contact=inner_force > 0.0,
        outer_contact=outer_force > 0.0,
        raw_pairs=(),
        inner_normal_force_n=inner_force,
        outer_normal_force_n=outer_force,
        inner_signed_gap_m=0.0,
        outer_signed_gap_m=0.0,
    )
    return SimpleNamespace(
        command_index=command_index,
        settle_index=settle_index,
        evidence=evidence,
    )


def test_shape_level_gate_requires_twenty_complete_bilateral_frames() -> None:
    nineteen = tuple(_shape_gate_frame(0, index) for index in range(19))
    twenty = tuple(_shape_gate_frame(0, index) for index in range(20))

    assert not shape_level_gate_passes(nineteen)
    assert shape_level_gate_passes(twenty)


def test_shape_level_gate_requires_force_strictly_above_threshold() -> None:
    exact = tuple(
        _shape_gate_frame(0, index, inner_force=1e-6) for index in range(20)
    )
    above = tuple(
        _shape_gate_frame(0, index, inner_force=1.000001e-6)
        for index in range(20)
    )

    assert not shape_level_gate_passes(exact)
    assert shape_level_gate_passes(above)


def test_first_passing_shape_level_stops_only_after_complete_level() -> None:
    failed_level = tuple(
        _shape_gate_frame(0, index, outer_force=0.0) for index in range(20)
    )
    passing_level = tuple(_shape_gate_frame(1, index) for index in range(20))
    trailing_partial = tuple(_shape_gate_frame(2, index) for index in range(7))

    assert first_passing_shape_level(failed_level + passing_level) == 1
    assert first_passing_shape_level(failed_level + trailing_partial) is None


def test_all_failure_shape_sequence_keeps_full_2020_frames() -> None:
    failed = tuple(
        _shape_gate_frame(command_index, settle_index, outer_force=0.0)
        for command_index in range(101)
        for settle_index in range(20)
    )

    assert len(failed) == 2020
    assert first_passing_shape_level(failed) is None


def test_shape_ladder_uses_exact_order_pose_and_level_stop_semantics() -> None:
    root = __import__("pathlib").Path(__file__).parents[1]
    sphere_scene = str(root / "assets" / "contact_surface_sphere_scene.xml")
    cuboid_scene = str(root / "assets" / "contact_surface_cuboid_scene.xml")
    config = LinkageConfig(command_levels=2, steps_per_level=20)

    first = run_shape_ladder(sphere_scene, cuboid_scene, config)
    second = run_shape_ladder(sphere_scene, cuboid_scene, config)

    assert first == second
    assert isinstance(first[0], ShapeConditionResult)
    assert tuple((row.condition, row.dimensions) for row in first) == SHAPE_CONDITIONS
    assert all(row.ik_success for row in first)
    for row in first:
        assert row.object_position_xyz == pytest.approx(row.open_frame.origin)
        expected_rotation = (
            row.open_frame.rotation_matrix
            if row.condition == "aligned_cuboid"
            else np.eye(3)
        )
        assert row.object_rotation_matrix == pytest.approx(expected_rotation)
        if row.gate_pass:
            assert row.first_passing_command_index is not None
            assert len(row.frames) == (
                row.first_passing_command_index + 1
            ) * config.steps_per_level
            assert row.stop_reason == "bilateral_gate_passed"
        else:
            assert row.first_passing_command_index is None
            assert len(row.frames) == config.command_levels * config.steps_per_level
            assert row.stop_reason == "full_closure_without_bilateral_gate"


@pytest.mark.parametrize(
    ("sphere", "aligned", "world", "expected"),
    [
        (False, False, False, "linkage_stroke_or_contact_configuration"),
        (True, False, False, "face_orientation_or_mesh_normal"),
        (True, False, True, "face_orientation_or_mesh_normal"),
        (False, True, False, "aligned_surface_mechanism_pass"),
        (False, True, True, "aligned_surface_mechanism_pass"),
        (True, True, False, "world_orientation_or_placement"),
        (True, True, True, "prior_centroid_world_pose_failure"),
        (False, False, True, "contradictory_shape_controls"),
    ],
)
def test_shape_decision_truth_table(
    sphere: bool, aligned: bool, world: bool, expected: str
) -> None:
    assert select_shape_decision(sphere, aligned, world) == expected


def _synthetic_surface_run():
    config = LinkageConfig(command_levels=2, steps_per_level=20)
    linkage_frames = tuple(
        _linkage_frame(command_index, settle_index)
        for command_index in range(2)
        for settle_index in range(20)
    )
    linkage = LinkageResult(
        config=config,
        ik_success=True,
        frames=linkage_frames,
        valid_point_frames=40,
        pair_switch_count=0,
        minimum_probe_gap_m=0.2,
    )
    open_pair = _surface_pair((0.0, 0.0, 0.0), (0.0, 0.04, 0.0))
    open_frame = closing_frame_from_surface_pair(open_pair)
    shapes = []
    for condition, dimensions in SHAPE_CONDITIONS:
        frames = tuple(
            ShapeFrame(
                condition=condition,
                command_index=command_index,
                settle_index=settle_index,
                command=0.58905 * (1 - command_index),
                close_fraction=float(command_index),
                joint_qpos=0.1,
                actuator_force=0.0,
                evidence=ContactEvidence(False, False, (), 0.0, 0.0, 0.01, 0.01),
                finger_pair_signed_distance_m=0.04,
                finger_pair_inner_geom_name="inner",
                finger_pair_outer_geom_name="outer",
                inner_finger_surface_point_object_xyz=(0.0, -0.02, 0.0),
                inner_object_surface_point_object_xyz=(0.0, -0.01, 0.0),
                outer_finger_surface_point_object_xyz=(0.0, 0.02, 0.0),
                outer_object_surface_point_object_xyz=(0.0, 0.01, 0.0),
                contacts=(),
                opposing_normal_dot_product=None,
                inner_normal_closing_alignment=None,
                outer_normal_closing_alignment=None,
                contact_point_separation_local_xyz=None,
            )
            for command_index in range(2)
            for settle_index in range(20)
        )
        rotation = (
            open_frame.rotation_matrix
            if condition == "aligned_cuboid"
            else np.eye(3)
        )
        shapes.append(
            ShapeConditionResult(
                condition=condition,
                dimensions=dimensions,
                config=config,
                ik_success=True,
                open_pair=open_pair,
                open_frame=open_frame,
                object_position_xyz=open_frame.origin,
                object_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                object_rotation_matrix=tuple(tuple(row) for row in rotation),
                frames=frames,
                gate_pass=False,
                first_passing_command_index=None,
                stop_reason="full_closure_without_bilateral_gate",
            )
        )
    controls = (
        InstrumentControlResult("positive_segment_consistency", True, 0.04, 0.0),
        InstrumentControlResult("geom_order_swap", True, 0.04, 0.0),
        InstrumentControlResult("set_order_invariance", True, 0.04, 0.0),
    )
    return controls, linkage, tuple(shapes)


def test_surface_writer_emits_validated_tables_summary_and_plots(tmp_path) -> None:
    controls, linkage, shapes = _synthetic_surface_run()

    paths = write_surface_geometry_report(
        controls,
        linkage,
        shapes,
        "linkage_stroke_or_contact_configuration",
        tmp_path,
    )

    assert set(paths) == {
        "instrument_controls.csv",
        "linkage_kinematics.csv",
        "shape_ladder.csv",
        "summary.json",
        "midpoint_axis_drift.png",
        "shape_gap_force.png",
    }
    assert len((tmp_path / "linkage_kinematics.csv").read_text().splitlines()) == 41
    assert len((tmp_path / "shape_ladder.csv").read_text().splitlines()) == 121
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["protocol"] == "contact_surface_geometry_v1"
    assert summary["instrument_gate_pass"] is True
    assert summary["linkage"]["valid_point_frames"] == 40
    assert summary["linkage"]["sampled_frames"] == 40
    assert summary["linkage"]["min_signed_distance_m"] == pytest.approx(0.04)
    assert summary["linkage"]["max_finger_finger_contact_count"] == 0
    assert summary["decision"] == "linkage_stroke_or_contact_configuration"
    assert tuple(row["condition"] for row in summary["shape_conditions"]) == (
        "sphere",
        "aligned_cuboid",
        "world_cube",
    )
    assert all(
        row["bilateral_contact_normal_diagnostic_frames"] == 0
        and row["min_opposing_normal_dot_product"] is None
        and row["max_opposing_normal_dot_product"] is None
        and row["max_inner_normal_closing_alignment"] is None
        and row["max_outer_normal_closing_alignment"] is None
        for row in summary["shape_conditions"]
    )
    assert set(summary["csv_sha256"]) == {
        "instrument_controls.csv",
        "linkage_kinematics.csv",
        "shape_ladder.csv",
    }


def test_surface_writer_rejects_duplicate_conditions_before_mutation(tmp_path) -> None:
    controls, linkage, shapes = _synthetic_surface_run()
    sentinel = tmp_path / "summary.json"
    sentinel.write_bytes(b"old-summary")

    with pytest.raises(ValueError, match="condition order"):
        write_surface_geometry_report(
            controls,
            linkage,
            (shapes[0], shapes[0], shapes[2]),
            "linkage_stroke_or_contact_configuration",
            tmp_path,
        )

    assert sentinel.read_bytes() == b"old-summary"


def test_shape_frame_rejects_nonfinite_scalar() -> None:
    _, _, shapes = _synthetic_surface_run()

    with pytest.raises(ValueError, match="finite"):
        replace(shapes[0].frames[0], actuator_force=np.nan)


def test_shape_result_rejects_mismatched_frame_condition() -> None:
    _, _, shapes = _synthetic_surface_run()
    mismatched = replace(shapes[0].frames[0], condition="world_cube")

    with pytest.raises(ValueError, match="frame condition"):
        replace(shapes[0], frames=(mismatched,) + shapes[0].frames[1:])


@pytest.mark.parametrize("failure_index", range(6))
def test_surface_writer_rolls_back_every_replacement_failure(
    tmp_path, monkeypatch: pytest.MonkeyPatch, failure_index: int
) -> None:
    controls, linkage, shapes = _synthetic_surface_run()
    names = (
        "instrument_controls.csv",
        "linkage_kinematics.csv",
        "shape_ladder.csv",
        "summary.json",
        "midpoint_axis_drift.png",
        "shape_gap_force.png",
    )
    for name in names:
        (tmp_path / name).write_bytes(f"old-{name}".encode())
    real_replace = os.replace
    replacement_count = 0

    def failing_replace(source, destination):
        nonlocal replacement_count
        source_path = __import__("pathlib").Path(source)
        if source_path.name.endswith(".stage"):
            if replacement_count == failure_index:
                raise OSError("injected replacement failure")
            replacement_count += 1
        return real_replace(source, destination)

    monkeypatch.setattr(os, "replace", failing_replace)
    with pytest.raises(OSError, match="injected"):
        write_surface_geometry_report(
            controls,
            linkage,
            shapes,
            "linkage_stroke_or_contact_configuration",
            tmp_path,
        )

    for name in names:
        assert (tmp_path / name).read_bytes() == f"old-{name}".encode()


def test_real_surface_instrument_controls_pass_in_exact_order() -> None:
    root = __import__("pathlib").Path(__file__).parents[1]
    scene_path = str(root / "assets" / "cube_contact_fixed_scene.xml")

    controls = run_surface_instrument_controls(scene_path)

    assert tuple(row.name for row in controls) == (
        "positive_segment_consistency",
        "geom_order_swap",
        "set_order_invariance",
    )
    assert all(row.passed for row in controls)
    assert all(row.signed_distance_m > 1e-6 for row in controls)
    assert all(row.error_m <= 1e-9 for row in controls)


def test_cli_orchestration_uses_exact_stage_order_and_writes_once(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    import run_surface_geometry_diagnosis as runner

    calls = []
    controls = tuple(
        InstrumentControlResult(name, True, 0.04, 0.0)
        for name in (
            "positive_segment_consistency",
            "geom_order_swap",
            "set_order_invariance",
        )
    )
    linkage = object()
    shapes = tuple(
        SimpleNamespace(gate_pass=False) for _ in range(3)
    )

    monkeypatch.setattr(
        runner,
        "run_surface_instrument_controls",
        lambda scene: calls.append("instrument") or controls,
    )
    monkeypatch.setattr(
        runner,
        "run_unloaded_linkage_baseline",
        lambda scene, config: calls.append("linkage") or linkage,
    )
    monkeypatch.setattr(
        runner,
        "run_shape_ladder",
        lambda sphere, cuboid, config: calls.append("shape") or shapes,
    )
    monkeypatch.setattr(
        runner,
        "select_shape_decision",
        lambda sphere, aligned, world: calls.append("decision")
        or "linkage_stroke_or_contact_configuration",
    )
    monkeypatch.setattr(
        runner,
        "write_surface_geometry_report",
        lambda *args: calls.append("writer") or {},
    )

    result = runner.execute_surface_geometry_diagnosis(
        "fixed.xml", "sphere.xml", "cuboid.xml", tmp_path
    )

    assert calls == ["instrument", "linkage", "shape", "decision", "writer"]
    assert result.decision == "linkage_stroke_or_contact_configuration"
    assert result.instrument_controls is controls
    assert result.linkage is linkage
    assert result.shapes is shapes


def test_cli_instrument_failure_prevents_physical_stages(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    import run_surface_geometry_diagnosis as runner

    calls = []
    controls = (
        InstrumentControlResult("positive_segment_consistency", False, 0.04, 1e-3),
        InstrumentControlResult("geom_order_swap", True, 0.04, 0.0),
        InstrumentControlResult("set_order_invariance", True, 0.04, 0.0),
    )
    monkeypatch.setattr(
        runner,
        "run_surface_instrument_controls",
        lambda scene: calls.append("instrument") or controls,
    )
    monkeypatch.setattr(
        runner,
        "run_unloaded_linkage_baseline",
        lambda *args: calls.append("linkage"),
    )

    with pytest.raises(RuntimeError, match="instrument gate"):
        runner.execute_surface_geometry_diagnosis(
            "fixed.xml", "sphere.xml", "cuboid.xml", tmp_path
        )

    assert calls == ["instrument"]
