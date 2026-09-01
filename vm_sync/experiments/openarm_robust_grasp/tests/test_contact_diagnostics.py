from dataclasses import FrozenInstanceError
import json
from types import SimpleNamespace

import numpy as np
import pytest

from openarm_robust_grasp.contact_diagnostics import (
    CONTACT_CONTROL_NAMES,
    ClosureConfig,
    DiagnosticRunResult,
    ContactEvidence,
    ContactSequenceStats,
    _gripper_joint_qpos,
    _named_finger_side_geom_ids,
    _place_fixed_probe,
    closure_gate_passes,
    command_profile,
    contact_class,
    run_contact_controls,
    run_fixed_cube_closure,
    sample_contact_evidence,
    write_contact_diagnostic_report,
)
from openarm_robust_grasp.grasp_diagnostics import GripperCalibration


def _evidence(
    inner_force: float = 0.0,
    outer_force: float = 0.0,
    *,
    finite: bool = True,
) -> ContactEvidence:
    inner_value = float(inner_force) if finite else float("nan")
    return ContactEvidence(
        inner_contact=inner_force > 0.0,
        outer_contact=outer_force > 0.0,
        raw_pairs=(),
        inner_normal_force_n=inner_value,
        outer_normal_force_n=float(outer_force),
        inner_signed_gap_m=0.0,
        outer_signed_gap_m=0.0,
    )


@pytest.mark.parametrize(
    ("inner", "outer", "expected"),
    [
        (0.0, 0.0, "negative"),
        (1.0, 0.0, "inner_only"),
        (0.0, 1.0, "outer_only"),
        (1.0, 1.0, "bilateral"),
    ],
)
def test_contact_class_uses_inner_outer_positive_force_semantics(
    inner: float, outer: float, expected: str
) -> None:
    assert contact_class(_evidence(inner, outer)) == expected


def test_contact_evidence_is_frozen_and_rejects_nonfinite_values() -> None:
    sample = _evidence(1.0, 1.0)
    with pytest.raises(FrozenInstanceError):
        sample.inner_contact = False  # type: ignore[misc]
    with pytest.raises(ValueError, match="finite"):
        _evidence(1.0, 1.0, finite=False)


def test_closure_gate_requires_twenty_consecutive_bilateral_positive_force_frames() -> None:
    nineteen = ContactSequenceStats.from_frames([_evidence(1.0, 1.0)] * 19)
    twenty = ContactSequenceStats.from_frames([_evidence(1.0, 1.0)] * 20)

    assert not closure_gate_passes(nineteen)
    assert closure_gate_passes(twenty)


def test_sequence_break_resets_consecutive_bilateral_counter() -> None:
    stats = ContactSequenceStats.from_frames(
        [_evidence(1.0, 1.0)] * 12
        + [_evidence(1.0, 0.0)]
        + [_evidence(1.0, 1.0)] * 11
    )

    assert stats.sampled_frames == 24
    assert stats.max_consecutive_bilateral_positive_force_frames == 12
    assert not closure_gate_passes(stats)


def test_contact_class_requires_force_strictly_above_threshold() -> None:
    assert contact_class(_evidence(1e-6, 1.0)) == "outer_only"
    assert contact_class(_evidence(1.000001e-6, 1.0)) == "bilateral"


def _fake_contact_context(
    contacts: list[tuple[int, int]], names: dict[int, str]
) -> SimpleNamespace:
    return SimpleNamespace(
        model=SimpleNamespace(names=names),
        data=SimpleNamespace(
            ncon=len(contacts),
            contact=[SimpleNamespace(geom1=g1, geom2=g2) for g1, g2 in contacts],
        ),
    )


def _install_fake_mujoco_primitives(
    monkeypatch: pytest.MonkeyPatch,
    *,
    forces: list[float],
    distances: dict[tuple[int, int], float],
) -> None:
    def fake_contact_force(model, data, contact_id, result) -> None:
        result[:] = 0.0
        result[0] = forces[contact_id]

    def fake_geom_distance(model, data, geom1, geom2, distmax, fromto) -> float:
        return distances[tuple(sorted((int(geom1), int(geom2))))]

    def fake_id_to_name(model, object_type, geom_id) -> str:
        return model.names[int(geom_id)]

    monkeypatch.setattr("mujoco.mj_contactForce", fake_contact_force)
    monkeypatch.setattr("mujoco.mj_geomDistance", fake_geom_distance)
    monkeypatch.setattr("mujoco.mj_id2name", fake_id_to_name)


def test_sample_contact_evidence_aggregates_named_pairs_for_each_side(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_contact_context(
        contacts=[(10, 1), (2, 10)],
        names={
            1: "finger_inner_left_collision_00",
            2: "finger_outer_left_collision_00",
            10: "probe_geom",
        },
    )
    _install_fake_mujoco_primitives(
        monkeypatch,
        forces=[0.4, 0.6],
        distances={(1, 10): -0.001, (2, 10): -0.002},
    )

    result = sample_contact_evidence(ctx, ({1}, {2}), {10})

    assert result.inner_normal_force_n == pytest.approx(0.4)
    assert result.outer_normal_force_n == pytest.approx(0.6)
    assert result.inner_signed_gap_m == pytest.approx(-0.001)
    assert result.outer_signed_gap_m == pytest.approx(-0.002)
    assert result.bilateral_positive_force
    assert [
        (pair.side, pair.object_geom_id, pair.finger_geom_id)
        for pair in result.raw_pairs
    ] == [("inner", 10, 1), ("outer", 10, 2)]


def test_sample_contact_evidence_ignores_non_object_and_non_finger_contacts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_contact_context(
        contacts=[(1, 20), (10, 30)],
        names={1: "inner", 2: "outer", 10: "object", 20: "arm", 30: "floor"},
    )
    _install_fake_mujoco_primitives(
        monkeypatch,
        forces=[9.0, 8.0],
        distances={(1, 10): 0.01, (2, 10): 0.02},
    )

    result = sample_contact_evidence(ctx, ({1}, {2}), {10})

    assert not result.inner_contact
    assert not result.outer_contact
    assert result.raw_pairs == ()
    assert result.inner_normal_force_n == 0.0
    assert result.outer_normal_force_n == 0.0


def test_sample_contact_evidence_uses_minimum_gap_across_each_side(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_contact_context(
        contacts=[],
        names={1: "inner0", 3: "inner1", 2: "outer", 10: "object"},
    )
    _install_fake_mujoco_primitives(
        monkeypatch,
        forces=[],
        distances={(1, 10): 0.02, (3, 10): -0.003, (2, 10): 0.01},
    )

    result = sample_contact_evidence(ctx, ({1, 3}, {2}), {10})

    assert result.inner_signed_gap_m == pytest.approx(-0.003)
    assert result.outer_signed_gap_m == pytest.approx(0.01)


@pytest.mark.parametrize(
    ("side_ids", "object_ids", "message"),
    [
        ((set(), {2}), {10}, "inner"),
        (({1}, set()), {10}, "outer"),
        (({1, 2}, {2}), {10}, "overlap"),
        (({1}, {2}), set(), "object"),
    ],
)
def test_sample_contact_evidence_rejects_invalid_geom_sets(
    side_ids: tuple[set[int], set[int]],
    object_ids: set[int],
    message: str,
) -> None:
    ctx = _fake_contact_context([], {})
    with pytest.raises(ValueError, match=message):
        sample_contact_evidence(ctx, side_ids, object_ids)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf")])
def test_sample_contact_evidence_rejects_nonfinite_force(
    monkeypatch: pytest.MonkeyPatch, bad_value: float
) -> None:
    ctx = _fake_contact_context(
        [(10, 1)], {1: "inner", 2: "outer", 10: "object"}
    )
    _install_fake_mujoco_primitives(
        monkeypatch,
        forces=[bad_value],
        distances={(1, 10): 0.0, (2, 10): 0.1},
    )
    with pytest.raises(ValueError, match="force.*finite"):
        sample_contact_evidence(ctx, ({1}, {2}), {10})


def test_sample_contact_evidence_rejects_nonfinite_distance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ctx = _fake_contact_context([], {1: "inner", 2: "outer", 10: "object"})
    _install_fake_mujoco_primitives(
        monkeypatch,
        forces=[],
        distances={(1, 10): np.nan, (2, 10): 0.1},
    )
    with pytest.raises(ValueError, match="distance.*finite"):
        sample_contact_evidence(ctx, ({1}, {2}), {10})


def test_contact_controls_have_exact_predeclared_order() -> None:
    assert CONTACT_CONTROL_NAMES == (
        "negative",
        "inner_only",
        "outer_only",
        "bilateral",
    )


def test_place_fixed_probe_preserves_robot_dynamic_state() -> None:
    import mujoco

    from openarm_robust_grasp.model import load_scene

    scene_path = str(
        __import__("pathlib").Path(__file__).parents[1]
        / "assets"
        / "cube_contact_fixed_scene.xml"
    )
    ctx = load_scene(scene_path)
    inner_ids, outer_ids = _named_finger_side_geom_ids(ctx, "left")
    finger_ids = sorted(inner_ids | outer_ids)
    probe_body_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_BODY, "contact_probe"
    )
    probe_geom_id = mujoco.mj_name2id(
        ctx.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_probe_geom"
    )
    qpos_before = ctx.data.qpos.copy()
    qvel_before = ctx.data.qvel.copy()
    ctrl_before = ctx.data.ctrl.copy()
    fingers_before = ctx.data.geom_xpos[finger_ids].copy()

    _place_fixed_probe(
        ctx,
        probe_body_id,
        probe_geom_id,
        np.array([0.0, 0.0, 3.0]),
        0.001,
    )

    assert ctx.data.qpos == pytest.approx(qpos_before)
    assert ctx.data.qvel == pytest.approx(qvel_before)
    assert ctx.data.ctrl == pytest.approx(ctrl_before)
    assert ctx.data.geom_xpos[finger_ids] == pytest.approx(fingers_before)


def test_run_contact_controls_requires_two_fresh_repeats_and_exact_classes() -> None:
    scene_path = str(
        __import__("pathlib").Path(__file__).parents[1]
        / "assets"
        / "cube_contact_fixed_scene.xml"
    )

    results = run_contact_controls(scene_path, repeats=2)

    assert [(row.repeat_index, row.expected_class) for row in results] == [
        (repeat_index, expected_class)
        for repeat_index in range(2)
        for expected_class in CONTACT_CONTROL_NAMES
    ]
    assert all(row.geometry_precondition_pass for row in results)
    assert all(row.observed_class == row.expected_class for row in results)
    assert all(row.pass_control for row in results)

    for row in results:
        if row.expected_class == "negative":
            assert row.evidence.inner_signed_gap_m >= 0.020
            assert row.evidence.outer_signed_gap_m >= 0.020
        elif row.expected_class == "inner_only":
            assert row.evidence.inner_normal_force_n > 1e-6
            assert row.evidence.outer_signed_gap_m > 0.0
        elif row.expected_class == "outer_only":
            assert row.evidence.outer_normal_force_n > 1e-6
            assert row.evidence.inner_signed_gap_m > 0.0
        else:
            assert row.evidence.inner_normal_force_n > 1e-6
            assert row.evidence.outer_normal_force_n > 1e-6
            assert row.evidence.inner_signed_gap_m <= 0.0
            assert row.evidence.outer_signed_gap_m <= 0.0


@pytest.mark.parametrize("invalid_repeats", [0, -1, 1.5, True])
def test_run_contact_controls_rejects_invalid_repeat_counts(invalid_repeats) -> None:
    with pytest.raises(ValueError, match="repeats"):
        run_contact_controls("unused.xml", repeats=invalid_repeats)


def _fixed_calibration() -> GripperCalibration:
    return GripperCalibration(
        closed_command=0.0,
        open_command=0.58905,
        closed_distance_m=0.009,
        open_distance_m=0.061,
        aperture_span_m=0.052,
        command_increases_aperture=True,
        distance_monotonic=True,
        geometric_motion_meaningful=True,
    )


@pytest.mark.parametrize(
    ("profile", "expected_endpoints"),
    [
        ("raw_full", (0.7854, 0.0)),
        ("calibrated", (0.58905, 0.0)),
    ],
)
def test_command_profile_has_exact_direction_endpoints_and_101_levels(
    profile: str, expected_endpoints: tuple[float, float]
) -> None:
    ctx = SimpleNamespace(
        model=SimpleNamespace(actuator_ctrlrange=np.array([[0.0, 0.7854]]))
    )
    arm = SimpleNamespace(gripper_actuator_id=0)

    values = command_profile(ctx, arm, _fixed_calibration(), profile)

    assert len(values) == 101
    assert values[0] > values[-1]
    assert values[[0, -1]].tolist() == pytest.approx(expected_endpoints)


def test_raw_command_profile_derives_direction_from_calibration() -> None:
    ctx = SimpleNamespace(
        model=SimpleNamespace(actuator_ctrlrange=np.array([[-2.0, 3.0]]))
    )
    arm = SimpleNamespace(gripper_actuator_id=0)
    calibration = GripperCalibration(
        closed_command=3.0,
        open_command=-1.0,
        closed_distance_m=0.01,
        open_distance_m=0.05,
        aperture_span_m=0.04,
        command_increases_aperture=False,
        distance_monotonic=True,
        geometric_motion_meaningful=True,
    )

    values = command_profile(ctx, arm, calibration, "raw_full")

    assert values[[0, -1]].tolist() == pytest.approx([-2.0, 3.0])


def test_closure_config_has_exact_fixed_protocol_defaults() -> None:
    config = ClosureConfig()

    assert config.command_levels == 101
    assert config.steps_per_level == 20
    assert config.placement_names == ("current_waypoint", "geometric_center")
    assert config.profile_names == ("raw_full", "calibrated")


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"command_levels": 1}, "command_levels"),
        ({"command_levels": True}, "command_levels"),
        ({"steps_per_level": -1}, "steps_per_level"),
        ({"steps_per_level": 1.5}, "steps_per_level"),
    ],
)
def test_closure_config_rejects_invalid_values(kwargs, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        ClosureConfig(**kwargs)


def test_command_profile_rejects_unknown_name() -> None:
    ctx = SimpleNamespace(
        model=SimpleNamespace(actuator_ctrlrange=np.array([[0.0, 0.7854]]))
    )
    arm = SimpleNamespace(gripper_actuator_id=0)
    with pytest.raises(ValueError, match="profile"):
        command_profile(ctx, arm, _fixed_calibration(), "adaptive")


def test_gripper_joint_qpos_rejects_non_joint_transmission() -> None:
    import mujoco

    ctx = SimpleNamespace(
        model=SimpleNamespace(
            actuator_trntype=np.array([int(mujoco.mjtTrn.mjTRN_TENDON)]),
            actuator_trnid=np.array([[0, -1]]),
            jnt_qposadr=np.array([0]),
        ),
        data=SimpleNamespace(qpos=np.array([0.25])),
    )
    arm = SimpleNamespace(gripper_actuator_id=0, gripper_actuator_name="gripper")
    with pytest.raises(ValueError, match="joint transmission"):
        _gripper_joint_qpos(ctx, arm)


def test_gripper_joint_qpos_reads_valid_joint_transmission() -> None:
    import mujoco

    ctx = SimpleNamespace(
        model=SimpleNamespace(
            actuator_trntype=np.array([int(mujoco.mjtTrn.mjTRN_JOINT)]),
            actuator_trnid=np.array([[1, -1]]),
            jnt_qposadr=np.array([0, 2]),
        ),
        data=SimpleNamespace(qpos=np.array([0.0, 0.1, 0.25])),
    )
    arm = SimpleNamespace(gripper_actuator_id=0, gripper_actuator_name="gripper")
    assert _gripper_joint_qpos(ctx, arm) == pytest.approx(0.25)


def test_fixed_cube_runner_uses_exact_condition_order_and_sampling_counts() -> None:
    scene_path = str(
        __import__("pathlib").Path(__file__).parents[1]
        / "assets"
        / "cube_contact_fixed_scene.xml"
    )
    config = ClosureConfig(command_levels=2, steps_per_level=2)

    results = run_fixed_cube_closure(scene_path, config)

    assert [(result.placement, result.profile) for result in results] == [
        ("current_waypoint", "raw_full"),
        ("current_waypoint", "calibrated"),
        ("geometric_center", "raw_full"),
        ("geometric_center", "calibrated"),
    ]
    assert all(result.ik_success for result in results)
    assert all(len(result.frames) == 4 for result in results)
    assert all(result.stats.sampled_frames == 4 for result in results)
    for result in results:
        assert [(frame.command_index, frame.settle_index) for frame in result.frames] == [
            (0, 0), (0, 1), (1, 0), (1, 1)
        ]
        assert result.mechanism_gate_pass is (
            result.placement == "geometric_center" and closure_gate_passes(result.stats)
        )


def test_writer_records_valid_fixed_gate_failure_and_skipped_downstream_stages(
    tmp_path,
) -> None:
    root = __import__("pathlib").Path(__file__).parents[1]
    fixed_scene = str(root / "assets" / "cube_contact_fixed_scene.xml")
    controls = run_contact_controls(fixed_scene, repeats=2)
    fixed = run_fixed_cube_closure(
        fixed_scene, ClosureConfig(command_levels=2, steps_per_level=2)
    )
    run = DiagnosticRunResult(
        controls=tuple(controls),
        fixed_closures=tuple(fixed),
        free_closures=(),
        conditional_lifts=(),
        failure_branch="fixed_mechanism_gate_failed",
        free_closure_not_run_reason="fixed_mechanism_gate_failed",
        conditional_lift_not_run_reason="free_closure_not_run",
    )

    paths = write_contact_diagnostic_report(run, tmp_path)

    assert set(paths) == {
        "contact_controls.csv",
        "fixed_closure.csv",
        "free_closure.csv",
        "conditional_lift.csv",
        "summary.json",
    }
    summary = json.loads((tmp_path / "summary.json").read_text())
    assert summary["measurement_gate_pass"] is True
    assert summary["fixed_mechanism_gate_pass"] is False
    assert summary["failure_branch"] == "fixed_mechanism_gate_failed"
    assert summary["free_closure_not_run_reason"] == "fixed_mechanism_gate_failed"
    assert summary["conditional_lift_not_run_reason"] == "free_closure_not_run"
    assert summary["control_count"] == 8
    assert summary["fixed_condition_count"] == 4
    assert summary["fixed_frame_count"] == 16
    assert (tmp_path / "free_closure.csv").read_text().splitlines() == [
        "not_run_reason"
    ]
    assert (tmp_path / "conditional_lift.csv").read_text().splitlines() == [
        "not_run_reason"
    ]


def test_cli_builder_stops_after_fixed_mechanism_gate_failure() -> None:
    import run_contact_diagnostics as runner

    root = __import__("pathlib").Path(__file__).parents[1]
    fixed_scene = str(root / "assets" / "cube_contact_fixed_scene.xml")

    result = runner.build_diagnostic_run(
        fixed_scene, ClosureConfig(command_levels=2, steps_per_level=2)
    )

    assert result.failure_branch == "fixed_mechanism_gate_failed"
    assert result.free_closures == ()
    assert result.conditional_lifts == ()
    assert result.free_closure_not_run_reason == "fixed_mechanism_gate_failed"
    assert result.conditional_lift_not_run_reason == "free_closure_not_run"
