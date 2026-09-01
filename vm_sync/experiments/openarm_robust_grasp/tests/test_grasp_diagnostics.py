import csv
import json
from dataclasses import FrozenInstanceError
from pathlib import Path

import mujoco
import numpy as np
import pytest

import openarm_robust_grasp.grasp_diagnostics as diagnostics_module
import openarm_robust_grasp.experiment as experiment_module
from openarm_robust_grasp.grasp_diagnostics import (
    GripperCalibration,
    GripperSweepSample,
    StabilityConfig,
    StabilityResult,
    _configure_runtime_cube,
    gripper_command_for_close_fraction,
    gripper_command_grid,
    infer_gripper_calibration,
    run_gripper_sweep,
    run_stability_trial,
    write_gripper_sweep_report,
)
from openarm_robust_grasp.model import load_scene


CUBE_SCENE = Path(
    "/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml"
)


def _sample(command: float, distance: float) -> GripperSweepSample:
    return GripperSweepSample(
        command=command,
        joint_qpos=0.0,
        inner_outer_center_distance_m=distance,
        min_opposing_geom_center_distance_m=distance,
        actuator_force=0.0,
    )


def test_infer_gripper_calibration_detects_increasing_aperture():
    calibration = infer_gripper_calibration(
        [_sample(0.0, 0.010), _sample(0.5, 0.020), _sample(1.0, 0.030)]
    )

    assert calibration.closed_command == pytest.approx(0.0)
    assert calibration.open_command == pytest.approx(1.0)
    assert calibration.closed_distance_m == pytest.approx(0.010)
    assert calibration.open_distance_m == pytest.approx(0.030)
    assert calibration.aperture_span_m == pytest.approx(0.020)
    assert calibration.command_increases_aperture is True
    assert calibration.distance_monotonic is True
    assert calibration.geometric_motion_meaningful is True


def test_infer_gripper_calibration_detects_decreasing_aperture():
    calibration = infer_gripper_calibration(
        [_sample(0.0, 0.030), _sample(0.5, 0.020), _sample(1.0, 0.010)]
    )

    assert calibration.closed_command == pytest.approx(1.0)
    assert calibration.open_command == pytest.approx(0.0)
    assert calibration.command_increases_aperture is False
    assert calibration.distance_monotonic is True
    assert calibration.geometric_motion_meaningful is True


def test_infer_gripper_calibration_chooses_transition_edge_of_saturated_plateaus():
    calibration = infer_gripper_calibration(
        [
            _sample(0.0, 0.010),
            _sample(0.25, 0.0100000005),
            _sample(0.5, 0.020),
            _sample(0.75, 0.030),
            _sample(1.0, 0.0300000005),
        ]
    )

    assert calibration.closed_command == pytest.approx(0.25)
    assert calibration.open_command == pytest.approx(0.75)


def test_infer_gripper_calibration_reports_insufficient_geometric_span():
    calibration = infer_gripper_calibration([_sample(0.0, 0.010), _sample(1.0, 0.014)])

    assert calibration.distance_monotonic is True
    assert calibration.aperture_span_m == pytest.approx(0.004)
    assert calibration.geometric_motion_meaningful is False


@pytest.mark.parametrize(
    ("samples", "message"),
    [
        ([_sample(0.0, 0.010)], "at least two"),
        ([_sample(0.0, 0.010), _sample(0.0, 0.020)], "strictly increasing"),
        ([_sample(0.0, 0.010), _sample(float("nan"), 0.020)], "non-finite"),
        ([_sample(0.0, 0.010), _sample(1.0, -0.020)], "negative distance"),
    ],
)
def test_infer_gripper_calibration_rejects_invalid_samples(samples, message):
    with pytest.raises(ValueError, match=message):
        infer_gripper_calibration(samples)


def test_gripper_command_for_close_fraction_interpolates_open_to_closed():
    calibration = GripperCalibration(
        closed_command=0.0,
        open_command=1.0,
        closed_distance_m=0.010,
        open_distance_m=0.030,
        aperture_span_m=0.020,
        command_increases_aperture=True,
        distance_monotonic=True,
        geometric_motion_meaningful=True,
    )

    assert gripper_command_for_close_fraction(calibration, 0.0) == pytest.approx(1.0)
    assert gripper_command_for_close_fraction(calibration, 0.25) == pytest.approx(0.75)
    assert gripper_command_for_close_fraction(calibration, 1.0) == pytest.approx(0.0)
    with pytest.raises(ValueError, match=r"within \[0, 1\]"):
        gripper_command_for_close_fraction(calibration, 1.01)


def test_gripper_command_grid_spans_model_ctrlrange_with_nine_monotonic_points():
    ctx = load_scene(str(CUBE_SCENE))

    commands = gripper_command_grid(ctx, ctx.left, points=9)
    ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[ctx.left.gripper_actuator_id]

    assert commands.shape == (9,)
    assert commands[0] == pytest.approx(ctrl_min)
    assert commands[-1] == pytest.approx(ctrl_max)
    assert np.allclose(commands, np.linspace(ctrl_min, ctrl_max, 9))
    assert np.all(np.diff(commands) > 0.0)


def test_run_gripper_sweep_returns_finite_non_negative_center_proxy_metrics():
    samples = run_gripper_sweep(str(CUBE_SCENE), arm_name="left", points=9, settle_steps=100)

    assert len(samples) == 9
    assert [sample.command for sample in samples] == pytest.approx(
        sorted(sample.command for sample in samples)
    )

    for sample in samples:
        assert isinstance(sample, GripperSweepSample)
        assert np.isfinite(
            [
                sample.command,
                sample.joint_qpos,
                sample.inner_outer_center_distance_m,
                sample.min_opposing_geom_center_distance_m,
                sample.actuator_force,
            ]
        ).all()
        assert sample.inner_outer_center_distance_m >= 0.0
        assert sample.min_opposing_geom_center_distance_m >= 0.0

    calibration = infer_gripper_calibration(samples)
    assert calibration.geometric_motion_meaningful is True
    assert calibration.command_increases_aperture is True
    assert calibration.closed_command == pytest.approx(0.0)
    max_distance = max(sample.min_opposing_geom_center_distance_m for sample in samples)
    first_open_plateau_command = min(
        sample.command
        for sample in samples
        if abs(sample.min_opposing_geom_center_distance_m - max_distance) <= 1e-9
    )
    assert calibration.open_command == pytest.approx(first_open_plateau_command)


def test_run_gripper_sweep_is_deterministic_for_same_scene_and_parameters():
    first = run_gripper_sweep(str(CUBE_SCENE), arm_name="left", points=5, settle_steps=10)
    second = run_gripper_sweep(str(CUBE_SCENE), arm_name="left", points=5, settle_steps=10)

    assert [sample.to_row() for sample in first] == [sample.to_row() for sample in second]


def test_write_gripper_sweep_report_writes_csv_and_json_summary(tmp_path):
    samples = [
        GripperSweepSample(
            command=0.0,
            joint_qpos=0.7854,
            inner_outer_center_distance_m=0.020,
            min_opposing_geom_center_distance_m=0.010,
            actuator_force=0.10,
        ),
        GripperSweepSample(
            command=0.7854,
            joint_qpos=0.0000,
            inner_outer_center_distance_m=0.040,
            min_opposing_geom_center_distance_m=0.030,
            actuator_force=0.25,
        ),
    ]

    write_gripper_sweep_report(
        samples,
        tmp_path,
        scene_path=str(CUBE_SCENE),
        arm_name="left",
        settle_steps=100,
    )

    csv_rows = list(csv.DictReader((tmp_path / "results.csv").open(encoding="utf-8", newline="")))
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))

    assert len(csv_rows) == 2
    assert float(csv_rows[0]["command"]) == pytest.approx(0.0)
    assert float(csv_rows[1]["min_opposing_geom_center_distance_m"]) == pytest.approx(0.03)

    assert summary["scene_path"] == str(CUBE_SCENE)
    assert summary["arm_name"] == "left"
    assert summary["points"] == 2
    assert summary["settle_steps"] == 100
    assert summary["command_monotonic_increasing"] is True
    assert summary["inner_outer_distance_monotonic_nonincreasing"] is False
    assert summary["min_opposing_geom_center_distance_monotonic_nonincreasing"] is False
    assert summary["inner_outer_distance_reduction_m"] == pytest.approx(-0.02)
    assert summary["min_opposing_geom_center_distance_reduction_m"] == pytest.approx(-0.02)
    assert summary["closed_command"] == pytest.approx(0.0)
    assert summary["open_command"] == pytest.approx(0.7854)
    assert summary["command_increases_aperture"] is True
    assert summary["geometric_closure_gate_passed"] is True


def test_write_gripper_sweep_report_preserves_empty_sample_compatibility(tmp_path):
    write_gripper_sweep_report(
        [],
        tmp_path,
        scene_path=str(CUBE_SCENE),
        arm_name="left",
        settle_steps=100,
    )

    csv_rows = list(csv.DictReader((tmp_path / "results.csv").open(encoding="utf-8", newline="")))
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))

    assert csv_rows == []
    assert summary["points"] == 0
    assert summary["command_monotonic_increasing"] is False
    assert summary["inner_outer_distance_monotonic_nonincreasing"] is False
    assert summary["min_opposing_geom_center_distance_monotonic_nonincreasing"] is False
    assert summary["inner_outer_distance_reduction_m"] == 0.0
    assert summary["min_opposing_geom_center_distance_reduction_m"] == 0.0
    assert summary["closed_command"] is None
    assert summary["open_command"] is None
    assert summary["closed_distance_m"] is None
    assert summary["open_distance_m"] is None
    assert summary["aperture_span_m"] is None
    assert summary["command_increases_aperture"] is None
    assert summary["distance_monotonic"] is False
    assert summary["geometric_motion_meaningful"] is False
    assert summary["geometric_closure_gate_passed"] is False


def _stability_config(**overrides) -> StabilityConfig:
    values = {
        "close_fraction": 1.0,
        "cube_width_m": 0.046,
        "sliding_friction": 4.0,
    }
    values.update(overrides)
    return StabilityConfig(**values)


def _stability_result(**overrides) -> StabilityResult:
    values = {
        "close_fraction": 1.0,
        "resolved_close_command": 0.0,
        "cube_width_m": 0.046,
        "sliding_friction": 4.0,
        "cube_mass_kg": 0.015,
        "seed": 0,
        "ik_success": True,
        "close_sampled_frames": 100,
        "lift_sampled_frames": 120,
        "pre_disturb_hold_sampled_frames": 500,
        "disturbance_sampled_frames": 100,
        "post_disturb_hold_sampled_frames": 250,
        "achieved_lift_height_m": 0.08,
        "final_lift_height_m": 0.05,
        "pre_disturb_elevation_m": 0.05,
        "close_two_finger_contact_fraction": 0.5,
        "lift_two_finger_contact_fraction": 1.0,
        "pre_disturb_hold_two_finger_contact_fraction": 0.5,
        "disturbance_two_finger_contact_fraction": 0.5,
        "post_disturb_hold_two_finger_contact_fraction": 0.5,
        "max_object_gripper_displacement_m": 0.01,
        "final_object_gripper_displacement_m": 0.01,
        "disturbance_drop_m": 0.02,
        "total_steps": 1330,
    }
    values.update(overrides)
    return StabilityResult(**values)


def _fixed_calibration() -> GripperCalibration:
    return GripperCalibration(
        closed_command=0.0,
        open_command=0.58905,
        closed_distance_m=0.01,
        open_distance_m=0.03,
        aperture_span_m=0.02,
        command_increases_aperture=True,
        distance_monotonic=True,
        geometric_motion_meaningful=True,
    )


def test_stability_config_is_frozen_and_has_exact_protocol_defaults():
    config = _stability_config()

    assert config.seed == 0
    assert config.arm_name == "left"
    assert config.object_name == "orange_cube"
    assert config.cube_mass_kg == pytest.approx(0.015)
    assert config.lift_height_m == pytest.approx(0.08)
    assert config.open_settle_steps == 100
    assert config.close_settle_steps == 100
    assert config.approach_steps == 80
    assert config.grasp_steps == 80
    assert config.lift_steps == 120
    assert config.pre_disturb_hold_steps == 500
    assert config.disturb_steps == 100
    assert config.post_disturb_hold_steps == 250
    assert config.disturbance_force_n == (0.05, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        config.seed = 1


@pytest.mark.parametrize(
    ("field_name", "invalid_value", "message"),
    [
        ("close_fraction", float("nan"), "close_fraction"),
        ("close_fraction", 1.01, "close_fraction"),
        ("cube_width_m", 0.0, "cube_width_m"),
        ("sliding_friction", float("inf"), "sliding_friction"),
        ("cube_mass_kg", -0.01, "cube_mass_kg"),
        ("lift_height_m", 0.0, "lift_height_m"),
        ("open_settle_steps", -1, "open_settle_steps"),
        ("grasp_steps", 1.5, "grasp_steps"),
        ("disturbance_force_n", (0.05, 0.0), "disturbance_force_n"),
        ("disturbance_force_n", (0.05, float("nan"), 0.0), "disturbance_force_n"),
    ],
)
def test_stability_config_rejects_invalid_values(field_name, invalid_value, message):
    with pytest.raises(ValueError, match=message):
        _stability_config(**{field_name: invalid_value})


def test_runtime_cube_configuration_mutates_only_loaded_context_and_preserves_other_friction():
    original = load_scene(str(CUBE_SCENE))
    body_id = mujoco.mj_name2id(
        original.model, mujoco.mjtObj.mjOBJ_BODY, "orange_cube"
    )
    geom_id = mujoco.mj_name2id(
        original.model, mujoco.mjtObj.mjOBJ_GEOM, "orange_cube_geom"
    )
    original_size = original.model.geom_size[geom_id].copy()
    original_friction = original.model.geom_friction[geom_id].copy()
    original_mass = float(original.model.body_mass[body_id])
    original_inertia = original.model.body_inertia[body_id].copy()
    original_qpos = original.data.qpos.copy()
    original_qvel = original.data.qvel.copy()
    original_ctrl = original.data.ctrl.copy()
    original_time = float(original.data.time)
    config = _stability_config(
        cube_width_m=0.04,
        sliding_friction=1.75,
        cube_mass_kg=0.02,
    )

    resolved_body_id, resolved_geom_id = _configure_runtime_cube(original, config)

    assert resolved_body_id == body_id
    assert resolved_geom_id == geom_id
    np.testing.assert_allclose(original.model.geom_size[geom_id], np.full(3, 0.02))
    assert original.model.geom_friction[geom_id, 0] == pytest.approx(1.75)
    np.testing.assert_allclose(original.model.geom_friction[geom_id, 1:], original_friction[1:])
    assert original.model.body_mass[body_id] == pytest.approx(0.02)
    np.testing.assert_allclose(
        original.model.body_inertia[body_id],
        np.full(3, 0.02 * 0.04**2 / 6.0),
    )
    np.testing.assert_allclose(original.data.qpos, original_qpos)
    np.testing.assert_allclose(original.data.qvel, original_qvel)
    np.testing.assert_allclose(original.data.ctrl, original_ctrl)
    assert float(original.data.time) == pytest.approx(original_time)

    reloaded = load_scene(str(CUBE_SCENE))
    np.testing.assert_allclose(reloaded.model.geom_size[geom_id], original_size)
    np.testing.assert_allclose(reloaded.model.geom_friction[geom_id], original_friction)
    assert reloaded.model.body_mass[body_id] == pytest.approx(original_mass)
    np.testing.assert_allclose(reloaded.model.body_inertia[body_id], original_inertia)


def test_stability_trial_uses_close_fraction_mapping_and_exact_pose_lift_target(monkeypatch):
    calibration = _fixed_calibration()
    mapping_calls: list[tuple[GripperCalibration, float]] = []
    move_targets: list[np.ndarray] = []
    pose_targets: list[np.ndarray] = []
    original_mapping = diagnostics_module.gripper_command_for_close_fraction

    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: calibration,
    )

    def spy_mapping(observed_calibration, close_fraction):
        mapping_calls.append((observed_calibration, close_fraction))
        return original_mapping(observed_calibration, close_fraction)

    def fake_move_to(_ctx, _arm, target, _rng, _noise, steps=80, step_observer=None):
        move_targets.append(np.asarray(target).copy())
        return True, 0.0, 0

    def fake_move_to_pose(
        _ctx,
        _arm,
        target,
        _target_rot,
        _rng,
        _noise,
        steps=120,
        step_observer=None,
    ):
        pose_targets.append(np.asarray(target).copy())
        return True, 0.0, 0

    monkeypatch.setattr(diagnostics_module, "gripper_command_for_close_fraction", spy_mapping)
    monkeypatch.setattr(experiment_module, "_move_to", fake_move_to)
    monkeypatch.setattr(experiment_module, "_move_to_pose", fake_move_to_pose)

    config = _stability_config(
        close_fraction=0.25,
        open_settle_steps=0,
        close_settle_steps=0,
        approach_steps=0,
        grasp_steps=0,
        lift_steps=0,
        pre_disturb_hold_steps=0,
        disturb_steps=0,
        post_disturb_hold_steps=0,
    )
    result = run_stability_trial(str(CUBE_SCENE), config)

    assert mapping_calls == [(calibration, 0.25)]
    assert result.resolved_close_command == pytest.approx(0.4417875)
    assert len(move_targets) == 2
    assert len(pose_targets) == 1
    np.testing.assert_allclose(
        pose_targets[0] - move_targets[1],
        np.array([0.0, 0.0, config.lift_height_m]),
    )


def test_stability_trial_reports_exact_default_phase_counts_metric_bounds_and_determinism(monkeypatch):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    config = _stability_config()

    first = run_stability_trial(str(CUBE_SCENE), config)
    second = run_stability_trial(str(CUBE_SCENE), config)

    assert first.to_row() == second.to_row()
    assert first.ik_success
    assert first.close_sampled_frames == 100
    assert first.lift_sampled_frames == 120
    assert first.pre_disturb_hold_sampled_frames == 500
    assert first.disturbance_sampled_frames == 100
    assert first.post_disturb_hold_sampled_frames == 250
    assert first.total_steps == 1330
    assert first.achieved_lift_height_m >= 0.0
    assert first.max_object_gripper_displacement_m >= 0.0
    assert first.final_object_gripper_displacement_m >= 0.0
    assert first.disturbance_drop_m >= 0.0
    for fraction in (
        first.close_two_finger_contact_fraction,
        first.lift_two_finger_contact_fraction,
        first.pre_disturb_hold_two_finger_contact_fraction,
        first.disturbance_two_finger_contact_fraction,
        first.post_disturb_hold_two_finger_contact_fraction,
    ):
        assert 0.0 <= fraction <= 1.0


def test_stability_trial_applies_force_only_during_disturbance_and_clears_it(monkeypatch):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    original_step = diagnostics_module.mujoco.mj_step
    observed_forces: list[tuple[float, float, float]] = []
    final_data = None
    final_model = None

    def spy_step(model, data):
        nonlocal final_data, final_model
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "orange_cube")
        observed_forces.append(tuple(float(value) for value in data.xfrc_applied[body_id, :3]))
        final_data = data
        final_model = model
        return original_step(model, data)

    monkeypatch.setattr(diagnostics_module.mujoco, "mj_step", spy_step)
    config = _stability_config(
        open_settle_steps=5,
        close_settle_steps=5,
        pre_disturb_hold_steps=3,
        disturb_steps=4,
        post_disturb_hold_steps=2,
        disturbance_force_n=(0.03, -0.01, 0.0),
    )

    result = run_stability_trial(str(CUBE_SCENE), config)

    disturbance_start = 5 + 80 + 80 + 5 + 120 + 3
    assert result.ik_success
    assert len(observed_forces) == result.total_steps
    assert observed_forces[:disturbance_start] == [(0.0, 0.0, 0.0)] * disturbance_start
    assert observed_forces[disturbance_start : disturbance_start + 4] == [
        config.disturbance_force_n
    ] * 4
    assert observed_forces[disturbance_start + 4 :] == [(0.0, 0.0, 0.0)] * 2
    assert final_data is not None
    assert final_model is not None
    body_id = mujoco.mj_name2id(final_model, mujoco.mjtObj.mjOBJ_BODY, "orange_cube")
    np.testing.assert_allclose(final_data.xfrc_applied[body_id, :3], np.zeros(3))


def test_stability_trial_loads_a_fresh_context_for_every_call(monkeypatch):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    original_load_scene = diagnostics_module.load_scene
    contexts = []

    def spy_load_scene(scene_path):
        ctx = original_load_scene(scene_path)
        contexts.append(ctx)
        return ctx

    monkeypatch.setattr(diagnostics_module, "load_scene", spy_load_scene)
    config = _stability_config(
        open_settle_steps=0,
        close_settle_steps=0,
        approach_steps=0,
        grasp_steps=0,
        lift_steps=0,
        pre_disturb_hold_steps=0,
        disturb_steps=0,
        post_disturb_hold_steps=0,
    )

    run_stability_trial(str(CUBE_SCENE), config)
    run_stability_trial(str(CUBE_SCENE), config)

    assert len(contexts) == 2
    assert contexts[0] is not contexts[1]
    assert contexts[0].model is not contexts[1].model


def test_stability_trial_ik_failure_skips_all_later_phases_and_returns_zero_safe_metrics(
    monkeypatch,
):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    monkeypatch.setattr(
        experiment_module,
        "_move_to",
        lambda *_args, **_kwargs: (False, 0.123, 0),
    )
    config = _stability_config(open_settle_steps=3)

    result = run_stability_trial(str(CUBE_SCENE), config)

    assert not result.ik_success
    assert result.close_sampled_frames == 0
    assert result.lift_sampled_frames == 0
    assert result.pre_disturb_hold_sampled_frames == 0
    assert result.disturbance_sampled_frames == 0
    assert result.post_disturb_hold_sampled_frames == 0
    assert result.achieved_lift_height_m == 0.0
    assert result.final_lift_height_m == 0.0
    assert result.pre_disturb_elevation_m == 0.0
    assert result.close_two_finger_contact_fraction == 0.0
    assert result.lift_two_finger_contact_fraction == 0.0
    assert result.pre_disturb_hold_two_finger_contact_fraction == 0.0
    assert result.disturbance_two_finger_contact_fraction == 0.0
    assert result.post_disturb_hold_two_finger_contact_fraction == 0.0
    assert result.max_object_gripper_displacement_m == 0.0
    assert result.final_object_gripper_displacement_m == 0.0
    assert result.disturbance_drop_m == 0.0
    assert not result.final_elevation_success
    assert not result.hold_contact_success
    assert not result.disturbance_eligible
    assert result.disturbance_survival_success is None
    assert not result.stable_grasp_success
    assert result.total_steps == 3


@pytest.mark.parametrize("ik_success", [True, False])
@pytest.mark.parametrize(
    (
        "final_lift_height_m",
        "pre_disturb_elevation_m",
        "pre_hold_fraction",
        "post_hold_fraction",
        "disturbance_drop_m",
        "expected_elevation",
        "expected_hold",
        "expected_eligible",
        "expected_survival",
        "expected_stable",
    ),
    [
        (0.05, 0.05, 0.5, 0.5, 0.02, True, True, True, True, True),
        (0.049, 0.05, 0.5, 0.5, 0.02, False, True, True, True, False),
        (0.05, 0.049, 0.5, 0.5, 0.02, True, True, False, None, False),
        (0.05, 0.05, 0.499, 0.5, 0.02, True, False, False, None, False),
        (0.05, 0.05, 0.5, 0.499, 0.02, True, False, True, True, False),
        (0.05, 0.05, 0.5, 0.5, 0.0201, True, True, True, False, False),
    ],
)
def test_stability_result_uses_exact_success_thresholds_and_conjunction(
    ik_success,
    final_lift_height_m,
    pre_disturb_elevation_m,
    pre_hold_fraction,
    post_hold_fraction,
    disturbance_drop_m,
    expected_elevation,
    expected_hold,
    expected_eligible,
    expected_survival,
    expected_stable,
):
    result = _stability_result(
        ik_success=ik_success,
        final_lift_height_m=final_lift_height_m,
        pre_disturb_elevation_m=pre_disturb_elevation_m,
        pre_disturb_hold_two_finger_contact_fraction=pre_hold_fraction,
        post_disturb_hold_two_finger_contact_fraction=post_hold_fraction,
        disturbance_drop_m=disturbance_drop_m,
    )

    assert result.final_elevation_success is expected_elevation
    assert result.hold_contact_success is expected_hold
    assert result.disturbance_eligible is expected_eligible
    assert result.disturbance_survival_success is expected_survival
    assert result.stable_grasp_success is expected_stable
    assert result.stable_grasp_success == (
        result.final_elevation_success
        and result.hold_contact_success
        and result.disturbance_survival_success is True
    )
    assert result.to_row()["stable_grasp_success"] is expected_stable


def test_stability_trial_uses_post_open_settle_height_as_lift_baseline(monkeypatch):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    monkeypatch.setattr(
        experiment_module,
        "_move_to",
        lambda *_args, **_kwargs: (True, 0.0, 0),
    )
    monkeypatch.setattr(
        experiment_module,
        "_move_to_pose",
        lambda *_args, **_kwargs: (True, 0.0, 0),
    )
    original_step = diagnostics_module.mujoco.mj_step
    settled_heights = []

    def observed_open_settle_step(model, data):
        original_step(model, data)
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "orange_cube"
        )
        settled_heights.append(float(data.xpos[body_id, 2]))

    monkeypatch.setattr(diagnostics_module.mujoco, "mj_step", observed_open_settle_step)
    config = _stability_config(
        open_settle_steps=1,
        close_settle_steps=0,
        approach_steps=0,
        grasp_steps=0,
        lift_steps=0,
        pre_disturb_hold_steps=0,
        disturb_steps=0,
        post_disturb_hold_steps=0,
    )

    result = run_stability_trial(str(CUBE_SCENE), config)

    assert len(settled_heights) == 1
    assert result.achieved_lift_height_m == 0.0
    assert result.pre_disturb_elevation_m == pytest.approx(0.0)
    assert result.final_lift_height_m == pytest.approx(0.0)


def test_stability_trial_samples_every_close_settle_step(monkeypatch):
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: _fixed_calibration(),
    )
    monkeypatch.setattr(
        experiment_module,
        "_move_to",
        lambda *_args, **_kwargs: (True, 0.0, 0),
    )
    monkeypatch.setattr(
        experiment_module,
        "_move_to_pose",
        lambda *_args, **_kwargs: (True, 0.0, 0),
    )
    frames = iter(
        [
            experiment_module.ContactFrame(1, True, True, True, True),
            experiment_module.ContactFrame(0, False, False, False, False),
            experiment_module.ContactFrame(1, True, True, True, True),
        ]
    )
    monkeypatch.setattr(
        experiment_module,
        "_sample_contact_frame",
        lambda *_args, **_kwargs: next(frames),
    )
    config = _stability_config(
        open_settle_steps=0,
        close_settle_steps=3,
        approach_steps=0,
        grasp_steps=0,
        lift_steps=0,
        pre_disturb_hold_steps=0,
        disturb_steps=0,
        post_disturb_hold_steps=0,
    )

    result = run_stability_trial(str(CUBE_SCENE), config)

    assert result.close_sampled_frames == 3
    assert result.close_two_finger_contact_fraction == pytest.approx(2 / 3)
    assert result.total_steps == 3
