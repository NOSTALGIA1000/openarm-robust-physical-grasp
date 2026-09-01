from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

import openarm_robust_grasp.experiment as experiment_module
from openarm_robust_grasp.control import (
    IKResult,
    finger_collision_center,
    forward_kinematics,
    set_gripper,
)
from openarm_robust_grasp.grasp_diagnostics import GripperCalibration, GripperSweepSample
from openarm_robust_grasp.experiment import (
    ContactFrame,
    NoiseConfig,
    StageContactStats,
    TrialResult,
    _calibrated_gripper_control_endpoints,
    _finger_center_contact_waypoints,
    _finger_side_geom_ids,
    _sample_contact_frame,
    run_trial,
)
from openarm_robust_grasp.model import load_scene
from openarm_robust_grasp.report import summarize


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")
CUBE_SCENE = Path(
    "/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml"
)
PER_STEP_STAGE_FIELDS = {
    "approach_sampled_frames",
    "grasp_sampled_frames",
    "lift_sampled_frames",
    "approach_contact_frames",
    "grasp_contact_frames",
    "lift_contact_frames",
    "approach_two_finger_contact_frames",
    "grasp_two_finger_contact_frames",
    "lift_two_finger_contact_frames",
    "approach_contact_frame_rate",
    "grasp_contact_frame_rate",
    "lift_contact_frame_rate",
    "grasp_two_finger_contact_success",
    "lift_two_finger_contact_success",
    "sustained_lift_contact_success",
}
FORMAL_CONTACT_POLICIES = (
    "ee_base_contact_push",
    "finger_center_contact_push",
    "pre_grasp_finger_center_correction",
)
DEMO_CUBE_SCENE = "/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/demo.xml"


def _fake_contact_ctx(
    geom_names: dict[int, str], contacts: list[tuple[int, int]]
):
    return SimpleNamespace(
        model=SimpleNamespace(ngeom=len(geom_names)),
        data=SimpleNamespace(
            ncon=len(contacts),
            contact=[SimpleNamespace(geom1=g1, geom2=g2) for g1, g2 in contacts],
        ),
    )


def _trial_result(**overrides) -> TrialResult:
    defaults = {
        "policy": "finger_center_contact_push",
        "seed": 0,
        "translation_std_m": 0.01,
        "fixed_bias_x_m": 0.0,
        "fixed_bias_y_m": 0.0,
        "fixed_bias_z_m": 0.0,
        "command_noise_std_rad": 0.0,
        "ik_success": True,
        "proxy_success": True,
        "contact_success": True,
        "grasp_success": False,
        "final_ee_error_m": 0.0,
        "min_ee_object_distance_m": 0.02,
        "final_ee_object_distance_m": 0.03,
        "object_motion_m": 0.04,
        "object_lift_m": 0.01,
        "finger_object_contacts": 10,
        "approach_finger_object_contacts": 1,
        "grasp_finger_object_contacts": 6,
        "lift_finger_object_contacts": 3,
        "approach_sampled_frames": 80,
        "grasp_sampled_frames": 120,
        "lift_sampled_frames": 80,
        "approach_contact_frames": 20,
        "grasp_contact_frames": 60,
        "lift_contact_frames": 40,
        "approach_two_finger_contact_frames": 10,
        "grasp_two_finger_contact_frames": 30,
        "lift_two_finger_contact_frames": 20,
        "approach_contact_frame_rate": 0.25,
        "grasp_contact_frame_rate": 0.5,
        "lift_contact_frame_rate": 0.5,
        "grasp_two_finger_contact_success": False,
        "lift_two_finger_contact_success": False,
        "sustained_lift_contact_success": False,
        "post_grasp_contact_success": True,
        "lift_contact_success": True,
        "lift_height_m": 0.12,
        "steps": 280,
    }
    defaults.update(overrides)
    return TrialResult(**defaults)


def _expected_contact_frame_from_contacts(
    ctx,
    inner_geom_ids: set[int],
    outer_geom_ids: set[int],
    object_geom_ids: set[int],
) -> ContactFrame:
    contact_pairs: set[tuple[int, int]] = set()
    inner_contact = False
    outer_contact = False

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

        if finger_geom_id in inner_geom_ids:
            inner_contact = True
        elif finger_geom_id in outer_geom_ids:
            outer_contact = True
        else:
            continue

        contact_pairs.add(tuple(sorted((object_geom_id, finger_geom_id))))

    return ContactFrame(
        contact_pairs=len(contact_pairs),
        left_finger_contact=inner_contact,
        right_finger_contact=outer_contact,
        any_finger_contact=inner_contact or outer_contact,
        two_finger_contact=inner_contact and outer_contact,
    )


def _assert_per_step_stage_metrics(
    result,
    *,
    expected_approach_frames: int,
    expected_grasp_frames: int,
    expected_lift_frames: int,
    expected_steps: int,
    expected_pre_open_steps: int = 100,
) -> None:
    row = result.to_row()

    assert PER_STEP_STAGE_FIELDS <= set(row)
    for field_name in PER_STEP_STAGE_FIELDS:
        assert getattr(result, field_name) == row[field_name]

    assert result.approach_sampled_frames == expected_approach_frames
    assert result.grasp_sampled_frames == expected_grasp_frames
    assert result.lift_sampled_frames == expected_lift_frames
    assert (
        result.approach_sampled_frames + result.grasp_sampled_frames + result.lift_sampled_frames
        == expected_steps - expected_pre_open_steps
    )
    assert result.steps == expected_steps

    for stage_name in ("approach", "grasp", "lift"):
        sampled_frames = getattr(result, f"{stage_name}_sampled_frames")
        contact_frames = getattr(result, f"{stage_name}_contact_frames")
        two_finger_frames = getattr(result, f"{stage_name}_two_finger_contact_frames")
        contact_frame_rate = getattr(result, f"{stage_name}_contact_frame_rate")

        assert 0 <= contact_frames <= sampled_frames
        assert 0 <= two_finger_frames <= contact_frames
        assert 0.0 <= contact_frame_rate <= 1.0
        if sampled_frames == 0:
            assert contact_frame_rate == 0.0
        else:
            assert contact_frame_rate == pytest.approx(contact_frames / sampled_frames)

    assert result.grasp_two_finger_contact_success == (
        result.ik_success and result.grasp_two_finger_contact_frames > 0
    )
    assert result.lift_two_finger_contact_success == (
        result.ik_success and result.lift_two_finger_contact_frames > 0
    )
    assert result.sustained_lift_contact_success == (
        result.ik_success
        and result.lift_sampled_frames > 0
        and result.lift_two_finger_contact_frames / result.lift_sampled_frames >= 0.5
    )


def test_stage_contact_stats_zero_frame_rates_are_zero():
    stats = StageContactStats()

    assert stats.sampled_frames == 0
    assert stats.contact_frames == 0
    assert stats.two_finger_frames == 0
    assert stats.contact_frame_rate == 0.0
    assert stats.two_finger_contact_frame_rate == 0.0


def test_summary_reports_exact_per_step_contact_frame_means_and_boolean_rates():
    summary = summarize(
        [
            _trial_result(
                policy="pre_grasp_finger_center_correction",
                translation_std_m=0.03,
                approach_contact_frame_rate=0.25,
                grasp_contact_frame_rate=0.5,
                lift_contact_frame_rate=0.0,
                grasp_two_finger_contact_success=True,
                lift_two_finger_contact_success=False,
                sustained_lift_contact_success=False,
            ),
            _trial_result(
                policy="pre_grasp_finger_center_correction",
                seed=1,
                translation_std_m=0.03,
                approach_contact_frame_rate=0.75,
                grasp_contact_frame_rate=0.25,
                lift_contact_frame_rate=1.0,
                grasp_two_finger_contact_success=False,
                lift_two_finger_contact_success=True,
                sustained_lift_contact_success=True,
            ),
        ]
    )

    assert len(summary) == 1
    row = summary[0]

    assert row["policy"] == "pre_grasp_finger_center_correction"
    assert row["translation_std_m"] == pytest.approx(0.03)
    assert row["trials"] == 2
    assert row["mean_approach_contact_frame_rate"] == pytest.approx(0.5)
    assert row["mean_grasp_contact_frame_rate"] == pytest.approx(0.375)
    assert row["mean_lift_contact_frame_rate"] == pytest.approx(0.5)
    assert row["grasp_two_finger_contact_success_rate"] == pytest.approx(0.5)
    assert row["lift_two_finger_contact_success_rate"] == pytest.approx(0.5)
    assert row["sustained_lift_contact_success_rate"] == pytest.approx(0.5)


def test_finger_side_geom_ids_groups_geom_names_by_opposing_side(monkeypatch):
    geom_names = {
        0: "finger_inner_left_collision_pad",
        1: "finger_outer_left_collision_pad",
        2: "table_geom",
    }
    ctx = _fake_contact_ctx(geom_names, contacts=[])
    monkeypatch.setattr(experiment_module, "_geom_name", lambda _ctx, geom_id: geom_names[geom_id])

    left_ids, right_ids = _finger_side_geom_ids(ctx, "left")

    assert set(left_ids) == {0}
    assert set(right_ids) == {1}


def test_finger_side_geom_ids_rejects_missing_inner_group(monkeypatch):
    geom_names = {
        0: "finger_outer_left_collision_pad",
        1: "table_geom",
    }
    ctx = _fake_contact_ctx(geom_names, contacts=[])
    monkeypatch.setattr(experiment_module, "_geom_name", lambda _ctx, geom_id: geom_names[geom_id])

    with pytest.raises(ValueError, match="Missing inner finger collision geoms"):
        _finger_side_geom_ids(ctx, "left")


def test_finger_side_geom_ids_rejects_missing_outer_group(monkeypatch):
    geom_names = {
        0: "finger_inner_left_collision_pad",
        1: "table_geom",
    }
    ctx = _fake_contact_ctx(geom_names, contacts=[])
    monkeypatch.setattr(experiment_module, "_geom_name", lambda _ctx, geom_id: geom_names[geom_id])

    with pytest.raises(ValueError, match="Missing outer finger collision geoms"):
        _finger_side_geom_ids(ctx, "left")


def test_finger_side_geom_ids_rejects_ambiguous_geom_name(monkeypatch):
    geom_names = {
        0: "finger_inner_left_collision_pad",
        1: "finger_side_left_collision_pad",
        2: "finger_outer_left_collision_pad",
    }
    ctx = _fake_contact_ctx(geom_names, contacts=[])
    monkeypatch.setattr(experiment_module, "_geom_name", lambda _ctx, geom_id: geom_names[geom_id])

    with pytest.raises(ValueError, match="Ambiguous finger collision geom"):
        _finger_side_geom_ids(ctx, "left")


def test_sample_contact_frame_rejects_overlapping_cached_finger_side_ids():
    ctx = _fake_contact_ctx({0: "geom0", 1: "geom1", 2: "geom2"}, contacts=[(1, 2)])

    with pytest.raises(ValueError, match="Overlapping finger side geom ids"):
        _sample_contact_frame(ctx, ({0, 1}, {1}), {2})


def test_sample_contact_frame_uses_cached_finger_side_ids_without_reparsing(monkeypatch):
    geom_names = {
        0: "finger_inner_left_collision_tip_a",
        1: "finger_inner_left_collision_tip_b",
        2: "finger_outer_left_collision_tip_a",
        3: "orange_cube_geom",
        4: "table_geom",
    }
    ctx = _fake_contact_ctx(
        geom_names,
        contacts=[
            (0, 3),
            (1, 3),
            (2, 3),
            (0, 4),
        ],
    )
    monkeypatch.setattr(
        experiment_module,
        "_finger_side_geom_ids",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("should use cached geom ids")),
    )

    frame = _sample_contact_frame(ctx, ({0, 1}, {2}), {3})

    assert frame == ContactFrame(
        contact_pairs=3,
        left_finger_contact=True,
        right_finger_contact=True,
        any_finger_contact=True,
        two_finger_contact=True,
    )

    stats = StageContactStats()
    stats.observe(frame)
    stats.observe(
        ContactFrame(
            contact_pairs=2,
            left_finger_contact=True,
            right_finger_contact=False,
            any_finger_contact=True,
            two_finger_contact=False,
        )
    )

    assert stats.sampled_frames == 2
    assert stats.contact_frames == 2
    assert stats.two_finger_frames == 1
    assert stats.contact_frame_rate == 1.0
    assert stats.two_finger_contact_frame_rate == 0.5


def test_real_cube_scene_contact_frame_matches_independent_contact_oracle():
    ctx = load_scene(str(CUBE_SCENE))
    arm = ctx.left
    rng = np.random.default_rng(0)
    noise = NoiseConfig()
    object_name = "orange_cube"
    object_geoms = experiment_module._object_geom_ids(ctx, object_name)
    left_geom_ids, right_geom_ids = _finger_side_geom_ids(ctx, arm.prefix)

    left_geom_names = {experiment_module._geom_name(ctx, geom_id) for geom_id in left_geom_ids}
    right_geom_names = {experiment_module._geom_name(ctx, geom_id) for geom_id in right_geom_ids}

    assert len(left_geom_ids) == 4
    assert len(right_geom_ids) == 4
    assert all(name.startswith("finger_inner_left_collision_") for name in left_geom_names)
    assert all(name.startswith("finger_outer_left_collision_") for name in right_geom_names)

    perceived = experiment_module._object_pos(ctx, object_name)
    approach, grasp, _ = _finger_center_contact_waypoints(ctx, arm, perceived)

    ok, _, _ = experiment_module._move_to(ctx, arm, approach, rng, noise)
    assert ok
    ok, _, _ = experiment_module._move_to(ctx, arm, grasp, rng, noise)
    assert ok

    set_gripper(ctx, arm, 0.0)

    frame = None
    expected = None
    for _ in range(40):
        mujoco.mj_step(ctx.model, ctx.data)
        expected = _expected_contact_frame_from_contacts(ctx, left_geom_ids, right_geom_ids, object_geoms)
        if expected.contact_pairs > 0:
            frame = _sample_contact_frame(ctx, (left_geom_ids, right_geom_ids), object_geoms)
            break

    assert frame is not None
    assert expected is not None
    assert frame == expected


def test_zero_noise_open_loop_trial_returns_metrics():
    result = run_trial(str(SCENE), policy="open_loop", noise=NoiseConfig(), seed=0)

    assert result.policy == "open_loop"
    assert result.seed == 0
    assert result.ik_success
    assert result.proxy_success
    assert result.grasp_success == result.contact_success
    assert result.final_ee_error_m >= 0.0
    assert result.min_ee_object_distance_m >= 0.0
    assert result.object_motion_m >= 0.0
    assert result.finger_object_contacts >= 0


def test_correction_policy_is_deterministic_for_same_seed():
    noise = NoiseConfig(translation_std_m=0.01)

    first = run_trial(str(SCENE), policy="pre_grasp_correction", noise=noise, seed=7)
    second = run_trial(str(SCENE), policy="pre_grasp_correction", noise=noise, seed=7)

    assert first.to_row() == second.to_row()


def test_demo_cube_trial_reports_contact_aware_fields():
    scene = "/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/demo.xml"

    result = run_trial(scene, policy="open_loop", noise=NoiseConfig(), seed=0, object_name="orange_cube")
    row = result.to_row()

    assert "proxy_success" in row
    assert "contact_success" in row
    assert "object_motion_m" in row
    assert "finger_object_contacts" in row
    assert "approach_finger_object_contacts" in row
    assert "grasp_finger_object_contacts" in row
    assert "lift_finger_object_contacts" in row
    assert "post_grasp_contact_success" in row
    assert "lift_contact_success" in row
    assert result.object_motion_m >= 0.0


def test_contact_push_policy_touches_demo_cube():
    result = run_trial(
        str(CUBE_SCENE), policy="contact_push", noise=NoiseConfig(), seed=0, object_name="orange_cube"
    )

    assert result.ik_success
    assert result.finger_object_contacts > 0
    assert result.contact_success
    assert not result.grasp_success
    assert result.object_motion_m > 0.005


def test_stage_contact_counts_partition_total_contacts():
    result = run_trial(
        str(CUBE_SCENE), policy="pre_grasp_finger_center_correction", noise=NoiseConfig(), seed=0, object_name="orange_cube"
    )

    assert result.approach_finger_object_contacts >= 0
    assert result.grasp_finger_object_contacts >= 0
    assert result.lift_finger_object_contacts >= 0
    assert (
        result.approach_finger_object_contacts
        + result.grasp_finger_object_contacts
        + result.lift_finger_object_contacts
        == result.finger_object_contacts
    )
    assert result.post_grasp_contact_success == (
        result.grasp_finger_object_contacts + result.lift_finger_object_contacts > 0
    )
    assert result.lift_contact_success == (result.lift_finger_object_contacts > 0)


@pytest.mark.parametrize("policy", FORMAL_CONTACT_POLICIES)
def test_formal_contact_policies_report_expected_stage_totals(policy):
    result = run_trial(
        str(CUBE_SCENE),
        policy=policy,
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert result.ik_success
    _assert_per_step_stage_metrics(
        result,
        expected_approach_frames=80,
        expected_grasp_frames=120,
        expected_lift_frames=80,
        expected_steps=380,
    )


def test_cube_lift_reports_expected_stage_totals_without_changing_120_step_pose_lift():
    result = run_trial(DEMO_CUBE_SCENE, policy="cube_lift", noise=NoiseConfig(), seed=0, object_name="orange_cube")

    assert result.ik_success
    _assert_per_step_stage_metrics(
        result,
        expected_approach_frames=80,
        expected_grasp_frames=120,
        expected_lift_frames=120,
        expected_steps=420,
    )


def test_inverse_direction_calibration_swaps_control_range_endpoints(monkeypatch):
    ctx = load_scene(str(CUBE_SCENE))
    inverse_calibration = GripperCalibration(
        closed_command=1.0,
        open_command=0.0,
        closed_distance_m=0.010,
        open_distance_m=0.030,
        aperture_span_m=0.020,
        command_increases_aperture=False,
        distance_monotonic=True,
        geometric_motion_meaningful=True,
    )
    monkeypatch.setattr(
        experiment_module,
        "_gripper_calibration_for_scene",
        lambda _scene_path, _arm_name: inverse_calibration,
        raising=False,
    )

    closed_command, open_command = _calibrated_gripper_control_endpoints(ctx, ctx.left)
    ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[ctx.left.gripper_actuator_id]

    assert closed_command == pytest.approx(ctrl_max)
    assert open_command == pytest.approx(ctrl_min)


def test_gripper_calibration_for_scene_is_cached(monkeypatch):
    calls: list[tuple[str, str]] = []
    samples = [
        GripperSweepSample(0.0, 0.0, 0.010, 0.010, 0.0),
        GripperSweepSample(1.0, 0.0, 0.030, 0.030, 0.0),
    ]

    def fake_sweep(scene_path, arm_name):
        calls.append((scene_path, arm_name))
        return samples

    monkeypatch.setattr(experiment_module, "run_gripper_sweep", fake_sweep, raising=False)

    assert hasattr(experiment_module, "_gripper_calibration_for_scene")
    experiment_module._gripper_calibration_for_scene.cache_clear()
    first = experiment_module._gripper_calibration_for_scene(str(CUBE_SCENE), "left")
    second = experiment_module._gripper_calibration_for_scene(str(CUBE_SCENE), "left")

    assert first is second
    assert calls == [(str(CUBE_SCENE), "left")]


def test_trial_orders_open_approach_grasp_close_and_close_settle(monkeypatch):
    experiment_module._gripper_calibration_for_scene.cache_clear()
    experiment_module._gripper_calibration_for_scene(DEMO_CUBE_SCENE, "left")
    events: list[tuple[str, float | None]] = []
    original_set_gripper = experiment_module.set_gripper
    original_move_to = experiment_module._move_to
    original_step = experiment_module.mujoco.mj_step
    move_names = iter(("approach", "grasp"))

    def spy_set_gripper(ctx, arm, value):
        events.append(("gripper", value))
        return original_set_gripper(ctx, arm, value)

    def spy_move_to(*args, **kwargs):
        events.append((next(move_names), None))
        return original_move_to(*args, **kwargs)

    def spy_step(*args, **kwargs):
        events.append(("step", None))
        return original_step(*args, **kwargs)

    monkeypatch.setattr(experiment_module, "set_gripper", spy_set_gripper)
    monkeypatch.setattr(experiment_module, "_move_to", spy_move_to)
    monkeypatch.setattr(experiment_module.mujoco, "mj_step", spy_step)

    result = run_trial(
        DEMO_CUBE_SCENE,
        policy="cube_lift",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )
    ctx = load_scene(DEMO_CUBE_SCENE)
    ctrl_min, ctrl_max = ctx.model.actuator_ctrlrange[ctx.left.gripper_actuator_id]
    approach_index = events.index(("approach", None))
    grasp_index = events.index(("grasp", None))
    close_index = next(
        index
        for index, event in enumerate(events)
        if index > grasp_index and event == ("gripper", pytest.approx(ctrl_min))
    )

    assert events[0] == ("gripper", pytest.approx(ctrl_max))
    assert events[1:approach_index] == [("step", None)] * 100
    assert approach_index < grasp_index < close_index
    assert events[close_index + 1 : close_index + 41] == [("step", None)] * 40
    assert result.steps == 420


def test_cube_lift_stage_routing_uses_stage_name_not_target_identity(monkeypatch):
    original_waypoints = experiment_module._finger_center_contact_waypoints

    def aliased_waypoints(ctx, arm, perceived):
        approach, grasp, _ = original_waypoints(ctx, arm, perceived)
        shared_target = grasp.copy()
        return approach, shared_target, shared_target

    monkeypatch.setattr(experiment_module, "_finger_center_contact_waypoints", aliased_waypoints)

    result = run_trial(DEMO_CUBE_SCENE, policy="cube_lift", noise=NoiseConfig(), seed=0, object_name="orange_cube")

    assert result.ik_success
    _assert_per_step_stage_metrics(
        result,
        expected_approach_frames=80,
        expected_grasp_frames=120,
        expected_lift_frames=120,
        expected_steps=420,
    )


def test_zero_noise_contact_policy_per_step_stage_metrics_are_deterministic():
    first = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )
    second = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert first.ik_success
    assert second.ik_success
    assert {field_name: getattr(first, field_name) for field_name in PER_STEP_STAGE_FIELDS} == {
        field_name: getattr(second, field_name) for field_name in PER_STEP_STAGE_FIELDS
    }


def test_approach_ik_failure_reports_zero_sampled_frames_and_false_diagnostics(monkeypatch):
    def fail_solve_ik(_ctx, _arm, _target_pos, seed_q):
        return IKResult(success=False, q=seed_q.copy(), error_m=0.123, iterations=1)

    monkeypatch.setattr(experiment_module, "solve_ik", fail_solve_ik)

    result = run_trial(
        str(CUBE_SCENE),
        policy="finger_center_contact_push",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert not result.ik_success
    _assert_per_step_stage_metrics(
        result,
        expected_approach_frames=0,
        expected_grasp_frames=0,
        expected_lift_frames=0,
        expected_steps=100,
    )
    assert not result.grasp_two_finger_contact_success
    assert not result.lift_two_finger_contact_success
    assert not result.sustained_lift_contact_success


def test_grasp_ik_failure_keeps_close_40_steps_but_leaves_diagnostics_false(monkeypatch):
    original_solve_ik = experiment_module.solve_ik
    solve_ik_calls = {"count": 0}

    def fail_on_second_solve_ik(ctx, arm, target_pos, seed_q):
        solve_ik_calls["count"] += 1
        if solve_ik_calls["count"] == 2:
            return IKResult(success=False, q=seed_q.copy(), error_m=0.456, iterations=1)
        return original_solve_ik(ctx, arm, target_pos, seed_q=seed_q)

    monkeypatch.setattr(experiment_module, "solve_ik", fail_on_second_solve_ik)

    result = run_trial(
        str(CUBE_SCENE),
        policy="finger_center_contact_push",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert not result.ik_success
    _assert_per_step_stage_metrics(
        result,
        expected_approach_frames=80,
        expected_grasp_frames=40,
        expected_lift_frames=0,
        expected_steps=220,
    )
    assert not result.grasp_two_finger_contact_success
    assert not result.lift_two_finger_contact_success
    assert not result.sustained_lift_contact_success


def test_explicit_contact_policy_names_run_on_cube_scene():
    ee_base = run_trial(
        str(CUBE_SCENE),
        policy="ee_base_contact_push",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )
    finger_center = run_trial(
        str(CUBE_SCENE),
        policy="finger_center_contact_push",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert ee_base.policy == "ee_base_contact_push"
    assert finger_center.policy == "finger_center_contact_push"
    assert ee_base.ik_success
    assert finger_center.ik_success
    assert ee_base.min_ee_object_distance_m != finger_center.min_ee_object_distance_m


def test_pre_grasp_finger_center_correction_runs_and_is_deterministic():
    noise = NoiseConfig(translation_std_m=0.005)

    first = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=noise,
        seed=11,
        object_name="orange_cube",
    )
    second = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=noise,
        seed=11,
        object_name="orange_cube",
    )

    assert first.policy == "pre_grasp_finger_center_correction"
    assert first.ik_success
    assert first.to_row() == second.to_row()


def test_pre_grasp_finger_center_correction_replans_after_approach():
    biased_noise = NoiseConfig(fixed_bias_m=(0.02, -0.02, 0.0))

    open_loop = run_trial(
        str(CUBE_SCENE),
        policy="finger_center_contact_push",
        noise=biased_noise,
        seed=0,
        object_name="orange_cube",
    )
    corrected = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=biased_noise,
        seed=0,
        object_name="orange_cube",
    )

    assert open_loop.ik_success
    assert corrected.ik_success
    assert open_loop.to_row() != corrected.to_row()


def test_pre_grasp_finger_center_correction_recomputes_waypoints_after_motion(monkeypatch):
    calls = []
    original = experiment_module._finger_center_contact_waypoints

    def spy_waypoints(ctx, arm, perceived):
        calls.append(
            {
                "perceived": perceived.copy(),
                "q": ctx.data.qpos[arm.joint_qpos_ids].copy(),
            }
        )
        return original(ctx, arm, perceived)

    monkeypatch.setattr(experiment_module, "_finger_center_contact_waypoints", spy_waypoints)

    result = run_trial(
        str(CUBE_SCENE),
        policy="pre_grasp_finger_center_correction",
        noise=NoiseConfig(),
        seed=0,
        object_name="orange_cube",
    )

    assert result.ik_success
    assert len(calls) == 2
    assert np.linalg.norm(calls[1]["q"] - calls[0]["q"]) > 0.01
    assert calls[1]["perceived"][2] < calls[0]["perceived"][2]


def test_cube_lift_policy_reports_physical_lift_metrics():
    scene = "/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/demo.xml"

    result = run_trial(scene, policy="cube_lift", noise=NoiseConfig(), seed=0, object_name="orange_cube")

    assert result.ik_success
    assert result.finger_object_contacts >= 0
    assert result.object_lift_m >= 0.0


def test_finger_center_contact_waypoints_compensate_ee_base_offset():
    ctx = load_scene(str(CUBE_SCENE))
    perceived = ctx.data.xpos[ctx.objects["orange_cube"]].copy()

    _, grasp, _ = _finger_center_contact_waypoints(ctx, ctx.left, perceived)

    current_offset = finger_collision_center(ctx, ctx.left) - forward_kinematics(
        ctx, ctx.left, ctx.data.qpos[ctx.left.joint_qpos_ids]
    )
    planned_finger_center = grasp + current_offset

    assert np.allclose(planned_finger_center[:2], perceived[:2], atol=1e-6)
    assert planned_finger_center[2] > perceived[2]
