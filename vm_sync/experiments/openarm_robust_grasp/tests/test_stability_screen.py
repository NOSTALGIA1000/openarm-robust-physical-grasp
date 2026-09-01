import csv
import json
import os
from dataclasses import replace
from itertools import product

import pytest

import run_stability_screen as screen_module
from openarm_robust_grasp.grasp_diagnostics import StabilityConfig, StabilityResult
from run_stability_screen import (
    run_stability_screen,
    stability_config_grid,
    write_stability_screen_report,
)


CLOSE_FRACTIONS = [0.25, 0.50, 0.75, 1.00]
CUBE_WIDTHS_M = [0.036, 0.042, 0.046]
SLIDING_FRICTIONS = [1.0, 2.0, 4.0]


def _result_for(
    config: StabilityConfig,
    *,
    stable: bool = False,
    eligible: bool = False,
    ik_success: bool = True,
) -> StabilityResult:
    contact_formed = stable or eligible
    return StabilityResult(
        close_fraction=config.close_fraction,
        resolved_close_command=0.58905 * (1.0 - config.close_fraction),
        cube_width_m=config.cube_width_m,
        sliding_friction=config.sliding_friction,
        cube_mass_kg=config.cube_mass_kg,
        seed=config.seed,
        ik_success=ik_success,
        close_sampled_frames=100,
        lift_sampled_frames=120,
        pre_disturb_hold_sampled_frames=500,
        disturbance_sampled_frames=100,
        post_disturb_hold_sampled_frames=250,
        achieved_lift_height_m=0.08 if contact_formed else 0.01,
        final_lift_height_m=0.06 if contact_formed else -0.02,
        pre_disturb_elevation_m=0.06 if contact_formed else 0.0,
        close_two_finger_contact_fraction=0.80 if contact_formed else 0.10,
        lift_two_finger_contact_fraction=0.75 if contact_formed else 0.0,
        pre_disturb_hold_two_finger_contact_fraction=0.75 if contact_formed else 0.0,
        disturbance_two_finger_contact_fraction=0.70 if contact_formed else 0.0,
        post_disturb_hold_two_finger_contact_fraction=0.65 if contact_formed else 0.0,
        max_object_gripper_displacement_m=0.16,
        final_object_gripper_displacement_m=0.15,
        disturbance_drop_m=0.01 if stable else 0.03,
        total_steps=1330,
    )


def _screen_results() -> list[StabilityResult]:
    grid = stability_config_grid()
    return [
        _result_for(
            config,
            stable=index in {0, len(grid) - 1},
            eligible=index == 2,
            ik_success=index != 1,
        )
        for index, config in enumerate(grid)
    ]


def test_stability_config_grid_has_exact_36_unique_configs_in_required_order():
    configs = stability_config_grid()
    expected_levels = list(product(CLOSE_FRACTIONS, CUBE_WIDTHS_M, SLIDING_FRICTIONS))

    assert len(configs) == 36
    assert len(set(configs)) == 36
    assert [
        (config.close_fraction, config.cube_width_m, config.sliding_friction)
        for config in configs
    ] == expected_levels
    assert all(config.seed == 0 for config in configs)
    assert all(config.cube_mass_kg == pytest.approx(0.015) for config in configs)


def test_run_stability_screen_runs_one_task_2_trial_per_ordered_config(monkeypatch):
    observed: list[tuple[str, StabilityConfig]] = []

    def fake_run_stability_trial(scene_path: str, config: StabilityConfig) -> StabilityResult:
        observed.append((scene_path, config))
        return _result_for(config)

    monkeypatch.setattr(screen_module, "run_stability_trial", fake_run_stability_trial)

    results = run_stability_screen("scene.xml")

    assert [config for _, config in observed] == stability_config_grid()
    assert [scene for scene, _ in observed] == ["scene.xml"] * 36
    assert len(results) == 36
    assert all(isinstance(result, StabilityResult) for result in results)


def test_report_writes_exact_csv_rows_and_complete_summary(tmp_path):
    results = _screen_results()

    write_stability_screen_report(results, tmp_path, "scene.xml")

    with (tmp_path / "results.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))

    assert rows[0].keys() == results[0].to_row().keys()
    assert len(rows) == 36
    assert [float(row["close_fraction"]) for row in rows] == [
        result.close_fraction for result in results
    ]
    assert [float(row["cube_width_m"]) for row in rows] == [
        result.cube_width_m for result in results
    ]
    assert [float(row["sliding_friction"]) for row in rows] == [
        result.sliding_friction for result in results
    ]
    assert summary == {
        "scene": "scene.xml",
        "row_count": 36,
        "parameter_levels": {
            "close_fraction": CLOSE_FRACTIONS,
            "cube_width_m": CUBE_WIDTHS_M,
            "sliding_friction": SLIDING_FRICTIONS,
            "cube_mass_kg": [0.015],
            "seed": [0],
        },
        "stable_count": 2,
        "stable_rate": pytest.approx(2 / 36),
        "stable_grid_fraction": pytest.approx(2 / 36),
        "ik_success_count": 35,
        "ik_success_rate": pytest.approx(35 / 36),
        "final_elevation_success_count": 3,
        "hold_contact_success_count": 3,
        "disturbance_eligible_count": 3,
        "disturbance_survival_success_count": 2,
        "disturbance_survival_rate_among_eligible": pytest.approx(2 / 3),
        "min_achieved_lift_height_m": pytest.approx(0.01),
        "max_achieved_lift_height_m": pytest.approx(0.08),
        "min_final_lift_height_m": pytest.approx(-0.02),
        "max_final_lift_height_m": pytest.approx(0.06),
        "max_pre_disturb_hold_two_finger_contact_fraction": pytest.approx(0.75),
        "max_post_disturb_hold_two_finger_contact_fraction": pytest.approx(0.65),
        "max_disturbance_drop_m": pytest.approx(0.03),
        "stable_counts_by_close_fraction": {
            "0.25": 1,
            "0.5": 0,
            "0.75": 0,
            "1.0": 1,
        },
        "stable_counts_by_cube_width_m": {
            "0.036": 1,
            "0.042": 0,
            "0.046": 1,
        },
        "stable_counts_by_sliding_friction": {
            "1.0": 1,
            "2.0": 0,
            "4.0": 1,
        },
    }


def test_report_output_bytes_are_deterministic(tmp_path):
    results = _screen_results()
    first = tmp_path / "first"
    second = tmp_path / "second"

    write_stability_screen_report(results, first, "scene.xml")
    write_stability_screen_report(results, second, "scene.xml")

    assert (first / "results.csv").read_bytes() == (second / "results.csv").read_bytes()
    assert (first / "summary.json").read_bytes() == (second / "summary.json").read_bytes()


def test_report_uses_null_survival_rate_when_no_row_is_disturbance_eligible(tmp_path):
    results = [_result_for(config) for config in stability_config_grid()]

    write_stability_screen_report(results, tmp_path, "scene.xml")

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["disturbance_eligible_count"] == 0
    assert summary["disturbance_survival_success_count"] == 0
    assert summary["disturbance_survival_rate_among_eligible"] is None


@pytest.mark.parametrize(
    ("malformed_results", "message"),
    [
        (lambda rows: rows[:-1], "missing grid row"),
        (lambda rows: rows[:-1] + [rows[0]], "duplicate grid row"),
        (
            lambda rows: rows[:-1]
            + [replace(rows[-1], close_fraction=0.33)],
            "unexpected grid row",
        ),
        (lambda rows: [rows[1], rows[0], *rows[2:]], "deterministic order"),
    ],
)
def test_report_rejects_missing_duplicate_or_unexpected_grid_rows(
    tmp_path, malformed_results, message
):
    with pytest.raises(ValueError, match=message):
        write_stability_screen_report(
            malformed_results(_screen_results()), tmp_path, "scene.xml"
        )


@pytest.mark.parametrize(
    "overrides",
    [
        {"close_sampled_frames": -1},
        {"lift_sampled_frames": -1},
        {"total_steps": -1},
        {"achieved_lift_height_m": -0.001},
        {"final_lift_height_m": float("nan")},
        {"pre_disturb_elevation_m": float("nan")},
        {"close_two_finger_contact_fraction": 1.001},
        {"lift_two_finger_contact_fraction": -0.001},
        {"pre_disturb_hold_two_finger_contact_fraction": 1.001},
        {"max_object_gripper_displacement_m": -0.001},
        {"final_object_gripper_displacement_m": float("inf")},
        {"disturbance_drop_m": -0.001},
    ],
)
def test_report_rejects_out_of_bounds_or_nonfinite_metrics(tmp_path, overrides):
    results = _screen_results()
    results[0] = replace(results[0], **overrides)

    with pytest.raises(ValueError, match="invalid metric"):
        write_stability_screen_report(results, tmp_path, "scene.xml")


def _corrupt(result: StabilityResult, field_name: str, value) -> StabilityResult:
    object.__setattr__(result, field_name, value)
    return result


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("seed", False),
        ("close_fraction", True),
        ("cube_width_m", "0.036"),
        ("resolved_close_command", "0.0"),
        ("achieved_lift_height_m", "0.01"),
        ("close_sampled_frames", True),
    ],
)
def test_report_rejects_bool_or_numeric_string_numeric_fields(
    tmp_path, field_name, value
):
    results = _screen_results()
    results[0] = _corrupt(results[0], field_name, value)

    with pytest.raises(ValueError, match="invalid"):
        write_stability_screen_report(results, tmp_path, "scene.xml")


def test_report_rejects_non_bool_ik_success(tmp_path):
    results = _screen_results()
    results[0] = _corrupt(results[0], "ik_success", 1)

    with pytest.raises(ValueError, match="ik_success"):
        write_stability_screen_report(results, tmp_path, "scene.xml")


def test_validation_failure_leaves_existing_output_pair_unchanged(tmp_path):
    csv_path = tmp_path / "results.csv"
    json_path = tmp_path / "summary.json"
    csv_path.write_bytes(b"old csv\n")
    json_path.write_bytes(b"old json\n")

    with pytest.raises(ValueError):
        write_stability_screen_report(_screen_results()[:-1], tmp_path, "scene.xml")

    assert csv_path.read_bytes() == b"old csv\n"
    assert json_path.read_bytes() == b"old json\n"
    assert sorted(path.name for path in tmp_path.iterdir()) == ["results.csv", "summary.json"]


def test_summary_failure_leaves_existing_output_pair_unchanged(tmp_path, monkeypatch):
    csv_path = tmp_path / "results.csv"
    json_path = tmp_path / "summary.json"
    csv_path.write_bytes(b"old csv\n")
    json_path.write_bytes(b"old json\n")

    def fail_summary(_rows, _scene_path):
        raise RuntimeError("summary failed")

    monkeypatch.setattr(screen_module, "_build_summary", fail_summary, raising=False)

    with pytest.raises(RuntimeError, match="summary failed"):
        write_stability_screen_report(_screen_results(), tmp_path, "scene.xml")

    assert csv_path.read_bytes() == b"old csv\n"
    assert json_path.read_bytes() == b"old json\n"
    assert sorted(path.name for path in tmp_path.iterdir()) == ["results.csv", "summary.json"]


def test_second_replace_failure_rolls_back_existing_output_pair(tmp_path, monkeypatch):
    csv_path = tmp_path / "results.csv"
    json_path = tmp_path / "summary.json"
    csv_path.write_bytes(b"old csv\n")
    json_path.write_bytes(b"old json\n")
    real_replace = os.replace
    summary_replace_attempted = False

    def fail_first_summary_replace(source, destination):
        nonlocal summary_replace_attempted
        if str(destination).endswith("summary.json") and not summary_replace_attempted:
            summary_replace_attempted = True
            raise OSError("summary replace failed")
        return real_replace(source, destination)

    monkeypatch.setattr(os, "replace", fail_first_summary_replace)

    with pytest.raises(OSError, match="summary replace failed"):
        write_stability_screen_report(_screen_results(), tmp_path, "scene.xml")

    assert csv_path.read_bytes() == b"old csv\n"
    assert json_path.read_bytes() == b"old json\n"
    assert sorted(path.name for path in tmp_path.iterdir()) == ["results.csv", "summary.json"]


def test_second_staging_failure_removes_first_temp_and_preserves_pair(
    tmp_path, monkeypatch
):
    csv_path = tmp_path / "results.csv"
    json_path = tmp_path / "summary.json"
    csv_path.write_bytes(b"old csv\n")
    json_path.write_bytes(b"old json\n")
    real_stage = screen_module._stage_bytes
    stage_calls = 0

    def fail_second_stage(output, destination_name, content):
        nonlocal stage_calls
        stage_calls += 1
        if stage_calls == 2:
            raise OSError("second staging failed")
        return real_stage(output, destination_name, content)

    monkeypatch.setattr(screen_module, "_stage_bytes", fail_second_stage)

    with pytest.raises(OSError, match="second staging failed"):
        write_stability_screen_report(_screen_results(), tmp_path, "scene.xml")

    assert csv_path.read_bytes() == b"old csv\n"
    assert json_path.read_bytes() == b"old json\n"
    assert sorted(path.name for path in tmp_path.iterdir()) == ["results.csv", "summary.json"]
