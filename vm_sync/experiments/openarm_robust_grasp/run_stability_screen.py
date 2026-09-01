from __future__ import annotations

import argparse
import csv
import io
import json
import os
import tempfile
from collections.abc import Iterable
from numbers import Integral, Real
from pathlib import Path

import numpy as np

from openarm_robust_grasp.grasp_diagnostics import (
    StabilityConfig,
    StabilityResult,
    run_stability_trial,
)


CLOSE_FRACTIONS = (0.25, 0.50, 0.75, 1.00)
CUBE_WIDTHS_M = (0.036, 0.042, 0.046)
SLIDING_FRICTIONS = (1.0, 2.0, 4.0)
CUBE_MASS_KG = 0.015
SEED = 0


def stability_config_grid() -> list[StabilityConfig]:
    return [
        StabilityConfig(
            close_fraction=close_fraction,
            cube_width_m=cube_width_m,
            sliding_friction=sliding_friction,
            cube_mass_kg=CUBE_MASS_KG,
            seed=SEED,
        )
        for close_fraction in CLOSE_FRACTIONS
        for cube_width_m in CUBE_WIDTHS_M
        for sliding_friction in SLIDING_FRICTIONS
    ]


def run_stability_screen(scene_path: str) -> list[StabilityResult]:
    return [
        run_stability_trial(scene_path, config)
        for config in stability_config_grid()
    ]


def _grid_key(
    row: StabilityConfig | StabilityResult,
) -> tuple[float, float, float, float, int]:
    return (
        row.close_fraction,
        row.cube_width_m,
        row.sliding_friction,
        row.cube_mass_kg,
        row.seed,
    )


def _validate_result(result: StabilityResult) -> None:
    real_fields = (
        "close_fraction",
        "resolved_close_command",
        "cube_width_m",
        "sliding_friction",
        "cube_mass_kg",
        "achieved_lift_height_m",
        "final_lift_height_m",
        "pre_disturb_elevation_m",
        "close_two_finger_contact_fraction",
        "lift_two_finger_contact_fraction",
        "pre_disturb_hold_two_finger_contact_fraction",
        "disturbance_two_finger_contact_fraction",
        "post_disturb_hold_two_finger_contact_fraction",
        "max_object_gripper_displacement_m",
        "final_object_gripper_displacement_m",
        "disturbance_drop_m",
    )
    for field_name in real_fields:
        value = getattr(result, field_name)
        if (
            not isinstance(value, Real)
            or isinstance(value, bool)
            or not np.isfinite(value)
        ):
            raise ValueError(
                f"invalid metric or config field {field_name}: {value!r}"
            )

    if (
        not isinstance(result.seed, Integral)
        or isinstance(result.seed, bool)
        or result.seed < 0
    ):
        raise ValueError(f"invalid numeric field seed: {result.seed!r}")

    count_fields = (
        "close_sampled_frames",
        "lift_sampled_frames",
        "pre_disturb_hold_sampled_frames",
        "disturbance_sampled_frames",
        "post_disturb_hold_sampled_frames",
        "total_steps",
    )
    for field_name in count_fields:
        value = getattr(result, field_name)
        if not isinstance(value, Integral) or isinstance(value, bool) or value < 0:
            raise ValueError(f"invalid metric {field_name}: {value!r}")

    fraction_fields = (
        "close_two_finger_contact_fraction",
        "lift_two_finger_contact_fraction",
        "pre_disturb_hold_two_finger_contact_fraction",
        "disturbance_two_finger_contact_fraction",
        "post_disturb_hold_two_finger_contact_fraction",
    )
    for field_name in fraction_fields:
        value = getattr(result, field_name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"invalid metric {field_name}: {value!r}")

    nonnegative_fields = (
        "achieved_lift_height_m",
        "max_object_gripper_displacement_m",
        "final_object_gripper_displacement_m",
        "disturbance_drop_m",
    )
    for field_name in nonnegative_fields:
        value = getattr(result, field_name)
        if value < 0.0:
            raise ValueError(f"invalid metric {field_name}: {value!r}")

    for field_name in (
        "ik_success",
        "final_elevation_success",
        "hold_contact_success",
        "disturbance_eligible",
        "stable_grasp_success",
    ):
        value = getattr(result, field_name)
        if not isinstance(value, bool):
            raise ValueError(f"invalid boolean field {field_name}: {value!r}")
    if result.disturbance_survival_success is not None and not isinstance(
        result.disturbance_survival_success, bool
    ):
        raise ValueError(
            "invalid boolean field disturbance_survival_success: "
            f"{result.disturbance_survival_success!r}"
        )
    if result.disturbance_eligible != (
        result.disturbance_survival_success is not None
    ):
        raise ValueError("invalid disturbance eligibility/survival pairing")


def _validated_results(results: Iterable[StabilityResult]) -> list[StabilityResult]:
    rows = list(results)
    if not all(isinstance(result, StabilityResult) for result in rows):
        raise ValueError("screen results must contain only StabilityResult rows")

    for result in rows:
        _validate_result(result)

    actual_keys = [_grid_key(result) for result in rows]
    duplicate_keys = {
        key for key in actual_keys if actual_keys.count(key) > 1
    }
    if duplicate_keys:
        raise ValueError(f"duplicate grid row: {sorted(duplicate_keys)!r}")

    expected_keys = [_grid_key(config) for config in stability_config_grid()]
    unexpected_keys = set(actual_keys) - set(expected_keys)
    if unexpected_keys:
        raise ValueError(f"unexpected grid row: {sorted(unexpected_keys)!r}")

    missing_keys = set(expected_keys) - set(actual_keys)
    if missing_keys:
        raise ValueError(f"missing grid row: {sorted(missing_keys)!r}")
    if len(rows) != len(expected_keys):
        raise ValueError(f"expected {len(expected_keys)} grid rows, received {len(rows)}")
    if actual_keys != expected_keys:
        raise ValueError("grid rows are not in deterministic order")

    return rows


def _stable_counts(
    results: list[StabilityResult], field_name: str, levels: tuple[float, ...]
) -> dict[str, int]:
    return {
        str(level): sum(
            result.stable_grasp_success
            for result in results
            if getattr(result, field_name) == level
        )
        for level in levels
    }


def _build_summary(
    rows: list[StabilityResult], scene_path: str
) -> dict[str, object]:
    stable_count = sum(result.stable_grasp_success for result in rows)
    ik_success_count = sum(result.ik_success for result in rows)
    final_elevation_success_count = sum(
        result.final_elevation_success for result in rows
    )
    hold_contact_success_count = sum(result.hold_contact_success for result in rows)
    disturbance_eligible_count = sum(result.disturbance_eligible for result in rows)
    disturbance_survival_success_count = sum(
        result.disturbance_survival_success is True for result in rows
    )
    stable_grid_fraction = stable_count / len(rows)
    return {
        "scene": str(scene_path),
        "row_count": len(rows),
        "parameter_levels": {
            "close_fraction": list(CLOSE_FRACTIONS),
            "cube_width_m": list(CUBE_WIDTHS_M),
            "sliding_friction": list(SLIDING_FRICTIONS),
            "cube_mass_kg": [CUBE_MASS_KG],
            "seed": [SEED],
        },
        "stable_count": stable_count,
        "stable_grid_fraction": stable_grid_fraction,
        "stable_rate": stable_grid_fraction,
        "ik_success_count": ik_success_count,
        "ik_success_rate": ik_success_count / len(rows),
        "final_elevation_success_count": final_elevation_success_count,
        "hold_contact_success_count": hold_contact_success_count,
        "disturbance_eligible_count": disturbance_eligible_count,
        "disturbance_survival_success_count": disturbance_survival_success_count,
        "disturbance_survival_rate_among_eligible": (
            disturbance_survival_success_count / disturbance_eligible_count
            if disturbance_eligible_count
            else None
        ),
        "min_achieved_lift_height_m": min(
            result.achieved_lift_height_m for result in rows
        ),
        "max_achieved_lift_height_m": max(
            result.achieved_lift_height_m for result in rows
        ),
        "min_final_lift_height_m": min(result.final_lift_height_m for result in rows),
        "max_final_lift_height_m": max(result.final_lift_height_m for result in rows),
        "max_pre_disturb_hold_two_finger_contact_fraction": max(
            result.pre_disturb_hold_two_finger_contact_fraction for result in rows
        ),
        "max_post_disturb_hold_two_finger_contact_fraction": max(
            result.post_disturb_hold_two_finger_contact_fraction for result in rows
        ),
        "max_disturbance_drop_m": max(result.disturbance_drop_m for result in rows),
        "stable_counts_by_close_fraction": _stable_counts(
            rows, "close_fraction", CLOSE_FRACTIONS
        ),
        "stable_counts_by_cube_width_m": _stable_counts(
            rows, "cube_width_m", CUBE_WIDTHS_M
        ),
        "stable_counts_by_sliding_friction": _stable_counts(
            rows, "sliding_friction", SLIDING_FRICTIONS
        ),
    }


def _build_csv_bytes(rows: list[StabilityResult]) -> bytes:
    row_dicts = [result.to_row() for result in rows]
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=list(row_dicts[0]))
    writer.writeheader()
    writer.writerows(row_dicts)
    return buffer.getvalue().encode("utf-8")


def _build_json_bytes(summary: dict[str, object]) -> bytes:
    return (json.dumps(summary, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _stage_bytes(output: Path, destination_name: str, content: bytes) -> Path:
    descriptor, temporary_name = tempfile.mkstemp(
        dir=output,
        prefix=f".{destination_name}.",
        suffix=".tmp",
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return temporary_path


def _replace_output_pair(output: Path, csv_bytes: bytes, json_bytes: bytes) -> None:
    output.mkdir(parents=True, exist_ok=True)
    destinations = (output / "results.csv", output / "summary.json")
    old_contents = {
        destination: destination.read_bytes() if destination.exists() else None
        for destination in destinations
    }
    staged: list[Path] = []
    replaced: list[Path] = []
    try:
        staged.append(_stage_bytes(output, destinations[0].name, csv_bytes))
        staged.append(_stage_bytes(output, destinations[1].name, json_bytes))
        for temporary_path, destination in zip(staged, destinations):
            os.replace(temporary_path, destination)
            replaced.append(destination)
    except BaseException:
        for destination in reversed(replaced):
            old_content = old_contents[destination]
            if old_content is None:
                destination.unlink(missing_ok=True)
            else:
                rollback_path = _stage_bytes(
                    output, f"{destination.name}.rollback", old_content
                )
                try:
                    os.replace(rollback_path, destination)
                finally:
                    rollback_path.unlink(missing_ok=True)
        raise
    finally:
        for temporary_path in staged:
            temporary_path.unlink(missing_ok=True)


def write_stability_screen_report(
    results: Iterable[StabilityResult],
    output_dir: str | Path,
    scene_path: str,
) -> None:
    rows = _validated_results(results)
    csv_bytes = _build_csv_bytes(rows)
    summary = _build_summary(rows, scene_path)
    json_bytes = _build_json_bytes(summary)
    _replace_output_pair(Path(output_dir), csv_bytes, json_bytes)


def main() -> None:
    experiment_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Run fixed 36-condition stability screen.")
    parser.add_argument(
        "--scene",
        default=str(experiment_root / "assets" / "cube_lift_scene.xml"),
    )
    parser.add_argument(
        "--output-dir",
        default=str(experiment_root / "outputs" / "stability_screen"),
    )
    args = parser.parse_args()

    results = run_stability_screen(args.scene)
    write_stability_screen_report(results, args.output_dir, args.scene)
    print(f"Wrote {len(results)} rows to {args.output_dir}")


if __name__ == "__main__":
    main()
