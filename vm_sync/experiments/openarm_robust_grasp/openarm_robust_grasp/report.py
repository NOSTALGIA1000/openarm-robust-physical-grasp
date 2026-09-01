from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from openarm_robust_grasp.experiment import TrialResult


def write_results(results: list[TrialResult], out_dir: str | Path) -> None:
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    rows = [result.to_row() for result in results]
    if not rows:
        raise ValueError("No trial results to write")

    with (output / "results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(results)
    with (output / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    plot_success(summary, output / "success_vs_noise.png")
    plot_error(summary, output / "error_vs_noise.png")


def summarize(results: list[TrialResult]) -> list[dict[str, float | str | int]]:
    grouped: dict[tuple[str, float], list[TrialResult]] = defaultdict(list)
    for result in results:
        grouped[(result.policy, result.translation_std_m)].append(result)

    rows: list[dict[str, float | str | int]] = []
    for (policy, noise), group in sorted(grouped.items()):
        n = len(group)
        success = sum(1 for item in group if item.grasp_success)
        proxy_success = sum(1 for item in group if item.proxy_success)
        contact_success = sum(1 for item in group if item.contact_success)
        post_grasp_contact_success = sum(1 for item in group if item.post_grasp_contact_success)
        lift_contact_success = sum(1 for item in group if item.lift_contact_success)
        ik_success = sum(1 for item in group if item.ik_success)
        rows.append(
            {
                "policy": policy,
                "translation_std_m": noise,
                "trials": n,
                "success_rate": success / n,
                "proxy_success_rate": proxy_success / n,
                "contact_success_rate": contact_success / n,
                "post_grasp_contact_success_rate": post_grasp_contact_success / n,
                "lift_contact_success_rate": lift_contact_success / n,
                "ik_success_rate": ik_success / n,
                "mean_min_distance_m": sum(item.min_ee_object_distance_m for item in group) / n,
                "mean_final_distance_m": sum(item.final_ee_object_distance_m for item in group) / n,
                "mean_object_motion_m": sum(item.object_motion_m for item in group) / n,
                "mean_object_lift_m": sum(item.object_lift_m for item in group) / n,
                "mean_finger_object_contacts": sum(item.finger_object_contacts for item in group) / n,
                "mean_approach_finger_object_contacts": sum(
                    item.approach_finger_object_contacts for item in group
                )
                / n,
                "mean_grasp_finger_object_contacts": sum(item.grasp_finger_object_contacts for item in group)
                / n,
                "mean_lift_finger_object_contacts": sum(item.lift_finger_object_contacts for item in group)
                / n,
                "mean_approach_contact_frame_rate": sum(item.approach_contact_frame_rate for item in group) / n,
                "mean_grasp_contact_frame_rate": sum(item.grasp_contact_frame_rate for item in group) / n,
                "mean_lift_contact_frame_rate": sum(item.lift_contact_frame_rate for item in group) / n,
                "grasp_two_finger_contact_success_rate": sum(
                    1 for item in group if item.grasp_two_finger_contact_success
                )
                / n,
                "lift_two_finger_contact_success_rate": sum(
                    1 for item in group if item.lift_two_finger_contact_success
                )
                / n,
                "sustained_lift_contact_success_rate": sum(
                    1 for item in group if item.sustained_lift_contact_success
                )
                / n,
                "mean_lift_height_m": sum(item.lift_height_m for item in group) / n,
            }
        )
    return rows


def _series(summary: list[dict[str, float | str | int]], key: str) -> dict[str, tuple[list[float], list[float]]]:
    by_policy: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for row in summary:
        by_policy[str(row["policy"])].append((float(row["translation_std_m"]), float(row[key])))
    return {
        policy: (
            [noise * 1000.0 for noise, _ in sorted(values)],
            [value for _, value in sorted(values)],
        )
        for policy, values in by_policy.items()
    }


def plot_success(summary: list[dict[str, float | str | int]], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4))
    for policy, (x_vals, y_vals) in _series(summary, "success_rate").items():
        ax.plot(x_vals, y_vals, marker="o", label=policy)
    ax.set_xlabel("Translation noise std (mm)")
    ax.set_ylabel("Success rate")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_error(summary: list[dict[str, float | str | int]], path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7, 4))
    for policy, (x_vals, y_vals) in _series(summary, "mean_min_distance_m").items():
        ax.plot(x_vals, [value * 1000.0 for value in y_vals], marker="o", label=policy)
    ax.set_xlabel("Translation noise std (mm)")
    ax.set_ylabel("Mean min EE-object distance (mm)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
