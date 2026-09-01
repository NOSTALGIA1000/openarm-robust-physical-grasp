from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Iterable

from openarm_robust_grasp.experiment import NoiseConfig, run_trial
from openarm_robust_grasp.report import write_results


DEFAULT_SCENE = "/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml"
DEFAULT_OBJECT = "bottle"
DEFAULT_POLICIES = ("open_loop", "pre_grasp_correction")
DEFAULT_TRANSLATION_NOISE = (0.0, 0.005, 0.01, 0.02, 0.03)


@dataclass(frozen=True)
class TrialJob:
    policy: str
    translation_noise_m: float
    local_seed: int
    seed: int


def iter_trial_jobs(
    policies: Iterable[str], translation_noise: Iterable[float], trials: int
) -> Iterable[TrialJob]:
    for noise_index, noise_m in enumerate(translation_noise):
        for local_seed in range(trials):
            paired_seed = 100000 * noise_index + local_seed
            for policy in policies:
                yield TrialJob(
                    policy=policy,
                    translation_noise_m=noise_m,
                    local_seed=local_seed,
                    seed=paired_seed,
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run OpenArm MuJoCo grasp robustness experiments.")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--object", default=DEFAULT_OBJECT)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--out", default="outputs/latest")
    parser.add_argument(
        "--policies",
        nargs="*",
        default=list(DEFAULT_POLICIES),
        help=(
            "Policies to run: open_loop, pre_grasp_correction, contact_push, "
            "ee_base_contact_push, finger_center_contact_push, "
            "pre_grasp_finger_center_correction, cube_lift."
        ),
    )
    parser.add_argument(
        "--translation-noise",
        type=float,
        nargs="*",
        default=list(DEFAULT_TRANSLATION_NOISE),
        help="Translation noise std values in meters.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results = []
    for job in iter_trial_jobs(args.policies, args.translation_noise, args.trials):
        result = run_trial(
            args.scene,
            policy=job.policy,
            noise=NoiseConfig(translation_std_m=job.translation_noise_m),
            seed=job.seed,
            object_name=args.object,
        )
        results.append(result)
        print(
            f"{job.policy:34s} noise={job.translation_noise_m * 1000:5.1f}mm "
            f"seed={job.seed:6d} ik={int(result.ik_success)} "
            f"proxy={int(result.proxy_success)} "
            f"contact={int(result.contact_success)} "
            f"min_dist={result.min_ee_object_distance_m * 1000:6.1f}mm "
            f"contacts={result.finger_object_contacts}"
        )

    out_dir = Path(args.out)
    write_results(results, out_dir)
    print(f"Wrote {len(results)} results to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
