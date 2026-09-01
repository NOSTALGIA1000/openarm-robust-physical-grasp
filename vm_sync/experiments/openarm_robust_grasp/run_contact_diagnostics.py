from __future__ import annotations

import argparse
from pathlib import Path

from openarm_robust_grasp.contact_diagnostics import (
    ClosureConfig,
    DiagnosticRunResult,
    run_contact_controls,
    run_fixed_cube_closure,
    write_contact_diagnostic_report,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_FIXED_SCENE = ROOT / "assets" / "cube_contact_fixed_scene.xml"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "contact_formation_diagnostics"


def build_diagnostic_run(
    fixed_scene: str, closure_config: ClosureConfig = ClosureConfig()
) -> DiagnosticRunResult:
    controls = tuple(run_contact_controls(fixed_scene, repeats=2))
    if not all(row.pass_control for row in controls):
        raise RuntimeError("measurement contact controls failed")

    fixed_closures = tuple(run_fixed_cube_closure(fixed_scene, closure_config))
    if any(row.mechanism_gate_pass for row in fixed_closures):
        raise RuntimeError(
            "fixed mechanism gate passed; free-cube stage requires separate approved execution"
        )

    return DiagnosticRunResult(
        controls=controls,
        fixed_closures=fixed_closures,
        free_closures=(),
        conditional_lifts=(),
        failure_branch="fixed_mechanism_gate_failed",
        free_closure_not_run_reason="fixed_mechanism_gate_failed",
        conditional_lift_not_run_reason="free_closure_not_run",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run staged OpenArm contact-formation diagnostics."
    )
    parser.add_argument("--fixed-scene", default=str(DEFAULT_FIXED_SCENE))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run = build_diagnostic_run(args.fixed_scene)
    paths = write_contact_diagnostic_report(run, args.output_dir)
    print("measurement_gate_pass=1")
    print("fixed_mechanism_gate_pass=0")
    print("failure_branch=fixed_mechanism_gate_failed")
    for name, path in paths.items():
        print(f"{name}={path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
