from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from openarm_robust_grasp.surface_geometry import (
    InstrumentControlResult,
    LinkageConfig,
    LinkageResult,
    ShapeConditionResult,
    run_shape_ladder,
    run_surface_instrument_controls,
    run_unloaded_linkage_baseline,
    select_shape_decision,
    write_surface_geometry_report,
)


ROOT = Path(__file__).resolve().parent
DEFAULT_FIXED_SCENE = ROOT / "assets" / "cube_contact_fixed_scene.xml"
DEFAULT_SPHERE_SCENE = ROOT / "assets" / "contact_surface_sphere_scene.xml"
DEFAULT_CUBOID_SCENE = ROOT / "assets" / "contact_surface_cuboid_scene.xml"
DEFAULT_OUTPUT_DIR = ROOT / "outputs" / "contact_surface_geometry"


@dataclass(frozen=True)
class SurfaceGeometryRun:
    instrument_controls: tuple[InstrumentControlResult, ...]
    linkage: LinkageResult
    shapes: tuple[ShapeConditionResult, ...]
    decision: str
    output_paths: dict[str, Path]


def execute_surface_geometry_diagnosis(
    fixed_scene: str | Path = DEFAULT_FIXED_SCENE,
    sphere_scene: str | Path = DEFAULT_SPHERE_SCENE,
    cuboid_scene: str | Path = DEFAULT_CUBOID_SCENE,
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
) -> SurfaceGeometryRun:
    config = LinkageConfig()
    controls = run_surface_instrument_controls(str(fixed_scene))
    if not all(row.passed for row in controls):
        raise RuntimeError("instrument gate failed; physical stages are invalid")
    linkage = run_unloaded_linkage_baseline(str(fixed_scene), config)
    shapes = run_shape_ladder(str(sphere_scene), str(cuboid_scene), config)
    decision = select_shape_decision(
        shapes[0].gate_pass,
        shapes[1].gate_pass,
        shapes[2].gate_pass,
    )
    output_paths = write_surface_geometry_report(
        controls, linkage, shapes, decision, output_dir
    )
    return SurfaceGeometryRun(
        instrument_controls=controls,
        linkage=linkage,
        shapes=shapes,
        decision=decision,
        output_paths=output_paths,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run deterministic OpenArm contact-surface geometry diagnosis."
    )
    parser.add_argument("--fixed-scene", default=str(DEFAULT_FIXED_SCENE))
    parser.add_argument("--sphere-scene", default=str(DEFAULT_SPHERE_SCENE))
    parser.add_argument("--cuboid-scene", default=str(DEFAULT_CUBOID_SCENE))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    args = parser.parse_args()
    result = execute_surface_geometry_diagnosis(
        args.fixed_scene,
        args.sphere_scene,
        args.cuboid_scene,
        args.output_dir,
    )
    print(f"decision={result.decision}")
    for name, path in result.output_paths.items():
        print(f"{name}={path}")


if __name__ == "__main__":
    main()
