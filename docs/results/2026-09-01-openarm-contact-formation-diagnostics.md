# OpenArm Contact-Formation Diagnostics Result

Date: 2026-09-01

## Outcome

Measurement gate passed. Fixed-cube bilateral-contact mechanism gate failed. Free-cube closure and conditional 20 mm lift were therefore not eligible and were not run.

This is a deterministic simulation-only negative result. It localizes the next investigation to gripper/object contact geometry, contact-surface alignment, actuator stroke/transmission behavior, or fixed-scene contact configuration. It does not support further friction tuning or visual-noise experiments yet.

## Environment and Protocol

- VM experiment: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp`
- Host mirror: `C:\Users\ASUS\Documents\ChatGPT\open-arm相关\vm_sync\experiments\openarm_robust_grasp`
- Fixed scene: `assets/cube_contact_fixed_scene.xml`
- Object full width: `0.042 m`
- Placements: `current_waypoint`, `geometric_center`
- Command profiles: raw actuator range, calibrated open `0.58905` to closed `0.0`
- Closure sampling: 101 inclusive commands x 20 steps = 2020 frames/condition
- Bilateral gate: positive normal force greater than `1e-6 N` on both named sides for at least 20 consecutive frames

## Measurement Controls

Negative, inner-only, outer-only, and bilateral controls all matched expected classes on two fresh-context repeats. This validates named inner/outer grouping, raw geom-pair extraction, finite normal-force extraction, and signed-gap measurement for this model.

One canonical repeat:

| Control | Inner gap | Outer gap | Inner force | Outer force |
| --- | ---: | ---: | ---: | ---: |
| negative | 0.183204 m | 0.181624 m | 0 N | 0 N |
| inner-only | -0.001068 m | 0.007584 m | 10.5587 N | 0 N |
| outer-only | 0.007586 m | -0.001086 m | 0 N | 10.7851 N |
| bilateral | -0.005568 m | -0.005569 m | 921.285 N | 919.884 N |

Fixed-probe positive controls intentionally overlap geometry. Their force magnitudes are detectability controls, not calibrated or realistic grasp forces.

## Fixed-Cube Results

All four conditions completed IK and all 8080 declared frames were sampled.

| Placement | Profile | Bilateral frames | Max consecutive bilateral | Max inner force | Max outer force | Gate |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| current waypoint | raw full | 0/2020 | 0 | 0 N | 5752.860 N | fail |
| current waypoint | calibrated | 0/2020 | 0 | 0 N | 1316.786 N | fail |
| geometric center | raw full | 0/2020 | 0 | 5708.200 N | 0 N | fail |
| geometric center | calibrated | 0/2020 | 0 | 1740.612 N | 0 N | fail |

Current-waypoint placement detected only outer-side contact. Geometric-center placement detected only inner-side contact. Large forces arise from a kinematically fixed cube and deep overlap; they must not be interpreted as plausible actuator grasp force.

Supported observation: neither raw nor calibrated closure formed simultaneous named inner/outer contact under either predeclared fixed placement.

Unsupported claim: gripper can never grasp, friction has no effect, or one specific mesh/actuator defect is already proven. More geometry evidence is required to distinguish those causes.

## Repeatability

Two complete canonical CLI runs produced byte-identical files:

- `contact_controls.csv`: `db904c31a7782f51d9396ccac5bd08e6cbcbd4c0a7d0aa709a77c796703ae452`
- `fixed_closure.csv`: `0051cfbebd9b8303217cf35050a2d17ae8efa70f4e82bd6df8adbf971a670f27`
- `free_closure.csv`: `eda6d0545afd41ff44dbd93fe6d8f7fd1a96a0675ee8c96548b6c16c3d7fca91`
- `conditional_lift.csv`: `eda6d0545afd41ff44dbd93fe6d8f7fd1a96a0675ee8c96548b6c16c3d7fca91`
- `summary.json`: `a6410c75e45afbf082b1e097a9d1a243ee28dbfc564a0aa83e2e8242d6c30954`

## Next Research Task

Run contact-surface geometry diagnosis before any free-object experiment:

1. reconstruct nearest points returned by `mj_geomDistance` for each side/object pair over closure;
2. express cube pose and contact points in gripper closing-axis coordinates;
3. inspect inner/outer mesh normals, collision margins, and asymmetric finger linkage motion;
4. test a thin object aligned to the measured closing axis with no world-axis assumption;
5. rerun the same 20-frame bilateral gate without changing its threshold.

If this aligned-surface fixed test passes, proceed to free-cube closure. If it still fails, repair experiment-owned contact geometry or actuator/contact configuration first.
