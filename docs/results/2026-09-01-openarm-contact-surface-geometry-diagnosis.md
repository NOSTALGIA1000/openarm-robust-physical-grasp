# OpenArm Contact-Surface Geometry Diagnosis Result

Date: 2026-09-01

## Outcome

The real MuJoCo nearest-point instrument gate passed. The unloaded linkage baseline completed all 2020 frames. Sphere, closing-frame-aligned cuboid, and identity-world cube each completed all 2020 fixed-object frames and all failed the unchanged requirement: both named finger sides must exceed `1e-6 N` for every frame of one complete 20-step command level.

Predeclared decision: `linkage_stroke_or_contact_configuration`.

Consequences: do not run free-cube closure, friction tuning, mesh repair, or lift trials from this branch. Next work must separate linkage/stroke geometry from contact/collision configuration.

## Protocol

- Simulation only, fresh context per physical condition.
- Calibrated gripper command: `0.58905 -> 0.0`.
- 101 inclusive command levels, 20 MuJoCo steps per level.
- Fixed order and dimensions:
  - sphere radius `0.021 m`;
  - aligned cuboid half-sizes `(0.021, 0.010, 0.015) m` along measured `(e_close, e_tangent, e_vertical)`;
  - world cube half-sizes `(0.021, 0.021, 0.021) m`, identity world orientation.
- Object center: measured open-pose nearest-surface midpoint.
- No threshold, dimension, order, or pose adaptation after data inspection.

## Instrument Evidence

The canonical instrument controls passed positive segment consistency, geom-order point ownership, and set-order invariance. The fresh fixed scene required 100 calibrated-open settle steps before the closest finger surfaces became positively separated: initial minimum distance was `-3.275727633648008e-06 m`, 20-step distance was `-1.3231107385124279e-05 m`, and 100-step distance was `0.005001302224517471 m`.

Penetrating states never interpret `mj_geomDistance` nearest points. Derived midpoint/axis fields are null whenever signed finger-finger distance is non-positive.

## Unloaded Linkage Evidence

- Sampled frames: `2020`.
- Positive-point valid frames: `1712/2020` (`0.8475247524752475`).
- Probe clearance: `1.9483701283627284 m`.
- Selected-pair switches: `520`.
- Maximum valid-frame midpoint drift: `0.11667818999490626 m`.
- Maximum valid-frame closing-axis drift: `0.19070662235106797 rad`.
- Maximum valid-frame symmetry residual: `0.04376582487105043 m`.
- Minimum finger-finger signed distance: `-0.05828877207553513 m`.
- Maximum unique finger-finger contact-pair count: `0`.

These values describe nearest collision surfaces, not anatomical fingertip landmarks. The 520 pair switches can introduce local discontinuities, so midpoint and symmetry extrema must not be interpreted as one smooth rigid linkage trajectory without pair-conditioned analysis.

## Fixed-Shape Outcomes

| Condition | Frames | Gate | Max inner/outer force (N) | Min inner/outer gap (m) |
| --- | ---: | --- | ---: | ---: |
| sphere | 2020 | fail | `7184.962799939039 / 6467.99803837755` | `-0.01848675698661359 / -0.01850957789413029` |
| aligned cuboid | 2020 | fail | `14391.162746403852 / 11351.54571749501` | `-0.018671868939175384 / -0.018660426443257352` |
| world cube | 2020 | fail | `8439.606547678035 / 6973.458689917848` | `-0.021737029501039718 / -0.02131117084910214` |

Some frames contained simultaneous secondary contact-normal diagnostics on both sides, especially the aligned cuboid. None formed one complete command level with bilateral force strictly above threshold on every frame. Instantaneous bilateral contact therefore does not satisfy the primary mechanism gate.

Force magnitudes are fixed-overlap solver responses. They are not calibrated or plausible grasp-force estimates.

## Repeatability and Artifacts

Two complete canonical CLI runs produced identical hashes for all six files:

- `instrument_controls.csv`: `fb74068ff3edd0da87c58c296f1d399d84dd053b58594705d8243f14c4199d62`
- `linkage_kinematics.csv`: `e134889fbe5d998771cd9d8ec1b87a7d728d519f723e7195e17cf40c9f178e6e`
- `shape_ladder.csv`: `089019c281f8e0589662f191d36d8603fbf226e7bece5491d23d5bc2505a86f3`
- `summary.json`: `248a700986c773c4694c3d0ed8cda91250701e3a5d27d4d54544ed644d1c417e`
- `midpoint_axis_drift.png`: `e7613ef7c535ae973100a7b68ab2588109ed7d8b13497789663f47b3aacc8f9a`
- `shape_gap_force.png`: `1da210f414715a8ab595608825887574e2ae1d8f4e445979d0b38b24eafff2e4`

Windows mirror: `C:\Users\ASUS\Documents\ChatGPT\open-arm相关\vm_sync\experiments\openarm_robust_grasp\outputs\contact_surface_geometry\`.

VM canonical directory: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/outputs/contact_surface_geometry/`.

## Supported Interpretation

Orientation freedom and explicit closing-frame-aligned faces did not recover sustained bilateral contact. Under the predeclared rules, face/world orientation alone is insufficient to explain prior failure. Evidence supports the broader branch `linkage/stroke/contact configuration` in this OpenArm MuJoCo model.

This experiment does not yet distinguish actuator transmission/linkage kinematics from collision grouping, self-contact masks, solver contact persistence, or local collision-geom topology. It also does not support real-hardware conclusions, calibrated force claims, grasp probability, friction irrelevance, or universal gripper impossibility.

## Branch-Authorized Next Task

Run a pair-conditioned linkage/contact-configuration audit before any free object:

1. map each named inner/outer collision geom's signed distance and projected motion versus gripper joint position;
2. separate continuous intervals by selected geom-pair identity instead of aggregating across 520 switches;
3. compare actuator transmission, tendon/joint constraints, collision masks, and actual finger-finger constraints against expected linkage closure;
4. use fixed non-penetrating bilateral controls to measure contact persistence per complete command level;
5. propose a model change only after one mechanism is isolated, then test it with a new preregistered protocol.

Free-cube closure remains blocked until a sustained fixed-object bilateral gate passes.
