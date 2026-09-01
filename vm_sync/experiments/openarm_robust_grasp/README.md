# OpenArm Robust Grasp Experiment

Minimal MuJoCo benchmark for OpenArm visual-grasp robustness experiments.

## Environment

Expected VM environment:

- Ubuntu 22.04
- Python 3.10
- `mujoco==3.10.0`
- OpenArm MuJoCo assets at `/home/nostalgia/openarm_ws/src/openarm_mujoco`

The default scene is:

`/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml`

This repository also includes an experiment-owned cube scene:

`/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml`

It includes the official OpenArm MuJoCo `cell.xml` by absolute path, but keeps the cube size, mass, friction, and contact parameters under this experiment directory. This avoids modifying the upstream OpenArm assets while supporting controlled computational reruns.

## What This Runs

The first benchmark reports both proxy and contact-aware metrics:

- one OpenArm arm
- MuJoCo ground-truth bottle pose
- configurable translation noise as a perception-error proxy
- open-loop policy
- pre-grasp correction policy
- CSV, JSON summary, and PNG plots

Metric meanings:

- `proxy_success`: first-stage reachability metric based on EE-base proximity and lift pose reachability.
- `contact_success`: IK succeeded and finger-object contact was observed.
- `post_grasp_contact_success`: IK succeeded and finger-object contact was observed in the grasp/close window or at the lift endpoint, excluding the approach endpoint.
- `lift_contact_success`: IK succeeded and finger-object contact was observed at the end of the lift motion. It does not prove continuous contact throughout the motion.
- `grasp_success`: the current contact-plus-2cm-lift threshold: IK succeeded, contact was observed, and the object's maximum rise exceeded 2 cm. It does not prove stable two-finger lifting.

The bottle policy reaches plausible grasp poses but does not create reliable finger-object contact. Treat `proxy_success` as a controller/perception robustness signal, not as physical grasp evidence. The `contact_push` policy on `demo.xml/orange_cube` creates reliable contact, but not lift.

For cube/contact policies, waypoints are expressed in fingertip-contact geometry rather than raw EE-base geometry. The controller estimates the mean position of the left finger collision geoms and compensates the current EE-to-finger-center offset before solving IK. This makes the benchmark closer to a contact-planning problem and avoids using the OpenArm EE base as a misleading grasp point.

The CLI runner uses paired seeds: for the same translation-noise value and local trial index, every policy receives the same random seed. This makes policy comparisons less sensitive to different sampled perception noise.

Policy names:

- `ee_base_contact_push`: legacy baseline using direct EE-base Cartesian offsets.
- `finger_center_contact_push`: contact-geometry baseline using mean finger collision center compensation.
- `pre_grasp_finger_center_correction`: finger-center approach from the first noisy observation, followed by a second simulated observation and replanned finger-center grasp/lift.
- `contact_push`: backward-compatible alias for `finger_center_contact_push`.
- `cube_lift`: finger-center contact followed by a lift motion.

## Test

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 -m pytest tests -q
```

## Smoke Run

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py --trials 2 --out outputs/smoke
```

## Cube Contact Benchmark

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/src/openarm_mujoco/v2/demo.xml \
  --object orange_cube \
  --policies contact_push \
  --translation-noise 0 0.005 0.01 \
  --trials 3 \
  --out outputs/cube_contact_smoke
```

This benchmark currently produces reliable finger-cube contact. It does not lift the cube, so `contact_success_rate` can be high while `success_rate` remains zero.

## Cube Lift Probe

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/src/openarm_mujoco/v2/demo.xml \
  --object orange_cube \
  --policies cube_lift \
  --translation-noise 0 0.005 0.01 \
  --trials 3 \
  --out outputs/cube_lift_smoke
```

Current result: `cube_lift` reaches the cube and creates finger contact, but `object_lift_m` remains zero. This indicates the current gripper/object/contact setup is not yet a physical lifting grasp. The next fix should inspect or modify the MJCF gripper closing geometry, object size, friction, or contact parameters instead of only changing Cartesian waypoints.

## Experiment-Owned Cube Lift Scene

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies cube_lift \
  --translation-noise 0 \
  --trials 3 \
  --out outputs/cube_lift_scene_smoke
```

Verified result:

- `ik_success_rate`: `1.0`
- `proxy_success_rate`: `1.0`
- `contact_success_rate`: `1.0`
- `success_rate` / contact-plus-2cm-lift threshold rate: `0.0`
- `mean_object_lift_m`: `0.0`
- `mean_finger_object_contacts`: `82.0`

Diagnostic note: lowering the cube mass and increasing friction makes contact stable but still does not produce lift. A waypoint sweep showed that moving the left-arm target x-offset from about `-0.04 m` toward `-0.10` to `-0.13 m` centers the finger collision geometry near the cube, yet `max_object_lift` remains zero. This suggests the next research task should model the end-effector/finger contact geometry explicitly instead of relying on EE-base waypoints.

After switching cube/contact policies to finger-center waypoints, a small robustness smoke run is:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies contact_push cube_lift \
  --translation-noise 0 0.005 0.01 \
  --trials 3 \
  --out outputs/finger_center_cube_smoke
```

Observed contact-success rates:

- `contact_push`: `1.0` at `0 mm`, `0.33` at `5 mm`, `0.67` at `10 mm`
- `cube_lift`: `1.0` at `0 mm`, `0.33` at `5 mm`, `0.33` at `10 mm`

True lift success remains `0.0`, so this is currently a contact-robustness benchmark rather than a physically solved lift benchmark.

## EE-Base vs Finger-Center Baseline Comparison

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies ee_base_contact_push finger_center_contact_push \
  --translation-noise 0 0.005 0.01 \
  --trials 5 \
  --out outputs/baseline_vs_finger_center_smoke
```

Observed smoke summary:

| policy | noise | contact success | contact + 2cm lift | mean object motion |
| --- | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | `0 mm` | `1.0` | `0.0` | `0.041 m` |
| `ee_base_contact_push` | `5 mm` | `1.0` | `0.0` | `0.026 m` |
| `ee_base_contact_push` | `10 mm` | `1.0` | `0.0` | `0.025 m` |
| `finger_center_contact_push` | `0 mm` | `1.0` | `0.0` | `0.022 m` |
| `finger_center_contact_push` | `5 mm` | `0.0` | `0.0` | `0.022 m` |
| `finger_center_contact_push` | `10 mm` | `0.4` | `0.0` | `0.023 m` |

Interpretation: the old EE-base baseline creates more contacts in this tuned scene, but those contacts are likely push/collision contacts rather than stable grasp contacts: the contact-plus-2cm-lift threshold remains zero and object motion is higher. The finger-center baseline is stricter and more sensitive to perception noise, which makes it useful for studying visual localization robustness.

## Pre-Grasp Correction Comparison

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies ee_base_contact_push finger_center_contact_push pre_grasp_finger_center_correction \
  --translation-noise 0 0.005 0.01 \
  --trials 5 \
  --out outputs/pregrasp_correction_smoke_paired
```

Observed smoke summary:

| policy | noise | contact success | contact + 2cm lift | mean object motion | mean finger contacts |
| --- | ---: | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | `0 mm` | `1.0` | `0.0` | `0.041 m` | `82.0` |
| `ee_base_contact_push` | `5 mm` | `1.0` | `0.0` | `0.027 m` | `82.0` |
| `ee_base_contact_push` | `10 mm` | `1.0` | `0.0` | `0.026 m` | `72.4` |
| `finger_center_contact_push` | `0 mm` | `1.0` | `0.0` | `0.022 m` | `3.0` |
| `finger_center_contact_push` | `5 mm` | `0.0` | `0.0` | `0.022 m` | `0.0` |
| `finger_center_contact_push` | `10 mm` | `0.2` | `0.0` | `0.064 m` | `16.4` |
| `pre_grasp_finger_center_correction` | `0 mm` | `1.0` | `0.0` | `0.043 m` | `119.0` |
| `pre_grasp_finger_center_correction` | `5 mm` | `1.0` | `0.0` | `0.036 m` | `125.6` |
| `pre_grasp_finger_center_correction` | `10 mm` | `1.0` | `0.0` | `0.034 m` | `126.4` |

Interpretation: in this small smoke run, pre-grasp correction restores contact success for the stricter finger-center strategy under `5-10 mm` simulated translation noise. This should be reported as an effect of an additional simulated pre-grasp observation, not as a pure controller improvement. The higher object motion and contact count also mean the policy may be making stronger contact/push interactions; it still does not solve physical lifting.

## First Grid

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py --trials 20 --out outputs/latest
```

Default grid:

- policies: `open_loop`, `pre_grasp_correction`
- translation noise std: `0`, `5`, `10`, `20`, `30` mm
- trials per setting: `20`

Recommended contact-robustness grid:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies ee_base_contact_push finger_center_contact_push pre_grasp_finger_center_correction \
  --translation-noise 0 0.002 0.005 0.01 0.015 0.02 0.03 \
  --trials 30 \
  --out outputs/contact_robustness_grid_stage_paired
```

The stage-level 30-trial paired-seed result is summarized in the host-side note:

```text
C:\Users\ASUS\Documents\ChatGPT\open-arm相关\docs\results\2026-08-12-contact-robustness-grid.md
```

## Outputs

Each run writes:

- `results.csv`: one row per trial
- `summary.json`: grouped success/error summary
- `success_vs_noise.png`: success rate curve
- `error_vs_noise.png`: mean minimum EE-object distance curve

Recent versions also write stage-level contact fields to CSV/JSON:

- `approach_finger_object_contacts`
- `grasp_finger_object_contacts`
- `lift_finger_object_contacts`
- `post_grasp_contact_success`
- `lift_contact_success`

Per-step contact observation also writes:

- `approach_sampled_frames`, `grasp_sampled_frames`, `lift_sampled_frames`
- `approach_contact_frames`, `grasp_contact_frames`, `lift_contact_frames`
- `approach_two_finger_contact_frames`, `grasp_two_finger_contact_frames`, `lift_two_finger_contact_frames`
- `approach_contact_frame_rate`, `grasp_contact_frame_rate`, `lift_contact_frame_rate`
- `grasp_two_finger_contact_success`, `lift_two_finger_contact_success`, `sustained_lift_contact_success`

Sampling windows are stage-based and per simulation step:

- `approach`: every step of the 80-step approach motion
- `grasp`: every step of the 80-step grasp motion plus all 40 gripper-close steps, so 120 sampled frames
- `lift`: every step of the 80-step lift motion

For the three formal robustness policies (`ee_base_contact_push`, `finger_center_contact_push`, `pre_grasp_finger_center_correction`), this is an `80/120/80` sampling split. `cube_lift` keeps a compatibility exception: its lift motion still runs for 120 steps, so its sampled-frame split is `80/120/120`.

The per-step rates are normalized contact-frame fractions, so each stage rate is `contact_frames / sampled_frames` and becomes `0.0` when a stage sampled zero frames. Two-finger contact means both opposing finger sides touched the object in the same simulation step, not just at different times in the same stage.

`grasp_two_finger_contact_success` and `lift_two_finger_contact_success` are IK-gated boolean diagnostics for whether at least one same-frame opposing-finger contact was observed in that stage. `sustained_lift_contact_success` is also IK-gated and requires opposing-finger contact in at least 50% of sampled lift frames. These are diagnostic contact-observation metrics only; they do not by themselves prove force closure, stable grasping, or a physically solved lift.

## 36-Condition Exploratory Stability Screen

Run the fixed deterministic simulation screen from the experiment root:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_stability_screen.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --output-dir outputs/stability_screen
```

The CLI defaults to those scene and output paths, so `python3 run_stability_screen.py` is equivalent when run from any directory.

The Cartesian grid contains exactly 36 rows in this deterministic order:

- close fraction outer loop: `0.25`, `0.50`, `0.75`, `1.00`
- cube full width middle loop: `0.036`, `0.042`, `0.046 m`
- sliding friction inner loop: `1.0`, `2.0`, `4.0`
- fixed cube mass: `0.015 kg`
- fixed seed: `0`

All other values remain the Task 2 protocol defaults. Close fraction means fraction of the calibrated closing stroke, not a raw actuator command. Calibration endpoints are `0.58905` at fraction `0.0` (open) and `0.0` at fraction `1.0` (closed); the screen samples fractions `0.25-1.0` through that linear mapping.

Protocol-v2 height metrics use object height after the 100-step open-settle phase as baseline. Close-settle contact is sampled on every close step. Disturbance eligibility requires pre-disturb elevation at least `0.05 m` and pre-disturb bilateral-contact fraction at least `0.5`. `disturbance_survival_success` is blank/NA when ineligible; eligible rows pass it only when disturbance drop is at most `0.02 m`.

Outputs:

- `outputs/stability_screen/results.csv`: one `StabilityResult.to_row()` row per condition
- `outputs/stability_screen/summary.json`: exact parameter levels, component/eligibility counts, conditional disturbance survival, aggregate metrics, and stable counts grouped by each screened factor

The writer validates all 36 rows and builds both output byte streams before replacing either canonical file. Both files are staged in the output directory and replaced as one rollback-protected pair.

Stable grasp means final lift at least `0.05 m`, both pre- and post-disturb hold two-finger-contact fractions at least `0.5`, and eligible disturbance survival exactly true. IK success means the existing motion routine reported completion; it is not verified Cartesian placement accuracy and is not sufficient for stability.

`stable_grid_fraction` is the fraction of the 36 fixed design points meeting the conjunction. Compatibility field `stable_rate` has the same value. Neither is a probability estimate.

Gripper characterization artifacts used by the calibrated mapping:

- `outputs/gripper_characterization/results.csv` — SHA-256 `7506b62b064e323be248d5de49542423e238d90298406c47af1db195d8ea5991`
- `outputs/gripper_characterization/summary.json` — SHA-256 `bbcd9c39490aad1c11eabf6a5c85186c222cb4730e9851019a448e256ec19ee0`

Key gate values: closed/open commands `0.0/0.58905`, closed/open opposing-geometry center distances `0.009055178757362791/0.06176896571432234 m`, aperture span `0.05271378695695955 m`, monotonic distance `true`, meaningful geometric motion `true`, closure gate `true`.

This is a deterministic simulation-only exploratory map. It does not establish real-world robustness, force closure, statistical significance, or sim-to-real transfer. An all-failure screen is valid evidence and must be reported without adapting this fixed grid after inspection.

## Next Research Upgrade

Next task must be a staged contact-formation decision experiment, not tuning of this screen:

1. Define negative, left-only, right-only, and bilateral contact measurement controls.
2. Run a fixed centered cube with quasi-static full-range closure.
3. Run free-cube closure with predeclared `-2 mm` and `+2 mm` offsets.
4. Only after sustained bilateral contact passes, attempt a `20 mm` lift.

Historical Task 3 status: those measurement controls were mandatory gates but were not implemented in that stability-screen fix round. The later contact-formation section below supersedes this implementation-status sentence. The protocol-v2 all-failure floor itself still identifies no factor effects and no root cause.

## Contact-Formation Diagnostics

Run the validated staged contact protocol:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_contact_diagnostics.py
```

The runner first executes two repeats of negative, inner-only, outer-only, and bilateral fixed-probe controls. It then runs four fixed-cube closures: current-waypoint/geometric-center placement crossed with raw-full/calibrated 101-point command profiles, sampling 20 steps per command.

Canonical result directory: `outputs/contact_formation_diagnostics/`.

- `contact_controls.csv`: expected/observed class, signed gaps, forces, raw named pairs.
- `fixed_closure.csv`: 8080 per-frame command, joint, force, gap, contact, and relative-pose rows.
- `free_closure.csv` and `conditional_lift.csv`: header-only when upstream fixed gate fails; skip reasons are in summary.
- `summary.json`: gate states, branch reason, fixed-condition summaries, and CSV hashes.

Current result: measurement gate passes; fixed mechanism gate fails with zero bilateral frames in all four conditions. Free-cube closure and lift are intentionally not run. Fixed overlap forces are diagnostic only and must not be interpreted as realistic grasp forces. See host result note `docs/results/2026-09-01-openarm-contact-formation-diagnostics.md`.

## Contact-Surface Geometry Diagnosis

Run the deterministic nearest-surface, unloaded-linkage, and fixed-shape protocol:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_surface_geometry_diagnosis.py
```

The CLI has fixed scientific constants: calibrated command `0.58905 -> 0.0`, 101 inclusive levels, 20 steps per level, strict per-side force threshold `>1e-6 N`, and exact shape order `sphere`, `aligned_cuboid`, `world_cube`. Dimensions and gate parameters are intentionally not exposed as tuning flags.

Canonical directory: `outputs/contact_surface_geometry/`.

- `instrument_controls.csv`: positive nearest-point segment, geom-order swap, and set-order invariance controls.
- `linkage_kinematics.csv`: 2020 unloaded frames with selected geom pair, positive-gap points, midpoint/axis drift, symmetry, and finger-finger contact evidence.
- `shape_ladder.csv`: 6060 fixed-shape frames with named contacts, per-side gap/force, object-local points, contact positions/normals, and alignment diagnostics.
- `summary.json`: protocol constants, denominators, extrema, shape gates, decision, and CSV hashes.
- `midpoint_axis_drift.png`, `shape_gap_force.png`: deterministic diagnostic plots.

Current canonical result: instrument gate passes; all three fixed shapes fail the unchanged complete-level bilateral gate. The selected branch is `linkage_stroke_or_contact_configuration`; free-cube closure remains ineligible. Unloaded positive-point coverage is `1712/2020`; maximum measured midpoint drift is `0.11667818999490626 m`, maximum closing-axis drift is `0.19070662235106797 rad`, and pair-switch count is `520`. Pair switches are local nearest-geom identity changes, not automatically actuator discontinuities. Per-side surface fields are displacements from the selected open-pair surface points projected onto open `e_close`, not positions relative to the midpoint.

Large forces arise from overlap against kinematically fixed objects and are solver diagnostics only. This protocol supports claims about this MuJoCo model's deterministic contact formation; it does not establish force closure, real-world grasp force, robustness probability, or sim-to-real performance. See host result note `docs/results/2026-09-01-openarm-contact-surface-geometry-diagnosis.md`.
