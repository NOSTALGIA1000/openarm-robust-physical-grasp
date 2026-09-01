# Simulation Grasp Stability — 36-Condition Exploratory Screen

## Scope and authority

Protocol v2 is authoritative. It corrects height baseline, close-phase contact observation, and conditional disturbance interpretation after independent review. Protocol v1 remains historical evidence below. Both used the same fixed 36-condition simulation grid; no parameter adaptation or tuning followed either result.

No XML/MJCF or official OpenArm repository file was modified. This is simulation-only exploratory evidence, not evidence of real-world robustness, force closure, statistical significance, hardware safety, or sim-to-real transfer.

## Environment

- Run date: 2026-08-22
- VM: Linux 6.8.0-136-generic x86_64
- Python: 3.10.12
- MuJoCo: 3.10.0
- NumPy: 1.21.5
- Scene: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml`
- Scene SHA-256: `10b2402dba317623aacbd638d931c0f8b2ded4d8f7c7e879fad092b270fe2d24`
- Cube mass: `0.015 kg`
- Seed: `0`

MuJoCo emitted the existing attach/timestep conflict warning while loading the scene: parent timestep `0.002`, child timestep `0.001`, parent retained. Execution completed without scene changes.

## Calibration and fixed grid

Close fraction means fraction of calibrated closing stroke. Fraction `0.0` maps to measured open command `0.58905`; fraction `1.0` maps to closed command `0.0`. Screen levels are `0.25`, `0.50`, `0.75`, and `1.00`.

Gripper characterization artifacts:

- VM/experiment path: `outputs/gripper_characterization/results.csv`
- Windows path: `vm_sync/experiments/openarm_robust_grasp/outputs/gripper_characterization/results.csv`
- CSV SHA-256: `7506b62b064e323be248d5de49542423e238d90298406c47af1db195d8ea5991`
- VM/experiment path: `outputs/gripper_characterization/summary.json`
- Windows path: `vm_sync/experiments/openarm_robust_grasp/outputs/gripper_characterization/summary.json`
- JSON SHA-256: `bbcd9c39490aad1c11eabf6a5c85186c222cb4730e9851019a448e256ec19ee0`

Key characterization values: closed/open opposing-geometry center distances `0.009055178757362791/0.06176896571432234 m`, aperture span `0.05271378695695955 m`, monotonic distance `true`, meaningful geometric motion `true`, geometric closure gate `true`.

Grid order is close fraction outer, cube width middle, sliding friction inner:

- close fraction: `0.25`, `0.50`, `0.75`, `1.00`
- cube full width: `0.036`, `0.042`, `0.046 m`
- sliding friction: `1.0`, `2.0`, `4.0`

The Cartesian product contains 36 unique design points.

## Protocol v2

Each row uses 100 open-settle steps; 80 approach steps; 80 grasp steps; 100 close-settle steps; 120 lift steps targeting `0.08 m`; 500 pre-disturb hold steps; 100 disturbance steps with world force `(0.05, 0.0, 0.0) N`; and 250 post-disturb hold steps. Every completed row records 1,330 total steps.

Height baseline is object height after open-settle. Achieved lift, final lift, and pre-disturb elevation use that settled baseline. Close-settle bilateral contact is sampled every close step.

Disturbance eligibility requires both:

1. pre-disturb elevation `>= 0.05 m`;
2. pre-disturb bilateral-contact fraction `>= 0.5`.

`disturbance_survival_success` is NA when ineligible. For eligible rows only, it is `disturbance_drop_m <= 0.02 m`. Stable grasp requires final lift `>= 0.05 m`, both pre- and post-disturb bilateral-contact fractions `>= 0.5`, and disturbance survival exactly true.

IK success means the existing motion routine reported completion. It does not verify Cartesian placement accuracy.

`stable_grid_fraction` is successful fixed design points divided by 36. Compatibility field `stable_rate` has the same value. Neither is a probability estimate.

## Protocol-v2 computational repeatability

Two complete runs used separate VM temporary directories:

- run 1: 36 rows, `31.88 s`
- run 2: 36 rows, `31.11 s`

Byte-identical pairs:

- `results.csv`: `278665c8b8f076813af078c831dfcda0217cda12abb7485570f63621de3c1019`
- `summary.json`: `ce0fe64310802bc594be7aae19e5632fdcc2cb7d53e2e5efcb1039423cbfd1c6`

Run 1 became canonical VM and Windows output.

## Protocol-v2 aggregate result

- rows: 36
- IK motion-routine completion: 36/36
- close sampled frames: 100 per row
- maximum close bilateral-contact fraction: `0.0`
- final-elevation success: 0/36
- hold-contact success: 0/36
- disturbance eligible: 0/36
- disturbance-survival successes: 0; rate among eligible: NA
- stable-grasp success: 0/36
- stable grid fraction: `0.0`
- achieved lift: `0.00039812909691194776` to `0.004946805896357764 m`
- final lift: `1.2537038074356133e-10` to `7.438614169075208e-10 m`
- pre-disturb elevation: `1.2536971460974655e-10` to `7.438605287291011e-10 m`
- maximum pre/post hold bilateral-contact fractions: `0.0/0.0`
- maximum disturbance drop: `4.069322656619079e-11 m`

All rows are ineligible for disturbance-success evaluation. Floor-supported cubes therefore receive no survival-success claim.

## Descriptive best observations

The current sampler detected no bilateral contact during close, lift, or either hold. All 36 rows tie at zero for these measured contact fields.

Largest transient achieved lift was `0.004946805896357764 m`, observed at close fraction `1.0`, width `0.036 m`, and each of the three friction levels. Largest final lift was `7.438614169075208e-10 m`, shared by 12 width-`0.036 m` rows. These values remain far below the `0.05 m` elevation threshold and do not identify factor effects.

The all-failure floor cannot distinguish effects of close fraction, cube width, or friction. It does not show that those factors have no effect. It also does not identify a root cause.

## Protocol v1 — historical pre-review evidence

Protocol v1 used pre-open-settle object height as baseline, omitted close-settle contact sampling, and marked floor-supported rows as disturbance-survival successes based only on tiny drop. Those interpretations are superseded by v2.

Historical v1 runs:

- runtimes: `31.88 s` and `31.58 s`
- CSV SHA-256: `3306e495914d895b751a9285761948f79498a86dc59d70b3ab14b2d016606890`
- JSON SHA-256: `08824d3340369385d8f40e9f3cf2b5bd77948c30ab3fe5eb52ea9297337d90b1`
- historical stable-grasp result: 0/36

V1 files are not canonical. V2 canonical outputs are authoritative.

## Limitations

One deterministic seed, one scene, and one fixed grid provide a computational screen, not uncertainty estimates. Motion-routine completion does not establish target-placement accuracy. Contact controls needed to validate bilateral-contact measurement are not yet defined. No causal or factor-effect claim follows from the all-failure floor.

## Mandatory next decision experiment

Next task must stage contact formation before lifting:

1. negative, left-only, right-only, and bilateral contact measurement controls;
2. fixed centered cube with quasi-static full-range closure;
3. free-cube closure with predeclared `-2 mm` and `+2 mm` offsets;
4. only after sustained bilateral contact passes, a `20 mm` lift.

Those controls are deferred because this fix round lacks explicit safe geometry/control definitions; adding them would invent geometry. They remain mandatory gates, not completed work.

Canonical artifacts:

- `vm_sync/experiments/openarm_robust_grasp/outputs/stability_screen/results.csv`
- `vm_sync/experiments/openarm_robust_grasp/outputs/stability_screen/summary.json`
