# OpenArm Contact-Formation Diagnostics Design

Date: 2026-09-01

## Objective

Determine whether the current all-failure lift screen is caused by contact measurement, gripper geometry/actuation, or arm-object alignment. Establish a validated bilateral-contact baseline before any new friction screen, visual-noise study, disturbance trial, or ROS 2 integration.

This phase is simulation-only. It may support claims about this OpenArm MuJoCo model and protocol; it cannot establish real-world force closure or sim-to-real robustness.

## Research Questions and Decision Logic

1. Does the contact sampler correctly distinguish no contact, inner-finger-only contact, outer-finger-only contact, and simultaneous bilateral contact?
2. With the arm and object fixed, can quasi-static gripper closure create sustained bilateral compressive contact?
3. If fixed geometry succeeds, can a free cube retain bilateral contact without ejection under small predeclared alignment offsets?
4. Only if free-cube closure succeeds, can the object follow a slow 20 mm upward gripper motion while contact remains continuous?

Decision branches:

- Measurement controls fail: repair sampler or geom grouping. Do not interpret robot trials.
- Centered fixed cube fails: inspect geometry, actuator transmission/stroke, collision masks, and contact configuration.
- Centered fixed cube passes but current-waypoint fixed cube fails: diagnose waypoint/alignment error.
- Fixed cube passes but free cube fails: diagnose object dynamics, ejection, friction, closure speed, or force regulation.
- Free cube passes but 20 mm lift fails: diagnose lift trajectory, arm tracking, gravity retention, and object-gripper relative motion.

## Considered Approaches

### A. Continue broad parameter tuning

Vary width, friction, close fraction, and waypoint offsets again. Low implementation cost, but previous 36-condition floor gives no validated contact signal and therefore no causal localization. Rejected.

### B. Unit-test the sampler only

Feed synthetic contact IDs into the current classifier. Fast and useful for software semantics, but does not validate MuJoCo collision generation, actual geom naming, or force extraction. Retained only as one layer.

### C. Staged measurement controls and physical decision gates

Combine synthetic classifier tests with real MuJoCo contact controls, then fixed-cube closure, free-cube closure, and conditional lift. Highest diagnostic value and prevents downstream claims from relying on an unvalidated instrument. Selected.

## Scope

### Included

- Contact sampler evidence with raw geom pairs and per-contact normal force.
- Deterministic negative, inner-only, outer-only, and bilateral MuJoCo controls.
- Fixed-cube quasi-static closure using both raw actuator range and calibrated range.
- Current-waypoint versus geometric-center placement comparison.
- Free-cube closure at centered, closing-axis `-2 mm/+2 mm`, and world-Z `-2 mm/+2 mm` alignment offsets.
- Conditional slow 20 mm lift.
- CSV/JSON reports, deterministic rerun checks, tests, and result documentation.

### Excluded

- New friction, mass, width, perception-noise, or controller-policy screens.
- Disturbance testing.
- ROS 2 nodes, cameras, object detectors, hand-eye calibration, or real hardware.
- Changes to official OpenArm repositories or official MJCF assets.

## Model Semantics

Current collision grouping uses names beginning with:

- `finger_inner_left_collision` for the inner side;
- `finger_outer_left_collision` for the outer side.

Existing `ContactFrame.left_finger_contact` and `right_finger_contact` names are spatially misleading. New diagnostic records use `inner_contact` and `outer_contact`. Compatibility conversion may read existing fields, but reports must not call them physical left/right fingers.

Object contacts include only geoms under the named object body's root. Ground, frame, arm-link, and self contacts are logged separately only when useful for diagnosing ejection; they do not count as finger-object contact.

## Architecture

Create `openarm_robust_grasp/contact_diagnostics.py` as a focused module. Keep existing stability protocol behavior unchanged.

Primary units:

- `ContactEvidence`: one frame's inner/outer booleans, bilateral boolean, raw named geom pairs, per-pair normal force, per-side summed normal force, and minimum signed side-object gaps.
- `ContactSequenceStats`: sampled frames, contact frames per side, bilateral frames, maximum consecutive bilateral-positive-force frames, and force extrema.
- `ContactControlResult`: expected versus observed class for one measurement control.
- `ClosureConfig` and `ClosureResult`: immutable predeclared protocol inputs and outputs.
- `sample_contact_evidence(...)`: authoritative diagnostic sampler using named geom sets, `mj_contactForce`, and minimum `mj_geomDistance` over each finger-side/object pair.
- `run_contact_controls(...)`: four deterministic MuJoCo controls.
- `run_fixed_cube_closure(...)`: current-waypoint or geometric-center placement with raw or calibrated command profile.
- `run_free_cube_closure(...)`: settled free cube plus one predeclared alignment offset.
- `run_conditional_lift(...)`: 20 mm lift callable only after closure gate passes.
- `write_contact_diagnostic_report(...)`: validated CSV/JSON pair writer.

Create `assets/cube_contact_fixed_scene.xml` under the experiment directory. It includes the same official `cell.xml` asset but owns a fixed cube body without a free joint. No official file is edited.

Create `run_contact_diagnostics.py` as the CLI orchestrator. It stops at the first failed gate and writes explicit `not_run_reason` values for downstream stages.

## Measurement Controls

Controls exercise both classifier semantics and real MuJoCo collision/force extraction.

### Synthetic classifier controls

Use known geom-ID contact pairs for exact no-contact, inner-only, outer-only, and bilateral classifications. These tests establish software mapping independent of scene dynamics.

### MuJoCo integration controls

Use the fixed-body object geom from the experiment-owned fixed-cube scene as a temporary probe in a freshly loaded context. Move and size that fixed body from named finger collision geometry, preserve robot dynamic state around `mj_setConst`, then sample immediately after `mj_forward` with zero dynamic settle steps. This prevents actuator motion from changing the known control class before measurement:

- `negative`: probe separated from every finger geom by at least 20 mm;
- `inner_only`: small probe intersects a selected inner collision geom while remaining positively separated from all outer geoms;
- `outer_only`: mirror condition for outer collision geom;
- `bilateral`: centered probe spans the minimum inner-outer gap and intersects at least one geom from each side.

Control construction validates its own geometry using signed distances. Expected class is accepted only when geometry preconditions hold and observed contact class matches. Each positive side also requires summed compressive normal force greater than `1e-6 N`; touching with zero/negative or non-finite extracted force does not pass.

Measurement gate passes only when all four synthetic controls and all four MuJoCo controls are classified exactly on two repeated fresh-context runs.

These fixed-probe controls intentionally create geometric overlap. Their forces validate sign, finiteness, side grouping, and detectability only; force magnitude is not a calibrated or physically plausible grasp-force measurement.

## Fixed-Cube Closure Protocol

1. Load fresh fixed-cube scene and left arm.
2. Move arm using existing approach and grasp routine with zero perception/control noise.
3. Freeze arm Cartesian target and retain actuator control.
4. Evaluate two cube placements:
   - `current_waypoint`: fixed cube remains at protocol world pose;
   - `geometric_center`: move experiment-owned fixed body to the inner/outer centroid midpoint in world X/Y, while preserving declared cube-center world Z and cube orientation.
5. Evaluate two command profiles:
   - `raw_full`: model control endpoint with largest measured aperture to opposite raw endpoint;
   - `calibrated`: measured open command `0.58905` to measured closed command `0.0`.
6. Use 101 inclusive command levels. At each level, settle for 20 MuJoCo steps and sample every step.

Log command, close fraction, joint position, actuator force, inner/outer signed gaps, per-side normal forces, raw named contact pairs, cube pose, gripper pose, object-gripper relative translation, and relative rotation angle.

Fixed-closure gate passes when geometric-center placement produces bilateral normal force greater than `1e-6 N` for at least 20 consecutive sampled frames in either predeclared command profile. Current-waypoint outcome remains a diagnostic comparison, not part of this minimum mechanism gate.

## Free-Cube Closure Protocol

Use fresh contexts and existing free-cube scene. Settle open gripper and cube for 100 steps before capturing baseline. Apply alignment error to grasp target, not by teleporting the free cube after settlement.

Five deterministic offsets:

- centered: `(closing_axis=0 mm, world_z=0 mm)`;
- closing axis: `-2 mm`, `+2 mm`;
- world Z: `-2 mm`, `+2 mm`.

Closing axis is the normalized vector from inner collision-group centroid to outer collision-group centroid at the pre-close grasp pose. Closure uses the calibrated profile over 101 levels with 20 sampled settle steps per level. No lift occurs during this stage.

Primary free-closure pass criterion:

- simultaneous inner/outer contact;
- summed compressive normal force on each side greater than `1e-6 N`;
- at least 20 consecutive frames;
- cube center displacement during that 20-frame run no greater than 10 mm from its settled baseline;
- no non-finite state and no cube-frame contact transition classified as ejection.

Centered condition must pass before offset conditions can be interpreted. Offset outcomes map a local deterministic tolerance boundary; they are not success probabilities.

## Conditional 20 mm Lift

Run only for each free-cube condition that passes closure.

- Preserve end-effector orientation at closure completion.
- Raise target by exactly `0.020 m` over 200 MuJoCo steps.
- Sample contact and relative pose every step.
- No hold or disturbance phase.

Lift passes when:

- object elevation reaches at least `0.015 m` above settled baseline;
- bilateral positive-force contact persists for at least 80% of lift frames;
- final object-gripper translation drift is at most 10 mm from lift start;
- relative rotation change is at most 15 degrees;
- cube is not supported by the frame at final sample.

The 15 mm elevation threshold allows 5 mm tracking tolerance while testing a commanded 20 mm lift. This is a baseline-retention gate, not stable-grasp proof.

## Outputs

Canonical directory: `outputs/contact_formation_diagnostics/`.

- `contact_controls.csv`: expected/observed classes and force/gap evidence.
- `fixed_closure.csv`: per-frame fixed-cube trajectories.
- `free_closure.csv`: per-frame free-cube trajectories.
- `conditional_lift.csv`: per-frame lift trajectories, empty with recorded reason when no condition is eligible.
- `summary.json`: protocol constants, gate outcomes, failure branch, environment/model identifiers, condition summaries, and output hashes.

Writers validate schema, finite numeric values, non-negative counts/forces, exact condition identity/order, and cross-file stage eligibility before replacing canonical outputs. Generate all bytes first and use the existing rollback-protected replacement pattern.

## Testing and Verification

Development follows test-first RED-GREEN-REFACTOR cycles in the VM.

Required tests:

- exact inner/outer/bilateral classifier semantics;
- geom-set validation, including empty and overlapping sets;
- normal-force extraction and aggregation;
- signed-gap calculation and finite-value rejection;
- exact four-control identity and repeated fresh-context determinism;
- raw and calibrated command endpoint/direction semantics;
- exact 101-by-20 sampling counts;
- exact five free-cube offsets and axis definition;
- consecutive-frame gate boundaries at 19/20 frames;
- ejection, non-finite state, frame support, translation drift, and rotation drift failures;
- conditional lift cannot execute after failed closure;
- report validation and rollback on write failure;
- full existing suite remains green.

Real integration runs execute twice in the same VM. Reports must state byte equality or list nondeterministic fields and their measured tolerance; nondeterminism cannot be silently discarded.

## Scientific Claim Boundary

Primary outcome is stage reached in the decision tree, not number of successful tuned conditions. Negative results remain valid.

Predeclared thresholds are engineering diagnostic thresholds, not literature-validated biological or industrial standards. No p-values or confidence intervals are appropriate for this deterministic gate study. Future seeded robustness experiments begin only after centered free closure and conditional lift pass.

All interpretations separate observations from causes. Example: zero sampler output after measurement controls pass supports absence of detected named bilateral contact in this model run; it does not prove universal physical impossibility.

## Completion Criteria

Phase completes when:

1. measurement controls pass or produce a localized sampler/geometry defect report;
2. next reachable physical gate is executed without post-hoc threshold changes;
3. canonical artifacts and cautious result note are written;
4. two-run repeatability and full test suite are recorded;
5. next task is selected from the predeclared decision branch.
