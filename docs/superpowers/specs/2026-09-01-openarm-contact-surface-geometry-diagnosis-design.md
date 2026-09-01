# OpenArm Contact-Surface Geometry Diagnosis Design

Date: 2026-09-01

## Objective

Determine why the validated OpenArm MuJoCo contact sampler observes only one named finger side during fixed-object closure. Separate three competing explanations before changing collision meshes or continuing to free-object grasping:

1. asymmetric finger linkage or actuator-stroke motion shifts the closing midpoint;
2. object surface orientation or mesh normal prevents an opposing contact pair;
3. world-frame placement is inconsistent with the measured gripper closing frame.

This phase is deterministic, simulation-only, and mechanism-diagnostic. It does not estimate grasp probability or real-hardware performance.

## Prior Evidence

The previous contact-formation protocol validated negative, inner-only, outer-only, and bilateral measurement controls on two fresh-context repeats. Its 8080 fixed-cube frames contained zero bilateral frames:

- current-waypoint placement contacted only the outer side;
- centroid-centered placement contacted only the inner side;
- raw-full and calibrated command profiles produced the same side-exclusion pattern;
- large fixed-object overlap forces are solver diagnostics, not plausible grasp forces.

In the calibrated trajectories, current-waypoint outer contact begins at open command `0.58905`, while centroid-centered inner contact begins near command `0.2886345`. The selected finger joint ends at different signed positions under the two fixed-contact loads. These observations motivate a no-object kinematic baseline before another object trial.

## Considered Approaches

### A. Add nearest-point logging only

Low code and runtime cost. Describes surface motion but cannot test whether orientation-free or closing-axis-aligned objects restore bilateral contact. Rejected as insufficient for causal localization.

### B. No-object linkage baseline plus controlled shape ladder

Measure opposing surface points and closing-frame drift without object reaction forces. Then compare an orientation-free sphere, a closing-axis-aligned cuboid, and a world-axis cube under the same calibrated closure. Selected because each condition removes one confound while preserving the validated contact gate.

### C. Edit finger collision meshes immediately

Could produce bilateral contact quickly but would erase evidence distinguishing model geometry from control and placement errors. Deferred until the shape ladder excludes simpler explanations.

## Research Questions and Falsifiable Predictions

### RQ1: Does the unloaded closing frame drift asymmetrically?

Prediction supporting linkage asymmetry: inner/outer nearest-surface midpoint or closing-axis direction changes systematically across calibrated closure instead of remaining approximately centered and parallel. Magnitudes are reported continuously; no post-hoc binary drift threshold will be invented.

### RQ2: Can an orientation-free object produce bilateral contact?

Prediction: a fixed sphere centered at the measured open-pose surface midpoint reaches the unchanged bilateral gate if stroke and contact configuration can oppose an object independent of face orientation.

### RQ3: Can explicit parallel faces produce bilateral contact?

Prediction: a fixed cuboid whose thickness axis follows the measured closing axis reaches the bilateral gate if prior failure was caused by world-axis face orientation or centroid-based placement.

### RQ4: Does a world-axis cube still fail at the same measured surface midpoint?

Prediction: aligned cuboid pass plus world-cube fail supports an orientation/face-normal explanation. Failure of all shapes supports linkage/stroke/contact configuration, not friction.

## Instrument Validation

Nearest-point evidence uses `mujoco.mj_geomDistance(model, data, geom1, geom2, 1.0, fromto)`.

Before robot experiments, tests and real MuJoCo controls must establish:

- finite signed distance;
- for positive separation, `norm(point_geom2 - point_geom1)` equals reported distance within `1e-9 m`;
- swapping geom order preserves distance and swaps point ownership;
- selected minimum is independent of geom-set iteration order;
- empty, overlapping, missing, and non-finite inputs fail explicitly.

Nearest-point coordinates are not interpreted during penetration. Once signed distance is non-positive, contact position, normal, geom pair, and normal force come from MuJoCo contact constraints. This avoids treating undefined or coincident penetration `fromto` values as physical surface points.

## Gripper Closing Frame

At the open calibrated command after approach/grasp motion:

1. evaluate every named inner/outer collision-geom pair;
2. choose the pair with minimum positive signed surface distance, breaking exact ties by `(inner_geom_id, outer_geom_id)`;
3. define `p_inner` and `p_outer` from returned nearest points;
4. define closing axis `e_close = normalize(p_outer - p_inner)`;
5. define surface midpoint `p_mid = 0.5 * (p_inner + p_outer)`;
6. project world Z into the plane orthogonal to `e_close` and normalize it as `e_vertical`;
7. define `e_tangent = normalize(cross(e_vertical, e_close))`;
8. require pair distance greater than `1e-6 m`, finite orthonormal axes, right-handed determinant within `1e-6` of `+1`, and `abs(dot(world_z, e_close)) < 0.95`.

Frame columns are `[e_close, e_tangent, e_vertical]`. Object poses use this frame without assuming OpenArm EE local-axis naming.

## Stage 1: No-Object Linkage Baseline

Use the fixed-probe scene with probe moved at least 0.20 m from every finger geom. Reuse zero-noise approach and grasp motion. Command the measured calibrated open value `0.58905`, then execute 101 inclusive command levels to measured closed value `0.0`, with 20 MuJoCo steps per level.

Every frame records:

- command index, settle index, command, calibrated close fraction;
- finger joint position, actuator force, and actuator transmission identity;
- minimum opposing signed distance and selected geom names;
- nearest inner/outer surface points while distance is positive;
- closing axis and midpoint;
- midpoint translation from open baseline in world and closing-frame coordinates;
- closing-axis angular change from open baseline;
- inner and outer surface-point displacement projected onto open `e_close`;
- symmetry residual: signed sum of the two projected surface-point displacements;
- finger-finger contacts, if any.

Selected-pair identity and pair-switch count are explicit outputs because nearest-pair switching can create discontinuities. When minimum opposing distance is non-positive, nearest-point, closing-frame, and derived drift fields are null for that frame; finger-finger contact evidence is recorded instead. Trajectory extrema use valid positive-distance frames only and report their denominator.

This stage is descriptive and identifies motion structure. It does not itself pass or fail the grasp mechanism.

## Stage 2: Controlled Shape Ladder

Each condition uses a fresh context, the same zero-noise approach/grasp routine, calibrated-only 101-level closure, and 20 sampled steps per level. Object center is the measured open-pose `p_mid` in all three conditions. Object remains kinematically fixed so object motion cannot confound contact formation.

### Condition order

1. `sphere`: radius `0.021 m`; orientation irrelevant.
2. `aligned_cuboid`: half-sizes `(0.021, 0.010, 0.015) m` along `(e_close, e_tangent, e_vertical)`.
3. `world_cube`: half-sizes `(0.021, 0.021, 0.021) m`; identity world orientation.

Use experiment-owned scenes/assets only. Official OpenArm MJCF remains unchanged.

Every frame records Stage 1 kinematics plus object pose, object-relative surface points, named raw contact pairs, per-side normal force, signed finger-object gaps, contact position, contact normal, and normal alignment with `e_close`.

Primary gate remains unchanged: inner and outer named finger groups each have compressive normal force greater than `1e-6 N` for at least 20 consecutive frames. A condition passes at the first complete 20-step command level satisfying bilateral positive force on every frame. The runner may stop that condition after this level; otherwise it executes all 2020 frames.

Secondary contact-normal diagnostics do not alter the gate:

- opposing-normal dot product;
- absolute alignment of each contact normal with `e_close`;
- contact-point separation along `e_close`, `e_tangent`, and `e_vertical`.

Large fixed-overlap forces remain non-calibrated. They are logged but cannot support a grasp-force claim.

## Decision Rules

- Sphere, aligned cuboid, and world cube all fail: select linkage/stroke/contact-configuration diagnosis. Do not run free cube.
- Sphere passes but aligned cuboid fails: select face-orientation/mesh-normal diagnosis. Do not run free cube.
- Aligned cuboid passes: contact-surface mechanism gate passes; next task may run aligned free-cube closure using the existing 20-frame gate.
- Aligned cuboid passes and world cube fails: world-frame orientation/placement is the leading supported explanation.
- All three pass: prior centroid/world-pose construction was the supported failure source; proceed to free-cube closure.
- World cube passes while both sphere and aligned cuboid fail: mark `contradictory_shape_controls`, audit shape pose/collision construction, and draw no mechanism cause.
- Any instrument control fails: repair measurement code first; shape outcomes are invalid.

No condition, dimension, threshold, or order changes after observing results. Negative outcomes remain reportable.

## Architecture

Create `openarm_robust_grasp/surface_geometry.py` with focused units:

- `NearestSurfacePair`: geom IDs/names, signed distance, nearest points, and validity state;
- `ClosingFrame`: origin and orthonormal axes with validation;
- `LinkageFrame`: one no-object sampled frame;
- `ShapeFrame`: one object-condition sampled frame;
- `ShapeConditionResult`: frame sequence, contact statistics, gate result, and stop reason;
- `minimum_opposing_surface_pair(...)`;
- `closing_frame_from_surface_pair(...)`;
- `run_unloaded_linkage_baseline(...)`;
- `run_shape_ladder(...)`;
- `write_surface_geometry_report(...)`.

Create `run_surface_geometry_diagnosis.py` as thin orchestration. Reuse validated contact evidence, gripper calibration, model loading, state-preserving `mj_setConst` mutation, and existing motion helpers. Do not change previous canonical contact-diagnostic files.

Create experiment-owned sphere and cuboid fixed-probe MJCF assets, or one experiment-owned scene with explicitly named inactive probes if implementation tests prove collision-mask isolation. Do not mutate official assets.

## Outputs

Canonical directory: `outputs/contact_surface_geometry/`.

- `instrument_controls.csv`: distance/point consistency and order-swap controls;
- `linkage_kinematics.csv`: 2020 unloaded frames;
- `shape_ladder.csv`: per-frame rows for three ordered conditions, with early-stop reason where applicable;
- `summary.json`: protocol constants, instrument gate, kinematic extrema, shape outcomes, decision branch, and hashes;
- `midpoint_axis_drift.png`: midpoint displacement and axis-angle change versus close fraction;
- `shape_gap_force.png`: per-side gap and force versus close fraction by condition.

All tabular/summary bytes are built and validated before rollback-protected canonical replacement. Plots are generated from canonical tabular data and include units, condition labels, and gate annotation.

## Testing and Verification

Development follows test-first RED-GREEN-REFACTOR cycles in VM.

Required tests:

- nearest-point distance, point ordering, tie-breaking, and set-order invariance;
- penetration-state refusal for nearest-point interpretation;
- closing-frame orthonormality, handedness, axis-degeneracy rejection, and quaternion conversion;
- exact 101 x 20 unloaded sampling;
- state preservation around runtime object pose/shape changes;
- exact shape names, dimensions, order, center, and orientations;
- contact gate boundaries at 19/20 frames and force exactly/effectively above `1e-6 N`;
- first-passing-level early stop and full 2020-frame failure path;
- decision-rule truth table;
- report schema, input validation, rollback, and deterministic rerun;
- existing full suite remains green.

Canonical protocol runs twice in the same VM. Byte equality is required for CSV/JSON; plot hashes are reported separately. Final verification runs full pytest and Python compileall.

## Scientific Claim Boundary

Supported claims concern this OpenArm MuJoCo model's unloaded finger-surface kinematics, fixed-shape contact formation, and deterministic decision branch.

Unsupported claims include real-world force closure, calibrated contact force, statistical robustness, friction irrelevance, universal gripper impossibility, or direct sim-to-real transfer.

This is a mechanism experiment, not a statistical population study. No p-values or confidence intervals are appropriate. Continuous drift metrics, exact condition outcomes, protocol deviations, and all negative results are reported.

## Completion Criteria

Phase completes when:

1. nearest-point instrument controls pass;
2. unloaded linkage trajectory is recorded over all 2020 frames;
3. all reachable shape conditions execute in fixed order without post-hoc changes;
4. decision branch is selected from predeclared rules;
5. canonical outputs repeat deterministically;
6. full tests and compile checks pass;
7. next task follows only the selected branch.
