# OpenArm Per-Step Contact Observation Design

Date: 2026-08-13

## Objective

Upgrade the MuJoCo contact-robustness benchmark from endpoint-biased stage sampling to per-simulation-step contact observation. The upgrade must distinguish transient approach contact, grasp-close contact, lift contact, and simultaneous two-finger contact without changing policy trajectories.

## Scope

This phase covers instrumentation, metrics, reports, tests, a small regression grid, and documentation. It does not change IK, waypoints, gripper geometry, object parameters, perception noise, ROS2 integration, or the definition of a physically successful grasp.

## Architecture

`execute_joint_move` will accept an optional step observer. After each `mujoco.mj_step`, it will call the observer with the current `SceneContext`. Existing callers that omit the observer retain identical behavior and return values.

`experiment.py` will own contact semantics. It will construct an observer for each motion stage and use the same contact sampler during the 40 gripper-close steps. This keeps low-level motion control independent from object-specific metrics while ensuring every simulation step is sampled once.

The observer will accumulate a stage record rather than raw full trajectories. This avoids large per-trial files while preserving the metrics needed for analysis.

## Contact Semantics

A contact frame is one simulation step in which at least one collision pair joins a finger collision geometry and an object geometry.

For each sampled step, record:

- `contact_pairs`: number of active finger-object geom contact pairs.
- `left_finger_contact`: whether a left finger collision geometry contacts the object.
- `right_finger_contact`: whether a right finger collision geometry contacts the object.
- `any_finger_contact`: logical OR of the two finger flags.
- `two_finger_contact`: logical AND of the two finger flags.

Here, left/right finger means the two opposing finger sides of the selected OpenArm gripper, not the left and right robot arms. Geometries must be resolved by MuJoCo names and grouped explicitly; no hard-coded geom indices are allowed.

## Stage Windows

- `approach`: every step of the 80-step approach motion.
- `grasp`: every step of the 80-step grasp motion plus all 40 gripper-close steps.
- `lift`: every step of the 80-step lift motion for the three formal contact policies. The backward-compatible `cube_lift` pose-control probe retains its existing 120-step lift trajectory and reports all 120 frames.

Each simulation step belongs to exactly one stage. Endpoint samples must not be added a second time.

## New Trial Metrics

Add the following fields to `TrialResult` and `results.csv`:

- `approach_sampled_frames`
- `grasp_sampled_frames`
- `lift_sampled_frames`
- `approach_contact_frames`
- `grasp_contact_frames`
- `lift_contact_frames`
- `approach_two_finger_contact_frames`
- `grasp_two_finger_contact_frames`
- `lift_two_finger_contact_frames`
- `approach_contact_frame_rate`
- `grasp_contact_frame_rate`
- `lift_contact_frame_rate`
- `grasp_two_finger_contact_success`
- `lift_two_finger_contact_success`
- `sustained_lift_contact_success`

Frame rates equal contact frames divided by sampled frames and are `0.0` when no frames were sampled.

`grasp_two_finger_contact_success` is true when at least one grasp-stage frame has simultaneous opposing-finger contact and IK remains successful.

`lift_two_finger_contact_success` is true when at least one lift-stage frame has simultaneous opposing-finger contact and IK remains successful.

`sustained_lift_contact_success` is true when opposing-finger contact occurs in at least 50% of sampled lift frames and IK remains successful. This is a diagnostic threshold, not proof of stable grasping.

## Compatibility

Existing `TrialResult` fields remain present. Their current endpoint/count definitions are preserved for backward compatibility in this phase. New scientific conclusions must use the per-step frame metrics.

With observation enabled, joint commands, MuJoCo stepping order, random-number draws, IK results, object motion, and existing non-stage metrics must remain unchanged relative to the current implementation for identical scene, policy, noise, and seed.

The CLI remains backward compatible and writes the new columns automatically.

## Summary Metrics

Add grouped summary fields:

- `mean_approach_contact_frame_rate`
- `mean_grasp_contact_frame_rate`
- `mean_lift_contact_frame_rate`
- `grasp_two_finger_contact_success_rate`
- `lift_two_finger_contact_success_rate`
- `sustained_lift_contact_success_rate`

Raw contact-pair counts may remain available for compatibility but must not be described as force or contact strength.

## Testing

Testing follows red-green-refactor.

Unit tests must prove:

- An observer is called exactly once after every step of `execute_joint_move`.
- Omitting an observer preserves existing motion behavior.
- Finger-side classification is name-based and rejects an unresolved or ambiguous finger grouping with a clear error.
- Contact frames count once even when multiple contact pairs exist in the same step.
- Two-finger frames require both opposing finger sides in the same simulation step.
- Frame rates handle zero sampled frames.
- Summary fields equal trial-level aggregates.

Integration tests must prove:

- A deterministic trial returns the expected sampled-frame totals for all completed stages.
- The same trial run twice produces identical new metrics.
- Existing non-stage fields remain unchanged before and after instrumentation for fixed inputs.

## Experiment Gate

First run a smoke grid with the three formal policies, noise levels `0`, `0.01`, and `0.03 m`, and 3 paired trials per condition: 27 trials total.

Accept the instrumentation when:

- All tests pass.
- Every completed motion stage has the expected number of sampled frames.
- Every contact-frame count is between zero and its sampled-frame count.
- Every two-finger frame count is no greater than the corresponding any-contact frame count.
- All frame rates lie in `[0, 1]`.
- Paired seeds remain complete.
- Existing trajectory-dependent fields match a pre-instrumentation reference run.

Only after this gate passes should the full 630-trial grid be rerun.

## Scientific Interpretation

The new metrics can support claims about when contacts are observed and whether opposing fingers contact simultaneously. They cannot alone prove force closure, grasp stability, successful visual perception, sim-to-real transfer, or a solved physical lift.

The next physical-grasp milestone should require sustained two-finger lift contact together with object elevation and final hold, rather than relying on cumulative contact plus a transient 2 cm rise.
