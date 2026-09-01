# OpenArm Simulation-Only Grasp Stability Design

Date: 2026-08-13

## Protocol-v2 amendment — authoritative from 2026-08-22

Independent review found two construct-validity defects in the original protocol: elevation used a pre-settle object-height baseline, and floor-supported objects could receive unconditional disturbance-survival success. Protocol v2 supersedes those v1 semantics:

- elevation baseline is captured after open-settle;
- close-settle bilateral contact is sampled every step;
- disturbance evaluation is eligible only after at least `0.05 m` pre-disturb elevation and at least `0.5` pre-disturb bilateral-contact fraction;
- ineligible survival is `None`/NA; eligible survival uses the existing `0.02 m` vertical-drop threshold;
- stable success requires final elevation, both hold-contact thresholds, and disturbance survival exactly true.

The screen factor is fraction of calibrated closing stroke from measured open command `0.58905` toward measured closed command `0.0`, not fraction of the raw `[0.0, 0.7854]` actuator range. Original v1 wording below remains historical and must not be used to revert the implemented protocol.

## Objective

Determine why the current OpenArm MuJoCo gripper produces intermittent two-finger contact but almost never sustains an object through lift. Build a reproducible simulation-only diagnostic and stability protocol; no real-hardware validation is required for this phase.

## Model Facts

The left gripper actuator is `left_finger1_ctrl`. Its model control range is `[0.0, 0.7854]`, it drives `openarm_left_finger_joint1`, and its initial command is `0.0`. All command sweeps must derive values from the model control range rather than assume normalized commands.

## Approach

Use a two-stage study.

1. Gripper mechanism characterization: sweep nine equally spaced commands across the actuator control range, settle for 100 MuJoCo steps at each command, and record finger-joint position, inner/outer collision-center distance, minimum opposing-geom distance, and actuator force. This identifies which command means open/closed and whether the available stroke can geometrically clamp the cube.
2. Lift-hold-disturb protocol: use the existing finger-center approach and grasp waypoints, close to a chosen command, lift 8 cm, hold for 500 steps, apply a bounded horizontal body force for 100 steps, then hold for another 250 steps.

## Parameter Screen

The first screen uses the experiment-owned cube scene only:

- Close commands: 25%, 50%, 75%, and 100% of the calibrated closing stroke defined by the protocol-v2 amendment.
- Cube full widths: 36, 42, and 46 mm.
- Sliding friction coefficients: 1, 2, and 4; torsional/rolling terms remain `2` and `0.1` as in the current scene.
- Cube mass remains 15 g in the first screen.
- One deterministic zero-perception-noise trial per combination: 36 diagnostic trials.

This screen is exploratory and maps feasibility; it is not a statistical robustness comparison.

## Metrics

Each trial records:

- achieved lift height;
- fraction of lift frames with simultaneous opposing-finger contact;
- fraction of pre-disturb hold frames with simultaneous opposing-finger contact;
- fraction of post-disturb hold frames with simultaneous opposing-finger contact;
- maximum and final object displacement relative to the gripper;
- whether the object remains at least 5 cm above its settled post-open height at the end;
- whether the object retains simultaneous opposing-finger contact for at least 50% of both hold windows;
- for eligible trials only, whether the object survives the disturbance without falling more than 2 cm from its pre-disturb height; ineligible trials report NA.

`stable_grasp_success` requires all three conditions: final elevation at least 5 cm above settled baseline, at least 50% two-finger contact in both hold windows, and eligible disturbance survival exactly true. It is a simulation stability criterion, not proof of real-world force closure.

## Architecture

Create `openarm_robust_grasp/grasp_diagnostics.py` for actuator characterization, runtime experiment-owned cube parameter application, protocol metrics, and CSV/JSON output. Reuse named model access, IK, motion observers, and contact samplers. Do not modify official OpenArm MJCF files.

The initial implementation first completes and validates actuator characterization. The lift-hold-disturb screen proceeds only if the sweep shows a monotonic and physically meaningful reduction in opposing-finger distance. If it does not, the next task is to repair the experiment-owned gripper model rather than tune friction.

## Validation

- All new behavior is developed test-first in the VM.
- Actuator sweep commands exactly span the model range and are monotonic.
- Repeated characterization runs are deterministic.
- Finger-distance values are finite and non-negative.
- Runtime cube changes affect only the experiment-owned loaded model and do not write official assets.
- Protocol phase frame counts and metric bounds are checked automatically.
- Existing 40-test benchmark suite remains green.

## Claim Boundary

Supported outcomes may describe MuJoCo gripper stroke, simulated geometric feasibility, parameter sensitivity, and simulated lift/hold/disturb stability. They must not claim real-world grasp robustness, calibrated contact forces, or direct sim-to-real transfer.
