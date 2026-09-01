# OpenArm ROS2 Physical Grasp Design

Date: 2026-09-01

## Goal

Convert the reproduced `openarm_mujoco_ros2` course workflow from software-attached object motion to measurable MuJoCo contact-dynamics grasping while preserving the original demonstration as a control baseline.

The first milestone covers one left-arm pick and lift. It does not cover dual-arm handover, placement, learned perception, real-hardware execution, or full torque-level arm control.

## Existing Baseline

The isolated course reproduction is located at:

`/home/nostalgia/ros2_ws_course_repro/ros2_ws`

It runs in VM `nostalgia` with ROS2 Humble, MuJoCo 3.10.0, and `ROS_DOMAIN_ID=71`. The existing `/home/nostalgia/openarm_ws` workspace remains untouched.

Current workflow:

`MuJoCo RGB-D -> depth detector -> TF -> MoveIt -> GripperCommand -> software object attachment -> handover -> placement`

Current non-physical mechanisms:

- `pick_apple.py` sets `is_attached_left` or `is_attached_right` after gripper closure.
- `magic_loop()` publishes object coordinates at 500 Hz.
- `teleport_cube()` commands `cube_x`, `cube_y`, and `cube_z` slide joints.
- The active object in `table_scene.xml` is constrained by those three slide joints and position actuators.
- Non-PID position commands in `mujoco_system.cpp` directly write `mj_data_->qpos`.
- No contact, hold, lift, or relative-slip evidence gates task progression.

## Chosen Approach

Use staged physicalization.

Stage 1 physicalizes the object and sliding gripper while retaining the current MoveIt arm-position path. This isolates grasp geometry and contact dynamics from arm-controller redesign. Stage 2 adds visual uncertainty and pre-grasp correction only after Stage 1 passes. Stage 3 may replace direct arm position writes with actuator-level control.

Rejected alternatives:

- Removing only object teleportation leaves direct gripper `qpos` writes and cannot support a credible physical-grasp claim.
- Replacing all arm and gripper control simultaneously introduces too many coupled failure sources for the first milestone.

## Mode Isolation

Two explicit runtime modes remain available:

- `demo_mode`: unchanged course behavior, including software attachment and full collaborative sequence.
- `physics_mode`: free object, no object controller, no attachment loop, dynamic gripper closure, left-arm pick/lift only.

The modes use separate launch and model files. Physics work must not silently alter the baseline launch path. Generated build and install directories remain separate from the original ZIP contents.

## Physics Model

### Object

`physics_mode` uses one free rigid body with a MuJoCo `freejoint`. No `cube_x`, `cube_y`, or `cube_z` joint, actuator, ROS2 control interface, controller, SRDF entry, or command publisher is allowed in this mode.

Initial object shape is the existing red sphere with radius `0.025 m`. Initial mass is `0.1 kg`. Existing friction and solver parameters are retained for the first untuned run so baseline failure remains interpretable. Parameter changes require recorded experiment configurations rather than hidden edits.

### Sliding Gripper

Both fingers retain opposite physical slide axes and nonnegative joint ranges. Their coupling represents equal joint coordinates, `q_finger2 = q_finger1`; no negative multiplier is used.

Only gripper joints switch from direct position teleportation to bounded force-producing control in Stage 1. Arm joints retain current MoveIt command behavior. Gripper limits are enforced in MuJoCo and ROS2 Control. Closure commands wait for action completion or timeout rather than fixed `sleep()` alone.

First implementation preference:

1. Reuse the plugin's existing per-joint PID path for gripper position commands.
2. Apply bounded effort and velocity limits.
3. Keep arm joints on the existing path.

If the installed ROS2 Control metadata cannot enable PID per gripper joint without affecting arm joints, use a narrow gripper-only actuator mapping. Do not replace the whole bridge in Stage 1.

## Contact Evidence

`physics_mode` publishes a simulator-ground-truth grasp state at a fixed rate no higher than the physics rate. The state contains:

- simulation timestamp;
- left-finger/object contact boolean and normal force;
- right-finger/object contact boolean and normal force;
- bilateral-contact boolean;
- object world position and orientation;
- gripper reference position and orientation;
- object pose relative to gripper;
- object height above settled baseline;
- relative translation drift from first accepted grasp.

Contact classification uses explicit MuJoCo geom names for the two left fingers and object. It does not count palm contact, arm contact, table contact, or contacts occurring on only one finger.

Normal force is computed from MuJoCo contact constraints for matching finger-object pairs. Raw samples remain available for experiment logging.

## Grasp State Machine

`physics_mode` runs this bounded sequence:

1. Wait for MoveIt, gripper action, grasp-state stream, and one target observation.
2. Move left arm to hover pose.
3. Open gripper and verify aperture reaches open tolerance.
4. Remove planning-only protection geometry.
5. Descend to predeclared grasp pose.
6. Close gripper under bounded effort.
7. Accept grasp only after bilateral contact persists for the configured confirmation window.
8. Abort and retreat if contact confirmation times out.
9. Lift `20 mm` while continuing contact and relative-pose monitoring.
10. Hold for `2.0 s` of simulation time.
11. Classify success or failure and publish one result.
12. Retreat safely; no handover or placement occurs in Stage 1.

State transitions use simulation timestamps. Wall-clock sleeps are not evidence windows because current VM real-time factor is below 1.0.

## Failure Classification

Every run ends in exactly one category:

- `planning_failure`: hover, descend, lift, or retreat planning/execution failed.
- `gripper_command_failure`: gripper action rejected, aborted, or timed out.
- `no_contact`: neither finger established object contact.
- `unilateral_contact`: contact occurred but never became bilateral during confirmation.
- `contact_lost_during_lift`: accepted bilateral contact disappeared below threshold during lift.
- `insufficient_lift`: object elevation remained below `0.020 m`.
- `excessive_slip`: relative object/gripper translation drift exceeded `0.010 m`.
- `hold_failure`: lift passed but contact or height failed during hold.
- `success`: every acceptance condition passed.

Diagnostic details may contain several observed symptoms, but canonical result contains one category selected in sequence order.

## Acceptance Criteria

A successful trial requires all conditions:

- no software attachment or object-position command path active;
- object maximum elevation at least `0.020 m` above settled baseline;
- `2.0 s` simulation-time hold completed;
- bilateral-contact frame ratio during hold at least `0.80`;
- relative object/gripper translation drift no more than `0.010 m`;
- no planning, command, or monitoring timeout.

First milestone passes when a fixed deterministic condition produces at least 8 successes in 10 trials. These 10 trials validate repeatability only; they are not a population-level probability estimate.

## Data Products

Each trial records:

- configuration snapshot and random seed;
- mode and source revision identifiers where available;
- planning and action outcomes;
- per-sample contact booleans and forces;
- object and gripper poses;
- elevation and relative drift;
- stage timestamps;
- canonical failure category;
- final success boolean.

One run writes CSV samples, JSON trial summary, and concise Markdown report. Canonical outputs live under an experiment-owned result directory, not package source directories.

## Testing

### Static tests

- `physics_mode` model contains one object `freejoint` and no object position actuators.
- `physics_mode` launch contains no cube controller.
- `physics_mode` picker contains no object command publisher or attachment loop.
- finger coupling has equal-coordinate semantics.
- gripper joints select dynamic control while arm-joint behavior remains unchanged.

### Unit tests

- named contact pairs classify left, right, bilateral, and unrelated contacts correctly.
- force aggregation handles multiple matching contacts in one step.
- state machine accepts only sustained bilateral contact.
- timeout and failure precedence produce one deterministic category.
- success evaluator enforces every height, hold, contact-ratio, and slip threshold boundary.

### Integration tests

- launch graph becomes ready on isolated `ROS_DOMAIN_ID=71`.
- object remains on table without commands before grasp.
- closing without correct alignment does not move object with end effector.
- aligned close/lift records contact and free-body motion.
- software attachment topics and controllers are absent in `physics_mode`.

### Regression tests

- original `demo_mode` launch remains available.
- existing experiment package tests remain green.
- physics changes do not modify `/home/nostalgia/openarm_ws`.

## Performance

Physics timestep remains `0.002 s`. Physics semantics must not be changed to improve apparent speed.

Visual rendering is decimated independently from physics and sensor updates. Validation can run headless; visible inspection uses reduced render rate. Contact evidence always uses simulation steps or an explicitly declared sampling divisor.

## Stage Boundaries

Stage 1 ends at deterministic physical-grasp acceptance. It may include geometry or contact-parameter corrections only when diagnostics identify a cause and each change is recorded.

Stage 2 adds controlled target-pose and hand-eye calibration error, then compares open-loop grasp against one pre-grasp visual correction.

Stage 3 evaluates actuator-level arm control, command latency, sensor noise, and limited real-robot transfer.

Dual-arm handover starts only after single-arm Stage 1 passes. Learned detectors and policies remain outside this design.

## Safety and Preservation

- Never edit `/home/nostalgia/openarm_ws` for this milestone.
- Preserve ZIP reproduction and original launch behavior.
- Work only in experiment-owned copies or new physics-mode files.
- Keep `ROS_DOMAIN_ID=71` during validation.
- Do not start, stop, or control the VM desktop; use SSH only.
- Real hardware remains out of scope.
