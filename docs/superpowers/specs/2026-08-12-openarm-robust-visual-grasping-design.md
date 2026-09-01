# OpenArm Robust Visual Grasping Research Design

Date: 2026-08-12

## Target

Recommended research target:

`Robustness Analysis and Closed-Loop Correction for OpenArm Visual Grasping with ROS2 and MuJoCo`

The first version should be simulation-led, with limited real-robot validation only after the MuJoCo experiment loop is stable. The core question is:

> In OpenArm visual grasping tasks, how do visual localization error, hand-eye calibration error, and low-level control disturbance affect task success, and can a simple closed-loop correction step improve robustness over an open-loop baseline?

## Current Environment Facts

The working VM is `nostalgia`.

- OS: Ubuntu 22.04.5 LTS
- ROS2: Humble is installed under `/opt/ros/humble`
- Workspace: `/home/nostalgia/openarm_ws`
- OpenArm ROS2 packages are built and discoverable after sourcing ROS:
  - `openarm_description`
  - `openarm_bringup`
  - `openarm_bimanual_moveit_config`
- MuJoCo Python package is installed: `mujoco==3.10.0`
- Official repos present:
  - `/home/nostalgia/openarm_ws/src/openarm_mujoco`
  - `/home/nostalgia/openarm_ws/src/openarm_ros2`
  - `/home/nostalgia/openarm_ws/src/openarm_description`
- `openarm_mujoco/v2/demo.xml` loads successfully in MuJoCo.
- The `openarm-mujoco-launch` CLI is not currently installed, but direct Python loading of MJCF files works.
- Dora is not currently installed in the default Python environment.

## Existing Prototype Assets

The VM already contains useful prototype scripts:

- `fk_ik_final.py`: MuJoCo FK/IK with damped least squares.
- `stage3_pid.py`: position-servo control comparisons.
- `stage3_trajectory_qdd.py`: trajectory tracking and preliminary QDD/backdrivability exploration.
- `stage4_operational.py`: operational-space ideas, pick-and-place state machine, and trajectory recording.
- `stage5_bc.py`: behavior cloning from IK-generated demonstrations.
- `catch_controller.py`: contact-rich catch task prototype.

Existing results suggest:

- FK/IK is usable for reachable-point generation.
- Pick-and-place has a strong baseline result in the simple scripted setting.
- Resolved-rate and null-space control need improvement before being treated as research evidence.
- Behavior cloning is currently a learning demo, not yet competitive with IK.
- The catch task uses artificial guidance force, so it should not be used as the first rigorous research benchmark.

## Recommended First Research Loop

Use `openarm_mujoco/v2/bottle_scene.xml` or a simplified derivative as the first benchmark.

Why:

- It already has both arms, table object bodies, and cameras.
- It is simpler than peg insertion or catching.
- It is closer to visual grasping than pure kinematic trajectory following.

Initial task:

1. Use one arm only.
2. Use MuJoCo ground-truth object pose as the ideal perception signal.
3. Add controlled pose noise to simulate visual detection/calibration error.
4. Plan grasp using IK and the existing servo execution style.
5. Measure success, final object pose, end-effector error, task time, and failure mode.
6. Add one closed-loop correction step before grasp closure.
7. Compare open-loop vs correction across noise levels.

## Experimental Variables

Primary independent variables:

- Object pose noise: translation noise in millimeters and yaw noise in degrees.
- Camera/extrinsic-like error: fixed transform bias applied to perceived object pose.
- Control disturbance: joint command noise or latency-like update delay.

Primary metrics:

- Grasp success rate.
- Final object displacement from target.
- Minimum end-effector-to-object distance before grasp closure.
- Final end-effector error.
- Number of failed IK/planning attempts.
- Time or simulation steps per trial.

Recommended first noise grid:

- Translation noise: 0 mm, 5 mm, 10 mm, 20 mm, 30 mm.
- Rotation noise: 0 deg, 2 deg, 5 deg, 10 deg.
- Trials per setting: start with 20, later scale to 100.

## Baselines

Baseline A: open-loop grasp.

- Read object pose once.
- Generate approach, descend, close, lift sequence.
- No perception update before closing gripper.

Baseline B: pre-grasp correction.

- Read object pose at approach pose.
- Recompute descend/grasp waypoint.
- Then close and lift.

Optional later baseline C: visual servo loop.

- During approach, repeatedly update end-effector target from the observed object pose.
- Stop when distance threshold is met or timeout occurs.

## ROS2 Role

Do not make ROS2 the first bottleneck.

Stage 1 should run as a pure MuJoCo Python experiment because the physics loop and data logging are already available.

Stage 2 should wrap the same actions and observations behind ROS2-like interfaces:

- Joint command publisher/consumer shape matching `JointTrajectoryController`.
- Joint state and camera observation messages.
- Later, a real ROS2 node can reuse this contract.

Stage 3 can connect to real OpenArm or a ROS2 mock hardware launch:

- `openarm_bringup` supports fake hardware launch arguments.
- `openarm_bimanual_moveit_config` has MoveIt2 launch/config assets.
- Current local repo does not show a complete Dora or ROS2-MuJoCo bridge installed, so that should be treated as future integration rather than the first milestone.

## Risks

Major risks:

- MuJoCo gripper-object contact may not match real OpenArm behavior.
- Existing scripts use some hard-coded joint indices and body names.
- Current BC result is much weaker than IK and should not be framed as a core contribution yet.
- ROS2-MuJoCo integration may require additional bridge work.
- Real-robot access is limited to weekends, so real-world validation must be narrow.

Mitigations:

- Keep the first benchmark simple and measurable.
- Use model names instead of hard-coded indices in new code.
- Separate experiment logic from plotting and demos.
- Treat real-robot runs as qualitative or small quantitative validation.
- Maintain reproducible trial configs and random seeds.

## Definition of Done for First Milestone

The first milestone is complete when the VM can run one command that:

1. Loads an OpenArm MuJoCo scene.
2. Runs at least 20 trials per noise setting.
3. Compares open-loop and pre-grasp correction.
4. Saves raw results as structured data.
5. Generates plots for success rate and error vs noise.
6. Writes a concise summary table.

This milestone does not require real hardware, YOLO, Dora, or a complete ROS2 bridge.

## Next Implementation Plan

The next implementation should create a small experiment package in the VM workspace:

- `experiments/openarm_robust_grasp/`
- reusable scene/model utilities
- named-index resolver for joints, actuators, bodies, cameras
- open-loop and correction policies
- trial runner with seeded perturbations
- metrics logger
- plotting/report script

Existing scripts should be treated as references, not edited in place unless necessary.
