# OpenArm Simulation Grasp Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Characterize the OpenArm MuJoCo gripper and gate a simulation-only lift-hold-disturb parameter study on demonstrated geometric closure.

**Architecture:** Add a focused diagnostics module that derives actuator sweep values from the loaded model, measures named finger collision geometry, and writes structured results. After the mechanism gate, add a reusable stability protocol and a 36-condition exploratory screen without editing official OpenArm assets.

**Tech Stack:** Ubuntu 22.04 VM, Python 3.10, MuJoCo 3.10.0, NumPy, pytest.

## Global Constraints

- Use VM `nostalgia` for execution.
- Do not modify official OpenArm repositories or MJCF assets.
- Derive actuator commands from `model.actuator_ctrlrange`.
- Resolve joints, actuators, bodies, and geoms by name.
- Treat all results as simulation-only evidence.

---

### Task 1: Gripper Mechanism Characterization

**Files:**
- Create: `openarm_robust_grasp/grasp_diagnostics.py`
- Create: `tests/test_grasp_diagnostics.py`

**Interfaces:**
- Produces: `GripperSweepSample`
- Produces: `gripper_command_grid(ctx, arm, points=9) -> np.ndarray`
- Produces: `run_gripper_sweep(scene_path, arm_name="left", points=9, settle_steps=100) -> list[GripperSweepSample]`

- [ ] Write failing tests for exact model-range endpoints, nine monotonic commands, finite non-negative distances, and deterministic repeated sweeps.
- [ ] Run targeted tests in the VM and verify missing-interface failures.
- [ ] Implement named actuator/joint lookup and inner/outer collision-center measurement.
- [ ] Run targeted tests and the full suite.
- [ ] Run the real nine-point sweep and write CSV/JSON under `outputs/gripper_characterization/`.
- [ ] Check the geometric-closure gate and document whether the screen may proceed.

### Task 2: Lift-Hold-Disturb Protocol

**Files:**
- Modify: `openarm_robust_grasp/grasp_diagnostics.py`
- Modify: `tests/test_grasp_diagnostics.py`

**Interfaces:**
- Produces: `StabilityConfig`
- Produces: `StabilityResult`
- Produces: `run_stability_trial(scene_path, config) -> StabilityResult`

- [ ] Write failing tests for phase counts, metric bounds, deterministic output, and exact stable-success logic.
- [ ] Run tests and verify missing-interface failures.
- [ ] Implement runtime cube size/friction changes on the loaded experiment model only.
- [ ] Implement close, 8 cm lift, 500-step hold, 100-step horizontal disturbance, and 250-step final hold.
- [ ] Run targeted and full tests.

### Task 3: Exploratory Parameter Screen

**Files:**
- Create: `run_stability_screen.py`
- Modify: `README.md`
- Create: `docs/results/2026-08-13-simulation-grasp-stability.md`

- [ ] Run 36 deterministic combinations of four close commands, three cube widths, and three sliding-friction values.
- [ ] Verify row counts, parameter coverage, metric bounds, and reproducibility.
- [ ] Summarize stable-success regions and failure modes without inferential overclaiming.
- [ ] Request independent code and scientific review.

## Self-Review

- The mechanism gate prevents friction tuning when the gripper cannot geometrically close.
- The protocol has explicit phase durations and success thresholds.
- No hardware or official-asset mutation is required.
- No placeholders or ambiguous parameter values remain.
