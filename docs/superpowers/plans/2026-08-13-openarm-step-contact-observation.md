# OpenArm Per-Step Contact Observation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add trajectory-preserving per-step, per-stage, and opposing-finger contact metrics to the OpenArm MuJoCo benchmark.

**Architecture:** Extend the motion utility with an optional post-step observer. Keep finger/object contact classification and accumulation in the experiment layer, expose normalized trial metrics, and aggregate them in reports. Preserve all existing outputs and policy trajectories.

**Tech Stack:** Ubuntu 22.04 VM, Python 3.10, MuJoCo 3.10.0, NumPy, pytest.

## Global Constraints

- Primary execution environment is VM `nostalgia`.
- Do not modify official OpenArm repositories or assets.
- Resolve finger and object geometries by MuJoCo names, never hard-coded indices.
- Observer calls occur exactly once after each `mujoco.mj_step` in a motion.
- Existing random-number draws, commands, stepping order, and existing `TrialResult` fields remain compatible.
- New metrics are diagnostic contact-frame metrics, not force-closure or stable-grasp proof.

---

### Task 1: Motion Step Observer

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/control.py`
- Test: `vm_sync/experiments/openarm_robust_grasp/tests/test_control.py`

**Interfaces:**
- Produces: optional `step_observer: Callable[[SceneContext], None] | None` parameter on `execute_joint_move`.

- [x] Write a failing test using an observer counter and a five-step move; assert exactly five calls.
- [x] Run the targeted test in the VM and verify failure because the argument is unknown.
- [x] Add the optional observer and invoke it immediately after every `mujoco.mj_step`.
- [x] Run targeted and full control tests.

### Task 2: Finger-Side Contact Sampler and Accumulator

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/experiment.py`
- Test: `vm_sync/experiments/openarm_robust_grasp/tests/test_experiment.py`

**Interfaces:**
- Produces: `ContactFrame` with contact-pair count, opposing finger flags, any-contact flag, and two-finger flag.
- Produces: `StageContactStats` with sampled frames, contact frames, two-finger frames, and frame-rate properties.
- Produces: name-based `_finger_side_geom_ids(...)` and `_sample_contact_frame(...)` helpers.

- [x] Write failing tests for per-frame de-duplication, simultaneous opposing-finger classification, and zero-frame rates.
- [x] Run targeted tests and verify failures are caused by missing interfaces.
- [x] Implement minimal immutable frame data and mutable stage accumulator.
- [x] Resolve the two finger-side geom sets by names under the selected arm prefix; reject missing or overlapping groups.
- [x] Run targeted tests and full experiment tests.

### Task 3: Integrate Per-Step Stage Metrics

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/experiment.py`
- Test: `vm_sync/experiments/openarm_robust_grasp/tests/test_experiment.py`

**Interfaces:**
- Extends: `TrialResult` with sampled-frame, contact-frame, two-finger-frame, frame-rate, and two-finger/sustained-lift success fields from the approved design.

- [x] Write failing integration tests for deterministic sampled-frame totals, metric bounds, additivity constraints, and repeated-run determinism.
- [x] Run targeted tests and verify expected failure.
- [x] Attach stage-specific observers to approach, grasp, and lift moves.
- [x] Sample each of the 40 gripper-close steps in the grasp accumulator.
- [x] Compute IK-gated two-finger and 50%-lift diagnostic flags.
- [x] Run targeted tests and full suite.

### Task 4: Summary and Documentation

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/report.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/README.md`
- Test: `vm_sync/experiments/openarm_robust_grasp/tests/test_experiment.py`

**Interfaces:**
- Produces summary fields: `mean_approach_contact_frame_rate`, `mean_grasp_contact_frame_rate`, `mean_lift_contact_frame_rate`, `grasp_two_finger_contact_success_rate`, `lift_two_finger_contact_success_rate`, `sustained_lift_contact_success_rate`.

- [x] Write a failing summary-field test.
- [x] Run it and verify missing-key failure.
- [x] Add exact grouped aggregations.
- [x] Document sampling windows, normalization, and scientific limitations.
- [x] Run full tests.

### Task 5: VM Synchronization and 27-Trial Gate

**Files:**
- Create output: `outputs/per_step_contact_smoke_paired/`
- Update: `docs/results/2026-08-13-per-step-contact-smoke.md`

- [x] Sync modified files to the VM.
- [x] Run the full test suite.
- [x] Run three policies at noise `0`, `0.01`, `0.03 m`, three paired trials per condition.
- [x] Verify 27 rows, 9 summaries, 9 paired groups, metric bounds, frame-count relationships, and deterministic zero-noise repeats.
- [x] Compare existing fields against a matching pre-instrumentation 27-trial reference.
- [x] Copy outputs to Windows and document supported and unsupported conclusions.
- [x] Request independent review and fix any important findings.

## Self-Review

- Spec coverage: observer, contact semantics, stage windows, fields, summaries, compatibility, tests, and experiment gate are covered.
- Placeholder scan: no TBD/TODO or unspecified implementation steps remain.
- Type consistency: observer, `ContactFrame`, `StageContactStats`, and `TrialResult` extensions are consistent across tasks.
