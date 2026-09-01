# OpenArm Robust Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a minimal, reproducible MuJoCo experiment framework for OpenArm grasp robustness analysis.

**Architecture:** Create a self-contained experiment package under `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp`. Keep model loading, named indexing, policies, metrics, runner, and plotting separated. Use pure MuJoCo Python first; ROS2 integration is deferred until the benchmark produces stable results.

**Tech Stack:** Ubuntu 22.04 VM, Python 3.10, MuJoCo 3.10.0, NumPy, Matplotlib, pytest.

## Global Constraints

- Primary execution environment is VM `nostalgia`.
- First milestone does not require real hardware, YOLO, Dora, or a complete ROS2 bridge.
- Use named MuJoCo bodies, joints, cameras, and actuators instead of hard-coded indices.
- Store raw trial results as structured CSV/JSON and plots under `outputs/`.
- Use existing scripts only as references; do not overwrite them.

---

### Task 1: Package Skeleton and Named Model Access

**Files:**
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/model.py`
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/tests/test_model.py`

**Interfaces:**
- Produces: `load_scene(scene_path: str) -> SceneContext`
- Produces: `SceneContext` with `model`, `data`, `left`, `right`, `cameras`, `objects`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path

from openarm_robust_grasp.model import load_scene


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")


def test_load_scene_resolves_named_indices():
    ctx = load_scene(str(SCENE))

    assert ctx.model.nu == 16
    assert "tablecam" in ctx.cameras
    assert "bottle" in ctx.objects
    assert ctx.left.ee_body_name == "openarm_left_ee_base_link"
    assert len(ctx.left.joint_qpos_ids) == 7
    assert len(ctx.left.actuator_ids) == 7
    assert ctx.left.gripper_actuator_id >= 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp && python3 -m pytest tests/test_model.py -q`

Expected: FAIL because package does not exist.

- [ ] **Step 3: Implement named model access**

Create dataclasses for arm indices and scene context. Resolve all indices by MuJoCo names.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp && python3 -m pytest tests/test_model.py -q`

Expected: PASS.

### Task 2: Kinematics and Servo Motion Utilities

**Files:**
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/control.py`
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/tests/test_control.py`

**Interfaces:**
- Consumes: `SceneContext`
- Produces: `forward_kinematics(ctx, arm, q) -> np.ndarray`
- Produces: `solve_ik(ctx, arm, target_pos, seed_q=None) -> IKResult`
- Produces: `execute_joint_move(ctx, arm, q_goal, steps=80) -> MotionLog`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path

import numpy as np

from openarm_robust_grasp.control import forward_kinematics, solve_ik
from openarm_robust_grasp.model import load_scene


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")


def test_ik_reaches_current_end_effector_position():
    ctx = load_scene(str(SCENE))
    q0 = ctx.data.qpos[ctx.left.joint_qpos_ids].copy()
    target = forward_kinematics(ctx, ctx.left, q0)

    result = solve_ik(ctx, ctx.left, target, seed_q=q0)

    assert result.success
    assert result.error_m < 1e-3
    assert np.all(result.q >= ctx.left.lower_limits)
    assert np.all(result.q <= ctx.left.upper_limits)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_control.py -q`

Expected: FAIL because `control.py` does not exist.

- [ ] **Step 3: Implement minimal IK and servo helpers**

Use damped least squares and MuJoCo `mj_jacBody`.

- [ ] **Step 4: Run model and control tests**

Run: `python3 -m pytest tests -q`

Expected: PASS.

### Task 3: Trial Policies and Metrics

**Files:**
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/experiment.py`
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/tests/test_experiment.py`

**Interfaces:**
- Consumes: `SceneContext`, control helpers
- Produces: `NoiseConfig`
- Produces: `TrialResult`
- Produces: `run_trial(scene_path, policy, noise, seed) -> TrialResult`

- [ ] **Step 1: Write failing tests**

```python
from pathlib import Path

from openarm_robust_grasp.experiment import NoiseConfig, run_trial


SCENE = Path("/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/bottle_scene.xml")


def test_zero_noise_open_loop_trial_returns_metrics():
    result = run_trial(str(SCENE), policy="open_loop", noise=NoiseConfig(), seed=0)

    assert result.policy == "open_loop"
    assert result.seed == 0
    assert result.ik_success
    assert result.final_ee_error_m >= 0.0
    assert result.min_ee_object_distance_m >= 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_experiment.py -q`

Expected: FAIL because `experiment.py` does not exist.

- [ ] **Step 3: Implement deterministic trial runner**

Use object ground truth plus configurable pose noise. Implement `open_loop` and `pre_grasp_correction`.

- [ ] **Step 4: Run all tests**

Run: `python3 -m pytest tests -q`

Expected: PASS.

### Task 4: CLI Runner, Results, and Plots

**Files:**
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/run_experiments.py`
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/report.py`

**Interfaces:**
- Consumes: `run_trial`
- Produces: `outputs/results.csv`
- Produces: `outputs/summary.json`
- Produces: `outputs/success_vs_noise.png`
- Produces: `outputs/error_vs_noise.png`

- [ ] **Step 1: Add CLI script**

Run a small default grid: policies `open_loop`, `pre_grasp_correction`; translation noise `0, 0.005, 0.01, 0.02, 0.03`; 20 trials each.

- [ ] **Step 2: Run smoke experiment**

Run: `python3 run_experiments.py --trials 2 --out outputs/smoke`

Expected: CSV, JSON, and PNG files are created.

- [ ] **Step 3: Run full first grid**

Run: `python3 run_experiments.py --trials 20 --out outputs/latest`

Expected: full result files are created without exceptions.

### Task 5: README and Handoff

**Files:**
- Create: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/README.md`

**Interfaces:**
- Documents exact commands and outputs.

- [ ] **Step 1: Write concise README**

Include environment assumptions, test command, smoke command, full command, and result file descriptions.

- [ ] **Step 2: Verify final state**

Run: `python3 -m pytest tests -q`

Run: `python3 run_experiments.py --trials 2 --out outputs/final_smoke`

Expected: tests pass and smoke outputs are generated.

## Self-Review

- Spec coverage: first milestone is covered by Tasks 1-4. ROS2 integration is explicitly deferred.
- Placeholder scan: no TBD/TODO implementation placeholders remain.
- Type consistency: `SceneContext`, `NoiseConfig`, `TrialResult`, and `run_trial` are consistently named across tasks.

---

## Phase 2: Contact-Robustness Research Baselines

**Goal:** Turn the MuJoCo scaffold into a defensible contact-robustness experiment by comparing direct EE-base contact, finger-center contact, and pre-grasp finger-center correction under translation-noise perturbations.

**Research Hypothesis:** A second object-pose observation immediately before grasp should improve finger-center contact robustness under simulated visual localization noise, compared with a purely open-loop finger-center plan.

**Primary Metrics:**
- `contact_success_rate`: main metric for this phase.
- `mean_finger_object_contacts`: contact strength proxy, not a grasp-quality guarantee.
- `mean_object_motion_m`: separates gentle/centered contact from push-collision behavior.
- `success_rate` / `object_lift_m`: reported as negative/diagnostic evidence only until true lift is solved.

**Seed Control:** CLI experiment jobs use paired seeds: all policies share the same seed for each `(translation_noise_m, local_seed)` pair.

### Task 6: Explicit Contact Baselines

**Files:**
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/experiment.py`
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/run_experiments.py`
- Test: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/tests/test_experiment.py`
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/README.md`

**Interfaces:**
- Produces: policy `ee_base_contact_push`
- Produces: policy `finger_center_contact_push`
- Keeps: policy `contact_push` as backward-compatible alias for `finger_center_contact_push`

- [x] **Step 1: Write failing policy-name test**
- [x] **Step 2: Verify it fails with `Unknown policy`**
- [x] **Step 3: Implement explicit baseline policy names**
- [x] **Step 4: Verify all tests pass**
- [x] **Step 5: Run smoke comparison and document results**

### Task 7: Pre-Grasp Finger-Center Correction

**Files:**
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/openarm_robust_grasp/experiment.py`
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/run_experiments.py`
- Test: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/tests/test_experiment.py`
- Modify: `/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/README.md`

**Interfaces:**
- Produces: policy `pre_grasp_finger_center_correction`
- Behavior: plan initial approach from noisy perceived pose, then re-observe object pose at the pre-grasp stage and recompute finger-center grasp/lift waypoints.
- Constraint: use the same `NoiseConfig` model as the other policies so the comparison isolates policy structure, not different noise assumptions.

- [x] **Step 1: Write failing policy-name and determinism tests**
- [x] **Step 2: Verify tests fail because policy is unknown**
- [x] **Step 3: Implement the correction branch in `run_trial`**
- [x] **Step 4: Verify targeted tests pass**
- [x] **Step 5: Run all tests**
- [x] **Step 6: Run smoke comparison against `finger_center_contact_push`**
- [x] **Step 7: Document observed result and limitations**

### Phase 2 Self-Review Checklist

- [x] The correction policy must not silently use ground-truth pose without labeling it as a simulated second observation.
- [x] Contact success must not be described as true grasp success.
- [x] Any improvement claim must include the trial count and noise grid.
- [x] If the correction policy worsens results, report it as evidence and inspect the failure mode instead of hiding it.
- [x] True lift remains a separate physical-modeling task unless `object_lift_m > 0.02` appears in repeated trials.
