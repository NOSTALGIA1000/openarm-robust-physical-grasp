# OpenArm Contact-Surface Geometry Diagnosis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure unloaded finger-surface kinematics and execute a deterministic sphere/aligned-cuboid/world-cube fixed-shape ladder that localizes OpenArm bilateral-contact failure.

**Architecture:** Add isolated `surface_geometry.py` beside existing validated contact diagnostics. It owns nearest-point measurement, closing-frame construction, unloaded linkage sampling, shape runners, branch logic, and report generation. Thin CLI executes instrument gate first, then linkage baseline and fixed ordered shape ladder; official OpenArm assets and previous canonical outputs remain untouched.

**Tech Stack:** Python 3.10, MuJoCo Python bindings, NumPy, pytest, matplotlib, CSV/JSON, SSH-connected Ubuntu VirtualBox VM.

## Global Constraints

- Simulation only; no real hardware.
- Use calibrated command `0.58905 -> 0.0`, exactly 101 inclusive levels and 20 steps/level.
- Bilateral gate remains both named sides above `1e-6 N` for 20 consecutive frames.
- Shape order is exactly `sphere`, `aligned_cuboid`, `world_cube`.
- Sphere radius is `0.021 m`.
- Aligned cuboid half-sizes are `(0.021, 0.010, 0.015) m` along `(e_close, e_tangent, e_vertical)`.
- World cube half-sizes are `(0.021, 0.021, 0.021) m` with identity world orientation.
- Do not interpret penetration `mj_geomDistance` points.
- Do not change official OpenArm MJCF, prior canonical outputs, thresholds, conditions, or order after data inspection.
- Repository has no Git metadata. Record RED/GREEN and verification evidence in SDD ledger; do not initialize Git or fabricate commits.

---

## File Structure

- Create `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/surface_geometry.py`: records, nearest-point/frame math, linkage and shape runners, decisions, writer.
- Create `vm_sync/experiments/openarm_robust_grasp/tests/test_surface_geometry.py`: unit and real-scene integration tests.
- Create `vm_sync/experiments/openarm_robust_grasp/assets/contact_surface_sphere_scene.xml`: experiment-owned fixed sphere.
- Create `vm_sync/experiments/openarm_robust_grasp/assets/contact_surface_cuboid_scene.xml`: experiment-owned fixed box reused for aligned cuboid/world cube.
- Create `vm_sync/experiments/openarm_robust_grasp/run_surface_geometry_diagnosis.py`: CLI orchestration.
- Modify `vm_sync/experiments/openarm_robust_grasp/README.md`: protocol and claim boundary.
- Modify `.superpowers/sdd/2026-08-13-openarm-simulation-grasp-stability/progress.md`: Task 5 evidence.
- Create `docs/results/2026-09-01-openarm-contact-surface-geometry-diagnosis.md`: measured result.
- Create through CLI `vm_sync/experiments/openarm_robust_grasp/outputs/contact_surface_geometry/`.

---

### Task 1: Nearest-Surface Instrument and Closing Frame

**Files:**
- Create: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/surface_geometry.py`
- Create: `vm_sync/experiments/openarm_robust_grasp/tests/test_surface_geometry.py`

**Interfaces:**
- Consumes: MuJoCo context and validated inner/outer geom-ID sets.
- Produces: `NearestSurfacePair`, `ClosingFrame`, `minimum_opposing_surface_pair(...)`, `closing_frame_from_surface_pair(...)`.

- [x] **Step 1: Write failing nearest-pair tests**

```python
def test_minimum_pair_uses_distance_then_geom_id_tie_break(monkeypatch):
    ctx = fake_context({1: "inner1", 2: "inner2", 8: "outer8", 9: "outer9"})
    install_distances(monkeypatch, {(1, 8): 0.02, (1, 9): 0.01,
                                    (2, 8): 0.01, (2, 9): 0.03})
    pair = minimum_opposing_surface_pair(ctx, {2, 1}, {9, 8})
    assert (pair.inner_geom_id, pair.outer_geom_id) == (1, 9)
    assert pair.signed_distance_m == pytest.approx(0.01)
    assert np.linalg.norm(pair.outer_point - pair.inner_point) == pytest.approx(0.01)


def test_minimum_pair_refuses_penetration_points(monkeypatch):
    ctx = fake_context({1: "inner", 2: "outer"})
    install_distances(monkeypatch, {(1, 2): -0.001})
    pair = minimum_opposing_surface_pair(ctx, {1}, {2})
    assert pair.signed_distance_m == pytest.approx(-0.001)
    assert pair.inner_point is None
    assert pair.outer_point is None
```

Also test geom-order swap, set-order invariance, empty/overlapping sets, non-finite values, and exact positive segment-length consistency.

- [x] **Step 2: Run RED**

Run:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 -m pytest -q tests/test_surface_geometry.py
```

Expected: `ModuleNotFoundError: No module named 'openarm_robust_grasp.surface_geometry'`.

- [x] **Step 3: Implement minimal nearest-pair record and function**

```python
@dataclass(frozen=True)
class NearestSurfacePair:
    inner_geom_id: int
    inner_geom_name: str
    outer_geom_id: int
    outer_geom_name: str
    signed_distance_m: float
    inner_point: tuple[float, float, float] | None
    outer_point: tuple[float, float, float] | None


def minimum_opposing_surface_pair(ctx, inner_ids, outer_ids):
    inner, outer = _validated_side_ids((set(inner_ids), set(outer_ids)))
    candidates = []
    for inner_id in sorted(inner):
        for outer_id in sorted(outer):
            fromto = np.zeros(6, dtype=float)
            distance = float(mujoco.mj_geomDistance(
                ctx.model, ctx.data, inner_id, outer_id, 1.0, fromto
            ))
            if not np.isfinite(distance):
                raise ValueError("opposing surface distance must be finite")
            points = None if distance <= 0.0 else (tuple(fromto[:3]), tuple(fromto[3:]))
            if points is not None and not np.isclose(
                np.linalg.norm(fromto[3:] - fromto[:3]), distance, atol=1e-9, rtol=0.0
            ):
                raise ValueError("nearest-point segment length does not match distance")
            candidates.append((distance, inner_id, outer_id, points))
    distance, inner_id, outer_id, points = min(candidates, key=lambda row: row[:3])
    return NearestSurfacePair(
        inner_id, _geom_name(ctx, inner_id), outer_id, _geom_name(ctx, outer_id),
        distance, None if points is None else points[0], None if points is None else points[1]
    )
```

- [x] **Step 4: Verify nearest-pair GREEN**

Run focused tests; expected pass.

- [x] **Step 5: Write failing closing-frame tests**

```python
def test_closing_frame_is_right_handed_and_orthonormal():
    pair = surface_pair((0, 0, 0), (0, 0.04, 0))
    frame = closing_frame_from_surface_pair(pair)
    rotation = frame.rotation_matrix
    assert frame.origin == pytest.approx((0, 0.02, 0))
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-6)
    assert frame.e_close == pytest.approx((0, 1, 0))
```

Also reject null points, distance `<=1e-6`, closing axis within cosine `0.95` of world Z, non-finite points, and invalid handedness.

- [x] **Step 6: Run RED, implement `ClosingFrame`, verify GREEN**

Implementation uses projected world Z and columns `[e_close, e_tangent, e_vertical]`. Validate finite unit axes, pairwise dot magnitude `<=1e-9`, and determinant tolerance `1e-6`.

- [x] **Step 7: Run Task 1 full regression and record evidence**

Run `python3 -m pytest -q tests/test_surface_geometry.py` then `python3 -m pytest -q`. Record counts/timing in ledger.

---

### Task 2: Unloaded Linkage Baseline

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/surface_geometry.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_surface_geometry.py`

**Interfaces:**
- Consumes: Task 1 nearest pair/frame, existing calibration/motion helpers, fixed-probe scene.
- Produces: `LinkageConfig`, `LinkageFrame`, `LinkageResult`, `run_unloaded_linkage_baseline(...)`.

- [x] **Step 1: Write failing config/frame-math tests**

Assert exact defaults `101`, `20`, `0.58905`, `0.0`; immutable records; close fraction endpoints; midpoint world/local drift; axis-angle clipping; surface projections; symmetry residual; null-derived fields after non-positive gap.

```python
def test_linkage_config_has_exact_protocol():
    config = LinkageConfig()
    assert (config.command_levels, config.steps_per_level) == (101, 20)
    assert (config.open_command, config.closed_command) == pytest.approx((0.58905, 0.0))
```

- [x] **Step 2: Verify RED, implement records/math, verify GREEN**

`LinkageFrame` includes command indices, joint/actuator values, pair identity, optional points/frame/drift, and finger-finger contact count. `LinkageResult` validates exact frame count and valid-positive-frame denominator.

- [x] **Step 3: Write failing runner orchestration tests**

Use reduced `LinkageConfig(command_levels=2, steps_per_level=3)` and real scene. Assert probe final gaps to every finger at least `0.20 m`, IK success, six ordered frames, exact calibrated endpoints, state isolation, and deterministic duplicate rows.

- [x] **Step 4: Verify RED, implement runner, verify GREEN**

Fresh context; move probe using state-preserving helper; approach/grasp with zero noise; set open command; capture open frame; run `np.linspace(0.58905, 0.0, levels)` and sample after each `mj_step`. Pair switches increment only when selected `(inner_id, outer_id)` changes.

- [x] **Step 5: Run formal 2020-frame baseline twice**

Require exact duplicate serialized rows. Record maximum midpoint drift, closing-axis angular drift, symmetry residual, pair-switch count, and valid-point denominator without inventing pass threshold.

---

### Task 3: Fixed Shape Ladder

**Files:**
- Create: `vm_sync/experiments/openarm_robust_grasp/assets/contact_surface_sphere_scene.xml`
- Create: `vm_sync/experiments/openarm_robust_grasp/assets/contact_surface_cuboid_scene.xml`
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/surface_geometry.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_surface_geometry.py`

**Interfaces:**
- Consumes: Task 1 frame, Task 2 linkage sampling primitives, validated contact sampler.
- Produces: `SHAPE_CONDITIONS`, `ShapeFrame`, `ShapeConditionResult`, `run_shape_ladder(...)`.

- [x] **Step 1: Create experiment-owned assets**

Sphere scene contains fixed body `surface_probe`, geom `surface_probe_geom`, type sphere, size `0.021`. Cuboid scene contains same names, type box, initial half-sizes `0.021 0.010 0.015`. Both include official `/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/cell.xml` read-only and use current option/contact parameters.

- [x] **Step 2: Write failing condition identity/pose tests**

```python
def test_shape_conditions_are_exact_and_ordered():
    assert SHAPE_CONDITIONS == (
        ("sphere", (0.021,)),
        ("aligned_cuboid", (0.021, 0.010, 0.015)),
        ("world_cube", (0.021, 0.021, 0.021)),
    )
```

Test sphere center equals open `p_mid`; aligned body rotation columns equal closing frame; world cube quaternion is identity; runtime mutation preserves qpos/qvel/ctrl/act/time.

- [x] **Step 3: Verify RED, implement pose/configuration helpers, verify GREEN**

Use `model.body_pos`, `model.body_quat`, `model.geom_size`, state preservation around `mj_setConst`, and `mj_forward`. Reject geom type/name mismatch.

- [x] **Step 4: Write failing gate/early-stop tests**

Test 19/20 fails, 20/20 passes, force exactly `1e-6` fails, greater passes, first complete passing level stops at `command_index + 1` times 20 frames, and all-failure path has 2020 frames.

- [x] **Step 5: Verify RED, implement shape runner, verify GREEN**

Fresh context/condition; same zero-noise motion; calculate open closing frame before object placement; place object at frame origin; sample contact evidence plus nearest/contact-normal diagnostics. Stop only after a whole 20-frame command level is bilateral-positive on every frame.

- [x] **Step 6: Run formal ladder once, then exact duplicate**

Do not tune after result. Record each pass state, first passing command or full-failure count, force/gap extrema, contact-normal diagnostics, and exact decision inputs.

---

### Task 4: Decision Logic, Reports, Plots, and CLI

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/surface_geometry.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_surface_geometry.py`
- Create: `vm_sync/experiments/openarm_robust_grasp/run_surface_geometry_diagnosis.py`

**Interfaces:**
- Consumes: instrument controls, `LinkageResult`, three ordered `ShapeConditionResult` values.
- Produces: `select_shape_decision(...)`, `write_surface_geometry_report(...)`, CLI canonical artifacts.

- [x] **Step 1: Write failing decision truth-table tests**

```python
@pytest.mark.parametrize(("sphere", "aligned", "world", "expected"), [
    (False, False, False, "linkage_stroke_or_contact_configuration"),
    (True, False, False, "face_orientation_or_mesh_normal"),
    (True, False, True, "face_orientation_or_mesh_normal"),
    (False, True, False, "aligned_surface_mechanism_pass"),
    (False, True, True, "aligned_surface_mechanism_pass"),
    (True, True, False, "world_orientation_or_placement"),
    (True, True, True, "prior_centroid_world_pose_failure"),
    (False, False, True, "contradictory_shape_controls"),
])
def test_shape_decision_truth_table(sphere, aligned, world, expected):
    assert select_shape_decision(sphere, aligned, world) == expected
```

- [x] **Step 2: Verify RED, implement exact decision table, verify GREEN**

- [x] **Step 3: Write failing writer/rollback tests**

Assert exact four tabular/JSON names, row identity/order, null encoding, protocol constants, valid-point denominator, decision branch, hashes, and rejection before replacement for malformed/duplicate conditions. Inject `os.replace` failure at each index and verify all previous canonical bytes restore.

- [x] **Step 4: Implement writer and plots, verify GREEN**

Build CSV/JSON bytes first; rollback-protected replacement. Generate `midpoint_axis_drift.png` and `shape_gap_force.png` from validated in-memory canonical rows using noninteractive matplotlib backend, labeled SI units, shape legends, and gate annotation.

- [x] **Step 5: Write failing CLI orchestration test**

Assert order: instrument controls -> linkage baseline -> shape ladder -> decision -> writer exactly once. Instrument failure prevents both physical stages.

- [x] **Step 6: Implement CLI, verify full suite**

CLI defaults to approved asset paths and `outputs/contact_surface_geometry`. No flags expose thresholds, dimensions, condition order, or adaptive tuning.

---

### Task 5: Canonical Runs, Documentation, and Reviews

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/README.md`
- Modify: `.superpowers/sdd/2026-08-13-openarm-simulation-grasp-stability/progress.md`
- Create: `docs/results/2026-09-01-openarm-contact-surface-geometry-diagnosis.md`
- Create through CLI: `vm_sync/experiments/openarm_robust_grasp/outputs/contact_surface_geometry/*`

**Interfaces:**
- Consumes: tested CLI.
- Produces: canonical evidence, bounded interpretation, next selected task.

- [x] **Step 1: Run canonical protocol twice and hash artifacts**

```bash
python3 run_surface_geometry_diagnosis.py
sha256sum outputs/contact_surface_geometry/*
python3 run_surface_geometry_diagnosis.py
sha256sum outputs/contact_surface_geometry/*
```

CSV/JSON hashes must match. Plot hashes reported separately; mismatch triggers evidence comparison, not silent overwrite.

- [x] **Step 2: Write result note and README section**

Report instrument outcome, unloaded drift extrema/denominator/pair switches, three shape outcomes, exact decision branch, hashes, limitations, and only branch-authorized next task. Separate fixed-overlap force from realistic grasp force.

- [x] **Step 3: Perform code self-review**

Check point ownership, penetration nulls, pair tie-break, frame handedness, runtime state restoration, fresh contexts, exact conditions, early-stop semantics, writer rollback, and old-output preservation. Fix every finding through failing tests.

- [x] **Step 4: Perform scientific self-review**

Check manipulation validity, confounds, unsupported causal language, negative-result reporting, fixed-object force caveat, deterministic-study language, and no post-hoc threshold changes.

- [x] **Step 5: Run fresh final verification**

```bash
python3 -m pytest -q
python3 -m compileall -q openarm_robust_grasp run_surface_geometry_diagnosis.py
```

Record exact test count/timing and unchanged canonical hashes. No Git integration action because workspace lacks Git metadata.

---

## Plan Self-Review

- Spec coverage: instrument validation, closing frame, 2020-frame unloaded baseline, exact shape ladder, unchanged gate, decision table, outputs, plots, repeatability, and scientific boundaries map to Tasks 1-5.
- Placeholder scan: no TBD/TODO/implement-later instructions.
- Type consistency: Task 1 `NearestSurfacePair/ClosingFrame` feed Tasks 2-3; Task 2-3 results feed Task 4 writer/decision; Task 4 artifacts feed Task 5 documentation.
- Scope: one mechanism experiment; no mesh repair, free cube, friction sweep, ROS 2, or hardware work.
- Git exception: ledger evidence replaces commits because no repository metadata exists.
