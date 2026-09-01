# OpenArm Contact-Formation Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a validated MuJoCo contact-measurement pipeline and execute staged fixed-cube, free-cube, and conditional 20 mm lift gates that localize the current OpenArm grasp failure.

**Architecture:** Add a focused `contact_diagnostics.py` module beside the existing stability code. It owns immutable diagnostic records, raw contact/force/gap sampling, stage runners, gates, and report validation. A dedicated experiment-owned fixed-cube MJCF supplies known MuJoCo controls; a thin CLI executes only reachable stages and records explicit skipped-stage reasons.

**Tech Stack:** Python 3.10, MuJoCo Python bindings, NumPy, pytest, CSV/JSON, existing OpenArm model/IK/control helpers, Windows host sync plus SSH to Ubuntu VirtualBox VM.

## Global Constraints

- Simulation only; no real hardware.
- Do not modify official OpenArm repositories or official MJCF assets.
- Existing stability protocol and canonical stability-screen outputs remain unchanged.
- Use `inner` and `outer` collision-group terminology; do not report current compatibility fields as physical left/right fingers.
- Positive contact requires finite summed compressive normal force greater than `1e-6 N` on the relevant side.
- Fixed and free closure use exactly 101 inclusive command levels and 20 sampled MuJoCo steps per level.
- Free-cube offsets are exactly centered, closing-axis `-0.002/+0.002 m`, and world-Z `-0.002/+0.002 m`.
- Conditional lift command is exactly `0.020 m` over 200 steps and runs only after free-closure pass.
- No post-hoc threshold changes after integration data inspection.
- Repository has no Git metadata. Do not create Git metadata or fabricate commit steps; record file/test evidence in SDD ledger instead.

---

## File Structure

- Create `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`: diagnostic records, sampler, controls, closure/lift runners, gates, writer.
- Create `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`: unit and real-scene integration tests for this phase.
- Create `vm_sync/experiments/openarm_robust_grasp/assets/cube_contact_fixed_scene.xml`: experiment-owned fixed object scene.
- Create `vm_sync/experiments/openarm_robust_grasp/run_contact_diagnostics.py`: CLI and stage orchestration.
- Modify `vm_sync/experiments/openarm_robust_grasp/README.md`: command, protocol, fields, decision gates, claim limits.
- Modify `.superpowers/sdd/2026-08-13-openarm-simulation-grasp-stability/progress.md`: Task 4 RED/GREEN/integration evidence and selected failure branch.
- Create `docs/results/2026-09-01-openarm-contact-formation-diagnostics.md`: measured outcomes and bounded interpretation.
- Create canonical outputs under `vm_sync/experiments/openarm_robust_grasp/outputs/contact_formation_diagnostics/` only through runner.

---

### Task 1: Contact Evidence and Sequence Gates

**Files:**
- Create: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`
- Create: `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`

**Interfaces:**
- Consumes: MuJoCo context with `.model` and `.data`; `(set[int], set[int])` inner/outer geom IDs; `set[int]` object geom IDs.
- Produces: `ContactPairEvidence`, `ContactEvidence`, `ContactSequenceStats`, `sample_contact_evidence(...)`, `contact_class(...)`, and `closure_gate_passes(...)`.

- [ ] **Step 1: Write failing immutable-record and classifier tests**

Add imports and exact behavior tests:

```python
from dataclasses import FrozenInstanceError
import numpy as np
import pytest

from openarm_robust_grasp.contact_diagnostics import (
    ContactEvidence,
    ContactPairEvidence,
    ContactSequenceStats,
    closure_gate_passes,
    contact_class,
)


def _evidence(inner_force=0.0, outer_force=0.0, *, finite=True):
    value = float(inner_force) if finite else float("nan")
    return ContactEvidence(
        inner_contact=inner_force > 0.0,
        outer_contact=outer_force > 0.0,
        raw_pairs=(),
        inner_normal_force_n=value,
        outer_normal_force_n=float(outer_force),
        inner_signed_gap_m=0.0,
        outer_signed_gap_m=0.0,
    )


@pytest.mark.parametrize(
    ("inner", "outer", "expected"),
    [(0.0, 0.0, "negative"), (1.0, 0.0, "inner_only"),
     (0.0, 1.0, "outer_only"), (1.0, 1.0, "bilateral")],
)
def test_contact_class_uses_inner_outer_positive_force_semantics(inner, outer, expected):
    assert contact_class(_evidence(inner, outer)) == expected


def test_contact_evidence_is_frozen_and_rejects_nonfinite_values():
    sample = _evidence(1.0, 1.0)
    with pytest.raises(FrozenInstanceError):
        sample.inner_contact = False
    with pytest.raises(ValueError, match="finite"):
        _evidence(1.0, 1.0, finite=False)


def test_closure_gate_requires_twenty_consecutive_bilateral_positive_force_frames():
    nineteen = ContactSequenceStats.from_frames([_evidence(1.0, 1.0)] * 19)
    twenty = ContactSequenceStats.from_frames([_evidence(1.0, 1.0)] * 20)
    assert not closure_gate_passes(nineteen)
    assert closure_gate_passes(twenty)
```

- [ ] **Step 2: Run focused tests and verify RED**

Run in VM:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
pytest -q tests/test_contact_diagnostics.py
```

Expected: collection fails with `ModuleNotFoundError: No module named 'openarm_robust_grasp.contact_diagnostics'`.

- [ ] **Step 3: Implement minimal records and pure gates**

Create module with these exact public definitions:

```python
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable

POSITIVE_FORCE_N = 1e-6
REQUIRED_CONSECUTIVE_FRAMES = 20


@dataclass(frozen=True)
class ContactPairEvidence:
    object_geom_id: int
    object_geom_name: str
    finger_geom_id: int
    finger_geom_name: str
    side: str
    normal_force_n: float

    def __post_init__(self) -> None:
        if self.side not in {"inner", "outer"}:
            raise ValueError("side must be inner or outer")
        if not isfinite(self.normal_force_n) or self.normal_force_n < 0.0:
            raise ValueError("normal_force_n must be finite and non-negative")


@dataclass(frozen=True)
class ContactEvidence:
    inner_contact: bool
    outer_contact: bool
    raw_pairs: tuple[ContactPairEvidence, ...]
    inner_normal_force_n: float
    outer_normal_force_n: float
    inner_signed_gap_m: float
    outer_signed_gap_m: float

    def __post_init__(self) -> None:
        values = (self.inner_normal_force_n, self.outer_normal_force_n,
                  self.inner_signed_gap_m, self.outer_signed_gap_m)
        if not all(isfinite(value) for value in values):
            raise ValueError("contact evidence values must be finite")
        if self.inner_normal_force_n < 0.0 or self.outer_normal_force_n < 0.0:
            raise ValueError("normal force sums must be non-negative")

    @property
    def bilateral_positive_force(self) -> bool:
        return bool(
            self.inner_contact and self.outer_contact
            and self.inner_normal_force_n > POSITIVE_FORCE_N
            and self.outer_normal_force_n > POSITIVE_FORCE_N
        )


@dataclass(frozen=True)
class ContactSequenceStats:
    sampled_frames: int
    inner_contact_frames: int
    outer_contact_frames: int
    bilateral_frames: int
    max_consecutive_bilateral_positive_force_frames: int
    max_inner_normal_force_n: float
    max_outer_normal_force_n: float

    @classmethod
    def from_frames(cls, frames: Iterable[ContactEvidence]) -> "ContactSequenceStats":
        items = tuple(frames)
        current = maximum = 0
        for frame in items:
            current = current + 1 if frame.bilateral_positive_force else 0
            maximum = max(maximum, current)
        return cls(
            sampled_frames=len(items),
            inner_contact_frames=sum(frame.inner_contact for frame in items),
            outer_contact_frames=sum(frame.outer_contact for frame in items),
            bilateral_frames=sum(frame.inner_contact and frame.outer_contact for frame in items),
            max_consecutive_bilateral_positive_force_frames=maximum,
            max_inner_normal_force_n=max((frame.inner_normal_force_n for frame in items), default=0.0),
            max_outer_normal_force_n=max((frame.outer_normal_force_n for frame in items), default=0.0),
        )


def contact_class(frame: ContactEvidence) -> str:
    inner = frame.inner_contact and frame.inner_normal_force_n > POSITIVE_FORCE_N
    outer = frame.outer_contact and frame.outer_normal_force_n > POSITIVE_FORCE_N
    return { (False, False): "negative", (True, False): "inner_only",
             (False, True): "outer_only", (True, True): "bilateral" }[(inner, outer)]


def closure_gate_passes(stats: ContactSequenceStats) -> bool:
    return stats.max_consecutive_bilateral_positive_force_frames >= REQUIRED_CONSECUTIVE_FRAMES
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run `pytest -q tests/test_contact_diagnostics.py`.

Expected: all Task 1 tests pass.

- [ ] **Step 5: Add failing real sampler tests**

Tests must construct a minimal context with known contact IDs, monkeypatch only MuJoCo force/distance primitives, and assert:

```python
def test_sample_contact_evidence_aggregates_named_pairs_for_each_side(monkeypatch):
    ctx = _fake_contact_context(
        contacts=[(10, 1), (2, 10)],
        names={1: "finger_inner_left_collision_00", 2: "finger_outer_left_collision_00",
               10: "probe_geom"},
    )
    monkeypatch.setattr("mujoco.mj_contactForce", _force_by_contact_id([0.4, 0.6]))
    monkeypatch.setattr("mujoco.mj_geomDistance", _distance_by_pair(-0.001))
    result = sample_contact_evidence(ctx, ({1}, {2}), {10})
    assert result.inner_normal_force_n == pytest.approx(0.4)
    assert result.outer_normal_force_n == pytest.approx(0.6)
    assert result.bilateral_positive_force
    assert [(pair.side, pair.object_geom_id, pair.finger_geom_id) for pair in result.raw_pairs] == [
        ("inner", 10, 1), ("outer", 10, 2)
    ]
```

Also test empty/overlapping finger sets, non-object contacts ignored, geom-order invariance, minimum side gap, and non-finite force/distance rejection.

- [ ] **Step 6: Verify sampler tests RED**

Run `pytest -q tests/test_contact_diagnostics.py -k sample_contact_evidence`.

Expected: import error for missing `sample_contact_evidence`.

- [ ] **Step 7: Implement sampler**

Implementation rules:

```python
def sample_contact_evidence(ctx, finger_side_geom_ids, object_geom_ids) -> ContactEvidence:
    inner_ids, outer_ids = _validated_side_ids(finger_side_geom_ids)
    object_ids = {int(value) for value in object_geom_ids}
    if not object_ids:
        raise ValueError("object geom ids must not be empty")

    pair_rows = []
    side_forces = {"inner": 0.0, "outer": 0.0}
    side_contacts = {"inner": False, "outer": False}
    for contact_id in range(int(ctx.data.ncon)):
        contact = ctx.data.contact[contact_id]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if g1 in object_ids:
            object_id, finger_id = g1, g2
        elif g2 in object_ids:
            object_id, finger_id = g2, g1
        else:
            continue
        side = "inner" if finger_id in inner_ids else "outer" if finger_id in outer_ids else None
        if side is None:
            continue
        force_torque = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(ctx.model, ctx.data, contact_id, force_torque)
        normal_force = max(0.0, float(force_torque[0]))
        if not np.isfinite(force_torque).all():
            raise ValueError("contact force must be finite")
        side_contacts[side] = True
        side_forces[side] += normal_force
        pair_rows.append(ContactPairEvidence(
            object_id, _geom_name(ctx, object_id), finger_id, _geom_name(ctx, finger_id),
            side, normal_force,
        ))

    def minimum_gap(side_ids):
        distances = []
        for finger_id in side_ids:
            for object_id in object_ids:
                value = float(mujoco.mj_geomDistance(
                    ctx.model, ctx.data, finger_id, object_id, 1.0, None
                ))
                if not isfinite(value):
                    raise ValueError("signed geom distance must be finite")
                distances.append(value)
        return min(distances)

    return ContactEvidence(
        side_contacts["inner"], side_contacts["outer"], tuple(pair_rows),
        side_forces["inner"], side_forces["outer"],
        minimum_gap(inner_ids), minimum_gap(outer_ids),
    )
```

Use local `_geom_name` and `_validated_side_ids`; do not import private contact helpers from `experiment.py`.

- [ ] **Step 8: Verify Task 1 and regression suite GREEN**

Run:

```bash
pytest -q tests/test_contact_diagnostics.py
pytest -q
```

Expected: all focused tests and existing 120 tests pass.

- [ ] **Step 9: Record evidence**

Append exact RED command/failure and GREEN counts/timings to progress ledger. No commit because workspace has no Git metadata.

---

### Task 2: Synthetic and Real MuJoCo Contact Controls

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`
- Create: `vm_sync/experiments/openarm_robust_grasp/assets/cube_contact_fixed_scene.xml`

**Interfaces:**
- Consumes: Task 1 sampler and existing `load_scene`, arm lookup, named finger collision geoms.
- Produces: `ContactControlResult`, `CONTACT_CONTROL_NAMES`, `run_contact_controls(scene_path, repeats=2)`.

- [ ] **Step 1: Write fixed-scene asset**

Create exact experiment-owned scene structure, using same visual/options and official asset include as current cube scene, but object has no free joint:

```xml
<mujoco model="openarm bimanual fixed contact probe">
  <visual><global offwidth="960" offheight="600"/></visual>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic"/>
  <asset><model name="cell" file="/home/nostalgia/openarm_ws/src/openarm_mujoco/v2/cell.xml"/></asset>
  <worldbody>
    <attach model="cell" prefix=""/>
    <body name="contact_probe" pos="0.45 0 1.05">
      <geom name="contact_probe_geom" type="box" size="0.005 0.005 0.005"
            mass="0.015" friction="4 2 0.1" solref="0.002 1"
            solimp="0.95 0.99 0.001" contype="1" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>
```

- [ ] **Step 2: Write failing control identity, construction, and determinism tests**

Required assertions:

```python
def test_contact_controls_have_exact_predeclared_order():
    assert CONTACT_CONTROL_NAMES == ("negative", "inner_only", "outer_only", "bilateral")


def test_run_contact_controls_requires_two_fresh_repeats_and_exact_classes(fixed_scene):
    results = run_contact_controls(fixed_scene, repeats=2)
    assert [(row.repeat_index, row.expected_class) for row in results] == [
        (0, name) for name in CONTACT_CONTROL_NAMES
    ] + [(1, name) for name in CONTACT_CONTROL_NAMES]
    assert all(row.geometry_precondition_pass for row in results)
    assert all(row.observed_class == row.expected_class for row in results)
    assert all(row.pass_control for row in results)
```

Also assert each repeat calls `load_scene` four times, negative minimum gap `>= 0.020`, single-side control opposite gap `> 0`, bilateral both signed gaps `<= 0`, and positive expected sides exceed `1e-6 N`.

- [ ] **Step 3: Run control tests and verify RED**

Run `pytest -q tests/test_contact_diagnostics.py -k contact_control`.

Expected: missing `ContactControlResult` or `run_contact_controls`.

- [ ] **Step 4: Implement deterministic control construction**

Add immutable result:

```python
CONTACT_CONTROL_NAMES = ("negative", "inner_only", "outer_only", "bilateral")

@dataclass(frozen=True)
class ContactControlResult:
    repeat_index: int
    expected_class: str
    observed_class: str
    geometry_precondition_pass: bool
    evidence: ContactEvidence
    pass_control: bool
```

For every condition load fresh context. Identify one closest inner/outer geom pair by current center distance. Mutate only experiment-owned fixed probe `body_pos` and `geom_size`, preserve box type and orientation, call `mj_setConst` and `mj_forward`, settle 10 steps, then sample. Construction rules:

- negative center: all named finger-geom center bounding-box maximum plus world-Z `0.200 m`; half-size `0.001 m`. Validate every finger-probe signed gap is at least `0.020 m`; if not, double world-Z offset deterministically up to `0.800 m` before failing construction.
- inner-only center: selected inner geom center; half-size `0.001 m`, shrinking to `0.0005 m` if opposite signed gap is not positive.
- outer-only: symmetric.
- bilateral center: selected-pair midpoint; isotropic half-size `0.5 * center distance + 0.001 m`.

After positioning, calculate signed gaps before classifying. Raise `RuntimeError` if finite bounded construction cannot satisfy declared geometry precondition; do not relabel expected class from observed result.

- [ ] **Step 5: Verify controls GREEN and repeatability**

Run focused tests twice, then full suite. Expected: eight control rows; identical serialized rows across repeated CLI-free calls; all tests pass.

- [ ] **Step 6: Record control gate**

Append exact observed classes, force ranges, signed gaps, test counts, and `measurement_gate_pass` to ledger. If gate fails, stop physical stages and write localized result note under Task 6.

---

### Task 3: Fixed-Cube Quasi-Static Closure

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`

**Interfaces:**
- Consumes: Task 1 sampler/stats, Task 2 fixed scene, existing motion/gripper calibration helpers.
- Produces: `ClosureConfig`, `ClosureFrame`, `ClosureResult`, `command_profile(...)`, `run_fixed_cube_closure(...)`.

- [ ] **Step 1: Write failing config and command-profile tests**

```python
@pytest.mark.parametrize("profile", ["raw_full", "calibrated"])
def test_command_profile_has_exact_direction_endpoints_and_101_levels(profile, context, calibration):
    values = command_profile(context, context.left, calibration, profile)
    assert len(values) == 101
    assert values[0] > values[-1]
    expected = ((0.7854, 0.0) if profile == "raw_full" else (0.58905, 0.0))
    assert values[[0, -1]].tolist() == pytest.approx(expected)


def test_closure_config_has_exact_fixed_protocol_defaults():
    config = ClosureConfig()
    assert config.command_levels == 101
    assert config.steps_per_level == 20
    assert config.placement_names == ("current_waypoint", "geometric_center")
    assert config.profile_names == ("raw_full", "calibrated")
```

Test invalid profile, levels other than at least 2, negative settle count, non-finite values, and raw direction derived from measured aperture rather than hard-coded command ordering.

- [ ] **Step 2: Verify RED, then implement records/profile**

Implement frozen config/frame/result. `ClosureFrame` fields exactly: placement, profile, command_index, settle_index, command, close_fraction, joint_qpos, actuator_force, evidence, cube_position_xyz, cube_quaternion_wxyz, gripper_position_xyz, gripper_quaternion_wxyz, relative_translation_xyz, relative_rotation_rad. `ClosureResult` includes config identity, 2020 frame rows, sequence stats, maximum cube displacement, gate pass, and failure reason.

- [ ] **Step 3: Verify record/profile tests GREEN**

Run focused tests; expected pass.

- [ ] **Step 4: Write failing runner orchestration tests**

Monkeypatch movement and step sampling to assert:

- fresh context per placement/profile combination;
- exact order `current_waypoint/raw_full`, `current_waypoint/calibrated`, `geometric_center/raw_full`, `geometric_center/calibrated`;
- existing approach then grasp call once per context;
- geometric-center body X/Y equals centroid midpoint and body Z/orientation unchanged;
- exactly 101 command values and 20 observer calls each;
- actuator transmission type validated as joint transmission before reading `jnt_qposadr`;
- gate depends only on geometric-center positive-force continuity.

- [ ] **Step 5: Verify runner RED**

Run `pytest -q tests/test_contact_diagnostics.py -k fixed_cube`.

Expected: missing `run_fixed_cube_closure`.

- [ ] **Step 6: Implement minimal fixed runner**

Reuse `load_scene`, `_arm_from_name`, existing zero-noise `_move_to`, `_finger_center_contact_waypoints`, `set_gripper`, and gripper calibration. Do not alter those helpers. Before qpos indexing require actuator transmission to reference a joint; raise descriptive `ValueError` otherwise. Calculate gripper quaternion from existing end-effector rotation and relative rotation using clipped quaternion geodesic angle `2*acos(abs(dot(q0, q1)))`.

- [ ] **Step 7: Verify Task 3 GREEN**

Run focused tests and full suite. Then run real fixed protocol only if Task 2 measurement gate passed. Record which branch passes; do not tune thresholds or geometry after observation.

---

### Task 4: Free-Cube Closure and Conditional Lift

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`

**Interfaces:**
- Consumes: Task 3 records/sampling, existing free-cube scene and pose-preserving move helper.
- Produces: `FREE_CUBE_OFFSETS`, `run_free_cube_closure(...)`, `free_closure_gate_passes(...)`, `run_conditional_lift(...)`, `LiftResult`.

- [ ] **Step 1: Write failing exact-offset and closing-axis tests**

```python
def test_free_cube_offsets_are_exact_and_ordered():
    assert FREE_CUBE_OFFSETS == (
        ("centered", 0.0, 0.0),
        ("closing_axis_minus_2mm", -0.002, 0.0),
        ("closing_axis_plus_2mm", 0.002, 0.0),
        ("world_z_minus_2mm", 0.0, -0.002),
        ("world_z_plus_2mm", 0.0, 0.002),
    )


def test_closing_axis_is_normalized_inner_to_outer_centroid_vector():
    axis = closing_axis(np.array([1.0, 2.0, 3.0]), np.array([1.0, 5.0, 3.0]))
    assert axis.tolist() == pytest.approx([0.0, 1.0, 0.0])
```

Reject coincident/non-finite centroids.

- [ ] **Step 2: Verify RED, implement constants and axis helper, verify GREEN**

Run exact focused tests before and after minimal implementation.

- [ ] **Step 3: Write failing free-closure gate tests**

Test all boundaries independently:

```python
def test_free_closure_gate_requires_contact_continuity_and_no_ejection():
    passing = _free_result(consecutive=20, displacement=0.010, nonfinite=False,
                           frame_support_transition=False)
    assert free_closure_gate_passes(passing)
    assert not free_closure_gate_passes(_free_result(consecutive=19))
    assert not free_closure_gate_passes(_free_result(displacement=0.010001))
    assert not free_closure_gate_passes(_free_result(nonfinite=True))
    assert not free_closure_gate_passes(_free_result(frame_support_transition=True))
```

Runner tests assert 100-step open/cube settle baseline, offsets applied to grasp target instead of object qpos, calibrated 101-by-20 closure, object-frame contact recorded separately, centered condition evaluated first, and offset interpretation disabled when centered fails.

- [ ] **Step 4: Verify RED, implement free closure, verify GREEN**

Use existing experiment-owned free cube scene. Fresh context per offset. Preserve cube qpos after baseline; modify only Cartesian grasp target by `closing_axis_offset * axis + [0,0,z_offset]`. Track displacement from settled cube center and detect new cube-frame contacts by named object geom against geoms rooted under `black_frame`.

- [ ] **Step 5: Write failing conditional-lift eligibility and metric tests**

Assert failed closure raises `ValueError("conditional lift requires passed free closure")`; passed closure commands exact `[0, 0, 0.020]` target delta over 200 sampled steps. Gate boundaries:

- elevation `0.015 m` passes, lower fails;
- bilateral-positive-force fraction `0.80` passes, lower fails;
- final translation drift `0.010 m` passes, greater fails;
- relative rotation `15 degrees` passes, greater fails;
- final frame support always fails.

- [ ] **Step 6: Verify RED, implement lift, verify GREEN**

Implement frozen `LiftResult`; call existing `_move_to_pose` with preserved orientation and 200-step observer. Compute all metrics from sampled frames, not endpoint-only values.

- [ ] **Step 7: Run integration gate**

If Task 3 fixed gate passed, run centered free closure. Run remaining offsets only when centered passes. Run lift only for each closure-passing condition. Record explicit not-run reasons at each blocked branch.

---

### Task 5: Validated Reports and CLI Orchestration

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/openarm_robust_grasp/contact_diagnostics.py`
- Modify: `vm_sync/experiments/openarm_robust_grasp/tests/test_contact_diagnostics.py`
- Create: `vm_sync/experiments/openarm_robust_grasp/run_contact_diagnostics.py`

**Interfaces:**
- Consumes: all Task 1-4 results and approved design constants.
- Produces: `DiagnosticRunResult`, `write_contact_diagnostic_report(...)`, CLI exit status `0` for scientifically valid completed/blocked decision tree and nonzero for protocol/runtime corruption.

- [ ] **Step 1: Write failing writer validation tests**

Use `tmp_path`. Assert exact filenames, stable column order, JSON constants, stage order, explicit `not_run_reason`, hashes, and rejection before replacement for duplicate/missing controls, wrong offsets, non-finite values, downstream rows after failed upstream gate, and malformed result types.

- [ ] **Step 2: Verify writer RED**

Run `pytest -q tests/test_contact_diagnostics.py -k report`.

Expected: missing writer.

- [ ] **Step 3: Implement validated rollback-protected writer**

Reuse Task 3 stability writer pattern: validate complete in-memory result graph; serialize four CSV byte streams and summary JSON before touching canonical paths; stage all five files in output directory; back up existing canonical files; replace; roll back every replaced file on any exception; remove staging/backups in `finally`.

Empty conditional-lift CSV contains header only. `summary.json` must carry `conditional_lift_not_run_reason` when no eligible condition exists.

- [ ] **Step 4: Verify writer GREEN including injected failure rollback**

Monkeypatch `os.replace` to fail on each replacement index. Assert prior canonical bytes restored for every injected point.

- [ ] **Step 5: Write failing CLI orchestration tests**

Patch runners to assert exact branch behavior:

1. controls fail -> no fixed/free/lift call;
2. controls pass, fixed fails -> no free/lift call;
3. fixed passes, centered free fails -> no offset/lift call;
4. centered and some offsets pass -> lift only those conditions;
5. writer called exactly once with complete graph.

- [ ] **Step 6: Implement CLI**

Arguments:

```text
--fixed-scene assets/cube_contact_fixed_scene.xml
--free-scene assets/cube_lift_scene.xml
--output-dir outputs/contact_formation_diagnostics
--repeats 2
```

Reject overrides that alter approved thresholds/condition levels. Print one line per gate and canonical output paths.

- [ ] **Step 7: Verify focused and full suite GREEN**

Run:

```bash
pytest -q tests/test_contact_diagnostics.py
pytest -q
```

Expected: all tests pass with no warnings/errors.

---

### Task 6: Canonical Runs, Repeatability, Documentation, and Review

**Files:**
- Modify: `vm_sync/experiments/openarm_robust_grasp/README.md`
- Modify: `.superpowers/sdd/2026-08-13-openarm-simulation-grasp-stability/progress.md`
- Create: `docs/results/2026-09-01-openarm-contact-formation-diagnostics.md`
- Create through runner: `vm_sync/experiments/openarm_robust_grasp/outputs/contact_formation_diagnostics/*`

**Interfaces:**
- Consumes: tested CLI and canonical VM environment.
- Produces: two-run evidence, bounded result interpretation, next predeclared branch.

- [ ] **Step 1: Run canonical protocol twice**

Run:

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_contact_diagnostics.py
sha256sum outputs/contact_formation_diagnostics/*
python3 run_contact_diagnostics.py
sha256sum outputs/contact_formation_diagnostics/*
```

Expected: run completes at valid decision-tree leaf. Compare hashes exactly. If per-frame floating output differs, report differing fields and maximum absolute deltas; do not delete evidence or round until equal.

- [ ] **Step 2: Run final verification**

Run:

```bash
pytest -q
python3 -m compileall -q openarm_robust_grasp run_contact_diagnostics.py
```

Expected: all tests pass; compile command exits 0 with no output.

- [ ] **Step 3: Write result note using measured branch**

Document environment, model paths, exact constants, control evidence, fixed/free/lift gate outcomes, output hashes, test count/timing, limitations, and one next step selected exactly from:

- repair sampler/grouping;
- inspect fixed geometry/actuator/contact configuration;
- repair waypoint/alignment;
- inspect free-object dynamics/ejection/closure control;
- inspect lift tracking/gravity retention;
- begin seeded perception-correction robustness study.

Do not claim unobserved causes.

- [ ] **Step 4: Update README and ledger**

README gains CLI command, output schemas, gate definitions, and claim boundary. Ledger records every RED/GREEN cycle, canonical hashes, review findings, and no-Git ruling.

- [ ] **Step 5: Perform code self-review**

Check exact design constants, no private-helper semantic duplication, no official asset writes, no post-hoc thresholds, force sign handling, fixed/free context isolation, eligibility enforcement, error cleanup, writer rollback, and old protocol regressions. Fix findings through new failing tests.

- [ ] **Step 6: Perform scientific self-review**

Separate measurement from interpretation; confirm controls establish construct validity; mark fixed-grid outcomes deterministic; reject factor-effect/probability language; report negative branch as valid evidence; list remaining confounds.

- [ ] **Step 7: Final verification after review fixes**

Repeat full pytest, compileall, and canonical run only if production behavior changed. Record final evidence. Stop at selected decision branch; do not expand into its follow-up task in this plan.

---

## Plan Self-Review

- Spec coverage: measurement controls, raw evidence, fixed closure, placement comparison, five free offsets, conditional 20 mm lift, outputs, repeatability, testing, and scientific claim boundary each map to Tasks 1-6.
- Placeholder scan: no TBD/TODO/implement-later steps.
- Type consistency: Task 1 evidence/stats feed Tasks 2-4; Task 3 closure records feed Task 4 eligibility; Task 1-4 results feed Task 5 writer/CLI; Task 5 outputs feed Task 6 documentation.
- Scope: one diagnostic decision tree; no friction screen, visual pipeline, ROS 2, or hardware expansion.
- Git exception: all commit steps replaced by ledger evidence because project root has no Git metadata.
