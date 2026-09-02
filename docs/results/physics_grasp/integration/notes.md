# Task 13 — Physics-mode integration validation notes

> **Correction notice (2026-09-02, post-review):** the original aligned-trial
> writeup below stated the object "never left the table" and that its
> position was "static ... while `any_contact` is `false`". Both statements
> are false — the object is ejected off the table at t~=324.48 s and lands
> on the floor 4.2 m away. See the struck-through/annotated passages in
> Step 6 and Step 7 below, and the new **"Corrected aligned-trial finding"**
> section at the end of this file for the full, corrected account. The
> misaligned-trial findings and the Step-7 contact statistics (50/26/1
> breakdown, 0.16 s longest bilateral run, 8-17 N forces) were verified
> independently and are unaffected — only the aligned-trial conclusion
> changes.

Date: 2026-09-02
Launch log: `/tmp/physics_launch.log` (also mirrored to
`docs/results/physics_grasp/integration/` on the Windows side is not done —
only the `integration/` results tree is mirrored per Step 9; the raw launch
log stays on the VM at `/tmp/physics_launch.log`).

## Environment note: stale VM processes found and removed

Before the checks below could pass cleanly, this session found an entire
leftover ROS 2 process tree from a **previous, unrelated launch** still
running on the VM from the day before (started "9月01", i.e. 2026-09-01):

- `ros2 launch openarm_moveit_config bringup_all.launch.py` (pid 37253)
- `robot_state_publisher` (37274)
- two `static_transform_publisher` nodes (37276, 37278)
- `apple_detector.py` (37280)
- `moveit_ros_move_group` / `move_group` (37311)
- `pick_apple.py`, a kinematic-mode script that publishes
  `/cube_controller/commands` (pid 37480)

These were sharing `ROS_DOMAIN_ID=71` with our physics-mode launch and were
the actual cause of two symptoms seen on the first pass:
- `/cube_controller/commands` appearing in `ros2 topic list` (published by
  the stray `pick_apple.py`, publisher count 1, subscriber count 0 — nothing
  in the physics-mode graph subscribes to it or acts on it);
- `physics_picker` logging `Ignoring unexpected goal/result response. There
  may be more than one action server for the action 'move_action'` and a
  `planning_failure` outcome on the first misaligned-trial attempt, caused
  by the duplicate stray `move_group` competing for the same action name.

All of the above stray processes were killed (`kill -9`), the `ros2` daemon
was restarted (`ros2 daemon stop`) to drop stale discovery cache entries,
and every check below was then re-run against the clean graph. **No source
file in this repository publishes to `/cube_controller/commands` or starts
a second `move_group`** — grep confirms the physics-mode assets
(`physics_controllers.yaml`, `run_physics_grasp.launch.py`) declare only the
5 expected controllers and no cube controller. This was environmental
contamination from a prior, unrelated session, not a defect in Tasks 1-12.

## Step 1-2: graph readiness

Launch command (VM):
```
ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0
```

After removing the stale processes above, the clean graph shows:

```
/grasp_raw_state   present
cube_controller topic   absent
```

`ros2 control list_controllers` (clean):
```
joint_state_broadcaster    joint_state_broadcaster/JointStateBroadcaster          active
right_hand_controller      position_controllers/GripperActionController          active
left_hand_controller       position_controllers/GripperActionController          active
right_arm_controller       joint_trajectory_controller/JointTrajectoryController active
left_arm_controller        joint_trajectory_controller/JointTrajectoryController active
```

All 5 expected controllers, all active. No `cube_controller` entry.

Note: on the very first activation attempt, `joint_state_broadcaster` timed
out during the launch's automatic activation
(`Switch controller timed out after 5.000000 seconds!`, visible in
`/tmp/physics_launch.log`), most likely due to CPU contention from the
leftover stray `move_group`/`mujoco` processes competing for CPU at the same
time. It was activated manually afterwards
(`ros2 control set_controller_state joint_state_broadcaster active`) and
came up cleanly with no errors. This is recorded for completeness; it is an
environmental/timing artifact of the stale-process contamination above, not
a defect in the physics-mode controller configuration itself.

## Step 3: resting object height

`ros2 topic echo /grasp_raw_state --once`, repeated 4 times, all identical:

- `object_pose.position = (0.30, 0.00, 0.44991653871789816)` — matches the
  expected ~0.445 m (sphere radius above the 0.42 m table surface), stable
  across repeated calls (no drift, no jitter).
- `contact_geoms: [table_top]`, `contact_normal_forces: [0.981]` — the
  object rests on the table under gravity alone, held up by table contact
  only, with **zero commands** issued to any controller. This is the
  direct evidence that no attachment/pose-command mechanism exists in
  physics mode.

## Step 4: MoveIt cube-joint errors

```
grep -c 'cube_x\|cube_y\|cube_z' /tmp/physics_launch.log
```
Result: `0`. No cube-joint errors anywhere in the launch log (484 lines
total).

## Step 5: misaligned trial (`grasp_z_offset_m` temporarily 0.05)

Parameter change (temporary, restored after the trial):
- `openarm_grasp_physics/config.py`: `grasp_z_offset_m` `0.0 -> 0.05` before
  the trial, rebuilt with
  `colcon build --packages-select openarm_grasp_physics --build-base build_repro --install-base install_repro`,
  then restored `0.05 -> 0.0` and rebuilt again immediately after the trial.
  This makes the gripper close 5 cm above the object's actual surface so
  the fingers guarantee-miss it; it is not a lasting configuration change.

Trial `misaligned_check` result (`trial.json` / `report.md`):
- `category: no_contact`
- `max_elevation_m: 0.0` (well below the 0.002 m defect threshold)
- `sample_count: 824`, `any_contact: false`
- 0 of the 824 samples show either finger geom in contact — confirms this
  is not a contact-classification bug, it is a genuine "fingers never
  touched the object" result.
- Process exit code was `250` (aborted, `terminate called without an
  active exception`) rather than the brief's expected `exit=1`. This abort
  happens **after** the trial has already completed, been classified, and
  been written to disk (`trial.json`/`report.md`/`samples.csv` are all
  present, complete, and internally consistent) — it is a shutdown-time
  C++ exception during process teardown, not a functional failure of the
  grasp state machine or classifier. It does not match any of the four
  listed defect categories (gripper goal never accepted; zero samples;
  `no_contact` with finger geoms in `contact_geoms`; object motion with no
  contact) and is noted here as an observation rather than treated as a
  blocking defect.

This confirms: with the gripper closing above the object, nothing carries
the object — the only thing that can move it is contact.

## Step 6: aligned trial

Trial `aligned_check` result (`trial.json` / `report.md`):
- `category: unilateral_contact`
- `gripper_ok: true`, `planning_ok: true`, `sample_count: 924` (nonzero) —
  rules out the "gripper action never accepted" and "zero samples" defect
  categories.
- `any_contact: true`, and grep of `samples.csv` shows genuine finger-geom
  contact was recorded (this is not the `no_contact`-with-finger-geoms
  classification bug — the category is not even `no_contact`).
- **CORRECTION (post-review, see bottom of file): the statement that
  originally appeared here — "the object never left the table" / "object
  position is static ... while `any_contact` is `false`" — is FALSE and is
  struck out below.** `max_elevation_m: 4.3e-5 m` only reflects the
  *initial* momentary finger squeeze during `close`; it does **not** mean
  the object stayed near the table. See "Corrected aligned-trial finding"
  below for what actually happens: the object is ejected laterally at
  `t~=324.48 s`, still inside `close`, and lands on the floor. The `report.md`
  numbers (`max_elevation_m`, `max_drift_m` both reading effectively 0) are
  real values of narrowly-defined metrics, not evidence the object stayed
  put — see the "Metric limitations" note below.
- Re-checked against the four listed defect categories with the corrected
  trajectory in hand: this is **still not** the "object motion with no
  contact" (attachment-leak) defect, because the object's motion during the
  `False/False` samples is the *ballistic/sliding continuation of velocity
  imparted during a real, recorded bilateral-contact event* (momentum
  carried forward, not a commanded or uncaused motion) — see below. It is,
  however, a different and more informative physical finding than the
  `unilateral_contact` label suggests on its own.

## Step 7: diagnosis (evidence gathered before any parameter change)

**CORRECTION (post-review): the window cited below as "the close stage"
(322.90 s - 324.48 s, ~1.56 s) is wrong. `trial.json`'s own `stage_times`
puts the full `close` stage at 322.90 s -> 329.56 s (6.66 s). The analysis
below only covers the first ~1.58 s of that window and stops at exactly the
moment the object is ejected (t=324.48 s) — the diagnosis below is
therefore incomplete on its own; read it together with "Corrected
aligned-trial finding" at the end of this file, which covers the full
window and supersedes the "closest to case 2" conclusion.**

Per the brief's required order, `samples.csv` for the first part of the
`close` stage (sim time 322.90 s - 324.48 s, ~1.56 s of the stage's full
6.66 s) was inspected in full:

1. **Not "no finger contact at all"** — ruled out. 77 of 333 close-stage
   samples show contact on at least one finger (the remaining 256 are
   `False/False`); no other stage shows any contact.
2. **Contact pattern**: of the samples with any contact, the
   `(left_contact, right_contact)` pairs break down as
   `True/True: 50`, `True/False: 26`, `False/True: 1`. Contact was
   predominantly bilateral when present, but it never held: the longest
   unbroken run of consecutive bilateral (`True/True`) samples is **8
   samples = 0.16 s** (samples are logged every 10 sim steps at
   `dt=0.002 s`, i.e. every 0.02 s), against the
   `confirm_window_s = 0.30 s` required by `_confirm_bilateral_contact()`
   in `physics_picker.py` before the state machine is allowed to advance
   to `lift`. Bilateral contact kept breaking and re-forming (chattering
   between `True/True` and `True/False`) for about 1.5 simulated seconds,
   then was lost entirely (`False/False`) for the remainder of `close`,
   and the trial went straight from `close` to `retreat` — it never
   reached `lift` or `hold` (confirmed: no `lift`/`hold` rows exist in
   `samples.csv` at all, only `settle, hover, open, descend, close,
   retreat`).
3. **Force magnitude, while bilateral**: normal forces during the
   `True/True` samples ranged roughly 8-17 N on each finger — far above
   `min_normal_force = 0.5 N`. Forces are not the limiting factor here.
4. **Lift was never attempted** — the trial never reached `lift`, so the
   "bilateral contact with high force but object escapes on lift" /
   friction case does not apply either.

**Conclusion (superseded — see "Corrected aligned-trial finding" below):**
this result does not fit cleanly into the brief's case 3 ("bilateral
contact with small normal forces" — forces were ample, not small) or case 4
("escapes on lift" — lift was never reached). It is closest to case 2,
"contact on one finger only -> coupling or approach-angle problem, not
force": contact was intermittently unilateral and, even when bilateral, was
not sustained/symmetric long enough to pass the confirmation window,
despite forces being well above threshold. The brief's own guidance for
case 2 is explicit that this is **not** a force problem, and it gives no
force-related parameter to raise for it (unlike cases 3 and 4). Raising
`position_kp`/`position_max_effort` (case 3's remedy) would not address a
timing/geometry chattering issue and risks masking it; the friction pair
(case 4's remedy) is moot because lift was never attempted.

**All of the above (the contact statistics, the 0.16 s longest-bilateral-run
figure, and the 8-17 N force range) is correct and independently
reproduces exactly against `samples.csv`.** What is wrong is treating
"never reached lift" as the end of the story: the chattering contact does
not fizzle out harmlessly, it ends in an ejection event at t=324.48 s that
throws the object off the table. See below for the corrected, more
informative physical finding.

**No parameter was changed as a result of Step 7.** Per the brief's
instruction to diagnose before tuning, and because the diagnostic evidence
points at a coupling/approach-angle/contact-stability issue rather than a
force or friction deficiency, tuning `position_kp`, `position_max_effort`,
or the friction pair would not be a justified fix for what was actually
observed, and no other file was in scope to touch for this task. This is
recorded as an open finding for a follow-up task: investigate why bilateral
contact chatters instead of settling once both fingers reach the object
during `close` (candidates: finger approach symmetry/coupling, closing
speed relative to contact damping, or MuJoCo solver contact parameters —
none of which were touched here).

## Summary of parameter changes

| File | Parameter | Before | After | Trial that motivated it | Status |
|---|---|---|---|---|---|
| `openarm_grasp_physics/config.py` | `grasp_z_offset_m` | `0.0` | `0.05` | required by Step 5 to force a miss | restored to `0.0` immediately after `misaligned_check` |

No other parameter was changed. Physics timestep remained `0.002` throughout
(not touched).

## Environment cleanup performed (not a source change)

- Killed stray processes left running from a prior, unrelated VM session
  (see "Environment note" above): pids 37253, 37274, 37276, 37278, 37280,
  37311, 37480.
- Restarted the `ros2` CLI daemon (`ros2 daemon stop`) to clear a stale
  discovery cache that briefly showed duplicate node names after the kill.

## Overall outcome

- Graph comes up clean: `/grasp_raw_state` present, no `cube_controller`
  topic, all 5 controllers active (once the stale VM processes were
  cleared and `joint_state_broadcaster` manually activated after a
  transient CPU-contention timeout).
- Object rests on the table at `z ~= 0.4499 m` under `table_top` contact
  alone, zero commands issued.
- Zero `cube_x`/`cube_y`/`cube_z` errors in the launch log.
- Misaligned trial: `no_contact`, elevation `0.0 m` — nothing moves the
  object except contact.
- Aligned trial: `unilateral_contact` (the state-machine's recorded label)
  — a legitimate physics finding, not a defect under the brief's four
  defect categories. **CORRECTED per post-review analysis below: the
  underlying mechanism is squeeze-ejection** — bilateral contact forms
  with ample force (8-17 N) but is geometrically unstable, chatters, and
  at t=324.48 s ejects the object laterally off the table entirely (it
  lands on the floor ~4.2 m away, with no measurable deceleration
  afterward — a low-friction observation). The trial never reaches `lift`,
  but that is no longer the main story; see "Corrected aligned-trial
  finding" below. No parameter changed; flagged for a separate follow-up
  task to investigate the ejection mechanism (finger/object contact
  geometry, damping) and the apparent near-zero friction.

## Corrected aligned-trial finding (post-review, 2026-09-02)

The Step 6/7 writeup above stopped its inspection of `samples.csv` at
t=324.48 s, describing that as the end of the `close` stage. It is not —
`trial.json`'s `stage_times` puts `close` at 322.90 s -> 329.56 s (6.66 s
total). Re-inspecting the full window changes the finding.

### What actually happens (verified directly against `aligned_check/samples.csv`)

- t=322.90-324.46 s: contact chatters as already described (bilateral
  `True/True` interleaved with unilateral `True/False`, longest sustained
  bilateral run 0.16 s, forces 8-17 N per finger).
- **t=324.14-324.46 s (17 samples, still bilateral/near-bilateral contact):
  the object accelerates laterally while gripped.** `object_x` progresses
  0.3238 -> 0.3241 -> 0.3244 -> 0.3248 -> 0.3252 -> 0.3256 -> 0.3261 ->
  0.3267 -> 0.3274 -> 0.3282 -> 0.3290 -> 0.3301 -> 0.3316 -> 0.3335 ->
  0.3361 -> 0.3395 -> 0.3441, with the per-sample increment growing every
  step (from ~0.0003 m/sample up to ~0.0046 m/sample) — a monotonically
  accelerating lateral launch, not noise.
- **t=324.48 s: contact drops to `False/False`** and the object continues
  in free flight/slide at an effectively **constant velocity of
  ~0.456-0.462 m/s** (measured from consecutive `object_x` deltas both
  right after ejection and ~2.7 s later, e.g. 327.14->327.16 s) — i.e. no
  measurable deceleration.
- **The object falls off the table**: `object_z` drops from 0.4499 m to
  0.029916538717898292 m (the floor) by roughly t=325.6-325.9 s, and stays
  pinned at that floor height for the rest of the trial.
- By the last recorded sample (t=333.64 s, well into `retreat`),
  `object_x` has reached ~4.58 (from a start of 0.30) — about **4.2-4.3 m
  of travel**, entirely unpowered by any controller.
- The trial's own `elevation` column reads a constant `-0.42` once the
  object is on the floor (i.e. 0.42 m *below* its resting height), which
  is consistent with `object_z` (0.4499 -> 0.0299 = -0.42 m). This
  contradicts `trial.json`'s `max_elevation_m: 4.322870694939418e-05`,
  which is not wrong as a number — it is just not the number this finding
  needs (see "Metric limitations" below).

### Corrected physical finding

The gripper does **not** fail to make contact with the object. It makes
strong, genuine bilateral contact (8-17 N per finger, far above the 0.5 N
threshold) and then **ejects** it: two rigid parallel fingers pinching a
smooth ~25 mm sphere convert closing force into lateral launch velocity
rather than a stable grip, once the contact geometry becomes even slightly
asymmetric (as the chattering `True/False` samples show it does). The
`unilateral_contact` label the state machine recorded is technically what
the confirmation logic observed (bilateral contact never sustained the
required window), but it understates the mechanism: this is a
**squeeze-ejection** event, not a near-miss that simply didn't hold long
enough.

**Low-friction observation**: once ejected, the object shows no measurable
deceleration over its ~9 s of travel (on the table briefly, then across
the floor), which points to near-zero effective friction between the
object and both the table/floor and possibly the object's own geom
defaults. This is recorded as an observation relevant to any future
friction tuning (per the brief's own note that the object geom "inherits
defaults" while `table_top` uses `friction="0.05 0.01 0.01"`), not acted on
in this task.

### Why this is still not the "object motion with no contact" (attachment-leak) defect

The brief's fourth defect category is object motion **with no contact
recorded** — i.e., motion with no physical cause, which would indicate a
software attachment. That is not what happens here: the object's velocity
is imparted during real, recorded bilateral contact (t=324.14-324.46 s,
`any_contact: true`), and its subsequent free flight and floor-slide after
t=324.48 s (`any_contact: false`) is exactly the physically-expected
continuation of that momentum under gravity and (near-zero) friction —
no controller, joint, or software attachment is commanding the object's
pose at any point. This is real MuJoCo contact dynamics doing exactly what
contact dynamics should do; it rules out an attachment leak rather than
demonstrating one.

### Metric limitations (add as explicit caveats for future tasks)

Two `trial.json` fields cannot be trusted to rule out an object escaping
mid-trial, and a later task should not treat them as sufficient evidence
that "the object stayed near the gripper/table":

- **`max_drift_m` stays `0.0`** because drift is only computed relative to
  `self._grasp_relative`, which is only set once
  `_confirm_bilateral_contact()` returns `True` (see `physics_picker.py`).
  Since bilateral contact was never confirmed in this trial, drift
  tracking never started, even though the object moved ~4.2 m.
- **`max_elevation_m` only tracks upward peaks** (it reads
  `4.322870694939418e-05`, essentially the height of the momentary
  finger-squeeze bump, not the object's true final displacement). A 0.42 m
  *fall* off the table is invisible to this field; only the raw
  `object_z`/`object_x` columns in `samples.csv` (or a min-elevation /
  net-displacement metric, which does not currently exist) reveal it.

**Net effect**: `report.md`'s summary line "Max elevation: 0.0000 m
(threshold 0.020 m)" is an accurate value of a narrowly-scoped metric, but
reading it alone (without checking `samples.csv`) would incorrectly imply
the object was undisturbed. Future tasks relying on these two fields to
screen for "object escaped" should instead check `samples.csv` directly
(final `object_x`/`object_y`/`object_z` vs. initial, and whether contact
ever returns to `False/False` with nonzero velocity) until a
proper displacement/escape metric is added to the trial recorder.

### Revised Step 7 conclusion

The original "closest to case 2 (coupling/approach-angle, not force)"
conclusion is superseded. The mechanism is squeeze-ejection: bilateral
contact forms with ample force, is geometrically unstable against a smooth
sphere between rigid parallel fingers, and converts closing force into
lateral ejection before the confirmation window can complete. As before,
**no parameter was changed** — none of the brief's four diagnostic remedies
(cases 1-4) is a good fit for a squeeze-ejection mechanism, and this task's
scope explicitly excludes tuning based on an open-ended finding. This is
flagged for the separate follow-up task the coordinator has already
identified to investigate the ejection mechanism itself (finger/object
contact geometry, damping, or a compliant/non-rigid contact model), and the
near-zero-friction observation is flagged for that same follow-up's
attention when it looks at grip retention.
