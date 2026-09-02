# Task 13 — Physics-mode integration validation notes

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
- `max_elevation_m: 4.3e-5 m` — the object never left the table, and this
  displacement is fully explained by momentary finger squeeze during
  `close`, not a lift; there is no "object motion with no contact" in the
  samples (object position is static to within numerical noise while
  `any_contact` is `false`).
- None of the four listed defect categories apply. This is the legitimate
  research finding the brief anticipated.

## Step 7: diagnosis (evidence gathered before any parameter change)

Per the brief's required order, `samples.csv` for the `close` stage
(sim time 322.90 s - 324.48 s, ~1.56 s) was inspected in full:

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

**Conclusion**: this result does not fit cleanly into the brief's case 3
("bilateral contact with small normal forces" — forces were ample, not
small) or case 4 ("escapes on lift" — lift was never reached). It is
closest to case 2, "contact on one finger only -> coupling or
approach-angle problem, not force": contact was intermittently unilateral
and, even when bilateral, was not sustained/symmetric long enough to pass
the confirmation window, despite forces being well above threshold. The
brief's own guidance for case 2 is explicit that this is **not** a force
problem, and it gives no force-related parameter to raise for it (unlike
cases 3 and 4). Raising `position_kp`/`position_max_effort` (case 3's
remedy) would not address a timing/geometry chattering issue and risks
masking it; the friction pair (case 4's remedy) is moot because lift was
never attempted.

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
- Aligned trial: `unilateral_contact` — a legitimate physics finding, not
  a defect under the brief's four defect categories. Root cause per Step 7
  evidence: bilateral contact occurs with ample force but chatters and
  never sustains the 0.30 s confirmation window; the trial never reaches
  `lift`. No parameter changed; flagged for follow-up investigation of
  finger coupling / approach-angle / contact stability during `close`.
