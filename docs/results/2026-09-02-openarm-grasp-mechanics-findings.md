# OpenArm Physics-Mode Grasp — Mechanics Findings

Date: 2026-09-02
Status: **Stage 1 grasp not yet achieved.** This document records the model and
control defects found while debugging it, and the one unresolved contradiction
that blocks progress.

All trials ran in `physics_mode` (free rigid body, no software attachment) on the
isolated course reproduction workspace. Raw per-trial data is under
`results/physics_grasp/lift_diagnostic/` on the VM.

---

## 1. The course demo's gripper could never have gripped anything

Measured jaw kinematics from the model:

```
separation = 0.012 + 2q   metres
```

The original course `pick_apple.py` closes with `position = 0.03`, giving a
**72 mm** jaw against a **50 mm** object. The fingers never reach it. The demo's
apparent grasp was produced entirely by the software attachment
(`is_attached_*` / `teleport_cube()`).

This is independent corroboration of the Stage 1 premise, obtained from the
gripper geometry alone rather than from the attachment code.

## 2. The plan commanded a crush, not a grip

Stage 1's `close_position` was `0.0` — a **12 mm** jaw, i.e. driving 38 mm past
the surface of a 50 mm object. The gripper PID sits pinned at its force clamp for
the entire closure. On a smooth object this ejects rather than grasps.

Contact begins at `q = 0.019` for a 50 mm sphere and `q = 0.014` for a 40 mm cube.

## 3. Ejection was caused by closing speed, not clamp force

| change | ejection speed |
|---|---|
| baseline | 0.43 m/s |
| clamp force 20 N → 3 N | no improvement (slightly worse) |
| finger joint damping 0.2 → 100 | **0.068 m/s** (6× reduction) |

Lowering force alone does not help because the failure is an impulsive collision:
sampled contact force reached 16.9 N against a 5 N actuator clamp, i.e. the
recorded force is collision impulse, not quasi-static squeeze. Telemetry sampled
every 10 physics steps largely *misses* the impulse — logs show `0.00 N`
alongside a violent launch.

## 4. MJCF finger origins were asymmetric — a stale-export bug

| finger | URDF / canonical | MJCF (as shipped) |
|---|---|---|
| right (`joint1`) | −0.006 | **0.000** |
| left (`joint2`) | +0.006 | **+0.012** |

Both fingers were shifted +6 mm as a pair, so the **jaw centre sat 6 mm off the
hand frame**. MoveIt plans against the URDF and believed the jaw was centred;
the physics ran the MJCF where it was not. One finger therefore always reached
the object first, pushed it, and spun it out.

The MJCF was exported from an older revision of the description:
`openarm_description/config/hand/openarm_hand/kinematics_link.yaml` still carries
the old asymmetric values (`0.0775` / `0.143`) as commented-out lines above the
current symmetric ones (`−0.05` / `+0.05`).

**Fixed** on both arms. Verified statically: `jaw_mid − hand_y = 0.000000` at
every opening, with `separation = 0.012 + 2q` preserved. Descend misalignment
improved 11.3 mm → 5.7 mm → 3.6 mm and finger contact counts became balanced
(L=32/R=37, previously L=0/R=22).

## 5. The finger pads were colliding with the table

At the commanded grasp height the pad bottom sat at **0.4217** against a table
top of **0.4200** — the fingers were jammed into the table and could not close
cleanly. Raising the grasp target restored 8.8 mm clearance.

## 6. The descent was spinning the object before the gripper closed

Logging the object quaternion resolved the failure that had survived every
alignment fix:

| stage | object yaw |
|---|---|
| open (entire) | 0.00° |
| descend, t≈2.7 s | **11.88°** ← struck here |
| close (end) | −16.55° |

Tilt stayed ~0°, so the object was not tipping — it was being **spun about
vertical by the descending fingers**. Closing on a yawed box then strikes its
corners and cams it out (~+43 mm). MoveIt plans the descent in joint space, so
the TCP does not travel a straight vertical line.

**Attempted fix that failed:** splitting the descent into 6 waypoints made it
*much* worse (object spun to 159°). Each waypoint is an independent plan free to
take its own joint-space route, so more waypoints means more sweeping. Do not
retry this.

**Working fix:** widen the pre-shape opening to 72 mm (16 mm clearance per side).
The object is then **completely undisturbed** through the descent — yaw
`0.00 → 0.00`, dx and dy exactly zero.

---

## Remaining blocker: run-to-run alignment variance

With the descent disturbance eliminated, the failure is isolated to the close
phase. The cause is **variance** in lateral alignment, not its magnitude:

| trial | descend-end `dy` |
|---|---|
| `gentle_close` | 0.1 mm |
| `wideopen` | 6.05 mm |
| `jawfix02` | 12.3 mm |

Identical configuration, different results. With a 38 mm jaw on a 40 mm cube
there is only 1 mm of interference per side, so a 6 mm run lands one finger 6 mm
early → push → yaw → cam-out, while a 0.1 mm run nearly succeeds.

Closing deeper tolerates the variance but reintroduces the crush of §2. That
tension cannot be tuned away — the variance itself has to be removed.

## Unresolved: a 25 mm planning/physics model contradiction

Queried from the running system (`ros2 param get /move_group robot_description`),
MoveIt plans with the flattened bimanual URDF. The real parent joint of the hand
— whose name still contains an unexpanded xacro placeholder,
`${ee_prefix}hand_joint`, `parent=openarm_left_link7`, `child=openarm_left_hand`
— has origin `xyz = "0 -0.0 0.1001"`, i.e. **y = 0**.

The MJCF places that same body at **y = −0.025**.

Arm links 0–7 match in both position and rotation, so link7's frame is identical
in both models, and at the grasp orientation the TCP offset is purely vertical.
MuJoCo's hand should therefore coincide with MoveIt's when the MJCF is set to
y = 0 — but setting it to 0 moved `dy` from +3.6 mm to **−22.2 mm** (worse), and
−0.025 is empirically what makes alignment work.

This is a genuine unexplained inconsistency and the most likely source of the
`dy` variance, since different IK wrist configurations would project the
mismatch differently.

**Next step — do not guess at this again.** It needs a direct forward-kinematics
comparison: feed identical joint vectors to both models and compare the resulting
`openarm_left_hand` world pose. That isolates the discrepancy without running a
simulation.

---

## Method notes worth keeping

- **MuJoCo can silently auto-reset mid-trial.** A diverging sim logs
  `WARNING: Nan, Inf or huge value in QACC ... The simulation is unstable` and
  resets `mj_data->time` to 0. The trial still produces a well-formed
  `trial.json` with a plausible category — 66259 of 67240 samples in one run were
  post-reset fiction and nothing detected it. Any acceptance run **must** assert
  `sim_time` monotonicity per trial and grep the launch log for `unstable`/`QACC`.
- **One launch per trial.** A reused 6-hour-old instance produced the divergence
  above. The scene has no reset service; object pose only restores on relaunch.
- **Check VM load before trusting timing-sensitive results.** Load average
  reached 121.9 during one batch; launches took 360 s instead of 60 s and MoveIt
  returned spurious planning failures indistinguishable from real ones.
- **`friction` in MuJoCo combines element-wise `max`.** The table's μ=0.05 never
  applied because the object's μ=1.0 governed — worth checking before "fixing"
  a surface property.

---

# Session 2 (2026-09-03): model agreement achieved, grip forms, extrusion remains

## 7. The 25 mm model contradiction — resolved

The forward-kinematics comparison called for at the end of session 1 was run:
identical joint vectors fed to both models, comparing the resulting
`openarm_left_hand` world pose.

Links 0–7 matched to **machine precision**. The discrepancy appeared only at the
hand, and was **4.5 mm, not 25 mm** — and it *varied with pose*, which is what
made run-to-run alignment inconsistent.

Cause: the MJCF has an intermediate body the URDF does not.

```
openarm_left_link8   pos="1e-06 0.0205 0.0"      <- absent from the URDF
  openarm_left_hand  pos="0 -0.025  0.1001"
```

Net MJCF offset = `0.0205 − 0.025` = **−0.0045 m**, exactly the measured delta.
The URDF's is 0. This also explains why session 1's attempted fix (setting the
hand to y=0) made things *worse*: it produced a net `+0.0205`, matching the
−22.2 mm that was observed. The wrong term was being corrected.

**Fix:** hand y `−0.025 → −0.0205` on both arms, making the net exactly zero.
Verified: FK delta is now `[0, 0, 0]` at every test pose, and measured descend
misalignment fell to **0.07–0.5 mm**, repeatably.

## 8. Contact margin — the dominant physics defect

The object geom carried `margin="0.005"` — a 5 mm contact margin, inherited from
the sphere and never revisited. Contacts activated 5 mm before surfaces met, so
the object floated in a force cushion.

| | margin 0.005 | margin 0 |
|---|---|---|
| `table_top` max force | 22.27 N | **0.64 N** |
| settled height | 0.4450 (floating) | **0.4400** (on the table) |
| slide during close | +43 to +50 mm | **+12.6 mm** |
| yaw change | −16° | **−0.15°** |

0.64 N across 4–5 corner contacts is correct for a 0.98 N object. Rotation and
cam-out were essentially eliminated by this one change.

## 9. Object must be smaller than the finger pad

The pads are 40 × 4 × 40 mm. With a 40 mm cube the two are the same size, so a
~2 mm lateral offset puts a cube corner exactly at the pad edge, producing an
oblique contact normal with an x-component. Measured: at first contact the cube
received a +2 mm x-impulse and then coasted 44 mm away while the arm was
provably stationary (`grip_x` constant to 0.4 mm).

Shrinking the object to 30 mm so it sits wholly inside the pad face:

| | 40 mm cube | 30 mm cube |
|---|---|---|
| net x-slide during close | +44 mm | **+0.4 mm** |
| yaw change | 11.4° | **−0.21°** |

## 10. Grip force is set by position error, not by the clamp

With `position_kp` lowered to 300 during ejection debugging, commanding 3 mm of
overshoot yields only `300 × 0.003` = **0.9 N** — barely above the 0.5 N contact
threshold, so contact flickered and never confirmed. The low gain had been
introduced to fight an ejection that geometry and margin fixes had since removed.

Restoring `position_kp = 2000` (clamp still bounded at 5 N):

| | kp 300 | kp 2000 |
|---|---|---|
| bilateral frames | 8 / 287 | **136 / 268** |
| longest bilateral run | 0.02 s | **0.159 s** (need 0.30) |
| contact force | 1.3 N | **3–9 N both fingers** |
| object lift during close | — | **+5.7 mm** |

A genuine two-finger grip now forms and lifts the object.

## Remaining failure: extrusion from the jaw

The grip forms but the object squeezes forward-and-up out of the jaw
(+37.9 mm x, rising z) and is released. Cause is vertical: the 40 mm-tall pad
cannot centre on a 30 mm object resting on a table without its lower edge
penetrating the table, so the pressure centre sits ~6 mm above the object's
centre and squeezing produces an upward-forward moment.

Attempted remedy — a taller 30 × 30 × 50 mm object so the centres coincide —
**failed**: it is top-heavy on a 30 mm base and tips (bilateral fell back to
0.040 s). A 30 × 30 × 44 mm compromise was prepared but not yet measured; the VM
degraded before it ran.

**Next step:** measure the 30 × 30 × 44 mm object (centre 0.442, pad spans
0.422–0.462, clears the table by 2 mm, squat enough not to tip). If extrusion
persists, the honest conclusion is that this pad/object/table geometry cannot
produce a centred grasp, and the object should be raised on a pedestal so a
genuine cube can be gripped about its centre.

## Infrastructure defects found (would have corrupted an acceptance run)

- **Controllers silently fail to activate.** The launch's `load_controller`
  chain races under load and leaves `left_arm_controller` / `left_hand_controller`
  **unconfigured** while still advertising their action servers, so the picker
  blocks forever in `wait_for_server`. A trial runner must verify actual
  controller state and repair it, not grep a log line — the "Successfully loaded"
  line appears even on a failed retry.
- **`grep "active"` matches `inactive`.** Cost one wasted trial; compare the
  state field exactly.
- **Stale DDS shared memory** accumulates across killed runs (279 segments
  observed) and corrupts `ros2 control list_controllers` output. Clear
  `/dev/shm/fastrtps*` between batches.
- **VM resource exhaustion is the top confound.** Load average reached 121 and
  later 45 with 141 MB free; launches stretched 60 s → 360 s and MoveIt returned
  spurious planning failures. Always check `uptime` and `free` before trusting a
  timing-sensitive result.
- **The detector is not a reliable Stage 1 trigger.** It cannot see a 30 mm
  object and silently starves the picker, which then reports `planning_failure`.
  Since Stage 1 is scoped to mechanics with ground-truth targeting, the detector
  was made a non-blocking trigger; Stage 2 re-introduces perception deliberately.
