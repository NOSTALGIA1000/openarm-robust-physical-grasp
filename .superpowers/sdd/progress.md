# Progress Ledger: OpenArm ROS2 Physical Grasp Stage 1

Plan: docs/superpowers/plans/2026-09-01-openarm-ros2-physical-grasp-stage1.md
Branch: stage1-physical-grasp
Base before Task 1: eede010

Task 1: complete (commits eede010..ab8fb4e, review clean/Approved)
  Minor findings deferred to final review: sync_from_vm.sh not executable (chmod +x); no retry/atomicity in sync loop (plan-mandated, inherited from brief); commit bundles pre-existing third-party mujoco_ros2_control mirrored files as side effect of running full sync script (expected per brief Step 9).
  Env fix applied to VM (not part of any task diff): pip3 install --user packaging==24.2 on nostalgia VM, fixes colcon ament_cmake_python egg-info TypeError (setuptools 78.1.0 vs system packaging 21.3 mismatch). User-local only, does not touch system dist-packages.

Plan fixes applied mid-run (both committed, both content-loss bugs from initial multi-part authoring, not review findings):
  - da7ba51 / 71e2238: restored a stray duplicate code-fence (71e2238) and a fully-missing Task 2 Step 3 (package.xml dependency edit, da7ba51) that were silently dropped when the plan file was assembled from three parts. Verified: fence parity even (226 total), step numbering contiguous in every task, no other seams damaged.

Standing VM environment note (NOT a plan edit — inject into every remaining dispatch's Context section instead, since these are VM shell-environment facts, not part of any task's deliverable):
  - Non-interactive `ssh -p 2224 nostalgia@127.0.0.1 "..."` sessions do NOT source ~/.bashrc (interactive-only guard), so two env vars needed by mujoco_ros2_control are absent unless exported inline:
    - `MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco` (needed by colcon build of mujoco_ros2_control whenever `find_package(mujoco QUIET)` doesn't resolve)
    - `DISPLAY=:0` (needed by mujoco_ros2_control's glfwInit() call whenever the node runs, i.e. every launch — VM's GNOME/Xwayland desktop session is already up at :0)
  - Confirmed both values against a prior successful build/session on this same VM (task-2-report.md). Also confirmed ~/.bashrc itself does NOT propagate them to fresh non-interactive ssh sessions even after appending — so every future task's colcon-build and ros2-launch commands need `export MUJOCO_DIR=... && export DISPLAY=:0 &&` prefixed explicitly, same as they already prefix `source /opt/ros/humble/setup.bash`.

Task 2: complete (commits 71e2238..7ca63bd, review clean/Approved after one fix)
  Fixed: gripper_body empty-string could silently match worldbody instead of failing loudly (Important, plan-mandated gap; user approved fixing now). Fix commit 7ca63bd, re-reviewed Approved.
  Minor findings deferred to final review: no unit tests for MujocoGraspTelemetry (no C++ test harness exists in this diff; not requested by brief).

Task 3: complete (commits 7ca63bd..f69fb14, review clean/Approved)
  Minor findings deferred to final review: pid_max_effort field lacks inline comment on its own declaration (rationale comment sits at the read site instead); no validation that position_max_effort is non-negative (mirrors existing unvalidated stod pattern for _kp/_ki/_kd/_i_max/_i_min in same file, not a new defect).

Task 4: complete (commits f69fb14..a1b758c, review clean/Approved)
  Minor findings deferred to final review: no lower-bound guard on viewer_render_period/camera_period (trusted launch-file config, not user input, low severity); one stale comment describing old hard-coded 60fps cadence left untouched (not in scope of this task's edit).

Task 5: complete (commits a1b758c..3087a4c, review clean/Approved)
  Minor findings deferred to final review: setup.py declares console_scripts entry physics_picker referencing a module that doesn't exist yet (plan-mandated, Task 12 adds it); tests_require deprecation warning from colcon (harmless, plan-mandated verbatim).

Task 6: complete (commits 3087a4c..f37270b, review clean/Approved, independently verified)
  Minor findings deferred to final review: dormant <default class="motor_finger"><position kp="5000"/></default> template still present though no live actuator references it now (Task 7 or later must ensure any future position-class finger actuator doesn't silently inherit this stale kp default).

Task 7: complete (commits f37270b..489cdce, review clean/Approved, independently verified against diff)
  Minor findings deferred to final review: table_scene_physics.urdf lacks trailing newline (cosmetic, tolerated by check_urdf).

Task 8: complete (commits 489cdce..a570681, review clean/Approved, all 9 tests traced against implementation)
  No new findings deferred.

Task 9: complete (commits b7001c2..bb3070a, review clean/Approved, rotation math hand-verified)
  Plan fix applied mid-run (committed b7001c2, user approved): two test assertions used exact == against computed floating-point rotation/subtraction results, which is not guaranteed bit-exact in binary64 (0.5-0.6 != -0.1). Switched to math.isclose. geometry.py implementation itself was correct throughout, untouched by the fix.
  Minor findings deferred to final review: _rotation_matrix has no unit-quaternion validation (matches brief's reference implementation, not a new defect; worth a docstring caveat before any future task feeds in unnormalized quaternions).

Task 10: complete (commits bb3070a..0908c38, review clean/Approved, precedence chain hand-traced against 8 test cases)
  No new findings deferred.

Task 11: complete (commits 0908c38..af19031, review clean/Approved, key trial.json/csv behaviors hand-traced)
  Plan fix applied mid-run (committed d1882b4, mechanical arithmetic correction, no user approval needed): stale cumulative test-count expectations "44 passed" corrected to "49 passed" in Task 11 Step 5 and Task 14 Step 4 (Task 14's changed to "at least 49" since Tasks 12-13 may add more).
  Minor findings deferred to final review: _write_report sorts stage_times by value not insertion order (harmless, times are naturally monotonic in practice); GraspConfig.snapshot()/TrialObservations JSON-serializability not independently re-verified here (covered by Task 5/10's own suites).

Task 12: complete (commits d1882b4..30739ff, review clean/Approved after two fixes, concurrency independently traced by opus reviewer)
  Fixed (both plan-mandated, user approved fixing now): (1) _confirm_bilateral_contact had no wall-clock safety bound, could hang the whole node forever against a dead simulator; (2) TrialRecorder.mark_stage/finish calls had no lock coordination with add_sample across threads. Fix commit 30739ff, re-reviewed Approved.
  Minor findings deferred to final review: unused imports (math, Pose) in physics_picker.py; unused PROTECTION_RADIUS constant; _on_telemetry holds lock for entire body including recorder.add_sample call (low practical risk); _relative/_summary not initialized in __init__ (dynamically created, defensively guarded, correct but inconsistent style).

Task 13: IN PROGRESS (commit cd8bd04 landed, review found Critical factual error, correction dispatched)
  CORE STAGE 1 CLAIM PROVEN: misaligned trial = 824 samples, 0 contacts, object x pinned at exactly 0.3, z at 0.44991653871789816, never moved. No attachment leak. Independently verified by controller.
  CRITICAL finding (reviewer caught, controller independently confirmed): aligned trial's "object never left the table" is FALSE. Object is EJECTED — accelerates laterally while bilaterally gripped (rows 455-466), contact drops at t=324.48, flies ballistically at ~0.4556 m/s, falls off table (z 0.4499 -> 0.0299), travels ~4.2m. Final elevation -0.42m. Implementer's diagnosis inspected only 322.90-324.48s = first 23% of the actual close stage (322.90-329.56s per trial.json stage_times), stopping right before the ejection.
  REAL STAGE 1 MECHANISM: squeeze-ejection. Gripper makes strong bilateral contact (8-17N) then squirts the 25mm sphere out. Two rigid parallel fingers on a smooth sphere convert closing force to lateral launch velocity. Also: object shows no deceleration over ~9s floor travel => near-zero effective friction.
  METRIC BLIND SPOTS discovered: max_drift_m stays 0.0 pre-grasp-confirmation (only tracks after _grasp_relative set); max_elevation_m is upward-only so a 42cm fall reads as ~0. Neither can detect object escape before confirmation. Task 14 must not rely on them alone.
  USER DECISION: attempt ejection fix before running Task 14 acceptance (design spec permits contact-parameter corrections in Stage 1 when diagnostics identify a cause and each change is recorded — cause is now identified).
  Task 13 CORRECTED and complete (commits 30739ff..0893d13). Doc fix 0893d13 annotates rather than silently rewrites the original false claims — correct handling for a research record.
  Minor findings deferred to final review: no fresh `ros2 node list` shown immediately before the aligned_check run (cleanup demonstrated earlier, asserted for that step); "close stage" window label in report describes the contact-active sub-window not the full stage span.
  VM STATE CHANGE (surface to user in final summary): implementer killed a ~1-day-old leftover ROS2 process tree from an unrelated prior session (bringup_all.launch.py + pick_apple.py + duplicate move_group, pids 37253/37274/37276/37278/37280/37311/37480) that was contaminating the graph via shared ROS_DOMAIN_ID=71 (stray move_group answering move_action goals, pick_apple.py publishing /cube_controller/commands). Necessary for valid measurement; final trials confirmed run against a clean graph.
  Also noted: misaligned trial exited code 250 (shutdown-time C++ exception in teardown) instead of 1; trial data itself verified complete and uncorrupted (824 rows, 14 fields each, no truncation).

EJECTION INVESTIGATION: complete (commit 9950623), user-approved detour before Task 14.
  Controller's primary hypothesis (grasp depth, grasp_z_offset_m 0.0 vs demo's -0.02) was REJECTED with evidence — bracketed -0.03/-0.02/0.0, identical squeeze-ejection at all three depths. Controller's brief also wrongly asserted condim=3 governs finger contact; investigator showed the live geoms were already condim=4/friction 1.0 isotropic (the condim=3 values exist only in a commented-out legacy mesh-geom block). Both controller errors corrected by the investigator.
  ACTUAL FIX: openarm_bimanual_physics.xml, both left finger collision geoms: condim 4->6, friction "1.0 1.0 1.0" -> "2.0 1.0 0.01". grasp_z_offset_m and gripper PID restored to baseline after testing (net change is the two geoms only).
  RESULT: catastrophic ejection eliminated. Object stays on table (z range 0.4461-0.4510, never reaches 0.0299 floor). bilateral_confirmed=true for the FIRST time — reaches lift and hold stages. Reproducible across 2 runs.
  RESIDUAL FAILURE: category contact_lost_during_lift. max_elevation 0.0011m (vs 0.020 required), max_drift 0.0917m (vs 0.010 allowed). Left finger loses contact ~1.7s into lift; object never rises. Investigator flagged as coupling/symmetry issue under lift acceleration, not addressable by further friction/depth/PID tuning.
  CONTROLLER HYPOTHESIS for residual (unverified): Stage 1 keeps arm joints on direct qpos teleport (no actuator control), so lift imposes kinematically-forced, unphysical accelerations the force-controlled gripper cannot track. Design spec already defers actuator-level arm control to Stage 3 and the plan's own caveats note arm joint velocities are not physical. If true, this residual is structural to Stage 1, not a tuning target.
  NO-REGRESSION CONFIRMED: misaligned control re-verified PASS — no_contact, object pinned bit-identical at (0.3, 0.0, 0.44991653871789816) across 1315 samples.
  METHODOLOGY FINDING (critical for Task 14): MuJoCo scene has NO reset service. Object pose only restores on full launch restart. Any trial ending with the object displaced contaminates every subsequent trial against the same launch instance (one trial was invalidated this way: settle read baseline z=0.0299, the floor). Task 14's acceptance script already relaunches per trial, which handles this — but it must not be "optimized" into a single shared launch.
  Softer PID was tested as a secondary candidate and made things worse (reintroduced ejection, 2.95m drift); reverted.

CONTROLLER ERROR + NEW CRITICAL FINDING (slow-lift diagnostic, first attempt INVALID):
  My mistake: to save a relaunch I ran the slow-lift diagnostic against a launch instance that had been running ~6h (started 13:44, 5h44m CPU). This violated the methodology finding already recorded in this very ledger (fresh launch per trial). Result was invalid and I discarded it.
  NEW FINDING (more important than the diagnostic): MuJoCo can go numerically unstable and SILENTLY AUTO-RESET mid-trial. Evidence: /tmp/physics_launch_trial8.log contains
    "WARNING: Nan, Inf or huge value in QACC at DOF 0. The simulation is unstable. Time = 6483.0300."
  and samples.csv has exactly one backward sim_time step at index 1146: 6483.02 -> 0.01. Post-reset samples show the object respawned at its settle pose with the arm snapped back, producing a plausible-looking but entirely fictitious "lift" trace (66259 of 67240 samples were post-reset garbage; stage_times still read as a normal sequence). The trial STILL produced a well-formed trial.json with a category, elevation and drift. Nothing in the pipeline detected it.
  Scope check: grepped all nine /tmp/physics_launch*.log — only trial8 (my stale-instance run) has this warning. Trials 1-7 (fresh launches) are clean, so no previously reported result is retroactively invalidated.
  MANDATORY FOR TASK 14: the acceptance script must assert sim_time monotonicity within each trial and discard+rerun any trial containing a backward step, and must grep its launch log for "unstable"/"QACC". Without this guard a diverged trial is indistinguishable from a real one in the recorded data. This is a genuine gap in the current evaluation pipeline, not just an operational caution.

SLOW-LIFT DIAGNOSTIC (clean rerun, trial slowlift_clean): VALID (1659 samples, 0 backward steps, 0 instability warnings, monotonic 159.18->192.34).
  RESULT: MY ACCELERATION HYPOTHESIS IS REFUTED. 10x slower lift (velocity/accel scale 0.05 -> 0.005) did NOT help; it was marginally worse.
    normal lift: max_elevation 0.0011 m, max_drift 0.0917 m, contact_lost_during_lift
    slow lift:   max_elevation 0.0000 m, max_drift 0.1421 m, contact_lost_during_lift
  So the residual failure is NOT driven by unphysical lift acceleration from the Stage 1 qpos-teleport arm path. My earlier "structural to Stage 1, needs Stage 3 actuator control" claim is NOT supported by this evidence and should not be reported as a finding.
  WHAT THE TRACE ACTUALLY SHOWS (slowlift_clean): during lift, gripper_z rises 33.8 mm (0.5497->0.5835) while object_z is flat (0.4491->0.4492, elevation pinned at -0.0008). Fingers slide up the object without lifting it. Contact force goes strongly asymmetric: left 10.30 -> 0.00 N (lost at dt=5.22 s), right 8.37 -> 24.37 N. During close the object also slides +40 mm in x (0.3000 -> 0.3398) while bilaterally gripped -- same squeeze mechanism as the original ejection, now bounded by the friction fix rather than eliminated.
  NEW GEOMETRIC HYPOTHESIS (supported, not yet confirmed): the grasp is too shallow. Detector reports z_surface = 0.47; grasp_z_offset_m = 0.0 so TCP descends to 0.47. Hand frame sits ~0.100 m above object center and TCP is 0.08 m below the hand => TCP ~= 0.467. Object center is 0.4499, sphere top is 0.4749. So the fingers pinch ~17 mm ABOVE the equator, where the sphere's local half-width is only sqrt(25^2-17^2) = 18.3 mm and the surface slopes away -- converting grip force into downward/lateral ejection instead of vertical friction. Explains both the +40 mm close-phase slide and the zero elevation under 8-24 N of normal force (0.1 kg needs only ~1 N of friction to hold).
  NOTE: this is the same grasp-depth hypothesis the ejection investigator REJECTED, but that rejection predates the condim/friction fix. Retesting -0.02 post-fix is justified; the original demo's own descend offset was GRIPPER_LENGTH = -0.02, which the plan dropped to 0.0.

=== GRASP DEBUGGING SESSION (controller-run, ~12 trials). Substantive findings, NOT yet committed to source. ===

CONFIRMED MECHANICAL FINDINGS (measured from the model, high confidence):
  1. Jaw kinematics: separation = 0.012 + 2q metres. So a 50 mm sphere first contacts at q=0.019; a 40 mm cube at q=0.014.
  2. THE PLAN COMMANDED A CRUSH, NOT A GRIP. close_position = 0.0 => 12 mm jaw separation, i.e. driving 38 mm PAST the object surface. PID sits pinned at its force clamp and ejects a smooth object.
  3. THE ORIGINAL COURSE DEMO COULD NEVER HAVE GRIPPED ANYTHING. Its close command was 0.03 => 72 mm separation, wider than the 50 mm object. The demo's "grasp" was 100% the software attachment. Independent corroboration of the whole Stage 1 premise; worth stating in the Stage 1 writeup.
  4. Jaw centre is offset a constant -6.0 mm in y from the openarm_left_hand frame (invariant across q; measured (0,+0.006,+0.005) m in TCP local frame). Aligning the hand to the object therefore leaves the object ~6 mm toward one finger, so that finger always contacts first.
  5. Finger collision geoms are 40x4x40 mm boxes centred 85 mm below the hand. At grasp height they clear the table top by only 2.6 mm.
  6. Closing SPEED, not force, caused the violent ejections. Raising motor_finger damping 0.2 -> 100 cut ejection speed 0.43 -> 0.068 m/s (6x). Lowering clamp force alone (20N -> 3N) did NOT help and slightly worsened it.

SPHERE VS CUBE (user approved sphere-first, cube fallback; fallback taken after 8 systematic sphere trials):
  Sphere: every configuration ended in ejection or unilateral contact. A smooth 25 mm sphere converts any asymmetry into lateral escape.
  Cube (40 mm, the scene's own commented-out variant): NO ejection. Both fingers contact (L=84/R=122 frames), a genuine bilateral moment appeared, and the object ROSE 7.7 mm during close before settling. Clearly the more tractable object; recommend keeping it for Stage 1.

UNRESOLVED BLOCKER:
  A systematic ~5-6 mm bias between commanded TCP y and achieved hand y persists even after tightening orientation tolerance 0.05 -> 0.01 rad (which only accounts for ~0.8 mm over the 80 mm TCP lever) and position tolerance 0.01 -> 0.003 m. Suspected MJCF-vs-URDF frame mismatch between the MuJoCo `openarm_left_hand` body and the URDF link MoveIt plans for. NOT confirmed. With a 36 mm jaw on a 40 mm cube each finger only overlaps 2 mm, so a 5-6 mm bias means the far finger never reaches the object -> permanent unilateral contact.

CRITICAL PROCESS FAILURE - RESULTS FROM THE LAST FEW TRIALS ARE UNRELIABLE:
  VM load average peaked at 121.94 (15-min) / 87.36 (5-min) - roughly 100x healthy - with only ~260 MB free RAM and systemd-oomd active. Under that starvation a launch took 360 s instead of 60 s, MoveIt returned spurious planning_failure, and the picker hung until its 900 s timeout. Trials cube03, cube04b, cube05, cube06 all ran during the build-up and their contact/timing behaviour cannot be distinguished from scheduling starvation. THEY MUST BE RE-RUN ON AN IDLE VM before any conclusion is drawn from them. VM has now been fully cleaned (only the ros2 daemon remains).

CONTROLLER ERRORS THIS SESSION (recorded so they are not repeated):
  - Ran a diagnostic against a 6 h old launch instance, violating the fresh-launch-per-trial rule recorded in this same ledger. Result was invalid and discarded.
  - Chained two separate ssh sessions for launch-then-trial; the second hung and burned an hour producing nothing. Replaced with a single self-contained VM-side script.
  - Nearly raised table friction on a wrong premise; MuJoCo combines friction element-wise MAX, so the object's mu=1.0 already governs and the table's 0.05 never applied. No change made.
  - Asserted the residual lift slip was "structural to Stage 1, needs Stage 3 actuator control". The 10x slow-lift test REFUTED this (elevation 0.0011 -> 0.0000 m, i.e. marginally worse). Do not report that claim.

UNCOMMITTED VM-SIDE CHANGES (working tree only, none mirrored or committed yet):
  config.py: close_position 0.0 -> 0.012, grasp_z_offset_m 0.0 -> -0.02, new jaw_offset_x/y/z fields (currently 0.0 / -0.0057 / 0.005)
  physics_picker.py: ground-truth object pose used for grasp target (user-approved), jaw-offset compensation, descend tolerance 0.01 -> 0.003, orientation tolerance 0.05 -> 0.01 rad
  table_scene_physics.urdf: position_kp 2000 -> 300, position_max_effort 20.0 -> 5.0
  openarm_bimanual_physics.xml: motor_finger damping 0.2 -> 100.0; object geom sphere r=0.025 -> box 0.02 (40 mm cube), spawn z 0.46 -> 0.45
  Sphere scene backed up at /tmp/table_scene_physics.xml.sphere

=== MODEL-INCONSISTENCY INVESTIGATION (user-approved static check). Two real bugs found; one fix kept, one reverted. ===

METHOD: compared MJCF body origins against URDF joint origins for the whole left-arm chain. All arm links 0-7 match exactly. Two discrepancies found at the wrist.

BUG A - FINGER ASYMMETRY (REAL, FIXED, KEPT):
  MJCF had left-hand fingers at y = 0.000 and +0.012; URDF and the canonical description have them symmetric at -0.006 / +0.006. Both arms affected. The MJCF was exported from a stale revision -- openarm_description/config/hand/openarm_hand/kinematics_link.yaml still carries the OLD asymmetric values as commented-out lines (0.0775 / 0.143) above the current symmetric ones (-0.05 / +0.05).
  Consequence: MoveIt planned believing the jaw was centred on the hand while the physics jaw sat 6 mm off, so one finger always reached the object first.
  FIX APPLIED to openarm_bimanual_physics.xml (both arms): finger body y -> -0.006 / +0.006. Verified statically: jaw_mid - hand_y is now exactly 0.000000 at every opening, and separation = 0.012 + 2q is preserved.
  EMPIRICAL EFFECT: descend dy improved 11.3 mm -> 5.7 mm -> 3.6 mm, and finger contact counts became balanced (L=32/R=37, previously L=0/R=22 or L=84/R=122).

BUG B - HAND ORIGIN (REVERTED; MY OVER-REACH):
  MJCF has openarm_left_hand at y = -0.025; the flattened table_scene_physics.urdf and the canonical yaml both say y = 0. I "fixed" the MJCF to 0 and it made alignment much WORSE (dy went from +3.6 mm to -22.2 mm, a ~25 mm shift exactly matching the edit).
  WHY I WAS WRONG: move_group does NOT plan from table_scene_physics.urdf. It builds from config/openarm.urdf.xacro, and that file with default args generates a SINGLE-ARM model whose links are openarm_hand / openarm_hand_tcp / openarm_left_finger -- it does not even contain openarm_left_hand_tcp, the link our position constraints target. So the canonical yaml and the flattened URDF are NOT authoritative for what MoveIt actually plans with; the empirical agreement (MJCF -0.025 <-> MoveIt) is.
  REVERTED to -0.025 on both arms. Lesson: verify what the running system loads before trusting a config file that merely looks canonical.

REMAINING UNSOLVED BEHAVIOUR (cube, 40 mm box):
  With alignment now essentially perfect (descend dy as low as -0.15 mm, dx 0.1 mm), gentle 2 mm interference (close q=0.013 -> 38 mm jaw), pre-shaped approach (open q=0.022 -> 56 mm jaw instead of 122 mm), damped slow fingers, and verified 8.8 mm table clearance, the cube STILL slides ~+43 mm in +x during close and bilateral contact never persists (BOTH <= 12 frames, longest run 0.02-0.04 s vs the 0.30 s required).
  Also: the cube is already displaced 3.3 mm in x / 4.5 mm in y by the END OF DESCEND, before closing begins -- so the descend itself disturbs it. MoveIt plans descend in joint space, so the TCP does not travel a straight vertical line and the fingers can sweep through the object.
  NEXT HYPOTHESES (untested, in priority order):
    1. Cube rotation. A box nudged during descend rotates about z; its effective width along the closing axis then exceeds 40 mm (up to 56.6 mm on the diagonal), so the fingers strike its CORNERS and cam it out. Sample does not log orientation -- add quaternion to the sample record to confirm or kill this.
    2. Non-Cartesian descend. Use a Cartesian/straight-line descend (or a path constraint) so the fingers do not sweep laterally through the object on the way down.
  Table-clearance fix (jaw_offset_z 0.005 -> 0.013) is sound and should be kept regardless.

STATE OF UNCOMMITTED VM EDITS (still working-tree only, nothing mirrored/committed):
  config.py: open_position 0.055->0.022, close_position 0.0->0.013, jaw_offset_x/y/z = 0.0/0.0/0.013, grasp_z_offset_m now DEAD (physics_picker computes grasp_z from obj_z + jaw_offset_z)
  physics_picker.py: ground-truth grasp target, jaw-offset compensation, descend tol 0.01->0.003, orientation tol 0.05->0.01 rad
  table_scene_physics.urdf: position_kp 2000->300, position_max_effort 20.0->5.0
  openarm_bimanual_physics.xml: motor_finger damping 0.2->100.0, finger y symmetric (BUG A fix), hand y left at -0.025 (BUG B reverted), object = 40 mm cube
  Backups on VM: /tmp/mjcf.pre_jawfix, /tmp/table_scene_physics.xml.sphere, /tmp/config.py.bak

=== HYPOTHESIS 1 TESTED AND CONFIRMED; DESCEND DISTURBANCE ELIMINATED ===
  Added object quaternion to Sample/telemetry logging to test the cube-rotation hypothesis.
  CONFIRMED: trial `rotcheck` shows the cube sits at yaw EXACTLY 0.00 deg through the whole open stage and the first ~2.2 s of descend, then jumps discontinuously to 11.88 deg mid-descend (tilt stays ~0, so it spins about vertical, it does not tip). It is then cammed out during close. Causal chain: descend sweeps laterally -> clips cube -> spins it ~12 deg -> closing on a yawed box strikes its CORNERS -> cams it out ~+43 mm. This explains the +x escape that survived every alignment fix.
  STEPPED DESCEND (descend_steps=6) MADE IT MUCH WORSE: cube spun to 159 deg during descend. Each step is an independent MoveIt plan free to take its own joint-space route, so more waypoints = more sweeping. Reverted to descend_steps=1. Recorded so this is not retried.
  ACTUAL FIX - WIDER PRE-SHAPE CLEARANCE: open_position 0.022 -> 0.030 (72 mm jaw vs 40 mm cube, 16 mm clearance per side). Trial `wideopen`: cube COMPLETELY undisturbed through descend - yaw 0.00 -> 0.00, dx = -0.0000, dy = -0.00000. Descend-phase disturbance fully eliminated.

REMAINING FAILURE (now isolated to the close phase alone):
  Cube still cams out during close (yaw 0 -> 10.7 deg, dx +49.8 mm), bilateral contact never persists (BOTH <= 6 frames, longest 0.02 s vs 0.30 s required).
  ROOT CAUSE IS dy VARIANCE, NOT dy MAGNITUDE. Across runs with identical config, descend-end dy ranged 0.1 mm (gentle_close) to 6.05 mm (wideopen) to 12.3 mm (jawfix02). With a 38 mm jaw on a 40 mm cube there is only 1 mm interference per side, so a 6 mm run lands one finger 6 mm early -> push -> yaw -> cam-out, while a 0.1 mm run nearly works. Deeper closure tolerates the variance but reintroduces crush/cam-out. That tension cannot be tuned away; the variance itself must be removed.

DEFINITIVE MODEL CONTRADICTION (unresolved, needs its own focused session):
  Queried the RUNNING move_group via `ros2 param get /move_group robot_description`. It plans with the flattened bimanual URDF (contains openarm_left_hand_tcp / openarm_left_left_finger). The REAL parent joint of the hand is the one whose name still contains an unexpanded xacro placeholder, `${ee_prefix}hand_joint`, parent=openarm_left_link7 child=openarm_left_hand, xyz = "0 -0.0 0.1001" => y = 0.
  The MJCF has that same body at y = -0.025. All arm links 0-7 match in both position AND rotation (quat vs rpy checked), so link7's frame is identical in both models, and the TCP offset is purely vertical at the grasp orientation. Therefore MuJoCo's hand should coincide with MoveIt's when the MJCF is set to y=0 - but empirically setting it to 0 moved dy from +3.6 mm to -22.2 mm (WORSE), and -0.025 is what makes alignment work.
  Genuine unexplained 25 mm inconsistency between planning model and physics model. Most likely source of the run-to-run dy variance (different IK wrist configurations project the mismatch differently). DO NOT guess at it again - it needs a dedicated forward-kinematics comparison: feed identical joint vectors to both models and compare the resulting openarm_left_hand world pose directly.
