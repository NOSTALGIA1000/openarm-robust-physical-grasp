#!/usr/bin/env bash
# Relative (differential) visual correction: object and gripper both measured
# optically, so a hand-eye translation error cancels in their difference.
# Fresh launch per trial (a parked arm from a previous trial pollutes the
# camera view -- see the 2026-09-04 vision-in-loop documents).
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
TRIAL="$1"; export CAM_BIAS_X="${2:-0.0}"; export CAM_BIAS_Y="${3:-0.0}"
OUTDIR="${4:-stage4_relative}"; OFFX="${5:-0.0}"; OFFY="${6:-0.0}"
export DETECTOR_UNLATCHED=1
export DETECTOR_CAM_FRAME=1
source "$ROS_SETUP"
source "$OPENARM_INSTALL/setup.bash"
OUT="$OPENARM_RESULTS/$OUTDIR"
# Logs live with the trial they belong to, not in /tmp. Everything written to
# /tmp is lost on the next reboot, and the per-trial detector traces and picker
# reasoning for three completed batches went that way before anyone noticed -
# the trial records survived because they were always written here.
TRIALDIR="$OUT/$TRIAL"
mkdir -p "$TRIALDIR"
LOG="$TRIALDIR/launch.log"
PICKERLOG="$TRIALDIR/picker.log"
bash "$EXPDIR/preflight.sh" 1024 >/dev/null || { echo "$TRIAL preflight_abort"; exit 5; }
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &
for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
for k in $(seq 1 60); do ros2 action list 2>/dev/null | grep -q '/move_action' && break; sleep 2; done
sleep 6
strip(){ sed 's/\x1b\[[0-9;]*m//g'; }
is_active(){ ros2 control list_controllers 2>/dev/null | strip | awk -v c="$1" '$1==c && $3=="active"{f=1} END{exit !f}'; }
for a in 1 2 3 4 5; do
  ok=1
  for c in joint_state_broadcaster left_arm_controller left_hand_controller; do
    if ! is_active "$c"; then ok=0
      if ros2 control list_controllers 2>/dev/null | strip | awk -v c="$c" '$1==c{f=1} END{exit !f}'; then
        ros2 control set_controller_state "$c" inactive >/dev/null 2>&1; sleep 1
        ros2 control set_controller_state "$c" active >/dev/null 2>&1
      else
        ros2 control load_controller --set-state active "$c" >/dev/null 2>&1
      fi
    fi
  done
  [ "$ok" = 1 ] && break; sleep 3
done
for c in joint_state_broadcaster left_arm_controller left_hand_controller; do
  is_active "$c" || { echo "$TRIAL controller_abort:$c"; exit 2; }
done
timeout 400 ros2 run openarm_grasp_physics physics_picker --ros-args \
  -p use_sim_time:=true -p trial_id:="$TRIAL" -p output_dir:="$OUT" \
  -p use_detector_target:=true -p relative_correction:=true \
  -p rel_offset_x:="$OFFX" -p rel_offset_y:="$OFFY" \
  -p view_gain_gripper:="${VIEW_GAIN_GRIPPER:-0.0}" \
  -p view_gain_object:="${VIEW_GAIN_OBJECT:-0.0}" \
  > "$PICKERLOG" 2>&1
echo "INSTABILITY=$(grep -c 'unstable\|QACC' "$LOG")"
grep -E 'relative correction|trial .* ->' "$PICKERLOG" | tail -3
grep -m3 'blobs:' "$LOG" | tail -2
# Compress once the greps above have run: a launch log is about 76KB raw and
# under 10KB gzipped, so a full batch costs well under a megabyte to keep.
gzip -f "$LOG" "$PICKERLOG" 2>/dev/null || true
