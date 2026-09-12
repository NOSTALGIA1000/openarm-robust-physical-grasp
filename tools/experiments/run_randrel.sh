#!/usr/bin/env bash
# Relative (differential) visual correction: object and gripper both measured
# optically, so a hand-eye translation error cancels in their difference.
# Fresh launch per trial (a parked arm from a previous trial pollutes the
# camera view -- see the 2026-09-04 vision-in-loop documents).
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
TRIAL="$1"; DX="$2"; DY="$3"; YAW="$4"
export CAM_BIAS_X="${5:-0.0}"; export CAM_BIAS_Y="${6:-0.0}"
OUTDIR="${7:-stage8_randrel}"; OFFX="${8:-0.0}"; OFFY="${9:-0.0}"
export DETECTOR_UNLATCHED=1
export DETECTOR_CAM_FRAME=1
MODEL="$OPENARM_MODEL"
SCENE="_rand_${TRIAL}.xml"
export DETECTOR_UNLATCHED=1
export DETECTOR_CAM_FRAME=1
source "$ROS_SETUP"
source "$OPENARM_INSTALL/setup.bash"
LOG="/tmp/physics_launch_${TRIAL}.log"
OUT="$OPENARM_RESULTS/$OUTDIR"
# A cell topped up by raising the repeat count reruns from 1 and overwrites the
# records already there; see the note in run_rel.sh. Refuse; FORCE=1 to redo.
if [ -f "$OUT/$TRIAL/trial.json" ] && [ -z "${FORCE:-}" ]; then
  echo "trial $TRIAL -> already_recorded (set FORCE=1 to redo)"
  exit 0
fi
mkdir -p "$OUT"
bash "$EXPDIR/preflight.sh" 1024 >/dev/null || { echo "$TRIAL preflight_abort"; exit 5; }
python3 "$EXPDIR/add_pose_rand.py" 0 "$DX" "$DY" "$YAW" "$SCENE" >/dev/null || exit 6
cp "$MODEL/$SCENE" "$OPENARM_SHARE/model/$SCENE"
export PHYSICS_SCENE="$SCENE"
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
  > "/tmp/picker_${TRIAL}.log" 2>&1
echo "INSTABILITY=$(grep -c 'unstable\|QACC' "$LOG")"
grep -E 'relative correction|trial .* ->' "/tmp/picker_${TRIAL}.log" | tail -3
grep -m3 'blobs:' "$LOG" | tail -2
rm -f "$MODEL/$SCENE"
