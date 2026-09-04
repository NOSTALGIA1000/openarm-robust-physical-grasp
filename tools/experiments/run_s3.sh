#!/usr/bin/env bash
# Stage 2 with real detector: calibration bias injected at the camera->world
# extrinsic (CAM_BIAS_X/Y), not synthesised after ground truth. The detector's
# own real measurement noise rides along automatically. One fresh launch per
# trial is mandatory here -- a reused instance's parked retreat pose occludes
# the camera for the next trial's read (see 2026-09-04 vision-in-loop doc).
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
TRIAL="$1"; export CAM_BIAS_X="${2:-0.0}"; export CAM_BIAS_Y="${3:-0.0}"; OUTDIR="${4:-stage3_closedloop}"; export DETECTOR_UNLATCHED=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
LOG="/tmp/physics_launch_${TRIAL}.log"
OUT="$HOME/ros2_ws_course_repro/results/physics_grasp/$OUTDIR"
mkdir -p "$OUT"
bash /tmp/preflight.sh 1024 >/dev/null || { echo "$TRIAL preflight_abort"; exit 5; }
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
timeout 600 ros2 run openarm_grasp_physics physics_picker --ros-args \
  -p use_sim_time:=true -p trial_id:="$TRIAL" -p output_dir:="$OUT" \
  -p use_detector_target:=true -p pregrasp_redetect:=true \
  > "/tmp/picker_${TRIAL}.log" 2>&1
echo "INSTABILITY=$(grep -c 'unstable\|QACC' "$LOG")"
grep -E 'pre-grasp re-detection|vision target|trial .* ->' "/tmp/picker_${TRIAL}.log" | tail -3
