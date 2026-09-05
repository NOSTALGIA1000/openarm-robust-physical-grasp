#!/usr/bin/env bash
# Side-by-side demo: same injected hand-eye error, open loop vs optical
# relative correction.
#
# Viewer period is 0.2s (5fps). Experiment batches use 1.0s to save resources;
# 0.05s (20fps) was tried once and drove this 4-core VM into a kernel soft
# lockup, so the frame rate is a real constraint here, not a cosmetic choice.
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
TRIAL="$1"; MODE="$2"; export CAM_BIAS_X="${3:-0.0}"; export CAM_BIAS_Y="${4:-0.0}"
FPS="${5:-0.2}"
export DETECTOR_UNLATCHED=1
export DETECTOR_CAM_FRAME=1
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
LOG="/tmp/physics_launch_${TRIAL}.log"
OUT="$HOME/ros2_ws_course_repro/results/physics_grasp/demo"
mkdir -p "$OUT"
bash /tmp/preflight.sh 1024 >/dev/null || { echo "$TRIAL preflight_abort"; exit 5; }
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py \
  viewer_render_period:="$FPS" > "$LOG" 2>&1 < /dev/null &
for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
for k in $(seq 1 60); do ros2 action list 2>/dev/null | grep -q '/move_action' && break; sleep 2; done
sleep 6
strip(){ sed 's/\x1b\[[0-9;]*m//g'; }
is_active(){ ros2 control list_controllers 2>/dev/null | strip | awk -v c="$1" '$1==c && $3=="active"{f=1} END{exit !f}'; }
for a in 1 2 3 4 5; do ok=1
  for c in joint_state_broadcaster left_arm_controller left_hand_controller; do
    if ! is_active "$c"; then ok=0
      ros2 control load_controller --set-state active "$c" >/dev/null 2>&1
      ros2 control set_controller_state "$c" active >/dev/null 2>&1
    fi
  done
  [ "$ok" = 1 ] && break; sleep 3
done
for c in joint_state_broadcaster left_arm_controller left_hand_controller; do
  is_active "$c" || { echo "$TRIAL controller_abort:$c"; exit 2; }
done
echo ">>> 仿真就绪，开始：$TRIAL（$MODE，y偏置=${4}m，viewer=${FPS}s/帧）"
if [ "$MODE" = "relative" ]; then
  EXTRA="-p relative_correction:=true -p rel_offset_x:=-0.0209 -p rel_offset_y:=-0.0007"
else
  EXTRA="-p relative_correction:=false"
fi
timeout 400 ros2 run openarm_grasp_physics physics_picker --ros-args \
  -p use_sim_time:=true -p trial_id:="$TRIAL" -p output_dir:="$OUT" \
  -p use_detector_target:=true $EXTRA > "/tmp/picker_${TRIAL}.log" 2>&1
grep -E 'stage ->|vision target|relative correction|residual|trial .* ->' "/tmp/picker_${TRIAL}.log" \
  | sed 's/\[INFO\] \[[0-9.]*\] \[physics_picker\]: //;s/\[ERROR\] \[[0-9.]*\] \[physics_picker\]: /[错误] /'
echo "INSTABILITY=$(grep -c 'unstable\|QACC' "$LOG")"
