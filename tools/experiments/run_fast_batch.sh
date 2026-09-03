#!/usr/bin/env bash
# Fast batch: one launch, object restored between trials via /reset_object.
# MUST be validated against the relaunch-per-trial baseline before being trusted
# — reusing an instance is exactly the shortcut that produced contaminated
# results earlier in this project.
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
N="${1:-5}"; OUTDIR="${2:-fastbatch}"
LOG=/tmp/physics_launch_fast.log
OUT="$HOME/ros2_ws_course_repro/results/physics_grasp/$OUTDIR"
mkdir -p "$OUT"

bash /tmp/preflight.sh 1024 >/dev/null || exit 5
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
  is_active "$c" || { echo "controller_abort:$c"; exit 2; }
done

START=$(date +%s)
for i in $(seq 1 "$N"); do
  T=$(printf "fb%02d" "$i")
  timeout 20 ros2 service call /reset_object openarm_grasp_msgs/srv/ResetObject "{use_spawn: true}" >/dev/null 2>&1
  sleep 2
  timeout 600 ros2 run openarm_grasp_physics physics_picker --ros-args \
    -p use_sim_time:=true -p trial_id:="$T" -p output_dir:="$OUT" > "/tmp/picker_${T}.log" 2>&1
  grep -oE 'trial .* -> .*' "/tmp/picker_${T}.log" | tail -1
done
END=$(date +%s)
echo "elapsed=$((END-START))s for $N trials -> $(( (END-START)/N ))s per trial"
echo "instability=$(grep -c 'unstable\|QACC' "$LOG")"
