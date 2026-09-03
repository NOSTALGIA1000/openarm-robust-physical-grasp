#!/usr/bin/env bash
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
TRIAL="$1"
LOG="/tmp/physics_launch_${TRIAL}.log"
OUT="$HOME/ros2_ws_course_repro/results/physics_grasp/lift_diagnostic"
# Stage 1 is left-arm only; the right arm plays no role in the grasp.
NEED="joint_state_broadcaster left_arm_controller left_hand_controller"

strip() { sed 's/\x1b\[[0-9;]*m//g'; }
is_active() { ros2 control list_controllers 2>/dev/null | strip | awk -v c="$1" '$1==c && $3=="active"{f=1} END{exit !f}'; }

bash /tmp/preflight.sh 1024 || { echo "ABORT: preflight failed"; exit 5; }
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &

for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
for k in $(seq 1 60); do ros2 action list 2>/dev/null | grep -q '/move_action' && break; sleep 2; done
sleep 6

# The launch's load_controller chain races under load and leaves controllers
# unconfigured while still advertising their action servers, which makes the
# picker block forever in wait_for_server. Walk them to active explicitly.
for attempt in 1 2 3 4 5; do
  ALLOK=1
  for c in $NEED; do
    if ! is_active "$c"; then
      ALLOK=0
      # The launch's load_controller processes can die with !rclpy.ok(), leaving
      # the controller not merely inactive but entirely absent. Load it first if
      # it is missing, then walk unconfigured -> inactive -> active.
      if ! ros2 control list_controllers 2>/dev/null | strip | awk -v c="$c" '$1==c{f=1} END{exit !f}'; then
        ros2 control load_controller --set-state active "$c" >/dev/null 2>&1
      else
        ros2 control set_controller_state "$c" inactive >/dev/null 2>&1
        sleep 1
        ros2 control set_controller_state "$c" active   >/dev/null 2>&1
      fi
    fi
  done
  [ "$ALLOK" = "1" ] && break
  sleep 3
done

echo "=== controller states ==="
ros2 control list_controllers 2>/dev/null | strip | awk '{printf "   %-26s %s\n", $1, $3}'

for c in $NEED; do
  is_active "$c" || { echo "ABORT: $c not active"; exit 2; }
done
echo "required controllers active"

timeout 600 ros2 run openarm_grasp_physics physics_picker --ros-args \
  -p use_sim_time:=true -p trial_id:="$TRIAL" -p output_dir:="$OUT" > /tmp/picker_${TRIAL}.log 2>&1
echo "PICKER_RC=$?"
grep -E 'stage ->|trial .* ->|baseline' /tmp/picker_${TRIAL}.log | tail -12
echo "INSTABILITY=$(grep -c 'unstable\|QACC' "$LOG")"
