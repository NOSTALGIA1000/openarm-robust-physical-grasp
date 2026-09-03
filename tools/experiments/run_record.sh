#!/usr/bin/env bash
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
TRIAL=jrec
LOG=/tmp/physics_launch_${TRIAL}.log
bash /tmp/preflight.sh 1024 >/dev/null || exit 5
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &
for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
for k in $(seq 1 60); do ros2 action list 2>/dev/null | grep -q '/move_action' && break; sleep 2; done
sleep 6
strip() { sed 's/\x1b\[[0-9;]*m//g'; }
is_active(){ ros2 control list_controllers 2>/dev/null | strip | awk -v c="$1" '$1==c && $3=="active"{f=1} END{exit !f}'; }
for a in 1 2 3 4 5; do
  ok=1
  for c in joint_state_broadcaster left_arm_controller left_hand_controller; do
    if ! is_active "$c"; then ok=0
      ros2 control list_controllers 2>/dev/null | strip | awk -v c="$c" '$1==c{f=1} END{exit !f}' \
        && { ros2 control set_controller_state "$c" inactive >/dev/null 2>&1; sleep 1; ros2 control set_controller_state "$c" active >/dev/null 2>&1; } \
        || ros2 control load_controller --set-state active "$c" >/dev/null 2>&1
    fi
  done
  [ "$ok" = 1 ] && break; sleep 3
done
python3 /tmp/record_joints.py > /tmp/jrec.log 2>&1 &
REC=$!
sleep 2
timeout 600 ros2 run openarm_grasp_physics physics_picker --ros-args -p use_sim_time:=true \
  -p trial_id:=$TRIAL -p output_dir:=$HOME/ros2_ws_course_repro/results/physics_grasp/lift_diagnostic \
  > /tmp/picker_${TRIAL}.log 2>&1
grep -E 'stage ->|trial .* ->' /tmp/picker_${TRIAL}.log
kill -INT $REC 2>/dev/null; sleep 3
tail -1 /tmp/jrec.log
