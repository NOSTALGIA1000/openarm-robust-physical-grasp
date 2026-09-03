#!/usr/bin/env bash
export MUJOCO_DIR=/home/nostalgia/.local/lib/python3.10/site-packages/mujoco
export DISPLAY=:0
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws_course_repro/ros2_ws/install_repro/setup.bash"
export ROS_DOMAIN_ID=71
LOG=/tmp/physics_launch_resettest.log
bash /tmp/preflight.sh 1024 >/dev/null || exit 5
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &
for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
sleep 8
echo "--- service present? ---"
ros2 service list 2>/dev/null | grep reset_object
echo "--- object pose before ---"
timeout 10 ros2 topic echo /grasp_raw_state --once 2>/dev/null | grep -A4 'object_pose' | grep -E 'x:|y:|z:' | head -3
echo "--- displace object via reset to (0.32, 0.02, yaw 25) ---"
timeout 15 ros2 service call /reset_object openarm_grasp_msgs/srv/ResetObject "{x: 0.32, y: 0.02, yaw_deg: 25.0, use_spawn: false}" 2>&1 | tail -4
sleep 2
echo "--- object pose after ---"
timeout 10 ros2 topic echo /grasp_raw_state --once 2>/dev/null | grep -A4 'object_pose' | grep -E 'x:|y:|z:' | head -3
echo "--- restore to spawn ---"
timeout 15 ros2 service call /reset_object openarm_grasp_msgs/srv/ResetObject "{use_spawn: true}" 2>&1 | tail -3
sleep 2
timeout 10 ros2 topic echo /grasp_raw_state --once 2>/dev/null | grep -A4 'object_pose' | grep -E 'x:|y:|z:' | head -3
echo "--- sim_time still monotonic? (should be large, not reset) ---"
timeout 10 ros2 topic echo /grasp_raw_state --field sim_time --once 2>/dev/null
