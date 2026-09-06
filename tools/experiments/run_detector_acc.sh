#!/usr/bin/env bash
EXPDIR="${EXPDIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
source "$EXPDIR/env.sh"
source "$ROS_SETUP"
source "$OPENARM_INSTALL/setup.bash"
LOG=/tmp/physics_launch_detacc.log
bash "$EXPDIR/preflight.sh" 1024 >/dev/null || exit 5
# Detector unlatched so it keeps re-measuring as the object is moved.
setsid nohup ros2 launch openarm_moveit_config run_physics_grasp.launch.py \
  viewer_render_period:=1.0 > "$LOG" 2>&1 < /dev/null &
for i in $(seq 1 120); do grep -q 'Successfully loaded controller' "$LOG" 2>/dev/null && break; sleep 3; done
sleep 8
pkill -f apple_detector.py
sleep 2
setsid nohup python3 "$OPENARM_SHARE/launch/apple_detector.py" \
  --ros-args -p use_sim_time:=true -p lock_first_detection:=false > /tmp/det_unlatched.log 2>&1 < /dev/null &
sleep 6
grep -c UNLATCHED /tmp/det_unlatched.log
python3 "$EXPDIR/measure_detector.py" 2>&1 | tail -22
