#!/usr/bin/env bash
# Mirror physics-mode course-workspace files from the VM into the research repo.
# Run from the repo root. Read-only with respect to the VM.
set -euo pipefail

VM="nostalgia@127.0.0.1"
PORT=2224
WS="/home/nostalgia/ros2_ws_course_repro/ros2_ws"
DEST="vm_sync/ros2_course_physics"

PATHS=(
  "src/openarm_grasp_msgs"
  "src/openarm_grasp_physics"
  "src/mujoco_ros2_control/mujoco_ros2_control/CMakeLists.txt"
  "src/mujoco_ros2_control/mujoco_ros2_control/package.xml"
  "src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_grasp_telemetry.hpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_grasp_telemetry.cpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/include/mujoco_ros2_control/mujoco_scene_reset.hpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_scene_reset.cpp"
  "src/openarm_grasp_msgs/srv"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control_node.cpp"
  "src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp"
  "src/openarm_moveit_config/model/openarm_bimanual_physics.xml"
  "src/openarm_moveit_config/model/table_scene_physics.xml"
  "src/openarm_moveit_config/model/table_scene_physics.urdf"
  "src/openarm_moveit_config/config/openarm_physics.srdf"
  "src/openarm_moveit_config/launch/physics_controllers.yaml"
  "src/openarm_moveit_config/launch/move_group_physics.launch.py"
  "src/openarm_moveit_config/launch/run_physics_grasp.launch.py"
)

for p in "${PATHS[@]}"; do
  if ssh -p "$PORT" "$VM" "test -e $WS/$p"; then
    mkdir -p "$DEST/$(dirname "$p")"
    scp -q -P "$PORT" -r "$VM:$WS/$p" "$DEST/$(dirname "$p")/"
    echo "synced $p"
  else
    echo "skip (absent on VM) $p"
  fi
done

# Never mirror build products.
find "$DEST" -type d \( -name '__pycache__' -o -name '.pytest_cache' \) -prune -exec rm -rf {} + 2>/dev/null || true
echo "mirror complete"
