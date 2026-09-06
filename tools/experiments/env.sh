#!/usr/bin/env bash
# Shared environment for every experiment script.
#
# Each value keeps the VM's original setting as its default, so behaviour on
# the machine every recorded result came from is unchanged, while any of them
# can be overridden from the caller's environment to run the same scripts
# elsewhere. Before this file the paths were written literally into all 19
# runners, which is why a checkout could not run them.
export ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export MUJOCO_DIR="${MUJOCO_DIR:-$HOME/.local/lib/python3.10/site-packages/mujoco}"
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"

export OPENARM_ROOT="${OPENARM_ROOT:-$HOME/ros2_ws_course_repro}"
export OPENARM_WS="${OPENARM_WS:-$OPENARM_ROOT/ros2_ws}"
export OPENARM_INSTALL="${OPENARM_INSTALL:-$OPENARM_WS/install_repro}"
export OPENARM_RESULTS="${OPENARM_RESULTS:-$OPENARM_ROOT/results/physics_grasp}"
export OPENARM_MODEL="${OPENARM_MODEL:-$OPENARM_WS/src/openarm_moveit_config/model}"
export OPENARM_SHARE="${OPENARM_SHARE:-$OPENARM_INSTALL/openarm_moveit_config/share/openarm_moveit_config}"
