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

# Millimetres to a metre string a ROS parameter will accept as a double.
#
# `bc` drops the leading zero, and the ROS CLI resolves parameter values with
# PyYAML, whose float pattern accepts an UNSIGNED leading dot (.008 -> 0.008)
# but not a signed one: `-.008` comes back as PARAMETER_STRING. So a negative
# value from bc silently becomes a string, and a node that declared the
# parameter as a double either rejects it or keeps its default - either way the
# sweep runs on the wrong constant with nothing in the log to say so.
#
# The positive sweeps get away with it because every one of them prepends "0"
# by hand. That does not work for a negative, which is why this exists.
#
#   mm_to_m 128     -> 0.1280
#   mm_to_m -0.7    -> -0.0007
#   mm_to_m 0       -> 0.0000
mm_to_m() {
  awk -v v="$1" 'BEGIN{ printf "%.4f", v / 1000.0 }'
}
