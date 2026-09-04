#!/usr/bin/env bash
# Pre-flight cleanup and resource guard for MuJoCo/ROS2 grasp trials.
# Rationale: stale MuJoCo instances and leaked FastRTPS shared-memory segments
# have repeatedly exhausted VM memory (observed: 279 segments, 141 MB free,
# load average 121). Under that starvation, launches stretch 60s -> 360s and
# MoveIt returns spurious planning failures indistinguishable from real ones,
# which silently corrupts timing-sensitive results.
# Usage: preflight.sh [min_free_mb]   (default 1024)
set -o pipefail

MIN_FREE_MB="${1:-1024}"
LOGDIR="$HOME/ros2_ws_course_repro/results/physics_grasp"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/preflight.log"
ts() { date '+%Y-%m-%d %H:%M:%S'; }

mem_free() { free -m | awk '/^(Mem|内存)/ {print $7; exit}'; }
shm_count() { find /dev/shm -maxdepth 1 -name 'fastrtps*' -o -maxdepth 1 -name 'sem.fastrtps*' 2>/dev/null | wc -l; }
ros_count() { ps -u "$USER" -o pid=,args= | grep -E 'mujoco_ros2_control|move_group|apple_detector|robot_state_pub|physics_picker|contact_probe' | grep -v grep | wc -l; }

BEFORE_MEM=$(mem_free); BEFORE_SHM=$(shm_count); BEFORE_PROC=$(ros_count)
echo "[$(ts)] PRE  free=${BEFORE_MEM}MB shm=${BEFORE_SHM} procs=${BEFORE_PROC} load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$LOG"

# 1. Kill leftover simulation processes. Match on args but never on this script's
#    own command line (a bare 'ros2' pattern has killed the controlling ssh
#    session before, so the pattern is restricted to concrete executables).
ps -u "$USER" -o pid=,args= \
  | grep -E 'mujoco_ros2_control|move_group|apple_detector|robot_state_pub|static_transform|physics_picker|contact_probe' \
  | grep -v grep | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 3

# 1b. Restart the ros2 CLI daemon. A stale daemon makes 'ros2 control'
#     commands fail with '!rclpy.ok()' and return empty listings, which looks
#     exactly like controllers failing to load.
# Bound this: 'ros2 daemon stop' has been observed to hang forever (once for
# 11.6 h, silently stalling an entire overnight batch at trial 3 of 11). It is
# best-effort only -- the kill -9 below is what actually guarantees the daemon
# is gone.
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true
ps -u "$USER" -o pid=,args= | grep 'ros2cli.daemon' | grep -v grep | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 2

# 2. Clear leaked DDS shared memory owned by this user only.
find /dev/shm -maxdepth 1 -user "$USER" \( -name 'fastrtps*' -o -name 'sem.fastrtps*' \) -delete 2>/dev/null

# 3. Let the kernel settle, then enforce the memory floor.
sleep 2
AFTER_MEM=$(mem_free); AFTER_SHM=$(shm_count); AFTER_PROC=$(ros_count)
LOADNOW=$(cut -d' ' -f1 /proc/loadavg)
echo "[$(ts)] POST free=${AFTER_MEM}MB shm=${AFTER_SHM} procs=${AFTER_PROC} load=${LOADNOW}" >> "$LOG"
echo "preflight: free ${BEFORE_MEM}MB -> ${AFTER_MEM}MB, shm ${BEFORE_SHM} -> ${AFTER_SHM}, procs ${BEFORE_PROC} -> ${AFTER_PROC}, load ${LOADNOW}"

if [ "$AFTER_PROC" -ne 0 ]; then
  echo "[$(ts)] ABORT: ${AFTER_PROC} simulation process(es) survived cleanup" >> "$LOG"
  echo "ABORT: ${AFTER_PROC} simulation process(es) survived cleanup"; exit 3
fi
if [ "$AFTER_MEM" -lt "$MIN_FREE_MB" ]; then
  echo "[$(ts)] ABORT: free ${AFTER_MEM}MB < required ${MIN_FREE_MB}MB" >> "$LOG"
  echo "ABORT: only ${AFTER_MEM}MB free, need ${MIN_FREE_MB}MB — let the VM settle"; exit 4
fi
echo "preflight OK"
