#!/usr/bin/env bash
# Control container: builds the workspace, launches the platform + the task, and shuts
# down cleanly on SIGTERM so destroy_node() runs (the real arm gets a proper disconnect).
set -e
set -m  # each background job gets its own process group

source /opt/ros/jazzy/setup.bash
cd /ros2_ws
colcon build --symlink-install --event-handlers console_cohesion- status-
source install/setup.bash

# Serial device nodes for the real arm (the cgroup rule allows them, they must exist)
for i in 0 1 2 3; do
  [ -e /dev/ttyACM$i ] || mknod /dev/ttyACM$i c 166 $i 2>/dev/null && chmod 666 /dev/ttyACM$i 2>/dev/null || true
  [ -e /dev/ttyUSB$i ] || mknod /dev/ttyUSB$i c 188 $i 2>/dev/null && chmod 666 /dev/ttyUSB$i 2>/dev/null || true
done

ros2 launch so101_bringup control.launch.py \
  "task:=${TASK:-pick_place}" "use_sim:=${USE_SIM:-true}" \
  "port:=${REAL_ROBOT_PORT:-/dev/ttyACM0}" "passive:=${PASSIVE:-false}" \
  "viewer:=${SIM_VIEWER:-false}" "camera_url:=${CAMERA_URL:-}" \
  "marker_xy:=${MARKER_XY:-}" "marker_size:=${MARKER_SIZE:-0.10}" &
pid=$!

shutdown() {
  trap - TERM INT
  set +e
  # Signal the whole group, as Ctrl-C does: ros2 launch ignores a SIGINT sent to it alone.
  kill -INT -"$pid" 2>/dev/null
  ( sleep 5; kill -KILL -"$pid" 2>/dev/null ) &
  wait "$pid"
  exit 0
}
trap shutdown TERM INT
wait "$pid"
