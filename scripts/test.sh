#!/usr/bin/env bash
# Builds the workspace and runs the unit tests in the control image, like the CI.
# Builds into the container's /tmp: nothing is written to the host's ros2_ws.
set -euo pipefail
cd "$(dirname "$0")/.."
docker run --rm -v "$PWD/ros2_ws/src:/src:ro" so101/control bash -c '
  source /opt/ros/jazzy/setup.bash && cd /tmp &&
  colcon build --base-paths /src --event-handlers console_cohesion- >/dev/null &&
  colcon test --base-paths /src --packages-select so101_manipulation so101_vision so101_pick_place so101_decap \
    --event-handlers console_direct+ --pytest-args -q &&
  colcon test-result --verbose'
