#!/usr/bin/env bash
# Opens RViz inside the control container (X11 forwarded).
set -euo pipefail
cd "$(dirname "$0")/../docker"
xhost +local:docker >/dev/null 2>&1 || echo "warning: xhost failed, RViz may not open"
docker compose exec -d control bash -lc \
  'source /ros2_ws/install/setup.bash && rviz2 -d "$(ros2 pkg prefix so101_bringup)/share/so101_bringup/rviz/so101.rviz"'
