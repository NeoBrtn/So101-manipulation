#!/usr/bin/env bash
# Starts the simulation with the live MuJoCo window (and optionally RViz).
#
# Usage: scripts/sim.sh [TASK] [--rviz] [--no-viewer]
#   TASK         task package so101_<TASK>: pick_place | decap (default: TASK in docker/.env)
#   --rviz       also open RViz
#   --no-viewer  no MuJoCo window (headless)
#
# The MuJoCo window belongs to the driver process: the stack is (re)started with it.
# Then, for example:
#   scripts/pick_place_demo.sh           # TASK=pick_place
#   scripts/decap_demo.sh                # TASK=decap
#   docker compose -f docker/docker-compose.yml down   # stop
set -uo pipefail
cd "$(dirname "$0")/../docker" || exit 1

viewer=true; rviz=false
for arg in "$@"; do
  case $arg in
    --rviz) rviz=true ;;
    --no-viewer) viewer=false ;;
    -*) echo "usage: $0 [TASK] [--rviz] [--no-viewer]"; exit 2 ;;
    *) export TASK=$arg ;;
  esac
done
export USE_SIM=true SIM_VIEWER=$viewer
task=${TASK:-$(grep -E '^TASK=' .env 2>/dev/null | cut -d= -f2)}
task=${task:-pick_place}

if $viewer || $rviz; then
  xhost +local:docker >/dev/null 2>&1 || echo "warning: xhost failed, the windows may not open (X11 / WSLg needed)"
fi

echo "Starting the simulation: task $task, MuJoCo window $viewer"
docker compose up -d control perception viz || exit 1

ros() { docker compose exec -T control bash -lc "source /ros2_ws/install/setup.bash && $*"; }
echo "Waiting for the driver (the first start builds the workspace)..."
for _ in $(seq 180); do
  if ros "ros2 service list" 2>/dev/null | grep -qx "/reset_object"; then
    ready=true; break
  fi
  sleep 1
done
if [ "${ready:-false}" != true ]; then
  echo "error: the driver did not start, see: docker compose -f docker/docker-compose.yml logs control"
  exit 1
fi
$rviz && ../scripts/rviz.sh

echo "Ready."
case $task in
  pick_place) echo "  scripts/pick_place_demo.sh --runs 3   (or: ros2 service call /pick_and_place std_srvs/srv/Trigger)" ;;
  decap)      echo "  scripts/decap_demo.sh --runs 3        (or: ros2 service call /decap std_srvs/srv/Trigger)" ;;
esac
echo "  MuJoCo window: drag to rotate, right-drag to pan, scroll to zoom, double-click a body to track it."
