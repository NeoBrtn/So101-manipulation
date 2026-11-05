#!/usr/bin/env bash
# Phase 2 in the simulation, end to end: starts the stack if needed (TASK=decap,
# USE_SIM=true), resets the scene (bottle in the fixture, cap on), triggers /decap and
# waits for the result.
#
# Usage: scripts/decap_demo.sh [--runs N] [--rviz] [--viewer]
set -uo pipefail
cd "$(dirname "$0")/../docker" || exit 1

runs=1; rviz=false; viewer=false
while [ $# -gt 0 ]; do
  case $1 in
    --runs) runs=$2; shift ;;
    --rviz) rviz=true ;;
    --viewer) viewer=true ;;
    *) echo "usage: $0 [--runs N] [--rviz] [--viewer]"; exit 2 ;;
  esac
  shift
done

export TASK=decap USE_SIM=true
if $viewer; then
  export SIM_VIEWER=true
  xhost +local:docker >/dev/null 2>&1 || true
else   # keep the running stack's setting (e.g. started by scripts/sim.sh): no restart
  running=$(docker compose exec -T control printenv SIM_VIEWER 2>/dev/null)
  [ -n "$running" ] && export SIM_VIEWER=$running
fi
ros() { docker compose exec -T control bash -lc "source /ros2_ws/install/setup.bash && $*"; }
call() {   # retried: a short-lived CLI client sometimes misses the response (DDS discovery)
  local out
  for _ in 1 2 3; do
    out=$(ros "timeout 15 ros2 service call $1 std_srvs/srv/Trigger") && { echo "$out"; return 0; }
  done
  return 1
}

docker compose up -d control viz || exit 1
echo "Waiting for the ROS nodes (the first start builds the workspace)..."
for _ in $(seq 180); do
  if ros "ros2 service list" 2>/dev/null | grep -qx "/decap"; then
    ready=true; break
  fi
  sleep 1
done
[ "${ready:-false}" = true ] || { echo "error: /decap not available, see: docker compose logs control"; exit 1; }
sleep 15   # the node folds the arm to its rest pose at startup
$rviz && ../scripts/rviz.sh

ok=0
for run in $(seq "$runs"); do
  call /reset_object >/dev/null || { echo "error: /reset_object failed"; exit 1; }
  sleep 2
  out=$(mktemp)
  ros "timeout 300 ros2 topic echo --once --field data /decap/result std_msgs/msg/String" > "$out" &
  listener=$!
  sleep 2
  start=$(date +%s)
  reply=$(call /decap)
  if ! grep -q "success=True\|busy" <<<"$reply"; then   # busy: a retried call already started it
    echo "error: not started: $(grep -o "message='[^']*'" <<<"$reply")"
    kill $listener 2>/dev/null; exit 1
  fi
  wait $listener
  result=$(head -1 "$out"); rm -f "$out"
  echo "[$run/$runs] ${result:-no result (timeout)} ($(( $(date +%s) - start )) s)"
  [[ $result == SUCCESS* ]] && ok=$((ok + 1))
done
echo "Success: $ok/$runs"
[ "$ok" -eq "$runs" ]
