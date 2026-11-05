#!/usr/bin/env bash
# One pick & place in the simulation, end to end:
#   starts the stack if needed (TASK=pick_place, USE_SIM=true), puts the cube at a new
#   random pose, triggers /pick_and_place and waits for the result.
#
# Usage: scripts/pick_place_demo.sh [--keep] [--rviz] [--viewer] [--runs N]
#   --keep    keep the cube where it is (no new random pose)
#   --rviz    open RViz
#   --viewer  live MuJoCo window (restarts the stack with it)
#   --runs N  repeat N times and print the success rate
set -uo pipefail
cd "$(dirname "$0")/../docker" || exit 1

keep=false; rviz=false; viewer=false; runs=1
while [ $# -gt 0 ]; do
  case $1 in
    --keep) keep=true ;;
    --rviz) rviz=true ;;
    --viewer) viewer=true ;;
    --runs) runs=$2; shift ;;
    *) echo "usage: $0 [--keep] [--rviz] [--viewer] [--runs N]"; exit 2 ;;
  esac
  shift
done

export TASK=pick_place USE_SIM=true
if $viewer; then
  export SIM_VIEWER=true
  xhost +local:docker >/dev/null 2>&1 || true
else   # keep the running stack's setting (e.g. started by scripts/sim.sh): no restart
  running=$(docker compose exec -T control printenv SIM_VIEWER 2>/dev/null)
  [ -n "$running" ] && export SIM_VIEWER=$running
fi
ros() { docker compose exec -T control bash -lc "source /ros2_ws/install/setup.bash && $*"; }
# A short-lived CLI client sometimes misses the service response (DDS discovery race):
# time out and retry instead of hanging
call() {
  local out
  for _ in 1 2 3; do
    out=$(ros "timeout 15 ros2 service call $1 std_srvs/srv/Trigger") && { echo "$out"; return 0; }
  done
  return 1
}

docker compose up -d control perception viz || exit 1
echo "Waiting for the ROS nodes (the first start builds the workspace)..."
for _ in $(seq 180); do
  services=$(ros "ros2 service list" 2>/dev/null)
  if grep -qx "/pick_and_place" <<<"$services" && grep -qx "/reset_object" <<<"$services"; then
    ready=true; break
  fi
  sleep 1
done
if [ "${ready:-false}" != true ]; then
  echo "error: /pick_and_place not available, see: docker compose logs control"
  exit 1
fi
$rviz && ../scripts/rviz.sh

ok=0
for run in $(seq "$runs"); do
  if ! $keep; then
    call /reset_object | grep -o "object at ([^)]*)" || { echo "error: /reset_object failed"; exit 1; }
    sleep 2   # let perception settle on the new pose
  fi
  # Subscribe before triggering, so the result cannot be missed
  out=$(mktemp)
  ros "timeout 240 ros2 topic echo --once --field data /pick_place/result std_msgs/msg/String" > "$out" &
  listener=$!
  sleep 2
  reply=$(call /pick_and_place)
  if ! grep -q "success=True\|busy" <<<"$reply"; then   # busy: a retried call already started it
    echo "error: not started: $(grep -o "message='[^']*'" <<<"$reply")"
    kill $listener 2>/dev/null; exit 1
  fi
  wait $listener
  result=$(head -1 "$out"); rm -f "$out"
  echo "[$run/$runs] ${result:-no result (timeout)}"
  [[ $result == SUCCESS* ]] && ok=$((ok + 1))
done
echo "Success: $ok/$runs"
[ "$ok" -eq "$runs" ]
