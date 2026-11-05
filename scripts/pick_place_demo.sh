#!/usr/bin/env bash
# One pick & place in the simulation, end to end:
#   starts the stack if needed (TASK=pick_place, USE_SIM=true), puts the cube at a new
#   random pose, triggers /pick_and_place and waits for the result.
#
# Usage: scripts/pick_place_demo.sh [--keep] [--rviz] [--runs N]
#   --keep    keep the cube where it is (no new random pose)
#   --rviz    open RViz
#   --runs N  repeat N times and print the success rate
set -uo pipefail
cd "$(dirname "$0")/../docker" || exit 1

keep=false; rviz=false; runs=1
while [ $# -gt 0 ]; do
  case $1 in
    --keep) keep=true ;;
    --rviz) rviz=true ;;
    --runs) runs=$2; shift ;;
    *) echo "usage: $0 [--keep] [--rviz] [--runs N]"; exit 2 ;;
  esac
  shift
done

export TASK=pick_place USE_SIM=true
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
