#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
set -euo pipefail

mode="${1:-preflight}"
if [[ "$mode" != "preflight" && "$mode" != "localized" ]]; then
  printf 'Usage: %s [preflight|localized]\n' "$0" >&2
  exit 2
fi

tmp_output="$(mktemp)"
trap 'rm -f "$tmp_output"' EXIT

require_topic() {
  local topic="$1"
  timeout 10 ros2 topic list | grep -Fxq "$topic"
  printf 'PASS topic: %s\n' "$topic"
}

require_message() {
  local topic="$1"
  timeout 10 ros2 topic echo "$topic" --once >"$tmp_output"
  test -s "$tmp_output"
  printf 'PASS message: %s\n' "$topic"
}

require_rate() {
  local topic="$1"
  local status

  set +e
  timeout 6 ros2 topic hz "$topic" --window 5 >"$tmp_output" 2>&1
  status=$?
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    cat "$tmp_output" >&2
    return "$status"
  fi
  grep -q "average rate:" "$tmp_output"
  printf 'PASS rate: %s\n' "$topic"
}

require_tf() {
  local parent="$1"
  local child="$2"
  local status

  set +e
  timeout 6 ros2 run tf2_ros tf2_echo "$parent" "$child" \
    >"$tmp_output" 2>&1
  status=$?
  set -e

  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    cat "$tmp_output" >&2
    return "$status"
  fi
  grep -Eq "Translation:|At time" "$tmp_output"
  printf 'PASS TF: %s -> %s\n' "$parent" "$child"
}

require_active() {
  local node="$1"
  timeout 10 ros2 lifecycle get "$node" >"$tmp_output"
  grep -Fq "active [3]" "$tmp_output"
  printf 'PASS lifecycle: %s\n' "$node"
}

for topic in \
  /scan /Odometry /map /amcl_pose /cmd_vel /go1/control_state; do
  require_topic "$topic"
done

require_rate /scan
require_rate /Odometry
require_message /map
require_message /go1/control_state
require_tf camera_init body

for node in \
  /map_server \
  /amcl \
  /controller_server \
  /planner_server \
  /behavior_server \
  /bt_navigator \
  /waypoint_follower; do
  require_active "$node"
done

arm_value="$(timeout 10 ros2 param get /go1_driver arm)"
if [[ "$arm_value" != "Boolean value is: False" ]]; then
  printf 'FAIL: go1_driver must remain arm=false, got: %s\n' \
    "$arm_value" >&2
  exit 1
fi
printf 'PASS safety: /go1_driver arm=false\n'

if [[ "$mode" == "localized" ]]; then
  require_message /amcl_pose
  require_tf map camera_init
fi

printf 'PASS: existing-map navigation is active with arm=false\n'
