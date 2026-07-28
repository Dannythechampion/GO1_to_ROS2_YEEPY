#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -euo pipefail

tmp_output="$(mktemp)"
trap 'rm -f "$tmp_output"' EXIT

require_topic() {
  local topic="$1"
  timeout 10 ros2 topic list | grep -Fxq "$topic"
  printf 'PASS topic: %s\n' "$topic"
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

for topic in \
  /livox/lidar /livox/imu /cloud_registered_body /scan /Odometry /map; do
  require_topic "$topic"
done

for topic in \
  /livox/lidar /livox/imu /cloud_registered_body /scan /Odometry; do
  require_rate "$topic"
done

require_tf camera_init body
require_tf map camera_init

printf 'PASS: MID-360, FAST-LIO, scan projection, and SLAM mapping are active\n'
