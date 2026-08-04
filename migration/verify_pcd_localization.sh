#!/usr/bin/env bash
# Runtime gates for the 3D PCD localization workflow.
#
# Usage:
#   ./migration/verify_pcd_localization.sh            # full check
#   ./migration/verify_pcd_localization.sh sensors    # gates 1-2 only
#
# Every gate is a hard failure. Do not send a goal and do not set arm:=true
# until this script exits 0.
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
source /opt/ros/humble/setup.bash
source "${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}/install/setup.bash"
set -euo pipefail

mode="${1:-full}"
# Nav2 costmaps use transform_tolerance: 0.5 in the PCD profile.
tf_max_delta="${TF_MAX_DELTA:-0.5}"

tmp_output="$(mktemp)"
trap 'rm -f "$tmp_output"' EXIT

fail() { printf 'FAIL: %s\n' "$1" >&2; exit 1; }

require_topic() {
  timeout 10 ros2 topic list | grep -Fxq "$1" || fail "topic missing: $1"
  printf 'PASS topic: %s\n' "$1"
}

require_rate() {
  local topic="$1" status
  set +e
  timeout 8 ros2 topic hz "$topic" --window 5 >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then cat "$tmp_output" >&2; fi
  grep -q "average rate:" "$tmp_output" || fail "no messages on $topic"
  printf 'PASS rate: %s (%s)\n' "$topic" "$(grep -m1 'average rate:' "$tmp_output")"
}

require_message() {
  timeout 10 ros2 topic echo "$1" --once >"$tmp_output" || fail "no message on $1"
  test -s "$tmp_output" || fail "empty message on $1"
  printf 'PASS message: %s\n' "$1"
}

require_tf() {
  local parent="$1" child="$2" status
  set +e
  timeout 6 ros2 run tf2_ros tf2_echo "$parent" "$child" >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then cat "$tmp_output" >&2; fi
  grep -Eq "Translation:|At time" "$tmp_output" || fail "TF missing: $parent -> $child"
  printf 'PASS TF: %s -> %s\n' "$parent" "$child"
}

# Gate 6b: exactly one publisher, and no gap larger than the costmap tolerance.
# This is the check that catches NDT/GICP blocking the TF timer.
require_tf_continuity() {
  local parent="$1" child="$2" status delta
  set +e
  timeout 15 ros2 run tf2_ros tf2_monitor "$parent" "$child" >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then cat "$tmp_output" >&2; fi

  if grep -qi "Broadcasters" "$tmp_output"; then
    local count
    count="$(sed -n '/Broadcasters/,$p' "$tmp_output" | grep -c 'Node:' || true)"
    if [[ "$count" -gt 1 ]]; then
      cat "$tmp_output" >&2
      fail "$count publishers for $parent -> $child; exactly one is allowed"
    fi
  fi

  delta="$(grep -Eo 'Max Delta: *[0-9.]+' "$tmp_output" | head -1 | grep -Eo '[0-9.]+' || true)"
  if [[ -z "$delta" ]]; then
    cat "$tmp_output" >&2
    fail "tf2_monitor reported no timing for $parent -> $child"
  fi
  if awk -v d="$delta" -v m="$tf_max_delta" 'BEGIN{exit !(d > m)}'; then
    cat "$tmp_output" >&2
    fail "$parent -> $child max delta ${delta}s exceeds ${tf_max_delta}s (Nav2 will time out)"
  fi
  printf 'PASS TF continuity: %s -> %s max delta %ss\n' "$parent" "$child" "$delta"
}

require_active() {
  timeout 10 ros2 lifecycle get "$1" >"$tmp_output" || fail "lifecycle query failed: $1"
  grep -Fq "active [3]" "$tmp_output" || fail "not active: $1 ($(cat "$tmp_output"))"
  printf 'PASS lifecycle: %s\n' "$1"
}

# ---------------------------------------------------------------- gate 1-2
printf '\n== Gate 1-2: sensors and FAST-LIO ==\n'
for topic in /livox/lidar /livox/imu /cloud_registered_body /Odometry; do
  require_topic "$topic"
done
require_rate /livox/imu
require_rate /cloud_registered_body
require_rate /Odometry
require_tf camera_init body

if [[ "$mode" == "sensors" ]]; then
  printf '\nPASS: sensor and odometry gates only\n'
  exit 0
fi

# ---------------------------------------------------------------- gate 3-4
printf '\n== Gate 3-4: maps and registration ==\n'
for topic in /scan /map /pcd_localizer/status /pcd_localizer/pose \
  /pcd_localizer/map_cloud /pcd_localizer/aligned_cloud /cmd_vel /go1/control_state; do
  require_topic "$topic"
done
require_rate /scan
require_message /map
require_message /pcd_localizer/map_cloud
require_message /pcd_localizer/pose

# A single --once can catch a lucky success between rejections. Collect a
# window instead and require it to be clean.
printf 'Collecting 12 s of /pcd_localizer/status ...\n'
set +e
timeout 12 ros2 topic echo /pcd_localizer/status >"$tmp_output" 2>&1
set -e
grep -q "LOCALIZED fitness=" "$tmp_output" || {
  cat "$tmp_output" >&2
  fail "PCD localizer never reported LOCALIZED"
}
if grep -qE "REJECTED|STALE" "$tmp_output"; then
  grep -E "REJECTED|STALE" "$tmp_output" | sort -u >&2
  fail "PCD localizer is flapping; fix the rejections before driving"
fi
printf 'PASS state: %s\n' "$(grep -m1 'LOCALIZED fitness=' "$tmp_output")"

# ---------------------------------------------------------------- gate 6
printf '\n== Gate 6: TF ownership and continuity ==\n'
require_tf map camera_init
require_tf map body
require_tf_continuity map camera_init

if timeout 10 ros2 node list | grep -Fxq /amcl; then
  fail "/amcl must not run in the PCD localization workflow"
fi
printf 'PASS architecture: AMCL is not running\n'

# ---------------------------------------------------------------- gate 7
printf '\n== Gate 7: Nav2 servers ==\n'
for node in /map_server /controller_server /planner_server /smoother_server \
  /behavior_server /bt_navigator /waypoint_follower /velocity_smoother; do
  require_active "$node"
done

# ---------------------------------------------------------------- gate 8
printf '\n== Gate 8: safety and load ==\n'
arm_value="$(timeout 10 ros2 param get /go1_driver arm)"
if [[ "$arm_value" != "Boolean value is: False" ]]; then
  fail "go1_driver must remain arm=false, got: $arm_value"
fi
printf 'PASS safety: /go1_driver arm=false\n'

printf 'CPU / memory snapshot (record this for the Jetson benchmark):\n'
top -b -n 2 -d 1 | tail -n +8 | \
  grep -E "pcd_localizer|fastlio|fast_lio|rviz2|controller_ser|planner_serv" | \
  tail -20 || true
printf 'Free memory:\n'
free -m | head -2

printf '\nPASS: guarded 3D PCD localization and Nav2 are active\n'
printf 'Next: confirm in RViz that Aligned Scan overlaps PCD Map and the 2D map\n'
printf 'walls before sending any goal. Keep arm:=false.\n'
