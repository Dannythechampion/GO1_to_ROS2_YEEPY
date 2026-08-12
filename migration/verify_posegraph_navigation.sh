#!/usr/bin/env bash
# Pose-graph localization dry-run verifier.  This script never arms Go1.
set -euo pipefail

mode="${1:-preflight}"
if [[ "$mode" != "preflight" && "$mode" != "ready" ]]; then
  printf '사용법: %s [preflight|ready]\n' "$0" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
go1_ros2_ws="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  return 1
}

require_file() {
  local path="$1"
  [[ -r "$path" ]] || fail "필수 환경 파일을 읽을 수 없습니다: $path"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "필수 명령을 찾을 수 없습니다: $1"
}

tmp_output="$(mktemp)"
trap 'rm -f "$tmp_output"' EXIT

require_file /opt/ros/humble/setup.bash
require_file "$go1_ros2_ws/install/setup.bash"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$go1_ros2_ws/install/setup.bash"
require_command ros2
require_command timeout

require_package() {
  local package="$1"
  if ! ros2 pkg prefix "$package" >"$tmp_output" 2>&1; then
    cat "$tmp_output" >&2
    fail "ROS 2 패키지가 없습니다: $package (workspace를 build/source 했는지 확인)"
  fi
  printf '통과: 의존 패키지 %s\n' "$package"
}

for package in omx_navigation go1_driver nav2_map_server nav2_lifecycle_manager slam_toolbox; do
  require_package "$package"
done

require_topic() {
  local topic="$1"
  if ! timeout 10 ros2 topic list | grep -Fxq "$topic"; then
    fail "토픽이 없습니다: $topic (해당 launch와 Livox/FAST-LIO를 확인)"
  fi
  printf '통과: 토픽 %s\n' "$topic"
}

require_message() {
  local topic="$1"
  if ! timeout 10 ros2 topic echo "$topic" --once >"$tmp_output" 2>&1 || [[ ! -s "$tmp_output" ]]; then
    cat "$tmp_output" >&2
    fail "메시지를 받지 못했습니다: $topic"
  fi
  printf '통과: 메시지 %s\n' "$topic"
}

require_rate() {
  local topic="$1" status
  set +e
  timeout 6 ros2 topic hz "$topic" --window 5 >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]] || ! grep -q 'average rate:' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail "발행 주파수를 확인하지 못했습니다: $topic"
  fi
  printf '통과: 주파수 %s\n' "$topic"
}

require_tf() {
  local parent="$1" child="$2" status
  set +e
  timeout 8 ros2 run tf2_ros tf2_echo "$parent" "$child" >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]] || ! grep -Eq 'Translation:|At time' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail "TF를 확인하지 못했습니다: $parent -> $child"
  fi
  printf '통과: TF %s -> %s\n' "$parent" "$child"
}

require_active() {
  local node="$1"
  if ! timeout 10 ros2 lifecycle get "$node" >"$tmp_output" 2>&1 || ! grep -Fq 'active [3]' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail "Nav2 lifecycle 노드가 active가 아닙니다: $node"
  fi
  printf '통과: lifecycle %s\n' "$node"
}

for topic in \
  /scan /Odometry /map /slam_toolbox/pose \
  /localization_supervisor/status /localization_supervisor/ready \
  /cmd_vel_nav /cmd_vel; do
  require_topic "$topic"
done
require_rate /scan
require_rate /Odometry

if [[ "$mode" == "ready" ]]; then
  if timeout 10 ros2 node list | grep -Fxq /amcl; then
    fail 'AMCL이 실행 중입니다: pose-graph localization과 함께 /amcl을 실행하지 마십시오'
  fi
  printf '통과: /amcl 미실행\n'
  require_message /map
  require_message /slam_toolbox/pose
  require_message /localization_supervisor/status
  require_message /localization_supervisor/ready
  require_tf map camera_init
  require_tf camera_init body_nav
  for node in \
    /map_server /controller_server /smoother_server /planner_server \
    /behavior_server /bt_navigator /waypoint_follower /velocity_smoother; do
    require_active "$node"
  done
  # ros2 expected output: Boolean value is: False
  if ! timeout 10 ros2 param get /go1_driver arm >"$tmp_output" 2>&1 || ! grep -Eq 'Boolean value is: (False|false)' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail '안전 설정 오류: /go1_driver arm 파라미터는 false여야 합니다'
  fi
  printf '통과: /go1_driver arm=false\n'
fi

printf '통과: pose-graph navigation %s 검증이 완료되었습니다 (ROS_DOMAIN_ID=%s).\n' "$mode" "$ROS_DOMAIN_ID"
