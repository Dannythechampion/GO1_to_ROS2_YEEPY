#!/usr/bin/env bash
# Pose-graph localization dry-run verifier. This script never arms Go1.
set -euo pipefail

mode="${1:-preflight}"
if [[ "$mode" != "preflight" && "$mode" != "ready" ]]; then
  printf '사용법: %s [preflight|ready]\n' "$0" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
ros_setup_file="${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
go1_ros2_ws="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
required_topics=(
  /scan
  /Odometry
  /map
  /slam_localization/pose
  /localization_supervisor/status
  /localization_supervisor/ready
  /cmd_vel_nav
  /cmd_vel
)
ready_nodes=(
  /map_server
  /controller_server
  /smoother_server
  /planner_server
  /behavior_server
  /bt_navigator
  /waypoint_follower
  /velocity_smoother
)

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

require_file "$ros_setup_file"
require_file "$go1_ros2_ws/install/setup.bash"
# shellcheck disable=SC1090
source "$ros_setup_file"
# shellcheck disable=SC1090
source "$go1_ros2_ws/install/setup.bash"
require_command ros2
require_command timeout
require_command python3

require_package() {
  local package="$1"
  if ! ros2 pkg prefix "$package" >"$tmp_output" 2>&1; then
    cat "$tmp_output" >&2
    fail "ROS 2 패키지가 없습니다: $package (workspace build/source 상태를 확인)"
  fi
  printf '통과: 의존 패키지 %s\n' "$package"
}

require_posegraph_artifacts() {
  local omx_prefix graph_base suffix
  if ! omx_prefix="$(ros2 pkg prefix omx_navigation 2>"$tmp_output")"; then
    cat "$tmp_output" >&2
    fail 'omx_navigation 설치 경로를 읽지 못했습니다'
  fi
  graph_base="$omx_prefix/share/omx_navigation/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f"
  for suffix in .posegraph .data; do
    [[ -s "${graph_base}${suffix}" ]] || fail "필수 posegraph artifact가 없거나 비어 있습니다: ${graph_base}${suffix}"
  done
  printf '통과: 저장 posegraph artifact 확인\n'
}

topic_exists() {
  local topic="$1" listed
  while IFS= read -r listed; do
    [[ "$listed" == "$topic" ]] && return 0
  done <<<"$topic_list"
  return 1
}

require_topic() {
  local topic="$1"
  if ! topic_exists "$topic"; then
    fail "토픽이 없습니다: $topic (해당 launch와 Livox/FAST-LIO를 확인)"
  fi
  printf '통과: 토픽 %s\n' "$topic"
}

require_message() {
  local topic="$1"
  if ! timeout 10 ros2 topic echo "$topic" --once >"$tmp_output" 2>&1; then
    cat "$tmp_output" >&2
    fail "메시지를 받지 못했습니다: $topic"
  fi
  [[ -s "$tmp_output" ]] || fail "메시지가 비어 있습니다: $topic"
  printf '통과: 메시지 %s\n' "$topic"
}

require_rate() {
  local topic="$1" status
  set +e
  timeout 6 ros2 topic hz "$topic" --window 5 >"$tmp_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    cat "$tmp_output" >&2
    fail "발행 주파수 명령이 실패했습니다: $topic"
  fi
  if ! grep -q 'average rate:' "$tmp_output"; then
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
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    cat "$tmp_output" >&2
    fail "TF 명령이 실패했습니다: $parent -> $child"
  fi
  if ! grep -Eq 'Translation:|At time' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail "TF를 확인하지 못했습니다: $parent -> $child"
  fi
  printf '통과: TF %s -> %s\n' "$parent" "$child"
}

require_active() {
  local node="$1"
  if ! timeout 10 ros2 lifecycle get "$node" >"$tmp_output" 2>&1; then
    cat "$tmp_output" >&2
    fail "Nav2 lifecycle 조회가 실패했습니다: $node"
  fi
  if ! grep -Fq 'active [3]' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail "Nav2 lifecycle 노드가 active가 아닙니다: $node"
  fi
  printf '통과: lifecycle %s\n' "$node"
}

has_amcl_node() {
  local node basename
  while IFS= read -r node; do
    basename="${node%/}"
    basename="${basename##*/}"
    [[ "$basename" == "amcl" ]] && return 0
  done <<<"$node_list"
  return 1
}

strip_ros_separator() {
  local value="$1" line index start=0 end
  local -a lines=()
  while IFS= read -r line || [[ -n "$line" ]]; do
    [[ "$line" == '---' ]] || lines+=("$line")
  done <<<"$value"
  end=$((${#lines[@]} - 1))
  while (( start <= end )) && [[ "${lines[start]}" =~ ^[[:space:]]*$ ]]; do
    start=$((start + 1))
  done
  while (( end >= start )) && [[ "${lines[end]}" =~ ^[[:space:]]*$ ]]; do
    end=$((end - 1))
  done
  for ((index = start; index <= end; index += 1)); do
    printf '%s' "${lines[index]}"
    if (( index < end )); then
      printf '\n'
    fi
  done
  return 0
}

require_supervisor_ready() {
  local ready_value
  if ! ready_value="$(timeout 10 ros2 topic echo /localization_supervisor/ready std_msgs/msg/Bool --field data --once)"; then
    fail 'ready 토픽을 읽지 못했습니다: /localization_supervisor/ready'
  fi
  ready_value="$(strip_ros_separator "$ready_value")"
  if [[ "$ready_value" != "true" ]]; then
    fail "ready 토픽 값이 true가 아닙니다: $ready_value"
  fi
  printf '통과: localization_supervisor ready=true\n'
}

require_supervisor_status() {
  local status_json
  if ! status_json="$(timeout 10 ros2 topic echo /localization_supervisor/status std_msgs/msg/String --field data --once)"; then
    fail '상태 토픽을 읽지 못했습니다: /localization_supervisor/status'
  fi
  status_json="$(strip_ros_separator "$status_json")"
  if ! STATUS_JSON="$status_json" python3 -c '
import json
import os
import sys
status = json.loads(os.environ["STATUS_JSON"])
if not isinstance(status, dict):
    raise SystemExit(1)
if status.get("state") != "READY" or status.get("error") != "NONE":
    raise SystemExit(1)
'; then
    fail "상태 토픽이 READY/NONE이 아닙니다: $status_json"
  fi
  printf '통과: localization_supervisor state=READY error=NONE\n'
}

for package in omx_navigation go1_driver nav2_map_server nav2_lifecycle_manager slam_toolbox; do
  require_package "$package"
done
require_posegraph_artifacts

if ! topic_list="$(timeout 10 ros2 topic list)"; then
  fail '토픽 목록을 읽지 못했습니다: ROS_DOMAIN_ID와 launch 상태를 확인'
fi
for topic in "${required_topics[@]}"; do
  require_topic "$topic"
done
require_rate /scan
require_rate /Odometry

if [[ "$mode" == "ready" ]]; then
  if ! node_list="$(timeout 10 ros2 node list)"; then
    fail '노드 목록을 읽지 못했습니다: ROS 그래프 상태를 확인'
  fi
  if has_amcl_node; then
    fail 'AMCL이 실행 중입니다: pose-graph localization과 함께 amcl 노드를 실행하지 마십시오'
  fi
  printf '통과: amcl 노드 미실행\n'
  require_message /map
  require_message /slam_localization/pose
  require_supervisor_status
  require_supervisor_ready
  require_tf map camera_init
  require_tf camera_init body_nav
  for node in "${ready_nodes[@]}"; do
    require_active "$node"
  done
  if ! timeout 10 ros2 param get /go1_driver arm >"$tmp_output" 2>&1; then
    cat "$tmp_output" >&2
    fail '안전 설정을 읽지 못했습니다: /go1_driver arm'
  fi
  if ! grep -Eq 'Boolean value is: (False|false)' "$tmp_output"; then
    cat "$tmp_output" >&2
    fail '안전 설정 오류: /go1_driver arm 파라미터는 false여야 합니다'
  fi
  printf '통과: /go1_driver arm=false\n'
fi

printf '통과: pose-graph navigation %s 검증이 완료되었습니다 (ROS_DOMAIN_ID=%s).\n' "$mode" "$ROS_DOMAIN_ID"
