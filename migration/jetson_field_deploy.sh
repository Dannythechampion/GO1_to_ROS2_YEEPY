#!/usr/bin/env bash
# Fail-closed field deployment runner for NVIDIA Jetson AGX Orin + Go1.
set -euo pipefail

readonly armed_token="GO1_ARMED_AND_ESTOP_READY"
readonly default_workspace="/mnt/t500/go1_ros2_ws"
readonly default_sdk_root="/mnt/t500/go1_sdk"
readonly default_diagnostics_root="/mnt/t500/localization_logs"
readonly default_coarse_search_translation_radius="1.0"
readonly expected_sdk_library_sha256="4ec2f384271ecc6cc4266e10b888d5bb076d73f10ee680436718c26d1d865a6d"
readonly expected_wrapper_source_sha256="d98151de542eacb74532af6aba35d79b36bed9398c8de09c0aaa1724aad049b7"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Killing a `ros2 launch` orphans its child nodes instead of terminating them.
# On 2026-08-18 three generations of go1_driver, planar_base_frame and
# cmd_vel_safety_gate ended up running at once; the duplicate planar_base_frame
# instances published the same TF and the supervisor reported TF_CONFLICT.
# These patterns match the installed executables this runner owns, so a relaunch
# fails closed instead of stacking a fourth generation. Livox and FAST-LIO are
# deliberately absent: this runner requires them to be already running.
readonly -a managed_node_patterns=(
  "lib/go1_driver/go1_driver"
  "lib/omx_navigation/planar_base_frame"
  "lib/omx_navigation/cmd_vel_safety_gate"
  "lib/omx_navigation/rviz_goal_bridge"
  "lib/omx_navigation/localization_supervisor"
  "lib/nav2_map_server/map_server"
  "lib/nav2_lifecycle_manager/lifecycle_manager"
  "lib/nav2_velocity_smoother/velocity_smoother"
  "lib/rclcpp_components/component_container_isolated"
  "lib/slam_toolbox/localization_slam_toolbox_node"
  "lib/slam_toolbox/async_slam_toolbox_node"
  "lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node"
)

fail() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat >&2 <<EOF
Usage:
  $0 stage <repository> [workspace]
  $0 build [workspace]
  $0 preflight [workspace]
  $0 cleanup
  $0 dry-run [workspace]
  $0 armed $armed_token [workspace]
EOF
}

source_file() {
  local path="$1"
  [[ -r "$path" ]] || fail "required setup file is unreadable: $path"
  set +u
  # shellcheck disable=SC1090
  source "$path"
  set -u
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "required command is unavailable: $1"
}

source_ros_workspace() {
  local workspace="$1"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
  source_file "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
  [[ "${ROS_DISTRO:-}" == "humble" ]] || fail "ROS_DISTRO must be humble"
  source_file "$workspace/install/setup.bash"
}

verify_nonzero_test_results() {
  local workspace="$1"
  python3 - "$workspace" <<'PY'
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

workspace = Path(sys.argv[1])
for package in ("go1_driver", "omx_navigation"):
    report = workspace / "build" / package / "pytest.xml"
    if not report.is_file():
        raise SystemExit(f"ERROR: missing pytest result for {package}: {report}")
    root = ET.parse(report).getroot()
    suites = [root] if root.tag == "testsuite" else root.findall(".//testsuite")
    tests = sum(int(suite.attrib.get("tests", 0)) for suite in suites)
    errors = sum(int(suite.attrib.get("errors", 0)) for suite in suites)
    failures = sum(int(suite.attrib.get("failures", 0)) for suite in suites)
    skipped = sum(int(suite.attrib.get("skipped", 0)) for suite in suites)
    if tests <= 0 or errors or failures or skipped:
        raise SystemExit(
            f"ERROR: unsafe test result for {package}: "
            f"tests={tests} errors={errors} failures={failures} skipped={skipped}"
        )
    print(
        f"PASS: {package} tests={tests} errors=0 failures=0 skipped=0"
    )
PY
}

stage_sources() {
  [[ $# -ge 1 && $# -le 2 ]] || { usage; exit 2; }
  local repository="$1"
  local workspace="${2:-$default_workspace}"
  local staging="$repository/migration/stage_local_ros2_packages.sh"
  [[ -x "$staging" ]] || fail "staging script is unavailable: $staging"
  GO1_ROS2_WS="$workspace" "$staging"
}

build_workspace() {
  [[ $# -le 1 ]] || { usage; exit 2; }
  local workspace="${1:-$default_workspace}"
  source_file "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
  [[ "${ROS_DISTRO:-}" == "humble" ]] || fail "ROS_DISTRO must be humble"
  require_command rosdep
  require_command colcon
  require_command python3
  [[ -d "$workspace/src/go1_driver" ]] || fail "staged go1_driver is missing"
  [[ -d "$workspace/src/omx_navigation/test" ]] || fail "staged omx_navigation tests are missing"

  rosdep install --from-paths "$workspace/src" --ignore-src -r -y
  (
    cd "$workspace"
    colcon build --symlink-install --packages-select go1_driver omx_navigation
    source_file "$workspace/install/setup.bash"
    colcon test --event-handlers console_direct+ --packages-select go1_driver omx_navigation
    colcon test-result --verbose
  )
  verify_nonzero_test_results "$workspace"
}

verify_unitree_wrapper() {
  local sdk_root="${GO1_SDK_ROOT:-$default_sdk_root}"
  local environment="$sdk_root/setup_unitree_sdk.bash"
  local build_info="$sdk_root/BUILD_INFO.txt"
  [[ -s "$build_info" ]] || fail "Unitree wrapper BUILD_INFO is missing"
  grep -Fxq 'sdk_version=v3.8.6' "$build_info" || fail "Unitree SDK version is not v3.8.6"
  grep -Fxq 'architecture=aarch64' "$build_info" || fail "Unitree wrapper was not built on aarch64"
  grep -Fxq "arm64_library_sha256=$expected_sdk_library_sha256" "$build_info" || \
    fail "Unitree SDK library hash evidence is missing or wrong"
  grep -Fxq "wrapper_source_sha256=$expected_wrapper_source_sha256" "$build_info" || \
    fail "Unitree wrapper source hash evidence is missing or wrong"
  source_file "$environment"

  local module_path
  module_path="$(python3 - <<'PY'
import robot_interface
print(robot_interface.__file__)
PY
)" || fail "robot_interface cannot be imported"
  [[ -f "$module_path" ]] || fail "robot_interface module is missing: $module_path"
  case "$module_path" in
    "$sdk_root"/unitree_legged_sdk/lib/python/arm64/*) ;;
    *) fail "robot_interface was imported outside the verified SDK root: $module_path" ;;
  esac
  file "$module_path" | grep -Eq 'ARM aarch64|ARM64' || fail "robot_interface is not ARM64"
  if ldd "$module_path" | grep -q 'not found'; then
    ldd "$module_path" >&2
    fail "robot_interface has unresolved shared libraries"
  fi
  python3 - <<'PY'
import robot_interface as sdk
sdk.HighCmd()
sdk.HighState()
print("PASS: Unitree robot_interface import and constructors")
PY
}

verify_installed_artifacts() {
  local prefix graph_base suffix
  prefix="$(ros2 pkg prefix omx_navigation)" || fail "omx_navigation is not installed"
  graph_base="$prefix/share/omx_navigation/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f"
  for suffix in .posegraph .data _annotated.yaml _annotated.pgm; do
    [[ -s "${graph_base}${suffix}" ]] || fail "installed map artifact is missing: ${graph_base}${suffix}"
  done
}

preflight() {
  local workspace="$1"
  [[ "$(uname -m)" == "aarch64" ]] || fail "Jetson preflight requires aarch64"
  [[ -r /etc/os-release ]] || fail "/etc/os-release is unavailable"
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${VERSION_ID:-}" == "22.04" ]] || fail 'Ubuntu VERSION_ID="22.04" is required'

  require_command python3
  require_command file
  require_command ldd
  require_command timeout
  source_ros_workspace "$workspace"
  require_command ros2
  local package
  for package in \
    go1_driver omx_navigation livox_ros_driver2 fast_lio \
    nav2_bringup slam_toolbox pointcloud_to_laserscan tf2_ros; do
    ros2 pkg prefix "$package" >/dev/null || fail "required ROS package is missing: $package"
  done
  verify_unitree_wrapper
  verify_installed_artifacts

  local diagnostics_root="${DIAGNOSTICS_ROOT:-$default_diagnostics_root}"
  mkdir -p "$diagnostics_root"
  local probe
  probe="$(mktemp "$diagnostics_root/.go1-write-test.XXXXXX")" || fail "diagnostics path is not writable"
  rm -f -- "$probe"
  printf 'PASS: Jetson static preflight (ROS_DOMAIN_ID=%s, diagnostics=%s)\n' \
    "$ROS_DOMAIN_ID" "$diagnostics_root"
}

verify_live_inputs() {
  local topic tf_output status
  for topic in /livox/lidar /livox/imu /cloud_registered_body /Odometry; do
    timeout 12 ros2 topic echo "$topic" --once >/dev/null || fail "no live message on $topic"
  done
  tf_output="$(mktemp)"
  set +e
  timeout 12 ros2 run tf2_ros tf2_echo camera_init body >"$tf_output" 2>&1
  status=$?
  set -e
  if [[ "$status" -ne 0 && "$status" -ne 124 ]]; then
    cat "$tf_output" >&2
    rm -f -- "$tf_output"
    fail "camera_init -> body TF command failed"
  fi
  if ! grep -Eq 'Translation:|At time' "$tf_output"; then
    cat "$tf_output" >&2
    rm -f -- "$tf_output"
    fail "camera_init -> body TF is unavailable"
  fi
  rm -f -- "$tf_output"
  printf 'PASS: live MID-360, FAST-LIO, odometry, and body TF inputs\n'
}

# Match argv[0] -- the program actually being executed -- not the whole command
# line. `pgrep -f` also matches any shell whose arguments merely mention the
# path, which includes this script and the operator's own SSH command, and that
# turns a cleanup into self-destruction.
list_managed_nodes() {
  local entry pid argv0 argv1 candidate pattern
  local -a argv
  for entry in /proc/[0-9]*; do
    pid="${entry#/proc/}"
    [[ "$pid" == "$$" || "$pid" == "${PPID:-0}" ]] && continue
    [[ -r "$entry/cmdline" ]] || continue
    mapfile -t -d '' argv <"$entry/cmdline" 2>/dev/null || continue
    argv0="${argv[0]:-}"
    argv1="${argv[1]:-}"
    [[ -n "$argv0" ]] || continue
    # A C++ node is its own argv[0]. A node behind a shebang -- every ROS
    # console_script is -- runs as `<interpreter> <script>`, so argv[1] holds
    # the real target. An argv[1] beginning with `-` is an interpreter flag such
    # as `bash -c`, never a node path, and skipping it is what keeps a shell
    # that merely quotes the path from being matched.
    for candidate in "$argv0" "$argv1"; do
      [[ -n "$candidate" && "$candidate" != -* ]] || continue
      for pattern in "${managed_node_patterns[@]}"; do
        if [[ "$candidate" == *"/$pattern" ]]; then
          printf '%s\t%s\n' "$pid" "$candidate"
          break 2
        fi
      done
    done
  done
}

require_no_stale_nodes() {
  local running
  running="$(list_managed_nodes)"
  [[ -n "$running" ]] || return 0
  printf 'ERROR: nodes from an earlier launch are still running:\n' >&2
  printf '%s\n' "$running" >&2
  printf 'Killing a launch orphans its children. Duplicate publishers caused the\n' >&2
  printf '2026-08-18 TF_CONFLICT. Clear them first:\n  %s cleanup\n' "$0" >&2
  exit 1
}

cleanup_managed_nodes() {
  local running pid pattern signal
  running="$(list_managed_nodes)"
  if [[ -z "$running" ]]; then
    printf 'PASS: no leftover nodes from an earlier launch\n'
    return 0
  fi
  printf 'Terminating leftover nodes:\n%s\n' "$running"
  for signal in INT TERM KILL; do
    running="$(list_managed_nodes)"
    [[ -n "$running" ]] || break
    while IFS=$'\t' read -r pid pattern; do
      [[ -n "$pid" ]] || continue
      kill "-$signal" "$pid" 2>/dev/null || true
    done <<<"$running"
    sleep 3
  done
  running="$(list_managed_nodes)"
  if [[ -n "$running" ]]; then
    printf 'ERROR: these nodes survived SIGKILL:\n%s\n' "$running" >&2
    exit 1
  fi
  printf 'PASS: leftover nodes cleared\n'
}

launch_navigation() {
  local armed="$1"
  local workspace="$2"
  preflight "$workspace"
  require_no_stale_nodes
  verify_live_inputs
  if [[ "$armed" == "true" ]]; then
    exec ros2 launch omx_navigation go1_posegraph_navigation.launch.py \
      rviz:=false start_go1_driver:=true arm:=true \
      record_localization:=true diagnostics_root:="${DIAGNOSTICS_ROOT:-$default_diagnostics_root}" \
      coarse_search_translation_radius:="${COARSE_SEARCH_TRANSLATION_RADIUS:-$default_coarse_search_translation_radius}" \
      armed_confirmation:="$armed_token" ros_domain_id:="$ROS_DOMAIN_ID"
  fi
  exec ros2 launch omx_navigation go1_posegraph_navigation.launch.py \
    rviz:=false start_go1_driver:=true arm:=false \
    record_localization:=true diagnostics_root:="${DIAGNOSTICS_ROOT:-$default_diagnostics_root}" \
    coarse_search_translation_radius:="${COARSE_SEARCH_TRANSLATION_RADIUS:-$default_coarse_search_translation_radius}" \
    ros_domain_id:="$ROS_DOMAIN_ID"
}

main() {
  local mode="${1:-}"
  [[ -n "$mode" ]] || { usage; exit 2; }
  shift
  case "$mode" in
    stage)
      stage_sources "$@"
      ;;
    build)
      build_workspace "$@"
      ;;
    preflight)
      [[ $# -le 1 ]] || { usage; exit 2; }
      preflight "${1:-$default_workspace}"
      ;;
    cleanup)
      [[ $# -eq 0 ]] || { usage; exit 2; }
      cleanup_managed_nodes
      ;;
    dry-run)
      [[ $# -le 1 ]] || { usage; exit 2; }
      launch_navigation false "${1:-$default_workspace}"
      ;;
    armed)
      [[ $# -ge 1 && $# -le 2 ]] || { usage; exit 2; }
      [[ "$1" == "$armed_token" ]] || fail "armed mode requires exact token: $armed_token"
      launch_navigation true "${2:-$default_workspace}"
      ;;
    *)
      usage
      exit 2
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
