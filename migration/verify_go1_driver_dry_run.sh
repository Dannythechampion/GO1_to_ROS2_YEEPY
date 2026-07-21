#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${GO1_ROS2_WS:-$HOME/go1_ros2_ws}"
log_dir="$workspace/log/dry_run_verification"
driver_log="$log_dir/go1_driver.log"
driver_pid=""

cleanup() {
  if [[ -n "$driver_pid" ]] && kill -0 "$driver_pid" 2>/dev/null; then
    kill -INT "$driver_pid" 2>/dev/null || true
    wait "$driver_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'ERROR: ROS2 Humble is not installed.\n' >&2
  exit 1
fi
if [[ ! -r "$workspace/install/setup.bash" ]]; then
  printf 'ERROR: workspace is not built: %s\n' "$workspace" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"

if ! ros2 pkg prefix go1_driver >/dev/null 2>&1; then
  printf 'ERROR: go1_driver is not installed in the sourced workspace.\n' >&2
  exit 1
fi

mkdir -p "$log_dir"
ros2 launch go1_driver go1_driver.launch.py arm:=false >"$driver_log" 2>&1 &
driver_pid="$!"

python3 "$script_dir/verify_go1_driver_dry_run.py"

printf 'Driver log: %s\n' "$driver_log"
