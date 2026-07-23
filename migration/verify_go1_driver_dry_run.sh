#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
log_dir="$workspace/log/dry_run_verification"
driver_log="$log_dir/go1_driver.log"
driver_pid=""

cleanup() {
  if [[ -z "$driver_pid" ]]; then
    return
  fi

  local pid="$driver_pid"
  driver_pid=""

  # The launch process and its node run in a dedicated session so they can be
  # stopped together. Bound the wait to avoid hanging after a successful test.
  kill -INT -- "-$pid" 2>/dev/null || true
  for _ in {1..50}; do
    if ! kill -0 -- "-$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done

  kill -TERM -- "-$pid" 2>/dev/null || true
  sleep 1
  kill -KILL -- "-$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
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

# ROS/colcon setup scripts probe optional environment variables that may be
# absent in a deliberately clean `env -i` shell. Disable nounset only while
# sourcing them, then restore strict mode for the verifier itself.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$workspace/install/setup.bash"
set -u

if ! ros2 pkg prefix go1_driver >/dev/null 2>&1; then
  printf 'ERROR: go1_driver is not installed in the sourced workspace.\n' >&2
  exit 1
fi

mkdir -p "$log_dir"
setsid ros2 launch go1_driver go1_driver.launch.py arm:=false >"$driver_log" 2>&1 &
driver_pid="$!"

python3 "$script_dir/verify_go1_driver_dry_run.py"

printf 'Driver log: %s\n' "$driver_log"
