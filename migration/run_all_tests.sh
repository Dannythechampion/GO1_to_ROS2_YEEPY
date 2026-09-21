#!/usr/bin/env bash
# Run every test in this checkout as one suite, the way it has to pass.
#
# On 2026-09-18 the Jetson suite passed file by file and failed as a suite
# (test stubs leaked between modules), and a stray user-site anyio pytest plugin
# crashed the system pytest outright. Both are handled; this script is the one
# command that shows it on the robot computer.
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -r "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}" ]]; then
  # With ROS importable the stub isolation is exercised for real.
  set +u
  # shellcheck disable=SC1090
  source "${ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
  set -u
fi
# Test this checkout, never a copy installed in some workspace.
export PYTHONPATH="$repo/packages/omx_navigation:$repo/packages/go1_driver${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd "$repo"
exec python3 -m pytest packages migration -q -p no:cacheprovider "$@"
