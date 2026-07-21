#!/usr/bin/env bash
set -euo pipefail

workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
livox_dir="$workspace/src/livox_ros_driver2"
fast_lio_dir="$workspace/src/FAST_LIO_ROS2"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'ERROR: ROS2 Humble is not installed.\n' >&2
  exit 1
fi
for required in \
  "$livox_dir/CMakeLists.txt" \
  "$livox_dir/package_ROS2.xml" \
  "$livox_dir/launch_ROS2" \
  "$fast_lio_dir/package.xml"; do
  if [[ ! -e "$required" ]]; then
    printf 'ERROR: required source is missing: %s\n' "$required" >&2
    exit 1
  fi
done
if [[ ! -f /usr/local/lib/liblivox_lidar_sdk_static.a ]]; then
  printf 'ERROR: Livox-SDK2 is not installed. Run install_livox_sdk2.sh first.\n' >&2
  exit 1
fi

# The upstream build.sh replaces these files and deletes workspace-wide build
# outputs. Prepare the ROS2 package explicitly so this integrated workspace is
# not cleaned behind the user's back.
cp "$livox_dir/package_ROS2.xml" "$livox_dir/package.xml"
mkdir -p "$livox_dir/launch"
cp -a "$livox_dir/launch_ROS2/." "$livox_dir/launch/"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -r "$workspace/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$workspace/install/setup.bash"
fi

cd "$workspace"
rosdep install --from-paths src --ignore-src -r -y
colcon build \
  --symlink-install \
  --packages-up-to fast_lio \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# shellcheck disable=SC1090
source "$workspace/install/setup.bash"
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio

printf '\nLivox ROS2 driver and FAST-LIO build completed.\n'
printf 'Workspace: %s\n' "$workspace"
