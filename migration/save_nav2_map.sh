#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
map_root="${GO1_MAP_ROOT:-/mnt/t500/maps}"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -euo pipefail

map_root="$(realpath -m "$map_root")"
map_prefix="${1:-$map_root/floor9_v001/go1_map}"
map_prefix="${map_prefix%.yaml}"
map_prefix="${map_prefix%.pgm}"
map_prefix="$(realpath -m "$map_prefix")"

case "$map_prefix" in
  "$map_root"/*) ;;
  *)
    printf 'ERROR: map prefix must stay under %s: %s\n' \
      "$map_root" "$map_prefix" >&2
    exit 2
    ;;
esac

if [[ -e "${map_prefix}.yaml" || -e "${map_prefix}.pgm" ]]; then
  printf 'ERROR: refusing to overwrite an existing map version: %s\n' \
    "$map_prefix" >&2
  exit 3
fi

timeout 10 ros2 topic list | grep -Fxq /map
timeout 10 ros2 topic list | grep -Fxq /scan
timeout 10 ros2 topic list | grep -Fxq /Odometry

mkdir -p "$(dirname "$map_prefix")"
ros2 run nav2_map_server map_saver_cli -f "$map_prefix"

test -s "${map_prefix}.yaml"
test -s "${map_prefix}.pgm"
grep -Eq '^[[:space:]]*image:[[:space:]]*.*\.pgm[[:space:]]*$' \
  "${map_prefix}.yaml"

printf 'Saved Nav2 map version:\n'
printf '  %s\n' "${map_prefix}.yaml"
printf '  %s\n' "${map_prefix}.pgm"
