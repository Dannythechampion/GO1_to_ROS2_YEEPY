#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
src_dir="$workspace/src"

if [[ -d "$repo_root/packages/go1_driver" ]]; then
  go1_source="$repo_root/packages/go1_driver"
  omx_source="$repo_root/packages/omx_navigation"
  pcd_source="$repo_root/packages/omx_pcd_localization"
else
  go1_source="$repo_root/go1_ros2_driver"
  omx_source="$repo_root"
  pcd_source="$repo_root/packages/omx_pcd_localization"
fi
go1_target="$src_dir/go1_driver"
omx_target="$src_dir/omx_navigation"
pcd_target="$src_dir/omx_pcd_localization"

for required in \
  "$go1_source/package.xml" \
  "$omx_source/package.xml" \
  "$omx_source/setup.py" \
  "$omx_source/omx_navigation" \
  "$pcd_source/package.xml" \
  "$pcd_source/CMakeLists.txt" \
  "$pcd_source/src/pcd_localizer.cpp"; do
  if [[ ! -e "$required" ]]; then
    printf 'ERROR: required source is missing: %s\n' "$required" >&2
    exit 1
  fi
done

for target in "$go1_target" "$omx_target" "$pcd_target"; do
  if [[ -e "$target" ]]; then
    printf 'ERROR: target already exists; refusing to overwrite: %s\n' "$target" >&2
    exit 1
  fi
done

mkdir -p "$src_dir"
cp -a "$go1_source" "$go1_target"
mkdir -p "$omx_target"

omx_entries=(
  package.xml
  setup.py
  setup.cfg
  README.md
  resource
  omx_navigation
  config
  launch
  rviz
)

for entry in "${omx_entries[@]}"; do
  if [[ -e "$omx_source/$entry" ]]; then
    cp -a "$omx_source/$entry" "$omx_target/"
  fi
done

cp -a "$pcd_source" "$pcd_target"

printf 'Staged local ROS2 packages:\n'
printf '  %s\n' "$go1_target" "$omx_target" "$pcd_target"
printf '\nNext:\n'
printf '  cd %q\n' "$workspace"
printf '  source /opt/ros/humble/setup.bash\n'
printf '  rosdep install --from-paths src --ignore-src -r -y\n'
printf '  colcon build --symlink-install --packages-select go1_driver omx_navigation omx_pcd_localization\n'
