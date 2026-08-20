#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
src_dir="$workspace/src"

if [[ -d "$repo_root/packages/go1_driver" ]]; then
  go1_source="$repo_root/packages/go1_driver"
  omx_source="$repo_root/packages/omx_navigation"
else
  go1_source="$repo_root/go1_ros2_driver"
  omx_source="$repo_root"
fi
go1_target="$src_dir/go1_driver"
omx_target="$src_dir/omx_navigation"
maps_source="$repo_root/maps"

for required in \
  "$go1_source/package.xml" \
  "$omx_source/package.xml" \
  "$omx_source/setup.py" \
  "$omx_source/omx_navigation" \
  "$maps_source"; do
  if [[ ! -e "$required" ]]; then
    printf 'ERROR: required source is missing: %s\n' "$required" >&2
    exit 1
  fi
done

for target in "$go1_target" "$omx_target"; do
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
  test
)

for entry in "${omx_entries[@]}"; do
  if [[ -e "$omx_source/$entry" ]]; then
    cp -a "$omx_source/$entry" "$omx_target/"
  fi
done

# Keep the bundled pose-graph map with the staged package so its setup.py can
# install it on the Jetson without relying on the source repository path.
cp -a "$maps_source" "$omx_target/maps"
cp -a "$repo_root/migration/verify_posegraph_navigation.sh" "$omx_target/verify_posegraph_navigation.sh"
cp -a "$repo_root/migration/jetson_field_deploy.sh" "$omx_target/jetson_field_deploy.sh"
chmod +x "$omx_target/verify_posegraph_navigation.sh" "$omx_target/jetson_field_deploy.sh"

printf 'Staged local ROS2 packages:\n'
printf '  %s\n' "$go1_target" "$omx_target"
printf '\nNext:\n'
printf '  cd %q\n' "$workspace"
printf '  source /opt/ros/humble/setup.bash\n'
printf '  rosdep install --from-paths src --ignore-src -r -y\n'
printf '  colcon build --symlink-install --packages-select go1_driver omx_navigation\n'
