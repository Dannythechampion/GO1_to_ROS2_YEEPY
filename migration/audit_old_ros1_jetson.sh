#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-$HOME/migration_audit}"
catkin_ws="${CATKIN_WS:-$HOME/catkin_ws}"
mkdir -p "$output_dir"
output_file="$output_dir/old_ros1_jetson.txt"

{
  printf '[timestamp]\n'
  date --iso-8601=seconds

  printf '\n[system]\n'
  hostname
  uname -a
  cat /etc/os-release
  python3 --version 2>&1 || true

  printf '\n[network]\n'
  ip -br address
  ip route

  printf '\n[ros-environment]\n'
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
  command -v rosversion >/dev/null 2>&1 && rosversion -d || true

  printf '\n[catkin-workspace]\n'
  printf 'CATKIN_WS=%s\n' "$catkin_ws"
  if [[ -d "$catkin_ws/src" ]]; then
    find "$catkin_ws/src" -name package.xml -print | sort
  else
    printf 'missing: %s/src\n' "$catkin_ws"
  fi

  printf '\n[important-data]\n'
  if [[ -d "$catkin_ws/src" ]]; then
    find "$catkin_ws/src" -type f \
      \( -name '*.yaml' -o -name '*.yml' -o -name '*.pcd' \
         -o -name '*.pgm' -o -name '*.rviz' -o -name '*.launch' \) \
      -printf '%s\t%p\n' | sort -n
  fi

  printf '\n[generated-directories-not-to-transfer]\n'
  for generated in build devel install log; do
    if [[ -e "$catkin_ws/$generated" ]]; then
      du -sh "$catkin_ws/$generated" 2>/dev/null || true
    fi
  done

  printf '\n[ros-topics-if-master-is-running]\n'
  command -v rostopic >/dev/null 2>&1 && timeout 5 rostopic list 2>/dev/null || true

  printf '\n[ros-nodes-if-master-is-running]\n'
  command -v rosnode >/dev/null 2>&1 && timeout 5 rosnode list 2>/dev/null || true
} | tee "$output_file"

printf '\nSaved audit to %s\n' "$output_file"
