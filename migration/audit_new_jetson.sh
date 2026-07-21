#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-$HOME/migration_audit}"
mkdir -p "$output_dir"
output_file="$output_dir/new_jetson_system.txt"

run_optional() {
  local label="$1"
  shift
  printf '\n[%s]\n' "$label"
  if command -v "$1" >/dev/null 2>&1; then
    "$@" 2>&1 || true
  else
    printf 'not installed: %s\n' "$1"
  fi
}

{
  printf '[timestamp]\n'
  date --iso-8601=seconds

  printf '\n[hostname]\n'
  hostname

  printf '\n[architecture]\n'
  uname -m

  printf '\n[kernel]\n'
  uname -a

  printf '\n[os-release]\n'
  cat /etc/os-release

  printf '\n[python]\n'
  python3 --version 2>&1 || true
  python3-config --extension-suffix 2>&1 || true

  printf '\n[network-addresses]\n'
  ip -br address

  printf '\n[network-routes]\n'
  ip route

  run_optional "network-manager" nmcli device status
  run_optional "block-devices" lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINTS,MODEL
  run_optional "jetson-release" jetson_release
  run_optional "nvidia-smi" nvidia-smi
  run_optional "cuda" nvcc --version

  printf '\n[jetpack-packages]\n'
  dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null \
    | grep -E '^(nvidia-jetpack|nvidia-l4t-|cuda-)' || true

  printf '\n[ros-packages]\n'
  dpkg-query -W -f='${binary:Package}\t${Version}\n' 2>/dev/null \
    | grep -E '^ros-(humble|foxy|iron|jazzy)-' || true

  printf '\n[ros-environment]\n'
  printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-}"
  if command -v ros2 >/dev/null 2>&1; then
    command -v ros2
    ros2 pkg list 2>/dev/null | wc -l
  else
    printf 'ros2 command not found\n'
  fi

  printf '\n[ssh-service]\n'
  systemctl is-enabled ssh 2>&1 || true
  systemctl is-active ssh 2>&1 || true

  printf '\n[disk-usage]\n'
  df -hT
} | tee "$output_file"

printf '\nSaved audit to %s\n' "$output_file"
