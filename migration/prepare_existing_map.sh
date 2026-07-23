#!/usr/bin/env bash
set -euo pipefail

source_yaml="${1:-/home/unicon/ros2_ws/src/navigation2/nav2_map_server/map/scans_new.yaml}"
source_pgm="${2:-${source_yaml%.yaml}.pgm}"
target_dir="${3:-/mnt/t500/maps}"

test -f "$source_yaml"
test -f "$source_pgm"

mkdir -p "$target_dir"
install -m 0644 "$source_yaml" "$target_dir/scans_new.yaml"
install -m 0644 "$source_pgm" "$target_dir/scans_new.pgm"
sed -i \
  's#^[[:space:]]*image:.*#image: scans_new.pgm#' \
  "$target_dir/scans_new.yaml"

grep -q '^image: scans_new.pgm$' "$target_dir/scans_new.yaml"
printf 'Prepared existing map:\n'
printf '  %s\n' "$target_dir/scans_new.yaml"
printf '  %s\n' "$target_dir/scans_new.pgm"
