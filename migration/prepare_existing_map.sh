#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd -- "$script_dir/.." && pwd -P)"
session_id="${GO1_MAP_SESSION_ID:-20260728_204825}"

source_yaml="${1:-$repo_root/maps/hanyang_9f/$session_id/slam_toolbox/hanyang_9f_annotated.yaml}"
source_pgm="${2:-${source_yaml%.yaml}.pgm}"
target_dir="${3:-/mnt/t500/maps/hanyang_9f/$session_id/slam_toolbox}"
target_yaml="$target_dir/hanyang_9f_annotated.yaml"
target_pgm="$target_dir/hanyang_9f_annotated.pgm"

test -f "$source_yaml"
test -f "$source_pgm"

mkdir -p "$target_dir"
install -m 0644 "$source_yaml" "$target_yaml"
install -m 0644 "$source_pgm" "$target_pgm"
sed -i \
  's#^[[:space:]]*image:.*#image: hanyang_9f_annotated.pgm#' \
  "$target_yaml"

image_line="$(sed -n '1{s/\r$//;p;}' "$target_yaml")"
test "$image_line" = 'image: hanyang_9f_annotated.pgm'
printf 'Prepared existing map:\n'
printf '  %s\n' "$target_yaml"
printf '  %s\n' "$target_pgm"
