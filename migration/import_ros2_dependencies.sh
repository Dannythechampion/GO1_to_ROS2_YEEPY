#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repos_file="${1:-$script_dir/ros2.repos}"
workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
src_dir="$workspace/src"

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf 'ERROR: ROS 2 Humble is not installed.\n' >&2
  exit 1
fi

if [[ ! -r "$repos_file" ]]; then
  printf 'ERROR: repos file not found: %s\n' "$repos_file" >&2
  exit 1
fi

command -v vcs >/dev/null 2>&1 || {
  printf 'ERROR: vcs is not installed. Run bootstrap_ros2_humble.sh first.\n' >&2
  exit 1
}

mkdir -p "$src_dir"

if find "$src_dir" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  printf 'ERROR: %s is not empty. Refusing to overwrite an existing workspace.\n' \
    "$src_dir" >&2
  printf 'Import into an empty src directory or merge repositories manually.\n' >&2
  exit 1
fi

vcs import --recursive "$src_dir" < "$repos_file"

while IFS= read -r -d '' git_dir; do
  repo_dir="${git_dir%/.git}"
  git -C "$repo_dir" submodule update --init --recursive
done < <(find "$src_dir" -mindepth 2 -maxdepth 2 -type d -name .git -print0)

vcs status "$src_dir"

printf '\nImported pinned ROS 2 dependencies into %s\n' "$src_dir"
printf 'Do not build until Livox-SDK2 prerequisites and MID-360 network values are verified.\n'
