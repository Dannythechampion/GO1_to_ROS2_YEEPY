#!/usr/bin/env bash
set -euo pipefail

readonly repository_url="https://github.com/Livox-SDK/Livox-SDK2.git"
readonly expected_commit="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
third_party_root="${GO1_THIRD_PARTY:-/mnt/t500/go1_third_party}"
source_dir="$third_party_root/Livox-SDK2"
build_dir="$source_dir/build"

command -v git >/dev/null 2>&1 || {
  printf 'ERROR: git is not installed.\n' >&2
  exit 1
}
command -v cmake >/dev/null 2>&1 || {
  printf 'ERROR: cmake is not installed.\n' >&2
  exit 1
}

mkdir -p "$third_party_root"
if [[ ! -e "$source_dir" ]]; then
  git clone "$repository_url" "$source_dir"
elif [[ ! -d "$source_dir/.git" ]]; then
  printf 'ERROR: target exists but is not a Git checkout: %s\n' "$source_dir" >&2
  exit 1
fi

if ! git -C "$source_dir" diff --quiet \
  || ! git -C "$source_dir" diff --cached --quiet; then
  printf 'ERROR: Livox-SDK2 checkout has local changes; refusing to switch commits.\n' >&2
  exit 1
fi

git -C "$source_dir" fetch --tags origin
git -C "$source_dir" checkout --detach "$expected_commit"

cmake -S "$source_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel "$(nproc)"
sudo cmake --install "$build_dir"
sudo ldconfig

if [[ ! -f /usr/local/lib/liblivox_lidar_sdk_static.a ]]; then
  printf 'ERROR: expected Livox SDK2 static library was not installed.\n' >&2
  exit 1
fi

printf 'Livox-SDK2 installed successfully.\n'
printf 'Source: %s\n' "$source_dir"
printf 'Commit: %s\n' "$(git -C "$source_dir" rev-parse HEAD)"
printf 'Library: /usr/local/lib/liblivox_lidar_sdk_static.a\n'
