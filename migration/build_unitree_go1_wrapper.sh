#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  printf 'Usage: %s /absolute/path/to/unitree_legged_sdk\n' "$0" >&2
  exit 2
fi

source_dir="$(readlink -f "$1")"
target_root="${GO1_SDK_ROOT:-$HOME/go1_sdk}"
target_dir="$target_root/unitree_legged_sdk"
readonly expected_arm64_library_sha256="4ec2f384271ecc6cc4266e10b888d5bb076d73f10ee680436718c26d1d865a6d"
readonly expected_wrapper_source_sha256="d98151de542eacb74532af6aba35d79b36bed9398c8de09c0aaa1724aad049b7"

if [[ "$(uname -m)" != "aarch64" ]]; then
  printf 'ERROR: this wrapper must be built natively on the ARM64 Jetson.\n' >&2
  printf 'Detected architecture: %s\n' "$(uname -m)" >&2
  exit 1
fi

required_sources=(
  CMakeLists.txt
  python_wrapper/CMakeLists.txt
  python_wrapper/python_interface.cpp
  lib/cpp/arm64/libunitree_legged_sdk.a
  include/unitree_legged_sdk/unitree_legged_sdk.h
)
for relative in "${required_sources[@]}"; do
  if [[ ! -e "$source_dir/$relative" ]]; then
    printf 'ERROR: source is not the expected Go1 SDK snapshot; missing %s\n' \
      "$source_dir/$relative" >&2
    exit 1
  fi
done

actual_library_sha256="$(sha256sum \
  "$source_dir/lib/cpp/arm64/libunitree_legged_sdk.a" | awk '{print $1}')"
actual_wrapper_sha256="$(sha256sum \
  "$source_dir/python_wrapper/python_interface.cpp" | awk '{print $1}')"
if [[ "$actual_library_sha256" != "$expected_arm64_library_sha256" \
   || "$actual_wrapper_sha256" != "$expected_wrapper_source_sha256" ]]; then
  printf 'ERROR: Unitree SDK snapshot does not match the archived Go1 baseline.\n' >&2
  printf 'ARM64 library: expected=%s actual=%s\n' \
    "$expected_arm64_library_sha256" "$actual_library_sha256" >&2
  printf 'Wrapper source: expected=%s actual=%s\n' \
    "$expected_wrapper_source_sha256" "$actual_wrapper_sha256" >&2
  printf 'Do not mix another SDK release with this wrapper.\n' >&2
  exit 1
fi

if [[ -e "$target_dir" ]]; then
  printf 'ERROR: target already exists; refusing to overwrite: %s\n' "$target_dir" >&2
  exit 1
fi

for command_name in cmake g++ python3 python3-config; do
  command -v "$command_name" >/dev/null 2>&1 || {
    printf 'ERROR: required command is unavailable: %s\n' "$command_name" >&2
    exit 1
  }
done

mkdir -p "$target_root"
cp -a "$source_dir" "$target_dir"

wrapper_cmake="$target_dir/python_wrapper/CMakeLists.txt"
if grep -q 'add_subdirectory(third-party/pybind11)' "$wrapper_cmake"; then
  sed -i \
    's#add_subdirectory(third-party/pybind11)#find_package(pybind11 CONFIG REQUIRED)#' \
    "$wrapper_cmake"
fi
if ! grep -q 'find_package(pybind11 CONFIG REQUIRED)' "$wrapper_cmake"; then
  printf 'ERROR: could not configure the wrapper to use the system pybind11.\n' >&2
  exit 1
fi

build_dir="$target_dir/build"
cmake -S "$target_dir" -B "$build_dir" \
  -DPYTHON_BUILD=ON \
  -DPYTHON_EXECUTABLE="$(command -v python3)" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel "$(nproc)"

extension_suffix="$(python3-config --extension-suffix)"
module_path="$target_dir/lib/python/arm64/robot_interface${extension_suffix}"
if [[ ! -f "$module_path" ]]; then
  printf 'ERROR: wrapper for the active Python ABI was not generated.\n' >&2
  printf 'Expected: %s\n' "$module_path" >&2
  find "$target_dir/lib/python/arm64" -maxdepth 1 -type f -name 'robot_interface*.so' -print
  exit 1
fi

if ! file "$module_path" | grep -Eq 'ARM aarch64|ARM64'; then
  printf 'ERROR: generated wrapper is not an ARM64 binary: %s\n' "$module_path" >&2
  file "$module_path" >&2
  exit 1
fi

if ldd "$module_path" | grep -q 'not found'; then
  printf 'ERROR: generated wrapper has unresolved shared-library dependencies.\n' >&2
  ldd "$module_path" >&2
  exit 1
fi

PYTHONPATH="$target_dir/lib/python/arm64${PYTHONPATH:+:$PYTHONPATH}" \
  python3 - <<'PY'
import robot_interface as sdk

print("Module:", sdk.__file__)
print("HighCmd:", type(sdk.HighCmd()))
print("HighState:", type(sdk.HighState()))
print("Unitree Go1 wrapper import OK")
PY

env_file="$target_root/setup_unitree_sdk.bash"
cat > "$env_file" <<EOF
#!/usr/bin/env bash
export UNITREE_SDK_ROOT="$target_dir"
export PYTHONPATH="\$UNITREE_SDK_ROOT/lib/python/arm64:\${PYTHONPATH:-}"
EOF
chmod +x "$env_file"

{
  printf 'source=%s\n' "$source_dir"
  printf 'built_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'architecture=%s\n' "$(uname -m)"
  printf 'python=%s\n' "$(python3 --version 2>&1)"
  printf 'extension_suffix=%s\n' "$extension_suffix"
} > "$target_root/BUILD_INFO.txt"

printf '\nUnitree Go1 Python wrapper build passed.\n'
printf 'Module: %s\n' "$module_path"
printf 'Environment helper: %s\n' "$env_file"
