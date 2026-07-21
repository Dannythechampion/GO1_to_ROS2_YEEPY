#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_root="$(cd -- "$script_dir/.." && pwd)"
export_dir="${1:-/mnt/t500/go1_ros2_project_export}"

if [[ -e "$export_dir" ]]; then
  printf 'ERROR: export target already exists; refusing to overwrite: %s\n' \
    "$export_dir" >&2
  exit 1
fi
if [[ "$export_dir" != /* ]]; then
  printf 'ERROR: export target must be an absolute path.\n' >&2
  exit 1
fi

required=(
  "$source_root/migration"
  "$source_root/go1_ros2_driver/package.xml"
  "$source_root/package.xml"
  "$source_root/omx_navigation"
)
for path in "${required[@]}"; do
  if [[ ! -e "$path" ]]; then
    printf 'ERROR: required project source is missing: %s\n' "$path" >&2
    exit 1
  fi
done

mkdir -p \
  "$export_dir/migration" \
  "$export_dir/packages/go1_driver" \
  "$export_dir/packages/omx_navigation" \
  "$export_dir/docs"

copy_tree_without_generated_files() {
  local source="$1"
  local destination="$2"
  (
    cd "$source"
    tar \
      --exclude='.git' \
      --exclude='__pycache__' \
      --exclude='.pytest_cache' \
      --exclude='*.pyc' \
      -cf - .
  ) | (
    cd "$destination"
    tar -xf -
  )
}

copy_tree_without_generated_files \
  "$source_root/migration" "$export_dir/migration"
copy_tree_without_generated_files \
  "$source_root/go1_ros2_driver" "$export_dir/packages/go1_driver"

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
  if [[ -d "$source_root/$entry" ]]; then
    mkdir -p "$export_dir/packages/omx_navigation/$entry"
    copy_tree_without_generated_files \
      "$source_root/$entry" "$export_dir/packages/omx_navigation/$entry"
  elif [[ -f "$source_root/$entry" ]]; then
    cp "$source_root/$entry" "$export_dir/packages/omx_navigation/$entry"
  fi
done

cp "$source_root/ROS2_MIGRATION_PLAN.md" "$export_dir/docs/"

cat > "$export_dir/.gitignore" <<'EOF'
# colcon/catkin outputs
build/
install/
log/
devel/

# Python
__pycache__/
*.py[cod]
.pytest_cache/
.venv/
venv/

# ROS data and runtime output
*.bag
*.db3
*.mcap
*.log

# Editors
.vscode/
.idea/
EOF

cat > "$export_dir/README.md" <<'EOF'
# Go1 ROS2 Project

Ubuntu 22.04 / ROS2 Humble deployment project for Unitree Go1 and Livox
MID-360. Start with `migration/README.md` and execute each Gate in order.

The original Ubuntu 20.04 / ROS1 snapshot is intentionally kept in a separate
private repository. It must not be copied wholesale into this ROS2 workspace.
EOF

cat > "$export_dir/ROS1_ARCHIVE.md" <<'EOF'
# ROS1 archive relationship

The original Go1 ROS1 source snapshot is stored in the separate
`go1_project_data` private repository. Record its Git URL and immutable commit
here before deploying:

```text
URL: https://github.com/Dannythechampion/GO-_project_data.git
COMMIT: f18fa0fe1f9e6cdcdabb83e89b628b9bb7ad7b40
```

Only calibration, network values, maps, Unitree SDK baseline and documented
robot-specific changes are migrated from that archive.
EOF

{
  printf 'exported_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'source_layout=%s\n' 'OMX-AI ROS2 selective export'
  printf 'ros1_archive_url=%s\n' \
    'https://github.com/Dannythechampion/GO-_project_data.git'
  printf 'ros1_archive_commit=%s\n' \
    'f18fa0fe1f9e6cdcdabb83e89b628b9bb7ad7b40'
  printf 'unitree_sdk_version=%s\n' 'v3.8.6'
  printf 'unitree_arm64_library_sha256=%s\n' \
    "$(sha256sum "$source_root/GO-_project_data/catkin_ws/src/unitree_legged_sdk/lib/cpp/arm64/libunitree_legged_sdk.a" | awk '{print $1}')"
  printf 'unitree_wrapper_source_sha256=%s\n' \
    "$(sha256sum "$source_root/GO-_project_data/catkin_ws/src/unitree_legged_sdk/python_wrapper/python_interface.cpp" | awk '{print $1}')"
} > "$export_dir/SOURCE_BASELINE.txt"

if find "$export_dir" -type d -name .git -print -quit | grep -q .; then
  printf 'ERROR: nested Git metadata was copied into the export.\n' >&2
  exit 1
fi

if command -v git >/dev/null 2>&1; then
  if ! git -C "$export_dir" init -b main >/dev/null 2>&1; then
    git -C "$export_dir" init >/dev/null
    git -C "$export_dir" branch -m main
  fi
  git -C "$export_dir" add .
  printf 'Created and staged a clean Git working tree.\n'
  git -C "$export_dir" status --short
else
  printf 'WARNING: git is unavailable; files were exported but not initialized.\n' >&2
fi

printf '\nROS2 Git export created at: %s\n' "$export_dir"
printf 'Review ROS1_ARCHIVE.md, commit intentionally, then push to a private remote.\n'
