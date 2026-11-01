#!/usr/bin/env bash
# Run the taped start/goal session's own processes and keep their logs safe.
#
#   tape_field_session.sh start            # mission node, gait monitor, bag
#   tape_field_session.sh status
#   tape_field_session.sh stop             # clean stop, recover, report
#   tape_field_session.sh recover [dir]    # fix bags left by a kill or power cut
#   tape_field_session.sh report  [dir]
#
# The driving stack itself (Livox, FAST-LIO, jetson_field_deploy.sh dry-run or
# armed) runs as before, in its own terminals. Everything this script starts
# writes under one directory on the T500:
#
#   /mnt/t500/go1_field_logs/tape_<UTC>/
#     decisions.jsonl   every mission cycle: what was seen, decided, commanded
#     stability.csv     IMU windows: LiDAR riser vs Go1 trunk, tagged by phase
#     bag/              rosbag2, sqlite3 "resilient" profile, split every 60 s
#     *.log, ros_log/   node output (the systemd journal is volatile here)
#     mission.yaml      copy of the mission file at start and at stop
#     report.md/.png    written by `stop` or `report`
#
# Why it survives a forced stop: the decision log and the CSV are flushed per
# line and fsynced every second; the bag uses sqlite3's WAL journal (the
# "resilient" preset) and closes a file every 60 s, so a power cut costs at
# most the open minute, and `recover` rebuilds a missing metadata.yaml with
# `ros2 bag reindex` -- what was missing when the 2026-10-25 bag was lost.
set -euo pipefail

readonly logs_root="${GO1_FIELD_LOGS:-/mnt/t500/go1_field_logs}"
readonly workspace="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
readonly mission_file="${GO1_MISSION_FILE:-$HOME/.ros/go1_tape_mission.yaml}"
readonly current_link="$logs_root/current"

bag_topics=(
  /livox/imu /go1/imu /go1/robot_state /go1/cmd_vel_applied /go1/control_state
  /go1/manual_override /go1/execution_fault /cmd_vel /cmd_vel_nav /Odometry /scan
  /tf /tf_static /initialpose /goal_pose /plan /navigation/goal_status
  /localization_supervisor/status /localization_supervisor/ready /slam_localization/pose
  /mission/status /mission/decision /gait/stability
)
# The raw cloud lets the tape detection be replayed offline, at ~4 MB/s and a
# recorder busy at ~45% CPU. Off unless asked for: on 2026-11-01 the Jetson
# ran at load 12 on 12 cores with it on and FAST-LIO stalled. GO1_RECORD_LIDAR=1.
[[ "${GO1_RECORD_LIDAR:-0}" == 1 ]] && bag_topics+=(/livox/lidar)

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

ros_environment() {
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH ROS_PACKAGE_PATH
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$workspace/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="${GO1_ROS_DOMAIN_ID:-100}"
}

session_dir() {
  local dir="${1:-}"
  if [[ -z "$dir" ]]; then
    [[ -L "$current_link" ]] || fail "no current session; pass a directory"
    dir="$(readlink -f "$current_link")"
  fi
  [[ -d "$dir" ]] || fail "no such session: $dir"
  printf '%s\n' "$dir"
}

alive() { kill -0 "$1" 2>/dev/null; }

start() {
  if [[ -L "$current_link" ]]; then
    local previous
    previous="$(readlink -f "$current_link")"
    if [[ -f "$previous/pids" ]]; then
      while read -r name pid; do
        alive "$pid" && fail "session $previous is still running ($name pid $pid); stop it first"
      done < "$previous/pids"
    fi
  fi
  ros_environment
  local dir="$logs_root/tape_$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$dir/ros_log"
  ln -sfn "$dir" "$current_link"
  export GO1_SESSION_DIR="$dir" ROS_LOG_DIR="$dir/ros_log" GO1_MISSION_FILE="$mission_file"
  [[ -f "$mission_file" ]] && cp "$mission_file" "$dir/mission_at_start.yaml"
  {
    echo "started_utc=$(date -u +%FT%TZ)"
    echo "ros_domain_id=$ROS_DOMAIN_ID"
    echo "workspace=$workspace"
    echo "mission_file=$mission_file"
    echo "omx_navigation_sha256=$(cd "$workspace/src/omx_navigation" && find omx_navigation -name '*.py' -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-16)"
  } > "$dir/session.txt"

  local lib="$workspace/install/omx_navigation/lib/omx_navigation"
  # Job control gives background jobs a default SIGINT, so `stop` can end
  # each one cleanly; the executables are run directly because `ros2 run`
  # does not pass SIGINT on when started like this.
  set -m
  : > "$dir/pids"
  "$lib/gait_monitor" --ros-args -p log_dir:="$dir" > "$dir/gait_monitor.log" 2>&1 &
  echo "gait_monitor $!" >> "$dir/pids"
  "$lib/tape_mission" --ros-args -p log_dir:="$dir" -p mission_file:="$mission_file" > "$dir/tape_mission.log" 2>&1 &
  echo "tape_mission $!" >> "$dir/pids"
  ros2 bag record --output "$dir/bag" --storage sqlite3 --storage-preset-profile resilient \
    --max-bag-duration 60 "${bag_topics[@]}" > "$dir/bag.log" 2>&1 &
  echo "bag $!" >> "$dir/pids"
  set +m
  sleep 3
  local failed=0
  while read -r name pid; do
    if alive "$pid"; then printf '  %-13s pid %s\n' "$name" "$pid"; else printf '  %-13s DIED, see %s/%s.log\n' "$name" "$dir" "$name"; failed=1; fi
  done < "$dir/pids"
  printf 'Session %s\n' "$dir"
  [[ $failed == 0 ]] || fail "a session process died at start"
}

status() {
  local dir
  dir="$(session_dir "${1:-}")"
  printf 'Session %s\n' "$dir"
  while read -r name pid; do
    printf '  %-13s pid %-7s %s\n' "$name" "$pid" "$(alive "$pid" && echo running || echo stopped)"
  done < "$dir/pids"
  printf '  decisions: %s lines; stability: %s windows; bag: %s\n' \
    "$(wc -l < "$dir/decisions.jsonl" 2>/dev/null || echo 0)" \
    "$(( $(wc -l < "$dir/stability.csv" 2>/dev/null || echo 1) - 1 ))" \
    "$(du -sh "$dir/bag" 2>/dev/null | cut -f1)"
  grep '"kind":"event"' "$dir/decisions.jsonl" 2>/dev/null | tail -n 5 || true
}

stop() {
  local dir
  dir="$(session_dir "${1:-}")"
  # The mission first, so the robot is commanded to stand before the
  # recorder that would show it goes.
  for wanted in tape_mission gait_monitor bag; do
    while read -r name pid; do
      [[ "$name" == "$wanted" ]] || continue
      alive "$pid" || continue
      kill -INT "$pid" 2>/dev/null || true
      for _ in $(seq 1 75); do alive "$pid" || break; sleep 0.2; done
      if alive "$pid"; then
        printf '%s did not stop on SIGINT; sending SIGTERM\n' "$name" >&2
        kill -TERM "$pid" 2>/dev/null || true
        for _ in $(seq 1 25); do alive "$pid" || break; sleep 0.2; done
        alive "$pid" && kill -KILL "$pid" 2>/dev/null || true
      fi
      printf 'stopped %s\n' "$name"
    done < "$dir/pids"
  done
  [[ -f "$mission_file" ]] && cp "$mission_file" "$dir/mission_at_stop.yaml"
  echo "stopped_utc=$(date -u +%FT%TZ)" >> "$dir/session.txt"
  recover "$dir"
  report "$dir"
}

recover() {
  ros_environment
  local dirs=()
  if [[ -n "${1:-}" ]]; then dirs=("$1"); else mapfile -t dirs < <(find "$logs_root" -maxdepth 1 -name 'tape_*' -type d | sort); fi
  for dir in "${dirs[@]}"; do
    local bag="$dir/bag"
    [[ -d "$bag" ]] || continue
    if [[ -f "$bag/metadata.yaml" ]]; then
      printf 'bag ok: %s\n' "$bag"
      continue
    fi
    printf 'reindexing %s (no metadata.yaml: the recorder was killed)\n' "$bag"
    ros2 bag reindex -s sqlite3 "$bag" || printf 'WARNING: reindex failed for %s; the .db3 files are still readable with tools/rosbag_lite.py\n' "$bag" >&2
  done
}

report() {
  local dir
  dir="$(session_dir "${1:-}")"
  ros_environment
  python3 -m omx_navigation.tape_report "$dir" > /dev/null && printf 'report: %s/report.md\n' "$dir"
}

command="${1:-}"
shift || true
case "$command" in
  start) start ;;
  status) status "${1:-}" ;;
  stop) stop "${1:-}" ;;
  recover) recover "${1:-}" ;;
  report) report "${1:-}" ;;
  *) sed -n '2,8p' "$0"; exit 2 ;;
esac
