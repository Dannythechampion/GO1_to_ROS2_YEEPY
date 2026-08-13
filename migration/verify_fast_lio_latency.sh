#!/usr/bin/env bash
set -eo pipefail

export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
go1_ros2_ws="${GO1_ROS2_WS:-/mnt/t500/go1_ros2_ws}"
source "$go1_ros2_ws/install/setup.bash"
set -euo pipefail

mode="${1:-}"
duration="${2:-}"
shift "$(( $# >= 2 ? 2 : $# ))"

if [[ "$mode" != "fast-lio" && "$mode" != "amcl" ]]; then
  printf 'Usage: %s fast-lio|amcl DURATION_SEC [--observe-only]\n' "$0" >&2
  exit 2
fi
if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "$duration" == "0" ]]; then
  printf 'ERROR: DURATION_SEC must be positive.\n' >&2
  exit 2
fi

observe_only=false
for argument in "$@"; do
  case "$argument" in
    --observe-only) observe_only=true ;;
    *)
      printf 'ERROR: unsupported argument: %s\n' "$argument" >&2
      exit 2
      ;;
  esac
done
if [[ "$observe_only" == true && "$mode" != "fast-lio" ]]; then
  printf 'ERROR: --observe-only is valid only with fast-lio.\n' >&2
  exit 2
fi

project_root="${GO1_PROJECT_ROOT:-/mnt/t500/go1_ros2_project}"
runtime_root="${GO1_RUNTIME_ROOT:-/mnt/t500/go1_runtime}"
process_root="$runtime_root/processes"
fast_lio_log="${GO1_FAST_LIO_LOG:-$process_root/fast_lio.log}"
navigation_log="${GO1_NAVIGATION_LOG:-$process_root/navigation.log}"
evidence_root="$runtime_root/latency"
stamp="$(date +%Y%m%d_%H%M%S)"
suffix="$mode"
if [[ "$observe_only" == true ]]; then
  suffix="diagnostic"
fi
evidence_dir="$evidence_root/${stamp}-${suffix}"
mkdir -p "$evidence_dir"

arm_value="$(timeout 10 ros2 param get /go1_driver arm)"
if [[ "$arm_value" != "Boolean value is: False" ]]; then
  printf 'ERROR: /go1_driver must remain arm=false, got: %s\n' "$arm_value" >&2
  exit 1
fi
control_state="$(timeout 10 ros2 topic echo /go1/control_state --once)"
if [[ "$control_state" != *"DRY-RUN"* ]]; then
  printf 'ERROR: /go1/control_state is not DRY-RUN.\n' >&2
  exit 1
fi

livox_info="$(timeout 10 ros2 topic info /livox/lidar)"
if ! grep -Fxq 'Publisher count: 1' <<<"$livox_info"; then
  printf 'ERROR: /livox/lidar must have Publisher count: 1.\n%s\n' "$livox_info" >&2
  exit 1
fi
odom_info="$(timeout 10 ros2 topic info /Odometry)"
if ! grep -Fxq 'Publisher count: 1' <<<"$odom_info"; then
  printf 'ERROR: /Odometry must have Publisher count: 1.\n%s\n' "$odom_info" >&2
  exit 1
fi

fast_lio_pids="$(pgrep -x fastlio_mapping || true)"
if [[ "$(grep -c . <<<"$fast_lio_pids")" -ne 1 ]]; then
  printf 'ERROR: expected exactly one fastlio_mapping process, got:\n%s\n' "$fast_lio_pids" >&2
  exit 1
fi
fast_lio_pid_before="$fast_lio_pids"

if [[ ! -r "$fast_lio_log" ]]; then
  printf 'ERROR: FAST-LIO log is not readable: %s\n' "$fast_lio_log" >&2
  exit 1
fi
fast_lio_log_start="$(wc -l < "$fast_lio_log")"
navigation_log_start=0
if [[ "$mode" == "amcl" ]]; then
  if [[ ! -r "$navigation_log" ]]; then
    printf 'ERROR: navigation log is not readable: %s\n' "$navigation_log" >&2
    exit 1
  fi
  navigation_log_start="$(wc -l < "$navigation_log")"
fi

warmup=0
if [[ "$mode" == "fast-lio" && "$observe_only" == false ]]; then
  if awk "BEGIN { exit !($duration > 60) }"; then
    warmup=60
  fi
fi

probe_args=(
  python3 "$project_root/migration/measure_localization_latency.py"
  --mode "$mode"
  --duration "$duration"
  --warmup "$warmup"
  --output "$evidence_dir/latency.json"
  --age-p95 0.10
  --age-max 0.30
  --rate-min 9.0
  --rate-max 11.0
  --fast-lio-log "$fast_lio_log"
  --log-start-line "$fast_lio_log_start"
)
if [[ "$observe_only" == true ]]; then
  probe_args+=("--observe-only")
fi

set +e
"${probe_args[@]}" > >(tee "$evidence_dir/probe.stdout") \
  2> >(tee "$evidence_dir/probe.stderr" >&2)
probe_status=$?
set -e

tail -n "+$((fast_lio_log_start + 1))" "$fast_lio_log" \
  | grep '\[fast_lio_realtime\]' > "$evidence_dir/fast_lio_realtime.log" || true

fast_lio_pid_after="$(pgrep -x fastlio_mapping || true)"
if [[ "$fast_lio_pid_after" != "$fast_lio_pid_before" ]]; then
  printf 'ERROR: FAST-LIO process restarted during measurement (%s -> %s).\n' \
    "$fast_lio_pid_before" "$fast_lio_pid_after" >&2
  exit 1
fi

navigation_error_count=0
if [[ "$mode" == "amcl" ]]; then
  tail -n "+$((navigation_log_start + 1))" "$navigation_log" \
    > "$evidence_dir/navigation_interval.log"
  navigation_error_count="$(grep -Eic \
    'Message Filter dropping message|Timed out waiting for transform|extrapolation' \
    "$evidence_dir/navigation_interval.log" || true)"
  if [[ "$navigation_error_count" -ne 0 ]]; then
    printf 'ERROR: navigation emitted %s new timestamp/TF errors.\n' \
      "$navigation_error_count" >&2
    exit 1
  fi
fi

control_state_after="$(timeout 10 ros2 topic echo /go1/control_state --once)"
if [[ "$control_state_after" != *"DRY-RUN"* ]]; then
  printf 'ERROR: DRY-RUN was lost during measurement.\n' >&2
  exit 1
fi

{
  printf 'mode=%s\n' "$mode"
  printf 'duration_sec=%s\n' "$duration"
  printf 'warmup_sec=%s\n' "$warmup"
  printf 'observe_only=%s\n' "$observe_only"
  printf 'fast_lio_pid_before=%s\n' "$fast_lio_pid_before"
  printf 'fast_lio_pid_after=%s\n' "$fast_lio_pid_after"
  printf 'navigation_error_count=%s\n' "$navigation_error_count"
  printf 'arm=%s\n' "$arm_value"
  printf 'control_state=DRY-RUN\n'
  printf 'probe_status=%s\n' "$probe_status"
} > "$evidence_dir/metadata.txt"

if [[ "$probe_status" -ne 0 ]]; then
  printf 'ERROR: latency probe failed; evidence: %s\n' "$evidence_dir" >&2
  exit "$probe_status"
fi
printf 'PASS: latency evidence saved to %s\n' "$evidence_dir"
