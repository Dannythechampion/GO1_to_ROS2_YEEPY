# FAST-LIO Realtime Buffer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent FAST-LIO's internal LiDAR deque from preserving a permanent backlog, then prove `/Odometry`, `/scan`, and AMCL remain time-aligned during sustained operation.

**Architecture:** Extend the existing migration patcher instead of vendoring FAST_LIO_ROS2. First add internal low-rate diagnostics and capture a diagnostic-only baseline, then add a bounded queue policy that retains one IMU-waiting scan plus the newest queued scan. Use a small ROS 2 probe for timestamp statistics and keep all Go1 motion paths disarmed.

**Tech Stack:** Python 3 patch tooling and pytest, C++17, ROS 2 Humble/rclcpp, FAST_LIO_ROS2 at pinned commit `2fffc570a25d0df172720bac034fbdb6a13d2162`, Bash, colcon, Livox MID-360

## Global Constraints

- Work only in the existing isolated worktree `C:/Users/kimgk/OneDrive/문서/OMX-AI/.codex-worktrees/fix-scan-qos` on branch `codex/fix-scan-qos`.
- Keep `/go1_driver` parameter `arm=false`; never activate motion or send a Nav2 goal.
- Keep exactly one Livox publisher and one FAST-LIO process during runtime verification.
- Keep the paired invariant `lidar_buffer.size() == time_buffer.size()` for every mutation.
- Preserve an in-flight front scan while it is waiting for IMU unless it is older than `0.20 s` and a newer waiting scan exists.
- Bound the normal internal LiDAR queue to two scans: one in-flight front plus one newest waiting scan.
- Do not save an initial pose in this plan.
- FAST-LIO soak pass: after 60 seconds of initialization, nine continuous minutes with `/Odometry` and `/scan` at 9-11 Hz, header-age p95 <= `0.10 s`, maximum <= `0.30 s`, queue depth <= 2 without growth, processing p95 <= `100 ms`, zero timestamp reversals, zero process restarts, and zero duplicate publishers.
- AMCL pass: three continuous minutes with `/amcl_pose` header-age p95 <= `0.10 s`, maximum <= `0.30 s`, zero TF/costmap timestamp drops, stationary speed <= `0.01`, covariance x/y <= `0.04`, covariance yaw <= `0.0305`, ten-sample position spread <= `0.10 m`, yaw spread <= 5 degrees, and scan-map residual passing the existing supervisor.

---

## File Structure

- Modify `migration/patch_fast_lio_low_latency.py`: deterministic transformation of the pinned FAST-LIO source; expose diagnostic-only and final bounded-buffer modes.
- Replace `migration/test_fast_lio_low_latency_patch.py`: realistic source fixture and behavioral/static contract tests for diagnostics, queue pairing, replacement, stale in-flight handling, idempotence, and rejection.
- Modify `migration/build_livox_fastlio.sh`: select the patch mode with `FAST_LIO_LOW_LATENCY_MODE=diagnostic|bounded`, defaulting to `bounded`.
- Create `migration/measure_localization_latency.py`: subscribe only to small ROS messages, compute per-topic age/rate/reversal statistics, verify stationary odometry and AMCL covariance/spread, and write JSON evidence.
- Create `migration/verify_fast_lio_latency.sh`: enforce `arm=false`, unique publishers/processes, invoke the probe for the requested duration, inspect FAST-LIO diagnostic logs, and fail on threshold violations.
- Create `migration/test_latency_verifier.py`: pure unit tests for statistics and static safety/threshold tests for the shell verifier.
- Modify `migration/README.md`: document diagnostic baseline, bounded deployment, ten-minute soak, and AMCL three-minute verification commands.
- Create runtime evidence only on Jetson under `/mnt/t500/go1_runtime/latency/`; do not commit generated logs.

### Task 1: Add FAST-LIO Diagnostic-Only Patch Mode

**Files:**
- Modify: `migration/patch_fast_lio_low_latency.py`
- Replace: `migration/test_fast_lio_low_latency_patch.py`

**Interfaces:**
- Consumes: pinned `laserMapping.cpp` path and optional CLI `--mode diagnostic|bounded`.
- Produces: `patch(source_path: Path, mode: str = "bounded") -> None`; diagnostic marker `FAST_LIO_REALTIME_DIAGNOSTICS`; log prefix `[fast_lio_realtime]`.

- [ ] **Step 1: Replace the tiny fixture with relevant pinned-source blocks and failing diagnostic tests**

```python
def test_diagnostic_mode_adds_observation_without_bounding_queue(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    result = run_patch(source, "diagnostic")
    assert result.returncode == 0, result.stderr
    text = source.read_text(encoding="utf-8")
    assert "FAST_LIO_REALTIME_DIAGNOSTICS" in text
    assert '"[fast_lio_realtime] queue_depth=%zu front_age=%.6f drops=%zu process_ms=%.3f imu_margin=%.6f"' in text
    assert "lidar_buffer.push_back(ptr);" in text
    assert "FAST_LIO_BOUNDED_BUFFER" not in text


def test_diagnostic_mode_clears_paired_lidar_queues_on_time_reversal(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "diagnostic", check=True)
    text = source.read_text(encoding="utf-8")
    assert "lidar_buffer.clear();\n        time_buffer.clear();\n        lidar_pushed = false;" in text
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run: `python -m pytest migration/test_fast_lio_low_latency_patch.py -v`

Expected: FAIL because the current patcher accepts no mode, has no realistic transformations, and emits no diagnostics.

- [ ] **Step 3: Implement deterministic diagnostic-mode transformations**

Implement these exact behaviors in `patch_fast_lio_low_latency.py`:

```python
VALID_MODES = {"diagnostic", "bounded"}


def replace_exactly_once(source: str, old: str, new: str, label: str) -> str:
    count = source.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one {label}, found {count}")
    return source.replace(old, new, 1)


def patch(source_path: Path, mode: str = "bounded") -> None:
    if mode not in VALID_MODES:
        raise ValueError(f"unsupported mode: {mode}")
    # Recognize the final marker for the requested mode and return unchanged.
    # Transform all blocks in memory, then write once so failures are atomic.
```

The diagnostic C++ insertion must:

- add counters for drops, last/max processing time, and last 1 Hz log time;
- acquire `mtx_buffer` for all `sync_packages()` deque access;
- clear both paired LiDAR deques and reset `lidar_pushed` on timestamp reversal;
- calculate queue depth, front age using `this->get_clock()->now().seconds()`, and IMU margin;
- log at most once per second using `RCLCPP_INFO` and use `RCLCPP_WARN` only when depth > 2 or age > 0.30;
- record total timer processing milliseconds at every early return as well as the normal end path.

The CLI must be:

```python
parser.add_argument("--mode", choices=sorted(VALID_MODES), default="bounded")
parser.add_argument("source", type=Path)
patch(args.source, args.mode)
```

- [ ] **Step 4: Add idempotence and rejection tests**

```python
def test_diagnostic_patch_is_idempotent(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "diagnostic", check=True)
    once = source.read_bytes()
    run_patch(source, "diagnostic", check=True)
    assert source.read_bytes() == once


def test_patch_rejects_unknown_upstream_without_partial_write(tmp_path):
    source = tmp_path / "laserMapping.cpp"
    source.write_text("unrecognized source\n", encoding="utf-8")
    before = source.read_bytes()
    result = run_patch(source, "diagnostic")
    assert result.returncode != 0
    assert source.read_bytes() == before
```

- [ ] **Step 5: Run focused tests and verify GREEN**

Run: `python -m pytest migration/test_fast_lio_low_latency_patch.py -v`

Expected: all diagnostic, pairing, idempotence, and rejection tests PASS.

- [ ] **Step 6: Commit the diagnostic patch mode**

```bash
git add migration/patch_fast_lio_low_latency.py migration/test_fast_lio_low_latency_patch.py
git commit -m "feat: instrument FAST-LIO input latency"
```

### Task 2: Add the Bounded In-Flight-Plus-Latest Queue Policy

**Files:**
- Modify: `migration/patch_fast_lio_low_latency.py`
- Modify: `migration/test_fast_lio_low_latency_patch.py`

**Interfaces:**
- Consumes: Task 1 diagnostic transformation and the same pinned source fixture.
- Produces: final marker `FAST_LIO_BOUNDED_BUFFER`; helper C++ functions `clear_lidar_buffers_locked()`, `enqueue_latest_lidar_locked(PointCloudXYZI::Ptr, double)`, and `discard_stale_inflight_locked(double)`.

- [ ] **Step 1: Write failing bounded-policy tests**

```python
def test_bounded_mode_keeps_front_and_latest_waiting_scan(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "bounded", check=True)
    text = source.read_text(encoding="utf-8")
    assert "FAST_LIO_BOUNDED_BUFFER" in text
    assert "const size_t keep_count = lidar_pushed ? 1U : 0U;" in text
    assert "while (lidar_buffer.size() > keep_count)" in text
    assert "lidar_buffer.pop_back();" in text
    assert "time_buffer.pop_back();" in text
    assert "enqueue_latest_lidar_locked(ptr, last_timestamp_lidar);" in text


def test_stale_inflight_is_replaced_only_when_latest_waits(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "bounded", check=True)
    text = source.read_text(encoding="utf-8")
    assert "lidar_pushed && lidar_buffer.size() > 1" in text
    assert "newest_lidar_time - time_buffer.front() > 0.20" in text
    assert "lidar_buffer.pop_front();" in text
    assert "time_buffer.pop_front();" in text
    assert "lidar_pushed = false;" in text
```

- [ ] **Step 2: Run only the new tests and verify RED**

Run: `python -m pytest migration/test_fast_lio_low_latency_patch.py -v -k "bounded or stale_inflight"`

Expected: FAIL because bounded helpers and marker do not exist.

- [ ] **Step 3: Implement the minimal bounded helpers and callback integration**

Insert helpers adjacent to the global deques, with the mutex contract in names/comments:

```cpp
// FAST_LIO_BOUNDED_BUFFER: caller must hold mtx_buffer.
void clear_lidar_buffers_locked() {
    lidar_buffer.clear();
    time_buffer.clear();
    lidar_pushed = false;
}

void enqueue_latest_lidar_locked(PointCloudXYZI::Ptr scan, double stamp) {
    const size_t keep_count = lidar_pushed ? 1U : 0U;
    while (lidar_buffer.size() > keep_count) {
        lidar_buffer.pop_back();
        time_buffer.pop_back();
        ++lidar_drop_count;
    }
    lidar_buffer.push_back(std::move(scan));
    time_buffer.push_back(stamp);
}

void discard_stale_inflight_locked(double newest_lidar_time) {
    if (lidar_pushed && lidar_buffer.size() > 1 &&
        newest_lidar_time - time_buffer.front() > 0.20) {
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        ++lidar_drop_count;
    }
}
```

Call `discard_stale_inflight_locked(last_timestamp_lidar)` and then
`enqueue_latest_lidar_locked(ptr, last_timestamp_lidar)` in both LiDAR callbacks.
Keep preprocessing outside the mutex only if copied message ownership remains valid; otherwise
retain existing locking to avoid broad callback redesign.

- [ ] **Step 4: Add structural tests for every paired mutation**

```python
def test_every_bounded_removal_mutates_both_lidar_queues(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "bounded", check=True)
    text = source.read_text(encoding="utf-8")
    assert text.count("lidar_buffer.pop_back();") == text.count("time_buffer.pop_back();")
    assert text.count("lidar_buffer.pop_front();") == text.count("time_buffer.pop_front();")
    assert text.count("lidar_buffer.clear();") == text.count("time_buffer.clear();")


def test_bounded_patch_is_idempotent(tmp_path):
    source = write_fast_lio_fixture(tmp_path)
    run_patch(source, "bounded", check=True)
    once = source.read_bytes()
    run_patch(source, "bounded", check=True)
    assert source.read_bytes() == once
```

- [ ] **Step 5: Run all FAST-LIO patch tests and verify GREEN**

Run: `python -m pytest migration/test_fast_lio_low_latency_patch.py -v`

Expected: all tests PASS; final transformed source contains both diagnostic and bounded markers.

- [ ] **Step 6: Commit the bounded policy**

```bash
git add migration/patch_fast_lio_low_latency.py migration/test_fast_lio_low_latency_patch.py
git commit -m "fix: bound FAST-LIO lidar backlog"
```

### Task 3: Add a Reproducible ROS Latency Probe and Safety Wrapper

**Files:**
- Create: `migration/measure_localization_latency.py`
- Create: `migration/verify_fast_lio_latency.sh`
- Create: `migration/test_latency_verifier.py`

**Interfaces:**
- Consumes: ROS topics `/Odometry`, `/scan`, optional `/amcl_pose`, `/go1/control_state`, and FAST-LIO log path.
- Produces: JSON schema with `duration_sec`, `topics.<name>.count/rate_hz/age_p95_sec/age_max_sec/reversals`, `stationary`, optional `amcl`, and overall `passed`; process exit 0 only when every requested gate passes.

- [ ] **Step 1: Write failing pure-statistics tests**

```python
def test_topic_stats_compute_rate_percentile_max_and_reversals():
    tracker = TopicTracker(start_monotonic=10.0)
    tracker.add(received_monotonic=10.1, header_sec=100.00, ros_now_sec=100.02)
    tracker.add(received_monotonic=10.2, header_sec=100.10, ros_now_sec=100.13)
    tracker.add(received_monotonic=10.3, header_sec=100.05, ros_now_sec=100.14)
    result = tracker.summary(end_monotonic=10.4)
    assert result["count"] == 3
    assert result["rate_hz"] == pytest.approx(10.0)
    assert result["age_max_sec"] == pytest.approx(0.09)
    assert result["reversals"] == 1


def test_amcl_window_enforces_covariance_and_spread():
    window = AmclWindow(limit=10)
    for index in range(10):
        window.add(x=1.0 + index * 0.005, y=2.0, yaw=0.01, cov_x=0.02, cov_y=0.02, cov_yaw=0.01)
    assert window.summary()["passed"] is True
```

- [ ] **Step 2: Run the probe tests and verify RED**

Run: `python -m pytest migration/test_latency_verifier.py -v`

Expected: collection FAIL because `measure_localization_latency.py` does not exist.

- [ ] **Step 3: Implement ROS-independent trackers, then conditional ROS imports**

Implement:

```python
def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    rank = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[rank]


class TopicTracker:
    def add(self, *, received_monotonic: float, header_sec: float, ros_now_sec: float) -> None:
        age = ros_now_sec - header_sec
        if age < 0.0:
            self.future_stamps += 1
        if self.last_header_sec is not None and header_sec <= self.last_header_sec:
            self.reversals += 1
        self.ages.append(age)
        self.received.append(received_monotonic)
        self.last_header_sec = header_sec
```

Import `rclpy`, `LaserScan`, `Odometry`, and `PoseWithCovarianceStamped` only after these pure
classes so local pytest remains usable without ROS. Use `qos_profile_sensor_data` for scan and
odometry subscriptions. The CLI must accept:

```text
--duration SEC --warmup SEC --mode fast-lio|amcl --output PATH
--age-p95 0.10 --age-max 0.30 --rate-min 9.0 --rate-max 11.0
```

For `amcl` mode also enforce stationary speed, covariance, and ten-sample spread from Global
Constraints. Write JSON atomically via `PATH.tmp` then `os.replace()`.

- [ ] **Step 4: Write failing wrapper safety/threshold tests**

```python
def test_shell_wrapper_enforces_disarm_uniqueness_and_diagnostic_thresholds():
    text = (ROOT / "migration" / "verify_fast_lio_latency.sh").read_text()
    assert "ros2 param get /go1_driver arm" in text
    assert "Boolean value is: False" in text
    assert "ros2 topic info /livox/lidar --verbose" in text
    assert "Publisher count: 1" in text
    assert "pgrep -fc" in text
    assert "FAST_LIO_REALTIME_DIAGNOSTICS" in text
    assert "queue_depth" in text and "front_age" in text


def test_observe_only_is_restricted_to_fast_lio_baseline():
    text = (ROOT / "migration" / "verify_fast_lio_latency.sh").read_text()
    assert '"$mode" != "fast-lio"' in text
    assert "--observe-only is valid only with fast-lio" in text
    assert 'probe_args+=("--observe-only")' in text
```

- [ ] **Step 5: Implement the Bash verifier**

`verify_fast_lio_latency.sh` must:

1. set `ROS_DOMAIN_ID=100`, source ROS and workspace before `set -u`;
2. require `mode` in `fast-lio|amcl`, a duration argument, and accept optional `--observe-only` only for `fast-lio` mode;
3. fail unless `/go1_driver arm` is false and `/go1/control_state` contains `DRY-RUN`;
4. fail unless `/livox/lidar` has exactly one publisher and FAST-LIO has exactly one process;
5. save process IDs and verify they are unchanged after the probe;
6. run `measure_localization_latency.py` with the exact thresholds;
7. extract `[fast_lio_realtime]` lines written during the interval and fail if any parsed queue depth > 2 or front age > 0.30, except that `--observe-only` records these violations without failing so the diagnostic-only baseline can test the hypothesis;
8. scan navigation logs in AMCL mode and fail on `Message Filter dropping message`, `extrapolation`, or `Timed out waiting for transform`;
9. place timestamped JSON and text evidence under `/mnt/t500/go1_runtime/latency/`.

- [ ] **Step 6: Run verifier tests and existing script tests**

Run: `python -m pytest migration/test_latency_verifier.py migration/test_existing_map_scripts.py -v`

Expected: all tests PASS.

- [ ] **Step 7: Commit the probe and wrapper**

```bash
git add migration/measure_localization_latency.py migration/verify_fast_lio_latency.sh migration/test_latency_verifier.py
git commit -m "test: add sustained localization latency verifier"
```

### Task 4: Wire Patch Modes Into the Build and Document the Runbook

**Files:**
- Modify: `migration/build_livox_fastlio.sh:4-58`
- Modify: `migration/README.md:154-310`
- Modify: `migration/test_end_to_end_workflow.py`

**Interfaces:**
- Consumes: `FAST_LIO_LOW_LATENCY_MODE=diagnostic|bounded`.
- Produces: repeatable diagnostic-only and final build commands; default remains safe final `bounded` mode.

- [ ] **Step 1: Write failing build-script tests**

```python
def test_build_script_selects_validated_fast_lio_patch_mode():
    text = (ROOT / "migration" / "build_livox_fastlio.sh").read_text()
    assert 'fast_lio_low_latency_mode="${FAST_LIO_LOW_LATENCY_MODE:-bounded}"' in text
    assert 'diagnostic|bounded' in text
    assert '--mode "$fast_lio_low_latency_mode"' in text
```

- [ ] **Step 2: Run the focused workflow test and verify RED**

Run: `python -m pytest migration/test_end_to_end_workflow.py -v -k low_latency`

Expected: FAIL because the build script has no selectable mode.

- [ ] **Step 3: Implement mode validation and pass it to the patcher**

Add near the top of `build_livox_fastlio.sh`:

```bash
fast_lio_low_latency_mode="${FAST_LIO_LOW_LATENCY_MODE:-bounded}"
case "$fast_lio_low_latency_mode" in
  diagnostic|bounded) ;;
  *)
    printf 'ERROR: FAST_LIO_LOW_LATENCY_MODE must be diagnostic or bounded.\n' >&2
    exit 2
    ;;
esac
```

Invoke:

```bash
python3 "$script_dir/patch_fast_lio_low_latency.py" \
  --mode "$fast_lio_low_latency_mode" \
  "$fast_lio_dir/src/laserMapping.cpp"
```

- [ ] **Step 4: Document exact diagnostic, bounded, soak, and AMCL commands**

Add to `migration/README.md`:

```bash
FAST_LIO_LOW_LATENCY_MODE=diagnostic ./migration/build_livox_fastlio.sh
./migration/verify_fast_lio_latency.sh fast-lio 120

FAST_LIO_LOW_LATENCY_MODE=bounded ./migration/build_livox_fastlio.sh
./migration/verify_fast_lio_latency.sh fast-lio 600
./migration/verify_fast_lio_latency.sh amcl 180
```

State that the diagnostic-only build is evidence gathering, must not be considered the fix, and
must be replaced with the default bounded build before final validation.

- [ ] **Step 5: Run workflow tests and verify GREEN**

Run: `python -m pytest migration/test_end_to_end_workflow.py migration/test_existing_map_scripts.py migration/test_latency_verifier.py -v`

Expected: all tests PASS.

- [ ] **Step 6: Commit build integration and documentation**

```bash
git add migration/build_livox_fastlio.sh migration/README.md migration/test_end_to_end_workflow.py
git commit -m "docs: add FAST-LIO latency validation workflow"
```

### Task 5: Run the Full Local Regression Suite

**Files:**
- Verify only; no expected source changes.

**Interfaces:**
- Consumes: Tasks 1-4 commits.
- Produces: clean working tree and complete local pytest evidence.

- [ ] **Step 1: Run formatting/static checks**

Run:

```bash
git diff --check
python -m compileall -q migration packages/omx_navigation/omx_navigation
```

Expected: both exit 0.

- [ ] **Step 2: Run every repository test**

Run: `python -m pytest -q`

Expected: all available tests PASS; ROS-dependent tests may retain their existing skip behavior when ROS is unavailable locally.

- [ ] **Step 3: Inspect scope and commit any test-only correction separately**

Run:

```bash
git status --short
git diff --stat origin/main...HEAD
git log --oneline origin/main..HEAD
```

Expected: no uncommitted files and only FAST-LIO latency/QoS-related commits plus the approved spec/plan.

### Task 6: Capture Diagnostic-Only Jetson Baseline

**Files:**
- Deploy from: `migration/patch_fast_lio_low_latency.py`, `migration/build_livox_fastlio.sh`, `migration/measure_localization_latency.py`, `migration/verify_fast_lio_latency.sh`
- Runtime evidence: `/mnt/t500/go1_runtime/latency/<timestamp>-diagnostic/`

**Interfaces:**
- Consumes: reachable Jetson `unicon@192.168.0.138`, source workspace `/mnt/t500/go1_ros2_ws`, MID-360 on `192.168.1.145`.
- Produces: a two-minute diagnostic-only JSON/log baseline proving whether internal queue depth/age grows.

- [ ] **Step 1: Verify safety and capture exact pre-deploy state**

Run remotely:

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 param get /go1_driver arm
timeout 5 ros2 topic echo /go1/control_state --once
pgrep -af 'livox|laser_mapping'
git -C /mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2 rev-parse HEAD
```

Expected: `arm=false`, `DRY-RUN`, one Livox process, one FAST-LIO process, pinned source commit.

- [ ] **Step 2: Back up current source and binary metadata**

Run remotely:

```bash
stamp=$(date +%Y%m%d_%H%M%S)
backup=/mnt/t500/deploy_backups/${stamp}_fast_lio_realtime
mkdir -p "$backup"
cp /mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp "$backup/"
find /mnt/t500/go1_ros2_ws/install/fast_lio/lib/fast_lio -maxdepth 1 -type f \
  -exec stat {} \; > "$backup/binary.stat"
```

Expected: explicit backup directory and source copy exist.

- [ ] **Step 3: Deploy scripts and build diagnostic-only mode**

Copy the four deployment files to `/mnt/t500/go1_ros2_project/migration/`, then run remotely:

```bash
cd /mnt/t500/go1_ros2_project
FAST_LIO_LOW_LATENCY_MODE=diagnostic ./migration/build_livox_fastlio.sh
```

Expected: colcon exits 0 and source contains `FAST_LIO_REALTIME_DIAGNOSTICS` but not `FAST_LIO_BOUNDED_BUFFER`.

- [ ] **Step 4: Restart only FAST-LIO and verify unique processes**

Use the existing safe restart helper, then run:

```bash
pgrep -af 'livox|laser_mapping'
ros2 topic info /livox/lidar --verbose
ros2 topic info /Odometry --verbose
```

Expected: one source publisher and one FAST-LIO publisher; no Go1 motion activation.

- [ ] **Step 5: Run a two-minute diagnostic baseline**

Run remotely:

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_fast_lio_latency.sh fast-lio 120 --observe-only
```

Expected: the command records rather than rejects queue depth/age; evidence shows whether depth or age grows. If depth never exceeds 2 and age never approaches 0.30, stop and return to root-cause analysis instead of deploying the bounded hypothesis.

- [ ] **Step 6: Record the hypothesis decision**

Write `/mnt/t500/go1_runtime/latency/<timestamp>-diagnostic/decision.txt` containing:

```text
hypothesis=confirmed|rejected
max_queue_depth=<integer>
max_front_age_sec=<float>
reason=<one sentence tied to observed values>
```

Proceed to Task 7 only if the internal queue hypothesis is confirmed.

### Task 7: Deploy the Bounded Build and Run the Ten-Minute FAST-LIO Soak

**Files:**
- Runtime evidence: `/mnt/t500/go1_runtime/latency/<timestamp>-bounded/`

**Interfaces:**
- Consumes: confirmed Task 6 hypothesis and default bounded build.
- Produces: successful ten-minute FAST-LIO soak evidence or a precise next bottleneck classification.

- [ ] **Step 1: Build final bounded mode**

Run remotely:

```bash
cd /mnt/t500/go1_ros2_project
FAST_LIO_LOW_LATENCY_MODE=bounded ./migration/build_livox_fastlio.sh
grep -n 'FAST_LIO_BOUNDED_BUFFER' /mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp
```

Expected: build exits 0 and marker exists exactly once.

- [ ] **Step 2: Restart FAST-LIO and verify the installed binary/process timestamp**

Run the safe restart helper, then compare:

```bash
find /mnt/t500/go1_ros2_ws/install/fast_lio/lib/fast_lio -maxdepth 1 -type f \
  -exec stat {} \;
pgrep -af '/fast_lio/.*mapping|laserMapping|fastlio_mapping'
```

Expected: running process started after the rebuilt binary modification time.

- [ ] **Step 3: Run the full ten-minute verifier**

Run remotely:

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_fast_lio_latency.sh fast-lio 600
```

Expected: after the scripted 60-second warmup, every FAST-LIO soak constraint in Global Constraints passes.

- [ ] **Step 4: Classify any failure before another change**

Use the generated JSON and diagnostic lines:

- queue depth > 2: bounded policy integration defect;
- queue <= 2 but processing p95 > 100 ms: CPU/point-processing bottleneck;
- raw LiDAR age high before FAST-LIO: Livox/host network boundary regression;
- odometry age high with queue/processing normal: publishing/executor boundary;
- timestamp reversal > 0: sensor/host time source regression.

Do not adjust voxel size, LiDAR rate, or AMCL parameters in the same iteration. Return to the
corresponding single hypothesis and add a failing test or boundary measurement first.

### Task 8: Run AMCL Time-Alignment Verification

**Files:**
- Runtime evidence: `/mnt/t500/go1_runtime/latency/<timestamp>-amcl/`

**Interfaces:**
- Consumes: passing Task 7 FAST-LIO soak, existing mapped location, one manual initial-pose estimate.
- Produces: three-minute AMCL alignment evidence; does not save an initial pose.

- [ ] **Step 1: Start existing-map localization with motion disabled**

Use the existing safe navigation helper and verify:

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
ros2 param get /go1_driver arm
timeout 5 ros2 topic echo /go1/control_state --once
```

Expected: `arm=false` and `DRY-RUN` before any pose operation.

- [ ] **Step 2: Verify map overlap and set one manual pose**

Use RViz `2D Pose Estimate` at the robot's actual mapped location and heading. Confirm `/amcl_pose`
starts and scan points visually align with occupied walls. Do not write
`/mnt/t500/go1_runtime/start_pose.yaml`.

- [ ] **Step 3: Run the three-minute AMCL verifier**

Run remotely:

```bash
cd /mnt/t500/go1_ros2_project
./migration/verify_fast_lio_latency.sh amcl 180
```

Expected: every AMCL constraint in Global Constraints passes, including zero new transform and
costmap drop messages.

- [ ] **Step 4: Cross-check localization supervisor and stationary state**

Run remotely:

```bash
timeout 5 ros2 topic echo /localization/ready --once
timeout 5 ros2 topic echo /go1/control_state --once
```

Expected: localization readiness true, `DRY-RUN`, and zero commanded velocity.

- [ ] **Step 5: Preserve evidence and leave the system disarmed**

Record the evidence directory, process IDs, git commit, build timestamp, and thresholds in
`summary.txt`. Stop navigation if it is no longer needed; do not stop LiDAR/FAST-LIO unless the
user asks. Re-read `/go1/control_state` and confirm `arm=false` at handoff.

### Task 9: Completion Audit

**Files:**
- Verify only.

**Interfaces:**
- Consumes: local git/test evidence and Jetson diagnostic, bounded, and AMCL evidence directories.
- Produces: requirement-by-requirement pass/fail report.

- [ ] **Step 1: Verify repository state and tests from current HEAD**

Run:

```bash
git status --short --branch
git log --oneline origin/main..HEAD
python -m pytest -q
```

Expected: clean worktree and all tests pass.

- [ ] **Step 2: Audit every runtime requirement against raw evidence**

For each Global Constraint, cite the JSON field or log line that proves it. Treat missing fields,
shorter durations, restarted processes, or indirect observations as failure.

- [ ] **Step 3: Confirm non-goals remain untouched**

Run remotely:

```bash
test ! -e /mnt/t500/go1_runtime/start_pose.yaml
ros2 param get /go1_driver arm
timeout 5 ros2 topic echo /go1/control_state --once
```

Expected: no commissioned pose was written, `arm=false`, and DRY-RUN remains active.

- [ ] **Step 4: Mark complete only when every item is proven**

If FAST-LIO passes but AMCL cannot be tested because the location is outside the map, report that
specific missing evidence and keep the goal active. If all items pass, report the exact evidence
paths and commit hashes; only then mark the goal complete.

### Task 10: Publish a GitHub-Ready Investigation Handoff

**Files:**
- Create: `docs/reports/2026-08-13-fast-lio-amcl-time-alignment.md`

**Interfaces:**
- Consumes: local commits/tests and every Jetson evidence directory produced by Tasks 6-9.
- Produces: a self-contained Markdown report that another engineer can use after cloning the GitHub branch, even if the runtime issue remains unresolved.

- [ ] **Step 1: Record reproducible context before runtime changes**

Include the repository branch and commit graph, pinned FAST-LIO/Livox revisions, Jetson and ROS
paths, MID-360 address, `ROS_DOMAIN_ID`, process launch helpers, safety state, and the exact local
test command including `PYTHONPATH`.

- [ ] **Step 2: Maintain an evidence table during each experiment**

For every diagnostic or fix attempt record: timestamp, hypothesis, single variable changed,
commands, process IDs, input/output topic statistics, FAST-LIO queue statistics, outcome, raw
evidence path, and commit containing the change. Never replace failed attempts; append them so the
report preserves the causal history.

- [ ] **Step 3: Document reproduction, deployment, rollback, and continuation commands**

Provide copyable commands for cloning/checking out the branch, setting `PYTHONPATH`, running unit
tests, deploying migration files, restoring the newest backup, building diagnostic/bounded modes,
restarting only FAST-LIO, running both verifiers, and collecting logs. Use placeholders only for
the remote Git URL and future evidence timestamps, and label those values explicitly.

- [ ] **Step 4: End with a current-state decision table**

List each required gate as `PASS`, `FAIL`, or `NOT RUN`, cite direct evidence, state the next single
hypothesis to test, and list actions that must not be taken (`arm=true`, initial-pose save, Nav2 goal)
until all gates pass.

- [ ] **Step 5: Commit the report independently**

```bash
git add docs/reports/2026-08-13-fast-lio-amcl-time-alignment.md
git commit -m "docs: report FAST-LIO time alignment investigation"
```
