# Fixed-Start AMCL Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 동일한 타일에서 출발하는 Go1에 AMCL 초기 자세 자동 공급, 정지 상태 localization 검증, 독립 motion gate 및 안전한 commissioning 절차를 추가한다.

**Architecture:** `localization_supervisor`가 프리셋을 AMCL에 공급하고 covariance, 프리셋 편차, TF/토픽 신선도 및 지도-스캔 정합을 검사한다. 독립 ROS 2 노드인 `motion_gate`만 `/cmd_vel_nav`를 `/cmd_vel_safe`로 전달하며, `go1_driver`는 안전 출력과 기존 watchdog만 처리한다. 초기 좌표가 없을 때 `arm:=false`는 `UNCOMMISSIONED`로 실행하고 `arm:=true`는 launch에서 거부한다.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, Nav2 AMCL, `tf2_ros`, `nav_msgs/OccupancyGrid`, `sensor_msgs/LaserScan`, `geometry_msgs`, `std_srvs`, PyYAML, pytest/ament

## Global Constraints

- Localization 알고리즘은 AMCL만 사용한다.
- Localization 중 Go1은 움직이지 않는다.
- 초기 seed variance는 x/y `0.0225 m²`, yaw `0.0305 rad²`로 시작한다.
- READY 기준은 covariance x/y `<= 0.04 m²`, yaw `<= 0.0305 rad²`이다.
- AMCL 결과와 프리셋 차이는 평면 거리 `<= 0.25 m`, yaw 최단 각도 차이 `<= 15 deg`여야 한다.
- Scan 정합은 유효 beam 100개 이상, 중앙값 `<= 0.15 m`, 80 percentile `<= 0.30 m`, 최소 10 scan 및 2초 연속 통과가 필요하다.
- `/scan`, `/Odometry`, pose 및 TF는 `0.30 s`보다 오래되면 유효하지 않다.
- `motion_gate`는 별도 ROS 2 노드이며 시작 시 E-stop latch 상태다.
- 프리셋 없이 `arm:=false`는 허용하고 `arm:=true`는 거부한다.
- 실제 현장 좌표나 commissioning 표본은 저장소에 커밋하지 않는다.
- 현재 작업공간의 PCD 관련 미커밋 파일은 수정하거나 포함하지 않는다.

---

## File Structure

### 새 파일

- `packages/omx_navigation/omx_navigation/start_pose.py`: 프리셋 파싱, 검증, 각도/편차 계산
- `packages/omx_navigation/omx_navigation/scan_match.py`: occupancy distance field와 scan 정합 통계
- `packages/omx_navigation/omx_navigation/localization_state.py`: ROS 비의존 supervisor 상태 머신
- `packages/omx_navigation/omx_navigation/localization_supervisor.py`: AMCL/TF/센서 ROS adapter
- `packages/omx_navigation/omx_navigation/motion_gate_core.py`: ROS 비의존 명령 게이트
- `packages/omx_navigation/omx_navigation/motion_gate.py`: motion gate ROS adapter와 E-stop services
- `packages/omx_navigation/omx_navigation/commission_start_pose.py`: 10개 자세 표본으로 프리셋 생성
- `packages/omx_navigation/config/localization_supervisor.yaml`: 판정 threshold
- `packages/omx_navigation/test/fixtures/start_pose_valid.yaml`: 테스트 전용 자세
- `packages/omx_navigation/test/test_start_pose.py`
- `packages/omx_navigation/test/test_scan_match.py`
- `packages/omx_navigation/test/test_localization_state.py`
- `packages/omx_navigation/test/test_motion_gate.py`
- `packages/omx_navigation/test/test_commission_start_pose.py`
- `packages/omx_navigation/test/test_goal_gate.py`
- `migration/verify_fixed_start_localization.sh`: Jetson runtime contract 검증

### 수정 파일

- `packages/omx_navigation/setup.py`: console scripts와 config 설치
- `packages/omx_navigation/package.xml`: ROS 메시지, TF, diagnostic, service 의존성
- `packages/omx_navigation/config/nav2_existing_map_params.yaml`: 정지 초기화를 위한 AMCL update threshold
- `packages/omx_navigation/omx_navigation/rviz_goal_bridge.py`: READY 이전 goal 거부
- `packages/omx_navigation/launch/go1_existing_map.launch.py`: 노드와 안전 토픽 연결
- `packages/omx_navigation/test/test_existing_map_launch.py`: launch contract
- `packages/omx_navigation/test/test_existing_map_params.py`: AMCL startup 설정
- `packages/go1_driver/config/go1_driver.yaml`: 기본 안전 입력 토픽
- `packages/go1_driver/go1_driver/node.py`: 안전 입력 명칭과 상태 사유 명확화
- `packages/go1_driver/test/test_command_filter.py`: gate 종료 시 watchdog 회귀 시험
- `migration/stage_local_ros2_packages.sh`: 새 Python/config/script 배포
- `migration/verify_existing_map_navigation.sh`: ready/gate/driver 토픽 검증
- `packages/omx_navigation/README.md`: commissioning과 운영 절차
- `README.md`: 최종 실행 명령과 실패 복구

---

### Task 1: Start Pose Contract and Commissioning Tool

**Files:**
- Create: `packages/omx_navigation/omx_navigation/start_pose.py`
- Create: `packages/omx_navigation/omx_navigation/commission_start_pose.py`
- Create: `packages/omx_navigation/test/fixtures/start_pose_valid.yaml`
- Create: `packages/omx_navigation/test/test_start_pose.py`
- Create: `packages/omx_navigation/test/test_commission_start_pose.py`
- Modify: `packages/omx_navigation/setup.py`

**Interfaces:**
- Produces: `StartPose(frame_id, x, y, yaw, variance_x, variance_y, variance_yaw)`
- Produces: `load_start_pose(path: str) -> StartPose`
- Produces: `pose_delta(seed: StartPose, x: float, y: float, yaw: float) -> tuple[float, float]`
- Produces: `summarize_samples(samples: Sequence[PoseSample]) -> CommissioningSummary`

- [ ] **Step 1: Write failing parser and math tests**

```python
def test_loads_valid_start_pose():
    pose = load_start_pose(str(FIXTURE))
    assert pose.frame_id == "map"
    assert pose.variance_x == pytest.approx(0.0225)

def test_pose_delta_wraps_yaw():
    seed = StartPose("map", 1.0, 2.0, math.radians(179), 0.0225, 0.0225, 0.0305)
    distance, yaw_error = pose_delta(seed, 1.1, 2.0, math.radians(-179))
    assert distance == pytest.approx(0.1)
    assert yaw_error == pytest.approx(math.radians(2))

@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_rejects_non_finite_pose_values(tmp_path, bad):
    with pytest.raises(ValueError, match="finite"):
        validate_start_pose(StartPose("map", bad, 0.0, 0.0, 0.0225, 0.0225, 0.0305))
```

- [ ] **Step 2: Run tests and confirm failure**

Run: `python3 -m pytest packages/omx_navigation/test/test_start_pose.py -v`
Expected: FAIL because `omx_navigation.start_pose` does not exist.

- [ ] **Step 3: Implement immutable data types and strict YAML parsing**

Implement `StartPose`, `normalize_angle`, `pose_delta`, `validate_start_pose`, and `load_start_pose`. Require `frame_id == "map"`, finite pose values, and strictly positive finite variances. Do not silently supply missing pose coordinates.

- [ ] **Step 4: Write failing commissioning tests**

```python
def test_commissioning_uses_circular_yaw_mean():
    samples = [PoseSample(1.0, 2.0, math.radians(179)), PoseSample(1.0, 2.0, math.radians(-179))] * 5
    result = summarize_samples(samples)
    assert abs(abs(result.mean_yaw) - math.pi) < math.radians(1)

def test_commissioning_requires_ten_samples():
    with pytest.raises(ValueError, match="exactly 10"):
        summarize_samples([PoseSample(0.0, 0.0, 0.0)] * 9)

def test_commissioning_rejects_excessive_spread():
    samples = [PoseSample(0.0, 0.0, 0.0)] * 9 + [PoseSample(0.20, 0.0, 0.0)]
    with pytest.raises(ValueError, match="0.15 m"):
        summarize_samples(samples)
```

- [ ] **Step 5: Implement commissioning CLI**

Accept an input YAML containing exactly 10 `{x, y, yaw}` samples and an explicit `--output` path. Refuse to overwrite an existing output. Write the fixed initial variances and print mean/max deviations. Register:

```python
"commission_start_pose = omx_navigation.commission_start_pose:main"
```

- [ ] **Step 6: Run the focused tests**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_start_pose.py packages/omx_navigation/test/test_commission_start_pose.py -v`
Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add packages/omx_navigation/omx_navigation/start_pose.py packages/omx_navigation/omx_navigation/commission_start_pose.py packages/omx_navigation/test/fixtures/start_pose_valid.yaml packages/omx_navigation/test/test_start_pose.py packages/omx_navigation/test/test_commission_start_pose.py packages/omx_navigation/setup.py
git commit -m "feat: add AMCL start pose commissioning"
```

---

### Task 2: Deterministic Map-to-Scan Scoring

**Files:**
- Create: `packages/omx_navigation/omx_navigation/scan_match.py`
- Create: `packages/omx_navigation/test/test_scan_match.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Produces: `DistanceField.from_occupancy(width, height, resolution, origin_x, origin_y, cells, occupied_threshold=65)`
- Produces: `score_scan(field, ranges, angle_min, angle_increment, range_min, range_max, map_x, map_y, map_yaw) -> ScanScore`
- `ScanScore` fields: `valid_beams`, `median_distance`, `percentile_80`

- [ ] **Step 1: Write synthetic-map failing tests**

```python
def test_aligned_scan_scores_near_zero():
    field = vertical_wall_field(x=2.0)
    score = score_scan(field, [2.0] * 181, -math.pi / 4, math.pi / 360, 0.2, 10.0, 0.0, 0.0, 0.0)
    assert score.valid_beams >= 100
    assert score.median_distance <= 0.05

def test_shifted_pose_exceeds_threshold():
    field = vertical_wall_field(x=2.0)
    score = score_scan(field, [2.0] * 181, -math.pi / 4, math.pi / 360, 0.2, 10.0, 0.4, 0.0, 0.0)
    assert score.median_distance > 0.15
```

- [ ] **Step 2: Verify failure**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_scan_match.py -v`
Expected: FAIL because `scan_match` is missing.

- [ ] **Step 3: Implement distance field and scoring**

Use a multi-source grid traversal from occupied cells with 8-neighbor metric costs, then bilinear or nearest-cell lookup for endpoints. Ignore non-finite/out-of-range beams and endpoints outside the map. Calculate median and 80th percentile without NumPy so the ROS package adds no runtime Python dependency.

- [ ] **Step 4: Add edge-case tests**

Test empty maps, fewer than 100 valid beams, endpoints outside the map, rotated poses, negative map origin, and invalid map resolution. Require explicit `ValueError` for malformed map metadata.

- [ ] **Step 5: Run tests and commit**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_scan_match.py -v`
Expected: PASS.

```bash
git add packages/omx_navigation/omx_navigation/scan_match.py packages/omx_navigation/test/test_scan_match.py packages/omx_navigation/package.xml
git commit -m "feat: score AMCL scan alignment against map"
```

---

### Task 3: Localization Supervisor State Machine and ROS Node

**Files:**
- Create: `packages/omx_navigation/omx_navigation/localization_state.py`
- Create: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Create: `packages/omx_navigation/config/localization_supervisor.yaml`
- Create: `packages/omx_navigation/test/test_localization_state.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`
- Modify: `packages/omx_navigation/config/nav2_existing_map_params.yaml`

**Interfaces:**
- Produces states: `BOOT`, `UNCOMMISSIONED`, `WAIT_INPUTS`, `SEEDING`, `VALIDATING`, `READY`, `FAULT`
- Produces: `LocalizationStateMachine.tick(now: float, evidence: LocalizationEvidence) -> Decision`
- ROS outputs: `/initialpose`, `/localization/ready`, `/localization/state`, `/diagnostics`
- ROS services: `/localization/reseed`, `/localization/reset_fault`

- [ ] **Step 1: Write failing state-machine tests**

```python
def test_missing_preset_enters_uncommissioned():
    machine = LocalizationStateMachine(start_pose=None)
    assert machine.tick(0.0, no_inputs()).state == State.UNCOMMISSIONED
    assert machine.tick(0.0, no_inputs()).ready is False

def test_ready_requires_all_checks_for_two_seconds():
    machine = seeded_machine()
    assert machine.tick(1.0, passing_evidence()).ready is False
    assert machine.tick(3.01, passing_evidence()).ready is True

def test_preset_position_or_yaw_error_blocks_ready():
    assert seeded_machine().tick(3.0, passing_evidence(position_error=0.251)).ready is False
    assert seeded_machine().tick(3.0, passing_evidence(yaw_error=math.radians(15.1))).ready is False

def test_ready_faults_when_scan_is_stale():
    machine = ready_machine()
    decision = machine.tick(10.0, passing_evidence(scan_age=0.301))
    assert decision.state == State.FAULT
    assert decision.ready is False

def test_ready_faults_on_fast_lio_odometry_discontinuity():
    machine = ready_machine()
    decision = machine.tick(10.0, passing_evidence(odom_discontinuity=True))
    assert decision.state == State.FAULT
    assert decision.reason == "fast_lio_odometry_discontinuity"
```

- [ ] **Step 2: Verify failure and implement the pure state machine**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_localization_state.py -v`
Expected: FAIL before implementation, then PASS after implementing immutable `LocalizationEvidence` and `Decision` types with explicit failure reasons.

- [ ] **Step 3: Add ROS node adapter**

The node must:

- load `start_pose_file`; empty means `UNCOMMISSIONED`
- publish `/initialpose` once inputs are ready and again only on explicit reseed
- subscribe to `/map`, `/scan`, `/Odometry`, `/amcl_pose`
- look up `camera_init -> body` and `map -> camera_init`
- convert OccupancyGrid and LaserScan using Task 2
- publish `/localization/ready` at 10 Hz even when false
- publish machine-readable state text such as `READY reason=all_checks_passed`
- publish DiagnosticArray with level OK/WARN/ERROR
- latch `FAULT` until `/localization/reset_fault`
- flag FAST-LIO restart when `/Odometry` frame IDs change, its timestamp moves backward, or consecutive samples within 0.50 s jump by more than 0.50 m or 20 deg

- [ ] **Step 4: Configure thresholds and AMCL stationary updates**

Create `localization_supervisor.yaml` with the exact Global Constraints. In `nav2_existing_map_params.yaml`, set AMCL `update_min_d: 0.0` and `update_min_a: 0.0`; add tests asserting both.

- [ ] **Step 5: Register node and dependencies**

Add console script:

```python
"localization_supervisor = omx_navigation.localization_supervisor:main"
```

Add `diagnostic_msgs`, `nav_msgs`, `std_msgs`, `std_srvs`, and `tf2_ros`. Calculate quaternion yaw directly with `math.atan2`; do not add `tf_transformations`.

- [ ] **Step 6: Run focused and package tests**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_localization_state.py packages/omx_navigation/test/test_scan_match.py packages/omx_navigation/test/test_existing_map_params.py -v`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add packages/omx_navigation/omx_navigation/localization_state.py packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/config/localization_supervisor.yaml packages/omx_navigation/test/test_localization_state.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml packages/omx_navigation/config/nav2_existing_map_params.yaml packages/omx_navigation/test/test_existing_map_params.py
git commit -m "feat: add guarded AMCL localization supervisor"
```

---

### Task 4: Standalone Motion Gate

**Files:**
- Create: `packages/omx_navigation/omx_navigation/motion_gate_core.py`
- Create: `packages/omx_navigation/omx_navigation/motion_gate.py`
- Create: `packages/omx_navigation/test/test_motion_gate.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Consumes: `/cmd_vel_nav`, `/localization/ready`
- Produces: `/cmd_vel_safe`, `/motion_gate/state`
- Services: `/motion_gate/engage_estop`, `/motion_gate/release_estop` using `std_srvs/srv/Trigger`
- Core API: `MotionGate.evaluate(now, command, command_time, ready, ready_time, estop_latched) -> GateDecision`

- [ ] **Step 1: Write fail-closed tests**

```python
def test_gate_starts_estopped():
    gate = MotionGate()
    assert gate.estop_latched is True

def test_gate_passes_only_fresh_ready_command_after_release():
    gate = MotionGate(); gate.release_estop()
    decision = gate.evaluate(1.0, Twist2D(0.1, 0.0, 0.1), 0.9, True, 0.9)
    assert decision.output == Twist2D(0.1, 0.0, 0.1)

@pytest.mark.parametrize("ready_age,cmd_age", [(0.301, 0.1), (0.1, 0.251)])
def test_stale_inputs_output_zero(ready_age, cmd_age):
    gate = released_gate()
    assert gate.evaluate(1.0, active_cmd(), 1.0-cmd_age, True, 1.0-ready_age).output == Twist2D.zero()

def test_localization_false_relatches_estop():
    gate = released_gate()
    gate.evaluate(1.0, active_cmd(), 0.9, False, 0.9)
    assert gate.estop_latched is True
```

- [ ] **Step 2: Verify failure and implement core**

Enforce finite values, `hypot(vx, vy) <= 0.20`, `abs(yaw) <= 0.40`, ready timeout 0.30 s, command timeout 0.25 s, and zero output on every rejection. Releasing E-stop succeeds only while a fresh true ready heartbeat exists.

- [ ] **Step 3: Implement ROS adapter**

Publish `/cmd_vel_safe` at 20 Hz. Use QoS depth 1 for command topics. Publish structured state reason whenever it changes. On shutdown, publish zero Twist multiple times before destroying the node.

- [ ] **Step 4: Run tests and commit**

Run: `PYTHONPATH=packages/omx_navigation python3 -m pytest packages/omx_navigation/test/test_motion_gate.py -v`
Expected: PASS.

```bash
git add packages/omx_navigation/omx_navigation/motion_gate_core.py packages/omx_navigation/omx_navigation/motion_gate.py packages/omx_navigation/test/test_motion_gate.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: add standalone localization motion gate"
```

---

### Task 5: Goal Gate and Driver Safety Boundary

**Files:**
- Modify: `packages/omx_navigation/omx_navigation/rviz_goal_bridge.py`
- Create: `packages/omx_navigation/test/test_goal_gate.py`
- Modify: `packages/go1_driver/config/go1_driver.yaml`
- Modify: `packages/go1_driver/go1_driver/node.py`
- Modify: `packages/go1_driver/test/test_command_filter.py`

**Interfaces:**
- Goal bridge consumes `/localization/ready` heartbeat and rejects goals after 0.30 s timeout.
- Driver consumes only `/cmd_vel_safe` and remains independent of AMCL/Nav2.

- [ ] **Step 1: Extract and test pure goal eligibility**

Add `goal_is_allowed(now: float, ready: bool, ready_time: Optional[float], timeout: float = 0.30) -> bool` and tests for false, missing, stale, and fresh heartbeats.

- [ ] **Step 2: Integrate READY subscription into RViz goal bridge**

Reject before `wait_for_server()` and log `Ignoring goal: localization is not READY`. Do not queue rejected goals for later automatic execution.

Track the accepted RViz goal handle. When READY becomes false or its heartbeat expires, call `cancel_goal_async()` once and clear the handle after the cancellation response. Add a fake action-client test proving an active goal is cancelled and never automatically resent.

- [ ] **Step 3: Change driver default input and preserve watchdog**

Set `cmd_vel_topic: /cmd_vel_safe` in YAML and Python default. Keep the driver unaware of localization state. Extend the existing watchdog test to assert a gate-output interruption causes `STAND` after 0.35 s.

- [ ] **Step 4: Run focused tests and commit**

Run: `PYTHONPATH=packages/omx_navigation:packages/go1_driver python3 -m pytest packages/omx_navigation/test/test_goal_gate.py packages/go1_driver/test/test_command_filter.py -v`
Expected: PASS.

```bash
git add packages/omx_navigation/omx_navigation/rviz_goal_bridge.py packages/omx_navigation/test/test_goal_gate.py packages/go1_driver/config/go1_driver.yaml packages/go1_driver/go1_driver/node.py packages/go1_driver/test/test_command_filter.py
git commit -m "feat: enforce localization at goal and driver boundary"
```

---

### Task 6: Safe Launch Integration

**Files:**
- Modify: `packages/omx_navigation/launch/go1_existing_map.launch.py`
- Modify: `packages/omx_navigation/launch/rviz_navigation.launch.py`
- Modify: `packages/omx_navigation/test/test_existing_map_launch.py`
- Modify: `migration/stage_local_ros2_packages.sh`

**Interfaces:**
- New launch arguments: `start_pose_file` default `""`, `localization_params_file`
- Safety chain: Nav2 `/cmd_vel` → `/cmd_vel_nav` → motion gate → `/cmd_vel_safe` → driver

- [ ] **Step 1: Write failing launch contract tests**

Assert the launch text contains supervisor and motion gate nodes, remaps controller output, gives the driver `/cmd_vel_safe`, accepts empty `start_pose_file` with `arm=false`, and raises `RuntimeError("arm=true requires start_pose_file")` for an empty armed configuration.

- [ ] **Step 2: Implement launch validation**

Extend `OpaqueFunction` validation:

- map must exist
- empty start pose is valid only when arm is false
- non-empty start pose must exist and pass `load_start_pose`
- invalid boolean strings are rejected rather than treated truthy

- [ ] **Step 3: Wire the nodes and topics**

Start supervisor and gate before driver. Remap every Nav2 velocity producer feeding the robot to `/cmd_vel_nav`. Pass `/cmd_vel_safe` explicitly to `go1_driver.launch.py`. Ensure there is no remaining direct `/cmd_vel` → driver route.

- [ ] **Step 4: Update deployment staging**

Ensure new modules, YAML, fixtures needed by package tests, and scripts are copied without adding PCD files to this commit.

- [ ] **Step 5: Run contract suite and commit**

Run: `PYTHONPATH=packages/omx_navigation:packages/go1_driver python3 -m pytest packages/omx_navigation/test packages/go1_driver/test migration/test_existing_map_scripts.py -v`
Expected: all platform-independent tests PASS; ROS installation-specific tests may skip with an explicit reason.

```bash
git add packages/omx_navigation/launch/go1_existing_map.launch.py packages/omx_navigation/launch/rviz_navigation.launch.py packages/omx_navigation/test/test_existing_map_launch.py migration/stage_local_ros2_packages.sh
git commit -m "feat: wire fixed-start localization safety chain"
```

---

### Task 7: Runtime Verification and Operator Documentation

**Files:**
- Create: `migration/verify_fixed_start_localization.sh`
- Modify: `migration/verify_existing_map_navigation.sh`
- Modify: `migration/test_existing_map_scripts.py`
- Modify: `packages/omx_navigation/README.md`
- Modify: `README.md`

**Interfaces:**
- Verification modes: `uncommissioned`, `localized`
- Runtime assertions: ready heartbeat, supervisor state, gate state, safe output, driver input parameter

- [ ] **Step 1: Write failing script contract tests**

Require checks for `/localization/ready`, `/localization/state`, `/motion_gate/state`, `/cmd_vel_nav`, `/cmd_vel_safe`, `go1_driver cmd_vel_topic`, and both modes. In uncommissioned mode require `UNCOMMISSIONED` and `ready=false`; in localized mode require `READY` and fresh TF.

- [ ] **Step 2: Implement runtime verification**

Use bounded `timeout` for all ROS commands. Never release E-stop from the verification script. Verify that publishing a test `/cmd_vel_nav` while uncommissioned leaves `/cmd_vel_safe` zero. Require `arm=false` during all automated verification.

- [ ] **Step 3: Document commissioning without current coordinates**

Document these exact phases:

1. Build and test with `start_pose_file:="" arm:=false`.
2. At the site, collect ten RViz-localized samples into a file outside the repo.
3. Save the ten collected samples to `/mnt/t500/go1_runtime/start_pose_samples.yaml`, then run `ros2 run omx_navigation commission_start_pose --input /mnt/t500/go1_runtime/start_pose_samples.yaml --output /mnt/t500/go1_runtime/start_pose.yaml`.
4. Run ten automatic dry-run placements with the generated preset.
5. Run deliberate `0.30 m` and `20 deg` rejection trials.
6. Only after all trials pass, begin armed testing at `0.05 m/s`.

Clarify that the implementation can be completed now even though the operational x/y/yaw values are unavailable.

- [ ] **Step 4: Run full static verification**

Run:

```bash
PYTHONPATH=packages/omx_navigation:packages/go1_driver \
python3 -m pytest -q \
  packages/omx_navigation/test \
  packages/go1_driver/test \
  migration/test_existing_map_scripts.py \
  migration/test_end_to_end_workflow.py
```

Expected: zero failures. Record skipped ROS-dependent tests separately.

- [ ] **Step 5: Run ROS 2 build and tests on Jetson/Humble**

```bash
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
colcon test --packages-select go1_driver omx_navigation --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build exit 0 and zero failed tests.

- [ ] **Step 6: Commit**

```bash
git add migration/verify_fixed_start_localization.sh migration/verify_existing_map_navigation.sh migration/test_existing_map_scripts.py packages/omx_navigation/README.md README.md
git commit -m "docs: add fixed-start AMCL operating procedure"
```

---

## Final Acceptance Sequence

- [ ] Confirm the diff contains no PCD localization files.
- [ ] Confirm empty preset + `arm=false` reaches `UNCOMMISSIONED` and never outputs motion.
- [ ] Confirm empty preset + `arm=true` fails before driver startup.
- [ ] Confirm a valid test fixture can reach READY under synthetic passing evidence.
- [ ] Confirm `0.251 m` or `15.1 deg` preset deviation prevents READY.
- [ ] Confirm scan median `> 0.15 m` or p80 `> 0.30 m` prevents READY.
- [ ] Confirm supervisor death stops gate output within 0.30 s.
- [ ] Confirm gate death triggers driver STAND within 0.35 s.
- [ ] Confirm RViz goals are rejected before READY and are not queued.
- [ ] Confirm all platform-independent pytest suites pass.
- [ ] Confirm Jetson/Humble colcon build/test results before any real robot claim.
