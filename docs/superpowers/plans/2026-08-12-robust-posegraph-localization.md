# Go1 Robust Pose-Graph Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 한 번의 대략적인 RViz 초기 자세 입력을 정리된 2D 지도에서 `±3 m`, `±90 deg` 범위로 보정하고, 검증된 pose-graph localization 상태에서만 Nav2 속도를 Go1 dry-run driver로 전달한다.

**Architecture:** ROS 비의존 코어가 coarse scan-to-map search, 상태 전이, 속도 gate 결정을 담당하고 ROS2 노드는 토픽과 TF만 연결한다. Map server는 정리된 occupancy map을, SLAM Toolbox localization은 저장 pose graph를 사용하며 유일하게 `map -> camera_init`을 발행한다. Nav2 출력은 `/cmd_vel_nav`를 거쳐 localization safety gate가 `READY`일 때만 `/cmd_vel`로 전달한다.

**Tech Stack:** Python 3.10+, ROS2 Humble, rclpy, Nav2, SLAM Toolbox, pytest, PyYAML, WSL2 Ubuntu 22.04

## Global Constraints

- 기본 실행은 항상 `arm:=false`이고 이 launch에서 `arm:=true`는 거부한다.
- 운영 TF 계약은 `map -> camera_init -> body_nav`이며 `camera_init -> body`는 FAST-LIO 원본 TF로 유지한다.
- 초기 후보 검색 범위는 위치 `±3.0 m`, 방향 `±90 deg`, 전체 timeout `20 s`, 최대 시도 `3회`다.
- `READY` heartbeat 또는 Nav2 명령이 `0.30 s`보다 오래되면 출력은 0이다.
- LiDAR 장착과 extrinsic이 고정되지 않았으므로 Jetson 실기 성공이나 실제 구동을 완료 조건으로 삼지 않는다.
- Windows 순수 Python 테스트와 WSL x86_64 ROS2 Humble build/test 결과만 완료 근거로 사용한다.
- 기존 AMCL launch는 fallback으로 보존하고 pose-graph launch와 동시에 실행하지 않는다.

---

### Task 1: Localization 상태 머신 코어

**Files:**
- Create: `packages/omx_navigation/omx_navigation/localization_state.py`
- Create: `packages/omx_navigation/test/test_localization_state.py`

**Interfaces:**
- Produces: `LocalizationState`, `ErrorCode`, `LocalizationPolicy`, `QualityObservation`, `Transition`, `LocalizationStateMachine`
- `LocalizationStateMachine.receive_initial_pose(now: float) -> Transition`
- `LocalizationStateMachine.observe(observation: QualityObservation) -> Transition`
- `LocalizationStateMachine.retry(now: float) -> Transition`

- [ ] **Step 1: Write failing state transition tests**

```python
def good(now: float) -> QualityObservation:
    return QualityObservation(
        now=now,
        inputs_fresh=True,
        pose_available=True,
        overlap=0.70,
        ambiguity_margin=0.20,
        position_jump=0.01,
        yaw_jump=0.01,
        odom_reset=False,
        tf_conflict=False,
    )


def test_one_input_reaches_ready_after_stable_verification():
    machine = LocalizationStateMachine(LocalizationPolicy(verify_duration=3.0))
    assert machine.receive_initial_pose(0.0).state is LocalizationState.ALIGNING
    assert machine.observe(good(0.1)).state is LocalizationState.VERIFYING
    assert machine.observe(good(3.2)).state is LocalizationState.READY


def test_alignment_timeout_retries_three_times_then_loses():
    machine = LocalizationStateMachine(
        LocalizationPolicy(alignment_timeout=2.0, max_attempts=3)
    )
    machine.receive_initial_pose(0.0)
    first = machine.observe(QualityObservation.missing(now=2.1))
    second = machine.observe(QualityObservation.missing(now=4.2))
    final = machine.observe(QualityObservation.missing(now=6.3))
    assert first.republish_initial_pose is True
    assert second.republish_initial_pose is True
    assert final.state is LocalizationState.LOST
    assert final.error is ErrorCode.ALIGNMENT_TIMEOUT


def test_ready_degrades_then_loses_after_two_seconds():
    machine = ready_machine()
    degraded = machine.observe(replace(good(4.0), overlap=0.10))
    lost = machine.observe(replace(good(6.1), overlap=0.10))
    assert degraded.state is LocalizationState.DEGRADED
    assert lost.state is LocalizationState.LOST
    assert lost.error is ErrorCode.LOW_OVERLAP


def test_odom_reset_and_tf_conflict_are_immediate_loss():
    for field, error in (
        ("odom_reset", ErrorCode.ODOM_RESET),
        ("tf_conflict", ErrorCode.TF_CONFLICT),
    ):
        machine = ready_machine()
        transition = machine.observe(replace(good(4.0), **{field: True}))
        assert transition.state is LocalizationState.LOST
        assert transition.error is error
```

- [ ] **Step 2: Run the tests and confirm RED**

Run:

```powershell
$env:PYTHONPATH=(Resolve-Path 'packages\omx_navigation').Path
python -m pytest -q -p no:cacheprovider packages/omx_navigation/test/test_localization_state.py
```

Expected: collection fails with `ModuleNotFoundError: omx_navigation.localization_state`.

- [ ] **Step 3: Implement the minimal deterministic state machine**

```python
class LocalizationState(str, Enum):
    WAITING_INPUT = "WAITING_INPUT"
    ALIGNING = "ALIGNING"
    VERIFYING = "VERIFYING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


class ErrorCode(str, Enum):
    NONE = "NONE"
    INPUT_MISSING = "INPUT_MISSING"
    POSE_OUTSIDE_MAP = "POSE_OUTSIDE_MAP"
    ALIGNMENT_TIMEOUT = "ALIGNMENT_TIMEOUT"
    LOW_OVERLAP = "LOW_OVERLAP"
    AMBIGUOUS = "AMBIGUOUS"
    ODOM_RESET = "ODOM_RESET"
    TF_CONFLICT = "TF_CONFLICT"
    EXTRINSIC_UNCALIBRATED = "EXTRINSIC_UNCALIBRATED"


@dataclass(frozen=True)
class LocalizationPolicy:
    alignment_timeout: float = 20.0
    max_attempts: int = 3
    verify_duration: float = 3.0
    degraded_timeout: float = 2.0
    min_overlap: float = 0.45
    min_ambiguity_margin: float = 0.05
    max_position_jump: float = 0.30
    max_yaw_jump: float = math.radians(10.0)


@dataclass(frozen=True)
class Transition:
    state: LocalizationState
    error: ErrorCode
    republish_initial_pose: bool
    publish_stop: bool
```

Validate every duration/count/range in `LocalizationPolicy.__post_init__`. Keep all clocks injected through observation values so Windows tests never sleep.

- [ ] **Step 4: Run state tests and the existing suite**

Run the Task 1 test command, then:

```powershell
$env:PYTHONPATH=((Resolve-Path 'packages\go1_driver').Path + ';' + (Resolve-Path 'packages\omx_navigation').Path)
python -m pytest -q -p no:cacheprovider packages/go1_driver/test packages/omx_navigation/test migration/test_existing_map_scripts.py migration/test_end_to_end_workflow.py
```

Expected: all tests pass with the existing single environment-dependent skip preserved.

- [ ] **Step 5: Commit Task 1**

```powershell
git add packages/omx_navigation/omx_navigation/localization_state.py packages/omx_navigation/test/test_localization_state.py
git commit -m "feat: add localization safety state machine"
```

---

### Task 2: 2D scan-map coarse search와 모호성 판정

**Files:**
- Create: `packages/omx_navigation/omx_navigation/scan_map_quality.py`
- Create: `packages/omx_navigation/test/test_scan_map_quality.py`

**Interfaces:**
- Consumes: occupancy values, map geometry, LaserScan ranges converted to `ScanPoint`
- Produces: `GridMap`, `Pose2D`, `SearchWindow`, `PoseScore`, `SearchResult`
- `build_distance_field(grid: GridMap) -> tuple[float, ...]`
- `score_pose(grid: GridMap, field: Sequence[float], points: Sequence[ScanPoint], pose: Pose2D, hit_distance: float) -> PoseScore`
- `coarse_search(grid: GridMap, points: Sequence[ScanPoint], initial: Pose2D, window: SearchWindow) -> SearchResult`

- [ ] **Step 1: Write failing behavior tests with hand-derived maps**

```python
def corridor_map() -> GridMap:
    width = height = 21
    cells = [0] * (width * height)
    for y in range(2, 19):
        cells[y * width + 5] = 100
        cells[y * width + 15] = 100
    cells[18 * width + 5 : 18 * width + 16] = [100] * 11
    return GridMap(width, height, 0.5, 0.0, 0.0, 0.0, tuple(cells))


def test_coarse_search_corrects_two_meter_and_ninety_degree_error():
    grid = asymmetric_room_map()
    scan = scan_points_for_pose(grid, Pose2D(5.0, 5.0, 0.0))
    result = coarse_search(
        grid,
        scan,
        Pose2D(3.0, 5.0, math.pi / 2),
        SearchWindow(3.0, 0.5, math.pi / 2, math.radians(15)),
    )
    assert result.best.pose.x == pytest.approx(5.0, abs=0.51)
    assert result.best.pose.y == pytest.approx(5.0, abs=0.51)
    assert angle_distance(result.best.pose.yaw, 0.0) <= math.radians(15)
    assert result.best.overlap >= 0.80
    assert result.ambiguous is False


def test_symmetric_corridor_is_ambiguous():
    result = coarse_search(
        corridor_map(), symmetric_corridor_scan(), Pose2D(5.0, 5.0, 0.0),
        SearchWindow(3.0, 0.5, math.pi / 2, math.radians(30)),
    )
    assert result.ambiguous is True


def test_empty_nonfinite_and_outside_scan_never_looks_good():
    result = coarse_search(
        corridor_map(),
        (ScanPoint(float("nan"), 0.0), ScanPoint(100.0, 100.0)),
        Pose2D(-20.0, -20.0, 0.0),
        SearchWindow(1.0, 0.5, math.radians(30), math.radians(15)),
    )
    assert result.best.overlap == 0.0
    assert math.isinf(result.best.mean_distance)
```

- [ ] **Step 2: Run and confirm RED**

Run:

```powershell
$env:PYTHONPATH=(Resolve-Path 'packages\omx_navigation').Path
python -m pytest -q -p no:cacheprovider packages/omx_navigation/test/test_scan_map_quality.py
```

Expected: missing module failure.

- [ ] **Step 3: Implement map geometry and distance field**

```python
@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        gx = (c * dx + s * dy) / self.resolution
        gy = (-s * dx + c * dy) / self.resolution
        ix, iy = math.floor(gx), math.floor(gy)
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None
```

Use a multi-source eight-neighbor Dijkstra seeded by cells `>= occupied_threshold`. Distances are meters and unknown cells are not treated as occupied.

- [ ] **Step 4: Implement bounded candidate generation and scoring**

```python
@dataclass(frozen=True)
class SearchWindow:
    translation_radius: float = 3.0
    translation_step: float = 0.5
    yaw_radius: float = math.pi / 2
    yaw_step: float = math.radians(15.0)
    hit_distance: float = 0.25
    ambiguity_margin: float = 0.05
    max_scan_points: int = 180


@dataclass(frozen=True)
class PoseScore:
    pose: Pose2D
    overlap: float
    mean_distance: float
    score: float
    points_used: int


@dataclass(frozen=True)
class SearchResult:
    best: PoseScore
    runner_up: PoseScore | None
    ambiguous: bool
```

The score is `overlap - 0.20 * min(mean_distance, 1.0)`. Select a runner-up only when it differs from the best by at least `0.75 m` or `20 deg`; mark ambiguous when `best.score - runner_up.score < ambiguity_margin`.

- [ ] **Step 5: Run Task 2 and full tests**

Expected: all new and existing tests pass.

- [ ] **Step 6: Commit Task 2**

```powershell
git add packages/omx_navigation/omx_navigation/scan_map_quality.py packages/omx_navigation/test/test_scan_map_quality.py
git commit -m "feat: add bounded scan map pose search"
```

---

### Task 3: Localization-aware velocity gate

**Files:**
- Create: `packages/omx_navigation/omx_navigation/cmd_vel_gate_core.py`
- Create: `packages/omx_navigation/omx_navigation/cmd_vel_safety_gate.py`
- Create: `packages/omx_navigation/test/test_cmd_vel_gate_core.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Produces: `VelocityCommand`, `VelocityGate`, console script `cmd_vel_safety_gate`
- `VelocityGate.update_ready(ready: bool, now: float) -> GateDecision`
- `VelocityGate.filter(command: VelocityCommand, now: float) -> GateDecision`
- `VelocityGate.watchdog(now: float) -> GateDecision`
- ROS topics: `/cmd_vel_nav` input, `/localization_supervisor/ready` input, `/cmd_vel` output

- [ ] **Step 1: Write failing gate tests**

```python
def test_gate_never_passes_before_fresh_ready_heartbeat():
    gate = VelocityGate(ready_timeout=0.30, command_timeout=0.30)
    decision = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.0)
    assert decision.command == VelocityCommand.zero()
    assert decision.reason == "localization_not_ready"


def test_gate_passes_only_while_ready_and_fresh():
    gate = VelocityGate(ready_timeout=0.30, command_timeout=0.30)
    gate.update_ready(True, now=1.0)
    moving = gate.filter(VelocityCommand(0.2, 0.0, 0.1), now=1.1)
    stale = gate.watchdog(now=1.31)
    assert moving.command.vx == 0.2
    assert stale.command == VelocityCommand.zero()
    assert stale.reason == "ready_heartbeat_stale"


def test_ready_false_closes_gate_and_requests_stop():
    gate = ready_gate(now=1.0)
    closed = gate.update_ready(False, now=1.1)
    assert closed.publish is True
    assert closed.command == VelocityCommand.zero()
```

- [ ] **Step 2: Run and confirm RED**

Expected: missing `cmd_vel_gate_core` module.

- [ ] **Step 3: Implement the ROS-independent gate**

```python
@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    vy: float
    yaw: float

    @classmethod
    def zero(cls) -> "VelocityCommand":
        return cls(0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GateDecision:
    command: VelocityCommand
    publish: bool
    reason: str
```

Reject non-finite commands. A false/stale ready signal always overrides command freshness.

- [ ] **Step 4: Implement the ROS wrapper and package entries**

The node publishes latched transition stops plus a `20 Hz` watchdog stop while closed. Add:

```python
"cmd_vel_safety_gate = omx_navigation.cmd_vel_safety_gate:main",
```

Add `std_msgs` to `package.xml` and keep the node import isolated from the core so Windows can import `cmd_vel_gate_core` without ROS2.

- [ ] **Step 5: Run Task 3 and full tests**

- [ ] **Step 6: Commit Task 3**

```powershell
git add packages/omx_navigation/omx_navigation/cmd_vel_gate_core.py packages/omx_navigation/omx_navigation/cmd_vel_safety_gate.py packages/omx_navigation/test/test_cmd_vel_gate_core.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: gate velocity on localization readiness"
```

---

### Task 4: FAST-LIO 평면 주행 프레임

**Files:**
- Create: `packages/omx_navigation/omx_navigation/planar_base_frame.py`
- Create: `packages/omx_navigation/omx_navigation/planar_transform.py`
- Create: `packages/omx_navigation/test/test_planar_transform.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`
- Modify: `packages/omx_navigation/config/mid360_scan.yaml`

**Interfaces:**
- `planarize_transform(x, y, z, qx, qy, qz, qw) -> PlanarTransform`
- console script `planar_base_frame`
- TF input: `camera_init -> body`; TF output: `camera_init -> body_nav`

- [ ] **Step 1: Write failing quaternion tests**

```python
@pytest.mark.parametrize(
    ("roll", "pitch", "yaw"),
    ((0.3, -0.2, 0.7), (-0.4, 0.1, -1.2), (0.0, 0.0, math.pi)),
)
def test_planar_transform_keeps_translation_and_yaw_only(roll, pitch, yaw):
    q = quaternion_from_rpy(roll, pitch, yaw)
    result = planarize_transform(1.0, 2.0, 0.4, *q)
    out_roll, out_pitch, out_yaw = rpy_from_quaternion(result.quaternion)
    assert result.translation == pytest.approx((1.0, 2.0, 0.0))
    assert out_roll == pytest.approx(0.0, abs=1e-9)
    assert out_pitch == pytest.approx(0.0, abs=1e-9)
    assert angle_distance(out_yaw, yaw) < 1e-9


def test_zero_norm_quaternion_is_rejected():
    with pytest.raises(ValueError, match="quaternion"):
        planarize_transform(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
```

- [ ] **Step 2: Run and confirm RED**

- [ ] **Step 3: Implement planar math and TF broadcaster**

The ROS node looks up `odom_frame -> source_base_frame` at `20 Hz`, preserves x/y and yaw, sets z/roll/pitch to zero, stamps with the lookup time, and broadcasts `odom_frame -> planar_base_frame`. Parameters:

```text
odom_frame=camera_init
source_base_frame=body
planar_base_frame=body_nav
publish_rate=20.0
transform_timeout=0.10
```

- [ ] **Step 4: Point scan projection at `body_nav` and add dependencies**

Change only `target_frame`; retain the current height window until recorded walking data exists.

- [ ] **Step 5: Run Task 4 and full tests**

- [ ] **Step 6: Commit Task 4**

```powershell
git add packages/omx_navigation/omx_navigation/planar_transform.py packages/omx_navigation/omx_navigation/planar_base_frame.py packages/omx_navigation/test/test_planar_transform.py packages/omx_navigation/config/mid360_scan.yaml packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: add gravity aligned navigation frame"
```

---

### Task 5: Localization supervisor와 goal 취소

**Files:**
- Create: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Create: `packages/omx_navigation/omx_navigation/ros_conversions.py`
- Create: `packages/omx_navigation/test/test_ros_conversions.py`
- Modify: `packages/omx_navigation/omx_navigation/rviz_goal_bridge.py`
- Create: `packages/omx_navigation/omx_navigation/goal_gate.py`
- Create: `packages/omx_navigation/test/test_goal_gate.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`

**Interfaces:**
- Subscribes: `/map`, `/scan`, `/Odometry`, `/initialpose`, `/slam_toolbox/pose`, `/tf`
- Publishes: `/slam_localization/initialpose`, `~/status` (`std_msgs/String`), `~/ready` (`std_msgs/Bool`)
- `status` JSON keys: `state`, `error`, `message_ko`, `attempt`, `overlap`, `ambiguity_margin`, `stamp`
- `GoalGate.update_ready(bool) -> bool` returns whether an active goal must be cancelled

- [ ] **Step 1: Write failing conversion and goal gate tests**

```python
def test_laser_ranges_convert_to_bounded_evenly_sampled_points():
    points = laser_ranges_to_points(
        ranges=(1.0, float("inf"), 2.0, float("nan"), 3.0),
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.2,
        range_max=2.5,
        max_points=2,
    )
    assert len(points) == 2
    assert all(math.isfinite(p.x) and math.isfinite(p.y) for p in points)


def test_goal_gate_rejects_until_ready_and_cancels_on_loss():
    gate = GoalGate()
    assert gate.accept_goal() is False
    gate.update_ready(True)
    assert gate.accept_goal() is True
    gate.set_goal_active(True)
    assert gate.update_ready(False) is True
```

- [ ] **Step 2: Run and confirm RED**

- [ ] **Step 3: Implement conversions and goal policy**

Keep message-free functions in `ros_conversions.py`; ROS messages are unpacked by the supervisor before calling them. Normalize quaternion yaw and reject maps whose cell count differs from `width * height`.

- [ ] **Step 4: Implement supervisor orchestration**

On user initial pose:

1. Reject a non-`map` frame or pose outside `/map`.
2. Wait for a fresh scan and map.
3. Run `coarse_search` within the configured window.
4. If overlap is below `0.45`, emit `LOW_OVERLAP`; if ambiguous, emit `AMBIGUOUS`.
5. Publish the refined pose to `/slam_localization/initialpose`.
6. Verify `/slam_toolbox/pose` for 3 seconds through the state machine.

At `2 Hz`, publish a JSON status and `ready`. Detect `/amcl` in `get_node_names()` as `TF_CONFLICT`. Detect odom jumps above `3.0 m/s` between valid samples as `ODOM_RESET`. Append each status row to the configured `diagnostics_csv` with `flush()` after every write.

- [ ] **Step 5: Make RViz goal bridge readiness-aware**

Subscribe to `/localization_supervisor/ready`. Reject goals until true. Store the accepted goal handle; on ready transition true→false call `cancel_goal_async()` and clear the handle on result.

- [ ] **Step 6: Run Task 5 and full tests**

- [ ] **Step 7: Commit Task 5**

```powershell
git add packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/omx_navigation/ros_conversions.py packages/omx_navigation/omx_navigation/goal_gate.py packages/omx_navigation/omx_navigation/rviz_goal_bridge.py packages/omx_navigation/test/test_ros_conversions.py packages/omx_navigation/test/test_goal_gate.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: supervise pose graph localization"
```

---

### Task 6: Pose-graph Nav2 설정과 통합 launch

**Files:**
- Create: `packages/omx_navigation/config/slam_toolbox_localization_hanyang_9f.yaml`
- Create: `packages/omx_navigation/config/nav2_posegraph_params.yaml`
- Create: `packages/omx_navigation/launch/go1_posegraph_navigation.launch.py`
- Create: `packages/omx_navigation/test/test_posegraph_config.py`
- Create: `packages/omx_navigation/test/test_posegraph_launch.py`
- Modify: `packages/omx_navigation/config/nav2_existing_map_params.yaml`
- Modify: `packages/omx_navigation/test/test_existing_map_params.py`
- Modify: `packages/omx_navigation/test/test_nav2_humble_compatibility.py`

**Interfaces:**
- Launch arguments: `map`, `posegraph`, `cloud_topic`, `scan_topic`, `odom_topic`, `odom_frame`, `source_base_frame`, `base_frame`, `record_localization`, `record_cloud`, `diagnostics_root`, `rviz`, `start_go1_driver`, `arm`, `use_composition`
- Default map: `.../hanyang_9f_annotated.yaml`
- Default posegraph base: `.../hanyang_9f` (without extension)

- [ ] **Step 1: Write failing parsed-config tests**

```python
def test_posegraph_localization_owns_map_to_odom_contract():
    params = load_slam()["slam_toolbox"]["ros__parameters"]
    assert params["mode"] == "localization"
    assert params["map_frame"] == "map"
    assert params["odom_frame"] == "camera_init"
    assert params["base_frame"] == "body_nav"
    assert params["map_file_name"].endswith("/hanyang_9f")
    assert params["scan_queue_size"] == 1
    assert params["correlation_search_space_dimension"] <= 1.0


def test_posegraph_nav2_has_no_amcl_and_uses_safe_footprint():
    config = load_nav2()
    assert "amcl" not in config
    for name in ("local_costmap", "global_costmap"):
        params = config[name][name]["ros__parameters"]
        assert params["robot_base_frame"] == "body_nav"
        assert params["footprint"] == "[[0.37, 0.19], [0.37, -0.19], [-0.37, -0.19], [-0.37, 0.19]]"
        assert params["transform_tolerance"] == 0.5
    assert config["controller_server"]["ros__parameters"]["FollowPath"]["BaseObstacle.scale"] == 0.02
```

- [ ] **Step 2: Write failing launch contract tests**

Load the launch module with stubbed `ament_index_python`, call the pure `validate_inputs(paths, arm)` helper, and assert:

```python
with pytest.raises(RuntimeError, match="arm:=true"):
    validate_inputs(valid_paths(), arm=True)

with pytest.raises(RuntimeError, match="posegraph"):
    validate_inputs(paths_without_data_file(), arm=False)
```

Also assert the launch remaps:

```text
/initialpose -> /slam_localization/initialpose (SLAM Toolbox only)
controller output -> /cmd_vel_nav
SLAM Toolbox /map -> /slam_localization/map
```

- [ ] **Step 3: Run and confirm RED**

- [ ] **Step 4: Create localization and Nav2 YAML**

Copy only non-localizer Nav2 sections from the existing profile, remove `amcl`, set `body_nav`, footprint, `BaseObstacle.scale: 0.02`, and all relevant transform tolerances to `0.5`. Keep `velocity_smoother.feedback: OPEN_LOOP` with an explanatory comment.

Use the saved pose graph with local matcher values:

```yaml
slam_toolbox:
  ros__parameters:
    mode: localization
    map_file_name: /mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f
    map_frame: map
    odom_frame: camera_init
    base_frame: body_nav
    scan_topic: /scan
    transform_publish_period: 0.05
    scan_queue_size: 1
    scan_buffer_size: 10
    correlation_search_space_dimension: 0.5
    coarse_search_angle_offset: 0.349
```

- [ ] **Step 5: Implement integrated dry-run launch**

Use `slam_toolbox/launch/localization_launch.py`, a separate Nav2 map server and lifecycle manager, and `nav2_bringup/navigation_launch.py`. Start planar frame before scan projection. Validate `.yaml`, referenced `.pgm`, `.posegraph`, `.data`, all params, and RViz config before nodes start. Raise unconditionally for `arm:=true` in this launch.

- [ ] **Step 6: Harden the AMCL fallback settings without changing its architecture**

Apply footprint, `BaseObstacle.scale: 0.02`, and `0.5 s` transform tolerance fixes to `nav2_existing_map_params.yaml`; retain AMCL frames on `body` because the fallback launch does not start `body_nav`.

- [ ] **Step 7: Run Task 6 and full tests**

- [ ] **Step 8: Commit Task 6**

```powershell
git add packages/omx_navigation/config packages/omx_navigation/launch/go1_posegraph_navigation.launch.py packages/omx_navigation/test
git commit -m "feat: add pose graph navigation bringup"
```

---

### Task 7: 현장 진단 기록, 검증 스크립트, 한국어 문서

**Files:**
- Create: `migration/verify_posegraph_navigation.sh`
- Create: `migration/test_posegraph_scripts.py`
- Modify: `migration/stage_local_ros2_packages.sh`
- Modify: `packages/omx_navigation/README.md`
- Modify: `README.md`

**Interfaces:**
- `verify_posegraph_navigation.sh [preflight|ready]`
- Diagnostic output root defaults to `/mnt/t500/localization_logs`
- rosbag topics are fixed by the design spec; `record_cloud:=true` adds `/cloud_registered_body`

- [ ] **Step 1: Write failing executable script contract tests**

Run scripts through `bash -n` when bash is available and parse their declared topic arrays. Assert the verifier requires:

```text
/scan
/Odometry
/map
/slam_toolbox/pose
/localization_supervisor/status
/localization_supervisor/ready
/cmd_vel_nav
/cmd_vel
```

Assert ready mode rejects `/amcl`, requires `map -> camera_init -> body_nav`, checks all Nav2 lifecycle nodes, and requires `/go1_driver arm` false.

- [ ] **Step 2: Run and confirm RED**

- [ ] **Step 3: Add launch rosbag action and verifier**

The launch creates a timestamped directory only when `record_localization:=true`, starts:

```bash
ros2 bag record --output "${session_dir}/rosbag" /scan /Odometry /tf /tf_static /initialpose /slam_toolbox/pose /localization_supervisor/status /localization_supervisor/ready /cmd_vel_nav /cmd_vel
```

and passes `${session_dir}/localization_status.csv` to the supervisor. With
`record_cloud:=true`, append `/cloud_registered_body`.

- [ ] **Step 4: Update Korean operator documentation**

Document only:

1. Windows tests.
2. WSL build/test commands.
3. Jetson future dry-run with `arm:=false`.
4. Status/error code interpretation and generated diagnostic paths.
5. The explicit restriction that sensor mounting calibration is required before implementing an armed launch.

- [ ] **Step 5: Run script, documentation, and full regression tests**

- [ ] **Step 6: Commit Task 7**

```powershell
git add migration/verify_posegraph_navigation.sh migration/test_posegraph_scripts.py migration/stage_local_ros2_packages.sh packages/omx_navigation/README.md README.md packages/omx_navigation/launch/go1_posegraph_navigation.launch.py
git commit -m "docs: add pose graph localization diagnostics"
```

---

### Task 8: Windows 및 WSL ROS2 Humble 최종 검증

**Files:**
- Modify only if verification exposes a defect in Tasks 1-7.

**Interfaces:**
- Consumes all deliverables above.
- Produces fresh Windows pytest, WSL colcon, launch argument, and git-diff evidence.

- [ ] **Step 1: Run the full Windows test suite**

```powershell
$env:PYTHONPATH=((Resolve-Path 'packages\go1_driver').Path + ';' + (Resolve-Path 'packages\omx_navigation').Path)
python -m pytest -q -p no:cacheprovider packages/go1_driver/test packages/omx_navigation/test migration/test_existing_map_scripts.py migration/test_end_to_end_workflow.py migration/test_posegraph_scripts.py
```

Expected: zero failures and only explicitly environment-dependent skips.

- [ ] **Step 2: Create an isolated WSL build workspace**

```bash
workspace="$(mktemp -d /tmp/go1-posegraph-XXXXXX)"
mkdir -p "$workspace/src"
cp -a /mnt/c/Users/npgy2/Documents/go1/GO1_to_ROS2_YEEPY/packages/go1_driver "$workspace/src/"
cp -a /mnt/c/Users/npgy2/Documents/go1/GO1_to_ROS2_YEEPY/packages/omx_navigation "$workspace/src/"
printf '%s\n' "$workspace"
```

- [ ] **Step 3: Build and test in WSL ROS2 Humble**

```bash
source /opt/ros/humble/setup.bash
cd "$workspace"
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
colcon test --packages-select go1_driver omx_navigation --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build exit 0 and test-result reports zero failures.

- [ ] **Step 4: Verify installed launch imports and arguments**

```bash
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
ros2 launch omx_navigation go1_posegraph_navigation.launch.py --show-args
python3 -m py_compile "$workspace"/install/omx_navigation/lib/python3.10/site-packages/omx_navigation/*.py
```

Expected: `--show-args` lists every Task 6 argument without import errors; Python compilation exits 0.

- [ ] **Step 5: Inspect repository integrity**

```powershell
git status --short
git diff --check HEAD~7..HEAD
git diff --stat origin/main...HEAD
```

Confirm `.worktrees/` and inaccessible pre-existing pytest cache directories are not staged.

- [ ] **Step 6: Request independent code review and fix Critical/Important findings**

Provide the reviewer with `origin/main` as base, current `HEAD`, this plan, and the design spec. Re-run Steps 1-5 after any fix.

- [ ] **Step 7: Commit verification-only fixes if present**

If review required fixes, stage each named file from the applicable task's
`git add` list and commit with `git commit -m "fix: address pose graph localization review"`.
If review found no defects, do not create an empty commit.
