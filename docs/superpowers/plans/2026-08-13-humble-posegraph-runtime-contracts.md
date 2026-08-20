# Humble Posegraph Runtime Contracts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the saved-posegraph localization launch use the real ROS 2 Humble SLAM Toolbox interfaces, reach READY while stationary, preserve fail-closed navigation, and produce trustworthy Windows/WSL test evidence before GitHub push.

**Architecture:** SLAM Toolbox loads the serialized graph with an explicit start pose and remaps its real `/pose` output into the localization namespace. A post-initialpose pose event is only a scan-match handshake; continuous map pose and quality come from composing fresh `map -> camera_init` and `camera_init -> body_nav` transforms. The supervisor rejects stale sensor/TF data, map-transform discontinuities, invalid reinitialization requests, and known AMCL conflicts before publishing READY.

**Tech Stack:** Python 3.10, ROS 2 Humble, rclpy, SLAM Toolbox 2.6.x, tf2 messages, Nav2, ament_python, pytest, PowerShell, WSL2 Ubuntu 22.04

## Global Constraints

- Default launch remains `arm:=false`; posegraph launch must reject `arm:=true` before starting nodes.
- Posegraph search remains bounded to a Euclidean `3.0 m` radius and `±90 deg` yaw.
- `/cmd_vel` is published only through the localization safety gate while READY heartbeat and command are each newer than `0.30 s`.
- `/scan`, `/Odometry`, `map -> camera_init`, and `camera_init -> body_nav` must each be newer than `0.50 s`.
- One `/slam_localization/pose` received after the refined initialpose epoch is required, but it is not a periodic heartbeat.
- Do not alter or stage the pre-existing untracked `.worktrees/` directory.
- Jetson movement and `arm:=true` are outside verification scope.

---

### Task 1: Match the Humble SLAM Toolbox startup and pose API

**Files:**
- Modify: `packages/omx_navigation/config/slam_toolbox_localization_hanyang_9f.yaml`
- Modify: `packages/omx_navigation/launch/go1_posegraph_navigation.launch.py`
- Modify: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Modify: `packages/omx_navigation/test/test_posegraph_config.py`
- Modify: `packages/omx_navigation/test/test_posegraph_launch.py`
- Modify: `packages/omx_navigation/test/test_localization_supervisor.py`

**Interfaces:**
- Consumes: Humble SLAM Toolbox relative topic `pose` with `geometry_msgs/msg/PoseWithCovarianceStamped`.
- Produces: `/slam_localization/pose` and `_on_slam_pose(message: PoseWithCovarianceStamped)`.

- [ ] **Step 1: Write failing configuration and subscription tests**

```python
def test_posegraph_localization_requests_startup_deserialization():
    params = load_slam()["slam_toolbox"]["ros__parameters"]
    assert params["map_start_pose"] == [0.0, 0.0, 0.0]

def test_supervisor_uses_humble_pose_topic_and_type(supervisor_module):
    node = supervisor_module.LocalizationSupervisor()
    subscription = next(item for item in node.subscriptions if item.topic == "/slam_localization/pose")
    assert subscription.message_type is PoseWithCovarianceStamped
```

Also assert the launch group contains a `/pose -> /slam_localization/pose` remap and no `/slam_toolbox/pose` recorder topic.

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```powershell
$env:PYTHONPATH='packages/omx_navigation'
py -3 -m pytest -q -p no:cacheprovider packages/omx_navigation/test/test_posegraph_config.py packages/omx_navigation/test/test_posegraph_launch.py packages/omx_navigation/test/test_localization_supervisor.py
```

Expected: failures for missing `map_start_pose`, missing remap/topic, and wrong message type.

- [ ] **Step 3: Implement the minimal Humble contract**

```yaml
map_file_name: /path/replaced/by/launch
map_start_pose: [0.0, 0.0, 0.0]
```

```python
SetRemap(src="/pose", dst="/slam_localization/pose")
self.create_subscription(
    PoseWithCovarianceStamped,
    "/slam_localization/pose",
    self._on_slam_pose,
    reliable_qos,
)
pose = message.pose.pose
```

The fixed start pose only triggers graph deserialization. The gate remains closed until the user's refined pose is accepted and verified.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run the Step 2 command. Expected: zero failures.

- [ ] **Step 5: Commit Task 1**

```powershell
git add packages/omx_navigation/config/slam_toolbox_localization_hanyang_9f.yaml packages/omx_navigation/launch/go1_posegraph_navigation.launch.py packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/test/test_posegraph_config.py packages/omx_navigation/test/test_posegraph_launch.py packages/omx_navigation/test/test_localization_supervisor.py
git commit -m "fix: match Humble slam toolbox pose contracts"
```

---

### Task 2: Derive continuous map pose from fresh TF without stationary deadlock

**Files:**
- Create: `packages/omx_navigation/omx_navigation/pose_tracking.py`
- Create: `packages/omx_navigation/test/test_pose_tracking.py`
- Modify: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Modify: `packages/omx_navigation/test/test_localization_supervisor.py`

**Interfaces:**
- Produces: `compose_pose(parent_to_mid: Pose2D, mid_to_child: Pose2D) -> Pose2D`.
- Supervisor stores independent receive times for `map -> camera_init` and `camera_init -> body_nav` and requires both after the current SLAM epoch.
- `/slam_localization/pose` sets a post-epoch handshake flag; it does not refresh every `0.50 s`.

- [ ] **Step 1: Write failing pure transform tests**

```python
def test_compose_pose_rotates_child_translation():
    result = compose_pose(Pose2D(1.0, 2.0, math.pi / 2), Pose2D(2.0, 0.0, -math.pi / 2))
    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(0.0)
```

Cover finite validation and yaw wrapping.

- [ ] **Step 2: Run the pure test and verify RED**

Run:

```powershell
$env:PYTHONPATH='packages/omx_navigation'
py -3 -m pytest -q -p no:cacheprovider packages/omx_navigation/test/test_pose_tracking.py
```

Expected: import failure because `pose_tracking.py` does not exist.

- [ ] **Step 3: Implement SE(2) composition**

```python
def compose_pose(parent_to_mid: Pose2D, mid_to_child: Pose2D) -> Pose2D:
    c, s = math.cos(parent_to_mid.yaw), math.sin(parent_to_mid.yaw)
    return Pose2D(
        parent_to_mid.x + c * mid_to_child.x - s * mid_to_child.y,
        parent_to_mid.y + s * mid_to_child.x + c * mid_to_child.y,
        wrap_yaw(parent_to_mid.yaw + mid_to_child.yaw),
    )
```

- [ ] **Step 4: Write supervisor RED tests for stationary verification and TF discontinuity**

The stationary test publishes exactly one post-epoch `/slam_localization/pose`, then keeps only scan, odometry, and both TF edges fresh for longer than `3.0 s`; it must reach READY. A second test injects a post-baseline `map -> camera_init` jump above `0.30 m` or `10 deg` and requires `LOST/TF_CONFLICT` with `ready=false`.

- [ ] **Step 5: Verify the supervisor tests fail for the old freshness model**

Run:

```powershell
$env:PYTHONPATH='packages/omx_navigation'
py -3 -m pytest -q -p no:cacheprovider packages/omx_navigation/test/test_localization_supervisor.py -k "stationary or map_tf_discontinuity"
```

Expected: stationary test cannot remain VERIFYING and discontinuity is not classified as TF conflict.

- [ ] **Step 6: Implement TF-backed pose tracking**

Parse finite planar transforms in `_on_tf`, store both edges and timestamps, compose the current base pose whenever either edge changes, and feed that pose to continuous `score_pose`. Reset handshake, edge baselines, jump metrics, and quality timestamp on every refined-pose publication. `_tf_fresh()` must require both edges after the epoch; `_slam_pose_fresh()` must require the one post-epoch pose handshake plus a current composed pose, not a periodically refreshed pose message.

- [ ] **Step 7: Run Task 2 tests and verify GREEN**

Run both Task 2 test files plus `test_scan_map_quality.py`. Expected: zero failures.

- [ ] **Step 8: Commit Task 2**

```powershell
git add packages/omx_navigation/omx_navigation/pose_tracking.py packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/test/test_pose_tracking.py packages/omx_navigation/test/test_localization_supervisor.py
git commit -m "fix: verify stationary localization from live transforms"
```

---

### Task 3: Fail closed on reinitialization and remove test false-greens

**Files:**
- Modify: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Modify: `packages/omx_navigation/test/test_localization_supervisor.py`
- Modify: `packages/omx_navigation/setup.py`
- Modify: `packages/omx_navigation/package.xml`
- Create: `.gitattributes`
- Modify mechanically: `migration/*.sh`

**Interfaces:**
- Any new `/initialpose` immediately revokes old READY; invalid frame, non-finite pose, or invalid quaternion ends in `LOST/POSE_OUTSIDE_MAP`.
- `colcon test --packages-select omx_navigation` must report a non-zero pytest test count.
- Every tracked `*.sh` checkout uses LF.

- [ ] **Step 1: Write invalid-reinitialization RED tests**

```python
def test_invalid_new_initialpose_revokes_existing_ready(supervisor_module):
    node = ready_supervisor(...)
    invalid = pose_message()
    invalid.pose.pose.orientation.w = float("nan")
    node._on_initial_pose(invalid)
    node._on_ready_heartbeat()
    assert node._machine.state is LocalizationState.LOST
    assert publisher(node, "~/ready").messages[-1].data is False
```

Change the async coarse-search test to use `ImmediateExecutor` or explicitly complete its fake future before reading the result; no wall-clock sleep is allowed.

- [ ] **Step 2: Run the focused test and verify RED**

Run `test_localization_supervisor.py`; expected invalid reinitialization keeps the old READY state before the fix.

- [ ] **Step 3: Implement immediate revocation**

At the start of every initialpose callback, clear old SLAM/TF/quality state and transition invalid inputs with `reject_initial_pose(now)`. Do not return from conversion errors without a state transition.

- [ ] **Step 4: Register pytest with ament_python**

Add `tests_require=["pytest"]` to `setup()` while retaining `<test_depend>python3-pytest</test_depend>` in `package.xml`. Validate in a fresh WSL workspace that colcon reports more than zero omx_navigation tests.

- [ ] **Step 5: Enforce shell LF and normalize the checkout**

Create:

```gitattributes
*.sh text eol=lf
```

Mechanically convert tracked `migration/*.sh` to LF, then run:

```powershell
git add --renormalize .gitattributes migration
git ls-files --eol migration/*.sh
```

Expected: every shell file reports `i/lf w/lf attr/text eol=lf`.

- [ ] **Step 6: Run Task 3 validation**

Run focused Windows pytest, WSL `bash -n` on checkout and `git show HEAD:path` blobs, and isolated WSL `colcon test`. Expected: deterministic pytest, valid Bash syntax, and non-zero omx_navigation test count.

- [ ] **Step 7: Commit Task 3**

```powershell
git add .gitattributes migration packages/omx_navigation/setup.py packages/omx_navigation/package.xml packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/test/test_localization_supervisor.py
git commit -m "test: enforce deployable Humble verification"
```

---

### Task 4: Align diagnostics, execute full verification, and publish

**Files:**
- Modify: `migration/verify_posegraph_navigation.sh`
- Modify: `migration/test_posegraph_scripts.py`
- Modify: `packages/omx_navigation/README.md`
- Modify: `README.md`
- Modify: `docs/superpowers/plans/2026-08-12-robust-posegraph-localization.md`
- Modify: `.superpowers/sdd/2026-08-12-robust-posegraph-localization/task-5-report.md`

**Interfaces:**
- Diagnostics and rosbag use `/slam_localization/pose`.
- Preflight verifies saved graph artifacts, no AMCL node, both TF edges, READY/NONE status, and `arm=false`.

- [ ] **Step 1: Write documentation/script contract tests first**

Update `migration/test_posegraph_scripts.py` and launch tests to reject `/slam_toolbox/pose`, require `/slam_localization/pose`, and require the explicit posegraph start parameter.

- [ ] **Step 2: Run contract tests and verify RED**

Run migration and posegraph launch/config tests. Expected: stale script/document topic references fail.

- [ ] **Step 3: Update operational files**

Replace stale topic references, document pose handshake versus TF heartbeat, and explain that the `[0,0,0]` startup pose loads the graph but never opens the gate. Preserve all `arm:=false` instructions.

- [ ] **Step 4: Run fresh Windows verification**

```powershell
$env:PYTHONPATH=((Resolve-Path 'packages\go1_driver').Path + ';' + (Resolve-Path 'packages\omx_navigation').Path)
py -3 -m pytest -q -rs -p no:cacheprovider packages/go1_driver/test packages/omx_navigation/test migration/test_existing_map_scripts.py migration/test_end_to_end_workflow.py migration/test_posegraph_scripts.py
py -3 -m compileall -q packages/go1_driver/go1_driver packages/omx_navigation/omx_navigation packages/omx_navigation/launch
git diff --check origin/main...HEAD
```

Expected: zero failures; skips must be explicitly environment-dependent.

- [ ] **Step 5: Run fresh isolated WSL Humble verification**

Build both packages in `/tmp/go1-posegraph-*`, run colcon tests and direct Python 3.10 pytest, verify a non-zero omx_navigation colcon test count, run installed launch `--show-args`, compile/import the four ROS entry modules, and run both shell scripts through `bash -n`.

- [ ] **Step 6: Request independent origin/main-to-HEAD review**

Fix every Critical and Important finding, then repeat Steps 4–5. Minor findings must be documented if intentionally deferred.

- [ ] **Step 7: Commit verification updates**

```powershell
git add migration packages/omx_navigation README.md docs/superpowers/plans/2026-08-12-robust-posegraph-localization.md .superpowers/sdd/2026-08-12-robust-posegraph-localization/task-5-report.md
git commit -m "docs: align posegraph runtime verification"
```

- [ ] **Step 8: Push the verified branch**

```powershell
git push -u origin codex/verified-posegraph-navigation
```

Do not create a PR unless separately requested. Report the remote branch URL, exact commits, Windows/WSL results, and remaining Jetson-only limitations.
