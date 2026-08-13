# Fixed-Start AMCL Navigation and Dual-Goal Mission Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Automatically seed and verify AMCL from a commissioned fixed-tile start, keep the GO1 stationary behind a standalone motion gate, and support both an explicit single-destination mission command and validated arbitrary RViz goals through one Nav2 action path.

**Architecture:** localization_supervisor owns AMCL seeding and readiness, motion_gate is the only velocity source for go1_driver, and fixed_mission_manager owns both fixed and RViz goal admission. Shared pose-file and occupancy-map geometry modules keep commissioning and runtime validation identical. Missing field-measured files remain explicit uncommissioned states; no guessed coordinates are committed.

**Tech Stack:** ROS 2 Humble, Python 3, rclpy, Nav2 NavigateToPose, AMCL, NavFn, DWB, tf2_ros, pytest, and the repository's existing launch source tests.

**Canonical design:** docs/superpowers/specs/2026-08-13-fixed-start-navigation-mission-design.md

**Supersedes:** docs/superpowers/plans/2026-08-13-fixed-start-amcl-safety.md

---

## Task 1: Shared Pose Configuration and Start Commissioning

**Files:**

- Create: packages/omx_navigation/omx_navigation/pose_config.py
- Create: packages/omx_navigation/omx_navigation/commission_start_pose.py
- Create: packages/omx_navigation/test/test_pose_config.py
- Create: packages/omx_navigation/test/test_commission_start_pose.py
- Modify: packages/omx_navigation/setup.py
- Modify: packages/omx_navigation/package.xml

### Step 1: Write failing pose-contract tests

Specify:

- load_planar_pose returns an immutable map-frame x, y, yaw value.
- Only frame_id map is accepted.
- Empty optional path returns UNCONFIGURED instead of origin.
- A configured but missing file raises a configuration error.
- Invalid YAML, schema, kind, missing fields, and non-finite values fail closed.
- Start and destination kinds cannot be exchanged.
- Atomic writer refuses overwrite unless explicitly enabled.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_pose_config.py
~~~

Expected: FAIL because the module does not exist.

### Step 2: Implement the minimal pose contract

Add PlanarPose, PoseFileState, PoseConfigurationError, angle normalization, loader, and atomic writer. Use schema_version, kind, frame_id, x, y, yaw, and optional metadata. Keep all runtime paths out of this pure module.

### Step 3: Write failing start-estimator tests

Cover:

- Ten samples required by default.
- Median x/y reject one outlier.
- Circular yaw mean handles the pi boundary.
- Moving, stale, out-of-order, high-covariance, or excessively spread samples are rejected.
- The recorder requires an explicit path and refuses overwrite.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_commission_start_pose.py
~~~

Expected: FAIL.

### Step 4: Implement estimator and node

Implement a pure StartPoseEstimator, then a thin ROS node subscribing to AMCL pose and odometry. Record kind start through pose_config.py. Register the console entry point and dependencies. Do not provide a default production output path.

### Step 5: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_pose_config.py packages/omx_navigation/test/test_commission_start_pose.py
git add packages/omx_navigation/omx_navigation/pose_config.py packages/omx_navigation/omx_navigation/commission_start_pose.py packages/omx_navigation/test/test_pose_config.py packages/omx_navigation/test/test_commission_start_pose.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: add pose commissioning contract"
~~~

Expected: PASS.

---

## Task 2: Shared Occupancy-Map Geometry

**Files:**

- Create: packages/omx_navigation/omx_navigation/map_geometry.py
- Create: packages/omx_navigation/test/test_map_geometry.py

### Step 1: Write failing geometry tests

Cover:

- Rotated map-origin world-to-cell conversion.
- Outside-map rejection.
- Occupied threshold 50 and unknown value -1.
- Distance to nearest occupied-or-unknown cell.
- Goal occupancy from 0 through 49 and minimum 0.35 m clearance.
- Quaternion finiteness, normalization, and planar yaw.
- Scan endpoint transform and invalid-beam filtering.
- Median and p80 residuals with at least 100 beams.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_map_geometry.py
~~~

Expected: FAIL.

### Step 2: Implement the reusable map model

Implement OccupancyMap, a precomputed obstacle distance field, validate_goal_pose, and score_scan_pose. Keep this module independent of rclpy.

### Step 3: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_map_geometry.py
git add packages/omx_navigation/omx_navigation/map_geometry.py packages/omx_navigation/test/test_map_geometry.py
git commit -m "feat: add shared map geometry validation"
~~~

Expected: PASS.

---

## Task 3: AMCL Localization Supervisor

**Files:**

- Create: packages/omx_navigation/omx_navigation/localization_state.py
- Create: packages/omx_navigation/omx_navigation/localization_supervisor.py
- Create: packages/omx_navigation/test/test_localization_state.py
- Create: packages/omx_navigation/test/test_localization_supervisor.py
- Modify: packages/omx_navigation/setup.py
- Modify: packages/omx_navigation/package.xml

### Step 1: Write failing state-machine tests

Cover:

- Empty start path plus arm false enters UNCOMMISSIONED and never seeds.
- Empty start path plus arm true and configured-invalid files are errors.
- A valid armed pose moves through waiting, seeding, and verifying.
- AMCL pose must arrive after the current seed.
- Initial pose delta must be within 0.25 m and 15 degrees.
- x/y variance 0.04 m², yaw variance 0.0305 rad², freshness 0.30 s, 100 beams, 10 scans, median 0.15 m, p80 0.30 m, and hold time 2.0 s all fail closed.
- After initial readiness, moving away from the start is allowed.
- Runtime criterion loss immediately enters DEGRADED.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_localization_state.py
~~~

Expected: FAIL.

### Step 2: Implement the pure state machine

Implement explicit states, timestamped input snapshots, initial-only checks, continuous checks, and reason-bearing transitions. Keep ROS messages outside the core.

### Step 3: Add restart-detection tests and implementation

Test frame-ID change, timestamp rollback, and an unexplained jump greater than 0.50 m or 20 degrees within 0.50 s. Confirm each immediately clears readiness and requests one controlled reseed cycle. Avoid false positives when commanded/odometry motion explains the change.

### Step 4: Write failing node-contract tests

Verify:

- A map-frame PoseWithCovarianceStamped is published once per seed cycle on /initialpose.
- Inputs include AMCL pose, scan, odometry, map, and timestamped TF.
- /localization/ready is a 10 Hz heartbeat.
- /localization/status is transient local.
- Scan scoring delegates to map_geometry.py.
- Error and shutdown publish false readiness.

### Step 5: Implement the ROS adapter

Keep callbacks thin: convert input, perform TF lookup, update the core, and publish heartbeats. Register localization_supervisor.

### Step 6: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_localization_state.py packages/omx_navigation/test/test_localization_supervisor.py
git add packages/omx_navigation/omx_navigation/localization_state.py packages/omx_navigation/omx_navigation/localization_supervisor.py packages/omx_navigation/test/test_localization_state.py packages/omx_navigation/test/test_localization_supervisor.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: add AMCL localization supervisor"
~~~

Expected: PASS.

---

## Task 4: Standalone Motion Gate

**Files:**

- Create: packages/omx_navigation/omx_navigation/motion_gate_core.py
- Create: packages/omx_navigation/omx_navigation/motion_gate.py
- Create: packages/omx_navigation/test/test_motion_gate_core.py
- Create: packages/omx_navigation/test/test_motion_gate_node.py
- Modify: packages/omx_navigation/setup.py
- Modify: packages/omx_navigation/package.xml

### Step 1: Write failing gate-core tests

Test default zero/disabled, localization age 0.30 s, explicit arm and released E-stop, command age 0.25 s, finite values, hard limits, immediate zero on any failure, and a specific block reason. Prove that enabled becomes true before any Nav2 command exists when localization, E-stop, arm, and configuration prerequisites pass; output must still be zero until a fresh valid command arrives.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_motion_gate_core.py
~~~

Expected: FAIL.

### Step 2: Implement the pure gate

Implement timestamped setters plus evaluate(now), returning safe twist, enabled, and reason. Calculate enabled from localization, E-stop, arm, and configuration health only. Apply command freshness and numeric/limit checks separately when producing safe twist. Do not import ROS.

### Step 3: Write node-contract tests and implement adapter

Verify /cmd_vel_nav input, /cmd_vel_safe output, 10 Hz /motion_gate/enabled heartbeat, status output, continuous zero while blocked, and fail-closed E-stop/arm defaults. Register motion_gate.

### Step 4: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_motion_gate_core.py packages/omx_navigation/test/test_motion_gate_node.py
git add packages/omx_navigation/omx_navigation/motion_gate_core.py packages/omx_navigation/omx_navigation/motion_gate.py packages/omx_navigation/test/test_motion_gate_core.py packages/omx_navigation/test/test_motion_gate_node.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git commit -m "feat: add fail-closed navigation motion gate"
~~~

Expected: PASS.

---

## Task 5: Mission Admission and State Core

**Files:**

- Create: packages/omx_navigation/omx_navigation/mission_state.py
- Create: packages/omx_navigation/test/test_mission_state.py

### Step 1: Write failing mission-state tests

Cover:

- Missing destination produces UNCONFIGURED_DESTINATION for fixed missions.
- A valid RViz goal remains acceptable without a destination file.
- Fixed start loads exactly one kind destination pose.
- Both sources require fresh true localization and gate heartbeats within 0.30 s.
- Both require action availability and shared map validation.
- SENDING, ACTIVE, and CANCELING reject all new goals.
- Goal source remains in status.
- Explicit cancel works only for pending/active work.
- Localization or gate loss requests cancel immediately.
- Action results map to SUCCEEDED, FAILED, and CANCELED.
- Terminal state persists until another goal is accepted.
- Recovery never re-sends an old goal.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_mission_state.py
~~~

Expected: FAIL.

### Step 2: Implement the pure state machine

Create state, source, and effect enums. Event methods return SEND_GOAL, CANCEL_GOAL, REJECT, or NONE so ROS action handles stay outside the core.

### Step 3: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_mission_state.py
git add packages/omx_navigation/omx_navigation/mission_state.py packages/omx_navigation/test/test_mission_state.py
git commit -m "feat: add dual-goal mission state machine"
~~~

Expected: PASS.

---

## Task 6: Mission Manager and Destination Recorder

**Files:**

- Create: packages/omx_navigation/omx_navigation/fixed_mission_manager.py
- Create: packages/omx_navigation/omx_navigation/record_destination_pose.py
- Create: packages/omx_navigation/test/test_fixed_mission_manager.py
- Create: packages/omx_navigation/test/test_record_destination_pose.py
- Modify: packages/omx_navigation/setup.py
- Modify: packages/omx_navigation/package.xml
- Delete: packages/omx_navigation/omx_navigation/rviz_goal_bridge.py

### Step 1: Write failing manager tests

Verify:

- /mission/start and /mission/cancel are Trigger services.
- /goal_pose is the RViz input.
- Exactly one NavigateToPose action client exists.
- Fixed and RViz inputs use one validation/send function.
- Rejections have a concrete reason.
- Action rejection, result, cancel, and exceptions map to correct states.
- Safety heartbeat loss cancels once.
- /mission/active is 10 Hz.
- /mission/status is transient local with state, reason, and source.
- Readiness becoming true never sends a mission automatically.
- A monotonically increasing mission ID prevents late callbacks from altering a newer mission.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_fixed_mission_manager.py
~~~

Expected: FAIL.

### Step 2: Implement the manager

Load the optional destination during configuration without substituting coordinates. Convert accepted poses to map-frame NavigateToPose goals. Serialize pending action operations to prevent duplicate sends.

### Step 3: Write recorder tests and implement it

Test explicit path, next valid /goal_pose capture, map frame, shared map validation, no write for invalid goals, overwrite refusal, and one-shot success. Register fixed_mission_manager and record_destination_pose.

### Step 4: Remove the bypass bridge

Delete rviz_goal_bridge.py and its setup entry point. Confirm no other operator goal component sends NavigateToPose.

### Step 5: Verify and commit

~~~bash
pytest -q packages/omx_navigation/test/test_fixed_mission_manager.py packages/omx_navigation/test/test_record_destination_pose.py
git add packages/omx_navigation/omx_navigation/fixed_mission_manager.py packages/omx_navigation/omx_navigation/record_destination_pose.py packages/omx_navigation/test/test_fixed_mission_manager.py packages/omx_navigation/test/test_record_destination_pose.py packages/omx_navigation/setup.py packages/omx_navigation/package.xml
git add -u packages/omx_navigation/omx_navigation/rviz_goal_bridge.py
git commit -m "feat: unify fixed and RViz navigation missions"
~~~

Expected: PASS and no bridge entry point remains.

---

## Task 7: Driver Velocity Boundary

**Files:**

- Modify: packages/go1_driver/go1_driver/node.py
- Modify: packages/go1_driver/go1_driver/command_filter.py
- Modify: packages/go1_driver/config/go1_driver.yaml
- Modify: packages/go1_driver/test/test_command_filter.py
- Create: packages/go1_driver/test/test_safe_command_input.py
- Modify: packages/go1_driver/README.md

### Step 1: Write failing driver tests

Verify the driver subscribes only to /cmd_vel_safe by default, never directly to planner topics; a command older than 0.35 s or containing non-finite data becomes zero; startup and shutdown stop; existing limits remain enforced.

Run:

~~~bash
pytest -q packages/go1_driver/test/test_command_filter.py packages/go1_driver/test/test_safe_command_input.py
~~~

Expected: at least one new test FAILS.

### Step 2: Implement input and watchdog

Use a monotonic receive time and 0.35-second default timeout. Preserve final finite-value and limit checks as defense in depth.

### Step 3: Verify and commit

~~~bash
pytest -q packages/go1_driver/test/test_command_filter.py packages/go1_driver/test/test_safe_command_input.py
git add packages/go1_driver/go1_driver/node.py packages/go1_driver/go1_driver/command_filter.py packages/go1_driver/config/go1_driver.yaml packages/go1_driver/test/test_command_filter.py packages/go1_driver/test/test_safe_command_input.py packages/go1_driver/README.md
git commit -m "feat: enforce safe velocity input watchdog"
~~~

Expected: PASS.

---

## Task 8: Launch and Parameter Integration

**Files:**

- Create: packages/omx_navigation/config/localization_supervisor.yaml
- Create: packages/omx_navigation/config/motion_gate.yaml
- Create: packages/omx_navigation/config/fixed_mission_manager.yaml
- Modify: packages/omx_navigation/config/nav2_existing_map_params.yaml
- Modify: packages/omx_navigation/launch/go1_existing_map.launch.py
- Modify: packages/omx_navigation/test/test_existing_map_launch.py
- Create: packages/omx_navigation/test/test_navigation_safety_wiring.py
- Modify: packages/omx_navigation/setup.py

### Step 1: Write failing launch tests

Verify:

- Launch arguments include start_pose_file, initial_pose_arm, and destination_pose_file.
- Both paths default empty and no coordinates are invented.
- Empty start plus arm true is rejected.
- Supervisor, gate, manager, and driver are launched.
- Controller output is /cmd_vel_nav.
- Gate output and sole driver input are /cmd_vel_safe.
- Manager consumes /goal_pose.
- rviz_goal_bridge is absent.
- AMCL is the only localization implementation.
- YAML values match every design threshold.

Run:

~~~bash
pytest -q packages/omx_navigation/test/test_existing_map_launch.py packages/omx_navigation/test/test_navigation_safety_wiring.py
~~~

Expected: FAIL.

### Step 2: Add parameter files and launch wiring

Keep E-stop/arm blocked by default. Start the supervisor false, gate disabled/zero, and manager even when destination is absent. Wire map and Nav2 dependencies without editing unrelated PCD-localization changes already present in the dirty tree.

### Step 3: Verify existing-map behavior

~~~bash
pytest -q packages/omx_navigation/test/test_existing_map_params.py packages/omx_navigation/test/test_nav2_humble_compatibility.py packages/omx_navigation/test/test_existing_map_launch.py packages/omx_navigation/test/test_navigation_safety_wiring.py
~~~

Expected: PASS and NavFn plus DWB remain selected.

### Step 4: Commit

~~~bash
git add packages/omx_navigation/config/localization_supervisor.yaml packages/omx_navigation/config/motion_gate.yaml packages/omx_navigation/config/fixed_mission_manager.yaml packages/omx_navigation/config/nav2_existing_map_params.yaml packages/omx_navigation/launch/go1_existing_map.launch.py packages/omx_navigation/test/test_existing_map_launch.py packages/omx_navigation/test/test_navigation_safety_wiring.py packages/omx_navigation/setup.py
git commit -m "feat: integrate safe AMCL mission launch"
~~~

---

## Task 9: Runtime Runbooks

**Files:**

- Create: docs/runbooks/commission-fixed-start.md
- Create: docs/runbooks/commission-destination.md
- Create: docs/runbooks/run-fixed-or-rviz-mission.md
- Create: docs/runbooks/verify-navigation-safety.md
- Modify: packages/omx_navigation/README.md
- Modify: README.md

### Step 1: Document start commissioning

Include front-foot tile alignment, full stop, E-stop/gate state, temporary RViz initial estimate, ten samples, explicit /mnt/t500/go1_runtime/start_pose.yaml output, backup, three cold starts, and re-commission triggers.

### Step 2: Document destination commissioning

Include disabled gate, RViz goal selection at the elevator-front waiting pose, shared validation, non-overwrite behavior, and explicit /mnt/t500/go1_runtime/destination_pose.yaml output.

### Step 3: Document normal operations

Include status inspection and the two service commands:

~~~bash
ros2 topic echo --once /localization/status
ros2 topic echo --once /motion_gate/status
ros2 topic echo --once /mission/status
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
~~~

Explain that RViz 2D Goal Pose is accepted only after localization and gate heartbeats are fresh and true.

### Step 4: Document robot verification

Record pass/fail evidence for absent start, invalid arm, absent destination, three cold starts, no pre-arm velocity, fixed goal, arbitrary goal, active-goal rejection, cancel, E-stop, AMCL/scan degradation, FAST-LIO restart, Nav2 failure, and no automatic resume.

### Step 5: Verify and commit docs

~~~bash
git add docs/runbooks/commission-fixed-start.md docs/runbooks/commission-destination.md docs/runbooks/run-fixed-or-rviz-mission.md docs/runbooks/verify-navigation-safety.md packages/omx_navigation/README.md README.md
git commit -m "docs: add fixed-start mission runbooks"
~~~

---

## Task 10: Final Automated and Field-Readiness Review

**Files:**

- Modify only files required by review findings.

### Step 1: Run the complete Python suite

~~~bash
pytest -q packages/omx_navigation/test packages/go1_driver/test
~~~

Expected: PASS. Record the exact count.

### Step 2: Build and test in ROS 2 Humble

When the ROS environment is available:

~~~bash
colcon build --symlink-install --packages-select omx_navigation go1_driver
colcon test --packages-select omx_navigation go1_driver --event-handlers console_direct+
colcon test-result --verbose
~~~

Expected: no failed results.

### Step 3: Inspect prohibited bypasses

~~~bash
rg -n "rviz_goal_bridge|/cmd_vel_safe|/cmd_vel_nav|NavigateToPose|/goal_pose" packages/omx_navigation packages/go1_driver
~~~

Confirm only the gate publishes /cmd_vel_safe, only the driver consumes it, only the manager consumes operator /goal_pose for navigation, and Nav2 velocity reaches /cmd_vel_nav.

### Step 4: Confirm no field coordinates are committed

~~~bash
git status --short
rg -n "start_pose.yaml|destination_pose.yaml|initial_pose_arm" . --glob "!docs/**" --glob "!build/**" --glob "!install/**" --glob "!log/**"
~~~

Confirm generated runtime pose files and measured x/y/yaw values are not staged.

### Step 5: Review the canonical invariants

Check:

- Distance-to-start is initial-only.
- Localization never arms motion.
- Readiness never starts or resumes a mission.
- Missing destination blocks fixed missions only.
- Fixed and RViz goals share one validation/action path.
- Every velocity command shares one gate/watchdog path.

Commit review fixes only if required.

### Step 6: Defer physical commissioning explicitly

Implementation can be complete while production pose files are absent. Operational commissioning is incomplete until the field team records and validates:

- /mnt/t500/go1_runtime/start_pose.yaml
- /mnt/t500/go1_runtime/destination_pose.yaml

Never describe unmeasured coordinates as verified.
