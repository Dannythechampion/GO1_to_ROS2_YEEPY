# Field session log — 2026-10-25

Jetson AGX Orin + Go1 + Livox MID-360 (unit `.145`, SN `47MCN7J0032745`),
Hanyang 9F map. First session on `main` since the 2026-09-18 remediation was
pushed, and the first armed drive with that code.

Mechanical change since 09-18: the MID-360 was remounted 13 cm higher on a
3D-printed module, and a robot arm was fitted in front of it.

## Outcome

The deployment, the sensors, localization and the goal pipeline all worked.
**The robot moved, but never reached either of the two goals**: it hunted
around them instead of settling. Then localization collapsed and the
slam_toolbox node died.

Two defects were found. One has a fix in this commit; the other is a design
question that is still open.

## 1. Deployment

`main` @ `c7dbb98` was cloned to `/mnt/t500/go1_main_20261025` and staged into
`/mnt/t500/go1_ros2_ws` with `jetson_field_deploy.sh stage`. All 70 package
files hash-match `main`. The deployment it replaced was still the pre-09-18
code — 24 files differing and 12 missing, including `arbitration.py`,
`execution_monitor.py`, `robot_state.py` and `drift_monitor.py`. The previous
tree is preserved at `/mnt/t500/go1_ws_backup_20261025`.

`build` passes (`go1_driver` 63, `omx_navigation` 286, 0 failures) once pytest
is pinned to `>=7,<8` on the Jetson: `anyio` 4.14.2 in the user site needs
`_pytest.scope` (pytest 7+), while ROS Humble's `launch_testing` plugin fails
hook validation on pytest 8+. Both `preflight` and `dry-run` pass all four
gates.

## 2. The MID-360 had a 160 deg azimuth FOV mask

The LiDAR returned nothing over 160 deg of azimuth. This was **not** occlusion
from the new mount or the arm: the gap is present in raw `/livox/lidar`, the
cut is a hard azimuth edge with the sensor's full elevation span intact right
up to it (at -96 deg elevation still spans -2.6..+47.0; at -100 deg there are
zero points at any elevation), and a physical occluder clips low elevations
first and returns points from itself.

The unit had an FOV configuration persisted in its own non-volatile memory,
independent of `MID360_config.json`:

```
fov_cfg0   yaw   0..97     pitch -7..52
fov_cfg1   yaw 263..359    pitch -7..52
fov_cfg_en 3               (both windows active)
```

Cleared with `DisableLivoxLidarFov`, which zeroes `fov_cfg_en` and leaves the
window definitions stored, so `EnableLivoxLidarFov(handle, 3)` restores the old
behaviour exactly. Backup at
`/mnt/t500/go1_ws_backup_20261025/mid360_145_fov_backup_20261025.txt`.

| | masked | cleared |
|---|---|---|
| azimuth with zero returns | 160 deg | 0 deg |
| non-zero points per message | 73,914 / 199,872 | 164,716 / 200,064 |
| max range | 12.1 m | 19.7 m |
| elevation span | -7.0..+49.8 | -9.2..+55.1 |
| `/scan` valid beams | 45.1% | **94.0%** |

## 3. The 13 cm raise needed no configuration change

`extrinsic_T` is the LiDAR's internal laser-to-IMU offset and does not change
with mounting height, and `extrinsic_est_en` estimates it online anyway. No
static sensor-to-base transform exists in the stack: `planarize_transform`
forces `body_nav` to z = 0 and that frame is Nav2's `robot_base_frame`. The
only place the raise could matter is the `mid360_scan.yaml` height window, and
the measured margin after locking (0.171 against a required 0.05) left no case
for moving it. `mid360_scan.yaml` is unchanged.

## 4. The armed drive

Session `posegraph_20261025T102846Z_151370`, 436.5 s, two goals, neither
reached.

| t | event |
|---|---|
| 0–129.5 s | `WAITING_INPUT` / `initial_pose` |
| 129.5–135 | `ALIGNING` → `VERIFYING` (overlap 0.800) → `READY`, attempt 1, margin 0.171 |
| 135–340 | `READY`, the driving window (205 s) |
| 340.0→340.5 | collapse in 0.5 s: overlap 0.694 → 0.309 → 0.000, `consistency_gap` 0.0189 → 0.3080, `LOST`/`TF_CONFLICT` |
| 410.0 | `localization_slam_toolbox_node` died, exit code **-11** |
| 340.5–436 | `LOST` for 96 s, values frozen, never recovered |

READY-window statistics (411 samples):

```
overlap            min 0.694  mean 0.861  max 0.950    13.6% below 0.80
consistency_gap    min 0.000  mean 0.009  max 0.041    0 samples above the 0.08 threshold
offset radius      min 0.000  mean 0.084  max 0.340 m  8.0% above the 0.20 m goal tolerance
|offset yaw|       max 2.0 deg
drift_corrections  0          tf_corrections_explained 0
```

Excursions past 0.20 m: t+279 (0.214), t+289 (0.224), **t+293.5 (0.340)**,
t+324 (0.206). Every offset is a multiple of 0.025 m — half the 0.05 m map
resolution — so these are discrete scan-match jumps, not smooth drift. The
0.340 m excursion is about seven map cells.

## 5. Why it hunted, and why `drift_corrections` is 0

The pose the controller used disagreed with the live scan by up to 0.34 m while
`xy_goal_tolerance` is 0.20 m. An arrival tolerance below the localization
uncertainty cannot be satisfied reliably: the goal checker alternates between
inside and outside, and the controller keeps re-approaching.

`drift_corrections = 0` is **correct behaviour, not a bug**.
`DriftPolicy.gap_threshold` is 0.08 and `consistency_gap` never exceeded 0.041,
so no correction was ever due. The open problem is that the quantity gating a
correction (`consistency_gap`) does not track the quantity that decides arrival
(offset radius against `xy_goal_tolerance`): at the 0.340 m excursion the gap
was 0.0166. A radius-based trigger would have fired at all four excursions.
That change is **not** in this commit — it needs a threshold justified by
measurement, and a drive to validate it.

Yaw was never the failing condition: the localization yaw disagreement peaked
at 2.0 deg against an 8.6 deg tolerance.

## 6. A rotation dead zone in the final alignment

`SimpleGoalChecker` runs with `stateful: true`, so position is latched once
satisfied and only yaw is checked afterwards — the "arrive, then turn" split is
already the configured behaviour. But the turn itself can be swallowed:

```
DWB      max_vel_theta 0.40,  RotateToGoal.slowing_factor 8.0  -> ~0.05 rad/s near the goal
driver   yaw_deadband 0.04    -> abs(yaw) < 0.04 becomes exactly 0.0, and with
                                 zero linear speed the command becomes STAND
```

`vtheta_samples` is 20 over roughly [-0.05, +0.05], so most sampled yaw rates
fall inside the driver's deadband and produce no motion at all. Nav2 believes it
is rotating; the robot stands. Nothing then translates either, and
`SimpleProgressChecker` requires 0.15 m within 20 s — which is the
`Failed to make progress` signature from 2026-09-18.

Added payload and floor friction make this worse rather than cause it: they
raise the yaw rate needed to break stiction, widening the effective deadband.

This commit does not change the deadband, the slowing factor or the progress
checker. Picking those numbers needs a measured minimum yaw rate for the robot
as it is now loaded.

## 7. slam_toolbox died twice — fixed here

Both deaths were `exit code -11` (SIGSEGV):

- session `092530Z`, seconds after deserializing the posegraph. FAST-LIO had
  diverged and `/scan` carried 0 valid beams, which explained that one.
- session `102846Z`, 410 s into the armed drive, with `/scan` at 94% valid
  beams. Nothing explained that one.

`slam_toolbox_localization_hanyang_9f.yaml` did not set `stack_size_to_use`,
so the node ran on the default 8 MB stack while matching against a
deserialized posegraph. The mapping config already carried 40 MB, and
slam_toolbox's own `mapper_params_localization.yaml` ships the same value with
the comment "program needs a larger stack size to serialize large maps". The
launch's `RewrittenYaml` rewrites only `map_file_name`, `odom_frame`,
`base_frame` and `scan_topic`, so nothing supplied it. **Set in this commit,
with a test.** Not yet confirmed on the robot.

## 8. Evidence that was lost

The rosbag is truncated and unreadable: its SQLite header claims 9032 pages and
the file holds 8924, because the robot was powered off while rosbag2 was still
writing and `metadata.yaml` was never produced. Node stdout went to the
volatile systemd user journal and vanished when the Jetson rebooted; only
`.ros/log/<stamp>/launch.log` survived, which is where the segfault line was
found.

So this session has no `cmd_vel`, `cmd_vel_nav`, `Odometry` or
`/navigation/goal_status` trace. Sections 5 and 6 are characterised from the
supervisor CSV and from configuration; the hunting is consistent with the
numbers but was not confirmed against commanded-versus-executed data.

Before the next armed run: make the journal persistent or send node output to a
file on `/mnt/t500`, and stop the stack with `systemctl --user stop` before
cutting power so the bag finalises.

## 9. Also verified

- The 09-18 arrival-reporting failure is fixed and live: `bt_navigator` has 0
  subscriptions on `/goal_pose` (remapped to `/bt_navigator/goal_pose_disabled`),
  leaving `rviz_goal_bridge` as the only consumer, and
  `/navigation/goal_status` and `/navigation/goal_marker` are published.
- The armed driver reads the SDK reply it previously ignored:
  `ARMED mode=0 ... | robot mode=0 v=(-0.00,0.00) yaw_rate=-0.00 remote=off`.
  `/go1/manual_override` carries the remote state as a topic. Execution
  monitoring is armed-only; dry-run logs `execution monitoring off`.
