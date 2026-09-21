# Field session log — 2026-09-18

Jetson AGX Orin + Go1 + Livox MID-360 (unit `.135`), Hanyang 9F map. First
armed drive with Nav2 goals from RViz: session
`posegraph_20260918T114201Z_154779`, 1084 s.

Every number below was measured from that session's rosbag, status CSV and
console log. The fixes were validated by replaying the same recordings
through the new code (`tools/replay_localization.py`,
`tools/session_report.py`); the robot itself has not run them yet — see the
last section.

## Outcome

Localization locked on the first push and the robot followed Nav2 at 85-103%
for three minutes. Then Nav2 reported `Failed to make progress` three times.
A same-day review concluded that "Nav2 commanded rotation but no forward
motion". **The recording shows the opposite:** Nav2 commanded continuously;
the robot did not execute.

## 1. The aborts were a manual-override collision

In the 20 s before each abort Nav2 commanded steady, non-oscillating turns
(zero yaw sign flips); odometry shows what the robot did:

| abort | commanded turn | executed | commanded forward | net motion |
|---|---|---|---|---|
| t+390 | -401.5 deg | 5.4 deg (1%) | 0.000 m | 0.039 m |
| t+459 | +339.3 deg | 11.0 deg (3%) | 0.265 m | 0.029 m |
| t+480 | -74.9 deg | 0.1 deg (0%) | 0.000 m | 0.019 m |

Between t+398 and t+438 the robot moved 9.3 m at up to 0.92 m/s -- 4.5 times
the driver's 0.20 m/s limit -- with 0.20 m commanded. The operator confirmed
driving it back to the start with the handheld remote. The sport controller
follows the remote over any HighCmd, so Nav2 spent minutes commanding a robot
that was not listening, and nothing in the stack could tell: the driver called
`GetRecv` every cycle and never read the reply, where `wirelessRemote` is.

The review was misled by the driver log, which printed a line only when the
command *reason* changed: 103 lines in 1084 s. Its three `vx=0.000` lines
were samples taken at reason edges, not a 20 s command trace.

Replaying the driver's new commanded-vs-executed check on the recording also
found a 12 s stall inside the "good" minutes (t+238-250, 0 deg turned while
0.19-0.32 rad/s was commanded) that the per-minute follow ratio hid.

**Fixes** (`go1_driver`): HighState is decoded every cycle and published on
`/go1/robot_state`; a pressed key or deflected stick is manual override
(`/go1/manual_override`) and holds stand; a refusal lasting 5 s or motion
without a command lasting 1 s is an execution fault (`/go1/execution_fault`);
after either, nothing moves until `/cmd_vel` has been zero for 0.5 s, so a
stale goal cannot resume when the remote is released; the log records
requested vs applied every second while anything moves. Replayed on this
session the check flags the first abort stall at t+361, 27 s before Nav2 did,
and raises nothing during the minutes of good following.

## 2. Every RViz click reached Nav2 twice

Humble's `bt_navigator` subscribes to `goal_pose` itself; the launch left that
in place next to `rviz_goal_bridge`:

```
971.847 [rviz_goal_bridge] Ignoring navigation goal while another goal is pending or active
971.851 [bt_navigator]     Received goal preemption request
971.852 [bt_navigator]     Begin navigating from (0.74,-3.42) to (1.21,-4.62)
971.857 [rviz_goal_bridge] Navigation finished with action status 6
```

The bridge refused the click and Nav2 ran it anyway. The bridge then lost
the goal it was tracking, so `Navigation goal reached` never appeared in the
whole run, the operator had no arrival signal, clicked again, and the robot
turned back toward a stale target. The readiness gate on goals stopped
nothing (the velocity gate still did).

**Fixes**: `goal_pose` is remapped away from `bt_navigator` in both navigation
launches; the bridge preempts on a new click instead of ignoring it, cancels
on localization loss, manual override or execution fault, and publishes every
transition on `/navigation/goal_status` plus a coloured marker on
`/navigation/goal_marker` (shown in the field RViz config): `ARRIVED`,
`IGNORED: <reason>`, `CANCELED: <reason>`, `PREEMPTED`.

## 3. TF_CONFLICT was almost always our own correction

8 of 14 sessions that day and on 08-18 latched `TF_CONFLICT`. On 09-18 every
case was one of two kinds, and neither was a second publisher:

- **At startup, before any pose was set**: slam_toolbox's first match from
  `map_start_pose` jumps 20+ degrees (sessions 89895, 106202, 139807).
- **2.3-2.7 s after a pose push**: slam_toolbox publishes its old transform
  with fresh stamps until it applies the push, so the baseline taken after the
  reset was the pre-correction transform and the correction (1.76 m / 123 deg)
  looked like a conflict. The reference run's first push only escaped by the
  race going the other way -- the habitual second push was unnecessary.

With healthy FAST-LIO, slam_toolbox's corrections while tracking were p99
1.5-8 cm and at most 9 cm / 0.4 deg, so the 0.30 m / 10 deg limit itself is
sound. The corrected transform reproduces slam_toolbox's reported pose
exactly (0.000 m), and slam_toolbox settles up to 0.34 m / 3.4 deg from the
pushed pose because the coarse search works on a 0.5 m grid.

**Fix** (`localization_supervisor`): no jump tracking before the first push;
after a push, a jump that lands within 0.5 m / 10 deg of the pushed pose is
the correction arriving; once the transform agrees with slam_toolbox's pose,
that allowance ends, so a discontinuity shortly after lock still fails closed
(the existing test for it is unchanged). The status text no longer names AMCL.

## 4. `ambiguity_margin` was a frozen lock-time value

2027 of 2168 samples carried the identical margin 0.06202301204373306 while
overlap moved between 0.45 and 0.91: it is computed once, at lock. Simply
recomputing it would be wrong: replaying the lock search at the tracked pose,
the margin sat under the 0.05 guard for most of the minutes in which Nav2 was
driving well -- a pose one metre along the corridor always scores almost as
well.

What the frozen value hid is drift. At the far end of the corridor the
tracked pose picked up a heading error of 3-4.5 deg (plus ~0.25 m lateral)
that slam_toolbox never corrected, because it processes no scan while the
robot stands still; overlap sank from 0.86 to about 0.65 and stayed there
while the supervisor reported READY. The map is not the cause: annotation
changed 0 occupied cells.

**Fix**: a bounded local refinement around the tracked pose every 2 s while
tracking (`refine_pose_locally`, ~46 score evaluations). Its score gap was a
median 0.015 (p90 0.032, max 0.056) while tracking was good and a median 0.18
once drifted. Three consecutive checks above 0.08 whose implied map->odom
headings agree within 2 deg trigger a correction pushed to slam_toolbox (the
per-component median; along a corridor only heading and lateral position are
observable). Replayed, every one of the 35 corrections this policy would
have issued made the following 20 s of scans match the map better, none
worse (parked: 0.71 -> 0.85). Drift that corrections do not clear becomes
`POSE_DRIFT`. `drift_auto_correct:=false` turns correction off. Status and
CSV now carry `consistency_gap`, drift offset and correction counts;
`ambiguity_margin` is documented as the lock-time value it is.

## 5. The test suite could not be run as a suite on the Jetson

Per file green, 22 failures together, plus a user-site `anyio` pytest plugin
that crashes the system pytest. Stubs installed by one test module leaked
into the next. Each package's `test/conftest.py` now drops every project
module and stub first imported during a test; `test_module_isolation.py`
fails without it. The exact Jetson failure could not be reproduced on the
laptop (Python 3.14, pytest 9, no real ROS); `migration/run_all_tests.sh` is
the one command that shows it on the Jetson.

## Not verified on the robot yet

Run these first next session, in this order:

1. `bash migration/run_all_tests.sh` on the Jetson -- the whole suite in one run.
2. `jetson_field_deploy.sh stage/build/preflight`; preflight now refuses a
   workspace whose installed files differ from src.
3. Dry-run: `/go1/robot_state` reports `robot: null`; one RViz click produces
   `SENT`/`ACCEPTED` once on `/navigation/goal_status` and a yellow marker.
4. Armed, robot standing, remote in hand: move a stick. Expect
   `/go1/manual_override` true, the goal `CANCELED: manual override`, stand
   held. Release: nothing moves until a new goal. **This is the scenario of
   09-18.**
5. One `/initialpose` push: READY without `TF_CONFLICT`; `tf_corrections_explained` 1.
6. A goal and arrival: green `ARRIVED` marker.
7. Drive to the corridor end and back; watch `consistency_gap` and
   `drift_corrections` in the status CSV. If a correction misbehaves,
   relaunch with `drift_auto_correct:=false`.
8. `python3 tools/session_report.py <session>/rosbag/rosbag_0.db3`.

Still unknown: whether the HighState binding returns `Recv()`'s byte count
(the link check falls back to reply fingerprints if not) and the Go1's
behaviour with the remote on but centred.
