# Field session log — 2026-09-18

Jetson AGX Orin + Go1 + Livox MID-360 (unit `.135`), Hanyang 9F map. First
armed drive with Nav2 goals from RViz: session
`posegraph_20260918T114201Z_154779`, 1084 s.

Every number below was measured from that session's rosbag, status CSV and
console log. The fixes were validated by replaying the same recordings
through the new code (`tools/replay_localization.py`,
`tools/session_report.py`) and, closed loop with the real slam_toolbox and
Nav2, through the launched stack (section 6); the robot itself has not run
them yet — see the last section.

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
laptop: under Python 3.10, pytest 6.2.5 and real ROS Humble (WSL) even the
09-18 code passes as one suite, both from the checkout and from an installed
workspace. What differs on the Jetson is the login shell: `~/.bashrc` sources
another project's `~/nav_ws` and exports `ROS_DOMAIN_ID=84`, and every Go1
script inherited both. `jetson_field_deploy.sh`, the verifier and
`run_all_tests.sh` now clear the inherited workspace paths before sourcing
ROS, and the deploy script and verifier always use the Go1 domain
(`GO1_ROS_DOMAIN_ID`, default 100), saying so when the calling shell had
another. `migration/run_all_tests.sh` is the one command that shows whether
the failures remain.

## 6. Closed loop on the laptop, with the real slam_toolbox

`migration/replay_field_bag.py` feeds a session's raw inputs -- `/scan`,
`/Odometry`, FAST-LIO's `camera_init -> body` and the operator's clicks,
re-stamped to now -- into the launched stack, so slam_toolbox, the
supervisor, Nav2 and the driver (dry-run) run closed loop on field data.
`migration/verify_control_chain_sim.py` drives the real driver and goal
bridge against a simulated robot and Nav2 action server (22 checks: remote
override, execution fault, re-arm on zero, preemption, arrival).

Replaying the reference run this way found a lock failure the robot had
only survived by luck:

- **slam_toolbox's answer was rejected half the time.** slam_toolbox applies
  a pushed pose to the next scan it *receives*, and that scan was usually
  acquired before the push (LiDAR to `/scan` to its TF filter takes
  50-100 ms), so the answer carries an earlier stamp. The supervisor
  required the answer's stamp to follow the push. Parked, slam_toolbox sends
  no other pose (`minimum_travel_distance`), so the supervisor waited for
  nothing, re-pushed, and went `LOST`/`ALIGNMENT_TIMEOUT` after five pushes
  with slam_toolbox correctly localized. On 09-18 the first click's answer
  happened to be stamped after the push (READY 6.1 s after the click: ~2.8 s
  of search on the Jetson, 3 s of verification); the second click's was
  not, and READY came only after the retry, 9.9 s after the click.
  **Fix**: an answer stamped before the push counts while that push is
  pending and only if it lands on the pushed pose (0.5 m / 10 deg); it
  completes the handshake but cannot end the correction window early, so a
  pose slam_toolbox matched just before it saw the push cannot turn the real
  correction into a `TF_CONFLICT`. Replayed: both clicks locked on their
  first push (READY 4.0 s and 3.9 s after the click; the laptop searches in
  ~1 s), no `TF_CONFLICT`, overlap 0.83 on average while READY, consistency
  gap at most 0.032.
- **A lifecycle bring-up can lose a response and hang.** Under load Fast DDS
  sometimes drops map_server's `change_state` reply ("failed to send response
  ... client will not receive response"); the lifecycle manager then waits
  forever, `/map` stays in the topic list but is never published, and every
  click ends in `ALIGNMENT_TIMEOUT`. `verify_posegraph_navigation.sh
  preflight` now requires `/map_server` to be active and says to relaunch.
- **`INPUT_MISSING` did not say what was missing.** Both failures above, and
  the normal second before slam_toolbox answers, showed the same
  `ALIGNING/INPUT_MISSING`. Status and CSV now carry `missing_inputs`, e.g.
  `map,alignment` (map never arrived), `slam_answer,tf` (pushed, waiting for
  slam_toolbox), empty when nothing is. (It also shows the field recording's
  own two 2.3 s `/scan` dropouts at t+38 and t+44, before the first click.)

With those fixed, the first 700 s of the session replayed closed loop:
READY without a break from the second lock to the end, no `TF_CONFLICT`,
`POSE_DRIFT` or `DEGRADED`; four drift corrections pushed at the corridor
end and on the way back, each accepted by slam_toolbox, and their transform
jumps recognised as the supervisor's own. Scan-to-map overlap while READY,
same bag windows:

| bag window | 09-18 on the robot (old code) | replayed (new code) |
|---|---|---|
| t+80-300, lock to corridor end | mean 0.85, min 0.73 | mean 0.85, min 0.72 |
| t+300-400, corridor end | mean 0.64, min 0.48, 88% below 0.7 | mean 0.79, min 0.61, 11% below 0.7 |
| t+400-700, driven back | mean 0.63, min 0.45, 99% below 0.7 | mean 0.87, min 0.62, 2% below 0.7 |

The good minutes are unchanged; the five minutes the robot spent READY on a
drifted pose are gone. Consistency gap while READY: median 0.027, p90 0.087.

Goals clicked into the same replay reach Nav2 once each: `/goal_pose` has
the bridge (and the recorder) as its only subscribers, three clicks give
three `Begin navigating`, a goal is `CANCELED: localization not ready` when a
new initial pose resets localization, and a new click `PREEMPTED` the
active goal.

## Not verified on the robot yet

Run these first next session, in this order:

1. `bash migration/run_all_tests.sh` on the Jetson -- the whole suite in one run,
   now without the login shell's `~/nav_ws` on the path. The deploy script
   prints a NOTE when it overrides the shell's `ROS_DOMAIN_ID=84`.
2. `jetson_field_deploy.sh stage/build/preflight`; preflight now refuses a
   workspace whose installed files differ from src. After launch,
   `verify_posegraph_navigation.sh preflight` must show `/map_server` active
   before the first click.
3. Dry-run: `/go1/robot_state` reports `robot: null`; one RViz click produces
   `SENT`/`ACCEPTED` once on `/navigation/goal_status` and a yellow marker.
4. Armed, robot standing, remote in hand: move a stick. Expect
   `/go1/manual_override` true, the goal `CANCELED: manual override`, stand
   held. Release: nothing moves until a new goal. **This is the scenario of
   09-18.**
5. One click, robot parked: READY about 6 s later (search, then 3 s of
   verification) without `TF_CONFLICT`, one `LocalizePoseCallback` in
   slam_toolbox's log (a second one is a retry), `missing_inputs` empty.
6. A goal and arrival: green `ARRIVED` marker.
7. Drive to the corridor end and back; watch `consistency_gap` and
   `drift_corrections` in the status CSV. If a correction misbehaves,
   relaunch with `drift_auto_correct:=false`.
8. `python3 tools/session_report.py <session>/rosbag/rosbag_0.db3`.

Still unknown: whether the HighState binding returns `Recv()`'s byte count
(the link check falls back to reply fingerprints if not) and the Go1's
behaviour with the remote on but centred.
