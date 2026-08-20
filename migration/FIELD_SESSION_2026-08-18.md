# Field session log — 2026-08-18

Jetson AGX Orin + Go1 + Livox MID-360, on the Hanyang 9F map. Goal: reach a
verified `READY` localization and perform a first short test drive.

Every number below was measured on the robot.

## Outcome

Reached and held `READY` localization, armed the driver through all preflight
gates, and verified the full command path end to end. **The robot did not
execute velocity commands.** That is still open — see the last section.

## 1. FAST-LIO stopped producing odometry

`/Odometry` and `/scan` were dead, `map -> camera_init` had been frozen for 11
minutes, supervisor was `LOST` with `overlap 0.0`.

Root cause was an IMU/lidar QoS asymmetry inside FAST-LIO — full analysis in
[FAST_LIO_IMU_QOS_FIX.md](FAST_LIO_IMU_QOS_FIX.md). A restart recovered it
immediately (`imu_margin` -1.25 -> +0.089, `drops` 9077 -> 0), and the QoS fix
made it survive a 67 ms CPU spike where 28 ms used to break it permanently.

## 2. RViz was showing the wrong config

`jetson_field_deploy.sh dry-run` hardcodes `rviz:=false`, so RViz had been
started separately with the stock `nav2_default_view.rviz`, whose frames do not
match this stack (`body_nav`). Restarted with
`go1_existing_map_low_load.rviz`.

## 3. Localization never reached READY in the corridor

`overlap` was fine (0.97+) but `ambiguity_margin` was 0.021 against a required
0.05, so the guard rejected every initial pose.

Scoring all 1469 candidates with the supervisor's own matcher showed why:

```
rank  score   overlap  x       y       yaw
   1  0.9626  0.9722   1.367  -1.322  -99.7
   2  0.9429  0.9556   1.367  -1.822  -99.7
   3  0.9415  0.9556   1.867   1.178  -99.7   <- runner-up, 2.55 m away
```

Every top candidate shared the same yaw. Heading was unambiguous; position
along the corridor was not — the score was nearly flat across a 5 m stretch,
because at a 0.5 m step almost every scan point stays within the 0.25 m hit
tolerance of a wall running parallel to the slide direction.

Fix: narrow `coarse_search_translation_radius` from 3.0 m to 1.0 m, which
excludes the competing candidate instead of loosening the ambiguity guard. A
click farther than the radius now fails closed rather than locking onto a wrong
pose. Result: margin 0.115, `READY`.

The pose was then validated by real motion — the operator drove the robot with
the RC and localization tracked it and *improved* to `overlap 0.9889`. A wrong
pose would have diverged.

## 4. Duplicate nodes from earlier launches

Three generations of `go1_driver`, `planar_base_frame`, and
`cmd_vel_safety_gate` were running simultaneously, three `planar_base_frame`
instances publishing the same TF. This is what produced the supervisor's
`TF_CONFLICT`. An earlier cleanup pass had missed these names.

When killing a launch, its child nodes are orphaned rather than terminated —
they must be cleaned up explicitly before relaunching.

## 5. Armed and verified

```
PASS: Unitree robot_interface import and constructors
PASS: Jetson static preflight (ROS_DOMAIN_ID=100)
PASS: live MID-360, FAST-LIO, odometry, and body TF inputs
[go1_driver] Go1 driver started in ARMED mode; listening on /cmd_vel
```

Localization held `READY` for 60 s, `overlap` 0.956-0.994, sub-cm jitter while
stationary, `map`/`odom` displacement agreeing to 0.000 m.

The goal gate also proved itself: a navigation goal set while localization was
not ready was refused with `Ignoring navigation goal until localization is
ready`, and `/cmd_vel` stayed at zero.

## Still open: the robot does not walk

Commands reach the SDK with correct values and the robot answers, but it does
not move.

Verified working:

- network to `192.168.123.161`: 30/30 packets, 0% loss, 0.77 ms
- driver transmitting: `Send-Q` 0, 349 packets in 3 s (100 Hz as configured)
- command values: `mode=2 gaitType=1 velocity=[0.1, 0]`
- port config identical to the vendor `example_walk.py`:
  `UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)`
- `HighState` returns live data: `bodyHeight 0.301`,
  `footForce [134, 253, 140, 257]`, `imu.rpy [-0.005, -0.024, -0.058]`
- RC control works, so the robot's sport mode is healthy

Two hypotheses were tested and disproved:

1. **Network loss.** There was a real intermittent drop (ARP `INCOMPLETE`,
   `Send-Q` growing to 154 KB, only 71 TX packets in 5 s). After the link was
   restored and verified clean, the robot still did not move.
2. **Mode transition.** The driver held `mode=1` (force stand) and switched
   straight to `mode=2`, while the vendor example always passes through
   `mode=0` (idle stand). Changed to `mode=0`; the robot still did not move.
   The change was kept because it matches the vendor reference.

The remaining suspect is another controller on the robot holding priority — the
robot reported `mode=1` continuously even while being commanded `mode=0` at
100 Hz for 5 s. The Go1 Nano boards (`192.168.123.13/14/15`) run onboard
programs that can command the sport controller. Next step is to inspect what is
running on the robot's Pi and whether an AI/vision mode is active.
