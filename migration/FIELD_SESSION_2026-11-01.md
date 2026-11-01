# Field session log — 2026-11-01

Jetson AGX Orin + Go1 + Livox MID-360 (unit `.135`), Hanyang 9F map. First
session of the taped start/goal mission: one vertical retroreflective strip on
each corridor wall at the start and at the goal, 35-85 cm above the floor.

Network: the laptop and the Jetson were on a phone hotspot, where the Jetson
took `172.20.10.12` (the `go1jetson` alias points at the home address). RViz
ran on the laptop in WSL with `networkingMode=mirrored`, which lets DDS reach
the Jetson both ways.

## Outcome

Start tape recorded; three armed teach runs toward the goal tape; **the goal
was never reached and is still unrecorded.**

The decisive finding came from the third run: **localization lagged the
robot's real progress along the corridor by 4.5 m after 36 s, while the
supervisor reported READY the whole time.** FAST-LIO and the Go1's own leg
odometry agreed within 2%; the map pose moved at about 45% of the real speed.
The robot reached the corner while the stack believed it was still 1.5-2 m
short, so Nav2 turned the wrong way and spun in place. The operator stopped it.

## 1. Sensors and the tape, measured

- MID-360 at `.135` again (config said `.145`; fixed in `src/` and `install/`).
  This unit has **no** device FOV mask: 0 empty 5 deg azimuth bins, elevation
  -7.6..52.6 deg.
- The tape returns **reflectivity 150-164 median, never above ~185**; painted
  walls beside it return 20-34. The detector's original 180 threshold saw
  nothing. Threshold is now **120**: strips keep 99% of their points (~100-155
  per 0.2 s window in band), non-tape returns within 2 m fall to ~3 per frame.
  The pair was found 10 of 10 windows with the midpoint steady to 1 mm.
- The strips are seen from z -0.10 to +0.33 m in the LiDAR frame with the top
  at 85 cm above the floor, so **the LiDAR sits 0.52 m above the floor**.
- Start tape recorded at map (1.608, -1.796), heading -98 deg, walls 2.029 m
  apart.

## 2. Defects found and fixed during the session

All on `main`, deployed to `/mnt/t500/go1_ros2_ws` and tested there.

| commit | defect | evidence |
|---|---|---|
| 6510ddd | tape threshold 180 detected nothing | strips max 175-185 |
| fd56ff6 | `tape_waypoint check` failed on ceiling lights | clusters at z 1.4-2.1 m |
| 2856289 | Go1 link always "down": SDK `Recv()` never returns a positive count | IMU and foot forces kept changing; this hid the remote override and `/go1/imu` |
| 5b1f19c | `tape_waypoint init` lost its `/initialpose` to discovery | supervisor never logged it |
| launch arg | lock-time coarse search ±1.0 m reads the corridor as ambiguous | margin 0.003-0.025 vs 0.05; with ±0.3 m and a tape-seeded pose, 0.35-0.39 |
| fa6dfcb | `tape_mission` deserialised every Livox frame in rclpy: 124-189 ms each, 112% CPU idle | Jetson at load 12 on 12 cores; now 1.3 ms with a numpy CDR view, identical output |
| fa6dfcb | "navigation failed, tape in view" rescue docked onto the **start** tape 8 m from the goal | rescue now needs `tape_near_goal` |
| 05dd829 | one READY lapse cancelled a whole leg | resume after READY holds 1 s, ≤3 per leg |
| 05dd829 | the session bag duplicated the launch bag at another 45% CPU | off unless `GO1_SESSION_BAG=1` |

Also found, not code: **FAST-LIO degrades with run time.** After ~45 min its
worst cycle had grown 58 → 634 ms and `/Odometry` arrived at ~5 Hz with gaps up
to 800 ms, which with `input_max_age` 0.5 s drops READY. A restart (stack
stopped, Livox first) gave 9.5 Hz with a 118 ms worst gap. Restart FAST-LIO
before every armed run.

## 3. The three runs

| | run 1 | run 2 | run 3 |
|---|---|---|---|
| session | `tape_20261101T120542Z` | `tape_20261101T124740Z` | `tape_20261101T130614Z` |
| start | tape-seeded | 0.47 m past the start tape | between the start tapes |
| LiDAR mount | loose | loose | **taped down** |
| walked (legs / FAST-LIO) | — | 7.4 / 6.4 m | 7.9 / 8.0 m |
| map pose moved | — | 3.7 m | 3.5 m |
| localization | READY lapse 4 s out (FAST-LIO stall + CPU) | LOW_OVERLAP 0.42 → slam jump → TF_CONFLICT | **READY throughout** |
| ended by | bridge cancel, then a wrong-tape rescue (fixed) | Go1 link lost | spinning near the corner; operator stopped it |

Runs 1 and 2: the operator confirmed the LiDAR mount had come loose. The logs
agree — in the last 3 s of run 2 the MID-360's IMU read 2.1-2.5 rad/s of
roll/pitch rate while the Go1's trunk read 0.3, and taping the LiDAR down is
what made localization hold in run 3.

## 4. Run 3: the map pose did not follow the robot

`tools/compare_odometry.py` on the run-3 bag (window: teach start to +36 s,
before the spinning):

```
  t (s)   FAST-LIO   legs    map    map lags FAST-LIO by
    6.0     1.15    1.15    0.72    +0.43 m
   12.0     2.50    2.49    1.14    +1.36 m
   18.0     3.84    3.86    1.55    +2.30 m
   24.0     5.29    5.22    2.36    +2.92 m
   30.0     6.65    6.56    2.79    +3.86 m
  +36 s     7.99    7.86    3.51    +4.48 m
```

Run 2 shows the same pattern (lag 2.7-2.9 m by 37 s). The lag starts at zero
and grows steadily, so it is **accumulated while walking**, not a wrong start:
the tape-seeded start agreed with the start tape to 9 cm. During the whole
walk the supervisor stayed READY — consistency gap peaked once at 0.109,
overlap ≥ 0.58, zero drift corrections — because a corridor scan matches
almost equally well anywhere along the corridor, which is the same
observability problem as 2026-10-25, now measured directly.

What the operator saw follows from it: at the real corner the stack still
believed the corner was ahead, so the global path continued straight into the
end wall and the controller turned right and back, in place, instead of left.

Two explanations fit a steady 45% ratio, and the logs cannot separate them:

1. slam_toolbox's localization keeps snapping the pose back along the
   corridor (its correction absorbs the odometry), or
2. the map itself is short along this corridor — a degenerate corridor during
   the 2026-07-28 mapping run would have compressed it the same way.

**Measure the physical distance from the start tape to the corner** and
compare it with the map (start at y -1.8, the corner wall at about y -7.5 to
-8.3, i.e. ~6 m). If reality is ~8 m, the map is short and needs rebuilding.

## 5. What to change in the algorithm

In order of value:

1. **Guard map progress against odometry.** Compare map-frame displacement
   with FAST-LIO (and leg) displacement over a sliding window; a sustained lag
   beyond ~0.5 m means localization is not following the robot → stop, LOST.
   This is exactly what was invisible today, and the data to compute it is
   already in the supervisor. `tools/compare_odometry.py` is the offline
   version.
2. **Do not let corridor scan matching correct the along-corridor axis.**
   FAST-LIO and the legs agreed within 2% over 8 m; scan matching was off by
   55%. Between taped spots, hold `map → camera_init` along the corridor (or
   weight odometry far above the scan match) and re-anchor at every tape pair.
   An extra tape pair at the corner would bound the error before the turn.
3. **Check the map scale** (section 4) before tuning anything else.
4. **Fix `gait_stability`'s tilt estimate.** Gravity from the accelerometer is
   swamped by footfall impacts: a normal trot read tilt > 100 deg and every
   walking window UNSTABLE. Use the Go1 quaternion for the trunk and a
   complementary filter for the Livox IMU. Gyro RMS is sound.
5. Make the FAST-LIO restart part of the armed start-up (`to_armed.sh`).

## 6. Logs

On the Jetson: `/mnt/t500/go1_field_logs/tape_20261101T*` (decision logs,
stability CSVs, reports), `/mnt/t500/localization_logs/posegraph_20261101T*`
(resilient bags, split every 60 s), `/mnt/t500/go1_field_logs/stack_20261101/`
(Livox, FAST-LIO and stack console logs). Mirrored without the bags to the
laptop at `fielddata/20261101/`, with `runs_on_map.png` (runs 2 and 3 on the
map, as localization believed them).
