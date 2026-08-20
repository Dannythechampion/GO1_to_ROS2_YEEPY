# FAST-LIO IMU QoS fix

This change lives in the `FAST_LIO_ROS2` checkout on the Jetson
(`/mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2`), which is a separate repository from
this one. It is recorded here so it is not lost and can be reapplied after a
clean checkout or a Jetson rebuild.

Field session: 2026-08-18, Jetson AGX Orin + Go1 + MID-360.

## Symptom

FAST-LIO ran normally for a few tens of seconds, then stopped publishing
`/Odometry` and `/cloud_registered_body` permanently. Because
`pointcloud_to_laserscan` feeds on `/cloud_registered_body`, `/scan` died too,
slam_toolbox stopped publishing `map -> camera_init`, and the localization
supervisor fell to `LOST` with `overlap 0.0`.

The node stayed alive and its diagnostics kept printing:

```
[fast_lio_realtime] queue_depth=2 front_age=0.243 drops=9077 process_ms=0.005 imu_margin=-1.248
```

`imu_margin = last_timestamp_imu - lidar_end_time`. FAST-LIO waits for IMU
samples to cover the end of the lidar frame before processing it, so a
permanently negative margin means **no frame is ever processed** and every one
is dropped.

## Why it was not the sensor

Measured directly on `/livox/lidar` and `/livox/imu`:

- lidar frame span 0.0992-0.1011 s (exactly 10 Hz, correct)
- `timebase` matched the header stamp
- IMU 200 Hz, and IMU stamps were *ahead* of lidar frame end by +0.005 to +0.24 s
- no clock drift: the offset measured 45 s apart did not grow

So the raw streams were healthy. Only FAST-LIO's internal `last_timestamp_imu`
lagged, by about 1.1 s, and never recovered.

## Root cause

The lidar and IMU subscriptions used different QoS:

```cpp
// laserMapping.cpp — lidar: best effort, depth 1, drops when late
sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    lid_topic, rclcpp::SensorDataQoS().keep_last(1), livox_pcl_cbk);

// laserMapping.cpp — IMU: default RELIABLE, depth 10
sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
```

A RELIABLE subscription does not drop late samples, it delivers them in order,
late. At 200 Hz, once a CPU spike pushes the consumer behind, it only catches up
if it consumes faster than it produces — and it consumes at exactly the arrival
rate. So a one-off stall becomes a permanent latency offset.

The trigger was reproducible: `max_process_ms` jumped from 11.6 ms to 28.4 ms
(nav2 stack startup, 14 nodes at once) and `imu_margin` flipped from +0.087 to
-1.009 in the same tick, then never recovered.

## The change

```diff
--- a/src/laserMapping.cpp
+++ b/src/laserMapping.cpp
@@
-        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);
+        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
+            imu_topic, rclcpp::SensorDataQoS().keep_last(200), imu_cbk);
```

`SensorDataQoS()` is BEST_EFFORT, so the middleware stops applying in-order
back-pressure. `keep_last(200)` is 1 s at 200 Hz, enough IMU continuity to
integrate across the 10 Hz lidar frames.

Rebuild with:

```bash
source /opt/ros/humble/setup.bash
cd /mnt/t500/go1_ros2_ws
colcon build --packages-select fast_lio
```

## Verification

Confirmed in effect: `ros2 topic info -v /livox/imu` reports the subscription as
`Reliability: BEST_EFFORT` (previously RELIABLE).

Confirmed under load, not just at idle. The armed nav2 launch was used as the
stress test, since it is the same 14-node startup that broke it before:

| | before fix | after fix |
|---|---|---|
| CPU spike (`max_process_ms`) | 28.4 ms | 67.2 ms |
| `imu_margin` after the spike | -1.009 (permanent) | +0.085 |
| `drops` | climbing 10/s | 0 |

A larger spike than the one that used to break it left the node healthy.

## Operational note

Order still matters when bringing the system up, because the fix bounds the
damage rather than removing the load spike: start Livox, then the nav2 stack,
then FAST-LIO last, and confirm `/Odometry` stays near the origin while the
robot is stationary before trusting it.
