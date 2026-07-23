# ROS2 Humble 기존 지도 기반 Go1 내비게이션 설계

## 1. 목표

Jetson의 ROS2 Humble 환경에서 Livox MID-360, FAST-LIO2, 기존 2D 지도,
AMCL, Nav2 및 RViz를 하나의 안전한 시험 구성으로 연결한다.

이번 단계의 완료 범위는 다음과 같다.

- 실제 Go1을 움직이지 않고 센서부터 Nav2 경로 생성까지 검증한다.
- 기존 `scans_new.pgm`과 `scans_new.yaml`을 먼저 사용한다.
- RViz에서 지도, LaserScan, TF, 로봇 위치, 전역·지역 경로와 costmap을 확인한다.
- ROS1 Noetic의 DWA 경로 추종 튜닝 의도를 ROS2 DWB에 이식한다.
- Jetson 부하를 줄이기 위해 센서와 제어 주기를 맞추고 RViz 표시를 최소화한다.
- 모든 시험에서 Go1 드라이버는 기본 `arm=false`로 유지한다.

실제 SLAM 지도 작성과 실제 Go1 주행은 이 설계의 범위 밖이다.

## 2. 확정된 제약

- LiDAR 장착 위치가 확정되지 않았으므로 영구적인
  `base_link -> lidar` 고정 TF와 최종 FAST-LIO extrinsic은 만들지 않는다.
- 현재 FAST-LIO가 제공하는 `camera_init -> body`와
  `/cloud_registered_body`를 임시 시험 기준으로 사용한다.
- 기존 ROS1 지도의 공개 저장소 커밋은 피하고 Jetson 로컬 지도 경로를
  launch 인자로 전달한다.
- 기존 ROS1 작업공간과 충돌하지 않도록 기본 `ROS_DOMAIN_ID=100`으로 실행한다.
- `/cmd_vel`은 계산할 수 있지만 하드웨어 전송은 `arm=false`로 차단한다.

## 3. 런타임 구조

```text
MID-360
  -> livox_ros_driver2 (/livox/lidar, /livox/imu)
  -> FAST-LIO2 (camera_init -> body, /cloud_registered_body)
  -> pointcloud_to_laserscan (/scan, frame=body)
  -> 기존 2D 지도 + AMCL (map -> camera_init)
  -> Nav2 planner/controller/behavior/velocity_smoother
  -> /cmd_vel
  -> go1_driver (arm=false)

RViz:
  Map + LaserScan + TF + RobotModel + Global/Local Plan + Costmaps
```

Nav2 프레임은 초기 시험 동안 다음과 같이 둔다.

- `global_frame`: `map`
- `odom_frame`: `camera_init`
- `robot_base_frame`: `body`
- LaserScan frame: `body`

LiDAR가 고정된 뒤에는 `body` 또는 `base_link` 기준의 실제 정적 TF와
고정 extrinsic으로 교체해야 한다.

## 4. ROS1 DWA 튜닝의 ROS2 이식

ROS1에서 실제로 수정했던 파일은 다섯 YAML 중
`base_local_planner_params.yaml` 하나였다. 다른 네 YAML의 기존 값은
분석만 했으며 직접 변경하지 않았다.

기존 핵심 값과 ROS2 대응은 다음과 같다.

| ROS1 DWA 값 | ROS2 Nav2/DWB 대응 |
| --- | --- |
| `xy_goal_tolerance: 0.20` | `general_goal_checker.xy_goal_tolerance: 0.20` |
| `yaw_goal_tolerance: 0.15` | `general_goal_checker.yaw_goal_tolerance: 0.15` |
| `sim_time: 2.0` | `FollowPath.sim_time: 2.0` |
| `vx_samples: 10` | `FollowPath.vx_samples: 10` |
| `vy_samples: 1` | `FollowPath.vy_samples: 1` |
| `vth_samples: 20` | `FollowPath.vtheta_samples: 20` |
| `path_distance_bias: 40.0` | `PathAlign.scale`, `PathDist.scale` |
| `goal_distance_bias: 20.0` | `GoalAlign.scale`, `GoalDist.scale` |
| `occdist_scale: 0.01` | `BaseObstacle.scale` |
| `oscillation_reset_dist: 0.20` | `Oscillation.oscillation_reset_dist: 0.20` |
| `acc_lim_x: 0.50` | DWB와 velocity smoother의 `acc_lim_x` |
| `acc_lim_theta: 1.00` | DWB와 velocity smoother의 `acc_lim_theta` |

ROS1의 최고속도 `0.30 m/s`, 최고 회전속도 `0.60 rad/s`는 현재
Go1 드라이버 상한인 `0.20 m/s`, `0.40 rad/s`보다 크다. Nav2가 실행
불가능한 속도를 계획하면 추종 오차와 목표 부근 진동이 커지므로 ROS2
구성은 드라이버 상한과 같거나 작은 값으로 제한한다.

추가 흔들림 억제 항목은 다음과 같다.

- `controller_frequency: 10 Hz`: 10 Hz LaserScan과 맞추고 CPU 부하를 낮춘다.
- stateful goal checker로 위치 도달 후 방향 정렬 중 재이탈을 줄인다.
- progress checker 허용 시간을 보수적으로 설정해 정지 시험의 오탐을 줄인다.
- `velocity_smoother`의 속도·가속도 상한을 DWB 및 Go1 드라이버와 일치시킨다.
- RotateToGoal의 감속과 lookahead를 명시한다.
- 경로 및 회전 샘플 수를 제한해 Jetson 부하를 줄인다.

## 5. 구성 파일

다음 산출물을 `omx_navigation`에 추가하거나 갱신한다.

- 기존 지도 로컬라이제이션용 Nav2 파라미터
- pointcloud-to-laserscan 저부하 파라미터
- 기존 지도 통합 launch
- RViz 저부하 설정
- 지도 파일을 Jetson 로컬 경로에 준비하는 스크립트
- `arm=false` 무구동 검증 스크립트 및 실행 문서

지도 경로, RViz 사용 여부, `ROS_DOMAIN_ID`, Go1 드라이버 사용 여부는
launch 인자로 변경할 수 있게 한다. 기본값은 안전한 무구동 시험이다.

## 6. 오류 처리와 안전 조건

통합 launch는 다음 조건에서 명확히 실패하거나 경고해야 한다.

- 지도 YAML 또는 PGM이 없음
- 필수 ROS2 패키지가 없음
- `/scan`, `/Odometry` 또는 필수 TF가 일정 시간 안에 생성되지 않음
- Nav2 lifecycle 노드가 active 상태에 도달하지 못함
- Go1 드라이버가 예기치 않게 `arm=true`로 시작됨

검증 중에는 Nav2 goal을 실제 로봇에 전달하지 않는다. 경로 생성 시험 후
goal을 취소하고 `/cmd_vel`이 0으로 복귀했는지 확인한다.

## 7. 성능 기준

- FAST-LIO 및 `/scan`: 약 10 Hz 유지
- Nav2 controller: 10 Hz
- RViz point cloud 원본 표시 비활성
- LaserScan 표시 점 수와 history 최소화
- costmap, TF, plan 위주로 표시
- FAST-LIO 자체 RViz 및 PCD 저장 비활성

## 8. 검증 기준

다음 조건이 모두 만족되면 “SLAM 전 단계 완료”로 판단한다.

1. 관련 패키지가 ROS2 Humble에서 빌드된다.
2. Livox, FAST-LIO, LaserScan 토픽이 지속 발행된다.
3. TF 체인 `map -> camera_init -> body`가 조회된다.
4. 기존 지도와 AMCL이 active 상태가 된다.
5. Nav2 lifecycle 노드가 모두 active 상태가 된다.
6. RViz에서 지도와 scan을 동시에 확인할 수 있다.
7. RViz 초기 자세 지정 후 AMCL pose가 갱신된다.
8. 목표 지정 시 전역·지역 경로와 제한 범위 내 `/cmd_vel`이 생성된다.
9. Go1 드라이버는 계속 `arm=false`이며 실제 로봇은 움직이지 않는다.
10. 시험 종료 또는 goal 취소 후 `/cmd_vel`이 0으로 복귀한다.

기존 지도와 실시간 scan이 물리적으로 맞지 않으면 소프트웨어 결함으로
간주하지 않고 지도 재작성 후보로 기록한다. LiDAR 고정 후 새 SLAM과
extrinsic 보정은 다음 단계에서 수행한다.
