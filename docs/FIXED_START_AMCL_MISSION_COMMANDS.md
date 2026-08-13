# 고정 출발점 AMCL 및 Mission 실행 명령어

이 문서는 GO1을 동일한 바닥 타일에서 출발시켜 AMCL로 초기 위치를 검증하고, 등록 목적지 또는 RViz 임의 목적지로 주행시키는 데 필요한 명령을 정리한다.

## 구현 상태

| 기능 | 상태 |
|---|---|
| 지도 제작, 기존 지도 Nav2 실행, RViz 수동 초기 위치 및 goal | 현재 브랜치에서 실행 가능 |
| AMCL 자동 초기화 supervisor, motion gate, mission manager, pose recorder | 구현 완료, 현장 좌표 commissioning 전 |

ROS 2 노드와 서비스는 구현되어 있다. 실제 자동 초기화와 등록 목적지 mission은 현장에서 start/destination 좌표를 commissioning한 뒤 사용할 수 있다. 좌표 파일이 없으면 시스템은 의도적으로 fail-closed 상태가 된다.

## 1. 저장소 준비

PR 브랜치를 직접 확인할 때:

~~~bash
cd /mnt/t500
git clone https://github.com/Dannythechampion/GO1_to_ROS2_YEEPY.git go1_ros2_project
cd /mnt/t500/go1_ros2_project
git fetch origin
git switch --track origin/agent/amcl-mission-design-pr
~~~

이미 clone한 저장소에서 전환할 때:

~~~bash
cd /mnt/t500/go1_ros2_project
git fetch origin
git switch --track origin/agent/amcl-mission-design-pr
git pull --ff-only
~~~

PR이 main에 병합된 뒤:

~~~bash
cd /mnt/t500/go1_ros2_project
git switch main
git pull --ff-only origin main
~~~

공통 경로:

~~~bash
export GO1_PROJECT=/mnt/t500/go1_ros2_project
export GO1_ROS2_WS=/mnt/t500/go1_ros2_ws
export ROS_DOMAIN_ID=100
~~~

## 2. ROS 2 workspace 배치와 빌드

빈 workspace에 처음 배치할 때:

~~~bash
cd "$GO1_PROJECT"
GO1_ROS2_WS="$GO1_ROS2_WS" ./migration/stage_local_ros2_packages.sh
~~~

배치 스크립트는 기존 target을 덮어쓰지 않는다. 이미 배치된 workspace라면 백업하거나 새 workspace를 사용한다.

~~~bash
source /opt/ros/humble/setup.bash
cd "$GO1_ROS2_WS"
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select go1_driver omx_navigation
source "$GO1_ROS2_WS/install/setup.bash"
~~~

새 terminal마다:

~~~bash
source /opt/ros/humble/setup.bash
source /mnt/t500/go1_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=100
~~~

## 3. 테스트

ROS 2 환경:

~~~bash
cd "$GO1_ROS2_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select go1_driver omx_navigation --event-handlers console_direct+
colcon test-result --verbose
~~~

저장소 Python 테스트:

~~~bash
cd "$GO1_PROJECT"
PYTHONPATH="$GO1_PROJECT/packages/go1_driver:$GO1_PROJECT/packages/omx_navigation" \
  python3 -m pytest -q packages/omx_navigation/test packages/go1_driver/test
~~~

## 4. 현재 코드로 기존 지도 실행

~~~bash
export MAP_FILE=/mnt/t500/go1_ros2_project/maps/hanyang_9f/20260728_204825/slam_toolbox/hanyang_9f_cleaned.yaml
test -s "$MAP_FILE"
~~~

안전한 disarm 상태:

~~~bash
ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  start_go1_driver:=true \
  rviz:=true \
  arm:=false
~~~

현재 구현에서 실제 driver 명령을 허용할 때:

~~~bash
ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  start_go1_driver:=true \
  rviz:=true \
  arm:=true
~~~

현재 순서는 RViz 2D Pose Estimate, AMCL overlay 확인, RViz 2D Goal Pose 순이다.

점검 명령:

~~~bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic hz /scan
ros2 topic hz /Odometry
ros2 topic hz /amcl_pose
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo camera_init body
ros2 action info /navigate_to_pose
ros2 lifecycle nodes
~~~

## 5. 현재 mapping 명령

~~~bash
ros2 launch omx_navigation go1_mapping.launch.py \
  start_go1_driver:=false \
  rviz:=true
~~~

~~~bash
cd "$GO1_PROJECT"
GO1_ROS2_WS="$GO1_ROS2_WS" ./migration/verify_mapping_pipeline.sh
~~~

~~~bash
mkdir -p /mnt/t500/maps/floor9_v001
cd "$GO1_PROJECT"
GO1_ROS2_WS="$GO1_ROS2_WS" \
  ./migration/save_nav2_map.sh /mnt/t500/maps/floor9_v001/go1_map
~~~

## 6. 통합 기능 runtime 준비

좌표를 기록하기 전 runtime 디렉터리만 준비한다.

~~~bash
sudo mkdir -p /mnt/t500/go1_runtime
sudo chown "$(id -u):$(id -g)" /mnt/t500/go1_runtime
chmod 750 /mnt/t500/go1_runtime
export START_POSE_FILE=/mnt/t500/go1_runtime/start_pose.yaml
export DESTINATION_POSE_FILE=/mnt/t500/go1_runtime/destination_pose.yaml
~~~

## 7. 출발 위치 commissioning

앞발을 지정한 타일 기준에 맞추고 완전히 정지시킨다. E-stop을 누르고 gate를 disarm 상태로 유지한다.

~~~bash
ros2 run omx_navigation commission_start_pose --ros-args \
  -p output_path:="$START_POSE_FILE" \
  -p sample_count:=10 \
  -p overwrite:=false
~~~

~~~bash
ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  start_pose_file:="" \
  initial_pose_arm:=false \
  destination_pose_file:="" \
  arm:=false \
  rviz:=true
~~~

RViz 2D Pose Estimate로 AMCL을 수렴시킨 뒤:

~~~bash
test -s "$START_POSE_FILE"
sed -n '1,120p' "$START_POSE_FILE"
cp -a "$START_POSE_FILE" "$START_POSE_FILE.$(date +%Y%m%d_%H%M%S).bak"
~~~

## 8. 목적지 commissioning

E-stop과 gate disarm을 유지한다.

~~~bash
ros2 run omx_navigation record_destination_pose --ros-args \
  -p output_path:="$DESTINATION_POSE_FILE" \
  -p map_topic:=/map \
  -p goal_topic:=/goal_pose \
  -p minimum_clearance:=0.35 \
  -p overwrite:=false
~~~

RViz 2D Goal Pose로 엘리베이터 앞 위치와 방향을 지정한 뒤:

~~~bash
test -s "$DESTINATION_POSE_FILE"
sed -n '1,120p' "$DESTINATION_POSE_FILE"
cp -a "$DESTINATION_POSE_FILE" "$DESTINATION_POSE_FILE.$(date +%Y%m%d_%H%M%S).bak"
~~~

## 9. 통합 시스템 실행

처음에는 driver와 gate가 정지 상태가 되도록 arm false로 시작한다.

~~~bash
ros2 launch omx_navigation go1_existing_map.launch.py \
  map:="$MAP_FILE" \
  start_pose_file:="$START_POSE_FILE" \
  initial_pose_arm:=true \
  destination_pose_file:="$DESTINATION_POSE_FILE" \
  start_go1_driver:=true \
  arm:=false \
  rviz:=true
~~~

Localization 확인:

~~~bash
ros2 topic echo --once /localization/status
ros2 topic echo --once /localization/ready
ros2 topic hz /localization/ready
ros2 topic echo --once /amcl_pose
~~~

localization ready가 true가 아니면 gate arm이나 mission을 실행하지 않는다.

## 10. Motion gate

E-stop을 해제하고 주변을 확인한 뒤:

~~~bash
ros2 service call /motion_gate/arm std_srvs/srv/Trigger "{}"
ros2 topic echo --once /motion_gate/status
ros2 topic echo --once /motion_gate/enabled
~~~

주행 권한 제거:

~~~bash
ros2 service call /motion_gate/disarm std_srvs/srv/Trigger "{}"
~~~

Gate disarm과 action cancel을 모두 실행할 때:

~~~bash
ros2 service call /motion_gate/disarm std_srvs/srv/Trigger "{}"
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
~~~

## 11. 등록 목적지 mission

~~~bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
~~~

상태 확인과 취소:

~~~bash
ros2 topic echo --once /mission/status
ros2 topic echo --once /mission/active
ros2 action info /navigate_to_pose
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
~~~

중단 후에는 새 start 명령이 필요하며 자동 재출발은 없다.

## 12. RViz 임의 목적지

~~~bash
ros2 topic echo --once /localization/ready
ros2 topic echo --once /motion_gate/enabled
ros2 topic echo --once /mission/active
~~~

Localization과 gate가 true이고 active mission이 없을 때 RViz 2D Goal Pose를 사용한다.

~~~bash
ros2 topic echo --once /goal_pose
ros2 topic echo --once /mission/status
~~~

## 13. 안전 경로 감시

~~~bash
ros2 topic hz /cmd_vel_nav
ros2 topic hz /cmd_vel_safe
ros2 topic echo /cmd_vel_safe
ros2 topic hz /localization/ready
ros2 topic hz /motion_gate/enabled
ros2 topic hz /mission/active
ros2 topic info /cmd_vel_safe --verbose
ros2 topic info /cmd_vel_nav --verbose
ros2 topic info /goal_pose --verbose
~~~

기대 경로:

~~~text
Nav2 controller -> /cmd_vel_nav -> motion_gate -> /cmd_vel_safe -> go1_driver
RViz /goal_pose -> fixed_mission_manager -> NavigateToPose
~~~

## 14. 문제 발생 시

예상과 다르게 움직이면 terminal 명령보다 물리 E-stop을 우선한다.

~~~bash
ros2 service call /motion_gate/disarm std_srvs/srv/Trigger "{}"
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
ros2 topic echo --once /localization/status
ros2 topic echo --once /motion_gate/status
ros2 topic echo --once /mission/status
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
~~~

## 15. 구현 후 회귀 검증

~~~bash
cd "$GO1_PROJECT"
PYTHONPATH="$GO1_PROJECT/packages/go1_driver:$GO1_PROJECT/packages/omx_navigation" \
  python3 -m pytest -q packages/omx_navigation/test packages/go1_driver/test
~~~

~~~bash
cd "$GO1_ROS2_WS"
colcon build --symlink-install --packages-select go1_driver omx_navigation
source install/setup.bash
colcon test --packages-select go1_driver omx_navigation --event-handlers console_direct+
colcon test-result --verbose
~~~

직접 우회 연결 검사:

~~~bash
cd "$GO1_PROJECT"
rg -n "rviz_goal_bridge|/cmd_vel_safe|/cmd_vel_nav|NavigateToPose|/goal_pose" \
  packages/omx_navigation packages/go1_driver
~~~

## 16. 운영 종료

~~~bash
ros2 service call /motion_gate/disarm std_srvs/srv/Trigger "{}"
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
~~~

각 launch terminal에서 Ctrl+C로 종료한다. 운영 중 생성한 start/destination 파일은 저장소에 commit하지 않는다.
