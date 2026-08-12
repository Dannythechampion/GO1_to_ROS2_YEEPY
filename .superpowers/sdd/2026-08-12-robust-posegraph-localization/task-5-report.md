# Task 5 보고서: Localization supervisor와 goal 취소

## 구현

- ROS 메시지 비의존 변환 모듈에서 유한 범위 LaserScan 점 변환·균등 샘플링, quaternion yaw 정규화, occupancy map cell 수 검증을 구현했습니다.
- `LocalizationSupervisor`는 `/map`, `/scan`, `/Odometry`, `/initialpose`, `/slam_toolbox/pose`, `/tf`를 구독하고, 보정 자세는 `/slam_localization/initialpose`로만 발행합니다. 따라서 사용자의 `/initialpose` 입력이 SLAM 출력과 다시 연결되지 않습니다.
- 신선한 map/scan에서 bounded coarse search를 수행하며, LOW_OVERLAP·AMBIGUOUS·입력 누락·AMCL TF 충돌·3.0 m/s 초과/시간 역행 odom reset을 상태 머신에 전달합니다.
- 2 Hz 상태 JSON(`state`, `error`, `message_ko`, `attempt`, `overlap`, `ambiguity_margin`, `stamp`)과 ready를 발행하고, CSV 진단 행은 표준 CSV escaping 후 매 행 flush합니다. 종료 시 CSV handle을 닫습니다.
- RViz goal bridge는 `/localization_supervisor/ready` 전에는 goal을 거부하고, ready가 false로 바��면 이미 수락된 goal을 취소합니다. send-goal 응답과 ready false가 경합할 때에도 응답 뒤 취소하도록 처리했습니다.
- 패키지 entry point 및 `nav_msgs`, `tf2_msgs` 런타임 의존성을 추가했습니다.

## TDD와 검증

- RED: 새 conversion/goal gate 테스트는 모듈 부재로 import collection error를 확인했습니다.
- RED: supervisor와 readiness-aware bridge 테스트는 새 모듈/구독 부재로 실패를 확인했습니다.
- GREEN focused: `py -3 -m pytest test/test_ros_conversions.py test/test_goal_gate.py test/test_localization_supervisor.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 11 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 114 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## 유의사항

- Windows 환경에는 ROS 2 runtime이 없으므로 실제 ROS graph/SLAM Toolbox와 action server의 런타임 통합은 WSL ROS 2 Humble 또는 Jetson에서 별도 확인이 필요합니다.

## Fix 1 안전 보완

- `/map`은 transient-local/reliable depth 1로 구독하여 유효한 map-server snapshot을 보존합니다. 정적 map_server가 사용자 클릭 뒤 map을 다시 발행하지 않는 정상 동작이므로, map 재발행을 초기화 선행 조건으로 삼지 않았습니다. 대신 구조적으로 잘못된 최신 map은 snapshot과 수신 시각을 모두 제거합니다.
- `/scan`은 sensor-data QoS이며 각 `/initialpose` 세대 뒤의 새 scan이 들어올 때만 한 번 search worker에 제출합니다. scan은 0.50초 신선도 조건을 유지하고, 변환 실패 시 data와 timestamp를 모두 제거합니다. worker 결과는 2 Hz timer가 generation token을 확인한 후 적용하므로 ROS callback을 막지 않습니다.
- 보정 자세 발행 시 SLAM epoch/baseline을 재설정합니다. epoch 이후의 fresh `/slam_toolbox/pose` 첫 샘플이 VERIFYING을 시작하고, 이후 연속 pose의 위치·yaw jump를 state machine에 전달합니다. 관련 `camera_init -> body` 또는 `camera_init -> body_nav` TF가 0.50초 이내에 없거나 SLAM pose가 없으면 READY가 될 수 없습니다.
- Odom reset은 message header source timestamp가 양수·유한하면 이를 사용하고, zero/invalid stamp는 receive time으로 fallback합니다. source time 역행·0 이하 dt·3.0 m/s 초과는 현재 initial-pose attempt에 latch하며 새 initial pose에서 초기화합니다.
- RViz bridge는 `wait_for_server(0.0)`만 사용하고 pending/active 중복 goal을 거부합니다. generation token으로 stale send response/result/cancel callback이 최신 handle을 지우지 못하게 했습니다.

### Fix 1 검증

- RED: 클릭 전 scan 재사용, 기본 QoS, 중복 in-flight goal을 재현하는 tests가 기존 구현에서 실패했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 12 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 120 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.
