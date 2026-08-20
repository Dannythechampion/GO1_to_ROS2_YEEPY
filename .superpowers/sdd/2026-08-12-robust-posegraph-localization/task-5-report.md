# Task 5 보고서: Localization supervisor와 goal 취소

## 구현

- ROS 메시지 비의존 변환 모듈에서 유한 범위 LaserScan 점 변환·균등 샘플링, quaternion yaw 정규화, occupancy map cell 수 검증을 구현했습니다.
- `LocalizationSupervisor`는 `/map`, `/scan`, `/Odometry`, `/initialpose`, `/slam_localization/pose`, `/tf`를 구독하고, 보정 자세는 `/slam_localization/initialpose`로만 발행합니다. 따라서 사용자의 `/initialpose` 입력이 SLAM 출력과 다시 연결되지 않습니다.
- 신선한 map/scan에서 bounded coarse search를 수행하며, LOW_OVERLAP·AMBIGUOUS·입력 누락·AMCL TF 충돌·3.0 m/s 초과/시간 역행 odom reset을 상태 머신에 전달합니다.
- 상태 JSON(`state`, `error`, `message_ko`, `attempt`, `overlap`, `ambiguity_margin`, `stamp`)과 CSV 진단 행은 `2 Hz`로 발행·기록하고, ready는 최신 안전 상태를 평가한 뒤 `10 Hz` heartbeat로 발행합니다. CSV는 표준 escaping 후 매 행 flush하며 종료 시 handle을 닫습니다.
- RViz goal bridge는 `/localization_supervisor/ready` 전에는 goal을 거부하고, ready가 false로 바뀌면 이미 수락된 goal을 취소합니다. send-goal 응답과 ready false가 경합할 때에도 응답 뒤 취소하도록 처리했습니다.
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
- 보정 자세 발행 시 SLAM epoch/baseline을 재설정합니다. epoch 이후의 `/slam_localization/pose` 첫 샘플은 VERIFYING을 시작하는 one-shot scan-match handshake입니다. 이후 위치·yaw 품질은 신선한 `map -> camera_init`와 `camera_init -> body_nav` TF를 합성해 판정합니다. 두 TF edge가 0.50초 이내에 없거나 handshake가 없으면 READY가 될 수 없습니다.
- Odom reset은 message header source timestamp가 양수·유한하면 이를 사용하고, zero/invalid stamp는 receive time으로 fallback합니다. source time 역행·0 이하 dt·3.0 m/s 초과는 현재 initial-pose attempt에 latch하며 새 initial pose에서 초기화합니다.
- RViz bridge는 `wait_for_server(0.0)`만 사용하고 pending/active 중복 goal을 거부합니다. generation token으로 stale send response/result/cancel callback이 최신 handle을 지우지 못하게 했습니다.

### Fix 1 검증

- RED: 클릭 전 scan 재사용, 기본 QoS, 중복 in-flight goal을 재현하는 tests가 기존 구현에서 실패했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 12 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 120 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## Fix 2 경합 조건 보완

- scan gate는 localization generation 수와 무관한 단조 증가 `_scan_sequence`를 사용합니다. `/initialpose`가 당시 sequence를 baseline으로 저장하고 그보다 큰 유효 scan에서만 search를 시작하므로, 클릭 전 scan 뒤의 map callback이 search를 재개할 수 없습니다.
- search worker는 실행 중 future 하나와 최신 pending snapshot 하나만 유지합니다. 새 클릭은 queued future를 우선 `cancel()`하고, 실행 중이면 pending snapshot을 최신 값으로 교체합니다. 실행 작업이 끝난 timer 처리에서 최신 pending 하나만 제출합니다.
- 보정 자세를 재발행할 때마다 SLAM epoch, baseline pose/receive time, jump metric을 모두 초기화합니다. 따라서 재시도 뒤 첫 SLAM pose는 항상 새 baseline입니다.
- cancel service 응답은 acceptance/failure 기록만 하며 action handle과 `GoalGate` active 상태를 지우지 않습니다. matching `get_result_async()` 완료만 이를 해제하고, 그 전 새 goal은 거부합니다.
- odom source stamp는 `sec >= 0`, `0 <= nanosec < 1e9`, 유한성 및 all-zero convention을 검증합니다. 위반하면 receive time을 사용합니다.

### Fix 2 검증

- RED: pre-click scan의 map-callback 재사용, cancel-ack handle 조기 해제, publish 후 SLAM baseline 잔존, out-of-range nanosecond, rapid-click worker 누적을 기존 구현에서 각각 확인했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_rviz_goal_bridge_readiness.py test/test_goal_gate.py -q -p no:cacheprovider` — 19 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 125 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## Fix 3 세대 무효화

- 모든 `/initialpose` 수신(유효·무효 포함)은 lock 안에서 generation을 증가시키고 `_pending_search` 및 이전 initial pose를 제거합니다. 실행 중 future에는 cancel을 시도하지만, 실행 중인 작업은 강제 중단하지 않습니다.
- future/pending/generation snapshot의 확인·제출·완료 적용을 단일 `threading.Lock`으로 보호했습니다. worker는 node 상태를 직접 바꾸지 않으며, timer만 완료 결과를 적용합니다.
- 따라서 gen1 실행 중 gen2의 scan으로 대기 스냅샷이 생긴 뒤 gen3 클릭에 새 scan이 없으면, gen1 완료가 gen2를 제출하지 않습니다.

### Fix 3 검증

- RED: gen1 실행, gen2 pending, gen3 무scan 클릭 후 gen1 완료가 gen2 search를 재제출하는 기존 동작을 확인했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_rviz_goal_bridge_readiness.py test/test_goal_gate.py -q -p no:cacheprovider` — 20 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 126 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## 최종 리뷰 Fix A

- 2 Hz 상태/CSV timer는 유지하고 ready 전용 10 Hz timer를 추가했습니다. 상태 발행 때도 ready를 함께 발행할 수 있지만 CSV 기록은 계속 2 Hz이므로 진단 파일 증가율은 바뀌지 않습니다. 0.10초 heartbeat는 safety gate의 0.30초 timeout에 충분한 실행 여유를 제공하며, 상태 머신이 READY를 잃으면 다음 heartbeat가 즉시 false를 발행합니다.
- coarse search의 최종 후보 중심이 지도 밖이면 보정 자세를 발행하지 않고 `LOST/POSE_OUTSIDE_MAP`으로 전환합니다. epoch 이후 합성한 TF 중심이 지도 밖이어도 pose baseline을 제거하고 같은 오류로 fail-closed 처리합니다.
- AMCL 충돌은 전체 노드명이 아니라 ROS namespace를 제거한 basename이 정확히 `amcl`인지 판정합니다. 따라서 `/fallback/amcl`은 충돌이고 `/fallback/amcl_helper`는 충돌이 아닙니다.

### 최종 리뷰 Fix A 검증

- RED: 0.5초 단일 timer, 지도 밖 coarse 후보 발행, 지도 밖 SLAM pose 수용, namespaced AMCL 누락을 기존 구현에서 각각 확인했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_cmd_vel_gate_core.py test/test_goal_gate.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 48 passed.
- heartbeat 통합 mock은 10 Hz supervisor heartbeat와 실제 `VelocityGate` command/watchdog를 1.1초 동안 함께 구동하여 stale 차단이 발생하지 않고, LOST 전환 직후 false heartbeat가 gate를 닫는 것을 확인합니다.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 142 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## 최종 리뷰 Fix A2

- ready heartbeat와 status timer가 공통 `_evaluate_state(now)`를 호출합니다. 이 경로는 완료된 search를 적용하고, 동일 시각의 fresh observation에 map/scan/TF/SLAM 신선도와 AMCL·odom fault를 포함해 상태 머신을 갱신한 뒤 retry 보정 자세를 처리합니다.
- 10 Hz heartbeat는 평가 뒤 Bool을 발행하므로 저장된 READY를 0.5초 동안 반복하지 않습니다. namespaced AMCL은 첫 heartbeat에서 즉시 `LOST/false`, stale live 입력은 첫 heartbeat에서 `DEGRADED/false`가 되어 gate를 닫습니다.
- retry transition의 `republish_initial_pose`는 공통 평가 경로에서 소비됩니다. 상태 머신이 재시도 시작 시각을 갱신하므로 뒤따르는 2 Hz 평가가 같은 자세를 중복 발행하지 않습니다.

### 최종 리뷰 Fix A2 검증

- RED: 기존 heartbeat callback에는 상태 평가가 없어 새 `_on_ready_heartbeat` 안전 회귀 3건이 실패하는 것을 확인했습니다.
- focused: `py -3 -m pytest test/test_localization_supervisor.py test/test_cmd_vel_gate_core.py test/test_goal_gate.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 51 passed.
- AMCL heartbeat 즉시 loss, stale heartbeat 즉시 non-ready, heartbeat/status 연속 평가의 retry 자세 1회 발행을 검증했습니다.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 145 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## 최종 리뷰 Fix A3

- stale heartbeat 회귀는 초기 입력 부재로 shortcut하지 않고, map/scan/SLAM/TF와 품질이 모두 fresh인 `READY` 상태를 먼저 구성한 뒤 `0.50 s` 한계를 막 지난 시각의 단일 heartbeat에서 `DEGRADED/false`가 되는 계약을 검증합니다.
- retry 회귀는 heartbeat 직후 attempt가 `2`로 증가하고 refined pose가 정확히 한 번 발행된 것을 먼저 확인한 뒤, 이어지는 status timer가 추가 발행하지 않는 것을 별도로 확인합니다.
- 설계와 구현 계획의 발행률 계약을 status JSON·CSV `2 Hz`, ready heartbeat `10 Hz`로 통일했습니다.
- 집중 회귀: `py -3 -m pytest test/test_localization_supervisor.py test/test_cmd_vel_gate_core.py test/test_goal_gate.py test/test_rviz_goal_bridge_readiness.py -q -p no:cacheprovider` — 51 passed.
- 전체 패키지: `py -3 -m pytest test -q -p no:cacheprovider` — 154 passed, 1 skipped.
- 구문 검증: `py -3 -m compileall -q omx_navigation` — 성공.

## 최종 리뷰 Fix E

- 최초 coarse search에서 만든 지도 distance field를 보존하고, 이후 최신 `/scan`을 두 live TF edge에서 합성한 현재 자세에 투영해 scan-map overlap을 계속 다시 계산합니다. 초기 정합값을 READY 이후에도 고정 재사용하지 않습니다.
- 최신 overlap이 `0.45` 아래로 내려가면 첫 10 Hz heartbeat에서 `DEGRADED/LOW_OVERLAP`과 `ready=false`를 발행해 velocity gate를 닫습니다.
- `/Odometry` 수신 시각도 map/scan/SLAM/TF와 함께 `input_max_age=0.50 s` freshness 계약에 포함합니다. odometry가 정지하거나 비유한 위치를 보내면 `DEGRADED/INPUT_MISSING`으로 fail-closed 처리합니다.
- 비동기 coarse search 중 지도가 갱신되면 실행 중이던 이전 지도 결과와 distance field를 폐기하고 최신 지도 스냅샷으로 검색을 다시 제출합니다.
- 한국어 상태 문자열에 남아 있던 UTF-8 replacement character를 제거하고, 진단 문자열 전체에 `U+FFFD`가 없음을 회귀 테스트로 고정했습니다.

### 최종 리뷰 Fix E 검증

- RED: READY 이후 불일치 scan에서도 초기 overlap이 유지되는 문제, odometry 정지에도 READY가 유지되는 문제, 검색 중 map 변경 시 이전 distance field가 수용되는 문제, 손상된 한국어 진단 문자열을 각각 재현했습니다.
- focused: `py -3 -m pytest packages/omx_navigation/test/test_scan_map_quality.py packages/omx_navigation/test/test_localization_supervisor.py -q -p no:cacheprovider` — 49 passed.
- 전체 Windows 명시 세트: go1_driver, omx_navigation, migration 테스트 — 184 passed, 8 skipped.
- WSL ROS 2 Humble 및 Jetson ARM64 실기 검증은 이번 Windows-only 수정 뒤 아직 재실행하지 않았습니다.
