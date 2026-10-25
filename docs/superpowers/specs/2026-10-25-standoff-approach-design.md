# 목표물 상대 접근 설계 — standoff 배치와 최종 접근

## 1. 목표

베이스를 대상물 앞 standoff 자세에 **거칠게** 배치하고, 마지막 구간을
대상물 상대 제어로 닫는다. 파지에 필요한 정밀도는 지도 정합이 아니라
**대상물 상대 정합**이 담당한다.

이 단계의 완료 범위는 다음과 같다.

- 대상물 자세에서 standoff 베이스 목표를 생성한다.
- Nav2 목표 완료 후 대상물 상대 오차를 닫는 최종 접근 단계를 둔다.
- 기존 안전 경로(`cmd_vel_safety_gate`)를 우회하지 않는다.
- 검출기는 교체 가능한 인터페이스로 두고, 구현은 범위 밖으로 한다.

팔 제어 자체, 검출기 구현, 움직이는 대상물은 이 설계의 범위 밖이다.

## 2. 왜 이 구조인가

2026-10-25 주행에서 베이스 정합만으로 파지 공차를 맞출 수 없다는 것이
측정으로 확인되었다. 복도에서는 축방향 병진이 관측되지 않아, 정합 자세와
스캔이 가리키는 자세의 차이가 2026-10-25 p95 0.214 m / 최대 0.340 m,
2026-09-18 양호 구간 p95 0.335 m / 최대 0.350 m까지 벌어진다. 이 퍼짐보다
좁은 도착 조건은 유지될 수 없어 로봇이 목표 주위를 왕복했다.

따라서 `xy_goal_tolerance`를 0.35 m로 두었다. 두 세션 모두 "오차만으로
도착 판정이 뒤집힐 수 있는 샘플"이 0.0% / 0.9%가 되는 값이다. 이 값은
팔의 작업 공차보다 크므로, **두 번째 루프가 반드시 필요하다**.

이는 이 분야의 표준 구성이기도 하다. 베이스를 멈추고 팔이 수행하는
stop-and-grasp가 기본값이고, 베이스 오차는 주행 거리에 따라 커지므로
시각 피드백으로 보정한다. 사족보행에 가장 가까운 선행 연구(QuadWBG)는
도달성 지표로 몸체 배치를 고르고 one-time grasp 정확도 89%를 보고한다.
Nav2 자체도 도킹 서버에서 같은 2단계를 쓴다: 내비게이션이 구역까지
데려다주고, 라이브 검출을 쓰는 제어 루프가 최종 정렬을 맡는다.

## 3. 현재 런타임 체인 (사실 확인된 값)

```
FAST-LIO            camera_init -> body
planar_base_frame   camera_init -> body_nav      (z=0, yaw만 유지)
slam_toolbox        map -> camera_init
Nav2 controller  -> /cmd_vel_nav
cmd_vel_safety_gate -> /cmd_vel                  (/localization_supervisor/ready,
                                                  ready_timeout 0.30, command_timeout 0.30)
go1_driver                                       holonomic: true, invert_lateral: false
                                                 max_linear_speed 0.20, max_yaw_speed 0.40
                                                 linear_deadband 0.025, yaw_deadband 0.04
```

`body_nav`가 Nav2의 `robot_base_frame`이고, 스택에 센서→베이스 정적 변환은
없다.

## 4. 단계 ① standoff 목표 생성

### 4.1 입력과 출력

| | |
|---|---|
| 입력 | `target_pose` (대상물, `map` 프레임), `standoff_distance` d, 선택적 `approach_yaw` |
| 출력 | 베이스 목표 자세 — 대상물에서 d만큼 떨어져 대상물을 향함 |

### 4.2 d를 무엇으로 정하나

팔은 **OpenMANIPULATOR-X**다. ROBOTIS URDF(`open_manipulator_x_arm.urdf.xacro`)의
관절 원점에서 링크 길이가 그대로 나온다.

| 구간 | URDF xyz | 길이 |
|---|---|---|
| joint2 → joint3 | `0.024 0 0.128` | 0.13023 m |
| joint3 → joint4 | `0.124 0 0` | 0.124 m |
| joint4 → end_effector | `0.126 0 0` | 0.126 m |
| **합(어깨 기준 최대 도달)** | | **0.38023 m** |
| joint1 → joint2 (어깨 높이) | `0 0 0.0595` | 0.0595 m |

합이 ROBOTIS 공표값 380 mm와 일치하므로 체인을 올바로 읽은 것이다.

완전 신전은 특이점이라 힘도 여유도 없으므로 실제 예산은 그 일부만 쓴다
(초기값 75% → 0.285 m). 대상물이 어깨보다 위/아래면 수평 도달은 더
줄어든다(구면 포락선).

**여기서 첫 초안의 제약식이 틀렸다.** "베이스 도착 오차 0.35 m를 흡수하고도
도달 가능"은 성립할 수 없다 — 0.35 m는 팔 전체 도달 거리 0.38 m에 육박해서
d가 음수가 된다. 팔이 실제로 마주하는 오차는 Nav2의 허용오차가 아니라
**최종 접근 단계가 보장하는 오차**다. 올바른 제약은 다음과 같다.

```
reach_budget   = horizontal_reach(대상물 높이 − 어깨 높이)
                 − approach_tolerance − grasp_margin
d              = arm_base_offset_x + reach_budget        (reach_budget > 0 이어야 함)
```

Nav2의 0.35 m가 만족해야 하는 조건은 따로 있고 훨씬 약하다 — **그 지점에서
대상물이 검출기 시야 안에 들어오기만 하면 된다.**

접근 단계 공차가 커질수록 d가 줄고, 어느 선을 넘으면 `reach_budget`이
음수가 되어 어떤 standoff도 불가능해진다. 그때는 접근 단계를 더 조이거나,
팔 장착 위치 또는 대상물 높이를 바꿔야 한다. 구현은 이 경우를 숫자로
반환하지 않고 거부한다.

### 4.3 접근 방향 선택

`approach_yaw`가 주어지지 않으면 후보 각도를 샘플링해 다음으로 점수화한다.

- costmap 자유 공간 (베이스가 설 수 있는가)
- 대상물 가시성 (검출기 시야가 가려지지 않는가)
- 도달성 (팔이 그 방향에서 닿는가)

선행 연구가 쓰는 reachability 기반 배치와 같은 구조다.

### 4.4 배치

새 노드 `approach_planner`로 둔다. `rviz_goal_bridge`는 지금의 게이트
역할을 유지하며 건드리지 않는다. standoff 목표는 기존 경로 그대로
`navigate_to_pose`로 보낸다 — 기존 중단/취소 규칙을 그대로 물려받기 위해서다.

## 5. 단계 ② 최종 접근

### 5.1 핵심 결정 — 회전하지 않는다

최종 접근은 **순수 병진(vx, vy)** 으로만 수행한다. 근거는 다음과 같다.

- Nav2의 stateful 래치가 이미 yaw를 `yaw_goal_tolerance` 0.15 rad 안으로
  맞춰 둔 상태에서 넘어온다.
- Go1 드라이버가 `holonomic: true`라 횡이동이 가능하다.
- 회전을 쓰지 않으면 `yaw_deadband` 0.04 rad/s 사각지대를 통째로 피한다.
  2026-09-18과 2026-10-25의 최종 정렬 실패가 모두 이 사각지대와 관련이 있다.
- 회전이 없으므로 progress checker와도 충돌하지 않는다.

`linear_deadband`는 0.025 m/s로 작아 제약이 되지 않는다. 0.35 m를
0.05 m/s로 닫으면 약 7초다.

### 5.2 제어

대상물 상대 오차를 0으로 만드는 비례 제어로 시작한다.

```
e      = target_in_body_nav - desired_offset
vx     = clamp(kp * e.x, -v_approach, +v_approach)
vy     = clamp(kp * e.y, -v_approach, +v_approach)
yaw    = 0
```

`v_approach`는 주행 속도보다 낮게 둔다(예: 0.08 m/s). 오차가 줄면
감속하되, **명령이 `linear_deadband` 아래로 내려가기 전에 정지 판정**한다.
이것이 yaw에서 일어난 사각지대의 병진판 재발을 막는 조건이다.

### 5.3 종료 조건

| 결과 | 조건 |
|---|---|
| `ARRIVED` | `|e| < approach_tolerance` 가 N주기 연속 유지 |
| `ABORTED:detection_lost` | 검출이 `detection_timeout` 이상 끊김 |
| `ABORTED:timeout` | 전체 접근이 `approach_timeout` 초과 |
| `ABORTED:override` | `/go1/manual_override` true |
| `ABORTED:fault` | `/go1/execution_fault` true |
| `ABORTED:localization` | `/localization_supervisor/ready` false |

`approach_tolerance`도 팔 제원에서 나온다.

### 5.4 안전

- 명령은 **반드시 `/cmd_vel_nav`로 발행**한다. `cmd_vel_safety_gate`를
  그대로 통과시켜 ready 게이트와 watchdog을 물려받는다. 게이트를 우회하는
  경로는 만들지 않는다.
- Nav2 목표가 **완료된 뒤에만** 동작한다. 따라서 `/cmd_vel_nav` 발행자가
  겹치지 않는다. 겹치면 두 제어기가 번갈아 명령하게 되므로, 이 순서가
  설계의 전제다.
- 중단 규칙은 `rviz_goal_bridge`와 동일하게 맞춘다.

### 5.5 검출 인터페이스 (센서 비종속)

```
/approach/target_detection   geometry_msgs/PoseStamped
                             body_nav 또는 카메라 프레임, stamp 필수
```

검출기는 교체 가능하게 둔다 — 마커, 학습 기반 검출, 포인트클라우드 클러스터
어느 쪽이든 이 토픽만 맞추면 된다.

**MID-360은 근접 검출에 적합하지 않다.** FAST-LIO `preprocess.blind`가
0.5 m이고, 설령 낮추더라도 거치대와 팔 자체가 먼저 잡힌다. 최종 접근
검출은 카메라를 전제로 설계한다.

### 5.6 상태 보고

`/approach/status` (String, JSON)로 모든 전이를 낸다. 필드는
`rviz_goal_bridge`의 `/navigation/goal_status`와 같은 모양으로 맞추고,
RViz 마커도 `/navigation/goal_marker`와 같은 방식으로 띄운다. 조작자가
한 곳만 보면 되도록 하기 위해서다.

## 6. 상태 기계

```
IDLE
 └─ target 수신 ─> PLANNING ─ standoff 목표 산출 ─> NAVIGATING
                                                      │
                               Nav2 succeeded ────────┘
                                      │
                                      v
                                 APPROACHING ── 수렴 ──> ARRIVED ──> (팔에게 신호)
                                      │
   override / fault / LOST / timeout / 검출 상실 ──> ABORTED
```

`ARRIVED`에서 팔로 넘기는 신호가 이 설계의 출력이다. 팔이 무엇을 하는지는
범위 밖이다.

## 7. 확정된 값과 그 귀결

팔 장착 위치가 실측되어(2026-10-25 조립) 다음이 확정되었다.

```
팔 베이스 (body_nav)    x = +0.150 m,  z = -0.190 m    (라이다 기준)
어깨 높이 (body_nav)    z = -0.1305 m                  (= -0.190 + 0.0595)
최대 / 사용 도달        0.3802 / 0.2852 m              (75% 기준)
어깨 높이 대상물 standoff   0.355 m                    (approach_tolerance 0.05 기준)
```

### 7.1 도달 가능 높이 — 바닥은 닿지 않는다

포락선이 어깨 중심 구면이므로, 어떤 standoff를 써도 닿지 않는 높이가 있다.

```
도달 가능 범위 (body_nav)   z = -0.4157 .. +0.1547 m
```

센서가 바닥 위 0.50~0.60 m에 있다고 보면 바닥은 `z = -0.50 ~ -0.60`이고,
**0.08~0.18 m 부족해 닿지 않는다.** 대상물은 테이블 등 높은 면에 있어야 하며,
바닥 물체를 집으려면 팔 장착을 낮추거나 Go1이 자세를 낮춰야 한다.
(센서의 바닥 대비 높이는 직접 측정된 값이 아니므로 범위로 적는다. MID-360은
하향 FOV가 -7°뿐이라 바닥을 거의 보지 못해 측정이 어려웠다.)

### 7.2 검출기 요구사항

standoff가 0.355 m인데 Nav2 도착 오차가 0.35 m이므로, 접근 단계 시작 시점에
대상물은 **전방 0 ~ 0.71 m 어디에나 있을 수 있다.** 검출기는 이 범위를
시야와 작동 거리 안에 담아야 한다. 이것이 Nav2 허용오차가 만족해야 하는
유일한 조건이다.

### 7.3 아직 미정

- 검출기 종류와 실제 시야 — `approach_tolerance` 0.05 m가 지켜지는지는
  검출기가 정해져야 확정된다.
- `usable_fraction` 0.75 — 로봇에서 검증되지 않은 초기값.
- `v_approach`, `kp` — 기본값은 있으나 실주행 미검증.

## 8. 검증 계획

- **단위 테스트**: standoff 자세 계산(거리·방향), 제어 법칙의 포화와
  데드밴드 상한, 상태 전이표, 모든 중단 경로.
- **로그 기반**: 기존 세션의 자세 기록으로 standoff 계산을 돌려
  베이스 도착 오차 0.35 m가 흡수되는지 확인한다.
- **실주행**: 팔 제원 확정 후. 이 설계는 주행으로 검증된 바 없다.
