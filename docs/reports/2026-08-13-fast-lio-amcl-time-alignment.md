# FAST-LIO / AMCL 시간 정합: 센서 없는 검증 요약 보고서

작성일: 2026-08-13 (Asia/Seoul)

이 문서는 MID-360 및 Go1을 사용하지 않고 완료한 검증, 센서가 있어야만 가능한 후속
검증, 재현·롤백 절차를 한곳에 정리한 인계 보고서다. 현재 결과는 **빌드와 정적 검증
통과**를 의미하며, 실제 센서 데이터에서 지연 문제가 해결됐다는 뜻은 아니다.

## 1. 결론

| 항목 | 상태 | 결과 |
|---|---|---|
| 격리 브랜치 | PASS | `codex/fix-scan-qos` |
| 로컬 전체 테스트 | PASS | `162 passed, 2 skipped` |
| Jetson diagnostic 빌드 | PASS | 2개 패키지, 135초 |
| diagnostic → bounded 전환 | PASS | marker가 각 1개이며 중복 패치 없음 |
| Jetson bounded 빌드 | PASS | 2개 패키지, 131초 |
| 설치 바이너리 형식 | PASS | ARM64/aarch64 ELF PIE |
| ROS 환경에서 동적 링크 | PASS | 누락 라이브러리 0개 |
| 실행 중 프로세스에 새 바이너리 적용 | NOT RUN | FAST-LIO를 재시작하지 않음 |
| 실제 LiDAR/IMU 지연 검증 | NOT RUN | 센서 없는 범위 |
| AMCL/TF/costmap 검증 | NOT RUN | 센서와 지도상 초기 위치 필요 |
| 실제 주행 | 의도적으로 미실행 | arm 및 goal 명령 없음 |

센서 없이 가능한 범위에서는 코드, 테스트, Jetson 컴파일, 설치 결과와 재현성이 모두
확인됐다. 다음 판정 단계는 Go1을 `arm=false`, `DRY-RUN`으로 만든 뒤 새 FAST-LIO
바이너리를 로드하고 센서 입력으로 장시간 측정하는 것이다.

## 2. 시스템과 고정 버전

```text
repository: https://github.com/Dannythechampion/GO1_to_ROS2_YEEPY
branch: codex/fix-scan-qos
Jetson: unicon@192.168.0.138
Jetson project: /mnt/t500/go1_ros2_project
Jetson ROS workspace: /mnt/t500/go1_ros2_ws
ROS_DOMAIN_ID: 100
MID-360 IP: 192.168.1.145
Jetson LiDAR NIC: 192.168.1.5
FAST-LIO revision: 2fffc570a25d0df172720bac034fbdb6a13d2162
livox_ros_driver2 revision: 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
```

로컬 SSH 키와 Jetson runtime 파일은 저장소에 포함하지 않는다.

## 3. 증상과 원인 가설

기존 관찰에서 `/Odometry`, `/scan`, `/amcl_pose`의 header age는 처음 약 0.03초였지만
장시간 후 `/Odometry` 약 1.35초, `/scan` 약 1.37초로 증가한 채 10 Hz 출력은 유지됐다.
즉 출력 정지가 아니라 오래된 데이터를 계속 처리하는 backlog 패턴이다.

이미 적용했던 Livox 최신 프레임 정책과 FAST-LIO의
`SensorDataQoS().keep_last(1)`만으로는 재발을 막지 못했다. FAST-LIO 내부
`lidar_buffer`와 `time_buffer`는 제한 없는 deque이고, callback은 뒤에 추가하며
`sync_packages()`는 앞의 가장 오래된 항목부터 처리한다.

따라서 현재 가설은 “초기화 또는 순간 처리 지연 때 FAST-LIO 내부 deque에 누적된 과거
LiDAR frame이 제거되지 않아 Odometry, 파생 scan, AMCL이 함께 늦어진다”이다. 내부
queue와 front age를 실제 센서로 관찰하지 않았으므로 아직 확정 원인으로 보지 않는다.

## 4. 구현 내용

`migration/patch_fast_lio_low_latency.py`는 두 모드를 제공한다.

- `diagnostic`: 1 Hz `[fast_lio_realtime]` 로그로 queue depth, front age, drop 수,
  processing time, IMU synchronization margin을 기록하고 paired queue 접근을 잠근다.
- `bounded`: diagnostic 계측에 더해 in-flight front와 최신 대기 scan만 유지하고,
  front가 0.20초 이상 오래되면 교체한다. LiDAR/time queue는 항상 함께 조작한다.

동일 source에 재적용해도 marker가 중복되지 않으며 diagnostic source를 bounded로
승격할 수 있다. `measure_localization_latency.py`와
`verify_fast_lio_latency.sh`는 topic rate/age, timestamp 역행·미래값, 정지 속도,
AMCL covariance/spread, 내부 queue, 프로세스 PID, arm/DRY-RUN 및 TF 오류를 판정한다.

## 5. 센서 없이 완료한 검증

### 5.1 로컬 테스트

```powershell
cd "C:\Users\kimgk\OneDrive\문서\OMX-AI\.codex-worktrees\fix-scan-qos"
$taskPythonPath = "$PWD\migration;$PWD\packages\go1_driver;$PWD\packages\omx_navigation"
$env:PYTHONPATH = $taskPythonPath
python -m compileall -q migration packages/omx_navigation/omx_navigation packages/go1_driver/go1_driver
python -m pytest -q
```

결과: `162 passed, 2 skipped`. FAST-LIO patch 단위 테스트 8개, latency verifier 테스트
9개, 관련 workflow 테스트 18개가 통과했고 두 shell script의 Bash 문법도 통과했다.

### 5.2 Jetson 백업과 빌드

변경 전 source와 binary metadata를 다음에 보존했다.

```text
/mnt/t500/deploy_backups/20260813_225413_fast_lio_realtime/
  laserMapping.cpp
  source-before.diff
  binary.stat
  binary.target
```

Diagnostic 빌드는 Livox 5.16초, FAST-LIO 약 2분 10초, 총 135초에 성공했다.
Diagnostic source에 bounded 모드를 적용한 뒤 marker가 각각 정확히 1개인지 확인했고,
같은 패치를 재적용한 source hash가 유지되어 idempotence를 확인했다.

Bounded 빌드는 Livox 5.14초, FAST-LIO 약 2분 5초, 총 131초에 성공했다. 설치된
FAST-LIO는 aarch64용 64-bit PIE ELF이며 package prefix는 다음과 같다.

```text
/mnt/t500/go1_ros2_ws/install/fast_lio
source sha256: 466165771d834abfed8216a8d6b35dda37a0afac7ea22ba527be9250e6bc58f5
```

ROS setup을 source하지 않은 일반 shell의 `ldd`에서는 ROS 라이브러리가 보이지 않았으나,
`/opt/ros/humble/setup.bash`와 workspace setup을 source한 실행 환경에서는 `not found`가
0개였다. 실제 ROS 실행 조건을 기준으로 동적 링크 검증은 PASS다.

빌드 로그의 overlay override, PCL CMP0074, 미사용 변수, Boost bind, GCC ABI 메시지는
빌드를 막지 않는 기존 경고이며 새 compile/link 오류는 없었다.

### 5.3 안전 및 실행 상태

빌드 전 `/go1_driver`는 `arm=false`, `/go1/control_state`는
`DRY-RUN mode=1 vx=0 vy=0 yaw=0`이었다. 빌드 과정에서 Livox, FAST-LIO, Go1 프로세스를
재시작하거나 goal/속도 명령을 보내지 않았다.

빌드 후 ROS graph에서는 `/go1_driver` 노드가 확인되지 않아 사후 안전 상태는
`NOT AVAILABLE`로 기록했다. `arm=false`가 계속 유지됐다고 추정하지 않는다. 기존
FAST-LIO PID `129247`도 재시작하지 않았으므로 실행 중 프로세스는 빌드 전 바이너리이고,
새 bounded 바이너리는 설치만 된 상태다.

원시 증거:

```text
/mnt/t500/go1_runtime/latency/20260813_2259-sensorless-build/
  metadata.txt
  bounded-build.log
  binary.file
  binary.ldd
  binary.stat
  binary.target
  package-prefix.txt
```

## 6. 센서가 있어야 검증 가능한 항목

아래 항목은 빌드 성공만으로 판정할 수 없으며 모두 NOT RUN이다.

- raw LiDAR/IMU rate, timestamp age, 역행 및 future timestamp
- diagnostic 모드에서 내부 queue depth/front age가 실제로 증가하는지
- bounded 모드에서 queue가 2 이하로 유지되고 stale frame이 제거되는지
- `/Odometry`와 `/scan`의 10분 rate, p95/max age, timestamp reversal
- FAST-LIO processing p95/max와 IMU margin
- `map → odom → base` TF의 연속성·시간 정합
- `/amcl_pose` age, covariance, position/yaw spread
- costmap message filter 및 TF extrapolation 오류
- AMCL 초기 위치 지정 후 지도와 scan의 실제 겹침

따라서 이 보고서의 PASS 항목을 “AMCL 지연 해결 완료”로 해석하면 안 된다.

## 7. 다음 현장 검증 순서

1. Go1 driver를 다시 확인하고 반드시 `arm=false`, `DRY-RUN`, 속도 0을 확보한다.
2. 원인 가설을 직접 입증하려면 백업 source를 복원해 diagnostic으로 빌드하고 FAST-LIO만
   재시작한 뒤 120초 동안 queue depth/front age를 관찰한다.
3. bounded로 다시 빌드하고 FAST-LIO만 재시작한 뒤 600초 soak를 수행한다.
4. 기존 지도와 실제 위치가 겹치는 장소에서 RViz `2D Pose Estimate`를 지정하고 AMCL을
   180초 검증한다.

```bash
cd /mnt/t500/go1_ros2_project
FAST_LIO_LOW_LATENCY_MODE=diagnostic ./migration/build_livox_fastlio.sh
./migration/verify_fast_lio_latency.sh fast-lio 120 --observe-only

FAST_LIO_LOW_LATENCY_MODE=bounded ./migration/build_livox_fastlio.sh
./migration/verify_fast_lio_latency.sh fast-lio 600

./migration/verify_fast_lio_latency.sh amcl 180
```

합격 기준은 FAST-LIO warmup 60초 이후 9분 동안 age p95 ≤ 0.10초, max ≤ 0.30초,
queue depth ≤ 2이며, AMCL 구간에는 새 message-filter/TF extrapolation 오류가 없어야 한다.

## 8. 실험 이력

| 단계 | 결과 | 비고 |
|---|---|---|
| FAST-LIO DDS depth 1 | FAIL | 장시간 후 약 1.35초 지연 재발 |
| 내부 계측 및 bounded queue 구현 | LOCAL PASS | 단위·통합 테스트 통과 |
| 첫 Jetson precompile | FAIL | 원격 project에 Livox patch helper 2개 누락; 배포 후 해소 |
| 두 번째 Jetson precompile | FAIL | ROS setup이 nounset 뒤 실행되어 `AMENT_TRACE_SETUP_FILES` 오류 |
| source-before-nounset 수정 | PASS | 회귀 테스트 추가, commit `d1980e8` |
| Jetson diagnostic build | PASS | 135초, 프로세스 재시작 없음 |
| diagnostic → bounded/idempotence | PASS | marker 각 1개, source hash 유지 |
| Jetson bounded build | PASS | 131초, 새 바이너리 설치 |
| 센서 runtime/AMCL | NOT RUN | 본 작업 범위에서 제외 |

## 9. 다른 PC에서 이어서 작업하기

```bash
git clone https://github.com/Dannythechampion/GO1_to_ROS2_YEEPY.git
cd GO1_to_ROS2_YEEPY
git fetch origin
git switch --track origin/codex/fix-scan-qos
git log --oneline origin/main..HEAD
```

관련 설계와 실행 계획:

```text
docs/superpowers/specs/2026-08-13-fast-lio-realtime-buffer-design.md
docs/superpowers/plans/2026-08-13-fast-lio-realtime-buffer.md
docs/reports/2026-08-13-fast-lio-amcl-time-alignment.md
```

## 10. 롤백

명시적인 백업 경로를 확인한 뒤 source만 복원하고 FAST-LIO를 다시 빌드한다. 광범위한
workspace 삭제나 `git reset --hard`는 사용하지 않는다.

```bash
backup=/mnt/t500/deploy_backups/20260813_225413_fast_lio_realtime
test -f "$backup/laserMapping.cpp"
cp "$backup/laserMapping.cpp" /mnt/t500/go1_ros2_ws/src/FAST_LIO_ROS2/src/laserMapping.cpp
cd /mnt/t500/go1_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-up-to fast_lio \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

모든 runtime 게이트가 통과하기 전에는 `/go1_driver arm=true`, Nav2 goal 전송, 실제 이동,
초기 위치 파일 저장을 수행하지 않는다.
