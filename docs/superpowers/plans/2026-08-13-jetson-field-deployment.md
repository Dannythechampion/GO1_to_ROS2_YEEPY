# Jetson Field Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson AGX Orin에서 staging, ARM64 사전검사, 비무장 dry-run, 명시적으로 승인된 armed 주행과 정상 종료까지 재현 가능한 현장 배포 경로를 만든다.

**Architecture:** 기존 posegraph navigation과 velocity safety gate를 유지하고, staging 및 현장 preflight 스크립트를 fail-closed 경계로 사용한다. armed launch는 기본 거부하되 ARM64·Unitree wrapper·확인 토큰·driver·진단 기록 조건을 모두 만족할 때만 허용한다. 각 ROS Python entry point는 공통적인 Humble external shutdown 의미를 따르고 이미 종료된 context를 재종료하지 않는다.

**Tech Stack:** ROS 2 Humble, Python 3.10/rclpy, Bash, colcon/ament_python, pytest, NVIDIA Jetson aarch64, Unitree SDK v3.8.6.

---

## 파일 구조

- `migration/stage_local_ros2_packages.sh`: Jetson workspace에 소스·테스트·현장 도구를 선택 복사한다.
- `migration/jetson_field_deploy.sh`: Jetson의 `stage`, `build`, `preflight`, `dry-run`, `armed` 명령을 제공한다.
- `migration/test_posegraph_scripts.py`: staging과 field script의 정적·fake-command 계약을 검증한다.
- `packages/omx_navigation/launch/go1_posegraph_navigation.launch.py`: armed 조건을 launch 시작 전에 검증한다.
- `packages/omx_navigation/test/test_posegraph_launch.py`: armed 조건 조합과 기본 비무장 계약을 검증한다.
- `packages/omx_navigation/omx_navigation/runtime_shutdown.py`: rclpy external shutdown과 context 상태를 처리하는 작은 공통 helper다.
- `packages/omx_navigation/omx_navigation/{localization_supervisor,planar_base_frame,rviz_goal_bridge,cmd_vel_safety_gate}.py`: helper로 정상 종료한다.
- `packages/omx_navigation/test/test_runtime_shutdown.py`: helper의 순수 단위 테스트다.
- `packages/omx_navigation/test/test_*lifecycle.py`: 각 main의 node 정리 및 stop 발행을 검증한다.
- `packages/go1_driver/go1_driver/node.py`: ExternalShutdownException에서도 SDK stand 후 정상 종료한다.
- `packages/go1_driver/test/test_node_lifecycle.py`: driver main 종료 계약을 검증한다.
- `README.md`, `packages/omx_navigation/README.md`: 한국어 현장 명령과 체크리스트를 제공한다.

### Task 1: staging이 실제 테스트와 현장 도구를 보존하도록 수정

**Files:**
- Modify: `migration/test_posegraph_scripts.py`
- Modify: `migration/stage_local_ros2_packages.sh`
- Create: `migration/jetson_field_deploy.sh`

- [ ] **Step 1: 실패하는 staging 계약 테스트 작성**

`test_stage_copies_posegraph_runtime_files_and_maps`에 `test`와 `jetson_field_deploy.sh`를 요구하고, 임시 source/workspace에서 stage script를 실행해 `src/omx_navigation/test`와 두 실행 스크립트가 존재하는지 확인한다.

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

Expected: staged `test` 또는 `jetson_field_deploy.sh`가 없어서 FAIL.

- [ ] **Step 3: 최소 staging 구현**

`omx_entries`에 `test`를 추가하고 다음 복사를 추가한다.

```bash
cp -a "$repo_root/migration/jetson_field_deploy.sh" \
  "$omx_target/jetson_field_deploy.sh"
chmod +x "$omx_target/verify_posegraph_navigation.sh" \
  "$omx_target/jetson_field_deploy.sh"
```

- [ ] **Step 4: GREEN 확인**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add migration/stage_local_ros2_packages.sh migration/jetson_field_deploy.sh migration/test_posegraph_scripts.py
git commit -m "fix: stage Jetson verification assets"
```

### Task 2: rclpy 정상 종료 공통 계약 구현

**Files:**
- Create: `packages/omx_navigation/omx_navigation/runtime_shutdown.py`
- Create: `packages/omx_navigation/test/test_runtime_shutdown.py`
- Modify: `packages/omx_navigation/omx_navigation/localization_supervisor.py`
- Modify: `packages/omx_navigation/omx_navigation/planar_base_frame.py`
- Modify: `packages/omx_navigation/omx_navigation/rviz_goal_bridge.py`
- Modify: `packages/omx_navigation/omx_navigation/cmd_vel_safety_gate.py`
- Modify: `packages/omx_navigation/test/test_planar_base_frame.py`
- Modify: `packages/omx_navigation/test/test_cmd_vel_safety_gate_lifecycle.py`
- Modify: `packages/omx_navigation/test/test_rviz_goal_bridge_readiness.py`
- Modify: `packages/omx_navigation/test/test_localization_supervisor.py`

- [ ] **Step 1: 실패하는 helper와 main 테스트 작성**

희망 API를 먼저 테스트한다.

```python
def test_shutdown_context_calls_shutdown_only_while_ok():
    runtime_shutdown.shutdown_context(FakeRclpy(ok=True))
    assert fake.shutdown_calls == 1
    runtime_shutdown.shutdown_context(FakeRclpy(ok=False))
    assert fake.shutdown_calls == 1
```

각 main mock의 `spin`은 `ExternalShutdownException`을 발생시키고 main이 예외 없이 반환하며 node가 destroy되는지 확인한다. gate는 context가 살아 있을 때 stop publish가 destroy보다 먼저 호출되는지 확인한다.

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest packages/omx_navigation/test/test_runtime_shutdown.py packages/omx_navigation/test/test_planar_base_frame.py packages/omx_navigation/test/test_cmd_vel_safety_gate_lifecycle.py packages/omx_navigation/test/test_rviz_goal_bridge_readiness.py packages/omx_navigation/test/test_localization_supervisor.py -q -p no:cacheprovider`

Expected: helper 미존재 또는 ExternalShutdownException 전파로 FAIL.

- [ ] **Step 3: 최소 helper 구현**

```python
def spin_exceptions(external_shutdown_exception):
    return (KeyboardInterrupt, external_shutdown_exception)

def shutdown_context(rclpy_module):
    if rclpy_module.ok():
        rclpy_module.shutdown()
```

각 main은 `from rclpy.executors import ExternalShutdownException`을 사용해 두 정상 종료 예외를 잡고 `finally`에서 node 정리 후 `shutdown_context(rclpy)`를 호출한다.

- [ ] **Step 4: GREEN 확인**

Step 2의 명령을 다시 실행한다. Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add packages/omx_navigation/omx_navigation packages/omx_navigation/test
git commit -m "fix: exit ROS nodes cleanly on shutdown"
```

### Task 3: Go1 driver가 external shutdown에서도 stand 후 정상 종료

**Files:**
- Modify: `packages/go1_driver/go1_driver/node.py`
- Create or Modify: `packages/go1_driver/test/test_node_lifecycle.py`

- [ ] **Step 1: 실패 테스트 작성**

`rclpy.spin`이 `ExternalShutdownException`을 발생시키는 mock에서 다음 순서를 검증한다.

```text
shutdown_robot -> destroy_node -> optional rclpy.shutdown
```

이미 context가 종료된 경우 `rclpy.shutdown` 호출은 0회여야 한다.

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest packages/go1_driver/test -q -p no:cacheprovider`

Expected: ExternalShutdownException 전파로 FAIL.

- [ ] **Step 3: 최소 구현**

`rclpy.executors.ExternalShutdownException`을 정상 종료 예외로 처리하고 기존 `rclpy.ok()` 가드를 유지한다. `shutdown_robot()`은 node destroy 전에 호출한다.

- [ ] **Step 4: GREEN 확인**

Run: `py -3 -m pytest packages/go1_driver/test -q -p no:cacheprovider`

Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add packages/go1_driver/go1_driver/node.py packages/go1_driver/test
git commit -m "fix: stand Go1 on external ROS shutdown"
```

### Task 4: armed launch를 이중 확인으로 제한

**Files:**
- Modify: `packages/omx_navigation/launch/go1_posegraph_navigation.launch.py`
- Modify: `packages/omx_navigation/test/test_posegraph_launch.py`

- [ ] **Step 1: 실패하는 armed 조합 테스트 작성**

`validate_operating_mode` 희망 API에 다음 표를 고정한다.

| arm | driver | recording | token | 결과 |
|---|---|---|---|---|
| false | false | false | 빈 값 | 허용 |
| false | true | true | 빈 값 | 허용 |
| true | false | true | 올바름 | 거부 |
| true | true | false | 올바름 | 거부 |
| true | true | true | 틀림 | 거부 |
| true | true | true | `GO1_ARMED_AND_ESTOP_READY` | 허용 |

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest packages/omx_navigation/test/test_posegraph_launch.py -q -p no:cacheprovider`

Expected: 현재 `arm=True`가 항상 거부되어 마지막 행 FAIL.

- [ ] **Step 3: 최소 launch 구현**

`armed_confirmation` launch argument를 기본 빈 문자열로 추가한다. `_validate_launch_inputs`에서 `arm`, `start_go1_driver`, `record_localization`을 명시적 boolean으로 파싱하고 `validate_operating_mode`로 조합을 검사한다. 기존 파일 검증은 arm 여부와 무관하게 유지한다.

- [ ] **Step 4: GREEN 확인**

Run: `py -3 -m pytest packages/omx_navigation/test/test_posegraph_launch.py -q -p no:cacheprovider`

Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add packages/omx_navigation/launch/go1_posegraph_navigation.launch.py packages/omx_navigation/test/test_posegraph_launch.py
git commit -m "feat: require explicit armed field confirmation"
```

### Task 5: Jetson field script 구현

**Files:**
- Modify: `migration/jetson_field_deploy.sh`
- Modify: `migration/test_posegraph_scripts.py`

- [ ] **Step 1: fake-command RED 테스트 작성**

`preflight`가 x86_64, 잘못된 ROS 배포판, Unitree wrapper 미존재, posegraph 누락, diagnostics 쓰기 실패를 거부하는지 검사한다. `armed`는 올바른 확인 토큰 없이 launch 명령을 호출하지 않아야 하고, `dry-run`은 항상 `arm:=false`, `armed`는 모든 preflight 성공 후에만 `arm:=true`와 확인 토큰을 전달해야 한다.

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

Expected: field script 동작 미구현으로 FAIL.

- [ ] **Step 3: 최소 shell 구현**

명령 인터페이스를 고정한다.

```text
jetson_field_deploy.sh stage <repo> [workspace]
jetson_field_deploy.sh build [workspace]
jetson_field_deploy.sh preflight [workspace]
jetson_field_deploy.sh dry-run [workspace]
jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY [workspace]
```

`preflight`는 aarch64·Jammy·Humble·필수 패키지·wrapper·artifact·쓰기 경로를 검사한다. `dry-run`과 `armed`는 먼저 preflight를 실행하고 `exec ros2 launch`로 신호를 그대로 전달한다.

- [ ] **Step 4: GREEN 확인**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

Expected: PASS.

- [ ] **Step 5: 커밋**

```bash
git add migration/jetson_field_deploy.sh migration/test_posegraph_scripts.py
git commit -m "feat: add fail-closed Jetson field runner"
```

### Task 6: 현장 한국어 문서 정렬

**Files:**
- Modify: `README.md`
- Modify: `packages/omx_navigation/README.md`

- [ ] **Step 1: 문서 계약 테스트 보강**

`migration/test_posegraph_scripts.py`에서 `stage`, `build`, `preflight`, `dry-run`, `armed GO1_ARMED_AND_ESTOP_READY`, `arm=false` 선행, e-stop, 0.3m 첫 goal, 정상 종료 확인 문구를 요구한다.

- [ ] **Step 2: RED 확인**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

Expected: 새 현장 명령이 문서에 없어 FAIL.

- [ ] **Step 3: 한국어 runbook 작성**

복사 가능한 명령을 다음 순서로 적는다.

```text
APT/rosdep 갱신 -> stage -> Unitree wrapper -> build -> test -> Livox -> FAST-LIO
-> preflight -> dry-run -> 2D Pose Estimate -> READY/NONE -> armed -> 0.3m goal
-> cancel/ready loss/Ctrl-C 정지 확인
```

- [ ] **Step 4: GREEN 확인 및 커밋**

Run: `py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider`

```bash
git add README.md packages/omx_navigation/README.md migration/test_posegraph_scripts.py
git commit -m "docs: add Jetson field deployment runbook"
```

### Task 7: Windows 및 fresh WSL 최종 검증

**Files:**
- Modify only if verification exposes a regression.

- [ ] **Step 1: Windows full suite**

Run:

```powershell
$env:PYTHONPATH='packages/omx_navigation;packages/go1_driver'
py -3 -m pytest packages/go1_driver/test packages/omx_navigation/test migration/test_existing_map_scripts.py migration/test_end_to_end_workflow.py migration/test_posegraph_scripts.py -q -p no:cacheprovider
py -3 -m compileall -q packages migration
git diff --check origin/main...HEAD
```

Expected: 0 failures, compile exit 0, diff check exit 0.

- [ ] **Step 2: fresh WSL remote-equivalent staging**

새 `/tmp/go1-jetson-final-*` workspace를 만들고 현재 HEAD를 clean export 또는 clone한다. `bash -n`, stage, `rosdep`, `colcon build`, `colcon test`, `colcon test-result --verbose`를 실행한다.

Expected: 두 패키지 모두 non-zero tests, 0 errors/failures/skips.

- [ ] **Step 3: Humble runtime smoke**

설치 모듈 import/compile, launch `--show-args`, invalid armed 조합 거부, 임시 테스트 TF를 사용한 `arm=false` bringup을 확인한다. SIGINT 후 모든 사용자 Python 노드는 exit 0이어야 한다.

- [ ] **Step 4: 독립 코드리뷰**

`origin/main..HEAD`를 기준으로 Critical/Important Jetson 배포 이슈를 검토하고, 발견 시 수정 후 전체 검증을 반복한다.

- [ ] **Step 5: 최종 커밋 및 push**

```bash
git status --short
git push -u origin codex/verified-posegraph-navigation
git rev-parse HEAD
git rev-parse '@{upstream}'
```

Expected: 의도한 파일 외 변경 없음, local/upstream SHA 일치.
