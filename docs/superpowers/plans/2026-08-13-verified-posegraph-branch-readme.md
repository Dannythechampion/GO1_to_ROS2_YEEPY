# Verified Posegraph Branch README Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `codex/verified-posegraph-navigation` 루트 README 첫 화면에 브랜치 목적, 전체 구조, 최신 `main` 대비 차이, Jetson 실행과 검증 범위를 정확히 설명한다.

**Architecture:** 기존 상세 README를 유지하고 제목 바로 아래에 독립적인 브랜치 안내 영역을 삽입한다. 문서 계약 테스트가 브랜치 이름, `main` 비교, 핵심 컴포넌트, canonical Jetson 명령, 검증 수치와 실기 제한을 고정한다.

**Tech Stack:** GitHub Markdown, Mermaid, pytest, Git

---

### Task 1: 브랜치 README 계약 고정

**Files:**
- Modify: `migration/test_posegraph_scripts.py`
- Test: `migration/test_posegraph_scripts.py`

- [ ] **Step 1: 실패하는 문서 계약 테스트 작성**

`test_verified_branch_readme_explains_main_delta_and_field_path`를 추가하고 다음 문자열을 요구한다.

```python
required = (
    "codex/verified-posegraph-navigation",
    "origin/main",
    "localization_supervisor",
    "cmd_vel_safety_gate",
    "241 passed, 13 skipped",
    "229 passed, 0 errors, 0 failures, 0 skipped",
    "jetson_field_deploy.sh dry-run",
    "jetson_field_deploy.sh armed GO1_ARMED_AND_ESTOP_READY",
)
```

또한 README 상단 300줄 안에서 `main` 비교표와 `Jetson AGX Orin` 실기 제한을 요구한다.

- [ ] **Step 2: RED 확인**

Run:

```powershell
py -3 -m pytest migration/test_posegraph_scripts.py::test_verified_branch_readme_explains_main_delta_and_field_path -q -p no:cacheprovider
```

Expected: 새 브랜치 안내 영역이 없어 FAIL.

### Task 2: README 상단 브랜치 안내 구현

**Files:**
- Modify: `README.md`
- Test: `migration/test_posegraph_scripts.py`

- [ ] **Step 1: README 제목 아래에 브랜치 전용 영역 추가**

다음 순서로 작성한다.

```text
브랜치 상태 배너
핵심 개선 사항
운영 데이터 흐름 Mermaid
origin/main 대비 비교표
Jetson canonical quick start
검증 증거와 현장 잔여 항목
기존 상세 문서 안내
```

비교표에는 main의 레거시 AMCL/수동 초기 pose 중심 경로와 이 브랜치의 bounded coarse search, SLAM Toolbox posegraph, 연속 scan-map 감시, READY 속도/goal gate, 이중 armed 확인, staged test gate를 대조한다.

- [ ] **Step 2: GREEN 문서 계약 확인**

Run:

```powershell
py -3 -m pytest migration/test_posegraph_scripts.py -q -p no:cacheprovider
```

Expected: Windows에서 Bash 전용 항목만 skip되고 나머지 PASS.

- [ ] **Step 3: 전체 회귀와 Markdown 무결성 확인**

Run:

```powershell
$env:PYTHONPATH='packages/omx_navigation;packages/go1_driver'
py -3 -m pytest migration packages/omx_navigation/test packages/go1_driver/test -q -p no:cacheprovider
git diff --check
```

Expected: 0 failures, `git diff --check` exit 0.

- [ ] **Step 4: 변경 범위 확인과 커밋**

```powershell
git status --short
git diff -- README.md migration/test_posegraph_scripts.py
git add README.md migration/test_posegraph_scripts.py
git commit -m "docs: explain verified posegraph branch"
```

`.worktrees/`는 stage하지 않는다.

- [ ] **Step 5: 현재 브랜치만 푸시**

```powershell
git push origin codex/verified-posegraph-navigation
```

Expected: `origin/main`은 변경하지 않고 현재 브랜치 remote HEAD만 전진한다.
