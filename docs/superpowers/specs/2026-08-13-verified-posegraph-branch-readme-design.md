# Verified Posegraph 브랜치 README 설계

## 목적

`codex/verified-posegraph-navigation` 브랜치를 처음 방문한 사용자가 이 브랜치가
무엇을 해결하는지, `main`과 무엇이 다른지, Jetson에서 어떤 순서로 실행해야 하는지
루트 `README.md` 첫 화면에서 이해할 수 있게 한다.

## 적용 범위

- 변경은 `codex/verified-posegraph-navigation` 브랜치에만 커밋하고 푸시한다.
- 기존 루트 README의 상세 AMCL·매핑·운영 설명은 삭제하지 않는다.
- README 최상단에 브랜치 전용 안내 영역을 추가한다.
- 비교 기준은 작성 시점의 `origin/main`과 현재 브랜치 HEAD 실제 diff이다.

## 문서 구조

1. 브랜치 이름, 목적, 현재 검증 수준을 나타내는 안내 배너
2. 초기 로컬라이제이션 개선과 안전한 Nav2 경로의 핵심 요약
3. 입력 토픽부터 SLAM Toolbox, supervisor, Nav2, Go1 driver까지의 데이터 흐름
4. `main` 대비 기능·로컬라이제이션·안전·배포·검증 차이 비교표
5. Jetson의 `stage → build → preflight → dry-run → armed` 실행 순서
6. 통과한 Windows/WSL 검증과 Jetson 현장에 남은 실기 확인 항목
7. 기존 상세 문서로 이어지는 안내

## 정확성 및 안전 원칙

- WSL x86_64 검증을 Jetson ARM64·실센서·모터 성공으로 표현하지 않는다.
- armed 실행은 canonical `jetson_field_deploy.sh` 명령만 안내한다.
- `main`에 없는 기능을 추측하지 않고 최신 `origin/main`과 브랜치 HEAD를 직접
  대조해 기술한다.
- 이미 통과한 최종 증거인 Windows `242 passed, 13 skipped`와 fresh staged Humble
  `229 passed, 0 errors/failures/skipped`를 날짜와 함께 기록한다.

## 완료 기준

- 브랜치 전용 README 영역에 `main` 비교표와 현장 실행 명령이 모두 존재한다.
- Markdown 링크와 명령이 현재 저장소 파일·스크립트 이름과 일치한다.
- 기존 안전 정책과 모순되는 직접 `arm:=true` 우회 명령이 없다.
- 문서 계약 테스트, 전체 Python 테스트, `git diff --check`가 통과한다.
