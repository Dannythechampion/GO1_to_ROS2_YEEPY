# Fixed or RViz Mission Operation

1. 타일 기준에 GO1을 완전히 정지시켜 놓는다.
2. 두 runtime pose file을 지정해 existing-map launch를 실행한다.
3. `/localization/ready`가 true인지 확인한다.
4. 물리 E-stop을 해제하고 `/motion_gate/arm`을 호출한다.
5. 고정 목적지는 `/mission/start`, 임의 목적지는 RViz `2D Goal Pose`를 사용한다.
6. 중단 시 물리 E-stop을 우선하고 `/motion_gate/disarm`과 `/mission/cancel`을 호출한다.

자동 출발, goal 교체, queue와 자동 재개는 없다. 상태와 전체 명령은 [FIXED_START_AMCL_MISSION_COMMANDS.md](../FIXED_START_AMCL_MISSION_COMMANDS.md)를 따른다.

FAST-LIO restart가 감지되면 로봇을 출발 타일로 되돌린 뒤 다음 명령으로만 재-seed한다.

```bash
ros2 service call /localization/reset std_srvs/srv/Trigger "{}"
```
