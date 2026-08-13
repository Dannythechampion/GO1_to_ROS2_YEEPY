# Navigation Safety Verification

현장에서 각 항목의 날짜, 담당자와 결과를 기록한다.

- [ ] Start pose가 없으면 `UNCOMMISSIONED`이며 `/initialpose`를 자동 발행하지 않는다.
- [ ] Start pose 없이 `initial_pose_arm:=true`이면 launch가 거절된다.
- [ ] Destination pose가 없으면 `/mission/start`만 거절된다.
- [ ] 세 번의 cold start에서 routine RViz 초기 위치 입력 없이 readiness가 true가 된다.
- [ ] E-stop heartbeat가 없거나 pressed이면 gate arm이 거절된다.
- [ ] Localization ready 전과 gate arm 전 `/cmd_vel_safe`는 zero다.
- [ ] 등록 목적지와 RViz 임의 목적지가 동일 mission manager를 통과한다.
- [ ] Active mission 중 두 번째 goal이 거절된다.
- [ ] E-stop, AMCL degradation과 FAST-LIO restart가 zero와 goal cancel을 유발한다.
- [ ] 복구 후 자동 재출발하지 않는다.
- [ ] Driver subscriber는 `/cmd_vel_safe` 하나뿐이다.

Automated verification과 topic inspection 명령은 [FIXED_START_AMCL_MISSION_COMMANDS.md](../FIXED_START_AMCL_MISSION_COMMANDS.md)를 사용한다.
