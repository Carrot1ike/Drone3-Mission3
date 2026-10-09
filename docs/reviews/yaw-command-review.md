# Yaw 명령 계약 독립 검토

2026-10-04. `AGENTS.md`와 `docs/yaw-correction-design.md` 및 지정된 로컬 제어 경로를 정적으로 검토했다. PC 계산·기존 센서·제조사 장착을 전제로 한다. 구현, 설치, 시뮬레이션 실행, 비행 명령 없이 이 보고서만 작성했다. PnP 후보·기하 정확도는 다른 검토 범위다.

**판정:** 전방 마커를 보고 이륙한 뒤 상대 방향을 PC에서 보정하는 구조는 제어 인터페이스로 표현할 수 있다. 다만 초기 정렬은 지속적인 yaw 관측이나 firmware 추정기 초기화를 대신하지 않는다. 설계가 이미 구분한 상위 제어·내부 추정의 경계는 타당하며, 아래 계약을 구체화하기 전에는 폐루프 동작이 확인됐다고 볼 수 없다.

1. **단위·부호는 firmware protocol까지 묶어야 한다.**

   **근거:** [ROS callback][server-hover]은 입력 `u`를 `-degrees(u)`로 바꾼다. [CFLib][cflib-hover]은 protocol ≤8이면 legacy type 5로 보내면서 다시 부호를 반전한다. 로컬 [firmware protocol][protocol]은 7이며, [hover decoder][hover-decoder]는 수신 yawrate를 다시 반전한다. 따라서 이 소스 조합에서는 `u=+1 rad/s → CFLib 인자 −57.30 deg/s → 패킷 +57.30 → attitudeRate.yaw −57.30 deg/s`다. protocol >8의 CFLib은 type 10을 보내지만 읽은 firmware decoder는 type 5 경로이므로 같은 계약으로 일반화할 수 없다.

   **영향:** 콜백과 decoder의 두 반전만 계산하면 제어 부호를 잘못 정해 양의 피드백을 만들 수 있다. `vx/vy`는 body 기준 m/s, `z_distance`는 decoder에서 world `position.z`의 m 단위 절대 목표다. 이동 차량 표면까지의 직접 거리 명령으로 해석하면 높이 기준도 어긋난다.

   **최소 수정안:** 프로젝트 소유 명령 어댑터의 설계에 protocol·패킷 type·rad/deg·상대 yaw 축·목표 높이 기준을 명시하고 부호 변환을 한 경계에서 관리한다. 양/음 입력과 내부 목표 각도의 기대 관계를 검증 항목으로 정한다.

   **미검증:** 실행 중 실제 CFLib 경로·협상 protocol·활성 controller, 물리 회전 방향 및 기체 응답.

2. **yaw_rate는 내부 yaw 추정값을 수정하지 않으며 PID에서는 목표 각도를 움직인다.**

   **근거:** [PID][pid-yaw]는 `modeVelocity`에서 `attitudeDesired.yaw += attitudeRate.yaw × dt`를 적분·wrap하고, 설정에 따라 현재 추정 yaw와의 차이를 제한한다. [자세 PID][attitude-yaw]는 목표와 `state->attitude.yaw`의 오차로 내부 rate 목표를 만든다. hover 경로에는 시각 측정 주입이 없다.

   **영향:** PC P 제어기는 내부 자세/rate 루프를 포함하는 동적 계통을 제어한다. ROS rate 값이 즉시 실제 gyro rate와 같다고 가정할 수 없다. `yaw_rate=0`도 적분된 목표 각도를 유지하므로 순간 회전 정지나 yaw 추정 bias 제거를 뜻하지 않는다.

   **최소 수정안:** 첫 목표를 ‘마커 상대 기수 유지’로 한정하고 PC 상대 yaw와 firmware yaw를 별도 변수로 기록한다. 작은 P 이득·포화·deadband에 더해 내부 목표와 응답을 검증 항목으로 둔다. 내부 추정 보정은 별도 좌표·측정 융합 설계로 유지한다.

   **미검증:** 실제 루프 응답, `yawMaxDelta` 설정, 드리프트 하에서의 안정성. 처음 마커를 향한 배치만으로 이 항목들이 해결되지 않는다.

3. **촬영 시각과 동기화된 gyro 입력이 확보되지 않았다.**

   **근거:** [카메라][image-stamp]는 TCP 수신·decode 뒤 `_publish_frame()`에서 ROS `now()`를 찍는다. [서버 로그][log-vars]의 pose에는 gyro가 없다. odom 경로는 gyro를 읽지만 [callback][odom]이 deg/s 값을 `twist.angular`에 변환 없이 넣고 stamp도 현재 시각으로 덮는다. [firmware gyro 로그][gyro]는 deg/s임을 명시한다. 반면 [custom log][custom-log]는 firmware `timestamp`를 별도 보존한다.

   **영향:** Image stamp를 노출 시각으로 쓰면 숨은 전송·decode 지연이 yaw 오차로 들어간다. parent가 확인한 영상 약 18 Hz·pose 10 Hz 수신만으로 IMU 예측을 검증할 수 없다. odom angular 값을 rad/s로 쓰면 크기가 약 57.3배 어긋난다.

   **최소 수정안:** 공유 소스를 바꾸지 않고 프로젝트 소유 설정/어댑터에서 gyro x/y/z와 자세의 custom log를 선택하는 계획을 세운다. deg/s→rad/s, 자세 축 변환, firmware 시계↔ROS 시계 매핑과 버퍼를 정의한다. 촬영 시각이 없으면 ‘decode 후 발행 시각 근사’로 명시하고 지연 불확실성에 따라 관측 가중치·명령 한계를 낮춘다.

   **미검증:** gyro 스트림 수신·주기·누락, 시계 매핑, 촬영 지연과 변동. 기본 sim의 IMU noise 꺼짐은 드리프트 강건성 근거가 아니다.

4. **hover 보정은 high-level 이륙·착륙에 덧붙이는 yaw 채널이 아니다.**

   **근거:** [우선순위][priority]는 high-level=1, CRTP=2다. [commander][commander]는 CRTP setpoint 수락 시 high-level planner를 중단한다. [notify decoder][notify]는 우선순위만 낮추며 `remainValidMillisecs`를 사용하지 않는다. ROS [notify callback][server-notify]도 요청의 유효시간을 전달하지 않는다. [중단된 high-level planner][hl-stopped]는 null setpoint를 반환한다.

   **영향:** 이륙 중 hover를 보내면 전체 계획과 높이·수평 제어 소유권을 바꾼다. hover 발행 종료나 notify만으로 이전 계획이 복귀하거나 안전한 호버/착륙이 보장되지 않는다. 새 계획 없이 우선순위만 반환하면 중단 planner의 null 목표가 수락될 수 있다.

   **최소 수정안:** ‘이륙 완료 확인→전체 hover 목표의 단일 발행자로 인계→보정→명시적 복구 계획으로 반환’ 상태 전이를 설계한다. 반환 시 마지막 목표·새 high-level 계획·우선순위 해제의 순서와 전송 경쟁을 정하고, 남은 hover 발행이 새 계획을 다시 중단하지 않도록 한다.

   **미검증:** 서비스 처리 순서, 계획 준비와 인계 확인, 복구 시 과도응답.

5. **관측 상실과 명령 상실을 별도로 처리해야 한다.**

   **근거:** [hover callback][server-hover]은 Header 시각·유효기간·범위를 검사하지 않는다. firmware는 [수락 시각][commander]으로 목표를 갱신한다. [watchdog][wdt]는 목표 age >500 ms/2000 ms를 경고/timeout으로 판정하며, 비행 중 [상태 전이][wdt-state]와 [override][wdt-override]는 수평 제어 해제·yaw rate 0·Z 유지, 이어 모터 정지 경로를 정의한다.

   **영향:** 오래된 시각 오차로 명령을 계속 발행하면 watchdog은 관측 노후화를 검출하지 못한다. 반대로 발행을 단순히 끊으면 정지 표적 시험의 제어된 종료/착륙을 보장하지 않는다.

   **최소 수정안:** 프로젝트 소유 supervisor 설계에서 영상 age·IMU age·명령 발행 age를 분리한다. 짧은 누락의 제한된 예측 시간, 장기 누락의 보정 중지, 상태별 복구 목표·제어권 반환을 수치 계약으로 정한다. 오래된 명령의 재발행을 금지하고 별도 명령 발행 주기·만료 기준을 둔다. firmware watchdog은 최후 경계로 취급한다.

   **미검증:** 종단 지연·큐 적체·통신 단절 시 실제 행동과 실행 빌드의 watchdog 적용. 참값은 평가용만 사용하며 수신 확인을 폐루프/안전성 검증으로 확대하지 않는다.

[server-hover]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:1189
[cflib-hover]: ./src/crazyflie-ros2-sil/crazyflie-lib-python/cflib/crazyflie/commander.py:167
[protocol]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/interface/crtp.h:33
[hover-decoder]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/crtp_commander_generic.c:299
[pid-yaw]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/controller/controller_pid.c:63
[attitude-yaw]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/controller/attitude_pid_controller.c:168
[image-stamp]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/aideck_ros2_bridge/aideck_ros2_bridge/aideck_camera_node.py:172
[log-vars]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:97
[odom]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:665
[gyro]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/stabilizer.c:632
[custom-log]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:743
[priority]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/interface/commander.h:35
[commander]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/commander.c:74
[notify]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/crtp_commander.c:94
[server-notify]: ./src/crazyflie-ros2-sil/crazyswarm2_ws/src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:1022
[hl-stopped]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/crtp_commander_high_level.c:318
[wdt]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/supervisor.c:330
[wdt-state]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/supervisor_state_machine.c:164
[wdt-override]: ./src/crazyflie-ros2-sil/crazyflie-firmware/src/modules/src/supervisor.c:417
