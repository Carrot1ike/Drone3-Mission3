# 미션3 단계 인터페이스와 이후 미션 연결

현재 구현 범위는 미션3이다. 미션1 경로·미션2 장애물 회피는 구현하지 않았다. 공통 시간·자기 추정·하위 제어기를 유지하고 미션별 단계 상태만 교체하는 구조를 사용한다. 미션2→3의 입력은 알려진 진입 좌표·초기 정면과1m 호버 조건이다. 이를 실제 미션2 비행이라고 표시하지 않고 별도 sensor-controlled hover fixture로 검증한다.

## 실행 경계

- `Mission3(cfg, entry, start_time_s)`는 공개 지도·기하·제어 설정과 단계 시작 시각만 받는다. 실제 차량 plant 설정을 넣으면 거부한다.
- `update(now, own, vehicle)`는 같은 증가하는 센서 시각과 자기/영상 기반 차량 추정값으로 `Command`를 만든다. `now`는 전체 engine 기준이며 WAIT·탐색·미션 timeout은 시작 origin과의 차이로 계산한다.
- `Command`는 phase, world velocity m/s, yaw rate rad/s, done/failure, motors_off, physical_success를 갖는다. 차량0.25m/s는 환경 값이며 FF는 관측에서 추정한 속도다.
- 속도/yaw 출력은 PC velocity PID→RPYT→onboard attitude/rate PID로 내려간다. 일반 단계 전환마다 이 하위 PID·모터·추정기 상태를 초기화하지 않는다.
- `GEOMETRY_READY`는 마지막 영상 이후 제한 예측 조건을 만족한 인계다. 접촉 성공·모터off 명령으로 해석하지 않는다. 실제 최종 접촉 후보·motoroff/hold는 물리 adapter가 sensor-only 방식으로 처리하고, 성공 평가는 종료 후 별도로 한다.

## 단계와 전이

| 단계 | 진행 근거와 다음 단계 |
| --- | --- |
| HANDOFF_PREROLL fixture | 센서1m 안정 호버2초·바닥 기준·정확range·자세/속도/위치 제한. 이때 모터/추정기를 계속 실행. 준비 실패는 미션 시작 전 실패 |
| WAIT | 알려진 진입점에서 초기 정면 유지15초. 영상 발견이 먼저 되면 획득으로 진행. 차는 독립적으로 계속 이동 |
| EXPLORE / WATCH / 선택적 SCAN | 정적 지도에서 가까운 관측점 선택·이동·한정 yaw 탐색. 차량 실시간 위치를 사용하지 않음. 트랙을 영상으로 인식했다고 가정하지 않음 |
| COARSE_ACQUIRE | ID가 검출됐으나 작아서 metric pose가 신뢰되지 않는 영상 방향만 사용. 거리/속도 FF 추정으로 승격하지 않음 |
| ACQUIRE | metric 상대 pose를 여러 영상에서 받아 차량 방향·속도·추정 품질 확인 |
| LINEUP / LOWER_ALIGN | 차량 후방70cm 높이에서 주행 방향과 일직선 추종. 선택한 straight-preparation 설정에서는 관측된 안정 직선·준비거리 여유 뒤40cm로 낮춤. 상대 위치·yaw를 FB로 보정 |
| APPROACH / TRACK | 추정 차량 속도 FF + 상대 오차 FB. 마지막 신뢰 바닥 Flow와 영상 초점 여유 확보. 직선 여유를 추정 지도상에서 검사 |
| BLIND_APPROACH / BLIND_DESCEND | 마지막 영상·자기 센서로 짧게 상대 상태 전파. 센서 범위/무늬·추정 age/불확실성·직선 조건 제한 |
| GEOMETRY_READY → TOUCHDOWN_DESCEND | geometry가 접촉을 뜻하지 않음. 보정된 명령/IMU와 추정 상판높이에서만 제한 probe. no contact truth |
| MOTOR_OFF_HOLD | sensor support 후보 뒤 PWM0, 자동 재시동 없이 IMU3초 유지 검토. 실제 패드/네 발 유지 판정은 offline evaluator |
| BLIND_RECOVER / BLIND_RISE / ABORT | 근거 소실 시 후퇴·상승을 제한시간 시도. partial-support 뒤 기울기 실패가 남아 안전한 복구 완료는 미검증 |

`done`은 현재 시뮬레이션 adapter의 종료 신호다. 실제 radio/RTOS의 안전한 종료·watchdog·supervisor와 같지 않다. ABORT 뒤 로그상 명령0이나 물리 harness 정지를 실제 기체의 모터 정지/호버 보장으로 바꾸지 않는다. 실기 실행 coordinator에는 센서/명령 age와 stocksafety 경계를 별도로 넣어야 한다.

## 이후 모듈 재사용 조건

새 미션은 공통 자기 운동·시간·하위 PID를 유지하고 자체 observation/stage policy를 갖는다. 인계는 target-specific 관측/phase state만 바꾸며 무조건 성공을 가정하지 않는다. 위치 frame·단위·timestamp·공분산/품질·gyro/accel residual·현재 motor/armed state를 명시한 handoff record가 필요하다. 초기화가 필요한 vision target model과 계속 유지할 aircraft state를 구분한다.

현재 `HoverHandoff`는 그중 센서 상태를 보존하는 실행 예시다. 실제 미션2의 목표·기체상태·누적 drift·통신/clock을 만들어낸 것은 아니다. [구현](implementation.md),[편집 가능한 제어도](../diagrams/mission3-rpyt-control.svg),[PC telemetry gap](pc-telemetry-gap.md)에 구체적 코드와 미확정 경계를 연결한다.

곡선 또는 직선 끝의 획득에서는70cm LINEUP을 유지한다. 이때 정지 바닥 Flow·IMU free-flight 보정을 유지하고, 충분한 다음 직선에서 높이 조절을 준비한다. [실패와 전환 수정 근거](nearest-static-search.md)에45초 보정 만료 실패 및 준비시간 가정을 기록했다. 이 제어 단계 선택은 제조사 기체 성능 사양이 아니다.
