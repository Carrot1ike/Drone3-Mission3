# 실패 명령 전달과 관측 종료 계약

2026-10-09 07:45 KST 설계 초안. v17의 28회 source/config/runtime 동결 비교가 모두 끝나고 집계되기 전에는 실행 코드를 변경하지 않는다.

독립 v15 검토는 실패 12개 모두 `command.done` 직후 adapter가 새 RPYT packet과 physics step 이전에 끝난다고 확인했다. ABORT 로그의 0속도는 실제 송신·감속 증거가 아니다. v16 실패613에서도 같은 결함이 남았다. 원자료와 실패 판정은 유지한다.

제조사 [Commander API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/commander/)에서 world velocity 명령과 STOP은 다른 명령이다. STOP은 모터를 끄며 공중에서는 추락할 수 있다. [Commander framework](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/)는 비행 중 연속 명령 공급과 수신 중단 시 watchdog 동작을 설명한다. 현재 고정 firmware `f6e0f0a3b526861328caf3296a7ecf00bb915d39`의 `supervisor.c`에는 안정화500ms/종료2000ms가 있다(C). 웹 설명은 이전 파일 위치 commander.c를 언급하므로 현재 실행 snapshot을 별도로 대조한다. 지금 host bench는 실제 commander/supervisor/CRTP/RTOS 전체 경로가 아니며 이 watchdog을 시험했다고 주장하지 않는다(U).

다음 source에서는 유한한 추정 상태에서 실패 이유를 한 번 고정하고, 0 world velocity/0 yaw-rate를 PC PID→RPYT→기체 PID→모터→physics에 계속 공급한다. ABORT를 착륙 성공으로 바꾸지 않는다. 공중 ABORT에서 STOP/PWMzero를 생성하지 않는다. 원 urgent zero 명령 예외를 유지하고 normal saturation 검사와 분리한다.

별도 종료 관측은 fresh floor Flow/accurate ToF, 유한 추정 속도·자세·gyro를 요구한다. 3D속도≤.05m/s, tilt≤10°, gyro norm≤5°/s가 .5초 연속 유지되고, 실패 이후 packet과 physics step이 실제 있었을 때에만 `sensor_stationary_hold_observed`로 기록한다. consumer stamp age≤.03s와 시간 증가도 검사한다. 관측이 끊기거나 부적합하면 dwell을 초기화하며, 최대3초 뒤 `bounded_delivery_elapsed_without_stationary_evidence`로 끝낸다. 이 값들은 시험용 소프트웨어 선택(S), 제조사 안정성 한계가 아니다. global position gauge/net clearance·실제 기체 정지·안전한 회수·모터 정지의 증명이 아니다. 추정 상태가 nonfinite이면 정상 PID에 보내지 못했음을 별도 기록하고 harness를 종료한다.

기존 실제 접촉·모터 종료·3초 pad 유지 경로는 변경하지 않는다. wall deadline/floor contact/60° tilt/hold-support 실패는 harness 종료이며 새로운 정지 계약의 완료로 세지 않는다. 실제 무선 연결에서 감독·수동 인계 또는 검증된 착륙 정책을 이어받을 다음 단계는 미구현이다. 시뮬레이션을 닫고도 실기가 계속 호버한다고 주장할 수 없다.

검증 계획은 (1) 전달 이전 종료 금지/연속 관측/실효성 상실·future stamp/3초 상한의 작은 계약 시험, (2) 기존 성공571·619의 입력·명령·모터·물리 로그 byte 동일 회귀, (3) 기존 실패613 timeout·617 uncertainty의 새 full physical 실행에서 실패 이후 wire/PWM/physics와 sensor 관측을 검사하는 것이다. 후보 선택은 이 검증이 끝나기 전 하지 않는다. 모델 매칭/qualified/formal/실기 승인은0으로 유지한다.

초안 검증 기록: 5개 작은 계약 시험은 v2 로그에서 통과했다. 최초 테스트의 마지막 관측 간격을 실수로40ms로 적어 연속성 기준30ms에서 4개 subcase가 올바르게 거절됐다. 테스트를20ms 간격으로 정정했으며 계약 문턱은 바꾸지 않았다. 관측 간격>30ms도 dwell을 초기화한다. 이 초기 helper 오류는 물리 비행 시행이 아니다. 실행 adapter/module/test draft는 artifacts/mission3/acquisition/20261009-abort-delivery-design-v18/에 저장했으며 현 frozen source에는 적용하지 않았다.

실행 수정 기록08:05KST: 최초v18은false인pending분기뒤에서기존mission.update가실행돼실패전달상태를50Hz마다초기화했다. 모든4회와failed-delivery-regression-audit.json/원source를보존했다. v18b는미션갱신조건에abort_delivery is None을추가하여분리를보장하고actualadapter I/O통합시험2개를추가했다. 별도deadline판정은double적산으로3.020999...s가되는반례에서1e-9초clockroundoff허용만추가했다. 상태/landing문턱완화없음;전체211tests는pass지만물리4회v18b는아직진행중이다. 정상positive2원raw10파일동일은최초v18에서만확인됐고새source에서는재확인필요하다. 초안negative617은baseline과같은timeout이라다른실패분기인wide10x3/619 uncertainty로사전계획을명시정정했다.

08:17KST v18b 검증 완료: same4 wholephysics/normalcaps/native17872exact와별도rawaudit모두pass. baselinepositive2의raw10files동일, timeout/uncertaintynegative2는firstABORT전prefix동일과원failure유지,wire273/115·physics2721/1141 이후2.721/1.141s에sensor상.52sstationary관측. 공중PWM/모터는켜진상태이며실기안전회수판정false. 실패v18을포함한전체분모를보존;동일existingreviewer가두source whole8을독립검토중. 전달수정검증을새착륙성공능력으로세지않는다.
