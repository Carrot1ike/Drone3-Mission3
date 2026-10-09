# PC 센서 전달과 실제 로그의 차이

2026-10-08 로컬 제조사 원문 대조. 현재 물리 adapter의 PC100Hz 평균 IMU·정확 sensor stamp·zero-delay PWM telemetry는 S이며 실제 기본 radio 로그와 동일하지 않다. 기체/덱 개조는 전제하지 않는다. 필요한 timestamp·평균화 확장은 허용된 소프트웨어 변경 후보이며 아직 실기 펌웨어로 구현·검증하지 않았다.

원문은 공식 firmware commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`, 프로젝트 공식 CFLib checkout commit `3c1cb0d024fef6f45d08f0ad236a22ac395dd924`다. 앞선 RPYT serializer 검증의 다른 고정 CFLib snapshot과 혼동하지 않는다.

| 공식 필드 / 원문 | 뜻과 시뮬레이션 이식 경계 |
| --- | --- |
| `acc.x/y/z`, `stabilizer.c:617` | float, 단위g. PC SI값은9.81을 곱해야 함. 로그는 해당 시점 sensorData이며 시뮬레이션의1kHz 열 표본 평균과 동일하지 않음 |
| `gyro.x/y/z`, `stabilizer.c:677` | 필터 뒤 float deg/s. PC rad/s 변환 필요. 실제 필터·startup bias처리와 가상의 작은 post-calibration 잡음은 별개 |
| `motor.m1..m4`, `motors.c:1061` | uint16 실제 commanded PWM ratio. 요청값 `motor.m1req..m4req` int32와 구분. 실제 rotor 힘·RPM 측정은 아님 |
| `motion.deltaX/Y`, `flowdeck_v1v2.c:262` | 센서 원시 int16. 공식 드라이버의 기체측 flow는 `(-deltaY,-deltaX)` 변환. 현재 PC 가상 raw flow는 이미 해당 기체측 규약이라 로그를 직접 대입하면 축이 틀림 |
| `motion.motion/squal/shutter` | 실제 품질 관련 원시 정보. generic texture-correlation score와 서로 대체할 수 없음 |
| `range.zrange`, `range.c:91` | uint16mm. `rangeSet`은 metre×1000을 저장. 읽는 시각과 ToF 측정 완료 시각은 같다는 보장이 없음 |
| `pm.vbat`, `pm_stm32f4.c:511` | floatV. 현재4.2V 일정 S는 실제 배터리 변화를 재현하지 않음 |
| `log.c:564` / CFLib `log.py:627` | 24bitms logger stamp. 측정 capture stamp가 아니라 block 획득 시각. wrap주기는2^24ms=16777.216초이며 unwrap·재시작 처리가 필요 |

`flowdeckTask`는 nominal10ms delay 뒤 값을 읽고 `usecTimestamp` 차이로실제 dt를 만든다. 그러나 `motion.deltaX/Y` 로그에는 그 flow dt와 sample sequence/stamp가 없다. log block을100Hz로 읽었다고 동일 sample의누락/재사용/정확dt를 구분할 수 없다. 값이 같다는 이유로중복 sample로 판정해서도 안 된다. ToF도40Hz 준비기간과 logger100Hz의sample identity를 구분해야 한다.

## 패킷 배치 계산

공식 `crtp.h`의 data최대는30B, `log.c`/CFLib의blockpayload최대는26B다.4B는blockID와timestamp다. 아래는 배열 저장값에 대한D계산으로 radio 성능 검증이 아니다.

- blockA: acc3float12B +gyro3float12B =24B,10ms 요청.
- blockB: motor4uint16 8B +Flow delta2int16 4B +motion1B +squal1B +shutteruint16 2B +rangeuint16 2B +vbatfloat4B =22B,10ms 요청.
- 두 block 모두26B 한도 안이다. 평균IMU·Flowseq/dt·ToFseq/stamp 등 새필드를 더하려면 재배치/추가block/compact표현을 검토해야 함.
- header1B와logtimestamp/ID4B 포함 blockA29B +blockB27B를100Hz로전달하면5600B/s, RPYT14B+CRTPheader1B를100Hz면1500B/s다. 합계7100B/s=56.8kbps는D인명목애플리케이션계산이다. ACK/poll/재전송/USB·radioheader/공유채널·지연이 빠져 있어 실제100Hz 전달 보장은 아니다.

CFLib legacy log 요청은 `period=int(period_in_ms/10)`로 만든다. timestamp가 같은 두 block의callback이 동시에 오거나 모든field가동일sensor epoch라는 보장도 없다. PC는 각필드의stamp/age/sequence를 유지하고 frame·IMU·Flow·ToF·PWM을 무조건같은시각으로합치지 않아야 한다.

## 다음 이식 검증

1. stocklog의 snapshot100Hz와현재100Hz평균가정을별도profile로시험하고startup/LPF/축/단위를원문경계로검증한다.
2. 필요하다면기체software에서평균IMU·Flowmeasurementdt/sequence·ToFmeasurementstamp를정의한compacttelemetry를구현한다. 구현후deadline·buffer·실제전달지연을측정한다. 현재무선기체없어서실행성능U다.
3. 기체clock과PCclock의offset/drift·loggerwrap·restart·packetloss/out-of-order를테스트한다. AI-deck기본영상헤더에는capturetimestamp가없어영상-PCattitude시각정렬도별도U다.
4. 오래된PWM/IMU를접촉후보허가에쓰지않는다. lowerdecoder/stocksupervisor/linktimeout과PC명령의정지경로를별도검증한다. 현재host는decoder/PID/mixer만재현하며전체RTOS/supervisor안전성을재현하지않음.

현재 rawsensor cadence/전달S로얻은조건부접촉을stocklogs+CFLib를바로연결한실기제어검증으로표시하지않는다. [하드웨어근거](hardware-sensor-evidence.md),[현재구현](implementation.md),[RPYT명령계약](rpyt-command-contract.md)이각경계를기록한다.
