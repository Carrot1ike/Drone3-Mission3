# 실기 입력의 시간 계약과 공식 예제 대안

2026-10-09 읽기 전용 조사. 실제 장비·radio·Wi-Fi 연결·빌드·flash는 실행하지 않았다. 새 외부 파일이나 저장소를 변경하지 않고 공식 코드의 버전을 고정해 로컬 reference만 보존했다. P 제조사 물리 사양, C 공식 소프트웨어 동작, D 계산, S 실험 가정, U 미확정을 구분한다.

## 시뮬레이션의 stamp를 그대로 가져갈 수 없는 이유

현재 plant·센서·PC·영상은 같은 simulation clock이다(S). `v19`는 이 공통 시간축에서 아직 사용할 수 없는 미래 측정과 새 표본으로 재소비된 중복 데이터를 fusion 전에 거부한다. 실제 드론에서는 STM32, GAP8, PC가 서로 다른 시계를 가지므로 원 카운터 숫자를 그대로 비교하면 이 계약을 만족하지 않는다. clock domain·측정/완료/수신 의미를 먼저 정해야 한다.

| 경로 | 공식 코드 C에서 확인한 시각 | 현재 sim / 실제 전환 한계 |
|---|---|---|
| Crazyflie CRTP log | 로그 block 생성 시점의 24bit STM32 tick 시각. 고정 firmware는1000Hz tick, Python은3byte를 정수로 복원한다. | 개별 Flow/ToF 측정의 producer 시각으로 자동 승격할 수 없다. 같은 log block에 있다고 각 센서가 동시에 측정됐다는 뜻도 아니다. |
| Flow | driver에서 `usecTimestamp` 차이로 delta interval을 계산한다. currentMotion과 품질을 별도 주기로 갱신한다. | PC가 last delta를 더 느리게 읽으면 skipped burst를 복원할 수 없다. producer sequence/interval 또는 accumulated delta를 전송하는 소프트웨어 계약이 필요하다. |
| ToF | driver가 읽은 거리와 estimator enqueue tick을 사용한다. | sensor integration 끝/SDK read/enqueue/log/PC receipt는 다르다. 근접 RangeStatus·signal/sigma·반사·capture 정보는 여전히U다. |
| 기본 AI Wi-Fi streamer | packed11byte header: magic/width/height/depth/type/size. 촬영 시각이나 frame sequence 필드가 없다. | PC receipt는 저장할 수 있으나 capture timestamp를 대체할 수 없다. 현재 50ms camera delay/20FPS는S이고 실기 측정값이 아니다. |
| NanoCockpit streamer | metadata version10: frame ID, GAP8 frame completion timestamp, GAP8 state receipt timestamp, STM32 state tick, RTT 관련 필드. | 이미지를 받은 시각만 쓰는 방식보다 시간 정보를 분리할 수 있는 공식 예제 후보다. EOF는 rolling shutter의 모든 행이 동시에 노출된 시각이 아니다. |

Crazyflie library는 실제 설치 소스 commit `3c1cb0d024fef6f45d08f0ad236a22ac395dd924`, firmware는 기존 실행 고정 commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`로 확인했다. 라이브러리의 log unpack과 firmware의 tick 생성·Flow interval은 [원본 SHA 목록](../references/bitcraze-telemetry-clock-20261009/source-manifest-v2.json)에 결합했다. 웹 master를 기존 비행 runtime으로 바꾸지 않았다.

24bit millisecond counter의 주기는 `2^24/1000=16777.216s`, GAP8 uint32 microsecond 표기의 주기는 `2^32/1e6=4294.967296s`다(D). wrap와 역순 packet을 구별하려면 session/reset 정보와 최대 허용 간격이 필요하다. modular 차이만으로 오래된 packet을 새 측정이라고 승인하지 않는다. 실제 GAP time 구현은32kHz reference 기반이라고 설명하며 약30μs 해상도다(C); field가 μs 단위라고 1μs 실측 정확도를 주장하지 않는다.

## 검토할 소프트웨어 경로

[공식 Wi-Fi streamer 문서](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/examples/wifi-streamer/)의 기본 예제와 [보존 header 소스](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/wifi-img-streamer/cpx_stream.c)를 확인했다. 기본 source 설정324×244와 colorized viewer를 우리 native mono320×320 조건의 검증으로 사용하지 않는다. 프로토콜의 width/height만 바꾸거나 PC에서 resize하는 해결책은 채택하지 않는다.

[공식 저장소 NanoCockpit 설명](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/nanocockpit/README.md)과 [metadata 소스](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/nanocockpit/lib/streamer.h)를 읽었다. 이 후보에는 상태 forwarding과 프레임 번호/EOF 시각이 있다. 그러나 기본162×162/60FPS, exposure 설정, NINA 호환 firmware 및 debugger 요구가 현재 지급 조건과 다르다. 즉시 설치·전체 도입할 근거가 아니다. metadata의64-element ToF 구조를 Flow deck에 있는 다중영역 센서라고 해석하지 않으며, 추가 deck를 요구하거나 가상 측정으로 공급하지 않는다.

제안은 시간 관련 정보를 프로젝트에 맞게 명세하고 검사하는 것이다. hardware 변경 없이 소프트웨어만으로 구현할 수 있는지 지급 revision과 허용 flashing 경로를 나중에 확인한다. 우선 packet에 device/session/sequence/raw producer counter·단위·timestamp 의미·PC monotonic receipt를 분리해 남긴다. 공통 시간축 매핑은 offset/drift·전송 지연 불확실성을 갖는 별도 단계여야 한다. RTT 절반을 일방향 지연이나 exposure 시각으로 확정하지 않는다. exposure/readout 구간·IMU/ToF 관측 시각도 각각 보존하고, gap·wrap·reset·지연·역순·해상도 위반을 검사한다.

사용자 차량0.25m/s는50ms 동안1.25cm 이동한다(D). 이는 시간 정보를 무시할 수 없는 크기를 보여주는 예일 뿐 실제 지연이나 추적 오차 추정값이 아니다. 아직 PC에 얻어지지 않은 차량 속도를 이 값으로 대입하는 live feedforward는 허용하지 않는다.

현재 simulation/controller/runtime/해상도/stock mount/착륙 평가를 바꾸지 않았다. source23개는 공식 예제 snapshot과 기존 로컬 pinned 코드의 읽기 전용 근거이며 실행되지 않았다. 처음 추측한 다른 repository 경로404도 discovery 기록에 보존하고 실제 공식 문서가 가리킨 저장소를 사용했다. 독립 검토 중인 v19 회귀와 이 실기 protocol 설계 조사는 별도다. 실제 clock sync·radio/CPX metadata·과부하·rolling shutter·energy·안전 회수는 미검증이다.
