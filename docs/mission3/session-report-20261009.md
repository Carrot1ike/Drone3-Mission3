# 미션3 연구·구현·시험 보고 — 2026-10-09

2026-10-09 개발 결과. 제어 구조와 조건부 물리 착륙은 구현했지만, **제조사와 동일한 실기 환경에서의 착륙 검증과 진입1→8 전체 성공은 완료하지 못했다.** 현재 기준 정책의 사전 미사용 5조건에서는 조건부 접촉 성공2/실패3이었다. 이 결과를 보고 개선한 뒤부터 해당 조건은 개발에 사용한 데이터다. 제조사 일치·qualified·실기 성공은0이며 진입2 이후는 진행하지 않았다.

현재 완료 물리 실행 원장은165회다. 서로 다른 source·정책·training/holdout·실패 구현·회귀를 포함하므로 전체를 하나의 성공률로 합산하지 않는다. v20의 같은 spent4 정상 입력 회귀가 종료되어161→165로 갱신했다. 현재 source의 전체219시험, 네 실행의 평가·포화·native17,872frame 검사와 원 v19 대비10raw×4파일 byte 일치도 통과했다. 전체 접촉 독립 검토는 v19까지 완료했고 v20에는 새 접촉 서명을 만들거나 과거 서명을 복제하지 않았다.

## 구현한 제어 경로

[간단 제어 블록선도](../diagrams/mission3-simple-control.png) · [편집 원본](../diagrams/mission3-simple-control.mmd) · [제어 설명](simple-control-diagram.md).

AI-deck 영상과 IMU/Flow/ToF → 자기 운동·마커 상대 위치·차량 운동 추정 → 단계 판단 → 추정 차량 속도 FF + 상대 위치/속도/yaw 오차 FB → 속도 목표의 포화·변화율 제한 → PC 속도 PID의 RPYT 명령 → 공식 기체 자세·각속도 제어 → 모터·물리 운동의 경로다. 차량 참값·설정 속도는 제어기에 직접 입력하지 않는다. 시뮬레이터가 센서와 차량 운동을 만들고, 접촉 truth는 종료 후 평가에만 사용한다. 실제 radio/ROS 전송·기체 배포 검증은 별도 미완료다.

진입1의 알려진 위치·초기 정면·높이1m에서 15초 대기 → 같은 자리의 제한된 회전 탐색 → 진입 위치와 경기장 경계를 이용한 연속 영상 탐색 → 관측 후 차량 뒤 일직선 정렬 → 카메라 가시성을 위한 높이 조정 → 접근·상대속도 맞춤 → 직선 여부·불확실도·관측 나이 조건을 통과하면 제한된 예측 하강 → 접촉 후보 관측·모터 명령 종료·3초 유지로 구성했다. 마커 관측 전 트랙에 가서 기다리는 동작은 사용하지 않는다. 말기 이미지 소실은 제한된 예측 구간으로 기록하며 관측된 마커와 새 영상이 계속 있다는 뜻으로 표시하지 않는다.

## 유지한 시험 조건

| 항목 | 적용 조건 |
|---|---|
| 공간/트랙 | 실내35×12m·높이5m 그물·무풍, 중앙 트랙 직선20m/회전 지름7.5m, 트랙 전체27.5×7.5m |
| 차량/착륙 | 시작부터0.25m/s 연속 운동, 착륙은 직선만 |
| 패드/마커 | 패드20×20cm·높이30cm, 후향 수직판20×20cm·코드15×15cm, 패드 앞 가장자리부터 마커 면15cm |
| 영상 | native8bit mono320×320, 해상도 변경·컬러 입력·PC resize로 통과시키지 않음 |
| 정상 명령 제한 | 3D속도0.4m/s·가속0.15m/s²·yaw10°/s·yaw가속20°/s²·상승0.10/하강0.06m/s |
| 실제 성공 평가 | 패드 실제 발 접촉·PWM0·차량 위3초 지지·직선 접촉, 근접 정지로 대체하지 않음 |

속도 제한은 명령 제한이며 실제 속도 상한을 보장하지 않는다. 독립 검토한 사례571/619의 실제 최대3D속도는0.40815/0.40531m/s였다. 긴급 ABORT·PWM 종료의 예외도 원 로그에 분리했다. 위치 점수는 실제 첫 접촉의 합성 CoM 중심 오차를 사용한 내부 과녁 지표이며 대회 공식 점수가 아니다.

## 결과를 해석하는 분모

| 시험 | 해당 source·조건에서의 결과 | 판단 |
|---|---|---|
| v14 수직 IMU 보상 창 비교 | 0.8s/1.2s 각각 개발6/6 접촉 | 미리 정한 선택 기준으로1.2s 사용, 일반 성공률 아님 |
| v14 사전 미사용 조건 | 5회 중2회 접촉,3회 탐색 실패 | 607:8점·29.0mm,619:10점·9.4mm; 전5건 독립 검토 |
| v15 탐색 범위 후보 | 두 후보 개발9회씩2/9·4/9 | 기존 성공619 소실, 채택하지 않음 |
| v17 조기 방향 추종 후보 | 기준/후보14회씩8/14·8/14 | 개선 없음, 후보 거절·기준 유지 |
| v18/v18b 실패 명령 전달 | 각4회, 접촉2·실패2 | 최초 전달 구현 결함 보존; v18b에서 관측 종료 경로 수정 |
| v19 시간 입력 검사 | 동일 spent4 중접촉2·실패2,10raw×4가 각v18b와byte동일 | 정상 입력 보존 회귀, 새 효능 표본0; 독립 검토 완료 |
| v20 보존 이력 사전 검사 | 219 CPU tests·동일 spent4 접촉2/실패2·normal caps4·17,872native exact·원v19와10raw×4 byte동일 | 정상 입력 보존 회귀, 현재 source의 새 접촉 독립 서명0 |

[v14 미사용5조건 실제 검토](reviews/20261009-v14-vertical-holdout-contact-review.md) · [v17 전체28 비교 및 검토](reviews/20261009-v17-coarse-bearing-paired-training-contact-review.md) · [v18 실패와 수정 검토](reviews/20261009-v18-abort-delivery-regressions-contact-review.md) · [v19 실제 독립 검토](reviews/20261009-v19-sensor-chronology-regressions-contact-review.md).

v19는 원 실행855파일·이전 판정99파일 불변, native17,872프레임의 controller 입력 SHA 일치, 정상 명령 포화와 접촉·모터·3초·직선 조건을 독립 검사했다. main도 새 서명·원 평가·검토 증거 SHA를 [별도 대조](../../artifacts/mission3/acquisition/20261009-sensor-chronology-regression-training-v19/main-independent-binding-validation.json)했다. 이 검토는 v19 snapshot에만 적용한다.

v19의 내부 이력 주입 반례는 오류 전에 추정 상태 일부가 바뀌었다. v20은 보존 이력과 새 표본 추가의 시각·형상·finite를 공식 deck 융합과 PC accumulator 소비 전에 검사한다. 수정 전20실패를 보존한 새 CPU 시험과 정상 물리 회귀를 분리했다. [v20 사전 계약](../../artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/contract-at-plan.md) · [전체4집계](../../artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/summary.json) · [원40파일 byte 감사](../../artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/whole-raw-chronology-regression-audit.json). 전체 update의 임의 수치·예외 rollback이나 실기 복구까지 보장하는 패치가 아니다.

08:56에 [v20 독립 CPU 패치 검토](reviews/20261009-v20-history-preflight-patch-review.md)가 실제 완료됐다. 원1,318파일·이전103판정 불변, 기존 정확한 이력 반례2개와 추가18개가 상태 변경 없이 거부됐으며 시작·held·정상 fallback을 유지했다. 승인 범위는 이력 오류의 사전 거부다. 별도 NaN 가속도 입력은 여전히 위치를 nonfinite로 만들 수 있다는 반례를 남겼다. 필수 수치 입력 검증과 오류 후 폐기/제어 전달/복구 구현이 다음 software 작업이다. 이번 CPU 검토는 네 물리 회귀의 접촉 독립 승인으로 계상하지 않는다.

## 관측 실패의 구체적 근거

현재 실패의 핵심은 작은 마커와 늦은 관측이다. v17 전체28 원 영상의 후평가에서 양쪽 정책의 같은 실패6개는 최소 모서리 최대값이약5.47~14.06px였고, 원 metric pose 조건18px 이상 후보 frame은0이었다. 무조건 문턱을 낮추거나 차량 시작 phase·마커·해상도를 바꿔 성공을 만들지 않았다.

별도 baseline14 전체 진단에서 실패557/601/613의 height-bearing 상대 거리 정보는23/20/21개만 얻을 수 있었고 첫 관측은handoff 뒤291.0/284.76/281.26초였다. 상대 위치 오차 중앙값23.2/31.0/36.9mm와 달리 기존 자기 세계 위치로 변환한 오차는611/456/499mm였다. 가정한 독립 잡음에 대한 민감도도225/212/279mm다. **상대 거리 한 값이 정확해 보여도 세계 위치·차량 속도·착륙 정렬이 해결되는 것은 아니다.** 이 후보는 후평가용이고 live 추정기에 연결하지 않았다.

[원14분모·표·그림](height-bearing-baseline14-20261009.md). 이미지 공간 상대 제어와 가시성을 포함한 입력 제약 검토는 [Keipour 등](https://arxiv.org/abs/2104.01272), [Falanga 등](https://arxiv.org/abs/1804.04811)의 방법적 근거를 참고했다. 논문 기체·센서·성능을 이 모델에 이식하거나 해당 NMPC를 구현했다고 주장하지 않는다.

## 실기와 아직 다른 부분

[제조사·공식 코드 근거](hardware-sensor-evidence.md)에 P(공개 물리 사양)/C(공식 코드)/D(계산)/S(실험 가정)/U(미확정)를 분리했다. HM01B0 유효320×320와 공칭H/V87°·D115°를 확인했지만 현재 이상적87°핀홀은 대각115°·왜곡·실제K를 동시에 재현하지 못한다. 실제 rolling shutter에 비해 현재 순간 frame·20FPS·50ms·PSF·readout은S/U다. stock 정량 장착 변환, 지급 revision, 조립 질량·관성·CoM·모터 응답·마찰·접촉·배터리 잔량도 미확정이다. 원 null을 제조사 값으로 꾸며 채우지 않았다.

Flow는 관측 표면에 대한 상대 운동을 측정하고 ToF는 표면 거리를 측정한다. 바닥→차량 패드 전환에서 이를 세계 속도·바닥 높이로 그대로 융합하면 문제가 생긴다. surface-aware 상위 추정기와 독립 PC IMU 자세를 분리했지만, 공식 onboard EKF에 들어가는 근접 raw ToF의 영향·교차상관은 해결하지 못했다. 접촉 사례 HOLD에서 정확한ToF·유효Flow 표본은0이며 IMU/모델 잔차가 실제 네 발 접촉을 확증하지 않는다. 네 발 지지는 별도 시뮬레이션 truth 평가로 확인했다.

실기 시간 동기화도 다르다. 기본 공식 AI 스트리머 header에는 capture timestamp가 없고 CRTP log stamp는 센서별 capture 시각이 아니다. [공식 NanoCockpit 예제](https://github.com/bitcraze/aideck-gap8-examples/blob/d39d2ec5001f94bf774dbd1187ae4cf869a3094d/examples/other/nanocockpit/README.md)의 EOF·frameID·GAP8/STM32 시각은 참고 후보지만 기본162×162/60FPS·펌웨어 조건이 달라 도입하지 않았다. [시간 전송 계약과 source23개 SHA](hardware-telemetry-time-contract-20261009.md)를 남겼다. 다운로드한 코드를 실행·flash하지 않았고 외부 인터넷 자원을 변경하지 않았다.

실패 시0속도/yaw 명령을 하위 PID·모터·physics까지 보내고 센서 안정 관측으로 종료하는 경로를 수정했으나 실radio ACK·RTOS·supervisor·링크 손실 회수는 미검증이다. 관측 종료 시 공중 모터가 켜져 있으므로 안전 착륙·회수로 부르지 않는다. 시각 오류를 거부하는 ValueError 역시 실기 복구 정책이 아니다. 일정4.2V 모델은 세 미션 연속 비행의 energy/SOC·payload endurance를 검증하지 못한다.

## 영상과 다음 검증 순서

[전체1× 영상 — v14 seed571 조건부 접촉 예시](../../artifacts/mission3/replays/20261009-imu12-v14-571-full/full-flight-1x-h264.mp4) · [다른1× 예시 — v13 seed541](../../artifacts/mission3/replays/20261009-entry-scan15-v13-541-full/full-flight-1x-h264.mp4). 두 영상은 표시한 과거 source의 실제 로그 재생이며 v20 비행 영상이나 실기 영상으로 바꾸어 부르지 않는다. 원 native camera는 무손실 FFV1·capture SHA로, 사람이 보는 재생 영상은 H264로 보존했다.

다음 작업은 작은 마커의 관측 정보와 자기 위치 gauge를 분리하는 상대 추종 설계를 먼저 개발 조건에서 비교하고, 남긴 성공 보존 기준을 통과한 후보에만 새 미사용 조건을 적용하는 것이다. 함께 실제 영상/Flow/ToF producer 시각·sequence·arrival 의미와 오류 거부 이후 제어 전달 계약을 구현해야 한다. 그다음 K/D·stock 장착·rolling shutter·근접 sensor·추력/접촉/잔량 불확실성을 포함한 시험을 수행한다. 제조사 일치와 독립 검토를 통과한 진입1 접촉·모터 종료·3초 유지5회를 확보한 뒤에만 진입2→8→임의 지점으로 확장한다.

전체 세부 실행·실패·수정·진행 프로세스 기록은 [work-log](work-log.md)에 보존했다.

