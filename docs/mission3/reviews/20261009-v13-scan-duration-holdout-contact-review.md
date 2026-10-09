# v13 scan15 holdout5 독립 접촉 검토

Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`가 원 실행 저자와 별도로 검토했다. 계획 당시 미사용이었던557/563/569/571/577 중 563·577 두 건은 조건부 물리 접촉 계약 승인, 557·569 검색 실패와 571 모터 미종료 접촉은 승인하지 않는다. 전체5 분모를 유지한다. 이제 관측한 데이터이므로 후속 개발에 쓰면 spent data다. training scan15 5/6·scan30 4/6 및 v11/v12 결과와 합산하지 않는다. 제조사·실기·qualified·formal·진입2 승인은0이다.

원 [사전 계획](../../../artifacts/mission3/acquisition/20261009-entry-scan-duration-holdout-v13/plan.json)의 “550 이상 첫5 소수/기록된 physical manifest seed inventory에 미사용” 선택을 수치 확인했다. 선택값557·563·569·571·577은 계획 inventory에 없고 현재 physical-root의 해당 seed 실행도 이 holdout 다섯 건이었다. [확인 근거](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed-selection-check.json). 기록된 실험 inventory 범위의 확인이며 전체 가능한 시행/모집단의 독립성을 증명하지 않는다. 선택된 profile은 이미 본 training 결과로 고른 scan15이고, 이 다섯 결과를 보고 사전 선택했다고 꾸미지 않는다.

| seed | 원결과 | 첫 실제 접촉(s) | PWM off / 실행 끝(s) | 내부 점수 / XY(mm) | 본인 판정 |
|---|---|---:|---:|---:|---|
| 557 | decoded34·metric0·search timeout | 없음 | 없음 / 309.72 | 없음 | 실패 감사 / 승인 없음 |
| 563 | 접촉·정상 caps | 232.58 | 233.44 / 236.46 | 9 / 12.904 | 조건부 접촉 승인 |
| 569 | ID0·metric0·search timeout | 없음 | 없음 / 309.34 | 없음 | 실패 감사 / 승인 없음 |
| 571 | 접촉127표본·prediction expiry | 109.92 | 없음 / 116.58 | 접촉 오차는 미션 통과가 아님 | 실패 감사 / 승인 없음 |
| 577 | 접촉·정상 caps | 125.26 | 126.18 / 129.20 | 7 / 31.915 | 조건부 접촉 승인 |

각 원 `source-snapshot`의 code60/runtime10 bytes, 계획 core63, 원 trial metadata·frozen config·원평가·commands/observations/estimates/deck_samples/sensors/motors/RPYT/truth를 대조했다. 다섯 source/runtime/config는 선택된 training scan15와 같다. frozen config SHA `6f7e1dd85d6c9d79d0484db30d2095dab847c267b47b82ea110b4d2f8c4761c9`; 새 trial entry/seed SHA는 따로 결합했다. 스냅샷 밖 부가 config 원파일은 계획 SHA와 맞는 bytes만 확인했다. raw mission 설정에 profile의 software/optics override가 적용된 public controller-settings도 따로 확인했다. [전체 동결·평가 재계산](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/source-series-verification.json). 원평가 다섯 건 모두 같은 evaluator와 원 상대 run_dir로 재계산한 모든 필드가 일치했다.

WAIT15·진입 높이1·초기 scan15 및 entry-forward 시선, strict pose/std/contact, 명령 포화, 원 장면·차량 연속 운동과 seeded phase 선택을 유지했다. 실제 scan은 제어 tick 때문에15.02s다. WAIT/SCAN은 진입 위치 오차를 보정하는 작은 속도 명령이 있으므로 실제 운동0을 주장하지 않는다. source는 optional v12 raw range 하한 필드가 있는 버전이며 본 holdout에서는 기본0이다. 원 v11과 source bytes가 같다고 하지 않는다. `physical_estimator.py` SHA `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5`, controller SHA `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180`, evaluator SHA `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0`이다. 현행 수정 코드를 과거 실행 대신 검토하지 않았다.

563·577은 각3초 hold 참값151/151 모두 네 발 pad 접촉·양의 법선력이며, PWM/output/static-force 1kHz3001개 모두0이었다. 법선력의 발당 최소는 .033517/.029825N, off 위치 대비 최대 XY slip은6.279/15.470µm다. 다른 표면·비발 접촉은 없고, 선언된 하강부터 hold 종료까지 직선이며 차량 속력은 .25m/s다. 모터 곡선 및 가정20ms 힘 재귀를 직접 대조했다. PWM off 첫 step의 실제 합력 약.20N이 남았다가 .3초 후 약6×10⁻⁸N으로 감쇠하므로 PWM0와 기계적인 즉시 정지를 구분한다.

563·577의 raw IMU/PWM support 전체 재생은 scalar오차0과 calibration stamp 일치, sensor-only 접촉 후보는 실제 off와 일치했다. off calibration age는19.89/19.65s, 마지막 qualified image age5.30/5.12s였다. sensor snapshot 뒤 영상 높이 갱신이 다음 calibration 허용에 영향을 줄 수 있는 실행 순서도 재생에 반영했다. 추정 off 높이는 pad 위11.061/18.756mm로 S foot/CoM gate 안이었다. HOLD의 정확 ToF·유효 Flow는0이며 IMU/모터 모델 안정성으로 pad 신원이나 네 발을 증명할 수 없다. 접촉 승인은 별도 오프라인 참값 평가로 한정한다.

다섯 명령 모두 정상 .4m/s·.15m/s²·yaw10°/s·20°/s²·상승.10/하강.06m/s 통과를 재확인했고, 긴급 off/recovery/ABORT 전환은 예외로 그대로 기록했다. 563·577 실제 최대 속력 .409123/.409579m/s는 명령 .4 한도를 넘는다. 준비 실측13.46/13.40s도 계획 가정12s보다 길다. 실제 직선 접촉은 확인됐지만 실기 운동 한도나 단계 시간 보장으로 승인하지 않는다. 양성 unsigned 개발 review와 대조한 hold·센서·world오차 필드는 불일치0이며 그 문서의 판정을 복사하지 않았다.

FFV1 557/563/569/571/577 각각6001/4320/5749/1793/2185프레임, 총20,048장을 직접 mono320 controller-input SHA와 대조해 원픽셀 모두 일치함을 확인했다. Flow100Hz/ToF40Hz producer stamp·새 range의 단회 소비·capture/delivery 기록·RPYT100Hz 14byte payload 부호/PWM stop를 대조했다. 이는 실제 AI-deck/HAL/radio/RTOS/PSF 검증이 아니다. [원픽셀 검사 예](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed577/independent-FFV1-validation.json).

557은 decoded corner34개가 있어도 metric0이다. 기록된 코드 짧은 변8.19~12.67px, 원18px metric gate 미달이었다. COARSE_ACQUIRE와 재탐색 뒤300초 미션 검색 budget에 도달했다. 569는ID0/metric0으로 같은 budget에 종료됐다. 둘 다 실제 접촉0/off없음이며, sensor-only controller 재생이 원 phase와 일치했다. 작은 코드가 보인다는 이유로 pose/std gate를 낮추거나 무인식을 성공으로 바꾸지 않는다. [557 실패 감사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed557/failure-audit.json), [569 실패 감사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed569/failure-audit.json).

571은 109.92s부터 실제 pad 접촉127표본, 네 발 양의 지지96표본을 갖고 모두 직선이며 다른 표면/비발 접촉은 없다. 하지만 접촉 후 PWM 전부0인 표본은0이고 stop packet/off없음이다. 원 예측 종료111.74s는 마지막 qualified image 이후6초, handoff106.90s 이후4.84초다. 정상 calibration freshness나 실제 접촉만으로 off를 허용하지 않는 원 계약은 유지됐다.

571의 raw IMU/PWM support 전체 재생은 gain·expected force·specific force·residual scalar오차0과 calibration stamp 일치다. 별도 sensor-only gate 재생에서 모든 gate가 동시에 true인 것은110.66~110.86s의11개 제어 표본, 시간 폭.20s였다. 다음110.88s에는 추정 높이26.615mm가 S 상한26.5mm를 넘는다. support_since110.66→gate이탈110.88까지.22s로 원 confirmation .25s에 못 미쳐 후보0이었다. 이 패턴은 v11/457의 “두 gate 교집합0”과 같지 않다. [571 실패 감사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed571/failure-audit.json), [시간별 gate CSV](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/seed571/failed-support-gates.csv).

110.88s의 실제 body 높이는 pad 위13.565mm, 네 발 양의 지지4개, 실제 합산 법선력.14266N이지만 추정 높이는26.615mm였다. 이어111.72s 추정31.714mm/실제13.565mm다. 이는 오프라인 비교이며 actual height를 제어기에 주입하지 않았다. 해당 순간 다른 support gate는 통과한다. 관측된 높이 상승의 마지막 영상/ToF anchor, 초기 vz, IMU 중력보정·적분 각각의 기여는 미분해다. guard를 넓히거나 .25s를 줄여 이 실패를 받아들이는 것을 권고하지 않는다. 같은 입력에서 last qualified 높이/vz와 수직 IMU 순수 전파를 대조하고, 별도 paired 시험에서 오차 예산을 확인하는 것이 타당하다. 6초 예측을 연장하는 처방도 하지 않는다.

raw range gate0은 근거리 측정의 onboard 융합을 막지 않는다. 공식 mm_tof의 stationary-ground Z 기준과 pad 표면 거리 해석, H의 Z-only 시작 이후 crosscovariance·error attitude finalize 경로는 계속 남는다. 별도 PC IMU context는 deck를 직접 받지 않지만 하위 RPYT PID의 full onboard 자세가 plant와 다음 IMU에 영향을 줄 수 있다. qpos/qvel aircraft는 초기화 뒤 강제로 고치지 않았고, 차량 처방 운동만 업데이트한다. 센서 생성은 조건부 plant state를 쓰며 contacttruth는 오프라인 평가와 명시된 floor/tilt60 harness 중단에만 쓰고 support/controller 입력에는 쓰지 않았다. real supervisor 증거는 아니다.

조립 mount/K/D·질량/CoM/관성/마찰·공통 힘모델/lag·IMU calibration 잔류bias·Flow texture/finite patch·ToF RangeStatus/PSF·radio timing·battery4.2V 에너지/SOC·실제 Mission2 인계는 U/S로 남는다. 작은 training 결과로 고른 scan15가 이번 미사용5에서2건만 완료했다는 사실을 보존하며 일반 성공률·제조사 일치·진입2 허가로 격상하지 않는다. 원자료·원평가·공용source/config/work-log·기존 서명은 불변이다. 신규 [2개 접촉 서명과3개 실패 색인](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/signed-review-index.json), [최종 불변/결합 검사](../../../artifacts/mission3/reviews/20261009-v13-scan-duration-holdout-independent/final-verification.json)를 제공한다.
