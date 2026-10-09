# v14 imu12 holdout5 독립 접촉 검토

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. **사전 미사용601/607/613/617/619 전5건을 검토했고607·619의 조건부 모델 접촉만 승인한다(2/5).** 601·613·617은 각각 실제접촉0/PWMoff없음/metric0인 탐색 실패로 거절한다. training imu12 6/6·imu08 6/6 및 v13/v11과 합산하지 않는다. 제조사 일치·실기·qualified·formal0, 진입2·후속시험 승인0이다. 이5개 seed는 결과를 본 현재부터 spent data다.

원 [corrected plan](../../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-holdout-v14-v2/plan.json) SHA `11c09294ed2a7f1e367a69542dd46e7b2fb2d5e1d84c54872d3cac675f6490b5`와 [완료 summary](../../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-holdout-v14-v2/summary.json)를 원 각 run의 source-snapshot/평가/raw로 확인했다. main 자체 판정이나 training 서명을 복제하지 않았다.

## 전5분모와 개별 결과

시각은 simulation 시작 기준s이며 hold끝=PWMoff+3.00s, 원 종료는 추가0.02s 후다. 내부점수는 실제 첫 접촉의 합성 aircraft CoM 기준이다.

| seed | decoded / metric수락 | 실제접촉 / PWMoff / hold끝 | 원 종료 | 본인 판정 | 원해시 결합 기록 |
|---|---:|---:|---:|---|---|
| 601 | 20 / 0 | — | 309.64 | 거절: metric0·접촉0·off없음 | [검토 기록](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed601/failure-review.json) |
| 607 | 483 / 479 | 127.60 / 128.48 / 131.48 | 131.50 | 조건부접촉 승인 / 8점 / 29.041mm | [검토 기록](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed607/signedreview.json) |
| 613 | 21 / 0 | — | 309.60 | 거절: metric0·접촉0·off없음 | [검토 기록](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed613/failure-review.json) |
| 617 | 0 / 0 | — | 309.70 | 거절: metric0·접촉0·off없음 | [검토 기록](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed617/failure-review.json) |
| 619 | 682 / 621 | 243.76 / 244.64 / 247.64 | 247.66 | 조건부접촉 승인 / 10점 / 9.375mm | [검토 기록](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed619/signedreview.json) |

실패601/613/617 모두 Mission3 handoff 뒤300.02초에 `search_timeout_without_metric_pose`로 끝났다. 물리 total309.64/309.60/309.70s와 탐색300s 제한을 구별한다.601의20 decoded 코너 최소변7.397~10.676px,613의21코너7.584~10.002px는 원18px 문턱 미만이며 이유는 `marker_too_small`다.617은 ID decoded0이다. 일부 invalid coarse 관측이 있어vehicle record가 완전히 None이라고 주장하지 않는다. 접촉·지원·근접 높이 확인 단계에 도달하지 않은 실패이므로 vertical fit을 바꿔 모든 탐색 실패를 해결했다고 할 수 없다. 세 건의 sensor/estimate-only FSM 재생은 원phase와 일치했고 실제truth 접촉도0이었다.

개별 [601 실패 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed601/failure-audit.json), [613 실패 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed613/failure-audit.json), [617 실패 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/seed617/failure-audit.json)에 모든 검출 시각·이유·크기, abort급변 예외, 원wire/newstamp를 보존했다. 원문턱/검색시간/phase/차량 시작조건을 넓혀 통과시키지 않았으며 승인 서명이 없다.

## 두 접촉 원자료 재검사

607·619 각각 원196개 contact truth표본, motoroff부터151개 50Hz truth표본 모두 네 발 pad 지지/양의 법선력,3001개 1kHz PWM/output/staticforce0이었다. 비발·vehicle body·floor·net 충돌0, 모든 접촉/hold는 실제직선, 차속은 전비행 연속.25m/s다. 이 단순 rigid-body/foot/friction 모델의 결과다.

| 항목 |607|619|
|---|---:|---:|
| hold 발별 최소법선력N |.028310|.033200|
| hold 최대 상대미끄럼µm |14.058|4.567|
| hold 최대실제tilt° |.000353|.000108|
| PWMoff 순간 실제 rotor force총N |.201824|.202167|
| off+.3s force총N |6.174e-8|6.184e-8|
| off 추정 pad위 높이mm |15.636|14.042|
| off 마지막 qualified image age s |5.44|5.30|
| off handoff 경과s |4.40|4.22|
| off calibration age s |20.10|19.96|
| 실제 최대3D speed m/s |.401856|.405305|

원 strict 높이14.5±12mm, 지원확인.25s, IMU near-g±.2/gyro·tilt, low expectedforce6.5, calibration45s 및 terminal prediction6s를 유지했다. raw bodyIMU+명령 PWM곡선/가정 lag의 전체 support/calibration 재생이 원로그와 일치했고 candidate 최초시각도 각 PWMoff와 일치했다. 원13.60/13.50s preparation은 planning12초를 넘으므로 강제durationcap이라고 부르지 않는다. main unsigned development의 비교한접촉/hold/state필드와 불일치0이었다.

HOLD의 validFlow와 accurateToF는 각각0개이며 surface는301개 소비표본 모두 ambiguous다. IMU stable만으로 실제 pad·네 발·미끄럼을 확인할 수 없어 본인은 오프라인 물리 contact를 별도로 확인했다. PWMoff는 기계적 즉시 force0가 아니며 가정20ms lag에 따른 실제잔류힘을 기록했다.

## 소스·config·native·시각

원 실행 코드62/runtime10/plan core65(config포함)는 전5에서 원 manifest SHA와 일치하며, 선택된 training imu12와 source/runtime/frozen config가 동일했다. frozen config SHA `bba87b2cfeb71cafb4083d7386cc5ac8991337d9d6cb5d9807968d1bd11d7da7`; seed/entry/split/series는 각 experiment.json의 새로운 trial_metadata SHA로 결합됐다. 모든 평가를 원 snapshot evaluator SHA `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0`로 다시 계산해 원파일 모든 필드와 일치했다. 현행 v15 코드/같은 seed 후속비행은 읽어 source로 대체하거나 이번분모에 넣지 않았다.

주요 원source SHA: estimator.py `aee18061e2ea283e943adc285837d8124fb4a54e4e1a8835a03a1f459e0eb668`, vertical_fit.py `35f6587922b0aaeded60d46d3ee1949a6ae7a9df677853ef3172d888e9d8f5e0`, physical_estimator.py `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5`, controller.py `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180`, touchdown.py `b258eeea56689f03c7ff1c258c1703f666b514c3ff34761f10856d8f9b239f7b`. [전5 source/config/phase 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/source-series-verification.json).

**24,758개 전체 native mono320 FFV1 프레임**은 원 controller_input SHA와 정확히 일치했다(601/613/617 각6001,6072221,6194534). 실제607/619 및601의 first/middle/last 원픽셀 sample도 직접 확인했다. 원영상이 실기 PSF/초점/AI-deck 전송을 검증한다는 뜻이 아니다.

Flow100Hz/ToF40Hz/newflag/producer값은 전5에서 일치했다. 소비 시각은 순서대로이며 ToF capture age0~20ms, 미래/중복 사용0건이었다. 기록된 surface/image acceptance를 고정한 수직 산술 재생의 Z/vZ 오차는 전5에서0, fit 총 51,822회/fallback0이었다. 센서·추정값만 사용하고 재생 산술 검증 뒤에만 truth를 비교했다. [전5 수직 시각·재생](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/vertical-fit-all-cases.json).

## 첫 CLI 실패와 미사용 근거

원 plan SHA `518f41430fcf7ec68bb1926070f9f8a89b227f5b92236fbc9c2d5e932505c214`의 --split unused_holdout은 CLI의 training/holdout choices에 없어5건 모두 exit2였다. 원5log/execution을 읽었고 snapshot의 parse_args가 run()/plant/output 생성보다 앞임을 확인했다. 따라서 preflight거부5와 실제물리5회는 별도다. 후에 생성된 동일 run 경로는 corrected launch의 결과다.

v2는 각case split/argv를 holdout으로, 로그를 retry 이름으로 바꿨고 다른 실행 인자·seed/source/config/scene는 같았다. 계획생성시각과 correction metadata도 명시적으로 바뀌었음을 기록했다. recorded manifest inventory173개 모두 원 SHA/seed와 일치하고,600이상 첫5prime601/607/613/617/619가 그 사전목록에 없었다. 미사용 증명 범위는 그 기록된 inventory이며 전세계/다른 작업의 통계적 독립성을 주장하지 않는다. 지금 같은 seed의 후속 v15 trainingmanifest가 있더라도 원 사전선정의 소급 변경은 아니다. 초기에 본인 감사가 현행 selectedseed manifest5개뿐이라고 잘못 가정해 실패했고 원log를 남긴 뒤 원plan5개와 후속 사용을 분리한 v2검사를 완료했다.

[CLI·사전 seed 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/preflight-and-predeclared-seed-audit.json)에 원계획/원오류/새계획/훈련 선택 SHA를 연결했다. 원 물리누적95→100과 preflight5거부를 섞지 않는다.

## 비판·실행 가능한 후속검사

training6/6은 이번 holdout2/5로 일반화되지 않았다. 두 착륙은 terminal조건을 만족했지만 세 실패는 관측획득 예산 문제다. 후속에는 원 native 코너/pose/no-ID 원인과 sensor-only 불확실성·알려진 entry/arena 기하로 탐색 정책을 별도고정해 시험하고,18px/metric/std/300s 문턱을 낮춰 기존실패를 성공으로 재평가하지 않는 것이 타당하다. 새정책에 이5를 쓰면 training이고 새로운 미사용 시험은 따로 필요하다.

원source의 미래 IMU API 반례와 legacyfallback 정책 문제는 여전히 있다. 실제5건 미래IMU는0이지만 함수 단독으로는 now뒤 right-end 가속도를 받아 결과가 달라진다. 외부packet에서 causal trim/거부, timestamp/finite/duplicate 검증을 먼저하고 fit 실패 시 fusion 생략/불확실성 전파 정책을 명시적으로 시험해야 한다. fit의 .12높이 갱신은 검사보다 먼저이고 fallback은 globalrangegate가 아니다. heldToF의 producer시각과 소비현재 tilt/lever 변환 차이 최대20ms, 실제 IMU averaging/capture/RTOS/radio시각 U를 남긴다. [독립 함수 반례](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/scalar-fit-independent-fixtures.json).

onboard rawToF lowergate0은 계속4cm 아래 값을 융합하고, PC독립 IMU 자세와 fullonboard deck context는 분리돼도 plant feedback 경로까지 독립인 것은 아니다. near qualityflag는 가상 이상 ray/4mmnoise 기반 S이며 실제 RangeStatus/PMWquality가 아니다. [근접센서 경계](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/near-range-observer-scope.json).

607 최종 own world오차94.55mm와 relative pad오차130.22mm는 각각 다르고 실제접촉 오차29.041mm/실제slip14.058µm와도 다르다. 공통 gauge는 상대식에서 상쇄되지만 blind 상대속도/pose 전파 drift는 남는다. HOLD 지원이 그 drift·pad identity를 보장한다고 말하지 않는다. 실제속도 .4019/.4053과 명령cap .4도 구별한다.

장착/K/D/CoM·질량·관성·추력·lag·finitepatch 반사·마찰/접촉·PSF·capture/전송·보드revision은 S/U다. 상수4.2V의 battery는 energy/SOC·Mission1/2 잔여시간을 검증하지 않는다. 공식 component 최소거리나 nominal angular envelope를 조립된deck 응답/캘리브레이션으로 격상하지 않는다. scalar fit은 편향 추정기·Forster manifold VIO가 아니다.

[새서명2·실패감사3 index](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/signed-review-index.json) / [최종 원파일·결합 검증](../../../artifacts/mission3/reviews/20261009-v14-vertical-holdout-independent/final-verification.json)에 원521파일·기존37서명 불변과 새검토의 원평가/input/source/evidence SHA를 기록한다. 신규2접촉 승인만 있으며 qualified/formal/hardware0, entry2 승인0이다.
