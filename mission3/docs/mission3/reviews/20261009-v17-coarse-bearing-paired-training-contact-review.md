# v17 coarse bearing: 원 paired training28회 독립 검토

검토자 **Heisenberg / Codex-Independent-Flight-Reviewer**, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 기존 담당이 각 run의 source-snapshot, 고정 config/runtime, 원 evaluation/native/raw logs를 새로 검토했다. 이전 승인 서명이나 main의 개발 판정을 새 독립 서명으로 복제하지 않았다. 공용 source/config/tests/work-log·원자료·기존 서명을 바꾸거나 새 물리 실험을 실행하지 않았다.

**baseline8 조건부 접촉8/14·실패6/14, coarse40bearing035 조건부 접촉8/14·실패6/14.** 신규 contact 서명16개와 negative 감사12개를 개별 source/input/evaluation SHA에 결합한다. 사전 retention8개는 모두 통과하지만 strict improvement0으로 후보 채택false/새 holdout 없음이라는 원 whole-series 선택과 일치한다. 이28회는 전부 spent training이며 성공률 일반화·제조사 일치·실기·qualified/formal·진입2 승인은0이다. 기존121→149 fullphysics는 장부 참조이며 v14/v15/v16 및 holdout 분모를 합치지 않는다. 전28검사 완료 후에만 선택 조건을 검증했으며 중간 효과/판정을 변경하지 않았다.

[원 사전계획](../../../artifacts/mission3/acquisition/20261009-coarse-bearing-paired-training-v17/plan.json) SHA256 `a5afa00e6e1f78dcaabec40ccfbe696c4c24d20d03f98eb8da4e4bd2eec9a173`; [전체 분모·retention·선택 검증](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/whole-policy-denominators-retention-selection.json); [새 개별 서명·실패 인덱스](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/signed-review-index.json); [final-verification](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/final-verification.json).

| 정책 / seed | first metric receipt / accepted | 실제 first contact / PWMoff / 3s hold끝 (s) | 원 평가·본인 판정 | 원자료 / 새 검토 |
|---|---:|---|---|---|
| baseline8 / 521 | 없음 / 0 | contact0 / off없음 (종료309.60) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed521/failure-review.json) |
| baseline8 / 557 | 없음 / 0 | contact0 / off없음 (종료309.62) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed557) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed557/failure-review.json) |
| baseline8 / 569 | 없음 / 0 | contact0 / off없음 (종료309.58) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed569) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed569/failure-review.json) |
| baseline8 / 601 | 없음 / 0 | contact0 / off없음 (종료309.64) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed601) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed601/failure-review.json) |
| baseline8 / 613 | 없음 / 0 | contact0 / off없음 (종료309.60) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed613) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed613/failure-review.json) |
| baseline8 / 617 | 없음 / 0 | contact0 / off없음 (종료309.70) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed617) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed617/failure-review.json) |
| baseline8 / 433 | 157.08 / 514 | 187.04 / 187.92 / 190.92 | 조건부 승인 9점/11.798mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed433) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed433/signedreview.json) |
| baseline8 / 467 | 123.24 / 495 | 152.58 / 153.50 / 156.50 | 조건부 승인 10점/8.055mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed467) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed467/signedreview.json) |
| baseline8 / 479 | 180.92 / 549 | 212.70 / 213.58 / 216.58 | 조건부 승인 10점/8.825mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed479) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed479/signedreview.json) |
| baseline8 / 541 | 72.98 / 532 | 103.62 / 104.48 / 107.48 | 조건부 승인 8점/22.808mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed541) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed541/signedreview.json) |
| baseline8 / 571 | 72.50 / 584 | 106.26 / 107.16 / 110.16 | 조건부 승인 9점/14.585mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed571) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed571/signedreview.json) |
| baseline8 / 577 | 96.46 / 470 | 124.86 / 125.76 / 128.76 | 조건부 승인 8점/28.588mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed577) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed577/signedreview.json) |
| baseline8 / 607 | 98.96 / 479 | 127.60 / 128.48 / 131.48 | 조건부 승인 8점/29.041mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed607) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed607/signedreview.json) |
| baseline8 / 619 | 208.40 / 621 | 243.76 / 244.64 / 247.64 | 조건부 승인 10점/9.375mm | [run](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/baseline8-seed619/signedreview.json) |
| coarse40bearing035 / 521 | 없음 / 0 | contact0 / off없음 (종료309.60) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed521) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed521/failure-review.json) |
| coarse40bearing035 / 557 | 없음 / 0 | contact0 / off없음 (종료309.62) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed557) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed557/failure-review.json) |
| coarse40bearing035 / 569 | 없음 / 0 | contact0 / off없음 (종료309.58) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed569) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed569/failure-review.json) |
| coarse40bearing035 / 601 | 없음 / 0 | contact0 / off없음 (종료307.28) | 거절 odometry_uncertainty | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed601) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed601/failure-review.json) |
| coarse40bearing035 / 613 | 없음 / 0 | contact0 / off없음 (종료309.60) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed613) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed613/failure-review.json) |
| coarse40bearing035 / 617 | 없음 / 0 | contact0 / off없음 (종료309.70) | 거절 search_timeout_without_metric_pose | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed617) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed617/failure-review.json) |
| coarse40bearing035 / 433 | 157.08 / 514 | 187.04 / 187.92 / 190.92 | 조건부 승인 9점/11.798mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed433) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed433/signedreview.json) |
| coarse40bearing035 / 467 | 123.24 / 495 | 152.58 / 153.50 / 156.50 | 조건부 승인 10점/8.055mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed467) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed467/signedreview.json) |
| coarse40bearing035 / 479 | 180.92 / 549 | 212.70 / 213.58 / 216.58 | 조건부 승인 10점/8.825mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed479) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed479/signedreview.json) |
| coarse40bearing035 / 541 | 70.28 / 488 | 99.22 / 100.12 / 103.12 | 조건부 승인 8점/25.627mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed541) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed541/signedreview.json) |
| coarse40bearing035 / 571 | 72.50 / 584 | 106.26 / 107.16 / 110.16 | 조건부 승인 9점/14.585mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed571) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed571/signedreview.json) |
| coarse40bearing035 / 577 | 96.46 / 470 | 124.86 / 125.76 / 128.76 | 조건부 승인 8점/28.588mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed577) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed577/signedreview.json) |
| coarse40bearing035 / 607 | 98.96 / 479 | 127.60 / 128.48 / 131.48 | 조건부 승인 8점/29.041mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed607) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed607/signedreview.json) |
| coarse40bearing035 / 619 | 207.26 / 640 | 243.64 / 244.54 / 247.54 | 조건부 승인 9점/10.867mm | [run](../../../artifacts/mission3/physical/20261009-coarse40bearing035-v17-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed619/signedreview.json) |

모든 원 offline-evaluation28개를 각 **실행 당시 evaluator snapshot**으로 다시 계산하여 전 필드 일치를 확인했다. 원 relative run_dir로 정규화했고 원 평가를 덮어쓰지 않았다. 점수는 첫 실제 접촉 시 합성 CoM/body-origin 내부 기준으로 공식 경쟁/실기 점수가 아니다. 각 서명은 full evaluation-file SHA와 canonical digest를 모두 보존한다.

원 code62개·runtime10개(.so/build metadata 포함)가 전28개 snapshot/manifest 및 plan65개(source62+config3)와 일치했다. code/runtime는 v16과도 같고, 수정된 source 없이 기존 coarse 옵션 세 값을 비교한다. baseline config identity `bba87b2cfeb71cafb4083d7386cc5ac8991337d9d6cb5d9807968d1bd11d7da7`, candidate `53883864cca89d2e736ef2798776c71e3e8167a063eb9336193be63124b74071`; 같은 정책 안에서는 동일하다. 각 seed/trial metadata는 별도 SHA로 검증했다. source65에 config3개가 포함되어 실행 Python/C가65개라는 뜻은 아니다. 원 physical profile 및 scene/초기 pad phase는 해당 seed의 기준과 같았다. [전28 source/config/phase/evaluation 검증](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/source-series-verification.json).

| 변경된 제어 옵션 | baseline | candidate |
|---|---|---|
| coarse_translation_axis | body_forward (기본) | fresh_decoded_bearing |
| coarse_translate_turn_error_deg | 15 (기본) | 40 |
| coarse_turn_translation_speed_m_s | .15 (기본) | .35 |

설명용 `coarse_policy_basis` 문구도 config에 추가돼 해시에 포함되지만 source의 제어 분기로 사용하지 않는다. raster8m/.75m, entry-forward search gaze, WAIT15/scan15/height1, imu12 range window1.2s, 원 .12/.08 gains, strict pose18px/std1m, Flow-slew/caps, 장면/차량 연속.25m/s,6s prediction/.25s support는 유지됐다. 현행 v18 ABORT 전달 구현을 v17 원 run의 실행 코드로 대체하지 않았다. build manifest의 열거 C/header는 실제 일치 SHA를 확인했으나 모든 전이 include/compiler 환경의 재빌드 재현은 주장하지 않는다.

| 직접 원 snapshot source | SHA256 |
|---|---|
| [scripts/run_mission3_physical.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/run_mission3_physical.py) | `bb9dec0ff969e801aeae6017e38380ed27f3c9a024f808bb4763f04a28733925` |
| [scripts/mission3/controller.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/controller.py) | `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180` |
| [scripts/mission3/physical_estimator.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/physical_estimator.py) | `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5` |
| [scripts/mission3/physical_scene.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/physical_scene.py) | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| [scripts/mission3/touchdown.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/touchdown.py) | `b258eeea56689f03c7ff1c258c1703f666b514c3ff34761f10856d8f9b239f7b` |
| [scripts/mission3/estimator.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/estimator.py) | `aee18061e2ea283e943adc285837d8124fb4a54e4e1a8835a03a1f459e0eb668` |
| [scripts/mission3/uncertainty.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/uncertainty.py) | `fb2bf490da436d76a767a2cb931008c9a81995ece7d23153365e20fabc109bb2` |
| [scripts/mission3/vertical_fit.py](../../../artifacts/mission3/physical/20261009-baseline8-v17-entry1-seed521/source-snapshot/scripts/mission3/vertical_fit.py) | `afba09e5003b5f77d13916854cd54f015f73e76345f74408e2646bb59c1f2153` |

원 FFV1를 전28개 독립 decode하여 **116,183 frames**의 mono320 uint8 controller-input bytes/SHA가 모두 일치했다. resize/color conversion이나 lossy archive를 원픽셀로 부르지 않는다. archive exact는 실제 AI-deck 송신/노출/렌즈 PSF/latency 검증과 다르다. 일부 원 native 표본을 눈으로 확인했으며 전체 영상1× 재생 QA를 했다는 주장은 아니다. [candidate541 픽셀 표본](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/coarse40bearing035-seed541/native-camera-samples.png).

16개 접촉 모두 실제 기록은 pad foot1–4이며 chassis/marker/floor 접촉은 없었다. PWMoff부터3초 동안151개50Hz truth 표본 모두 양의 네 발 법선 지지,3001개1kHz PWM/output/static-force0, contact/hold 전구간 직선, 전비행 차량 speed.25m/s 연속을 확인했다. raw IMU/command-force/lag를 이용한 freeflight calibration과 support observer 전체 재생, sensor-only FSM, RPYT100Hz·14byte little-endian/yaw sign, Flow100/ToF40 producer 및 stamp/new/raw 전달, normal command caps가 일치했다. source observer/main unsigned 개발 검토의 주요 hold 필드와 불일치0이었다. 정확한 전체 hidden PID/EKF 상태 또는 실기 radio scheduler 재현은 아니다.

| 승인 case | contact 표본 | hold 최소 발법선 N | hold 최대 slip μm | 실제 최대3D속도 m/s | 준비시간 s |
|---|---:|---:|---:|---:|---:|
| baseline8-seed433 | 196 | 0.033044 | 5.739 | 0.408573 | 14.18 |
| baseline8-seed467 | 198 | 0.033531 | 3.921 | 0.405492 | 13.40 |
| baseline8-seed479 | 196 | 0.031483 | 4.289 | 0.405744 | 13.44 |
| baseline8-seed541 | 195 | 0.031666 | 11.062 | 0.412898 | 14.86 |
| baseline8-seed571 | 197 | 0.033122 | 7.093 | 0.408150 | 14.88 |
| baseline8-seed577 | 197 | 0.029543 | 13.856 | 0.408081 | 13.56 |
| baseline8-seed607 | 196 | 0.028310 | 14.058 | 0.401856 | 13.60 |
| baseline8-seed619 | 196 | 0.033200 | 4.567 | 0.405305 | 13.50 |
| coarse40bearing035-seed433 | 196 | 0.033044 | 5.739 | 0.408573 | 14.18 |
| coarse40bearing035-seed467 | 198 | 0.033531 | 3.921 | 0.405492 | 13.40 |
| coarse40bearing035-seed479 | 196 | 0.031483 | 4.289 | 0.405744 | 13.44 |
| coarse40bearing035-seed541 | 197 | 0.032232 | 12.445 | 0.413794 | 14.42 |
| coarse40bearing035-seed571 | 197 | 0.033122 | 7.093 | 0.408150 | 14.88 |
| coarse40bearing035-seed577 | 197 | 0.029543 | 13.856 | 0.408081 | 13.56 |
| coarse40bearing035-seed607 | 196 | 0.028310 | 14.058 | 0.401856 | 13.60 |
| coarse40bearing035-seed619 | 197 | 0.033253 | 5.294 | 0.406013 | 14.66 |

실제 속도는 명령 .4m/s cap과 다르며 candidate541의 최대3D속도 .413794m/s가 관측됐다. 명령 normal 3D가속 .15m/s²/속도.4/상승.1/하강.06/yaw10°/yaw가속20° 제한은 전28개 본인 계산과 원 audit가 일치했다. 원 ABORT/PWMoff urgent 예외를 따로 남기고 coarse/Flow loss에 새 예외를 넣지 않았다. PWMoff 첫 step 실제 힘 .201417–.202281N은0이 아니고 .3초 뒤6.161e−8–6.188e−8N으로 줄었다. 가정20ms motor lag의 기록 재구성 결과로 실측 rotor 즉시 정지라 쓰지 않는다. 준비시간은13.40–14.88s로 planning12s를 초과해 이 예산을 엄격한 runtime cap이라고 부를 수 없다.

각 hold의301개100Hz sensor 표본에는 정확한 ToF/valid Flow가 모두0이고 ambiguous 가설이었다. hold 안정 IMU/model residual은 실기 네 발이나 pad identity를 관측하지 않는다. 그 판정은 오프라인 물리 contact truth로만 했다. 예를 들어 baseline/candidate607은 마지막 world 위치 오차94.55mm·relative XY130.22mm지만 첫 contact29.04mm/실제 hold slip14.06μm다. 공통 위치 gauge, 예측 상대오차, 접촉시 score와 물리 slip은 서로 다른 양이다. 이후 센서가 없는데 이 값들을 비행 안정성/정확한 relative odometry로 등치시키지 않는다.

**coarse 입력 경계:** source vision은 두 decoded target ID의 temporal bearing agreement(.7s 이내/12°), native 최소7px coarse gate로 bearing을 만든다. undistorted pixel ray와 capture-time 추정 IMU/camera transform만 사용하며 metric pose·vehicle range/velocity를 만들어 넣지 않는다. candidate의 decoded 방위 방향 translation은 fresh age≤.15s,7≤min-side<18px, max-side<36px인 경우에만 선택한다. 실제 coarse ID를 metric acquisition18px로 낮춘 것이 아니다. episode20s/추정 total path4m와 원 slew/net/uncertainty 제한도 유지됐다. 4m는 추정 odometry 누적 경로이며 실제 path/실기 안전 한계 보장은 아니다. controller search는 known entry/arena에서 만들고 차량 truth/track waiting station을 사용하지 않는다.

본인은 각 run의 직전 warm command slew 상태를 이어받아 snapshot Mission3를 logged own/image-derived vehicle 입력만으로 재생했다. terminal adapter 시작 전 velocity/yaw/coarse_guard/FSM 오차0이었다. 이는 raw perception/firmware hidden-state 재실행이 아닌 실제 기록된 sensor-derived 입력에서 제어 옵션이 정확히 작동했는지의 검사다. 각 case의 `coarse-command-independent-replay.json`에 freshness·path·active samples를 남겼다. source AST는 aircraft qpos/qvel 초기화 뒤 처방 쓰기가 없고 prescribed되는 것은 차량뿐임을 다시 확인한다. plant truth는 sensor 합성과 오프라인 contact/force/gauge 및 floor/tilt harness stop에서 쓰이며 controller live branch에 넣지 않았다. [source 경계·near-range](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/source-boundary-and-near-range.json).

**원 paired 효과와 전체 분모:** 14쌍 중9쌍(433467479521569571577607617)은 각 commands/truth/observations/motors/sensors/deck_samples/estimates/rpyt_packets/camera_frames/camera.avi10개가 byte 동일이다. 옵션 미활성 재현을 새로운 효과나 독립 효능 표본으로 세지 않는다. 나머지5쌍은 새 raw/source/evaluation 결합의 각 결과를 그대로 검토했다. [본인140 file-pair SHA와 실제 명령 activation](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/paired-raw-and-gate-audit.json).

| 변화 seed | candidate fresh decoded translation 표본 | 실제 결과 차이 |
|---|---:|---|
| 541 | 335 | first metric72.98→70.28s/contact103.62→99.22s; 첫 접촉22.808→25.627mm 악화, 양쪽 승인 |
| 557 | 47 | decoded23→23/metric0, 양쪽timeout; 새접촉0 |
| 601 | 129 | decoded20→41/metric0; baseline309.64timeout→candidate307.28std1m abort, 양쪽접촉0 |
| 613 | 31 | decoded21→12/metric0, 양쪽309.60timeout; 새접촉0 |
| 619 | 106 | first metric208.40→207.26s/contact243.76→243.64s;9.375→10.867mm(10점→9점), 양쪽승인 |

candidate601은 coarse state `valid:true`여도 `kind:coarse_image_bearing_only`이며 metric pose가 아니다. 마지막 std1.000242m/heading std15.803°, 실제 world XY오차579.62mm였고 fresh image age.06s에서도 uncertainty abort했다. baseline601은 std.869819m/실제 worldXY505.84mm로 search timeout했다. 관측 수 증가와 센서 품질·추적 완료·불확실성 예산은 별개다. 이 공분산은 보정된 확률값이 아니며 실제 오차/truth로 reset하거나1m gate를 늘리지 않는다. 나머지 실패도 표의12회를 남기며, decoded ID만의 metric0 실패를 contact 성공으로 바꾸지 않는다.

required seeds433467479541571577607619 retention8개 모두 contact+caps였으나 양쪽8/14이므로 strict improvement 조건은 false다. main `selection.json`의 plan/summary SHA 및 선택 규칙을 본인 전체 검사가 끝난 뒤 검증했다. 따라서 candidate 채택false/새 holdout없음이라는 원 결론을 유지한다. 접촉을 조금 앞당긴541/619나 decoded41인601만 발췌해 주효과로 채택하지 않는다. baseline 유지도 제조사·전체 방향·진입2 승격 승인이 아니다.

**ABORT 전달 공백은 원 v17에 남는다.** 실패12개 모두 ABORT zero-velocity를 기록한 다음 wrapper `command.done` break가 새 RPYT/PWM/physics보다 먼저 실행됐다. 마지막 wire는 abort10ms 전, 마지막 motor는1ms 전/PWM nonzero/stop packet0이었다. 실패 종료를 안전한 실제 회복·정지로 승인하지 않는다. main의 후속 v18 수정 의도와 이번 원 run 판정을 구분한다. 새 source에서 전달 이후 상태 관측 계약을 검사해야 하며 미완성 후속 구현을 v17 안전 근거로 붙이지 않는다.

v17 미래 IMU guard는 v16 원afba09 source와 같다. 전28의 기록된 surface/image acceptance를 고정한 Z/vZ 산술 재생 오차0/fallback0/future·duplicate0이었다. pure API 반례도 새 snapshot에서 재실행해 now.055/IMU끝.06 future0·1 모두거부, current끝=now=.06의−.020m/s 유지, nextafter/1msfuture 거부 및6해석 fixture를 확인했다. 동시에 SelfMotion은 invalid future history의 fit ValueError 뒤 legacy−.03m/s fallback을 반환했으므로 전체 height/range 융합/외부 adapter 시각 안전 보장은 아니다. producer40/consumer100의 range age0/5/10/15/20ms 및 consumer-time tilt/IMU box average는 합성 가정이다. scalar acceleration displacement regression을 Forster manifold VIO/bias 식별기로 부르지 않는다. [현재경계·fallback 반례](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/scalar-fit-independent-fixtures.json); [전28 chronology 산술](../../../artifacts/mission3/reviews/20261009-v17-coarse-bearing-paired-training-independent/vertical-fit-all-cases.json).

근접 rawToF default0 full-onboard EKF, 독립 PC quaternion에 deck update 없음, 실제 rate PID의 onboard attitude 경로는 원 그대로다. 근접 accurate flag/Flow texture는 실제 RangeStatus/SQUAL 대신 합성 geometry/quality이며 실제 mount/K/D/PSF/mass/CoM/inertia/force gain/lag/contact/friction/capture/radio/RTOS timing은 S/U다. 전 motor 공급4.2V 고정에 SOC/energy/mission1·2 residual reserve가 없다. 약310초 실패 비행은 payload를 장착한 실제 기체 endurance 증거가 아니다.16개 조건부 접촉 승인도 hardware/qualified/formal/entry2 승인0이다.

구체적인 후속 권고는 frozen 원 v17을 보존한 채 별도 source의 실패 명령 전달 계약을 검증하고, coarse bearing의 실제 활성·freshness/경로·남은 metric acquisition 시간·covariance budget을 함께 평가하는 것이다. 성공 수가 같고 두 접촉 오차가 악화한 후보를 adopt하거나 pose/std/support 문턱을 완화하지 않는다. timing/RangeStatus/near quality 및 에너지 불확실성은 source causal guard와 별도 증거가 필요하다.

원자료2,876개와 이전 서명47개의 시작/종료 SHA를 확인한다. 실제 새16positive는 본인 agentID/원 snapshot source/runtime/config/trial/full evaluation-file SHA·canonical digest 및 실제 audit evidence를 결합하고,12negative는 승인false의 별도 failure-review로 보존한다. 최종 index/final-verification은 전28 검사·해시 결합이 끝난 뒤 저장한다. 이번 자체 감사 스크립트들은 이전 검토의 검사 로직을 재사용했지만 모든 raw/source/evaluation 결과는 이번 run을 직접 읽었으며 승인 서명은 복사하지 않았다. helper의 계획미검증 flag는 source-series 별도 계획검증과 구분한다.
