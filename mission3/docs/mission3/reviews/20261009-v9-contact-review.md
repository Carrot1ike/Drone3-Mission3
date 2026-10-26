# v9 독립 조건부 접촉 검토 — Heisenberg

2026-10-09. 검토자 `Codex-Independent-Flight-Reviewer` / Heisenberg, agent ID `01a117a1-f802-75f1-9980-91813ff9f0f9`. 실행 저자 `main-flight-adapter`와 구분한다. 사용자 요청으로 기존 담당을 재사용했으며 새 agent 생성·물리 재실행·runtime 수정은 하지 않았다.

**421·433·457을 각각 조건부 모델의 접촉 acceptance로 승인한다. 제조사 일치, 실기 착륙, 안전성, qualified 승인은 모두 아니다. qualified 승인/형식 집계는 0이다.** 원자료의 자체 `development-review-unsigned.json`를 독립 서명으로 전환하지 않았다. 원로그 재계산, 보존 source/실행 binary 대조, 센서만 사용하는 후보 재현을 끝낸 뒤 본인 판단을 별도로 기록했다.

사전 고정 v9 전체 7회는 재사용 training 2/2 통과와 unused holdout 1/5 통과·4실패다. 결과를 합쳐 일반 실기 성공률이나 진입2 진행 기준 충족으로 주장하지 않는다. v8 및 과거 모델 성공 수를 더하지 않는다.

| seed | split | 원 조건부 접촉 | 접촉 / PWM 종료 / 3초 종료 (s) | 내부 중심오차 / 점수 | 독립 판정 |
|---|---|---|---|---|---|
| 421 | reused training | pass | 233.20 / 234.04 / 237.04 | 18.179 mm / 9 | 조건부 접촉 승인 |
| 433 | reused training | pass | 190.36 / 191.20 / 194.20 | 14.459 mm / 9 | 조건부 접촉 승인 |
| 457 | unused holdout | pass | 192.02 / 192.90 / 195.90 | 33.055 mm / 7 | 조건부 접촉 승인 |
| 461 | unused holdout | fail | 접촉·PWM 종료 없음; 277.32 종료 | 없음 | odometry_uncertainty, 승인 없음 |
| 463 | unused holdout | fail | 접촉·PWM 종료 없음; 278.66 종료 | 없음 | odometry_uncertainty, 승인 없음 |
| 467 | unused holdout | fail | 접촉·PWM 종료 없음; 309.36 종료 | 없음 | search_timeout_without_metric_pose, 승인 없음 |
| 479 | unused holdout | fail | 접촉·PWM 종료 없음; 309.56 종료 | 없음 | search_timeout_without_metric_pose, 승인 없음 |

내부 점수는 첫 실제 접촉 시 조건부 body/CoM 원점의 패드 중심 오차다. 공식 경기 점수가 아니다. 네 실패의 원평가도 보존 평가기로 재현했으며, 실패 영상 전체를 독립 디코딩했다고 주장하지 않는다.

사전 계획 [plan.json](../../../artifacts/mission3/acquisition/20261009-search-reverse-flow-slew-v9/plan.json)과 [원 summary](../../../artifacts/mission3/acquisition/20261009-search-reverse-flow-slew-v9/summary.json), [독립 7회 분모·동결 감사](../../../artifacts/mission3/reviews/20261009-v9-independent/full-seven-denominator-and-frozen-identity.json)를 연결했다. 원 summary의 당시 independent=0 표기는 역사 기록으로 남겼다. 이번 서명은 별도 신규 파일이다.

## 개별 접촉 근거

세 run은 각 194/194/196개 접촉 truth 표본에서 모두 네 발이 실제 pad에 닿았다. 비발·차체·바닥·보드·그물 충돌 기록은 없었다. PWM 종료부터 3초 동안 각 151개 50 Hz truth 표본이 네 발 지지이고 모든 발 법선력이 양수였다. 각 3,001개 1 kHz motor 표본은 PWM·motor_output·명령 staticforce가 0이다. 차량 속력은 전체 기록에서 .25 m/s이며 HOLD 3초 동안 pad가 .75 m 이동한다. LOWER_ALIGN 이후 실제 turn 표본도 0이다. 위치를 패드에 붙이는 equality/mocap 제약은 없다.

| seed | HOLD 발별 법선력 범위 (N) | 3D 상대속력 최대 (m/s) | PWM 종료 대비 수평 slip 최대 (m) | 실제 최대 속력 (m/s) |
|---|---|---|---|---|
| 421 | .033847–.085839 | .000073459 | .000008846 | .419953 |
| 433 | .032717–.085839 | .000073240 | .000007030 | .408012 |
| 457 | .030887–.085839 | .000073314 | .000016031 | .410144 |

정착 뒤 네 발 법선력 합 약 .343350 N은 모델 질량 **0.035 kg × 9.81**과 일치한다. 질량·발 형상·CoM 높이 .0145 m·마찰 .6·접촉 solver는 S이며 실기 지지력 측정이 아니다. 접촉 모형의 규칙성은 실제 장비 강건성의 증거로 바꾸지 않는다.

PWM 0는 기계적인 즉시 정지나 즉시 추력 0를 뜻하지 않는다. 457의 off 시각 truth 표본에 .201795087 N이 남고, .3초 뒤 6.17295854×10⁻⁸ N이다. 421/433도 각각 .203051901/.202618012 N이 남는다. 보존 binary의 공식 software staticcurve 계산과 기록된 motorforce의 20 ms S recurrence를 직접 대조했다. 샘플 curve 계산 차이와 전체 truth 시점 force recurrence 차이는 0이다. 회전수 자체는 기록되지 않았다.

[421 접촉/PWM 감사](../../../artifacts/mission3/reviews/20261009-v9-independent/seed421/log-audit.json) · [433](../../../artifacts/mission3/reviews/20261009-v9-independent/seed433/log-audit.json) · [457](../../../artifacts/mission3/reviews/20261009-v9-independent/seed457/log-audit.json); 각 폴더의 `terminal-contact-command-sensor.csv`에 명령, 실제 추력·지지력, 상대운동, 센서 결손을 함께 보존했다.

## 소스·runtime·입력 SHA와 truth 경계

각 run의 실행 전 보존 **58개 source/test/C 파일과 10개 binary/build-manifest/header**를 원 manifest SHA와 직접 대조했다. 61개 plan 항목은 58개 파일과 3개 config 항목이다. 실제 v9 config 별도 SHA도 확인했다. 7회 모두 source/runtime/config identity가 같고 seed·entry·split은 개별 trial metadata에 결합된다. CLI가 `inject_initial_uncertainty=false`를 명시해 넣는 정상 변환도 원 source에서 확인했다.

보존 evaluator로 원 `offline-evaluation.json`를 전필드 동일하게 재현했다. canonical `run_dir`를 프로젝트 상대 경로로 유지했다. 각 서명에 원평가 파일 전체 SHA와 canonical JSON digest를 모두 넣었다. 현재 main의 v10 파일을 과거 source라고 취급하지 않았다. build-manifest가 지정한 공식 source/header 33개도 snapshot 또는 **현재 bytes가 지정 SHA와 일치하는 경우에만** 대조했다. 모든 transitive include/컴파일러 환경의 완전 보존이나 전체 build 재현을 주장하지 않는다.

| 대상 | 실행 당시 SHA256 |
|---|---|
| run_mission3_physical.py | `bb9dec0ff969e801aeae6017e38380ed27f3c9a024f808bb4763f04a28733925` |
| controller.py | `1f989d1e635eeda163960703bf6410e0b7ec3138fca3d033a70626809cfb7f1a` |
| physical_scene.py | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| physical_estimator.py | `d8eab7c2e126307b3970f54f3db9cd0a1835c3fd8f5c37b6ff3747b900ee054c` |
| touchdown.py | `b258eeea56689f03c7ff1c258c1703f666b514c3ff34761f10856d8f9b239f7b` |
| evaluation.py | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |
| frozen-experiment-settings.json | `af08508255d9d05bfdaa25473786e3ced1d997e758dfcea1adf133d9b16385b9` |

`physical_scene.py`의 qpos/qvel 쓰기는 aircraft 초기 속도 선택과 차량 vq/vv 갱신뿐이다. 이후 항공기에는 force/torque를 적용하고 `mj_step`으로 적분한다. 차량 .25 m/s는 허용된 환경 처방이고 시간0부터 연속이다. controller 공개 설정에는 `vehicle_plant_only`가 없다. 카메라/IMU/가상 Flow·ToF 생성에는 plant state가 필요하지만 contact ID·법선력·차량 truth를 접촉 후보 branch에 넘기지 않는다. floor 접촉과 실제 tilt>60°는 별도 하네스 조기 중단이며 성공을 만들어내지 않는다.

raw body IMU·100 Hz held PWM curve telemetry만 읽는 전체 touchdown observer 재생에서 freeflight gain/신선도/잔차/기대 가속도 오차가 모두 0이었다. 조건부 후보 최초 시각이 각 PWM 종료 시각과 같았다. 접촉 truth는 재생 입력이 아니다. 센서·추정값만 넣는 보존 FSM의 terminal 이전 phase 재생도 불일치0이었다. hidden native PID/EKF 상태 전체 재생을 주장하지 않는다. 공식 RPYT wire의 roll/pitch와 -yaw_rate·uint16 thrust, 100 Hz packet 간격과 stop payload 14 zero bytes를 검사했다. 공식 PID의 gyro.y 반전/기체 pitch convention을 함께 읽었다. motorsCompensateBatteryVoltage 추출 body SHA도 공식 파일과 정확히 일치했다.

근거: [plant force/vehicle/센서](../../../artifacts/mission3/physical/20261009-search-reverse005-flow-slew-v9-entry1-seed457/source-snapshot/scripts/mission3/physical_scene.py:109) · [terminal 후보와 truth logging 경계](../../../artifacts/mission3/physical/20261009-search-reverse005-flow-slew-v9-entry1-seed457/source-snapshot/scripts/run_mission3_physical.py:213) · [PC/onboard 추정 경계](../../../artifacts/mission3/physical/20261009-search-reverse005-flow-slew-v9-entry1-seed457/source-snapshot/scripts/mission3/physical_estimator.py:66) · [독립 source 감사](../../../artifacts/mission3/reviews/20261009-v9-independent/independent-source-boundary-review.json).

## 명령 한도와 말기 관측 결손

세 run의 정상 3D 명령 가속도 최대는 .150000000000083 수준으로 원 .15 m/s² 한도를 지킨다. 명령 속력 .4 m/s·상승 .10·하강 .06·yaw 10°/s·yaw 가속도 20°/s²도 통과한다. 실제 속력은 표처럼 상한을 넘는다. 본 판정은 명령 envelope에 대한 것이며 실제 강체 운동이 한도 안이라는 승인으로 해석하면 안 된다. PWM 종료 시 즉시 zero 명령으로 바뀌는 12.610/11.509/12.917 m/s² 점프는 명시적 종료 예외로 보존했다.

v8/433의 BLIND_APPROACH→BLIND_DESCEND 186.74 s에서 정상 가속도 .3206897 위반1점을 독립 재계산했다. v9의 opt-in `terminal_flow_loss_slew_brake`는 Flow 손실 때 하강 목표를 0으로 유지하되 `last_v[2]`를 덮어쓰지 않는다. 기존 3D slew로 제동한다. v8 실패, 평가기, 물리 profile, 원 문턱을 바꾸지 않았다. 수정으로 실기 제동 성능이나 안전성을 보증하지 않는다.

Flow100 Hz·ToF40 Hz와 stamp/newflag를 원 deck_samples와 대조했으며 중복 range_new 소비·원 packet 불일치0이다. 말기 정보는 크게 줄어든다.

| seed | 마지막 Flow fusion / 정확 ToF stamp (s) | 마지막 비행 명령의 유효 영상 나이 (s) | off 유효 영상 나이 / 6초 잔여 (s) | off 추정 body 높이−pad (mm) |
|---|---|---|---|---|
| 421 | 227.90 / 232.175 | 5.12 | 5.14 / .86 | 4.975 |
| 433 | 184.94 / 189.300 | 5.20 | 5.22 / .78 | 9.017 |
| 457 | 186.68 / 191.025 | 5.22 | 5.24 / .76 | 15.421 |

off는 terminal handoff 뒤 4.06/4.26/4.28초, freeflight calibration age는 20.55/19.73/19.75초로 각 6초/45초 limit 이내다. 3초 HOLD 중 151개 control/truth 표본은 모두 surface ambiguous이며 정확 ToF·유효 Flow는 0이다. 100 Hz sensor 표본 기준은 301개로 같게 결손된다. observer의 안정성 로그는 종료 다음 tick부터 150개이며 physical endpoint를 포함한151개와 구분한다. 421 추정 contact 높이는 S gate 하한2.5 mm보다 2.475 mm밖에 높지 않다.

HOLD 동안 마지막 유효 영상 나이가 8.14/8.22/8.24초까지 커진다. 보존 source는 motoroff 뒤 재비행하지 않는 HOLD에서 IMU 안정성만 검사한다. 이것은 추가 blind 비행 허가가 아니다. **IMU near-g와 낮은 gyro는 지지 표면·패드 정체성·4발을 식별하지 못한다.** 이번 패드 지지는 따로 오프라인 truth로 확인했다.

457 마지막 aircraft world 오차 .245275 m, relative pad XY 추정 오차 .089370 m, 첫 접촉 실제 중심 오차 .033055 m는 서로 다른 양이다. 공통 위치 gauge 이동은 pad-minus-own에서 상쇄되지만 differential drift는 남는다. 단일 원인으로 IMU/영상/FF bias를 확정하거나 truth 상수를 online 튜닝하지 않았다.

[v8→v9 보존 source diff](../../../artifacts/mission3/reviews/20261009-v9-independent/v8-to-v9-controller-snapshot.diff) · [v8 위반 재계산](../../../artifacts/mission3/reviews/20261009-v9-independent/v8-v9-source-and-violation-comparison.json) · [457 말기 증분 CSV](../../../artifacts/mission3/reviews/20261009-v9-independent/seed457/terminal-contact-command-sensor.csv).

## 실기 관련 우려와 후속 피드백

1. **합성 quality flag와 실제 센서 계약의 차이.** `range_accurate`는 잡음 추가 전 ideal ray distance로 정한다. flag=true인데 noisy raw<4 cm인 표본이 421/433/457에서12/8/10개 있었다. Flow flag 역시 ideal min-ray range와 generic rendered texture로 정해진다. 실제 PMW3901 quality·ST reflectance/crosstalk·조립 응답이 아니다. 실제 logger 값으로 구현할 수 있는 validity/불확실성 정책을 별도 정의하고, noisy 경계·낡은 timestamp·quality 미확정 입력의 paired 반례를 먼저 검사할 것을 권한다. 4/8 cm는 제조사 부품 문서를 근거로 main 개발자가 선택한 보수적 시험 정책이며 인간 사용자가 지정한 문턱이 아니다.
2. **전 경로가 근접 ToF metric fusion을 끊은 것은 아니다.** PC metric 높이는 accuracy gate를 쓰지만 보존 `physical_estimator.py`의 official onboard fullEKF는 new0≤range<5 m ToF를 accuracy flag와 무관하게 받는다. 따라서 near-ground-model로 기체 자세 PID가 오염될 가능성은 남는다. PC 독립 IMU 분리만으로 모든 자세 경로가 해결됐다고 말할 수 없다. 기존 run을 보존한 채, 동일 seed/새 frozen source의 onboard phase-specific measurement gate 비교 등으로 영향 분리를 권한다.
3. **sensor/observer가 공유하는 추력·접촉 가정.** mass .035 kg,20 ms lag,forcegain1,고정4.2 V,정해진 foot/마찰 및 CoM IMU 위치가 S다. gain fit의 성공은 이후 motor gain 변화·지면 효과·IMU offset·잘못된 지지면의 거짓 contact를 배제하지 않는다. 접촉 후보·허용 command·실제 pad acceptance를 별도로 유지하고, 기존 wrong-body/airborne model-error 반례를 보존해야 한다. gate를 넓혀 chassis contact를 받아들이는 변경은 근거 없다.
4. **광학·mount·시간의 미확정.** 가상 센서 ray는 floor/pad만 계산하며 board/artwork/net 가림·실제 광학 PSF·PMW transfer는 재현하지 않는다. 렌더 카메라는 true marker depth<.30 m이면 전체 frame을 지우므로 실제 초점/검출 실패의 연속성을 모델링하지 않는다. actual stock/AI/Flow revision·mount/K/D/조립 mass/dynamics·radio/RTOS·capture jitter는 U/S다. 100 Hz 평균 IMU·PWM telemetry와 delay0는 실제 native log snapshot의 timestamp/boxing 계약을 입증하지 않는다.
5. **준비 시간과 에너지.** LOWER_ALIGN→BLIND_APPROACH 준비 시간14.38/13.44/13.40초가 S planning budget12초를 넘었다. 여기서는 실제 직선이 유지됐지만 더 짧은 남은 구간에서 margin을 낙관할 수 있으므로 계획 margin을 paired 검사해야 한다. 고정4.2 V에는 SOC/current/에너지/덱부하/이전M1M2소비량이 없다. 실제3미션 잔여 비행시간은 U이며, 본 fixture의 sensor-stable hover는 실제 Mission2 경로 인계가 아니다.

이 항목들은 실기·safety approval의 공백이다. 현재 세 run의 실제 모델 내 pad 접촉 조건을 거짓으로 만들지는 않았으므로 조건부 접촉을 승인했다. 모든 핵심 가정이 확인됐다는 승인으로 확대하지 않는다.

## 원픽셀·영상과 최종 파일

421/433/457의 FFV1을 전부 디코딩해 각각4364/3490/3525장, 총 **11,379장**의 native320×320 uint8 픽셀 SHA를 controller 입력 SHA와 직접 비교했다. 불일치0이다. 실패 네 run의 frame 수를 이 독립 디코딩 수에 더하지 않는다. FFV1은 보관 format이며 실제 AI-deck 전송 사양이 아니다. focus 소실·50 Hz 제어 샘플링 때문에 capture 간격이 일정하지 않으므로 `camera.avi`의20 FPS header만으로 전체 비행의1× 시간선을 복원하면 안 된다. `camera_frames.jsonl` stamp가 필요하다.

421/433 원픽셀 표본은 각 v9 run의 원 `camera.avi`에서 새 경로에 추출했다: [421 표본](../../../artifacts/mission3/reviews/20261009-v9-independent/seed421/native-camera-samples.png) · [433 표본](../../../artifacts/mission3/reviews/20261009-v9-independent/seed433/native-camera-samples.png). v8의 별도 영상·poster를 v9 원영상으로 사용하지 않았다.

457의 [acquisition-to-contact-157-196s-1x-h264.mp4](../../../artifacts/mission3/physical/20261009-search-reverse005-flow-slew-v9-entry1-seed457/acquisition-to-contact-157-196s-1x-h264.mp4)는10 FPS/390 frames/39초/1280×720/1×의157~195.9초 발췌이며, 전체195.92초 비행이 아니다. 전390프레임 디코딩·원파생 SHA·timing을 확인하고192.0/195.9초 표본을 직접 보았다. 오른쪽 흑백 영상은 held last-delivered frame이며 접촉 중 신선한 관측이 아니다. 화면의 pass/점수는 전체 run의 최종 오프라인 결과로 읽어야 한다.

개별 서명과 원평가 결합 SHA:

- seed421: [signedreview.json](../../../artifacts/mission3/reviews/20261009-v9-independent/seed421/signedreview.json) — 서명 파일 SHA `9e9dd91508f7120b409e0626590f8bc0fba5cb9d79c5371ef469ec32c565a590`; 원평가 SHA `4e3cf33521e49b5f57823cae45f555a2eefacc2327b10d048146fc8f95e3f6bd`.
- seed433: [signedreview.json](../../../artifacts/mission3/reviews/20261009-v9-independent/seed433/signedreview.json) — 서명 파일 SHA `4a6eba6cc8b77412515ceb126500c50e9d40bbdb17bb6eaf28ef78e7c56da0d5`; 원평가 SHA `edef7c88e9e016fae22b42b0ba46d2a7a5383e70501e5059adce2625afe850db`.
- seed457: [signedreview.json](../../../artifacts/mission3/reviews/20261009-v9-independent/seed457/signedreview.json) — 서명 파일 SHA `ae0e7a3451db984f02c531a11518bd160d7bab3c00dfa59be2480babf294d0c6`; 원평가 SHA `516912898f8a20f2a6578dbb7cc35be83ce5dcfd83c60fcb57705def1e474d23`.

[서명 index](../../../artifacts/mission3/reviews/20261009-v9-independent/signed-review-index.json) · [검토 범위와 본인 ID](../../../artifacts/mission3/reviews/20261009-v9-independent/review-scope.json). 기존 원로그/원평가/source/config/libs/test/work-log/과거 서명은 수정하지 않았다. 검사 도중 ffprobe 부재와 OpenCV codec 표기 대소문자 문제로 실패한 자체 감사 log를 남겼고 최종 완료 감사로 구분했다. 이 개발 실패를 원 비행 실패나 유효 검증으로 계상하지 않았다.
