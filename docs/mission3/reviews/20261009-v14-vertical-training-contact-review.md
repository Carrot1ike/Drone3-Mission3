# v14 수직 IMU 회귀 training12 독립 접촉 검토

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. **imu08 6/6, imu12 6/6을 각각 조건부 모델 접촉으로 승인한다.** 재사용된 seed433/467/479/541/571/577의 spent training이며, 일반 성공률·진입2 승인·미래 holdout 승인으로 합치지 않는다. **제조사 일치/실기/qualified/formal 성공0**이다. 원 평가와 main unsigned 판정은 수정하지 않았고 본인의 원자료 재검사 결과로 새12서명을 작성했다.

사전 [plan](../../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-training-v14/plan.json) SHA `4ef68899e192eaa0f0ec867348e9d2c3ac590dac7a6662f6e7bcc2c372760db3`와 [summary](../../../artifacts/mission3/acquisition/20261009-vertical-inertial-window-training-v14/summary.json)의 전12분모를 확인했다. 후속 holdout은 별도 요청·별도 검토다. 조건부 물리 분모83→95와 이 검토의 두 정책 분모6을 구별한다.

## 개별 판정

시각은 원 물리 simulation clock(s)이며 warm Mission3 시작 시각과 다르다. hold끝은 PWMoff+3.00초, 원 로그는 추가0.02초까지 남는다. 점수는 첫 실제 접촉의 합성 CoM 기준 내부 값이다.

| 정책 | seed | 실제 첫 접촉 | PWMoff | hold끝 | 내부점수 / 오차mm | off 추정 패드위 높이mm | 접촉 truth표본 | 신규 본인서명 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| imu08 | 433 | 187.16 | 188.06 | 191.06 | 8 / 21.421 | 19.516 | 197 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed433/signedreview.json) |
| imu08 | 467 | 151.78 | 152.64 | 155.64 | 9 / 18.402 | 11.972 | 195 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed467/signedreview.json) |
| imu08 | 479 | 213.54 | 214.48 | 217.48 | 10 / 8.288 | 20.353 | 199 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed479/signedreview.json) |
| imu08 | 541 | 103.78 | 104.66 | 107.66 | 9 / 19.389 | 16.096 | 196 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed541/signedreview.json) |
| imu08 | 571 | 110.06 | 111.00 | 114.00 | 7 / 35.480 | 20.767 | 199 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed571/signedreview.json) |
| imu08 | 577 | 125.22 | 126.16 | 129.16 | 7 / 36.937 | 21.422 | 199 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu08-seed577/signedreview.json) |
| imu12 | 433 | 187.04 | 187.92 | 190.92 | 9 / 11.798 | 14.420 | 196 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed433/signedreview.json) |
| imu12 | 467 | 152.58 | 153.50 | 156.50 | 10 / 8.055 | 19.952 | 198 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed467/signedreview.json) |
| imu12 | 479 | 212.70 | 213.58 | 216.58 | 10 / 8.825 | 14.833 | 196 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed479/signedreview.json) |
| imu12 | 541 | 103.62 | 104.48 | 107.48 | 8 / 22.808 | 12.291 | 195 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed541/signedreview.json) |
| imu12 | 571 | 106.26 | 107.16 | 110.16 | 9 / 14.585 | 17.374 | 197 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed571/signedreview.json) |
| imu12 | 577 | 124.86 | 125.76 | 128.76 | 8 / 28.588 | 15.719 | 197 | [서명](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/imu12-seed577/signedreview.json) |

전12에서 hold151개 50Hz truth표본의 네 발이 모두 pad에 양의 법선력으로 지지되고, off 이후3001개 1kHz motor표본의 PWM/output/static-force가0이었다. 네 발 외의 기체 부품·vehicle body·floor·net 충돌은 없다. 실제 모든 접촉과 hold는 직선이며 차량 속도는 연속0.25m/s였다. hold 발별 최소 법선력 0.028336N, 최대 상대 미끄럼 17.890µm, 최대 실제 tilt 0.000364°다. 이는 현재 강체/마찰/발 형상의 조건부 결과다.

## 원자료·소스·픽셀 결합

각 run/source-snapshot의 **실행 코드62개와 runtime/build-manifest10개**를 원 run_manifest SHA와 대조했다. 사전 core65에는 별도 config3개가 포함되어 code65라고 부르지 않는다. 모든12의 source/runtime는 동일하고 정책 내 frozen config는 같으며 두 정책간 다르다. imu08 frozen SHA `b2c0d6e3432ed10c67ded3407ee39d6c9be87d9b0a3026cb0a42bcfd2b132f92`, imu12 `bba87b2cfeb71cafb4083d7386cc5ac8991337d9d6cb5d9807968d1bd11d7da7`. 개별 seed/entry/series/split은 experiment.json의 별도 trial_metadata SHA로 결합했다. controller-settings는 frozen mission 공개 subset과 physical software override로 재구성해 일치했다.

v13 baseline과 달라진 코드는 estimator.py, 추가된 파일은 vertical_fit.py/test_vertical_fit.py뿐이다. runtime·controller·touchdown·physical scene은 동일하다. active observer 변경은 model=imu_compensated와 구간 .8/1.2초이며 기존 미지정 기본 linear_window/.4초를 보존했다. 설명용 vertical_fit_basis metadata도 추가됐으므로 v13 source/config와 byte동일이라고 하지 않는다. 원 sceneXML과 t=0 전체 truth값이 seed별 baseline과 일치해 차량 phase 변경이 없음을 확인했다.

| 실행 snapshot 파일 | SHA256 |
|---|---|
| scripts/mission3/estimator.py | `aee18061e2ea283e943adc285837d8124fb4a54e4e1a8835a03a1f459e0eb668` |
| scripts/mission3/vertical_fit.py | `35f6587922b0aaeded60d46d3ee1949a6ae7a9df677853ef3172d888e9d8f5e0` |
| scripts/mission3/test_vertical_fit.py | `f742f37f4b6c502a95af968f8a496feb4dd7bf0e1a64cb7492d2841a81717155` |
| scripts/mission3/physical_estimator.py | `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5` |
| scripts/mission3/physical_scene.py | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| scripts/mission3/controller.py | `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180` |
| scripts/mission3/touchdown.py | `b258eeea56689f03c7ff1c258c1703f666b514c3ff34761f10856d8f9b239f7b` |
| scripts/mission3/evaluation.py | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |

[source·config·phase 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/source-series-verification.json)와 [원파일 전체해시](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/protected-original-files-sha256.json)에 원12run의 snapshot/로그/평가/영상 등 및 원계획·summary 총1253파일을 결합했다. 원 evaluator를 snapshot에서 로드해 canonical relative run_dir로 평가를 다시 계산했고 원 offline-evaluation의 모든 필드와 일치했다. **FFV1 전체31,461프레임**은 mono320×320 uint8 원 controller_input SHA와 모두 일치했다. 이는 저장 픽셀 검증이며 AI-deck 전송/실제 PSF·초점·노출 검증이 아니다. 571 두 정책/577 imu12의 처음·중간·마지막 native sample을 직접 보았다.

처음203tests의 `directions` fixture KeyError와 수정 후203tests OK 로그를 모두 읽고 [시험·이전 replay 근거 해시](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/reference-and-test-evidence.json)에 보존했다. 독립 전체203개 재실행을 했다고 주장하지 않는다. 본인 수학 fixture6개와 raw 재생은 별도로 실행했다. 초기 본인 source diff 비교가 기본 .4초를 explicit config값으로 잘못 기대한 assertion도 execution.log에 남겼다. 실제 미지정 기본과 설명 metadata를 확인한 v2 감사가 전12 통과했다.

## 실제 시각·수직 재생과 API 빈틈

각 run에서 Flow와 소비 packet은100Hz, ToF producer는40Hz였다. stamp/newflag/producer 값이 일치했고 미래·중복 소비와 중복 새거리 융합은 없었다. ToF age는0/5/10/15/20ms이며 최초 IMU packet1표본, 이후10개 1kHz 표본 평균을100Hz packet에 넣는 합성 코드다. PC quaternion 예측 tick(.009/.019…s)과 소비 tick(.000/.010…s)의 위상이 다르다. 실제 radio capture time/IMU box average/동기화는 U다.

저장된 표면 분류와 valid 영상 수락을 고정하고, 원 .12 높이/.08 속도·영상 .3/.15 보정 순서를 적용한 sensor-only Z/vZ 산술 재생은 **전12에서 오차0**였다. 수직 fit 호출은 총 70,336회, 선형 fallback은0이다. PC 현재 회전×누적 body specific-force−g 계산도 packet과 일치했다. 이는 surface 분류·영상 검출·full EKF·전체 비행을 새로 검증한 재실행이 아니다. [전12 재생 결과](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/vertical-fit-all-cases.json).

**구체적 수정 권고:** vertical_fit의 검사는 range미래·중복/IMU gap을 거부하지만, IMU 이력 자체의 마지막 시각>now를 거부하지 않는다. 본인 fixture에서 now=.055s/마지막 range=.050s/IMU=.060s의 미래 가속도0→1m/s²만 바꾸자 결과−.020→−.015m/s가 됐다. 현재12run은 현재 packet만 append하므로 해당 미래 입력 사용0건이지만, 외부 입력 API는 미래 IMU를 거부하거나 causal trim해야 한다. [독립 fixture·반례](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/scalar-fit-independent-fixtures.json).

fit 거부 뒤 wrapper는 legacy 선형 기울기를 사용하고, .12 거리 높이 갱신은 fit검사보다 먼저 수행한다. 따라서 함수 거부는 전체 rangefusion 차단이 아니다. 외부 packet 수신에서 시각/유한값/중복/관측 상태를 먼저 검증하고 fit 실패 시 velocity 갱신 생략·불확실성 확대 등 명시 정책을 별도로 시험해야 한다. 또한 held40Hz 거리의 tilt/lever 변환은 producer 당시 자세가 아니라 소비 시점의 현재 PC 자세를 써서 과거 producer stamp를 붙인다. 최대20ms 이동·회전에 대한 time-alignment 감도를 시험할 필요가 있다. 현재 synthetic timing 통과를 하드웨어 동기화로 격상하지 않는다.

이 스칼라 회귀는 가속도 변위를 제거한 뒤 초기 기울기+현재 속도 증가를 계산한다. 수직 편향·gravity/tilt 오차를 식별하지 않고 Forster manifold VIO/factor graph도 아니다. 긴 구간의 잡음·편향 절충은 S다. 기존 원 replay의 .4초 IMU571 RMS10.931→12.746mm 악화와 479/577 일부 긴 구간 악화를 보존한다. [기존 근거 문서](../terminal-vertical-fit-20261009.md)의 두 replay는 새 착륙이 아니다.

## 571의 교집합 변화 / 577 retention

571 v13은 실제 접촉127표본(네 발96)이 있어도 off가 없었다. 기존 전체 지지 조건은110.66~110.86s, span .20초만 유지했고110.88s 추정높이26.615mm가 strict 상한26.5mm를 넘었다. 실제 높이는13.565mm였고 마지막 image age6s에서 실패했다. 그 진단 접촉 오차6.893mm는 성공 점수가 아니다.

| 571 조건 | terminal | probe | 실제 접촉 | 전 gate 연속 시작 | PWMoff / 연속 span | off 추정높이 |
|---|---:|---:|---:|---:|---:|---:|
| v13 baseline 실패 |106.90|109.80|109.92|110.66|없음 / .20s|없음|
| v14 imu08 |106.40|109.88|110.06|110.74|111.00 / .26s|20.767mm|
| v14 imu12 |102.98|106.04|106.26|106.90|107.16 / .26s|17.374mm|

height14.5±12mm(strict), confirm.25s, IMU near-g±.2, residual≥1.2, 예상 motoracc≤6.5, calibration45s/예측6s 문턱과 source는 그대로다. 새 두 비행은 그 교집합을 유지해 PWMoff와 실제 hold가 이어졌다. 이미지/궤적도 달라졌으므로 원 입력571에 대한 단일 인과 해결이나 안전 보장이 아니다. [교집합 CSV·비교 결과](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/seed571-and577-support-comparisons.json).

![571 높이·지지 교집합](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/seed571-height-and-support.png)

577은 baseline31.915mm→imu08 **36.937mm 악화**, imu12 **28.588mm 감소**이며 모두 실제 접촉 유지다. 실제 tilt/속도/scene은 [12개 closed-loop 비교](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/closed-loop-comparisons.json)에 분리했다. training 선택용 마지막 새거리 이후 height RMS 최댓값은 .8=5.058mm,1.2=3.927mm로 원 selection을 raw에서 재계산해 일치했다. 그 기준은 spent six에서의 선택이며 미래 holdout·실기 성능 승인 기준이 아니다.

## 유지되는 물리·센서 한계

항공기 qpos/qvel은 초기화 외에 직접 설정되지 않았고 vehicle만 독립 연속 prescribed .25m/s다. controller/PC PID/실행 wire/지원 관측기의 분기에는 contact/실제법선력/기체 위치·속도 truth가 들어가지 않는다. 센서 생성은 plant를 관측하는 합성 모델이고, truth 기반 floor/60° harness stop은 실험 중단 장치다. 실기 안전 감독기로 오해하면 안 된다. [source 경계와 근접 range 감사](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/source-boundary-and-near-range.json).

전12 normal명령3D속도≤.4, 가속도≤.15, yaw≤10°/s 및 yaw가속도≤20°/s² 등 원 caps는 유지했다. motoroff의 급격한 zero 명령 override는 따로 남겼다. **실제 최대속도는 .4013~.4129m/s**로 명령 cap와 다르다. PWMoff 시 실제 잔류 motorforce 총 0.2013~0.2023N, .3초 후 약6e-8N다. 공식 PWM곡선/battery extraction은 snapshot .so와 일치하지만20ms motorlag와4.2V 상수 배터리/무에너지·무SOC 모델은 S다. 실제 Mission1/2 잔여 비행시간을 검증하지 않았다.

HOLD 전12은 정확 ToF0·valid Flow0·surface ambiguous였다. IMU near-g/tilt/gyro 안정만으로 pad·네 발·미끄럼을 알아낼 수 없으므로 본인은 오프라인 실제 contact를 추가 확인했다. 정지 시 마지막 image age는 5.02~5.64s, hold 동안 더 오래된다. 최종 세계 위치 오차와 relative pad 오차를 구별해야 한다. 예를 들어577 imu12 최종 world오차170.9mm/relative오차102.9mm인데 실제 접촉은 유지된다. 공통 위치 gauge가 상쇄되는 상대식에도 blind 전파 오차가 남으며 이 값은 물리 접촉 오차28.588mm 또는 slip과 같지 않다.

PC독립 IMU 자세는 deck 융합이 없지만 하위 onboard fullEKF는 rawToF default lower gate0으로4cm 아래 새거리193/194…등을 계속 받는다(실제 개별값은 JSON). Z-only measurement H도 교차상관을 통해 다른 상태/자세에 영향을 줄 수 있다. producer accuracy=true인데 noise후 raw<4cm인1~4개 새표본도 있다. 가상 validity는 실제 ST RangeStatus/PMW SQUAL·motion/shutter가 아니다. optics K/D·실제 mount/CoM·질량/관성/추력·lag/contact/마찰/PSF/timing은 S/U 유지. 12초 preparation budget 역시 실행 강제상한이 아닌 planning S 가정이다.

참고용 main 소유 terminal-vertical-fit 문서는 검토 중07:06 완료 문단이 추가돼 별도 context-document-update-audit에 전후 SHA를 기록했다. 이 외부 문서 갱신을 불변 비행 원자료 변경으로 섞지 않으며 본인은 해당 문서를 수정하지 않았다.

최종 [서명 index](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/signed-review-index.json) / [원자료·서명 최종 검증](../../../artifacts/mission3/reviews/20261009-v14-vertical-training-independent/final-verification.json)에 모든 원파일/기존25서명 보존, 새12서명의 원평가·input/source/evidence SHA 결합과 승인 범위를 기록한다. qualified/formal/hardware0, 진입2·다음 시험 승인0이다.
