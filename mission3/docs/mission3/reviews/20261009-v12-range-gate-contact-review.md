# v12 raw ToF 하한 게이트: 독립 조건부 접촉 검토

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 실행 저자 `main-flight-adapter`와 별도 판정이다. v12 재사용 training 4건 중 433·467·479는 조건부 물리 접촉 계약을 승인하고, 457은 모터 종료와 3초 유지가 없어 거절한다. 제조사 일치·실기·qualified·formal 승인은 모두 0이다. v11 training/holdout, v13 및 다음 진입의 승인과 합산하지 않는다.

원 [사전 계획](../../../artifacts/mission3/acquisition/20261009-onboard-raw-range-training-v12/plan.json)의 네 실행을 각 원 `source-snapshot`으로 검토했다. [독립 결과](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/independent-series-summary.json)와 각 seed 폴더에 원파일 SHA, 새 원평가 재계산, 접촉·센서·명령 감사, 필터 차이 CSV 및 새 서명을 보존했다. 원평가 모든 필드는 재계산과 일치한다. informational `run_dir`는 원 ROOT 상대 경로로 맞췄으며, 원평가 파일 SHA와 canonical evaluation digest를 별도로 결합했다.

| 재사용 training | 첫 실제 접촉(s) | PWM off(s) | 3초 유지 끝 / 실행 끝(s) | 내부 점수 / 착지 XY(mm) | 독립 판정 |
|---|---:|---:|---:|---:|---|
| 433 | 191.40 | 192.26 | 195.26 / 195.28 | 9 / 15.253 | 조건부 접촉 승인 |
| 457 | 190.44 | 없음 | 없음 / 197.22 | 접촉 오차는 미션 통과가 아님 | 거절 |
| 467 | 156.88 | 157.76 | 160.76 / 160.78 | 8 / 21.122 | 조건부 접촉 승인 |
| 479 | 213.48 | 214.42 | 217.42 / 217.44 | 10 / 7.472 | 조건부 접촉 승인 |

각 성공의 hold 참값 151개 모두 네 발이 pad에 닿고 법선력이 양수였다. 발당 최소 힘은 433/467/479 각각 .033331/.032244/.032028N, off 위치 대비 최대 XY 미끄럼은 7.42/10.25/3.63µm였다. 3초 동안 1kHz 모터 표본 3001개 모두 PWM/output/static-force가 0이다. 전체 접촉의 다른 표면·비발 접촉은 없으며, 선언한 하강부터 유지 종료까지 차량은 직선, 전체 차량 속력은 .25m/s였다. 원 모터 곡선과 가정 20ms 지연의 힘 재귀를 실제 로그에 대조했다. PWM off 첫 참값의 실제 합력은 .20235/.20203/.20134N으로 남고 .3초 뒤 약 6.19/6.18/6.16×10⁻⁸N이다. PWM 종료를 기계적인 즉시 정지와 동일시하지 않는다.

세 성공 모두 정상 명령 .4m/s·.15m/s²·yaw10°/s·yaw가속도20°/s²·상승.10/하강.06m/s를 통과했다. 모터 off의 긴급 명령 전환은 예외로 그대로 기록했다. 실제 최대 3D 속력은 .40594/.40605/.40823m/s로 명령 .4 한도를 넘는다. 명령 포화가 실제 운동의 동일 한도나 실기 안전성을 보장하지 않는다. 세 실행의 `development-review-unsigned`와 대조한 hold/센서/실제 속도/최종 world 오차 필드에 불일치는 없었지만, 개발자의 문서로 독립 승인을 대신하지 않았다.

433/457/467/479의 FFV1 3511/3558/2806/3943프레임, 합계 13,818장을 직접 디코딩해 원 controller-input SHA와 모두 일치함을 확인했다. mono8 320×320, capture/delivery 순서, 실제 Flow100Hz·ToF40Hz 및 `range_new`·stamp를 대조했다. 이 보관 일치는 실제 AI-deck 통신·노출·PSF의 검증이 아니다. 세 성공 HOLD의 정확 ToF·유효 Flow·사용 가능한 Flow는 모두 0, 표면 추론은 ambiguous이다. IMU/모터 모델의 안정성만으로 네 발과 pad를 식별할 수 없으며 접촉 승인은 오프라인 참값에 의존한다.

v11 대비 실제 코드 차이는 `physical_estimator.py` 한 파일과 추가 검사 `test_onboard_range_gate.py`다. 제어기·장면·touchdown·C 코드·실행 라이브러리는 동일하다. v12 실행 스냅샷 code60/runtime10 및 같은 frozen configuration을 네 건 모두 확인했다. 계획 core63 중 부가 config3과 두 실행 profile/config 원파일은 스냅샷 밖의 현재 파일을 계획 SHA와 일치한 경우에만 대조했으며, 실행 설정은 각 원 `frozen-experiment-settings.json`·`experiment.json`으로 직접 확인했다. 이를 과거 v11과 코드 전체가 동일하다는 주장으로 바꾸지 않는다. 원 200-test 로그 SHA와 새 두 게이트 검사 코드를 확인했으며, 이 검토에서 200개를 다시 실행했다고 주장하지 않는다.

중요 SHA256:

| 대상 | SHA256 |
|---|---|
| v12 physical_estimator.py | `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5` |
| 새 게이트 검사 | `6fb282b95c8cb6a567f8555bca7431a684c45c26c518cd2a9b97e49a8a023466` |
| 유지된 controller.py | `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180` |
| 유지된 evaluator | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |
| v12 frozen configuration | `e13856a671a607e784f8fb843fc8f2e886e5896b0e23fdc090d92a889c1ef2db` |
| 원 200-test 로그 | `6cfe4afe867116c7830e0467663d39984e7da9397944c2c61d529d6503f7072c` |

게이트의 효과는 제한적이다. 433·467·479의 commands/truth/observations/motors/RPYT/deck_samples/sensors 7개 원파일과 카메라 원자료는 각각 v11과 SHA가 같다. mission estimate 전체와 독립 PC 상태, onboard quaternion도 정확히 같다. 따라서 새 평가 결합의 조건부 접촉은 승인하되, 추가 독립 착륙 조건·능력 증가로 계상하는 수는 0이다. 원 v11 서명을 복사하지 않았다. byte 동일 입력과 유지된 touchdown/controller/C 소스를 확인한 뒤 예전 센서 기반 support/직선 준비/패킷 분석 일부만 출처·SHA와 함께 재사용했다. 접촉·힘·PWM·명령·원픽셀·원평가는 v12에서 다시 계산했다.

| seed | 새 range 거부 수 | onboard Z 최대 차이(mm) | onboard q 최대 차이(°) | 첫 저장 estimate 차이(s) |
|---|---:|---:|---:|---:|
| 433 | 197 | 58.040 | 0 | 190.06 |
| 457 | 167 | 58.029 | 1.95613 | 189.48 |
| 467 | 195 | 39.258 | 0 | 155.60 |
| 479 | 198 | 27.397 | 0 | 212.24 |

하한은 새 raw≥.04m만 검사하며 held ToF를 다시 융합하지 않는다. 제조사 부품 문서를 근거로 메인 개발자가 선택한 시험 정책S이고 실제 deck 정확도 보장이 아니다. 노이즈 이전 거리로 정확도를 표시하는 조건부 producer와 달리, raw에는 4mm Gaussian 잡음S가 들어간다. raw가 4cm를 넘어 융합됐지만 producer의 정확도 표시가 false인 표본이 433/457/467/479에서 2/2/5/1개 있었다. 예컨대 467의 156.13s raw .04000153m도 그러하다. [구체 표본](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/seed467/raw-threshold-noncertification-cases.json). 이를 수정하려고 시뮬레이터 참값 기반 flag를 실기 제어 입력으로 넣어서는 안 된다. 실제 driver validity/RangeStatus·품질·timestamp 경계가 필요하며 아직 U이다. 게이트는 기존 stationary-ground 모델의 pad 기준 오해나 잡음 공분산의 적합성도 고치지 않는다. 근거리 C 표준편차 약2.5mm와 주입 잡음4mmS의 차이는 별도 검증 대상이다.

공식 펌웨어 commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`의 [mm_tof.c 보존본](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/official-source/mm_tof.c:30)은 H를 Z-only로 시작하고 거리 예측을 `S[Z]/cos(angle)`로 만든다. SHA `0cd4c17664a456091d6113e742feb0bde239b98e4fd8efdcd7adccf28dbcb7c8`는 실행 당시 build manifest와 일치한다. pad 높이 .30m나 mount lever 보정이 없고, 여기의 15°는 C 측정 모델 상수이지 조립 ToF 화각 확인값이 아니다.

[kalman_core.c 보존본](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/official-source/kalman_core.c:240), SHA `d0215770d05b213a4e19a91fe37ce5de802f33c826be8b9e44cfc9a31a82ea2f`에서 `K=P Hᵀ/(H P Hᵀ+R)`로 모든 상태를 갱신한다. H의 Z만 비영이라고 K까지 Z-only인 것은 아니다. 예측 Jacobian은 Z와 D0/D1/D2를 결합하고, finalize는 error attitude가 .0001rad를 넘으면 quaternion에 반영한다. 세 성공의 quaternion 동일성은 이 입력의 결과이며 항상 attitude 불변이라는 증거가 아니다. 저장 공분산은 대각선뿐이라 PᵢZ 전체나 단독 원인 기여를 재구성했다고 주장하지 않는다.

보존된 `firmware_host.c`/`firmware.py`의 onboard/pc_imu core는 별도 상태·공분산이며 PC에는 deck update API가 없다. `physical_estimator.py`의 PC 중력 보정/상위 제어는 독립 IMU100Hz 자세, RPYT 하위 자세 PID는 full onboard EKF 자세를 받는다. 따라서 onboard 영향은 하위 모터·plant·다음 IMU를 통해 나중에 PC에도 돌아올 수 있다. 457의 첫 motor 차이는 192.481s, RPYT/deck/sensor 차이는 192.49s, command/truth 차이는 192.50s이고 종료는 197.22s다. 원 v11은 197.24s였다. 후반 1.956°를 초기 near-ToF의 단독 효과나 개선으로 단정하지 않는다. 동일 시간 원로그 비교이며 동일 IMU/deck 입력의 fullEKF 인과 재생은 아니다. 독립 계산 뒤 대조한 [메인 참고 분석 비교](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/main-paired-reference-comparison.json)의 확인 필드는 일치했다.

457은 190.44~192.94s pad 접촉124표본, 네 발 양의 지지91표본을 갖지만 PWM 종료는 없었다. raw IMU/PWM으로 전체 support gain·residual을 재생해 scalar 오차0과 calibration stamp 일치를 확인했고, terminal sensor gate 재생에서 후보0/모든 gate 동시 통과0이었다. 추정 높이는 191.16s에 26.5mm 상한을 벗어나고 예상 모터 가속도≤6.5m/s²는 191.20s에 충족한다. 마지막 qualified image 186.22s 이후 6초가 된 192.22s에 prediction expiry가 발생했다. 실제 접촉 중 추정 높이가 상승하는 관측은 유지되나 IMU/초기 vz/영상 높이 anchor 각각의 기여는 미분해다. 높이 문턱 확대나 예측 예산 연장을 권고하지 않는다.

후속 소프트웨어 시험은 같은 IMU/deck 입력으로 fullEKF 두 분기를 재생해 PᵢZ·innovation·K·D-state·finalize 전후를 기록하고, 실제 하위 PID 사용 시점의 자세 차이를 분리하는 것이 타당하다. ToF 융합은 근거리뿐 아니라 관측 표면의 기준계가 모델에 적합한지 센서 관측으로 제한하는 별도 paired 시험이 필요하다. 457은 마지막 qualified 높이/vz와 IMU 순수 전파를 먼저 대조한다. 어떤 제안도 online truth 보정·실제 접촉 표면 입력·통과를 위한 문턱 조정을 포함하지 않는다.

실행 스냅샷의 항공기 qpos/qvel은 초기화 뒤 강제로 고치지 않는다. 차량의 처방 .25m/s 운동은 허용된 조건이다. plant 참값은 합성 센서 생성·오프라인 평가 및 명시된 floor/tilt60 harness 중단에 쓰이며 touchdown/controller 분기로 주입되지 않았다. harness는 실제 onboard supervisor가 아니다. mount/K/D·조립 질량/CoM/관성·motor lag/공통 힘 모델·마찰·광학/Flow 품질·IMU 평균·실제 radio timing은 U/S로 남는다. 일정4.2V는 SOC/전류/배터리 에너지/M1·M2 잔량 모델이 아니며 1m hover fixture도 실제 Mission2 인계 경로가 아니다. 이 제한 때문에 조건부 계약 통과를 실기 승인으로 격상할 수 없다.

새 [네 판정과 SHA 색인](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/signed-review-index.json), [원자료·기존 서명 불변 최종 검사](../../../artifacts/mission3/reviews/20261009-v12-range-gate-independent/final-verification.json)를 제공한다. 쓰기는 이 신규 문서와 신규 독립 검토 폴더에만 수행했다.
