# v18/v18b 실패 전달 source 회귀8회 독립 검토

검토자 **Heisenberg / Codex-Independent-Flight-Reviewer**, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 실행 당시 각 run source-snapshot/원 평가·raw/native/runtime/config를 직접 검사했다. main 개발 판정이나 이전 승인 서명을 복제하지 않았다. 공유 source/config/work-log·원자료·기존 서명을 바꾸거나 새 physics 비행을 실행하지 않았다.

**v18 조건부 접촉2/4·착륙 실패2/4, v18b 조건부 접촉2/4·착륙 실패2/4.** 착륙 실패의 bounded delivery는 v18 두 건 모두 FAIL, v18b 두 건 모두 조건부 dispatch/관측 PASS다. 네 실패는 contact0/off없음으로 유지한다. 신규 positive4 서명/negative4 감사를 서로 다른 source/input/evaluation SHA에 결합한다. 전체8은 spent source regression이며 이전 성공·holdout·효능 표본과 합치지 않는다. 원장149→153→157은 참조 수일 뿐이다. **manufacturer/qualified/formal/hardware/진입2 및 실제 안전회수·모터정지 승인0.**

[개별 새 서명/거절 인덱스](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/signed-review-index.json) · [final-verification](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/final-verification.json) · [본인 원로그 전달/회귀 재구성](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/all8-delivery-and-reference-regression.json)

| source | 전체 training | baseline policy | wide policy | 착륙 실패의 bounded delivery | native exact |
|---|---|---|---|---|---|
| v18 | 2/4접촉·2/4거절 | 2/3접촉 | 0/1접촉 | 0/2 PASS,2/2 FAIL | 21,148 |
| v18b | 2/4접촉·2/4거절 | 2/3접촉 | 0/1접촉 | 2/2 PASS | 17,872 |

정책의3회와1회 분모를 따로 남긴다. all command caps8/8 PASS는 착륙/실패 전달/실기 안전 성공과 다른 판정이다. source64+dependency config3/runtime10=77(v18),65+dependency config3/runtime10=78(v18b)와 실제 사용 baseline/wide/physical config3을 각각 검증했다. 계획 core에는 runtime10도 포함되므로 원 source 수와 혼동하지 않았다.

v18 [원 plan](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18/plan.json) SHA256 `9545beeb0fb0b07a02c74fa6b2017502f1f71efa685ac95d38d22080d07ac374`. [원 whole summary](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18/summary.json)
v18b [원 plan](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18b/plan.json) SHA256 `3f3327484e256a8f48697d45a3f6a473d01f52ff0f3cbb8702b17c2551b2a2fe`. [원 whole summary](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18b/summary.json)

| source/policy/seed | 접촉/off/3s hold끝 또는 최초ABORT/종료 s | 본인 착륙 판정 | 본인 전달 판정 | 원 run / 새 검토 |
|---|---|---|---|---|
| v18/baseline_abort_delivery/571 | 106.26/107.16/110.16 | 조건부 접촉 9점/14.585mm | 미실행: 접촉 보존 회귀 | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18-entry1-seed571) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18-baseline_abort_delivery-seed571/signedreview.json) |
| v18/baseline_abort_delivery/619 | 243.76/244.64/247.64 | 조건부 접촉 10점/9.375mm | 미실행: 접촉 보존 회귀 | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18-baseline_abort_delivery-seed619/signedreview.json) |
| v18/baseline_abort_delivery/613 | ABORT309.60/종료385.000 | 거절 search_timeout_without_metric_pose | FAIL: reset/시간초과 | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18-entry1-seed613) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18-baseline_abort_delivery-seed613/failure-review.json) |
| v18/wide_abort_delivery/619 | ABORT296.58/종료385.000 | 거절 odometry_uncertainty | FAIL: reset/시간초과 | [run](../../../artifacts/mission3/physical/20261009-wide_abort_delivery-v18-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18-wide_abort_delivery-seed619/failure-review.json) |
| v18b/baseline_abort_delivery/571 | 106.26/107.16/110.16 | 조건부 접촉 9점/14.585mm | 미실행: 접촉 보존 회귀 | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18b-entry1-seed571) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-baseline_abort_delivery-seed571/signedreview.json) |
| v18b/baseline_abort_delivery/619 | 243.76/244.64/247.64 | 조건부 접촉 10점/9.375mm | 미실행: 접촉 보존 회귀 | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18b-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-baseline_abort_delivery-seed619/signedreview.json) |
| v18b/baseline_abort_delivery/613 | ABORT309.60/종료312.321 | 거절 search_timeout_without_metric_pose | 조건부 bounded 관측PASS | [run](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18b-entry1-seed613) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-baseline_abort_delivery-seed613/failure-review.json) |
| v18b/wide_abort_delivery/619 | ABORT296.58/종료297.721 | 거절 odometry_uncertainty | 조건부 bounded 관측PASS | [run](../../../artifacts/mission3/physical/20261009-wide_abort_delivery-v18b-entry1-seed619) · [검토](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-wide_abort_delivery-seed619/failure-review.json) |

**접촉4건 실제 확인:** 네 발만 pad를 지지하고 다른 aircraft/nonfoot/floor/marker 접촉이 없었다. off부터3초 동안151개50Hz truth 표본 모두 네 발 양의 법선력,3001개1kHz PWM/output/static force0, 실제 접촉/hold 모두 직선, 차량 속력.25m/s 연속이다. 각 archived .so의 공식 PWM curve와4.2V/lag recurrence, body rawIMU+기록된 PWM 기반 전체 freeflight calibration/support 재생, sensor-only FSM/직선 margin, ToF40/Flow100/newstamp/raw 전달, RPYT100Hz/14byte/yaw sign/caps를 검사했다. support 최대오차 허용1e-10 미만, main unsigned hold 필드 불일치0. 전체 latent PID/EKF/real RTOS/radio 재현을 주장하지 않는다.

| positive source/seed | hold 최소 발력 N | hold slip μm | 실제 max3D speed m/s | PWMoff 첫 force N / +.3s |
|---|---:|---:|---:|---|
| v18/571 | 0.033122 | 7.093 | 0.408150 | 0.2017033 / 6.17e-08 |
| v18/619 | 0.033200 | 4.567 | 0.405305 | 0.2021670 / 6.184e-08 |
| v18b/571 | 0.033122 | 7.093 | 0.408150 | 0.2017033 / 6.17e-08 |
| v18b/619 | 0.033200 | 4.567 | 0.405305 | 0.2021670 / 6.184e-08 |

실제 speed .408150/.405305m/s는 명령 cap .4m/s와 다르다. PWMoff 첫 step 힘 .201703/.202167N은 즉시0이 아니며 가정 motor lag20ms 뒤 감쇠한다. HOLD accurateToF0/validFlow0에서 IMU/model 지지는 실기 발·pad identity 관측을 대체하지 않는다. 준비13.50/14.88s는 planning12s보다 길다. source fixed preparation 예산과 runtime cap을 등치시키지 않는다. 네 positive는 v17 각 reference의 commands/truth/observations/motors/sensors/deck_samples/estimates/rpyt_packets/camera_frames/camera.avi10개 전체SHA가 byte 동일이다. 새 source/평가 binding은 필요하지만 새로운 착륙 능력이나 독립 효능4표본으로 세지 않는다.

**v18 최초 통합 오류:** failure 첫309.60/296.58s 뒤 pending delivery 분기는 있으나 독립 `if not handoff_active`가 `mission.update`를 계속 호출했다. done의 재처리가 매20ms 새 AbortDelivery를 만들어 시작시각/카운터/dwell을 리셋했다. 실제 zero velocity/yaw→RPYT/physics 전달은 있었다. 따라서 전달이 전혀 없었다고 해석하지 않는다. 그러나 원 첫ABORT를 기준으로 시간·누적 전달·종료 관측을 결합하는 계약에는 실패한다. 종료385.000s이고 첫ABORT 이후75.400/88.420s로3초 상한을 넘었다. 마지막 객체 요약만2wire/20physics로 전체7540/8842wire·75400/88420physics를 대표하지 못한다.

| negative source/policy | 객체 초기화 횟수 | firstABORT 이후 s | 실제 wire / physics | 마지막 source finish/dwell |
|---|---:|---:|---|---|
| v18/baseline_abort_delivery | 3770 | 75.400 | 7540 / 75400 | None / 0.00s |
| v18/wide_abort_delivery | 4421 | 88.420 | 8842 / 88420 | None / 0.00s |
| v18b/baseline_abort_delivery | 1 | 2.721 | 273 / 2721 | sensor_stationary_hold_observed / 0.52s |
| v18b/wide_abort_delivery | 1 | 1.141 | 115 / 1141 | sensor_stationary_hold_observed / 0.52s |

**v18b 수정의 실제 범위:** run snapshot의 `if not handoff_active and abort_delivery is None`가 위 재호출을 막는다. AbortDelivery는 max deadline 비교에만1e-9s roundoff 허용을 추가했고 dwell.5s/속도.05m/s/tilt10°/gyro5°/s/fresh+gap30ms/floor-usable 및 기존 landing/caps/scene/차량 phase/quality는 바꾸지 않았다. 최초ABORT와 failure는 고정, 두 negative 원9개 JSONL의 최초ABORT 전 prefixSHA가 reference와 모두 동일이다. 본인은 각각273/115wire와2721/1141physics를 재계산하고 원 snapshot 클래스의 매 control-tick 관측·카운터·summary를 재생해 오차0을 확인했다. stable floor 관측 연속 약.52s 뒤 source finish가 sensor_stationary_hold_observed로 끝나며2.721/1.141s의 종료는 사전3s+1physics step 안이다.

착륙 실패2개가 사라진 것은 아니다. 마지막 actual speed는.003125/.000638m/s, estimate는.004348/.001131m/s로 작아졌지만 실제 높이는.997753/1.000181m로 공중에 남고 PWM은 약4만으로 계속 nonzero다. 최종 worldXY 추정 오차.338568/.588670m와 wide positionstd1.002668m도 남았다. stationarity는 위치복원·불확실성 해소·착륙·안전회수와 다르다. 실제 braking과 estimate 비교에는 오프라인 truth만 사용했다. [baseline613 actual/estimate CSV](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-baseline_abort_delivery-seed613/failure-actual-vs-estimate-offline.csv) · [wide619 actual/estimate CSV](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/v18b-wide_abort_delivery-seed619/failure-actual-vs-estimate-offline.csv)

원 [v18 false 감사](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18/failed-delivery-regression-audit.json)와 [v18b whole4 감사](../../../artifacts/mission3/acquisition/20261009-abort-delivery-regression-training-v18b/failed-flight-delivery-regression-audit.json)는 읽기 전용으로 보존했다. 본인 검사가 먼저였고 해당 whole 결론과 이후 대조했다. v18 통합 실패를 v18b source로 재해석하거나 과거 결과를 수정하지 않았다.

**snapshot 통합시험의 독립 재현:** v18b에 보존된 actual adapter mocked-I/O fixture를 v18 wrapper에 적용해 둘 다 실패(미션550회호출,기대1)를 다시 관측했고, v18b wrapper에는 둘 다 pass했다. v18b stationary.501s/관측미충족 deadline3.001s이며 추가 unit5개씩도 pass. 이는 원 adapter 제어분기 실행이고 Plant/Motion/PID/영상/지원은 mock이다. 새 physics/radio/hardware 시험이 아니며 분모를 늘리지 않는다. 원209/211 전체 test log의SHA를 확인했지만 전체suite를 다시 돌렸다고 쓰지 않는다. [v18 재현](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/archived-adapter-fixtures-v18.json) · [v18b 재현](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/archived-adapter-fixtures-v18b.json)

**계속 남는 전달/감시 경계:** 실제 C host entry는 decoded RPYT→controllerPid→mixer를 직접 호출한다. 공식 firmware stabilizer의 commander queue/timestamp·supervisorUpdate/Override·arming/모터허용/RTOS 전체경로와 radio ACK를 실행하지 않는다. 원 host [flight_controller_host.c](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v18b-entry1-seed571/source-snapshot/scripts/mission3/flight_controller_host.c)와 별도 pinned 공식 checkout commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`의 supervisor watchdog500/2000ms를 대조했다. 이 값은 공식 소프트웨어 reference이며 이번 runtime에서 해당 watchdog이 작동했다는 증거가 아니다. [공식 source 보존/경계](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/supervisor-transport-boundary.json)

pure API에서 note_wire/physics가 전혀 없으면 start0/now10s에도 finishNone이다. delivered 조건 때문에3초는 최초 I/O 자체가 실패하는 경로의 절대 wall-clock 종료를 보장하지 않는다. 이8개 실제 비행에는 진행하는 I/O가 있었고 v18b 계약PASS를 취소할 근거로 일반화하지 않는다. 다만 실기 transport 종료/ACK 유무·관측 age/gap·스케줄러 지연/링크손실·supervisor와의 역할 및 nonfinite 입력 차단 뒤 상태는 별도 명세/시험이 필요하다. [본인 I/O 없는 API 반례](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/independent-delivery-counterexamples.json)

**센서/물리/truth 경계:** AST로 aircraft qpos/qvel은 init 뒤 처방 쓰기없음, continuous vehicle .25만 prescribed다. AbortDelivery는 sensor own state와 deck stamp만 받는다. wrapper contact truth는 command 기록 뒤 오프라인 로그·floor/tilt harness stop에서만 읽으며 실패 전달/접촉 candidate의 live 입력이 아니다. sensor producer는 conditional plant를 측정하는 합성모델이다. geometry/SQUAL·RangeStatus·PSF/mount/K/D/CoM/mass/inertia/gain/lag/friction/actual capture/radio/RTOS timing S/U, fullonboard near rawToF gate default0 및 PC independent IMU 자세의 한계는 남는다. constant4.2V에 SOC/energy/mission1·2 reserve모델이 없고 v18 failed385s 비행이 stock payload endurance 증거가 될 수 없다.

전8개 기록된 surface/image acceptance를 고정한 Z/vZ 산술 재생 오차0/fallback0/future·duplicate0이다. 동일 causal-IMU guard source afba09를 확인했지만 scalar regression은 Forster manifold VIO/편향 추정기나 전체 range fusion chronology gate가 아니다. held40Hz range에 consumer-time tilt를 적용하는 최대20ms 시각 불일치와 IMU box 평균 합성계약은 이번 ABORT 수정으로 해결되지 않는다. [전8 재생/시각/검사범위](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/vertical-fit-all-cases.json)

**구체적 피드백:** v18의 false 판정과 source를 그대로 보존하고 v18b의 수정은 실패 이유/최초시각 latch·bounded 관측 종료 범위로만 승인한다. 후속 real adapter는 simulation break 이후 누가 어떤 setpoint를 얼마 동안 유지하는지, ACK/실행확인과 독립 wall-clock/link-loss/supervisor 계약을 구체화해야 한다. static floor quality가 불가능하거나 비유한 입력인 경로를 stationary라고 대체하지 않는다. 실제 aircraft 근접 품질/에너지/예측경계는 별도 근거가 필요하며 landing/std/support 문턱을 넓히는 권고가 아니다.

감사 도구 자체의 최초 plan source-filter assertion(runtime10도 core에 포함) 및 불필요한 scalar helper 중복 호출의 FileExistsError 로그를 보존했다. 필터를 고친 source 검사는8/8 완료했고 scalar helper의 최초 정상 invocation이전8을 검사한 결과를 각각 확인했다. exclusive-create 오류는 원 비행·평가 오류가 아니며 출력 덮어쓰기/physics 재실행 없이 처리했다. [검사 스케줄 범위](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/vertical-fit-execution-final.json) · [본인 metadata 정정](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/own-audit-metadata-corrections.json)

시작 hash inventory1,262개 가운데 실행 원자료·plan/config/log1,261개 및 기존 서명63개는 불변이다. 공유 비실행 참고 문서1개는 아래와 같이 감사 중 변경을 관측했다. 신규4positive/4negative는 본인 ID·실행 source/runtime/config/trial/full evaluation-file SHA/canonical digest 및 actual audit evidence에 결합하고 전8 완료 후 final/index를 검증한다. 공용 파일·원 평가·이전 서명은 그대로다.

실행 source의 핵심SHA(현재 source로 대체하지 않음):

| source | 원 wrapper SHA256 | 원 AbortDelivery SHA256 |
|---|---|---|
| v18 | `76b1b8a515bd0feb56d43e8a85f6d8f945228474c3a3b4848116ac507cbcbb6c` | `d79d0892d266dc8290b3945c65a854fbd5fbe5b592951ab005ea62db70babd83` |
| v18b | `1883b457695c21fb50ed07ac708b87dee2effb41fda259e9409f841bba0f4b3b` | `a75e5e3dff632f3a3b186d6fb6ed09002bcf3ac7dd754ee3cfe6c144661fbc27` |

v18b actual-adapter fixture SHA256 `1b61a3d745cfc589e69cc59e76303edd7bed867ec8dde877d8249c3130572928`; unchanged controller `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180`, physical estimator `e35fee9d6674a7aedacff3047ba6fae1fa81a4a34aa82a112f352a4aaf971bd5`, evaluator `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0`. 전체64/65 source/runtime/config SHA는 각 개별 원결합 결과와 [source-series-verification](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/source-series-verification.json)에 보존했다.

**공유 참고 문서 변경의 분리:** `docs/mission3/abort-delivery-contract-20261009.md`의 최초 읽기SHA `c73f44c488fded3ed4c721a73ece8ecc047b1bdb82220c1908be51ad18579975`→종료 검사SHA `6b9d0d3fef9379da6b5b6ee6057619e567fe576820c02ca56f0cff532b2af1c0` 변경을 관측했다. 본인은 이 공유 문서를 수정하지 않았다. 두 원 plan의 문서SHA `e213425052add224a518eff5741324c9c2bb1bf6130ba08924346a54b28cbf25`에 해당하는 당시 문서 revision은 이번 inventory에서 확보하지 못했으며 현행 문서를 과거 계약으로 대체하지 않는다. 실제 판정은 original snapshot 코드·frozen config·원raw·원평가와 사전 plan JSON의 숫자/판정 계약으로 도출했다. 실행 원run/source/runtime/config/evaluation 및 prior signatures 변화는0이다. [문서변경 근거](../../../artifacts/mission3/reviews/20261009-v18-abort-delivery-regressions-independent/mutable-document-reference-change.json)에 별도 기록하며 전체 inventory가 모두 unchanged라는 주장은 하지 않는다.
