# PC-RPYT / full onboard EKF 실패 독립 검토

검토자: **Heisenberg / Codex-Independent-Flight-Reviewer**, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 대상 원 실행은 [054951 seed1](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/run_manifest.json)이다. 실행 당시 source-snapshot과 런타임 바이너리를 검토했다. 이후 수정된 현재 파일을 과거 구현으로 간주하지 않았다. 분석과 재평가는 읽기 전용이며 truth는 추론 완료 후 비교·접촉 판정에만 사용했다. runtime, 원 로그·평가, 과거 서명은 수정하지 않았다.

**판정:** 원 실행의 PC 자세 경로가 수평 속도 추정 오류를 키운다는 근거가 강하고, 작은 초점 여유가 이를 영상 소실로 연결한다. 개별 Flow 또는 ToF 입력이 얼마만큼 자세를 손상시켰는지는 아직 분리되지 않는다. 같은 synthetic seed1의 PC 자세 분리 시험은 접촉으로 끝났으나, 새 동결 holdout 5회는 실제 패드 통과 **2/5**였다. 이번 검토에서는 training 5건과 holdout 2건을 개별 접촉만 승인하고 old53을 거부했다. 모두 `qualified=false`이며 제조사 조립 일치·실기 성능·공식 승격을 승인하지 않는다.

## 1. 원 실패에서 실제로 커진 오차

원 실행은 133.32s ACQUIRE → 134.90 LINEUP → 142.96 LOWER_ALIGN → 148.30 APPROACH → 153.62 TRACK → **154.62 visual_age 복구** → **156.60 visual_loss_after_discovery ABORT**였다. 실제 접촉 0, motor-off 없음, accepted 영상 421개다. paired baseline `060138-350364`는 truth, motors, RPYT packets, observations, camera frames **5개 파일의 바이트 SHA가 원 실행과 동일**하며 같은 실패를 재현했다. [원본·baseline 대조](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/source-and-input-integrity.json), [이벤트](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/events.json).

아래 x는 aircraft에서 pad로 향하는 세계 x 상대거리다. velocity error는 PC 추정 minus 실제 항공기 속도다. 값은 동일한 50Hz 기록 시각에 정렬했다.

| 시각 s | 실제 rel x m | 추정 rel x m | 상대거리 오차 m | 실제 vx m/s | PC vx m/s | PC pitch−실제 pitch ° | 실제 광학 depth m |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 153.60 | .112321 | .115306 | +.002986 | .290879 | .260107 | −.202703 | .342320 |
| 154.40 | .056416 | .072938 | +.016522 | .363235 | .223670 | −.941135 | .286440 |
| 154.60 | .032177 | .072195 | +.040018 | .380175 | .208489 | −.944178 | .262192 |

153.60→154.60의 PC vx 오차 변화는 **−.140915m/s**다. 같은 body IMU를 PC roll/pitch와 실제 roll/pitch로 각각 중력 보정해 얻은 수평 가속도 차이의 사후 적분은 **−.141269m/s**로, 차이가 .000354m/s다. 154.40의 수평 가속도 차이는 −.161005m/s²다. 약 .94° pitch 오차로 생길 수 있는 중력 투영 오류와 일치하는 크기다. 이는 유력한 작동 경로의 수치적 근거다. 100Hz IMU 평균과 50Hz truth 정렬, 다른 속도 갱신의 존재 때문에 모든 오차를 이 단일 항에 정확히 귀속하지 않는다. [전체 정렬 CSV](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/aligned-flight-diagnostics.csv), [진단 수치](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/diagnostic-summary.json).

![원 실행의 상대거리·속도·자세·초점 변화](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/original-terminal-failure.png)

### 영상 pose와 공통 gauge를 구분

마지막 accepted 영상은 capture 154.26 / receipt 154.32s다. 해당 capture의 raw relative x는 실제값보다 **+.637mm**, y는 +.343mm, z는 +6.653mm였고 fit RMSE .200px, marker 크기 89.42px였다. 사후 실제 자세로 같은 fit 벡터를 변환해도 x 오차 +.629mm다. fit 자체가 PC 자세를 사용하므로 이를 완전히 독립적인 영상 측정이라고 부르지는 않는다. 그러나 마지막 영상의 상대 x 자체가 4cm 틀렸다는 설명은 기록과 맞지 않는다. 이후 전파·속도/FF 갱신 구간에서 차이가 커진다. [capture별 pose CSV](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/capture-pose-diagnostics.csv).

153.60의 own world-x gauge 오차 −.176493m와 pad world-x 오차 −.173507m는 대부분 공통 이동이며 상대오차는 +.002986m다. 154.60에는 각각 −.271256m / −.231238m로 벌어져 상대오차 +.040018m가 된다. world 위치 오차를 곧 착륙 상대오차로 해석하면 안 된다. gauge anchor는 두 추정 위치를 같이 이동시킬 수 있으며, 공통 이동으로 취소되지 않은 차이와 전파 속도 오류를 따로 확인해야 한다.

FF도 153.60의 .237843에서 154.40 이후 .211274m/s로 실제 차량 .25보다 낮아졌다. 같은 추정 속도·위치 경로로 차량 속도를 회귀하는 구조이므로 자세/속도 오류가 FF에 다시 들어갈 가능성이 있다. 이 로그만으로 FF bias, 자세 오류, anchor 효과를 독립적인 세 원인으로 분해할 수 없다.

## 2. source에서 확인한 자세 오염 경로와 한계

원 [physical_estimator.py:51](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/physical_estimator.py:51)는 `full_onboard_ekf`에서 새 ToF의 0≤distance<5m이면 core에 입력하며, 이 분기에서는 `range_accurate`나 floor/pad 정체성을 검사하지 않는다. Flow는 raw-valid/count 범위로 core에 입력하며 PC surface observer의 floor-pure/moving 분류로 차단하지 않는다. 매 측정 finalize 후 얻은 core roll/pitch가 PC 중력 보정과 capture 자세에 사용된다. PC heading은 별도 관측 경로이므로 core yaw 전체를 그대로 쓰는 구조와는 구분한다.

공식 [mm_flow.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/kalman_core/mm_flow.c)는 world z와 정지 지면을 바탕으로 Flow를 해석하고, [mm_tof.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/kalman_core/mm_tof.c)는 z/cos(angle)로 지면 거리를 예측한다. H가 z/v 위주여도 [kalman_core.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/kalman_core/kalman_core.c)의 scalar update는 P 교차 공분산으로 attitude-error state를 갱신하고 finalize가 이를 quaternion에 흡수한다. pad가 시야에 들어온 뒤 world-floor 모델과 센서 표면이 달라질 때 자세가 영향을 받을 수 있다. 실제 154.40 core z error는 −.303758m다.

| 원인 후보 | 현재 증거 | 아직 확인하지 못한 부분 |
|---|---|---|
| fullEKF Flow/ToF ground 모델 → core 자세 → PC 가속도 | source 경로, z 약−.30m, pitch 약−.94°, 중력 보정 차이와 vx 오류 변화 일치 | Flow와 ToF 각각의 update innovation/K·자세 점프는 미기록 |
| PC rawIMU 중력 보정 | PC 자세 분리 intervention이 동일 seed1에서 실패→접촉 | 분리는 vision 자세와 Euler yaw-rate 경로도 함께 바꿔 gravity만의 인과 효과는 미분리 |
| focus/standoff 여유 | 원 .08m 기준 nominal depth 여유 .01m, 실제 depth가 .30m 아래로 내려가며 focus censor | 실제 out-of-focus 영상의 ArUco 검출률·렌즈/장착 보정 U |
| RPYT 부호·packet cadence | archived decoder/serializer 부호 및 100Hz/1kHz cadence 대조 오류 0 | 실제 radio/RTOS/supervisor, 정확 PC PID 호출 인자 replay는 미확인 |

원 1kHz IMU의 전체 입력과 매 update의 P/K가 저장되지 않았으므로 100Hz endpoint로 정확한 fullEKF 재현을 주장하지 않는다. 공식 입력 채널별 비교는 미래 고정 replay/paired 시험이 필요하다. 온라인 truth로 자세·속도·bias를 교정하는 방안은 이 검토에 없다.

## 3. 초점 여유와 RPYT 경계

첫 strict-focus censor는 154.30s, 실제 depth .297349m였다. scheduled 20fps 카메라는 50Hz loop에 맞춰 .04/.06s 간격이고 transport delay .06s다. 실제 기하에서 **84회 focus censor를 재구성했으며 원 rendered3133−실제frames3049와 일치**한다. 마지막 accepted capture가 154.26이므로 154.62의 capture age .36s가 .35s 한계를 넘는다. TRACK 2s 유지가 끝나기 전 영상이 끊겼다. [focus schedule CSV](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/original-focus-schedule.csv).

S 장착에서 camera body-x .02m, pad→marker-x .25m이면 level depth≈.23+relx다. 목표 relx .08은 nominal depth .31m로 .30m 정책까지 **10mm** 여유뿐이다. track tolerance .035m가 허용하는 relx .045이면 depth .275m다. 따라서 완벽한 추정에서도 위치 허용 범위와 초점 정책이 충돌할 수 있다. 실제 제조사 DOF를 simulation의 binary 검출 차단으로 구현한 것은 S 정책이며, 경계 밖 실기 검출 실패가 보장된다는 뜻이 아니다.

후속 .30m standoff는 메인 개발자가 선택한 S 시험 정책이다. `.405*tan(21°)+.10+.035≈.290m`를 반올림한 것으로 nominal depth .53m를 준다. cone/장착/조립 CoM은 U이고 이 scalar가 제조사 장착 사실을 확정하지 않는다. margin만 늘린 실행도 아래처럼 실패했다.

원 RPYT packet **15660개/.010s**, motor sample **156600개/.001s**를 대조했다. `<fffH` payload의 CF roll/pitch 그대로, wire yaw=−desired yaw, stop payload 14 zero bytes, held payload와 new-flag의 일치에 오류 0이었다. PC official velocity PID의 pitch/roll 변환과 공식 decoder의 yaw sign, gyro-y/pitch legacy 변환을 source에서 확인했다. [protocol/PID audit](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/rpyt-packet-pid-audit.json), [archived position bridge](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/position_bridge_host.c).

단, logged own state는 PID 호출 전 vision/map-height/gauge 갱신보다 먼저 기록된다. 이를 그대로 재입력한 replay는 max roll .006965°, pitch .003748°, collective 401.25 counts 차이가 있다. **정확한 PID 호출 인자 replay를 통과했다고 하지 않는다.** 다음에는 PID에 넘긴 position/velocity/attitude, capture 평균·stamp를 호출 시점에 기록해야 한다. gross packet 부호·주기 오류가 관측되지 않은 것과 실제 통신 경로 검증은 별개다.

## 4. 같은 seed1의 4조건 paired 결과

[사전 plan](../../artifacts/mission3/paired-rpyt/20261008-060137-plan/plan.json)의 4실행은 source 40개/runtime 10개가 동일했고, PC 자세 선택과 standoff 두 knob를 바꿨다. 원 실행은 39개 source/10개 runtime snapshot 모두 manifest SHA에 맞았다. 모든 offline evaluation은 당시 evaluator로 재평가해 정보성 `run_dir`를 원 상대경로로 맞춘 뒤 **전 필드 동일**했다. 평가 알고리즘/hash를 바꾸지 않았다.

| 조건 / run | 실제 결과 | 독립 판정 |
|---|---|---|
| baseline `060138-350364` | 원 실패와 5개 주요 로그 바이트 동일 | 실패 보존 |
| PC IMU만 `060138-311753` | contact162.40/off163.14, 8점/27.530mm | 접촉만 승인 |
| margin만 `060432-232760` | hull/partial contact, off 없음, 60° harness 중단 | 실패 보존 |
| combined `060432-216714` | contact162.86/off163.60, 7점/36.516mm | 접촉만 승인 |

이는 **이 synthetic seed1에서 PC 자세 공급 경로가 인과적으로 기여**한다는 증거다. onboard fullEKF는 그대로 유지하면서 PC만 IMU-only context로 분리했다. 서로 다른 trajectory의 같은 시각을 비교해 단일 측정의 효과라고 단정하지 않으며, training 2건을 새 holdout으로 세지 않는다.

### margin-only: 잘못된 속도 상태가 복구 saturation으로 이어짐

[margin-only 진단](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/followup-summary.json)의 최초 접촉 161.10s는 feet가 아니라 `hull_motor_2/3`였다. 실제 relx −.133826m인데 PC는 −.008552m, 실제 vx .187370인데 PC는 .499579m/s, FF는 .319626m/s였다. pitch error +2.9386°가 동반됐다. 전체 접촉 표본 277개에도 contact 후보의 모든 조건이 유지되지 않아 off하지 않았다.

163.58s handoff 후 6s 만료 → BLIND_RECOVER, 165.58s → BLIND_RISE였다. PC vx/실제 vx는 각각 **2.968/.246 → 8.085/.246 → 13.338/.246m/s**로 벌어졌다. RPYT pitch는 이 세 지점에서 +20° saturation이었다. recovery의 속도 명령 자체는 낮아졌지만 velocity PID가 큰 잘못된 own velocity를 계속 보므로 saturation이 남는다. terminal 전파 feedback은 만료 전 −5m/s command까지 도달했다. 166.735s harness가 실제 tilt60.00096°에서 종료했으며 마지막 truth 표본166.72는59.421°다. [명령/상태 CSV](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/margin-recovery-command-window.csv), [실제 접촉 표본](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/margin-contact-and-command-samples.json).

차체/hull 접촉 constraint가 기울기 증폭에 기여할 수 있으나 S 접촉 기하에서의 관측이다. 실기 복구 동역학으로 일반화하지 않는다. **6s 제한 완화가 해법은 아니다.** 관측 가능한 속도 한계, PID 적분기·saturation 처리, 이미 부분 지지 중인 복구의 위험을 새로운 frozen negative 시험으로 평가해야 한다.

## 5. 첫 동결 5회: 실제 패드 2/5

[frozen five plan](../../artifacts/mission3/frozen-series/rpyt-independent-entry1-five-v1/plan.json)의 unused43/47/53/59/61은 combined PC-IMU/.30 조건의 동일 code dictionary, configuration SHA, runtime SHA로 실행됐다. [동결 정체성과 분모](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/frozen-five-identity.json).

| seed / run | offline 실제 패드 판정 | 설명 |
|---|---|---|
| 43 `060809-039377` | fail | 두 발 pad 걸침 후 차체 접촉, off 없음 |
| 47 `060809-048471` | pass | off104.42/3s, 6점/47.372mm, 개별 승인 |
| 53 `060809-083394` | **fail** | off290.32/3s 안정이나 4발 모두 vehicle_body, pad0 |
| 59 `061219-709242` | pass | off141.86/3s, 4점/64.043mm, 개별 승인 |
| 61 `061219-757455` | fail | 횡방향 pad miss/floor contact, off 없음 |

adapter 종료 후보 3건을 실제 pass 3건으로 세면 안 된다. 판정은 **2pass/3fail**이고 실패를 분모에 유지한다. 형식 qualified0/current entry1과 사용자가 허용한 conditional 방향 연구 coverage는 구분한다. 이 5회에서 패드 성공 5건을 확보하지 못했지만, 제조사 미확정만으로 별도 연구 방향을 영구 금지하는 해석은 하지 않는다.

### seed43: blind overshoot이며 시간이 짧아서 실패한 것이 아님

마지막 terminal 전 accepted capture122.70/receipt122.76s의 raw relative x error는 +.324mm였고 FF latch는 .245094m/s로 실제 .25와 차이 −.004906m/s였다. capture 때 actual aircraft vx .413129와 PC .403261의 차이는 이미 −.009868m/s였다.

handoff124.08의 실제 relx+.002550/PC+.015450m가 127.00에는 실제 **−.081713/PC+.004275m**로 벌어진다. actual vx .286930이 PC .245466보다 빠르다. probe127.12, 첫 partial 두 발 pad127.30, 128.00 실제 relx−.102459m와 tilt10.60°에서 support residual2.598m/s²가 보였지만 tilt 조건이 탈락했다. 모든 candidate 조건의 최장 연속 시간은 **.08s<.25s**로 off하지 않은 것은 기록과 일치한다. expiry128.70은 last-image-age6s이며 handoff 후4.62s다. 그 전에 이미 pad miss가 생겼으므로 제한을 늘리지 않는다. [후속 CSV](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/followup-aligned-flight.csv), [candidate gate audit](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/seed43-contact-candidate-gates.json).

120–127s median pitch residual −.048957°와 gravity-comp Δax −.008382m/s²가 남는다. raw gyro xy bias=0은 synthetic S이며 실기 보장이 아니다. .05° 잔차만으로도 6s 동안 `.5*g*angle*t²=.1541m`, 시작 속도 .03m/s 오차만으로 .18m가 가능하다. nominal foot x/y±.031m, sphere radius.0028m, pad half .10m인 이 S 기하에서 4발 중심 여유는 대략 .066–.069m로 중심 .10m보다 작다. yaw/접촉 영역에 따라 달라진다.

![seed43 blind 상대오차와 작은 자세 잔차](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/seed43-blind-propagation.png)

그림의 속도/자세 축은 blind 구간의 작은 잔차를 확대했다. 접근과 접촉 뒤 큰 변화는 CSV에 보존되어 있다. FF를 실제 .25로 고정하는 온라인 oracle은 허용하지 않는다. qualified 영상 연속 구간의 상대 속도와 own floor velocity로 FF를 검증·불확실성에 따라 latch하는 대안은 paired 시험 후보이며 승인된 해결책은 아니다.

### seed53: 실제 지지는 있어도 패드 정체성은 틀릴 수 있음

old53은 182개 contact 표본 모두 vehicle_body였고 hold151개 모두 4발 vehicle_body였다. 3001 motor 표본은 zero였다. off290.32의 own z .269825m, 실제 z .273565m는 S pad nominal body height `.30+.0145=.3145m`보다 낮다. residual3.218, expected rotor acceleration6.390, measured specific acceleration9.818m/s², gyro.00197rad/s, nearxy/calibration 등 기존 조건은 통과했다. **IMU는 지지가 있다는 단서는 주지만 그 지지면이 pad인지, 네 발인지 확인하지 못한다.** [개별 거부와 wrong-surface audit](../../artifacts/mission3/independent-review/20261008-physical-060809-entry1-seed53-heisenberg/wrong-surface-contact-audit.json).

estimated pad height+.0145 ±.010~.015m gate는 이 과거 trace를 거부할 수 있는 S 후보다. 메인의 후속 ±.012m 정책을 과거에 소급 적용하지 않는다. height 추정이 +45mm 틀리면 같은 잘못된 차체 지지를 다시 허용할 수 있으므로 height 오류·지연·장착 U를 포함한 negatives가 필요하다. gate만으로 pad/4발을 보장할 수 없다.

seed61도 handoff192.74의 작은 y 오차가 196.66에는 actual rel y+.191999m / estimate−.000957m로 커졌다. PC roll residual 약−.05544°/Δay+.00944m/s²와 양립한다. estimated arena z .04482<.05로 recovery가 actual floor contact196.68보다20ms 먼저 일어났으며 off는 없었다. 이 또한 blind 전파 오차이며 budget 연장 대상이 아니다.

![wrong surface와 margin-only 복구 비교](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/wrong-surface-and-recovery.png)

## 6. effective horizontal residual observer와 3 training 접촉

새 frozen source [estimator.py:140](../../artifacts/mission3/physical/20261008-061833-162355-entry1-seed43/source-snapshot/scripts/mission3/estimator.py:140)는 heading-frame 평균 accel에서 추정 `b_h`를 빼고, **qualified static floor / !jump / floor-pure / Flow-valid / range-accurate에서만** pre-velocity innovation으로 `b_h -= Ki*dt*R_heading.T*(flow_world−pred_v)`를 갱신한다. 기존 covariance에만 있던 bias에 mean을 추가한 것이다. [uncertainty.py:39](../../artifacts/mission3/physical/20261008-061833-162355-entry1-seed43/source-snapshot/scripts/mission3/uncertainty.py:39)의 Joseph `K[6:8]=-Ki*dt*R_heading.T`는 mean과 같은 gain을 사용한다. Touchdown의 raw specific force에서는 이 수평 residual을 빼지 않는다.

Ki4/s², limit±.15m/s²는 S다. 10ms velocity gain .1339746에서 pre-update innovation의 선형화 고유값은 .868671/.996954, 느린 시정수 **3.278s**다(근사3.349s). residual sign도 맞는다. [오차 예산/선형화](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/effective-residual-observer-budget.json). 이 값은 gravity projection·IMU·Flow 모델 잔차의 묶음이며 chip 제조사 calibration이나 물리 accel bias로 식별됐다는 뜻이 아니다. static-floor 오분류, yaw/scale, clipping에서 nominal K 사용은 남은 한계다. ±.15 clip은 6s 위치오차를 작게 보장하지 않는다(.5*.15*36=2.7m).

[사전 training plan](../../artifacts/mission3/paired-rpyt/20261008-floor-residual-training-v1/plan.json)의 baseline43/후보43·61·1 source는 동결되어 있었다. baseline43 truth는 old43와 바이트 동일하게 실패했다. 후보 3건은 각각 source41/runtime10 SHA와 설정·trial metadata를 확인했고, 모든 bias update가 usable floor/valid Flow/accurate range 조건에서만 일어났다. 조건 밖 mean 변경 0, max|bias| .00710/.01198/.00793m/s²로 clip 미도달이었다. [3건 source/log 검증](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/floor-residual-training-review-summary.json).

| 재사용 training seed / run | 실제 contact/off s | 점수 / 수평오차 | 판정 |
|---|---|---|---|
| 43 `061833-162355` | 124.46 / 125.24 | 8 / 25.660mm | 접촉만 승인 |
| 61 `061833-152324` | 197.52 / 198.28 | 9 / 17.208mm | 접촉만 승인 |
| 1 `062140-173754` | 163.94 / 164.76 | 8 / 28.675mm | 접촉만 승인 |

사용한 seed43/61을 다시 holdout으로 세지 않는다. seed1 오차도 이전 PC-only27.530mm보다 작지 않아 전반적 우월성을 주장하지 않는다. 이번 source의 Touchdown은 **기존 rotor7.3/probe.65**이며, 후속 height/calibrationfresh45/near-g.2/rotor6.5/probe.6 guard는 이 세 실행에 없다. 새 supportguard v1의 np.bool JSON 경계 오류로 중단된 partial은 개발 실패 기록이고 미션 결과를 부여하지 않는다. serialization fix 이후 v2도 이 문서·서명의 검토 대상이 아니다.

## 7. 공식 selectable complementary / sensfusion6 경로

**제조사 알고리즘을 새로 수정하지 않고 onboard 자세를 complementary로 선택하는 경로는 있다.** 공식 설명은 runtime `stabilizer.estimator`를 제공하며 Flow deck이 Kalman을 요구함도 설명한다. [Bitcraze estimator/controller 설정](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/configure_estimator_controller/), [공식 state estimators](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/). 이 검토의 정확한 enum/override 판정은 프로젝트의 pinned 공식 commit `f6e0f0a3b526861328caf3296a7ecf00bb915d39`를 따른다.

| 항목 | pinned 제조사 source에서 확인한 조건 |
|---|---|
| enum | [estimator.h](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/interface/estimator/estimator.h): AutoSelect=0, Complementary=1; `CONFIG_ESTIMATOR_KALMAN_ENABLE`일 때 Kalman=2. 뒤의 선택지는 compile 조건 의존 |
| runtime selector | [stabilizer.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/stabilizer.c): `stabilizer.estimator` param, loop에서 변경 감지 후 switch |
| forced build override | [estimator.c:108](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/estimator/estimator.c:108): forced Kalman/UKF/Complementary/OOT 설정은 runtime 요청을 override. 따라서 write 성공만으로 실제 선택을 확정하면 안 됨 |
| Flow startup | [system.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/system.c)와 [flowdeck_v1v2.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/deck/drivers/src/flowdeck_v1v2.c): deck required Kalman이 초기 선택에 들어감 |
| 허용 build | [Kconfig](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/Kconfig): `CONFIG_ESTIMATOR_AUTO_SELECT=y`에서 runtime1 선택, 또는 `CONFIG_ESTIMATOR_COMPLEMENTARY=y`로1 강제. 제조사 알고리즘 patch 없이 기존 configuration 선택 가능 |
| 데이터 경로 | [estimator_complementary.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/estimator/estimator_complementary.c): Flow queue는 attitude에 사용하지 않으며 gyro/acc로250Hz sensfusion6 Q 갱신. ToF/baro altitude 갱신100Hz는 Q 뒤 별도 position 경로 |
| 기본 IMU 알고리즘 | [root Kconfig](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/Kconfig), [sensfusion6.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/sensfusion6.c): Mahony quaternion 기본, Madgwick 옵션; gyro deg→rad, normalized accel gravity feedback |

따라서 **arming 전 selector1과 실제 estimator type/name readback 확인**, PC 자체 position/velocity outer PID → RPYT angle/direct thrust → complementary onboard attitude/rate PID라는 software-only 대안을 시험할 수 있다. comp altitude에 ToF가 남으므로 direct-thrust/position-disabled 모드가 z PID를 사용하지 않는지 실제 controller 경계도 확인해야 한다. pinned Flow driver의 `motion.disable`은 Flow enqueue를 억제하는 기존 param이지만, pinned zranger/zranger2에서 이에 대응하는 ToF disable param은 확인하지 못했다. phase-specific ToF gate는 project wrapper 또는 별도 firmware software change 범위로 제시해야 한다.

이 경로는 ground-model correction을 attitude에서 분리하지만 **절대 yaw 관측은 없고**, translational acceleration이 gravity correction을 오염시킬 수 있다. startup level/gyro calibration, drift, PC telemetry 지연은 여전히 검증해야 한다. CF Euler pitch legacy 부호를 geometry 변환에서 한 번만 처리해야 한다. 현재 Kalman host .so로 complementary를 실행한 결과는 없으며, 실제 장치 parameter 변경·build·비행도 수행하지 않았다.

추가 host 이식 주의: pinned sensfusion6의 fast inverse-square-root는 float를 `long`으로 bit-cast한다. ARM32의 long4를 전제로 하지만 Linux LP64 long8에서 단순 host compile은 overread/undefined behavior 위험이 있다. 제조사 ARM 결과와 대조하는32bit ABI 또는 별도 hash를 갖는 host 호환 wrapper가 필요하며 vendor 원본을 몰래 고쳐 동일 source로 부르면 안 된다. [공식 source archive/hash index](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/evidence-index.json).

## 8. 다음 고정 비교에서 필요한 구체적인 판별

1. **Flow/ToF 자세 기여 분리:** 동일 기록 센서를 fullEKF의 both/Flow-only/ToF-only/neither에 입력하는 offline 비교로 update 전후 q, innovation, gain/cross-covariance를 기록한다. 100Hz 근사 입력 한계를 표시하고 이후 frozen closed-loop paired로 확인한다. phase gate를 늦게 걸어도 이미 흡수한 자세·속도 오류가 자동 복원되지 않는다.
2. **PC 경로 분리:** PC independent IMU 유지 + 원 standoff / 새 standoff를 유지하되, vision 자세와 acceleration 자세 intervention을 별도 시험하면 현재 bundled causal 효과를 분리할 수 있다. PID 정확 호출 인자·센서 평균 구간·packet receipt를 기록한다.
3. **blind 상대 오차 예산:** floor residual 관측 시간/정체성, 영상 relative velocity·FF latch 오차, heading/systematic floor를 합쳐 마지막 관측에서 terminal 예산을 제한한다. .05° 또는 .03m/s가 6s에 만든 오차는 pad 여유를 초과한다. 6s를 늘리거나 실제 차량 .25를 FF에 대입하지 않는다.
4. **잘못된 지지면/공중 후보 negatives:** estimated pad height guard는 wrong body/floor/height bias·delay/unknown CoM, gain 변화·lag 변화·calibration stale을 포함해 검증한다. 메인의 negative agent가 발견한 공중 false-off 보고도 기존 observer의 실기 보장 근거가 없음을 강화하며, 이 문서에서 새 negative 전체를 검증했다고 주장하지 않는다.
5. **복구 안전:** partial support 뒤 erroneous velocity로 pitch가 포화되는 margin 사례를 유지한다. 관측 불가 속도에서 command·integrator 제한과 recovery 전환을 paired test하되, 항공기 pose write나 groundtruth contact branch로 해결하지 않는다. 센서만으로 4발/pad 정체성이 확인되지 않는 범위는 offline success 검증에 남긴다.

## 9. 개별 접촉 서명과 검증 범위

7개 승인 실행에서 hold 3s의 **151개 truth 표본 전부 네 발 pad**, 각 양의 법선력, 3001개 모터 표본 PWM/output/staticforce0을 확인했다. motor response lag는 off 직후 실제 힘의 짧은 감쇠를 포함해 재계산했고 actual force가 즉시0이라고 주장하지 않았다. 각 run의 support 후보는 실제 접촉 뒤에 발생했고 100Hz rawIMU/held-PWM 및 S lag/model residual 재구성과 .25s 조건을 확인했다. HOLD sensor151표본은 모두 stable지만 accurateToF0/validFlow0으로, IMU stability가 surface identity를 증명하는 것은 아니다.

source audit에서 초기화 후 항공기 qpos/qvel write, controller로의 plant contact/force truth 입력은 발견되지 않았다. vehicle prescribed motion .25만 scene 조건대로 허용된다. finite-patch/texture, mount/CoM/mass/inertia/lag/K,D와 100Hz PC telemetry 가능성은 여전히 S/U다. official motor curve와 archived supply/battery input, signed evaluator hash를 보존했다. 각 독립 판단은 아래 실제 로그와 원 평가 file SHA/canonical digest에 결합되며 template autoapproval을 쓰지 않았다.

| 개별 검토 | 접촉/off/hold 종료 s | hold 각 발 법선력 N | off 이후 최대 slip µm | 서명/최종 검증 |
|---|---|---|---:|---|
| PC IMU training1 | 162.40/163.14/166.14 | 0.02560865–0.08583856 | 13.335 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-060138-pc-imu-entry1-seed1-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-060138-pc-imu-entry1-seed1-heisenberg/final-verification.json) |
| combined training1 | 162.86/163.60/166.60 | 0.02637284–0.08583840 | 17.706 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-060432-combined-entry1-seed1-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-060432-combined-entry1-seed1-heisenberg/final-verification.json) |
| frozen holdout47 | 103.68/104.42/107.42 | 0.02305388–0.08583862 | 22.935 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-060809-entry1-seed47-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-060809-entry1-seed47-heisenberg/final-verification.json) |
| frozen holdout59 | 141.12/141.86/144.86 | 0.02414296–0.08583856 | 31.057 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-061219-entry1-seed59-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-061219-entry1-seed59-heisenberg/final-verification.json) |
| floor residual training43 | 124.46/125.24/128.24 | 0.02563515–0.08583866 | 12.438 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed43-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed43-heisenberg/final-verification.json) |
| floor residual training61 | 197.52/198.28/201.28 | 0.02772520–0.08583871 | 8.361 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed61-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed61-heisenberg/final-verification.json) |
| floor residual training1 | 163.94/164.76/167.76 | 0.02610036–0.08583868 | 13.910 | [signedreview](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed1-heisenberg/signedreview.json) / [verification](../../artifacts/mission3/independent-review/20261008-physical-floor-residual-entry1-seed1-heisenberg/final-verification.json) |

old53의 [거부 서명](../../artifacts/mission3/independent-review/20261008-physical-060809-entry1-seed53-heisenberg/signedreview.json)과 [검증](../../artifacts/mission3/independent-review/20261008-physical-060809-entry1-seed53-heisenberg/final-verification.json)은 contact/qualified 승인 모두 false다. 실패 평가를 adapter 상태로 덮지 않는다.

전체 [final-verification.json](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/final-verification.json)은 원 입력 hash 재확인, 8개 개별 서명 validation, source/runtime/config/trial binding, 원 평가와 독립 재평가 전필드 일치를 기록한다. 역사적 서명은 수정하지 않았으며 이전에 명시적으로 저장된 signature digest가 있는 파일만 바이트 보존 비교 결과를 별도 표시한다. `attempt*-incomplete`와 `followup-incomplete-before-denominator-correction`는 완료 전 분석 시도이고 현재 판정 근거가 아니다.

## 10. 핵심 source SHA-256

표의 source는 각 실행의 snapshot이며 runtime .so도 같은 snapshot을 사용했다. 전체 manifest dictionary는 각 run manifest와 source-and-input-integrity에 남겨 두었다.

| 실행 / 파일 | SHA-256 |
|---|---|
| original054951 / [run_mission3_physical.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/run_mission3_physical.py) | `d3325301466c40e3e44a20d1faccaa07ec2c13357d9327b613a91b0caf30058e` |
| original054951 / [physical_estimator.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/physical_estimator.py) | `8845d0fb21a59f5b469dbd0b64f9a0ed58f211912a3c58d3ebeea7e65bc24aed` |
| original054951 / [estimator.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/estimator.py) | `2c9cb1413fd7c74daecb894a811030c8c2352573d00eca2513fa0f025fbcfaa3` |
| original054951 / [controller.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/controller.py) | `1f9b00ec75a41f93c989272bfabbd20ad4e814b366e9602c79dcfcd73b2f1a79` |
| original054951 / [physical_scene.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/physical_scene.py) | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| original054951 / [position_bridge_host.c](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/position_bridge_host.c) | `d67b6dc1fb0c7e8ca7bcc714db3dd8ce4df3b88eca0f628a861569bbd6b9e45f` |
| original054951 / [flight_controller_host.c](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/flight_controller_host.c) | `f6f370dd38b12a9ce9ec5a138edf02f00ff2f59f78b2796b59bcf696098bb737` |
| original054951 / [yaw_core.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/yaw_core.py) | `2971176642d5e5fdbdc3c2be6849323ed5242919eaf792457789f49d6330f09e` |
| original054951 / [evaluation.py](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/scripts/mission3/evaluation.py) | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |
| original054951 / [libmission3_estimator.so](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/build/mission3-firmware-host/libmission3_estimator.so) | `8ff946e9c319d60643486194b9aa351ba0c5d765382b9edd8665564340e41233` |
| original054951 / [libmission3_controller.so](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/build/mission3-flight-controller/libmission3_controller.so) | `e118b28ccd995bf9f4b59d3f31030c8b6b96afc70a7cb08248975390d287d5be` |
| original054951 / [libmission3_position_bridge.so](../../artifacts/mission3/physical/20261008-054951-258593-entry1-seed1/source-snapshot/build/mission3-position-bridge/libmission3_position_bridge.so) | `fbe8752bb9f8196c8bd6bbc84687e27d1db04892af247870817cc8b2be66f4d2` |
| paired060138 / [run_mission3_physical.py](../../artifacts/mission3/physical/20261008-060138-311753-entry1-seed1/source-snapshot/scripts/run_mission3_physical.py) | `958912b6aa21b989e5902121dabc640bf47ff0f2f58dcf38ff210bfe32ec50ab` |
| paired060138 / [physical_estimator.py](../../artifacts/mission3/physical/20261008-060138-311753-entry1-seed1/source-snapshot/scripts/mission3/physical_estimator.py) | `d8eab7c2e126307b3970f54f3db9cd0a1835c3fd8f5c37b6ff3747b900ee054c` |
| paired060138 / [firmware_host.c](../../artifacts/mission3/physical/20261008-060138-311753-entry1-seed1/source-snapshot/scripts/mission3/firmware_host.c) | `bab0f8154a809d7f9a8c0796424d08eae6cc70209398317ad57842fd7ee13990` |
| paired060138 / [firmware.py](../../artifacts/mission3/physical/20261008-060138-311753-entry1-seed1/source-snapshot/scripts/mission3/firmware.py) | `1a461c5ce547e1789fd804dfc29fdc1111c2f4be70b4a69673814c92b3ce58fc` |
| paired060138 / [libmission3_estimator.so](../../artifacts/mission3/physical/20261008-060138-311753-entry1-seed1/source-snapshot/build/mission3-firmware-host/libmission3_estimator.so) | `078b21e35f41a15dfe88ce4c2005beb41e7dc9197aeb743790071eb4e4769f50` |
| floor residual061833 / [estimator.py](../../artifacts/mission3/physical/20261008-061833-162355-entry1-seed43/source-snapshot/scripts/mission3/estimator.py) | `ce4ba6c32f605937c8671b0491a9bb1bc7857f585777ca7b3f03891edcf91dfc` |
| floor residual061833 / [uncertainty.py](../../artifacts/mission3/physical/20261008-061833-162355-entry1-seed43/source-snapshot/scripts/mission3/uncertainty.py) | `29084e83250bc32644fd8db047b4dfd1030d1255d350869832c23a77546a98d2` |
| floor residual061833 / [touchdown.py](../../artifacts/mission3/physical/20261008-061833-162355-entry1-seed43/source-snapshot/scripts/mission3/touchdown.py) | `22526f436231b94bb947a0d7d4caf24baf39e7ca56a64b7a2e98abd39c5920a4` |

공식 complementary 관련 source의 commit·URL·전체 SHA는 evidence index에 있으며 핵심 항목은 다음과 같다.

| pinned 공식 source | SHA-256 |
|---|---|
| [src/modules/src/estimator/estimator.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/estimator/estimator.c) | `655df82a38fafb743910801394a74eb2d378ff6ee38a0e21884eb59e15a237dc` |
| [src/modules/interface/estimator/estimator.h](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/interface/estimator/estimator.h) | `6f15502f6c643a639d48a89a30333170cced79bfbf428c853eb015576ce04570` |
| [src/modules/src/estimator/estimator_complementary.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/estimator/estimator_complementary.c) | `32a0036217c4101123d17eff99e508eca4bb91d14bcaa7b17d4ca9d142f80ab2` |
| [src/modules/src/sensfusion6.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/sensfusion6.c) | `f6ff3577a4ad929098519e1b54010e7b0bc55817841f4749bed50d7997bcdaa8` |
| [src/modules/src/Kconfig](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/modules/src/Kconfig) | `95beefb27e1ae52734e4e5e4cfa3e0012e11d0549d0a4a0902a6e95e99d82060` |
| [src/deck/drivers/src/flowdeck_v1v2.c](../../artifacts/mission3/rpyt-failure-review/20261008-054951-heisenberg/official-source/src/deck/drivers/src/flowdeck_v1v2.c) | `6e81daa5eeeeda576e291efabc92e949a41f7f5f9f1d45d2c77dd213997bcabe` |
