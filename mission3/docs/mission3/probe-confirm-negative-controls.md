# 최소 collective 확인과 두 수준 IMU 응답의 음성 대조

2026-10-08 KST. 새 [benchmark_mission3_probe_confirm.py](../../scripts/benchmark_mission3_probe_confirm.py) 안의 **비미션·sensor/command-only wrapper** 개발 기록이다. 공유 source/config/C/build/vendor를 수정하지 않았다. 동결된 `TouchdownObserver`를 artifact의 private runtime에 복사해 감쌌고, 접촉 후보에는 실제 높이·힘·gain·접촉·차량 상태를 전달하지 않았다. 접촉/미션/제조사 검증 성공은 0이다.

핵심 결과: minimum multiplier .60과 commanded prediction의 .25s 안정 확인은 gain1.5의 상위 plateau false off를 차단했지만, **minimum에서도 gain1.65·1.7의 공중 false off가 남았다.** 두 수준 IMU 응답은 이번 고정 gain 반례를 veto했으나, 두 수준 사이의 gain이 바뀌어 같은 IMU specific force를 만드는 S 반례에서는 dual 방식도 공중 off를 냈다. 어느 방식도 보장된 접촉 판정이나 hardware gain 상한을 제공하지 않는다.

## private wrapper와 입력

원본은 이전 실행의 [동결 touchdown.py](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/source/scripts/mission3/touchdown.py)이며 SHA256는 `94e339227e1c3c553ee18e7196bd36b34d77f9c2c5015a3fd9c91334cac77a8f`다. 이 파일의 byte hash를 확인한 뒤 새 출력의 `source/scripts/mission3/touchdown.py`로 복사한다. 물리 scene과 그 의존 source/config/library도 복사하고 **복사본에서만 import**한다. main의 실행 중 변경이 뒤 case에 섞이는 방식이 아니다. 각 복사본의 origin/hash는 `source-manifest.json`에 남는다.

실험 API는 `ProbeConfirm(observer, policy).candidate(now, sample, held_command, phase, roll, pitch, enabled) -> (confirmed, diagnostics)`다. `height` 입력은 없다. `roll/pitch`는 센서 gyro/accelerometer로 계산하며 실제 자세를 읽지 않는다. 별도 생산용 모듈이나 배포 준비 API가 아니다.

| 입력/처리 | 출처·주기 |
|---|---|
| commanded motor prediction | 이전에 실제로 보낸 대칭 PWM의 nominal software curve, 가정 τ20ms·35g; 1kHz |
| delivered IMU | 조건부 physical IMU 1kHz 원표본 10개 평균, 100Hz |
| roll/pitch | 위 gyro 적분 + accelerometer 방향 complementary 계산 |
| calibration | t<2.5s fixture 일정, sensor ToF valid/accurate·range>.65m, Flow valid, sensor tilt<10° |
| context enable | 알려진 비미션 probe 일정; Mission3 상대 정렬/고도/안전 조건을 실행하지 않음 |
| actual gain | 오직 plant 실제 force law에 사전 시간표로 적용 |
| truth/contact | 기록 후 오프라인 평가만 |

freeflight calibration의 sensor range 조건과 **접촉 후보 height gate**를 구분한다. 후보의 height gate와 calibration expiry는 이 private 설정에서 비활성이고, 2.499s경 유효하게 식별한 ratio를 유지한다. 각 실제 질량은 .035kg, 전압은 가정 4.2V다. 알려진 fixture 초기 위치 `(1,2,1)m` 또는 `(1,2,3.5)m`와 spinup 보상 초기 vz=`g×nominal τ`는 초기화에만 사용한다. 이후 aircraft qpos/qvel을 쓰지 않는다. 차량은 기존 scene에서 독립적으로 움직이며 controller/wrapper 입력이 아니다.

## 세 정책의 정확한 차이

공통 private observer 설정은 `supported_accel_tolerance_m_s2=.2`, `probe_motor_accel_max_m_s2=6.5`, `probe_multiplier_min=.60`, residual≥1.2, tilt<10°, gyro<.3rad/s, support confirm .25s다. base IMU 창은 .15s이며, 원 observer가 실제 유지한 window span도 기록한다.

- `baseline`: 공통 observer gate만 쓴다. upper plateau에서도 후보가 가능하다.
- `minimum_settled`: 현재 **held command**의 requested multiplier와 nominal curve multiplier 모두 `.60±.001`이어야 한다. nominal command×보정 ratio와 predicted motor acceleration 차이가 .03m/s² 이내인 표본을 연속 .25s 모으고, 창 안 prediction spread도 .03 이하로 제한한다. 100Hz stamp 중복/역행 또는 gap>.0201s는 창을 지운다. 이때부터 base의 별도 .25s support confirm을 시작한다. command가 minimum을 벗어나거나 안정 조건을 잃으면 support timer도 reset한다.
- `dual_response`: minimum/settle 조건에 더해 upper/minimum의 .25s 안정 구간 IMU 응답을 비교한다. 예측 가속도 차이≥.4m/s², 두 구간 IMU Z 평균 차이의 절댓값≤.2m/s², upper 마지막 stamp 이후≤2s일 때만 support-like 후보를 허가한다. ΔIMU/Δprediction을 diagnostic으로 남긴다.

`.001/.03/.4/.2/2s`는 모두 이번 S heuristic이다. **명령 예측**이 안정된 창이지 실제 rotor gain/응답이 안정됐음을 센서 독립적으로 확인한 창은 아니다. dual의 flat response도 접촉 보장이 아니다. gyro/tilt/residual 조건을 만족하지 않는 표본을 우회해 접촉을 만들지는 않는다.

최종 실행의 minimum-ready 첫 시각은 약4.279s, gain1.65·1.7/cancellation S의 off는4.529s다. minimum command와 예측 안정 확인 뒤 약 .25s의 support confirmation이 실제로 분리되어 실행됐다. 후보 순간 command PWM30518×4, requested multiplier=.60, nominal curve5.885986m/s²였다.

## 사전 시간표·시험 목록

대칭 명령은 PID/state feedback 없이 다음과 같이 고정했다.

| 시간(s) | nominal command |
|---|---|
| 0–2.5 | hover9.81; 허용 sensor packet으로 calibration |
| 2.5–2.9 | hover9.81; calibration latch |
| 2.9–3.15 | .25s ramp9.81→6.5 |
| 3.15–3.8 | upper6.5 유지 |
| 3.8–4.0 | .20s ramp6.5→5.886 |
| 4.0–5.15 | minimum5.886 유지; off 후보가 나면 실제 zero command |

실제 motor gain은 최종 실행에서 **3.15s**에 바꿨다. 이 시점은 명령 일정에서 정한 값으로 actual height에 따라 조절하지 않는다. 질량은 바꾸지 않았다.

28개 목록: initial height1/3.5m × gains1/1.5/1.65/1.7 × policies3=24개, 그리고 두 높이 × `minimum_settled/dual_response`에 time-varying gain S 4개다. seed1을 모든 짝에 고정했다. S 반례는 upper에서 `9.81/6.5=1.509231`, lower ramp 시작3.8s부터 `9.81/5.886=1.666667` gain을 실제 plant force에 적용한다. 두 plateau의 IMU를 약9.81로 만드는 의도적인 모델 변화 stress다. firmware, 전기계, 실제 ground effect의 크기나 발생 확률을 재현했다고 주장하지 않는다.

먼저 같은 28개를 gain 변경2.6s로 실행했다. nominal hover 중 gain이 커져 일부 case가5m 위로 상승했다. 원 반례를 보존하고, 요청한 두 초기 높이에 더 가까운 물리 상태를 확인하기 위해 변경 시점만 upper 시작3.15s로 옮긴 **진단용 후속 짝 시험**을 사전 등록했다. 이 수정은 첫 결과를 본 뒤 이뤄졌으므로 holdout으로 부르지 않는다. 이전 calibration-change 과제의77개는 다시 실행하지 않았다.

## 완료 결과와 별도 안전 분류

| 실행 | 계획/완료 | 공중 false off | 바닥 접촉 case | 접촉 없이 false off 없음 | sensor-only replay |
|---|---:|---:|---:|---:|---:|
| [065140 early](../../artifacts/mission3/probe-confirm-negative/20261008-065140-884228/summary.json) | 28/28 | 14 | 8 | 8 | 28/28 |
| [065452 upper/최종](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/summary.json) | 28/28 | 14 | 15 | 6 | 28/28 |

최종 정책별 결과:

| 정책 | cases | 공중 false off | 바닥 접촉 전체 | 그중 false off 이후 바닥 | 정상 음성 |
|---|---:|---:|---:|---:|---:|
| baseline | 8 | 6 | 6 | 4 | 0 |
| minimum_settled | 10 | 6 | 5 | 2 | 1 |
| dual_response | 10 | 2 | 4 | 1 | 5 |

바닥 접촉 전체15는 false off14와 서로 배타적인 수가 아니다. **7개는 공중 off 뒤의 바닥 접촉**이고, 나머지8개는 먼저 환경 접촉이 생겨 공중 음성 대조로 인정할 수 없었다. 후속 바닥 접촉을 이유로 앞선 공중 off 반례를 지우지 않는다.

critical 결과는 두 초기 높이에서 같은 후보 분류였다.

| plant law | baseline | minimum_settled | dual_response |
|---|---|---|---|
| fixed gain1.5 | upper에서 공중 off | 공중 off 차단; 1m군은 이후 바닥,3.5m군은 정상 음성 | 동일 |
| fixed gain1.65 | minimum에서 공중 off | **공중 off 유지** | 두 높이 모두 정상 음성 |
| fixed gain1.7 | minimum에서 공중 off | **공중 off 유지** | 두 높이 모두 정상 음성 |
| gain1.509→1.667 S | 이 변형의 baseline은 미시험 | 두 높이 공중 off | **두 높이 공중 off** |

nominal gain1의6개도 보존했다. 모두 probe 중 바닥에 닿았다. initial1m의 baseline/minimum 정책은 바닥 접촉 이후 off를 요청했고, 다른4개는 off 없이 끝났다. 이를 pad 착륙 성공이나 정상 공중 음성으로 취급하지 않는다. 특히 minimum/dual이 false off를 거부해도 initial1m/gain1.5는 바닥 접촉을 피하지 못했다. **후보 억제와 안전한 비행 종료는 별도 문제**다.

representative 후보의 실제 높이, 이후 바닥 시각, gyro/IMU/command와 원 contact 목록은 `evaluation.json`과 raw trace에 남는다. 높은 gain 때문에 생기는 상승, contact softness로 생긴 바닥 penetration, 바닥/몸체 접촉도 삭제하지 않았다. 안전 gate·climb/recovery를 새로 구현하지 않았으며 정해진5.15s horizon에서 simulation을 끝냈다. 실제 mission timeout이나 지원된3초 HOLD를 시험한 것으로 해석하면 안 된다.

## sensor-only 반례의 수치

[minimum gain1.65, initial1m](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/008-height1-gain1.65-minimum_settled/evaluation.json):

- t4.529s에 minimum/settle=true, prediction5.88556, IMU mean9.71374, residual3.82818m/s²다. 공중 실제 높이1.10535m, 이전/현재 contact 없음이다.
- upper/minimum IMU 차이1.01134, prediction 차이.61413, response slope1.64679다. dual이라면 이를 veto한다.
- zero가 실제 plant에 적용됐고, HOLD failure4.789s, 바닥 접촉5.045s다. 실제 contact는 off 뒤 .516s에 생겼으므로 지지 off 증거가 아니다.

[dual gain-change S, initial1m](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/014-height1-gain-level-cancellation-S-dual_response/evaluation.json):

- t4.529s에 prediction5.88556, IMU mean9.81184, residual3.92628m/s²다. 실제 높이.51571m, 이전/현재 contact 없음이다.
- upper IMU9.80916, minimum IMU9.81094, 차이−.001784, response slope−.002905로 flat heuristic을 통과했다.
- minimum/settle와 support confirm을 모두 통과해 zero가 실행됐으며, HOLD failure4.799s, 바닥 접촉4.838s다. flat response와 near-g가 실제 지지 없이도 생성되었다.

![minimum 및 dual 공중 off 이후 바닥 접촉](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/probe-confirm-counterexamples.png)

## 해석과 권고

minimum5.886에서 고정 gain1.65는 약9.7119,1.7은 약10.0062m/s²를 만든다. 둘 다9.81±.2 안에 들어갈 수 있다. multiplier가 실제로 하한에 도달하고 prediction을 기다려도 **보정 뒤 gain 변화와 지지력은 단일 IMU 값에서 구분되지 않는다.** 시간 확인은 상위 plateau 반례를 줄이지만 이 식별 문제를 해결하지 않는다.

두 수준 비교에서 gain이 같은 자유 비행이라면 `ΔIMU≈G×Δprediction`이므로 응답이 크게 보인다. rotor가 줄어도 접촉 지지가 총 specific force를 유지하면 작은 차이가 생길 수 있다. 그러나 이번 gain-change S는 `G_upper×a_upper≈G_lower×a_lower≈g`를 만들어 같은 센서 패턴을 낸다. **flat을 보장된 접촉 확인으로 쓰지 말고, 큰 응답을 접촉 후보 veto의 추가 근거로 다루는 편이 이 기록과 맞는다.** 안정된 actuator gain, 추가 힘, tilt/IMU 장착 오차를 보증하는 미확인 상한은 주장하지 않는다.

main이 후속 통합을 검토할 경우 minimum/settle는 명령 단계 확인으로 분리하고, target surface/추정고도·불확실성/최근 영상·경과시간 및 finite descent/recovery 조건은 별도로 유지해야 한다. 이 조건들이 gyro/near-g/dual flat만으로 실제 네 발 지지를 보증하는 것은 아니다. contact가 아니라는 veto가 났을 때 더 오래·더 낮게 probe하는 정책도 이 결과로 정당화되지 않는다. 이번 low-altitude 바닥 대조는 지연에 따른 안전 비용을 보여준다. 공유 touchdown 변경이나 runtime 배포는 이 sidecar에서 수행하지 않았다.

## 검사·source hash·재현

각 case는 `fixture.json`, `imu-raw.csv`, `observer.jsonl`, `commands.jsonl`, `truth-offline-only.jsonl`, `evaluation.json`을 보존했다. Truth는100Hz와1kHz contact signature 전환 시점에 기록하며 접촉이 생겨도 고정 horizon까지 계속한다. 후보 시 실제 contact 없음/그 전 contact 없음/body clearance>.05m를 **종료 후** 판정한다.

7개 self-check는 minimum 비허가, .25s 연속 창, stale gap reset, duplicate stamp reset, prediction spread 차단, settle .25가 support .25보다 먼저, minimum 이탈 시 confirm reset을 검사했다. 두 실행 모두7/7 pass다. 이 합성 검사는 physical 성공 증거가 아니다.

각 case 종료 뒤 별도 `audit_case()`는 raw IMU 평균, sensor tilt, calibration, frozen observer 수치, minimum/dual diagnostic, 후보, zero PWM와 HOLD 실패 시각을 재생했다. **audit는 truth/evaluation/matrix를 읽지 않으며** 각 실행28/28 일치다. 이는 계산 재현 pass이며 negative safety pass와 다르다. 두 batch 모두 정상 완료했고 반례/바닥 오염을 알리는 의도적 exit2다. harness 예외·미완료0이다.

| 항목 | SHA256 |
|---|---|
| 현재 script / 최종 실행 | `9cb97a1f25db011f532b6e0c234749d6c54b13537a334f202578eee147ae612b` |
| 첫 early 실행 script | `02504e7acb98498bcf42653e73bbba099d51f46fd892ff43e3e2eabc602030a4` |
| 복사한 frozen observer | `94e339227e1c3c553ee18e7196bd36b34d77f9c2c5015a3fd9c91334cac77a8f` |
| physical_scene.py | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| pure motor-curve library | `e118b28ccd995bf9f4b59d3f31030c8b6b96afc70a7cb08248975390d287d5be` |

Python3.10.12/MuJoCo3.13.0/NumPy2.2.6/team `.venv`다. [final source manifest](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/source-manifest.json)와 각 `source/`가 실행 코드/config/binary를 보존한다. 사용한 physical scene/config/library는 두 실행에서 동일하다. main의 controller/handoff 변경 snapshot이 포함되어 있어도 이 비미션 fixture는 해당 mission logic을 import/실행하지 않는다.

[all-cases.csv](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/all-cases.csv), [정책별 분석](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/analysis-summary.json), [오프라인 분석 source](../../artifacts/mission3/probe-confirm-negative/20261008-065452-203100/analyze_results.py)를 함께 제공한다. PNG/SVG 2D graph가 생성됐다. Matplotlib의 기존 Axes3D import 경고는 이 그래프의 실패가 아니다.

```bash
cd .
env PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
  src/crazyflie-ros2-sil/.venv/bin/python scripts/benchmark_mission3_probe_confirm.py \
  --gain-change-stage upper
```

`--gain-change-stage early`는2.6s 변경을 선택하며, 새 output directory를 사용한다. 기존 결과를 덮어쓰지 않는다. 실시간 source를 새로 복사한 물리 adapter/config가 바뀌면 새 실행의 의미도 달라지므로 과거 수치는 각 동결 source와 audit를 참조한다. deadline09:30KST 뒤 새 case를 시작하지 않는다. ROS transport, shared324 SITL, PC position/RPYT closed loop, hardware link/RTOS/supervisor, full warm handoff를 검증했다는 주장은 없다.
