# 보정 이후 모델 변화의 공중 접촉 반례

2026-10-08 KST. 35g 조건부 물리 모델 S의 **비미션 음성 fixture**다. 공중에서 접촉 후보가 확정되면 실제 가상 모터 명령을 0으로 내려 결과를 기록했다. 접촉 착륙·미션 성공·제조사 검증 건수는 모두 0이다. 공유 controller/estimator/touchdown/physical scene/config/C/build/vendor는 이 sidecar에서 수정하지 않았다.

기존 설정의 44개 시험에서 공중 false motor-off 14건, 바닥 접촉으로 대조가 오염된 경우 10건을 보존했다. 요청한 `near-g ±.2 / rotor prediction ≤6.5 / multiplier minimum .60` 로컬 overlay는 gain 1.2·1.3 반례를 차단했지만, gain 1.5와 허용 6.5m/s² plateau에서 다시 공중 motor-off를 만들었다. 보정이 최근이라는 조건만으로 힘 모델의 급격한 변화가 배제되지는 않는다.

## 구현과 입력 경계

구현은 [benchmark_mission3_calibration_change.py](../../scripts/benchmark_mission3_calibration_change.py)다. `rg`로 확인한 기존 파일은 `scripts/benchmark_mission3_contact_observer.py`이며 `_negative.py`라는 파일은 없다. 기존 시험·관측기·물리 scene은 읽기 전용으로 사용했다.

| 구간 | 관측기/명령이 사용하는 입력 | 사용하는 곳 |
|---|---|---|
| 초기 조건 | 알려진 fixture 위치 `(1,2,3.5)m`, ground-effect군은 1.5m; 초기 vz=`g×nominal motor τ` | freejoint 초기화 한 번만, spinup impulse 보상 S |
| 모터 예측 | 대칭 4모터 PWM 명령→기존 공식 software force curve, 가정 4.2V·τ20ms·35g | observer predict, 1kHz |
| IMU | 조건부 물리 센서의 body gyro/specific force, 1kHz 원표본을 10개 평균 | observer IMU, 100Hz |
| 자세 | 위 gyro 적분과 accelerometer 방향의 complementary roll/pitch | tilt/gyro gate; 실제 RPY 사용 없음 |
| 보정 허가 | t<2.5s라는 fixture 일정, 센서 ToF valid/accurate 및 range>.65m, Flow valid, 센서 tilt<10° | nominal 보정 구간만; 센서 packet을 별도 보존 |
| 접촉 후보 허가 | 사전 지정 probe 구간 | 비미션 enable; Mission3의 상대거리·추정고도·안전 조건을 대신 검증하지 않음 |
| plant 실제 gain | 일정 이후 step gain 또는 아래 ground-effect S law | **plant 힘 생성만**; observer/명령에 전달하지 않음 |
| 실제 위치/힘/접촉/차량 상태 | 기록 후 오프라인 평가 | 후보 판정·명령·종료 조건에 전달하지 않음 |

명령은 PID가 없는 대칭 nominal hover→collective probe다. 높이·실제 속도 피드백을 쓰지 않는다. 알려진 초기 높이와 명령 일정은 user가 허용한 fixture 조건이며, 차량을 찾고 착륙하는 미션 제어가 아니다. 공중 후보 시 높이가 수 m일 수 있으므로 실제 미션의 고도/상대 위치 gate까지 통과했다는 뜻이 아니다.

물리는 기존 MuJoCo freebody/motor lag/contact를 사용한다. 질량은 모든 경우 정확히 .035kg이며, gain 변화는 command PWM을 조작하는 대신 **실제 motor force law**를 바꾼다. aircraft qpos/qvel은 초기화 뒤 쓰지 않는다. 기존 library의 pure force/arm/torque 조회만 사용하고 PID init, build, radio, ROS/hardware command는 실행하지 않았다. 광학 acquisition, PC position PID, RPYT decoder, RTOS, supervisor, link timeout은 이 fixture의 검증 범위에 없다.

## 사전 지정 시험

0–2.5s nominal hover에서 force-model→IMU 비율을 보정하고 이후 보정 허가를 꺼 고정한다. 기존 observer의 1s minimum span/2s 창/RMS<.3 조건을 유지한다. 100Hz 보정의 마지막 표본은 약 2.499s다. 아래 age는 latch 끝부터 **gain 변경 시점까지**이며, 확정 시의 실제 보정 경과시간은 `calibration_age_s`에 별도로 기록한다.

Motor step군은 age .1/3/30s, gain .8/1/1.1/1.2/1.3이다. 변경 뒤 .3s nominal command를 유지한 다음 .25s 동안 9.81→7.0m/s²로 줄이고 1.0s 유지한다. 기존 profile의 multiplier minimum .65보다 높은 `7/9.81≈.714` operating point를 사용하는 독립 probe다. profile에 있는 미션 ramp 1s를 실행했다고 해석하면 안 된다.

Ground-effect군은 age .1s, peak/length `(1,1), (1.3,.5), (1.3,1), (1.5,.5), (1.5,1), (1.8,.5), (1.8,1)`이다. probe와 동시에 활성화한다. 가정한 추가 실제 gain은 다음과 같다.

`G(h,t) = 1 + activation(t) × (peak−1) × exp(−max(h,0)/length)`

`activation`은 .05s ramp다. h는 plant 내부의 floor 기준 실제 body 높이이며 힘 생성에만 사용한다. 계수와 law는 의도적인 **가상 ground-effect stress S**로, 실제 Crazyflie 지면 효과의 크기나 형상을 측정·검증한 값이 아니다. gain step과 ground effect를 같은 원인으로 해석하지 않는다.

22개 조건×사전 지정 seed1/2=44개를 모두 실행했다. seed2는 조건 재현이며 독립 mission holdout 성공률이 아니다. seed와 명령을 결과에 따라 고르지 않았다. 앞선 5개 smoke와 본 시험의 seed1 일부는 중복하므로 별도 실행군으로 보존한다.

## 기존 설정 결과

| 실행 | 계획/완료 | 공중 false motor-off | 바닥 접촉으로 오염 | 접촉 없이 false stop 없음 |
|---|---:|---:|---:|---:|
| [061949 smoke](../../artifacts/mission3/calibration-change-negative/20261008-061949-666083/summary.json) | 5/5 | 2 | 1 | 2 |
| [062316 full](../../artifacts/mission3/calibration-change-negative/20261008-062316-382148/summary.json) | 44/44 | 14 | 10 | 20 |

Motor step 결과는 두 seed와 세 age에서 동일한 분류였다.

| 변경 gain | .1s age, 2 seeds | 3s age, 2 seeds | 30s age, 2 seeds |
|---:|---|---|---|
| .8 | 바닥 접촉 2 | 바닥 접촉 2 | 바닥 접촉 2 |
| 1.0 | 정상 음성 2 | 정상 음성 2 | 정상 음성 2 |
| 1.1 | 정상 음성 2 | 정상 음성 2 | 정상 음성 2 |
| 1.2 | 공중 false off 2 | 공중 false off 2 | 공중 false off 2 |
| 1.3 | 공중 false off 2 | 공중 false off 2 | 공중 false off 2 |

Ground-effect peak/length별 결과는 `(1,1)`과 `(1.3,.5)`에서 각 2건 바닥 접촉, `(1.8,1)`에서 2건 공중 false off, 나머지 4조건×2seed에서 8건 정상 음성이었다. 바닥에 닿은 10건을 정상 음성이나 착륙 성공으로 바꾸지 않았다.

대표 원자료:

| 조건/시각 | command 예측 | IMU 창 평균 Z | residual | 마지막 보정 이후 | 오프라인 높이/접촉 |
|---|---:|---:|---:|---:|---|
| [gain1.2, seed1, t3.409s](../../artifacts/mission3/calibration-change-negative/20261008-062316-382148/004-motor_step-seed1-gain1.2-age0.1/evaluation.json) | 6.99955 | 8.39967 | 1.39992 | .910s | 3.90257m, 접촉 없음 |
| gain1.2, age30s, seed1, t33.309s | 약 7.0 | 약 8.4 | 약 1.4 | 약 30.81s | 접촉 없음 |
| [ground peak1.8/length1, seed1, t3.539s](../../artifacts/mission3/calibration-change-negative/20261008-062316-382148/022-ground_effect_S-seed1-peak1.8-length1/evaluation.json) | 6.99955 | 8.57488 | 1.57533 | 1.040s | 1.18163m, 접촉 없음; 실제 gain1.24518 |

가속도 단위는 m/s²다. gain1.2 반례의 보정 RMS는 .00628m/s²로 작고 `calibration_valid`는 시험 끝에도 true다. nominal 식별을 잘 했다는 사실은 **이후** gain 변경을 보증하지 않는다. zero PWM이 실제 적용된 뒤 HOLD 실패는 t3.679s, 즉 off 이후 약 .27s에 검출되었다. 관측 후 실패 표시는 이미 발생한 공중 모터 정지를 예방하지 못한다.

## 요청한 로컬 overlay의 짝 비교

[062707 paired](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/summary.json)는 동일 seed1·age.1s·7개 gains `.8/1/1.1/1.2/1.3/1.5/1.8`를 두 설정과 두 operating point에 짝지어 28개 실행했다. **공유 config는 변경하지 않았다.**

| 설정 | near-g tolerance | rotor prediction gate | multiplier minimum | gate plateau | minimum plateau |
|---|---:|---:|---:|---:|---:|
| original | 1.5 | 7.3 | .65 | 7.0m/s² | 9.81×.65=6.3765m/s² |
| tight | .2 | 6.5 | .60 | 6.5m/s² | 9.81×.60=5.886m/s² |

`gate`는 해당 설정에서 허용한 plateau다. `minimum`은 multiplier 하한까지 내려간 별도 plateau다. `.60` 하한을 설정했다는 이유만으로 모든 후보 순간의 command가 5.886이라는 가정을 하지 않았다. 두 방식 모두 .25s ramp/1.0s hold를 유지한다. 이 짝 비교는 tolerance/rotor gate/command를 함께 바꾸므로 단일 파라미터의 인과 효과를 분리한 시험은 아니다.

표의 `음성`은 접촉 없이 false off 없음, `바닥`은 fixture 오염, `false`는 접촉 전 공중 motor-off다.

| 실제 gain | original gate7.0 | tight gate6.5 | original minimum6.3765 | tight minimum5.886 |
|---:|---|---|---|---|
| .8 | 바닥 | 바닥 | 바닥 | 바닥 |
| 1.0 | 음성 | 음성 | 음성 | 음성 |
| 1.1 | 음성 | 음성 | 음성 | 음성 |
| 1.2 | false | 음성 | 음성 | 음성 |
| 1.3 | false | 음성 | 음성 | 음성 |
| 1.5 | false | **false** | false | 음성 |
| 1.8 | 음성 | 음성 | 음성 | 음성 |

합계 original 14개는 false4/바닥2/정상음성8, tight 14개는 false1/바닥2/정상음성11이다. tight minimum군 7개에서 false0/바닥1/정상음성6이었다. gain1.8의 false0은 gain이 더 커도 항상 안전하다는 증거가 아니다. near-g **밴드를 초과해서** 후보가 차단될 수도 있다.

[tight gain1.5 gate 반례](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/012-paired-seed1-gain1.5-gate-tight/evaluation.json): t3.559s, 예측6.49968, 측정9.75307, residual3.25339, 보정 age1.060s, 높이5.02388m/그 전 접촉 없음. zero가 적용됐고 HOLD 실패는 .28s 뒤다. 이 operating point는 `.60` floor보다 높은 약 .663 multiplier이며, 최저 collective에 도달한 시험으로 표기하지 않는다.

![공중 false motor-off 전후의 명령·IMU·오프라인 높이](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/false-stop-evidence.png)

## 왜 보정 age와 threshold만으로 부족한가

수평·steady·lag 정합의 단순한 경우, 보정 후 gain이 k배 되면 `a_IMU≈k×a_pred`, `residual≈(k−1)×a_pred`다. original plateau7.0에서는 near-g 조건이 약 `1.187<k<1.616`, residual gate가 `k≥1.171`이라 gain1.2·1.3이 두 조건을 함께 만족한다.

새 gate6.5에서 `.2` band의 허용 gain은 약 `9.61/6.5<k<10.01/6.5`, 즉 **1.478<k<1.540**다. gain1.5가 그 안에 들어온다. gain≤1.3이 실제로 보증된다면 `6.5×1.3=8.45<9.61`이라는 nominal 여유가 있지만, 이번 profile에는 그러한 제조사·실기 보증이 없다. minimum5.886에도 약 **1.633<k<1.701**의 이론적 혼동 구간이 남는다. 이 구간의 gain은 이번 7개 목록에서 직접 시험하지 않았으므로 물리 재현 주장과 구분한다.

권고는 main이 판단할 후속 사항이며 이 sidecar는 구현하지 않았다.

1. 마지막 **허용되면서 실제 fit도 유효했던** sensor stamp, RMS, excitation/force 구간, 모델 identity를 함께 유지하고 age 만료를 후보 허가에 적용한다. 후속 허용 구간에서 fit이 invalid이면 기존 valid latch를 그대로 믿지 않는 정책을 검토한다. terminal에서 가짜 접촉을 freeflight gain으로 다시 맞추는 무제한 식별은 피한다.
2. free-air로 확인할 수 있는 구간에서 command→IMU innovation의 지속적 모델 변화가 보이면 calibration을 취소하거나 contact permit을 억제한다. 최근 보정만으로 .9–1.1s 내 변화가 해결되지는 않는다. ground effect처럼 최종 구간에서 생기는 추가 힘도 별도 불확실성으로 다뤄야 한다.
3. nominal 수치가 아니라 gain/lag/IMU 장착/추가 힘의 상한 U를 포함한 공중 측정 상한을 쓴다. 예를 들어 `G_upper×a_pred_max + U < 9.81−tolerance`를 보증할 근거가 없으면 해당 probe를 독립 접촉 증거로 확정하지 않는다. 이번 stress gain 범위가 hardware 상한을 제공하지 않는다.
4. `.60` minimum 설정과 후보 순간의 실제 예측 collective를 구분한다. 하한까지 내려간 확인을 요구하는 후속 정책은 추력 손실과 남은 고도/시간을 함께 제한해야 한다. HOLD의 사후 실패 검출을 공중 off의 안전 대책으로 세지 않는다.
5. main이 검토 중인 **추정** body-height band는 wrong-surface motor-off 억제에 별도 도움이 될 수 있다. 하지만 추정 높이·surface identity의 오차도 평가해야 한다. 이 fixture는 그 band를 활성화하지 않았고, 실제 pad 접촉이나 네 발을 IMU만으로 식별했다는 증거가 아니다.

## 독립 재생과 보존

각 case는 `fixture.json`, `imu-raw.csv`, `observer.jsonl`, `commands.jsonl`, `truth-offline-only.jsonl`, `evaluation.json`을 보존한다. Truth는 100Hz에 더해 1kHz contact signature 전환 순간을 기록해 짧은 바닥 접촉도 제외하지 않는다. contact가 생겨도 정해진 종료 시각까지 계속 기록한다. 오프라인 false 판정은 후보 시 contact 없음, 그 전 환경 contact 없음, conservative body clearance>.05m를 함께 확인한다. 환경 접촉이 있는 fixture를 정상 음성으로 승격하지 않는다.

[truth-free audit source](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/audit_sensor_replay.py)는 각 실행의 **동결 touchdown.py**에 원 IMU 평균과 명령만 재생한다. gyro/accelerometer 센서 자세, 보정 허가·age, observer 수치, 후보 확정, zero PWM, HOLD 실패를 비교했다. truth/evaluation/matrix 파일은 읽지 않는다. 결과는 smoke5/5, full44/44, paired28/28 일치다. 이 재생 pass는 음성 대조의 안전 pass와 다르다.

[all-cases.csv](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/all-cases.csv), [evidence index](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/evidence-index.json), [offline plot source](../../artifacts/mission3/calibration-change-negative/20261008-062707-833982/analyze_results.py)는 실패를 포함한다. 두 분석 source는 재생/오프라인 분석 구분을 유지한다. 세 물리 실행은 모두 정상적으로 계획을 끝냈고, 반례/바닥 오염을 알리는 의도적 exit2였다. harness 예외·미완료 case는 0이다. Matplotlib의 Axes3D import 경고는 기록되었으며 2D PNG/SVG 생성에는 실패가 없었다.

## source hash와 재현

각 실행 전 실제 source/config/library를 `source/`에 복사하고 SHA256를 `source-manifest.json`에 기록했다. case 사이와 실행 직후 동결 대상 변경은 모두 0이었다. Python3.10.12, MuJoCo3.13.0, NumPy2.2.6, team `.venv`를 사용했다.

| 실행 | 실행한 benchmark SHA256 | 실행한 touchdown SHA256 |
|---|---|---|
| smoke061949 | `0a5577112467dbc5110b39fa2beb68cdcb0a3c336f86530330ee94e12ce2bd5d` | `22526f436231b94bb947a0d7d4caf24baf39e7ca56a64b7a2e98abd39c5920a4` |
| full062316 | `936bb5cc5db3716e5b70a669616115d398985b54e2b8bb11b7636f131bb185dd` | 위와 동일한 `22526f…` |
| paired062707/현재 benchmark | `ea200896a7cfd45d4c410dfe69d7a61bbf0d032fad0110c2c59272f5ebf04d7a` | `94e339227e1c3c553ee18e7196bd36b34d77f9c2c5015a3fd9c91334cac77a8f` |

paired에는 main이 그 사이 추가한 선택적 calibration freshness/estimated-height API가 포함되어 있다. **같은 paired source에서 original/tight를 비교**했으며 full과 source가 같다고 주장하지 않는다. 이 fixture의 base profile에는 `calibration_max_age_s`나 required support-height band가 없어서 선택적 gate는 비활성이다. 공유 물리 scene SHA256는 두 실행에서 `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59`, library는 `e118b28ccd995bf9f4b59d3f31030c8b6b96afc70a7cb08248975390d287d5be`로 동일하다. 각 전체 manifest가 snapshot과 실제 실행을 연결한다.

```bash
cd .
env PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
  src/crazyflie-ros2-sil/.venv/bin/python scripts/benchmark_mission3_calibration_change.py \
  --preset full --seeds 1 2
env PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl \
  src/crazyflie-ros2-sil/.venv/bin/python scripts/benchmark_mission3_calibration_change.py \
  --preset smoke --overlay paired-tight --seeds 1
```

현재 공유 observer/config가 나중에 바뀌면 새 실행의 의미도 바뀐다. 과거 숫자의 정확한 재생은 해당 `source/`와 `sensor-replay-audit.json`을 사용한다. 출력 폴더가 비어 있지 않으면 덮어쓰지 않는다. session deadline 뒤 새 case를 시작하지 않는다. 이 기록은 기존 constant-gain 보정 음성 대조를 대체하거나 full RPYT/mission 착륙 성공을 추가하지 않는다.
