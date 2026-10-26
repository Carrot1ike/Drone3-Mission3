# v11 training4 독립 접촉 검토 — 2026-10-09

Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 완료한 네 training run만 검토했다. **433·467·479 조건부 모델 접촉 승인, 457 거절: 3/4. 제조사 qualified·실기·formal 승인 0.** 재사용 training 표본이며 일반 성공률이나 unused holdout 통과 수로 해석하지 않는다. v9 서명을 복사하지 않았고 다른 split/새 실험은 별도 검토한다.

| seed | 첫 실제 접촉(s) | PWM off → 3초 끝(s) | 내부 첫접촉 CoM 오차 / 점수 | 독립 판정 | 원자료·서명 |
|---|---:|---:|---:|---|---|
| 433 | 191.40 | 192.26 → 195.26 | 15.253mm / 9 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-v11-entry1-seed433/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed433/signedreview.json) |
| 457 | 190.44 | 없음 | 13.639mm / 9 | 실패·접촉 승인 거절 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-v11-entry1-seed457/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed457/signedreview.json) |
| 467 | 156.88 | 157.76 → 160.76 | 21.122mm / 8 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-v11-entry1-seed467/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed467/signedreview.json) |
| 479 | 213.48 | 214.42 → 217.42 | 7.472mm / 10 | 조건부 접촉 승인 | [원 평가](../../../artifacts/mission3/physical/20261009-entry-forward-gaze-v11-entry1-seed479/offline-evaluation.json) · [본인 서명](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed479/signedreview.json) |

네 run의 전체필드를 보존 evaluator로 다시 계산해 원 평가와 같음을 확인했다. 모터 off가 없는457의13.639mm/9점은 첫 접촉 시점의 진단 수치이며 착륙 성공 점수가 아니다. [전체4 분모·동결 신원](../../../artifacts/mission3/reviews/20261009-v11-training-independent/snapshot-diff-and-denominator.json), [개별 서명 색인](../../../artifacts/mission3/reviews/20261009-v11-training-independent/signed-review-index.json).

## 이번 소스 변경과 독립성

네 실행 당시 snapshot의59개 source/test/C 파일과10개 runtime/buildmanifest bytes, frozen 설정·trialmetadata·실행 manifest를 직접 대조했다. v9 대비 변경은 controller의 opt-in `search_translation_gaze_policy='entry_forward'`와 새4계약검사 파일뿐이다. 설정에는 동작 옵션 외에 `scope`, `research_provenance` 설명 메타데이터가 달라졌다. 물리 profile·strict_focus·runtime·평가기준은 동일하다. [실제 snapshot diff](../../../artifacts/mission3/reviews/20261009-v11-training-independent/v9-to-v11-controller-snapshot.diff).

| 실행 근거 | SHA256 |
|---|---|
| plan | `c17406e7dcf1a86a177b3d8432dcc932e96c7132350a7ab547afa28923bd6467` |
| controller v11 | `a20b6d2cecbe5d818fbaca7cb690aece80e97d09d365a9e9d69d4c20c8e06180` |
| 새 gaze 계약검사 | `e4c7d16191c1a177ccc0053d3fe2c466d57e40206681a6f595401511f7a2e21a` |
| frozen 설정 | `58632e6b7291f27d5dd9d61bddcff199fe7567167478817330c126ea00e5b603` |
| evaluator | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` |

`ENTRY_SEARCH`에서 known entry heading과 추정 yaw 차이를 감쇠하며 기존 output yaw/속도/가속도 slew를 거친다. known heading은 진입 조건이며 차량 위치·위상·방향 truth 입력이 아니다. WAIT15/ENTRY_SCAN45/높이1과 pose18px/불확실성1m 기준을 바꾸지 않았다. 433의 ENTRY_SEARCH 이전 명령은 v9/433과 동일했다. 세 성공은 archived controller의 sensor/estimate-only 단계 재생과 접촉 observer의 rawIMU+100Hz heldPWM 전과정 재생에서 일치했다. 숨은 native PID/EKF 내부 상태 전체를 재생한 것은 아니다.

**시선 source 한 옵션 변경을 순수 광학 효과로 단정할 수 없다.** 기존 reverse-speed schedule이 추정 body heading에 투영한 속도를 사용하므로 yaw를 바꾸면 이동 명령, 후속 영상, 추정 상태도 함께 달라진다. 이번 결과는 닫힌루프 정책 비교다. 새4검사 내용을 읽고 plan에 묶인198tests 로그 해시를 확인했으며 전체198검사를 다시 실행하지 않았다. [plan62·test 근거 확인](../../../artifacts/mission3/reviews/20261009-v11-training-independent/plan-core62-and-test-report-crosscheck.json).

## 접촉·명령·센서 원자료

433/467/479의 실제 네발 접촉은 각각195/196/199개의50Hz표본이다. PWMoff부터 정확히3초 동안 각151표본 모두 네 발이 pad 위에서 양의 법선력을 보였다. 각3,001개의1kHz모터 표본은 PWM/출력/명령 static force가0이었다. 비발·차체·그물·바닥·marker board 충돌은 없었고 접촉·HOLD 전체가 실제 직선이었다. 차량은 전체로그에서 계속 .25m/s였으며 HOLD 동안 .75m 이동했다. 기체/차량 상태를 강제로 동일하게 쓰거나 접촉으로 기체를 패드에 붙인 흔적은 없다. source의 qpos/qvel 직접쓰기는 기체 초기화와 prescribed vehicle 갱신으로 구분된다.

| seed | HOLD 발당 최소 법선력(N) | HOLD 최대 3D 상대속력(m/s) | off 실제 추력(N) | off+.3s 실제 추력(N) | 최대 실제 속력(m/s) |
|---|---:|---:|---:|---:|---:|
| 433 | 0.033331 | 0.00007316 | 0.202353 | 6.190e-08 | 0.405938 |
| 467 | 0.032244 | 0.00007313 | 0.202026 | 6.180e-08 | 0.406046 |
| 479 | 0.032028 | 0.00007271 | 0.201339 | 6.159e-08 | 0.408225 |

PWM0는 즉시 기계적 rotor 정지 또는 실제 추력0과 같지 않다. 잔류추력 감쇠는 S20ms lag와 공식 software force curve로 정확히 재구성됐다. RPM/실기 motor response는 미측정이다. 정상 명령은3D .4m/s/.15m/s²·yaw10deg/s/20deg/s²·상승.10/하강.06을 통과했다. 실제 속력은 .4를 초과했고 off/회복/ABORT의 긴급 명령 변화는 별도 예외로 기록했다. 실제 운동 한도·안전 검증으로 바꾸어 말하면 안 된다.

100Hz Flow와40Hz ToF 원sample/stamp/newflag를 직접 대조해 held range 반복융합 방지 계약을 확인했다. 세 HOLD는100Hz301표본 모두 정확ToF0/유효Flow0/usableFlow0·surface ambiguous였다. rawIMU 지지 observer는50Hz150개 후속 표본을 안정으로 판단했으나 이 센서 판정만으로 pad 정체성이나 네발을 구별할 수 없다. 원 물리로그151표본의 실제 접촉과 별개다. off의 추정높이·영상age·freeflight calibration age는 각각433=11.630mm/5.14s/20.08s,467=13.925mm/5.42s/21.14s,479=20.774mm/5.38s/20.16s다. 마지막 유효 영상은 마지막 전달 영상과 다르며 HOLD의 더 오래된 영상age8.14/8.42/8.38s를 비행 prediction budget 연장으로 세지 않는다.

세 run의 main `development-review-unsigned.json` 공통 필드를 실제 재계산한 값과 대조해 불일치0이었다. 그 파일의 판정을 본인 서명으로 옮기지 않았다. 전체freeflight 보정gain/freshness/잔차는 rawIMU+PWM만으로 재생해 scalar오차0이었다. 12초 preparation 설정은 경로계획 예산이며 실제 준비시간13.88/14.62/13.76s는12초를 넘었다. 이번 세 착륙은 실제 직선에서 끝났지만 그 설정을 실행시간 보장으로 표현하지 말아야 한다.

## 457: 접촉은 있었으나 실패

190.44~192.94s 실제 pad접촉124표본, 그중 네발 양의 지지91표본이 있었다. 비발/다른 표면 접촉은 없었다. **PWMoff 명령·stop packet·접촉 이후 전모터PWM0 표본은 모두0개다. 따라서 모터off+3초 유지 계약을 승인하지 않는다.** 전체센서 gate를 archived observer로 재생하면 동시충족0표본/확정candidate0이며 원 평가의 실패를 재현한다.

| 시점(s) | 추정 pad위 body높이(mm) | 실제높이(mm, 오프라인) | 예측 motor가속도(m/s²) | 막힌 조건 |
|---|---:|---:|---:|---|
| 190.44 | 22.070 | 14.080 | 9.588 | 낮은 rotor force/지지잔차 |
| 191.00 | 25.502 | 13.566 | 7.208 | 낮은 rotor force |
| 191.16 | 26.592 | 13.566 | 6.583 | 높이·낮은 rotor force |
| 191.20 | 26.870 | 13.565 | 6.427 | 높이 |
| 192.00 | 32.227 | 13.565 | 5.697 | 높이 |

높이 guard는 S14.5±12mm의 strict 범위다. 추정값이191.16s에26.5mm 상한을 벗어난 다음191.20s에야 예측추력이6.5m/s² 이내가 됐다. 유효 높이와 저추력 조건이 겹치지 않았다. 물리높이는 거의 일정한데 IMU 전파 높이가 상승한 사실은 확인되지만 마지막 정확ToF·수직속도·IMU 잡음/바이어스·필터의 기여를 하나의 원인으로 분해하지 않았다.

마지막 유효 capture186.22s→192.22s imageage6초 만료가 `BLIND_RECOVER`를 유발했다. terminal 인계187.26s로부터는4.96s였다. 194.22s RISE→197.24s ABORT, 원 `inertial_texture_bridge_prediction_expired`를 유지한다. last delivered capture197.22s는 유효 metric pose 갱신을 의미하지 않는다. 전체1kHz harness 최대tilt1.858deg라는 이 한 결과가 다른 회복의 안전성을 증명하지 않는다. **높이 gate 확대나6초 예산 연장으로 이 접촉을 억지로 받지 말고, 마지막 qualified height/vz와 이후 수직 IMU innovation을 먼저 sensor-only paired 시험으로 검증할 것.**

[457 독립 gate 진단](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed457/failure-diagnosis.json) · [원표본 CSV](../../../artifacts/mission3/reviews/20261009-v11-training-independent/seed457/failed-support-gates.csv)

## 영상·남은 실기 근거의 한계

네 run 원 FFV1 3511/3558/2806/3943, 합13,818프레임을 전부 decode하여 mono320×320 uint8/controller-input SHA와 byte 동일함을 확인했고 본인 native sample 이미지를 열어 보았다. resize/color conversion으로 입력 계약을 맞추지 않았다. AVI20FPS는 보관 프레임 순서이며 focus 미제공으로 빠진 구간과 실제 .04/.06s capture 간격은 metadata를 사용해야 한다. 설정 전송지연 S.05s는50Hz 전달 경계에서 실제로그 .06s였다. FFV1 exact는 AI-deck 스트리밍/PSF/노출/하드웨어 timing 검증이 아니다.479의 별도1×H264 발췌는184~217.4s/335frames/33.5s의 표시 영상이며 전체비행217.44s/native 인식입력과 구분했다.

변하지 않은 위험은 [v9 독립 소스 경계·하드웨어 한계](20261009-v9-contact-review.md)의 동일 source/runtime hash로 유지한다. 특히 idealray 기반 accuracy/validity와generic texture는 actual PMW/ToF quality가 아니고, 정확flag true인 raw<4cm 표본이433/467/479에12/12/4개 있다. PC metric gate와 달리 stock onboard fullEKF는accuracy flag 없이 new range를 받는 경로가 남는다. near camera depth<.30m 전체frame 제거는 실제PSF가 아니다. 실제 mount/K/D/mass/CoM/추력/lag/마찰/gyro 잔류·센서시각·RTOS·radio·camera 전달은S/U다. observer와plant가 공유하는 추력·접촉 가정은 ground effect/모델변화에서 거짓 접촉 가능성을 배제하지 못한다.

고정4.2V에는SOC/전류/배터리 에너지·deck부하·M1M2소비가 없다. 제조사 stock7분 표기를 이 모델의3미션 잔여 비행시간으로 가져올 수 없다. 실제 Mission2 경로 인계가 아닌1m hover fixture이고 전체3미션·실기 착륙·제조사 일치·공식승격은 승인하지 않는다. 추가 holdout은 별도5분모·새 원로그로 검토해야 한다.

원로그/원평가/소스/config/이전서명은 수정하지 않았다. 새 검토 helper의 config 설명필드 누락·전송지연 가정 오류는 검토 단계에서 발견·정정하고 실패 로그를 보존했다. gate/원물리/evaluator에는 변경이 없다. 개별 helper의 좁은 보정·plan 범위 표기는 `audit-scope-clarifications.json`에서 전체 raw 재생·별도plan 검사로 보완했다. 네 서명의 공통 HOLD/개발필드 설명은 세 승인 run의 관측이며457에는HOLD/개발검토 파일이 없음을 위 실패 절에서 명시한다.
