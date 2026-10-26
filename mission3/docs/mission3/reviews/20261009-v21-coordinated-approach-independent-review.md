**v21 독립 판정: 원 조건부 물리 접촉 네 건 승인, coordinated 후보 채택 거절.**

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 사용자가 09:00 이후 재승인한 CPU 후평가 범위다. 새 비행·에이전트·공유 코드 변경 없이, staged와 coordinated 각각 재사용 training seed571/619 두 건을 검사했다. 두 정책의 2/2 접촉을 일반 성공률·unused holdout·실기 성과로 확대하지 않는다. qualified/formal/제조사/hardware/진입2 승인 모두 0이다.

근거는 원 [plan](../../../artifacts/mission3/acquisition/20261009-coordinated-approach-paired-training-v21/plan.json), [contract-at-plan](../../../artifacts/mission3/acquisition/20261009-coordinated-approach-paired-training-v21/contract-at-plan.md), 각 run의 source-snapshot/frozen settings/trial metadata/raw/evaluation이다. 현행 v22 controller나 변경된 설명 문서를 원 v21 코드로 쓰지 않았다. 최종 [개별 서명 index](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/signed-review-index.json)와 [최종 결합 검증](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/final-verification.json)에 원 입력·source·runtime·평가 파일/canonical digest·검토 산출물 SHA를 결합했다.

| 원 policy / seed | 접촉 / PWMoff / 3초 hold 끝, s | 내부 XY 오차 / 점수 | 원 mono320 exact 프레임 | 판정 |
|---|---:|---:|---:|---|
| staged571 | 106.26 / 107.16 / 110.16 | 14.585mm / 9 | 1,645 | 조건부 접촉 승인 |
| staged619 | 243.76 / 244.64 / 247.64 | 9.375mm / 10 | 4,534 | 조건부 접촉 승인 |
| coordinated571 | 105.08 / 106.02 / 109.02 | 29.356mm / 8 | 1,620 | 조건부 접촉 승인 |
| coordinated619 | 243.74 / 244.64 / 247.64 | 16.915mm / 9 | 4,535 | 조건부 접촉 승인 |

원 평가를 archived evaluator로 네 건 각각 재계산해 일치했다. 각 hold는 50Hz truth의 네 발 양의 pad 법선력 151표본, 1kHz PWM0 3,001표본이다. 최저 개별 발 법선력은 각 .033122/.033200/.032364/.031998N, 최대 상대 slip은 7.09/4.57/14.25/8.22µm였다. 실제 차속은 .25m/s로 연속이고 접촉/hold는 직선이다. 발 이외 hull/PCB/arm, pad 이외 floor/body/net 충돌로 성립한 접촉은 없었다. 원 scene/build manifest·finite patch·모터 곡선/lag·support/calibration·hold 명령·native FFV1 전 12,334프레임 SHA를 직접 대조했다. main의 unsigned development-review와 접촉 판정 불일치는 없으나 그것을 서명으로 복사하지 않았다. [개별 계산 집계](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/examined-result-summary.json)에 표본/힘/오차/영상 수를 보존했다.

**연속 접근의 사전 선택 기준은 미달이다.**

첫 LOWER_ALIGN부터 실제 첫 접촉 직전까지 `abs(actual vz)<.005m/s`인 50Hz truth의 왼쪽 표본 구간을 적분했다. 이것은 1kHz 전체 plant 속도 적분이나 실제 센서 측정치가 아니다. 오프라인 truth를 판정에만 사용했다.

| seed | staged 정지 s → coordinated 정지 s | 감소율 / 20% 기준 | 접촉 오차 증가 / 10mm 기준 |
|---|---:|---:|---:|
| 571 | 10.78 → 8.00 | 25.79%, 통과 | +14.772mm, 실패 |
| 619 | 9.44 → 8.50 | 9.96%, 실패 | +7.540mm, 통과 |

같은 두 pair 합계도 20.22→16.50s, 감소 18.3976%로 미달이다. 개별/합계 해석 모두 후보를 거절한다. 자체 계산 이후 main continuity와 대조해 차이는 수치 roundoff 수준이었다. [독립 선택 감사](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/independent-paired-selection-audit.json)의 원 분모와 기준을 유지했다. staged 두 건의 각 10개 원 raw 파일은 v20 baseline과 SHA/byte 동일이며 [main 원 감사](../../../artifacts/mission3/acquisition/20261009-coordinated-approach-paired-training-v21/baseline-byte-regression.json)도 직접 결합했다. 새 서명은 source 회귀 검토이며 새로운 착륙 능력 표본은 아니다.

![v21 paired 정지·접촉오차·말기 상대오차](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/paired-continuity-and-relative-error.png)

**원 source와 명령 경계.**

원 source69, dependency config3, runtime10의 core82와 사용 config/profile·experiment SHA를 검사했다. v20 대비 controller/adapter 변경 및 reference/test 추가를 구분했다. staged 공개 controller 설정은 원 baseline과 같고, candidate는 `coordinated_quintic`, duration10.5s와 preparation budget14.5s(기준12s)를 사용한다. 차이/metadata는 [config audit](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/config-policy-all-cases.json)에 보존했다. budget은 admission 계산용이고 실제 preparation의 강제 hard timeout으로 해석하면 안 된다. staged 실제14.88/13.50s를 숨기지 않는다.

원 [reference](../../../artifacts/mission3/physical/20261009-coordinated-approach-v21-entry1-seed571/source-snapshot/scripts/mission3/approach_reference.py:1)의 quintic은 위치/속도/가속도 endpoint C2, nominal 최대 standoff rate .0625m/s·height rate .053571m/s·height accel .015710m/s²다. jerk endpoint 연속이나 실제 피드백 jerk 제한은 아니다. LOWER/APPROACH/TRACK/BLIND의 원 sensor/vehicle estimate를 원 controller에 입력해 명령·yaw를 1e-10 허용오차 안에서 재생했다. full perception/firmware/RTOS 재실행으로 확대하지 않는다. 네 건 정상 3D 명령 가속도는 .15m/s²를 roundoff 내 지켰다. 실제 최대 3D 속도 .40815/.40531/.40554/.40487m/s는 명령 .4m/s 한도와 다르다.

원 [adapter](../../../artifacts/mission3/physical/20261009-coordinated-approach-v21-entry1-seed571/source-snapshot/scripts/run_mission3_physical.py:206)의 terminal/support 입력은 추정 상태·IMU·명령 모터 모델이다. 실제 pad 발 접촉/법선력은 live controller 분기에 들어가지 않는다. truth 기반 floor/tilt 실험 harness 종료는 별도로 존재하며 실기 supervisor로 간주하지 않는다. 기체 qpos/qvel은 초기화 뒤 직접 덮어쓰지 않고 prescribed vehicle .25m/s만 계속 허용했다.

14byte RPYT serialization·100Hz 로컬 발행·C uint32 ms stamp/call 선택·camera stamp를 재계산했다. 원 raw onboard ToF lower gate는 0: 네 건 4cm 미만 호출198/198/195/198회이고 range_accurate false도 존재한다. PC IMU attitude는 deck와 독립이지만 onboard full EKF의 근접 range/ground-model 영향이 사라졌다는 증거는 아니다. HOLD 정확 ToF/유효 Flow는 전부 0이다. [stamp/packet audit](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/packet-and-C-stamp-all-cases.json)은 합성 동일 clockS와 local wire 검사이며 실제 capture/arrival offset·radio ACK·RTOS chronology는 U다. Scalar Z/vZ fit은 원 새 range별 재생오차0, fallback0이며 전체 manifold VIO/융합 수치 안전/회복 증명이 아니다.

**오차 증가: along 피드백 결함 후보와 cross 관성 전파를 구분한다.**

원 [controller:703](../../../artifacts/mission3/physical/20261009-coordinated-approach-v21-entry1-seed571/source-snapshot/scripts/mission3/controller.py:703)는 `ff-forward*sdot+.65*e+.35*rel_v`다. 완벽한 moving reference 추종에서 e=0, rel_v=sdot*forward를 넣는 CPU 반례로 midpoint 이상 속도 .3125m/s 대비 raw .290625m/s를 얻었다. D 항이 −.021875m/s를 상쇄해 준정상 along lag .033654m를 요구한다. 속도 피드백을 reference-relative error로 정의하는 별도 시험이 합리적이다. 임의 변경/채택을 승인하지 않으며 실제 횡오차 전부를 이 along 식으로 설명하지 않는다. [CPU 반례](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/approach-reference-CPU-audit.json)는 원 4개 시험, uncertainty/stale visual/straight/margin 유지도 확인했다. 시험 좌표 전용 default config를 별도 복사하고 SHA를 남겼으며 실제 비행 config로 대체하지 않았다.

Candidate571 접촉 실제 pad−drone XY는 (+4.689,+28.979)mm인데 같은 시각 추정 relative XY는 (+.871,−.955)mm다. 횡 relative 추정−실제는 −29.935mm; BLIND 시작 −1.882mm에서 더 벌어졌다. terminal handoff에서 own cross velocity 오차 +.006362m/s, 접촉에서 +.010993m/s이며 terminal frozen target cross FF도 +.000978m/s(실제0)다. 접촉 마지막 metric image age4.54s이다. along D lag, 이 target FF, own 관성 속도/자세 잔차 및 영상 갱신이 동시에 바뀌어 단일 원인 분리에는 부족하다.

접촉 estimated own Y gauge는 실제보다 +35.808mm, estimated pad Y는 +5.874mm로 **상대오차는 차등 drift**다. 공통 world gauge가 동일하게 이동하면 relative에서 소거되지만 이 사례는 그렇지 않다. 로그 own sensor snapshot과 이후 vehicle/anchor 갱신의 순서 차이로 순간 pad−own vs logged relative 차이 최대 .491mm가 있어 반영했다; 이를 29.935mm 오차로 오인하지 않았다. 적분 velocity/FF budget도 영상/gauge 갱신과 heading 변화 때문에 유일한 인과분해가 아니다. 후보619는 frozen along FF 약 .245659 vs 실제 .25m/s로 차이가 있으나 실제 own velocity와 피드백도 함께 봐야 한다. [오프라인 gauge/속도 감사](../../../artifacts/mission3/reviews/20261009-v21-coordinated-approach-independent/terminal-relative-gauge-all-cases.json)에 독립 CSV와 계산을 보존했다.

다음 수정 시험은 reference-relative D 항, 마지막 영상의 상대속도/cov/age에 대한 terminal capture budget, 시간 정렬한 PC IMU gravity-compensation/영상 지연을 각각 분리해야 한다. truth 상수로 온라인 튜닝하거나 영상/uncertainty/support gate를 완화하면 안 된다. reference가 끝나도 TRACK·inertial bridge에 남은 정지 시간이 크므로 nominal C2 하나로 실제 하강 연속성을 보장할 수 없다.

**남은 조건과 불변 검증 범위.**

P는 제조사 부품 문서의 출력/각도/동작 범위, C는 archived 공식 firmware/serialization, D는 analytic reference/geometry, S는 현재 센서 생성·finite patch·trajectory·모터/lag/support/contact 설정이다. 실제 camera K/D/PSF/readout·deck/body mount·CoM/mass·quality/timing·접촉·하위 응답은 U/S로 남는다. PWMoff 첫 표본 실제 합력은 .2012~.2022N이고 .3초 뒤 약6.15~6.18e−8N으로 감쇠한다. PWM0는 기계적 즉시 정지가 아니다. 일정4.2V 모델에는 배터리 energy/SOC/탑재 중량 및 앞 두 미션 잔량이 없다.

원자료685개와 이전 판단104개를 시작/종료 SHA로 대조한다. scope 밖 빈 setup 파일 한 개를 즉시 허용 폴더로 옮긴 경로 정정, terminal 로그의 snapshot 정렬 가정 수정, matplotlib 없는 환경의 첫 plot 실패/별도 기존 venv 재실행은 검토 폴더 안에 공개했다. 원 log/evaluation/source/config/기존 서명은 수정하지 않았다. 원 영상 픽셀·plot QA도 완료했다. 빠진 실기/전체 firmware-perception replay를 성공으로 기록하지 않았으며, v22는 이 서명과 분리된 후속 검토다.
