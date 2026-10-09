**v23 독립 판정: 원 조건부 pad 접촉 네 건 승인, reference-relative 후보 채택 거절.**

검토자 Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`. 원 staged/coordinated 각각 seed571/619 두 건의 재사용 training을 source별로 검토했다. 전체4와 정책별2 분모를 유지하고 v21/v22·unused holdout과 합치지 않는다. Constant-heading D 항의 수식 반례 해결은 확인했지만 실제 오차/연속성 선택 기준은 미달이다. qualified/formal/제조사/hardware/진입2 승인 모두0이다. 새 physics/agent/shared 수정은 없다.

원 [plan](../../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23/plan.json), [contract-at-plan](../../../artifacts/mission3/acquisition/20261009-reference-relative-approach-paired-training-v23/contract-at-plan.md), 각 run의 source-snapshot/config/settings/trial/evaluation과 raw를 검사했다. Plan SHA `9af7c0d8326d752237797547d8e55602b709d0af57379c2fa54542e87ea312ca`; source69+dependency config3+runtime10=core82, 사용 config4가 일치한다. 정확한 source/input/evidence SHA는 [source 감사](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/source-series-verification.json), 각 새 서명과 [index](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/signed-review-index.json), [final](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/final-verification.json)에 결합했다. 현행 source나 번역 중인 문서로 원 bytes를 대체하지 않았다.

| 정책 / seed | 실제 접촉 / PWMoff / 3초 hold 끝, s | 내부 오차 / 점수 | 원 native exact 프레임 | 접촉 판정 |
|---|---:|---:|---:|---|
| staged571 | 106.26 / 107.16 / 110.16 | 14.585mm / 9 | 1,645 | 조건부 승인 |
| staged619 | 243.76 / 244.64 / 247.64 | 9.375mm / 10 | 4,534 | 조건부 승인 |
| coordinated571 | 104.16 / 105.00 / 108.00 | 33.088mm / 7 | 1,598 | 조건부 승인 |
| coordinated619 | 242.52 / 243.40 / 246.40 | 26.389mm / 8 | 4,515 | 조건부 승인 |

원 evaluator로 전4 JSON 전체를 재계산해 일치했다. 각 hold에 네 발 양의 pad 법선력151개 50Hz truth 표본·PWM0 3,001개 1kHz 표본을 확인했다. 개별 발 최저 법선력 .033122/.033200/.031770/.031358N, 최대 상대 slip7.09/4.57/16.06/12.79µm였다. 실제 contact 표본197/196/194/196, hull/PCB/arm 또는 pad 이외 floor/body/net 접촉 없음, contact/hold 직선·차속 .25m/s 연속이었다. Calibration/support와 원 sensor-only FSM·명령·모터 곡선/lag·build manifest·Cstamp/packet·영상 FFV1 전12,292프레임 SHA를 직접 검사했다. Main의 unsigned 판정과 접촉 불일치는 없었지만 이를 독립 서명으로 복사하지 않았다. [실제 개별 계산 집계](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/examined-result-summary.json)가 표본 수/힘/오차의 근거다.

**사전 개선 기준은 미달이다.**

첫 LOWER_ALIGN~첫 실제 접촉의 `abs(actual vz)<.005m/s`를 50Hz 왼쪽 truth 표본 구간으로 적분했다. 1kHz 전체 actual 속도 적분으로 표현하지 않는다.

| seed | staged → candidate 정지 s | 감소율 / 20% 기준 | 접촉 오차 증가 / 10mm 기준 |
|---|---:|---:|---:|
| 571 | 10.78 → 6.76 | 37.29%, 통과 | +18.503mm, 실패 |
| 619 | 9.44 → 7.80 | 17.37%, 실패 | +17.014mm, 실패 |

같은 pair 합계 감소는20.22→14.56s,27.992%지만 두 오차 retention 실패와619 개별 정지 실패를 대체하지 않는다. 접촉4/4만으로 후보를 채택하거나 unused holdout을 사전 승인하지 않는다. 자체 계산 후 main continuity와 차이가 roundoff 이내임을 확인했다. [독립 선택 계산](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/independent-paired-selection-audit.json)에 판정과 원 분모를 보존했다. Staged 각각10개 원 raw 파일은 v22 baseline과 byte/SHA 동일다. 새 source 회귀 서명이지 새 효능 표본이 아니다. Candidate raw는 별도 새 trajectory이며 과거 서명을 복사하지 않았다.

![v23 정지·접촉오차·차등 상대오차](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/paired-continuity-and-relative-error.png)

**D 수정의 제한된 검증과 유지된 gate.**

v22 대비 실행 source 변화는 controller와 test_approach_reference 두 파일이다. Candidate config의 의미 변화는 `coordinated_velocity_feedback_policy='reference_relative'`뿐이며 metadata와 구분했다. Default `vehicle_relative`는 유지한다. 원 [controller:711](../../../artifacts/mission3/physical/20261009-coordinated-approach-v23-entry1-seed571/source-snapshot/scripts/mission3/controller.py:711)는 D 입력을 `rel_v−forward*sdot`로 바꿨다. Constant-heading 완벽한 reference tracking에서 D=0, midpoint .3125m/s로 기존 .290625m/s 상쇄 반례를 고쳤다. 세 heading×세 reference 시각×두 policy의 CPU fixture18개를 확인했다. 이를 noisy50Hz 추종·실기 jerk·turn 보증으로 확대하지 않는다. Estimated heading이 변하면 `forward_dot*standoff` 미분항은 여전히 누락된다; .5deg/s와 midpoint .475m의 별도 해석 반례는 source의 constant-heading 한계를 보여주며 실제 run 위반이라고 단정하지 않는다.

원 시험9개를 직접 재실행해 통과했고, 같은 새 시험을 이전 frozen v22 controller에 적용해 알려진2fail/0error를 재현했다. Main before 로그와 after 전체228PASS 로그의 plan SHA도 확인했다. 전체228개를 본인이 재실행했다고 쓰지 않는다. [독립 CPU before/after와18fixture](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/reference-feedback-CPU-audit.json)에 원 source/test SHA·supplementary default 시험 config를 구분했다. 해당 default config는 시험 좌표 전용이며 비행 config 대체가 아니다.

Concurrent는 height<.025/cross<.035 확인을 reference 끝에서 누적하고 total1s·실패reset·reference완료 필수 조건을 유지한다. 완료 뒤 새로1s를 관측하는 계약과 동일하지 않다. Actual571 ready93.06→전환95.42s(2.36s),619231.74→234.18s(2.44s), post-reference 추가 확인0이다. 두 전환 높이 .402536/.402015m·vz +.000120/−.001185m/s·cross−.000193/−.000676m, LOWER50Hz gap 위반0을 검사했다. CPU12fixture에서 조기완료금지/reset/strict outer gates를 확인했고, 기존 held API의 sparse sample gap·임의 vertical-speed 상태 한계도 유지했다. [Concurrent 실제/반례 감사](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/concurrent-gate-CPU-and-actual-audit.json)에 있다. 원 TRACK2s·영상품질·불확실도·직선·terminal/support/hold 문턱은 별도로 남아 있다.

**말기 횡오차: own 전파와 frozen target을 함께 계산한다.**

571 접촉 실제 pad−own XY는(−15.141,+29.420)mm, 추정은(−6.298,−1.249)mm다. Cross 추정−실제는−30.669mm였다. 619 실제는(−.974,−26.371)mm, 추정은(−4.493,+1.135)mm여서 cross 차이+27.506mm다. Common world gauge가 같게 이동하면 소거되지만, 이 사례는 own과 pad의 **차등 drift**다. Contact own/pad cross gauge는571 +33.796/+3.126mm,619 −44.551/−17.045mm로 차등 relative와 일치한다. 기록 순서의 own snapshot/후속 visual anchor gap 최대 .721/.596mm도 반영했다.

Actual terminal handoff→첫 접촉 구간을 같은 50Hz 로그로 적분했다. 부호는 estimated−actual velocity/error다.

| 후보 말기 cross 예산 | 571,100.48~104.16s | 619,239.32~242.52s |
|---|---:|---:|
| Frozen target FF cross | +.000510m/s | −.003826m/s |
| FF 오차 적분 | +1.878mm | −12.243mm |
| Own velocity 오차 적분 | +29.972mm | −34.341mm |
| FF minus own 예측 relative-error 증가 | −28.094mm | +22.098mm |
| 실제 기록 relative-error 증가 | −28.213mm | +22.253mm |
| Closure 잔차 | −.118mm | +.155mm |

Along 예산도 별도 CSV에 남겼다. 571/619 actual cross 위치 변화는 +28.392/−21.965mm; command−actual cross velocity 적분은 +22.517/−29.563mm다. 이 response 항은 own velocity/피드백과 종속되어 독립 drift 원인으로 더하면 중복 계산이다. [terminal 예산](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/terminal-error-budget-all-cases.json)과 [gauge/자세 CSV](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/terminal-relative-gauge-all-cases.json)는 후평가 truth 비교만 사용한다. Fix가 실제 along lag·영상을 바꿨지만 말기 cross unobservability를 고쳤다는 증거는 아니다.

마지막 metric image age는 접촉4.66/4.18s이며 관측 처리 지연은 원 모든 valid record의 합성 .060s다. PC attitude/average body-force 회전의 offline cross 차이와 동결된 effective floor bias를 함께 봤다. 후보571의 평균 rotation 차이약+.00315m/s² 대비 bias+.00209, 순항 약+.00106이고,619는+.00289 대비 bias+.00494여서 순항 약−.00205m/s²다. Floor bias update 수는 terminal 내8,299/18,933으로 고정이다. 이 bias는 qualified floor의 속도 innovation으로 배운 모델/자세 residualS이며 제조사 gyro calibration 값이 아니다. Chip bias나 단일 원인으로 확정하지 않는다.

원 [PhysicalMotion](../../../artifacts/mission3/physical/20261009-coordinated-approach-v23-entry1-seed571/source-snapshot/scripts/mission3/physical_estimator.py:49)의 scheduling에서 PC predict는 tick%10==0, deck 소비는%10==1이다. Deck100Hz body mean의 nominal midpoint는 consumer보다4.5ms 앞, PC prediction은 consumer보다1ms 앞이며 합성 동일 clockS다. Actual attitude를50Hz truth에서 선형 보간한 midpoint 민감도는 후보 precontact mean cross 차이 약−7.8e−6/−1.5e−6m/s²로 작았지만,1kHz 실제 box average/자세시간을 재구성한 것은 아니다. Firstcontact endpoint의 충격/average-vs-latest 차이를 별도 precontact 통계와 구분했다. [IMU alignment 민감도](../../../artifacts/mission3/reviews/20261009-v23-reference-relative-approach-independent/IMU-alignment-sensitivity-all-cases.json)의 범위 밖에서 원인을 단정하거나 실기 timing 검증으로 승격하지 않는다.

후속 후보는 FF 한 상수만 교정하기보다 last-fix velocity/cov/age·floor effectivebias 동결 시점·capture-aligned IMU gravity compensation을 각각 분리한 paired 시험이 필요하다. No-Flow 구간에서 truth로 bias를 갱신하거나 support/영상/불확실도 문턱을 완화해서는 안 된다. Fixture 수식 수정 통과만으로 실제 lateral retention을 승인하지 않는다.

**센서·물리·검증의 지속되는 한계.**

HOLD 정확ToF0/validFlow0이다. IMU−명령 모터 모델 support는 실 pad/네 발 식별 보증이 아니다. 원 raw onboard ToF lower gate0은 그대로이고4cm 미만 호출198/198/192/191회가 남는다. PC 독립 IMU context와 full onboard EKF는 별개이고 근접 ground-model/covariance 영향·현실 range response가 해결된 것으로 표시하지 않는다. 원 scalar Z/vZ arithmetic 재생오차0/fallback0, held40Hz range·C timestamp/local14byte RPYT100Hz를 검사했지만 전체 firmware/RTOS/radio/ValueError 회복·arbitrary numerical transaction은 검증하지 않았다.

정상 명령 가속도 .15m/s²는 유지했다. Actual max3D speed .40815/.40531/.40793/.40452m/s는 .4 commandcap과 다르다. PWMoff 첫 실제 모터 합력 .2017~.2024N, .3초 뒤약6.17~6.19e−8N으로 감쇠하므로 PWM0가 기계적 즉시정지는 아니다. 기체 qpos/qvel 초기화 뒤 강제쓰기 없고 prescribed vehicle .25m/s만 계속 지정했다. Plant contacttruth는 live control 입력이 아니며 sensor 생성/후평가와 실험 floor/tilt harness를 구분했다. Safe recovery/real radio supervisor 승인도 없다.

제조사 부품 문서P·보존 공식 codeC·analytic reference/geometryD·합성 finitepatch/motor/contact/sensor timingS·실 mount/K/D/PSF/readout/질량/CoM/배터리/응답/quality/clock U를 유지한다. 일정4.2V에는 SOC/energy/앞 두 미션 잔량이 없다. 공개 값이나 소프트웨어 상수를 실제 조립 기체의 측정값으로 쓰지 않는다.

원자료908파일과 이전 판단112개를 시작/종료 SHA로 검사하며 전체 immutable source/evaluation/input/evidence를 새 서명에 결합한다. Helper 코드만 재사용했고 실행 전 네 actual RUN 경로를 assert했으며 prior 결과/서명은 복제하지 않았다. 영상 해시 전량과 후보 montage·독립 plot QA를 수행했다. CPU/gate·원로그 재계산은 실제 완료됐고 새 physics는 없으며, 미수행 실기/전체 perception-firmware/1kHz capture alignment·유일한 인과분리는 한계로 남긴다.
