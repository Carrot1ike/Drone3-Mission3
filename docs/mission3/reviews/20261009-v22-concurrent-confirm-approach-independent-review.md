**v22 독립 판정: 네 조건부 pad 접촉 승인, concurrent 후보 채택 거절.**

Heisenberg / Codex-Independent-Flight-Reviewer, agent `01a117a1-f802-75f1-9980-91813ff9f0f9`가 원 staged571/619와 coordinated571/619를 별도로 검사했다. 각 정책은 재사용 training 2/2이며 v21·unused holdout과 합치지 않는다. 정지 감소 기준은 통과했지만 619의 접촉 XY 오차 증가가 사전 10mm 한도를 넘었다. qualified/formal/제조사/hardware/진입2 승인 모두 0, 새 비행/에이전트 0이다.

원 [plan](../../../artifacts/mission3/acquisition/20261009-concurrent-confirm-approach-paired-training-v22/plan.json), [contract-at-plan](../../../artifacts/mission3/acquisition/20261009-concurrent-confirm-approach-paired-training-v22/contract-at-plan.md), 각 source-snapshot/raw/settings/evaluation을 사용했다. 원 whole4, source69/runtime10/dependency config3의 core82와 사용 config4, 시험225개 원 로그 SHA를 검증했다. v21 대비 바뀐 실행 source는 controller와 test_approach_reference 두 파일뿐이며 원 scene·camera·cart phase·range fit·caps/contact gates는 같다. Candidate 설정 차이는 `coordinated_confirm_policy='concurrent'`, 나머지 차이는 고지된 batch metadata다. [source diff 및 평가 재계산](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/source-series-verification.json)에 정확한 bytes/hash를 보존했다. 이후 현행 source/설명 문서를 원 코드 대신 사용하지 않았다.

| 원 policy / seed | 실제 접촉 / PWMoff / 3초 hold 끝, s | 내부 오차 / 점수 | native exact 프레임 | 조건부 판정 |
|---|---:|---:|---:|---|
| staged571 | 106.26 / 107.16 / 110.16 | 14.585mm / 9 | 1,645 | 승인 |
| staged619 | 243.76 / 244.64 / 247.64 | 9.375mm / 10 | 4,534 | 승인 |
| coordinated571 | 104.00 / 104.86 / 107.86 | 18.122mm / 9 | 1,599 | 승인 |
| coordinated619 | 242.88 / 243.78 / 246.78 | 24.565mm / 8 | 4,516 | 승인 |

전 네 건 평가를 원 evaluator로 재계산해 원 JSON 전체와 일치했다. 각 hold에 네 발 양의 pad 지지 151개 50Hz truth 표본과 PWM0 3,001개 1kHz 표본을 확인했다. 최저 발 법선력은 .033122/.033200/.033196/.031486N, 최대 상대 slip은 7.09/4.57/8.81/11.92µm다. 원 contact 표본197/196/195/197, pad 이외 접촉이나 hull/PCB/arm 접촉 없음, actual contact와 hold 모두 직선·차속 .25m/s 연속이다. 새 calibration/support 재생과 sensor-only FSM·실제 명령·모터 lag·build manifest·serialization 검사를 마쳤다. Unsigned development-review를 서명으로 복사하지 않았고 독립 판정 불일치는 없었다. [최신 원 검토 집계](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/examined-result-summary.json), [네 새 서명 index](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/signed-review-index.json), [최종 SHA 결합](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/final-verification.json)이 해당 판정 범위다.

**사전 개선 기준과 원 분모.**

첫 LOWER_ALIGN~실제 첫 접촉의 `abs(actual vz)<.005m/s`를 50Hz 왼쪽 truth 표본 구간으로 계산했다. 1kHz 실제 속도 전체 적분으로 표현하지 않는다.

| seed | staged → candidate 정지 s | 감소율 / 20% 기준 | 접촉 오차 증가 / 10mm 기준 |
|---|---:|---:|---:|
| 571 | 10.78 → 7.16 | 33.58%, 통과 | +3.538mm, 통과 |
| 619 | 9.44 → 7.16 | 24.15%, 통과 | +15.190mm, 실패 |

두 pair 합계 20.22→14.32s로 29.179% 감소하나 619 retention 실패가 남는다. main continuity는 자체 계산 뒤 비교했고 차이는 roundoff 이내였다. 접촉 성공과 후보 선택의 문턱을 바꾸지 않았다. [독립 선택 계산](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/independent-paired-selection-audit.json)에 분모4·정책별2·candidate_selected false를 기록했다. Staged의 각 10개 raw 파일 SHA는 v21 원 baseline과 byte 동일다; 반복을 추가 독립 효능으로 세지 않는다. Candidate는 원 새 trajectory/raw/evaluation으로 확인했다.

![v22 정지·접촉오차·말기 상대추정오차](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/paired-continuity-and-relative-error.png)

**Concurrent dwell의 의미와 반례 범위.**

원 [controller:710](../../../artifacts/mission3/physical/20261009-coordinated-approach-v22-entry1-seed571/source-snapshot/scripts/mission3/controller.py:710)는 `abs(height−lower)<.025`, `abs(cross)<.035`, 관측 dwell1s를 유지하고 reference complete가 참일 때만 APPROACH로 전환한다. Concurrent는 완료 전의 height/cross ready 시간도 누적한다. 따라서 **완료 후 새로 1초를 관측하는 기존 after_reference와 동일한 시간 계약은 아니다**. 이것은 opt-in 목적에 해당하는 의미 변경이며 문턱·총 관측 dwell을 보존했다는 제한된 주장만 인정한다.

Actual candidate571은 ready 누적93.06s부터, reference 완료95.42s에 APPROACH(총2.36s);619는231.72→234.18s(2.46s)다. 완료 뒤 추가 dwell은 두 건 모두0이다. 전환 추정 높이 .401905/.401863m, vz −.000471/−.001222m/s, cross −.002906/−.000280m였고 LOWER 명령 간격 최대 .0200000000001s, 끊긴 표본0이다. Reference 미완료 상태의 조기 전환이나 실패 후 이전 hold시간 재사용은 없었다.

원 시험6개를 archived source로 실행하고 독립 fixture12개에서 조기 ready·height/cross 실패·cross 경계·stale/nonmetric/불확실도·기존 after_reference 시간 차이를 확인했다. 원 [held API](../../../artifacts/mission3/physical/20261009-coordinated-approach-v22-entry1-seed571/source-snapshot/scripts/mission3/controller.py:195)는 두 호출 사이 maxgap을 확인하지 않으므로 sparse 두 표본만으로 elapsed dwell을 만족하는 기존 한계가 있다. 높이/cross 확인에 vertical-speed 문턱도 없어서 임의 −.15m/s 추정상태를 안전하다고 보증할 수 없다. 이 CPU 한계를 이번 실제 50Hz run의 위반으로 오인하지 않는다. [CPU 및 actual 전환 감사](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/concurrent-gate-CPU-and-actual-audit.json)에 보존했다. TRACK2s·영상/직선/불확실도/terminal prediction6s·support/hold 조건은 후속 분기로 유지된다. 미래 adapter에서는 명시적 sample-gap/freshness 계약을 권고하며, 여기서 gate를 완화하거나 patch를 승인하지 않는다.

**FF freeze·relative pose·camera timing·하위 응답 분해.**

v21의 `ff−forward*sdot+.65*e+.35*rel_v` 반례를 원 v22 source에서도 재현했다. 완벽한 moving-reference 추종의 midpoint에 D 항이 −.021875m/s를 상쇄해 raw속도 .3125→.290625m/s, 준정상 along lag33.654mm를 요구한다. 이 along 피드백 이슈는 concurrent dwell로 고쳐지지 않는다. Nominal reference C2는 실제 FB jerk/하위응답 연속성 보증이 아니다. [analytic CPU 감사](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/approach-reference-CPU-audit.json)는 시험 전용 copied default config의 SHA와 flight config가 아니라는 점을 명시했다.

619 terminal handoff239.40s~첫 접촉242.88s의 actual relative 위치는 along9.837→16.369mm, cross−2.896→−18.317mm였다. Frozen estimated target FF는 (.252483,−.001028,0)m/s, actual cart는 (.25,0,0)다. 접촉의 relative 추정−실제는 along−21.555mm/cross+18.569mm이며, own 추정 속도−실제는 along+.007910/cross−.008302m/s다. 자체 relative가 near zero로 보여도 실제 잔차가 커질 수 있다.

동일 terminal 구간의 오프라인 예산은 다음과 같다. 항들은 종속된 closed loop 경로이므로 독립 원인 비율로 더하지 않는다.

| 619 terminal 오프라인 적분 | along | cross |
|---|---:|---:|
| frozen target FF 오차 적분 | +8.641mm | −3.578mm |
| own velocity 추정 오차 적분 | +28.499mm | −18.946mm |
| FF minus own 예측 relative-error 증가 | −19.858mm | +15.368mm |
| 실제 기록 relative-error 증가 | −19.825mm | +15.461mm |
| command minus actual velocity 적분 | −4.181mm | −17.986mm |

첫 네 행의 closure 잔차 .032/.093mm는 작다. 말기 **차등 relative drift**를 world 공통 gauge 탓으로만 설명할 수 없다. 마지막 행은 추정 오차에 대응하는 실제 하위 응답과 명령 차이이며 별도 독립 drift항이 아니다. 571 terminal FF along .245986m/s는 반대 방향 오차이고 접촉 추정−실제 cross−13.875mm; 동일 source에서도 초기 영상/Flow/IMU 상태와 frozen 속도가 달라진다. 단일 FF 상수만 바꾸는 권고는 근거가 부족하다.

접촉 마지막 metric image age는 candidate4.46/4.48s이고 valid 관측 처리시각−capture 시각은 모든 case 정확히 합성 .060s다. 이것은 hardware 영상/radio/readout/PSF 지연 검증이 아니다. 영상 보정 뒤 vehicle/anchor와 먼저 기록한 own sensor snapshot 사이 gap 최대 .727/.567mm도 반영했다. PC-vs-actual 자세와 averaged body acceleration 회전 진단은 약 .004m/s² cross 잔차 후보를 보이나 1kHz sensor averaging과 capture alignment를 완전 분리하지 않았으므로 유일한 원인으로 단정하지 않는다. [terminal 오차 예산/CSV](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/terminal-error-budget-all-cases.json)와 [gauge/속도 계산](../../../artifacts/mission3/reviews/20261009-v22-concurrent-confirm-approach-independent/terminal-relative-gauge-all-cases.json)은 전부 후평가 truth 비교다.

후속 paired 시험에서는 reference-relative velocity error, last-image FF innovation/cov/age와 capture speed, 시간 정렬한 gravity-compensation/영상 delay/하위 PID 응답을 따로 바꿔야 한다. 현재 실제 stop 감소와 함께 오차가 악화되므로 무조건 연속 하강·관측 gate 완화·truth 기반 online 보정은 승인하지 않는다.

**지속되는 조건부 센서·물리 한계와 검사 범위.**

전 hold accurateToF0/validFlow0, near support는 추정 IMU−명령 모터 모델S다. 실제 pad 발 식별·실기 접촉 보증이 아니다. 원 raw onboard ToF lower gate0, 4cm 미만 호출198/198/194/197회와 부정확 range를 받은 full EKF 경로가 남고 PC IMU는 별도 독립 context다. 원 scalar Z/vZ 계산/새 stamp/repeat40Hz 동작은 오차0·fallback0이나 전체 융합의 arbitrary numerical atomicity/ValueError recovery/1kHz-camera-radio 보증으로 확대하지 않는다.

원 packet14byte·100Hz local wire·C stamp/call선택·mono FFV1 전12,294프레임 exact를 검사했다. 명령 가속도 .15m/s²는 지켰지만 actual 3D speed는 .40815/.40531/.40710/.40405m/s로 .4 commandcap보다 크다. PWMoff의 actual 모터 합력 .2017~.2022N은 이후 lag 감쇠이며 즉시 기계정지가 아니다. 일정4.2V에는 energy/SOC/앞 두 미션 잔량 모델이 없다. P/C/D/S/U는 v21과 같은 구분으로 유지: 부품 문서P·공식 원코드C·analytic referenceD·현재 모터/finitepatch/contact/timingS·실장 K/D/mount/PSF/CoM/질량/응답/quality/실시각 U. 기체 init 뒤 qpos/qvel 강제쓰기 없고 prescribed cart .25만 연속 지정한다. Plant contacttruth는 sensor 생성/오프라인 평가이며 live controller 입력이 아니고, truth floor/tilt 종료는 별도 실험 harness다. Safe recovery/real wire supervisor를 승인하지 않는다.

원자료520개와 이전 판단108개의 SHA 불변을 최종 검사했다. 검토 중 재사용 contact/FSM helper 두 종류의 run 경로가 v21에 남아있던 오류를 독립 continuity/평가 비교로 발견했다. 서명 전 해당 결과와 잘못된 요약을 `rejected-helper-run-binding` 등에 보존하고 **네 원 v22 접촉/FSM을 재검사**했다. Source/evaluation/native/packet/support 원 v22 검사는 별도로 일치했으며 최종 서명은 수정된 원입력 결과만 사용한다. 자체 configdiff field명·trial metadata 누락, np.arange 완료시각 roundoff fixture, summary reference-key 오류/재실행 로그도 보존했다. 어떤 원자료/공유 source/config/기존 서명도 수정하지 않았다. 전체 perception/firmware/RTOS replay·실기 clock/geometry/동역학/전지·단일 원인 분리는 수행하지 않았다고 명시한다.
