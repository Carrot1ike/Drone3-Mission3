# v20 retained-history preflight — 독립 CPU 패치 검토

Heisenberg / Codex-Independent-Flight-Reviewer (`01a117a1-f802-75f1-9980-91813ff9f0f9`)의 판정은 **제한된 이력 시각 오류 거부 패치 통과**다. 새 fullphysics 실행·agent 생성·착륙 서명은 0이다. v20의 네 fullphysics 원자료/접촉/포화/픽셀 평가는 이 검토에서 승인하지 않았다. qualified/formal/제조사/hardware/진입2 승인은 모두 0이다.

[원 계획](../../../artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/plan.json) · [계획 당시 계약](../../../artifacts/mission3/acquisition/20261009-history-preflight-regression-training-v20/contract-at-plan.md) · [새 CPU 판정](../../../artifacts/mission3/reviews/20261009-v20-history-preflight-patch-independent/patch-review.json) · [최종 SHA 검증](../../../artifacts/mission3/reviews/20261009-v20-history-preflight-patch-independent/final-verification.json)

네 실행 초기 source-snapshot의 scripts67/runtime10을 plan과 각각 대조했다. core80 중 의존 config3개는 snapshot에 없어 계획 SHA와 일치하는 보조 파일을 읽었다; 출처를 source-patch-audit.json에 명시했다. 선택 config3개도 plan SHA가 같으며 CPU fixture의 두 config는 본인 폴더에 복사해 보존했다. 실행 코드와 library는 snapshot에서 import했고 현행 source로 바꾸지 않았다. v19 대비 변경 파일은 estimator.py/sensor_time.py/test_sensor_chronology.py 세 개뿐, physical_estimator.py/controller/plant/evaluator/runtime은 불변이다. active raw·run_manifest·launch log는 main이 쓰는 중이므로 불변 완료자료로 취급하거나 결과를 승인하지 않았다.

[새 validate_sample_time](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v20-entry1-seed571/source-snapshot/scripts/mission3/estimator.py:142)는 packet 시각 검사 뒤 retained range/IMU 이력의 형상·finite 값·엄격 증가·consumer 이전 조건을 검사한다. imu_compensated에서 prospective IMU stamp가 마지막 이력보다 커야 하고, 새 유효 range도 마지막 range 이력보다 producer가 커야 한다. [PhysicalMotion의 호출 위치](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v20-entry1-seed571/source-snapshot/scripts/mission3/physical_estimator.py:73)는 그대로이며 이 검사가 C deck 측정/PC accumulator 소비 전에 호출된다. [history_time](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v20-entry1-seed571/source-snapshot/scripts/mission3/sensor_time.py:26)의 minimum0은 빈/단일 시작 이력을 허용하고 direct fit에서는 minimum2를 유지한다.

| 실제 CPU 검사 | v19 원 source | v20 원 source |
|---|---|---|
| 같은 v20 시험 module의 8개 method | 새2시험의20 subcase 실패 / error0 | 8개 통과 / failure0·error0 |
| 본인 이전 정확한 future-retained-range 반례2개 | SelfMotion/PhysicalMotion 모두 부분 상태 변경 후 ValueError | 모두 ValueError / 변경 필드0 |
| 추가18개 malformed/prospective 반례 | 별도 반복 범위 아님 | 전부 상태 변경0으로 거부 |
| 빈·단일 시작/held40Hz 표본 | 기존 정상 계약 | 6 fixture 통과, held 높이/C ToF 반복 융합0 |
| 정상 empty/late/gapped IMU coverage | 기존 fallback 계약 | 각 −0.03m/s, 명시적 선형 fallback1회 |
| direct fit 빈·단일 range | 최소 표본 제약 | 각각 거부, minimum2 보존 |

이전 정확한 fixture는 consumer .030s/producer .025s에 retained range .040s를 주입했다. v19에서 position/last_stamp/range_heights/vertical_imu, PhysicalMotion의 velocity·C core·counts·attitude history가 바뀌는 것을 다시 재현했다. v20은 그 호출 전에 거부한다. 별도18개에는 단일 future range, NaN IMU, Inf range, 3열/1차원/ragged 이력, prospective range equality/backwards, prospective IMU equality를 양 관측기로 넣었다. 검사는 producer latch·공분산·C 상태·PC accumulator·계수·이력을 포함한다. current timestamp equality를 retained helper에서 허용하고 nextafter 미래를 거부했다. 원 전체219시험 로그 SHA/OK를 확인했으며 본인이 재실행한 범위는 보존된 새 module8 method와 위 추가 CPU 반례다. 219개 전체를 본인이 다시 실행했다고 주장하지 않는다.

**판정 한계.** 이 수정은 retained finite/shape/time와 prospective **순서** 검사를 앞당긴 것이다. prospective 측정의 모든 숫자·단위/임의 C 실패까지 검증하거나 전체 update를 transaction/rollback으로 만들지는 않는다. 본인 별도 현재 horizontal NaN acceleration fixture는 첫 update에서 position이 nonfinite가 되는 것을 확인했다; 유효 입력·시각 이력 계약 밖의 반례로 기록했다. 따라서 수치 안전 보증을 붙이지 않는다. 외부 telemetry adapter를 연결할 때 필수 수치 필드 검증과 오류 기록·폐기/복구 정책을 별도 구현·시험하는 것이 타당하다.

ValueError 이후 실기 제어 전달/복구는 여전히 미구현이고 [1kHz IMU ingestion](../../../artifacts/mission3/physical/20261009-baseline_abort_delivery-v20-entry1-seed571/source-snapshot/scripts/mission3/physical_estimator.py:44)은 이 deck preflight보다 먼저 C 상태를 갱신한다. 전체1kHz IMU·camera·radio chronology, clock reset/offset/arrival/reordering/RTOS/ACK/supervisor·임의 numerical error 원자성은 검증하지 않았다. 공통 simulation clock S와 hardware timing U를 유지한다. 선형 fallback은 부족한 IMU coverage에서 유지되며 full range fusion quality gate/VIO/bias 추정/착륙 보증이 아니다. 근접 raw ToF·HOLD无정확ToF/Flow·mount/K/D/PSF·동역학/전지/SOC S/U도 해결하지 않는다.

**남은 별도 업무.** v20 네 완료 비행의 새 평가·10 raw SHA 대조·정상 포화·native FFV1·실제 접촉/모터off/직선3s와 양 negative 실패 전달 검토는 deferred다. main의 실행 상태나 기대한 v19 결과를 새 성공으로 계상하지 않았다. 해당 검토가 완료되기 전 새 landing signature/contact acceptance는 없다. 본인은 새 물리를 시작하지 않았고 CPU 작업은 모두 종료했다.

| 원 SHA 결합 | SHA256 |
|---|---|
| plan | `11a4a682fcdbc7f90d270e949c599044a153863e4f75bd1214a38bd1167ebd6f` |
| contract-at-plan | `89b4740e6f2ea00f44f12fb2d34c5de03fcfc782a60278993884c21486a7f6fa` |
| full219 test log | `4ed38f73db29e23b07cfb819cbfe42ba956ea5910a775ba47cb75a30356e1ca3` |
| `scripts/mission3/estimator.py` | `cf028bc03b3c5fcb134b4fb37e17a0bc07dac90ab1076a350de0b8715bd1b97d` |
| `scripts/mission3/sensor_time.py` | `66a70fc11c5ca20a305a34bb08c672b90c389dd663197de441be953ae09f335f` |
| `scripts/mission3/test_sensor_chronology.py` | `e181556c64cd9947be868a8f0400331ac098be6fbc0e764e514ec7fc8129970d` |
| `scripts/mission3/physical_estimator.py` | `30cb9636f49b6c486474c5ff005e5b5e7095fb31ef4cb5ae3250125945c3cd25` |

[원 snapshot 차이 및 출처](../../../artifacts/mission3/reviews/20261009-v20-history-preflight-patch-independent/source-patch-audit.json) · [v19 반례 재현](../../../artifacts/mission3/reviews/20261009-v20-history-preflight-patch-independent/patch-counterexamples-v19.json) · [v20 실제 검사](../../../artifacts/mission3/reviews/20261009-v20-history-preflight-patch-independent/patch-counterexamples-v20.json)

초기 setup은 core의 의존 config3개가 snapshot에 있다고 가정해 assertion을 냈다. 실제 누락을 확인한 뒤 SHA 일치 보조 파일로 한정하고 그 출처를 공개했다. 원 source/문서/평가/이전 서명은 바꾸지 않았다. 최종 검증은 보존된 원본과 이전99+v19판정4개까지 불변 및 본인 CPU 근거 결합을 확인한다.
