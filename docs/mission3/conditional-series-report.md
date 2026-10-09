# 조건부 series 보고서 독립 검토

2026-10-08. 처음에는 [Aggregator](../../scripts/summarize_mission3_series.py)를 읽기 전용으로 검토했다. 이후 사용자가 aggregator 수정 권한을 명시적으로 반환하여 aggregator·[검사](../../scripts/test_mission3_series_summary.py)·이 문서·own artifacts만 수정했다. Controller·coordinator·config·평가기·library는 수정하지 않았고 실제 미션을 실행하지 않았다.

## 수정 결과

[최종 34개 검사](../../artifacts/mission3/series-summary-review/20261008-081118-236983-tests/report.json) **34/34 통과**, 3.404초, exit0. [직전 32개 검사](../../artifacts/mission3/series-summary-review/20261008-080722-934736-tests/report.json)와 원래 23개 검사·추가 중간 identity 변경 반례를 모두 유지했다. 추가로 실제 coordinator 형태의 plan/state/snapshot 누락, job/status/plan digest 불일치, schema 제거를 통한 state 우회, public controller 설정 변경, 관련/무관한 broken review의 분리, legacy 직접 실행과 사전/후속 seed 이력 구별을 검증했다. 수정 전 code/test/doc은 [원본 snapshot](../../artifacts/mission3/series-summary-review/20261008-075952-060557-repair-original/source-sha256.json), 과거 실패 batch는 아래 링크에 그대로 남는다.

실제 [v2 재평가 보고서](../../artifacts/mission3/series-summary-review/20261008-080420-234234-tests/old-v2-repaired-report.json)와 [v3 재평가 보고서](../../artifacts/mission3/series-summary-review/20261008-080532-384085-tests/v3-repaired-report.json)는 각각 예정/완료5/5, 접촉5, 독립 승인5, invalid review0, conditional gate true, **formal0**이다. Raw 완료 로그를 immutable evaluator 경로로 다시 읽었으며 과거 평가/서명을 덮어쓰지 않았다. v2는 `legacy_direct_plan`, v3는 `coordinator_plan`으로 provenance를 구별한다. v2 seed107의 이후 training 재사용은 `later_seed_uses`에 기록하며 원래 v2의 사전 seed 오염으로 잘못 분류하지 않는다.

Coordinator plan에는 `plan.sha256`, archived management source, plan과 각 worker의 source/library snapshot bytes, frozen settings/profile, trial hash 및 exact series/split/entry/seed/job/output을 결합한다. State는 plan digest와 모든 예정 job을 결합하고 complete·정상 process provenance·manifest_check valid를 요구한다. `worker_succeeded/returncode0` 또는 **완료 물리 미션 실패 `worker_failed/returncode2`**를 구별한다. 후자는 completed manifest·adapter-summary.failure·state manifest_check의 동일한 termination reason과 보존 output/bytes가 있어야 한다. 이 정상 착륙 실패는 N에 남기되 ≥5 독립 승인 접촉이 있으면 conditional gate를 차단하지 않는다. Source 변경 flag/event나 stop reason, 출력 누락·incomplete·timeout·cancel·crash 및 무결성 실패는 gate를 차단한다. 최종 hash가 다시 같아져도 변경 관측은 지우지 않는다. Coordinator schema를 지우거나 state가 남은 폴더를 legacy로 재표시하여 우회할 수 없다. Legacy는 coordinator 상태/management 증거가 없는 과거 `runs`/top-level hashes 형태이며 각 run의 보존 bytes·trial·public projection은 계속 검증한다.

사용자가 재확인한 조건은 **사전 선언 N회 전부 완료·무결 +독립 승인 접촉 ≥5**이며 N/N 성공을 요구하지 않는다. [합성10회 검사](../../artifacts/mission3/series-summary-review/20261008-081118-236983-tests/test_ten_completed_with_five_approved_and_five_preserved_landing_failures_pass_gate/observed-report.json)는 접촉 승인5+정상 보존된 착륙 실패5를 모두 남기고 gate true/formal0이다. 반대로 exit2인데 일치하는 완료 실패 output이 없으면 gate false다. 이는 관리 규칙 fixture이며 실제 미션을 새로 실행한 결과가 아니다.

사전 seed 사용 검사는 metadata만 읽는다. Explicit training paths와 declared reuse를 검사하고 기존 coordinator registry의 정확한 fixture 구별 규칙을 재사용한다. Registry 손상은 오류를 기록하고 gate를 차단한다. 일반 외부 시도는 plan 생성 시각과 run metadata timestamp, timestamp가 있는 run ID 또는 보존 `experiment.json` filesystem mtime을 비교한다. 시각을 판단할 수 없는 충돌은 보수적으로 사전 사용으로 취급한다. Filesystem mtime은 변경 가능한 조건부 provenance이며 암호학적/global formal 증명이 아니다. Synthetic registry 검사는 다른 basename의 metadata와 registry lookup fixture를 사용하여 실제 project seed registry를 오염시키지 않는다.

Core `validate_review`에 더해 제공된 trial/library/plan/series/entry/seed/run_dir/evaluation-file hash 확장값도 대조한다. 잘못된 확장 결합은 승인 집계에서 제외하고 invalid로 남긴다. Broken review JSON은 파일별 `review_read_errors`로 보존하여 전체 scheduled 분모를 유지한다. Run ID에 연결된 손상 기록은 해당 실행의 invalid review이며, 연결 불가능한 손상 파일은 승인할 수 없는 별도 오류다. Independent reviewer JSON은 공개키 신원 인증이 아니다.

`formal_qualified_count`는 항상0이며 `global_formal_protocol_verified=false`다. Local subset `immutable_ledger`는 진단용이며 전역 training/entry 순서/외부 formal protocol 증명을 대신하지 않는다. 승인된 raw 접촉 수와 source/state freeze gate를 구분하고, 손상·실패·pending은 예정 전체 분모에 보존한다. 아래 표의 gate true/처리 오류는 **수정 전 재현 결과**다.

## 검사와 실제 v2 기록

최종 [24개 검사 결과](../../artifacts/mission3/series-summary-review/20261008-075514-707567-tests/report.json)는 **11개 통과, 안전 기대조건 위반 12개, 처리 오류 1개**, 1.904초, exit 1이다. 추가 검사는 coordinator가 관측한 중간 identity 변경을 최종 bytes 복원으로 지울 수 없는지 검증한다. 실패를 expectedFailure로 숨기지 않았다. [기존 23개 검사](../../artifacts/mission3/series-summary-review/20261008-074111-652257-tests/report.json), [첫 22개 검사](../../artifacts/mission3/series-summary-review/20261008-073924-206884-tests/report.json), [첫 상태 반례](../../artifacts/mission3/series-summary-review/20261008-075450-297338-tests/report.json)도 보존했다. 최종 상태 반례는 실제 coordinator의 `identity_change` stop reason, `identity_changed_during_run=true` 및 returncode0인 `identity_changed` job 형태에 맞췄다. 테스트는 시작 시 aggregator source를 복사해 고정하고 실제 evaluator/ledger를 사용했다. Synthetic offline truth·motor·native320 camera 로그와 synthetic review를 own fake-project에 만들었으며, 합성 승인·manufacturer profile은 실제 증거가 아니다. 모든 synthetic run manifest는 `test_double=true`로 실제 project seed registry와 구별한다. 첫 batch의 가짜 prior-training metadata도 원 bytes를 별도 보존한 뒤 이 표시를 명시했다.

통과한 검사는 5개 조건부 접촉 승인/formal 0, 미완료·누락의 scheduled 5 유지, 완료 접촉 실패 1개의 분모 유지, training manifest 비승격, 재사용 flag의 conditional gate 차단, 중복 seed·리뷰 누락 차단, 선언된 code/config/library 및 entry/seed 불일치 차단이다. 서명 전 template·self-review·자동 reviewer·code/config/evaluator/evaluation digest 불일치·review ID 누락·조건부 실행의 qualified 승인도 거부했다.

[기존 v2 읽기 전용 감사](../../artifacts/mission3/series-summary-review/20261008-074111-652257-tests/old-v2-read-only-binding-audit.json)는 예정/완료 **5/5**, 조건부 접촉 승인 5, invalid review 0, conditional gate true, **formal qualified 0**을 확인했다. 저장 reevaluation과 개별 review의 canonical digest 및 보고서의 파일 hash, 실제 frozen config/trial bytes, 각 source/library snapshot이 일치했다. Raw truth를 재평가하거나 이 sidecar가 v2 접촉 승인을 새로 작성한 것은 아니다. 아래 반례는 이 정상 v2 기록을 소급해서 실패로 바꾸는 근거가 아니다.

## 수정 권고와 재현

| 우선순위 | 독립 재현 결과 | 권고 |
|---|---|---|
| P1 | Frozen config/source/library/trial **bytes만 변경**하고 manifest hash를 유지하면 gate true: 4개 실패 | 원본 실행의 보존 snapshot/config/trial 파일 SHA를 plan/manifest와 실제로 대조. 현재 live source를 과거 실행에 대입하지 않음 |
| P1 | 기존 `plan.sha256`와 plan bytes가 달라도 gate true | `mission3-series/1`이면 coordinator plan digest 검증을 적용. Legacy plan의 digest 근거는 별도로 표시 |
| P1 | Plan에 별도 prior-training path와 같은 entry/seed가 있어도 gate true | Plan의 미사용 선언만 신뢰하지 말고 training/holdout 사용 이력을 metadata로 확인하고, 불명확하면 gate pending/false |
| P1 | Plan split=training인데 run split=holdout이면 gate true; 다른 series_id도 gate true | plan/job/manifest의 split·series·entry·seed·trial identity를 모두 결합 |
| P1 | 최종 snapshot/hash/평가/리뷰가 모두 일치해도 coordinator `identity_change`와 job `identity_changed_during_run=true`/`identity_changed`를 무시해 gate true | coordinator state를 plan digest·series·job ID/output에 결합하고 source 변경 관측을 보존. Worker returncode0 또는 최종 hash 복원이 중간 변경을 취소하지 못하게 함 |
| P1 | 합성 qualified 사례에서 `seeds_unused_at_entry=false` 또는 library mismatch인데 `formal_qualified_count=5` | 이 조건부 보고서는 공식 승격을 수행하지 않음. Local ledger count를 공식 count로 해석하지 말고, 전역 protocol/training 이력 및 series freeze를 충족하지 않으면 formal count를 0 또는 미검증으로 표시 |
| P2 | Review에 제공한 잘못된 trial/evaluation-file hash를 무시하고 gate true | 현재 core review 계약의 필수 code/config/evaluator/canonical-evaluation 결합은 작동한다. 확장 필드를 검증했다고 주장하려면 제공된 필드도 별도 대조하거나 미검증으로 명시 |
| P2 | 무관한 broken signedreview 하나가 전역 review scan에서 JSONDecodeError를 내어 보고서 전체 생성 중단 | 전체 scheduled job 상태를 보존하고 review 파일별 오류를 보고. 관련 run의 손상 review를 조용히 valid 처리하지 않음 |

`scheduled`는 최초 계획 N이고 완료 실패·pending은 top-level에 보존된다. `immutable_ledger`에는 완료 평가만 들어가므로 pending까지 포함한 전체 ledger 분모와 동일하지 않다. 현재 conditional gate는 전원 완료에 더해 **승인 접촉 ≥5개**를 요구한다. N>5에서 일부 실패가 있어도 승인 5개이면 true가 될 수 있으므로 전원 성공률이나 N/N 성공이라고 표현하면 안 된다. `signedreview`는 현재 계약상 독립 reviewer JSON 기록과 hash 결합 검사이며 공개키 기반 신원 인증/암호학적 서명 검증은 아니다.

반례별 raw fixture와 `observed-report.json` 또는 `observed-exception.json`, traceback을 보존했다. [최종 handoff](../../artifacts/mission3/series-summary-review/20261008-075514-707567-tests/handoff.json)는 신규 파일 및 읽은 aggregator/evaluator hash를 기록한다. [이전 handoff](../../artifacts/mission3/series-summary-review/20261008-074111-652257-tests/handoff.json)도 그대로 있다. Main의 새로운 frozen physical series에는 이 sidecar 코드를 주입하지 않았다.

Coordinator 상태 반례의 [state.json](../../artifacts/mission3/series-summary-review/20261008-075514-707567-tests/test_coordinator_identity_changed_blocks_gate_after_bytes_restored/state.json)과 [관측 보고서](../../artifacts/mission3/series-summary-review/20261008-075514-707567-tests/test_coordinator_identity_changed_blocks_gate_after_bytes_restored/observed-report.json)를 대조할 수 있다. 현재 report는 state를 아예 읽지 않는다. 같은 source identity인 다른 series의 holdout을 연결하는 [series_id 반례](../../artifacts/mission3/series-summary-review/20261008-075514-707567-tests/test_manifest_series_id_must_match_plan/observed-report.json)도 별도 실패로 남는다. 상태 파일이 없는 legacy 직접 실행과 coordinated series는 provenance를 구별하고, coordinated series의 상태 누락·불일치·중간 변경은 scheduled 분모에 보존하면서 gate를 false 또는 미검증으로 둬야 한다.

## 읽기 전용 직선 준비 gate 평가

Main의 opt-in `lineup_lowering_requires_straight`는 정렬 ready에 `stable_straight` 및 준비+terminal+hold 직선 여유를 결합하고, 그 조건 전체를 LINEUP hold에 넣었다. 이는 곡선에서 .4m로 조기에 내려 calibration을 오래 갱신하지 못한 training139 반례에 맞는 entry gate다. 곡선 LINEUP은 .7m 목표를 유지한다. 높이만으로 calibration 갱신을 보장하지 않으며 floor-clear Flow·accurate range·추정 높이>.65m·자세·IMU 잔차 조건도 필요하다.

현재 gain에서 .7→.4m, 오차 .025m까지의 이상적 하강은 `ln(.3/.025)/.8 + 1초 hold = 4.11초`다. Standoff .65→.30m, 오차 .035m까지의 이상적 수평 접근은 3.54초이고 velocity feedback의 이상적 폐루프를 포함하면 약 4.78초다. TRACK hold 2초까지 총 **9.65~10.89초**이므로 준비 12초는 조건부 초기 시험값이다. Actual actuator lag·수평/수직 결합·영상 gate 대기를 포함하는 검증된 상한은 아니다.

`준비12 + terminal12 + hold3 = 27초`를 **추정 차량 속도**로 거리화한다. 설명용 .25m/s이면 6.75m이고 기존 endpoint 여유 .5m를 합쳐 남은 거리 **>7.25m**다. Config 속도나 live plant curve는 controller 입력으로 쓰지 않는다. 현재 entry gate 뒤 LOWER_ALIGN/APPROACH에는 이 preparation budget의 실행 timeout/곡선 회수 검사가 없으므로, entry gate만으로 무한 준비 지연이나 calibration 만료를 방지한다고 주장할 수 없다. 후속 변경 후보는 남은 준비시간·직선 여유·calibration 유효시간을 계속 재검사하고, 부족하면 .7m 추적으로 회수하는 경로다. 기존 Pxy는 유지·전파하고 위치/방향 오차와 정지거리의 geofence 여유를 별도로 판단해야 한다.

권장 boundary 검사는 양방향 직선의 필요 거리 직전/정확 경계/직후(strict `>`), stable-straight 확인 직전/직후, hold 도중 margin 또는 straight 상실 시 timer reset, 준비 지연·속도 추정 증가·영상 stale·곡선 재진입 시 회수, .7m지만 calibration 갱신 불가/남은 유효시간 부족, opt-in off 기존 동작 유지다. 모든 경우 covariance reset·truth 입력·공중 motor-off를 허용하지 않는다. 이 절은 source 읽기와 계산에 따른 제안이며 실제 mission case를 실행한 검증이 아니다.

완료 v3의 별도 raw-log 독립 리뷰에서 seed179의 LOWER_ALIGN→BLIND_APPROACH는 **15.76초**, seed181은 **12.22초**였다. 둘 다 실제 하강~hold는 직선이고 접촉/off/3초 지지를 확인했지만, nominal preparation12초를 이미 초과했다. 따라서 준비값을 안전 상한으로 승격하기 전에 단계 내 계속 검사·유한 timeout/회수와 covariance/직선 여유를 검증해야 한다. 서명은 각 실행에 따로 있으며 [seed179](../../artifacts/mission3/independent-review/20261008-physical-straight-preparation-holdout-entry1-seed179-codex-series-reviewer/signedreview.json), [seed181](../../artifacts/mission3/independent-review/20261008-physical-straight-preparation-holdout-entry1-seed181-codex-series-reviewer/signedreview.json) 모두 조건부 접촉만 승인하고 qualified=false다. 실제 v3 coordinator state는 중간 identity 변경 표시 없이 전 job worker_succeeded였으며 위 합성 state 반례를 실제 v3 실패로 해석하지 않는다.

## v4 경계 실패에 관한 읽기 전용 의견

Main이 보고한 v4 seed193의 마지막 추정 optical depth .299699m가 .30m handoff gate 아래로 내려간 문제에는 **추정 pose를 state/quality/anchor에 쓰기 전** opt-in 최소 깊이 .32m로 거부하는 방향이 타당하다. 이후 읽은 현재 `vision.py`에는 이미 그 위치의 `metric_pose_min_depth_m` guard가 있다. 새 near-boundary pose를 거부할 때 이전 reliable pose의 capture/receipt stamp·quality·regression/anchor history를 갱신하지 않아야 한다. 새 영상 도착 자체로 옛 fix의 age를 0으로 만들면 안 된다. 기존 image age·전체 prediction time·straight/covariance·calibration freshness 제한은 유지한다. 이는 처음부터 acquisition을 차단하거나 터미널 prediction 여유를 소모할 수 있으므로 .32m가 두 실패를 해결했다고 아직 말할 수 없다.

90px side와 1.5px reprojection RMSE에서 20mm reserve를 택하는 것은 조건부 S다. RMSE는 depth 오차의 상한이나 확률 표준편차가 아니며 corner quantization·기울기·optics/model 오차가 함께 작용한다. `z≈fL/s`의 소오차 계산은 한 변 오차1.5px이면 .30×1.5/90≈5mm, 두 끝점 합산3px이면≈10mm라는 설명만 제공한다. 제조사 DOF의 lower limit와 추정 depth 오차 한계를 혼동하지 않는다. 권장 검사는 .32 직전/동일/직후·비유한 depth, good→marginal→good sequence, 거부 후 stamp/history/anchor 불변, old fix age 만료 후 회수다. Native320 계약과 기존 ambiguity/reprojection/innovation gate도 그대로 필요하다.

Main이 보고한 seed199의 실제 pad contact80샘플 뒤 추정 body height<.0025m가 먼저 되어 off confirm이 끝나지 않은 사례는 **높이 gate를 넓힐 근거가 아니다**. 접촉 후보 직전의 추정 높이·vz·마지막 accurate range 및 visual datum 시각·명령 rotor settling·IMU residual/near-g의 시간 순서를 분리해 확인해야 한다. 관측하지 못한 높이를 clamp하거나 near-g를 pad contact로 간주하지 않는다. 높이 gate를 충족하지 못하면 유한 회수/실패를 유지하며 새 estimator/probe 변경은 기존 gain-change 공중 false-off 반례와 paired 재검증해야 한다. v4의 신규 접촉 서명이나 5회 승격은 이 문서에서 수행하지 않았다.
