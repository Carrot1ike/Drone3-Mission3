# 동결 series 실행 관리자

2026-10-08 구현 범위는 [coordinator](../../scripts/run_mission3_series.py), [관리 테스트](../../scripts/test_mission3_series.py), 이 문서 및 `artifacts/mission3/series-runner/`다. 공유 controller·estimator·physical runner·config·C·library는 수정하지 않았다. **실제 미션을 실행하지 않았으며, 아래 검증은 독립 fake worker의 subprocess 관리 검증이다.** Main의 진행 중인 training/holdout series에는 적용하지 않았다.

## 계획과 실행 API

`create_plan(out, *, root, profile, entry_id, seeds, split, series_id, ...)`는 명시한 순서대로 job 전체를 고정한다. `dry_run(plan_path)`는 worker를 시작하지 않고 identity·seed 충돌·마감을 확인한다. `execute_plan(plan_path)`는 그 계획을 한 번 실행한다. CLI는 `plan`, `dry-run`, `run` 세 단계다.

아래는 사용 형식이며 이 문서를 작성하면서 실행한 명령이 아니다. 실제 미사용 seed 목록과 profile을 Main이 정한 뒤 새 출력 디렉터리를 지정한다.

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
src/crazyflie-ros2-sil/.venv/bin/python scripts/run_mission3_series.py plan \
  --out artifacts/mission3/series-runner/NEW-SERIES \
  --entry 2 --seeds SEED_A SEED_B SEED_C \
  --split holdout --series EXPLICIT-SERIES-ID \
  --profile config/CHOSEN-FROZEN-PROFILE.json --workers 3

src/crazyflie-ros2-sil/.venv/bin/python scripts/run_mission3_series.py dry-run \
  --plan artifacts/mission3/series-runner/NEW-SERIES/plan.json

src/crazyflie-ros2-sil/.venv/bin/python scripts/run_mission3_series.py run \
  --plan artifacts/mission3/series-runner/NEW-SERIES/plan.json
```

`SEED_A` 등은 정수로 치환해야 한다. 실제 worker argv는 list이고 `shell=False`다. 경로의 공백·세미콜론은 단일 인자로 전달된다. `.venv/bin/python`의 symlink 경로를 유지한다. 이를 `resolve()`해 system Python으로 바꾸지 않는다.

| 선택 | 기본값·의미 |
|---|---|
| `--entry` | 1; 알려진 static entry 목록에 있어야 함 |
| `--split` | holdout |
| `--series` | 필수, 명시한 식별자 |
| `--workers` | 3; 1~3만 허용 |
| `--cutoff` | mission config의 `deadline_kst`; 더 이른 timezone 포함 시각만 허용 |
| `--worker-timeout` | 900초 wall time; 미션 시뮬레이션 budget과 별개 |
| `--timeout` | 미지정 시 기존 mission timeout 유지; 정수 입력도 runner와 같은 float로 동결 |
| `--video`, `--perturb-entry`, `--allow-near-focus` | 계획에 명시한 경우에만 기존 physical runner 인자로 전달 |
| `--allow-training-reuse` | training에서만 명시적으로 허용; holdout 재사용 금지 |

이번 session의 기본 cutoff는 **2026-10-08 09:30 KST**다. Coordinator는 탐색/미션 budget을 늘리거나 임계값을 바꾸지 않는다. 모든 worker에도 `--respect-session-deadline`을 전달한다.

## 동결 identity와 provenance

`plan.json`은 `mission3-series/1`이고 별도 `plan.sha256`로 byte 변경을 거부한다. 알려진 entry 좌표/초기 방향, entry별 seed 목록, split/series, 순서 있는 모든 job/argv, cutoff, source/library/input hash를 저장한다. Native `320×320 mono8` 설정을 요구한다.

현재 [physical runner](../../scripts/run_mission3_physical.py)의 `frozen_experiment_settings`와 같은 순서의 `{"mission": cfg, "physical": physical, "strict_focus": bool}`을 `indent=2` 및 마지막 newline으로 저장한다. 이 hash에는 seed/entry를 넣지 않는다. `experiment.json`의 entry/seed 포함 bytes는 별도 trial metadata hash로 비교한다. Runner의 순수 settings 함수 AST가 이 schema에서 바뀌면 계획을 거부한다.

Source inventory는 현재 `run_manifest.json`과 같다: physical runner, `scripts/mission3/*.py` 전체(테스트 포함), `scripts/mission3/*_host.c`, `yaw_core.py`, `aruco_pose.py`; `pc_velocity_rpyt`이면 `mission3_rpyt_protocol.py`도 포함한다. Library inventory는 controller/firmware-host `.so`, RPYT일 때 position-bridge `.so` 및 존재하는 `source-manifest.json`, `autoconf.h`, `FreeRTOS.h`다. 원본 mission/profile/provided-hardware config hash도 추가 launch guard로 저장한다.

`source-snapshot/`에 source/library를 복사하고 hash 일치를 확인한다. 원본 config 세 개와 정규화한 `frozen-profile.json`, `frozen-experiment-settings.json`, coordinator/test 관리 소스도 보존한다. 실제 worker는 동결 profile을 사용하지만 **실행 코드와 build library는 기존 project 경로를 사용**하므로 실행 전·실행 중 hash 변경 감시가 필요하다.

Seed 사용 조회는 project의 `artifacts/mission3/**/run_manifest.json` metadata만 읽는다. 같은 entry의 seed는 `completed=false`도 이미 사용한 것으로 취급한다. Training 재사용이면 기존 manifest 경로를 plan에 남긴다. 읽을 수 없는 실제 registry는 미사용으로 간주하지 않고 오류를 낸다. `test_double=true` fake manifest는 미션 시도 목록에서 제외한다. 이 판단은 **해당 project/entry에서 기록된 시도 범위**이며, 다른 project의 실행이나 manifest를 아직 쓰지 않은 수동 예약까지 보장하지 않는다. 동시 독립 계획 간 seed 예약 lock은 없다. Main의 seed 배정이 필요하다.

중첩된 `series-runner/<batch>/<test_case>/fake-project/`는 그 경로 이름만으로 제외하지 않는다. 새 테스트는 `fixture-provenance.json`에 `mission3-series-test-fixture/1`, producer/test hash, case, 정확한 fixture root, synthetic scope 및 `test_double=true`를 기록한다. 이 marker와 fake hardware/profile metadata가 일치하는 subtree만 실제 project registry에서 제외한다. Marker 이전 archive는 batch report의 synthetic scope와 artifact 경로, report가 기록한 SHA256과 일치하는 보존 test source, 그 source에서 확인한 case 이름 및 fake hardware/profile metadata를 요구한다. 이전 archive에 marker를 소급해서 쓰거나 손상 manifest를 고치지 않았다.

Fixture 자체를 scan root로 검사할 때는 그 내부 registry를 그대로 검사하므로 corrupt-registry negative test도 계속 유효하다. Provenance가 없거나 맞지 않으면 손상 manifest를 계속 거부한다. **`series-runner` 전체를 제외하지 않는다.** 실제 `series-runner/<series>/workers/...`의 seed는 집계하고, 그 안의 손상 manifest도 오류로 처리한다. 이 구분은 로컬 테스트 provenance 규약이며 적대적 파일 위조를 막는 인증 체계는 아니다.

완료 worker에서는 `run_manifest.json`의 schema, completed, entry/seed/split/series, code/library/config/trial hash와 실제 settings/trial 파일 bytes를 모두 비교한다. `experiment.json`은 정적 실험 설정이다. Coordinator는 truth trajectory, contact truth, adapter summary, evaluation 결과를 읽지 않는다. Model S physical profile은 worker에 전달하는 실험 입력이며 controller에 live vehicle truth를 제공하는 경로를 만들지 않는다.

## 실행·중단·분모

기존 nonempty 출력은 계획 단계에서 거부한다. Worker 출력 중 하나라도 이미 있으면 실행 전 전체 launch를 거부한다. 계획 생성 및 실행에는 각각 exclusive owner 파일을 두며 동일 plan의 동시 실행과 자동 재시작을 거부한다. 실패하거나 cutoff로 미실행된 계획을 같은 경로에서 재개/재시도하지 않는다.

각 worker는 새 session/process group으로 시작한다. `state.json`에 직접 생성한 PID·PGID·argv·시작 요청/종료 시각·returncode·manifest 검사·신호 기록을 남긴다. 동시 worker는 최대 3개다. 신호 전에는 보유한 `Popen`이 살아 있고 `PGID == PID`인지 확인한다. Timeout/cutoff 또는 coordinator CLI의 SIGINT/SIGTERM은 자체 group에만 SIGINT → 기본 5초 후 SIGTERM → 기본 1초 후 SIGKILL을 보낸다. 반복 중단 신호는 이 drain 동안 무시한다. 이름 기반 `pkill`이나 Main의 다른 process를 종료하는 경로는 없다.

Cutoff를 loop와 각 launch 직전에 확인하며 기본 0.05초 launch margin을 둔다. 만료 뒤 대기 job을 시작하지 않고 active job에는 graceful stop을 요청한다. 신호 grace와 OS scheduling 때문에 **기존 worker의 회수 완료 시각은 cutoff 뒤일 수 있다**. CLI 자체 SIGKILL/호스트 종료는 catch할 수 없으므로 자동 orphan 회수를 보장하지 않는다. Library/source/config drift가 관측되면 후속 launch를 막고 기존 active job을 `identity_changed`로 남긴다. 이 경우 active job은 기존 timeout/cutoff 안에서 종료하게 두며 성공 상태로 세지 않는다.

| Job 상태 | 의미 |
|---|---|
| `worker_succeeded` | process exit 0이며 동결 manifest/파일 hash 일치 |
| `worker_failed` | exit nonzero 또는 manifest 불일치/누락 |
| `identity_changed` | active 동안 source/config/library 변경 관측 |
| `timed_out`, `cancelled_at_cutoff`, `interrupted`, `coordinator_error` | 중단 사유를 그대로 보존 |
| `launch_failed` | 해당 worker 시작 실패 |
| `not_launched_*` | cutoff/identity/seed/output/launch 오류 때문에 예정됐으나 시작되지 않음 |

`summary.json`의 `scheduled`는 최초 계획 N이며 모든 상태의 합이 N이다. 실패·미실행을 제거해 분모를 줄이지 않는다. `started`는 기록된 PID가 있는 job 수다. CLI run exit 0은 N개 모두 `worker_succeeded`일 때만 나온다. 이는 process/manifest 결과이고 **물리 착륙 또는 qualified success 판정이 아니다**. `qualified_mission_success=0`, `independent_reviews_performed=0`을 유지한다. 독립 agent의 offline 접촉·3초 hold·경계 검토는 기존 수동 절차로 수행해야 한다.

## 검증 기록

Team `.venv` Python으로 `PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 .../python scripts/test_mission3_series.py`를 실행했다. Fake root에 만든 `.so`는 표시용 bytes이며 로드하지 않았다. ROS·MuJoCo·실제 미션·radio/hardware command는 실행하지 않았다.

| 보존된 검사 artifact | 결과 |
|---|---|
| [첫 batch](../../artifacts/mission3/series-runner/20261008-071659-580157-tests/report.json) | 24/24 통과 |
| [CLI 확장 batch](../../artifacts/mission3/series-runner/20261008-072101-325940-tests/report.json) | 27/27 통과 |
| [첫 handoff batch](../../artifacts/mission3/series-runner/20261008-072445-263920-tests/report.json) | 30/30 통과, 5.715초 |
| [중첩 registry 수정 batch](../../artifacts/mission3/series-runner/20261008-073303-146375-tests/report.json) | **35/35 통과, 5.618초** |

최종 검사는 최대 3개 실제 fake subprocess, 6개 계획 중 4개 정상 종료/2개 실패의 분모 6 보존, source/config/library 및 동결 profile 변경, 임시 문법 오류, 신규 host source, seed 충돌, manifest 불일치, 출력 덮어쓰기 거부, cutoff/timeout, SIGINT/SIGTERM 자체 중단, 동일 plan 이중 coordinator 거부, argv 공백/메타문자와 CLI roundtrip을 검사했다. Timeout fixture에서 자체 worker/child만 종료하고 별도 process는 살아 있음을 확인했으며 그 PID 증거도 보존했다. 모든 fake 실패·로그·manifest·source snapshot을 남겼다.

첫 handoff 뒤 실제 plan 생성에서 의도적으로 손상한 nested fake manifest가 project registry를 막는 integration gap을 Main이 발견했다. 수정 batch는 명시한 nested fixture 제외, report/hash로 확인한 legacy fixture 제외 및 hash 불일치 거부, provenance 없는 fake 경로의 손상 거부, fixture 옆 실제 손상 manifest 거부, `series-runner`의 실제 worker 집계/손상 거부를 추가 검증했다. 모든 기존 30개 검사도 통과했다.

[실제 project의 읽기 전용 registry 확인](../../artifacts/mission3/series-runner/20261008-073303-146375-tests/actual-root-registry-read-only.json)은 2026-10-08 07:33:52 KST에 정상 완료했다. Provenance로 확인한 nested fixture manifest 24개를 제외하고 실제 manifest 71개, entry/seed 조합 28개를 집계했다. 이는 그 시점의 metadata 수이며 미션 성공 분모나 전체 과거 로그 수가 아니다. 실제 `series-runner` worker는 그 시점에 없었고, 해당 경로의 집계는 regression fixture로 확인했다. 실제 plan/worker를 시작하거나 Main의 실패 출력 경로를 수정하지 않았다.

현재 runner settings 순수 함수만 AST로 추출하여 byte schema를 읽기 전용 비교했다. Runner module 전체를 import하거나 physical plant를 생성하지 않았다. [수정 후 handoff](../../artifacts/mission3/series-runner/20261008-073303-146375-tests/handoff.json)에 세 파일의 최종 hash, 테스트 provenance, own process 확인을 남긴다. 이전 plan은 이전 coordinator hash에 동결되어 있으므로 재사용 대신 새 출력 경로에 새 계획이 필요하다.

## 남은 한계와 읽기 전용 Entry2 제안

Hash polling은 filesystem 원자적 freeze/loaded-library attestation이 아니다. Import와 manifest 생성 사이 race나 poll 사이에 변경됐다 복원된 파일을 놓칠 수 있다. 현재 runner inventory 외 vendor 전체·Python package 전체·asset hash를 보증하지 않으며, 추후 runner schema/inventory 변경 시 재검토해야 한다. Hardware mount·dynamics·RTOS/link timeout·제조사 일치·landing acceptance는 검증 범위 밖이다.

Main이 제공한 기존 Entry2 실패는 상류 `(33,14.1)`까지 긴 이동 후 첫 영상 전에 uncertainty gate에 도달한 사례다. 해당 실패 5개는 Main 기록이며 이 sidecar에서 재실행하지 않았다. 현재 controller/uncertainty를 읽은 변경 후보는 다음 두 가지다.

- Opt-in nearest static station: 진입에서 가까운 직선 투영점의 rear-face viewport·floor-Flow 가능성·net clearance를 먼저 확인하고 접근 경로를 짧게 만든다. Entry2의 가까운 위 직선 후보는 x≈15 부근이다. 차량 도착 시각/속도를 계산하지 않으며 coarse도 fresh pixel·유한 이동 guard를 유지한다. 곡선 후보는 지도 접선만으로 영상 검출을 가정하지 말고 같은 viewport/안전 검사를 통과한 경우에만 사용한다.
- 조기 영상 이후 곡선에서는 fresh 상대 pose/velocity/heading rate로 높이를 유지하며 추적하고, 직선 gyro-bias datum과 descent gate는 직선에서만 사용한다. 현재 covariance는 위치에 직접 correction gain을 주지 않으므로 조기 yaw 보정도 이미 쌓인 `Pxy`를 지우지 않는다. 관측점 도착/마커 획득/곡선 전환을 이유로 covariance를 reset하지 말고, 계속 전파한 `Pxy`·yaw 오차·정지/회수 거리로 geofence 여유와 다음 경로 길이를 판단해야 한다. 필요한 정보가 없으면 재탐색/중단하며 1m threshold를 늘려 해결하지 않는다. 이 covariance는 보정된 확률 보장이 아닌 조건부 S 오차 모델이다.

이는 설계 제안이며 공유 source 수정·효능 검증은 Main 소유다.
