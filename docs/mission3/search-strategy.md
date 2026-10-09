# 미션3 고정 관측점 탐색

최종 구현 SHA-256: `b17431f56a8e348a7058f1fb10de500f796e4c99d8a33cc3c367548dab82410d`.
worker 수정 범위는 `controller.py`, `test_search.py`, 이 문서다. 설정·추정기·물리 실행기·접촉 판정은 메인 범위로 유지했다.

## 변경 근거와 기본 선택

사용자가 지정한 물리 실패 `physical/20261008-034809-177643-entry1-seed7`은 300초까지 marker pose 0, coarse 단계 0이었다. 영상 판정은 missing 5968, too small 8, DOF 제외 23이었다. 기존 WATCH의 측방 ±2m/56초와 시선 ±35°/14초는 차량 위상을 모르는 상황에서 기회가 올 때 관측 위치와 시선을 바꿀 수 있다. 이 결과를 근거로 탐색시간이나 위치 불확실도 중단 임계값을 늘리지 않고 관측 기하를 바꿨다.

기본 `search_variant`는 **`fixed_point`**다. 알려진 지도에서 진입점에 가까운 직선을 고르고, 그 직선의 진행 접선 기준 상류 쪽 고정 관측점으로 이동한다. 접선은 rear-facing 판을 찾기 위한 지도 사전정보이며, 발견 이후 차량 방향·속도 FF는 계속 영상/센서 추정에서 얻는다.

| 항목 | 아래 직선 | 위 직선 |
| --- | --- | --- |
| 알려진 직선 y | 6.25m | 13.75m |
| 고정 관측점 x/y | (17.00, 5.90)m | (33.00, 14.10)m |
| 기본 높이 | 0.70m | 0.70m |
| 기준 시선 | +X에서 안쪽 +20° | −X에서 안쪽 +20° |
| 시선의 작은 sweep | ±5°, 주기 14초 | ±5°, 주기 14초 |
| WATCH 위치 목표의 주기 이동 | 없음 | 없음 |

x는 직선 시작에서 2m 하류다. 지도 `(cx ± L/2, cy ± radius)`로 계산한다. 기존에는 진입 x를 clamp해 직선 끝 2m 앞에서도 관측했지만, 기본안은 양방향 모두 상류 관측점이다. 먼 진입점에서는 이동 비용이 커지는 한계가 있다. WATCH의 위치 피드백은 자기 추정 위치로 고정점을 유지하며, 물리 기체의 실제 위치가 오차 없이 고정된다고 가정하지 않는다.

## 시야 기하와 SCAN

아래 직선 관측점, 시선 20°, 광학점 높이 0.70m에서 계산한 nominal pinhole 결과다. 320×320, FOV 87°, 코드 0.15m, 마커 중심 상판+.10m를 사용했다. 캘리브레이션·왜곡·렌즈 blur·기체 장착 검증은 포함하지 않는다.

| 관측점에서 패드까지 +X 거리 | 코드 최소 변 | 영상 범위 x | 영상 범위 y |
| --- | --- | --- | --- |
| .50m | 28.08px | 131.7–159.6 | 204.6–239.1 |
| .80m | 21.76px | 154.0–175.7 | 193.5–218.5 |
| 1.20m | 16.72px | 170.8–187.5 | 185.2–203.4 |
| 1.80m | 12.21px | 184.6–196.9 | 178.3–191.3 |

가까운 후보는 기존 18px metric 획득 게이트에 들어가고, 먼 후보는 기존 7px coarse 영역에 들어간다. 15/20/25° 시선과 위 네 거리에서 모든 코드 모서리가 viewport 안에 있음을 시험했다. 같은 .80m 후보에 기존 WATCH의 측방 +2m, 시선 0°를 적용하면 viewport 밖으로 나간다. 이는 정적 후보 기하의 비교이며 차량 도착 시각을 계산한 것이 아니다. 실제 렌더의 검출성은 아래 별도 실행 로그로 확인했다.

기본안은 관측점 도착 후 **SCAN 없이 WATCH로 간다**. 기존 360° SCAN은 15°/s에서 약 24초를 쓴다. 고정점의 높이·측방 거리·판의 단면을 바꾸지 않은 채 한 번 회전하므로, rear-facing 작은 판이 가까이 지나갈 때 유효한 시선이라는 보장이 없다. 이번 고정점 시험은 SCAN을 제거하고 영상 획득을 유지했다.

`search_scan_enabled=True`이면 진단용 SCAN을 다시 켤 수 있다. 이 경우에도 전체 탐색 예산 안에서만 실행한다. 코드가 사전 지도에서 트랙을 영상으로 검출했다고 처리하는 경로는 없다.

## 유한 탐색 계약

1. 영상 발견 전에는 15초 WAIT의 고정 시선·정지 명령을 유지한다. 예정된 차량 도착 시각으로 WAIT를 종료하지 않는다. 기존처럼 실제 영상의 metric pose 또는 두 번 일치한 coarse ID/bearing은 획득 전이를 만들 수 있다.
2. 미발견 15초 후 EXPLORE는 알려진 고정점으로 이동한다. 도착은 자기 추정 위치가 목표 .15m 안에 들어왔는지로 판단한다. 1m 안에서는 시선을 관측 방향으로 미리 정렬한다.
3. WATCH는 고정 위치와 안쪽 20°±5° 시선으로 관측한다. 시선 sweep의 시계는 관측점 도착 후 경과시간이며 차량 위상이나 속도 추정이 아니다.
4. metric pose 없이 끝낼 탐색 예산은 기본 `min(mission_timeout_s, 300초)`다. 공개 `search_timeout_s`가 있으면 `min(mission_timeout_s, search_timeout_s)`를 쓴다. 시각 기준은 미션 인계 t=0이며 WAIT·이동·WATCH·coarse 시간을 모두 포함한다. 만료 사유는 `search_timeout_without_metric_pose`다.
5. metric 획득 이후에도 기존 미션 timeout 360초와 위치 불확실도 임계값 1.0m를 유지한다. 기존 `.1 + .003t`의 약 300초 중단은 이번 worker가 수정하지 않았다. 그 이름이 std여도 검증된 확률값으로 해석하지 않는다.

[대회 조건 문서](../competition-requirements.md)는 세 미션 전체 한 시도의 예산을 7분=420초로 기록한다. 현재 미션3 내부 360초가 전체 예산을 보장하는 것은 아니다. 메인이 앞 단계의 소비 시간을 뺀 남은 예산을 `mission_timeout_s`/`search_timeout_s`로 전달할 수 있다. 이번 구현은 기본 예산을 늘리거나 차량 한 바퀴 시간을 계산해 추가 대기시간을 만들지 않았다.

실행 중 차량 phase·설정 속도·live 곡선 여부·예측 도착시간을 받지 않는다. `controller_settings`의 plant 전용 설정 배제, 전체 설정을 제어기에 넣었을 때 거부하는 계약을 유지했다. 실제 discovery와 FF 준비는 영상에서만 시작된다.

## Coarse 획득 후 제한 접근

고정점만 적용한 P0 시험에서 초기 오차 seed2는 201.72초에 coarse를 얻었다. 명령 로그에서 자기 추정 시선 약 23°와 coarse bearing 약 64°의 차이가 약 40°였다. 15° 이내로 정렬되기 전에 영상이 끊겨 수평 명령이 계속 0이었고, metric pose를 얻지 못했다. 이 진단은 명령·관측 로그에서 했다.

P1은 영상으로 시작한 `COARSE_ACQUIRE`에 유한 접근을 넣었다. 아래 기본 1.50m 접근은 최종 P2에도 남고, 신선한 픽셀 근거가 있는 경우의 확장은 다음 절에 따로 기록했다.

- 관측기는 기존 두 decoded ID와 시간상 일치하는 bearing을 그대로 제공한다. 7px/18px 게이트와 재투영·모호성·혁신 게이트는 바꾸지 않았다.
- 자기 yaw와 마지막 영상 bearing의 차이가 15° 안일 때만 전진한다. 속도는 `min(.40m/s, .8 × 남은 이동거리)`다. 높이는 관측점 높이로 유지한다.
- 접근의 누적 자기 추정 수평 이동거리는 에피소드당 최대 기본 **1.50m**다. 이 값은 소프트웨어 실험값이며 metric range나 기체 안전 사양이 아니다. 속도 .40m/s와 기존 5초 hold의 2m 이동 상한 안에서 거리 제한과 감속을 둔 값이다.
- coarse 영상이 잠시 사라져도 기존 `coarse_loss_hold_s=5` 안에서는 회전 후 제한 접근을 이어간다. 시간은 전파된 상태를 받은 시각 대신 `now-age_s`로 계산한 마지막 영상 시각부터 잰다. 오래된 coarse 상태 전달만으로 hold를 갱신하지 않는다.
- 마지막 영상 이후 5초가 지나면 WATCH로 돌아간다. 새 coarse가 계속 들어와도 동일 에피소드의 이동거리 예산은 리셋하지 않는다. 전체 metric 미획득 시간 제한도 계속 적용한다.

coarse 동안 차량 속도·방향 FF, 마지막 metric 차량 상태, 하강·물리 성공을 생성하지 않는다. `legacy_sweep` 옵션은 비교용 큰 측방/시선 sweep·SCAN·기존 coarse 정지 hold를 유지한다.

## 물리 seed9 불일치와 P2 픽셀 guard

메인의 [물리 seed9 요약](../../artifacts/mission3/physical/20261008-041109-874658-entry1-seed9/adapter-summary.json)은 P1 controller `139c8713…`와 조건부 실제 기체 운동을 사용했다. 이 worker는 **재실행하지 않고 기존 명령·관측 로그를 읽었다**. coarse 33.36→WATCH45.82, coarse50.32→WATCH60.02 뒤 metric pose는 296.76초에 처음 나왔다. LINEUP298.42→300초 `odometry_uncertainty` 종료였다. 채택64, missing5800, too-small99, DOF제외36, 모터 종료 없음, 접촉 표본0이다.

첫 coarse에서 관측점에 아직 도착하지 않은 자기 추정 x≈12.90m였다. 약36.28초의 명령은 [.156,.093,.013]m/s, 37.74초에는 [.040,.020,.003]m/s로 줄었다. 첫 회의 coarse-available 영상32장 최소 변은 7.10–9.57px, 최대 변은 7.74–11.35px였다. 접근거리 1.5m가 소진되는 동안 영상이 더 커져 metric 게이트에 들어가지 않았다. 이 수치는 명령/decoded corners의 사후 분석이며 차량 속도나 위치를 제어기에 되먹이지 않았다. [읽기 전용 검토 JSON](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/physical-seed9-readonly-review.json)에 남겼다.

이 결과 때문에 고정점의 L1 시야 기하 개선을 full physics 획득 보장으로 해석하지 않는다. 현재 yaw/위치 추정과 기체 운동, 관측점 이동 중 획득이 다른 조건이다. 전 미션 시간과 불확실도 임계값을 늘려 해결하지 않는다.

P2는 **영상 손실 hold 5초를 유지**하면서, 1.5m 기본 이동거리 이후에는 다음 신선한 픽셀 근거가 모두 있어야 전진하도록 했다.

- `age_s ≤ .15초`, 유한하고 양수인 `min_side_px ≤ max_side_px`.
- 최소 변이 기존 coarse 게이트 7px 이상, 기존 metric 획득 게이트 18px 미만.
- 최대 변이 소프트웨어 근접 중단 임계값 `coarse_max_side_stop_px=36px` 미만.
- 미션 전체 coarse 누적 자기 추정 이동거리 `coarse_live_path_limit_m=4m` 미만. 새 coarse 에피소드가 시작돼도 총 거리는 리셋하지 않는다.
- 에피소드 이동 시간 `coarse_motion_duration_s=20초` 미만. 이 시간은 이동 허용 상한이며 영상 손실 hold를 늘리는 값이 아니다.

조건이 충족되면 속도는 기존 `.40m/s`와 남은 총 거리의 `.8` gain 중 작은 값으로 제한한다. 조건이 끊기고 기본거리도 소진됐으면 전진 목표0으로 감속한다. 최대 변36px 이상은 최소 변이 작아도 전진 목표0이다. 작은 최소 변만으로 거리/안전을 추론하지 않도록 긴 변도 본다. 모든 감속에는 기존 가속도 제한이 적용되므로 이 픽셀 threshold가 실제 접촉 전 정지를 보장하는 사양은 아니다. 4m·20초·36px는 새 소프트웨어 민감도 값이며 제조사 성능이나 추정 차량 range가 아니다.

`coarse_guard` 명령 필드에 정책, 마지막 영상 나이, 최소/최대 변, 에피소드/총 이동거리, 전진 목표와 상한을 기록한다. 불량 품질이거나 `max_side_px`가 없으면 `base_distance_only`/`extension_requires_fresh_pixels`로 확장을 닫는다. stale 영상이나 반복된 ID만으로 확장하지 않는다.

현재 공유 observer의 coarse 상태는 `min_side_px`를 보내고 **`max_side_px`를 보내지 않는다**. worker가 vision 소유권 밖 파일을 고치지 않았으므로 공유 기본 실행에서는 P2 거리 확장이 비활성이다. 메인이 전달을 연결하려면, 해당 coarse를 만든 새 decoded corners의 네 변 길이 중 최대값만 `max_side_px`로 함께 전달하면 된다. metric 상태 생성이나 PnP 게이트 변경은 필요하지 않다. 실제 활성화 전에는 현재 metadata 없는 경로와 활성 경로의 차이를 구분해야 한다.

## 공개 제어 옵션

기존 JSON을 worker가 수정하지 않았다. `control`의 선택 키로 기본을 명시하거나 비교안을 선택할 수 있다.

| 키 | 기본값/의미 |
| --- | --- |
| `search_variant` | `fixed_point`; 비교용 `legacy_sweep` 허용, 다른 값 거부 |
| `search_scan_enabled` | fixed에서 False, legacy에서 True |
| `search_inward_gaze_deg` | 20° |
| `search_small_gaze_sweep_deg` | 5°; 기존 큰 `search_gaze_sweep_deg=35`를 사용하지 않음 |
| `search_small_gaze_period_s` | 14초 |
| `search_observation_world_height_m` | .70m |
| `search_timeout_s` | 300초; 미션 timeout을 넘지 않음 |
| `coarse_approach_distance_m` | 1.50m; fixed의 영상 시작 접근 한도 |
| `coarse_motion_image_max_age_s` | .15초; 기본거리 이후 확장에 필요한 픽셀 신선도 |
| `coarse_max_side_stop_px` | 36px; 긴 변의 근접 중단 실험값 |
| `coarse_live_path_limit_m` | 4m; 모든 coarse 에피소드의 총 자기 추정 이동 상한 |
| `coarse_motion_duration_s` | 20초; 에피소드 이동 상한. loss hold는 5초 유지 |

## 사전 목록과 렌더 비교

실행 전에 **entry1 seed7, 새 seed8, 초기 오차 주입 seed2**를 training 목록으로 저장했다. baseline/고정점 P0를 각 3회 실행했고, coarse 로그 진단 후 P1·P2에도 같은 3개 training 조건을 각각 사전 등록했다. seed 반복은 코드 조정용이며 holdout이나 독립 성공률 자료가 아니다.

산출물 루트:
`./artifacts/mission3/search/20261008-035834-fixed-watch-training/`

- [최초 사전 목록](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/predeclared-training.json)
- [P1 사전 목록](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/predeclared-coarse-training.json)
- [P2 사전 목록](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/predeclared-pixel-training.json)
- [decoded 픽셀 메타데이터 시험 사전 목록](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/predeclared-decoded-pixel-training.json)
- [전체 비교 JSON](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/comparison.json)
- [스냅샷 검증](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/snapshot-audit.json)

각 실행은 시작 전에 `source-snapshot/`으로 Python 소스와 설정을 복사하고 SHA를 기록했다. 복사한 실행기를 별도 프로세스에서 실행했다. baseline과 P0/P1은 **같은 frozen estimator·scene·vision·track·설정**을 사용했다. Main의 동시 센서/물리 변경이나 old-controller seed8 실행과 섞이지 않는다. 각 `search-run-manifest.json`에 실제 명령, 최초 소스 SHA, 종료코드, wall runtime과 스냅샷 불변 검증이 있다. 최초 logger의 naive KST를 UTC라고 이름 붙인 메타데이터는 원문을 보존하고 offset 있는 KST/UTC 필드로 바로잡았다.

모두 strict focus, last_fix_bridge, finite-patch 광학점 센서, 20fps, video=False다. 광학점 결과이므로 실제 aircraft contact·모터·3초 유지 시험에 넣지 않는다.

| 실행 폴더 | 첫 metric pose | 첫 coarse | 종료 시간/결과 | 채택 영상 |
| --- | --- | --- | --- | --- |
| baseline-entry1-seed7 | 없음 | 없음 | 300.00s / odometry_uncertainty | 0 |
| baseline-entry1-seed8 | 179.56s | 없음 | 204.56s / GEOMETRY_READY | 409 |
| baseline-entry1-seed2-perturbed | 없음 | 206.92s | 300.00s / odometry_uncertainty | 0 |
| fixed-entry1-seed7 (P0) | 103.96s | 없음 | 131.98s / GEOMETRY_READY | 469 |
| fixed-entry1-seed8 (P0) | 180.12s | 없음 | 205.96s / GEOMETRY_READY | 433 |
| fixed-entry1-seed2-perturbed (P0) | 없음 | 201.72s | 300.00s / search_timeout_without_metric_pose | 0 |
| fixed-coarse-entry1-seed7 (P1) | 103.96s | 없음 | 131.98s / GEOMETRY_READY | 469 |
| fixed-coarse-entry1-seed8 (P1) | 180.12s | 없음 | 205.96s / GEOMETRY_READY | 433 |
| fixed-coarse-entry1-seed2-perturbed (P1) | 없음 | 201.72s | 300.00s / search_timeout_without_metric_pose | 0 |
| pixel-guard-entry1-seed7 (P2, metadata 없음) | 103.96s | 없음 | 131.98s / GEOMETRY_READY | 469 |
| pixel-guard-entry1-seed8 (P2, metadata 없음) | 180.12s | 없음 | 205.96s / GEOMETRY_READY | 433 |
| pixel-guard-entry1-seed2-perturbed (P2, metadata 없음) | 없음 | 201.72s | 300.00s / search_timeout_without_metric_pose | 0 |
| decoded-pixel-entry1-seed7 (P2, 격리 metadata) | 103.96s | 없음 | 131.98s / GEOMETRY_READY | 469 |
| decoded-pixel-entry1-seed8 (P2, 격리 metadata) | 180.12s | 없음 | 205.96s / GEOMETRY_READY | 433 |
| decoded-pixel-entry1-seed2-perturbed (P2, 격리 metadata) | 없음 | 201.72s | 300.00s / search_timeout_without_metric_pose | 0 |

기본 비교에서 seed7의 미발견 실패가 pose 획득으로 바뀌었다. seed8은 첫 pose가 .56초 늦었으므로 모든 조건에서 더 빠르다고 주장하지 않는다. 초기 오차 seed2는 P1/P2 metadata 없는 경로에서도 실패다. coarse 노출은 P0의 too-small 27장에서 P1의 44장으로 늘었지만 metric 18px 획득으로 이어지지 않았다. 시간·threshold를 늘려 이 실패를 성공으로 바꾸지 않았다.

metadata 없는 개발 실행 **12회 전부 보존**, GEOMETRY_READY 7회, 실패 5회다. 동일 seed 반복이므로 7/12를 일반 성공률로 쓰지 않는다. baseline 1/3, P0 2/3, P1 2/3, P2 metadata 없음2/3은 이 training 목록의 기하 결과만 뜻한다. 물리 성공 추가 집계는 0이다. 모든 실패의 `evaluation.json`, 원 관측·센서·명령·truth 로그와 stdout을 남겼다.

공유 vision을 고치지 않고 실제 decoded pixels로 P2 활성 경로도 확인하기 위해, artifacts 안의 [격리 wrapper](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/decoded-corner-wrapper.py)에서 **원 Observer.frame을 그대로 실행한 결과의 corners 최대 변만** coarse에 추가했다. 변의 최대값은 새 coarse를 만든 동일 프레임에서 계산하며, metric pose/영상 게이트/plant 참값은 추가하지 않았다. wrapper도 각 실행에 복사하고 최초 SHA를 남겼다. 이 조건을 별도로 표시한 위 마지막 3회는 metadata 없는 12회에 합치면 총 **15회, GEOMETRY_READY 9회, 실패 6회**다. 전체 wall runtime은 10.87–31.08초/실행이었다.

초기 오차 seed2의 활성 경로에서 `fresh_pixel_extension` 명령은134표본 나왔고, coarse 총 자기 추정 이동은1.65784m였다. 마지막 coarse episode는216.46초에 끝났고 metric pose는0이었다. 따라서 guard가 실제 실행되는 것은 확인했지만, 이 추정오차 조건의 획득 실패를 해결했다고 주장하지 않는다. seed7·8은 coarse 경로를 쓰지 않아 metadata 유무 결과가 같다. 메인의 물리 seed9는 중복 실행하지 않았다.

## 테스트·해시·메인 인계

[탐색 테스트](../../scripts/mission3/test_search.py) 23개는 양방향 관측점, 고정 WATCH 위치, 작은 시선, 15초 WAIT, SCAN 기본 제거/선택 켜기, 예산, viewport, native coarse 영상 trigger, metric FF 없음, 유한 coarse 거리·영상 hold, plant 속도/seed 차단을 검사한다. 추가 픽셀 시험은 fresh 영상일 때의 1.5m 이후 접근과 4m 총 거리, stale/최대변 누락/나쁜 품질의 확장 거부, 긴 변 근접 중단, 재획득으로 총 거리 리셋 금지, 20초 이동 상한을 실행한다. 확정한 frozen 소스와 최종 공유 환경에서 각각 탐색23 + terminal21 + 기존 계약8 + 평가26 = **78/78 통과**다.

```bash
cd ./scripts
../src/crazyflie-ros2-sil/.venv/bin/python -m unittest \
  mission3.test_search mission3.test_terminal_recovery \
  mission3.test_contracts mission3.test_evaluation
```

`start_blind_recovery`, `recover_blind`, `output`, `abort`, `enter`는 baseline과 AST가 같다. blind Flow guard와 그 유예·복구 종료 높이도 변경하지 않았다. 메인이 물리 handoff를 .11m로 바꾸는 과정에서 기존 terminal 테스트의 `.06` 고정 기대값이 잠시 실패했다. 이후 공유 테스트가 `.08`보다 높은 handoff를 검사하도록 갱신된 상태에서 72개가 통과했다. 이 worker는 그 테스트나 물리 프로파일을 수정하지 않았다. 원 .06 기대값의 frozen 회귀 fixture에서도 72개 통과 로그를 별도로 남겼다.

P2의 공유78개 시험도 처음에는 모두 통과했지만, 작업 중 공유 vision 변경 후 한 확인에서 **77통과·1오류**였다. `vision.py` coarse `cv2.undistortPoints`가 `self.detector.distortion`을 참조했고, `MarkerYaw`의 실제 속성은 `self.dist`다. 오류 source SHA는 `26660202599c8f342b06af4957c38ac8a1476038e3b5912c3eb317efcb55c353`다. worker 소유권 밖이므로 수정하지 않았다. [공유 오류 로그](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/regression-shared-vision-distortion-error.log)에 남겼다. 이후 메인이 `.dist`로 수정한 공유 vision SHA `d0062b9f82b603198ed1e3143778e93d0f02420c9b549728a3b1cb865ff33668`에서 **78개 모두 재통과**했다.

P2 controller/test_search와 공통 frozen scene/estimator/vision을 고정하고, 메인 갱신 terminal 테스트·.11 프로파일만 별도 fixture로 복사한 `pixel-regression-source/`에서는 [78개 최종 frozen 회귀](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/regression-frozen-pixel-profile.log)가 통과했다. [최종 공유 회귀 로그](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/regression-shared-profile.log)도 따로 남겼다. 고정 소스의 광학 실행15회를 이후 공유 vision·센서의 재실행으로 간주하지 않는다.

| 대상 | SHA-256 |
| --- | --- |
| baseline controller | `24787756cf23f06d122becd99c699f7dccb180a6e07bf41d3672d6609adcb715` |
| P0 controller | `a6bbdc25169dd5c036e56e36c6fb1251776bddc35095449a1d86642c5456b70d` |
| P1 controller | `139c87137e342d475f671e605a8b4c8d85ade34fc50d3173dc249a38cd11401f` |
| 최종 P2 controller | `b17431f56a8e348a7058f1fb10de500f796e4c99d8a33cc3c367548dab82410d` |
| 최종 test_search.py | `13dd3896111f1a0ad9dab5a550cb11ae539a2d60cd4b18949360c19059187d86` |
| 실행 스냅샷 terminal 테스트 | `a40cfed6a1ebb5a46d18638f4b61e1d76c4620bb227330dc866f042c8e7c22e1` |
| 메인 갱신 후 공유 terminal 테스트 | `a9fb8091134f1a470f7e297b53cc1d7676c88005286cbf322fde8a5dd0258f1d` |

[최종 검증 기록](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/final-verification.json), [공유 환경 회귀 로그](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/regression-shared-profile.log), [원 프로파일 회귀 로그](../../artifacts/mission3/search/20261008-035834-fixed-watch-training/regression-original-profile.log)를 함께 읽을 수 있다.

메인에는 `fixed_point` 기본을 유지하고 coarse에 `max_side_px`를 전달한 뒤 물리 획득을 검증할 것을 권한다. 초기 오차 seed2를 위해서는 자기 yaw/위치 추정과 획득 시야의 오차 처리가 다음 과제다. 현재 bounded coarse 접근만으로 해결되지 않았고, 근거리 Flow guard 완화나 장기 시간 연장으로 보상하지 않는다. 메인의 .11m handoff와 마지막 IMU/force-residual 접촉 판단은 별도 검증해야 한다. 이 worker는 2cm 착륙 상대 오차를 분석하지 않았다.

## 메인 통합 후 기록 (04:53 KST)

worker 최종 P2 controller 소유권을 반환한 뒤 main이 `vision.py` coarse에 동일 decoded corners의 최대 변을 전달했다. fresh-pixel extension은 이후 새 hash/run부터 활성이고 위15회 스냅샷에 소급하지 않는다. 실제 body dynamics·initial perturb의 `044603-entry1-seed2`는 coarse3프레임 이후 metric0/search_timeout으로 실패했다. 이 실행은 Brown IPPE만 보정하고 constrained refinement의 D가 누락된 과거 partial-optics hash다.

main은 별도 `mission3-coarse-turn-sensitivity.json`에서 새로운 픽셀 근거가 신선하고 min7≤side<18·maxside<36이며 camera bearing error가40° 이내일 때, yaw를 맞추는 동안 .15m/s의 제한된 bearing 방향 병진을 허용한다. yaw error<15° 뒤에는 기존 .40m/s 탐색 한도를 사용한다. 이 변경은 속도가 빠른 차량에 정렬 회전만 하다가 희소한 ID 노출을 잃을 수 있다는 가설을 시험하는 S 정책이다. metric 차량 상태·속도 FF·하강을 coarse로 생성하지 않고 총4m/20s/영상 freshness/pixel근접 guard를 유지한다. 기본 경로의 회전 전 병진 조건은15°로 보존하며, 새 물리 시험 성능으로 채택 여부를 결정한다.
