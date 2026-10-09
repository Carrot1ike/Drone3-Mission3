# 미션3 평가 계약 (mission3-evaluation/1)

작성: 독립 검토자, 2026-10-08 KST. 구현: [evaluation.py](../../scripts/mission3/evaluation.py), CLI [evaluate_mission3.py](../../scripts/evaluate_mission3.py), 골든 사례 [test_evaluation.py](../../scripts/mission3/test_evaluation.py).
이 문서를 처음 저장한 시점의 해시 앞 12자리: evaluation.py 731aa757514d, evaluate_mission3.py 34ed0a332020, test_evaluation.py a0950a5e7640. 평가 결과에는 evaluation.py의 전체 SHA-256이 evaluator_sha256으로 기록된다.

## 1. 경계

- 평가기는 **완료된 실행의 로그만** 읽는다. run_manifest.json의 completed가 true가 아니면 판정 불가다.
- 평가 출력은 **실행 중 제어기·추정기·plant의 입력, 설정, 튜닝 신호로 쓰지 않는다.** 모든 출력에 consumer_restriction 문구가 들어간다.
- 실행이 끝난 뒤 평가 결과를 보고 제어를 고치는 것은 허용한다. 이때 변경 근거가 된 실행 ID를 기록하고, 고친 코드·설정은 새 해시가 되므로 새 시리즈와 **새 holdout seed**로만 집계한다. 이전 시도와 실패 분모는 지우지 않는다. truth에서 읽은 plant 고유값(차량 속도, 특정 arc의 직선 끝 시각 등)을 상수로 옮기는 것은 오프라인이라도 금지한다.
- 평가기는 성공을 승인하지 않는다. 평가기 단계의 qualified_mission_success는 최대 pending_review다. 집계는 검증된 독립 리뷰가 있는 경우에만 ledger에서 한다. 평가기와 CLI는 승인된 리뷰를 만들지 못하며, 리뷰 양식의 승인 플래그는 모두 False다.
- 평가기는 controller, estimator, vision, scene, settings, track, firmware 모듈을 import하지 않는다(테스트로 확인).

## 2. 판정 세 개의 분리

| 판정 | 뜻 | pass 조건 |
| --- | --- | --- |
| simulation_contact_success | 물리 시뮬레이션에서 상판 접촉, 모터 off, 3초 지지가 일어났다 | 증거 등급 L2/L3 + physics_contact true, 로그 무결, 충돌 없음, 첫 접촉이 상판, 접촉 뒤 모터 4개 모두 0이 3초 유지, 그 3초 동안 네 발 모두 상판 접촉, 상판이 선언 속도로 계속 이동 |
| manufacturer_profile_matches | 실행에 쓴 기체 파라미터가 모두 제조사 출처다 | 4.7절 필수 항목이 모두 값이 있고 source_class가 manufacturer이며 source_ref가 있다 |
| qualified_mission_success | 미션3 성공 후보 | 위 두 판정 pass + 전체 폐루프 미션 + 카메라 계약 + 제어기 격리 + 하강 직선 + 시간 예산. 평가기 출력은 pending_review까지이고, 7절 리뷰가 검증되면 ledger에서만 집계 |

조건부 S 파라미터(다른 작업자의 독립 MuJoCo 지지 fixture, 예: conditional_contact.py)는 simulation_contact_success를 통과할 수 있다. 그러나 run_kind가 contact_fixture이면 qualified는 contact_fixture_not_mission3로 실패한다. 조건부 파라미터의 source_class는 manufacturer가 아니므로 manufacturer_profile_matches도 실패한다. 이런 결과를 "제조사 일치 성공"이라고 부르지 않는다.

## 3. 상태값

| 상태 | 뜻 |
| --- | --- |
| pass | 해당 판정의 모든 조건을 로그로 확인 |
| fail | 로그로 확인한 위반이 하나 이상 |
| ineligible | 판정에 필요한 로그·필드가 없거나 손상, 또는 실행 미완료. 성공이 아니다 |
| pending_review | qualified 전용. 평가기 조건은 통과했고 독립 리뷰 대기 |

여러 하위 검사를 합칠 때 fail이 하나라도 있으면 fail, 그렇지 않고 ineligible이 있으면 ineligible이다. fail과 ineligible 모두 성공이 아니며 ledger 분모에 남는다. manifest를 신뢰할 수 없으면 manifest에서 파생되는 판정(물리 증거, 폐루프, 프로파일)은 fail이 아니라 ineligible로 둔다.

## 4. 입력 스키마 (실행 디렉터리 하나)

메인의 후속 adapter가 아래 파일을 만든다. 지금까지의 L1 산출물에는 run_manifest.json, motors.jsonl, camera_frames.jsonl이 없으므로 모두 판정 불가다.

### 4.1 run_manifest.json

| 필드 | 형식 | 규칙 |
| --- | --- | --- |
| schema | "mission3-run/1" | 다르면 판정 불가 |
| run_id, author | 문자열 | author는 실행 작성자. 리뷰어와 같으면 자기 승인으로 거부 |
| run_kind | mission3_closed_loop, contact_fixture, optical_geometry | |
| evidence_level | L0–L3 | L2/L3만 물리 접촉 판정 가능 |
| physics_contact | bool | 접촉이 물리 엔진에서 나왔는가 |
| completed | bool | true가 아니면 판정 불가 |
| entry_id | 1–8 또는 null(fixture) | |
| seed, split, series_id | 정수, training 또는 holdout, 문자열 | 6절 집계에 사용 |
| code_sha256 | {경로: SHA-256} | 실행 코드 전체 |
| config_sha256 | 문자열 | 실행 설정 |
| controller_settings_file | 실행 디렉터리 내부 상대 경로, 기본 controller-settings.json | 바깥 경로는 손상으로 처리 |
| mission_timeout_s, handoff_time_s | 수 | handoff_time_s가 없으면 첫 truth 시각 |
| vehicle_nominal_speed_m_s | 수 | plant 전용 값. 평가에만 사용하고 제어 설정에는 없어야 함 |
| descent_phases | 문자열 목록 | 하강 시작으로 보는 단계 이름 |
| controller_in_loop, started_from | bool, "HANDOFF_IN" | 폐루프 미션 판정 |
| aircraft_center_definition | 문자열 | 점수 기준점 정의(예: 기체 body 원점) |
| aircraft_profile | 4.7절 | |
| geom_class_map | 객체 | adapter가 MuJoCo geom을 접촉 분류로 바꾼 표. 리뷰 감사용 |

### 4.2 truth.jsonl (평가 전용 plant 참값, 시각 오름차순, 간격 0.05초 이하)

    {"t": 12.34,
     "aircraft": {"center_world_m": [x, y, z]},
     "pad": {"center_world_m": [x, y, z]},
     "vehicle": {"velocity_world_m_s": [vx, vy, vz], "in_turn": false},
     "contacts": [{"aircraft_part": "foot_1", "other": "pad"}]}

- contacts에는 기체가 포함된 접촉만 넣는다. 기체 부위는 foot_1–foot_4와 그 밖의 이름(body, prop 등), 상대는 pad, floor, marker_board, vehicle_body, net, other다.
- 로그는 HANDOFF_IN부터 시작한다. 이륙 전 바닥 대기는 미션3 로그에 넣지 않는다.

### 4.3 motors.jsonl (간격 0.05초 이하)

    {"t": 12.34, "motor_output": [m1, m2, m3, m4]}

펌웨어가 실제로 모터에 낸 출력이다. 0은 정확히 0이어야 한다.

### 4.4 commands.jsonl

    {"t": 12.34, "phase": "DESCEND"}

### 4.5 camera_frames.jsonl

    {"t": 12.34, "width": 320, "height": 320, "encoding": "mono8", "channels": 1, "resized": false, "color_converted": false}

### 4.6 controller-settings.json

실행 중 제어기가 실제로 받은 설정 그대로다.

### 4.7 aircraft_profile

각 항목은 {"value": ..., "source_class": "manufacturer", "source_ref": "문서명·개정·쪽"} 형식이다. 필수 항목: assembled_mass_kg, assembled_center_of_mass_m, assembled_inertia_kg_m2, pwm_to_thrust_model, motor_response_model, aerodynamic_model, camera_K, camera_distortion, R_body_camera, camera_position_body_m, camera_capture_timestamp, flow_deck_position_body_m, flow_fov_deg, flow_min_height_m, tof_fov_deg, tof_min_distance_m, landing_contact_geometry.

현재 [provided-hardware.json](../../config/provided-hardware.json)과 하드웨어팀 확인에 따르면 이 중 다수가 null 또는 U다. 따라서 지금은 manufacturer_profile_matches가 통과할 수 없다. 사용자 기체 실측값을 manufacturer로 인정할지는 이 계약 버전에서 정하지 않았다. 사용자 결정과 계약 버전 변경이 필요하다. 접촉 해석기 파라미터(마찰, solref 등)는 제조사 사양이 아니므로 필수 항목에서 제외하고, 실행 설정으로 기록해 리뷰한다.

## 5. 판정 규칙

| 검사 | 규칙 |
| --- | --- |
| 로그 무결성 | truth·motor 시각이 증가하고 간격이 0.05초 이하. 줄 형식이 맞음. 위반 시 판정 불가 |
| 첫 접촉 | 기체–상판 접촉이 처음 기록된 truth 표본 |
| 충돌 | 로그 시작부터 유지 종료(모터 off가 없으면 로그 끝)까지 상판 외 접촉이 있으면 실패. 바닥 접촉은 floor_contact로 실패. 발이 아닌 부위가 상판에 닿아도 실패. 유지 종료 뒤의 사건은 성공 판정에 쓰지 않음 |
| 모터 off | 첫 접촉 이후 네 출력이 모두 0이고 그 뒤 3.0초 동안 모든 모터 표본이 0인 가장 이른 시각. 첫 접촉 직전 표본이 이미 모두 0이면 공중 차단 낙하로 보고 실패. 3초를 채우기 전에 로그가 끝나면 판정 불가 |
| 3초 지지 | [모터 off, 모터 off + 3.0초]의 모든 truth 표본에서 foot_1–foot_4가 모두 상판에 닿고 다른 접촉이 없음. truth가 그 끝까지 없으면 판정 불가 |
| 차량 이동 | 첫 접촉부터 유지 종료까지 상판 수평 속력이 선언 속도와 0.02m/s 이내. 상판을 멈추거나 늦춰서 얻은 착륙을 막는 평가 설정값이며 물리 사양이 아님 |
| 하강 직선 | descent_phases에 속한 첫 명령 시각부터 유지 종료까지 truth in_turn이 모두 false. 첫 접촉 시 곡선이면 touchdown_in_turn을 따로 기록. 유지 3초를 포함한 것은 보수 해석이다 |
| 시간 | 유지 종료 - handoff 시각 ≤ mission_timeout_s |
| 카메라 | 모든 프레임이 320×320, mono8, 1채널. resized나 color_converted가 true면 실패 |
| 제어기 격리 | 설정의 모든 깊이에서 vehicle_plant_only, initial_arc 계열, seed, truth, in_turn, plant 키, vehicle 아래의 speed_m_s가 있으면 실패. 정적 검사이며 실행 중 프로세스 격리는 adapter와 리뷰가 확인 |
| 폐루프 미션 | run_kind mission3_closed_loop, entry 1–8, controller_in_loop true, started_from HANDOFF_IN, L2/L3 |

## 6. 위치 점수 (내부 규칙, 대회 공식 점수 아님)

첫 접촉 truth 표본에서 기체 중심(aircraft_center_definition)의 수평 투영과 상판 중심 사이 거리 d를 계산한다. d를 1nm 단위로 반올림한 뒤 아래 표를 적용한다. 각 구간은 아래 경계를 포함하고 위 경계를 포함하지 않으므로, 경계값은 낮은 점수 쪽이다.

| d (m) | 점수 |
| --- | --- |
| 0 ≤ d < 0.01 | 10 |
| 0.01 ≤ d < 0.02 | 9 |
| 0.02 ≤ d < 0.03 | 8 |
| ... | ... |
| 0.08 ≤ d < 0.09 | 2 |
| 0.09 ≤ d < 0.10 | 1 |
| d ≥ 0.10 (내접원 밖, 상판 모서리 약 0.141 포함) | 0 |

예: d = 0.01 → 9, d = 0.0999 → 1, d = 0.10 → 0. 1nm 반올림 때문에 0.0999999999m는 0.100000000m가 되어 0점이다. 점수는 성공 판정과 별개다. 점수가 있어도 접촉·모터·지지 조건을 통과하지 못하면 성공이 아니다. 시간 해상도는 truth 표본 간격(0.05초 이하)이다.

## 7. 독립 리뷰 기록 (mission3-review/1)

    {"schema": "mission3-review/1", "review_id": "...", "run_id": "...",
     "reviewer": {"id": "...", "role": "independent_reviewer"}, "run_author": "...",
     "code_sha256": {...}, "config_sha256": "...", "evaluator_sha256": "...",
     "evaluation_sha256": "...",
     "approved": {"simulation_contact_success": true, "qualified_mission_success": false},
     "findings": [...], "reviewed_at_kst": "..."}

검증 규칙:

- role이 independent_reviewer이고, id가 비어 있지 않으며 evaluator, auto, self, controller, template 같은 자동 식별자가 아니어야 한다.
- 리뷰어 id가 run_author와 같으면 자기 승인으로 거부한다.
- run_id, run_author, code_sha256, config_sha256, evaluator_sha256, evaluation_sha256이 다시 계산한 평가와 모두 일치해야 한다. 로그나 평가가 바뀌면 리뷰는 무효가 된다.
- 평가가 통과하지 않은 판정은 승인할 수 없다. qualified 승인에는 contact 승인이 함께 있어야 한다.
- review-template이 만든 양식(generated_by 표시)은 승인 플래그를 바꿔도 거부한다. 리뷰어가 양식 표시를 지우고 review_id와 reviewed_at_kst를 채워야 한다.
- 리뷰어 신원 자체는 파일만으로 증명할 수 없다. 누가 리뷰어인지는 사용자가 지정하고 작업 기록으로 확인한다.

## 8. 집계와 진입점 진행

ledger는 실행 순서대로 받은 모든 시도를 보존한다. 입력에 들어 있는 counted, success_count 같은 필드는 읽지 않는다. 집계 수를 임의로 지정하는 인자도 없다.

한 시도는 아래를 모두 만족할 때만 집계된다.

1. qualified_mission_success가 pending_review이고, 검증된 독립 리뷰가 qualified를 승인했다.
2. split이 holdout이다.
3. 같은 (entry, seed)가 어떤 training 시도에도 쓰이지 않았다.
4. 같은 (entry, seed)의 첫 holdout 시도다. 재실행은 holdout_seed_reused_after로 세지 않는다.
5. 코드·설정 해시가 그 시리즈의 첫 holdout 시도와 같다. 바뀌면 새 series_id를 써야 한다.
6. 진입점이 현재 프로토콜 단계와 같다. 진입 1에서 집계 5회가 되면 2로 넘어가고, 8까지 마친 뒤에는 무작위 단계다.

진입점별로 전체 시도 수, training·holdout 수, qualified 상태별 수, 집계 수를 낸다. 시도마다 split, seed, 코드·설정 해시, 집계되지 않은 이유(같은 seed의 training 실행 ID 포함)를 남긴다.

진입점 진행에 필요한 근거는 모두 함께 있어야 한다: 제조사 일치 기체 프로파일, 전체 폐루프 미션, 물리 시뮬레이션 접촉, 모터 off, 3초 지지, 독립 리뷰 기록. 현재는 프로파일 null과 L1 증거 때문에 **집계 가능한 시도가 없다**.

## 9. 사용법

프로젝트 루트에서 실행한다. 출력은 기본으로 표준출력이고, --out은 기존 파일을 덮어쓰지 않는다.

    PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/evaluate_mission3.py run RUN_DIR
    PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/evaluate_mission3.py review-template RUN_DIR --reviewer ID
    PYTHONNOUSERSITE=1 src/crazyflie-ros2-sil/.venv/bin/python scripts/evaluate_mission3.py ledger ATTEMPTS.json
    cd scripts && PYTHONNOUSERSITE=1 ../src/crazyflie-ros2-sil/.venv/bin/python -m unittest mission3.test_evaluation

평가기는 표준 라이브러리만 쓴다.

## 10. 골든 사례 (26개, 2026-10-08 통과)

정상 접촉이지만 프로파일이 null이면 qualified 실패. 완전한 제조사 프로파일이어도 pending_review까지만 감. 조건부 S 파라미터는 제조사 불일치. 바닥 접촉, 판 충돌, 세 발 지지, 본체–상판 접촉, 접촉 전 모터 차단 낙하, 재가동 후 계속 회전, 3초 전 로그 종료(판정 불가), 상판 정지, 하강 중 곡선(접촉은 통과·미션은 실패), 시간 초과, 카메라 크기·변환 위반, 제어 설정의 plant 키 3종, 모터 로그 누락, truth 간격, 미완료 실행과 manifest 없는 기존 L1 산출물, L1 광학 기하 실행, contact fixture의 미션 집계 금지, 점수 경계와 모서리, 리뷰 규칙 7종(자기 승인, 자동 식별자, 서명 전 양식, 해시 불일치, 평가 변조, 실패 승인, 양식 기본값), ledger 분모·training seed·중복 seed·순서 위반·코드 변경·입력 집계 필드 무시, 1→8 진행 후 무작위 단계, 제어 모듈 비의존, 평가 해시 안정성.

## 11. 한계

- 접촉 분류는 adapter의 geom 매핑을 믿는다. 매핑 오류는 리뷰가 geom_class_map과 장면 파일로 확인해야 한다.
- 0.05초 간격, 0.02m/s 차량 속력 허용치, 유지 3초의 직선 포함은 평가 설정이다. 바꾸려면 계약 버전을 올린다.
- 해시 결합은 사후 변조를 드러내지만, 실행 전에 만들어진 거짓 로그를 막지는 못한다. 원천 로그 생성 코드의 리뷰가 필요하다.
- 실행 중 프로세스 격리(제어 프로세스가 truth 파일을 열지 않음)는 정적 설정 검사만으로 증명되지 않는다.
