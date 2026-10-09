# 미션3 조건부 접촉·지지 fixture

작성: Codex-Independent-Flight-Reviewer, 2026-10-08 KST. Epicurus의 중단 전 저장 코드·450건 결과·최신 시험 기록을 독립 확인하고 누락 문서를 완성했다. 이 문서는 **민감도 접촉 fixture의 결과이며 비행 제어·미션 착륙·장비 승인·entry 진행 증거가 아니다.**

구현 [conditional_contact.py](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/source-snapshot/scripts/mission3/conditional_contact.py), 설정 [mission3-contact-sensitivity.json](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/source-snapshot/config/mission3-contact-sensitivity.json), [matrix 설정 사본](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/config-snapshot.json), [summary.json](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/summary.json), [450건 results.jsonl](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/results.jsonl), [대표영상 목록 index.json](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/index.json), [독립 재집계](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/fixture-audit.json)을 연결한다. 실제 코드·원 결과는 변경하지 않고 matrix도 재실행하지 않았다.

**P/D/S/U 및 환경 E의 구분.** P는 제조사 공개 사실, D는 그 값·공식 형상에서 계산한 값, S는 민감도 가정, U는 미해결이다. E는 사용자 환경 또는 내부 시험 설정이며 기체 사양이 아니다.

| 값·모델 | 분류 | 근거와 범위 |
| --- | --- | --- |
| Crazyflie2.1 29 g, 92×92×29 mm | P | 제조사 hardware commit 8207400d3e6090ea1dce2b22ddaa25243f718db1의 crazyflie-2_1/datasheet L92–93. 외형은 motor-to-motor/발 포함 |
| AI-deck1.1 4.4 g, Flow deckv2 1.6 g | P | 각 제조사 datasheet L46/L37. 실제 공급 개정은 조건부 |
| 35 g=29+4.4+1.6 g | D | 단품 합. 핀·홀더 포함과 조립 실측 질량은 U |
| box 관성 [2.7139583e-5,2.7139583e-5,4.9373333e-5] kg·m² | D 계산 + S 모델 | Ixx=m(Ly²+Lz²)/12 등. 균일 box 및 scale 0.5/1/2는 제조사 관성이 아님 |
| mesh31 발 좌표 ±31 mm | P 메시 문서→S 접촉 모델 | 공식 crazyflie-simulation commit 7e93752dbc803af2488c1db46bb79b3da55f5d8c의 motor mount origin. 원점이 실제 지지점이라는 가정은 S |
| mesh_lowest33 ±33.4 mm, sphere radius2.8 mm | D→S | 공식 low-poly STL 최저 vertex cluster 도출 선언. STL bounding/contact 표현은 조건부; 실제 deck 조립 발이 아님 |
| envelope46 좌표 ±43.2 mm | S | 외형 반폭46 mm−반경2.8 mm. 명칭46은 sphere 중심 좌표가 아님 |
| CoM 높이14.5/20 mm, hull primitive | S | 외형 중간 및 메시 bbox 근사. 조립 CoM·deck/header·propeller 간섭은 U |
| μ=0.3/0.6/1, solref/solimp | S | 접촉 마찰·강성·감쇠 가설. 실물 상판 재질 사양 아님 |
| 상판20×20 cm·높이30 cm, 판20×20 cm·간격15 cm, .25 m/s | E | 사용자 미션 환경. 판 두께5 mm·상판두께15 mm·차량질량50 kg/servo는 fixture 설정 |
| 모터·센서·공력·추정기 | U / 미모델링 | 이 fixture에는 실제 motor PWM, PID, propeller, IMU/Flow/ToF가 없다 |

제조사 원문을 로컬에서 읽고 아래 사본과 SHA로 남겼다. 제3자 동역학 수치를 보충하지 않았다. 상세 제조사 조사·미공개 항목은 [hardware-sensor-evidence.md](hardware-sensor-evidence.md)를 따른다.

| 제조사 원문 사본 | 전체 SHA-256 |
| --- | --- |
| [index.md](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/manufacturer-source-snapshot/hardware/src/products/crazyflie-2_1/datasheet/index.md) (crazyflie-2_1) | `a3f2231438e66ca7d7976c05adb91cc8e6cabf65e5f9f8ccdfd363f9371ca843` |
| [index.md](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/manufacturer-source-snapshot/hardware/src/products/ai-deck-1_1/datasheet/index.md) (ai-deck-1_1) | `2330659543886d672a2512a917eb11f34f3dede756e773b5428dbf4631a33282` |
| [index.md](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/manufacturer-source-snapshot/hardware/src/products/flow-deck-2/datasheet/index.md) (flow-deck-2) | `66c25d2fb051d98a5fbfa117a6b20285e5452ddd087305d0c81e16adb1c376c2` |
| [meshes.md](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/manufacturer-source-snapshot/crazyflie-simulation/docs/functional_areas/meshes.md) (docs) | `265342264fd98f04e88cee33083de1a215fcbb28a6bc478f966fb7325425f779` |

**실행 방법과 제어 경계.** 기체는 freejoint 수동 강체다. 초기 pose/velocity만 지정하고 이후 qpos/qvel 쓰기, weld/equality, mocap, 기체 actuator가 없다. 차는 slide joint+velocity servo(kv5000, force limit2000 N)로 .25 m/s를 유지한다. full physical run의 prescribed stadium vehicle과는 다른 fixture plant다.

초기 낙하 간격20 mm, nominal vz=−0.1 m/s에 대해 예정 접촉시각은 0.2 s다. precontact에는 world CoM의 이상적 수직 힘 mg=0.34335 N으로 중력을 상쇄한다. t_cut=gap/|vz|+cut_delay를 **시작 전에** 정하고 이때 힘을 0으로 제거한 뒤 3 s를 적분한다. support_N=0은 이상적 지지력 제거이고 실제 모터 출력 정지 측정이 아니다. 실제 contact flag로 cut 시각을 결정하지 않으며 참값은 접촉 분류·법선력 기록에만 사용한다.

기체 초기 world vx는 .25 또는 0 m/s로 상대 vx가 0 또는 −.25 m/s다. nominal 접촉시각의 목표 배치를 얻기 위해 x_init=x_contact+(v_pad−v_init)·t_contact로 초기위치를 미리 보정한다. 이는 알려진 fixture 초기 조건이며 영상으로 차량 속도를 추정한 landing controller가 아니다. baseline vz=−.1, extended vz=−.05/−.2 m/s, delay=0/.05/.1 s, contact softness timeconst=.005/.01/.02 s, damping=.5/1을 시험한다. timestep=.0005 s, implicitfast/pyramidal, 기록=.005 s다. MuJoCo3.13.0, NumPy2.2.6, Python3.10.12로 저장됐다.

**배치 정의.** 예정 접촉시 기체 CoM의 pad 상대 xy이며 외곽 반경·5 mm 여유를 쓴다. near_edge는 발 outer extent를 고려한 조건이어서 foot family마다 좌표가 다르다.

| 배치 | mesh31 xy (m) | envelope46 xy (m) | mesh_lowest33 xy (m, 일부 extended) |
| --- | --- | --- | --- |
| center | (0,0) | (0,0) | (0,0) |
| near_edge_lateral | (0,+.0612) | (0,+.0490) | (0,+.0588) |
| near_edge_rear | (−.0612,0) | (−.0490,0) | (−.0588,0) |
| edge_straddle | (0,+.1000) | (0,+.1000) | 시험 없음 |
| off_pad | (0,+.1538) | (0,+.1660) | 시험 없음 |

**450건의 분모와 분포.** core는 5배치×2 vx×2 foot×3관성×3마찰×2감쇠=360건이다. extended는 3배치×2 vx×2 foot의 12기준 조합에 7개 단일 변수 변형=84건과 mesh_lowest33 중복 제거 후6건으로90건이다. 광범위 무작위 비행 시험이나 전 조합 factorial이 아니므로 이 비율을 실제 착륙 확률로 읽지 않는다. 원 결과 450줄·고유 case_id450개와 summary를 직접 비교해 일치했다.

| 분류 | core360 | extended90 | 합450 |
| --- | ---: | ---: | ---: |
| hold | 180 | 78 | 258 |
| hold_after_slip | 24 | 8 | 32 |
| hull_contact | 6 | 4 | 10 |
| partial_support | 6 | 0 | 6 |
| floor_contact | 144 | 0 | 144 |
| tipover / obstacle_contact / no_pad_support / slip | 0 | 0 | 0 |

success_like는 hold+hold_after_slip=290/450(64.44%)이며 미션 성공 수는0이다. class priority 때문에 floor_contact가 tipover보다 먼저 선택된다. 따라서 tipover 분류0은 큰 기울기 사건0을 뜻하지 않는다. 바닥에 닿은144건 중 success_like는0건이다.

| 배치·초기 world vx | n | hold | after slip | hull | partial | floor | 접촉→settle 상대 이동 (mm) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| center|vx0.25 | 51 | 51 | 0 | 0 | 0 | 0 | 0.000000–0.058102 |
| center|vx0 | 51 | 35 | 16 | 0 | 0 | 0 | 1.254595–22.493628 |
| near_edge_lateral|vx0.25 | 51 | 51 | 0 | 0 | 0 | 0 | 0.000000–0.058102 |
| near_edge_lateral|vx0 | 51 | 35 | 16 | 0 | 0 | 0 | 1.254595–22.493628 |
| near_edge_rear|vx0.25 | 51 | 51 | 0 | 0 | 0 | 0 | 0.000000–0.065203 |
| near_edge_rear|vx0 | 51 | 35 | 0 | 10 | 6 | 0 | 1.254595–26.447391 |
| edge_straddle|vx0.25 | 36 | 0 | 0 | 0 | 0 | 36 | 1.594277–37.083377 |
| edge_straddle|vx0 | 36 | 0 | 0 | 0 | 0 | 36 | 1.561872–48.746290 |
| off_pad|vx0.25 | 36 | 0 | 0 | 0 | 0 | 36 | 없음 |
| off_pad|vx0 | 36 | 0 | 0 | 0 | 0 | 36 | 없음 |

중앙 및 옆 가장자리의 vx0에서는 모두 최종 지지는 얻지만 각각16/51건에서 접촉 후5 mm를 넘는 이동이 발생했다. 후방 가장자리 vx0는35/51만 success_like이며10건 hull/6건 partial이다. 둘 다 pad 밖 배치는 속도 일치 여부에 관계없이 모두 floor_contact다. core와 extended의 분모가 다르므로 위 그룹끼리 단일 원인 효과로 비교하지 않는다.

**분류 임계값과 full mission 차이.** fixture settle 기준은 max(t_cut,first_pad)+.3 s, 최종 평가 창은 마지막 .1 s다. 최종 발 접촉은 이 창의 접촉 비율>0.5인 발 수로 세고 **3발 이상**이면 지지 후보가 된다. 최종 tilt≤10°, 전체 tipover기준45°, CoM pad 내부, 모든 발 중심 margin≥0, 마지막1 s tilt creep≤.02°/s, settle 이후 slip≤5 mm 및 최종 상대속력≤.005 m/s가 hold 조건이다. touchdown→settle 이동>5 mm면 hold_after_slip로 나눈다.

바닥 접촉은 언제든 실패하고 floor→tipover→hull→obstacle→no support→partial→slip→hold_after_slip→hold 순서다. hull에는 PCB/battery/motor column/arm이 상판·기둥·판에 닿는 사건이 포함된다. 발의 board/column 접촉은 obstacle다. **끝에 네 발이 닿아도 발 중심이 edge 밖이거나 tilt creep가 크면 partial일 수 있다.**

[미션 평가 계약](evaluation-contract.md)은 모터 off부터3 s의 모든 truth 표본에서 **네 발 모두**, 모터 네 출력 정확0, 계속 움직이는 상판, 충돌 없음 등을 요구한다. fixture의 end-window3발 majority와 이상적 support 제거는 그 계약과 같지 않다. fixture 분류를 full mission contact 승인으로 복사하지 않는다.

**저장 대표 사례.** 대표영상6건은 위450건과 별도로 재생·저장한 예시이며 새 독립 mission 시도가 아니다. normal_force_N은 fixture에서 실제 mj_contactForce로 기록했고, 아래 peak는 기체의 모든 접촉 합이다. floor 사례의 peak를 pad 지지력으로 해석하지 않는다.

| 사례·원 결과 | 분류 | 최종 접촉 발 | 접촉→settle 이동 | peak 법선력 (N) | 영상 |
| --- | --- | ---: | --- | ---: | --- |
| [R1_center_matched](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R1_center_matched/result.json) | hold | 4 | 0.000367 mm | 0.691483 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R1_center_matched/video.mp4) |
| [R2_center_unmatched_mu03](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R2_center_unmatched_mu03/result.json) | hold_after_slip | 4 | 8.074580 mm | 0.797205 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R2_center_unmatched_mu03/video.mp4) |
| [R3_rear_edge_unmatched_mu03](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R3_rear_edge_unmatched_mu03/result.json) | partial_support | 4 | 8.173054 mm | 0.797205 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R3_rear_edge_unmatched_mu03/video.mp4) |
| [R4_edge_straddle_matched](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R4_edge_straddle_matched/result.json) | floor_contact | 0 | 14.994941 mm | 11.203525 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R4_edge_straddle_matched/video.mp4) |
| [R5_off_pad_matched](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R5_off_pad_matched/result.json) | floor_contact | 0 | 없음 | 12.012840 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R5_off_pad_matched/video.mp4) |
| [R6_center_unmatched_late_cut](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R6_center_unmatched_late_cut/result.json) | hold_after_slip | 4 | 22.460439 mm | 2.442171 | [영상](../../artifacts/mission3/contact-fixtures/20261008-030329-matrix/representative/R6_center_unmatched_late_cut/video.mp4) |

R3는 최종4발이지만 두 발 중심 margin≈−1.023 mm, tilt creep=.07992°/s여서 partial_support다. R4는 약180° 기울어져도 floor 우선 분류다. 영상은 진단 예시이며 실제 aircraft/센서/제어 검증 영상이 아니다.

**16개 시험 기록.** [20261008-032808-tests/report.json](../../artifacts/mission3/contact-fixtures/20261008-032808-tests/report.json)에서 pass16/fail0/skip0, wall1.76 s를 확인했다. 이번 검토는 그 기록과 source hash를 확인했으며 시험이나450건 matrix를 새로 실행하지 않았다. 검증 대상은 구조·질량/관성·발 평면·차 속도·바닥 실패·분류 우선순위·중앙 지지·미끄럼·연속성·고정 cut·결정성·claim label·artifact SHA·미션 기하 mirror·matrix 확장·공식 mesh 근거다. 이 pass는 실기 장착이나 기체 비행에 대한 검증이 아니다.

시험명 원문: structure_no_weld_mocap_teleport_or_aircraft_actuator, mass_and_uniform_box_inertia_with_scale, foot_family_positions_and_foot_plane, pad_moves_at_commanded_speed_including_failures, floor_contact_never_success, classification_priority_and_hull_contact_semantics, center_matched_velocity_holds_nominal, unmatched_velocity_produces_touchdown_slip, no_teleport_trajectory_continuity, cut_schedule_is_fixed_before_simulation_and_ignores_contact, determinism, claim_labels_on_every_result, artifacts_and_source_sha, config_mirrors_mission3_geometry, matrix_expansion, official_mesh_evidence_if_available.

**전체 source/artifact SHA-256.**

| 항목 | SHA-256 |
| --- | --- |
| conditional_contact.py (matrix·tests·review 사본 일치) | `901a12b1b30e799a21d31f1dc927c581b1e5b63beeb45722694d90cebfd3f07b` |
| config 원문 (matrix·tests·review 사본 일치) | `e474dc65bcde48bcaa52e455e7106a36b4ba4c066ddf21a291cf3549bd4d4adf` |
| test_mission3_contact.py (최신16건 시험) | `674d9c73dda32b5472166a72d19991dacc7fb44cb95aa0dddb3821e1a486c817` |
| config-snapshot.json | `35bedc52b0294ccc3c5f541a2f195dec987211b7b6c7a7008887601eccfaf497` |
| results.jsonl | `504e6d1f4beff41ae47f0ae2c409bbee24af6f9daf669aaaeb7e9161d499aa83` |
| summary.json | `3ff851e8319e0531d56c8360fea4d1ad641a549cf456d4402efcaa93e186be02` |
| 최신 tests/report.json | `9331bac6ef3b12af224ef03390b05dd19b49cae45d63b6edc11290b79ec91d2c` |

균일 관성·구형 발·마찰/softness·이상적 support·비행제어 부재·세 발 end-window라는 제한을 그대로 유지한다. 제조사 미공개 조립 값, 실제 센서·추력/응답·propeller/deck 간섭은 U로 남긴다. entry1 승격 집계는0이며 full mission의 별도 [물리 실행 독립 검토](physical-flight-review.md)와 구분한다.
