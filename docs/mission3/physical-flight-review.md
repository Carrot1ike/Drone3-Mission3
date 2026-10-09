# 미션3 첫 물리 실행 독립 검토

검토자: **Codex-Independent-Flight-Reviewer**, independent_reviewer. 검토 시각: 2026-10-08T03:38:48.592612+09:00.
실행 작성자: main-flight-adapter. 대상은 [20261008-032640-670598-entry1-seed1](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/run_manifest.json) 한 건이다. 다른 작업자의 회복 제어·finite patch 수정 및 이후 실행은 이 승인 범위 밖이다.

**결론: 조건부 L2 simulation_contact_success 승인, qualified_mission_success=false, 진입 승격용 성공 0.**
[evaluation 계약 7절](evaluation-contract.md)에 따른 [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/signedreview.json)을 직접 작성했다. 양식 자동승인이 아니며 [검증 결과](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/review-validation.json)는 valid=true, contact=true, qualified=false다. 제조사 프로파일 17항목 중 S 또는 null/U가 남고 이 실행은 training/entry1/seed1이다. entry1을 유지한다. 전체 시도·실패 분모는 변경하지 않았다.

**원 로그 확인.** [독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/reevaluation.json)는 [원 offline-evaluation.json](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/offline-evaluation.json)과 모든 필드가 일치한다. [직접 재계산 지표](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/log-audit.json)와 [접촉 구간 원 표본 사본](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/contact-log-samples.json)을 함께 보존했다.

| 항목 | 직접 확인한 값 |
| --- | --- |
| 최초 상판 접촉 | 166.44000000043067 s, 첫 표본부터 foot_1–foot_4 |
| 모터 off | 167.02000000043344 s, 첫 접촉 이후 0.58 s |
| 계약상 유지 종료 / 로그 종료 | 170.02000000043344 / 170.04000000044786 s |
| 네 발 접촉 | 유지 구간 151/151 표본, 최초 접촉부터 종료 181/181 표본 |
| 모터 정지 | 유지 구간 3,001/3,001 표본의 네 PWM·motor_output·static target 모두 정확히 0 |
| 접촉 직전 모터 | 166.439 s: [40119,40188,40188,40113]; 공중 모터 차단 낙하가 아님 |
| 충돌 | 전체 truth에는 네 foot→pad만 존재. floor·hull·board·vehicle 접촉 없음 |
| 차량 속력 | 전체 참값 0.24999999999999997–0.25 m/s; 상판 위치 차분도 약 0.25 m/s |
| 하강 직선 | 160.78 s BLIND_DESCEND부터 유지 종료까지 in_turn=false |
| 점수 | 기체 body 원점 수평오차 0.021027632086152104 m → 내부 8점. 대회 공식 점수 아님 |
| 유지 중 상대 미끄럼 | off시 상대위치 대비 최대 0.000010221363 m; 최초 접촉 대비 종료까지 최대 0.000035066690 m |
| 유지 중 상대 수평속력 | 5.98e-8–3.44e-6 m/s |
| 기울기 | truth 표본의 실제 법선 기울기 최대 3.474885°; 유지 구간 0.000007245–0.000038854° |
| 해상도 | truth·센서 20 ms, 모터 1 ms; 1 kHz 순간 접촉 유지는 원 로그만으로 입증하지 않음 |

adapter-summary의 max_tilt=3.477373°는 매 1 ms의 roll/pitch 벡터 노름이고 위 값은 20 ms truth 표본의 acos(cos roll·cos pitch)다. 다른 정의와 표본률을 혼동하지 않는다.

**정지 출력과 실제 추력·법선력.** 모터 PWM/static target가 0이어도 plant의 20 ms 1차 응답 때문에 actual force는 off 직후 합 0.339041305 N이다. 0.4 s 후 6.988e-10 N까지 감소한다. 실제 힘이 off 순간 정확히 0이라고 쓰지 않는다. 유지 중 CoM 높이는 상판 위 13.561–13.568 mm, contact distance는 −0.939~−0.932 mm로, 선언된 soft contact의 침투가 있다.

원 [truth.jsonl](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/truth.jsonl)에는 geom ID·접촉 거리만 있고 **접촉 법선력 원 기록은 없다**. 보조 [법선력 재구성 v2](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/normal-force-reconstruction-v2.json)는 저장 [physical-scene.xml](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/physical-scene.xml)에 원 pose·선속도·actual motor force를 넣고 별도 mj_forward/mj_contactForce로 계산했다. 각 발 법선력 0.000947258–0.085837571 N, 합 0.004308724–0.343350037 N으로 모두 양수이며 off+0.4 s 이후 약 mg=0.34335 N이다. 각속도는 pose 차분 추정이고 solver warm-start가 없으므로 당시 force 로그를 대체하지 않는다. 첫 diagnostic의 기본 torque 상수는 v2에서 실제 legacy 상수로 바로잡았으며 승인 근거로는 v2만 사용한다. 계약의 접촉·출력·네 발 표본 지지가 독립적으로 통과하므로 이 증거 공백을 명시하고 조건부 접촉만 승인한다.

**소스 available/missing.** 메인이 후속 보존한 [source-snapshot-audit.json](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot-audit.json)을 읽고 [독립 SHA 검증](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/run-source-snapshot-validation.json)을 만들었다. manifest의 18개 소스 중 17개 matching 사본을 확인했다. controller.py의 원 SHA f884b851… 사본은 Popper 수정 전 버전이며 실제 검토는 그 사본으로 했다. 최신 controller를 과거 코드로 취급하지 않았다.

scene.py 원 SHA f69adb7a…는 **missing**이다. 검토 시점의 수정본 SHA는 5ab969d4…이며 [검토 시점 스냅샷 목록](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/source-and-log-hashes.json)에 mismatch로 기록했다. 사용자 인계에 따르면 변경은 광학 sensor helper이고 물리 adapter는 scene.build 및 좌표 상수만 참조한다. physical step/deck는 matching physical_scene.py에 자체 구현되어 있다. 저장 XML로 이번 기하를 감사했으나 원 scene 전체와 build 함수의 바이트 동일성은 소급 증명하지 않는다.

| 실행 소스 / 보존 사본 | 원 manifest 전체 SHA-256 | 확보 상태 |
| --- | --- | --- |
| [scripts/run_mission3_physical.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/run_mission3_physical.py) | `670cbdac89771223b05bbbef159d0aed2b135075200e8edbc73fde5935a5e30b` | available, matching |
| [scripts/mission3/controller.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/controller.py) | `f884b851baca2bcbfc7cc1750c3396be3d42d2126faa0bc7d5a5f45a13af0602` | available, matching |
| [scripts/mission3/physical_scene.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/physical_scene.py) | `5781877f524a3888e93e1765048681c68e1e7be24e6ed9e168963942fc732c7c` | available, matching |
| [scripts/mission3/physical_estimator.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/physical_estimator.py) | `172993a74f8422ebab67aa74d2b2749e1b20bb166ade8a03e7a3efd929c30286` | available, matching |
| [scripts/mission3/vision.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/vision.py) | `7179e21c05b3d47639a42d3189a4839bca0c1ac8b7223af6a885ca3eba8d1147` | available, matching |
| [scripts/mission3/estimator.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/estimator.py) | `2f357c4b785a7d6c7cff383f8b096679038d51b02b8e23405950bc2f780a490f` | available, matching |
| [scripts/mission3/flight_controller.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/flight_controller.py) | `46b7fd18a08e58b1212beb48edc61cd9cff4404b47cd5a31d2f6ad9ecb86af6e` | available, matching |
| [scripts/mission3/flight_controller_host.c](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/flight_controller_host.c) | `af543cdc238fd5b2d565164024a8b0642b003b3c8ab79bd115140d8194d409d4` | available, matching |
| [scripts/mission3/evaluation.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/evaluation.py) | `731aa757514d6028af1743147630804896e42f7e030e7cf13e9e7d131da00da0` | available, matching |
| [scripts/mission3/conditional_contact.py](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/mission3/conditional_contact.py) | `901a12b1b30e799a21d31f1dc927c581b1e5b63beeb45722694d90cebfd3f07b` | available, matching |
| scripts/mission3/scene.py | `f69adb7ad231fe2c9f3fced1a1719b6909471a60a9751a43c857d6a110e4b0df` | missing; 현재 수정본은 원본 아님 |

**격리·상태 쓰기.** matching physical_scene.py에서 aircraft freejoint의 초기 qvel 설정 외에 기체 qpos/qvel 쓰기는 없다. motor forces와 torque를 xfrc_applied에 넣고 MuJoCo가 기체를 적분한다. vehicle freejoint만 매 step 전후 track.pose(initial_arc+0.25t)로 연속 지정하며, 이는 허용된 차량 운동이고 기체를 붙이는 weld/teleport는 없다. XML에도 equality/mocap/aircraft actuator가 없다.

공개 controller_settings에는 vehicle_plant_only·initial_arc·seed·live truth가 없다. 영상 관측→자기 odometry→차량 위치 회귀에서 방향·속도를 얻고, 정적 track prior는 추정 보정에만 사용한다. lane pixel 인식으로 꾸미지 않았다. runner의 contact truth는 로그·floor 시험중단에만 사용되고 기울기 참값도 시험중단에만 사용된다. 성공 단계나 motor off 판단에 contact flag가 전달되지 않는다. off 결정은 낮은 ToF 중앙값, 추정 |vz|<0.015, moving_surface_candidate, 하강 지속 및 0.4 s evidence에 의존한다. 이것은 조건부 높이·센서 가설이며 네 발 접촉을 onboard에서 식별한 증거가 아니다.

**물리·센서 가설의 분류.** P=제조사 공개 사실, C=공식 코드, D=계산, S=민감도 가정, U=미해결; 환경·실험 값은 E로 표시한다.

| 요소 | 분류와 감사 결과 |
| --- | --- |
| 29 g 본체+4.4 g AI-deck 1.1+1.6 g Flow v2 | 단품은 P, 합 35 g는 D; 핀/홀더 포함·실제 배정은 U, 조립 질량은 조건부 |
| 관성·CoM·접촉 | 균일 box 관성·CoM 14.5 mm·sphere 발·μ=0.6·softness는 S; 조립 실측 동역학 아님 |
| T_BC 및 t_BC | camera offset [0.020,0,0.015] m, rpy [0,0,0]는 S; 공식 정량 장착 변환은 U. vision은 capture-time 추정 roll/pitch/yaw와 이 변환을 사용 |
| Flow/ToF 전달 | flow half angle 21°, ToF 7.5°, 5×5 ray, area 평균 angular flow·최단 ToF, 잡음은 S; 실제 FOV/min height는 U |
| ray patch 내부 참값 | sensor 생성 plant에서만 pad ray hit·표면 속도를 계산. packet에 표면 ID를 내보내지 않음 |
| 실제 센서 한계 | texture/correlation·반사율·marker board 가림·최소거리·근접 saturation 미모델링. plant에는 sensor offset 회전속도를 넣지만 estimator에는 lever-arm 속도 항을 명시적으로 빼는 보정이 없음 |
| 영상 | 320×320 mono8 입력을 Observer가 검증. render RGB를 luminance로 만드는 합성 카메라이며 native 실기 획득·렌즈 보정을 검증한 결과 아님 |
| strict focus | 가상 depth<0.30 m 프레임 제외. 실제 blur·근거리 검출성·capture timestamp 보장은 아님 |
| 추정 | 공식 Kalman process noise 1 kHz/predict 100 Hz로 roll/pitch를 얻음. xyz/yaw·surface fusion은 프로젝트 SelfMotion. stock estimator 전체 정확도 승인 아님 |

**공식 PID·배터리 경로.** [빌드 감사](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/build-audit.json)에서 공식 펌웨어 commit f6e0f0a3b526861328caf3296a7ecf00bb915d39의 PID/position/attitude/mixer/pid/filter/num C SHA가 build source-manifest와 모두 일치했다. motorsCompensateBatteryVoltage 본문 추출이 원문과 정확히 같고 body SHA는 `91950373ae03cf003e456fd7c60b40edcfbc82db9acecce59ce1816923718112`다. body_modified=false라는 선언만 믿지 않고 직접 비교했다.

host는 modeVelocity, world velocity, degree 단위 state/gyro로 공식 PID에 넣는다. lower_state의 pitch 부호 반전, 공식 rate PID의 −gyro.y, control.yaw 부호 반전, CF2 M1 front-right→M4 front-left 및 yaw torque [−,+,−,+] 경로가 함께 일관된다. legacy mixer는 M1=T−r+p+y, M2=T−r−p−y, M3=T+r−p+y, M4=T+r+p−y다. battery filter는 4.2 V 초기값·0.01 계수이고 mixer→정확한 battery compensation→cap 순서로 실행한다.

legacy propeller C cubic fit은 v=4.2·PWM/65535에 적용한다. 선택 헤더의 THRUST2TORQUE=0.007350862856566459 m이며 default 헤더 상수와 혼동하지 않았다. 이 fit은 C 소프트웨어 모델이고 모터 응답 20 ms·gain1·고정 4.2 V·공력0·ground effect 미모델링은 S다. [모터 모델 교차 확인](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/motor-model-audit.json)의 2,020개 표본은 캡처 library static force와 로그가 정확히 일치한다. 당시 .so 및 platform headers/build scripts/build manifest 해시는 run manifest에 묶이지 않아 바이너리 신원은 U로 남는다. [캡처 빌드 manifest](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/source-snapshot/build/mission3-flight-controller/source-manifest.json)는 사후 캡처이지 당시 binary attestation이 아니다.

**후속 작업의 주요 위험.** 원 실행의 조건부 접촉 승인을 취소할 관측 위반은 없지만 아래 항목은 새 실행·제조사 일치 판단 전에 분리 검증해야 한다.

- runner tick%20==1에서 Flow/ToF를 동시에 호출하므로 둘 다 실제 50 Hz다. experiment simulation 설정의 Flow100/ToF40 Hz와 다르다.
- motor off 후 151개 센서 표본 중 moving_surface_candidate 2개, ambiguous 149개다. MOTOR_OFF_HOLD는 상태 악화에 대응하지 않고 시간을 채운다. 이번 지지는 사후 truth로 확인했으며 sensor 기반 지속 지지 보장은 없다.
- 마지막 capture는 160.00 s로 off시 7.02 s, 종료시 10.04 s 오래됐다. 12 s bridge는 경험적 예측 제한이고 곡선·노이즈·위치불확실성 아래의 보장이 아니다. 실제 초기 entry 오차 주입은 false다.
- 원 controller LOWER_ALIGN는 .50 m를 직접 사용하여 public lower_align_world_height_m=.40 m가 반영되지 않는다. 회복 분기 개선은 담당 다른 작업자의 새 hash/run으로 검토해야 한다.
- propeller/deck 형상과 차량 artwork·그물벽의 충돌 모델은 완전하지 않다. 로그의 충돌 없음은 모델에 있는 충돌체 범위 안의 결론이다.

원 평가 **파일 바이트** SHA-256: `ee3cc373a05cf45367ca01d3d3319c92dece80faa3d88bcd8951ec36443625d1`.
계약 7절의 **정렬 JSON 평가 digest**: `1c6557b16b49873378af0000b9444a1e68dfeb2459da027d2070d7b38c4b3ed5`. 둘을 별도 필드로 signedreview에 넣었다. 원 결과·truth·평가기·시뮬레이션 코드는 수정하지 않았다.

## 첫 실행의 마지막 영상부터 접촉까지: 2cm 오차 증분 분석

2026-10-08 추가 독립 분석이다. 대상은 위 seed1 한 건이며, 원 matching source-snapshot과 원 로그만 읽었다. 최신 sensor cadence·회복·HOLD 수정 코드를 과거 실행에 적용하지 않았다. [분석 코드](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/analyze.py), [수치·입력 전체 SHA](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/findings.json), [553표본 CSV](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/timeline.csv), [구간 CSV](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/segments.csv), [마지막 pose CSV](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/last-frame-pose.csv)를 별도 저장했다. 분석 코드 SHA-256은 `2d90708768b8636248946c249e2a0758992f5ba3d4e381204db49bef281d8d35`다. 원 truth·결과·온라인 상수는 변경하지 않았다.

**거리와 추정오차의 정의.** L=x_aircraft−x_pad, L̂=x̂_aircraft−x̂_pad, E=L̂−L이다. 기체·상판 각각의 세계좌표 편향 g_aircraft, g_pad에 대해 E=g_aircraft−g_pad가 정확히 성립한다. 같은 좌표 이동을 두 상태에 더하면 순간 상대위치는 보존된다. 영상 anchor는 own_state를 로그에 복사한 뒤 적용되므로 해당 tick의 제어에 쓰인 위치는 vehicle.pad_world−relative_pad_world로 복원했다. 단순히 commands.own의 사전 anchor 위치와 사후 vehicle 위치를 빼면 가짜 불연속이 생긴다.

| 시각·단계 | 참 L x/y (mm) | 추정 L̂ x/y (mm) | E x/y (mm) | 참 수평거리 (mm) |
| --- | --- | --- | --- | --- |
| 160.00 마지막 capture | −75.006 / +8.995 | −74.058 / +7.019 | +0.948 / −1.975 | 75.543 |
| 160.06 마지막 receipt | −68.305 / +8.575 | −67.723 / +6.156 | +0.582 / −2.419 | 68.842 |
| 160.78 BLIND_DESCEND | −22.520 / +8.276 | −20.105 / +2.279 | +2.414 / −5.996 | 23.992 |
| 162.00 하강 | −5.964 / +10.323 | −2.920 / +1.022 | +3.044 / −9.300 | 11.922 |
| 164.78 TOUCHDOWN_DESCEND | −7.081 / +16.895 | +0.275 / +1.510 | +7.356 / −15.385 | 18.319 |
| 166.44 첫 접촉 | −10.676 / +18.116 | −0.106 / +1.059 | +10.570 / −17.056 | 21.028 |

마지막 영상 때의 75.5mm는 아직 접근 중인 실제 간격이다. 이후 수평거리는 162.00s의 11.9mm까지 줄었다가 접촉 때 21.0mm로 커졌다. 제어기가 보는 간격은 거의 0으로 수렴하지만 참 y 간격은 계속 증가한다. 접촉 때 세계좌표 편향은 기체 [+209.162,−198.352]mm, 상판 [+198.592,−181.296]mm이고 그 차이가 E다. 약 20cm 공통 drift를 20cm 착륙오차로 해석하면 안 된다.

![첫 실행 상대오차와 공통 gauge](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/terminal-errors.png)

**구간 증분.** 마지막 receipt→BLIND_DESCEND의 0.72s에 E가 [+1.832,−3.577]mm, 다음 1.22s에 [+0.630,−3.304]mm, 162.00→164.78의 2.78s에 [+4.312,−6.085]mm, 마지막 1.66s에 [+3.214,−1.671]mm 증가했다. 마지막 receipt 이후 합은 [+9.988,−14.637]mm다. finite patch 혼합으로 Flow가 막혔다가 160.68s부터 융합되며, 첫 구간은 6/37 표본만 융합, 이후 세 구간은 모든 표본이 융합된다. 후기 오차 증가를 계속된 Flow 차단으로 설명할 수 없다.

**원인 후보의 구분.** 아래 분해는 원 측정·gate·참 궤적을 고정한 추정기 속도 재귀식의 대수적 분해다. 새로운 비행이나 폐루프 개입 효과를 예측하는 시험이 아니다. v_error,i=(1−α)v_error,i−1+(1−α)(a_est−a_true)Δt+α(flow_transfer_error+FF_bias)를 직접 재계산했으며 로그와의 최대 x/y 속도 잔차는 1.53e−16m/s다. 원 코드의 α=.08(pad), .25(floor), 0(blocked)를 사용했다.

| 마지막 receipt→첫 접촉의 E 증분 항 | x (mm) | y (mm) | 해석 범위 |
| --- | --- | --- | --- |
| 시작 시 자기 속도오차의 지속 | +29.616 | −24.822 | FF 항과 강하게 상쇄됨 |
| roll/pitch 가속도 투영 | +9.399 | −8.200 | 추정 자세와 truth 자세를 사후 비교한 투영 차이 |
| 나머지 가속도 잔차 | −1.655 | −0.093 | 잡음·시간평균·yaw·모델 등의 미분리 잔차 |
| Flow 전달 잔차 | −0.284 | −3.493 | raw Flow×보정 range의 상대속도와 참 상대속도의 차이 |
| FF 속도오차의 순 상대 항 | −25.988 | +21.898 | 자기 Flow 융합 경로와 상판 예측 경로를 함께 계산 |
| 위치 적분·나머지 잔차 | −1.099 | +0.072 | 이산 적분 등 |
| 합 | +9.988 | −14.637 | 원 E 증분과 일치 |

- **영상 pose 기준:** 마지막 네 corner로 원 ideal K/장착 변환을 사용해 독립 yaw+translation fit을 재현했다. raw relative pose 수평오차는 2.200mm, 재투영 RMSE .13909px다. 같은 corner에 truth roll/pitch만 사후 대입하면 1.896mm, yaw도 대입하면 1.707mm다. 마지막 영상 자체의 오차만으로 21mm를 설명할 수 없다. 누락된 원 scene.py 대신 저장 XML의 camera 변환과 명시적 marker rear-corner 관례를 썼으며, 실기 calibration 결론은 내리지 않는다.
- **body tilt/IMU drift:** 하강 중 roll 약 .028–.031°, pitch 약 .030–.043° 추정 오차는 중력 투영에 약 [+5.5~6.0,−5.1]mm/s² 차이를 만든다. 영상 pose에는 수백 μm 수준 영향이지만 수초 가속도 적분에는 mm 단위 영향이 보인다. 이 값은 사후 진단이며 truth 자세 보정 상수를 온라인에 넣는 근거가 아니다.
- **finite patch/Flow:** 초기 혼합 gate는 실제로 작동했고 뒤에는 contained patch로 융합된다. 전달 잔차 항의 y −3.49mm가 남는다. 원 로그는 range 잡음·ray 평균·offset 회전속도·근거리 모델을 완전히 분리하기에 부족하므로 특정 하나를 확정 원인으로 부르지 않는다. 특히 제조사 near Flow/ToF 정확도 검증이 없다.
- **FF 속도와 gauge anchor:** 마지막 FF는 [.281311,−.026383]m/s, truth는 [.25,0]이다. 상판만 적분하면 [+199.765,−168.323]mm 오차지만 같은 FF가 자기 Flow 융합에도 들어가 대부분 상쇄된다. 시작 속도+FF 항의 순합은 [+3.628,−2.924]mm다. FF 오차를 독립적인 20cm 상대오차로 더하면 이중 계산이다.

마지막 20개 영상의 159.06–160.00s 회귀는 저장 raw pad 위치와 기록된 공통 anchor 이동만으로 재현했다. 마지막 회귀 [.283114,−.026760]m/s의 truth 대비 편향은 자기 capture gauge 변화 [+ .033285,− .001929]m/s, raw relative pose 변화 [− .000172,+ .000218]m/s, 과거 회귀 표본에 적용된 anchor 이동 [0,− .025049]m/s의 합이며 폐합 잔차 1.11e−15m/s다. 따라서 x 속도 편향에는 자기 odometry gauge 변화, y에는 시간별 anchor 이력의 영향이 크다. 과거 표본 전체를 같은 δ로 이동하는 순간은 상대위치나 회귀 속도를 바꾸지 않지만, 서로 다른 capture gauge·시각의 표본이 쌓이면 회귀에 의존성이 남는다. 마지막 receipt 160.06s 뒤 anchor는 0회이므로 이후 2cm 상대오차 증가는 새로운 anchor 점프가 원인은 아니다. 구현 개선 효과는 새 hash와 별도 시험으로 확인해야 한다.

## 첫 실행 HOLD ambiguous 원인과 최소 센서 관찰 설계

[원 runner 116행](../../artifacts/mission3/physical/20261008-032640-670598-entry1-seed1/source-snapshot/scripts/run_mission3_physical.py:116)의 terminal phase 집합에는 MOTOR_OFF_HOLD가 없다. 167.02s에는 이전 TOUCHDOWN_DESCEND 때문에 hint가 남고, 167.04s 센서는 그 hint를 한 번 더 쓴 뒤 runner가 None으로 바꾼다. **167.06s부터 hint 없음과 ambiguous가 정확히 일치**한다. HOLD 151표본의 영상 age 7.02–10.02s는 모두 12s 한도 안이고 raw range는 모두 양수, 유효 range 151/151, 최대 상판 range residual .022641m는 기존 .07m gate 안이다. 한도 만료가 원인이 아니다.

원 궤적·측정을 고정하고 이전 추정 상태의 hint만 계속 존재한다고 놓은 표면 조건식은 151/151 moving candidate로 분류된다. 이는 조건식 진단이며 새 추정기·plant 재실행이나 지속 지지 검증이 아니다. 실제 네 발 접촉은 그대로 유지되지만 hint 중단 후 Flow 미융합으로 E는 유지 종료 때 [+40.629,−37.922]mm까지 커진다.

![첫 실행 HOLD hint와 예측 제한](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/hold-evidence.png)

최소 설계 제안은 구현 변경 없이 다음 관찰 계약으로 한정한다.

1. flight phase와 별개인 disarmed hold evidence 상태에 마지막 유효 상판 상대 pose·불확실성·원 capture time을 보존하고, motor off 뒤에도 제한 시간 내 표면 분류를 지속한다. 새 영상 없이 capture time을 갱신하거나 불확실성을 동결하지 않는다. 예측 여유는 남은 3s 유지와 직선 여유를 포함해야 한다.
2. Flow/ToF stamp·sequence·new flag를 기록하고 각 새 range를 한 번만 높이 융합한다. 새 측정의 freshness와 반복 예측을 구분한다. IMU tilt/gyro·specific force의 지지/자유낙하 차이, 새 유효 range의 중앙값·변화율, 검증된 거리·품질 범위의 relative Flow를 함께 관찰한다. 범위 밖/무효 Flow를 0속도 증거로 처리하지 않는다.
3. 불명확 상태와 안정 상태를 명시적으로 분리한다. .25s 유예는 순간 잡음 허용 규칙이며 unknown을 pass로 바꾸는 규칙이 아니다. 표본별 실패 이유·age·budget·freshness와 유지 시간을 남기고, 해당 evidence가 불가능하면 sensor HOLD unconfirmed로 보고한다. 모터 off 뒤 자동 재시동 안전 정책은 별도 설계 대상이다.
4. 현재 IMU/ToF/Flow는 네 발 각각의 접촉을 직접 식별하지 못한다. sensor stability와 offline four-foot 3s contract를 별도 결과로 유지한다. 약 9mm raw ToF와 near Flow의 실기 정확도 근거가 없으면 센서 지지 판단은 S/U 조건부다. 사용자 전달 ST 칩 4cm 최소 보장 거리의 공식 근거는 아래 하드웨어 근거 문서에 별도 검증·반영하며 이번 조건부 접촉 승인을 실기 승인으로 확대하지 않는다.

위 분석 그림은 화면 검토했고, CSV는 수치 행렬 read-back을 확인했다. seed1 한 건의 경로를 설명하는 증거이며 원인별 보편적 정확도·개선 성공률을 주장하지 않는다.

## seed8 새 물리 실행의 별도 독립 검토

대상: [20261008-035506-047754-entry1-seed8](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/run_manifest.json). 검토자 Codex-Independent-Flight-Reviewer(Heisenberg), independent_reviewer, 실행 작성자 main-flight-adapter. **조건부 L2 simulation_contact_success=true 승인, qualified_mission_success=false, 승격용 성공 0, entry1 유지.** [새 signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/signedreview.json)을 수동 판단 후 작성했고 [계약 7절 검증](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/review-validation.json)은 valid=true/contact=true/qualified=false다. 기존 seed1 승인과 별개인 새 코드·training seed 실행이다. 단순 두 실행 비교로 센서율 변경의 개선 효과나 성공률을 추정하지 않는다.

**실행 초기 소스 신원.** runner가 실행 전에 복사한 source-snapshot을 사용했고 최신 main 파일을 실행 당시 코드로 취급하지 않았다. [SHA 검증](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/source-snapshot-validation.json)의 소스 22/22, runtime library·build manifest 4/4가 원 manifest와 일치한다. seed1의 missing scene 문제는 이 실행에서는 없으며 controller는 Popper의 24787756… 버전으로 lower_align_world_height_m=.40을 읽는다.

| 사전 보존 파일 | 전체 SHA-256 |
| --- | --- |
| [run_mission3_physical.py](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/scripts/run_mission3_physical.py) | `76f300f6144e9bde7f499ce6b75d1ca6775e19b1f8ffd09f252b7d0ae527cb28` |
| [controller.py](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/scripts/mission3/controller.py) | `24787756cf23f06d122becd99c699f7dccb180a6e07bf41d3672d6609adcb715` |
| [physical_scene.py](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/scripts/mission3/physical_scene.py) | `a3977b1e11085fa203a700d8ddaad85fa5f6e5ed56cca3a9b969b339a713c880` |
| [physical_estimator.py](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/scripts/mission3/physical_estimator.py) | `45e4bbd0c40e4ca69c39be6bbc16521f1f5f5a1d60b34c512c7299694debf333` |
| [estimator.py](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/scripts/mission3/estimator.py) | `cdc5764bbf892c2a41eeebeb2322a3cd317edfb2caf1dde66b341101c9376779` |
| [controller .so](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/build/mission3-flight-controller/libmission3_controller.so) | `09eab0784f62c89d7b421e6aba75a6606e36ead189caad41172d9a0574bcc763` |
| [estimator .so](../../artifacts/mission3/physical/20261008-035506-047754-entry1-seed8/source-snapshot/build/mission3-firmware-host/libmission3_estimator.so) | `8ff946e9c319d60643486194b9aa351ba0c5d765382b9edd8665564340e41233` |

[공식 빌드 소스 감사](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/build-source-audit.json)는 두 saved build manifest의 공식 C/CMSIS/기록된 header 23항목을 사본 또는 원 SHA와 일치하는 현재 파일로 검증했다. PID 부호·legacy mixer·battery extraction 본문 SHA 91950373…는 앞 검토와 같고 body를 직접 재비교했다. run .so로 9,264개 static curve N값을 로그와 대조했으며 차이 0, 11,224개 truth 표본의 motor response 재귀도 차이 0이다. build script 자체의 당시 SHA 및 모든 transitive header 바이트가 이 실행에 보존된 것은 아니므로 완전 재현 빌드의 한계는 남는다. loaded binary 신원은 이번에는 run SHA로 묶여 있다.

**원 평가와 접촉 증거.** saved evaluator로 만든 [독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/reevaluation.json)는 원 offline-evaluation의 모든 필드와 같다. [직접 로그 재계산](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/log-audit.json)과 [178개 실제 접촉 표본](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/contact-log-samples.json)을 별도 보존했다.

| 항목 | 원 로그 직접 확인 |
| --- | --- |
| 첫 기하 접촉 / 모터 off | 220.9400000006909 / 221.46000000069338s, 차이 .52s |
| 계약 유지 종료 / 원 로그 종료 | 224.46000000069338 / 224.4800000007078s |
| 네 발 | 모든 접촉 178/178, 유지 151/151 truth 표본 |
| 모터 정지 | 유지 3,001/3,001 1ms 표본, 네 PWM/output/static target 모두 0 |
| 접촉 직전 모터 | 220.939s PWM [40121,40117,40004,40018], 공중 off 아님 |
| 유지 중 각 발 법선력 | .000627436–.085838658N, 604개 모두 양수 |
| 유지 중 법선력 합 | .004135721–.343350037N; off+.4s 이후 .343349769–.343350035N≈mg |
| 실제 모터 잔류 힘 | off .339214276N → off+.4s 6.9917e−10N, 순간 0이라고 하지 않음 |
| 유지 중 상대 미끄럼 / 수평속력 | off 대비 최대 3.7268e−6m / 3.901e−8–1.2585e−6m/s |
| 유지 중 실제 tilt | .000012835–.000018411° |
| 차량 / 하강 | 전체 .24999999999999997–.25m/s, 218.18s부터 유지 종료까지 직선 |
| 중심 거리 / 내부 점수 | .007604782780223816m / 10점, 대회 공식 점수 아님 |
| 영상 | accepted 707개, 저장 camera contract 4,310표본 |

첫 **220.94s 기하 접촉 표본의 네 법선력은 모두 0**이며, 다음 220.96s에 각 발 .00815–.00875N이 기록된다. 계약상 최초 접촉은 geom 접촉이며 그 순간의 지지력을 주장하지 않는다. 실제 motor-off 유지 구간의 법선력은 모두 양수다. seed1의 사후 force 재구성과 달리 이번 값은 원 truth에 mj_contactForce로 기록된 값이다. truth는 20ms 표본이므로 그 사이 1ms 접촉 연속성을 기록 자체로 증명하는 것은 아니다.

[collision 감사](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/scene-collision-audit.json)에서 net·vehicle artwork·board·floor·pad의 collision mask를 확인했다. 전체 기록에는 foot→pad 외 접촉이 없다. net은 box 벽, artwork는 box이고 propeller/deck 형상은 조건부여서 모든 실제 장비 충돌 형태를 검증한 것은 아니다. aircraft 초기 qvel 외 상태 쓰기는 없고 이후 xfrc_applied/mj_step으로 움직인다. vehicle의 연속 prescribed .25m/s만 허용된 상태 쓰기다. contact truth는 로그·시험중단 경로에만 있고 TOUCHDOWN/MOTOR_OFF 분기에는 입력되지 않는다.

**실제 sensor cadence와 HOLD.** [독립 센서 감사](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/sensor-and-hold-audit.json)는 deck_samples의 Flow 22,449개(10ms), ToF 8,980개(25ms)를 확인했다. ToF stamp/new flag 오류 0, raw packet 불일치 0, 새 ToF 소비 8,980회다. SelfMotion의 z correction·range regression·vz update는 range_new일 때만 실행한다. Flow는 매 호출 새 packet이며 별도의 flow_new 필드는 없고 flow_stamp_s로 새 발행을 확인했다. Flow tick에서 ToF age 최대 20ms다.

이번 runner의 phase 집합에는 MOTOR_OFF_HOLD가 들어가 bounded hint가 지속된다. 마지막 영상 capture 217.40s, 계약 유지 종료 때 age 7.06s<12s다. 유지 구간 100Hz 센서 301/301은 moving_surface_candidate 및 flow_fusion_usable이다. adapter의 HOLD stability는 **off+.02s(221.48s)부터 로그 종료까지 151/151** 표본이 true이며 원 센서·명령으로 직접 재계산해 모두 일치했다. estimated tilt .03013–.03554°, 계산된 relative Flow speed .000157–.010082m/s로 설정된 10°/.06m/s 이내다. .25s 유예는 이 실행에서는 쓰이지 않았다. range 중앙값은 50Hz 제어 표본의 최근 값을 사용하므로 같은 40Hz ToF를 일부 반복 가중한다. 새 측정 151개라는 뜻은 아니며 높이 융합 반복 방지와도 구분한다.

**실기관련성의 중대한 제한.** [near range 감사](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/near-range-limit-findings.json)에서 HOLD raw ToF 범위 −1.084~19.612mm, median 7.974mm다. 223.225s의 −1.084mm와 224.325s의 −.515mm는 raw range_valid=true로 기록되어 100Hz 센서 네 표본에서 body offset 후 +3.916/+4.485mm가 되었고 Flow usable에 들어갔다. range_packet의 valid는 잡음 전 기하 거리를 검사하며 returned range의 양수·accuracy를 보장하지 않는다. 이 run에는 ST 4cm metric accuracy 또는 PixArt 80mm 작동 범위 guard가 없다. 가까운 ToF·Flow가 계속 정확히 작동하는 낙관적 S 모델에서 얻은 sensor HOLD이고, 실기 착지·센서 지지 근거로 사용할 수 없다. 아래 [하드웨어 근거](hardware-sensor-evidence.md)의 부품 최소 범위와 실제 deck revision U를 구분한다. 조건부 물리 계약의 승인과 원 결과는 보존한다.

추가 회복 한계도 구분한다. 이 snapshot runner는 GEOMETRY_READY 뒤 별도 terminal adapter로 들어가며 예측 만료 때 `terminal_prediction_expired_before_touchdown`으로 break한다. 메인이 이후 연결한 post-GEOMETRY 회복은 이번 코드에 없다. 이번 run은 만료 전에 끝났으므로 접촉 승인을 막지는 않지만, 해당 실패 경로를 회복 가능한 것으로 승인한 것은 아니다. 첫 위치 불확실성 주입은 false이고 17개 manufacturer profile 항목에는 S/null이 남는다.

원 offline-evaluation 파일 SHA-256은 `0840a38ae90e135b87a535aa339719aa80fbe5cc21eafeb20f43c8018f9d71d6`, 계약 JSON digest는 `12384f12c70e080c468321b86ae29a5dcccc1e592e11ed87bf216317c8365da0`이다. [입력 무변경 확인](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed8-codex-reviewer/input-hashes-and-integrity.json)을 저장했다. 전체 실패·시도 분모, 메인 코드·로그·평가·기존 승인은 수정하지 않았다.

## 04:16 새 실행: component envelope와 IMU/명령 추력 접촉 후보

대상 [20261008-041621-455777-entry1-seed1](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/run_manifest.json)를 별도로 검토했다. **조건부 접촉만 승인, qualified=false, 승격용 성공 0.** [새 signedreview](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/signedreview.json)와 [검증 결과](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/review-validation.json)를 저장했다. 같은 training seed1 재사용과 새 hash의 실행이며 holdout 성공이 아니다. 기존 seed1/seed8 결과·서명을 보존한다.

**원 snapshot.** [소스 신원 검증](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/source-snapshot-validation.json)의 소스 26개와 runtime binary/build manifest 4개는 모두 run SHA와 일치한다. 이후 adaptive 자유비행 gain calibration·low expected force gate는 이 실행에 없다.

| 사전 보존 소스 | 전체 SHA-256 |
| --- | --- |
| [runner](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/run_mission3_physical.py) | `88f360ab0f9a63889f2bc3228f8c96059c11198efed21833cd6bb2610af1205b` |
| [touchdown.py](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/mission3/touchdown.py) | `0658fea4af7296b94736cb7902df7cc9212aabbfde2e3fed1b8b6ca3ec5c192a` |
| [controller.py](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/mission3/controller.py) | `139c87137e342d475f671e605a8b4c8d85ade34fc50d3173dc249a38cd11401f` |
| [physical_scene.py](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/mission3/physical_scene.py) | `19895b48bf046db9f73dd5e14ca27c4eca2578908c9477f6dcfa75e70503cc1e` |
| [physical_estimator.py](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/mission3/physical_estimator.py) | `6702944e8a240e2662d947244b5925d5dea9e1723adf6154e65a106dd714e2aa` |

보존 evaluator로 만든 [재평가](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/reevaluation.json)는 원 평가와 모든 필드가 같다. [직접 접촉 감사](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/log-audit.json)의 첫 기하 접촉은 **164.94s foot_4 한 발**이며 그 뒤부터 네 발이다. 접촉 180표본 중 네 발 179표본, motor off 165.50s 이후 계약 종료 168.50s까지는 **151/151 네 발**, 종료 168.52s다. 유지 중 각 발 법선력 .015612948–.085838385N, 합 .072386212–.343350045N으로 모두 양수이고 off+.4s 이후 합은 약 .34335N=mg다. [원 접촉 사본](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/contact-log-samples.json)을 보존했다.

모터 유지 구간 3,001표본의 PWM/output/static target가 모두 0이다. 첫 접촉 직전 PWM [39452,39440,39430,39434]로 공중 차단 낙하가 아니다. 실제 잔류추력은 off 때 .270964951N, .4s 후 5.5850e−10N이다. 유지 중 off 대비 상대 미끄럼 최대 6.9456e−6m, 상대 수평속력 최대 2.3441e−6m/s, tilt .000025052–.000147426°다. 기록된 접촉은 foot→pad만 있으며 net/artwork/board/floor 접촉이 없다. 차량은 계속 .25m/s, 161.98s 하강부터 유지 종료까지 직선이다. 첫 접촉 참 중심 오차 .014274617345761888m, 내부 9점이다. 실제 장비의 형상·관성·마찰·motor response는 계속 S/U다.

**truth 격리.** aircraft 상태 쓰기는 초기 외 없고, contact truth·실제 motor force는 logger/시험중단과 독립 사후 감사에만 사용된다. TouchdownObserver는 IMU specific force·gyro, 명령 PWM에서 계산한 공식 static curve, 자체 assumed lag/mass 상태만 받는다. 원 runner의 `touchdown.predict_motor(output["motor_static_force_N"],dt)`는 실제 plant motor_force를 읽지 않는다. 다만 observer의 assumed .035kg/.02s와 nominal plant가 같은 값이라는 조건은 모델 일치 S 가정이며 실기 보정 증거가 아니다.

**새 센서 경계와 관찰 결과.** [센서 감사](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/sensor-cadence-and-envelope-audit.json)의 Flow 16,853개/10ms, ToF 6,741개/25ms, stamp/new flag 오류 0, 새 ToF 소비 6,741회다. plant의 finite patch 최단 기하 거리로 .08m Flow validity, .04m ToF accuracy flag를 만들고, SelfMotion은 정확하지 않은 ToF의 높이 융합과 무효 Flow 융합을 막는다. HOLD 301개 100Hz 표본의 정확 ToF·유효 Flow·Flow fusion은 **모두 0**, surface는 ambiguous 301개다. 지지 stability가 되었다고 표면 인식이 복구된 것은 아니다.

accuracy flag는 잡음 전 **센서 모델의 기하 거리**에서 만든 조건부 driver 가설이다. [경계 잡음 표본](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/boundary-noise-samples.json)에서 raw .036884/.039379m 두 ToF가 range_accurate=true로 전달되어 100Hz packet 5개에 나타난다. 따라서 제조사 부품문서에 근거한 메인 개발자 시험 정책의 “raw <.04m 무조건 차단”을 바이트 수준으로 구현한 것은 아니며 실기 장치가 참 sensor 거리 기준 quality flag를 제공한다고 보장하지 않는다. 이 HOLD에는 그러한 정확 ToF가 없으므로 접촉 계약과 분리된 경계 사례다.

geometry handoff 설정 높이는 .11m, 실제 GEOMETRY_READY→TOUCHDOWN_DESCEND는 162.48s다. IMU로 전파한 body height<.025m에서 164.68s collective probe를 시작했고, 164.94s 첫 접촉은 probe+.26s(승수 .935), off 때 승수 .795다. 마지막 영상은 161.20s, 유지 종료 age 7.30s<12s다. 메인의 post-GEOMETRY 만료 회복 연결은 이 snapshot에는 들어 있지만 이 정상 완료 실행으로 해당 실패 경로의 성공을 주장하지 않는다.

**독립 support residual 계산.** [관찰기 감사](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/touchdown-observer-audit.json)는 명령 static force만으로 assumed lag 재귀를 만들었다. 10개 IMU 블록 평균과 경계 raw sample로 .15s 윈도우의 150/151개 acceleration 평균을 복원하고 8,376개 command 표본에서 비교했다. 예상 motor 가속도 차이 0, specific-force 평균 최대 차이 5.33e−15, 잔차 최대 차이 1.32e−14m/s²다. 임계값 1.2m/s²·near-g ±1.5·gyro<.3rad/s·tilt<10°·.25s confirmation 조건을 다시 계산해 off 165.50s와 일치했고 실제 첫 접촉 전 support 조건 true는 0개다. raw 조건은 165.08s에 처음 true지만 중간 reset이 있어 확인까지 .42s 걸린다. 1kHz raw gyro norm 전체는 로그에 없어 그 평균은 기록값으로 gate를 확인했으며 원 표본을 재현했다고 주장하지 않는다.

HOLD는 off+.02s부터 종료까지 151/151 stable다. 이 stable는 near-g specific force·gyro·추정 tilt만 검사하며 **positive residual, low expected force, pad 위치, 수평 미끄럼을 HOLD 조건으로 요구하지 않는다**. 명령 모델 current acceleration과 residual의 평균 시각도 다르므로 두 필드를 순간 차이로 해석하지 않는다. [실제 유지 force와 관찰 값](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/hold-force-evidence.json)은 사후 비교이며 online에 입력하지 않았다.

**가장 큰 남은 위험: 모델 오차와 실제 접촉의 비식별성.** 원 스냅샷 관찰기를 별도 synthetic fixture로 실행해 정확 모델 hover(9.81/9.81)와 자유낙하는 거부, 명령 모델 7m/s²·IMU 9.81m/s²는 접촉으로 확인됨을 재현했다. 같은 신호는 실제 지지 중 줄인 추력과, 실제 추력은 호버를 유지하지만 observer force/mass가 틀린 공중 hover에서 모두 가능하다. 무접촉 정속 hover를 가정한 대수식에서는 예상 thrust acceleration 약 **12.23% 과소추정** 또는 assumed mass 약 **13.94% 과대추정**만으로 1.2m/s² 잔차 임계값에 닿을 수 있다. 공력·ground effect·IMU offset도 잔차와 혼동될 수 있다.

이 버전의 enabled는 추정 near_xy와 terminal 지속시간>.5s이며 probe 시작이나 충분히 낮은 expected force를 필수로 묶지 않는다. HOLD의 near-g 검사만으로 pad 네 발·수평 slip·바닥/선/본체 지지를 구분하지 못한다. 따라서 **모델 식별·보정 및 low-force 조건은 다음 hash에서 별도로 반례 검증할 과제**이며 이번 성공을 실기 접촉 관찰기의 보장으로 읽으면 안 된다. 원 [test_touchdown.py](../../artifacts/mission3/physical/20261008-041621-455777-entry1-seed1/source-snapshot/scripts/mission3/test_touchdown.py)도 이 false-hover 반례를 명시한다. 현재 조건부 승인은 관찰기 자체가 아니라 원 물리 truth의 실제 네 발 지지·정지·3초 계약에 근거한다.

원 평가 파일 SHA-256 `aec5628855c245c3597b19ef9f02ebff32185b3543afc4fa351ac8f86efcd1ca`, 계약 평가 digest `ed030e6c30b5a05ff8c0f27ebdc2046d23665b938fc39e505447e9b7e8afeeba`를 서명에 묶었다. [입력 무변경](../../artifacts/mission3/independent-review/20261008-physical-041621-entry1-seed1-codex-reviewer/input-hashes-and-integrity.json)을 확인했으며 controller/estimator/touchdown/plant/evaluator·원 로그·결과는 수정하지 않았다.

## 04:20 adaptive calibration seed8 독립 검토

대상 [20261008-042051-360247-entry1-seed8](../../artifacts/mission3/physical/20261008-042051-360247-entry1-seed8/run_manifest.json). Heisenberg / Codex-Independent-Flight-Reviewer가 이 실행의 초기 snapshot과 로그를 별도로 검토했다. **조건부 접촉 승인, qualified=false, 승격 성공 0/entry1 유지.** [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/signedreview.json)과 [계약 7절 검증](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/review-validation.json)은 valid/contact=true, qualified=false다. 과거 04:16 실행의 관찰기에는 이 보정을 소급하지 않는다.

| 항목 | 독립 확인 |
| --- | --- |
| 소스/바이너리 | 사전 보존 26개 소스 및 .so/build manifest 4개 모두 원 SHA 일치 |
| 첫 접촉 / off / 유지 종료 / 로그 종료 | 209.82 / 210.54 / 213.54 / 213.56s |
| 접촉·정지 | 전체 188/188 네 발, 유지 151/151 네 발·양의 법선력, 3,001/3,001 네 motor PWM/output/static target 0 |
| 유지 각 발 / 법선력 합 | .025329092–.085838709N / .120015202–.343350048N |
| 실제 모터 잔류 힘 | off .223336404N → off+.4s 4.6033e−10N |
| 유지 slip / tilt | off 대비 최대 9.7333e−6m / .000035634–.000225011° |
| 중심 오차 / 점수 / 영상 | .020085416315441362m / 내부 8점 / accepted 510개 |
| 실제 센서율 | Flow 21,357개/10ms, ToF 8,543개/25ms, stamp/new flag 및 원 packet 대조 오류 0 |

[원 evaluator 독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/reevaluation.json)는 원 평가와 전 필드가 같다. [로그 감사](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/log-audit.json)에서 접촉은 네 foot→pad뿐이며, 하강·유지는 직선, 차량은 연속 .25m/s다. 초기 외 aircraft qpos/qvel 쓰기 없음을 matching plant에서 확인했다. online 후보는 IMU·명령 공식 software curve·자체 관찰기 상태만 받으며 plant contact/actual force는 logger와 harness 중단 경로에만 있다. 같은 공식 PID 부호·mixer·battery extraction과 런타임 curve, motor-response 재귀도 [build 감사](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/build-source-audit.json) 및 로그와 일치한다.

| 이 실행에서 달라진 입력 | 전체 SHA-256 |
| --- | --- |
| [runner snapshot](../../artifacts/mission3/physical/20261008-042051-360247-entry1-seed8/source-snapshot/scripts/run_mission3_physical.py) | `db7f8ab47d1e0000d5d7e2b44b566a597d5c733a79dd6823a62f6737abb01e55` |
| [touchdown snapshot](../../artifacts/mission3/physical/20261008-042051-360247-entry1-seed8/source-snapshot/scripts/mission3/touchdown.py) | `22526f436231b94bb947a0d7d4caf24baf39e7ca56a64b7a2e98abd39c5920a4` |
| experiment.json | `1aba7fca37b7a547f072afbf6cf424ff634e91de98b9e8cb29edb3c9aaf96153` |
| 원 offline-evaluation 파일 | `1fc2efb3b3b2778180c10857f60337fafa5b978cf6b9ad61014a91ef781ae573` |
| 계약 evaluation digest | `138c954a465bdeca9151b486266c1217f10f1e9209a6bbf34f68c896d65288bc` |

**보정과 접촉 gate.** calibration permission은 terminal 없음, 직전 qualified floor Flow 상태·정확 range, 추정 z>.65m, tilt<10°를 요구한다. freeflight 명령 모델 힘/IMU acceleration 비율을 2s window·최소 1s span·RMS<.3m/s²로 fit하고 terminal 동안 고정한다. force_gain/mass를 함께 식별하는 비율이며 실기 질량 측정이 아니다. 동결 값은 28.5682172451/kg, 기록 RMS 약 .0201152m/s²다. 1kHz calibration 원 IMU 전체가 없어 fit 자체의 정확 재현은 한계로 남긴다. [독립 residual 감사](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/touchdown-observer-audit.json)는 동결 후 .17s부터 296개 command의 명령 lag와 IMU 블록 평균을 재계산해 최대 9.73e−15m/s² 차이를 확인했다. support 조건 최초 210.28s와 확인/off 210.54s를 재현했으며 실제 첫 접촉 전 조건 true는 없다. probe는 209.54s 시작, 접촉 때 승수 .902, off 때 .65다.

valid calibration 및 expected motor acceleration≤7.3m/s² gate는 보정 전 나쁜 모델 hover와 일정한 모델 오차를 보정한 hover 반례를 거부한다. 그러나 **마지막 유효 fit의 valid/RMS는 latch되며 후속 fit 실패나 age로 취소되지 않는다.** 보정 뒤 힘/가속도 관계가 달라져 모델 expected=7, IMU=9.81이면 원 관찰기 synthetic fixture가 여전히 contact를 확인한다. steady hover에서 7.3 gate까지 예상 힘이 과소추정되는 비율은 약 25.59%다. 이는 observer 신호 반례이며 MuJoCo가 실제 그 공력 조건을 재현했다는 뜻이 아니다. ground effect·IMU offset·변하는 lag/gain의 허용 범위를 별도 검증해야 한다. 최소 보완 설계는 보정 마지막 시각·fit span/RMS·최근 gain 변화를 기록하고 terminal 진입 때 freshness/일관성을 검증하며, 실제 probe 진행과 low-force 지속 시간을 접촉 조건에 결합하는 것이다. 이번 검토는 이를 구현하지 않았다.

HOLD 151/151 stable는 IMU near-g/gyro/tilt만 근거다. 100Hz HOLD 301표본은 정확 ToF·유효 Flow·융합이 모두 0이고 surface ambiguous다. residual 양수나 pad·slip·네 발을 HOLD gate가 요구하지 않는다. 센서 자체가 접촉 위치를 확인했다고 읽으면 안 된다. raw ToF<.04m인데 geometric accuracy flag가 true인 packet 9개도 남아 있으며 잡음 전 기하 flag와 실제 driver accuracy를 구분한다.

### 기존 18개 negative physics matrix의 별도 감사

[원 matrix](../../artifacts/mission3/contact-observer-negative/20261008-042212-474802/matrix.json)와 각 trace/evaluation을 모두 읽어 [독립 감사 JSON](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/negative-matrix-audit.json)을 작성했다. 재실행하지 않았다. 질량 .029/.035/.041kg × gain .8/1/1.2 × lag .02/.05s의 전체 18개 조합이다. benchmark 원 코드 SHA `a0b45c2242d3aa7116ac6f645663422b5b2431eca1a458f7e1c0ac23a867a01d` 및 선언한 22개 소스의 matching 바이트를 확인했다. [matching benchmark 사본](../../artifacts/mission3/independent-review/20261008-physical-042051-entry1-seed8-codex-reviewer/negative-benchmark-source.py)을 보존했다.

| 집계 | 결과 |
| --- | --- |
| negative control pass | 16/18: 보정 valid 14개 + invalid 2개 |
| 실패 | .041kg/gain .8의 두 lag: 각각 1.041s, .961s에 네 foot와 네 motor hull→floor 충돌 |
| calibration invalid | .035kg/gain .8 두 lag 및 위 floor 실패 두 개, 총 4개; motor stop 허용 안 됨 |
| 공중 거짓 후보 기록 | 18개 모두 0; candidate/false 집계가 저장되는 20ms 표본 기준 |

candidate는 1ms에 평가하지만 false counter와 trace는 20ms에만 저장된다. 따라서 전 1kHz 순간에 거짓 후보가 없었다는 증거로 확대하지 않는다. trace truth는 observer 시각보다 1ms 뒤의 step 결과이며 그 차이도 확인했다. 두 floor 실패를 제거하거나 calibration invalid를 성공적인 보정으로 세지 않는다. 먼 pad의 공중 대조 조건이며, 착지·움직이는 pad 유지·보정 뒤 gain 변화·ground effect·IMU bias를 검증하지 않는다. 모든 실패와 원 입력 해시를 보존했고 이 matrix를 미션 성공 분모나 제조사 검증으로 취급하지 않는다.

## 04:25 relative_kinematics seed1 독립 검토

대상 [20261008-042518-735813-entry1-seed1](../../artifacts/mission3/physical/20261008-042518-735813-entry1-seed1/run_manifest.json). **별도 조건부 접촉 승인, qualified=false, 승격 성공 0.** [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/signedreview.json)과 [검증 결과](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/review-validation.json)는 valid/contact=true, qualified=false다. 원 26개 소스/4개 binary·build manifest가 일치하고 [saved evaluator 재평가](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/reevaluation.json)는 전 필드 동일하다.

직전 snapshot 대비 runner 변경은 `public["observer"].update(physical.get("observer_software",{}))` 한 줄이다. 실행 설정에서 `vehicle_velocity_method=relative_kinematics`를 선택했다. runner SHA `e985e55c42cb290ce1473e22111fe6b38fde5199ddd8724081b493c38d5951c1`, matching estimator SHA `c646b52b3f0ceb747dee14cdcd17b0afa9077432483a457594aa6510f2ed3de1`, experiment SHA `68a86eaf6b9ba65060741292e5f57c073cdef90c6c14e648e1013afd60232e5e`다. touchdown은 위 22526f…와 같지만 로그 계산과 접촉 판단은 이 실행에서 다시 했다. 기본 observer 교체 승인이 아니다.

| 항목 | 이 실행의 원 로그 |
| --- | --- |
| 접촉 / off / 유지 종료 / 로그 종료 | 187.04 / 187.72 / 190.72 / 190.74s |
| 네 발 / 모터 | 접촉 186/186 네 발, 유지 151/151 네 발·각 양의 법선력, 3,001/3,001 모터 0 |
| 유지 법선력 각 발 / 합 | .025482709–.085838432N / .119187344–.343350047N |
| 실제 off 추력 → +.4s | .224164270N → 4.62037e−10N |
| 유지 slip / tilt | 1.69090e−5m / .000062989–.000277833° |
| 중심 오차 / 점수 / 영상 | .034903708045110164m / 내부 7점 / accepted 982개 |
| Flow / ToF | 19,075 / 7,630개, 각각 100/40Hz, stamp/new flag 오류 0 |

[접촉·힘 감사](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/log-audit.json)에서 foot→pad 외 충돌은 없고 하강·유지는 직선/차량 .25m/s다. [support 감사](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/touchdown-observer-audit.json)의 동결 gain 28.5707812360/kg, residual 재계산 최대 차이 1.08e−14m/s², 최초 support 187.46s/확인 187.72s는 실제 접촉 이후다. HOLD 151/151 IMU stable이나 정확 ToF/유효 Flow 0, surface ambiguous다. raw 경계 packet 12개와 보정 latch/보정 이후 모델 변화·미관찰 slip 위험은 같은 관찰기 한계다.

**상대 운동학은 세계 위치 회귀를 회피하지만 공통 gauge를 제거하거나 상대 정확도를 보장하지 않는다.** 선택한 branch는 capture-relative pose의 차량 좌표 미분 + qualified floor own velocity + heading-rate 회전항으로 속도를 fit하고, 관측 heading으로 세계 변환한다. qualified floor reference가 끊긴 뒤에는 마지막 속도를 유지하며 pad-relative Flow를 차량 속도에 되먹임하지 않는다. plant heading/속력 또는 설정 .25 FF를 입력하지 않는다.

[간단한 원 로그 점검](../../artifacts/mission3/independent-review/20261008-physical-042518-entry1-seed1-codex-reviewer/relative-kinematics-log-audit.json)에서 마지막 capture/receipt는 183.10/183.16s다. handoff 184.56s의 실제 중심 거리 20.658mm가 접촉 때 34.904mm로 커졌다. 접촉의 실제 aircraft−pad는 [−34.816,−2.470]mm, 상대 추정 bias는 [+35.451,+2.576]mm다. 이 직선의 오차 대부분은 차량 후방 방향 x이며 y 오차는 약 2.47mm다. own world bias .940881m와 pad bias .905431m의 공통 부분을 이 상대 bias와 혼동하지 않는다. 기록된 유지 속도 추정 [.263826095,.002549338]m/s 대 실제 [.25,0]은 offline 진단이고 truth로 튜닝하지 않았다. 다른 궤적의 한 실행으로 위치 회귀 제거의 폐루프 인과·개선율·기본값 교체 필요성을 단정하지 않는다.

원 평가 파일 SHA `03768254ce4d378bdcfaf45b3694e0b8cb618811cf770498afd9fc3280c3fe11`, 계약 digest `3db0f821246adc544108028579cee641423d1fc4f8ee82140a727ea94c1c5938`를 서명에 묶었다. training seed1 재사용·S/U 프로파일·이전 실패는 그대로 보존한다.

## 04:31 yaw bias·공분산 seed9 독립 검토

대상 [20261008-043152-833218-entry1-seed9](../../artifacts/mission3/physical/20261008-043152-833218-entry1-seed9/run_manifest.json). **조건부 접촉 승인, qualified=false, 승격 성공 0/entry1.** [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/signedreview.json)과 [계약 검증](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/review-validation.json)은 valid/contact=true, qualified=false다. 초기 source 28개/.so·build manifest 4개가 일치하며 [독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/reevaluation.json) 전 필드가 원본과 같다. 이전 seed9 실패와 다른 실행이며 실패를 지우지 않는다.

| 항목 | 독립 확인 |
| --- | --- |
| 첫 실제 접촉 / off / 유지 종료 / 로그 종료 | **328.24** / 328.96 / 331.96 / 332.00s, 인계의 대략 328.4s와 구분 |
| 접촉·모터 | 189/189 접촉 네 발, 유지 151/151 네 발·양의 법선력, 3,001/3,001 모터 0 |
| 각 발 / 합 법선력 | .021006881–.085838563N / .119452663–.343350046N |
| 실제 off 잔류 추력 → +.4s | .223898967N → 4.61490e−10N |
| slip / tilt | 2.00727e−5m / .000075229–.000480579° |
| 중심 거리 / 점수 | .041510090775372516m / 내부 6점 |
| 실제 센서율 | Flow 33,201 / ToF 13,281, 100/40Hz, stamp/new flag 오류 0 |

[원 로그 감사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/log-audit.json)에서 네 foot→pad 외 충돌이 없고 직선·연속 .25m/s 조건을 통과했다. matching plant/runner의 초기 aircraft 쓰기·truth logger/harness 격리는 동일하다. [support residual 감사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/touchdown-observer-audit.json)는 동결 gain 28.5723806756/kg, 최대 재계산 차이 1.04e−14m/s², 최초 support 328.70s/확인 328.96s를 재현했다. 첫 접촉 전 support 조건은 없다. 같은 7.3 gate/.65 probe가 적용되지만 보정 이후 모델 변화 반례는 여전히 남는다. HOLD 151/151 stable는 IMU뿐이며 정확 ToF/유효 Flow/융합 0, surface ambiguous 301개다. geometric accurate=true인 raw<.04m 경계 packet은 6개다.

| 사전 보존 소스/입력 | 전체 SHA-256 |
| --- | --- |
| [uncertainty.py](../../artifacts/mission3/physical/20261008-043152-833218-entry1-seed9/source-snapshot/scripts/mission3/uncertainty.py) | `7ba4ccf7a2f901b74577cd010cf03f488287fc63a842e44b48aea88a043affd4` |
| [estimator.py](../../artifacts/mission3/physical/20261008-043152-833218-entry1-seed9/source-snapshot/scripts/mission3/estimator.py) | `2c9cb1413fd7c74daecb894a811030c8c2352573d00eca2513fa0f025fbcfaa3` |
| [controller.py](../../artifacts/mission3/physical/20261008-043152-833218-entry1-seed9/source-snapshot/scripts/mission3/controller.py) | `b17431f56a8e348a7058f1fb10de500f796e4c99d8a33cc3c367548dab82410d` |
| experiment.json | `bbb654c55cbe6ee1a76cce3a8c5010737782aaea0c60add29d86a3f8a2fb7939` |
| 원 평가 파일 / 계약 digest | `5c9cb79660f35c947527fbd924e1b75ffaf71ae3d2f143f26292146c1799e1c0` / `8528d7994908d0a2ad7c74d1b2797fa7791a7485600e6f9b1fee35923c7bc93f` |

**yaw datum과 오차 예산의 범위.** [소스·로그 감사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/yaw-covariance-source-log-audit.json)에서 straight bias는 영상으로 fit한 pad/heading/rate와 알려진 static Stadium.nearest만 사용한다. samples≥12, heading rate<1.5°/s, 추정 cross innovation·heading gap<20°, straight 1.5s 연속 조건 뒤 I gain .002/s²로 보정하고 ±.2°/s에 제한한다. tangent 0/π 선택은 관측 heading 기준이며 plant heading/in_turn/실제 gyro bias를 읽지 않는다. SelfMotion은 측정 Euler yaw rate에서 추정 bias를 빼고 공식 firmware parallel roll/pitch 전파는 그대로다. 첫 track constraint는 297.36s, bias 첫 센서 기록은 298.93s, 최종 추정은 .0257570773°/s다. 이 수치는 실기 bias calibration 증거가 아니다.

새 공분산은 기존 mean observer와 같은 Flow/straight gain의 Joseph update 및 S noise/bias 가정으로 전파한다. 저장한 수평 covariance는 모두 PSD이고 최대 고유값의 제곱근과 position_std가 일치하며 probability flag는 전 표본 false다. 전체 8×8 P는 저장되지 않았고, 지도·반복 영상·bias의 공동 상관성 및 noise calibration은 미확정이다. 2° systematic floor는 straight update 때 yaw 대각에 적용되므로 이후 모든 시각에서 2° 이상이라는 뜻도 아니다. 이 값을 실제 coverage 확률로 읽지 않는다.

원 설정 mission budget **360s**, search deadline **300s**는 늘어나지 않았다. metric 발견 296.76s, handoff 325.74s의 std .889413m, 접촉 때 .929456m다. 1m 초과는 330.82s HOLD에서 처음 기록되고 최종 std는 1.044980m다. GEOMETRY_READY 이후 runner는 motor-off 전 영상 prediction age·mission timeout을 검사하지만 position_std를 별도 gate하지 않는다. 이번 접촉 전 1m 초과는 없었으며, HOLD의 작은 실제 slip은 offline truth 근거다. 별도의 terminal 상대 불확실성·관찰 가능성 설계가 필요하다. 이 실행은 기본 world_position_regression을 사용했고 직전 relative_kinematics 옵션을 적용했다고 오인하지 않는다.

## 분석·하드웨어 근거의 읽기 범위 보충

첫 seed1 [timeline.csv](../../artifacts/mission3/independent-review/20261008-physical-entry1-seed1-codex-reviewer/terminal-error-analysis/timeline.csv)의 `surface_predicate_reconstructed` 및 continued-hint 대안 열은 **terminal hint 조건식 진단**이다. 새 영상의 일반 surface 분류 전체를 재현하는 열이 아니며, 앞서 서술한 HOLD 구간 비교에 사용했다. 전체 시간의 실제 surface는 원 센서 기록 열을 읽는다. 위치·속도·gauge 증분 및 HOLD hint/age 결론은 그대로다.

공식 부품 최소 거리 근거는 [hardware-sensor-evidence.md](hardware-sensor-evidence.md) 및 [mission3-hardware-evidence.json](../../config/mission3-hardware-evidence.json)에 별도 반영했다. VL53L1X의 4cm 아래 target detection은 metric accuracy 근거가 아니며, PMW3901MB-TXQT old LN03-ZSZ/new L214-ZSZ의 80mm working range·42° typical viewing angle은 P **부품** 근거다. 실제 덱 lens allocation·장착·조립 calibration/FoV는 U로 유지한다. 약 9mm near ToF/Flow를 유효 정확 측정으로 가정한 첫 두 실행의 실기관련성을 주장하지 않으며 과거 조건부 접촉 서명은 지우지 않는다. 현재 검토 6건 모두 training/조건부 접촉이며 제조사 일치·승격 성공은 0이다.

## ledger 경로 표기 차이 확인

04:40 원 ledger의 review mismatch는 접촉 검토 누락의 증거가 아니다. [원 evaluator 경로 차등 감사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/ledger-path-only-differential-audit.json)에서 첫 실행을 절대 경로로 직접 재평가하면 원 저장 평가와 다른 필드는 informational `run_dir` 하나뿐이다. 그 차이가 canonical JSON digest를 바꿔 `evaluation_sha256_mismatch`를 만든다. 원 상대 ROOT 표기로 복원하면 평가 전 필드와 서명 검증이 일치한다. 원 평가·서명·evaluator는 다시 쓰거나 재서명하지 않았다.

메인의 CLI canonical path 수정 뒤 [기존 attempts 그대로 재검사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/ledger-canonical-path-check.json)는 당시 등록된 5건의 접촉 서명을 확인한다. 당시 attempts-0440의 seed9 reviews=[]는 서명 완료 전 목록이다. 실패 포함 11개 시도를 보존한 새 독립 검사 입력에 완료된 seed9 서명 경로만 추가한 [6건 완료 ledger 검사](../../artifacts/mission3/independent-review/20261008-physical-043152-entry1-seed9-codex-reviewer/ledger-six-completed-review-check.json)는 6건 contact approval을 검증하고 qualified approval/count는 모두 0, current_entry=1이다. ledger의 qualified 관련 `no_valid_independent_approval` 문구를 유효 contact review가 없다는 뜻으로 해석하지 않는다. 후속 시도 전체 집계는 메인의 별도 최신 ledger 대상이다.

## 04:36 Brown 부분 보정 일치 seed1 독립 검토

대상 [20261008-043643-702409-entry1-seed1](../../artifacts/mission3/physical/20261008-043643-702409-entry1-seed1/run_manifest.json). **별도 조건부 접촉 승인, qualified=false.** [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/signedreview.json)과 [계약 검증](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/review-validation.json)은 valid/contact=true, qualified=false다. 원 30개 소스와 .so/build manifest 4개 모두 SHA가 일치하며 [saved evaluator 독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/reevaluation.json)는 원 결과와 전 필드 동일하다. 이 실행은 **fully matched camera가 아닌 partial calibration**이다.

| 항목 | 원 로그 독립 확인 |
| --- | --- |
| 첫 접촉 / off / 유지 종료 / 로그 종료 | 163.70 / 164.46 / 167.46 / 167.48s |
| 네 발 / 정지 | 접촉 190/190 네 발, 유지 151/151 네 발·각 양의 법선력, 3,001/3,001 네 PWM/output/static target 0 |
| 유지 법선력 각 발 / 합 | .026799189–.085838367N / .120434988–.343350048N |
| 실제 off 잔류 추력 → +.4s | .222916619N → 4.59465e−10N |
| 유지 slip / tilt | 6.72614e−6m / .000024194–.000209345° |
| 중심 거리 / 점수 / 영상 | .013859463274124687m / 내부 9점 / accepted 527개 |

[접촉 감사](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/log-audit.json)에서 foot→pad 외 충돌은 없고 하강·유지는 직선, 차량 연속 .25m/s다. 초기 외 aircraft qpos/qvel 쓰기 없으며 online contact 판단에 plant contact/actual force가 입력되지 않는다. [support 감사](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/touchdown-observer-audit.json)의 동결 gain 28.5727745002/kg, residual 재계산 최대 차이 9.81e−15m/s², 최초 support 164.20s/확인 164.46s는 실제 접촉 이후다. 같은 adaptive calibration/7.3 gate/.65 probe의 모델 변화 위험과 raw calibration/gyro 기록 한계는 유지된다. HOLD 151/151 IMU stable지만 정확 ToF/유효 Flow/융합은 0이고 surface ambiguous다. raw<.04m인데 geometric accurate=true인 경계 packet은 2개다.

**중대한 영상 모델 불일치.** [원 yaw_core.py L63](../../artifacts/mission3/physical/20261008-043643-702409-entry1-seed1/source-snapshot/scripts/yaw_core.py:63)의 IPPE와 L75 raw reprojection RMSE는 K·D를 사용한다. 그러나 [자세 제약 refinement residual L80](../../artifacts/mission3/physical/20261008-043643-702409-entry1-seed1/source-snapshot/scripts/yaw_core.py:80)은 L83의 `xyz @ self.K.T`와 L84 perspective divide로 투영하며 **D를 무시한다**. 따라서 IPPE seed와 마지막 constrained yaw/translation fit가 서로 다른 투영 모델이다. 원 constrained RMSE·pose를 Brown forward model의 정확도 검증으로 읽으면 안 된다. [독립 optics 감사](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/optics-partial-calibration-audit.json)에 원 코드 행을 보존했다. 메인이 이후 고친 `cv2.projectPoints`는 이 snapshot에 없으며, 새 코드·새 실행에서 검증할 사항이다.

synthetic lens는 87°H/87°V/115°D 경계값을 만족하는 f=182.6897594649px, k1=−.0856132110의 **S 후보**다. virtual sensor와 observer 설정의 K·D dict는 같지만 최종 pose의 Brown model 일치와는 구분한다. `SyntheticLens`는 100° pinhole 원 렌더에서 각 320×320 출력 pixel의 후보 ray를 sample하는 가상 센서 형성이다. PC 입력 resize/color repair가 아니며 실제 lens calibration도 아니다. coarse bearing은 undistortPoints로 D를 사용한다. 실제 camera K/D·mount·배정 lens는 U/null로 유지하고 이 성공으로 채우지 않는다.

| 사전 snapshot/입력 | 전체 SHA-256 |
| --- | --- |
| runner | `8cee0f2af3d46dac76017a88a1328fe2629290b35619842588cd0be67f636a92` |
| optics.py | `08e42abecbe2212b81c03b4775c87e00ce587062a55bd9afb1d5882b5a53e13a` |
| vision.py | `d0062b9f82b603198ed1e3143778e93d0f02420c9b549728a3b1cb865ff33668` |
| physical_scene.py | `95f96d2df9657990dda59f754a4bf132e35b1fa7d9582378d88e1f547b14f303` |
| 원 yaw_core.py | `ec907b4789e5774d9eb1d89ac0023d9b8c7fa4ca54ba51cc1b2b0e29ba621033` |
| experiment.json | `384e6b3380c880eb490bda03591980e49a1f328275ab0474d55482a26412cf76` |
| 원 평가 파일 / 계약 digest | `f3cf2e65e1f63412b26fa648915966bee0bc9c0bdf63970a5d1d81e38b3ccd23` / `eb63989d4f89e9f250dab6a9db1dabf77110795fc6a48f92009c1fef3c6780af` |

거리 경계 정책의 선택 주체도 [정정 기록](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/policy-attribution-correction.json)에 남겼다. 4/8cm는 **제조사 부품문서를 근거로 메인 개발자가 선택한 보수적 시험 정책**이며 인간 사용자가 threshold를 직접 지정한 것은 아니다. 하드웨어 문서 §12와 JSON의 `software_policy_developer_selected`에 반영했고 부품 값·기존 실제 하드웨어 null·과거 조건부 서명은 유지한다. 이 Brown 실행까지 완료한 별도 접촉 검토는 총 7건, 제조사 qualified approval/승격 성공은 0, entry1이다. 이후 expiry recovery 실패와 새 Brown seed는 이 서명의 성공 범위에 합치지 않는다.

[7건 완료 canonical ledger 검사](../../artifacts/mission3/independent-review/20261008-physical-043643-entry1-seed1-codex-reviewer/ledger-seven-completed-review-check.json)는 attempts-0440의 11개 원 시도·실패를 그대로 두고 뒤늦게 완료된 seed9/Brown 서명 경로만 추가한 별도 입력으로 재평가했다. 접촉 승인 7개가 유효하며 qualified 승인·count는 0, current_entry=1이다. 원 attempts/ledger 또는 서명을 덮어쓰지 않았고 재서명하지 않았다. 이 검사는 해당 11개 시도 집합의 서명 결합 검사이며 04:40 이후 모든 신규 시도의 최신 분모를 대신하지 않는다.

## 04:48 Brown D refinement 일치 seed1 독립 검토

대상 [20261008-044822-431033-entry1-seed1](../../artifacts/mission3/physical/20261008-044822-431033-entry1-seed1/run_manifest.json). **이 실행을 별도로 조건부 접촉 승인, qualified=false.** [signedreview.json](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/signedreview.json)과 [계약 7절 검증](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/review-validation.json)은 valid/contact=true, qualified=false다. 원 snapshot 30개 소스 및 runtime .so/build manifest 4개가 전부 SHA와 일치하며 [saved evaluator 독립 재평가](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/reevaluation.json)는 원 결과의 모든 필드와 같다. 이전 04:36 partial calibration 기록을 다시 쓰지 않았다.

| 항목 | 독립 로그 확인 |
| --- | --- |
| 첫 실제 접촉 / off / 계약 유지 종료 / 로그 종료 | 161.28 / 162.10 / 165.10 / 165.12s |
| 접촉·모터 정지 | 전체 193/193 네 발, 유지 151/151 네 발·각 양의 법선력, 3,001/3,001 네 PWM/output/static target 0 |
| 각 발 / 합 법선력 | .027152032–.085838493N / .121249310–.343350048N |
| 실제 off 잔류 추력 → +.4s | .222102286N → 4.57787e−10N |
| 유지 slip / tilt | 3.98136e−6m / .000013925–.000172270° |
| 중심 거리 / 내부 점수 / 영상 | .008192664688416017m / 10점 / accepted 490개 |
| 센서율 | Flow 16,513개/100Hz, ToF 6,605개/40Hz, stamp/new flag 오류 0 |

[로그 감사](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/log-audit.json)에서 foot→pad 외 충돌이 없고 하강·유지는 직선/차량 연속 .25m/s다. aircraft qpos/qvel은 초기 외 쓰기 없고, contact/actual motor force는 logger·harness 중단용이다. 새 영상·covariance 코드도 native image, capture-time IMU, 알려진 static track 및 S 설정만 사용하며 live heading/contact truth를 받지 않는다. 같은 공식 PID·legacy mixer·battery 함수/런타임 curve 및 motor response 재귀를 다시 로그와 대조했다.

**수정한 projection의 실제 기록 일치.** [원 yaw_core.py L86](../../artifacts/mission3/physical/20261008-044822-431033-entry1-seed1/source-snapshot/scripts/yaw_core.py:86)의 constrained residual은 `cv2.projectPoints(xyz,0,0,self.K,self.dist)`로 투영한다. IPPE·raw cost·constrained refinement가 모두 D를 사용하며 coarse bearing도 undistortPoints를 사용한다. [독립 optics 감사](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/optics-matched-refinement-audit.json)는 capture 시각에 기록한 추정 roll/pitch와 각 후보 pose에서 별도 회전행렬·Brown 투영으로 RMSE를 계산했다. accepted 490개/후보 980개의 보고 cost와 최대 차이 **1.49048e−14px**다. 같은 pose를 pinhole로 투영한 RMSE는 .04969–17.45630px이며, 이 숫자는 코드 경로 구분을 위한 동일 pose의 내부 잔차 비교다. true pose 정확도나 D 수정만의 폐루프 개선량을 뜻하지 않는다.

render lens와 observer K/D 설정은 앞과 같은 f=182.689759px/k1=−.085613211의 S 후보이며 dict가 동일하다. **fully matched라는 말은 이번 가상 센서와 관찰기의 투영 모형 일치에만 적용한다.** 실제 camera K/D·mount·lens allocation은 U/null이고 실측 calibration으로 승인하지 않는다. coarse max-side pixel 전달과 uncertainty floor 유지도 함께 달라졌고 training seed1을 재사용했으므로 .00819m 점수 결과를 D fix 하나의 인과 효과나 holdout 성능으로 주장하지 않는다.

| 변경된 snapshot/평가 결합 | 전체 SHA-256 |
| --- | --- |
| yaw_core.py | `2971176642d5e5fdbdc3c2be6849323ed5242919eaf792457789f49d6330f09e` |
| vision.py | `1cabad062c06b59b6bebfb6838983c7c6b29971a5b4fee09ae07ae3774d062fa` |
| uncertainty.py | `6f37004b936c7ae69588f28f2415ef97accfad570dc90557b0cd5a7611527e8f` |
| experiment.json | `384e6b3380c880eb490bda03591980e49a1f328275ab0474d55482a26412cf76` |
| 원 offline-evaluation 파일 | `8d1b5f277c705c22c06899d4b7684244890be22e413336a8ca9960471b5686c9` |
| 계약 evaluation digest | `e66c2ffa4223ae6d7984676dbe132fe25a993f10ef74a66e156badd1dd7e5654` |

새 uncertainty는 straight update 뒤 predict에서도 yaw floor를 유지하며 원 센서 기록의 이후 heading_std 최솟값은 2°다. calibrated probability flag는 모두 false다. [support 감사](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/touchdown-observer-audit.json)의 동결 ratio 28.5719990715/kg, 잔차 재계산 최대 차이 9.98e−15m/s², 최초 support 161.84s/확인 162.10s는 실제 접촉 이후다. calibration valid/RMS latch, 보정 후 추력 관계 변화, 미기록 1kHz gyro/calibration 원 표본의 한계는 남는다.

HOLD 151/151 stable는 IMU near-g/gyro/tilt 기준이다. HOLD 301센서 표본의 정확 ToF·유효 Flow·융합은 0이고 surface는 ambiguous다. 작은 실제 slip와 네 발 지지는 offline truth 근거이며 IMU stability만으로 그 위치·발별 접촉을 확인하지 못한다. raw ToF<.04m이면서 geometric accuracy=true인 경계 packet 2개도 남아 있고 실제 driver accuracy 보장은 아니다.

이번 실행까지 요청된 성공 후보 **8건의 개별 조건부 접촉 서명을 완료**했다. 제조사 qualified 승인·승격 성공은 0, entry1 유지다. seed2의 새 탐색 실패·partial Brown 실패·expiry recovery 실패를 지우거나 승인 성공으로 합치지 않는다. 하드웨어 근거 문서와 JSON의 4/8cm 정책 선택 주체 정정은 유지하며, 이 bounded 검토 종료 후 해당 두 파일의 쓰기 소유권을 메인에 반환한다. 후속 manufacturer CAD gap 연구는 별도 할당 범위다.

[종료 시점 전체 완료 시도 ledger](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/ledger-all-completed-with-eight-reviews.json)는 [기록 시점 입력](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/full-ledger-input-snapshot-scope.json)의 completed physical mission 16건을 원 로그로 재평가했다. 접촉 pass·유효 개별 승인 8건과 접촉 fail 8건이며 qualified approval/count 0, current_entry=1이다. 실패 실행에 저장 offline-evaluation이 없더라도 제외하지 않았다. 이전 부분 Brown mismatch 실패와 044318 expiry recovery, 044603/045238 seed2 탐색 실패도 포함한다. fixture/airborne/oracle benchmark는 별도 범위이며, 이 training 집계에서 실기 성공 확률을 추정하지 않는다. 원 ledger/시도/로그는 덮어쓰지 않았다.

[하드웨어 문서 소유권 반환 기록](../../artifacts/mission3/independent-review/20261008-physical-044822-entry1-seed1-codex-reviewer/hardware-evidence-ownership-handoff.json)에 반환할 두 파일의 최종 SHA를 남겼다. 이후 메인 또는 새 CAD 근거 작업자가 이 문서/JSON을 이어서 수정할 수 있다.


## 18. 051833: 선택적 IMU texture bridge 개별 검토

[실행](../../artifacts/mission3/physical/20261008-051833-059560-entry1-seed1) · [개별 서명](../../artifacts/mission3/independent-review/20261008-physical-051833-entry1-seed1-codex-reviewer/signedreview.json) · [최종 검증](../../artifacts/mission3/independent-review/20261008-physical-051833-entry1-seed1-codex-reviewer/final-verification.json). Heisenberg가 원snapshot과 로그를 개별 판독했다. **조건부 접촉만 승인, qualified=false, training seed1, entry1 유지.**

첫 접촉 161.28s, motoroff 162.08s, hold 끝 165.08s, 기록 끝 165.10s. 192접촉 모두 4발; hold 151/151표본에서 각 발 법선력 양수(.024159–.085839N). 3001 모터 기록의 PWM/output/static target은 0이다. 실제 lag 잔류추력은 정지 시 .224280N에서 +.4s에 4.62e−10N으로 감소한다. hold 상대 slip 최대 14.35µm, 실제 tilt 최대 .000283°. 내부 오차 **29.6344mm/8점**이며 공식 점수·실기 검증이 아니다.

[새 terminal/texture 감사](../../artifacts/mission3/independent-review/20261008-physical-051833-entry1-seed1-codex-reviewer/terminal-texture-audit.json)는 158.14s handoff 전 11명령/0.20s의 geometry·calibration·image age<3s·pixel/depth 게이트를 확인했다. 이후 motoroff까지 198명령의 공통 odometry/net/straight/prediction 및 6s bridge 예산을 snapshot 코드로 재평가해 모두 통과했다. off 시 image age4.88s, bridge elapsed3.94s다. 센서/영상 관찰만 입력이고 contact truth는 기록·바닥 실패 harness에만 쓰인다. qpos/qvel은 기체 초기화 이후 덮어쓰지 않는다.

Flow16511@100Hz, ToF6605@40Hz이며 stamp/newflag와 raw 전달 대조가 일치한다. texture reason은 two-axis13945/weak1751/shift-limit17/texture-or-aperture798이다. correlation은 유효성만 게이트하며 실제 Flow 수치는 기존 finite-ray 수학 모델에서 생성한다. generic 64px 렌더·gradient·phase threshold는 S이며 PixArt correlation/SQUAL/shutter 검증이 아니다. HOLD 정확 ToF·유효 Flow·융합 0, surface301/301 ambiguous이며 151/151 IMU stable는 발별 접촉·pad 위치·lateral slip을 알아낼 수 없다. 정지 후 6s 예산 초과는 비재시작 HOLD 상태에서 별도로 해석한다.

[support 독립 계산](../../artifacts/mission3/independent-review/20261008-physical-051833-entry1-seed1-codex-reviewer/touchdown-observer-audit.json)의 최초 조건161.82/확인162.08s는 실접촉 뒤다. 명령curve/lag와 기록 IMU 평균에서 계산한 잔차 차이 ≤1.038e−14m/s². calibration latch·미기록1kHz calibration/gyro·보정 뒤 gain 변화/ground effect/IMU 장착 한계는 남는다. 변화한 모델의 hover 거짓 candidate 반례를 보존했다. raw ToF<4cm인데 accuracy=true인 2packet은 noise 전 기하값으로 flag를 만든 모델 경계이며 실제 driver 사양으로 승격하지 않는다.

snapshot terminal 6 tests 및 순수 white/single-edge/two-axis texture controls 통과. 최초 plain Python의 texture suite는 mujoco import 실패로 기록했으며 renderer 통과를 주장하지 않는다. 권고는 **새 hash/새 seed에서 calibration freshness·excitation, 변동 gain/ground effect/IMU mount 음성 시험, IMU-only 상대 오차 예산을 검증**하는 것이다. 044822 대비 오차 증가를 단일 texture/handoff 원인으로 단정하지 않는다. 기존 실패051046 및 과거 서명은 유지한다.

| 결합 대상 | 전체 SHA-256 |
| --- | --- |
| 원snapshot scripts/run_mission3_physical.py | `2eb2e6c2cf6f7cff2d93f00de32c0aa23d5e677f609183ae7f8b87cd8bb2b5c9` |
| 원snapshot scripts/mission3/controller.py | `1f9b00ec75a41f93c989272bfabbd20ad4e814b366e9602c79dcfcd73b2f1a79` |
| 원snapshot scripts/mission3/physical_scene.py | `3557a66c9da2032b7245c3386af1fbbe78cd58a8a645c1602e91652a78314b59` |
| 원snapshot scripts/mission3/flow_texture.py | `0292bb8cb7948199c7c3379ec35538f1a277858658bd83f291be234532eb3e74` |
| 원 offline-evaluation 파일 | `69221bda27f54ad163c474abf1db750cdc29118df0a96949c2d8b5f1c9a06406` |
| 계약 evaluation digest | `0a64d5aff421d5802bcacda9fb338404abe7ac135e3463d68df960b9b86e62f5` |


## 19. 동결 holdout seed17·23 개별 판정

두 run을 각각 원 snapshot·기록으로 재평가하고 별도 서명했다. plan의 35source hash와 4실행binary/buildmanifest가 일치한다. 051833 이후 제어/plant 변화는 없으며 runner 변경은 config/trial identity 분리와 CLI다. 고정 설정 SHA `479a7e8da7575b99bdb18f5bfde0df3915b7459385012a0ec367ad1f7e8c4cb1`은 seed를 제외한 mission/physical/focus JSON, experiment.json은 seed·entry별 원파일 SHA로 각각 검증했다. 검토 시 기존 physical에서 동일 seed17/23을 찾지 못했다. **각 조건부 접촉 승인·qualified=false·entry1 유지.**

| 개별 실행/서명 | contact/off/holdend/logend s | 접촉/hold/모터0 표본 | 내부 오차/점수 | hold slip |
| --- | --- | --- | --- | --- |
| [seed17](../../artifacts/mission3/independent-review/20261008-physical-052513-entry1-seed17-codex-reviewer/signedreview.json) | 78.42/79.14/82.14/82.16 | 188/151/3001 | 40.6046mm/6 | 19.67µm |
| [seed23](../../artifacts/mission3/independent-review/20261008-physical-052514-entry1-seed23-codex-reviewer/signedreview.json) | 115.96/116.72/119.72/119.74 | 190/151/3001 | 67.8446mm/4 | 32.88µm |

둘 모두 모든 접촉·hold 표본은 pad의 4발이며 각 발 법선력이 양수다. 다른 물체 충돌 0, 차량 .25m/s·직선 착륙 조건을 확인했다. support 최초 조건은 각각78.88/116.46s로 실제 접촉 뒤이며 off와 재계산 확인 시각이 일치한다. 명령curve/lag와 IMU 창 평균·잔차의 독립 대조가 통과했다.

[seed17 terminal 감사](../../artifacts/mission3/independent-review/20261008-physical-052513-entry1-seed17-codex-reviewer/terminal-texture-audit.json), [seed23 terminal 감사](../../artifacts/mission3/independent-review/20261008-physical-052514-entry1-seed23-codex-reviewer/terminal-texture-audit.json)의 handoff 75.22/112.84s 직전0.2s11명령 조건·최종 공통 게이트·6s bridge 예산은 각각 통과. off image age4.88/4.82s, elapsed3.92/3.88s다. preoff world position std .769–.826/.790–.858m는 공통 gauge 불확실성이며 true landing error와 같은 숫자가 아니다.

Flow/ToF 표본은 seed17 8217/3287, seed23 11975/4790이고 실제100/40Hz·stamp/newflag 대조 통과. 각 HOLD151/151IMU stable, 정확ToF/유효Flow/융합0, surface301/301ambiguous다. generic texture S, geometric accuracy/noisy4cm 경계, calibration latch·보정 이후 모델 변화·pad/slip 관측 부재는 그대로 남는다. 6.78cm seed23 중심오차는 실제 4발 지지와 양립하지만 향후 footprint·상대오차 여유 검증이 필요하다. 성공만 고른 tuning 자료로 쓰거나 실제 제조사 성공확률로 해석하지 않는다.

| 원 평가 결합 | seed17 SHA-256 | seed23 SHA-256 |
| --- | --- | --- |
| evaluation_file_sha256 | `9248a0c11499527884c00928d4eba0897f516fe2c153bb6bd8ea93f18eaf0080` | `c3dae0edaedd85c381b6d1caa2e8bfa6be67199d2fc5ae5d670ab99dfc9e266c` |
| evaluation_sha256 | `a7bbab4671bc04536b0588781d91b18dec60da228796dfa22cdf2ee4b51fcef5` | `e85277e83907fd36b0935dc51b256b6a1ec17416a9a1b8544b36fa8d08080044` |
| trial_metadata_sha256 | `da16720796dccaceef5d9e957e98b6a41b47b46a678d088dbb38b5e38f37fdea` | `028e302b96c3c9c3688c44fcfc4689442efb31a340fcb2d117ec9072082b62a7` |


## 20. 동결 holdout seed31 개별 판정·3개 계획 완료

[seed31 개별 서명](../../artifacts/mission3/independent-review/20261008-physical-052649-entry1-seed31-codex-reviewer/signedreview.json) · [최종 검증](../../artifacts/mission3/independent-review/20261008-physical-052649-entry1-seed31-codex-reviewer/final-verification.json). 첫 정밀 획득288.26s로 검색기한300s 전에 획득했다. contact319.54/off320.26/holdend323.26/logend323.30s, 전체360s 예산 안이다. 189접촉 전부4발, 3s 평가창151/151네발·각 발 법선력 .027326–.085839N, 모터3001표본0, 다른물체충돌없음, vehicle .25m/s·직선이다. 오차6.2327mm/내부10점, hold slip3.04µm. **개별 조건부 접촉 승인·qualified=false.**

[terminal 감사](../../artifacts/mission3/independent-review/20261008-physical-052649-entry1-seed31-codex-reviewer/terminal-texture-audit.json)는316.54s handoff 전0.2s11명령 및 이후commonodometry/net/straight/prediction·6s bridge 통과를 확인한다. off image age4.66s/elapsed3.72s, preoffpositionstd .883–.946m는 world gauge 불확실성이며 6.23mm 상대착륙오차와 구분한다. support320.00/확인320.26s는 실접촉 이후다. HOLD adapter152/152stable는 +3.04s 추가 관찰을 포함하며 평가3s의truth151표본과 다른 집계다. 실제 정지 잔류추력 .223636N은+.4s에4.61e−10N으로 줄어든다.

Flow32331/ToF12933@100/40Hz 전달stamp/newflag 대조 통과. near 센서 융합0/IMU-only HOLD·texturegenericS·기하accuracy와noisy4cm 경계5packet·calibrationlatch/모델변화hover반례는 유지된다. absolute std가1m 게이트에 가까운 만큼 다음 변경에서 상대오차 예산·보정 freshness/excitation·변동 gain/ground effect/IMU 장착 반례를 검증해야 한다. true support를 온라인 상수튜닝에 쓰지 않는다.

[동결3개 리뷰 ledger](../../artifacts/mission3/independent-review/20261008-physical-052649-entry1-seed31-codex-reviewer/frozen-three-reviewed-ledger.json)는 plan seed17/23/31 **모두 완료·각 독립 접촉 승인 3/3, 제조사 qualified/count 0, entry1**이다. S 모델의 제한된 3회 결과이며 실기 성공확률로 해석하지 않는다. 이전 training 실패051046과 모든 과거 시도·서명은 별도 그대로 보존했다. 051833 포함 신규4개 개별 리뷰를 끝냈으며, [기존8서명 불변 감사](../../artifacts/mission3/independent-review/20261008-physical-052649-entry1-seed31-codex-reviewer/previous-eight-signature-integrity.json)도 통과했다.

| 결합 대상 | 전체 SHA-256 |
| --- | --- |
| evaluation_file_sha256 | `7f738fa594328acee4d52216bf42007d96e9336658ce9c5a12b4b2c24b1ab19c` |
| evaluation_sha256 | `b91c3e4444a562eebfce33882c6d0e50c83d187decc45bf2860be9ad542df05b` |
| trial_metadata_sha256 | `ca8b862e214af10bae35eac354a34a338a7e0bb8d81ba7fc553972a41d0eb69d` |
