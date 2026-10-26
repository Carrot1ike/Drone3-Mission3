# 미션2 run22 — 기술 원자료

기준일: 2026-10-27. 이 파일은 설명용 자료와 짝을 이루는 코드·수식·상수·로그·재현 근거다. 대상은 **run22 동결 소스와 run22 실행 기록**이다. 현재 작업 폴더에서 이어서 개발하는 코드, 삼각측량 변형, 출구 폭1.50m 후속 시험을 이 기록에 섞지 않는다.

실제 시험의 입구 폭은1.50m, 출구 폭은1.80m다. 정책의 입구 준비 전진1.50m는 입구 폭과 별개다. 정책에 출구 폭1.80m를 명시 인자로 전달했다. 통과 여유는0.20m이고 카메라 가산0.03m와 구분한다.

자료의 핵심 판정은 **8/8 순서 통과, 비바닥 접촉 표본0, DONE, 오류 없음, 런타임 정상 종료, 연착지 실패**다. 실제 접지 과속과 이후0.600초 안정 정지가 함께 관측됐다.

## R1. 파일 경로·식별자·읽는 규칙

### R1.1 경로 별칭

이 문서에서 “소스”의 파일·줄번호는 패키지의 source/mission2_fresh/를 뜻한다. 원래 경로는 .local/evaluation/fresh_mission2/run22/source/mission2_fresh/다. “run22/host/”는 패키지 evidence/run22/host/이고 원래 .local/evaluation/fresh_mission2/run22/host/다. JSONL 줄번호는1부터 센다.

분석표의 원래 경로는 .local/documentation/mission2_run22_20261027/command_outcome_examples.json, 같은 이름 CSV, selected_observations.json이다. 스캔 검증의 원래 경로는 .local/documentation/mission2_presentation_20261026/scan_speed_validation.json이다. 패키지에서는 대응 evidence/run22/ 파일을 확인한다.

| 대상 | 식별값 |
|---|---|
| 원본 저장소 | unLIMited72/Dynamic-Drone-Mission-SITL |
| 원본 기준 커밋 | 3c6e805abb3a42bfd76e0851c71d8f45b5b69851 |
| 평가 맵 | reshuffled_822026 |
| 실행 설정 이름 | reshuffled_822026_faster_scan_20cm |
| 펌웨어 | fresh_ring_pass_v2/ON/cf2 |
| 바이너리 SHA256 | ba55095224817fe98f07a88b990bf9c9005e20ba0e6ebd2b8cb97f3f185f4f3a |
| mission.py SHA256 | f66c3a560ec9cfae47725385a6e1f40cdd9b1875f898381365a36f481d375d46 |
| ring_scan.py SHA256 | 25c7abbe43e51512542a76efb1a09b771f4cdbca61301c54ea2cbd50d6978850 |
| wall_scan.py SHA256 | c4291991370d78b1da84921089690cf3a052bfe140dfa6eeb95e946190b08079 |
| main.py SHA256 | 3e3e2abf61887834aae37fbe5c89912a844f8585618e88a3c278617850c7446b |

전체 파일 해시는 constraints.json.controller_sha256 및 result_summary.json.controller_hashes로 대조한다. 원자료 번들의 최종 manifest는 별도 패키징에서 생성한다. 이 문서는 Git 게시·릴리스 업로드·PDF 생성의 완료 보고가 아니다.

### R1.2 코드 역할과 실제 줄번호

| 파일:줄 | 함수·범위 | 역할 |
|---|---|---|
| main.py:21 | run | 수신·판단·독립 송신의 실행 구성 |
| main.py:37 | receive | 영상 저장, 연결 상태 기록, 최신 입력 전달 |
| main.py:74 | 판단 입력 확인 | 유효한 연결 상태만 Mission.step에 전달 |
| main.py:78 | 판단 로그 | 명령·타깃·체크·거리·현재 단계 저장 |
| main.py:89 | DONE | 즉시 판단 루프 종료 |
| main.py:111 | CLI | ring-mask, exit-width, wall-scan 옵션 |
| commands.py:10 | command_worker | 원본 cflib 연결과20ms 송신 |
| commands.py:21 | parameter_ack | ringPass ACK 기록 |
| commands.py:47 | 요청·ACK 처리 | 마커 변경, ACK 전 수평0 |
| commands.py:56 | hover 송신 | 기체 기준 vx·vy, yawrate, EKF Z |
| mission.py:16 | clearance_margin_m | 통과 여유0.20m |
| mission.py:40 | transition | 벽 단계 전 스캔 삽입, stable·travel 초기화 |
| mission.py:50 | step | 인지·FSM·명령 생성 |
| mission.py:65 | ENTRY_FORWARD | 준비 전진과 링 관측 |
| mission.py:78 | SCAN | 초기 링 스캔과 최근접 선택 |
| mission.py:92 | FIND·ALIGN | 현재 링 추적·정렬 |
| mission.py:132 | STRAIGHT | 링 고정 전진 |
| mission.py:147 | EXIT_SCAN_BRAKE·SCAN | 출구 관측 연결 |
| mission.py:156 | WALL_SCAN_BRAKE·SCAN | 벽 좌우 관측 |
| mission.py:166 | passage 처리 | 블록·벽·출구 정렬과 commit |
| mission.py:201 | 통로 직진 | 적분으로 종료 |
| mission.py:210 | LAND·DONE | EKF 높이 기반 하강·종료 |
| mission.py:221 | 최종 변환 | 로컬 속도→body 속도, yaw_error 미사용 |
| vision.py:6 | F, CX, CY | 카메라 투영 상수 |
| vision.py:9, :14 | attitude, level_ray | 자세·광선 변환 |
| vision.py:19 | rings | 밝은 원형 띠 검출·거리 |
| vision.py:62 | vertical_obstacles | 남아 있는 별도 함수; 현재 passage 선택에 직접 쓰지 않음 |
| ring_track.py:12, :28 | predict, update | 관측된 링의 유지·재연결 |
| ring_scan.py:12–15 | 스캔 상수 | +45/−45/0, ±30°/s, 0.8/0.6 |
| ring_scan.py:25 | step | 후보 누적과 회전 단계 |
| ring_scan.py:71 | closest | 3D 추정 거리 최솟값 |
| passages.py:12 | level_image | 전체 자세 기준 보정 |
| passages.py:19 | solid_faces | 밝기 면과 바닥선 |
| passages.py:67, :74 | front_faces, rear_extent | 앞면·옆면 및 뒤쪽 범위 |
| passages.py:89 | blocks | 두 블록의 틈·중앙·거리 |
| passages.py:115 | wall | 스캔 미사용 경로; run22에서는 결과를 WallScan으로 대체 |
| passages.py:161 | exit_gate | 출구 폭 인자 없는 경로; run22에서는 ExitTrack 사용 |
| wall_scan.py:11 | plane_pixels | 관측 벽 평면과 영상 열 광선 교차 |
| wall_scan.py:33 | observe | 같은 벽 연결, 양쪽 바닥 후보 누적 |
| wall_scan.py:94 | step | 회전 완료 후 방향 선택 |
| wall_scan.py:101 | guidance | 선택 방향 유지, 상대 중앙·거리 반환 |
| exit_track.py:8, :15 | heading_image, posts | yaw 유지 보정, 기둥·바닥 접점 |
| exit_track.py:46, :76 | update, guidance | 기둥 쌍과 알려진 폭 척도 |
| floor_odometry.py:15 | step | 바닥 특징점 이동량 진단; 종료 적분 원천과 다름 |
| review.py:20 | evaluate | 독립 구간·접촉·착륙 평가 |
| telemetry_view.py:26 | render | 입력·판단·명령 합성 |
| telemetry_video.py:13 | main | 외부 실제 자세 재생과 내부 영상 |
| live.py:55 | main | 읽기 전용 실시간/기록 화면 |

## R2. 실행 구조와 사용 정보

### R2.1 프로세스·통신

수신 스레드는 실제 수신 영상을 NPZ로 저장하고 metadata와 연결 상태를 frames.jsonl에 쓴다. latest 한 슬롯을 갱신한다. 판단 루프는 최신 묶음에서 association_valid인 상태만 가져와 Mission.step(image,state,received)를 호출한다. 결과는 decisions.jsonl과 명령 큐에 전달한다.

별도 프로세스가 큐의 최신 명령을 선택하고 약20ms 간격으로 원본 cflib hover 명령을 보낸다. 새 판단이 없어도 기존 명령을 반복한다. 실제 송신을 commands.jsonl에 기록한다. 수신률과 판단률, 송신률, 영상 재생률은 별개다.

| 설정 | run22 값 |
|---|---|
| source_sim_fps | 40 |
| flowdeck | true |
| firmware_udp_port | 20150 |
| crtp_udp_port | 20050 |
| camera_udp_port | 15200 |
| cpx_tcp_port | 5000 |
| network_poll_us | 100 |
| state_link | UART, 1,000,000baud |
| frame_association / frame_state_send | true / true |
| state_link diagnostics | false |
| evaluation pacing | enabled, lag_budget_s=0.15 |
| host 시간 제한 | 330s; 실제230.913s에 DONE |
| 호스트 옵션 | --ring-mask 1 --exit-width 1.8 --wall-scan |

source_sim_fps=40은 호스트 수신40fps 보장이 아니다. 입력16.10994fps는 실제 수신 수를 전체 호스트 시간으로 나눈 값이다. 시험에 사용한 포트와 원본 경로는 다른 환경에 전달할 때 별도 맞춰야 한다.

근거: main.py:25–83; commands.py:33–57; runtime_config.json; constraints.json.

### R2.2 허용 prior와 남아 있는 가정

장애물 규격으로 링 중심선1.19m·안쪽1.10m, 출구1.80m를 사용한다. 카메라 사양과 연결된 기체 추정 상태를 쓴다. 코스 순서·개수, 이륙1m, 준비 전진1.5m, 감속1.5초, 정렬 허용치와 속도,0.20m 여유도 정책 정보다.

장애물 세계 좌표·벽 우회 방향·블록 치수·벽 치수·경기장 폭·시작 좌우 좌표·출구 위치는 정책 입력이 아니다. 실제 맵에 입구1.50m와 출구1.80m가 존재하지만 정책은 입구 폭으로 입구를 영상 정렬하지 않는다. 실제 입구 폭1.50m와 준비 전진1.50m는 다른 개념이다.

평평하고 정적인 바닥, 카메라 높이 척도로 사용할 수 있는 EKF 높이, 직립의 비교적 균일한 면, 밝은 원형 고리, 고정된 카메라 모델은 암묵적 환경 가정이다. 완전 무정보·무가정 비행으로 표현하지 않는다.

근거: constraints.json; mission.py:62–73; vision.py:52; passages.py:55; exit_track.py:81.

### R2.3 좌표계

| 표현 | 축·의미 | 사용 |
|---|---|---|
| 픽셀 | u 오른쪽, v 아래 | 영상 검출 |
| Body FLU | X 전방, Y 왼쪽, Z 위 | 입력 속도·각속도와 최종 수평 명령 |
| Heading 좌표 | roll·pitch 보정, 현재 yaw 유지 | 링 광선·스캔 영상 |
| 로컬 추정 좌표 | quaternion 기준 수평축·적분 원점 | 누적 링·벽·출구 후보와 명령 계산 |
| 시뮬레이터 world | 실제 맵 좌표 | 독립 평가·외부 표시 전용 |

body_v=Rz(−yaw)×local_v로 마지막에 명령을 변환한다. 높이는 EKF Z 절대 목표이고, 수평 속도는 기체 기준이다. 상태 quaternion은 xyzw다. body 각속도는rad/s이고 스캔에서는 회전 후°/s로 바꾼다.

heading 변수에 진입 yaw를 저장하지만 yaw_error는 출력에서 쓰지 않는다. 일반 구간 yawrate=0, 스캔에서만 비영 yawrate를 보낸다. 로컬 X 고정 직진이며 링 법선·관측 벽 법선 방향으로 비행속도 벡터를 최적화하지 않는다. WallScan에서 벽 법선은 관측 투영에 사용한다.

근거: mission.py:53–54, :104–107, :125, :221–226; wall_scan.py:45–60.

## R3. 전체 상태기계와 전이의 정확한 의미

~~~text
TAKEOFF → ENTRY_FORWARD → SCAN_BRAKE → SCAN → FIND ↔ ALIGN
                                                           ↓
                                                       STRAIGHT → BRAKE
                                                                   ├─ 다음 링 FIND
                                                                   └─ POST_RING_DESCENT
→ BLOCK_GAP → BLOCK_STRAIGHT
→ WALL_SCAN_BRAKE → WALL_SCAN → WALL_1 → WALL_STRAIGHT
→ WALL_SCAN_BRAKE → WALL_SCAN → WALL_2 → WALL_STRAIGHT
→ EXIT_SCAN_BRAKE → EXIT_SCAN → EXIT → EXIT_STRAIGHT
→ LAND → DONE
~~~

| 상태 | 명령·입력 | 전환 조건 | 코드 |
|---|---|---|---|
| TAKEOFF | 높이=min(1,시작높이+0.22×초) | EKF차<0.07m, 경과>5s | mission.py:62 |
| ENTRY_FORWARD | 링 관측, 로컬vx0.25 | travel≥1.5m | :65 |
| SCAN_BRAKE | 수평0, 기존높이 | 경과>1.5s | :74 |
| SCAN | 링 후보·자세·각속도 | +45/−45/0 완료 후 최근접 후보 전달 | :78 |
| FIND | 현재 링·예측 | 목표 있으면 ALIGN; 없으면 기본 수평0 | :92 |
| ALIGN | 오차·속도에 따른vx·vy·z | full·중심·깊이·속도 조건0.3s 초과 | :101 |
| STRAIGHT | 로컬vx0.7, 목표고정 | travel≥D, 마커0, 링카운트+1 | :132 |
| BRAKE | 수평0 | 1.5s 초과, 링3개 전이면FIND | :139 |
| POST_RING_DESCENT | 높이1m까지0.2m/s 목표 하강 | 오차<0.07m, 경과>3s | :144 |
| BLOCK_GAP | 양앞면·옆면·바닥선, 좌우 정렬 | 중앙·잔류속도 조건0.35s 초과 | :166 |
| BLOCK_STRAIGHT | 로컬vx0.4 | travel≥D → 벽 준비 | :201 |
| WALL_SCAN_BRAKE | passage=None, 수평0 | 경과>1.5s | :156 |
| WALL_SCAN | WallScan.observe와회전 | 스캔완료→pending WALL_1/2 | :160 |
| WALL_1/2 approach | 스캔 선택목표, 전후 정렬 | depth2.8±0.15m, abs(vx_f)<0.12 | :180 |
| WALL_1/2 align | 같은 방향 관측갱신, 좌우 정렬 | 중앙·속도0.35s 초과 | :185 |
| WALL_STRAIGHT | 로컬vx0.4 | travel≥D → 벽카운트+1, 다음벽/출구 | :201 |
| EXIT_SCAN_BRAKE | 수평0 | 1.5s 초과 | :147 |
| EXIT_SCAN | 기둥 누적·회전 | 스캔완료 | :150 |
| EXIT | 폭기반쌍 중앙 정렬 | 중앙·속도0.35s 초과 | :166 |
| EXIT_STRAIGHT | 로컬vx0.4 | travel≥D→LAND | :201 |
| LAND | z=max(0.03,시작높이−0.18×초) | 경과>5s, EKF높이<0.06m | :210 |
| DONE | 다음step이면stop반환 | main은DONE을 보고 루프종료, worker정리에서stop | :213; main.py:89 |

벽 스캔은 WALL_1/2로 들어가는 transition에서 끼워 넣는다. 이미 WALL_SCAN에서 해당 벽으로 넘어갈 때는 다시 삽입하지 않는다. 스캔 전에2.8m 접근을 완료하는 구조가 아니라 스캔 뒤에 approach 단계가 시작된다.

transition은 stage를 바꾸고 stable·travel을0으로 초기화한다. 기존 분기에서 이미 계산한 명령을 새 상태 기준으로 전부 재계산하지는 않는다. ALIGN→STRAIGHT 행은 아직 정렬 속도를 반환할 수 있다. 링 STRAIGHT→BRAKE 행에는 앞서 설정한vx0.7이 남는다. 통로 STRAIGHT 종료는 명시적으로vx=0으로 바꾼다.

따라서 decisions.stage가STRAIGHT인 첫 행과 실제0.7/0.4 송신 시작 시각은 다를 수 있다. 전이 후 travel_m=0을 기록하므로 그 행만 보고 마지막 적분량을 알아낼 수 없다. 실제 송신은 commands.jsonl을 봐야 한다.

근거: mission.py:40–48, :120–138, :201–209; commands.py:51–57.

## R4. 수식: 픽셀에서 거리·명령까지

### R4.1 카메라와 링

~~~text
F = 244 / (2 tan35°) ≈174.23px
CX=161.5px, CY=121.5px
body_ray=[1,−(u−CX)/F,−(v−CY)/F]
heading_ray=Rz(−yaw) R body_ray

depth_ring=F×1.19/(2r)
left=heading_ray.y/heading_ray.x×depth
up=heading_ray.z/heading_ray.x×depth
inner_diameter_px=2r×0.55/0.595
~~~

r은 밝은 띠 중심선 반지름이다. Hough 원 경계에서 중심·반지름을 미세조정해 안팎보다 밝은 띠를 찾는다. depth는 광선 전체 길이와 다르다. 링 원형 투영과 고정 척도에 의존한다. 카메라0.03m는 직진 거리의 가산 항이며 모든 광선 계산에 장착 위치·회전을 정밀 보정했다는 뜻이 아니다.

근거: vision.py:6–17, :19–60.

### R4.2 적분·평활·링 정렬

~~~text
dt=clip(received_t−last_received_t, 0,0.2) [s]
v_local=R body_velocity_flu
position += v_local×dt
v_filtered += (1−exp(−dt/0.35))×(v_local−v_filtered)
target_filtered += (1−exp(−dt/0.18))×(target_local−target_filtered)
travel += max(0,v_filtered.x)×dt

vx_local=clip(0.4×(depth−1.4)−0.8×v_filtered.x, ±0.25)
vy_local=clip(0.4×left−0.8×v_filtered.y, ±0.30)
height_goal=ekf_height+clip(0.5×up, ±0.25)

good=full AND |left|<0.09 AND |up|<0.09
     AND |depth−1.4|<0.15 AND norm(v_filtered)<0.15
각 step에서 good 확인, stable>0.30s → commit
D_ring=depth+0.03+0.20
~~~

position은 평활 전 속도, travel은 평활 후 전진 양수 부분을 사용한다. 후진해도 travel을 줄이지 않는다. target_local은 heading 관측을yaw로 로컬축에 옮긴 값이다. floor_odometry의 영상 특징점 결과는 decision.floor_motion에 기록되지만 position과travel을 대신하지 않는다.

근거: mission.py:52–60, :103–126; floor_odometry.py:15–38.

### R4.3 스캔 후보와 yaw

~~~text
P_ring=position+Rz(yaw)[depth,left,up]
동일 후보: norm(P_ring−P_stored)<0.55m
대표 품질=full+support−0.25×abs(atan2(left,depth))
대표 관측은 더 높은 품질일 때 교체
최근접 후보=min norm(P_stored−current_position)

goal=[+45°,−45°,0°]
error=wrap(reference+goal−yaw)
measured_rate_deg=degrees((R×body_angular_rate).z)
yawrate=clip(0.8×error_deg−0.6×measured_rate_deg, ±30)
|error|<2°를0.25s 이상 유지 → 다음목표
~~~

스캔 중 samples는 같은 후보와 연결될 때마다 늘지만 last_seen은 더 나은 대표 품질로 교체될 때만 바뀐다. 표시 age가 모든 관측 중 최신 시각과 같지는 않을 수 있다. RingTrack의 현재 영상 재연결은 예측 지름비0.7~1.4, 중심차45px 이내다. 예측값은full=False다.

근거: ring_scan.py:25–72; ring_track.py:12–49.

### R4.4 바닥선과 블록

~~~text
수평 보정 바닥 접점(u,v), EKF높이h:
depth=F h/(v−CY)
left=(CX−u) depth/F

front_face: depth_span<0.20×depth
rear_extent: 앞면과 인접한 옆면의 far_depth 포함
block_gap=L_inner−R_inner
block_center=(L_inner+R_inner)/2
block_rear=max(rear_a,rear_b)
D_block=block_rear+0.23
~~~

바닥선은 하단 경계점에 선형fit을 하고 잔차가 큰 점을 제외해 재fit한다. depth는 양끝의 중앙값, far_depth는 최댓값이다. 앞면의 옆에3px 이내로 붙고 가까운 깊이차<25%, 더 먼 끝·깊이차>0.08m이면 옆면 후보로 연결한다. 블록쌍은 적어도 한쪽 옆면 관측이 필요하다.

근거: passages.py:38–112.

### R4.5 WallScan의 벽 평면 추정

먼저 heading_image로 roll·pitch만 보정한다. 이 이미지를 quaternion이 단위인 상태로 solid_faces에 넣어 heading 좌표의 바닥선을 얻는다. 모서리를yaw와position으로 로컬XY에 옮긴다.

~~~text
E0,E1 = heading 바닥 모서리를 로컬XY로 변환한 점
depth = mean(E0.x,E1.x)−position.x
수용: depth>0, ptp(E.x)<0.20×depth, 영상폭≥25px
tangent=(E0−E1)/norm(E0−E1)
normal=[tangent.y, −tangent.x], normal.x≥0 방향으로 정규화
plane_point=(E0+E1)/2

기존 reference가 있으면:
mean(abs((E−reference.point)·reference.normal))≤0.20×depth
AND abs(gray−reference.gray)≤9
수용면 중 현재 depth가 가장 작은 면으로 reference 갱신
~~~

이 reference는 현재 영상과 추정 상태에서 만든 벽 평면이며 맵 정답 평면이 아니다. 평면 일치와 밝기 검사가 영구적으로 정확한 물체ID를 보장하는 것도 아니다.

근거: wall_scan.py:33–59.

### R4.6 회전 영상 열의 평면 교차와 바닥 후보

~~~text
ray_heading(u)=[1,(CX−u)/F]
ray_local=Rz(yaw) ray_heading
lambda=(plane_point−position.xy)·normal / (ray_local·normal)
floor_row(u)=CY+F×height/lambda
plane_point(u)=position.xy+lambda×ray_local

유효: 분모절댓값>1e−8, lambda>0,
      CY+12<floor_row<241, 유한값
~~~

각 열에서 예상 바닥 접점보다2~10px 위의 띠를 살핀다. 7×7 평균 밝기>30 영역을 바닥 외관으로 보고, 검출된 물체 면 영역은 제거한다. 열별띠 평균을폭7로 평활해0.45 초과이면 바닥 후보열이다. 벽 끝에서 밖으로 확장하다6열 연속 미관측이면 중단한다. 관측열7개 이상이면 후보를 만든다.

~~~text
A=벽 끝 열의 관측 평면 좌표
B=바닥 관측 끝 열의 관측 평면 좌표
gap=norm(B−A)
center=(A+B)/2
rear_local_x=max(앞면 모서리 local X)

각 방향 대표 품질=(not clipped, gap) 사전식 비교
스캔 완료 selected_side=대표 gap이 큰 방향
이후 선택방향을 유지하며 해당 관측은 갱신 가능

delta=center−current_position.xy
guidance.depth=delta.x
guidance.left=delta.y
guidance.rear_depth=rear_local_x−current_position.x
D_wall=rear_depth+0.23
~~~

WallScan.guidance의rear_observed는false다. 이름이rear_depth여도 뒷면 관측이 아니다. 방향 선택 후 반대쪽으로 다시 선택하는 루프는 없으며, 안내점이 현재보다뒤(delta.x≤0)이면None을 반환한다. 관측 나이 상한이나 미래 충돌영역 계산은 없다.

근거: wall_scan.py:11–23, :60–118.

### R4.7 출구

~~~text
기둥 바닥 접점 → heading 모서리
local_edges=Rz(yaw)×heading_edges+position.xy
동일 기둥 중심차<0.15×exit_width

두 기둥의 안쪽 모서리A,B:
gap=norm(A−B), center=(A+B)/2
앞깊이>0, |A.x−B.x|≤0.25×앞깊이
abs(gap−exit_width)/exit_width≤0.30
선택: 폭오차 우선, 같으면 현재거리 우선

scale=1.80/measured_gap
[depth,left]=(center−position.xy)×scale
D_exit=depth+0.23
~~~

현재 프레임에서 둘을 모두 검출하지 못해도 기존pair를 유지한다. gap_m은 반환 시 허용폭으로1.8이며 실제 관측간격은measured_gap_m이다. pair_observation_age_s는 두 기둥 중 오래된 관측의나이다. 한 기둥만 보고 반대 기둥을 가정해서 쌍을 생성하지 않는다.

근거: exit_track.py:15–87.

### R4.8 통로 정렬과 실제 송신

~~~text
벽 접근:
vx=clip(0.5×(depth−2.8)−0.8×vx_f, ±0.35)
|depth−2.8|<0.15 AND |vx_f|<0.12 → align

공통 align:
vy=clip(0.55×left−0.8×vy_f, ±0.40)
|left|<0.09 AND |vy_f|<0.10 AND |vx_f|<0.12
조건누적>0.35s → 통과거리commit
링straight local vx=0.70, 그외straight local vx=0.40
travel≥D → 다음상태

최종송신:
body_vx=cos(yaw)×local_vx+sin(yaw)×local_vy
body_vy=−sin(yaw)×local_vx+cos(yaw)×local_vy
height=EKF 절대 Z 목표
~~~

벽approach는 완료 후 align으로 바뀌며 이후 깊이가 달라져도 같은단계에서 자동으로approach로 복귀하지 않는다. 정렬조건은 현재 필드오차·속도 기준이고 미래 궤적의 통과여유 최적화가 아니다.

근거: mission.py:180–209, :224–226.

## R5. 원본 명령 계약과 ToF 수정 범위

송신은 cflib의 send_hover_setpoint(vx,vy,yawrate,height) 및 종료stop이다. 수평은body-fixed m/s, yawrate는degree/s, z는EKF absolute 목표다. 원본 decoder는 mode.x/y=Velocity, velocity_body=true, mode.z=Abs로 설정한다. legacy yaw 부호 변환은 원본 cflib와 decoder가 처리하므로 임의로 중복 반전하지 않는다.

--ring-mask 1에서 송신기는 kalman.ringPass callback을 등록하고 초기0을 요청한다. 정책ring_pass_mask가 바뀌면1 또는0을 요청한다. 요청1에ACK가 아직맞지 않으면 sent.vx와sent.vy를0으로 한다. 종료finally에서파라미터0과stop을 보낸다. --ring-mask 0은 파라미터를 쓰지 않는 원본 비교 경로다.

~~~c
case MeasurementTypeTOF:
  if (!ringPass) kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
  break;
case MeasurementTypeFlow:
  kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
  break;
~~~

위 발췌에서 중간의 다른 측정 분기는 생략했다. 실제 파일은 ToF 분기351–353, Flow357–359다. EKF 전체정지가 아니라ToF 갱신 호출만 생략한다. raw ToF·Flow·IMU·PID·supervisor는 이패치가 끄지 않는다. 원본mm_tof.c의 자세조건과scalarupdate는 존재하지만 “3샘플 패드높이 오프셋” 기능은 확인한패치·소스에 없다.

run22에서 활성화 요청→ACK 지연은 링1 약1.567ms, 링2 약5.362ms, 링3 약3.959ms였다. 이는프레임 수신부터직진명령까지 지연과다르다. 펌웨어manifest의automatic_timeout=null이며호스트가해제하는구조다.

근거: commands.py:21–64; firmware/ring_pass.patch; fresh_ring_pass_v2/src/modules/src/estimator/estimator_kalman.c:351–359, :544; 원본commander.py:167–190; run22/host/events.jsonl.



## R6. run22 실제 로그·명령·결과

### R6.1 로그 스키마와 실제 판단

- frames.jsonl: sequence, session_id, capture_tick_us, received_monotonic, image_file, association_valid, state. state에는 body_velocity_flu[m/s], body_angular_rate_flu[rad/s], quaternion_xyzw, ekf_height_m[m], tof_distance_m[m], 센서 age와 식별자가 있다.
- decisions.jsonl: frame, policy_completed_monotonic, command, stage, ring_index, target, decision, odometry, passage, passage_phase, travel_m, straight_distance_m. decision에는 checks, filtered_target, ring_scan, wall_scan, exit_scan, committed, clearance_margin_m가 단계별로 들어 있다.
- commands.jsonl: sent_monotonic, verdict, source_frame, source_identity, source_epoch, command, ring_pass_ack. 정책 출력과 실제 송신을 구분한다.
- events.jsonl: ring_pass_requested, ring_pass_parameter_ack 및 명령 프로세스 오류 사건.
- physical_score.json: ordered_sections, ordered_events, all_log_nonfloor_contact_samples, lifecycle_complete, host_stage, full_course_success, ground_landing.

아래는 첫 링 직진을 결정한 evidence/run22/host/decisions.jsonl:432의 발췌다. 전이 행의 정책 명령은 아직 정렬 명령이므로 실제 직진 송신과 같다고 읽지 않는다.

~~~json
{
  "stage": "STRAIGHT",
  "ring_index": 0,
  "source_frame": "frame_000612.npz",
  "stage_before": "ALIGN",
  "filtered_target": [
    1.383641267147709,
    -0.07960881481491133,
    -0.0014649779952939138
  ],
  "checks": {
    "full": true,
    "lateral": true,
    "vertical": true,
    "depth": true,
    "velocity": true
  },
  "committed": {
    "kind": "ring",
    "depth": 1.383641267147709,
    "distance": 1.613641267147709
  },
  "policy_command_at_transition": {
    "mode": "hover",
    "vx": -0.0010276281003184415,
    "vy": -0.015495070165479276,
    "yawrate": 0.0,
    "height_m": 0.9829063936393769,
    "ring_pass_mask": 1
  }
}
~~~

수신 시각·판단 완료·송신 시각은 호스트 monotonic 초이고 capture_tick_us와 snapshot_tick_ms는 다른 시간 도메인이다. 서로 다른 도메인을 직접 빼서 지연을 만들지 않는다. 원본 전체 행과 이미지 NPZ는 evidence/run22/host/에서 확인한다.

### R6.2 목표 거리와 측정 변위

아래는 command_outcome_examples.json/.csv의 실제 run22 값이다. 직진 결정부터 해당 상태 종료까지 각 경계 직전의 GT 표본을 사용하고 변위를 진입 시 실제 yaw 전방축에 투영했다. 평가용 계산이며 제어 입력이 아니다. 최대 경계 표본 나이는 약17.5ms다.

| 구간 | 결정→종료(s) | 목표D(m) | 전이 직전 기록 적분(m) | 실제 전방 변위(m) | 실제−목표(m) | body vx 중앙값(m/s) |
|---|---:|---:|---:|---:|---:|---:|
| 링1 | 42.235→45.469 | 1.613641 | 1.601000 | 1.840137 | +0.226495 | 0.699709 |
| 링2 | 54.768→57.958 | 1.592454 | 1.543938 | 1.797745 | +0.205291 | 0.699688 |
| 링3 | 69.291→72.389 | 1.521561 | 1.475859 | 1.592777 | +0.071216 | 0.699684 |
| 블록 | 84.486→94.475 | 3.676330 | 3.659269 | 4.291298 | +0.614968 | 0.399819 |
| 벽1 | 125.414→133.735 | 3.016049 | 3.002768 | 3.548238 | +0.532189 | 0.399193 |
| 벽2 | 168.655→176.777 | 2.931300 | 2.920777 | 3.372401 | +0.441101 | 0.398103 |
| 출구 | 216.312→224.278 | 2.885540 | 2.868617 | 3.289609 | +0.404069 | 0.396171 |

“전이 직전 기록 적분”은 종료 조건을 만족시킨 최종 적분값이 아니다. 종료 step에서 travel을 더한 뒤 transition이0으로 초기화하므로 직전 로그 값을 별도로 표기했다. 이 값이D보다 작다는 이유로 종료조건이 작동하지 않았다고 해석하면 안 된다. 실제 변위는 정지까지의 전체 이동량이나 영상 캡처 순간의 거리 오차가 아니다.

| 구간 | 실제 body vy 중앙값(m/s) | 높이 목표 중앙값(m) | 결정 입력 프레임 | 결정 로그 줄 |
|---|---:|---:|---|---:|
| 링1 | 0.020051 | 0.983639 | frame_000612.npz | 432 |
| 링2 | 0.020762 | 1.480584 | frame_000812.npz | 591 |
| 링3 | 0.020995 | 2.069638 | frame_001032.npz | 767 |
| 블록 | 0.012025 | 1.019272 | frame_001286.npz | 1020 |
| 벽1 | 0.025389 | 1.019272 | frame_001972.npz | 1706 |
| 벽2 | 0.038909 | 1.019272 | frame_002694.npz | 2428 |
| 출구 | 0.055214 | 1.019272 | frame_003489.npz | 3223 |

### R6.3 스캔 실측

| 구간 | 소요시간(s) | 실제 송신 수 | 수평명령 | 최대 yawrate | ringPass 요청/ACK |
|---|---:|---:|---|---:|---|
| 입구 | 16.906441 | 833 | vx=vy=0 | 30°/s | 전부0/0 |
| 벽1 | 18.008979 | 887 | vx=vy=0 | 30°/s | 전부0/0 |
| 벽2 | 17.755836 | 874 | vx=vy=0 | 30°/s | 전부0/0 |
| 출구 | 17.610050 | 868 | vx=vy=0 | 30°/s | 전부0/0 |

합계3,462개 실제 송신에서 수평0과 마커0을 확인했다. 물리 위치가 완전히 고정됐다는 검증과는 다르다. 목표 도달 허용치는2°·0.25초다. 정확한 visited 각도·시각은 scan_speed_validation.json을 확인한다.

### R6.4 벽과 출구의 선택 근거

| 단계 | 실제 관측·선택 |
|---|---|
| 벽1 스캔 종료 | 오른쪽2.326637m, 왼쪽0.327466m → side=−1 |
| 벽2 스캔 종료 | 왼쪽2.330275m, 오른쪽0.486644m → side=+1 |
| 벽1 직진 결정 | depth2.855173m, rear_depth2.786049m, left−0.065811m, 후보나이0.174137s |
| 벽2 직진 결정 | depth2.707743m, rear_depth2.701300m, left−0.020775m, 후보나이0.345178s |
| 출구 직진 결정 | 기둥ID1·2, 관측간격1.815011m, 폭척도0.991730, depth2.655540m, left+0.067291m, 쌍나이6.688194s |

벽2 스캔 종료의 선택 왼쪽 후보는16.207873초 전 누적관측이었다. 이후 정렬하며 갱신됐다. 두 벽의 rear_observed는false다. 출구 gap_m=1.8은 허용폭이며 영상 측정치는 measured_gap_m이다. 근거: decisions.jsonl:1515, :2167, :1706, :2428, :3223 및 selected_observations.json.

### R6.5 독립 평가와 버전 비교

| 실행 | 맵·변경 | 시간(s) | 순서통과 | 비바닥 접촉 표본 | 착륙 |
|---|---|---:|---:|---:|---|
| run20 | 같은 독립 재배치, 벽 스캔 없는 이전 코드 | 271.189 | 6/8 | 15,534 | 실패 |
| run21 | 벽 스캔,20°/s 한계,0.45m 여유 | 255.295 | 8/8 | 0 | 실패 |
| run22 | 벽 스캔,30°/s 한계,0.20m 여유 | 230.913 | 8/8 | 0 | 실패 |

코드 조건이 다른 개발 실행이므로 통계적 성공률로 합치지 않는다. run20의 첫 접촉은 WALL_2 판단 중 측면 경계 fence_1_north_collision과 기체 사이 접촉이었다. run22의 같은 맵 성공은 개선 근거지만 모든 맵 일반화를 입증하지 않는다.

run22는8/8·접촉0·DONE·호스트오류null·lifecycle_complete=true다. full_course_success에는 착륙이 포함되지 않는다. 접지 수직속도−0.483488m/s, 직전−0.467266m/s로 절댓값0.25m/s 기준을 초과했다. 이후0.600초 안정 정지와 모터 감속은 확인됐다. 실패 이유를 “착지를 관측하지 못함”으로 바꾸지 않는다.

구간 평가 모델은 기체 반지름0.12m, 뒤쪽 이탈 추가여유0.08m, 최대 GT 표본간격0.1s다. 착륙은 정지0.2s 이상·속도0.1m/s 이하·기울기10° 이하·이동0.02m 이하·모터100rpm 이하·접지수직속도0.25m/s 이하 등을 함께 요구한다. 평가 실제 좌표는 정책에 되돌리지 않는다.

## R7. 재현·검증·추적 절차

### R7.1 코드와 결과를 연결해 읽기

1. host/summary.json의 전이 시각에서 관심 구간을 고른다.
2. decisions.jsonl의 stage_before, checks, committed와 frame.image_file을 확인한다.
3. 해당 NPZ의 실제 픽셀과 연결된 state를 본다.
4. commands.jsonl에서 source_frame과 실제 송신을 연결한다.
5. events.jsonl의 ringPass 요청·ACK를 별도로 본다.
6. physical_score.json의 실제 순서통과·접촉·착륙을 확인한다.
7. 변위표의 방법과 표본 나이를 확인한 뒤 수치를 사용한다.

소스와 로그의해시는 constraints.json·result_summary.json·패키지manifest에서 확인한다. runtime_state/state.json의 log_dir에는 원래컴퓨터 절대경로가 있을 수 있어 다른컴퓨터에서는 패키지의 대응경로로 읽는다. 기존run18 문서·수치를 run22로이름만 바꿔 사용하지 않는다.

### R7.2 최소 실행 절차

아래는 기존개발컴퓨터 기준 명령이며 이 문서 작성중 새실험을 실행하지 않았다. 다른환경에서는 원본저장소·Python·펌웨어경로를 맞춰야 한다. main.py와commands.py의 원본저장소 절대경로도 확인한다. 동결소스 디렉터리에서 실행해야 run22정책이다. 같은native포트에 다른실행이 있으면 동시에띄우지 않는다.

~~~bash
cd /home/razer/Documents/ChatGPT/Drone3
MUJOCO_GL=egl /home/razer/Desktop/Drone3/Drone/bin/python tools/eval_runtime.py start \
  --run-dir .local/evaluation/run22_repeat/runtime \
  --config .local/evaluation/fresh_mission2/run22/runtime_config.json \
  --max-wall-s 370 --capture-every 0
~~~

다른터미널에서호스트:

~~~bash
cd /home/razer/Documents/ChatGPT/Drone3/.local/evaluation/fresh_mission2/run22/source
/home/razer/Documents/ChatGPT/Drone3/.venv/bin/python -m mission2_fresh.main \
  --out /home/razer/Documents/ChatGPT/Drone3/.local/evaluation/run22_repeat/host \
  --duration 330 --ring-mask 1 --exit-width 1.8 --wall-scan
~~~

호스트종료뒤해당런타임정리·평가:

~~~bash
cd /home/razer/Documents/ChatGPT/Drone3
/home/razer/Desktop/Drone3/Drone/bin/python tools/eval_runtime.py stop \
  --run-dir .local/evaluation/run22_repeat/runtime
.venv/bin/python -m mission2_fresh.review .local/evaluation/run22_repeat
~~~

출력폴더는새경로를사용한다. host생성은exist_ok=False이며이미완료된run22를덮어쓰지않는다. --exit-width 1.5는이시험의재현명령이아니다. 위명령은결정론적으로동일한모든프레임·운동을보장하지않는다.

### R7.3 확인된 검사

run22변경후focused검사35개통과가 run22_tests.txt에기록돼있다. 이원고작업에서재실행하지않았다.

~~~bash
.venv/bin/python -m pytest -q \
  tests/test_fresh_wall_scan.py tests/test_fresh_exit_track.py \
  tests/test_fresh_ring_scan.py tests/test_fresh_ring_tracking.py \
  tests/test_fresh_original_contract.py tests/test_fresh_passages.py
~~~

검사범위는원본hover계약,추적·예측commit제한,스캔회전·수평0,벽관측누적,출구연결,바닥접점기하등이다. 합성·단위검사는실기체일반화와동의어가아니다. 실제스캔시간·명령·코스결과는별도native run22기록이다.

## R8. 한계와자료사용범위

- 실제세계좌표는평가·외부영상용이다. 정책의로컬추정좌표·관측평면을GT로표현하지않는다.
- 링지름·출구폭·코스순서·초기동작·픽셀임계값·제어이득은남아있는사전정보다.
- 통과여유0.20m는고정가산이며실제거리오차20cm이하를보장하지않는다.
- 링·벽·출구누적관측에는드리프트·중복·오대응이가능하다. 관측나이를함께읽는다.
- WallScan은관측한앞면평면과바닥외관을사용한다. 실제벽뒷면·그물없는공간을보증하지않는다.
- 스캔선택방향은유지된다. 모든환경을처리하는재계획·충돌예측플래너가아니다.
- 직진중현재장애물영상으로방향·거리를갱신하지않는다. 속도적분과실제변위차이가남는다.
- FloorOdometry시각화결과가현재travel종료판정의적분원천은아니다.
- 일반단계yawrate=0이며계산한yaw_error는사용되지않는다. 능동진입yaw·링법선추종으로설명하지않는다.
- 현재성공은기존수정펌웨어조건이다.3샘플자동패드오프셋로직이나펌웨어무수정성공을주장하지않는다.
- 호스트DONE·full_course_success와독립연착지판정은다르다. run22는실제과속접지로실패했다.
- 이버전의무작위다중맵반복성공률·그물환경·실기체성능은확정하지않았다.

후속의출구폭1.50m및새벽인지변형은본자료의대상이아니다. 설명용문서·원자료·첨부소스·명령표는모두run22로일치시켜사용한다. 추정편향·필터지연·충돌·센서융합의개별인과기여처럼분해검증되지않은내용은원인으로단정하지않는다.
