# 35×12m, 진입 대기와 포화 시험

2026-10-09 사용자 변경을 적용했다. 이전 50×20m 시험은 보존하고 새 설정은 `config/mission3-35x12-entry-hold.json`, 물리 감도 프로필은 `config/mission3-rpyt-35x12-entry-hold-limited.json`으로 분리했다.

경기장 35×12m, 그물 높이5m, 중심(17.5,6)m. 직선20m와 회전지름7.5m를 유지한 트랙의 전체 외곽은27.5×7.5m로, 경기장 끝 여유는 긴 방향 각각3.75m, 짧은 방향 각각2.25m다. 진입1은(1,2)m/높이1m/정면+X, 다른 진입점도 새 경기장 안으로 옮겼다. 좌표 선택은 실험 설정S이며 주최측 확정 좌표가 아니다.

WAIT15s→ENTRY_SCAN(진입 위치 유지하며8°/s회전). 관측 전 EXPLORE/WATCH/PATROL로 트랙 관측점에 이동하는 기존 정책은 이 프로필에서 사용하지 않는다. 디코딩된 마커의 방위가 생긴 뒤 COARSE_ACQUIRE로 접근하며, 자세·속도 추정이 유효해지면 기존 정렬/직선 확인/접근/착륙 경로를 사용한다. 마커를 잃으면 현재 추정 위치를 고정해 탐색한다. 관측되기 전에 차량 위치를 제공하지 않는다.

명령 포화S: 3D 속력.4m/s, 가속도.15m/s², yaw10°/s, yaw 가속도20°/s², 상승.10m/s/하강.06m/s. 차량.25m/s에 대해 최대.15m/s 추격 여유이며, 정지→최대속력에는 최소2.67s, 이상적 최대속력 제동거리는.533m다. 실기 최대 성능이나 안전성을 확인한 값이 아니다. 실제 강체 운동을 강제로 자르지 않고 상위 명령을 제한한다. ABORT/모터off/하강 근거 소실 등의 즉시 중단은 별도 기록한다. 하위 PID/관성 때문에 실제 운동은 명령과 다를 수 있다.

첫 전체 시험 `artifacts/mission3/physical/20261009-35x12-entry-hold-entry1-seed421`: seed421을 실행 전에 고정, 차량은 처음부터 계속 운동. 9.54s 센서 안정 인계,24.54s 제자리 탐색,309.56s 탐색 시간 초과. 제공 프레임5958개 모두 marker_missing; 근접 광학 envelope로42개 미제공. 디코딩0/유효 자세0/접촉0/모터 종료 없음/성공0. 관측 전 트랙 대기 단계 없으며 실제 최대 수평속력.00568m/s, 진입점으로부터 최대 수평 편차.0967m. 이 편차는 실제 이동 지시가 아니라 오도메트리 오차가 남은 조건부 hover 결과다.

첫 시험에서 인계→WAIT 시점의 명령 가속도.2287m/s²를 발견했다. 단계가 바뀌며 limiter 이력이 초기화된 버그를 수정해 이전 명령 이력을 넘기고 한 제어 주기에서 limiter를 두 번 적용하지 않도록 했다. 광학 GEOMETRY_READY→실제 TOUCHDOWN_DESCEND 경계도 중간 zero 대신 이전 실제 명령을 사용한다. 전체 시험 원본은 수정하지 않았다. 수정 후 별도35s 경계 물리 회귀 `20261009-35x12-limit-boundary-regression`은 실제 인계/WAIT/ENTRY_SCAN을 통과했고 일반 명령 가속도 최대.150000m/s²·yaw가속도20°/s²로 포화 감사 통과. 의도적으로 설정한35s 시간 종료이며 착륙 시험 성공으로 세지 않는다. 종단 착륙 경계는 이번 실제 비행에서 도달하지 않았다. 전체 단위·물리 경계 검증155/155 통과.

첫 전체시험 영상은 `physical-review-h264.mp4`:1280×720, 5FPS 표시,1배속309.6초. 인식 입력은 원래mono320×320/20FPS이며 표시용 시간 샘플링은 인식 성능을 바꾸지 않는다. GPU EGL/CPU spawn4개 프로세스가 같은 원시 실행의 서로 겹치지 않는 시간 구간을 렌더링했다. 1548프레임이 누락/중복 없이 이어졌고 h264/yuv420p/faststart 재생과 길이를 확인했다. 초기시험 영상에는 수정 전 인계 포화 초과가 포함되어 있다. `video-timing.json`, `video-validation.json`, `command-limits-audit.json`에 근거를 보존했다.

오프라인 실제 자세만으로 계산한 조건부 광학 진단: 최근접 마커 거리3.984m, 마커 정면과 네 모서리가 공칭 화각에 들어오는 구간에서 코드 짧은 변 최대6.488px. coarse 기준7px/metric 최초 기준18px보다 작다. Brown 왜곡의 공칭 각도 범위를 벗어난 접힘 투영을 제외했다. 이는 실제 픽셀 디코딩 결과나 실제 카메라 보정값이 아니라 감도 모델D/S이다. raw projection 진단도 별도 보존했다.

위 첫 보고 시점에는 전체 시도1회/성공0이었다. 2026-10-09 04:01 기준 새 조건의 완료 전체 비행은34회/접촉 성공0이다: 최초 entry-hold1, raster v1 3, sector v2 3, 불확실성 관측 v3 4, 녹화 검증 재사용2, 탐색폭 v4 6, 첫ID시선 v5 3, cell8 디코더 v6 6, 회전 중 접근 방향 v7 6. 서로 다른 정책과 재사용 seed이므로 일반 성공률이나 독립34회 표본이라고 해석하지 않는다. 별도35s 경계 회귀1회는 착륙 성공0이고 전체 미션 분모와 구분한다. 진입2~8로 확대하지 않았다. 이전 모델 성공 수와 합치지 않는다. 기체 조립 질량·CoM·관성, 카메라 K/D/외부변환, 광학 품질/노출/rolling-shutter·덱 잡음·라디오 timing은 여전히 미확정이므로 제조사 완전 일치 검증0이다.

탐색폭 v4의6회는 접촉0/모터 종료0, 원시 프레임32806장 모두 source SHA256과 일치했다. 전방8m 두 조건은 위치 불확실성1m로 중단, 한 조건은159초에4프레임의 decoded ID만 얻고 자세 추정 없이 시간 초과했다. 측방±1.5m 세 조건은 인식 없이 시간 초과했다. 넓은 탐색 자체를 해결책으로 채택하지 않는다. 알려진 마커 중심 높이와 native 영상 모서리/바닥 ToF/IMU로 상대 거리를 따로 분석했지만 절대 위치·yaw 개선이나 착륙을 검증한 것이 아니다. 상세 수치·가정은 acquisition-research-20261009.md에 있다.

03:22 후보 v5는 forward8 조건에서 첫 decoded ID에 잠깐 제동/시선 유지만 한다. 기존 두 ID 방위 확인과 metric pose 기준은 유지하고, 미확인 영상이 .35s 내 갱신되지 않으면 원래 탐색 경로/시계를 이어간다. .35s는 실험S이며 탐색폭 v4를 해결책으로 승격한 것이 아니다. 전체182tests 통과 후 재사용 training421/433/439를 사전 계획해 병렬 실행했다. 완료3개 모두 불확실성 중단/접촉0,433은 단1프레임인식 후 확인 실패였다. 무인식421/439의7개 원로그는 v4와 byte동일하다. v5도 해결책으로 채택하지 않는다.

v4/v5의433 원본 픽셀 비교에서는 마커가 화면 안에 보이면서도 인식 여부가 다르다. 코드 짧은변9~10px/6module는1.5~1.7px/module D로, 광학·픽셀 샘플링·처리 설정 민감도를 검토해야 한다. 이 수치를 보편 인식 한계라고 주장하지 않는다. 새 작은코드 디코더 비교는 무손실 원본 전체 분모와 잘못된ID/조건부projection불일치도 포함하며, 완료 전후에도 live설정을 임의 변경하지 않는다.

cell8/SUBPIX v6 후보는 seed433에서 두ID 확인 시점을159.28→159.14s로 앞당겼고 coarse/gaze각6/5프레임을 얻었지만 metric0/검색 중단, 다른 네 조건은 불확실성 중단으로 끝났다. 여섯모두 접촉/off0/명령포화pass/원본29818프레임SHA전부일치. 디코더 설정만 바꾸어 착륙에 성공했다는 결론은 없다. 실패 당시 센서/명령·별도 실제기하 평가에서39°시선 정렬 동안 상대 수평거리가 약2.80→3.85m로 벌어졌다. truth는 원비행에 제공하지 않고 끝난 뒤 평가만 했다.

03:52 이후 v7 연구는 fresh decoded 영상이 있고code7~18px/maxside<36px/시선오차<40°일 때만 회전과 이동을 병행하는 정책이다. 목표 turn속도.4m/s도 원래 전체.4/.15 포화를 통과한다. 기체정면 vs영상마커방향의 두프로필만 서로 비교한다. v6 대비40°/turn목표속도 변경은 복합정책으로 명시했으며 direction효과와 혼동하지 않는다. 원base1.5m/path4m/duration20s/loss5s/std1m/pose/contact기준은 유지하고 LK추적·높이거리·gyro모델은 live미연결이다. 사전6training모두 종료: 두방식의433각5/6decoded·metric0·검색 시간 초과, 나머지 네 조건은ID0·불확실성 중단. 접촉/off0/6, 원본29818프레임SHA전부일치·원포화6pass·60core파일/라이브러리/configwithinpolicy동결검사통과. v7도 해결책으로 채택하지 않는다. 433 early-body는인식을160.38s까지유지했지만관측소실뒤유효추격이없었다. 전체 실패 결과는20261009-coarse-direction-training-v7/summary.json에 보존했다.


녹화 정정: 이전 camera.avi는 mono320 해상도를 유지했지만 MJPEG 손실 압축이었다. 실시간 제어기는 압축 전 mono8를 사용했으므로 당시 live 인식 실패는 그대로 유효하다. 그러나 이전 저장 영상의 대체 디코더/배경 회전 분석을 원본 픽셀 재생이라고 부를 수 없다. 새 FFV1 녹화2개는 각각5959/5041프레임을 source SHA256과 대조해 전부 byte 동일을 확인했다. coast439 재실행은 명령·센서·추정·관측·물리 궤적7개 원로그도 이전 실행과 byte 동일하다. FFV1은 보관 코덱이며 실제 AI-deck 전송 사양이 아니다.

무손실 entry-hold421에서 디코더6개를 다시 비교했으며 목표ID 모두0, C0 설정의 잘못된ID57개였다. 전방카메라 배경 회전 관측도 영상 간격/장면에 따라 gyro 차이가 달라져 현재 미채택이다. 자세 추정에 실제 정보가 생기는지 검증하며 인식 문턱이나1m 불확실성 한도를 단순 완화하지 않는다. 접촉 성공 뒤 제3자 관점의 평가 조건을 별도 검사하며 자신의 검토를 다른 에이전트의 독립 서명으로 표시하지 않는다. 상세 근거와 최신 실험은 acquisition-research-20261009.md/work-log.md를 따른다.


04:45 새 연구 결과: ENTRY_SEARCH에서 추정 기체 정면 반대로 이동하는 목표 속도를 .05m/s로 제한한 v8의 재사용421/433/439 training 3개 중2개가 실제 접촉/모터 종료/3초 유지/직선 착륙을 통과했다. 내부 착지 오차1.82/1.45cm로9점이다. .10 설정은 같은3개에서 접촉0이다. .05의433에는 Flow-loss 명령 이력 덮어쓰기로 정상 가속도.321m/s²(한도.15) 위반1회가 있다. 따라서 두 접촉 중 정상 command-envelope까지 통과한 것은421의1회뿐이다. 전체6분모·네 실패·433의포화 실패·조건부 모델 제한은 search-reverse-training-v8/summary.json에 보존한다. 전체 새조건40완료/contact2/독립0/제조사qualified0로, 진입2 진행 기준은 충족하지 못했다.

Flow와 정확 ToF 관측은 실제 모델에서도 말기에 사라졌고, 두 착륙은 조건부 IMU support 전달을 통해 완료했다. 실제 펌웨어/HAL/radio/센서 장면/접촉/기체 파라미터 미확정 조건은 그대로이므로 실기 성공이라 부르지 않는다. v9는 이력 보존 감속을 opt-in으로 수정한 별도 source/config다.194tests 통과 후421/433 회귀 중이며, 두 command-envelope pass 뒤에만 사전 고정 unused457/461/463/467/479를 실행한다. 새 profile은 config/mission3-35x12-search-reverse005-flow-slew-v9.json, physical profile은 기존 limited와 동일. default entry-hold 자체를 v9로 바꾼 것이 아니다.

04:50 v9 회귀421/433 모두 contact+명령포화 pass,7854원프레임SHAexact. 독립/qualified0. 이 고정source/config의 unused holdout457/461/463/467 병렬 실행을 시작했고479는대기 중이다. 결과는 regression-summary.json, 후속 전체5분모를 유지한다. 진입2~8은아직진행하지 않는다.

05:09 v9 전체7개종료. 훈련/회귀2/2와새holdout1/5(457 내부7점/3.31cm)조건부contact+포화pass를분리했다. 네실패 모두고정18px 위치관측기준에투영크기미달;geometry분석은오프라인전용이다. source33907프레임SHAexact/포화7pass. Qualified0,독립검토결과대기,진입2~8미진행. v10 reverse005+fresh40도조기접근 axisbody/bearing의6training을사전고정하고현재4개물리프로세스로실행중이다. 새상태/담당/후속조건은work-log최신항목에기록했다.

05:30 v10 6training 전부 종료: body/bearing 각1/3접촉이며 성공한433 원로그는 v9/433과 동일한 retention 재현이다. 실패467/479에서 새 위치 관측/착륙 없음. 포화6pass/native24022 SHA exact. 전체원장53full이나 서로 다른 정책·재현을 독립 성공률로 합산하지 않는다. 기존 담당의 실제 독립 검토 진행/제조사qualified0/진입2 미진행. 첫 발견 전의 탐색 시선·높이는 별도 오프라인 기하 민감도 분석 중이며 실제 입력/비행/인식 문턱 변경이 아니다.

05:45 최신: v9 independent conditional contact3 승인(제조사/실기/formal0). v11은 WAIT/초기scan/height1 유지,이동탐색 시선만 initialentryheading으로 고정.4spenttraining중3contact+caps pass,457은 접촉해도off없어실패.13818원픽셀SHAexact/198tests. source·split혼합없는새holdout5를고정실행중이며전체결과는아직미완료다.전체완료원장57full. 조건부 새479착륙1×영상과독립피드백·실제Flow/ToF입력계약문서는work-log최신항목참고. 진입2미진행.

05:58 확정: v11 unused5조건3contact+caps pass/2ID0timeout,24051nativeSHAexact. 원장62full.재사용training3/4와분리한다. 실제제조사/실기/formal0/진입2없음. 독립담당은새v11검토중이다. 새source v12는 onboard raw ToF4cm softwaregate의4paired시험을진행중이며200tests/63core동결;v11 control+별도physicalprofile을쓴다. 현재실행PTY와후속작업은work-log최신항목을따른다.

06:26 추가: v12 근접ToF gate4훈련3/4로v11대비성공증가0,원물리3pass는byte동일이었다. v11실제독립검토training3/4 및unusedholdout3/5각승인,qualified/formal0. v13초기scan45초를15/30으로비교한spenttraining12에서는5/6 대4/6contact+정상명령pass,원strictgates/phase/화각/native320/차량운동/caps동일. 탐색15초후보의unused5새조건검증진행중이며부분률미보고. 사전계획/모든실패/독립검토범위·1배속새541영상은search-scan-duration-20261009.md/work-log최신기록에있다. 제조사미확정파라미터·센서/접촉모델근거부족으로진입2~8은아직미진행.

06:30 완료 정정: 앞서진행중인 v13scan15新5 holdout은전부완료해2/5contact+정상명령pass이고3fail(위치관측0두개/접촉하지만off없음하나)다. 전체83full원장에추가했으나source/훈련/holdout/재사용을일반성공률로합산하지않는다. 원source·scene·strictgates·cartphase·포화/native픽셀조건동일. 새541 전체1배속영상은훈련성공한사례일뿐이고새조건평가2/5와구분한다. 新v13training/holdout독립검토는기존담당이진행중이며qualified/formal0,entry2~8未進行. 현재활성/次연구는work-log最終項참고.

06:46 KST 추가: v13training 독립 conditional scan15 5/6·scan30 4/6 확인 및 원 평가 결합 검사 완료. qualified/formal0으로 진입1유지. v14는 원scan15·원물리 profile에서 scalar range/IMU velocity model .8/1.2초만 선택적으로 변경한 12개 spent-development 사전고정 시험이며 현재 진행 중. 기존 83fullcompleted에 새 성공을 합쳐 성능 주장하지 않는다. 자세한 근거는 terminal-vertical-fit-20261009.md.

07:06 KST 최신: v14 candidate `config/mission3-35x12-entry-scan15-imu12-v14.json` +원limitedphysical profile. training6/6/holdout2/5 별도, 新전체physics100·qualified0. 新v15 forward6 vswide10x3 config18spentdevelopment현재실행(PTY37941), 원측정/flow/ToF/stockmono320/포화/scan15/scene/차량phase/판정불변, 사전관측track waiting없음. 전체18 aggregate까지source65/configfreeze, 완료재실행금지. 원v14training+holdout 실제independentreview pending(existingHeisenberg). terminal-vertical-fit-20261009.md 및 raster-coverage-20261009.md 참고.

07:28 KST 최신확정: v15경로비교전18의forward6 2/9/wide10x3 4/9는원positive619를모두잃어둘다retention미달·채택없음. 원v14imu12 config8/.75 유지. v14실제독립training각6/6·holdout2/5는각source/split대로승인,qualified/formal0. v16현재source미래IMUguard수정/204tests;spent3회는원2성공1실패와10raw파일모두byteexact,새효과성/실기성공아님. 현main모든physics종료·전체원장121full/진입1,기존reviewer만v15→v16검토중. 이후시험은새plan/sourcefreeze/wholedenominator로정의할것.

2026-10-09T07:33:56.311850+09:00 최신: v17은 기존8/.75 raster에서 거친 방위 기반 접근 옵션을28 paired 재사용 비행으로 비교 중이다. launcher83569/최대4physics,core65/config/runtime는 전체28/aggregate 종료까지 동결. 미완료를 합치지 않고121full 유지, 기존 담당이 실제v15→v16독립 검토중이다. 원영상/실패/source/S/U는 보존한다.

### 2026-10-09 07:56 KST — v17 전체 완료·기준 유지, v18 실패 전달 회귀 진행

v17 PTY83569 launcher와14956 aggregate,35407 phase 분석을 모두 실제 종료/회수했다. paired spent28회 전체 baseline8 8/14와coarse40bearing035 8/14, contact+normalcaps16/28, allcapsPass/native116183 exact, source62/runtime10/plan65불변. 완료full121→149. 8positive retention은 유지했으나 strictimprovement0이라 selection.json에서 후보거절/기준8/.75 raster와기존coarse정책 유지/no unused holdout. 새독립서명은 아직0, 기존Heisenberg에게 whole28 검토를 맡겼고 허용writes는 v17 리뷰 폴더/문서만이다. 후보601은 기준timeout에서 std1m 초과ABORT로 바뀌었으며 조건부성공률8/14를 모집단성과로 주장하지 않는다. phase별 원진단은 20261009-coarse-bearing-paired-training-v17-phase-analysis/summary.json.

v17집계 완료 후에만 v18 source를 적용했다: abort_delivery.py/test_abort_delivery.py 추가와 physical adapter 실패 전달 분기, runtime/config/landing gates 변경0. 209개 시험5.162s 통과. command.done을 실패 이유와 함께 고정하고 0속도/yaw 명령을 PID→RPYT→모터→physics에 공급하는 bounded 관측 경로다. 공중STOP 없음, 성공조건 변경없음. stationarity .05m/s/tilt10deg/gyro5deg/s/.5s dwell/age+gap30ms/max3s는S이고 실제안전회수/모터정지 증명아님. nonfinite는PID입력차단을 별도기록하며 wall/floor/tilt/support실패harness는완료관측으로세지않는다.

현재 main 소유 활성PTY25413는4물리 worker의동결회귀: baseline571/619 성공 보존, baseline613 search timeout 전달, wide10x3/619 odometry uncertainty 전달. 초안617도timeout이므로 실제 다른실패경로인 기존wide619로사전계획을명시정정했다; 새결과를보기전선정. plan: acquisition/20261009-abort-delivery-regression-training-v18/plan.json, code64+기존dependencyconfig3=67/runtimes10까지core77과사용config3을고정. 완료전집계153으로늘리지않음. 그후positive2 fullraw10filesbyteidentity/negative2 original firstABORT전prefixidentity/latchedreason/wire/PWM/physics 전달·sensor연속종료를별도감사해야한다. 기존qualified/formal/hardware/entry2=0, deadline09:00KST 유지. renderer0/newagent0/외부인터넷변경0.

### 2026-10-09 08:05 KST — v18 실패 전달 통합 오류 보존, v18b 재시험

v18 4회 PTY25413와aggregate18893/rawaudit95422 모두끝났다. 완료full149→153, contact2/4/allcaps4/native21148 exact, positive571619의raw10개파일은각기기존v17 baseline과byte동일. 그러나 negative613/와wide619의별도실패전달계약은 FAIL: 처음ABORT전9개rawprefix는원과동일이나 pending분기뒤의독립 `if not handoff_active`가 mission.update를계속호출하여매50Hz시각새AbortDelivery를초기화했다. 실제zero명령/wire/physics는진행했으나dwell/카운터/시작시각이리셋됐고3초상한도지키지못했다(원ABORT후75.4/88.42s까지harnessloop끝). 원자료와각false check를 failed-delivery-regression-audit.json에보존; 명령전달계약이통과했다는주장없음. 이결함은main구현에서생겼고수정한다.

실제physical adapter를controlled I/O로실행하는새2시험은원결함에서미션550회호출(기대1)로둘다실패했다. 초기패치를미션갱신가드 `if not handoff_active and abort_delivery is None`으로고친뒤stationary시험은pass,3초상한시험은floataccumulation으로3.021s여서fail. max3s결정에만1e-9초clockroundoff허용을추가했고landing/uncertainty/vision/support문턱은불변. 최초·중간실패로그와새전체211tests/5.210s pass로그를모두보존했다. 이는realwire/RTOS시험이아닌실제adapter의mock-I/O 통합회귀다.

현재main소유활성PTY89132, renderer0, 새agent0. acquisition/20261009-abort-delivery-regression-training-v18b/plan.json에같은4cases/원reference/fullraw·prefix·wire+physics·namedboundedstatus 판정을사전등록하여4독립물리worker재시험을시작했다. source65+dependencyconfig3=68/runtime10까지78개corehash와사용config3을끝까지동결한다. 완료전157로집계하지말고코드/config/runtime변경금지. whole→summarize_mission3_search_reverse.py 한번→audit_mission3_abort_delivery.py 새out→성공/실패엄격대조. 기존Heisenberg는v17whole28(조건부16positive/12negative)독립검토중;아직본인서명완료를추정하지않는다. v18/v18b 실제검토는둘의모든결과가끝난후동일담당에게부탁한다. qualified/formal/hardware/entry2=0.

### 2026-10-09 08:17 KST — v18b whole4 감사 통과, v17 독립 검토·픽셀 기하 완료

v18b mainPTY89132/aggregate31031/rawaudit42897 모두실제종료, source65+dependencyconfig3/runtimes10(core78)/사용config3동결보존. 완료full153→157(최초v18실패4도포함,자료삭제0). whole4 contact2/4/allnormalcaps4/native17872 exact; 새2positive의원raw10files전체각기v17referencebyte동일. Negative613timeout/wide619uncertainty는원firstABORT전9rawprefixbyte동일, 원failure고정, wire273/115와motor/physics2721/1141 이후2.721/1.141s에서floor Flow/ToF/gyro/속도조건의.52s연속sensorstationary관측으로종료했다. 全predeclaredchecksPass; motoroff/안전회수/실기/새landing능력승인아님. 실제원failure실패와프로토콜검증통과를구분한다. 도식은 acquisition/20261009-abort-delivery-regression-comparison-v18b/abort-delivery-comparison.png(6panels/units/legend/범위view_image QA통과,2D Axes3Doptional warning만있음)과provenance/visual-qa. plotPTY39114회수완료.

v17 실제Heisenberg report/index/final은08:08완료;mainbinding62457을회수하고원SHA/new서명16positive/12negative결합검증완료. 原2876/이전서명47불변이며whole28 arithmetic/FSM·supports·pixels근거에제한;latent전체firmware/perception재실행아님. 두정책8/14같음과retention8/strictimprovementfalse/candidate거절/noholdout를독립확인. 소스서명은원62/runtime10/plan65이며v18bsource로대체안함.

nativegeometry mainPTY83684도whole28/4CPU로완료·회수. 양쪽negative6전부frontface+fullyprojected+focus조건의maxshortedge 약5.47/14.06/7.35/13.25/11.26/6.43px,18px이상후보frame0. detector단독수정으로메트릭성공을보장할관측기하가아니다. 이진단은truth를후평가projection에만썼고occlusion/PSF/rollingreadout을배제못하므로실제인식확률증거아님. 원AVI+cameraSHA/원자료는변경0. acquisition/20261009-coarse-bearing-paired-v17-native-geometry/summary.json와copiedhelper보존.

새D/S분석 acquisition/20261009-coarse-pursuit-budget-ideal-v1/summary.json은manufacturermono camera datasheet의87deg/native320·사용자code.15 기반pinholen168.6을썼다. frontal7→18px의깊이간격2.208m인데constantstraight cart.25/initialdrone.15/accel.15/coarse4m path/20s일때drone.35상한의idealclosing1.048m(.4상한가정1.370m). 사례의거절원인전부를이식으로단정하거나모션/vision/uncertainty문턱확대를승인하지않음. opticalsource를productpage에서camera datasheet로정확히정정했고web원문Rev1을읽어87/87/115와rollingShutter/active324/effective320을확인했다. 현재instantglobalframe/readoutPSF가S/U라는범위유지.

현재mainphysics/renderer/analysis 모두0,본인이시작한PTY모두회수. 기존Heisenberg만v18+v18b whole8 source/positive4/negative4 independent검토중(write범위v18 unifiedreview문서/폴더), 아직실제final/index가없으면서명완료나승인수추정금지. source65 및 current imu12 baseline8/.75 유지;coarse40candidate거절상태. 다음heartbeat는실제review결과hash검증을먼저하고(1)독립검토가반복지적한future range/fallback 전체입력chronology 계약개선이나(2)현재metric18기하에못오는coarse관측을height-bearing geometry/관측가능성으로offline검증하는방법을선택해진행한다. 기존height_bearing_geometry.py는research-only/no livefusion이므로임의로cov축소/weakrange를정밀pose로승격하지말기. stockcamera/초기1m/entryWAIT15/scan15/continuouscarphase/strictgates/caps/전체분모/외부인터넷mutations전질문은고정. 09:00이후새실험금지/자기실행정리/최종보고/automation3-10-9-9삭제. qualified/formal/hardware/entry2는0.


2026-10-09 08:33 최신: 완료 물리 원장161(서로 다른 source/정책/분모, 성공률 합산 금지). v19 same4valid-input 시간계약 회귀2contact/2fail·normalcaps4/native17872exact·원v18b와10raw×4byte일치;217tests. v19 actual독립 검토는 기존담당 진행중. v18/v18b whole8actual검토4positive4negative와mainv2 hashbinding 완료, mutable 참고문서1개 변경을 실행원자료불변과 구분 공개. height-bearing14 진단은 research-only/no liveadoption이며 작은 마커 실패3건 relative거리정보만 생김;worldpose/velocity/착륙확정 아님. 현재 own physics/render/analysis0/모든PTY회수, 기존reviewer만활성. qualified/formal/hardware/entry2=0. 자세한 현상·원자료·다음순서는work-log 최신08:33항목.

2026-10-09 08:56 최신: 완료 물리 원장165, v19 실제 독립 whole4검토와main원해시결합 완료; 발견된 내부history 부분변경 문제는v20 preflight로수정했다. source67/core80·config3동결의same spent4 v20whole회귀contact2/negative2·normalcaps4·17,872nativeexact·전10raw×4가원v19와byte동일,219CPUtestsPASS. 새효능샘플0/qualified0/entry2미진행. main실행전부종료·회수;기존Heisenberg의v20CPU patch실제최종검토만대기중(새contact서명0). 새session-report-20261009.md에제어·분모·실기차이·남은문제·과거source1×영상모음. work-log08:55상세/09:00정리·최종보고·automation3-10-9-9삭제예정.

2026-10-09 09:00 최종: session-report-20261009.md와work-log09:00항목참고. 예약3-10-9-9삭제성공/현재실행0/새실험종료. 원장165·219tests·v20whole4정상입력회귀완료·실제CPU패치독립검토와main결합완료, v20새접촉독립검토대기·qualified0·진입2이후미진행.
