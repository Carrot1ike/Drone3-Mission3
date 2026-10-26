# 인식 실패 개선의 근거와 적용 한계

## 확인한 출처

- **P 제조사**: [Bitcraze AI-deck monochrome camera Rev1](https://www.bitcraze.io/documentation/hardware/ai_deck_mono_camera/ai_deck_mono_camera-datasheet.pdf),2026-10-09 확인. 유효320×320mono, rolling shutter, 초점35cm/DOF30cm~무한, 공칭H87°/V87°/D115°. 자동 노출/게인 기능이 있지만 실제 조도·노출시간·지연·장착 보정은 문서 값만으로 확정하지 못한다. 시뮬레이션20FPS는 센서 최대 성능 주장이 아닌S 타이밍 설정이다.
- **C 공식 소프트웨어**: [OpenCV4.13 ArUco detection](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html),2026-10-09 확인. 사각 윤곽 후보를 찾는 과정과 내부 비트를 검사해ID를 확인하는 과정은 다르다. 작은 윤곽/threshold/코너 보정 설정을 시험할 수 있지만 rejected candidate는 확인된 목표가 아니다. 내부 canonical sampling을 늘리는 것은 입력에 없던 비트 정보를 새로 만들지 않는다. 새 코드는 원본320×320 입력을 유지한다.
- **논문**: Keipour 등, [Visual Servoing Approach for Autonomous UAV Landing on a Moving Vehicle](https://arxiv.org/abs/2104.01272),2022개정/Sensors22(17),6549. 영상 공간에서 속도 명령을 계산하는 이동 플랫폼 접근·착륙 방법을 다룬다. 저해상도 초기 마커 발견 문제나 stock 전방카메라/20cm 작은패드/Flow deck 움직이는 바닥 전환이 자동으로 해결된다는 근거로 확대하지 않는다. 본 프로젝트 적용 전 본문 가정·카메라 방향·시험장비를 확인한다.
- 본문을 확인한 적용 한계: [3.1/3.3/3.5/4.1](https://arxiv.org/html/2104.01272v2). 도로 위 hover점, 차속·방향의 사전 획득, 짐벌,640×360카메라, 레이저3개, 자석을 이용한다. 도로 대기/짐벌/추가센서/자석은 사용자 조건에 적용하지 않는다. 마지막 blind 구간30~70ms라는 설명을 여기서6sIMU bridge의 실기 근거로 확대하면 안 된다. FF+영상피드백·직선에서만 착륙이라는 원리만 참고하며 영상기반 차속 추정은 따로 검증한다.
- **논문/인지 원리**: R. Bajcsy, [Active Perception, Proceedings of the IEEE76(8),1988](https://people.eecs.berkeley.edu/~yang/courses/cs294-6/papers/Bajcsy.active%20Perception.pdf). 수동 영상 처리뿐 아니라 관측의 위치·방법을 능동적으로 선택하는 관점이다. 입구 기준의 제한된 연속 탐색이 반드시 성공한다는 정리나 드론 포화 숫자의 근거는 아니며, 구체적 패턴·속도·안전조건은 아래의S 설계와 시험으로 평가한다.

## 현재 판단과 시험 순서

첫 전체 시험은 유효 이미지5958장 모두 디코딩 실패다. 코드의 기하학적 짧은 변도 조건부 모델에서 최대6.49px였으므로 영상에서 더 많은 정보를 확보하는 문제가 먼저다. 4×4 코드와 한 셀 테두리는 한 변6셀인 구조C/D이고,6.49px은 평균 약1.08px/셀이라 위치·축소 sampling에 민감할 것으로 예상한다. 이 수치로 보편적인 최소 인식 거리를 단정하지 않는다.

1. 원본 전체 영상에서 미리 고정한 디코더 후보들을 CPU 병렬 재생한다. 변경 가능한 소프트웨어만 바꾸고 소스/입력해시/모든 검출ID를 기록한다. 입력 확대·장면 변경·차량 초기 위치 선택은 하지 않는다.
2. 목표ID 검출은 다른ID 오검출 및 별도의 오프라인 투영 일치 기준과 비교한다. 목표ID가 틀린 곳에 생긴 결과를 성공으로 세지 않는다. 디코딩 없이 metric pose 또는 차량 FF를 만들지 않는다.
3. 후보 검출과 ID 확인이 부족하면 이미지 가시성 향상을 위한 제한된 능동 탐색을 비교한다. 트랙 대기점으로 맹목 이동하는 방식은 제외한다. 목표가 확인되지 않은 사각형에 접근하는 정책은 별도 오검출 반례·순간적인 관측 소실·망 경계·불확실성 검증이 먼저다.
4. 확인된ID와 유효 상대 상태가 확보되면 시간지연 보상 FF+피드백, 차량 뒤 일직선 정렬, 직선 구간 여유 확인, 접근, 낮은 하강속도, Flow/ToF 바닥 전환과 제한된 예측, 실제 접촉/off/3s 확인을 시험한다.

문헌 원리와 여기서 선택하는 탐색 폭·문턱·이득S를 구분한다. 실측하지 않은 사양은U로 남기며 제조사 완전 일치 성공으로 승격하지 않는다.

## 01:40 저장 영상 병렬 재생 결과와 능동 탐색 후보

`artifacts/mission3/acquisition/20261009-native-detector-bench-v1`: native 해상도 camera.avi5959장(실시간 처리5958+종료 시점 미배달1장)을 설정6개/CPU4프로세스로 재생했다. **02:30 정정: 이 AVI는 MJPEG 손실 압축이어서 원본 픽셀 검증이 아니다.** 현재subpix/보정없음/작은threshold C3/C0/C7/AprilTag corner 모두 목표ID0회. C0은 다른ID15회 오검출. 저장파일해시·계획·버전·검출위치·투영매칭 기준을 고정·기록했다. rejected 사각형은113945~514009개로 존재 자체가 목표라는 근거가 되지 않는다. 당시 결과와 원본은 보존하며 아래 무손실 재검증과 구분한다.

소프트웨어S 후보 `config/mission3-35x12-entry-raster-v1.json`: WAIT15s→ENTRY_SCAN45s(공칭8°/s×45s=한 회전)→ENTRY_SEARCH. knownentry/정면 기준 앞으로0~4m, 좌우±.75m 범위의 모서리를 이어 계속 움직이며 영상 검색을 유지한다. 이동속도.15m/s, 기존.4m/s/.15m/s²/10°/s 포화 유지. 트랙 중심/관측점/차량의 위치·phase로 목표점을 만들지 않는다. 실제 추정 위치와 알려진 arena의1m margin으로 경로를 제한한다. 정해진 좌표에 도착해 대기하는 WATCH단계는 없다. 판·코드·기체·화각·차량속도·초기차량phase를 바꾸지 않는다.

4m전방·.75m측방은 탐색 범위를 넓혀 원래4m이상 거리의 작은 코드를 더 큰 픽셀로 관측할 가능성을 시험하는 값S이다. 평균.15m/s로4m전진에 약26.7s가 필요하며 waypoint연결·가속포화로 더 오래 걸릴 수 있다. 알려진 진입 방향을 쓸 뿐 목표 방향을 안다고 가정하지 않는다. yaw는 천천히 계속 돌며, 목표ID를 확인한 뒤에만 목표 접근으로 전환한다. coarse마커 관측 후 손실이면 현재 추정 위치에서 다시 회전하며, 관측 전 탐색 경로로 임의 복귀하지 않는다. 유효 자세 획득·차속 추정·근접 정렬·접촉 성공은 별도 통과해야 한다.

무영상 조건에서 한 회전 이전 이동없음, trackprior를 바꿔도 탐색점 동일, 모든 진입 방향에서 netmargin내, 관측손실 후 raster재개금지 시험을 추가했다. 첫 접촉 성공을 아직 주장하지 않는다. 재사용421(training)과 새433/439(training)를 실행 전에 고정해 진입1의 독립 프로세스3개로 비교한다. 성공이 나오는 seed만 선택하거나 old결과를 새holdout으로 세지 않는다.


## 02:02 늦은 인식·yaw 오차 진단과 sector v2

첫 raster3개는 접촉0/3이다. 421/433은 metricpose0으로 timeout,439는252.20s decoded방위→255.44s metric→256.06s LINEUP→261.36s 불확실성 중단. 전방카메라가 입구 정면 반구 밖을 본 시간이 회전 탐색~첫ID 사이의49.55%였다. 이 값은 offline 실제yaw 진단D이며 카메라가 바깥 그물만 본 시간과 정확히 같지는 않다. 제한된 정면 탐색이 정보를 늘리는지 비교할 이유로 사용하며 성공 근거로 사용하지 않는다.

439의252.20s에는 conditional 위치std.525m/yawstd12.98°, 중단 시에는1.00075m/13.44°였다. 오프라인 실제위치오차.770m/yaw오차9.30°를 함께 보존했다. std는 실측 보정 확률이 아니다. 유효 마커 영상만으로 아직 직선 방향이 확정되지 않은 동안 worldyaw 기준을 임의로 얻을 수 없다. 기존1m 안전문턱을 완화하지 않는다. 개별자료 `artifacts/mission3/physical/20261009-entry-raster-entry1-seed439/late-acquisition-uncertainty-diagnostic.json`.

**P/C 추가 근거**: [Bitcraze state estimation의 Flowdeck model](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/)은 PMW3901의 흐름과 VL53L1x의 높이를 조합해 속도를 계산한다고 설명한다. 이 자체가 worldyaw 절대 기준을 제공한다는 설명은 없다. 현재 yaw/bias의 적분 및 위치 공분산 결합은 프로젝트의조건부 모델이다. [PAMPC, Falanga 등2018](https://arxiv.org/html/1804.04811v2)은 시야·영상상의 이동과 동역학/입력 포화를 함께 고려한다. 아래 단순 sector controller는 PAMPC/NMPC 구현이 아니며, 추적 중 시야를 고려해야 한다는 설계 원리만 참고한다. 이 논문의 VIO·장비·실험 성능을 stock Crazyflie 성능으로 가져오지 않는다.

**S paired 대안**: `config/mission3-35x12-entry-sector-raster-v2.json`은 입구 정면 중심±70° sinusoid/56s, analytic yaw-rate FF + heading feedback을 쓴다. 이상적인 최대각속도70×2π/56=7.854°/s, 최대각가속도70×(2π/56)²=.881°/s²D. 최종명령은10°/s·20°/s² saturation을 통과한다. H87°의 공칭 camera와 조합한 수평 각범위는±113.5°D이며 전체180°inward반구를 포괄하지만, 실제K/D·yaw오차·노출·픽셀 크기 때문에 인식을 보장하지 않는다. WAIT15s 고정시선/45s제자리sector관찰 뒤 entry-relative4m/±.75m/.15m/s 연속 raster는 v1과 동일하다. sector에서는45s를360°한회전이라고 부르지 않는다. 초기차량phase를 바꾸지 않는다.

반대 방향 입구와±π wrapping, 처음15s시선, 속도·yaw slew, 차량clock과 독립적인 탐색clock, 관측소실 시 현재위치/시선중심 유지, invalidparams거부를검증해162/162pass. 재사용421/433/439 모두training으로 사전 계획·동일소스동결, 원v1세개와 paired비교한다. 계획 `artifacts/mission3/acquisition/20261009-entry-sector-training-v2/plan.json`. 접촉/pass·비전발견시간·불확실성·actualspeed·전체분모를 모두 비교한다.


## sector 비교 결과와 다음 검증

세 조건 모두 접촉 실패0/3.439의metric획득은255.44→252.54s,LINEUP256.06→254.50s로빨라졌지만261.40s에conditionalpositionstd1.000332m로중단.421/433은metric0으로300s검색한도. 전체소스/runtime/config동결일치·명령포화감사3pass. actual439최대.403524m/s는.4m/s명령과다르다. 단순왕복yaw를착륙해결책으로채택하지않는다.

다음에는hardgate를유지하면서먼저감속해관측과추정이회복될여유를주는정책과,전방카메라배경으로gyro누적오차를관측할수있는지별도검증한다. [Jia/Evans,Online Camera-Gyroscope Auto-Calibration for Cellphones,2014저자원고](https://users.ece.utexas.edu/~bevans/papers/2015/autocalibration/autocalibrationIEEETransImageProcPaperDraft.pdf) §I/II:camera/gyro시간·장착·rolling shutter를고려하며,translation을pure-rotationhomography로해석하면보정이틀릴수있다. 우리native320규칙타일/희미한그물/차량이동은특징매칭반례를검사해야한다. 논문의온라인implicitEKF전체가구현됐다고부르지않고,미확정K/D/Rbc/readout시간은U로유지한다. 기본센서잡음·gyrooffsetS를정답으로사용하는calibration은금지한다.

## 02:46 예방적 불확실성 관측과 무손실 재검증

soft std.6m에서 접근 position feedback을 멈추고 높이/마커 관측을 유지하는 `UNCERTAINTY_OBSERVE`를 opt-in 구현했다. 원래 hard1m/vision loss/contact 규칙은 유지한다. 이후 위치std≤.9m/yawstd≤5°/안정 직선1.5s와 새로 들어온 qualified 직선 지도·영상 보정12회가 있어야 LINEUP으로 돌아간다. 모두 실험S이며 std는 확률 보장이 아니다. 관측 단계에서 속도FF0(brake)와 영상에서 추정한 차량 속도FF1(coast)을 분리했다. 제공 차속.25를 제어기에 주입하지 않는다. 새 evidence count만 로깅하며 공분산 수식을 임의 변경하지 않았다.

계획·동결·전 분모: `artifacts/mission3/acquisition/20261009-uncertainty-observation-training-v3/`. brake439 1/1실패:256.24s 감속→260.10s 영상 소실→262.10s 중단, finalpositionstd.737968m/yaw13.482°. coast421/433은 인식0으로 searchtimeout, coast439는 accepted183장→263.18s std1.000488m/yaw13.537°로 중단. coast3/3실패, brake1/1실패, 접촉/off0/4, 소스/runtime동일·coast config동일·caps4pass. qualified 직선 보정 모두0이었다. 정지는 불확실성 증가를 줄였지만 시야를 잃고, FF는 시야를 더 유지했지만 절대 자세 기준이 회복되지 않았다. 단순 soft brake가 해결책이라는 주장은 기각한다.

보관용 MJPEG가 원본 pixel을 바꾼다는 문제를 찾아 FFV1/source SHA256으로 수정했다. 원본과 손실 압축 재생을 같다고 설명한 부분은 정정했다. live input은 항상 압축 전 mono320이었다. 재사용 full rerun2건의11000프레임 모두 byte 동일; coast439의7개 원로그도 이전 비행과 동일했다. 새 무손실 six-preset도 목표ID0, C0wrongID57. 상세 공식 streaming 소스의324×244default와 native320 센서, RAW/JPEG/캡처 timestamp 한계는 [camera-recording-fidelity.md](camera-recording-fidelity.md)에 구분했다.

배경 특징/KLT/pure-rotation homography→SO3/몸체 gyro 비교 prototype은 noresize 입력, forward/backward.75px·≥20inlier/70%·coverage4096px²·rotationmedian.75px·estimatedspeed≤.03/tilt≤2°를 사용했다. K/D/장착/globalframe·문턱S이며 실제 보정이나 Jia/Evans의 전체 implicit EKF가 아니다. main은 모든 sensor/image worker 완료 뒤에만 truth를 평가에 읽었다. gyrooffset.03 설정값을 보정 정답으로 쓰지 않았다.

손실 AVI의 첫 benchmark8개는 보존하고 무손실2run×4간격으로 다시 확인했다. `artifacts/mission3/acquisition/20261009-lossless-background-rotation-bench-v2/`:

|영상/간격(s)|유효 회전 쌍|gyro-PC−visual 중앙값(°/s)|offline visual yaw-rate RMSE(°/s)|
|---|---:|---:|---:|
|entry-hold421/.1|2231|.09833|.45242|
|entry-hold421/.25|166|.03612|.20439|
|entry-hold421/.5|110|.02318|.13642|
|entry-hold421/1|7|.01737|.06791|
|coast439/.1|431|.00596|.46647|
|coast439/.25|61|.01248|.15039|
|coast439/.5|12|.01473|.08083|
|coast439/1|4|.05941|.02667|

장면/간격에 따라 bias 추정 중앙값이 달라지고 긴 간격에서는 표본이 극히 적다. moving raster 구간 다수는 translation gate로 제외된다. 작은 RMSE의4쌍만 보고 고정 gyro bias 보정이 검증됐다고 할 수 없다. 현재 제어에 미채택이다. 초기 공칭 장착/K/D, rolling shutter 및 시간축은 실제 자료 없어서U다.

## 곡선 지도 관측식의 정보 한계

[Hesch 등, Consistency Analysis and Improvement of Vision-aided Inertial Navigation, TRO2014](https://www-users.cs.umn.edu/~stergios/papers/TRO_2014_OC_VINS_Joel_Dimitris_Sean.pdf), Abstract/§I–III를 읽었다. 관측되지 않는 방향에 선형화가 가짜 정보를 만들어 불확실성을 줄일 수 있다는 점을 경계한다. 이 논문은 unknown-landmark VINS이며, 본 프로젝트의 알려진 stadium/이동 마커 모델과 같지 않다. 아래 circle 모델은 프로젝트가 따로 유도한D이며 OC-VINS 구현/성능을 가져온 것이 아니다.

연구 전용 `scripts/mission3/map_constraint_geometry.py`는 own pose x=(px,py,ψ), raw image/IMU 상대 pad q, 상대 차량 yaw β를 사용한다. world pad m=p+R(ψ)q, 원 중심c/r에서 residual은 `(‖m−c‖−r, wrap(ψ+β−atan2(m−c)−방향·π/2))`이다. 방향은 영상 heading에 가까운±후보이며 plant 방향/phase가 아니다. own pose와 cart를 함께 circle 중심 주위로 회전하는 N=(J(p−c),1)에 H·N=0이다. 한 프레임은 세 변수에 rank2 제약만 준다. 직선에서도 along-line 위치는 남는다. 누적 관측과 센서 예측/knownentry prior가 제공하는 정보는 별도 공분산·상관성 검증이 필요하다.

비행/추정기에 연결하지 않은 offline geometry2버전은 coast439 raw accepted183장을 분석했다. nearest 후보 leftcurve155/straight28, rank모두2/HN최대8.01e−16, 기존 innovation 문턱에 들어온 후보182장. 분석이 끝난 뒤 actual curve/straight와 비교한 agreement182/183이다. v1은 offline truth schema의 `turn`/`in_turn` key를 잘못 읽어 평가가 중단됐다. partial/실패로그를 보존하고 새 v2 output으로 고쳤다. 어떤 결과도 접촉 성공이나 공분산 개선으로 세지 않는다.

아직 full pose/yaw/velocity/bias 공분산 cross term, capture/receipt 지연 전파, 같은 IMU roll/pitch가 만든 raw PnP와 오도메트리의 상관성, segment association 경계, 시간에 따른 gauge 일관성을 모델링하지 않았다. 위치 covariance2×2와 yaw std만 합쳐 independent3×3로 가정하는 posterior는 계산하지 않는다. 4개 기하 tests는50randomstate의 numerical Jacobian/실제 공통 회전 불변성·단일 관측 정보 rank/null·직선 along 자유도·undefined 입력 거부를 검증했고 전부 통과했다. 이 검증은 sensor noise calibration이나 live observer 성공 검증이 아니다.

## 03:22 탐색폭 v4 전체 결과와 높이·방위 기하

사전 계획의6개 모두 같은 frozen core/runtime에서 종료했다. forward8 training421/433/439는 각각239.24s 불확실성 중단/309.44s 검색 중단/239.02s 불확실성 중단.433만159.08~159.48s에4 decoded 프레임(코드 최소변 약9~10px)을 얻고159.28s coarse방위로 전환했지만 metric0이다. lateral15 같은3seed는 모두309초 검색 중단/ID0이다. 접촉0/6/off0/독립승인0/qualified0. 명령 포화6pass, FFV1 총32806프레임 source SHA256 전부 일치. [summary](../../artifacts/mission3/acquisition/20261009-entry-coverage-training-v4/summary.json). v4 정책을 일반 해결책으로 채택하지 않는다. 소스 동결은6개 종료 뒤 해제했다.

연구 전용 `height_bearing_geometry.py`는 알려진 마커 중심 높이.40m(사용자 치수)을 추가로 사용한다. native 모서리를 왜곡 제거한 뒤 대각선 교점으로 마커 중심 광선을 구한다. 네 모서리의 산술 평균은 원근 투영에서 중심과 다르므로 쓰지 않는다. yaw=0 기준 IMU roll/pitch를 적용한 광선r와 camera offset o에서 λ=(.40−ownheight−o_z)/r_z, 상대 마커 중심q=o+λr이다. 바닥의 정확한 ToF 분류/원자료만 사용하며 near-horizon6°/잘못된 높이/뒤쪽 광선/퇴화 사각형을 거부한다. 6°는 민감도를 제한하는S이며 실측 성능 경계가 아니다. 기체 위치·차량 방향·차속·패드 offset을 확정하거나 full pose filter를 보정하는 코드가 아니다.

무손실 source 검증이 있는 두 training 로그를2CPU로 분석하고, worker 모두 끝난 뒤 truth를 오프라인 평가에만 읽었다. coast439의183개 기존 metric 프레임에서 상대XY오차 중앙값14.37mm/최대61.60mm, 기존 own pose로 world 변환한 오차 중앙값628.68mm다. forward8seed433의4개 작은 decoded 프레임에서도 조건부 상대 거리점4개가 생겼지만 상대XY오차 중앙값62.31mm/최대89.21mm, world오차 중앙값376.64mm다. 표본4개는 일반 정확도/초기 획득 개선을 입증하지 않는다. [계획·결과](../../artifacts/mission3/acquisition/20261009-height-bearing-training-v1/summary.json).

S 오차 민감도 세 조합(.5px/1cm/.5°;1px/3cm/1°;2px/5cm/2°)의 first-order Jacobian covariance만 계산했다. 중간 조합의 상대XY 최대축std 중앙값은183점117.09mm/4점343.22mm다. K/D/mount·실제 ToF tilt/ROI/전기적 응답·roll/pitch 상관성/rolling shutter U이므로 확률적으로 보정된 covariance나 세계 좌표 posterior가 아니다. 차 위로 올라가는 실제 표면 전환에는 이 바닥 높이 가정을 그대로 쓰지 않는다. 원래 PnP18px/품질/불확실성/접촉 gate는 변경하지 않았고 live연결0이다. 신규3tests(원근/tilt/native 작은 코드의 기하·민감도/PSD·퇴화/수평선/오류면 거부) 통과, 이후 전체182tests에 포함해 재검증했다.

## 공식 자이로 처리 단계와 현재 시뮬레이션 경계

[Bitcraze firmware BMI088/BMP3xx driver, 고정 commit f6e0f0a](https://github.com/bitcraze/crazyflie-firmware/blob/f6e0f0a3b526861328caf3296a7ecf00bb915d39/src/hal/src/sensors_bmi088_bmp3xx.c)의 코드C를 확인했다.1000Hz/±2000°/s(LSB4000/65536=.061035°/s), 일반 분기512 raw sample과 축별 variance<100LSB² 및1초 clock threshold로 평균 bias를 얻는다. `GYRO_BIAS_LIGHT_WEIGHT`가 선택되면1000샘플의 별도 분기다. 실제 지급 하드웨어 revision/build 선택은U이며 이 분기를 기체 전체 공인 실측으로 부르지 않는다. 일정한 실제 회전도 lowvariance를 만들 수 있으므로 부팅 중 정지 조건이 필요하다.

driver는 raw−bias→단위변환→airframe정렬→80Hz LPF를 적용한 `SensorData.gyro`를 estimator로 보낸다. [stabilizer](https://github.com/bitcraze/crazyflie-firmware/blob/f6e0f0a3b526861328caf3296a7ecf00bb915d39/src/modules/src/stabilizer.c)는 센서 calibration을 기다리며 gyro.x/y/z 로그는 이 처리된 °/s이다. raw로그와 같다고 가정하지 않는다. 공식 두 source를 read-only 보존하고 `docs/references/bitcraze-gyro-processing-20261009/source-manifest.json`에 SHA/URL/commit을 기록했다. 실제 기체 flash/upstream 변경0.

현재 MuJoCo adapter의 gyro는 물리 gyro+설정S offset/noise를 직접 host estimator와 PC로 보내며, full 부팅 bias/HAL/LPF/실기 로그 경로를 구현하지 않았다. 선택한 .03°/s offset/.08°/s white noise를 제조사 잔류 편향 사양이라고 할 수 없다. 현재 모델은 잔류오차 S sensitivity로 해석할 수 있으며, 부팅 보정 누락만이 실제 실패 원인이라는 결론도 아직 없다. .03을 임의로 없애 성공시키지 않는다. 독립 white noise라면 .08/√512≈.00354°/s라는 D 계산은 가능하나 LPF/온도/시간상관/실측 잔류bias를 보장하지 않는다. 부팅 raw모델과 측정값 기반 calibration은 향후 별도 모델/fixture 검증이 필요하다.

## 첫 decoded ID에 대한 제동·시선 유지 v5

forward8seed433의 짧은4프레임획득을 바탕으로 첫 ID에 아직 접근하지 않고 기존 탐색 속도/시선 이동을 제동하는 single-factor 후보를 구현했다. `VERIFY_BEARING`은 현재 own pose를 hold target으로 하고 실제 decoded bearing만 gaze feedback에 쓴다. S age.35s 이내에 다시 영상이 없으면 이전 WAIT/ENTRY_SCAN/ENTRY_SEARCH의 경로index/sectorclock을 복원한다. 한ID는 `valid=False/bearing_valid=True`인 partialstate이며 VehicleState·FF·metricdiscovery·접촉 판정에 쓸 수 없다. 기존 두ID temporalcoarse와 pose gate는 그대로다.

원native320의 실제16px ArUco decode·잘못된ID 거부·2차ID확인·만료·제동/yaw slew·metric전환·hard uncertainty/search timeout 보존5tests를 추가해 전체182/182pass했다. v5는 height-bearing나 gyro모델을 동시에 바꾸지 않는다. `config/mission3-35x12-entry-candidate-gaze-v5.json`, 사전3training/같은421433439/전방8baselinepaired 계획 `artifacts/mission3/acquisition/20261009-entry-candidate-gaze-training-v5/plan.json`. 모든3개 종료까지 core/config/runtime를 동결하고 접촉0인 원v4결과와 비교한다. 논문 PAMPC의 시야 유지 원리만 참조한 간단한 phase제어이며 논문 NMPC 구현이나 실기 성능 주장은 아니다.

v5 세개 종료 결과는 접촉/off0/3,421/433/439 모두239.24/238.54/239.02s 불확실성 중단이다.433은159.08s 첫ID→VERIFY_BEARING→159.38s 만료 후 탐색 복원, 총1decoded/metric0으로 기존 v4의4decoded/coarse보다 나빠졌다.421/439의 명령·truth·영상관측·모터·RPYT·덱·추정7원로그는 v4와 byte동일하다. 코드/runtime/config동일·사전54corefile동결일치·명령포화3pass, 무손실13491프레임 source SHA전부동일. [전체 결과](../../artifacts/mission3/acquisition/20261009-entry-candidate-gaze-training-v5/summary.json). v5도 미채택, originalfailedtraining보존.

## 작은 코드 픽셀 비교와 디코더 검증 범위 확대

159.02/159.22/159.32/159.42s의 v4/v5 native source8프레임을 SHA검증해 비교그림을 만들었다. 표시용4배 nearestcrop을 포함하지만 decoder는 항상 원320×320을 읽었다. [그림](../../artifacts/mission3/acquisition/20261009-entry-candidate-gaze-training-v5/native-pixel-loss-comparison.png)과 대응 metadata JSON은 원본파일을 수정하지 않는다. 초기 Pillow8 API 호환 실패는 원본/제어에 영향 없이 Image.NEAREST fallback으로 고쳤다. 그림을 보고 실제 마커가 여전히 화면 안에 있다는 사실과 marker_missing을 구분한다. 첫code minside10.107px, v4후속9.367/9.219/9.115px,4×4code+1cellborder이므로6module당1.5~1.7px/module D다. 특정원인이alias/blur라고 확정하거나 hardware 최소인식거리를 정하지 않는다.

[OpenCV 공식4.13 ArUco 설명](https://docs.opencv.org/4.13.0/d5/dae/tutorial_aruco_detection.html)의 threshold/bit extraction 항목을 다시 확인했다. 작은 코드에서 threshold window가 크면 경계가 깨질 수 있으며 cell rectification/margin과 계산량에도 영향이 있다. 이 설명은 native 정보가 부족해도 검출을 보장한다는 뜻이 아니다. 이전6preset은 목표code가 더작은 entryhold421전체를 대상으로 했으므로,4decoded가 있던 v4와1decoded였던 v5 구간의 검출 연속성을 검증한 것이 아니다.

새 offline16jobs=두 training의전체영상×8preset,4CPU boundedpool을 사전 계획했다. 기존6preset+smallC3/C7의 `perspectiveRemovePixelPerCell=8` 두 후보다. 표준 ArUco 내부rectification이 원본센서에 새정보를 만들지 않는다는 범위를 명시하고, 카메라 입력의 native 크기/바이트/ID/dictionary/error correction 설정은 유지한다. 새script는 모든worker를끝낸 뒤에만 truth기반projection평가를 읽는다. 각sourceframe SHA/목표ID/잘못된ID/conditionalprojectionmatch/CPU시간을 전부 보존한다. `benchmark_mission3_native_detection_v2.py`, 결과 폴더 `20261009-small-code-paired-detector-bench-v3`. 단순한 nearby두ID pair수는 worldbearing일치/metric추정/실제flight결과가 아니므로 별도 표기한다. 아직 live채택하지 않는다.

03:36 全16jobs 완료, 각v4 5857장/v5 4439장 전체를8번씩 읽고 source SHA동일 검증했다. 현재subpix는 원live4/1 target ID를 재현했고 모든검출은conditionalprojection과일치/틀린ID0이었다. smallC7cell8은 target5/4,unmatched0/wrongID0,평균검출3.865/3.331ms다. smallC3cell8은4/3,unmatched0/wrongID0,7.047/5.893ms다. C0는4/3에wrongID122/72,APRiLcorner는0/0이므로 채택 후보에서 제외한다. 기존smallC3/7만은4/1로개선없었다. C7cell8의v5추가인식은capture159.08/159.12/159.22s(기존159.02 포함)이며 가까운matchingpair3개다. 아직 세계방위일치 확인/상태추정/통신지연/실제제어성공이 아니다.

[완료표](../../artifacts/mission3/acquisition/20261009-small-code-paired-detector-bench-v3/summary.json)는 두 실패training표본이므로 unseen영상의 오검출률이나 일반 성능을 보장하지 않는다. 다음은C7cell8을 기존subpix와별도opt-in프로필로 제한해 core기준/단위 검증한 뒤 fixed세training조건의실제물리비교를 할 수 있다. 선택된preset은corner refinement가없는표준기본값을 포함하므로 pose품질gate를 완화하지 않는다. C7cell8+subpix별도민감도와확정관측만partialstate로쓰는옵션을 구분할 필요가 있다. 현재설정은바뀌지않았다.

## 03:52 모서리보정 유지·디코더 live 비교 v6 완료

추가offline4jobs의 C7cell8+기존SUBPIX도5/4 ID를 얻고 모두conditionalprojection일치/잘못된ID0이었다. 평균3.688/3.604ms, C3cell8+SUBPIX는4/3(6.218/6.233ms)였다. [추가표](../../artifacts/mission3/acquisition/20261009-small-code-subpix-detector-bench-v4/summary.json). 두 실패training표본에서 생긴 비교이며 일반 오검출률 보장 아님.

고정public `detector_profile=small_code_cell8_subpix`를 선택 가능하게 했고 default는 원래SUBPIX다. 후보의threshold3..13/step2/C7/perimeter.02/cell8이S이고, dictionary/ID/errorCorrection/border/refinement 및metric/minside/fit/temporal gates는 그대로다. arbitrary detector 설정을 받아들이지 않는다. 실제16pxnative코드의 1회 decode가metric이 되지 않고 두IDcoarse가 필요한 것,wrongID/native불일치/close-depth 거부·default모든parameter동일 검증3개를 추가해 전체185tests 통과했다.

같은421433439 training을 coarse/gaze각3개로 실행전계획/frozen59corefiles로 비교했다. 결과는0/6contact/off, code/runtime동일/configwithinpolicy동일, 일반명령caps6pass/FFV1총29818sourcepixelSHA모두일치.433은coarse/gaze각6/5decoded,두IDcoarse159.14s→164.52/164.62s 시야갱신 만료→309.44s검색timeout,metric0이다. 나머지 네 조건은239초 위치 불확실성 중단이며 no-ID조건7rawlogs는이전기준과byte동일하다. [전체 v6](../../artifacts/mission3/acquisition/20261009-cell8-detector-training-v6/summary.json). 디코더후보가인식을조금늘려도미션해결책은아니다.

433 센서/명령 budget진단에서159.14s heading39.06°→162s10.85°→163s1.29°로 시선이 움직이는 동안 기존15°이내translation 조건 때문에 거의 정지했다.5s decoded-age한도 가까이에서만command가 커져추종이끝났다. 별도 완료후truth평가에서pad와기체 수평거리는159.02s2.7955m→164s3.8548m로 증가했다. [오프라인 기하](../../artifacts/mission3/physical/20261009-cell8-coarse-v6-entry1-seed433/offline-coarse-pursuit-geometry.json). 이참값은 평가에만썼고controller지도/속도/initialphase에 넣지않았다. 추종이늦은것만이모든실패원인이라고단정하지않는다.

## 인식 사이 짧은 영상 추적: offline8조건

[Kalal·Mikolajczyk·Matas, Forward-Backward Error, ICPR2010](https://cmp.felk.cvut.cz/ftp/articles/matas/kalal-2010-fb_track-icpr.pdf) Abstract/§2/§3을 읽었다. 영상점의 전방/역방향 추적 불일치를 실패검사로쓰며 NCC와 상보적이라는 연구원리를 참고한다. 논문의 benchmarkprecision/recall을본320카메라로 가져오지않는다. [공식 OpenCV LK](https://docs.opencv.org/4.13.0/d4/dee/tutorial_optical_flow.html)는 짧은간격의 국소영상운동을계산한다. 같은무늬/가림/센서PSF 등에서identity나metricpose를보장하지않는다.

`benchmark_mission3_marker_tracking.py`는C7cell8+SUBPIX의decoded타깃quad로만9개의내부grid를seed하고, native입력을그대로 읽는다. LKwindow7/15,maxlevel2,FB≤.5px/≥6points,partialaffineinlier≥75%/residualmedian≤.5px/scale.8..1.25/NCC≥.85/native3pxedge/code최소7최대36/최종decodeage.5또는1s는모두S. 내부patch warp는진단용이고camera입력확대가아니다. 9개가독립feature라는확률가정이나원논문MedianFlow/TLD전체구현주장은없다. 모든8workers작업을bounded4CPU로끝낸뒤truthprojection평가를main에서만했다. live미연결/재decode성공수0이다.

v4/v5두training의.5s조건각13/10partialpoints,1s/window7각19/15(최소크기/가장자리guard에걸림),1s/window15각23/20(expiry)였다. conditionalprojectioncenter오차 중앙값.75~.93px/최대1.063px이며invalidprojection0이다. [완료표](../../artifacts/mission3/acquisition/20261009-marker-short-tracking-bench-v1/summary.json). nativeboard가작고두영상뿐이어서occlusion/다른마커/반복그림falseassociation 검증이 더필요하다. 원coded7px제약에서1초추적을전체추격해결책으로채택하지않으며metric/FF/직선/착륙진입으로쓸수없다.

## 회전 중 영상 방향 이동 v7 사전 비교

opt-in `coarse_translation_axis=fresh_decoded_bearing`는 fresh actualdecodedcode로시선과횡방향velocity축을분리한다. Quadrotor의horizontalvelocity를항상기수축으로제한할필요는없다는project제어해석이며,최종RPYT/PID/동역학에서가능한지는새physics로그로검증한다. native maxside<36/min7..18/age≤.15s일때만 영상bearing으로velocity를정하고 old/big영상은원bodyforward 유한예산으로돌아간다. gyro/heightbearing/LK/worldvehiclepose/차속prior를추가하지않는다.

새두profile은같은freshturn허용오차40°/turndesiredspeed.4m/s를쓰되axis가bodyforward vsdecodedbearing만다르다.40°는H87의half43.5°에서3.5°여유를둔S로K/D/실제yaw/foreshortening을보장하지않으며,code전체viewport/크기와freshvision검사가추가로필요하다. .4는상위softwarecapS를쓴것이지제조사최대값이아니다. v6와비교할때angle15→40/speed.15→.4도바뀐복합정책이라는점을기록한다. base1.5m/path4m/duration20s/loss5s/std1/metric18px/contact판정은변경하지않는다.

velocitydirection/원각속도·가속·속력cap/old-big-outsideview반례·path4소진/std1중단/invalidaxis3tests를추가해전체188/188pass. 사전6reuse/같은421433439training,최대4worker/frozen60corefiles `artifacts/mission3/acquisition/20261009-coarse-direction-training-v7/plan.json`. 현재4개먼저실행하고종료확인후나머지2를같은소스로시작한다. 성공은아직확인되지않았다.

## 04:12 v7 완료·추격 예산과 영상 추적 반례

v7 사전6개는 전부 실패했다. early-body/early-bearing의433은 각각5/6 decoded, metric0/309.44s검색 중단이며, 다른 네 조건은ID0/239초불확실성 중단이다. 전체6명령포화pass/29818원본프레임SHA일치/60core파일과실행라이브러리/configwithinpolicy동일을 확인했다. [전체 결과](../../artifacts/mission3/acquisition/20261009-coarse-direction-training-v7/summary.json). 새조건34완료/접촉0이며동일seed·정책·녹화재실행을포함해일반독립성공률로보지않는다. entry2로진행하지않는다.

완료후원로그를비교하면159.14s 첫coarse명령이vx약−.147m/s였고pad수평거리는2.843m였다. 검색에서후진중발견했기때문에회전중새목표속도를요청해도가속도.15m/s²가즉시전진명령으로바꿔주지않는다. 세정책의160s거리3.156~3.157m/164s3.840~3.855m,early-body166s3.648m다. [완료후비교](../../artifacts/mission3/acquisition/20261009-coarse-direction-training-v7/pursuit-budget-comparison.json). 차량정보/실제상대속도는원controller에주입하지않았다. 회전·코드샘플링·갱신소실·검색속도전환을함께검토하며한원인으로단정하지않는다.

별도해석D/S인완벽추종1차원직선모형을계산했다. a=.15/vmax=.4/차량.25/path4m/최대20s하에서초기−.15m/s이면절대이동거리4m를쓸때약12.146s/순전진3.85m/차량3.036m이므로최대거리감소.8135m다. 초기0이면1.1667m,+.15이면1.3698m다. blind기본1.5m예산에서초기−.15는최대거리감소0이다. 분석식의후진절대거리와전진전환을독립200000step수치적분으로대조했고차이.1mm미만이었다. [분석](../../artifacts/mission3/acquisition/20261009-coarse-direction-training-v7/ideal-straight-budget.json),source analyze_mission3_pursuit_budget.py. 이모형은명령추종·path추정이완벽하고광학/회전/동역학제약이없는이상화다. 실제물리도달불가능의증명이나원18px gate진입을보장하지않는다. 상한계산을이유로blind예산을늘리지않는다. config의plant-only차속은해석시나리오용이며제어입력아니다.

영상추적의반례시험은원native영상의decoded뒤연속쌍6개×LKwindow2개×synthetic변형10=120jobs/4CPU였다. 사용한원프레임SHA를검증하고원AVI/원실험로그를수정하지않았다. 완전black/white/gray/random가림과3개의다른ID대체는각window의전6쌍에서추적거부였다. 정상영상/같은ID재렌더는각6/6연속성을유지했다. 반쪽가림은window7에서0/6,window15에서2/6추적이유지됐다. 이는반쪽마커의정상부분운동일수도있으므로즉시falsepositive라고부르지않는다. 다만FB/NCC통과는전체code의identity재decode가아니다. 6쌍의합성변형으로실제오연결확률/가림강건성을정할수없다. [반례 전체120조건](../../artifacts/mission3/acquisition/20261009-marker-tracking-stress-v1/summary.json),source stress_mission3_marker_tracking.py. LK는아직live미연결이며이결과를기존ID/pose/contact문턱을우회할근거로쓰지않는다.

새실패영상은early-body433의원물리기록156~168s를4개EGL프로세스로독립후처리한1배속이다. 표시10FPS/121frames/12.1s(H264재생),인식원320mono/20FPS변경없음;시작끝포함표본때문에영상길이는12.1s다. 원비행은309.44s실패이며본영상은전체비행이아닌발견/추종소실구간이다. 원카메라/기체추적/전체실내지도/공칭화각/명령vs실제속력/접촉0/실패판정을표시했다. [영상](../../artifacts/mission3/physical/20261009-coarse-early-body-v7-entry1-seed433/acquisition-failure-156-168s-1x-h264.mp4). physical-review메트릭거리축이1m에서잘려큰거리변화를못보는후처리문제도전체범위로고쳤다. 제어·평가기는수정하지않았다.


2026-10-09 04:45의 v8은 [PAMPC 원논문](https://arxiv.org/html/1804.04811v2)의 시야/영상 운동과 행동을 함께 고려한다는 원리만 참고한 단순 desired-speed schedule이다. 논문의 NMPC 구현이나 실기 보장을 주장하지 않는다. 추정 기체 정면 반대방향 검색 속도를 .05/.10로 줄이며 방향/검색 범위/vehicle phase/표적 상태 priors를 바꾸지 않는다. 원native320/20FPS S/같은pose gates/차량 추정 전용은 유지한다. ideal-straight-budget 초기속도 -.15/-.10/-.05/0에 따른4m 예산 closing 상한 .81354/.95417/1.07188/1.16667m는 이상화D/S일 뿐이다. v8 물리 결과0052/3,0100/3 접촉이지만005/433 일반 가속도 위반1회로 전체 envelope 통과 접촉은1회다. 과거 실패34개도 그대로 보존한다. 새 source v9의 이력 보존 Flow-loss 감속 회귀 및 unused5조건을 사전 등록했다.

LK 이전/현재 patch NCC를 잇는 검사에는 점진적인 appearance drift 반례가 있다. 새 오프라인6앵커×2window×3wrongID×2morph=72조건에서 모든72가 마지막 decoded age1초 만료 전 완전히 다른ID 패턴을 받아들였다(총972frames). age expiry 자체는72 모두 작동했다. 초기 decoded reference와 고정 NCC.85 진단은 모든72를 경고했으나 synthetic morph에만 검증됐고 live tracker에 넣지 않았다. abrupt 다른ID/가림120조건의 거부 결과는 점진적 drift 강건성과 다른 증거다. 따라서 LK를 identity/metric 인증으로 사용할 근거가 부족하며 live미연결 유지. 보고한72/72는 합성 반례의 분모이지 실기 오연결 확률이 아니다. 원native source hash/exact와 source dependencies를 results/summary에 기록했다.


2026-10-09 05:58 업데이트: 초기 WAIT15/rotation45/height1을 유지하고 이동탐색 중에 진입 정면 시선만 고정한 v11은 재사용training3/4,unusedholdout3/5 조건부contact+normalcaps였다. 소모된training과새조건의분모·source를분리한다. native원픽셀SHA는13818/24051전부일치. 457훈련은실제접촉이있어도모터종료없어실패,521/541새조건은ID0 timeout이다. 새동일화각에서최대투영minside7.007/4.489px였고18pxframe0으로남는다. [전5결과](../../artifacts/mission3/acquisition/20261009-entry-forward-gaze-holdout-v11/summary.json),[시선/높이오프라인민감도](../../artifacts/mission3/acquisition/20261009-entry-search-visibility-counterfactual-v1/summary.json). 이는PAMPC의가시성과운동제약을함께고려하는원리만참고한휴리스틱으로논문NMPC구현/실기강건성을주장하지않는다. [실제Flow·ToF입력계약](flow-tof-runtime-contract.md)과독립검토피드백을바탕으로별도v12에서측정raw에기반한onboard근접ToF softwaregate를paired검증중이다.원캡/pose/std/contactgate/차량phase/물리/마커변경없음. qualified0,entry2미진행.
