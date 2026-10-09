# 제조사 기반 시뮬레이션 모델·SITL·단계별 검증 계획 (구상안)

작성·출처 확인일: 2026-09-30 (Asia/Seoul). 이 문서는 설계 제안이며 코드·설정·패키지는 바꾸지 않았다. 모든 수치 조건은 **단계 내부 후보**이고 대회 평가나 장비 지급의 합격 기준이 아니다. ArUco 관측 알고리즘과 외부 제어기 수식·상태기계는 다른 담당 범위이며, 이 문서는 그것을 시험할 모델·bench·SITL 조건만 정한다.

## 1. 핵심 판단과 확정 계획

- 제조사 자료만으로 지금 만들 수 있는 것은 **사양 원장, 카메라 중심 운동학 bench, Flow/ToF 센서점 bench**다. 반면 조립체 관성·무게중심, 적용 추력곡선, 모터 시정수, 공력, 방전 곡선과 카메라 외부변환 `T_BC`는 막혀 있다.
- **확정 계획**: 사용자가 모든 기체 사양을 제조사 기반으로 명시했으므로 제3자 대리 동역학은 채택하지 않는다. 사양이 채워지기 전에는 S2 이후 물리 검증(SITL 폐루프, 이동 표면 폐루프, 접촉 착지)을 보류하고 S0·S1·S3a를 계속 진행한다.
- 기존 CrazySim 기본 비행 성공은 SITL·CFLib 경로가 동작한다는 증거일 뿐이다. 조건은 `cf2x_T350`, 참값 pose 주입, 센서 잡음 꺼짐이었다. [run_sim.py](../../scripts/run_sim.py)가 `--flowdeck`·`--sensor-noise`를 넘기지 않으면 [pose 송신 분기](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1082)가 참값 pose를 추정기에 넣는다.
- 가장 먼저 작성할 파일은 `config/sim_profiles.toml`과 `scripts/surface_sensor_bench.py`다. 둘 다 미공개 동역학 값 없이 만들 수 있다.
- 지금 사용자에게 추가로 확인할 사항은 없다. Flow deck 지급 여부와 지상 PC 연산 허용 범위는 사업단의 확정 안내를 기다린다.

## 2. 대회 자료가 이 계획에 주는 영향

2026-09-18 경기규정(안) PPT와 평가 XLSX의 원문 대조는 메인 문서가 맡는다. 여기서는 시뮬레이션 계획에 미치는 영향만 적는다.

- **p4 미확정 조건**: Flow deck 지급 여부와 지상 PC 연산 허용 범위는 사업단 확정 안내에 따른다. 프로파일에 {Flow 지급, 미지급}과 연산 위치 축을 두고, S3a·8절 결과는 "Flow 지급 조건부"로 표시한다. PC 연산 범위가 좁아지면 S1의 지연·연산 위치 조건을 다시 정한다.
- **p4·p11 장비와 예선**: 기체 세트는 본선 진출팀에 지급된다. 예선은 설계·개발·검증 **계획**을 평가하며 실기 시험 결과·시뮬레이션 시연·결과 제출은 필수가 아니다. 따라서 "시뮬레이션 통과 후 장비 지급"은 정식 요구로 확인되지 않았고 이 문서에서 쓰지 않는다(다른 문서의 같은 표현은 메인이 정정). 이 문서는 XLSX D5의 SITL 계획과 D6 안전 예외 시험 조건의 근거 자료다.
- **p3·p9 연속 수행**: 같은 기체가 표식·경로 → 장애물 회피 → 이동 표적 착륙을 연속 자율수행한다. 실패하면 1단계부터 다시 하며, 미션은 7분이고 팀당 2회 시도한다. 착륙은 3단계 모듈로 개발하되 평가 에피소드는 1단계부터의 시도 1회로 정의한다. 3단계 진입 상태가 앞 단계의 종료 상태이므로 단계 사이에 추정기를 초기화하지 않는다. 1·2단계 장면의 치수는 규정 확정값만 쓰고 여기서 만들지 않는다.
- **비행 시간 위험**: 제조사 비행 시간 7분은 기본 배터리 기준값이며 덱 장착 구성의 7분 미션을 보증하지 않는다. 방전 곡선이 막혀 있어 시뮬레이션으로 검증할 수 없는 위험 항목으로 남긴다.

## 3. 지금 가능한 모델과 막힌 항목

| 요소 | 근거 | 분류 | 허용 용도 |
| --- | --- | --- | --- |
| 표준 2.1 이륙 질량 29 g, 92×92×29 mm, BMI088/BMP388 | S1 데이터시트 | 제조사 | 기체 단독 참조값. 덱 포함 질량·프로펠러 외곽 아님 |
| AI-deck 1.1 4.4 g, Flow v2 1.6 g, HM01B0 320×320·87°, PMW3901·VL53L1x | S3–S5 | 제조사(버전 조건부) | 센서 모델 파라미터 후보 |
| 29 + 4.4 + 1.6 = 35.0 g | 문서값 합 | 파생값 | 프로파일 표시용. 확정 이륙 질량 아님 |
| 공식 펌웨어 `CF_MASS 0.029`, `ARM_LENGTH 0.046`, legacy 전압→추력 3차 근사, `THRUST2TORQUE` | S7–S8, 커밋 e2fdc21 | 제조사 공개 **펌웨어 상수** | 제어·보상 구현값. 플랜트 사양 아님 |
| fork `CF_MASS 0.027`, 추진 분기 상수 없음, Kalman·PID 컴파일 고정 | [cf2 기본값](../../src/CrazySim/crazyflie-firmware/src/platform/interface/platform_defaults_cf2.h:49), [SITL CMake](../../src/CrazySim/crazyflie-firmware/sitl_make/CMakeLists.txt:48) | 제3자 fork 상수 | SITL 실행 조건 기록 |
| `cf2x_L250/P250/T350`, `cf21B_500`의 mass·J·rpm2thrust·drag | utiasDSL [params.toml](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/drone-models/drone_models/data/params.toml:3), J에 `# TODO`, T350 J는 "from L250" | 제3자 모델 | 사양 아님. 채택하지 않고 설치 검증 기록으로만 유지 |
| MJCF `fpv_cam` pos 0.03 m, fovy 70°, 640×480 | [cf2x_L250.xml](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/drone-models/drone_models/data/cf2x_L250.xml:81) | 제3자 기본값 | AI-deck 카메라·`T_BC` 아님 |
| 충돌 박스 11×11×4 cm, `condim=1`(마찰 없음) | 같은 XML, [CrazySim 패치](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:472) | 제3자 근사 | 접촉 판정 근거로 쓸 수 없음 |
| 조립체 관성 텐서·CoM, 적용 추력곡선, 모터 시정수, 공력, 방전 곡선 | 미공개·미확정 | **막힘** | 빈칸 유지 |
| `T_BC`, 스트림 모드 내부 파라미터·왜곡, fps·지연, 덱 적층 순서, Flow/ToF 센서 원점 | 미확정 | **막힘** | 빈칸 유지. S1·S3a는 해당 원점 중심으로만 시험 |

**표준·legacy·업그레이드와 펌웨어 상수의 구분.** 공식 Kconfig는 legacy, 2.1+ 프로펠러, 추력 업그레이드를 따로 고르며 기본값은 2.1+다. 표준 2.1의 기본 프로펠러가 legacy 분기와 같은지, 덱 장착·배터리 전압 범위에서 3차 근사의 오차가 얼마인지는 미확정이다. 이 근사는 제조사가 공개한 보상용 모델이므로 측정 사양이나 성능 보증으로 격상하지 않는다.

`cf2x_L250`은 CrazySim 헤더에서 "Loco-250 props", drone-models 문서에서 "Standard configuration"으로 설명되지만 어느 쪽도 Bitcraze 자료가 아니다. 질량도 params 31.9 g, MJCF 기본 27 g, fork 펌웨어 27 g, 공식 29 g으로 서로 다르다. 따라서 이름의 "standard"나 "250"을 표준 2.1 + 250 mAh 배터리의 공식 모델로 해석하지 않는다.

## 4. 추가 제조사 근거 탐색 결과 (커밋 고정)

- [bitcraze-mechanics @c70aa74](https://github.com/bitcraze/bitcraze-mechanics/tree/c70aa74368e713734ddebbf14238fd6c3c2079c6/models)(2026-07-27): `models/`에는 `cf2_model.skp`, `cf_model_s_revA.skp/.dxf`, 모터·프로펠러 mockup STEP, Brushless 가드 STL, 착륙장 모델이 있고 `system-id/`에는 `thrust-bracket.FCStd`만 있다. AI-deck·Flow deck 조립체 모델과 질량 특성은 없고 2.1 개정 대응도 확인되지 않았다. 형상 근거 후보로만 쓰며 균일 밀도 관성 계산은 하지 않는다.
- [bitcraze/hardware crazyflie-2_1 @3ad4739](https://github.com/bitcraze/hardware/tree/3ad4739ca4a81ab7ea108f635184c5db776a39c5/src/products/crazyflie-2_1)(2025-11-19): `datasheet/`와 `electronics/`만 있다. 데이터시트 원본은 29 g·7분·15 g을 반복하고 관성 값은 없다. `crazyflie_2_1-measurements.png`는 내용을 확인하지 않았으며 S4 형상 근거 후보로 남긴다.
- [Webots Crazyflie.proto @61bdf27](https://github.com/bitcraze/crazyflie-simulation/blob/61bdf271438f6263f28cf7446b8c65505956d36d/simulator_files/webots/protos/Crazyflie.proto)(2023-07-28): `mass 0.05`, `density -1`, 원통 충돌체(높이 0.03, 반지름 0.05)이고 관성 행렬 필드가 없다. 제조사 저장소에 있어도 시뮬레이터 자리표시값이므로 채택하지 않는다.
- [PWM to Thrust](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/): 본문에 관성 수치가 없다. 외부 연구 목록(2.1 시스템 식별로 소개된 Shi et al. 2021, Crazyflie 2.0 학사논문 Förster 2015)은 정책상 제외한다. Bitcraze 2021 측정의 `dev-systemid` 브랜치와 `tools/system_id`(페이지 표기 `sytem_id`)는 장비 지급 후 사용자 측정 절차 후보다. 2015년 과거 측정표(93.75% PWM에서 약 58 g)는 표준 2.1 곡선으로 쓰지 않는다.

## 5. MuJoCo 3.14.0 확인 사항

- [3.14.0 Modeling의 MoCap bodies](https://mujoco.readthedocs.io/en/3.14.0/modeling.html): 시뮬레이터는 mocap body를 고정된 것으로 취급하므로 mocap body와의 접촉에는 상대속도 성분이 빠진다. 같은 절은 대안으로 일반 body를 weld equality로 mocap에 연결하는 방법을 제시한다.
- [API functions](https://mujoco.readthedocs.io/en/3.14.0/APIreference/APIfunctions.html)·[API types](https://mujoco.readthedocs.io/en/3.14.0/APIreference/APItypes.html): `mj_ray`는 맞은 geom id를 돌려주고 `flg_static=0`이면 정적 geom을 제외한다. `mj_objectVelocity`는 객체 중심 6D 속도를 계산하고, `mjData.cvel`은 com 기준 속도(rot:lin)다.
- 설치된 3.14.0 메모리 확인(파일 생성 없음): `mocap_pos`로 움직인 차량 body의 `cvel`과 `mj_objectVelocity`는 0이었다. 하향 광선은 `flg_static` 값이 1이든 0이든 mocap 차량 상판에 맞았다(`body_weldid`가 자기 자신). Python `mj_ray`는 `normal`을 선택 인자로 받는다.
- **결론**: 차량이 mocap이면 ToF 거리는 맞게 나오지만 표면 속도는 MuJoCo 상태에서 얻을 수 없고, 접촉 마찰로 기체를 실어 나르지도 못한다. 따라서 S3a는 표면 속도를 스크립트 궤적에서 가져오고, S4는 차량을 일반 body(관절 구동 또는 mocap에 weld)로 바꿔야 한다.

## 6. 단계 순서와 의존성

| 단계 | 내용 | 선행 조건 | 상태·주장 범위 |
| --- | --- | --- | --- |
| S0 원장 | 요소별 값·단위·출처 등급·확인일과 프로파일 해시. 막힌 항목은 빈칸 | 없음 | 진행. 어떤 실행이 어떤 가정을 썼는지 |
| S1 카메라 중심 운동학 bench | 가시성 장면에서 광학중심을 명령 속도로 직접 이동(이상 추종, 선택적 1차 지연). 렌더 → 관측기 → 외부 제어기 → 명령의 폐루프 | 관측기·제어기 입출력 계약 | 진행. 관측·명령 계약과 지연·가림 민감도만 주장. 비행·착륙 주장 불가 |
| S3a 센서점 bench | 스크립트 센서점 경로로 Flow/ToF 출력만 비교, 펌웨어 없음 | S0 | 진행, Flow 지급 조건부 |
| S2 장면 + SITL 통합 | 차량·패드·마커를 CrazySim 장면에 넣은 폐루프 비행 | 제조사 사양으로 채운 동역학 프로파일 | **보류** |
| S3b 이동 표면 폐루프 | S2 + S3a 모델로 fusion 후보 비교 | S2, S3a | **보류**, 설계만 |
| S4 접촉 착지 | 형상·마찰·패드 경계·착지 후 안정 | S3b, 공식 형상, 일반 body 차량 | **보류**, 설계만 |

S3a는 동역학이 필요 없으므로 S1과 병렬로 먼저 끝낸다. 순서를 지키는 이유는 두 가지다. S2 없이 S3b를 돌리면 추정기 응답이 없고, S3b 없이 S4를 돌리면 차량 위 추정 오염이 접촉 결과에 섞여 원인을 가를 수 없다. 보류는 동역학 항목이 제조사 공개값으로 채워지면 푼다. 장비 지급 후의 측정값은 "사용자 기체 측정값"으로 따로 기록하고, S2에 쓸지는 그때의 정책에 따라 정한다.

## 7. 카메라 중심 운동학 bench와 명칭 규칙

- S1에는 기체 좌표가 없다. 상태와 명령은 광학중심과 광학 좌표로 표현하며, 미확정 `T_BC`를 0이나 가설 기체 좌표로 채우지 않는다. 결과는 "카메라 중심 운동학 bench 결과"로만 부른다.
- S1에서 외부 제어기 출력은 광학중심의 속도·yawrate 명령으로 해석한다. 이 해석은 입출력 계약에 적고, 기체 명령으로 바꾸는 변환은 `T_BC` 측정 전까지 미해결로 둔다.
- S1이 표현하지 못하는 효과는 결과와 함께 적는다. yaw 회전 때 `t_BC` 레버암으로 생기는 광학중심 이동, 수평 가속에 필요한 기체 기울기가 카메라 자세에 주는 영향, `R_BC` 피치가 여기에 해당한다.
- 드론 메시에 카메라를 붙인 GUI 장면은 만들지 않는다. 시각화로 보여주더라도 성공 근거가 아니며, upstream `fpv_cam`은 AI-deck 카메라가 아니다.
- 스트리머 예제의 324×244 창과 데이터시트 320×320·87°의 대응(크롭 후 화각)이 미확정이므로 두 카메라 프로파일을 분리한다.
- `cf2.1` 프로파일 이름은 제조사 값만으로 채워진 프로파일에만 쓰며 현재는 없다. S1은 `camera-centered-kinematic`, S3a는 `sensor-point-bench` 라벨을 쓴다.

## 8. 이동 표면 Flow/ToF 모델과 fusion 후보 검증

현재 [`compute_flow`](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1035)는 기체 world 속도(`cvel`)를 body 좌표로 돌려 쓰고 아래 표면의 속도를 빼지 않는다. 높이는 body 원점의 [하향 ray](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1020)로 구하며 0.1 m 미만은 0.1 m로 자른다. 예를 들어 h = 0.3 m에서 차량과 기체가 모두 0.3 m/s로 움직이면 물리적 상대 병진은 0인데도, 이 식은 프레임당 약 4.1 px(`0.01 × 30 / 4.2° × 0.3/0.3`)의 병진 광류를 만든다.

S3a는 Flow/ToF **센서점**의 경로를 직접 지정하는 bench다. 센서점과 기체 원점·무게중심의 관계는 미확정이므로 오프셋을 가정하지 않는다. 광선이 맞은 geom의 body로 바닥과 차량을 구분하고, 표면 속도 `v_s`와 센서점 속도 `v_p`는 5절 결과에 따라 스크립트 궤적에서 가져온다. 병진 광류는 `v_p − v_s`로 계산하고 회전 항은 상류 식을 그대로 쓴다.

| 시험 | 조건(잡음 끔, 고정 seed) | 내부 후보 조건 |
| --- | --- | --- |
| T1 정지 바닥 회귀 | `v_s = 0` | 같은 입력에서 상류 식과 차이 ≤ 1e-9 px |
| T2 동행 | 센서점·차량 모두 +0.3 m/s, 패드 위 | 병진 성분 ≤ 1e-6 px, 회전 항만 남음 |
| T3 경계 통과 | 바닥 → 패드 모서리 통과 | ToF 계단(상면 0.20 m)과 광류 스케일 변화 시각·hit body 기록 |
| T4 깃발·지지대 | 착륙장 앞쪽 모서리보다 0.25 m 앞선 마커판 아래 통과(현 설정의 중심 오프셋 0.40 m) | 광선이 판·차체 측면에 맞는 구간 기록 |
| T5 차량 표현 | mocap 차량 vs slide joint 일반 body 차량 | ToF·hit body 일치, 일반 body의 `mj_objectVelocity`가 스크립트 속도와 일치 |

**S3b 설계(보류).** 세 축을 교차한 행렬로 설계한다. 표면은 {정지 바닥만, 바닥 → 차량}, 피드포워드는 {FF 없음, nominal 0.3, 관측 추정값}, 추정기는 {stock Kalman, 후보 A: 차량 위 Flow 융합 차단, 후보 B: 표면 속도 보상 입력, 후보 C: 차량 상대 프레임 추정}이다. `stateEstimate.vx/vy/z`를 참값 `v_B`, `v_B − v_car`, 참값 z와 비교하고, 명령 속도 대비 실제 world 속도 초과량을 FF 중복 지표로 삼는다. 내부 후보 조건은 첫 실행 전에 고정한다. 차량 위 구간의 `|v̂_W − v_B,true|` 평균 ≤ 0.05 m/s, 경계 통과 후 추정 z 오차 ≤ 3 cm로 시작한다.

nominal 0.3은 사용자가 정한 **실험 설정값**이다. 후보 B에 이 값을 표면 속도로 넣은 실행은 "설정값 제공 상한" 조건으로 분리하고 성능 비교표에 넣지 않는다. 채택 후보는 관측에서 얻은 추정값만으로 같은 조건을 만족해야 한다.

## 9. 진실값 누설 차단

- 누설 경로: 참값 외부 pose 패킷, 참값 고도로 만든 baro, 구성에 없는 Multi-ranger ray(항상 송신), 차량 mocap 상태, nominal 0.3, 접촉 목록, 완벽한 시계 정렬.
- 제어 쪽 입력은 허용 목록으로 제한한다. 지연을 주입한 카메라 프레임과 그 시각, CFLib 로그, 명령 응답만 허용하고, 참값은 평가 로거가 별도 파일에 기록한다.
- 실행 manifest에 켜진 참값 경로를 기록하고 `oracle=true` 실행은 성능 표에서 뺀다. 제어 모듈이 장면·참값 모듈을 import하지 않는지는 정적 검사로 확인한다.

## 10. 접촉·패드 경계·완료 판정 (S4, 보류)

평가 쪽 판정에는 MuJoCo 접촉 목록을 쓴다. 내부 후보 조건은 모든 기체 접촉점이 30 × 30 cm 패드 다각형 안에 있을 것, 첫 접촉 때의 상대 수직·수평 속도를 기록하고 상한 후보를 둘 것, 깃발·차체·지면과 접촉하지 않을 것, 착지 후 1 s 이상 접촉을 유지하고 기울기가 30° 미만일 것이다. 제어기 쪽 완료 판정은 이 참값을 쓸 수 없다.

mocap 차량은 5절 결과 때문에 마찰 접촉 검증에 쓸 수 없다. 현재의 11 cm 박스와 마찰 없는 접촉으로도 미끄러짐을 검증할 수 없다. 조립체 하부 형상(Flow deck, 핀, AI-deck 위/아래)은 공식 CAD 내용을 확인하기 전까지 막힌 항목이며, 패드 마찰은 차량 쪽 실험 설정으로 표기한다.

## 11. 재현성과 초기 조건

- upstream 잡음·bias·바람은 seed 없는 전역 `np.random`을 쓴다([IMU bias](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:321), [Flow 잡음](../../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1065)). wrapper가 생성 전에 seed를 고정하고 에피소드마다 기록한다.
- 에피소드는 1단계부터의 시도 1회다. 추정기·펌웨어는 에피소드 사이에만 재시작하고 단계 사이에는 재시작하지 않는다. 3단계 단독 시험은 진입 상태 분포를 명시한 조건으로 표시한다.
- 초기 조건 분포에는 상대 위치·yaw, 차량 시작 위상, 카메라 프레임 위상, 지연 표본을 넣는다. 평가 seed 50개를 고정하고 튜닝·학습 seed와 분리한다.
- 렌더·스레드 타이밍 때문에 같은 seed라도 비트 단위로 같은 결과가 나오지 않을 수 있다. 같은 seed를 반복 실행해 변동을 따로 재고 결과는 분포로 보고한다.

## 12. 지연·가림·FF ablation·RL 비교 경계

- **주입 조건**: 지연은 카메라 frame age, 로그 age, 명령 전송에 따로 주입한다. 가림은 코너 마스킹, 단일 프레임 손실, 연속 손실로 나눈다. FF는 없음, nominal, 추정값의 세 조건을 모든 지연·가림 조건과 교차한다.
- **RL 비교**: [control-interface.md](../control-interface.md) 8절의 경계를 따른다. 시뮬레이션 쪽에서는 같은 프로파일 해시, 관측 허용 목록, 안전 계층, 초기 조건 분포, 지연·가림 조건과 학습에 쓰지 않은 평가 seed를 두 방법에 똑같이 적용한다.
- **참값 사용**: 보상 계산의 참값 사용은 학습 단계에서만 허용하고 기록한다. 참값을 받는 critic은 별도 조건으로 표기하며 학습 샘플 수와 튜닝 시도 수를 함께 보고한다.

## 13. 착수 파일, 진행 불가 항목, 사용자 확인

1. `config/sim_profiles.toml`: 요소별 값·단위·출처 URL·확인일을 기록하고, 등급을 `manufacturer`, `firmware_constant`, `third_party`, `experiment`, `unresolved`로 나눈다. Flow 지급 여부와 연산 위치 축을 두며, 실행기는 `unresolved`가 남은 프로파일을 `cf2.1` 이름으로 실행하지 않는다.
2. `scripts/surface_sensor_bench.py`: S3a의 T1–T5를 실행하고 결과 JSON과 그림을 남긴다. 상류 소스는 수정하지 않고 wrapper로 처리한다.

그다음 작업은 관측기·제어기 계약이 정해진 뒤의 `scripts/camera_kinematic_bench.py`(S1)다.

진행할 수 없는 사양 항목은 조립체 질량·CoM·관성 텐서, 표준 2.1 구성에 적용 가능한 추력곡선·모터 시정수·공력, 방전 곡선, `T_BC`와 스트림 모드 내부 파라미터·왜곡, 영상 fps·지연, 덱 버전·적층 순서, Flow/ToF 센서 원점, 조립체 접촉 형상이다.

사용자에게 지금 추가로 확인할 사항은 없다. S2 이후 보류와 S0·S1·S3a 진행은 확정된 계획이다. PC 연산 범위와 Flow 지급 여부는 사업단 안내를 기다리며, 외부 합격 기준이 없다는 점은 이미 확인되었다.

## 출처 (확인일 2026-09-30)

- 기존 S1–S10: [manufacturer-specs.md](../manufacturer-specs.md), 장착: [mounting-review.md](../mounting-review.md)
- MuJoCo 3.14.0: https://mujoco.readthedocs.io/en/3.14.0/modeling.html, https://mujoco.readthedocs.io/en/3.14.0/APIreference/APIfunctions.html, https://mujoco.readthedocs.io/en/3.14.0/APIreference/APItypes.html
- Bitcraze: 4절의 커밋 고정 URL, https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/pwm-to-thrust/, https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/
