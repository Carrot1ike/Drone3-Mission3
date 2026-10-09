# Mission3 하드웨어·센서 근거

확인 시각(실제): 로컬 판독 2026-10-08 01:52 KST 시작, 공식 원격 저장소 클론 02:00, 마지막 원격 조회 약 02:08, 클론 판독 약 02:16, 최초 작성 02:17:32, 보완 02:20 KST 이후(UTC 2026-10-07 16:52–17:08 원격 조회, 17:20 보완 시작). 대상: 표준 Crazyflie 2.1 + AI-deck 1.1(단색 HM01B0) + Flow deck v2.
실물 덱 배정·하드웨어 개정은 미확정이며 덱 값은 조건부 프로파일이다. 기계 판독용 요약:
[mission3-hardware-evidence.json](../../config/mission3-hardware-evidence.json).
[provided-hardware.json](../../config/provided-hardware.json)의 null(관성·장착·보정)은 유지하며 이 문서는 그 값을 채우지 않는다.

표기: **P** 제조사 공개 사실 · **C** 제조사 코드·설정 상수(측정 사양 아님) · **D** P/C에서 계산한 도출값 ·
**S** 민감도 조건(사양 아님) · **U** 미확인·미해결.

시나리오(사용자 최신 확인, AGENTS의 과거 30 cm·0.3 m/s 치수를 대체): 착륙장 20 cm 정사각, 상면 지면 위 30 cm.
마커판 20 cm 정사각, 코드 15 cm 판 중앙(확인). 판 하단 = 착륙장 상면 높이(30 cm), 상단 50 cm.
착륙장 앞 가장자리→마커 면 수평 15 cm, 인쇄면 후방(착륙장) 향함. 차량 0.25 m/s, 직선 20 m, 반원 지름 7.5 m.

## 1. main용 핵심 결론

1. 제조사 원본으로 확정되는 것은 **외형·단품 질량·추진 구성별 추력 다항식**까지다. 조립체 관성·무게중심·모터 응답·공력,
   카메라→기체 변환 `R_BC, t_BC`, 렌즈 내부 파라미터는 제조사 원본에 없다(U).
2. **조사 범위(2절 고정 커밋의 hardware·bitcraze-mechanics·crazyflie-simulation·aideck-gap8-examples 저장소와 공식 문서)에서 AI-deck 카메라 홀더·덱 조립체의 공식 정량 장착 CAD를 확인하지 못했다.** 존재하지 않는다고 확정한 것은 아니다. 확인된 자료는 카메라 모듈 단품 도면과 AI-deck 평면 도면이다.
   공식 사진·교체 안내는 카메라가 덱 FRONT 끝 홀더에서 덱 평면 방향 전방을 본다는 **정성적 근거**만 준다.
3. 고정 커밋 d39d2ec5의 공식 스트리머 빌드 옵션에 **320×320 출력 모드는 없다**: `qvga` 324×244(기본), `qqvga` 162×122, `native` 324×324.
   영상 헤더에 촬영 시각이 없고, 공개된 실측 fps·지연값도 없다.
4. 착륙장 중심 위 마커까지 **광축 깊이**는 `0.25 − x_f` m(피치 0)로 공개 피사계 심도 하한 0.30 m보다 가깝다.
   92 mm 외곽 전체가 착륙장 안이면서 깊이 ≥ 0.30 m가 되려면 `x_f ≤ 4 mm`여야 한다(D). 초점거리·F수 미공개로 근거리 흐림량은 계산할 수 없다(U).
5. 펌웨어 EKF는 ToF 거리를 높이 상태 `Z`로, Flow를 그 `Z` 위 표면 대비 속도로 해석한다.
   이동 상판 위에서 `Z`·속도가 상판 기준이 된다는 것은 측정모델 구조의 해석이며 필터 시험 전이다.
6. 공유 SIL `compute_flow`는 펌웨어 측정모델과 **배율(409.26 대 488.32 raw px/rad), 회전계수(1.25 대 1.0), 표면속도, SITL 게이트**가 다르다.
   0.838은 **대수적 센서 배율 불일치**이며 EKF 수렴값으로 시험하지 않았다.
7. Flow/ToF 재생은 **공식 최신 펌웨어 f6e0f0a3의 Python 바인딩**을 써야 한다. 로컬 고정 포크의 바인딩에는 `mm_flow.c`·`mm_tof.c`가 없다(6.1절).
8. 실기체·보정 없이 가능한 접촉 시험은 시뮬레이션 기하·접촉 시험뿐이며, 결과는 S 조건 아래 내부 검사다.

## 2. 고정 출처

| 저장소·자료 | 고정 커밋 | 확인 내용 |
| --- | --- | --- |
| bitcraze/hardware | 8207400d3e6090ea1dce2b22ddaa25243f718db1 | 2.1·2.1+·AI-deck 1.1·mono camera·Flow v2 데이터시트 원문 `src/products/*/datasheet/index.md`, 기계 도면 PNG. 제품 폴더 최종 변경 3ad4739c(2025-11-19, 상표 표기) |
| bitcraze/crazyflie-firmware | f6e0f0a3b526861328caf3296a7ecf00bb915d39 (2026-10-02) | Flow/ToF 드라이버, EKF, `platform_defaults*.h`, `Kconfig`, `controller_lee.c`, 바인딩, `docs/functional-areas/{pwm-to-thrust,battery_compensation}.md` |
| bitcraze/aideck-gap8-examples | d39d2ec5001f94bf774dbd1187ae4cf869a3094d | `examples/other/wifi-img-streamer/*`, `docs/examples/wifi-streamer.md`, `docs/guides/camera-replace.md`, `lib/cpx`. 스트리머 최종 기능 변경 2953bb2c(2026-08-17) |
| bitcraze/crazyflie-simulation | 7e93752dbc803af2488c1db46bb79b3da55f5d8c | 메시(MIT), `docs/functional_areas/meshes.md`, Webots PROTO, Gazebo SDF |
| bitcraze/bitcraze-mechanics | c70aa74368e713734ddebbf14238fd6c3c2079c6 | `models/`, `motor-mounts/`, `propellers/` (CC BY-NC-SA 3.0) |
| bitcraze/aideck-esp-firmware | 58f15fa5e14ec888f9ee965b6236aaf6d5c9e269 | `WIFI_TRANSPORT_MTU 1022`, TCP 5000 |
| GreenWaves gap_sdk (칩 SDK, Bitcraze 아님) | a23026507efe57410b98c5945544cff3150eb996 | `rtos/pmsis/bsp/camera/himax/himax.c`: QVGA 행 `320+4`, CPI `roi.slice` |
| 로컬 고정 llanesc 포크(제3자) | firmware aa6571dc465f06f7d1f9aaf7b0b861fbcd1b3d67, simulation 89d8cf79fb722bf4bd7e363ade7d90d6c45d6d5d | `crazysim.py`·`mm_flow.c`·`sensors_sitl.c` 로컬 무수정 확인. llanesc 기본 브랜치 HEAD 9fc77d29는 고정 커밋보다 24커밋 뒤이며 더 새로운 `compute_flow` 없음 |
| Bitcraze 웹 문서 | 2026-10-08 조회 | [Flow deck 튜토리얼 Measurement details](https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/), [Flow deck v2 제품 페이지](https://www.bitcraze.io/products/flow-deck-v2/) |

읽기 전용 검토 클론은 프로젝트 밖 `/tmp/drone3-evidence`에 있으며 프로젝트 산출물이 아니다. 외부 저장소는 변경하지 않았다.

## 3. 기체 모델 근거

| 항목 | 값 | 분류 | 출처 |
| --- | --- | --- | --- |
| 2.1 이륙 질량·크기 | 29 g, 92×92×29 mm(모터 간, 모터 마운트 발 포함) | P | hardware `crazyflie-2_1/datasheet/index.md` L92–93 |
| 2.1 PCB 도면 | 대각 80.000 mm, 56.650 mm, 헤더 간격 30.000 mm. 치수 기준 형상이 도면에서 명확하지 않음 | P/U | `crazyflie_2_1-measurements.png` |
| 프로펠러 구분 | 2.1 구성품 `propeller-pack`, 2.1+는 47-17 + 업그레이드 배터리. 표준 2.1은 펌웨어 `CRAZYFLIE_LEGACY_PROPELLERS`에 해당한다고 해석 | P→D | 2.1 L108–109, 2.1+ L10·L107–108 |
| 펌웨어 기본 추진 설정 | `default CRAZYFLIE_21_PLUS`, 즉 기본 빌드는 표준 2.1 프로펠러가 아님 | C | `Kconfig` L99 |
| legacy 추력 다항식 | `VMOTOR2THRUST0..3 = -0.014830744918356092, 0.04724465241828281, -0.01847364358025878, 0.005960923942142`; `THRUST_MIN 0.012817578393224994 N`; `THRUST_MAX 0.12 N`(배터리 일관성용 상한); `THRUST2TORQUE 0.007350862856566459 m` | C | `platform_defaults_cf2.h` L84–91. Bitcraze 추력대 측정 기반으로 문서화, 오차·개체차는 미공개 |
| 질량·팔 상수 | `CF_MASS 0.029`(업그레이드 키트 0.0325), `ARM_LENGTH 0.046 m`. 로컬 포크는 `CF_MASS 0.027`, 추력표 없음 | C | 같은 파일 L47–56 |
| 관성 | 제조사 측정값 없음. `controller_lee.c` L53 `J = {16.571710e-6, 16.655602e-6, 29.261652e-6}`은 Förster 2015 ETH **Crazyflie 2.0** 학위논문 인용(제3자) | U(사용 금지) | firmware L49–53 주석 |
| Bitcraze 시뮬레이터 설정 | Gazebo 0.025 kg + 위 관성·비대각 항. Webots 0.05 kg, 카메라 `fieldOfView 0.87` rad(49.8°) 324×324, 위치 (0.03, 0, 0.01) m. 데이터시트와 모순되어 사양으로 쓰지 않음 | C(시뮬 설정) | `model.sdf` L17–27, `Crazyflie.proto` L83–87·L223 |
| 공식 메시 | `cf2_assembly`(덱 없음, 저폴리). `meshes.md` 조립 위치: 프로펠러 (±0.031, ±0.031, 0.022), 모터·마운트 (±0.031, ±0.031, 0.014), 본체 z 0.015, 배터리 홀더 0.025, 배터리 0.02, 핀헤더 (0, ±0.011, 0.014) m | P(메시 문서) | 0.031 m은 모터 대각 87.7 mm로 PCB 도면 80.000 mm·`ARM_LENGTH` 46 mm와 기준이 다름(U) |
| mechanics 저장소 | `CF21_MotorMounts.stl`(2025-10-01) 사용 가능. 47-17 프로펠러 STL은 2.1+용. `cf2_model.skp`(2016)는 커뮤니티 기여 CF2 rev B. `PCB_Edges.dxf`(2013)는 Crazyflie 1 시대로 2.1 본체 CAD로 쓰지 않음 | P/D | CC BY-NC-SA 3.0(비상업) |
| 덱 | AI-deck 1.1: 4.4 g, 30×52×8 mm, PCB 28.600×45.400 mm, 홀더 나사 간격 12.000 mm, P3 FRONT, 핀 15–4–6 mm. Flow v2: 1.6 g, 21×28×4 mm(도면 27.940×21.079 mm) | P | 각 `index.md`·도면 |
| 질량 합 | 29 + 4.4 + 1.6 = 35.0 g, 핀헤더·홀더 포함 여부 미확인 | D | — |
| 정지비행 추력 | 29 g: 모터당 0.0711 N(`THRUST_MAX`의 59.3%, legacy 모터전압 2.343 V). 35 g: 0.0858 N(71.5%, 2.571 V). 공식 최대 권장 페이로드 15 g. 모터 응답 미확정으로 제어 여유는 판단하지 않음 | D/U | 위 다항식 |

## 4. 카메라 근거

공개 사양(P, `ai-deck-mono-camera/datasheet/index.md`):

- 센서·판독: HM01B0-MNA, 활성/유효 324×324/320×320, 3.6 µm, Image Diagonal 1.63 mm, 롤링 셔터, 6/8-bit RAW.
- 광학: Focus distance 35 cm, Depth of Field 30 cm ~ infinite, FOV 수평 87°·수직 87°·대각 115°.
- 프레임률: 324×244 윈도 최대 60 FPS, 2×2 binning 최대 120 FPS. 상하 반전·좌우 미러 판독 지원.
- 전체 센서 데이터시트는 "contact Himax"로 비공개다.

모듈 도면(P): 모듈 높이 5.32±0.30 mm, 렌즈 상단–강판 하단 3.27±0.15 mm, 홀더 5.00±0.10 mm 정사각, 커넥터–렌즈 중심 10.00 mm,
전체 14.75 mm, OP FOV 115°, ME FOV 125°. 모듈 단품 치수이며 홀더·덱·기체에서의 위치가 아니다.

장착 근거는 정성적이다. AI-deck 제품 사진과 `camera-replace.md`(교체 사진 1–8)에 따르면 모듈은 검은 플라스틱 홀더 안에 있고,
홀더는 덱 FRONT 쪽 12.000 mm 간격 구멍 두 개에 나사로 고정되며, 렌즈는 덱 평면 방향 바깥(전방)을 본다.
조사 범위에서 홀더의 정량 CAD·치수를 확인하지 못했으므로 `t_BC`, `R_BC`는 U다. AI-deck은 위·아래 장착이 모두 허용되므로(P) 최소 두 이산 구성을 다룬다.

도출값(D):

- `320 × 3.6 µm × √2 = 1.629 mm`가 공개 대각 1.63 mm와 일치한다. 공개 FOV는 320×320 유효 영역 기준으로 보는 것이 타당하다.
- 공칭 핀홀 `f = 160 / tan 43.5° = 168.6 px`. 같은 핀홀이면 대각 FOV가 106.6°로 공개 115°와 맞지 않는다.
  왜곡이 있거나 FOV 정의가 다르다는 뜻이며 이상적 핀홀은 근사다.

스트림(C, aideck-gap8-examples d39d2ec5):

| 항목 | 코드 근거 |
| --- | --- |
| 해상도 | `Makefile` L8–25: `CAMERA_RESOLUTION ?= qvga`(324×244), `qqvga`(162×122), `native`(324×324). `native`는 `SENSOR_FRAME_RATE=60`과 함께 쓸 수 없음(L38–39) |
| 헤더 | `cpx_stream.c` L30–38: `<BHHBBI>` = magic 0xBC, width, height, depth=1, type(0 raw, 1 jpeg), size. **촬영 시각 필드 없음** |
| 방향 | `camera_pipeline.c` L33·L124: 레지스터 `0x0101`에 3을 쓰고 읽어서 확인. 비트 의미(반전·미러)는 Himax 레지스터 표 비공개로 U |
| 캡처 | 기본 `start-stop`. `pipelined`는 버퍼 3개, 처리가 늦으면 프레임 전체를 버리고 최신 프레임 우선 |
| 계측 | `OUTPUT_PROFILING_DATA=1`이면 `q_ms, enc_ms, bytes, tx_ms, frame_ms, wait_ms, drops`(`wifi-img-streamer.c` L214). 기체 위 구간만 재며 Wi-Fi·PC 구간은 미포함 |
| 전송량 | CPX MTU 1022, 헤더 2 → 패킷당 1020 B. native 104,976 B = 103패킷, qvga 79,056 B = 78패킷, 320×320 = 102,400 B(D). 처리량·fps·지연 공개값 없음(U) |
| 320×320 | GAP8 SDK는 QVGA 행을 `320+4`로 읽고 CPI `roi.slice` 잘라내기를 지원하므로 기체 위 320×320 잘라내기는 소프트웨어로 가능해 보인다. 공식 예제는 하지 않으며 유효 320 창 오프셋은 미공개(U). Bitcraze 도커 이미지의 SDK 버전은 미확인 |

### 4.1 native 320 유효창과 87/87/115° 모순에 대한 시험정책

- 유효창: 공식 `native`는 324×324 전체 배열이다. 유효 320×320 창의 오프셋은 공개되지 않았다(U).
  공개 대각 1.63 mm가 320 화소와 일치하므로(D) 시뮬레이션은 지금처럼 320×320을 직접 렌더링하고 FOV를 320 기준으로 정의한다.
- 실기 어댑터: 중앙 잘라내기(테두리 2 px)는 가정이다. 잘라내기 오프셋 불확실성 때문에 주점 `cx, cy`를 ±2 px 범위(S)로 흔든다.
  `0x0101 = 3` 방향 설정의 비트 의미가 미확인이므로 영상 축 규약도 `{그대로, 180° 회전}` 두 경우(S)로 시험한다.
  어느 경우에도 resize하지 않으며, 최종값은 실기 보정으로 대체한다.
- 87/87/115° 모순: 왜곡 없는 핀홀이면 대각은 106.6°, 등거리 투영이면 123.0°다. 공개 115°는 그 사이이므로 중간 정도의 배럴 왜곡이나 서로 다른 FOV 정의와 맞는다.
  세 값을 동시에 만족하는 1매개변수 Brown 모델(OpenCV 규약, 정규화 좌표)은 `fx = fy ≈ 182.7 px`, `k1 ≈ −0.0856`다.
  이 모델은 63.1°까지 단조이며, 단일 매개변수 가정 아래의 도출값(D)이므로 보정값이 아니라 S 후보로 쓴다.
- 시험 묶음: M0은 핀홀 168.6 px(현재 기준), M1은 위 k1 모델, M2는 비교용 등거리 210.7 px다.
  렌더러와 추정기의 모델 조합(M0/M0, M1/M0, M1/M1)으로 검출률과 자세 편향을 보고한다.
  MuJoCo는 왜곡을 직접 렌더링하지 못하므로, 더 넓은 핀홀로 렌더링한 뒤 320×320 합성 센서 영상으로 remap한다. 이는 센서 모델 생성 단계이며 입력 영상 resize가 아니다.
- 판정: 착륙 판단 로직은 M0–M1 범위에서 유지되어야 한다. 실제 K와 왜곡은 실기 체커보드 보정으로 교체한다.

## 5. 착륙 기하와 피사계 심도(조건부)

기준좌표: 착륙장 상면 중심 원점, x는 마커 쪽(차량 전방), z는 위. 마커 면 `x = 0.25` m, 코드 모서리 `y = ±0.075`, `z = 0.025 / 0.175` m.
카메라는 공칭 핀홀(f = 168.6 px, 주점 160,160, 왜곡 없음)로 둔다. `x_f`(광학중심 전방 오프셋), `z_cam`(광학중심의 상면 위 높이),
`θ`(광축 하향 피치)는 **S이며 추정값이 아니다.**

| x_f (S) | 광축 깊이 d | 코드 높이 | 네 모서리가 보이는 z_cam(θ=0) | 공개 DOF(≥0.30 m) |
| ---: | ---: | ---: | --- | --- |
| 0 | 0.25 m | 101.2 px | ≤ 0.262 m | 밖 |
| 0.02 m | 0.23 m | 110.0 px | ≤ 0.243 m | 밖 |
| 0.04 m | 0.21 m | 120.4 px | ≤ 0.224 m | 밖 |

- 피치(d = 0.25): θ = −10°(기수 상향)이면 `z_cam ≤ 0.190`, +5°이면 `−0.024 ≤ z_cam ≤ 0.308`, +10°이면 `0.010 ≤ z_cam ≤ 0.363` m. 좌우 모서리는 전 조건에서 화면 안이다.
- 피사계 심도 판단에는 마커 중심까지의 직선거리(x_f = 0에서 0.25–0.29 m)가 아니라 **광축 깊이**를 써야 한다.
  피치 0으로 수직 하강하면 깊이가 일정하므로 문제는 접촉 순간이 아니라 **착륙장 중심 위 최종 하강 전체**에 걸친다.
- 외곽 조건(D): 92 mm 외곽 전체가 20 cm 착륙장 안에 들려면 기체 중심이 착륙장 중심에서 ±0.054 m 이내여야 한다.
  가장 뒤쪽 중심은 마커 면에서 0.304 m이므로 깊이 ≥ 0.30 m는 `x_f ≤ 0.004 m`일 때만 성립한다(피치 0).
  카메라는 정성적으로 덱 FRONT 끝에 있어 `x_f > 0`이 예상되지만 크기는 미공개다.
- 디옵터: 초점 35 cm = 2.857 D, DOF 하한 30 cm = 3.333 D, 25 cm = 4.000 D, 21 cm = 4.762 D.
  "초점 35 cm, DOF 30 cm–∞"는 단순 박막렌즈 과초점 모델과 맞지 않는다(그 모델에선 원거리 한계가 ∞이면 근거리 한계 ≤ 17.5 cm).
  제조사의 흐림 판정 기준을 알 수 없으므로 21–25 cm 흐림이나 ArUco 검출 가능성을 외삽하지 않는다(U).
  엄격 초점 시험에서 0.30 m를 합격선으로 쓰는 것은 정책 선택이며, 흐림 민감도 시험과 구분해 보고한다.

### 5.1 `strict_focus` 게이트 해석과 시험정책

- 현재 코드: `scripts/mission3/scene.py` L132–141의 `image(strict_focus)`는 광학점→마커 중심 **직선거리**가 `published_depth_of_field_near_m`(0.30)보다 작으면 프레임을 거부한다.
  main 소유 코드이며 이 작업에서는 수정하지 않았다.
- 직선거리와 광축 깊이: 어떤 점이든 광축 깊이 ≤ 직선거리다. 따라서 직선거리 게이트가 거부하는 프레임은 깊이 게이트도 거부하지만, 반대는 성립하지 않는다.
  예: `x_f = 0`, `z_cam = 0.30` m이면 직선거리는 0.320 m라 통과하지만 중심 깊이는 0.250 m다. 직선거리 게이트는 공개 DOF 해석보다 느슨하다.
- 판면 수직거리: 마커 판까지의 수직거리는 광축이 판에 수직일 때(피치 0, 요 정렬)만 광축 깊이와 같다.
  초점면은 광축에 수직이므로 판이 기울어 보이면 판 위 점마다 깊이가 다르다. 평면이므로 최솟값은 모서리에서 나온다.
- 공식 DOF 정의: 데이터시트는 "Focus distance 35 cm, Depth of Field 30 cm ~ infinite" 두 값만 준다.
  측정 기준점(렌즈 앞면·주평면·센서)과 흐림 판정 기준(CoC·MTF)은 공개되지 않았다.
  기준점 차이는 모듈 높이 5.32 mm 수준이라 0.30 m에 비해 작지만 0은 아니다.
- 시험정책:
  1. 엄격 게이트 지표는 **코드 네 모서리의 최소 광축 깊이 ≥ 0.30 m**로 한다. 이는 보수적 정책 선택이다.
     추적용으로 중심 깊이, 중심 직선거리, 판면 수직거리를 함께 기록하고, 판(quiet zone 포함) 모서리 기준 변형은 선택 사항으로 둔다.
  2. 게이트 통과가 선명함을 뜻하지 않고, 거부가 검출 불가를 뜻하지도 않는다. 의미는 "공개 DOF 범위 밖 관측 배제"다.
  3. 흐림 민감도는 PSF 범위(S)를 쓰는 별도 시험으로 하고, 제조사 광학값으로 표기하지 않는다.
  4. 단계별 게이트 거부 비율과 시각을 보고한다. 착륙장 중심 위 최종 하강은 피치 0에서 깊이가 일정해(5절) 전 구간 거부가 예상된다.

## 6. Flow/ToF와 펌웨어 필터

공개 사실(P): VL53L1x 최대 4 m, 표면·조명에 따라 수 mm. PMW3901은 무광 표면에서 가장 잘 동작한다.
ToF는 원뿔 안 **가장 가까운 검출**을 보고하고, Flow는 평평한 바닥을 가정하며 무늬 없는 표면에서 어렵다.
클라이언트 hover 모드 높이 0.2–3.0 m는 보조 모드 제한이며 센서 사양이 아니다.

### 6.1 재생용 공식 소스(f6e0f0a3, 바로 읽을 경로)

| 경로 | 읽을 내용 |
| --- | --- |
| `src/deck/drivers/src/flowdeck_v1v2.c` | 100 Hz 루프 L88, 축 변환 L94–95, 이상치 `OULIER_LIMIT 100` L45·L98, std L73·L77·L105, 측정 dt L121, `motion == 0xB0` 게이트 L157. Flow v2 초기화 시 zranger2 구동 |
| `src/drivers/src/pmw3901.c`, `src/drivers/interface/pmw3901.h` | `motionBurst_t`: `motion, deltaX, deltaY, squal, shutter` (L33–57) |
| `src/deck/drivers/src/zranger2.c` | MEDIUM 모드 L124, 측정 예산 25 ms L125, 25 ms 주기 L132, 5000 mm 게이트 L54·L140, std 모델 L48–51·L103 |
| `src/drivers/src/vl53l1x.c` | VL53L1 드라이버 |
| `src/modules/src/range.c` | `rangeEnqueueDownRangeInEstimator` L51–56 → `estimatorEnqueueTOF` |
| `src/modules/interface/stabilizer_types.h` | `flowMeasurement_t` L291–303, `tofMeasurement_t` L307–311 |
| `src/modules/src/estimator/estimator_kalman.c` | 예측 100 Hz L119·L248, 공정잡음 L259, ToF L335–336, Flow L341–342(`gyroLatest`, deg/s, L352) |
| `src/modules/src/kalman_core/mm_flow.c`, `mm_tof.c`, `kalman_core.c` | 측정모델·스칼라 갱신·예측 |
| `src/modules/interface/kalman_core/kalman_core.h`, `kalman_core_params_defaults.h` | `kalmanCorePredict(this, params, acc, gyro, nowMs, quadIsFlying)` L174, `kalmanCoreDefaultParams` L159 |
| `src/platform/interface/platform_defaults.h` | `FLOWDECK_POS_*` 기본 0 L239–247, `DRAG_B_*` 기본 0 L249–258 |
| `bindings/setup.py`, `bindings/cffirmware.i` | Python 바인딩에 `kalman_core.c, mm_tof.c, mm_flow.c` 포함(setup L51–57, .i L26–51) |
| `bindings/util/estimator_kalman_emulator.py` | 1 kHz 루프·100 Hz 예측 템플릿(169줄). **Flow/ToF 분기 없음**이므로 프로젝트 wrapper에서 추가 |

로컬 포크 aa6571dc의 `bindings/setup.py`에는 `kalman_core.c`(L50)만 있고 `mm_flow.c`·`mm_tof.c`가 없다.
`kalmanCorePredict`에도 `params` 인자가 없어 공식판과 API가 다르다. cf2 기본값에서는 Flow/ToF 측정식이 같다.

로컬 복사본 상태(읽기 전용 확인, 02:2x KST): `src/bitcraze-mission3-firmware`는 HEAD f6e0f0a3이다.
원본 클론은 `--depth 1`, `blob:limit=5242880` 부분 클론이고 `vendor/CMSIS`가 비어 있다.
`bindings/setup.py` L19–34는 `vendor/CMSIS` 헤더와 DSP 소스를 요구하므로 CMSIS를 채워야 빌드된다.
필요 커밋 a65b7c9a3e6502127fdb80eb288d8cbdf251a6f4는 로컬 `src/crazyflie-ros2-sil/crazyflie-firmware/vendor/CMSIS`와 같다.
`git submodule update --init vendor/CMSIS`(읽기 전용 원격 조회)나 같은 커밋 복사로 채울 수 있으며, 측정식을 다시 구현할 필요는 없다.

### 6.2 코드값(C)

- `mm_flow.c`: `Npix 35`, `thetapix 0.71674`(2·sin 21°, 주석상 42°는 "guessed"), `FLOW_RESOLUTION 0.10`, `z ≥ 0.1` 클램프(L31·L47·L48·L65).
  예측 `N = dt·Npix/thetapix·(v·R22/z − ω_y)`, y축은 `+ω_x`. 공식판 b5d21e69(2026-02-27)의 레버암 항은 cf2 `FLOWDECK_POS = 0`이라 영향 없음.
- `mm_tof.c`: `R22 > 0.1`일 때만 갱신(L35), `angle = acos(R22) − 7.5°`(0 클램프, L36), `예측 = Z / cos(angle)`(L40).
- `zranger2.c`: `std = 0.0025·(1 + e^{2.9214(d − 2.5)})` m → 0.3–1.0 m에서 약 2.5 mm.
- 공식판 추가 사항(항력 cf2 기본 0, general-purpose 옵션 등)은 위 측정식과 무관하며 EKF 전체 차이는 감사하지 않았다(U).

### 6.3 가상 센서 제한 권고

| 제한 | 값 | 분류 | 재생 처리 |
| --- | --- | --- | --- |
| ToF 주기 | 40 Hz, 예산 25 ms | C | 40 Hz로 샘플·enqueue |
| ToF 최대 | 4 m(P), 펌웨어 게이트 5 m(C) | P/C | 4 m 초과는 측정 없음 |
| ToF 최소 | Bitcraze 값 없음 | U | 최소 유효거리를 S 범위로 두고 무효 처리 |
| ToF 원뿔 | 가장 가까운 검출(P), 기울기 보정 반각 7.5°(C), 실제 FoV U | P/C/U | 기본은 7.5° 반각 원뿔 최근접, 각도는 S 범위 |
| ToF 자세 게이트 | `R22 ≤ 0.1`이면 미사용 | C | 그대로 재현 |
| ToF std | 위 지수 모델 | C | 드라이버 값으로 enqueue |
| Flow 주기·dt | 100 Hz, 실측 dt | C | 10 ms 누적, dt 기록 |
| Flow 이상치 | 축별 \|count\| ≥ 100 폐기 → EKF 단위 \|N\| < 10, dt 0.01에서 \|v/z\| < 20.5 s⁻¹ | C/D | 이 시나리오에서는 거의 걸리지 않음(0.25 m/s, z 0.1 m → 2.5 s⁻¹) |
| Flow 게이트 | `motion == 0xB0`일 때만 반영 | C | 표면 무늬 품질 모델로 결측 생성. 패드·바닥 무늬는 실험 설정으로 명시 |
| Flow std | 2.0 고정(적응형 꺼짐) | C | 2.0 |
| Flow 저고도 | EKF는 `Z < 0.1 m`에서 0.1로 계산. 실제 배정 deck 최소 높이는 U; PixArt 부품 80mm는 12절 | C/U, 부품 P 별도 | 칩 범위 밖 Flow를 유효 속도 증거로 쓰지 않음. 기존 sweep은 S이며 실제 deck 검증 아님 |
| 관측 표면 | 광선이 맞은 표면 대비 상대 운동(P 해석) | P→D | `v_rel = v_sensor − v_surface` |
| 축 | 칩 출력이면 `accpx = −deltaY`, `accpy = −deltaX` | C | 드라이버 출력 단계에서 생성하면 생략 |

재생 구성 권고:

1. **일관성 모드**는 펌웨어 측정식 자체(488.32 raw px/rad, 회전계수 1.0)와 실제 상대속도·실제 표면거리로 측정을 생성한다.
   배관과 단위를 확인하는 단계로, 혁신값이 잡음 수준이어야 한다.
2. **교란 모드**는 배율·회전계수·원뿔각·결측을 S 범위로 흔든다. 공유 SIL 값(409.26, 1.25)은 그중 이름 붙인 한 사례로 넣어 기존 불일치의 영향을 따로 정량화한다.
3. 드라이버 단계(이상치·게이트·std)는 wrapper에서 재현한다. SITL 경로 `sensors_sitl.c` L247–256은 이 단계를 건너뛴다.
4. 시나리오는 정지 바닥, 바닥→착륙장 30 cm 전환, 직선 0.25 m/s, 반원 R 3.75 m, `z_cam` 약 0.26 m에서 접촉까지 하강(`Z < 0.1` 통과), 모서리 혼합 반환이다.
5. 보고에는 혁신값, `Z` 추정 대 실제 표면거리, 속도 추정 대 상대속도를 쓴다. 이는 실기 검증이 아닌 내부 재생 결과다.

### 6.4 측정모델 구조 해석(D, 필터 시험 전)

- `Z`는 아래 관측 표면까지 수직거리로 동작한다. 바닥→착륙장 전환에서 ToF가 30 cm 단차를 보고하고,
  근거리 std 약 2.5 mm라서 EKF가 이를 `Z` 변화로 빨리 받아들일 수 있다. 그러면 `Z` 고정 제어기는 상승할 수 있다. 착륙장 위 Flow는 차량 기준 상대속도다.
- 직선 0.25 m/s 구간은 차량 좌표계가 관성계이므로 IMU 모순은 주로 전환·혼합 반환에서 생긴다.
- 반원 R 3.75 m: 차량 요각속도 0.0667 rad/s(3.82°/s), 구심가속도 0.0167 m/s², 47.1 s 동안 180° 회전하는 비관성계. 기체 요는 자동으로 따라가지 않는다.

## 7. 공유 SIL `compute_flow` 불일치(대수적)

| 항목 | 펌웨어 기대 | 공유 SIL(crazysim.py 89d8cf79) |
| --- | --- | --- |
| raw px/rad | `Npix/thetapix/FLOW_RESOLUTION = 488.32` | `FLOW_NPIX/FLOW_THETAPIX = 30/4.2° = 409.26` (L112–113) |
| 회전 계수 | 1.0 | `omega_factor = 1.25` (L1057) |
| 높이 | EKF 상태 `Z` | 기체 원점에서 기체 −z 단일 광선의 실제 거리, 원뿔·센서 오프셋 없음(L1020–1033) |
| 속도 | 관측 표면 대비 | MuJoCo `cvel` 기체 속도(L1041), 표면속도 미차감 |
| 잡음·게이트 | std 2.0, `0xB0`, \|count\| < 100, 실측 dt | 잡음 0.5 px. `sensors_sitl.c` L247–256이 게이트 없이 std 2.0으로 바로 enqueue |

배율비는 병진 `409.26/488.32 = 0.8381`, 회전 `1.0476`다. 단일 측정식을 등식으로 맞추면 `v̂ = 0.838·v − 0.048·z·ω`이지만
이는 **대수적 정합식**이다. EKF는 IMU 예측·ToF·공정잡음을 함께 쓰므로 실제 편향은 6.3절의 필터 재생으로 확인해야 한다.
`cvel` 기준점(서브트리 무게중심)이 의도와 맞는지도 확인 항목이다.

## 8. 물리·접촉 시험 가능성

기체·Crazyradio가 없어 실물 접촉·비행·광학 시험은 지금 불가능하다. AI-deck 실물이 없으면 근거리 초점 시험도 할 수 없다.
시뮬레이션에서는 데이터시트 외곽과 공식 메시로 패드 적합·발 접촉·전복 경계를 검사할 수 있다.
접촉 강성·마찰·발 탄성·관성·모터 응답이 미공개이므로 S 범위로 돌리고 결과는 "조건부 내부 검사"로만 쓴다.
장비 도착 후 측정 순서:

1. 질량과 무게중심(매달기)
2. 관성(이선·삼선 진자 방법)
3. 체커보드 내부 보정
4. 기준 지그와 마커판으로 `T_BC`
5. 0.15–0.40 m 근거리 ArUco 검출
6. 화면 내 시계·LED로 영상 지연
7. 이동 표면(카트) 위 Flow/ToF

## 9. main 권고(제어 구현 제외)

1. 장면 광학점은 카메라 좌표 `C`로만 다룬다. 기체 `B`와 잇는 `T_BC`는 `{x_f, z_cam, θ, 덱 위/아래}`의 S 범위로 두고,
   판정이 범위 전체에서 성립해야 한다. 선택값을 provided-hardware.json에 쓰지 않는다.
2. 임시 기체 모델: 형상은 공식 메시와 데이터시트 외곽, 질량은 29 g 또는 조건부 35.0 g(D), 추력은 legacy 다항식과 `THRUST_MAX 0.12 N`(C)을 쓴다.
   관성·모터 시정수·항력·접촉은 S 범위로 두고 "모델 가정"으로 표기한다. `cf2x_P250`·Förster 값은 넣지 않으며 결과는 민감도 결과로 보고한다.
3. 카메라 입력 계약(4.1절): 공식 전체 배열은 324×324이므로 320×320 창 규칙(예: 테두리 2 px 제거)을 사용자·main이 명시적으로 결정한다.
   그 전까지 임의 resize·crop으로 통과시키지 않고 324×244는 계속 거부한다.
4. 내부 파라미터는 4.1절의 M0–M2 묶음으로 시험한다(공개 87/87/115°가 핀홀과 불일치). `strict_focus`는 5.1절의 모서리 최소 광축 깊이 지표로 바꾸는 것을 권고한다.
5. 최종 단계가 접촉 순간의 선명한 마커에 의존하지 않도록 한다. 흐림은 S 범위로 민감도를 보고,
   착륙장 중심 위 전체 마커 가시 하강 시작 높이(피치 0)는 `z_cam ≤ 0.224–0.262 m`(x_f 0.04–0)를 참고한다.
6. 촬영 시각이 없으므로 PC 수신 시각만 쓴다. 지연·프레임 간격은 S 범위로 두고 실기에서 프로파일 필드와 외부 시계로 측정한다.
7. Flow/ToF 재생은 공식 f6e0f0a3 바인딩과 `estimator_kalman_emulator.py` 템플릿을 프로젝트 wrapper에서 확장해 한다(6.1–6.3절). 외부 원본은 수정하지 않는다.
8. 표준 2.1이면 펌웨어 추진 설정은 legacy일 가능성이 크다. 실물 프로펠러 확인 후 결정하며 지금 빌드를 바꾸지 않는다.

## 10. 미확인 목록

- 조사 범위 밖 자료를 포함한 공식 정량 장착 CAD의 존재 여부
- 조립체 질량·무게중심·관성, 모터 응답, 항력, 접촉 특성
- `t_BC`·`R_BC`, 덱 위/아래 배정, 홀더 치수, 렌즈 초점거리·F수·왜곡, 유효 320 창 오프셋, `0x0101` 비트 의미
- 스트림 실측 fps·지연, native 모드 기본 프레임률, Bitcraze 도커 이미지의 SDK 버전
- 실제 배정 deck의 ToF FoV·ROI·정확도·최소거리, PMW3901 렌즈/revision별 장착 후 작동높이·FOV, Flow deck 오프셋, 이동 표면 위 EKF 실제 거동, 공식판·포크의 EKF 전체 차이(부품 최소 범위 P는 12절)
- 21–25 cm 근거리 ArUco 검출 가능성

## 11. fixture bench: 공식 C 실제 실행에 대한 독립 검증

검증 시각: 2026-10-08 02:24–02:43 KST(UTC 2026-10-07 17:24–17:43). 범위는 main의 ctypes 래퍼가 **공식 Kalman core·Flow·ToF C를 정해진 센서 fixture로 실제 실행한 것**이다.
FreeRTOS, 센서 드라이버, 비행 제어기, 기체 동역학은 포함되지 않는다. 전체 물리 비행이나 장비 검증이 아니다.
main 파일은 읽기만 했다. 대상은 `scripts/mission3/firmware_host.c`, `firmware.py`, `build_mission3_firmware_host.py`, `replay_mission3_firmware.py`, `build/mission3-firmware-host/source-manifest.json`, `artifacts/mission3/firmware-transition-{consistent,corrected,gated}`다.
독립 재현용 빌드는 프로젝트 밖 `/tmp/drone3-evidence/calc`(`verify_host*.c`, `verify_*.py`, `*_result.json`)에서만 했다.

### 11.1 원문·설정 일치

| 항목 | 결과 |
| --- | --- |
| 원문 무결성 | `src/bitcraze-mission3-firmware` HEAD f6e0f0a3, `git status` 깨끗함. `kalman_core.c`·`mm_flow.c`·`mm_tof.c` sha256이 검토 클론과 같음(d0215770…, 13654725…, 0cd4c176…). CMSIS a65b7c9a는 공식 서브모듈 포인터와 같은 커밋 |
| 예측 입력 단위 | 공식 `estimator_kalman.c` L382–383이 가속도는 `GRAVITY_MAGNITUDE`(9.81f), 자이로는 `DEG_TO_RAD`로 변환한다. 래퍼의 `specific_force_body_m_s2 = (0, 0, 9.81)`, `gyro_body_rad_s`가 이에 맞음 |
| Flow 자이로 단위 | 공식 L341–342·L352의 `gyroLatest`는 deg/s 원값이다. 래퍼 `gyro_body_deg_s`가 맞음 |
| 측정 입력 | Flow raw = `0.01·35/0.71674/0.1·v/h`, std 2.0, dt 0.01, 이상치 100 미만. ToF std는 zranger2 지수식. 펌웨어와 일치 |
| 주기·순서 | 예측 100 Hz(공식 L119), Flow 100 Hz, ToF 40 Hz, finalize 매 1 ms, 예측→측정→finalize 순서가 일치 |
| 공정잡음 주기 | **불일치.** 공식 L258–259는 "Add process noise every loop, rather than every prediction"이며 루프는 안정화 루프 1 kHz(`stabilizer_types.h` L371, `estimatorKalman` L289–298)다. 래퍼는 `bench_predict` 안에서 100 Hz로만 부른다. 호출당 분산이 `(a·dt + b)²`라서(`kalman_core.c` L613–625, 기본 `procNoiseVel = procNoisePos = procNoiseAtt = 0`) 10 ms 호출은 속도·자세 분산을 약 10배, 위치 분산을 약 1000배 더 넣는다 |
| 플랫폼 설정 | `-dM` 전처리 결과 `CONFIG_PLATFORM_CF2` 반영, `DRAG_B_X`·`FLOWDECK_POS_X`·`CENTER_OF_PRESSURE_X` = 0, `CF_MASS 0.029`, `KALMAN_INITIAL_YAW_STD 0.01`이다. `CONFIG_ESTIMATOR_KALMAN_GENERAL_PURPOSE`는 미정의로, 실제 Kconfig 기본 n과 같아 `procNoiseAcc_z = 1.0` |
| 추진 상수 | `CONFIG_ENABLE_THRUST_BAT_COMPENSATED`(실제 Kconfig 기본 y)가 없어 legacy 추력표가 활성화되지 않고, `THRUST_MAX`는 `platform_defaults.h` L78–79의 0.1125로 떨어진다. Kalman core는 이 상수를 쓰지 않아 추정 결과에는 영향이 없다. 빌드 스크립트의 "legacy 상수 선택" 설명은 부정확하다 |
| 빠진 공식 루프 요소 | `kalmanSupervisorIsStateWithinBounds`(최대 위치 100 m, 속도 10 m/s 초과 시 리셋), 상태 외부화, `KALMAN_DECOUPLE_XY`(정의 없음)가 빠져 있다. 이번 실행은 한도 안이라 영향 없음. 자세 복원(reversion)은 비행 중 꺼져 있어(`kalman_core.c` L589) 무관 |
| 출처 기록 | `firmware-transition-consistent/experiment.json`에 `mode` 키가 없다(이전 스크립트 버전). 아래 재현으로 stock과 같음을 확인 |

### 11.2 독립 재현 결과(같은 fixture, 펌웨어 원문, 별도 /tmp 빌드)

| 사례 | 공정잡음 10 ms(main 래퍼와 같음) | 공정잡음 1 ms(공식 루프 주기) |
| --- | --- | --- |
| 전환 전 2.9 s의 z, vx | 0.500, 0.2619 | 0.500, 0.3022 |
| stock: 패드 위 6.9 s의 z, vx / 최대 vz | 0.200, −0.0061 / 2.1793 | 0.200, −0.0241 / 1.2271 |
| corrected(지연 50 ms, 높이 +1 cm, 속도 +3 cm/s): z, vx / 최대 vz | 0.510, 0.2964 / 2.0981 | 0.510, 0.3154 / 0.8674 |
| gate_moving(지연 50 ms): z, vx / 최대 vz | −7.8621, −0.0422 / 2.0985 | −3.0149, 0.4898 / 0.8680 |

10 ms 열은 main의 `evaluation.json` 값과 소수 넷째 자리까지 같아, 래퍼가 원문을 그대로 실행함을 재확인했다.
공식 주기(1 ms)에서도 정성적 결론은 같다. 전환 단차가 수직속도와 `Z`에 흡수되고, 이미 흡수된 뒤 늦게 게이트하면 z가 표류한다.
하지만 크기는 크게 달라진다. 최대 vz는 2.18→1.23(stock), 2.10→0.87(corrected), 게이트 시 z는 −7.86→−3.01이다.

### 11.3 기준선 속도 오프셋(원인 미확정)

측정이 정확히 일관된 정속 fixture에서도 공식 EKF의 vx는 참값 0.25보다 크다. 10 ms에서는 0.2617, 1 ms에서는 0.295이며 추정 피치는 각각 0.27°, 0.51°다.
5–19.9 s 동안 값이 같으므로 짧은 수렴 과도가 아니라 정상상태다. 이 표는 공식 주기(1 ms) 기준이다.

| 참 속도 | ToF 있음 | ToF 없음 |
| --- | --- | --- |
| 0 | vx 0, 오프셋 없음 | z 0.5009, vx 0 |
| 0.10 m/s | vx 0.1611 | 발산 경향(z 1.35) |
| 0.25 m/s | vx 0.295 | 발산(z 10.6, vx 4.8) |

정지비행에서는 오프셋이 없고, 비행 중에는 자세 복원이 꺼져 있으므로 복원이 원인은 아니다.
비비행 예측 모드로 바꾸면 오프셋이 줄지만(1 ms에서 0.2722) 사라지지는 않는다.
비행 모드 예측이 횡가속도를 쓰지 않아 피치의 관측성이 약한 것이 의심되지만 확정하지 않았다.
따라서 이 fixture에서 vx 절대값을 추정 정확도로 읽지 말고, 같은 기준선의 실행끼리 상대 비교해야 한다. 실제 기체에서도 같은 현상이 생기는지는 미확인(U)이다.

### 11.4 main 권고

1. 래퍼에 공정잡음 전용 함수를 두고 매 1 ms 호출한다. 예측은 100 Hz로 유지하고, 세 모드를 다시 생성한다. 위 1 ms 열은 독립 재현의 참고값이다.
2. 전환 효과는 같은 공정잡음 주기에서 기준선(stock) 대비 차이로 보고한다. 11.3절의 오프셋을 먼저 기록한다.
3. 빌드 스크립트 설명을 실제 설정에 맞춘다. 추진 상수가 필요하면 `CONFIG_ENABLE_THRUST_BAT_COMPENSATED`를 추가한다(추정기에는 불필요).
4. 모든 `experiment.json`에 `mode`와 공정잡음 주기를 기록한다. 긴 실행에서는 supervisor 한도 검사를 추가한다.
5. 결과 표기는 "공식 추정기 C의 fixture bench 실행"으로 제한하고, 물리 비행·접촉·장비 검증으로 부르지 않는다.

## 12. 부품 제조사 최소 범위: near sensor 물리 실행의 한계

2026-10-08 Heisenberg/Codex 독립 검토 추가. **부품 사양 P와 실제 배정 deck의 보정·장착·작동 범위 U를 분리한다.** 이 절은 기존 near sensor 모델의 실기관련성 한계를 보완하며, 과거 실행 로그·프로파일·조건부 접촉 승인을 수정하지 않는다.

| 제조사 원문·개정 | 확인한 부품 값 P | 실제 deck에 관한 남은 U |
| --- | --- | --- |
| [ST VL53L1X DS12385 Rev 8, August 2024](https://www.st.com/resource/en/datasheet/vl53l1x.pdf), §7.3, 인쇄 p21 | “The minimum ranging distance ensured is 4 cm.” 4cm 아래에서도 target를 감지할 수 있으나 거리 측정은 정확하지 않음 | Bitcraze 배정 deck의 ROI·mode·offset/crosstalk calibration·표면별 정확도·장착 기준 거리 |
| [PixArt PMW3901MB-TXQT v1.40, 28 Jan 2019](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.40-280119.pdf), p1 L214-ZSZ new lens, p3 §2.2 Table 3 | Working Range Z의 **Min 80mm**, Effective Viewing Angle의 **Typ 42°**. 해당 렌즈·문서의 권장 작동 조건 | 실제 Flow deck 렌즈·revision·배정, 표면/조명·정량 장착 후 유효 FOV와 정확도 |
| [PixArt v1.00, 20 Mar 2017](https://wiki.bitcraze.io/_media/projects:crazyflie2:expansionboards:pot0189-pmw3901mb-txqt-ds-r1.00-200317_20170331160807_public.pdf), p1 LN03-ZSZ old lens, PDF 3번째/인쇄 p8 Table 3 | 이 구형 렌즈 문서도 **Min 80mm**, **Typ 42°** | 구형 문서를 현재 배정 deck에 자동 적용하지 않음. 불명 revision은 U |

PixArt 두 표의 Min/Typ 열은 렌더링으로 확인했다. 42°는 부품·렌즈의 typical effective angle이며 조립 deck의 보정 FOV나 실제 finite patch를 증명하지 않는다. 이후 다른 revision의 값·변경/철회 여부는 이번 검증 범위 밖이다. unknown revision의 최소높이를 이미 검증한 값으로 채우지 않는다. [보존 사본·검증 기록](../../artifacts/mission3/independent-review/20261008-component-sensor-envelope/component-evidence.json)의 v1.40 PDF SHA-256은 `751ad896822a1376f9fa13c94d84057eb6f65cb10b5438c1d26d0f65a1d136cb`, v1.00은 `80a9a362beb61dd2bdc052ed2fd94e55bed95b7979ab7e21feff72e0975ed408`이다. ST는 공식 웹 PDF 원문을 확인했으나 로컬 바이트 다운로드가 실패해 파일 SHA는 null로 기록했다.

**물리 실행에 미치는 해석.** 첫 seed1과 seed8은 약 9mm 센서 거리에서도 ToF metric fusion과 Flow를 허용하는 S 모델이었다. 이는 ST의 4cm 정확 거리 범위 밖이고, 확인한 PixArt 렌즈의 80mm 작동 범위 밖이다. 실제 가까운 물체의 감지 가능성과 정확한 높이·상대속도 측정을 구분해야 한다. [독립 물리 검토](physical-flight-review.md)에 따라 조건부 simulation_contact_success 승인은 유지하지만 실기 센서 HOLD·착지 관련성을 주장할 수 없고 qualified는 false다.

제조사 부품문서를 근거로 메인 개발자가 선택한 보수적 시험 정책은 **ToF raw sensor 거리 <.04m에서 metric accuracy 사용/융합 금지, Flow 센서–표면 거리 <.08m에서 유효 속도 증거 차단**이다. 이는 위 P에서 정한 보수적 실험 정책이며 해당 경계 이상이면 실제 deck 정확도를 자동 보장한다는 뜻은 아니다. 기준은 sensor 원점에서 target까지의 거리이고 body/CoM 높이와의 변환은 여전히 S/U다. 근접 구간의 IMU·추력 감소/잔차 기반 착지 관찰은 별도 새 코드·실험에서 검증해야 하며 이 문서가 이를 구현하거나 승인하지 않는다. near range `detected`, `raw_valid`, `metric_accuracy_usable`, freshness를 분리해 기록하는 것이 필요하다. 정량 mounting·동역학·캘리브레이션을 추론해서 P로 올리거나 기존 null을 채우지 않는다.


## 07:43 제조사 질량 원문 사이의 차이

2026-10-08 읽기 전용 재확인에서 [현재 Crazyflie2.1 제품 페이지](https://www.bitcraze.io/crazyflie-2-1/)·[공식 Rev3 데이터시트](https://www.bitcraze.io/documentation/hardware/crazyflie_2_1/crazyflie_2_1-datasheet.pdf)·고정 hardware 문서는29g을 표시하지만, [동일 SKU114991551의 제조사 store 페이지](https://store.bitcraze.io/products/crazyflie-2-1)는27g 설명을 남긴다. 서로 다른 제품을 임의로 합친 값이라고 단정하거나 실제 배정 기체 질량을 확정하지 않는다. 현 profile은 데이터시트29g과 AI4.4/Flow1.6g의 단순 합35g을 조건부 S로 쓰며, 완성 질량은 계속U다. 수치를33g으로 바꿔도 실제 기체 확인을 대신하지 못한다. 공식 문서 간 차이와 배정 revision·장착부·배터리 포함 범위를 추후 대조해야 한다. 현재 실험 결과는 이 공백을 포함해 제조사 일치로 승격하지 않는다.
