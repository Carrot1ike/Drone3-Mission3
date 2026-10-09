# 이동 표면 Flow/ToF 의미 벤치

2026-10-04 실행 결과: **22/22 assertion case 통과**, JSON·CSV·PNG·SVG 생성 성공. 지정된 SIL 환경의 MuJoCo **3.13.0**, NumPy **2.2.6**, OpenCV **4.14.0**, Python **3.10.12**를 사용했다. 이 결과의 범위는 **비접촉 geometry-only 가상센서 광학점과 이동 표면의 상대운동**이다. 실제 Flow deck 출력, AI-deck 영상 기반 폐루프, 기체 비행, 물리적 접촉·착륙 성공을 검증한 결과가 아니다.

## 실행과 쓰기 범위

프로젝트 루트에서 다음 명령으로 재현한다.

```bash
cd .
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  src/crazyflie-ros2-sil/.venv/bin/python scripts/surface_sensor_bench.py
```

계산은 `src/crazyflie-ros2-sil/.venv/bin/python`과 `PYTHONNOUSERSITE=1`을 요구한다. assertion 또는 그림 생성 실패 시 종료코드는 1이다. 결과의 `checks`에는 사례별 실제값·기대값·통과 여부·오류가 들어간다. 기본 절대 허용오차는 `1e-9`, 상대 허용오차는 0이다.

그림만 **기존** `build/yaw-analysis-venv/bin/python`의 Matplotlib **3.10.8**으로 만든다. SIL 환경에서 보이는 시스템 Matplotlib 3.5.1은 NumPy 2.2.6과 ABI가 맞지 않아 import에 실패했다. 패키지나 환경을 설치·변경하지 않았다. 그래프 자식 프로세스는 저장된 JSON을 읽고 출력·캐시를 `artifacts/surface-sensor-bench/` 안에만 쓴다. `--plot-python`으로 이미 준비된 다른 분석 Python 경로를 지정할 수 있다.

작성 범위는 다음 세 경로다.

- `scripts/surface_sensor_bench.py`: 실행 가능한 독립 sidecar.
- `docs/surface-sensor-bench.md`: 의미·시험조건·upstream 근거·결과 설명.
- `artifacts/surface-sensor-bench/`: 결과·장면 복사본·그림·로그·그림 캐시.

벤치는 `--output`이 결과 디렉터리 또는 그 하위인지 확인한다. 프로젝트 모듈 import 시 bytecode 쓰기도 막는다. `moving_landing_scene.py`, `yaw_scene.py`, `provided_hardware.py`, 설정 두 파일, upstream `scene.xml`·`crazysim.py`의 실행 전후 SHA-256을 비교해 동일함을 확인했다. 인터넷에서는 MuJoCo 공식 API 문서만 읽었다. upstream 수정·업로드·PR·시스템 설치는 수행하지 않았다.

## 실제 장비 정보와 가상 시험조건의 구분

사용자 고정 조건은 제조사 제공 **Crazyflie 2.1 + AI-deck + Flow deck 기본 장착**, 메인 카메라 **mono8 320 × 320**이다. 벤치는 기존 카메라 설정을 확인하고 다른 크기·encoding을 거부한다. 이 벤치는 영상을 생성하거나 카메라 intrinsic/extrinsic을 추정하지 않는다.

실제 센서의 수치 장착점·방향·PMW pixel 보정은 알려져 있지 않으므로 새로 정하지 않았다. `provided-hardware.json`의 카메라 장착 변환·위치·조립 관성 미확정 값도 그대로 유지된다. 가상센서의 origin과 방향은 각 사례에 저장되는 **시험 입력**이며 실제 Flow deck 위치나 제조사 장착값이 아니다. `results.json`의 실제 센서 원점·장착 변환·PMW pixel 보정 필드는 `null`이다. 메인 카메라 광학점과 이 가상센서 광학점을 일치시키는 물리 변환도 가정하지 않는다.

가상 기준축 `refFrame = I`는 world 축과 일치하는 시험 기준이다. 자세 비교에서는 `R_world_virtual_sensor = Rz(yaw)`만 적용한다. yaw 0/90/180° 사례는 각각 **고정 자세**이며 센서 각속도는 0이다. 실제 장착 yaw, 동적 yaw에 따른 image rotation, gyro compensation을 검증하지 않는다.

기본 가상 광학점의 world 높이는 0.50 m이며 ray는 world −Z 단위벡터다. 이 값은 실험 설정이다. 같은 origin에서 floor까지 거리는 0.50 m, pad top까지 거리는 0.30 m다. 추가 0.25 m 높이 사례는 pad 위 거리 0.05 m에서 인위적인 최저 높이 clamp가 없는지 확인하기 위한 수학적 시험이며 실제 센서의 유효 최소거리 주장도 아니다.

## 표면 선택과 상대속도 계산

기존 `scripts/moving_landing_scene.py::build_scene`을 그대로 호출해 다음 환경을 만든다.

- `landing_vehicle`의 `vehicle_slide`: world +X 방향의 실제 MuJoCo slide joint.
- `landing_pad`: 0.30 × 0.30 m, top world z = 0.20 m, group 0.
- `floor`: world z = 0 m의 정적 plane, group 0.
- 깃발·marker post·차체 artwork·바퀴 등 시각 소품: group 2.
- aircraft body 없음. `world`와 `landing_vehicle`만 존재한다.

기존 장면에는 시험 차량 소품의 질량·관성이 있다. 벤치는 이를 기체 수치로 사용하지 않고 그대로 복사한다. 별도의 운동학 복사본에서 모든 geom의 `contype`·`conaffinity`를 0으로 만든다. `mj_step`·동역학 적분·접촉 계산을 호출하지 않고 `qpos = speed × time`, `qvel = speed`를 지정한다. `mj_kinematics`와 `mj_comPos`로 위치 및 Jacobian용 상태만 갱신한다.

ray group mask는 `[1, 1, 0, 1, 1, 1]`, `flg_static=1`, `bodyexclude=-1`이다. ray 결과의 `geom_id`를 `model.geom_bodyid[geom_id]`로 연결한다. 교차점을 `hit = origin + distance × unit_direction`으로 구한 뒤, **해당 교차점이 hit body에 붙어 있다고 보고** `mj_jac(model, data, jacp, jacr, hit, body_id)`를 계산한다.

```text
v_surface_at_hit_world = jacp @ qvel
omega_surface_world   = jacr @ qvel
v_relative_sensor     = R_world_virtual_sensor.T
                        @ (v_sensor_world - v_surface_at_hit_world)
angular_flow_xy        = v_relative_sensor[:2] / height
```

이 방법은 floor이면 0, 이동 pad이면 +0.3 m/s를 자동으로 선택한다. 회전 가능한 body라면 `v_translation + omega × lever_arm`도 교차점 속도에 포함된다. body COM 속도를 교차점 속도로 대체하지 않는다. [MuJoCo 공식 `mj_jac` 설명](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac)은 body에 붙은 world point의 translational/rotational Jacobian을 정의하며, [공식 `mj_ray` 설명](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-ray)은 교차 geom id·group 필터·정적 geom 포함 조건을 설명한다.

출력 단위는 **이상적 ToF ray distance(m)**와 **signed angular flow(rad/s)**다. 이 문서의 flow 부호는 `sensor − surface` 상대속도 부호로 정의한다. PMW raw pixel 축·부호 또는 실제 이미지 좌표계와 일치한다는 뜻이 아니다. pixel count·FOV-per-pixel·PMW 보정 상수는 채택하지 않는다.

flow 식은 수평 표면, 수직 ray, 센서 회전 0에서 광학축 중심의 lateral 상대속도/높이를 나타낸다. 수직 하강만 있으면 중심의 lateral flow는 0이지만 영상 주변의 expansion까지 0이라는 뜻은 아니다. 비스듬한 ray 사례는 **ToF 거리만** 검증한다. ray가 맞지 않으면 `valid=false`, 거리 `null`을 반환하며 하드웨어 최대거리로 치환하지 않는다. 실제 ToF cone·mixed return·반사율과 PMW texture correlation은 모델링하지 않는다.

## 통과한 assertion과 수치

주요 단일 상태 사례는 t = 0.75 s, cart slide = 0.225 m에서 실행한다. angular flow의 표기는 `(x, y)`다.

| 사례 | hit / 거리(m) | 기대 및 실제 angular flow(rad/s) | 확인한 의미 |
|---|---|---|---|
| 센서·차 +0.3 m/s 함께 이동 | pad / 0.30 | (0, 0) | 상대 lateral 속도 0 |
| 센서 +0.3 m/s, 정적 ground | floor / 0.50 | (+0.6, 0) | 동일 센서 속도라도 표면에 따라 flow가 다름 |
| 정지 센서, pad +0.3 m/s | pad / 0.30 | (−1, 0) | 함께 이동 사례와 구분되는 반대 방향 flow |
| 센서 +0.6 m/s, pad +0.3 m/s | pad / 0.30 | (+1, 0) | 세계 속도 대신 상대속도 사용 |
| ground 위 +Y 0.3 m/s | floor / 0.50 | (0, +0.6) | 두 번째 lateral 축 |
| 고정 yaw 90°, +X 0.3 m/s | floor / 0.50 | (0, −0.6) | `R.T`에 의한 가상 기준축 회전 |
| 고정 yaw 180°, +X 0.3 m/s | floor / 0.50 | (−0.6, 0) | 기준축 부호 반전 |
| 정지 센서 z=0.25 m, 이동 pad | pad / 0.05 | (−6, 0) | 이상적 거리의 하한 clamp 없음 |
| pad와 +0.3 m/s 동행, −0.035 m/s 하강 | pad / 0.30 | (0, 0) | 중심 lateral translation flow와 수직 이동 구분 |
| 정지 센서, 정적 pad | pad / 0.30 | (0, 0) | static pad 기준 |
| 정지 센서, ground | floor / 0.50 | (0, 0) | static floor 기준 |
| 이동 후 이전 pad 위치 | floor / 0.50 | (0, 0) | 이동 geom의 실제 위치로 ray 판정 |

추가 assertion도 모두 통과했다.

- **장면 계약:** speed 0.3 m/s, pad top 0.20 m, slide 0.225 m, 모든 contact mask 0, aircraft body 없음.
- **floor → pad 전환:** x가 pad 뒤쪽 edge 직전/직후인 두 점에서 거리는 **0.50 → 0.30 m**, 감소량 **0.20 m**. 센서 world z는 그대로다.
- **회전 표면:** 별도 `scene-kinematic-yaw.xml`에만 가상 yaw hinge를 추가했다. yaw 30°, surface yaw rate +1 rad/s, body 원점에서 local `(0.10, 0.04, 0)` m인 교차점의 속도는 **(0.21535898385, 0.06660254038, 0) m/s**. 정지 센서의 flow는 **(−0.71786327950, −0.22200846793) rad/s**다. `omega × r` 해석값 및 ±1 µs material point 위치의 중앙 차분값과 Jacobian 속도가 일치한다. 이것은 회전 표면의 추가 수학적 시험이며 원래 직진 차량의 요구사항이나 실제 장비 수치가 아니다.
- **전경 깃발:** 가상 origin world `(1.028, 0.04, 0.55)` m에서 −Z ray를 쏜다. group 2 포함 시 `marker_board` / 거리 **0.10 m**, group 2 제외 시 `floor` / 거리 **0.55 m**. 비접촉 geom도 ray에는 잡힐 수 있으므로 contact mask와 ray group 필터를 구분한다.
- **미교차:** 위쪽 ray는 hit 없음·거리 null.
- **비스듬한 ToF:** 높이 0.50 m, 수직축에서 45° ray는 floor까지 **0.70710678119 m**. ray distance가 vertical height와 다르다.
- **upstream 반례 두 개:** 동행 pad와 정지 센서/이동 pad에서 upstream translation proxy와 올바른 상대 flow의 x 오차가 각각 **+1 rad/s**.
- **통과 궤적:** 정지 광학점 world `(0.75, 0, 0.50)` m, 차량 +0.3 m/s, t=0…2 s에서 **101개** 표본. hit가 floor → pad → floor로 전환되고 거리는 0.50/0.30/0.50 m, flow는 0/−1/0 rad/s다. nominal edge 시각은 약 0.8333/1.8333 s이며 CSV는 0.02 s 간격의 표본 결과다.
- **입력 불변성:** 위에 열거한 입력 소스의 실행 전후 SHA-256 동일.

의미·장면 assertion 21개와 입력 소스 불변성 1개를 합쳐 **22개**다. yaw 계산에 나타나는 약 `1e-17`의 잔차는 부동소수점 오차이며 위 허용오차 안에 있다.

## 기존 upstream `compute_flow` 문제의 행 근거

현재 기준 소스는 다음 파일이다. 벤치는 import하거나 실행하지 않고 AST·원문만 읽었다.

`src/crazyflie-ros2-sil/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py`

| 위치 | 현재 구현 | 이동 표면에 대한 영향 |
|---|---|---|
| 739행 | `_ray_geomgroup = np.ones(6, dtype=np.uint8)` | group 2 시각 소품도 기본 ray 후보에 포함됨 |
| 1020–1033행 `read_tof_down` | 기체 body 위치에서 body −Z ray, `geom_id`를 받지만 **거리만 반환** | flow 단계로 hit geom/body·표면 속도가 전달되지 않음 |
| 1041–1042행 | `vel_world = self.data.cvel[bid][3:6]`, `vel_body = R.T @ vel_world` | 기체 세계 속도만 사용하며 `v_surface_at_hit` 차감 없음 |
| 1048–1050행 | `height = self.read_tof_down()`, 0.1 m 하한 | 표면 거리는 반영하지만 표면 운동은 반영하지 않음 |
| 1055–1057행 | pixel 변환 scale와 `omega_factor` | 이 벤치에서는 해당 pixel/회전 상수를 채택하지 않음 |
| 1059–1060행 | `vel_body[0 or 1] / height * cos_tilt`에 sensor angular 항만 추가 | 이동 pad와 동행하면 잘못된 비영 translation 항, 정지 센서에서 이동 pad면 잘못된 0 translation 항 |
| 1065–1066행 | random noise 추가 | 본 비교는 noise를 제외한 결정론적 translation 항 비교 |

`results.json::upstream_evidence`에는 이 파일의 SHA-256, 함수별 정확한 시작/끝 행과 원문, ray mask 초기화 행이 저장된다. 행 번호는 이번 실행에서 확인한 소스 기준이다. 별도로 보존된 `src/CrazySim/crazyflie-firmware/.../crazysim.py`도 확인 시 같은 문제 행을 가지고 있었지만, 생성 결과의 비교 기준은 현재 팀 SIL checkout이다.

본 벤치의 `upstream_translation_proxy_rad_s`는 rotation0·수평 시험에 한해 upstream의 **`v_sensor_xy / max(height, 0.1)` 항만** rad/s로 표시한다. 전체 `compute_flow`의 packet·pixel 값·noise·회전 동작을 실행 비교했다는 뜻이 아니다.

| 사례 | 올바른 surface-relative flow x(rad/s) | upstream translation proxy x(rad/s) |
|---|---:|---:|
| sensor/cart +0.3, pad까지 0.30 m | 0 | +1 |
| sensor +0.3, ground까지 0.50 m | +0.6 | +0.6 |
| sensor stationary, cart +0.3, pad까지 0.30 m | −1 | 0 |

ToF 높이를 pad 기준으로 사용하더라도 flow가 ground-relative velocity를 의미하는 것은 아니다. 실제 pad texture를 보는 flow는 pad-relative이므로 착륙 제어와 추정기의 world/vehicle/surface 속도 해석을 일관되게 연결해야 한다. ray mask에서 visual group 2를 제외하는 것은 이번 벤치의 명시적 표면 선택 정책이지 실제 광학센서가 깃발을 물리적으로 보지 않는다는 주장이 아니다.

## 산출물과 한계

| 경로 (`artifacts/surface-sensor-bench/` 기준) | 내용 |
|---|---|
| `results.json` | 범위·환경·생성 script hash·입력 hash·22개 assertion·단일 상태 표본·101개 궤적·ray 상세·upstream 행 근거 |
| `cases.csv` | 15개 단일 상태 표본, geom/body id, 거리, 센서/표면 속도, 각도 flow, upstream proxy |
| `cart-pass-trace.csv` | 101개 시간 표본과 hit 표면 전환 |
| `surface-sensor-bench.png` / `.svg` | 핵심 세 사례의 상대 flow 비교와 이동 pad 통과 시 거리·flow 변화 |
| `scene-original/scene.xml`, `scene-original/aruco.png` | 기존 builder가 만든 원장면·marker 이미지; 카메라 스트림 아님 |
| `scene-kinematic.xml` | contact mask를 0으로 한 기본 벤치 복사본 |
| `scene-kinematic-yaw.xml` | 회전 표면 시험용 가상 hinge가 추가된 복사본 |
| `run.log`, `plot.log`, `plot-cache/` | 실행 요약, 그림 생성 환경 로그·캐시 |

**실제 sensor noise, latency, 최대거리·최대속도·최대성능은 검증되지 않았다.** 실제 ToF/flow footprint·반사·광량·표면 texture·motion blur·장착 오차·센서 timestamp와 packet scheduling도 검증하지 않았다. 센서와 기체 간 lever arm, 제조사 미공개 calibration·조립 관성·추력·동역학을 만들어 넣지 않았다. 근거리 식이 계산된다는 사실을 하드웨어 운용 가능성으로 해석하면 안 된다.

이 sidecar 결과는 이동 표면을 보는 Flow/ToF의 의미와 기존 구현의 누락을 확인하는 근거다. 현재 메인 카메라 optical-point 추종·하강 코드에 대한 end-to-end 검증이나 대회·장비 승인 근거로 확장하지 않는다.
