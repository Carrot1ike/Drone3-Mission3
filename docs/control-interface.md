# 이동 차량 ArUco 착륙: 제어 인터페이스·좌표계 검토

검토일: 2026-09-30. 작업 루트는 `.`이며, 루트 AGENTS 지침을 적용했다. 이 문서는 통합 제어 그림에 사용할 인터페이스 계약과 제약을 정리한다. **world velocity API는 일관된 world 추정이 확보된 경우 후보**이다. API가 존재한다는 사실만으로 Flow deck을 이용한 이동 차량 착륙이 성립하지는 않는다.

조건은 Crazyflie 2.1 + AI-deck + Flow deck, 기체 개조 없이 기본 카메라 장착 유지, PC에서 영상 인식과 상위 착륙 제어 수행이다. 차량은 0.3 m/s 등속 직진하며 뒤쪽에 30 × 30 cm 착륙장이 있다. 차량 앞쪽의 수직 ArUco 마커는 뒤쪽 착륙장을 향한다. 마커 위치와 착륙 중심은 서로 다른 점이다. 마커 크기·장착 높이·카메라 외부 파라미터·게인·명령 및 영상 주기는 이번 검토에서 수치로 확정하지 않는다.

## 1. 확인 범위와 버전

실제 프로젝트 체크아웃인 `./src/crazyflie-lib-python`과 `./src/CrazySim`의 소스를 읽었다. 다음 버전은 로컬 Git HEAD와 기존 버전 기록을 대조했다.

| 대상 | 확인한 버전 | 의미 |
| --- | --- | --- |
| CFLib 소스 | `3c1cb0d024fef6f45d08f0ad236a22ac395dd924` | `commander.py`의 패킷 생성 구현 |
| 프로젝트에 기록된 CFLib 패키지 | `0.1.34rc1` | 설치 기록이며 실기 연결 성공을 뜻하지 않음 |
| CrazySim | `3ec8b55da4bff887da542a9f314da825460e65be` | 시뮬레이터 상위 체크아웃 |
| CrazySim firmware | `aa6571dc465f06f7d1f9aaf7b0b861fbcd1b3d67` | decoder·추정기·제어기·supervisor 구현 |
| crazyflie-simulation | `89d8cf79fb722bf4bd7e363ade7d90d6c45d6d5d` | 로컬 HEAD와 기존 기록이 일치하는 시뮬레이션 소스 |

근거: [설치 버전 기록](installed-versions.json), [CFLib commander](../src/crazyflie-lib-python/cflib/crazyflie/commander.py:120), [CrazySim decoder](../src/CrazySim/crazyflie-firmware/src/modules/src/crtp_commander_generic.c:65). 실제 Crazyflie에 설치된 펌웨어 버전·활성 추정기·활성 제어기는 아직 확인하지 않았다. 공식 `master` 문서와 로컬 fork의 구현 차이는 아래에서 구분한다.

## 2. 실제 명령 API와 내부 루프

| CFLib 호출 | 입력 좌표·단위 | 읽은 decoder의 동작 | 착륙 설계의 조건 |
| --- | --- | --- | --- |
| `send_velocity_world_setpoint(vx, vy, vz, yawrate)` | 펌웨어 world 기준 속도 3축, m/s. `yawrate`는 deg/s | x/y/z를 `modeVelocity`, yaw를 각속도 모드로 설정. `velocity_body`는 초기화된 false 유지 | 기체 내부의 속도 추정과 PC 명령이 동일하고 일관된 world 의미를 가져야 함 |
| `send_hover_setpoint(vx, vy, yawrate, zdistance)` | 기체 기준 수평 속도, m/s. `yawrate`는 deg/s. `zdistance`는 m | x/y 속도 모드, `velocity_body=true`; z는 `modeAbs`, `setpoint.position.z=zdistance` | body 기준은 차량 상대 기준이 아님. z 입력은 내부 추정 z와 비교되는 높이 목표 |
| `send_stop_setpoint()` | STOP | null setpoint로 모터 정지 | 공중에서 사용하면 추락할 수 있음. 정상적인 착륙 궤적 명령이 아님 |
| `send_notify_setpoint_stop(...)` | setpoint 우선순위 반환 메타 명령 | 로컬 firmware에서는 `commanderRelaxPriority()` 호출 | 모터 정지나 안전 착륙을 수행하지 않음. high-level commander 인계에 사용 |

공식 [Commander API](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/api/cflib/crazyflie/commander/)는 world 속도와 body 속도를 구분한다. [로컬 velocity decoder](../src/CrazySim/crazyflie-firmware/src/modules/src/crtp_commander_generic.c:98) 및 [hover decoder](../src/CrazySim/crazyflie-firmware/src/modules/src/crtp_commander_generic.c:308)에서도 이 모드 차이를 확인했다. velocity 패킷 구조체의 `vx` 주석에 있는 `m` 표기는 속도 모드와 CFLib의 m/s 설명을 대체하지 않는다.

hover의 `zdistance`는 문서상 아래 표면까지의 거리를 의도한 입력이지만, decoder가 매 명령마다 ToF 원시 거리와 직접 비교하는 구조는 아니다. 실제 비교 대상은 `state.position.z`이다. 정지된 평탄한 바닥을 높이 기준으로 삼을 때의 운용 의도를 이동 착륙장에 대한 높이 보장으로 확대하지 않는다. 차량 높이와 바닥/차량 경계에서 ToF가 보는 표면이 바뀌면 추정 높이와 물리적 착륙장 이격이 달라질 수 있다. 근거: [높이 PID 경로](../src/CrazySim/crazyflie-firmware/src/modules/src/controller/position_controller_pid.c:229), [ToF 모델](../src/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/mm_tof.c:28).

PID 제어기를 사용할 때 기체 내부 경로는 다음과 같다.

```text
CRTP decoder → commander / supervisor → 속도 제어 → 자세 제어
                                         → 각속도 제어 → 모터 출력 분배
IMU + Flow + ToF → 기체 내부 추정기 → state → 위 내부 제어 루프에 피드백
```

world 속도 명령은 기체 내부에서 yaw를 이용해 수평 속도 제어 좌표로 변환한다. hover의 body 수평 속도는 읽은 PID 구현에서 yaw에 정렬된 수평축 기준으로 처리된다. 큰 roll/pitch에서 3차원 body 벡터 전체를 단순 회전해 두 성분만 보내는 식으로 의미를 확장하지 않는다. world velocity 모드에서는 기체 내부 x/y/z 위치 PID를 이용한 목표점 추종 대신 PC가 위치 오차를 속도 보정으로 만드는 외부 루프를 담당한다. hover에는 내부 z 위치 PID가 남는다. 근거: [수평 좌표 변환과 속도 PID](../src/CrazySim/crazyflie-firmware/src/modules/src/controller/position_controller_pid.c:193), [자세·각속도 PID 호출](../src/CrazySim/crazyflie-firmware/src/modules/src/controller/controller_pid.c:93), 공식 [Controllers](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/).

따라서 통합 그림에서 PC 출력과 모터 사이에 기체 내부 제어기를 표시해야 한다. `stabilizer.estimator`, `stabilizer.controller` 및 시작 로그로 실제 선택을 확인해야 하며, 모든 펌웨어가 이 PID 경로를 사용한다고 단정하지 않는다. 이번 검토는 내부 및 PC 루프의 운용 Hz나 게인을 결정하지 않았다.

### 패킷 버전과 부호

읽은 CrazySim firmware는 CRTP protocol version 7을 알리고 generic setpoint type 1(world velocity), 5(hover)를 지원한다. 현재 CFLib는 protocol version이 8 이하이면 이 legacy type을 사용하고 송신 전에 yawrate 부호를 변환한다. 로컬 decoder의 부호 변환과 조합되어 API 입력의 yawrate가 내부 setpoint에 반영된다. 상위 제어기가 추가로 yaw 부호를 뒤집지 않는다. 새 protocol 경로의 type 8/10을 이 로컬 decoder에 수동 송신해서도 안 된다. 근거: [protocol 상수](../src/CrazySim/crazyflie-firmware/src/modules/interface/crtp.h:33), [CFLib의 버전 분기](../src/crazyflie-lib-python/cflib/crazyflie/commander.py:130), [decoder 초기화와 type 선택](../src/CrazySim/crazyflie-firmware/src/modules/src/crtp_commander_generic.c:414).

회전 방향은 아래의 좌표 정의와 실측으로 확인한다. yawrate는 각도 목표가 아니며, 수학적 계산에 rad/s를 쓰면 API 경계에서 deg/s로 변환한다. `send_notify_setpoint_stop`의 `remain_valid_milliseconds`도 로컬 decoder에서는 사용하지 않으므로 이 값을 착륙 유예 시간으로 해석하지 않는다. 근거: [메타 명령 decoder](../src/CrazySim/crazyflie-firmware/src/modules/src/crtp_commander.c:97).

## 3. 좌표 계약: 카메라·마커·차량·착륙점 분리

기호 `T^A_B`는 B 좌표의 점을 A 좌표로 옮기는 강체 변환이며, `R^A_B`는 그 회전 부분이다. 길이는 m, 속도는 m/s로 통일한다. API 경계의 각도는 deg/deg/s이다.

| 프레임 | 정의·확인 사항 |
| --- | --- |
| W: 기체 추정 world | 고정된 오른손 좌표, z-up. 초기화 또는 외부 위치 시스템이 원점·방향을 정함. 차량 진행 방향과 world +x가 같다는 보장은 없음 |
| B: Crazyflie body | x 전방, y 왼쪽, z 위. 원점은 제어·추정에서 사용하는 기체 기준점 |
| C: 카메라 optical | OpenCV 규약을 쓸 경우 x 오른쪽, y 아래, z 광축 전방. 영상 회전·반전과 실제 기본 장착을 포함해 B와의 관계를 보정 |
| M: 수직 마커 | 코너 순서와 마커 실제 크기로 정의한 PnP 물체 좌표. 마커 면 법선과 차량 진행 방향을 동일시하지 않음 |
| V: 차량 | 설계상 x 진행 방향, y 왼쪽, z 위. 마커와 차량 사이의 고정 변환을 별도로 측정 |
| P: 착륙장 | 뒤쪽 착륙장 중심 및 평면. `T^V_P`에 마커와 다른 위치·높이를 반영 |

공식 [Crazyflie 좌표계](https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system/)와 [OpenCV PnP](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)를 대조했다. OpenCV의 `rvec/tvec`로 만든 변환은 **M → C**, 즉 `T^C_M`이다. 카메라의 world 위치나 차량의 world 위치 자체가 아니다. 기체 추정 quaternion과 보고된 Euler pitch의 부호 규약도 다르므로, 임의의 Euler 변환식을 적용하지 않는다. 읽은 [Kalman 상태 외부화](../src/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/kalman_core.c:769)는 보고용 pitch만 반전한다.

z-up 오른손 수평 좌표에서 +yaw는 +x에서 +y로 향하는 회전이며, 위에서 내려다보면 반시계 방향이다. 영상 기반 yaw 오차의 축·부호를 이 정의와 API 입력에 맞춘다. PnP에는 보정한 카메라 내부 행렬·왜곡 계수·마커 실제 크기가 필요하며, 영상 회전/크기 변경에도 보정을 일관되게 적용한다. 추정기 리셋·좌표 재초기화·yaw 드리프트를 감지하면 기존 world 변환과 목표의 유효성을 다시 판정한다.

world 추정이 유효할 때의 변환 사슬은 다음과 같다.

```text
T^W_P = T^W_B · T^B_C · T^C_M · T^M_V · T^V_P
          로그    보정     영상      측정      측정
```

`T^B_C`는 기본 장착 상태에서의 외부 파라미터이며 카메라 이동·렌즈 교체를 전제하지 않는다. `T^M_V · T^V_P`를 직접 측정한 `T^M_P`로 합칠 수도 있지만, 마커 중심을 착륙 목표로 대체해서는 안 된다. 착륙장과 마커의 높이·회전 차이까지 포함해야 한다.

동일 시각의 착륙 목표와 기체 위치를 같은 프레임에서 빼야 한다.

```text
p_D^W = p_P^W + h_ref · z_up^W    # 수평 착륙장 위 접근 목표
e^W   = p_D^W - p_B^W
```

`h_ref`는 접근/하강 단계에서 정하는 목표 이격이며 API에 자동 존재하는 값이 아니다. 착륙장 평면이 기울어지면 법선·접촉 기준을 따로 정의한다. 카메라에서 구한 `tvec`, body 위치, world 위치를 섞어서 빼지 않는다. 기체 기준점과 실제 착륙 접촉점 사이의 차이도 완료 판정에 반영한다.

world 없이 상대 기하를 계산하는 경로도 가능하다. `T^B_V = T^B_C · T^C_M · T^M_V`, `T^V_B = inverse(T^B_V)`를 이용하면 V에서 기체와 착륙 목표의 상대 오차를 만들 수 있다. 이는 **상대 관측의 계산법**이며 stock CFLib에 차량 상대속도 제어 API가 생긴다는 뜻은 아니다.

기본 카메라가 수직 마커를 볼 수 있는 접근 방향과 하강 범위는 기하·실영상으로 확인해야 한다. 차량 앞의 마커 면이 뒤를 향한다는 배치만으로 최종 접촉까지 검출된다고 보장할 수 없다. 카메라 화각, 기체 자세, 마커 높이, 차체 가림 및 영상 내 마커 크기를 확인하며, 하향 카메라를 통합 그림에 가정하지 않는다.

## 4. 피드백 + 차량 속도 FF의 성립 조건

일관된 W에서 원리상의 외부 제어는 다음과 같다.

```text
e^W           = 착륙 목표 위치^W - 기체 위치^W
v_correction^W = f(e^W, 상대속도^W, 접근/하강 단계)
v_cmd^W        = v_vehicle_hat^W + v_correction^W
```

피드백 보정은 오차를 줄이고, FF는 움직이는 목표와 같은 속도로 이동할 기준을 제공한다. 등속 직진·고정된 마커/착륙장 변환일 때 착륙 중심의 수평 속도는 차량의 병진 속도와 같다. 목표 이격의 감소는 수직 보정/궤적 항으로 별도 표현한다. 회전 차량까지 확장하면 착륙 중심의 회전 오프셋 속도 항이 추가되므로 현재 직진 가정을 명시한다.

`v_vehicle_nominal = 0.3 × 진행방향 단위벡터`는 **실험 설정값**이다. `v_vehicle_hat`는 실제 시간에 맞춘 **추정값**으로 출처·좌표·신선도·불확실성을 가져야 한다. 설정값을 사용한 실험은 nominal FF로 표시하고, 영상에서 실제 속도를 추정했다고 표현하지 않는다. 차량 +x를 world +x로 그대로 대입하지도 않는다.

움직이는 기체에서 관측한 마커의 위치 변화에는 기체 이동과 회전이 포함된다. 상대 위치의 단순 차분만으로 차량 world 속도를 얻을 수 없다. 기체의 유효한 world 속도와 자세, 같은 시각의 상대 기하를 결합해야 한다. 예를 들어 같은 W에서 수평 오차를 시간 미분할 수 있을 때 `v_vehicle_hat,xy = v_B_hat,xy + d(e_xy)/dt`로 계산할 수 있지만, 기체 world 추정이 오염되면 이 결과도 오염된다. 영상과 로그의 시간 차이·회전 변환 변화·차분 잡음을 처리해야 한다. 움직이는 마커에 대한 상대 관측만으로 공통 병진 속도의 절대 기준이 확보되는 것은 아니다.

## 5. 핵심 제약: Flow의 표면 상대 관측

Flow deck의 광류는 관측 표면과 기체의 상대 운동에 반응한다. 회전·높이 보정을 생략한 수평 관계를 쓰면 다음과 같다.

```text
Flow에서 유도되는 속도 ≈ v_B^W - v_surface^W
정지 바닥: v_surface^W = 0
차량 표면: v_surface^W = v_vehicle^W
```

이 관계는 물리적 해석이며 원시 `motion.deltaX/deltaY` 자체가 m/s라는 뜻은 아니다. 공식 [Flow deck 설명](https://www.bitcraze.io/products/flow-deck-v2/)의 광류·거리 센서 구성을 확인했고, 로컬 [Flow 드라이버](../src/CrazySim/crazyflie-firmware/src/deck/drivers/src/flowdeck_v1v2.c:117), [Kalman Flow 모델](../src/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/mm_flow.c:37), [UKF Flow 모델](../src/CrazySim/crazyflie-firmware/src/modules/src/estimator/estimator_ukf.c:1183)을 읽었다. 이 모델들은 기체 상태·높이·각속도를 사용하며 차량 속도나 관측 표면 ID를 입력하지 않는다. UKF의 ToF 이상치에 따른 `flowActive` 판정도 이동 표면을 식별하는 전환 로직이 아니다.

Kalman은 내부 body 속도를 회전해 world `state.velocity`로 외부화한다. **축 회전은 속도의 기준 표면을 정지 world로 바꾸지 않는다.** Flow가 차량에 대한 상대 운동을 보는데 정지 표면 모델로 융합하면, `stateEstimate.vx/vy`라는 world 이름이 붙어도 실제 world 속도의 정확성이 확보되지 않는다. 추정값 전체가 곧바로 순수 상대속도가 된다고도 단정하지 않는다. IMU 융합·필터 상태·표면 경계에 따른 편향과 과도응답을 검증해야 한다. 근거: [속도 외부화](../src/CrazySim/crazyflie-firmware/src/modules/src/kalman_core/kalman_core.c:753).

단순화한 위험 예: 기체와 차량이 모두 0.3 m/s로 같이 이동하면 차량 위 Flow의 병진 상대 운동은 거의 0이다. 내부 속도 추정이 이 상대값에 가까워졌는데 PC가 world 명령으로 0.3 m/s FF를 추가하면 내부 속도 루프는 이미 잘 추종 중인 기체를 더 가속하려 할 수 있다. 이상화하면 `v_B - v_vehicle ≈ v_cmd = v_vehicle + v_correction`이 되어 차량 속도 항이 두 번 들어간다. 이는 정확한 실제 정상상태 예측이 아니라 **FF 중복·의미 불일치의 설명**이다. 바닥에서 차량 표면으로 넘어가는 동안에도 같은 문제가 연속적으로 변할 수 있다.

ToF도 착륙장 높이를 인식하는 센서가 아니다. 로컬 Kalman ToF 모델은 추정 z와 경사 보정된 거리를 비교하며, 차량 표면 높이를 독립 상태로 두지 않는다. 바닥/차량 높이 경계의 거리 변화가 곧바로 올바른 world 높이 변환이나 안전한 hover 높이 전환을 뜻하지 않는다.

따라서 채택 여부는 다음 조건으로 나눈다.

| 상태 | 판정 |
| --- | --- |
| PC와 기체 내부에 일관된 W가 있고 이동 표면 구간에서도 내부 속도·높이 추정의 의미가 검증됨 | world velocity + FF 후보. PC 오차와 기체 내부 속도 루프가 같은 기준을 사용해야 함 |
| 기체 내부 world 추정은 불명확하지만 마커 상대 위치는 관측됨 | 상대 외부 제어를 연구할 수 있으나 실제 지원 API로의 구현·추정기 연계는 미해결 |
| hover를 쓰기만 하면 차량 상대 제어로 전환된다고 가정 | 근거 없음. body 축 선택과 이동 표면 상대속도는 별개 |
| 제조사 펌웨어가 바닥/차량을 자동 식별·전환한다고 가정 | 읽은 코드에서 지원 근거 없음 |

필요하다면 외부의 유효한 world 관측과 기체 추정기 연계, Flow 융합의 처리 방법을 별도 설계해야 한다. PC에서만 상대 오차를 정확히 계산하는 것으로 내부 추정기가 수정되지는 않는다. 로컬 `motion.disable`은 Flow 측정 융합을 막는 파라미터이며 world 관측을 새로 제공하거나 ToF 표면 문제를 해결하지 않는다. 이를 자동 표면 전환 기능으로 묘사하지 않는다. 이번 작업은 파라미터 변경·외부 위치 주입·추정기 변경을 실행하거나 채택하지 않았다. 사용자는 소프트웨어 변경이 허용되는 것으로 이해한다고 확인했다. 하드웨어 개조 금지와 제조사 장착 조건을 유지하며 필요한 소프트웨어 처리를 설계하고, 실제 장비 지급 규정과 대조한다.

## 6. 영상·명령·추정·로그 경로

통합 그림에 전달할 연결 계약은 아래와 같다.

| 경로 | 화살표 라벨 | 전달하는 것 |
| --- | --- | --- |
| AI-deck 카메라 → GAP8 → ESP32 → PC | 영상 스트림 / Wi-Fi, 예제의 CPX·TCP 경로 | RAW 또는 JPEG 카메라 프레임. PC가 ArUco 인식 |
| PC 상위 제어 → CFLib → Crazyradio → 기체 CRTP commander | setpoint / CRTP over radio | 작은 제어 명령. world/body 및 단위를 API에 맞춤 |
| IMU + Flow + ToF → 기체 내부 추정기 → 내부 제어기 | onboard 상태 추정·피드백 | 기체 상태 및 관측. PC 영상 인식 경로와 구분 |
| 기체 로그 → CRTP / Crazyradio → CFLib → PC | 로그 피드백 | 상태 추정·센서 진단·supervisor 상태·링크 상태 |
| PC CFLib → CrazySim의 UDP endpoint/bridge → SITL firmware | CRTP over UDP | 시뮬레이션에서 radio transport를 대체하는 명령·로그 연결 |
| 시뮬레이션 물리·센서 → SITL firmware | 시뮬레이션 센서 입력 | 실기 센서 입력과 동등성 여부를 따로 검증 |

RAW 영상이 Crazyradio로 전송되는 화살표를 그리지 않는다. CPX가 일반적으로 여러 transport를 지원하더라도 이 과제의 영상 경로는 AI-deck Wi-Fi이다. 공식 [WiFi Video Streamer](https://www.bitcraze.io/documentation/repository/aideck-gap8-examples/master/examples/wifi-streamer/), [CPX](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/cpx/)는 영상과 MCU/host 연결을 설명한다. 예제 준비·deck 펌웨어 및 Wi-Fi 설정이 필요하며, 기본 카메라 장착 자체가 영상 스트리밍이 설정되었음을 뜻하지 않는다.

실기 URI는 `radio://<dongle>/<channel>/<rate>/<address>` 계열이며 채널·주소·rate는 실제 설정을 사용한다. CrazySim은 `udp://<host>:<port>` 계열의 CFLib 드라이버와 시뮬레이터 bridge를 사용한다. 로컬 UDP driver는 CRTP header/data를 UDP datagram에 넣는다. 이 연결을 실제 AI-deck 영상 Wi-Fi와 동일한 연결로 묘사하지 않는다. 근거: [radio driver](../src/crazyflie-lib-python/cflib/crtp/radiodriver.py:250), [UDP driver](../src/crazyflie-lib-python/cflib/crtp/udpdriver.py:180), [Gazebo bridge의 두 UDP 소켓](../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp:58).

### 로그 피드백 계약

연결 후 TOC에서 실제 존재·형식을 확인하고 log configuration을 시작해야 한다. 비동기 로그는 측정 시각과 PC 도착 시각을 구분한다. 영상·기체 로그·명령 시각을 정렬하고 프레임/로그 age, 누락, 순서 역전 및 firmware timestamp wrap을 처리한다. 영상에 신뢰할 촬영 시각이 없으면 PC 수신 시각만으로 촬영 지연까지 알고 있다고 가정하지 않는다. 지연 상한의 측정 또는 별도 동기화가 필요하다. 로그 패킷 크기와 지원 주기, radio의 명령/로그 공유 대역폭을 고려해 구성을 나눠야 한다. 주기는 여기서 고정하지 않는다. 공식 [Python API의 logging 설명](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/user-guides/python_api/).

| 후보 로그 / PC 기록 | 단위·사용 목적·주의 |
| --- | --- |
| `stateEstimate.x/y/z`, `vx/vy/vz` | m, m/s. 추정 world 상태이며 독립 참값이 아님 |
| `stateEstimate.qx/qy/qz/qw`, `yaw` | quaternion 및 deg. 카메라/world 변환과 yaw 제어에 사용 |
| `range.zrange` | 로컬 구현은 mm. m 변환 후 사용. 아래 표면 거리이며 world z와 구분 |
| `motion.deltaX/deltaY`, `squal`, `outlierCount` | 원시 Flow·품질 진단. m/s로 직접 취급하지 않음. TOC·추정기·deck 경로별 가용성 확인 |
| `kalman.varX/varY/varZ` 등 | Kalman 사용 시 진단. 작은 분산만으로 이동 표면 모델 오류가 없다고 판정하지 않음 |
| `ctrltarget.*`, `posCtl.targetVX/targetVY`, `bodyVX/bodyVY` | 적용 setpoint/내부 루프 진단. hover와 world 모드에 따라 좌표 의미를 해석 |
| `supervisor.info`, `pm.vbat`, 링크 callback/통계 | 비행·lock·전원·연결 상태. 실제 TOC 지원 확인 |
| PC: 마커 ID·PnP·재투영 오차·영상 age·추정 출처·FF·보정·최종 명령·모드 | 검출 성공, 추정 유효성, 포화 및 abort 원인을 구분 |

근거: [상태 로그](../src/CrazySim/crazyflie-firmware/src/modules/src/stabilizer.c:687), [거리 로그](../src/CrazySim/crazyflie-firmware/src/modules/src/range.c:88), [Flow 진단](../src/CrazySim/crazyflie-firmware/src/deck/drivers/src/flowdeck_v1v2.c:262), [내부 수평 속도 로그](../src/CrazySim/crazyflie-firmware/src/modules/src/controller/position_controller_pid.c:293). `kalman.statePX/PY/PZ`는 이 Kalman의 내부 body 속도이므로 `stateEstimate.vx/vy/vz`와 섞지 않는다.

## 7. 명령 중단·마커 손실·착륙 완료

**마커 손실과 radio 명령 중단은 다른 사건이다.** 계속 같은 명령을 재송신하면 firmware watchdog은 갱신될 수 있지만 영상 상태는 이미 오래되었을 수 있다. 반대로 PC가 송신을 멈추는 것으로 안전 착륙을 맡길 수는 없다.

읽은 local supervisor의 watchdog 상수는 경고 500 ms, timeout 2000 ms이다. 이는 해당 소스의 확인값이며 실기 펌웨어의 보장값이나 PC 송신 주기 권고가 아니다. 경고 상태에서는 수평 속도 모드를 끄고 roll/pitch와 yawrate를 0으로 만들어 자세를 수평화하며 **z 모드는 유지**한다. 따라서 이전 명령이 하강 속도였다면 경고만으로 하강 중지가 보장되지 않는다. timeout에 따른 상태 전환은 null setpoint·모터 정지/lock으로 이어질 수 있다. 공식 문서의 watchdog 설명은 위치가 `commander.c`로 표현되지만 이 fork의 실제 구현은 supervisor에 있다. 근거: [watchdog 상수](../src/CrazySim/crazyflie-firmware/src/modules/src/supervisor.c:59), [setpoint age 판정](../src/CrazySim/crazyflie-firmware/src/modules/src/supervisor.c:330), [override 동작](../src/CrazySim/crazyflie-firmware/src/modules/src/supervisor.c:417), 공식 [Commander Framework](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/commanders_setpoints/).

PC에는 영상 추론과 독립적으로 명령을 주기 갱신하는 송신 경로 및 아래 상태 처리를 둔다. 유효기간·품질·오차 임계값은 측정 후 정하며 새 수치를 지정하지 않는다.

| 사건/상태 | 제안하는 처리 | 제약 |
| --- | --- | --- |
| 정상 추종 | 최신 관측으로 FF + 보정을 계산하고 좌표/단위 변환·제한 후 송신 | world/내부 추정 일관성 확인이 선행 |
| 하강 허용 | 마커/기하·수평 오차·상대속도·높이 및 센서 신선도가 유효할 때만 하강 단계 진행 | 착륙장 크기만으로 허용 오차를 정하지 않음 |
| 마커 일시 손실·오래된 프레임 | 하강을 중지하고, 제한된 예측 구간에서만 추종 유지 여부 판단. 영상 age에 따라 유효성 하락 | 마지막 pose나 nominal 0.3을 무기한 사용하지 않음 |
| 지속 손실·변환 불명·Flow/ToF 표면 전환의 불확실성 | 착륙 시도를 중단하고 검증된 이탈/회복 절차 실행 | world 유효 시 높이 유지·회복 명령을 계속 송신할 수 있음. world 불명일 때 world 0속도나 hover를 보편적인 안전 fallback으로 지정할 수 없음 |
| radio 단절·PC 전체 중단 | 링크 callback/로그 age로 감지하고 가능한 회복을 시도하되 기체 watchdog 동작을 별도로 고려 | 끊어진 링크로 새 abort 명령 전달을 보장할 수 없음. firmware는 차량 추종/자동 착륙을 수행하지 않음 |
| 접촉·착륙 완료 | 기하·하강 상태·센서/상태 변화 등으로 접촉과 안정화를 확인한 뒤 모터 정지/인계 절차 실행 | ToF가 작거나 영상 마커가 사라진 사실 하나만으로 완료 판정하지 않음 |

추론이 멈춘 동안에도 통신 스레드가 오래된 비행 명령을 무기한 보낼 수 없도록 **정책/관측의 유효기간**을 별도로 검사한다. 최신 상태가 불명확한 조건에서 실행할 실기 회복 절차는 아직 검증되지 않았으므로, 이 문서를 완성된 failsafe 구현으로 해석하지 않는다.

low-level 명령은 high-level commander보다 우선하며 로컬 코드에서는 high-level planner를 중단한다. 이후 `land()`만 호출해 자동 인계된다고 가정하지 않는다. priority 반환과 새 계획을 포함한 명시적 인계가 필요하며, high-level land도 움직이는 차량을 자동 추종하지 않는다. 접촉 후 명령 스트림 종료와 supervisor가 landed로 판단하는 시점을 함께 확인한다. 근거: [우선순위 처리](../src/CrazySim/crazyflie-firmware/src/modules/src/commander.c:74), 위 공식 Commander Framework.

## 8. 피드백 + FF와 RL의 동일 입출력 비교 경계

주 비교의 교체 경계는 **PC 외부 제어 블록 전체: 피드포워드 + 피드백 vs RL 정책**이다. 기존 제어기는 블록 내부에서 차량속도 FF와 오차 보정을 결합하고, RL 정책은 동일 관측·목표로 같은 형식의 전체 명령을 출력한다. 좌표 변환·관측 추정·목표 생성/하강 허용·안전 상태기계·명령 제한·API adapter·송신·기체 내부 추정과 제어는 양쪽에 동일하게 둔다.

```text
동일 관측 o, 동일 목표 g
          → [PC 외부 제어 블록 전체]
             A: 차량속도 FF + 피드백 → 전체 명령 u
             B: RL 정책 π(o, g)      → 전체 명령 u
          → 동일 안전 제한/유효성 검사 → 동일 API adapter
          → 동일 transport → 동일 기체 내부 제어/추정
```

| 비교 항목 | 공통 계약 |
| --- | --- |
| 입력 `o` | 공통 프레임의 위치 오차·상대속도 추정·자세/방향 오차·차량속도 정보·신선도·검출/추정 품질·단계·이전 명령. 실제 제공할 항목을 사전에 동일하게 확정 |
| 목표 `g` | 같은 착륙 중심·접근 목표·목표 이격/하강 궤적·방향 목표. 목표 생성과 하강 허용 조건을 동일하게 적용 |
| 출력 `u` | 같은 프레임·단위의 전체 속도 명령 3축과 yawrate 명령. 주 비교에서 RL 출력에 차량속도 FF를 공통으로 추가하지 않음 |
| 착륙 단계 | 같은 하강 목표/허용 조건과 abort 처리. 제어 블록 교체만으로 예외 처리도 변경하지 않음 |
| 명령 경계 | 같은 API, 포화·변화율 제한·유효기간·지연 조건 및 동일 내부 controller/estimator |
| 정보 조건 | nominal 차량속도 정보를 사용하는 실험과 실제 추정 정보를 사용하는 실험을 분리하며 양쪽에 같은 정보를 제공. RL에만 차량 참값 속도·world pose·향후 궤적을 제공하지 않음 |
| 판정 | 실제 접촉 후 안정 착륙 성공, 착륙 중심 오차·수평 상대속도, 마커 손실·abort·명령 포화, 지연/누락 조건을 동일하게 기록 |

W 경계는 W 유효성을 먼저 만족해야 한다. 상대 V 경계를 채택하려면 두 방법 모두 같은 형식의 전체 상대 명령을 출력하고 같은 **추가 설계가 필요한 adapter/추정 구조**를 사용해야 한다. `π(o, g)`의 상대속도 출력을 그대로 hover/world API 입력으로 연결해 stock 지원처럼 표시하지 않는다. RL이 추정기의 이동 표면 가정을 자동 해결했다고도 주장하지 않는다. 이번 문서는 RL 실행·학습 또는 이 adapter 구현을 포함하지 않는다.

**별도의 잔차 RL 확장안**에서는 차량속도 FF를 공통으로 두고, 피드백 보정 `f(o, g)`만 RL 보정 `π_res(o, g)`으로 교체한다. 이 경우 출력은 전체 명령이 아닌 보정 `c`이며, `u = FF + c`를 공통으로 적용한다. 이 확장안은 위의 외부 제어 블록 전체 비교와 경계가 다르므로 실험명·통합 그림·결과를 분리해 표시한다.

## 9. CrazySim 검증의 실기 전이 한계

로컬 Gazebo plugin의 `OdomCallback`은 simulator pose를 외부 위치/자세 패킷으로 firmware에 보낸다. 이 경로로 안정적인 world 추정을 얻어 성공한 제어는 기본 Flow 기반 실기에 동일한 world 관측이 있다는 증거가 아니다. 근거: [Gazebo pose 입력](../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp:178).

읽은 MuJoCo 구현은 `--flowdeck`을 사용할 때 pose 송신을 억제한다. 그러나 `compute_flow()`는 기체의 world 속도를 body로 회전한 값을 사용하고 **아래 표면의 이동 속도를 빼지 않는다**. ToF raycast로 아래 물체까지의 거리를 구해도 차량 표면에 대한 상대 광류가 모델링되는 것은 아니다. 따라서 이 모델만으로 바닥 → 이동 차량 표면의 Flow 전환·FF 중복 위험을 검증할 수 없다. 근거: [MuJoCo Flow 생성](../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1035), [pose/Flow 송신 선택](../src/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/mujoco/crazysim.py:1082).

시뮬레이션용 `radio.rssi/isConnected`는 stub 값이므로 실제 Crazyradio 연결 품질 검증 자료로 사용하지 않는다. SITL stabilizer에는 실기 supervisor의 모터 허용 검사 일부를 우회하는 분기도 있어 실기 arming·안전 동등성을 따로 확인해야 한다. 근거: [radio stub](../src/CrazySim/crazyflie-firmware/src/hal/src/radiolink_sitl.c:30), [SITL 분기](../src/CrazySim/crazyflie-firmware/src/modules/src/stabilizer.c:315).

실제 제어기 비교 전에 기본 카메라의 마커 가시성·카메라/마커/착륙장 변환, 좌표와 yaw 부호, 추정기별 상태 의미, 이동 표면의 Flow/ToF 응답, 영상/로그/명령 지연, 마커 손실·링크 중단·접촉 판정을 단계적으로 확인해야 한다. 시뮬레이터 참값은 결과 평가용으로 분리하고 PC 정책에 제공하는 관측은 실기에서 얻을 수 있는 정보로 제한한다.

## 10. 이번 검토 결과와 통합 그림용 표기

소스와 공식 문서를 읽고, 프로젝트 `.venv`에서 지정한 `commander.py`를 로드해 링크 없이 패킷 인코딩을 확인했다. protocol 7/9에 대해 world/hover type 선택, CRTP generic port/channel, yawrate 부호 변환 검사가 통과했다. protocol 7 결과와 로컬 decoder 부호 조합도 대조했다. 파일을 추가하는 테스트, radio 연결, 비행, 제어 게인 조정, 실시간 주기 측정은 수행하지 않았다. 실기 이동 차량 착륙이나 특정 CrazySim backend의 구동 성공을 이번 결과로 주장하지 않는다.

통합 그림에는 다음 문구를 반영한다.

- `AI-deck 기본 카메라 → Wi-Fi 영상 → PC ArUco/PnP → M↔V↔P 변환`.
- `동일 PC 관측/목표 → PC 외부 제어 블록 전체: FF+피드백 vs RL 정책 → 동일 전체 명령 형식 → 동일 안전 제한/API adapter`.
- `공통 FF + 피드백 보정 vs RL 보정`은 별도의 잔차 RL 확장안으로 표시.
- `world velocity API: 일관된 world 추정이 확보된 경우 후보`.
- `hover: body 수평속도 + 내부 z 목표; 차량 상대속도 API 아님`.
- `CRTP/Crazyradio 명령·로그 ↔ 기체 내부 추정기·속도/자세/각속도 제어`.
- `IMU/Flow/ToF는 onboard 경로; 이동 표면 자동 전환은 미확인`.
- `CrazySim UDP/참값 pose/Flow 모델의 실기 차이`를 별도 주석으로 표시.

결론은 API와 경로가 확인되었다는 것이며, **기본 Flow/ToF 추정만으로 이동 차량 위에서도 일관된 world가 보장된다는 근거는 없다**. 상대제어 구현과 이동 표면 처리, 실기 회복 절차는 미해결 설계·검증 항목으로 남긴다.
