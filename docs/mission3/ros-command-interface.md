# 미션3 ROS2 센서·영상·명령 경계

2026-10-08 KST. `scripts/mission3_ros_gateway.py`는 센서로 변환된 패킷과 native 영상을 현재 `SelfMotion → Observer → Mission3`에 연결한다. 출력은 시뮬레이션 소비자를 위한 목표 속도다. 이 검증의 범위는 ROS2 전송과 추론 경계다. 공유 324 SITL, 기체 물리 폐루프, 제조사 하드웨어, 접촉·모터 종료를 검증한 결과가 아니다.

이 sidecar는 새 gateway/test/wrapper/이 문서와 `artifacts/mission3/ros-gateway/`만 소유한다. 메인이 소유한 controller/estimator/vision/physical runner/config/vendor 소스는 입력 계약을 읽고 호출한다. 과거 optical ROS node의 차량 설정 속도·발견 시 차량 시작 로직은 가져오지 않는다.

## 실행과 토픽

프로젝트 루트는 `.`다. wrapper가 `config/shared_ros_env.bash`를 source하고 팀 `src/crazyflie-ros2-sil/.venv/bin/python`을 사용한다. ROS 기록도 이 sidecar의 artifacts 아래에 둔다.

```bash
cd .
scripts/run_mission3_ros_gateway.sh --entry 1 --record-images
scripts/run_mission3_ros_gateway.sh --entry 1 --ros-args -p use_sim_time:=true
scripts/run_mission3_ros_gateway.sh --test --ros-roundtrip
```

`--config`는 미션 설정 파일, `--mount-profile`은 S 장착 프로파일을 선택한다. 기본값은 각각 `config/mission3.json`, `config/mission3-physical-sensitivity.json`이다. 기본 public 설정만 선택하며 physical 프로파일의 `controller_software`, motor, vehicle, sensor plant 설정을 자동 적용하지 않는다. 따라서 별도 조건부 physical adapter의 12초/0.40m/0.11m 설정이나 접촉 분기를 이 gateway에 암묵적으로 이식하지 않는다.

| 방향 | 기본 토픽 | ROS 타입 | 목표 주기/의미 |
|---|---|---|---|
| 입력 | `/drone3/mission3/sensor_packet` | `std_msgs/msg/String` | 100Hz, 아래의 엄격한 JSON 패킷 |
| 입력 | `/drone3/mission3/image` | `sensor_msgs/msg/Image` | 20Hz, native 320×320 mono8 |
| 출력 | `/drone3/mission3/cmd_velocity` | `geometry_msgs/msg/TwistStamped` | 50Hz, world 목표 속도 |
| 출력 | `/drone3/mission3/phase` | `std_msgs/msg/String` | 50Hz, 현재 phase |
| 출력 | `/drone3/mission3/status` | `std_msgs/msg/String` | 50Hz, 유효 JSON 상태/거부 원인 |
| 출력 | `/drone3/mission3/motors_off` | `std_msgs/msg/Bool` | 50Hz, 항상 false |

입력 QoS는 Humble `qos_profile_sensor_data`(best effort, volatile, depth 5), 출력은 reliable/volatile/depth 1이다. 목표 주기는 hard realtime 보장이 아니다. 출력의 ROS CLI remap은 publisher 생성 전에 차단한다. `/cf231` 명령 토픽과 `vehicle_start`를 생성하거나 발행하지 않는다. 일반 ROS 노드의 `/rosout`, `/parameter_events`는 존재한다. 시험은 `/drone3/mission3/test_<unique>/...`를 사용하고, sim-time 시험의 clock도 같은 전용 토픽으로 remap하여 공유 `/clock`에 발행하지 않는다.

`TwistStamped.header.frame_id`는 항상 `world_mission`이다. 정적 경기장 좌표의 +X는 50m 길이 방향, +Y는 20m 폭 방향, +Z는 위다. `linear.{x,y,z}` 단위는 m/s, `angular.z`는 rad/s이며 `angular.x/y=0`이다. yaw rate는 body-z gyro 명령이 아니라 이 world 좌표에서의 목표 heading rate다. 차량 좌표·설정 속도로 해석하면 안 된다. header와 상태의 시각은 ROS clock을 사용한다.

## 패킷 계약과 출처

모든 키가 필요하다. 아래 외의 최상위/attitude 키, 중복 JSON 키, NaN/Infinity, bool을 숫자로 사용한 값, 틀린 배열 길이, 정수/문자열로 대신한 bool을 거부한다.

```json
{
  "schema": "mission3.sensor.v1",
  "stamp_s": 10.0,
  "gyro_z_rad_s": 0.0,
  "accel_heading_m_s2": [0.0, 0.0, 0.0],
  "range_m": 1.0,
  "range_valid": true,
  "range_accurate": true,
  "range_new": true,
  "range_stamp_s": 10.0,
  "flow_angular_rad_s": [0.0, 0.0],
  "flow_valid": true,
  "attitude": {
    "stamp_s": 10.0,
    "roll_rad": 0.0,
    "pitch_rad": 0.0,
    "source": "sensor_imu"
  }
}
```

| 필드 | 의미와 upstream 책임 |
|---|---|
| `stamp_s` | 같은 ROS clock의 해당 IMU/Flow 변환 패킷 촬영·측정 시각. receipt 시각으로 오래된 값을 새로 표시하면 안 된다. |
| `gyro_z_rad_s` | 센서 gyro와 IMU roll/pitch로 산출한 heading yaw rate. 기울어진 body의 gyro.z 원시값과 구별한다. |
| `accel_heading_m_s2[3]` | 센서 specific force를 센서 유래 자세로 회전하고 중력을 제거한 heading 좌표계 가속도. heading의 수평 X/Y와 수직 Z를 사용한다. |
| `range_m` | 센서 ToF와 자세/Flow deck lever arm으로 변환한 body 원점의 표면 위 수직 높이. raw slant ToF, 절대 world-Z, 표면 참 ID가 아니다. 양수 또는 null. |
| `range_valid/accurate` | 실제 센서 유효성·계량 정확도 플래그. accurate=true이면 valid=true이며 값이 필요하다. valid=true/accurate=false는 허용하되 현재 추정기가 계량 융합하지 않는다. |
| `range_stamp_s/new` | ToF 자체 측정 시각과 새 측정 여부. 40Hz ToF를 100Hz 패킷으로 보낼 때 중간 패킷은 stamp/value/flags를 그대로 보존하고 new=false로 보낸다. |
| `flow_angular_rad_s[2]` | 프로젝트 기존 heading-frame Flow 각속도. upstream에서 gyro 회전 성분과 deck lever arm을 처리해야 한다. raw dpixel은 이 경계에서 해석하지 않는다. |
| `flow_valid` | 센서 및 upstream 품질 게이트의 최종 유효성. 메인의 선택적 texture/phase-correlation S 게이트도 여기에 반영할 수 있다. gateway는 false를 true로 바꾸지 않는다. |
| `attitude` | 센서 IMU 추정 roll/pitch와 그 측정 시각. `source`는 정확히 `sensor_imu`. camera capture 시각의 자세 이력에 사용한다. yaw는 패킷으로 주입하지 않고 알려진 초기 forward와 gyro로 추정한다. |

이미 변환된 프로젝트 `observer_packet`의 위 필드만 upstream이 명시적으로 구성하고 schema/IMU capture metadata를 추가해야 한다. 기존 physical 패킷의 설명용 `virtual_level_probe`, `mount_and_sensor_transfer_are_conditional`도 그대로 보내면 whitelist에 걸린다. physical estimator의 변환 수학은 출처 참조이며 gateway가 firmware/plant 객체를 생성하는 경로가 아니다.

`position_world_m`, 자체/차량 velocity, 차량 heading/phase/direction/speed, live track arc, `vehicle_plant_only`, `truth`, `surface_id`, contact 등은 입력으로 허용하지 않는다. 변환 publisher는 원본 센서와 센서 유래 IMU 자세만 사용해야 한다. 모델의 qpos/qvel/rotation, 차량 시간표·알려진 속도, truth 파일로 변환값이나 capture attitude를 만들면 이 계약을 충족하지 않는다.

**검증 가능한 출처 경계:** gateway는 타입/키/시각과 `sensor_imu` 선언을 검사하지만, 그 문자열만으로 publisher의 계산 출처를 입증할 수 없다. ROS 토픽은 인증된 센서 출처 증명이 아니다. upstream 소스·원본 센서 기록·해시 검토가 별도로 필요하다. 이번 시험의 출처는 고정 합성 native 영상과 합성 IMU/Flow/ToF 패킷이다. fixture 생성의 좌표는 영상 픽셀 생성에만 쓰고 online gateway에는 전달하지 않는다. 실제 기체 센서나 물리 모델의 truth/replay 파일은 읽지 않았다.

## 장착과 초기 handoff

정적 설정에서 선택한 entry XY·초기 forward와 1.0m 높이만 초기 위치/heading prior다. 초기 velocity는 0이다. 첫 유효 센서 패킷을 handoff/미션 시각 원점으로 삼는다. 실제 이륙·entry 도착·하드웨어 pose 측정은 구현하지 않았다. 미발견이면 현재 controller의 15초 고정 gaze WAIT를 보존하며, 발견과 acquire/coarse 전이는 현재 영상 관측이 결정한다. 차량 시작 신호나 움직임 시각 계산은 없다.

현재 profile-S 장착값을 정확히 읽어 `PacketMotion`에 적용한다.

- camera body 위치 `[0.020, 0.000, 0.015] m`
- camera RPY offset `[0, 0, 0] deg`; `rotation_rpy(offset) @ R_HEADING_CAMERA`
- Flow deck body 위치 `[0, 0, -0.005] m`, IMU 위치 `[0, 0, 0] m`는 upstream 변환 계약에 기록한다.
- `source_class=sensitivity_assumption`, `actual_manufacturer_transform=null`

camera 위치/회전은 capture-time `Observer` 변환에 적용한다. Flow/IMU 장착 위치는 이미 변환된 패킷에 두 번 적용하지 않는다. 이는 제조사 장착 확인이나 렌즈 K/distortion 실측을 대체하지 않는다. public camera 설정의 기존 observer optics 선택은 현재 관측기가 처리한다.

## 시각·freshness·영상 검사

wall-clock 모드에서는 ROS wall clock, `use_sim_time=true`에서는 `/clock`을 모든 producer와 gateway가 함께 사용해야 한다. node가 receipt 시각을 측정하며 패킷이 receipt를 주장할 수 없다. 절대 clock에서 첫 센서 capture를 빼서 추정기·영상·controller의 미션 상대 시각으로 변환한다. 이 계산은 차량 phase 추정에 쓰이지 않는다.

현재 소프트웨어 전송 한계는 다음과 같다. 제조사 수치나 성공 확률이 아니다.

| 검사 | 한계/동작 |
|---|---|
| 센서 capture → receipt/command | 최대 0.12초. capture와 receipt 두 age를 확인한다. |
| 영상 capture → receipt | 최대 `min(0.35, observer.image_max_age_s)`초 |
| range capture → packet capture | 최대 0.10초 |
| IMU attitude capture → packet capture | 최대 0.03초 |
| capture의 미래 허용 오차 | 0.002초; 음수 clock 거부 |
| 센서 순서 | capture가 엄격히 증가; duplicate/out-of-order 거부 |
| 센서 gap | 0.20초 초과 시 fault를 유지하고 restart 필요 |
| 새로운 range | `range_new=true`이면 자체 capture stamp가 엄격히 증가 |
| held range | 직전 자체 stamp/value/valid/accurate와 같아야 함; 반복값을 새 측정으로 표시하지 않음 |
| IMU 자세 순서 | 자체 stamp가 역행하면 거부; 같은 stamp에서 값 변경도 거부 |
| 영상 순서 | capture가 엄격히 증가; capture odometry/attitude history 필요 |
| clock 역행 | fault를 유지하고 restart 필요; 원점을 자동 재설정하지 않음 |
| sim clock 정지 | 새 command를 발행하지 않음; wall clock으로 미션을 진행하지 않음 |

capture odometry와 자세 보간은 현재 core와 같은 history API를 사용한다. history 마지막 시각 이후 최대 25ms의 끝값 허용도 같은 API에서 상속한다. 영상 receipt 시각의 최신 자세를 촬영 자세로 대신하지 않는다. 게이트웨이는 기존 image age·포즈 ambiguity·innovation·surface/Flow 융합 gate를 우회하지 않는다.

영상은 `header.frame_id=mission3_camera_optical`, width/height `320/320`, encoding `mono8`, 정확한 `height*step` byte 길이를 요구한다. `provided_hardware.decode_mono320`을 사용한다. row padding은 허용하고 실제 320열만 읽는다. 324/컬러를 resize/color-convert하지 않고 거부한다. mono8의 endian flag 0/1은 1byte 픽셀값을 바꾸지 않는다.

입력이 없거나 센서가 오래되면 `TRANSPORT_HOLD` phase와 `WAIT_INPUT`/`SENSOR_STALE` 상태로 0 목표 속도·motoroff=false를 보낸다. clock/gap/추론 fault는 `FAULT`로 기록한다. 이 전송 hold는 실제 기체의 비행 failsafe 구현이 아니다. 낮은 고도에서 센서 단절을 다루는 하위 물리/하드웨어 정책은 별도 연결 검증이 필요하다. 센서가 살아 있고 영상만 끊기는 경우에는 현재 controller의 유한 blind recovery를 그대로 실행한다.

## GEOMETRY_READY와 완료 경계

현재 controller가 `GEOMETRY_READY`를 출력하면 phase와 `geometry_ready=true`를 전달한다. status는 항상 `physical_success=false`, `deployment_complete=false`, `motors_off=false`다. 뜻밖에 core가 physical_success/motors_off를 요구해도 fault로 차단한다. 최신 SelfMotion/Observer의 출력은 `self_estimate`와 관측 결과로 기록하며 차량 속도·방향은 영상과 센서 추정에서만 생긴다.

11cm 이후 접촉 접근, IMU-command-force residual 접촉 관측, 모터 종료, 지지 3초, 2cm landing error 판정은 연결하지 않았다. 여기에 단순 시간 대기나 거리 조건을 더해 물리 착륙 완료를 주장하면 안 된다.

## 실제 검증 결과와 보존 산출물

최종 시험: `artifacts/mission3/ros-gateway/20261008-051735-028245-tests/`. **27/27 통과**, failure/error/skip 모두 0. boundary 24개와 ROS 시험 3개다. 실행 중 snapshot 대상 소스 변경은 없었다.

팀 환경은 Python 3.10.12, OpenCV 4.14.0, NumPy 2.2.6, ROS Humble, domain 25, `rmw_fastrtps_cpp`였다. 실제 ROS 노드/typed publisher/subscriber를 SingleThreadedExecutor에서 왕복시켰으며 일반 Python 함수 호출만으로 대체하지 않았다.

- wall-clock 3초 입력: 센서 300개, native 영상 59개를 수신했고 59개가 metric pose를 통과했다. 실제 관측으로 `ACQUIRE → LINEUP`을 확인했다. 합성 센서가 명령에 반응하는 물리 폐루프는 아니다.
- private sim clock: 센서 31개/영상 7개/metric pose 7개. clock 정지 중 command 증가 0, 역행 시 유지 fault 확인.
- 실행 가능한 별도 node subprocess에서 `/cf231/cmd_vel`로 command remap을 시도했고 publisher 생성 전에 exit 2로 거부했다. 그 로그의 command 발행 수는 0이다.
- 실제 ROS 입력에 live 차량 velocity 필드 1개, width 324 영상 1개, rgb8 영상 1개를 보내 모두 거부했다. 이를 의도한 음성 사례로 기록한다. 실제 node 출력 publisher 목록에는 project 토픽과 ROS 기본 토픽만 있다.
- 순서/중복/새 ToF와 held ToF 구분, 촬영 자세 이력, native row padding, invalid flow/accurate range, 센서 stale, 알려진 1m entry, 15초 미발견 WAIT, geometry-ready/예상 밖 motoroff 차단을 단위 시험으로 확인했다.

| 최종 측정 토픽 | 송신 Hz | 수신 Hz |
|---|---:|---:|
| sensor packet | 100.015 | 100.021 |
| native image | 20.002 | 20.001 |
| cmd_velocity | 50 목표 | 49.972 |
| phase | 50 목표 | 49.971 |
| status | 50 목표 | 49.970 |
| motors_off | 50 목표 | 49.968 |

주기는 첫/마지막 callback 간 평균이며 command 등은 warm-up을 뺀 2.5초 구간을 쓴다. 시험 허용 오차는 목표의 ±15%다. 장시간 부하·다중 호스트 지연·실제 카메라 throughput 검증으로 확대 해석하지 않는다.

시도별 기록을 모두 보존했다. 초기 실패를 최종 통과 기록으로 덮어쓰지 않았다.

| KST 디렉터리 | 실행 결과 | 원인/변경 |
|---|---|---|
| `20261008-050742-822231-tests` | 23개 중 1 failure | 첫 프레임 phase를 LINEUP으로 예상한 시험 오류. 현재 core의 ACQUIRE가 맞음. 합성 포즈 자체는 통과. |
| `20261008-050843-129067-tests` | 25개 중 1 failure/1 error | Humble에 없는 `qos_profile_clock`; 송신 중단 직전 큐에 남은 정상 패킷을 drain 전에 계수한 시험 오류. |
| `20261008-050954-203111-tests` | 25/25 통과 | Humble 호환 QoS와 queue drain 수정. |
| `20261008-051330-284344-tests` | 27/27 통과 | output remap/예상 밖 motoroff 차단 검증 추가. |
| `20261008-051735-028245-tests` | 27/27 통과 | 센서·영상·phase/status/motor의 실제 수신 주기 검사 추가. 최종 코드. |

각 실행의 `source-manifest.json`, `source/`, `public-config.json`, `mount-profile-S.json`, `unittest.log`, `test-result.json`을 확인할 수 있다. ROS 하위 시험에는 `transport.jsonl`, native PNG, `result.json`, remap 거부 process 로그를 남겼다. raw 센서는 허용된 패킷만 기록하며 거부 입력은 원인으로 보존한다. 지정 출력 디렉터리에 기존 source snapshot이 있으면 재사용을 거부한다. node의 `--record-images`를 켜면 수신 native PNG도 저장한다.

최종 실행 snapshot SHA-256:

```text
scripts/mission3_ros_gateway.py
00ea06a25bef14519b243bbe4279748c33bb37877b3c779c0d748e6336ee1a79
scripts/test_mission3_ros_gateway.py
deff67aa484950d5814c0c8afa150f66c1de42c40ceaa88b41b092f301b4858d
scripts/run_mission3_ros_gateway.sh
77c9753aa0e500660ab3270c81c8b1bd6b4c2bcc4f989471fb9b2cf37547b4df
public-config.json
43e9bf45600b560056418c701931a16d0c9dec12ca0c20286bc94837a2bf65a3
mount-profile-S.json
6b60dda5dc2b291df957bce981701ce6888feb411dd8b0fab47af99348aff255
```

공유 core의 정확한 당시 소스 해시는 같은 manifest에 있다. 향후 메인 변경 후에는 해당 snapshot과 새 실행을 구분해야 한다. 독립 검토/제조사 모델 확인/물리 폐루프 승격은 이번 sidecar에서 완료한 항목이 아니다. 이 파일들과 보존 로그를 인계하며, gateway/test process는 모두 종료됐다.
