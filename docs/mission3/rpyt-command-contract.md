# 오프라인 CF firmware RPYT 명령 계약

2026-10-08 KST. `scripts/mission3_rpyt_protocol.py`는 원하는 CF firmware RPYT 수치를 legacy 14byte payload로 직렬화하고 읽는다. 로컬 CFLib의 실제 packing과 공식 firmware decoder를 오프라인으로 확인했다. 라디오·driver 초기화·Crazyflie 연결·ROS/hardware 명령 전송은 없다. serializer 검증은 배포 준비나 물리 착륙 검증이 아니다.

## Main이 사용할 API

```python
from mission3_rpyt_protocol import encode_firmware_rpyt, decode_firmware_rpyt

# PCPositionPid.update의 CF native roll/pitch와 legacy_collective를 사용한다.
payload = encode_firmware_rpyt(
    roll_deg=6.0, pitch_deg=10.0, yaw_rate_deg_s=20.0, thrust=45000.9)
assert len(payload) == 14
assert decode_firmware_rpyt(payload) == {
    "roll_deg": 6.0, "pitch_deg": 10.0,
    "yaw_rate_deg_s": 20.0, "thrust": 45000}
```

- `encode_firmware_rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust, stop=False) -> bytes`: header 없는 little-endian 14byte legacy payload.
- `decode_firmware_rpyt(payload) -> dict`: 키는 정확히 `roll_deg`, `pitch_deg`, `yaw_rate_deg_s`, `thrust`. roll/pitch는 firmware native 자세각, yaw는 **firmware 내부 `setpoint.attitudeRate.yaw`**다. thrust는 packet의 raw uint16 collective count다.
- `legacy_thrust_count(thrust) -> int`: 범위/60000S cap/양수 방향 floor를 명시적으로 적용한다.
- `radians_to_firmware_degrees(roll_rad, pitch_rad, yaw_rate_rad_s, *, frame) -> tuple`: `frame="cf_firmware"` 인자가 필수다. 단위만 바꾸며 다른 좌표계를 CF로 변환하지 않는다. world_mission/ROS ENU/body FLU 태그는 거부한다.

Main의 `StockPid.update_rpyt`는 Python decoder로 payload 형식·유한값을 확인한 뒤 **같은 전체 14bytes**를 공식 `crtpCommanderRpytDecodeSetpoint`에 넘기면 된다. Python이 복원한 yaw를 다시 payload로 pack하거나, C 진입 전에 pitch/yaw를 또 반전하면 안 된다. 이 sidecar는 PCPositionPid/StockPid/C host/build/config를 변경하지 않는다.

## pitch와 yaw의 서로 다른 부호 경계

로컬 `src/crazyflie-lib-python/cflib/crazyflie/commander.py`의 `send_setpoint`는 client_xmode=False일 때 `<fffH`로 `(roll, -pitch, yawrate, thrust)`를 쓴다. 공식 firmware의 기본 roll/pitch 모드는 ANGLE, yaw 모드는 RATE다. decoder는 roll/pitch를 그대로 받고, legacy yaw rate에는 `-values->yaw`를 적용한다.

| 위치 | roll | pitch | yaw rate |
|---|---:|---:|---:|
| 원하는 firmware 내부 수치 | R | P | Y |
| CFLib `send_setpoint` 인자 | R | −P | −Y |
| wire `<fffH`의 float 세 개 | R | P | −Y |
| 기본 firmware decoder 출력 | R | P | Y |

따라서 **CFLib 인자는 `(R, -P, -Y, T)`**, serializer wire는 `(R, P, -Y, T)`다. pitch 역변환과 yaw 역변환은 각각 한 번 필요하다. CFLib API의 yawrate와 firmware 내부 yaw-rate가 같은 부호라고 가정하지 않는다.

예제 `(6, 10, 20, 45000.9)`는 wire `(6, 10, -20, 45000)`와 hex `0000c040000020410000a0c1c8af`가 된다. 공식 decoder에서 `(6, 10, 20, 45000)`을 얻었다. 비정수 1.1/−2.2/3.3은 float32 반올림 후 해당 float32 bit 패턴까지 일치한다. double 입력의 모든 소수점 자릿수가 보존된다는 뜻은 아니다.

공식 `commander.h`는 `DEFAULT_YAW_MODE=XMODE`다. 이 firmware 경로는 XMODE에서 추가 roll/pitch 회전을 하지 않는다. CFLib의 client_xmode=False는 **클라이언트 추가 회전 없음**을 뜻한다. 이를 firmware PLUSMODE 설정으로 해석하면 안 된다. client_xmode=True의 0.707 회전과 잘못된 pitch/yaw 인자를 넣은 음성 사례가 원하는 firmware 각도를 바꾸는 것도 시험했다.

## 단위·범위·추력 정책

roll/pitch는 deg, firmware yaw-rate는 deg/s다. 입력은 official positionController가 반환한 **CF firmware native 각도**다. ROS world 목표 속도, 기하 body RPY, 원시 IMU 자세를 이 API에 직접 보내면 안 된다. 좌표계 변환과 position/attitude 제어는 Main의 별도 책임이다. radian helper는 이미 CF frame인 수치에 한 번만 rad→deg 변환한다.

모든 수치는 bool/string을 제외한 유한 실수여야 한다. 각도/rate의 직렬화 범위는 유한 IEEE754 float32 범위다. serializer가 비행 각도/rate 한계를 새로 정하지 않는다. local CF2 positionController의 초기 roll/pitch 20deg limit는 공식 소프트웨어 튜닝값이며, serializer의 범위나 실측 제조사 안전 한계가 아니다. Main이 적용할 flight envelope/제어 gate는 별도다.

`thrust`는 `positionController`의 float legacy collective count, 즉 uint16 count 스케일이다. 0..1 추력 분율, Newton, 최종 PWM, 모터별 명령과 구별한다.

1. 실제 packet uint16 입력 범위는 `0..65535`다. 범위 밖, NaN/Infinity는 stop 요청에서도 거부한다.
2. 이 프로젝트의 encoder cap은 `LEGACY_THRUST_CAP_S=60000`이다. uint16 범위 안의 60000 초과만 cap한다. CFLib 문서의 60000 안내를 채택한 **소프트웨어 정책 S**이며 실측 최대 thrust/하드웨어 안전 보장이 아니다.
3. 정수화는 `floor(min(thrust, 60000))`다. 예: 45000.9→45000, 65535→60000. round-to-nearest나 0..1 자동 스케일을 하지 않는다.
4. decoder는 조사용이므로 wire의 60001..65535도 그대로 읽는다. Python decoder를 통과했다는 사실만으로 encoder cap이나 flight envelope를 통과했다고 해석하면 안 된다.

14byte 구성은 offset 0/4/8의 float32와 offset 12의 uint16이다. CRTP header는 반환 bytes에 포함하지 않는다. 로컬 CFLib 확인에서는 port=3/COMMANDER, channel=0, header=0x3c였다. header+payload 15bytes를 decoder에 넘기면 크기 오류로 거부한다.

## zero·thrust lock·접촉 경계

`stop=True`는 수치/stop 타입을 검증한 뒤 14개의 zero byte를 반환한다. Python 결과에는 thrust=0뿐이고 contact/physical_success/motors_off/landing 완료 필드를 만들지 않는다. 이 flag는 오프라인 명령의 의도이며, 기체가 실제로 정지했거나 pad에 지지됐다는 관측이 아니다. 비행 중 stop을 정당화하는 접촉 정책은 Main의 별도 sensor observer가 결정해야 한다.

공식 decoder는 초기 `thrustLocked=true`다. priority가 DISABLE이면 다시 잠그고, zero thrust 패킷을 받으면 unlock한다. 이번 host probe는 Main과 맞춰 `DISABLE → zero packet → CRTP priority(2)`를 실행했다. DISABLE을 유지한 채 다음 nonzero packet을 받으면 재잠기는 것도 확인했다. 이 동작은 실제 supervisor/RTOS/link 상태를 구현한 증거가 아니다.

raw thrust와 firmware가 허용한 setpoint thrust도 구별해야 한다. probe의 비배터리보정 빌드에서 raw 999는 decoder raw count=999지만 공식 C setpoint thrust=0이고, 1000은 1000이다. 배터리보정 옵션의 MIN_THRUST 분기는 이번 probe에서 활성화하지 않았다. 공식 소스는 그 옵션에서 THRUST_MIN/THRUST_MAX 기반의 다른 minimum을 사용한다. 잠금·minimum·assist mode 때문에 Python decoder의 raw count를 최종 모터 출력으로 취급하면 안 된다.

## 로컬 확인 방법과 결과

```bash
cd .
PYTHONDONTWRITEBYTECODE=1 src/crazyflie-ros2-sil/.venv/bin/python \
  scripts/test_mission3_rpyt_protocol.py
```

최종 실행은 `artifacts/mission3/rpyt-protocol/20261008-054746-273897-tests/`다. **20/20 통과**, failure/error/skip 모두 0, 실행 중 source 변경 0이었다.

- protocol 시험 12개: finite/type/float32/uint16 범위, 60000S cap/floor, signed zero, 단위·frame, 크기·wire NaN, stop0/contact 비승격.
- CFLib 시험 4개: 실제 vendor `Commander.send_setpoint`와 CRTPPacket을 실행해 **420개 비대칭 조합의 payload bytes가 전부 일치**했다. 잘못된 pitch/yaw와 client_xmode=True는 부호/회전 차이를 검출했다. zero stop payload도 일치했다.
- firmware 시험 4개: 공식 RPYT decoder block과 그 mode/회전 helper를 그대로 추출하여 artifacts 안의 C probe로 실행했다. 기본 ANGLE/ANGLE/RATE/XMODE의 desired CF 각도가 float32 bit 단위로 일치했다. priority/zero unlock/minimum과 stop0도 확인했다.

CFLib loader는 상위 패키지 namespace만 격리하고 실제 `commander.py`, `crtpstack.py`, `encoding.py`를 로드한다. driver/package init은 실행하지 않는다. `Commander`의 상대 객체는 `send_packet`으로 bytes를 저장하는 mock뿐이며 Crazyflie 객체를 생성하지 않는다. CRTP 링크 전송은 없었다.

C probe는 공식 decoder block을 변경하지 않고 host-only setpoint 타입·priority 조회·미사용 position reset stub을 제공했다. `/usr/bin/cc` 11.4.0과 팀 Python 3.10.12를 사용했고, `.c/.so/build.json`은 이 sidecar artifacts 안에만 생성했다. 이는 Main의 별도 full official decoder host 연결을 대체하지 않으며 실제 firmware ABI/RTOS/supervisor/watchdog/radio latency/mixer/battery response/기체 반응을 검증하지 않는다.

모든 시도를 보존했다. `20261008-054449-352605-tests` 20/20은 처음 priority=1 stub이었다. `20261008-054550-052393-tests` 20/20에서 Main의 DISABLE/zero/CRTP 순서를 적용했다. 최종 `054746` 20/20은 같은 순서를 유지하고 unittest discovery용 artifact 경로도 제공한다. 이번 세 시도에 실제 실패는 없었다. 의도한 잘못된 부호/client mode 사례는 성공 테스트 내부의 음성 사례이며 올바른 착륙으로 승격하지 않는다.

각 실행의 `source-manifest.json`, `source/`, `unittest.log`, `test-result.json`, `mock-cflib-packets.jsonl`, `official-decoder-roundtrips.json`, `official-decoder-probe/`에 bytes·소스 해시·컴파일 결과를 남겼다. 기존 nonempty 출력 디렉터리는 덮어쓰지 않는다.

참조한 로컬 공식 소스/문서의 commit과 핵심 SHA-256:

```text
CFLib commit: 3c1cb0d024fef6f45d08f0ad236a22ac395dd924
firmware commit: f6e0f0a3b526861328caf3296a7ecf00bb915d39
cflib/crazyflie/commander.py:
33a000502846668583447859d5df03dc800b8edfea59e35ea2f4e02cd6ec345b
firmware crtp_commander_rpyt.c:
2944a745f89fd7cbde5c13721430120afeca8210aa7ee655a60166d352aee29d
official decoder extracted block:
2b7644e957f8e8b7bcdba7fb4955e05d4766f5f419982a94cace4647fa5479ad
scripts/mission3_rpyt_protocol.py:
72eb89c6ef42d7d6cf32f5dbbcab54ea39cc318a5791e32d48533c0f7711e550
```

`src/crazyflie-lib-python/docs/user-guides/python_api.md`의 send_setpoint 절, `docs/api/index.md`와 실제 함수 docstring, firmware `commander.h`, `platform_defaults_cf2.h`, `position_controller_pid.c`도 읽기 전용으로 대조했다. 전체 SHA는 같은 manifest에 있다. Main의 PC position→RPYT→공식 attitude/mixer→조건부 physical 경로는 별도 작업이며, 여기서는 그 폐루프 성공을 주장하지 않는다.
