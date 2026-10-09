"""오프라인 CF legacy RPYT payload. 연결/라디오/ROS/접촉 판정 없음.

입력 roll/pitch는 positionController의 CF firmware 자세 각도(deg),
yaw_rate는 firmware setpoint.attitudeRate.yaw(deg/s)다. 기본 firmware의
ANGLE/ANGLE/RATE + XMODE, CFLib client_xmode=False 계약에 한정한다.
"""
import math
from numbers import Real
import struct


RPYT_FORMAT = "<fffH"
RPYT_PAYLOAD_BYTES = struct.calcsize(RPYT_FORMAT)
UINT16_THRUST_MAX = 65535
# 프로젝트 소프트웨어 상한 S. 실측 최대 추력이나 기체 안전 한계가 아니다.
LEGACY_THRUST_CAP_S = 60000
FLOAT32_MAX = float.fromhex("0x1.fffffep+127")
FIRMWARE_FRAME = "cf_firmware"
_PACKET = struct.Struct(RPYT_FORMAT)


class RPYTContractError(ValueError):
    """오프라인 payload/수치/좌표계 계약 위반."""


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise RPYTContractError(f"{name}: finite real number required, not bool/string")
    try:
        value = float(value)
    except (OverflowError, ValueError) as exc:
        raise RPYTContractError(f"{name}: finite real number required") from exc
    if not math.isfinite(value):
        raise RPYTContractError(f"{name}: finite real number required")
    return value


def _angle(value, name):
    value = _finite(value, name)
    if abs(value) > FLOAT32_MAX:
        raise RPYTContractError(f"{name}: outside finite float32 payload bounds")
    return value


def legacy_thrust_count(thrust):
    """legacy collective count -> floor(min(value, 60000S)). 0..1 분율 아님.

    실제 uint16 범위 0..65535 밖은 거부한다. 입력을 클램프해서 수치 오류를
    숨기지 않으며, 유효 범위 안의 60000S 초과에만 명시적 software cap을 쓴다.
    """
    thrust = _finite(thrust, "thrust")
    if not 0 <= thrust <= UINT16_THRUST_MAX:
        raise RPYTContractError("thrust: outside actual uint16 bounds [0, 65535]")
    return math.floor(min(thrust, LEGACY_THRUST_CAP_S))


def radians_to_firmware_degrees(roll_rad, pitch_rad, yaw_rate_rad_s, *, frame):
    """단위만 변환한다. world/ROS 자세를 CF body 자세로 바꾸는 함수가 아니다."""
    if frame != FIRMWARE_FRAME:
        raise RPYTContractError("radian input must already use frame='cf_firmware'")
    result = tuple(math.degrees(_finite(v, n)) for v, n in
                   ((roll_rad, "roll_rad"), (pitch_rad, "pitch_rad"), (yaw_rate_rad_s, "yaw_rate_rad_s")))
    return tuple(_angle(v, n) for v, n in zip(result, ("roll_deg", "pitch_deg", "yaw_rate_deg_s")))


def encode_firmware_rpyt(roll_deg, pitch_deg, yaw_rate_deg_s, thrust, stop=False):
    """원하는 CF firmware R/P/Y-rate/T -> little-endian 14byte payload.

    CFLib 인자는 (roll, -pitch, -yaw_rate, 정수 thrust)다. CFLib가 pitch를
    반전하고 firmware가 legacy yaw-rate를 반전하므로 wire=(roll,pitch,-yaw,T).
    반환값에는 CRTP port/channel/header가 포함되지 않는다.

    stop=True는 검증 후 14개의 0 byte를 반환한다. 접촉을 판정하거나
    motor-off의 정당성을 부여하지 않으며 실제 전송도 하지 않는다.
    """
    roll = _angle(roll_deg, "roll_deg")
    pitch = _angle(pitch_deg, "pitch_deg")
    yaw_rate = _angle(yaw_rate_deg_s, "yaw_rate_deg_s")
    count = legacy_thrust_count(thrust)
    if type(stop) is not bool:
        raise RPYTContractError("stop: bool required")
    if stop:
        return bytes(RPYT_PAYLOAD_BYTES)
    return _PACKET.pack(roll, pitch, -yaw_rate, count)


def decode_firmware_rpyt(payload):
    """14byte payload -> CF firmware 수치. 기본 XMODE/ANGLE/ANGLE/RATE 가정.

    thrust는 packet의 uint16 raw count다. firmware의 thrust lock/minimum,
    priority/assist mode/supervisor/mixer/motor 동작은 해석하지 않는다.
    decode는 cap을 적용하지 않으므로 60001..65535 wire도 조사할 수 있다.
    """
    try:
        view = memoryview(payload)
    except TypeError as exc:
        raise RPYTContractError("payload: bytes-like buffer required") from exc
    if view.nbytes != RPYT_PAYLOAD_BYTES:
        raise RPYTContractError("payload: exactly 14 bytes required, excluding CRTP header")
    roll, pitch, wire_yaw, thrust = _PACKET.unpack(view.tobytes())
    for value, name in ((roll, "wire roll"), (pitch, "wire pitch"), (wire_yaw, "wire yaw-rate")):
        _angle(value, name)
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_rate_deg_s": -wire_yaw, "thrust": thrust}
