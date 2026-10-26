#!/usr/bin/env python3
"""Stock logger snapshots: strict units/axes/clock metadata, no flight adapter.

Firmware f6e0f0a3b526861328caf3296a7ecf00bb915d39 and CFLib
3c1cb0d024fef6f45d08f0ad236a22ac395dd924 define the fields. A logger timestamp
is not a sensor capture timestamp. This module never estimates Flow dt, ToF
sample identity, IMU averaging, sensor validity, or a PhysicalMotion input.
It imports no mission runtime, radio driver, or third-party package.
"""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
import math
import struct
from types import MappingProxyType

FIRMWARE_COMMIT = "f6e0f0a3b526861328caf3296a7ecf00bb915d39"
CFLIB_COMMIT = "3c1cb0d024fef6f45d08f0ad236a22ac395dd924"
GRAVITY_UNIT_M_S2 = 9.81
TIMESTAMP_MODULUS_MS = 1 << 24
TIMESTAMP_HALF_MS = TIMESTAMP_MODULUS_MS // 2
FLOAT32_MAX = struct.unpack("<f", bytes.fromhex("ffff7f7f"))[0]
UNKNOWN = "UNKNOWN"


class TelemetryContractError(ValueError):
    """Stable error code; rejected input does not advance clock/cache state."""

    def __init__(self, code, detail=""):
        self.code = code
        super().__init__(code + (": " + detail if detail else ""))


@dataclass(frozen=True)
class Field:
    name: str
    fetch_type: str
    wire_format: str
    minimum: int | None = None
    maximum: int | None = None


BLOCK_FIELDS = MappingProxyType({
    "imu": tuple(Field(f"{group}.{axis}", "float", "f")
                 for group in ("acc", "gyro") for axis in "xyz"),
    "deck_power": (
        *(Field(f"motor.m{i}", "uint16_t", "H", 0, 65535) for i in range(1, 5)),
        Field("motion.deltaX", "int16_t", "h", -32768, 32767),
        Field("motion.deltaY", "int16_t", "h", -32768, 32767),
        Field("motion.motion", "uint8_t", "B", 0, 255),
        Field("motion.squal", "uint8_t", "B", 0, 255),
        Field("motion.shutter", "uint16_t", "H", 0, 65535),
        Field("range.zrange", "uint16_t", "H", 0, 65535),
        Field("pm.vbat", "float", "f"),
    ),
})


def _fields(block):
    if not isinstance(block, str) or block not in BLOCK_FIELDS:
        raise TelemetryContractError("unknown_block", repr(block))
    return BLOCK_FIELDS[block]


def payload_size(block):
    return struct.calcsize("<" + "".join(f.wire_format for f in _fields(block)))


def make_log_configs(log_config_class):
    """Build two explicit native-type CFLib LogConfigs, requesting 10ms.

    Caller supplies the pinned official LogConfig class. No connection/start is
    performed. Actual device TOC availability/type must still be checked.
    """
    configs = {}
    for block, fields in BLOCK_FIELDS.items():
        config = log_config_class("mission3_" + block, period_in_ms=10)
        for field in fields:
            config.add_variable(field.name, field.fetch_type)
        if payload_size(block) > config.MAX_LEN:
            raise TelemetryContractError("log_payload_exceeds_limit", block)
        configs[block] = config
    return configs


def validate_values(block, values):
    """Validate the exact native callback schema; retain full integer ranges."""
    fields = _fields(block)
    if not isinstance(values, Mapping):
        raise TelemetryContractError("invalid_mapping")
    if set(values) != {f.name for f in fields}:
        raise TelemetryContractError("field_schema_mismatch", block)
    result = {}
    for field in fields:
        value = values[field.name]
        if field.fetch_type == "float":
            if type(value) is not float:
                raise TelemetryContractError("invalid_float_type", field.name)
            if not math.isfinite(value) or abs(value) > FLOAT32_MAX:
                raise TelemetryContractError("invalid_float_value", field.name)
        else:
            if type(value) is not int:
                raise TelemetryContractError("invalid_integer_type", field.name)
            if not field.minimum <= value <= field.maximum:
                raise TelemetryContractError("integer_out_of_range", field.name)
        result[field.name] = value
    return result


def decode_payload(block, payload):
    """Decode only the variable payload, excluding block ID + 3 stamp bytes."""
    fields = _fields(block)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TelemetryContractError("invalid_payload_type")
    raw = bytes(payload)
    if len(raw) != payload_size(block):
        raise TelemetryContractError("invalid_payload_length", block)
    values = struct.unpack("<" + "".join(f.wire_format for f in fields), raw)
    return validate_values(block, {f.name: v for f, v in zip(fields, values)})


def normalize_values(block, values):
    """Units and documented driver axis mapping; no validity or time inference."""
    raw = validate_values(block, values)
    record = {
        "schema": "mission3-stock-telemetry/1",
        "record_kind": "logger_snapshot",
        "block": block,
        "raw": raw,
        "measurement_capture_time_s": None,
        "measurement_capture_time_status": UNKNOWN,
        "captured_dt_s": None,
        "captured_dt_status": UNKNOWN,
        "imu_average_window_s": None,
        "imu_average_window_status": UNKNOWN,
        "measurement_sample_identity": UNKNOWN,
        "physical_motion_ready": False,
        "firmware_commit": FIRMWARE_COMMIT,
        "cflib_commit": CFLIB_COMMIT,
    }
    if block == "imu":
        record["specific_acceleration_body_m_s2"] = [raw["acc." + a] * GRAVITY_UNIT_M_S2 for a in "xyz"]
        record["gyro_body_rad_s"] = [math.radians(raw["gyro." + a]) for a in "xyz"]
        record["imu_semantics"] = "sensorData snapshot; firmware filtering/startup calibration; averaging UNKNOWN"
    else:
        record["motor_pwm_counts"] = [raw[f"motor.m{i}"] for i in range(1, 5)]
        record["flow_sensor_delta_counts"] = [raw["motion.deltaX"], raw["motion.deltaY"]]
        record["flow_body_axis_counts_unfiltered"] = [-raw["motion.deltaY"], -raw["motion.deltaX"]]
        record["flow_axis_semantics"] = "driver (-deltaY,-deltaX); wide signed arithmetic, no C overflow/filter/outlier emulation"
        record["flow_motion_raw"] = raw["motion.motion"]
        record["flow_squal_raw"] = raw["motion.squal"]
        record["flow_shutter_raw"] = raw["motion.shutter"]
        record["flow_measurement_validity"] = UNKNOWN
        record["range_m"] = raw["range.zrange"] / 1000.0
        record["range_measurement_validity"] = UNKNOWN
        record["battery_voltage_V"] = raw["pm.vbat"]
        record["motor_semantics"] = "commanded PWM ratios; measured rotor force/RPM UNKNOWN"
    return record


def _host_time(value):
    if type(value) not in (int, float):
        raise TelemetryContractError("invalid_host_time")
    try:
        number = float(value)
    except OverflowError as error:
        raise TelemetryContractError("invalid_host_time") from error
    if not math.isfinite(number) or number < 0:
        raise TelemetryContractError("invalid_host_time")
    return number


@dataclass(frozen=True)
class ClockPolicy:
    """Explicit client diagnostic limits, not manufacturer clock guarantees."""
    reorder_window_ms: int
    max_forward_gap_ms: int
    max_host_gap_s: float

    def __post_init__(self):
        if (type(self.reorder_window_ms) is not int
                or type(self.max_forward_gap_ms) is not int
                or not 0 <= self.reorder_window_ms < self.max_forward_gap_ms < TIMESTAMP_HALF_MS):
            raise TelemetryContractError("invalid_clock_policy")
        if _host_time(self.max_host_gap_s) <= 0:
            raise TelemetryContractError("invalid_clock_policy")


class StockTelemetryParser:
    """Shared two-block 24bit unwrap in a caller-declared boot session.

    Small backwards packets and larger clock backjumps have distinct rejection
    codes. A device reset near wrap is indistinguishable from a genuine wrap
    using these fields alone: caller must reset_session on reconnect/reboot.
    Session-relative logger time is never a measurement capture time.
    """

    def __init__(self, session_id, clock_policy):
        if not isinstance(clock_policy, ClockPolicy):
            raise TelemetryContractError("invalid_clock_policy")
        self.policy = clock_policy
        self.session_id = None
        self.reset_session(session_id)

    def reset_session(self, session_id):
        if not isinstance(session_id, str) or not session_id.strip() or session_id == self.session_id:
            raise TelemetryContractError("invalid_new_session_id")
        self.session_id = session_id
        self._anchor_raw = None
        self._high_ms = None
        self._last_host = None
        self._latest = {}

    def ingest_payload(self, block, payload, timestamp_ms, received_monotonic_s):
        return self.ingest_callback(block, decode_payload(block, payload), timestamp_ms, received_monotonic_s)

    def ingest_callback(self, block, values, timestamp_ms, received_monotonic_s):
        # All validation precedes the state commit; malformed packets cannot
        # make a good later callback stale, duplicate or out of order.
        record = normalize_values(block, values)
        if type(timestamp_ms) is not int or not 0 <= timestamp_ms < TIMESTAMP_MODULUS_MS:
            raise TelemetryContractError("invalid_logger_timestamp")
        host = _host_time(received_monotonic_s)
        if self._last_host is not None:
            host_gap = host - self._last_host
            if host_gap < 0:
                raise TelemetryContractError("host_clock_backjump")
            if host_gap >= TIMESTAMP_HALF_MS / 1000:
                raise TelemetryContractError("wrap_count_ambiguous")
            if host_gap > self.policy.max_host_gap_s:
                raise TelemetryContractError("host_gap_exceeds_policy")
        if self._high_ms is None:
            candidate = 0
            event = "session_first_snapshot"
        else:
            high_raw = (self._anchor_raw + self._high_ms) % TIMESTAMP_MODULUS_MS
            delta = (timestamp_ms - high_raw + TIMESTAMP_HALF_MS) % TIMESTAMP_MODULUS_MS - TIMESTAMP_HALF_MS
            if delta == -TIMESTAMP_HALF_MS:
                raise TelemetryContractError("timestamp_half_wrap_ambiguous")
            candidate = self._high_ms + delta
            if delta > self.policy.max_forward_gap_ms:
                raise TelemetryContractError("logger_forward_gap_exceeds_policy")
            event = "forward_wrap_assuming_same_boot" if delta > 0 and timestamp_ms < high_raw else "forward"
        previous = self._latest.get(block)
        interval = None
        if previous is not None:
            gap = candidate - previous["logger_time_since_session_first_ms"]
            if gap == 0:
                raise TelemetryContractError("duplicate_block_logger_timestamp")
            if gap < 0:
                code = "out_of_order_block_packet" if -gap <= self.policy.reorder_window_ms else "clock_backjump_requires_reset"
                raise TelemetryContractError(code)
            interval = gap / 1000.0
            if timestamp_ms < previous["logger_timestamp_24bit_ms"]:
                event = "forward_wrap_assuming_same_boot"
        elif self._high_ms is not None and candidate < self._high_ms - self.policy.reorder_window_ms:
            raise TelemetryContractError("first_block_timestamp_too_old_or_reset")
        high_ms = candidate if self._high_ms is None else max(self._high_ms, candidate)
        record.update({
            "session_id": self.session_id,
            "logger_timestamp_24bit_ms": timestamp_ms,
            "logger_time_since_session_first_ms": candidate,
            "logger_timestamp_semantics": "block acquisition, session relative; boot epoch UNKNOWN",
            "logger_snapshot_interval_s": interval,
            "timestamp_unwrap_event": event,
            "logger_behind_high_watermark_ms": high_ms - candidate,
            "boot_reset_detection": "NOT_INFERABLE_FROM_24BIT_ONLY; caller declares reconnect/reboot",
            "received_monotonic_s": host,
        })
        if self._anchor_raw is None:
            self._anchor_raw = timestamp_ms
        self._high_ms = high_ms
        self._last_host = host
        self._latest[block] = deepcopy(record)
        return record

    def snapshot_view(self, now_monotonic_s, *, stale_after_s, max_block_skew_s):
        """Expose separate snapshots/receipt ages/skew, never a merged sample.

        Caller-selected freshness/skew budgets describe logger transport only.
        A 'within_logger_policy' view is still not PhysicalMotion-ready.
        """
        now = _host_time(now_monotonic_s)
        stale = _host_time(stale_after_s)
        skew_limit = _host_time(max_block_skew_s)
        if self._last_host is not None and now < self._last_host:
            raise TelemetryContractError("view_before_last_host_receipt")
        blocks = deepcopy(self._latest)
        ages = {name: now - r["received_monotonic_s"] for name, r in blocks.items()}
        missing = [name for name in BLOCK_FIELDS if name not in blocks]
        stale_blocks = [name for name, age in ages.items() if age > stale]
        logger_skew = receipt_skew = None
        if not missing:
            a, b = blocks["imu"], blocks["deck_power"]
            logger_skew = abs(a["logger_time_since_session_first_ms"] - b["logger_time_since_session_first_ms"]) / 1000.0
            receipt_skew = abs(a["received_monotonic_s"] - b["received_monotonic_s"])
        status = ("missing_block" if missing else "stale_block" if stale_blocks
                  else "block_logger_skew" if logger_skew > skew_limit else "within_logger_policy")
        return {"record_kind": "separate_logger_snapshots", "session_id": self.session_id,
                "blocks": blocks, "status": status, "missing_blocks": missing,
                "stale_blocks": stale_blocks, "host_receipt_age_s": ages,
                "age_semantics": "time since PC callback receipt, not measurement age or radio latency",
                "logger_block_skew_s": logger_skew, "host_receipt_skew_s": receipt_skew,
                "client_policy_s": {"stale_after": stale, "max_block_skew": skew_limit},
                "captured_dt_status": UNKNOWN, "physical_motion_ready": False}
