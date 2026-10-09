#!/usr/bin/env python3
"""CPU-only native logger contract tests; no radio or mission imports."""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import platform
import struct
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

from mission3_stock_telemetry import (
    BLOCK_FIELDS, CFLIB_COMMIT, FIRMWARE_COMMIT, FLOAT32_MAX,
    TIMESTAMP_HALF_MS, TIMESTAMP_MODULUS_MS, ClockPolicy,
    StockTelemetryParser, TelemetryContractError, decode_payload,
    make_log_configs, normalize_values, payload_size,
)

ROOT = Path(__file__).resolve().parents[1]
CFLIB = ROOT / "src/crazyflie-lib-python"
FIRMWARE = ROOT / "src/bitcraze-mission3-firmware"
EVIDENCE = {}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def imu_values():
    return dict(zip((f.name for f in BLOCK_FIELDS["imu"]), (1.0, -0.5, 0.0, 180.0, -90.0, 0.0)))


def deck_values():
    return dict(zip((f.name for f in BLOCK_FIELDS["deck_power"]),
                    (0, 1, 32768, 65535, 123, -456, 0xB0, 0, 65535, 40, 4.2)))


def pack(block, values):
    return struct.pack("<" + "".join(f.wire_format for f in BLOCK_FIELDS[block]),
                       *(values[f.name] for f in BLOCK_FIELDS[block]))


def parser():
    return StockTelemetryParser("declared-boot-A", ClockPolicy(50, 1000, 10.0))


def pinned_file(repo, commit, relative):
    path = repo / relative
    pinned = subprocess.run(["git", "show", commit + ":" + relative], cwd=repo,
                            check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE).stdout
    if path.read_bytes() != pinned:
        raise AssertionError("local source differs from pinned official commit: " + str(path))
    return {"path": str(path), "commit": commit, "sha256": sha(path),
            "pinned_git_blob_bytes_equal": True}


def load_pinned_log():
    """Execute four exact official files under inert package containers."""
    relative = ("cflib/crtp/crtpstack.py", "cflib/utils/callbacks.py",
                "cflib/crazyflie/toc.py", "cflib/crazyflie/log.py")
    EVIDENCE["cflib_sources"] = [pinned_file(CFLIB, CFLIB_COMMIT, p) for p in relative]
    modules = {}
    for name in ("cflib", "cflib.crtp", "cflib.utils", "cflib.crazyflie"):
        module = types.ModuleType(name)
        module.__path__ = [str(CFLIB / Path(*name.split(".")))]
        modules[name] = module
    before = set(sys.modules)
    with patch.dict(sys.modules, modules):
        for rel in relative:
            name = rel[:-3].replace("/", ".")
            spec = importlib.util.spec_from_file_location(name, CFLIB / rel)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            spec.loader.exec_module(module)
        added = sorted(set(sys.modules) - before)
        forbidden = [n for n in added if n.startswith("cflib") and any(
            p in n for p in ("radiodriver", "usbdriver", "crazyradio", "link_cpp"))]
        if forbidden:
            raise AssertionError("unexpected hardware module: " + repr(forbidden))
        EVIDENCE["offline_vendor_modules_loaded"] = [n for n in added if n.startswith("cflib")]
        EVIDENCE["hardware_driver_imported"] = False
        return sys.modules["cflib.crazyflie.log"]


class ContractTests(unittest.TestCase):
    def assertCode(self, code, callback):
        with self.assertRaises(TelemetryContractError) as context:
            callback()
        self.assertEqual(context.exception.code, code)

    def test_native_payload_sizes(self):
        self.assertEqual(payload_size("imu"), 24)
        self.assertEqual(payload_size("deck_power"), 22)

    def test_si_units_preserve_body_axes(self):
        r = normalize_values("imu", imu_values())
        self.assertEqual(r["specific_acceleration_body_m_s2"], [9.81, -4.905, 0.0])
        self.assertEqual(r["gyro_body_rad_s"], [math.pi, -math.pi / 2, 0.0])

    def test_flow_driver_axes_and_pwm_order(self):
        r = normalize_values("deck_power", deck_values())
        self.assertEqual(r["flow_sensor_delta_counts"], [123, -456])
        self.assertEqual(r["flow_body_axis_counts_unfiltered"], [456, -123])
        self.assertEqual(r["motor_pwm_counts"], [0, 1, 32768, 65535])
        self.assertEqual(r["range_m"], .04)
        self.assertEqual(r["battery_voltage_V"], 4.2)

    def test_int16_extrema_use_wide_mapping_no_overflow_emulation(self):
        v = deck_values();v["motion.deltaX"] = -32768;v["motion.deltaY"] = 32767
        r = normalize_values("deck_power", v)
        self.assertEqual(r["flow_body_axis_counts_unfiltered"], [-32767, 32768])
        self.assertEqual(r["flow_measurement_validity"], "UNKNOWN")

    def test_squal_and_motion_are_metadata_not_validity_gate(self):
        for squal, motion in ((0, 0), (255, 0xB0)):
            v = deck_values();v["motion.squal"] = squal;v["motion.motion"] = motion
            r = normalize_values("deck_power", v)
            self.assertEqual(r["flow_squal_raw"], squal)
            self.assertEqual(r["flow_motion_raw"], motion)
            self.assertEqual(r["flow_measurement_validity"], "UNKNOWN")
            self.assertFalse(r["physical_motion_ready"])

    def test_range_native_extrema_do_not_invent_sentinel(self):
        for value in (0, 9, 65535):
            v = deck_values();v["range.zrange"] = value
            r = normalize_values("deck_power", v)
            self.assertEqual(r["range_m"], value / 1000)
            self.assertEqual(r["range_measurement_validity"], "UNKNOWN")

    def test_capture_dt_and_imu_box_average_remain_unknown(self):
        for block, values in (("imu", imu_values()), ("deck_power", deck_values())):
            r = normalize_values(block, values)
            self.assertEqual(r["record_kind"], "logger_snapshot")
            for field in ("captured_dt", "measurement_capture_time", "imu_average_window"):
                self.assertIsNone(r[field + "_s"])
                self.assertEqual(r[field + "_status"], "UNKNOWN")
            self.assertFalse(r["physical_motion_ready"])

    def test_payload_roundtrip(self):
        for block, values in (("imu", imu_values()), ("deck_power", deck_values())):
            expected = dict(values)
            if block == "deck_power":
                expected["pm.vbat"] = struct.unpack("<f", struct.pack("<f", 4.2))[0]
            self.assertEqual(decode_payload(block, pack(block, values)), expected)

    def test_bad_lengths_include_packet_header_and_truncation(self):
        for block, values in (("imu", imu_values()), ("deck_power", deck_values())):
            payload = pack(block, values)
            for bad in (b"", payload[:-1], payload + b"\0", b"\x01\x00\x00\x00" + payload):
                self.assertCode("invalid_payload_length", lambda: decode_payload(block, bad))

    def test_bad_payload_and_block_types(self):
        for bad in (None, [], "00", 24):
            self.assertCode("invalid_payload_type", lambda: decode_payload("imu", bad))
        for block in (None, [], "unknown"):
            self.assertCode("unknown_block", lambda: payload_size(block))

    def test_callback_schema_exact(self):
        self.assertCode("invalid_mapping", lambda: normalize_values("imu", []))
        for missing in (True, False):
            v = imu_values()
            if missing:del v["acc.x"]
            else:v["capture_dt"] = .01
            self.assertCode("field_schema_mismatch", lambda: normalize_values("imu", v))

    def test_float_types_and_nonfinite(self):
        for value in (True, 1, "1", None):
            v = imu_values();v["acc.x"] = value
            self.assertCode("invalid_float_type", lambda: normalize_values("imu", v))
        for value in (float("nan"), float("inf"), -float("inf"), FLOAT32_MAX * 2):
            v = imu_values();v["acc.x"] = value
            self.assertCode("invalid_float_value", lambda: normalize_values("imu", v))

    def test_integer_types_and_bounds(self):
        for field in BLOCK_FIELDS["deck_power"]:
            if field.fetch_type == "float":continue
            for value in (False, 1.0, "1", None):
                v = deck_values();v[field.name] = value
                self.assertCode("invalid_integer_type", lambda: normalize_values("deck_power", v))
            for value in (field.minimum - 1, field.maximum + 1):
                v = deck_values();v[field.name] = value
                self.assertCode("integer_out_of_range", lambda: normalize_values("deck_power", v))

    def test_nonfinite_wire_float_rejected(self):
        payload = bytearray(pack("imu", imu_values()));payload[:4] = struct.pack("<f", float("nan"))
        self.assertCode("invalid_float_value", lambda: decode_payload("imu", payload))

    def test_snapshot_fields_are_copy_isolated(self):
        p = parser();v = imu_values();r = p.ingest_callback("imu", v, 100, 1.0)
        v["acc.x"] = 100.0;r["raw"]["acc.x"] = 200.0
        view = p.snapshot_view(1.0, stale_after_s=.1, max_block_skew_s=.02)
        self.assertEqual(view["blocks"]["imu"]["raw"]["acc.x"], 1.0)
        view["blocks"]["imu"]["raw"]["acc.x"] = 300.0
        self.assertEqual(p.snapshot_view(1.0, stale_after_s=.1, max_block_skew_s=.02)["blocks"]["imu"]["raw"]["acc.x"], 1.0)

    def test_repeated_sensor_values_with_new_logger_stamps_accepted(self):
        p = parser();p.ingest_callback("deck_power", deck_values(), 100, 1.0)
        r = p.ingest_callback("deck_power", deck_values(), 110, 1.01)
        self.assertEqual(r["logger_snapshot_interval_s"], .01)
        self.assertEqual(r["captured_dt_status"], "UNKNOWN")
        self.assertEqual(r["measurement_sample_identity"], "UNKNOWN")

    def test_shared_clock_same_stamp_different_blocks_not_duplicate(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        r = p.ingest_callback("deck_power", deck_values(), 100, 1.0)
        self.assertEqual(r["logger_time_since_session_first_ms"], 0)
        view = p.snapshot_view(1.0, stale_after_s=.1, max_block_skew_s=0.0)
        self.assertEqual(view["status"], "within_logger_policy")
        self.assertFalse(view["physical_motion_ready"])

    def test_wrap_keeps_two_blocks_in_same_epoch(self):
        p = parser();p.ingest_callback("imu", imu_values(), TIMESTAMP_MODULUS_MS - 10, 1.0)
        a = p.ingest_callback("imu", imu_values(), 5, 1.015)
        b = p.ingest_callback("deck_power", deck_values(), 8, 1.018)
        self.assertEqual(a["logger_time_since_session_first_ms"], 15)
        self.assertEqual(b["logger_time_since_session_first_ms"], 18)
        self.assertEqual(a["timestamp_unwrap_event"], "forward_wrap_assuming_same_boot")
        self.assertEqual(p.snapshot_view(1.02, stale_after_s=.1, max_block_skew_s=.01)["logger_block_skew_s"], .003)

    def test_late_other_block_near_wrap_has_correct_prior_epoch(self):
        p = parser();p.ingest_callback("imu", imu_values(), TIMESTAMP_MODULUS_MS - 10, 1.0)
        p.ingest_callback("imu", imu_values(), 5, 1.015)
        b = p.ingest_callback("deck_power", deck_values(), TIMESTAMP_MODULUS_MS - 5, 1.016)
        self.assertEqual(b["logger_time_since_session_first_ms"], 5)
        self.assertEqual(b["logger_behind_high_watermark_ms"], 10)

    def test_duplicate_block_logger_timestamp_separate(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("duplicate_block_logger_timestamp", lambda: p.ingest_callback("imu", imu_values(), 100, 1.01))

    def test_small_backwards_out_of_order_does_not_advance_state(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("out_of_order_block_packet", lambda: p.ingest_callback("imu", imu_values(), 95, 1.01))
        r = p.ingest_callback("imu", imu_values(), 110, 1.005)
        self.assertEqual(r["logger_snapshot_interval_s"], .01)

    def test_large_backjump_requires_explicit_reset(self):
        p = parser();p.ingest_callback("imu", imu_values(), 10000, 1.0)
        self.assertCode("clock_backjump_requires_reset", lambda: p.ingest_callback("imu", imu_values(), 10, 1.01))
        p.reset_session("declared-boot-B")
        r = p.ingest_callback("imu", imu_values(), 10, 1.02)
        self.assertEqual(r["logger_time_since_session_first_ms"], 0)
        view = p.snapshot_view(1.02, stale_after_s=.1, max_block_skew_s=.01)
        self.assertEqual(view["missing_blocks"], ["deck_power"])

    def test_explicit_reset_prevents_cross_session_pairing(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        p.ingest_callback("deck_power", deck_values(), 100, 1.001)
        p.reset_session("declared-boot-B")
        self.assertEqual(p.snapshot_view(1.01, stale_after_s=.1, max_block_skew_s=.01)["missing_blocks"], ["imu", "deck_power"])
        self.assertCode("invalid_new_session_id", lambda: p.reset_session("declared-boot-B"))

    def test_reset_near_wrap_cannot_be_detected_from_stamp_only(self):
        p = parser();p.ingest_callback("imu", imu_values(), TIMESTAMP_MODULUS_MS - 100, 1.0)
        r = p.ingest_callback("imu", imu_values(), 50, 1.15)
        self.assertEqual(r["timestamp_unwrap_event"], "forward_wrap_assuming_same_boot")
        self.assertIn("NOT_INFERABLE", r["boot_reset_detection"])
        self.assertFalse(r["physical_motion_ready"])

    def test_half_wrap_and_long_silence_ambiguities(self):
        p = parser();p.ingest_callback("imu", imu_values(), 0, 1.0)
        self.assertCode("timestamp_half_wrap_ambiguous", lambda: p.ingest_callback("imu", imu_values(), TIMESTAMP_HALF_MS, 1.01))
        self.assertCode("wrap_count_ambiguous", lambda: p.ingest_callback("imu", imu_values(), 10, 1.0 + TIMESTAMP_HALF_MS / 1000))

    def test_forward_gap_and_host_gap_are_policy_rejections(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("logger_forward_gap_exceeds_policy", lambda: p.ingest_callback("imu", imu_values(), 1101, 1.1))
        self.assertCode("host_gap_exceeds_policy", lambda: p.ingest_callback("imu", imu_values(), 110, 11.001))

    def test_first_other_block_too_old_rejected(self):
        p = parser();p.ingest_callback("imu", imu_values(), 1000, 1.0)
        self.assertCode("first_block_timestamp_too_old_or_reset", lambda: p.ingest_callback("deck_power", deck_values(), 900, 1.01))

    def test_host_clock_backjump_separate_from_device_clock(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("host_clock_backjump", lambda: p.ingest_callback("imu", imu_values(), 110, .99))

    def test_stamp_and_host_types(self):
        for stamp in (True, -1, TIMESTAMP_MODULUS_MS, 1.0, "100"):
            self.assertCode("invalid_logger_timestamp", lambda: parser().ingest_callback("imu", imu_values(), stamp, 1.0))
        for host in (True, -1, float("nan"), float("inf"), "1", 10 ** 1000):
            self.assertCode("invalid_host_time", lambda: parser().ingest_callback("imu", imu_values(), 100, host))

    def test_bad_payload_does_not_poison_clock(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("invalid_payload_length", lambda: p.ingest_payload("imu", b"", 110, 2.0))
        r = p.ingest_payload("imu", pack("imu", imu_values()), 110, 1.01)
        self.assertEqual(r["logger_snapshot_interval_s"], .01)

    def test_stale_receipt_age_separate_from_capture_age(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        p.ingest_callback("deck_power", deck_values(), 100, 1.005)
        view = p.snapshot_view(1.2, stale_after_s=.05, max_block_skew_s=.01)
        self.assertEqual(view["status"], "stale_block")
        self.assertEqual(set(view["stale_blocks"]), {"imu", "deck_power"})
        self.assertIn("not measurement age", view["age_semantics"])

    def test_skew_visible_even_when_both_fresh(self):
        p = parser();p.ingest_callback("imu", imu_values(), 100, 1.0)
        p.ingest_callback("deck_power", deck_values(), 130, 1.01)
        view = p.snapshot_view(1.02, stale_after_s=.1, max_block_skew_s=.01)
        self.assertEqual(view["status"], "block_logger_skew")
        self.assertEqual(view["logger_block_skew_s"], .03)
        self.assertEqual(set(view["blocks"]), {"imu", "deck_power"})
        self.assertFalse(view["physical_motion_ready"])

    def test_missing_block_and_view_before_receipt(self):
        p = parser();self.assertEqual(p.snapshot_view(0, stale_after_s=.1, max_block_skew_s=.01)["status"], "missing_block")
        p.ingest_callback("imu", imu_values(), 100, 1.0)
        self.assertCode("view_before_last_host_receipt", lambda: p.snapshot_view(.9, stale_after_s=.1, max_block_skew_s=.01))

    def test_invalid_clock_policy_and_session(self):
        for args in ((True, 1000, 10.0), (50, 50, 10.0), (-1, 1000, 10.0), (50, TIMESTAMP_HALF_MS, 10.0)):
            self.assertCode("invalid_clock_policy", lambda: ClockPolicy(*args))
        self.assertCode("invalid_clock_policy", lambda: StockTelemetryParser("A", None))
        self.assertCode("invalid_new_session_id", lambda: StockTelemetryParser("", ClockPolicy(50, 1000, 10.0)))


class OfficialSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log = load_pinned_log()

    def test_exact_logconfigs_sizes_and_native_fetch_types(self):
        configs = make_log_configs(self.log.LogConfig)
        details = {}
        for block, config in configs.items():
            size = sum(self.log.LogTocElement.get_size_from_id(v.fetch_as) for v in config.variables)
            self.assertEqual(size, payload_size(block))
            self.assertLessEqual(size, config.MAX_LEN)
            self.assertEqual(config.MAX_LEN, 26)
            self.assertEqual(config.period, 1)
            self.assertEqual(config.period_in_ms, 10)
            self.assertFalse(config.default_fetch_as)
            self.assertEqual([(v.name, v.fetch_as_string) for v in config.variables],
                             [(f.name, f.fetch_type) for f in BLOCK_FIELDS[block]])
            details[block] = {"payload_bytes": size, "max_payload_bytes": config.MAX_LEN,
                              "log_data_with_id_and_stamp_bytes": size + 4,
                              "requested_period_ms": 10, "wire_period_units_10ms": config.period,
                              "fields": [{"name": v.name, "native_fetch_type": v.fetch_as_string,
                                          "bytes": self.log.LogTocElement.get_size_from_id(v.fetch_as)} for v in config.variables],
                              "live_TOC_checked": False, "radio_started": False}
        EVIDENCE["official_logconfigs"] = details

    def test_actual_cflib_unpack_callback_equals_standalone_decode(self):
        configs = make_log_configs(self.log.LogConfig);fixtures = []
        for block, values in (("imu", imu_values()), ("deck_power", deck_values())):
            received = [];config = configs[block]
            config.data_received_cb.add_callback(lambda ts, data, conf: received.append((ts, data, conf.name)))
            payload = pack(block, values)
            config.unpack_log_data(payload, 123456)
            self.assertEqual(received, [(123456, decode_payload(block, payload), config.name)])
            normalized = parser().ingest_callback(block, received[0][1], received[0][0], 1.0)
            fixtures.append({"block": block, "payload_hex": payload.hex(), "logger_timestamp_ms": 123456,
                             "official_callback_values": received[0][1], "normalized": normalized})
        EVIDENCE["official_decode_fixtures"] = fixtures

    def test_firmware_source_units_types_axes_and_stamp_boundary(self):
        rels = ("src/modules/src/stabilizer.c", "src/drivers/src/motors.c",
                "src/deck/drivers/src/flowdeck_v1v2.c", "src/drivers/interface/pmw3901.h",
                "src/modules/src/range.c", "src/hal/src/pm_stm32f4.c",
                "src/modules/src/log.c", "src/modules/interface/crtp.h",
                "src/hal/src/sensors_bmi088_bmp3xx.c")
        evidence = [pinned_file(FIRMWARE, FIRMWARE_COMMIT, p) for p in rels]
        text = {p: (FIRMWARE / p).read_text() for p in rels}
        self.assertIn("int16_t accpx = -currentMotion.deltaY", text[rels[2]])
        self.assertIn("int16_t accpy = -currentMotion.deltaX", text[rels[2]])
        self.assertIn("flowData.dt", text[rels[2]])
        self.assertIn("LOG_UINT16, zrange", text[rels[4]])
        self.assertIn("LOG_FLOAT, vbat", text[rels[5]])
        self.assertIn("LOG_MAX_LEN 26", text[rels[6]])
        self.assertIn("CRTP_MAX_DATA_SIZE 30", text[rels[7]])
        EVIDENCE["firmware_sources"] = evidence


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--out", type=Path, help="new evidence directory; created exclusively")
    options = args.parse_args()
    stream = io.StringIO()
    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c)
                              for c in (ContractTests, OfficialSourceTests)])
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    output = stream.getvalue();sys.stdout.write(output)
    if options.out:
        out = options.out.resolve()
        allowed = ROOT / "artifacts/mission3/stock-telemetry"
        if allowed not in out.parents:
            raise ValueError("output must be a NEW stock-telemetry artifact subdirectory")
        out.mkdir(parents=True, exist_ok=False)
        (out / "test-output.txt").write_text(output)
        report = {"schema": "mission3-stock-telemetry-test/1", "tests_run": result.testsRun,
                  "success": result.wasSuccessful(), "failures": [(str(t), e) for t, e in result.failures],
                  "errors": [(str(t), e) for t, e in result.errors], "python": platform.python_version(),
                  "implementation_sha256": sha(ROOT / "scripts/mission3_stock_telemetry.py"),
                  "test_source_sha256": sha(Path(__file__)), "radio_used": False,
                  "mission_runtime_modified": False, "hardware_performance_verified": False,
                  "physical_motion_ready": False, "evidence": EVIDENCE}
        (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
