#!/usr/bin/env python3
"""로컬 CFLib 실제 packing + 공식 RPYT decoder 추출 함수의 오프라인 시험."""
import argparse
import ctypes as ct
from datetime import datetime
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import types
import unittest
from unittest.mock import patch

from mission3_rpyt_protocol import (FLOAT32_MAX, LEGACY_THRUST_CAP_S, RPYTContractError,
    decode_firmware_rpyt, encode_firmware_rpyt, legacy_thrust_count, radians_to_firmware_degrees)


ROOT = Path(__file__).resolve().parents[1]
CFLIB = ROOT / "src/crazyflie-lib-python"
FIRMWARE = ROOT / "src/bitcraze-mission3-firmware"
# unittest discovery에서도 C probe 산출물을 이 sidecar 안에 둔다.
OUT = ROOT / "artifacts/mission3/rpyt-protocol" / datetime.now().strftime("%Y%m%d-%H%M%S-%f-unittest")
PACKET_RECORDS = []


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def json_write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def load_local_cflib():
    """정확한 세 vendor 파일 실행. 패키지 __init__/driver를 로드하지 않는다."""
    names = ("cflib", "cflib.crtp", "cflib.crazyflie", "cflib.utils")
    modules = {}
    for name in names:
        module = types.ModuleType(name)
        module.__path__ = [str(CFLIB / Path(*name.split(".")))]
        modules[name] = module
    files = {"cflib.crtp.crtpstack": CFLIB / "cflib/crtp/crtpstack.py",
             "cflib.utils.encoding": CFLIB / "cflib/utils/encoding.py",
             "cflib.crazyflie.commander": CFLIB / "cflib/crazyflie/commander.py"}
    with patch.dict(sys.modules, modules):
        loaded = {}
        for name, path in files.items():
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[name] = module
            loaded[name] = module
            spec.loader.exec_module(module)
        Commander = loaded["cflib.crazyflie.commander"].Commander
        CRTPPort = loaded["cflib.crtp.crtpstack"].CRTPPort
        forbidden = [name for name in sys.modules if name.startswith("cflib") and
                     any(part in name for part in ("radiodriver", "usbdriver", "crazyradio", "link_cpp"))]
        if forbidden:
            raise RuntimeError("offline test imported a hardware driver: "+str(forbidden))
        return Commander, CRTPPort


class MockPacketSink:
    """send_packet 저장 외의 메서드/연결/라디오가 없는 객체."""
    def __init__(self):
        self.packets = []

    def send_packet(self, packet):
        self.packets.append({"port": packet.port, "channel": packet.channel,
                             "header": packet.get_header(), "payload": bytes(packet.data)})


class ProtocolTests(unittest.TestCase):
    def test_payload_is_14_bytes_little_endian_and_firmware_signed(self):
        payload = encode_firmware_rpyt(7.25, -12.5, 31.75, 10001)
        self.assertEqual(payload, struct.pack("<fffH", 7.25, -12.5, -31.75, 10001))
        self.assertEqual(len(payload), 14)
        self.assertEqual(decode_firmware_rpyt(payload),
                         {"roll_deg": 7.25, "pitch_deg": -12.5, "yaw_rate_deg_s": 31.75, "thrust": 10001})

    def test_all_finite_inputs_and_stop_are_strict(self):
        for value in (float("nan"), float("inf"), -float("inf"), True, "1", None, 10**1000):
            for index in range(4):
                args = [1., 2., 3., 10001.]
                args[index] = value
                with self.subTest(index=index, value=type(value).__name__), self.assertRaises(RPYTContractError):
                    encode_firmware_rpyt(*args)
        for stop in (0, 1, "true", None):
            with self.assertRaises(RPYTContractError):
                encode_firmware_rpyt(0., 0., 0., 0., stop=stop)

    def test_float32_capacity_is_separate_from_flight_envelope(self):
        for index in range(3):
            args = [0., 0., 0., 10001]
            args[index] = FLOAT32_MAX
            self.assertTrue(math.isfinite(list(decode_firmware_rpyt(encode_firmware_rpyt(*args)).values())[index]))
            for sign in (-1, 1):
                args[index] = sign*FLOAT32_MAX*2
                with self.assertRaises(RPYTContractError):
                    encode_firmware_rpyt(*args)

    def test_uint16_bounds_rejected_before_cap_even_with_stop(self):
        for thrust in (-.001, -1, 65535.001, 65536):
            for stop in (False, True):
                with self.subTest(thrust=thrust, stop=stop), self.assertRaises(RPYTContractError):
                    encode_firmware_rpyt(0., 0., 0., thrust, stop)

    def test_60000S_cap_and_fractional_legacy_count_floor(self):
        for value, expected in ((0., 0), (.99, 0), (10001.99, 10001), (45000.75, 45000),
                                (60000., 60000), (60000.9, 60000), (65535., 60000)):
            self.assertEqual(legacy_thrust_count(value), expected)
            self.assertEqual(decode_firmware_rpyt(encode_firmware_rpyt(0., 0., 0., value))["thrust"], expected)
        self.assertEqual(LEGACY_THRUST_CAP_S, 60000)

    def test_decoder_preserves_actual_uint16_wire_above_software_cap(self):
        self.assertEqual(decode_firmware_rpyt(struct.pack("<fffH", 1., 2., -3., 65535))["thrust"], 65535)

    def test_float32_quantization_is_explicit_and_not_double_flipped(self):
        desired = (1.1, -2.2, 3.3)
        result = decode_firmware_rpyt(encode_firmware_rpyt(*desired, 30000))
        for key, value in zip(("roll_deg", "pitch_deg", "yaw_rate_deg_s"), desired):
            self.assertEqual(result[key], f32(value))
        self.assertLess(result["pitch_deg"], 0.)
        self.assertGreater(result["yaw_rate_deg_s"], 0.)

    def test_signed_zero_roundtrip_without_stop(self):
        for r, p, y in itertools.product((0., -0.), repeat=3):
            result = decode_firmware_rpyt(encode_firmware_rpyt(r, p, y, 0))
            for key, value in zip(("roll_deg", "pitch_deg", "yaw_rate_deg_s"), (r, p, y)):
                self.assertEqual(struct.pack("<f", result[key]), struct.pack("<f", value))

    def test_motoroff_zero_is_canonical_payload_not_contact_evidence(self):
        payload = encode_firmware_rpyt(20., -20., 100., 60000, stop=True)
        self.assertEqual(payload, bytes(14))
        result = decode_firmware_rpyt(payload)
        self.assertEqual(set(result), {"roll_deg", "pitch_deg", "yaw_rate_deg_s", "thrust"})
        self.assertTrue(all(value == 0 for value in result.values()))
        with self.assertRaises(RPYTContractError):
            encode_firmware_rpyt(float("nan"), 0., 0., 0., stop=True)

    def test_radian_conversion_is_once_and_requires_CF_frame(self):
        degrees = radians_to_firmware_degrees(math.radians(10), math.radians(-5), math.radians(20), frame="cf_firmware")
        for value, expected in zip(degrees, (10., -5., 20.)):
            self.assertAlmostEqual(value, expected)
        result = decode_firmware_rpyt(encode_firmware_rpyt(*degrees, 30000))
        self.assertEqual(result["roll_deg"], 10.)
        self.assertEqual(result["pitch_deg"], -5.)
        self.assertEqual(result["yaw_rate_deg_s"], 20.)
        for frame in ("world_mission", "ros_enu", "body_flu", None):
            with self.assertRaises(RPYTContractError):
                radians_to_firmware_degrees(0., 0., 0., frame=frame)
        with self.assertRaises(RPYTContractError):
            radians_to_firmware_degrees(float("inf"), 0., 0., frame="cf_firmware")

    def test_decoder_rejects_bad_sizes_and_non_buffer_inputs(self):
        for value in (bytes(0), bytes(13), bytes(15), "0"*14, [0]*14, 14, None):
            with self.assertRaises(RPYTContractError):
                decode_firmware_rpyt(value)
        payload = encode_firmware_rpyt(1., 2., 3., 10001)
        self.assertEqual(decode_firmware_rpyt(bytearray(payload)), decode_firmware_rpyt(memoryview(payload)))

    def test_decoder_rejects_nonfinite_wire_float_fields(self):
        for index in range(3):
            for value in (float("nan"), float("inf"), -float("inf")):
                angles = [0., 0., 0.]
                angles[index] = value
                with self.assertRaises(RPYTContractError):
                    decode_firmware_rpyt(struct.pack("<fffH", *angles, 0))


class CflibTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.Commander, cls.CRTPPort = load_local_cflib()

    def send_mock(self, args, xmode=False):
        sink = MockPacketSink()
        commander = self.Commander(sink)
        commander.set_client_xmode(xmode)
        commander.send_setpoint(*args)
        self.assertEqual(len(sink.packets), 1)
        packet = sink.packets[0]
        self.assertEqual(packet["port"], self.CRTPPort.COMMANDER)
        self.assertEqual(packet["port"], 3)
        self.assertEqual(packet["channel"], 0)
        self.assertEqual(packet["header"], 0x3c)
        return packet

    def test_actual_CFLib_bytes_match_for_420_asymmetric_cases(self):
        for r, p, y, thrust in itertools.product((-20., -.125, 0., 3.75, 20.), (-17.25, 0., 7.125),
                                                (-80., 0., .125, 20.), (0., 999., 1000., 10001.9, 45000.5, 60000., 65535.)):
            expected_count = math.floor(min(thrust, 60000))
            packet = self.send_mock((r, -p, -y, expected_count))
            payload = encode_firmware_rpyt(r, p, y, thrust)
            self.assertEqual(packet["payload"], payload)
            PACKET_RECORDS.append({"desired_firmware": [r, p, y, thrust],
                "cflib_arguments": [r, -p, -y, expected_count], "header_hex": "3c", "payload_hex": payload.hex(),
                "decoded": decode_firmware_rpyt(payload)})

    def test_double_pitch_flip_negative_control_changes_firmware_pitch(self):
        desired = (6., 10., 20., 30000)
        correct = self.send_mock((6., -10., -20., 30000))["payload"]
        wrong = self.send_mock(desired)["payload"]
        self.assertNotEqual(correct, wrong)
        self.assertEqual(decode_firmware_rpyt(correct)["pitch_deg"], 10.)
        self.assertEqual(decode_firmware_rpyt(wrong)["pitch_deg"], -10.)
        self.assertEqual(decode_firmware_rpyt(wrong)["yaw_rate_deg_s"], -20.)

    def test_client_xmode_true_negative_control_rotates_CF_angles(self):
        args = (6., -10., -20., 30000)
        correct = self.send_mock(args)["payload"]
        wrong = self.send_mock(args, xmode=True)["payload"]
        self.assertNotEqual(correct, wrong)
        result = decode_firmware_rpyt(wrong)
        self.assertNotEqual(result["roll_deg"], 6.)
        self.assertNotEqual(result["pitch_deg"], 10.)

    def test_stop_bytes_match_actual_CFLib_and_do_not_claim_contact(self):
        packet = self.send_mock((0., -0., 0., 0))
        self.assertEqual(packet["payload"], encode_firmware_rpyt(1., 2., 3., 10001, stop=True))
        self.assertEqual(decode_firmware_rpyt(packet["payload"])["thrust"], 0)


def build_decoder_probe():
    """전체 공식 decoder block을 그대로 추출. C/build/vendor는 변경하지 않는다."""
    source_path = FIRMWARE / "src/modules/src/crtp_commander_rpyt.c"
    header_path = FIRMWARE / "src/modules/interface/commander.h"
    source = source_path.read_text()
    start = source.index("// Thrust below MIN_THRUST")
    end = source.index("\n/**\n * There are 2 levels", start)
    block = source[start:end]
    header = header_path.read_text()
    defines = "\n".join(line for line in header.splitlines() if
                       line.startswith("#define DEFAULT_YAW_MODE ") or line.startswith("#define COMMANDER_PRIORITY_DISABLE "))
    preface = '''#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define M_PI 3.14159265358979323846
#define ASSERT(x) assert(x)
enum {modeDisable=0,modeAbs=1,modeVelocity=2};
typedef struct {float x,y,z;} xyz_t;
typedef struct {float roll,pitch,yaw;} rpy_t;
typedef struct {int x,y,z,roll,pitch,yaw;} probe_modes_t;
typedef struct {float thrust;probe_modes_t mode;xyz_t velocity,position;rpy_t attitude,attitudeRate;} setpoint_t;
typedef struct {uint8_t data[30];} CRTPPacket;
static int probe_priority=0;
static int commanderGetActivePriority(void){return probe_priority;}
static void positionControllerResetAllPID(float x,float y,float z){(void)x;(void)y;(void)z;}
static void positionControllerResetAllfilters(void){}
'''
    suffix = '''
_Static_assert(sizeof(float)==4,"32-bit float required");
_Static_assert(sizeof(struct CommanderCrtpLegacyValues)==14,"legacy payload must be 14 bytes");
void protocol_probe_reset(void){
 stabilizationModeRoll=ANGLE;stabilizationModePitch=ANGLE;stabilizationModeYaw=RATE;
 yawMode=DEFAULT_YAW_MODE;thrustLocked=true;altHoldMode=false;posHoldMode=false;posSetMode=false;modeSet=false;probe_priority=COMMANDER_PRIORITY_DISABLE;
}
void protocol_probe_priority(int priority){probe_priority=priority;}
void protocol_probe_decode(const uint8_t *payload,float *output,int *modes){
 CRTPPacket packet={0};setpoint_t sp={0};memcpy(packet.data,payload,14);
 crtpCommanderRpytDecodeSetpoint(&sp,&packet);
 output[0]=sp.attitude.roll;output[1]=sp.attitude.pitch;output[2]=sp.attitudeRate.yaw;output[3]=sp.thrust;
 modes[0]=sp.mode.roll;modes[1]=sp.mode.pitch;modes[2]=sp.mode.yaw;modes[3]=yawMode;
}
'''
    directory = OUT / "official-decoder-probe"
    directory.mkdir(parents=True, exist_ok=True)
    c_path, so_path = directory / "protocol-probe.c", directory / "protocol-probe.so"
    c_path.write_text(preface+defines+"\n"+block+suffix)
    (directory / "official-extracted-block.c").write_text(block)
    compiler = shutil.which("cc")
    if compiler is None:
        raise RuntimeError("local C compiler missing; official decoder execution unverified")
    command = [compiler, "-std=c11", "-shared", "-fPIC", "-O2", str(c_path), "-lm", "-o", str(so_path)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=30.)
    json_write(directory / "build.json", {"command": command, "returncode": result.returncode,
        "stdout": result.stdout, "stderr": result.stderr,
        "compiler_version": subprocess.check_output([compiler, "--version"], text=True).splitlines()[0],
        "official_source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
        "extracted_block_sha256": hashlib.sha256(block.encode()).hexdigest(),
        "scope": "official decoder unchanged; host-only type/priority stubs; default modes; battery compensated minimum disabled"})
    if result.returncode:
        raise RuntimeError("official decoder probe compile failed: "+result.stderr)
    lib = ct.CDLL(str(so_path))
    lib.protocol_probe_reset.argtypes = []
    lib.protocol_probe_reset.restype = None
    lib.protocol_probe_priority.argtypes = [ct.c_int]
    lib.protocol_probe_priority.restype = None
    lib.protocol_probe_decode.argtypes = [ct.POINTER(ct.c_uint8), ct.POINTER(ct.c_float), ct.POINTER(ct.c_int)]
    lib.protocol_probe_decode.restype = None
    return lib


class FirmwareDecoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if sys.byteorder != "little":
            raise RuntimeError("official pointer-cast probe requires this local little-endian host")
        cls.lib = build_decoder_probe()

    def decode_c(self, payload):
        data = (ct.c_uint8*14).from_buffer_copy(payload)
        output, modes = (ct.c_float*4)(), (ct.c_int*4)()
        self.lib.protocol_probe_decode(data, output, modes)
        return list(output), list(modes)

    def setUp(self):
        self.lib.protocol_probe_reset()

    def unlock_at_CRTP_priority(self):
        self.decode_c(bytes(14))
        self.lib.protocol_probe_priority(2)

    def test_official_XMODE_exact_CF_angles_and_yaw_sign(self):
        self.unlock_at_CRTP_priority()
        records = []
        for desired in ((6., 10., 20.), (-7.25, -12.5, -31.75), (1.1, -2.2, 3.3), (20., -20., 80.), (0., -0., -0.)):
            payload = encode_firmware_rpyt(*desired, 45000.9)
            actual, modes = self.decode_c(payload)
            for value, expected in zip(actual[:3], desired):
                self.assertEqual(struct.pack("<f", value), struct.pack("<f", expected))
            self.assertEqual(actual[3], 45000.)
            self.assertEqual(modes, [1, 1, 2, 2])  # ANGLE/ANGLE/RATE + XMODE.
            records.append({"desired": list(desired), "payload_hex": payload.hex(), "official_output": actual, "modes": modes})
        json_write(OUT / "official-decoder-roundtrips.json", records)

    def test_official_yaw_negative_control_catches_unflipped_CFLib_yaw(self):
        desired = (6., 10., 20.)
        wrong_wire = struct.pack("<fffH", 6., 10., 20., 30000)
        actual, _ = self.decode_c(wrong_wire)
        self.assertEqual(actual[:2], list(desired[:2]))
        self.assertEqual(actual[2], -desired[2])

    def test_official_thrust_lock_and_minimum_are_not_packet_decoder_claims(self):
        actual, _ = self.decode_c(encode_firmware_rpyt(1., 2., 3., 30000))
        self.assertEqual(actual[3], 0.)
        self.decode_c(bytes(14))
        # DISABLE priority 상태를 계속 유지하면 다음 nonzero 패킷은 재잠금된다.
        actual, _ = self.decode_c(encode_firmware_rpyt(1., 2., 3., 30000))
        self.assertEqual(actual[3], 0.)
        self.unlock_at_CRTP_priority()
        for value, expected in ((999, 0), (1000, 1000), (10001, 10001), (65535, 60000)):
            payload = encode_firmware_rpyt(1., 2., 3., value)
            actual, _ = self.decode_c(payload)
            self.assertEqual(actual[3], expected)
        self.assertEqual(decode_firmware_rpyt(encode_firmware_rpyt(1., 2., 3., 999))["thrust"], 999)

    def test_official_stop_zero_has_no_contact_or_landing_branch(self):
        self.unlock_at_CRTP_priority()
        actual, modes = self.decode_c(encode_firmware_rpyt(6., 10., 20., 60000, stop=True))
        self.assertEqual(actual, [0., 0., 0., 0.])
        self.assertEqual(modes, [1, 1, 2, 2])


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    OUT = args.out or ROOT / "artifacts/mission3/rpyt-protocol" / datetime.now().strftime("%Y%m%d-%H%M%S-%f-tests")
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError("output directory is not empty; preserve previous attempts")
    OUT.mkdir(parents=True, exist_ok=True)
    files = [ROOT / "scripts/mission3_rpyt_protocol.py", Path(__file__).resolve(),
        CFLIB / "cflib/crazyflie/commander.py", CFLIB / "cflib/crtp/crtpstack.py", CFLIB / "cflib/utils/encoding.py",
        CFLIB / "docs/user-guides/python_api.md", CFLIB / "docs/api/index.md",
        FIRMWARE / "src/modules/src/crtp_commander_rpyt.c", FIRMWARE / "src/modules/interface/commander.h",
        FIRMWARE / "src/modules/src/controller/position_controller_pid.c",
        FIRMWARE / "src/platform/interface/platform_defaults_cf2.h"]
    hashes = {}
    for path in files:
        rel = path.relative_to(ROOT)
        target = OUT / "source" / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        hashes[str(rel)] = hashlib.sha256(target.read_bytes()).hexdigest()
    manifest = {"created_kst": datetime.now().astimezone().isoformat(), "source_hashes": hashes,
        "python_executable": sys.executable, "python_version": platform.python_version(),
        "cflib_commit": subprocess.check_output(["git", "-C", str(CFLIB), "rev-parse", "HEAD"], text=True).strip(),
        "firmware_commit": subprocess.check_output(["git", "-C", str(FIRMWARE), "rev-parse", "HEAD"], text=True).strip(),
        "radio_initialized": False, "connection_created": False, "hardware_or_ros_commands_sent": False}
    json_write(OUT / "source-manifest.json", manifest)
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(cls) for cls in
                               (ProtocolTests, CflibTests, FirmwareDecoderTests))
    with (OUT / "unittest.log").open("w", buffering=1) as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    with (OUT / "mock-cflib-packets.jsonl").open("w") as stream:
        for record in PACKET_RECORDS:
            stream.write(json.dumps(record, allow_nan=False)+"\n")
    changed = [rel for rel, digest in hashes.items() if hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()!=digest]
    summary = {"tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
        "skipped": len(result.skipped), "passed": result.wasSuccessful(), "mock_cflib_packets": len(PACKET_RECORDS),
        "changed_sources_during_run": changed, "scope": "offline serializer/vendor packet/default-mode decoder only; no deployment/contact proof"}
    json_write(OUT / "test-result.json", summary)
    print(json.dumps(summary, ensure_ascii=False))
    print(f"산출물 {OUT}")
    if not result.wasSuccessful():
        print((OUT / "unittest.log").read_text())
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
