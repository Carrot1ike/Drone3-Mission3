#!/usr/bin/env python3
"""미션3 센서/영상 ROS2 경계. 시뮬레이션 명령만 발행한다.

기체/차량 참값, 원시 deck 변환, 접촉 판정, 하드웨어 명령은 이 모듈의
입력/역할이 아니다. 이미 센서로 변환한 heading-frame 패킷만 허용한다.
"""
import argparse
from collections import Counter, deque
from copy import deepcopy
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import sys

import numpy as np

from provided_hardware import decode_mono320, validate_camera_config
from yaw_core import rotation_rpy
from mission3.controller import Mission3
from mission3.estimator import SelfMotion
from mission3.scene import R_HEADING_CAMERA
from mission3.settings import ROOT, controller_settings
from mission3.vision import Observer


PACKET_SCHEMA = "mission3.sensor.v1"
PACKET_FIELDS = frozenset(("schema", "stamp_s", "gyro_z_rad_s", "accel_heading_m_s2",
    "range_m", "range_valid", "range_accurate", "range_new", "range_stamp_s",
    "flow_angular_rad_s", "flow_valid", "attitude"))
ATTITUDE_FIELDS = frozenset(("stamp_s", "roll_rad", "pitch_rad", "source"))
TOPIC_PREFIX = "/drone3/mission3"
CAMERA_FRAME = "mission3_camera_optical"
WORLD_FRAME = "world_mission"


class InputRejected(ValueError):
    """온라인 계약 위반. 추정기/제어기에 전달하지 않는다."""


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise InputRejected(f"{name}: finite number required")
    return float(value)


def vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise InputRejected(f"{name}: {length} numbers required")
    return [number(v, name) for v in value]


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise InputRejected(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    def constant(value):
        raise InputRejected(f"nonfinite JSON constant: {value}")
    try:
        return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)
    except (ValueError, TypeError) as exc:
        raise InputRejected(f"invalid JSON: {exc}") from exc


def clean_json(value):
    """추정기의 미확인 range NaN도 유효한 JSON null로 기록한다."""
    if isinstance(value, dict):
        return {k: clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.generic):
        return value.item()
    return value


def dumps(value):
    return json.dumps(clean_json(value), ensure_ascii=False, allow_nan=False)


@dataclass(frozen=True)
class TimingPolicy:
    # 전송 시험용 소프트웨어 한계. 제조사 보장/확률 수치가 아니다.
    packet_max_age_s: float = .12
    image_max_age_s: float = .35
    range_max_age_s: float = .10
    attitude_max_age_s: float = .03
    future_tolerance_s: float = .002
    sensor_max_gap_s: float = .20


def load_public_config(path=ROOT / "config/mission3.json"):
    public = controller_settings(json.loads(Path(path).read_text()))
    validate_camera_config(public["camera"])
    if public["entry"]["height_m"] != 1.0:
        raise ValueError("gateway handoff requires the known 1m entry height")
    return public


def load_mount_s(path=ROOT / "config/mission3-physical-sensitivity.json"):
    # 같은 파일의 motor/vehicle/sensor plant 설정은 전달하지 않는다.
    mounts = json.loads(Path(path).read_text())["mounts"]
    if mounts["source_class"] != "sensitivity_assumption" or mounts["actual_manufacturer_transform"] is not None:
        raise ValueError("this adapter declares profile-S mounts, not a calibrated hardware transform")
    selected = {k: deepcopy(mounts[k]) for k in ("camera_position_body_m", "camera_rpy_offset_deg",
                "flow_position_body_m", "imu_position_body_m", "source_class", "actual_manufacturer_transform")}
    for k in ("camera_position_body_m", "camera_rpy_offset_deg", "flow_position_body_m", "imu_position_body_m"):
        vector(selected[k], 3, k)
    return selected


class PacketMotion(SelfMotion):
    """센서 유래 roll/pitch 촬영 이력 + 현재 SelfMotion. 참 자세 입력 없음."""
    def __init__(self, cfg, entry, mounts):
        super().__init__(cfg, entry)
        self.camera_position_body = np.asarray(mounts["camera_position_body_m"], float)
        self.camera_rotation = rotation_rpy(*np.deg2rad(mounts["camera_rpy_offset_deg"])) @ R_HEADING_CAMERA
        self.attitude_history = deque(maxlen=500)

    def attitude_at(self, stamp):
        if not self.attitude_history or stamp < self.attitude_history[0][0]-1e-6 or stamp > self.attitude_history[-1][0]+.025:
            return None
        times = [r[0] for r in self.attitude_history]
        return tuple(float(np.interp(stamp, times, np.unwrap([r[i] for r in self.attitude_history]))) for i in (1, 2))


class GatewayCore:
    """ROS를 제외한 입력 검증/시각 변환/추론 경계. now는 항상 ROS clock."""
    def __init__(self, cfg, entry_id=1, mounts=None, policy=None):
        if set(cfg) != {"camera", "arena", "track", "marker", "landing_geometry", "entry", "observer", "control"}:
            raise ValueError("only the public controller configuration is permitted")
        validate_camera_config(cfg["camera"])
        if cfg["entry"]["height_m"] != 1.0:
            raise ValueError("known entry handoff height must be 1m")
        self.cfg = deepcopy(cfg)
        self.mounts = deepcopy(mounts if mounts is not None else load_mount_s())
        entry = next(e for e in cfg["entry"]["positions"] if e["id"] == entry_id)
        self.motion = PacketMotion(self.cfg, entry, self.mounts)
        self.observer = Observer(self.cfg, self.motion)
        self.mission = Mission3(self.cfg, entry)
        self.policy = policy or TimingPolicy(image_max_age_s=min(.35, cfg["observer"]["image_max_age_s"]))
        self.origin = None
        self.last_clock = None
        self.last_packet = None
        self.last_packet_receipt = None
        self.last_image_stamp = None
        self.last_range = None
        self.last_attitude = None
        self.last_tick = None
        self.own = None
        self.last_image_result = None
        self.fault = None
        self.counts = Counter()
        self.rejects = Counter()
        self.last_rejection = None

    def clock(self, now):
        now = number(now, "receipt/command clock")
        if now < 0:
            raise InputRejected("negative ROS clock")
        if self.last_clock is not None and now < self.last_clock-1e-6:
            self.fault = "clock_regressed_restart_required"
        self.last_clock = now if self.last_clock is None else max(self.last_clock, now)
        if self.fault:
            raise InputRejected(self.fault)
        return now

    def freshness(self, stamp, receipt, maximum, name):
        if stamp < 0 or stamp > receipt+self.policy.future_tolerance_s:
            raise InputRejected(f"{name}: future/negative capture clock")
        if receipt-stamp > maximum+1e-6:
            raise InputRejected(f"{name}: stale capture clock")

    def reject(self, stream, exc):
        reason = str(exc)
        self.counts[f"{stream}_rejected"] += 1
        self.rejects[f"{stream}: {reason}"] += 1
        self.last_rejection = {"stream": stream, "reason": reason}

    def sensor(self, raw, receipt):
        receipt = self.clock(receipt)
        packet = strict_json(raw) if isinstance(raw, str) else deepcopy(raw)
        if not isinstance(packet, dict) or set(packet) != PACKET_FIELDS:
            keys = sorted(set(packet)-PACKET_FIELDS) if isinstance(packet, dict) else []
            raise InputRejected(f"sensor packet field allowlist: unknown={keys}; all fields required")
        if packet["schema"] != PACKET_SCHEMA:
            raise InputRejected("sensor packet schema")
        stamp = number(packet["stamp_s"], "stamp_s")
        self.freshness(stamp, receipt, self.policy.packet_max_age_s, "sensor")
        if self.last_packet is not None:
            if stamp <= self.last_packet["stamp_s"]:
                raise InputRejected("sensor capture timestamp order")
            if stamp-self.last_packet["stamp_s"] > self.policy.sensor_max_gap_s+1e-6:
                self.fault = "sensor_gap_restart_required"
                raise InputRejected(self.fault)
        packet["gyro_z_rad_s"] = number(packet["gyro_z_rad_s"], "gyro_z_rad_s")
        packet["accel_heading_m_s2"] = vector(packet["accel_heading_m_s2"], 3, "accel_heading_m_s2")
        packet["flow_angular_rad_s"] = vector(packet["flow_angular_rad_s"], 2, "flow_angular_rad_s")
        for key in ("range_valid", "range_accurate", "range_new", "flow_valid"):
            if type(packet[key]) is not bool:
                raise InputRejected(f"{key}: bool required")
        r = packet["range_m"]
        if r is not None and number(r, "range_m") <= 0:
            raise InputRejected("range_m must be positive or null")
        if packet["range_valid"] and r is None or packet["range_accurate"] and not packet["range_valid"]:
            raise InputRejected("range validity/accuracy contradiction")
        rs = packet["range_stamp_s"]
        if rs is None:
            if packet["range_new"] or packet["range_valid"] or r is not None or self.last_range is not None:
                raise InputRejected("missing range capture stamp")
        else:
            rs = number(rs, "range_stamp_s")
            self.freshness(rs, stamp, self.policy.range_max_age_s, "range")
            range_record = (rs, r, packet["range_valid"], packet["range_accurate"])
            if packet["range_new"]:
                if self.last_range is not None and rs <= self.last_range[0]:
                    raise InputRejected("range_new requires a strictly newer range capture stamp")
            elif self.last_range is None or range_record != self.last_range:
                raise InputRejected("held range must preserve its capture stamp/value/flags")
        attitude = packet["attitude"]
        if not isinstance(attitude, dict) or set(attitude) != ATTITUDE_FIELDS or attitude["source"] != "sensor_imu":
            raise InputRejected("attitude requires sensor_imu roll/pitch capture metadata only")
        ats = number(attitude["stamp_s"], "attitude.stamp_s")
        self.freshness(ats, stamp, self.policy.attitude_max_age_s, "attitude")
        rp = tuple(number(attitude[k], f"attitude.{k}") for k in ("roll_rad", "pitch_rad"))
        if any(abs(v) > math.pi for v in rp):
            raise InputRejected("attitude radians outside [-pi, pi]")
        attitude_record = (ats, *rp)
        if self.last_attitude is not None and (ats < self.last_attitude[0] or ats == self.last_attitude[0] and attitude_record != self.last_attitude):
            raise InputRejected("attitude capture timestamp order/held value")
        # 모든 항목을 검증한 뒤에만 추정기 상태를 변경한다.
        if self.origin is None:
            self.origin = stamp
        sample = {k: v for k, v in packet.items() if k not in ("schema", "attitude")}
        sample["stamp_s"] = stamp-self.origin
        if rs is not None:
            sample["range_stamp_s"] = rs-self.origin
        if self.last_attitude != attitude_record:
            self.motion.attitude_history.append((ats-self.origin, *rp))
        try:
            own = self.motion.update(sample)
            for key in ("position_world_m", "velocity_world_m_s"):
                vector(own[key], 3, key)
            number(own["heading_rad"], "estimated heading")
        except Exception as exc:
            self.fault = "inference_error_restart_required"
            raise InputRejected(f"{self.fault}: {type(exc).__name__}: {exc}") from exc
        own.update(sensor_scope="sensor-derived heading packet + capture IMU attitude; profile-S mount",
                   range_accurate=packet["range_accurate"], flow_valid=packet["flow_valid"])
        self.own = own
        self.last_packet, self.last_packet_receipt = packet, receipt
        if rs is not None:
            self.last_range = range_record
        self.last_attitude = attitude_record
        self.counts["sensor_accepted"] += 1
        return own

    def image(self, width, height, encoding, step, data, capture, receipt, frame_id=CAMERA_FRAME, is_bigendian=0):
        receipt = self.clock(receipt)
        capture = number(capture, "image capture stamp")
        self.freshness(capture, receipt, self.policy.image_max_age_s, "image")
        if frame_id != CAMERA_FRAME or is_bigendian not in (0, 1):
            raise InputRejected("image frame/endianness contract")
        gray = decode_mono320(width, height, encoding, step, data)
        if self.origin is None:
            raise InputRejected("image requires known-entry sensor handoff first")
        if self.last_image_stamp is not None and capture <= self.last_image_stamp:
            raise InputRejected("image capture timestamp order")
        elapsed = capture-self.origin
        if self.motion.at(elapsed) is None or self.motion.attitude_at(elapsed) is None:
            raise InputRejected("image capture odometry/attitude history missing")
        try:
            result = self.observer.frame(gray, elapsed, receipt-self.origin)
        except Exception as exc:
            self.fault = "image_inference_error_restart_required"
            raise InputRejected(f"{self.fault}: {type(exc).__name__}: {exc}") from exc
        self.last_image_stamp = capture
        self.last_image_result = result
        self.counts["image_accepted"] += 1
        self.counts["image_metric_accepted"] += int(result.get("valid", False))
        self.counts["image_coarse_available"] += int(result.get("coarse_guidance_available", False))
        return result, gray

    def tick(self, now):
        try:
            now = self.clock(now)
        except InputRejected:
            return self.output(now, "FAULT")
        if self.origin is None:
            return self.output(now, "WAIT_INPUT")
        if self.last_tick is not None and now <= self.last_tick:
            raise InputRejected("command clock must advance; paused simulation publishes no new command")
        self.last_tick = now
        if now-self.last_packet["stamp_s"] > self.policy.packet_max_age_s or now-self.last_packet_receipt > self.policy.packet_max_age_s:
            return self.output(now, "SENSOR_STALE")
        vehicle = self.observer.state(now-self.origin)
        try:
            command = self.mission.update(now-self.origin, self.own, vehicle)
            vector(command.velocity_world_m_s, 3, "command velocity")
            number(command.yaw_rate_rad_s, "command yaw rate")
            if command.motors_off or command.physical_success:
                raise ValueError("geometry adapter cannot authorize physical contact/motor-off")
        except Exception as exc:
            self.fault = f"command_inference_error_restart_required: {type(exc).__name__}: {exc}"
            return self.output(now, "FAULT")
        self.motion.terminal_hint = vehicle if command.phase in ("BLIND_APPROACH", "BLIND_DESCEND") else None
        return self.output(now, "ACTIVE", command)

    def output(self, now, transport, command=None):
        velocity = [0., 0., 0.] if command is None else command.velocity_world_m_s
        phase = self.mission.phase if command is not None else "TRANSPORT_HOLD"
        status = {"stamp_s": now, "mission_elapsed_s": None if self.origin is None else now-self.origin,
                  "transport": transport, "phase": phase, "core_phase": self.mission.phase,
                  "failure": self.fault or (command.failure if command else None),
                  "geometry_ready": bool(command and command.phase == "GEOMETRY_READY"),
                  "physical_success": False, "deployment_complete": False, "motors_off": False,
                  "scope": "simulation transport/inference boundary only; no contact observer or hardware commands",
                  "counts": dict(self.counts), "last_rejection": self.last_rejection,
                  "last_image_result": self.last_image_result, "self_estimate": self.own}
        return {"stamp_s": now, "frame_id": WORLD_FRAME, "phase": phase,
                "velocity_world_m_s": velocity,
                "yaw_rate_rad_s": 0. if command is None else command.yaw_rate_rad_s,
                "motors_off": False, "status": clean_json(status)}


class ArtifactLog:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.file = (self.directory / "transport.jsonl").open("a", buffering=1)

    def write(self, event, **fields):
        self.file.write(dumps({"event": event, **fields})+"\n")

    def close(self):
        self.file.close()


def snapshot(directory, cfg, mounts):
    directory = Path(directory)
    if (directory / "source-manifest.json").exists():
        raise ValueError("artifact snapshot already exists; use a fresh output directory")
    directory.mkdir(parents=True, exist_ok=True)
    files = list((ROOT / "scripts/mission3").glob("*.py")) + [ROOT / "scripts" / n for n in
        ("mission3_ros_gateway.py", "test_mission3_ros_gateway.py", "run_mission3_ros_gateway.sh",
         "provided_hardware.py", "yaw_core.py", "aruco_pose.py")]
    import cv2
    manifest = {"created_kst": datetime.now().astimezone().isoformat(), "source_hashes": {},
                "runtime": {"python_executable": sys.executable, "python_version": platform.python_version(),
                            "numpy_version": np.__version__, "opencv_version": cv2.__version__,
                            "ROS_DISTRO": os.environ.get("ROS_DISTRO"), "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
                            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION")},
                "scope": "sensor/image-only transport; no truth files read or plant object constructed"}
    for path in files:
        if not path.is_file():
            continue
        rel = path.relative_to(ROOT)
        target = directory / "source" / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        manifest["source_hashes"][str(rel)] = hashlib.sha256(target.read_bytes()).hexdigest()
    (directory / "public-config.json").write_text(dumps(cfg)+"\n")
    (directory / "mount-profile-S.json").write_text(dumps(mounts)+"\n")
    manifest["public_config_sha256"] = hashlib.sha256((directory / "public-config.json").read_bytes()).hexdigest()
    manifest["mount_profile_S_sha256"] = hashlib.sha256((directory / "mount-profile-S.json").read_bytes()).hexdigest()
    (directory / "source-manifest.json").write_text(dumps(manifest)+"\n")
    return manifest


def make_ros_node(core, artifact_log=None, topic_prefix=TOPIC_PREFIX, record_images=False, sim_clock_topic=None):
    # ROS imports stay here: pure boundary tests also run without sourcing ROS.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool, String
    if topic_prefix != TOPIC_PREFIX and not topic_prefix.startswith(TOPIC_PREFIX+"/test_"):
        raise ValueError("gateway outputs must stay under /drone3/mission3 (test subnamespace allowed)")
    if sim_clock_topic is not None and (topic_prefix == TOPIC_PREFIX or sim_clock_topic != topic_prefix+"/clock"):
        raise ValueError("private simulation clock is only allowed inside the test subnamespace")

    class GatewayNode(Node):
        def __init__(self):
            clock_args = [] if sim_clock_topic is None else ["--ros-args", "--remap", "/clock:="+sim_clock_topic]
            super().__init__("mission3_ros_gateway", cli_args=clock_args)
            # 절대 토픽도 ROS CLI remap 대상이다. publisher를 만들기 전에 차단한다.
            for suffix in ("cmd_velocity", "phase", "status", "motors_off"):
                expected = topic_prefix+"/"+suffix
                if self.resolve_topic_name(expected) != expected:
                    self.destroy_node()
                    raise ValueError("gateway output topic remapping is forbidden: "+expected)
            self.core = core
            self.log = artifact_log
            command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self.cmd = self.create_publisher(TwistStamped, topic_prefix+"/cmd_velocity", command_qos)
            self.phase = self.create_publisher(String, topic_prefix+"/phase", command_qos)
            self.status = self.create_publisher(String, topic_prefix+"/status", command_qos)
            self.motor = self.create_publisher(Bool, topic_prefix+"/motors_off", command_qos)
            self.sensor_sub = self.create_subscription(String, topic_prefix+"/sensor_packet", self.sensor_callback, qos_profile_sensor_data)
            self.image_sub = self.create_subscription(Image, topic_prefix+"/image", self.image_callback, qos_profile_sensor_data)
            self.timer = self.create_timer(1/50., self.command_callback)
            self.published = 0

        def now_s(self):
            return self.get_clock().now().nanoseconds / 1e9

        def rejected(self, stream, exc, receipt):
            core.reject(stream, exc)
            if self.log:
                self.log.write("rejected", stream=stream, receipt_s=receipt, reason=str(exc))

        def sensor_callback(self, message):
            receipt = self.now_s()
            try:
                core.sensor(message.data, receipt)
                if self.log:
                    self.log.write("sensor", receipt_s=receipt, packet=core.last_packet)
            except (InputRejected, ValueError, TypeError) as exc:
                self.rejected("sensor", exc, receipt)

        def image_callback(self, message):
            receipt = self.now_s()
            capture = message.header.stamp.sec + message.header.stamp.nanosec/1e9
            try:
                result, gray = core.image(message.width, message.height, message.encoding, message.step,
                    message.data, capture, receipt, message.header.frame_id, message.is_bigendian)
                if self.log:
                    image_name = None
                    if record_images:
                        import cv2
                        image_name = f"frame-{core.counts['image_accepted']:06d}.png"
                        cv2.imwrite(str(self.log.directory / image_name), gray)
                    self.log.write("image", capture_s=capture, receipt_s=receipt, width=320, height=320,
                                   encoding="mono8", pixels_sha256=hashlib.sha256(gray.tobytes()).hexdigest(),
                                   file=image_name, result=result)
            except (InputRejected, ValueError, TypeError) as exc:
                self.rejected("image", exc, receipt)

        def command_callback(self):
            now = self.now_s()
            try:
                output = core.tick(now)
            except InputRejected as exc:
                self.rejected("command", exc, now)
                return
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = WORLD_FRAME
            cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = output["velocity_world_m_s"]
            cmd.twist.angular.z = output["yaw_rate_rad_s"]
            self.cmd.publish(cmd)
            self.phase.publish(String(data=output["phase"]))
            self.status.publish(String(data=dumps(output["status"])))
            self.motor.publish(Bool(data=False))
            self.published += 1
            if self.log:
                self.log.write("command", **output)

    return GatewayNode()


def main():
    import rclpy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--entry", type=int, default=1, choices=range(1, 9))
    parser.add_argument("--config", type=Path, default=ROOT / "config/mission3.json")
    parser.add_argument("--mount-profile", type=Path, default=ROOT / "config/mission3-physical-sensitivity.json")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--record-images", action="store_true")
    args, ros_args = parser.parse_known_args()
    cfg, mounts = load_public_config(args.config), load_mount_s(args.mount_profile)
    out = args.out or ROOT / "artifacts/mission3/ros-gateway" / datetime.now().strftime("%Y%m%d-%H%M%S-%f-node")
    snapshot(out, cfg, mounts)
    log = ArtifactLog(out)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = make_ros_node(GatewayCore(cfg, args.entry, mounts), log, record_images=args.record_images)
        print(f"시뮬레이션 전용 게이트웨이: {TOPIC_PREFIX}, 산출물 {out}", flush=True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        log.write("startup_or_runtime_error", reason=f"{type(exc).__name__}: {exc}")
        print(f"게이트웨이 거부: {exc}", file=sys.stderr, flush=True)
        return 2
    finally:
        if node is not None:
            log.write("shutdown", counts=dict(node.core.counts), rejection_reasons=dict(node.core.rejects), fault=node.core.fault)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
