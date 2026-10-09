#!/usr/bin/env python3
"""참값 파일 없는 경계 단위 시험 + 실제 ROS2 센서/영상/명령 왕복 시험."""
import argparse
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
import unittest

import cv2
import numpy as np

from mission3_ros_gateway import (ArtifactLog, CAMERA_FRAME, GatewayCore, InputRejected,
    PACKET_SCHEMA, ROOT, WORLD_FRAME, dumps, load_mount_s, load_public_config,
    make_ros_node, snapshot, strict_json)
from mission3.scene import R_VEHICLE_MARKER
from mission3.track import rotation


ARTIFACT_DIR = None
RUN_ROS = False


def packet(stamp, range_stamp=None, range_new=True):
    return {"schema": PACKET_SCHEMA, "stamp_s": stamp, "gyro_z_rad_s": 0.,
            "accel_heading_m_s2": [0., 0., 0.], "range_m": 1.,
            "range_valid": True, "range_accurate": True, "range_new": range_new,
            "range_stamp_s": stamp if range_stamp is None else range_stamp,
            "flow_angular_rad_s": [0., 0.], "flow_valid": True,
            "attitude": {"stamp_s": stamp, "roll_rad": 0., "pitch_rad": 0., "source": "sensor_imu"}}


def native_marker(core):
    """고정 합성 ArUco 영상만 생성. fixture 좌표를 센서/게이트웨이에 전달하지 않는다."""
    motion, detector = core.motion, core.observer.detector
    yaw = math.radians(-8.)
    pad = motion.position + np.array([.80, .06, -.70])
    marker = pad + rotation(yaw) @ np.asarray(core.cfg["marker"]["known_marker_to_pad_vehicle_m"])
    rcm = motion.camera_rotation.T @ rotation(motion.heading).T @ rotation(yaw) @ R_VEHICLE_MARKER
    tcm = motion.camera_rotation.T @ (rotation(motion.heading).T @ (marker-motion.position)-motion.camera_position_body)
    corners = cv2.projectPoints(detector.points, cv2.Rodrigues(rcm)[0], tcm,
                               detector.K, detector.dist)[0].reshape(4, 2)
    code = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 180)
    transform = cv2.getPerspectiveTransform(np.float32([[0, 0], [179, 0], [179, 179], [0, 179]]), np.float32(corners))
    return cv2.warpPerspective(code, transform, (320, 320), flags=cv2.INTER_NEAREST, borderValue=255)


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_public_config()
        self.core = GatewayCore(self.cfg, mounts=load_mount_s())

    def sensor(self, stamp=100., **changes):
        p = packet(stamp)
        p.update(changes)
        return self.core.sensor(p, stamp+.005)

    def image(self, capture=100., receipt=100.01, **changes):
        args = dict(width=320, height=320, encoding="mono8", step=320,
                    data=np.full((320, 320), 255, np.uint8).tobytes(), capture=capture,
                    receipt=receipt, frame_id=CAMERA_FRAME)
        args.update(changes)
        return self.core.image(**args)

    def test_known_entry_height_forward_and_public_configuration_only(self):
        self.assertNotIn("vehicle_plant_only", self.core.cfg)
        own = self.sensor()
        self.assertEqual(own["position_world_m"], [1., 2., 1.])
        self.assertEqual(own["heading_rad"], 0.)
        bad = deepcopy(self.cfg)
        bad["vehicle_plant_only"] = {"speed_m_s": 99.}
        with self.assertRaises(ValueError):
            GatewayCore(bad)
        for entry_id in range(1, 9):
            core = GatewayCore(self.cfg, entry_id)
            entry = self.cfg["entry"]["positions"][entry_id-1]
            np.testing.assert_allclose(core.motion.position, entry["xy_m"]+[1.])
            self.assertAlmostEqual(core.motion.heading, math.radians(entry["heading_deg"]))

    def test_profile_S_camera_mount_exact(self):
        mounts = load_mount_s()
        np.testing.assert_array_equal(self.core.motion.camera_position_body, mounts["camera_position_body_m"])
        self.assertEqual(mounts["source_class"], "sensitivity_assumption")
        self.assertIsNone(mounts["actual_manufacturer_transform"])
        self.assertEqual(set(self.core.mounts), set(mounts))

    def test_transformed_sensor_updates_self_motion_without_state_input(self):
        self.sensor()
        self.sensor(100.01, gyro_z_rad_s=.2, accel_heading_m_s2=[.3, 0., 0.], flow_valid=False,
                    range_stamp_s=100., range_new=False)
        self.assertAlmostEqual(self.core.motion.heading, .002, places=8)
        self.assertGreater(self.core.motion.velocity[0], 0.)
        self.assertNotIn("vehicle_velocity_world_m_s", self.core.own)

    def test_unknown_live_and_truth_fields_rejected_before_inference(self):
        for key in ("vehicle_plant_only", "truth", "vehicle_velocity_world_m_s", "vehicle_phase",
                    "position_world_m", "heading_rad", "surface_id", "contact", "speed_m_s"):
            with self.subTest(key=key), self.assertRaises(InputRejected):
                self.sensor(**{key: 99.})
        self.assertIsNone(self.core.motion.last_stamp)
        self.assertIsNone(self.core.origin)

    def test_sensor_metadata_truth_and_missing_field_rejected(self):
        p = packet(100.)
        p["attitude"]["source"] = "plant_truth"
        with self.assertRaises(InputRejected):
            self.core.sensor(p, 100.)
        p = packet(100.)
        p["attitude"]["yaw_rad"] = 0.
        with self.assertRaises(InputRejected):
            self.core.sensor(p, 100.)
        p = packet(100.)
        del p["range_new"]
        with self.assertRaises(InputRejected):
            self.core.sensor(p, 100.)

    def test_json_duplicates_nan_and_non_object_rejected(self):
        for raw in ('{"stamp_s":1,"stamp_s":2}', '{"value":NaN}', '{"value":Infinity}', '[]', 'null'):
            with self.subTest(raw=raw), self.assertRaises(InputRejected):
                self.core.sensor(raw, 100.)

    def test_boolean_numerics_dimensions_and_flags_rejected(self):
        for changes in ({"gyro_z_rad_s": True}, {"flow_angular_rad_s": [0.]},
                        {"accel_heading_m_s2": [0., float("inf"), 0.]}, {"range_new": 1},
                        {"flow_valid": "true"}, {"stamp_s": float("nan")}):
            with self.subTest(changes=changes), self.assertRaises(InputRejected):
                self.sensor(**changes)

    def test_capture_clock_future_and_stale_rejected(self):
        for stamp in (99.8, 100.01, -1.):
            with self.subTest(stamp=stamp), self.assertRaises(InputRejected):
                self.core.sensor(packet(stamp), 100.)
        self.assertIsNone(self.core.origin)

    def test_packet_duplicate_order_and_gap_rejected(self):
        self.sensor()
        for stamp in (100., 99.99):
            with self.assertRaises(InputRejected):
                self.core.sensor(packet(stamp), 100.01)
        with self.assertRaisesRegex(InputRejected, "sensor_gap"):
            self.core.sensor(packet(100.25), 100.25)
        self.assertEqual(self.core.tick(100.26)["status"]["transport"], "FAULT")

    def test_range_40Hz_held_sample_does_not_become_new_100Hz(self):
        self.sensor()
        count = len(self.core.motion.range_heights)
        for t in (100.01, 100.02):
            self.sensor(t, range_stamp_s=100., range_new=False)
        self.assertEqual(len(self.core.motion.range_heights), count)
        self.sensor(100.03, range_stamp_s=100.025)
        self.assertEqual(len(self.core.motion.range_heights), count+1)
        with self.assertRaisesRegex(InputRejected, "strictly newer"):
            self.sensor(100.04, range_stamp_s=100.025)
        with self.assertRaisesRegex(InputRejected, "held range"):
            self.sensor(100.04, range_stamp_s=100.03, range_new=False)
        with self.assertRaisesRegex(InputRejected, "held range"):
            self.sensor(100.04, range_stamp_s=100.025, range_new=False, range_m=.9)

    def test_range_freshness_and_accuracy_not_implicitly_promoted(self):
        with self.assertRaisesRegex(InputRejected, "range: stale"):
            self.sensor(range_stamp_s=99.8)
        state = self.sensor(range_accurate=False, flow_valid=False)
        self.assertFalse(state["range_accurate"])
        self.assertFalse(state["flow_fusion_usable"])
        self.assertEqual(state["surface_hypothesis"], "ambiguous")

    def test_missing_range_is_allowed_only_as_explicit_invalid_sample(self):
        own = self.sensor(range_m=None, range_valid=False, range_accurate=False,
                          range_new=False, range_stamp_s=None, flow_valid=False)
        self.assertTrue(np.all(np.isfinite(own["position_world_m"])))
        self.assertIsNone(json.loads(dumps(own))["range_m"])
        with self.assertRaises(InputRejected):
            self.sensor(100.01, range_m=None)

    def test_attitude_freshness_order_and_capture_interpolation(self):
        self.sensor()
        p = packet(100.02)
        p["attitude"].update(roll_rad=.1, pitch_rad=.2)
        self.core.sensor(p, 100.025)
        np.testing.assert_allclose(self.core.motion.attitude_at(.01), [.05, .1], atol=1e-9)
        p = packet(100.03)
        p["attitude"]["stamp_s"] = 99.9
        with self.assertRaisesRegex(InputRejected, "attitude: stale"):
            self.core.sensor(p, 100.035)
        p["attitude"]["stamp_s"] = 100.02
        with self.assertRaisesRegex(InputRejected, "attitude capture"):
            self.core.sensor(p, 100.035)

    def test_camera_324_color_bad_step_and_length_rejected(self):
        self.sensor()
        for changes in ({"width": 324}, {"height": 324}, {"encoding": "rgb8"},
                        {"encoding": "bgr8"}, {"step": 319}, {"data": b"bad"},
                        {"frame_id": "world_truth"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.image(**changes)
        self.assertEqual(self.core.counts["image_accepted"], 0)

    def test_camera_native_row_padding_preserves_pixels(self):
        self.sensor()
        pixels = np.full((320, 328), 255, np.uint8)
        pixels[:, 320:] = 7
        _, gray = self.image(step=328, data=pixels.tobytes())
        self.assertEqual(gray.shape, (320, 320))
        np.testing.assert_array_equal(gray, np.full((320, 320), 255, np.uint8))

    def test_camera_capture_order_future_stale_history_rejected(self):
        with self.assertRaisesRegex(InputRejected, "handoff"):
            self.image(receipt=100.)
        self.sensor()
        self.image()
        for changes in ({"capture": 100., "receipt": 100.02},
                        {"capture": 100.04, "receipt": 100.02},
                        {"capture": 99., "receipt": 100.02},
                        {"capture": 100.04, "receipt": 100.04}):
            with self.subTest(changes=changes), self.assertRaises(InputRejected):
                self.image(**changes)

    def test_capture_attitude_missing_prevents_pose_inference(self):
        self.sensor()
        self.core.motion.attitude_history.clear()
        with self.assertRaisesRegex(InputRejected, "history"):
            self.image()

    def test_native_marker_reaches_current_observer_and_image_triggers_controller(self):
        self.sensor()
        gray = native_marker(self.core)
        result, _ = self.image(data=gray.tobytes())
        if ARTIFACT_DIR:
            cv2.imwrite(str(ARTIFACT_DIR / "synthetic-native320.png"), gray)
            (ARTIFACT_DIR / "synthetic-observation.json").write_text(dumps(result)+"\n")
        self.assertTrue(result["valid"], result)
        output = self.core.tick(100.02)
        self.assertIsNotNone(self.core.mission.discovery_time)
        self.assertEqual(output["phase"], "ACQUIRE")
        self.assertFalse(output["motors_off"])

    def test_wait15_preserved_when_no_discovery(self):
        for i in range(1501):
            t = 100.+i*.01
            self.core.sensor(packet(t), t)
            if i % 2 == 0:
                out = self.core.tick(t)
                if t < 115.:
                    self.assertEqual(out["phase"], "WAIT")
        self.assertEqual(out["phase"], "EXPLORE")

    def test_transport_stale_uses_capture_and_receipt_age_and_keeps_motors_on(self):
        self.assertEqual(self.core.tick(100.)["status"]["transport"], "WAIT_INPUT")
        self.sensor()
        out = self.core.tick(100.13)
        self.assertEqual(out["status"]["transport"], "SENSOR_STALE")
        self.assertEqual(out["velocity_world_m_s"], [0., 0., 0.])
        self.assertFalse(out["motors_off"])
        delayed = GatewayCore(self.cfg)
        delayed.sensor(packet(100.), 100.11)
        self.assertEqual(delayed.tick(100.13)["status"]["transport"], "SENSOR_STALE")

    def test_simulation_clock_pause_and_backward_reset_requires_restart(self):
        self.core.sensor(packet(0.), 0.)
        self.core.tick(.02)
        with self.assertRaisesRegex(InputRejected, "paused"):
            self.core.tick(.02)
        with self.assertRaisesRegex(InputRejected, "clock_regressed"):
            self.core.sensor(packet(.01), .01)
        out = self.core.tick(.03)
        self.assertEqual(out["status"]["transport"], "FAULT")
        self.assertFalse(out["motors_off"])

    def test_geometry_ready_does_not_claim_contact_or_deployment_complete(self):
        self.sensor()
        self.core.mission.phase = "GEOMETRY_READY"
        out = self.core.tick(100.02)
        self.assertEqual(out["phase"], "GEOMETRY_READY")
        self.assertTrue(out["status"]["geometry_ready"])
        for key in ("physical_success", "deployment_complete", "motors_off"):
            self.assertFalse(out["status"][key])

    def test_unexpected_core_motoroff_is_not_forwarded(self):
        from mission3.controller import Command
        self.sensor()
        self.core.mission.update = lambda *args: Command("TOUCHDOWN", [0., 0., 0.], 0., motors_off=True)
        out = self.core.tick(100.02)
        self.assertEqual(out["status"]["transport"], "FAULT")
        self.assertFalse(out["motors_off"])
        self.assertFalse(out["status"]["physical_success"])

    def test_output_status_is_finite_json_with_rejection_evidence(self):
        self.core.reject("sensor", InputRejected("test rejected packet"))
        self.sensor()
        out = self.core.tick(100.02)
        status = strict_json(dumps(out["status"]))
        self.assertEqual(status["counts"]["sensor_rejected"], 1)
        self.assertEqual(status["last_rejection"]["reason"], "test rejected packet")
        self.assertEqual(out["frame_id"], WORLD_FRAME)


@unittest.skipUnless(RUN_ROS, "--ros-roundtrip required")
class RosRoundtripTests(unittest.TestCase):
    # skipUnless 값은 import 시 결정되므로 main에서 아래 클래스를 직접 활성화한다.
    def setUp(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, String
        self.rclpy, self.String, self.Image = rclpy, String, Image
        self.sensor_qos = qos_profile_sensor_data
        self.prefix = "/drone3/mission3/test_"+str(time.time_ns())
        self.directory = ARTIFACT_DIR / self._testMethodName
        self.directory.mkdir(parents=True, exist_ok=True)
        self.log = ArtifactLog(self.directory)
        rclpy.init(args=[])
        self.core = GatewayCore(load_public_config(), mounts=load_mount_s())
        clock_topic = self.prefix+"/clock" if "sim_time" in self._testMethodName else None
        self.gateway = make_ros_node(self.core, self.log, self.prefix, record_images=True, sim_clock_topic=clock_topic)
        self.peer = Node("mission3_gateway_synthetic_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gateway)
        self.executor.add_node(self.peer)
        self.sensor_pub = self.peer.create_publisher(String, self.prefix+"/sensor_packet", qos_profile_sensor_data)
        self.image_pub = self.peer.create_publisher(Image, self.prefix+"/image", qos_profile_sensor_data)
        self.messages = {"command": [], "phase": [], "status": [], "motor": []}
        def append_command(msg):
            self.messages["command"].append((time.monotonic(), msg))
        def append_phase(msg):
            self.messages["phase"].append((time.monotonic(), msg.data))
        def append_status(msg):
            self.messages["status"].append((time.monotonic(), strict_json(msg.data)))
        def append_motor(msg):
            self.messages["motor"].append((time.monotonic(), msg.data))
        self.subs = [
            self.peer.create_subscription(TwistStamped, self.prefix+"/cmd_velocity", append_command, 10),
            self.peer.create_subscription(String, self.prefix+"/phase", append_phase, 10),
            self.peer.create_subscription(String, self.prefix+"/status", append_status, 10),
            self.peer.create_subscription(Bool, self.prefix+"/motors_off", append_motor, 10)]
        self.gray = native_marker(self.core)
        cv2.imwrite(str(self.directory / "input-native320.png"), self.gray)
        self.graph = []
        self.report = {"passed": False, "scope": "synthetic sensor/image-only ROS2 pub/sub; no physical loop or truth input"}

    def tearDown(self):
        self.report.update(counts=dict(self.core.counts), reject_reasons=dict(self.core.rejects),
                           fault=self.core.fault, topics=self.graph, prefix=self.prefix)
        (self.directory / "result.json").write_text(dumps(self.report)+"\n")
        self.log.write("test_end", **self.report)
        self.executor.remove_node(self.peer)
        self.executor.remove_node(self.gateway)
        self.peer.destroy_node()
        self.gateway.destroy_node()
        self.executor.shutdown()
        self.rclpy.shutdown()
        self.log.close()

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(.01, max(0., end-time.monotonic())))

    def discover(self):
        end = time.monotonic()+5.
        while time.monotonic() < end:
            self.spin_for(.02)
            if (self.sensor_pub.get_subscription_count() >= 1 and self.image_pub.get_subscription_count() >= 1
                    and all(pub.get_subscription_count() >= 1 for pub in
                            (self.gateway.cmd, self.gateway.phase, self.gateway.status, self.gateway.motor))):
                return
        self.fail("ROS discovery did not establish all six transport topics")

    def publish_packet(self, p):
        self.sensor_pub.publish(self.String(data=dumps(p)))

    def publish_image(self, stamp, width=320, encoding="mono8"):
        msg = self.Image()
        sec = int(stamp)
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int(round((stamp-sec)*1e9))
        if msg.header.stamp.nanosec == 1000000000:
            msg.header.stamp.sec += 1
            msg.header.stamp.nanosec = 0
        msg.header.frame_id = CAMERA_FRAME
        msg.width, msg.height, msg.encoding, msg.step = width, 320, encoding, width
        msg.data = self.gray.tobytes()
        self.image_pub.publish(msg)

    def test_wall_clock_100_20_50_native_marker_and_rejections(self):
        self.discover()
        start = time.monotonic()
        sent = {"sensor": [], "image": []}
        range_record = [None, None]
        def send_sensor():
            now = self.peer.get_clock().now().nanoseconds/1e9
            slot = int((time.monotonic()-start)/.025)
            new = slot != range_record[0]
            if new:
                range_record[:] = [slot, now]
            p = packet(now, range_record[1], new)
            self.publish_packet(p)
            sent["sensor"].append(time.monotonic())
        def send_image():
            self.publish_image(self.peer.get_clock().now().nanoseconds/1e9-.015)
            sent["image"].append(time.monotonic())
        sensor_timer = self.peer.create_timer(.01, send_sensor)
        image_timer = self.peer.create_timer(.05, send_image)
        self.spin_for(3.)
        self.peer.destroy_timer(sensor_timer)
        self.peer.destroy_timer(image_timer)
        # 송신 중단 직전 DDS/실행기 큐에 남은 마지막 정상 입력을 먼저 소비한다.
        self.spin_for(.04)
        self.graph = self.gateway.get_publisher_names_and_types_by_node("mission3_ros_gateway", "/")
        own_outputs = [name for name, _ in self.graph if name.startswith(self.prefix)]
        self.assertEqual(set(own_outputs), {self.prefix+"/"+s for s in ("cmd_velocity", "phase", "status", "motors_off")})
        self.assertFalse(any("cf231" in name or "vehicle_start" in name for name, _ in self.graph))
        self.assertGreater(self.core.counts["sensor_accepted"], 220)
        self.assertGreater(self.core.counts["image_accepted"], 40)
        self.assertGreater(self.core.counts["image_metric_accepted"], 2)
        self.assertTrue(any(p == "LINEUP" for _, p in self.messages["phase"]))
        commands = [(t, m) for t, m in self.messages["command"] if start+.3 <= t <= start+2.8]
        events = [strict_json(line) for line in (self.directory / "transport.jsonl").read_text().splitlines()]
        rates = {"sensor_pub_hz": self.rate(sent["sensor"]), "image_pub_hz": self.rate(sent["image"]),
                 "sensor_received_hz": self.rate([e["receipt_s"] for e in events if e["event"] == "sensor"]),
                 "image_received_hz": self.rate([e["receipt_s"] for e in events if e["event"] == "image"]),
                 "command_received_hz": self.rate([t for t, _ in commands])}
        for stream in ("phase", "status", "motor"):
            rates[stream+"_received_hz"] = self.rate([t for t, _ in self.messages[stream] if start+.3 <= t <= start+2.8])
        self.report["rates"] = rates
        for key, target in (("sensor_pub_hz", 100), ("image_pub_hz", 20), ("sensor_received_hz", 100),
                            ("image_received_hz", 20), ("command_received_hz", 50),
                            ("phase_received_hz", 50), ("status_received_hz", 50), ("motor_received_hz", 50)):
            self.assertAlmostEqual(rates[key], target, delta=target*.15)
        for _, msg in commands:
            self.assertEqual(msg.header.frame_id, WORLD_FRAME)
            self.assertEqual(msg.twist.angular.x, 0.)
            self.assertEqual(msg.twist.angular.y, 0.)
            self.assertTrue(np.all(np.isfinite([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z])))
        self.assertTrue(self.messages["motor"])
        self.assertFalse(any(v for _, v in self.messages["motor"]))
        before = dict(self.core.counts)
        now = self.peer.get_clock().now().nanoseconds/1e9
        bad = packet(now)
        bad["vehicle_velocity_world_m_s"] = [99., 0., 0.]
        self.publish_packet(bad)
        self.publish_image(now, width=324)
        self.spin_for(.03)
        now = self.peer.get_clock().now().nanoseconds/1e9
        self.publish_image(now, encoding="rgb8")
        self.spin_for(.04)
        self.assertEqual(self.core.counts["sensor_accepted"], before["sensor_accepted"])
        self.assertGreaterEqual(self.core.counts["sensor_rejected"], 1)
        self.assertGreaterEqual(self.core.counts["image_rejected"], 2)
        self.spin_for(.15)
        self.assertEqual(self.messages["status"][-1][1]["transport"], "SENSOR_STALE")
        self.report["passed"] = True

    @staticmethod
    def rate(times):
        return (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.

    def test_use_sim_time_clock_pause_and_backward_jump(self):
        from rclpy.parameter import Parameter
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.gateway.set_parameters([Parameter("use_sim_time", value=True)])
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        clock_pub = self.peer.create_publisher(Clock, self.prefix+"/clock", clock_qos)
        self.discover()
        # 전용 clock remap을 사용하므로 메인 시뮬레이션의 /clock에 발행하지 않는다.
        def set_clock(t):
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = int(t), int(round((t-int(t))*1e9))
            clock_pub.publish(msg)
            self.spin_for(.015)
        for i in range(31):
            t = 10.+i*.01
            set_clock(t)
            self.publish_packet(packet(t))
            self.spin_for(.005)
            if i % 5 == 0:
                self.publish_image(t)
                self.spin_for(.005)
        self.assertGreater(self.core.counts["sensor_accepted"], 25)
        self.assertGreater(self.core.counts["image_metric_accepted"], 1)
        self.assertAlmostEqual(self.core.origin, 10.)
        self.assertTrue(any(p == "ACQUIRE" for _, p in self.messages["phase"]))
        count = self.gateway.published
        self.spin_for(.10)
        self.assertEqual(self.gateway.published, count)
        set_clock(9.)
        self.publish_packet(packet(9.))
        self.spin_for(.02)
        self.assertEqual(self.core.fault, "clock_regressed_restart_required")
        set_clock(10.4)
        self.assertEqual(self.messages["status"][-1][1]["transport"], "FAULT")
        self.assertFalse(any(v for _, v in self.messages["motor"]))
        self.report.update(passed=True, simulation_time=True, paused_command_count=count)

    def test_hardware_output_remap_is_rejected_before_publisher_creation(self):
        out = self.directory / "rejected-node"
        command = [str(ROOT / "scripts/run_mission3_ros_gateway.sh"), "--out", str(out),
                   "--ros-args", "--remap", "/drone3/mission3/cmd_velocity:=/cf231/cmd_vel"]
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=15.)
        (self.directory / "remap-process.json").write_text(dumps({"returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr})+"\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("output topic remapping is forbidden", result.stderr)
        events = [strict_json(line) for line in (out / "transport.jsonl").read_text().splitlines()]
        self.assertFalse(any(e["event"] == "command" for e in events))
        self.report.update(passed=True, output_remap_rejected=True)


def main():
    global ARTIFACT_DIR
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-roundtrip", action="store_true")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    ARTIFACT_DIR = args.out or ROOT / "artifacts/mission3/ros-gateway" / datetime.now().strftime("%Y%m%d-%H%M%S-%f-tests")
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = snapshot(ARTIFACT_DIR, load_public_config(), load_mount_s())
    if args.ros_roundtrip:
        RosRoundtripTests.__unittest_skip__ = False
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(BoundaryTests)
    if args.ros_roundtrip:
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(RosRoundtripTests))
    with (ARTIFACT_DIR / "unittest.log").open("w", buffering=1) as stream:
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    changed = [rel for rel, digest in manifest["source_hashes"].items()
               if hashlib.sha256((ROOT / rel).read_bytes()).hexdigest() != digest]
    summary = {"tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
               "skipped": len(result.skipped), "passed": result.wasSuccessful(),
               "changed_sources_during_run": changed, "ros_roundtrip_requested": args.ros_roundtrip,
               "scope": "transport/inference only; no physical loop, shared324SITL or hardware command verification"}
    (ARTIFACT_DIR / "test-result.json").write_text(dumps(summary)+"\n")
    print(dumps(summary))
    print(f"산출물 {ARTIFACT_DIR}")
    if not result.wasSuccessful():
        print((ARTIFACT_DIR / "unittest.log").read_text())
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
