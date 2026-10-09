#!/usr/bin/env python3
"""Read-only ROS image contract check: publishes no aircraft commands."""
import argparse
import json
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from provided_hardware import CameraContractError, decode_mono320


class ProvidedCameraCheck(Node):
    def __init__(self, out):
        super().__init__("drone3_provided_camera_check")
        self.out, self.started = out, time.monotonic()
        self.done, self.failure, self.received, self.detected = False, None, 0, 0
        self.formats, self.receipts = set(), []
        self.observations = (out / "ros-images.jsonl").open("w", buffering=1)
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        self.create_subscription(Image, "/cf231/camera/image_raw", self.image, qos_profile_sensor_data)
        self.create_timer(.1, self.tick)

    def image(self, msg):
        if self.done:
            return
        now = time.monotonic()
        try:
            gray = decode_mono320(msg.width, msg.height, msg.encoding, msg.step, msg.data)
        except CameraContractError as exc:
            self.failure = str(exc)
            self.finish()
            return
        self.received += 1
        _, ids, _ = self.detector.detectMarkers(gray)
        found = ids is not None and 0 in ids.flatten()
        self.detected += int(found)
        self.receipts.append(now)
        self.formats.add((msg.width, msg.height, msg.encoding, msg.step, len(msg.data)))
        self.observations.write(json.dumps({"receipt_monotonic_s": now, "width": msg.width,
            "height": msg.height, "encoding": msg.encoding, "step": msg.step,
            "bytes": len(msg.data), "marker_detected": bool(found)}) + "\n")
        if self.received == 1:
            cv2.imwrite(str(self.out / "ros-received-frame.png"), gray)
        if self.received >= 60:
            self.finish()

    def tick(self):
        if not self.done and time.monotonic()-self.started > 15.:
            self.failure = "image_receive_timeout"
            self.finish()

    def finish(self):
        self.done = True
        report = {"passed": self.failure is None and self.received >= 60 and self.detected >= 54,
            "failure": self.failure, "received_frames": self.received, "marker_detected_frames": self.detected,
            "received_formats": [list(row) for row in sorted(self.formats)],
            "observed_test_receive_fps": (len(self.receipts)-1)/(self.receipts[-1]-self.receipts[0]) if len(self.receipts) > 1 else None,
            "pipeline": "native mono320 synthetic input -> UDP camera chunks -> shared CPX -> shared ROS camera bridge -> fixed image contract -> ArUco",
            "aircraft_firmware_started": False, "aircraft_commands_published": False,
            "scope": "camera format and transport integration only; not measured hardware fps, autonomous pursuit or landing",
            "eligible_for_supplied_aircraft_landing_validation": False}
        (self.out / "camera-transport.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        # Existing launcher completion sentinel; this node has no control publisher.
        (self.out / "control-summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        self.observations.close()
        self.get_logger().info(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = ProvidedCameraCheck(args.output)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
