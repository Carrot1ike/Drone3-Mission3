#!/usr/bin/env python3
"""Verify actual ROS telemetry delivery without sending flight commands."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/cf231/pose", "/cf231/status", "/cf231/tof", "/cf231/flow",
          "/cf231/camera/image_raw"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("drone3_sensor_verifier")
    subscriptions = {}
    stats = {name: {"count": 0} for name in TOPICS}
    last_camera = None

    def callback(name):
        def receive(msg):
            nonlocal last_camera
            now = time.monotonic()
            item = stats[name]
            item["count"] += 1
            item.setdefault("first", now)
            item["last"] = now
            if name.endswith("image_raw"):
                item["format"] = {"width": msg.width, "height": msg.height,
                                  "encoding": msg.encoding, "step": msg.step,
                                  "bytes": len(msg.data)}
                last_camera = msg
            else:
                item["sample"] = message_to_ordereddict(msg)
        return receive

    deadline = time.monotonic() + 12
    try:
        while time.monotonic() < deadline:
            types = dict(node.get_topic_names_and_types())
            for name in TOPICS:
                if name in types and name not in subscriptions:
                    stats[name]["type"] = types[name][0]
                    subscriptions[name] = node.create_subscription(
                        get_message(types[name][0]), name, callback(name), qos_profile_sensor_data)
            rclpy.spin_once(node, timeout_sec=0.1)
        success = True
        for name, item in stats.items():
            if item["count"] > 1 and item["last"] > item["first"]:
                item["received_hz"] = (item["count"] - 1) / (item["last"] - item["first"])
            ok = item["count"] >= 3
            if "sample" in item:
                def finite(value):
                    if isinstance(value, dict):
                        return all(finite(v) for v in value.values())
                    if isinstance(value, list):
                        return all(finite(v) for v in value)
                    return not isinstance(value, float) or math.isfinite(value)
                ok = ok and finite(item["sample"])
            item["passed"] = ok
            success &= ok
            item.pop("first", None)
            item.pop("last", None)
            print(f"[{'PASS' if ok else 'FAIL'}] received {name}: "
                  f"{item['count']} messages, {item.get('received_hz', 0):.2f} Hz", flush=True)
        if last_camera is not None:
            ok = (last_camera.width, last_camera.height, last_camera.encoding,
                  last_camera.step, len(last_camera.data)) == (324, 244, "mono8", 324, 79056)
            success &= ok
            if ok:
                path = args.output.with_name("camera-frame.pgm")
                path.write_bytes(b"P5\n324 244\n255\n" + bytes(last_camera.data))
        record = {"passed": success, "flight_commands_sent": False, "sample_duration_s": 12,
                  "topics": stats}
        args.output.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
        return 0 if success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
