#!/usr/bin/env python3
"""동일 관측/제어 모듈의 ROS 광학점 폐루프 연결. 기체 명령 발행 없음."""
import json
import os
from pathlib import Path
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from provided_hardware import decode_mono320, CameraContractError
from optical_landing_observer import OpticalLandingObserver
from optical_landing_plant import annotated_image
from landing_mission import LandingMission


class OpticalLandingROS(Node):
    def __init__(self, out):
        super().__init__("drone3_optical_landing")
        self.out, self.started, self.done = out, time.monotonic(), False
        self.config = json.loads((out / "experiment.json").read_text())
        profile = json.loads((out / "camera-profile.json").read_text())
        self.observer = OpticalLandingObserver(profile, self.config)
        self.mission = LandingMission(self.config)
        self.latest = None
        self.seen = None
        self.command = {"phase": self.mission.phase}
        self.events = []
        self.frames, self.accepted, self.published = 0, 0, 0
        self.observations = (out / "observations.jsonl").open("w", buffering=1)
        self.commands = (out / "commands.jsonl").open("w", buffering=1)
        self.video = cv2.VideoWriter(str(out / "mission.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 20., (320, 320), isColor=False)
        self.publisher = self.create_publisher(TwistStamped, "/drone3/optical/cmd_velocity", 1)
        self.vehicle_publisher = self.create_publisher(Bool, "/drone3/optical/vehicle_start", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vehicle_sent = False
        self.create_subscription(Image, "/cf231/camera/image_raw", self.image, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)

    def image(self, msg):
        self.latest = (msg, time.monotonic())

    def publish(self, command):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "marker_optical_axes"
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = [float(v) for v in command["velocity_m_s"]]
        message.twist.angular.z = float(command["yaw_rate_rad_s"])
        self.publisher.publish(message)
        self.published += 1
        if command["start_vehicle"] and not self.vehicle_sent:
            self.vehicle_publisher.publish(Bool(data=True))
            self.vehicle_sent = True

    def tick(self):
        if self.done:
            return
        now = time.monotonic()-self.started
        if self.latest is not None and self.latest[1] != self.seen:
            msg, receipt = self.latest
            self.seen = receipt
            try:
                gray = decode_mono320(msg.width, msg.height, msg.encoding, msg.step, msg.data)
            except CameraContractError as exc:
                self.finish({"phase": "ABORT", "failure": str(exc), "inferred_completion": False})
                return
            self.frames += 1
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            transport_age = max(0., ros_now-header)
            capture = max(0., now-transport_age-self.config["observer"]["assumed_image_delay_s"])
            result = self.observer.frame(gray, capture, now)
            self.accepted += int(result["valid"])
            self.observations.write(json.dumps(dict(time_s=now, **result)) + "\n")
            self.video.write(annotated_image(gray, self.command["phase"], self.observer.state(now)))
            image_path = self.out / f"frame-{self.command['phase']}.png"
            if not image_path.exists():
                cv2.imwrite(str(image_path), gray)
        observation = self.observer.state(now)
        previous = self.command["phase"]
        self.command = self.mission.update(now, observation)
        self.commands.write(json.dumps(dict(time_s=now, monotonic_s=time.monotonic(), observation=observation, **self.command)) + "\n")
        if previous != self.command["phase"]:
            self.events.append({"time_s": now, "phase": self.command["phase"]})
            self.get_logger().info(f"{previous} -> {self.command['phase']}")
        self.publish(self.command)
        if self.command["done"] or self.command["phase"] == "ABORT":
            self.finish(self.command)

    def finish(self, command):
        self.done = True
        report = {"phase": command["phase"], "failure": command.get("failure"),
            "inferred_completion": bool(command["inferred_completion"]), "events": self.events,
            "scenario": self.config.get("scenario", {}),
            "frames": self.frames, "accepted_observations": self.accepted, "published_commands": self.published,
            "command_topic": "/drone3/optical/cmd_velocity", "aircraft_commands_published": False,
            "scope": "ROS image-driven optical-point kinematics; no aircraft dynamics or physical touchdown",
            "hardware_landing_validated": False}
        self.video.release(); self.observations.close(); self.commands.close()
        (self.out / "control-summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        self.get_logger().info(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    rclpy.init()
    node = OpticalLandingROS(Path(os.environ["DRONE3_EXPERIMENT_DIR"]))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
