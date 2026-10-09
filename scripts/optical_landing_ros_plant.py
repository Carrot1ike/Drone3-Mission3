#!/usr/bin/env python3
"""ROS 속도 명령을 소비하는 가상 광학점. 실제 기체 topic/service 없음."""
import csv
import json
import math
import os
from pathlib import Path
import socket
import struct
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from optical_landing_plant import OpticalPlant


class OpticalROSPlant(Node):
    def __init__(self, out):
        super().__init__("drone3_optical_plant")
        self.config = json.loads((out / "experiment.json").read_text())
        self.plant = OpticalPlant(self.config, out)
        self.continuous = self.config["vehicle"].get("motion_mode") == "continuous"
        if self.continuous:
            # ROS 준비 시간과 임무 시간을 구분. 첫 WAIT 명령부터 독립 차량을
            # 움직이며 마커 검출/vehicle_start topic은 시작 조건이 아니다.
            self.plant.vehicle_started = False
        self.command = {"phase": "ROS", "velocity_m_s": [0., 0., 0.], "yaw_rate_rad_s": 0., "start_vehicle": False}
        self.receipt = None
        self.last_tick = self.started = time.monotonic()
        self.next_image = self.started
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.file = (out / "truth.csv").open("w", buffering=1)
        self.writer = csv.DictWriter(self.file, fieldnames=["monotonic_s", *self.plant.truth(0., self.command)])
        self.writer.writeheader()
        self.accepted = 0
        self.create_subscription(TwistStamped, "/drone3/optical/cmd_velocity", self.receive_command, 1)
        self.create_subscription(Bool, "/drone3/optical/vehicle_start", self.start_vehicle,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.02, self.tick)

    def start_vehicle(self, msg):
        self.command["start_vehicle"] |= msg.data

    def receive_command(self, msg):
        values = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.z]
        if msg.header.frame_id != "marker_optical_axes" or not np.all(np.isfinite(values)):
            return
        if np.linalg.norm(values[:2]) > .56 or abs(values[2]) > .30 or abs(values[3]) > math.radians(16.):
            return
        self.command.update(velocity_m_s=values[:3], yaw_rate_rad_s=values[3])
        self.receipt = time.monotonic()
        self.accepted += 1

    def tick(self):
        now = time.monotonic()
        dt = min(.10, max(0., now-self.last_tick))
        self.last_tick = now
        if self.continuous and self.receipt is not None:
            self.plant.vehicle_started = True
        command = dict(self.command)
        if self.receipt is None or now-self.receipt > .25:
            command.update(velocity_m_s=[0., 0., 0.], yaw_rate_rad_s=0., phase="watchdog_hold")
        self.plant.step(command, dt)
        self.writer.writerow(dict(monotonic_s=now, **self.plant.truth(now-self.started, command)))
        if now < self.next_image:
            return
        self.next_image = now + .05
        image = self.plant.render().tobytes()
        total = math.ceil(len(image)/60000)
        for seq in range(total):
            self.sender.sendto(struct.pack("<HHHH", seq, total, 320, 320) + image[seq*60000:(seq+1)*60000], ("127.0.0.1", 5200))

    def close(self):
        self.sender.close(); self.file.close(); self.plant.close()


if __name__ == "__main__":
    rclpy.init()
    node = OpticalROSPlant(Path(os.environ["DRONE3_EXPERIMENT_DIR"]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
