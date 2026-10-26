#!/usr/bin/env python3
"""ROS static-marker yaw demonstration, image + onboard gyro/attitude inputs."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Hover, LogDataGeneric, Status
from crazyflie_interfaces.srv import Arm, Takeoff, Land, NotifySetpointsStop
from yaw_core import MarkerYaw, YawFusion, wrap, rate_command


class ClockMap:
    """Unwrap the 24-bit firmware log clock and map to PC receipt clock.

    Minimum observed offset approximates the transport delay; not capture sync.
    """
    def __init__(self):
        self.previous = None
        self.epoch = 0
        self.offset = None

    def map(self, ticks, receipt):
        ticks = int(ticks) & 0xFFFFFF
        if self.previous is not None and ticks < self.previous and self.previous-ticks > 0x800000:
            self.epoch += 1 << 24
        self.previous = ticks
        seconds = (self.epoch + ticks) * .001
        offset = receipt - seconds
        self.offset = offset if self.offset is None else min(self.offset, offset)
        return seconds + self.offset


class YawDemo(Node):
    def __init__(self, out):
        super().__init__("drone3_static_yaw")
        self.out = out
        self.config = json.loads((out / "experiment.json").read_text())
        self.obs_cfg, self.ctrl = self.config["observer"], self.config["control"]
        self.perception = MarkerYaw(json.loads((out / "camera-profile.json").read_text()), self.obs_cfg)
        self.fusion = YawFusion(self.obs_cfg)
        self.clock_map = ClockMap()
        self.phase, self.phase_start = "WAIT", time.monotonic()
        self.started = self.phase_start
        self.events = []
        self.latest_image = None
        self.image_seen = None
        self.imu_receipt = None
        self.imu_count = 0
        self.pose = None
        self.pose_receipt = None
        self.status = None
        self.status_receipt = None
        self.motors, self.motors_receipt = None, None
        self.stop_condition = None
        self.visuals = []
        self.target = None
        self.sign = None
        self.failure = None
        self.done = False
        self.hover_owned = False
        self.pending = None
        self.last_frame_work = 0.
        self.last_publish = None
        self.video = cv2.VideoWriter(str(out / "camera.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 20., (324, 244))
        self.observations = (out / "observations.jsonl").open("w", buffering=1)
        self.control_file = (out / "control.csv").open("w", buffering=1)
        self.control_log = csv.writer(self.control_file)
        self.control_log.writerow(["monotonic_s", "phase", "yaw_est_deg", "target_deg", "yaw_imu_only_deg", "ros_rate_rad_s", "visual_age_s", "imu_age_s", "pose_z_m", "hover_owned", "last_publish_s"])
        self.imu_file = (out / "imu.csv").open("w", buffering=1)
        self.imu_log = csv.writer(self.imu_file)
        self.imu_log.writerow(["receipt_s", "mapped_s", "onboard_ms", "gyro_x_deg_s", "gyro_y_deg_s", "gyro_z_deg_s", "roll_deg", "pitch_deg"])
        self.status_file = (out / "status.csv").open("w", buffering=1)
        self.status_log = csv.writer(self.status_file)
        self.status_log.writerow(["receipt_s", "supervisor_info", "battery_voltage", "pm_state"])
        self.motors_file = (out / "motors.csv").open("w", buffering=1)
        self.motors_log = csv.writer(self.motors_file)
        self.motors_log.writerow(["receipt_s", "onboard_ms", "m1_pwm", "m2_pwm", "m3_pwm", "m4_pwm"])
        self.publisher = self.create_publisher(Hover, "/cf231/cmd_hover", 1)
        self.service_clients = {name: self.create_client(kind, f"/cf231/{name}") for name, kind in
                        [("arm", Arm), ("takeoff", Takeoff), ("land", Land), ("notify_setpoints_stop", NotifySetpointsStop)]}
        self.create_subscription(Image, "/cf231/camera/image_raw", self.image, qos_profile_sensor_data)
        self.create_subscription(LogDataGeneric, "/cf231/yaw_imu", self.imu, qos_profile_sensor_data)
        self.create_subscription(LogDataGeneric, "/cf231/yaw_motors", self.receive_motors, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/cf231/pose", self.receive_pose, qos_profile_sensor_data)
        self.create_subscription(Status, "/cf231/status", self.receive_status, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)

    def event(self, name, **values):
        entry = dict(monotonic_s=time.monotonic(), event=name, **values)
        self.events.append(entry)
        self.get_logger().info(json.dumps(entry))

    def phase_to(self, phase):
        self.phase, self.phase_start = phase, time.monotonic()
        self.event("phase", phase=phase)

    def call(self, name, request, callback):
        if not self.service_clients[name].service_is_ready():
            self.finish(f"service_unavailable:{name}")
            return
        self.pending = (name, time.monotonic())
        future = self.service_clients[name].call_async(request)
        def complete(f):
            self.pending = None
            if f.exception():
                self.finish(f"service_failed:{name}:{f.exception()}")
            elif not self.done:
                callback()
        future.add_done_callback(complete)

    def receive_pose(self, msg):
        # Only onboard position is used to check takeoff/landing and height stability.
        # Its yaw is deliberately excluded from the heading observer.
        self.pose, self.pose_receipt = msg.pose.position, time.monotonic()

    def image(self, msg):
        self.latest_image = (msg, time.monotonic())

    def receive_status(self, msg):
        self.status, self.status_receipt = msg, time.monotonic()
        self.status_log.writerow([self.status_receipt, msg.supervisor_info, msg.battery_voltage, msg.pm_state])

    def receive_motors(self, msg):
        if len(msg.values) == 4 and np.all(np.isfinite(msg.values)):
            self.motors, self.motors_receipt = list(msg.values), time.monotonic()
            self.motors_log.writerow([self.motors_receipt, msg.timestamp, *self.motors])

    def imu(self, msg):
        receipt = time.monotonic()
        if len(msg.values) != 5 or not np.all(np.isfinite(msg.values)):
            return
        gx, gy, gz, roll, fw_pitch = msg.values
        pitch = -fw_pitch  # same physical pitch conversion as pinned ROS pose callback
        if max(abs(roll), abs(pitch)) > 35.:
            self.abort("excessive_tilt")
            return
        mapped = self.clock_map.map(msg.timestamp, receipt)
        bias = self.config["test"]["observer_gyro_bias_deg_s"]
        self.fusion.imu(mapped, [gx, gy, gz], roll, pitch, bias)
        self.imu_count += 1
        self.imu_receipt = receipt
        self.imu_log.writerow([receipt, mapped, msg.timestamp, gx, gy, gz, roll, pitch])

    def process_frame(self, now):
        if self.latest_image is None or self.latest_image[1] == self.image_seen:
            return
        msg, receipt = self.latest_image
        self.image_seen = receipt
        if msg.encoding != "mono8" or (msg.width, msg.height) != (324, 244):
            self.abort("unexpected_image_format")
            return
        gray = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        # Header is server receive/decode time. This is explicitly a bounded
        # delay approximation; the protocol does not provide a capture timestamp.
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        transport_age = max(0., ros_now-header)
        capture_estimate = now-transport_age-self.obs_cfg["assumed_image_delay_s"]
        history = self.fusion.at(capture_estimate)
        if history is None:
            result = {"valid": False, "reason": "no_attitude_history"}
        elif transport_age > self.obs_cfg["max_visual_age_s"]:
            result = {"valid": False, "reason": "stale_image"}
        elif self.phase == "HOLD" and now-self.phase_start < self.config["test"]["drop_images_during_hold_s"]:
            result = {"valid": False, "reason": "test_image_dropout"}
        else:
            result = self.perception.detect(gray, history[1], history[2])
        if result["valid"]:
            fused, reason = self.fusion.visual(math.radians(result["yaw_deg"]), capture_estimate, now)
            result["fused"], result["fusion_reason"] = fused, reason
            if fused:
                self.visuals.append((now, self.fusion.angle))
                self.visuals = self.visuals[-100:]
        result.update(monotonic_s=now, phase=self.phase, image_receipt_s=receipt, capture_estimate_s=capture_estimate,
                      header_age_s=transport_age)
        self.observations.write(json.dumps(result) + "\n")
        picture = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if "corners" in result:
            cv2.polylines(picture, [np.array(result["corners"], dtype=np.int32)], True, (0, 200, 0) if result["valid"] else (0, 0, 255), 1)
        label = f"{self.phase} yaw={math.degrees(self.fusion.angle):+.1f}" if self.fusion.angle is not None else self.phase
        cv2.putText(picture, label, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 220, 255), 1)
        self.video.write(picture)
        if not (self.out / f"frame-{self.phase}.png").exists():
            cv2.imwrite(str(self.out / f"frame-{self.phase}.png"), picture)

    def publish_hover(self, rate):
        msg = Hover()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vx, msg.vy, msg.yaw_rate, msg.z_distance = 0., 0., float(rate), self.ctrl["height_m"]
        self.publisher.publish(msg)
        self.last_publish = time.monotonic()

    def abort(self, reason):
        if self.done or self.phase in ("QUIET", "LAND_PLAN", "RELEASE", "LAND", "DISARM", "LANDED_CHECK"):
            return
        self.failure = reason
        self.event("abort", reason=reason)
        if self.phase in ("WAIT", "ARM"):
            self.finish(reason)
        else:
            self.begin_land()

    def begin_land(self):
        if self.hover_owned:
            self.publish_hover(0.)
        # Allow the final low-level packet to drain before starting a NEW plan.
        self.hover_owned = False
        self.phase_to("QUIET")

    def start_land(self):
        request = Land.Request()
        request.height, request.duration.sec = .0, int(self.ctrl["land_s"])
        self.phase_to("LAND_PLAN")
        def release():
            self.phase_to("RELEASE")
            request = NotifySetpointsStop.Request()
            request.remain_valid_millisecs = 100  # ignored by this pinned server/firmware
            self.call("notify_setpoints_stop", request, lambda: self.phase_to("LAND"))
        self.call("land", request, release)

    def finish(self, failure=None):
        self.failure = failure or self.failure
        self.done = True
        self.event("finished", failure=self.failure)
        self.video.release()
        self.control_file.flush()
        self.imu_file.flush()
        self.status_file.flush()
        self.motors_file.flush()
        self.observations.flush()
        summary = {"completed": self.failure is None, "failure": self.failure,
                   "ros_to_physical_yaw_sign": self.sign,
                   "target_yaw_deg": None if self.target is None else math.degrees(self.target),
                   "imu_samples": self.imu_count, "events": self.events,
                   "stop_condition": self.stop_condition,
                   "truth_used_by_controller": False, "firmware_yaw_estimator_modified": False,
                   "image_timestamp": "post-decode ROS stamp minus configured approximate delay"}
        (self.out / "control-summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if now-self.last_frame_work >= .045:
            self.process_frame(now)
            self.last_frame_work = now
        elapsed = now-self.phase_start
        imu_age = float("inf") if self.imu_receipt is None else now-self.imu_receipt
        visual_age = float("inf") if self.fusion.last_visual_time is None else now-self.fusion.last_visual_time
        rate = 0.
        if self.pending and now-self.pending[1] > 2.:
            self.finish(f"service_timeout:{self.pending[0]}")
            return
        if self.phase == "WAIT":
            if elapsed > self.ctrl["ready_timeout_s"]:
                self.finish("ready_timeout")
                return
            ready = self.imu_count >= 30 and imu_age < .1 and visual_age < .2 and len(self.visuals) >= 10 and self.pose is not None
            if ready and all(c.service_is_ready() for c in self.service_clients.values()):
                angles = [x[1] for x in self.visuals[-10:]]
                self.target = math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
                self.imu_zero = self.fusion.predict_only
                self.phase_to("ARM")
                request = Arm.Request(); request.arm = True
                self.call("arm", request, lambda: self.phase_to("ARMED_WAIT"))
        elif self.phase == "ARMED_WAIT":
            if elapsed >= 1. and self.status is not None and self.status.supervisor_info & 2 and now-self.status_receipt < 2.:
                request = Takeoff.Request(); request.height = self.ctrl["height_m"]; request.duration.sec = int(self.ctrl["takeoff_s"])
                self.phase_to("TAKEOFF")
                self.call("takeoff", request, lambda: None)
            elif elapsed > 5.:
                self.abort("arming_not_confirmed")
        elif self.phase == "TAKEOFF":
            if elapsed > self.ctrl["takeoff_s"]+1.5 and self.pose is not None and abs(self.pose.z-self.ctrl["height_m"]) < .08:
                self.hover_owned = True
                self.phase_to("ACQUIRE")
            elif elapsed > self.ctrl["takeoff_s"]+8.:
                self.abort("takeoff_height_timeout")
        elif self.phase == "ACQUIRE":
            if elapsed > 2. and visual_age < .15:
                self.pulse_imu_start = self.fusion.predict_only
                self.pulse_visual_start = self.fusion.angle
                self.phase_to("SIGN_POS")
        elif self.phase in ("SIGN_POS", "SIGN_NEG"):
            direction = 1. if self.phase == "SIGN_POS" else -1.
            rate = direction*math.radians(self.ctrl["pulse_rate_deg_s"])
            if elapsed >= self.ctrl["pulse_s"]:
                gyro_delta = math.degrees(self.fusion.predict_only-self.pulse_imu_start)
                visual_delta = math.degrees(wrap(self.fusion.angle-self.pulse_visual_start))
                self.event("sign_response", command_direction=direction, gyro_delta_deg=gyro_delta, visual_delta_deg=visual_delta)
                if abs(gyro_delta) < 3. or abs(visual_delta) < 2. or gyro_delta*visual_delta <= 0:
                    self.abort("yaw_sign_not_confirmed")
                elif self.phase == "SIGN_POS":
                    self.sign = 1 if gyro_delta > 0 else -1
                    self.phase_to("RECOVER_POS")
                elif gyro_delta*direction*self.sign <= 0:
                    self.abort("negative_yaw_sign_mismatch")
                else:
                    self.phase_to("RECOVER_NEG")
                rate = 0.
        elif self.phase in ("RECOVER_POS", "RECOVER_NEG", "HOLD"):
            if visual_age <= self.obs_cfg["max_visual_age_s"] and imu_age <= self.obs_cfg["max_imu_age_s"]:
                rate = self.sign*rate_command(self.target, self.fusion.angle, self.ctrl)
            if self.phase == "RECOVER_POS" and elapsed >= self.ctrl["recovery_s"]:
                self.pulse_imu_start = self.fusion.predict_only
                self.pulse_visual_start = self.fusion.angle
                self.phase_to("SIGN_NEG")
            elif self.phase == "RECOVER_NEG" and elapsed >= self.ctrl["recovery_s"]:
                self.phase_to("HOLD")
            elif self.phase == "HOLD" and elapsed >= self.ctrl["hold_s"]:
                self.begin_land()
                rate = 0.
        elif self.phase == "QUIET" and elapsed >= .10:
            self.start_land()
        elif self.phase == "LAND":
            if elapsed > self.ctrl["land_s"]+1.5:
                if self.pose is not None and self.pose.z < .12:
                    request = Arm.Request(); request.arm = False
                    self.phase_to("DISARM")
                    self.call("arm", request, lambda: self.phase_to("LANDED_CHECK"))
                elif elapsed > self.ctrl["land_s"]+6.:
                    self.finish("landing_height_timeout")
        elif self.phase == "LANDED_CHECK":
            fresh_status = self.status is not None and now-self.status_receipt < 2.
            motors_zero = self.motors is not None and now-self.motors_receipt < .3 and max(self.motors) == 0.
            if elapsed > 1. and fresh_status and motors_zero and not self.status.supervisor_info & 16:
                if self.status.supervisor_info & 4:
                    # The pinned SITL auto-arms after disarming. Don't claim armed=0.
                    self.stop_condition = "landed_motor_pwm_zero_auto_arm_enabled"
                elif not self.status.supervisor_info & 2:
                    self.stop_condition = "landed_motor_pwm_zero_disarmed"
                if self.stop_condition:
                    self.event("landed_stopped", supervisor_info=self.status.supervisor_info, motor_pwm=self.motors,
                               stop_condition=self.stop_condition)
                    self.finish()
            elif elapsed > 5.:
                self.finish("landed_stop_not_confirmed")
        if self.hover_owned:
            if self.last_publish is not None and now-self.last_publish > self.ctrl["max_command_gap_s"]:
                self.abort("command_timer_gap")
            elif imu_age > self.obs_cfg["max_imu_age_s"]:
                self.abort("imu_stale")
            elif visual_age > self.ctrl["loss_abort_s"]:
                self.abort("visual_loss")
            elif self.pose_receipt is None or now-self.pose_receipt > .5:
                self.abort("height_telemetry_stale")
            else:
                if visual_age > self.obs_cfg["max_visual_age_s"]:
                    rate = 0.
                self.publish_hover(rate)
        angle = None if self.fusion.angle is None else math.degrees(self.fusion.angle)
        self.control_log.writerow([now, self.phase, angle, None if self.target is None else math.degrees(self.target),
                                  math.degrees(self.fusion.predict_only-getattr(self, "imu_zero", 0.)) + (0. if self.target is None else math.degrees(self.target)),
                                  rate if self.hover_owned else 0., visual_age, imu_age, None if self.pose is None else self.pose.z,
                                  self.hover_owned, self.last_publish])

    def close_files(self):
        self.video.release()
        for f in (self.observations, self.control_file, self.imu_file, self.status_file, self.motors_file):
            f.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    # This experiment is coupled to a localhost UDP-only config generated by run_yaw_demo.py.
    import yaml
    robots = yaml.safe_load((args.output / "crazyflies.yaml").read_text())["robots"]
    if list(robots) != ["cf231"] or robots["cf231"]["uri"] != "udp://127.0.0.1:19850":
        parser.error("이 데모는 localhost SIL 전용입니다.")
    rclpy.init()
    node = YawDemo(args.output)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
