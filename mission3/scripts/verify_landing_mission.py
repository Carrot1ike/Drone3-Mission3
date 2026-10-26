#!/usr/bin/env python3
"""순수 임무 제어기의 핵심 회귀 검사. 기체/영상/물리 접촉 검증이 아니다.

실행: PYTHONDONTWRITEBYTECODE=1 <python> scripts/verify_landing_mission.py
config/moving-landing.json을 기본으로 읽는다. 지연/잡음이 있는 광학점 운동학
폐루프, FF/제한, 하강 금지/손실, 시간 순서와 확인 시간을 검사한다.
파일/보고서/영상을 생성하지 않는다. --config로 다른 실험 설정을 지정할 수 있다.
"""

import argparse
from collections import deque
import copy
import json
import math
from pathlib import Path
import random
import unittest

try:
    from .landing_mission import LandingMission
except ImportError:
    from landing_mission import LandingMission


CONFIG = json.loads((Path(__file__).resolve().parents[1] / "config/moving-landing.json").read_text())


def observation(now, position=(0.0, 0.0, -0.30), velocity=(0.0, 0.0, 0.0), yaw=0.0, age=0.0):
    return {"valid": True, "stamp_s": now - age, "relative_pad_m": list(position),
            "relative_velocity_m_s": list(velocity), "heading_error_rad": yaw}


def norm(vector):
    return math.hypot(*vector)


class LandingMissionChecks(unittest.TestCase):
    """짧은 시나리오별 검증. 시간 적분은 광학점에 명령을 그대로 적용한다."""

    def make_mission(self, **control_overrides):
        config = copy.deepcopy(CONFIG)
        config.setdefault("control", {}).update(control_overrides)
        return LandingMission(config)

    def assert_stop(self, result, phase=None):
        self.assertEqual(result["velocity_m_s"], [0.0, 0.0, 0.0])
        self.assertEqual(result["yaw_rate_rad_s"], 0.0)
        if phase is not None:
            self.assertEqual(result["phase"], phase)
        if result["phase"] != "COMPLETE":
            self.assertFalse(result["done"])
            self.assertFalse(result["inferred_completion"])

    def enter_descent(self, mission):
        """현재 시각의 완전 추종 관측으로 DESCEND 진입; 내부 상태를 강제하지 않는다."""
        for index in range(150):
            now = index * 0.05
            result = mission.update(now, observation(now))
            if result["phase"] == "DESCEND":
                return now, result
        self.fail("DESCEND was not reached")

    def test_delayed_noisy_optical_point_closed_loop(self):
        """20 Hz 영상/50 Hz 제어, 0.1초 지연/잡음이 있는 광학점 폐루프 전이."""
        mission = self.make_mission()
        random_source = random.Random(42)
        dt = 0.01  # 적분 100 Hz, 영상 20 Hz, 제어 50 Hz
        control_dt = 0.02
        image_delay_s = 0.10
        pad = [0.35, 0.0, 0.20]
        camera = [0.0, 0.025, 0.50]
        heading = math.radians(5.0)
        camera_velocity = [0.0, 0.0, 0.0]
        vehicle_speed = 0.0
        queue = deque()
        last_received = None
        transitions = []
        previous_command = [0.0, 0.0, 0.0]
        first_track_s = None
        for index in range(6501):
            now = round(index * dt, 9)
            if index % 5 == 0:
                packet = observation(now, [p - c for p, c in zip(pad, camera)],
                                     [vehicle_speed - camera_velocity[0], -camera_velocity[1],
                                      -camera_velocity[2]], -heading)
                packet["relative_pad_m"] = [v + random_source.gauss(0, 0.0007)
                                            for v in packet["relative_pad_m"]]
                packet["relative_velocity_m_s"] = [v + random_source.gauss(0, 0.001)
                                                   for v in packet["relative_velocity_m_s"]]
                packet["heading_error_rad"] += random_source.gauss(0, math.radians(0.15))
                queue.append(packet)
            while queue and queue[0]["stamp_s"] <= now - image_delay_s + 1e-9:
                last_received = queue.popleft()
            received = None
            if last_received is not None:
                received = dict(last_received)
                received["relative_pad_m"] = [p + v * (now - received["stamp_s"])
                                              for p, v in zip(received["relative_pad_m"],
                                                              received["relative_velocity_m_s"])]
                received["prediction_stamp_s"] = now
            if index % 2:
                camera = [p + v * dt for p, v in zip(camera, camera_velocity)]
                pad[0] += vehicle_speed * dt
                heading += result["yaw_rate_rad_s"] * dt
                continue
            result = mission.update(now, received)
            if not transitions or transitions[-1][0] != result["phase"]:
                transitions.append((result["phase"], now))
            self.assertLessEqual(norm(result["velocity_m_s"]), mission.control["max_velocity_m_s"] + 1e-9)
            self.assertLessEqual(abs(result["yaw_rate_rad_s"]),
                                 math.radians(mission.control["max_rate_deg_s"]) + 1e-9)
            if received is not None and result["phase"] not in ("COMPLETE", "ABORT"):
                self.assertLessEqual(norm([v - old for v, old in zip(result["velocity_m_s"], previous_command)]),
                                     mission.control["max_accel_m_s2"] * control_dt + 1e-9,
                                     f"time={now} phase={result['phase']}")
            self.assertIsNone(result["failure"], result)
            if result["phase"] == "TRACK" and first_track_s is None:
                first_track_s = now
            if result["phase"] == "DESCEND":
                self.assertGreaterEqual(now - first_track_s + 1e-9, mission.control["track_hold_s"])
                self.assertGreaterEqual(result["velocity_m_s"][2], -mission.control["descent_m_s"] - 1e-9)
            if result["done"]:
                self.assertTrue(result["inferred_completion"])
                self.assertEqual(result["phase"], "COMPLETE")
                self.assertLessEqual(norm([p - c for p, c in zip(pad[:2], camera[:2])]), 0.01)
                self.assertLessEqual(abs(camera[2] - pad[2] - 0.03), 0.006)
                self.assertLess(now, 65.0)
                break
            vehicle_speed = CONFIG.get("vehicle", {}).get("speed_m_s", 0.3) if result["start_vehicle"] else 0.0
            camera_velocity = result["velocity_m_s"]
            camera = [p + v * dt for p, v in zip(camera, camera_velocity)]
            pad[0] += vehicle_speed * dt
            heading += result["yaw_rate_rad_s"] * dt
            previous_command = list(camera_velocity)
        else:
            self.fail(f"completion not reached: {transitions}, {result}")
        self.assertEqual([phase for phase, _ in transitions],
                         ["SEARCH", "ALIGN", "APPROACH", "TRACK", "DESCEND", "SETTLE", "COMPLETE"])
        self.assertTrue(result["start_vehicle"])
        self.assert_stop(mission.update(now + control_dt, None), "COMPLETE")
        print("\n광학점 운동학 폐루프: " + " → ".join(phase for phase, _ in transitions)
              + f"; {now:.2f} s, 실제 적분 높이 {camera[2] - pad[2]:.4f} m, 영상20/제어50 Hz"
              + " (영상 검출·기체 동역학·접촉 검증 제외)")

    def test_alignment_feedforward_and_feedback(self):
        mission = self.make_mission()
        original = copy.deepcopy(CONFIG)
        check_copy = LandingMission(CONFIG)
        check_copy.update(0.0, observation(0.0))
        self.assertEqual(CONFIG, original)
        for index in range(10):
            now = index * 0.1
            result = mission.update(now, observation(now, yaw=math.radians(3)))
            self.assertEqual(result["phase"], "ALIGN")
            self.assertFalse(result["start_vehicle"])
            self.assertEqual(result["velocity_m_s"][:2], [0.0, 0.0])
        for index in range(10, 20):
            now = index * 0.1
            result = mission.update(now, observation(now, yaw=math.radians(1.9)))
            self.assertFalse(result["start_vehicle"])
        result = mission.update(2.0, observation(2.0, yaw=math.radians(1.9)))
        self.assertEqual(result["phase"], "APPROACH")
        self.assertTrue(result["start_vehicle"])
        for index in range(21, 30):
            now = index * 0.1
            result = mission.update(now, observation(now, (0.1, 0.02, -0.30), (0.02, -0.01, 0.0)))
        self.assertAlmostEqual(result["velocity_m_s"][0], 0.3 + 0.7 * 0.1 + 0.5 * 0.02)
        self.assertAlmostEqual(result["velocity_m_s"][1], 0.7 * 0.02 - 0.5 * 0.01)
        self.assertEqual(result["velocity_m_s"][2], 0.0)
        self.assertTrue(result["start_vehicle"])
        no_ff_config = copy.deepcopy(CONFIG)
        no_ff_config.setdefault("test", {})["ff_enabled"] = False
        no_ff = LandingMission(no_ff_config)
        for index in range(30):
            now = index * 0.1
            no_ff_result = no_ff.update(now, observation(now))
        self.assertEqual(no_ff_result["velocity_m_s"], [0.0, 0.0, 0.0])
        self.assertTrue(no_ff_result["start_vehicle"])

    def test_vector_saturation_and_physical_yaw(self):
        mission = self.make_mission()
        now, _ = self.enter_descent(mission)
        old = list(mission._last_velocity)
        for index in range(1, 31):
            stamp = now + index * 0.05
            # 큰 위치/속도 오차에서도 norm 상한; 하강 금지 시 Z는 즉시 정지한다.
            result = mission.update(stamp, observation(stamp, (10, -10, -0.30),
                                                        (2, -2, 0), math.pi / 2))
            self.assertLessEqual(norm(result["velocity_m_s"]), 0.55 + 1e-9)
            self.assertLessEqual(norm([v - prev for v, prev in zip(result["velocity_m_s"][:2], old[:2])]),
                                 0.5 * 0.05 + 1e-9)
            self.assertEqual(result["velocity_m_s"][2], 0.0)
            self.assertAlmostEqual(result["yaw_rate_rad_s"], math.radians(15))
            old = result["velocity_m_s"]
        self.assertAlmostEqual(norm(old), 0.55)
        now += 1.55
        result = mission.update(now, observation(now, yaw=-math.pi / 2))
        self.assertAlmostEqual(result["yaw_rate_rad_s"], -math.radians(15))
        now += 0.05
        result = mission.update(now, observation(now, yaw=math.radians(359)))
        self.assertAlmostEqual(result["yaw_rate_rad_s"], 0.0, places=8)

    def test_descent_gates_and_dropout_recovery(self):
        """각 게이트에서 하강을 멈추고 짧은 손실 후에도 하강 시간을 누적하지 않는다."""
        mission = self.make_mission()
        now, result = self.enter_descent(mission)
        self.assertLess(result["velocity_m_s"][2], 0.0)
        cases = [
            {"velocity": (0.11, 0, 0)},
            {"velocity": (0, 0, 0.11)},
            {"position": (0.08, 0, -0.30)},
            {"yaw": math.radians(3)},
            {"age": 0.20},
        ]
        for case in cases:
            for _ in range(6):
                now = round(now + 0.05, 9)
                result = mission.update(now, observation(now))
            self.assertLess(result["velocity_m_s"][2], 0.0)
            # age=.2에서도 직전 프레임보다 새 관측이 되도록 최대 호출 간격 안에서 진행한다.
            now = round(now + (0.25 if "age" in case else 0.05), 9)
            result = mission.update(now, observation(now, **case))
            self.assertEqual(result["phase"], "DESCEND")
            self.assertEqual(result["velocity_m_s"][2], 0.0)
            self.assertFalse(result["done"])
        for _ in range(8):
            now = round(now + 0.05, 9)
            self.assert_stop(mission.update(now, None), "DESCEND")
        now = round(now + 0.05, 9)
        result = mission.update(now, observation(now, position=(0.0, 0.0, -0.22)))
        self.assertEqual(result["phase"], "DESCEND")
        self.assertTrue(result["start_vehicle"])
        self.assertGreaterEqual(result["velocity_m_s"][2], -0.025 - 1e-9)
        self.assertAlmostEqual(mission._descent_height_m, 0.22)
        last_good = now
        for index in range(1, 41):
            now = round(last_good + index * 0.05, 9)
            result = mission.update(now, {"valid": False})
            self.assert_stop(result)
        self.assertEqual(result["phase"], "ABORT")
        self.assertEqual(result["failure"], "observation_loss")
        self.assert_stop(mission.update(now + 0.05, observation(now + 0.05)), "ABORT")

    def test_monotonic_clock_stamps_and_hold_reset(self):
        mission = self.make_mission()
        for index in range(10):
            now = index * 0.1
            result = mission.update(now, observation(now))
        self.assertEqual(result["phase"], "ALIGN")
        self.assert_stop(mission.update(0.9, observation(0.9)), "ALIGN")
        self.assert_stop(mission.update(0.8, observation(0.8)), "ALIGN")
        # 같은 stamp는 age가 짧으면 캐시로 허용하며 실제 시간 역행은 정지한다.
        result = mission.update(1.0, observation(0.9))
        self.assertEqual(result["phase"], "ALIGN")
        self.assertFalse(result["start_vehicle"])
        self.assert_stop(mission.update(1.1, observation(0.85)), "ALIGN")
        self.assert_stop(mission.update(1.2, observation(1.3)), "ALIGN")
        self.assert_stop(mission.update(1.3, observation(1.3, age=0.5)), "ALIGN")
        bad = observation(1.4)
        bad["relative_velocity_m_s"] = None
        self.assert_stop(mission.update(1.4, bad), "ALIGN")
        bad = observation(1.5)
        bad["relative_pad_m"][0] = math.nan
        self.assert_stop(mission.update(1.5, bad), "ALIGN")
        # 시간 최댓값을 뒤로 옮기지 않고 ALIGN의 1초를 처음부터 다시 확인한다.
        for index in range(16, 26):
            now = index * 0.1
            result = mission.update(now, observation(now))
            self.assertFalse(result["start_vehicle"])
        result = mission.update(2.6, observation(2.6))
        self.assertEqual(result["phase"], "APPROACH")
        # 제어 호출의 긴 공백도 확인 시간을 초기화하며 누적 가속하지 않는다.
        self.assert_stop(mission.update(2.9, observation(2.9)), "APPROACH")
        result = mission.update(2.95, observation(2.95))
        self.assertLessEqual(norm(result["velocity_m_s"]), 0.025 + 1e-9)
        # 중복 패킷을 반복해도 장기 손실 시계가 연장되지 않는다.
        packet = observation(2.95)
        for index in range(1, 41):
            now = round(2.95 + index * 0.05, 9)
            result = mission.update(now, packet)
        self.assert_stop(result, "ABORT")
        self.assertEqual(result["failure"], "observation_loss")

    def test_cached_prediction_freshness_and_track_hold(self):
        """캐시의 예측 시각과 영상 시각을 분리하고 hold/손실 시계를 확인한다."""
        mission = self.make_mission()
        now, _ = self.enter_descent(mission)
        # 가속도 제한에 걸리지 않을 만큼 작은 X 피드백. 이미 현재인 위치는 그대로 쓴다.
        for index in range(1, 10):
            stamp = round(now + index * 0.02, 9)
            packet = observation(stamp, position=(0.005, 0, -0.30), velocity=(0.01, 0, 0))
            result = mission.update(stamp, packet)
        now = stamp
        cached = dict(packet)
        predicted_now = round(now + 0.10, 9)
        cached["prediction_stamp_s"] = predicted_now
        result = mission.update(predicted_now, cached)
        self.assertAlmostEqual(result["velocity_m_s"][0], 0.3 + 0.7 * 0.005 + 0.5 * 0.01)
        # prediction_stamp_s가 없으면 stamp_s부터 예측하며 XY 피드백에 반영된다.
        result = mission.update(round(predicted_now + 0.02, 9), packet)
        self.assertAlmostEqual(result["velocity_m_s"][0], 0.3 + 0.7 * (0.005 + 0.01 * 0.12) + 0.5 * 0.01)
        for index in range(7, 101):
            update_time = round(now + index * 0.02, 9)
            cached["prediction_stamp_s"] = update_time
            result = mission.update(update_time, cached)
            if index > 20 and index < 100:
                self.assert_stop(result, "DESCEND")
            elif index > 7 and index <= 20:
                self.assertEqual(result["velocity_m_s"][2], 0.0)
        self.assert_stop(result, "ABORT")
        self.assertEqual(result["failure"], "observation_loss")

        tracking = self.make_mission()
        for index in range(120):
            now = round(index * 0.02, 9)
            frame_stamp = round((index // 3) * 0.06, 9)  # 더 느린 영상에서도 캐시 hold 허용
            packet = observation(frame_stamp)
            packet["prediction_stamp_s"] = now
            result = tracking.update(now, packet)
        self.assertEqual(result["phase"], "TRACK")
        now = round(now + 0.02, 9)
        # 상대속도 gate가 깨지면 2초 추종 확인은 처음부터 시작한다.
        result = tracking.update(now, observation(now, velocity=(0.08, 0, 0)))
        self.assertEqual(result["phase"], "TRACK")
        for index in range(1, 102):
            update_time = round(now + index * 0.02, 9)
            result = tracking.update(update_time, observation(update_time))
            self.assertEqual(result["phase"], "TRACK" if index <= 100 else "DESCEND")

    def test_timeout_and_settle_requires_fresh_continuous_evidence(self):
        timeout = self.make_mission(mission_timeout_s=65.0)
        for index in range(651):
            now = index * 0.1
            result = timeout.update(now, observation(now, yaw=math.radians(10)))
        self.assert_stop(result, "ABORT")
        self.assertEqual(result["failure"], "mission_timeout")
        missing = self.make_mission()
        self.assert_stop(missing.update(0.0, None), "SEARCH")
        # 2초 공백 직후 도착한 새 관측으로도 장기 손실을 취소하지 않는다.
        self.assert_stop(missing.update(2.0, observation(2.0)), "ABORT")
        invalid_clock = self.make_mission()
        self.assert_stop(invalid_clock.update(math.nan, None), "ABORT")
        self.assertEqual(invalid_clock.failure, "invalid_clock")
        mission = self.make_mission()
        now, _ = self.enter_descent(mission)
        height = 0.30
        velocity_z = -0.025
        for _ in range(300):
            now = round(now + 0.05, 9)
            result = mission.update(now, observation(now, position=(0, 0, -height),
                                                    velocity=(0, 0, -velocity_z)))
            if result["phase"] == "SETTLE":
                break
            velocity_z = result["velocity_m_s"][2]
            height += velocity_z * 0.05
        else:
            self.fail("SETTLE not reached")
        for _ in range(3):
            now = round(now + 0.05, 9)
            result = mission.update(now, observation(now, position=(0, 0, -0.03)))
            self.assertFalse(result["done"])
        now = round(now + 0.05, 9)
        self.assert_stop(mission.update(now, None), "SETTLE")
        for _ in range(6):
            now = round(now + 0.05, 9)
            result = mission.update(now, observation(now, position=(0, 0, -0.03)))
            self.assertFalse(result["done"])
        now = round(now + 0.05, 9)
        result = mission.update(now, observation(now, position=(0, 0, -0.03)))
        self.assertEqual(result["phase"], "COMPLETE")
        self.assertTrue(result["done"])
        self.assertTrue(result["inferred_completion"])
        self.assertIsNone(result["failure"])


def main():
    """핵심 사례만 실행하고 실패 시 0이 아닌 종료 코드를 반환한다."""
    global CONFIG
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    if args.config is not None:
        CONFIG = json.loads(args.config.read_text())
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(LandingMissionChecks)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
