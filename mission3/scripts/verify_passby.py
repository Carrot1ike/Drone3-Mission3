#!/usr/bin/env python3
"""사용자 그림의 대기·지나가는 차량·일직선 정렬 제어 계약 검사."""
import json
import math
import unittest
from landing_mission import LandingMission
from optical_landing_scenarios import configure_scenario
from provided_hardware import ROOT


def config():
    return configure_scenario(json.loads((ROOT / "config/moving-landing.json").read_text()), "passby70")


def state(t, x=-.30, y=.35, heading=None):
    if heading is None:
        heading = -math.atan2(y, x+.45)
    return {"valid": True, "stamp_s": t, "prediction_stamp_s": t,
        "relative_pad_m": [x, y, -.30], "relative_velocity_m_s": [0., 0., 0.], "heading_error_rad": heading}


class PassbyChecks(unittest.TestCase):
    def test_fixed_wait_has_no_scan_and_no_vehicle_start_request(self):
        c = config()
        self.assertEqual(c["vehicle"]["motion_mode"], "continuous")
        p = c["scenario"]["rear_reference_initial_world_m"]
        bearing = math.atan2(p[1]-c["spawn"]["y_m"], p[0]-c["spawn"]["x_m"])
        self.assertAlmostEqual(abs(math.degrees(math.radians(c["spawn"]["yaw_deg"])-bearing)), 70.)
        mission = LandingMission(c)
        for i in range(151):
            command = mission.update(i*.02, None)
            self.assertEqual(command["phase"], "WAIT")
            self.assertEqual(command["velocity_m_s"], [0., 0., 0.])
            self.assertEqual(command["yaw_rate_rad_s"], 0.)
            self.assertFalse(command["start_vehicle"])
        found = mission.update(3.02, state(3.02))
        self.assertEqual(found["phase"], "ACQUIRE")
        self.assertFalse(found["start_vehicle"])

    def test_lineup_requires_behind_distance_before_approach(self):
        mission = LandingMission(config())
        phases = []
        for i in range(151):
            t = i*.02
            # 마커 중심·yaw·횡오차가 맞아도 차량 앞쪽이면 접근을 금지.
            command = mission.update(t, state(t, x=-.30, y=0., heading=0.))
            if not phases or phases[-1] != command["phase"]:
                phases.append(command["phase"])
        self.assertEqual(phases, ["ACQUIRE", "LINEUP"])
        for i in range(1, 70):
            t = 3.+i*.02
            command = mission.update(t, state(t, x=.35, y=0., heading=0.))
        self.assertEqual(command["phase"], "APPROACH")
        self.assertFalse(command["start_vehicle"])
        self.assertGreater(command["velocity_m_s"][0], .3)

    def test_lineup_steers_toward_marker_during_lateral_motion(self):
        mission = LandingMission(config())
        for i in range(25):
            t = i*.02
            command = mission.update(t, state(t, x=.35))
        self.assertEqual(command["phase"], "LINEUP")
        t += .02
        # 차량 heading은 오른쪽에 있어도 영상 마커는 왼쪽에 남은 상황.
        command = mission.update(t, state(t, x=.35, heading=-math.radians(10.)))
        self.assertGreater(command["yaw_rate_rad_s"], 0.)

    def test_wait_timeout_does_not_imply_success(self):
        c = config()
        c["search"]["timeout_s"] = .5
        mission = LandingMission(c)
        for i in range(27):
            command = mission.update(i*.02, None)
        self.assertEqual(command["phase"], "ABORT")
        self.assertEqual(command["failure"], "wait_timeout")
        self.assertFalse(command["inferred_completion"])
        self.assertEqual(command["velocity_m_s"], [0., 0., 0.])
        self.assertEqual(command["yaw_rate_rad_s"], 0.)

    def test_post_detection_long_loss_aborts(self):
        mission = LandingMission(config())
        mission.update(0., state(0.))
        for i in range(1, 103):
            command = mission.update(i*.02, None)
        self.assertEqual(command["phase"], "ABORT")
        self.assertEqual(command["failure"], "observation_loss")


if __name__ == "__main__":
    unittest.main(verbosity=2)
