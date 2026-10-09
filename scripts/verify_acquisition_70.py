#!/usr/bin/env python3
"""70도 회전 탐색/발견 후 정렬/미발견·손실의 제어 계약 검사."""
import copy
import json
import math
from pathlib import Path
import unittest
from landing_mission import LandingMission
from optical_landing_scenarios import configure_scenario
from provided_hardware import ROOT


def configuration():
    return configure_scenario(json.loads((ROOT / "config/moving-landing.json").read_text()), "rear-bearing70")


def observation(t, y=.0, heading=0.):
    return {"valid": True, "stamp_s": t, "prediction_stamp_s": t,
            "relative_pad_m": [.35, y, -.30], "relative_velocity_m_s": [0., 0., 0.],
            "heading_error_rad": heading}


class AcquisitionChecks(unittest.TestCase):
    def test_smaller_angle_and_unobserved_scan(self):
        config = configuration()
        point = config["scenario"]["rear_reference_initial_world_m"]
        line = math.atan2(point[1]-config["spawn"]["y_m"], point[0]-config["spawn"].get("x_m", 0.))
        delta = (math.radians(config["spawn"]["yaw_deg"])-line+math.pi)%(2*math.pi)-math.pi
        self.assertAlmostEqual(abs(math.degrees(delta)), 70.)
        mission = LandingMission(config)
        for i in range(151):
            result = mission.update(i*.02, None)
            self.assertEqual(result["phase"], "SEARCH")
            self.assertEqual(result["velocity_m_s"], [0., 0., 0.])
            self.assertAlmostEqual(result["yaw_rate_rad_s"], -math.radians(12.))
            self.assertFalse(result["start_vehicle"])
        discovered = mission.update(3.02, observation(3.02, heading=-.5))
        self.assertEqual(discovered["phase"], "ACQUIRE")
        self.assertTrue(discovered["start_vehicle"])
        self.assertLess(discovered["yaw_rate_rad_s"], 0.)

    def test_lineup_before_approach_and_post_discovery_loss(self):
        mission = LandingMission(configuration())
        phases = []
        t = 0.
        for i in range(300):
            t = i*.02
            # Farther lateral position prevents LINEUP even with aligned yaw.
            y = .02 if mission.phase == "LINEUP" else 0.
            command = mission.update(t, observation(t, y=y))
            if not phases or phases[-1] != command["phase"]:
                phases.append(command["phase"])
            if command["phase"] == "LINEUP" and t > 3.:
                break
        self.assertEqual(phases, ["ACQUIRE", "ALIGN", "LINEUP"])
        self.assertLess(command["velocity_m_s"][1], .02)
        for i in range(1, 70):
            command = mission.update(t+i*.02, observation(t+i*.02))
        self.assertIn(command["phase"], ("APPROACH", "TRACK"))
        last = t+69*.02
        for i in range(1, 103):
            command = mission.update(last+i*.02, None)
        self.assertEqual(command["phase"], "ABORT")
        self.assertEqual(command["failure"], "observation_loss")
        self.assertEqual(command["yaw_rate_rad_s"], 0.)

    def test_no_marker_timeout_has_no_translation_or_vehicle_motion(self):
        config = configuration()
        config["search"]["timeout_s"] = .50
        mission = LandingMission(config)
        for i in range(27):
            command = mission.update(i*.02, None)
            self.assertEqual(command["velocity_m_s"], [0., 0., 0.])
            self.assertFalse(command["start_vehicle"])
        self.assertEqual(command["phase"], "ABORT")
        self.assertEqual(command["failure"], "search_timeout")
        self.assertFalse(command["inferred_completion"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
