"""블라인드 종료의 명령 실행 시험. 운동 적분 fixture이며 접촉 성공 시험 아님."""
from copy import deepcopy
import json
import math
import unittest

import numpy as np

from .controller import Mission3
from .settings import ROOT, controller_settings, load


class TerminalRecovery(unittest.TestCase):
    def setUp(self):
        self.cfg = controller_settings(load())
        self.cfg["control"]["terminal_policy"] = "last_fix_bridge"
        self.entry = self.cfg["entry"]["positions"][0]

    def fixture(self, phase="BLIND_DESCEND", height=.39, speed=.37, heading=0.):
        mission = Mission3(self.cfg, self.entry)
        mission.phase = phase
        mission.discovery_time = 9.
        mission.last_good = 9.8
        mission.terminal_started = 10.
        mission.straight_start = 7.
        ff = speed * np.array([math.cos(heading), math.sin(heading), 0.])
        y = 6.25 if abs(heading) < .1 else 13.75
        own = {"position_world_m": [25., y, height],
               "velocity_world_m_s": ff.tolist(), "heading_rad": heading,
               "position_std_m": .1, "flow_fusion_usable": True,
               "flow_fusion_reason": "pad_patch_contained"}
        vehicle = {"valid": False, "age_s": .4, "capture_stamp_s": 9.6,
                   "pad_world_m": [25., y, self.cfg["landing_geometry"]["pad_height_m"]],
                   "relative_pad_world_m": [0., 0., self.cfg["landing_geometry"]["pad_height_m"]-height],
                   "relative_velocity_world_m_s": [0., 0., 0.],
                   "vehicle_velocity_world_m_s": ff.tolist(),
                   "vehicle_heading_rad": heading, "vehicle_heading_rate_rad_s": 0.,
                   "samples": 30, "last_image_quality": {"min_side_px": 70., "marker_optical_depth_m": .34}}
        mission.last_vehicle = deepcopy(vehicle)
        mission.last_v = ff.copy()
        return mission, own, vehicle

    def tick(self, mission, own, vehicle, now, advance=True):
        # Only test observations enter the controller. No scene/plant speed or curve flag.
        dt = .02 if mission.last_time is None else now-mission.last_time
        if vehicle is not None:
            vehicle["relative_pad_world_m"] = (
                np.asarray(vehicle["pad_world_m"])-own["position_world_m"]).tolist()
        command = mission.update(now, own, vehicle)
        self.assertFalse(command.motors_off)
        self.assertFalse(command.physical_success)
        self.assertTrue(np.isfinite(command.velocity_world_m_s).all())
        if advance and not command.done:
            own["position_world_m"] = (
                np.asarray(own["position_world_m"])+dt*np.asarray(command.velocity_world_m_s)).tolist()
            own["velocity_world_m_s"] = command.velocity_world_m_s.copy()
            if vehicle is not None:
                vehicle["pad_world_m"] = (
                    np.asarray(vehicle["pad_world_m"])+dt*np.asarray(vehicle["vehicle_velocity_world_m_s"])).tolist()
        return command

    def finish_recovery(self, mission, own, vehicle, start):
        commands = []
        for i in range(1, 501):
            now = start+i*.02
            if vehicle is not None:
                vehicle["age_s"] = now-9.6
            command = self.tick(mission, own, vehicle, now)
            commands.append(command)
            if command.done:
                break
            self.assertGreaterEqual(command.velocity_world_m_s[2], 0.)
            self.assertGreater(np.linalg.norm(command.velocity_world_m_s), 0.)
        self.assertTrue(commands[-1].done, "estimated climb must reach bounded height target")
        self.assertEqual(commands[-1].phase, "ABORT")
        self.assertGreaterEqual(own["position_world_m"][2], .60)
        self.assertEqual(commands[-1].velocity_world_m_s, [0., 0., 0.])
        return commands

    def test_forced_full_six_second_expiry_executes_climb_before_abort(self):
        mission, own, vehicle = self.fixture()
        # Deliberately pause descent: relative speed fails the preparation gate.
        vehicle["relative_velocity_world_m_s"] = [.09, 0., 0.]
        vehicle["capture_stamp_s"] = 10.
        for i in range(300):
            now = 10.+i*.02
            vehicle["age_s"] = now-10.
            vehicle["valid"] = vehicle["age_s"] <= self.cfg["observer"]["image_max_age_s"]
            command = self.tick(mission, own, vehicle, now)
            self.assertEqual(command.phase, "BLIND_DESCEND")
            self.assertFalse(command.done)
            self.assertAlmostEqual(command.velocity_world_m_s[2], 0.)
        command = self.tick(mission, own, vehicle, 16.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.assertFalse(command.done)
        self.assertGreater(command.velocity_world_m_s[2], 0.)
        self.assertLess(command.velocity_world_m_s[0], .37+.35*.09)
        result = self.finish_recovery(mission, own, vehicle, 16.)
        self.assertAlmostEqual(result[49].velocity_world_m_s[0], .37-.12)
        self.assertIn("BLIND_RISE", [c.phase for c in result])
        self.assertEqual(result[-1].failure, "terminal_prediction_expired")

    def test_paused_descent_short_budget_also_executes_recovery(self):
        self.cfg["control"]["terminal_max_prediction_s"] = .6
        mission, own, vehicle = self.fixture(height=.45)
        vehicle["relative_velocity_world_m_s"] = [.09, 0., 0.]
        for i in range(30):
            vehicle["age_s"] = i*.02
            self.assertFalse(self.tick(mission, own, vehicle, 10.+i*.02).done)
        command = self.tick(mission, own, vehicle, 10.6)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.finish_recovery(mission, own, None, 10.6)

    def test_fresh_images_cannot_reset_absolute_blind_budget(self):
        mission, own, vehicle = self.fixture()
        vehicle.update(valid=True, age_s=.01)
        command = self.tick(mission, own, vehicle, 16.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.assertFalse(command.done)

    def test_image_age_budget_can_expire_before_phase_budget(self):
        mission, own, vehicle = self.fixture()
        vehicle["age_s"] = 6.
        self.assertEqual(self.tick(mission, own, vehicle, 10.).phase, "BLIND_RECOVER")

    def test_missing_predicted_state_does_not_zero_low_over_pad(self):
        mission, own, _ = self.fixture()
        command = self.tick(mission, own, None, 10.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.assertFalse(command.done)
        self.finish_recovery(mission, own, None, 10.)

    def test_bad_flow_all_reasons_pause_then_execute_recovery(self):
        for reason in ("surface_unconfirmed", "floor_patch_clear", "floor_patch_overlap_or_invalid",
                       "pad_patch_mixed_or_invalid", "pad_patch_contained"):
            with self.subTest(reason=reason):
                mission, own, vehicle = self.fixture()
                own.update(flow_fusion_usable=False, flow_fusion_reason=reason)
                for i in range(13):
                    now = 10.+i*.02
                    command = self.tick(mission, own, vehicle, now)
                    self.assertEqual(command.phase, "BLIND_DESCEND")
                    self.assertAlmostEqual(command.velocity_world_m_s[2], 0.)
                command = self.tick(mission, own, vehicle, 10.26)
                self.assertEqual(command.phase, "BLIND_RECOVER")
                result = self.finish_recovery(mission, own, None, 10.26)
                self.assertEqual(result[-1].failure, "terminal_relative_flow_unusable")
                self.assertEqual(mission.events[0]["flow_fusion_reason"], reason)

    def test_usable_floor_flow_is_not_usable_relative_pad_flow(self):
        mission, own, vehicle = self.fixture()
        own["flow_fusion_reason"] = "floor_patch_clear"
        self.tick(mission, own, vehicle, 10.)
        self.assertEqual(self.tick(mission, own, vehicle, 10.26).phase, "BLIND_RECOVER")

    def test_missing_flow_contract_fails_closed(self):
        mission, own, vehicle = self.fixture()
        del own["flow_fusion_usable"]
        del own["flow_fusion_reason"]
        self.tick(mission, own, vehicle, 10.)
        self.assertEqual(self.tick(mission, own, vehicle, 10.26).phase, "BLIND_RECOVER")

    def test_brief_bad_flow_can_resume_descent_before_grace(self):
        mission, own, vehicle = self.fixture(height=.45)
        own.update(flow_fusion_usable=False, flow_fusion_reason="pad_patch_mixed_or_invalid")
        self.tick(mission, own, vehicle, 10.)
        own.update(flow_fusion_usable=True, flow_fusion_reason="pad_patch_contained")
        for i in range(1, 31):
            command = self.tick(mission, own, vehicle, 10.+i*.02)
            self.assertEqual(command.phase, "BLIND_DESCEND")
        self.assertLess(command.velocity_world_m_s[2], 0.)

    def test_edge_crossing_mixed_flow_in_approach_does_not_abort(self):
        mission, own, vehicle = self.fixture(phase="BLIND_APPROACH", height=.5)
        own.update(flow_fusion_usable=False, flow_fusion_reason="floor_patch_overlap_or_invalid")
        vehicle["pad_world_m"][0] += .08
        for i in range(15):
            command = self.tick(mission, own, vehicle, 10.+i*.02)
            self.assertEqual(command.phase, "BLIND_APPROACH")
            self.assertFalse(command.done)
        own.update(flow_fusion_usable=True, flow_fusion_reason="pad_patch_contained")
        for i in range(15, 120):
            command = self.tick(mission, own, vehicle, 10.+i*.02)
            if command.phase == "BLIND_DESCEND":
                break
        self.assertEqual(command.phase, "BLIND_DESCEND")

    def test_near_curve_bridge_entry_accounts_for_full_budget_and_hold(self):
        for heading, pad_x in ((0., 31.8), (math.pi, 18.2)):
            with self.subTest(heading=heading):
                mission, own, vehicle = self.fixture(phase="TRACK", height=.5, heading=heading)
                mission.terminal_started = None
                vehicle.update(valid=True, age_s=.05)
                vehicle["pad_world_m"][0] = pad_x
                forward = np.array([math.cos(heading), math.sin(heading), 0.])
                own["position_world_m"] = (np.asarray(vehicle["pad_world_m"])-.08*forward).tolist()
                own["position_world_m"][2] = .5
                for i in range(110):
                    command = self.tick(mission, own, vehicle, 10.+i*.02, advance=False)
                    self.assertEqual(command.phase, "TRACK")
                self.assertIsNone(mission.terminal_started)

    def test_near_curve_during_blind_executes_recovery_from_map_estimate(self):
        mission, own, vehicle = self.fixture()
        vehicle["pad_world_m"][0] = own["position_world_m"][0] = 34.
        command = self.tick(mission, own, vehicle, 10.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        result = self.finish_recovery(mission, own, None, 10.)
        self.assertEqual(result[-1].failure, "terminal_straight_margin")

    def test_estimated_turn_rate_during_blind_executes_recovery(self):
        mission, own, vehicle = self.fixture(phase="BLIND_APPROACH")
        vehicle["vehicle_heading_rate_rad_s"] = math.radians(3.)
        self.assertEqual(self.tick(mission, own, vehicle, 10.).phase, "BLIND_RECOVER")
        result = self.finish_recovery(mission, own, None, 10.)
        self.assertEqual(result[-1].failure, "terminal_straight_unconfirmed")

    def test_recovery_freezes_last_image_estimate_then_discards_cart_ff(self):
        mission, own, vehicle = self.fixture(height=.31, speed=.46, heading=math.pi)
        vehicle.update(age_s=6., vehicle_velocity_world_m_s=[-99., 0., 0.])
        command = self.tick(mission, own, vehicle, 10.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.assertGreater(command.velocity_world_m_s[0], -.46)
        self.assertLess(command.velocity_world_m_s[0], -.46+.12)
        # Stalled-climb fixture: no observation reaches safe height.
        for i in range(1, 301):
            command = self.tick(mission, own, vehicle, 10.+i*.02, advance=False)
            self.assertFalse(command.done)
            self.assertGreater(command.velocity_world_m_s[2], 0.)
            if i == 50:
                self.assertAlmostEqual(command.velocity_world_m_s[0], -.46+.12)
        self.assertEqual(command.phase, "BLIND_RISE")
        self.assertAlmostEqual(command.velocity_world_m_s[0], 0.)
        self.assertAlmostEqual(command.velocity_world_m_s[1], 0.)

    def test_bad_flow_stops_already_commanded_descent_immediately(self):
        mission, own, vehicle = self.fixture(height=.45)
        mission.last_v[2] = -.035
        own.update(flow_fusion_usable=False, flow_fusion_reason="pad_patch_mixed_or_invalid")
        command = self.tick(mission, own, vehicle, 10.)
        self.assertEqual(command.phase, "BLIND_DESCEND")
        self.assertEqual(command.velocity_world_m_s[2], 0.)
        self.assertEqual(self.tick(mission, own, vehicle, 10.26).phase, "BLIND_RECOVER")

    def test_optional_flow_loss_brake_keeps_3d_command_slew_and_stops(self):
        self.cfg["control"].update(terminal_flow_loss_slew_brake=True,
                                   max_accel_m_s2=.15, max_speed_m_s=.4,
                                   max_descent_speed_m_s=.06, max_climb_speed_m_s=.1)
        mission, own, vehicle = self.fixture(height=.45, speed=.25)
        mission.last_v[2]=-.035
        own.update(flow_fusion_usable=False,flow_fusion_reason="pad_patch_mixed_or_invalid")
        previous=mission.last_v.copy()
        descent=0.
        for i in range(13):
            command=self.tick(mission,own,vehicle,10.+i*.02,advance=False)
            velocity=np.asarray(command.velocity_world_m_s)
            self.assertEqual(command.phase,"BLIND_DESCEND")
            self.assertLessEqual(np.linalg.norm(velocity-previous),.15*.02+1e-10)
            self.assertGreaterEqual(velocity[2],previous[2]-1e-12)
            self.assertLessEqual(velocity[2],1e-12)
            descent+=max(0.,-velocity[2])*.02
            previous=velocity
        self.assertAlmostEqual(previous[2],0.,places=12)
        self.assertLessEqual(descent,.035**2/(2*.15)+1e-12)
        self.assertEqual(self.tick(mission,own,vehicle,10.26,advance=False).phase,"BLIND_RECOVER")

    def test_optional_flow_loss_brake_transition_from_approach_slews(self):
        self.cfg["control"].update(terminal_flow_loss_slew_brake=True,
                                   max_accel_m_s2=.15, max_speed_m_s=.4,
                                   max_descent_speed_m_s=.06, max_climb_speed_m_s=.1)
        mission, own, vehicle=self.fixture(phase="BLIND_APPROACH",height=.45,speed=.25)
        own.update(flow_fusion_usable=False,flow_fusion_reason="pad_patch_mixed_or_invalid")
        # The .2s stable gate was already accumulating in the prior phase.
        mission.hold_start=9.7
        mission.last_v=np.array([.25407998339215293,-.002699924332796092,-.006272513344269041])
        previous=mission.last_v.copy()
        command=self.tick(mission,own,vehicle,10.,advance=False)
        self.assertEqual(command.phase,"BLIND_DESCEND")
        velocity=np.asarray(command.velocity_world_m_s)
        self.assertLessEqual(np.linalg.norm(velocity-previous),.15*.02+1e-10)
        self.assertLess(velocity[2],0.)

    def test_expiry_from_descent_commands_positive_rise_on_first_tick(self):
        mission, own, vehicle = self.fixture()
        mission.last_v[:] = [0., 0., -.035]
        vehicle["age_s"] = 6.
        command = self.tick(mission, own, vehicle, 10.)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.assertGreater(command.velocity_world_m_s[2], 0.)
        self.finish_recovery(mission, own, None, 10.)

    def test_fresh_image_during_recovery_cannot_restart_unsafe_descent(self):
        mission, own, vehicle = self.fixture()
        self.tick(mission, own, None, 10.)
        vehicle.update(valid=True, age_s=.01)
        command = self.tick(mission, own, vehicle, 10.02)
        self.assertEqual(command.phase, "BLIND_RECOVER")
        self.finish_recovery(mission, own, None, 10.02)

    def test_mission_timeout_during_blind_must_first_climb(self):
        self.cfg["control"]["mission_timeout_s"] = 10.
        mission, own, vehicle = self.fixture()
        self.assertEqual(self.tick(mission, own, vehicle, 10.).phase, "BLIND_RECOVER")
        result = self.finish_recovery(mission, own, None, 10.)
        self.assertEqual(result[-1].failure, "mission_timeout")

    def test_nondefault_pad_height_controls_safe_climb_criterion(self):
        self.cfg["landing_geometry"]["pad_height_m"] = .45
        mission, own, vehicle = self.fixture(height=.50)
        vehicle["age_s"] = 6.
        self.tick(mission, own, vehicle, 10.)
        self.finish_recovery(mission, own, None, 10.)
        self.assertGreaterEqual(own["position_world_m"][2], .75)

    def test_conditional_lower_height_reaches_blind_without_climbing_back_to_default(self):
        profile = json.loads((ROOT/"config/mission3-physical-sensitivity.json").read_text())
        self.assertEqual(self.cfg["control"]["terminal_max_prediction_s"], 6.)
        self.cfg["control"].update(profile["controller_software"])
        mission, own, vehicle = self.fixture(phase="LOWER_ALIGN", height=.40)
        vehicle.update(valid=True, age_s=.05)
        vehicle["pad_world_m"][0] += .65
        for i in range(55):
            command = self.tick(mission, own, vehicle, 10.+i*.02, advance=False)
        self.assertEqual(command.phase, "APPROACH")
        vehicle["pad_world_m"][0] = own["position_world_m"][0]+.08
        for i in range(55, 180):
            command = self.tick(mission, own, vehicle, 10.+i*.02, advance=False)
            self.assertAlmostEqual(command.velocity_world_m_s[2], 0.)
            if command.phase == "BLIND_APPROACH":
                break
        self.assertEqual(command.phase, "BLIND_APPROACH")
        self.assertEqual(self.cfg["control"]["terminal_max_prediction_s"], 12.)
        # Physical near-pad handoff is above PMW3901's component working
        # envelope; precise Flow/ToF cannot be assumed at foot contact.
        self.assertGreater(self.cfg["control"]["geometry_terminal_optical_height_m"], .08)

    def test_nondefault_pad_height_geometry_completion_uses_public_height(self):
        self.cfg["landing_geometry"]["pad_height_m"] = .45
        mission, own, vehicle = self.fixture(height=.53)
        for i in range(40):
            command = self.tick(mission, own, vehicle, 10.+i*.02, advance=False)
            if command.done:
                break
        self.assertEqual(command.phase, "GEOMETRY_READY")


if __name__ == "__main__":
    unittest.main()
