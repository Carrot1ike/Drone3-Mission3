"""Final support path must retain estimate-only flight safety gates."""
from copy import deepcopy
import math
import unittest
from . import test_terminal_recovery as recovery


class TerminalAdapter(unittest.TestCase):
    def setUp(self):
        fixture=recovery.TerminalRecovery();fixture.setUp()
        self.mission,self.own,self.vehicle=fixture.fixture()

    def test_supported_sensor_range_loss_is_allowed_with_bounded_state(self):
        self.own.update(flow_valid=False,range_accurate=False)
        self.assertIsNone(self.mission.terminal_safety_reason(10.5,self.own,self.vehicle))

    def test_elapsed_prediction_limit_is_not_reset_by_geometry_handoff(self):
        self.vehicle["age_s"]=.01
        self.assertEqual(self.mission.terminal_safety_reason(16.,self.own,self.vehicle),
                         "terminal_prediction_expired_before_touchdown")

    def test_final_adapter_cannot_bypass_odometry_net_or_curve_limits(self):
        for own_change,vehicle_change,reason in (
                ({"position_std_m":1.01},{},"odometry_uncertainty"),
                ({"position_world_m":[.1,6.25,.39]},{},"estimated_arena_boundary"),
                ({},{"vehicle_heading_rate_rad_s":math.radians(1.6)},"terminal_straight_unconfirmed"),
                ({},{"pad_world_m":[34.5,6.25,.3]},"terminal_straight_margin"),
                ({},{"relative_velocity_world_m_s":[float("nan"),0,0]},"terminal_state_nonfinite")):
            with self.subTest(reason=reason):
                own=deepcopy(self.own);own.update(own_change)
                state=deepcopy(self.vehicle);state.update(vehicle_change)
                failure=self.mission.terminal_safety_reason(10.5,own,state)
                self.assertEqual(failure,reason)
                command=self.mission.start_blind_recovery(failure,10.5,own,.02)
                self.assertGreaterEqual(command.velocity_world_m_s[2],0.)
                self.assertFalse(command.motors_off)
                fixture=recovery.TerminalRecovery();fixture.setUp()
                self.mission,self.own,self.vehicle=fixture.fixture()

    def test_final_adapter_rejects_bearing_only_state(self):
        self.vehicle["kind"]="coarse_image_bearing_only"
        self.assertEqual(self.mission.terminal_safety_reason(10.5,self.own,self.vehicle),
                         "terminal_state_unavailable")

    def test_optional_inertial_handoff_is_geometry_only_after_calibration(self):
        self.mission.c.update(terminal_inertial_handoff_enabled=True,
                              geometry_terminal_optical_height_m=.11)
        self.own.update(flow_fusion_usable=False,flow_fusion_reason="pad_patch_mixed_or_invalid",
                        inertial_support_handoff_ready=True)
        for i in range(11):
            command=self.mission.update(10.+i*.02,self.own,self.vehicle)
            self.assertFalse(command.motors_off)
            self.assertFalse(command.physical_success)
        self.assertEqual(command.phase,"GEOMETRY_READY")
        self.assertEqual(self.mission.support_handoff_mode,"conditional_inertial_texture_bridge")

    def test_optional_handoff_rejects_missing_calibration_and_stale_visual_fix(self):
        for calibrated,age in ((False,.4),(True,3.1)):
            with self.subTest(calibrated=calibrated,age=age):
                fixture=recovery.TerminalRecovery();fixture.setUp()
                mission,own,state=fixture.fixture()
                mission.c.update(terminal_inertial_handoff_enabled=True,
                                 geometry_terminal_optical_height_m=.11)
                own.update(flow_fusion_usable=False,flow_fusion_reason="pad_patch_mixed_or_invalid",
                           inertial_support_handoff_ready=calibrated)
                state["age_s"]=age
                for i in range(14):command=mission.update(10.+i*.02,own,state)
                self.assertEqual(command.phase,"BLIND_RECOVER")
                self.assertFalse(command.motors_off)

if __name__=="__main__":unittest.main()
