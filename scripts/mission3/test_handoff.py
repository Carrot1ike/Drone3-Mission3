"""Nonzero mission clock and sensor-only continuous hover readiness."""
from copy import deepcopy
import math
import unittest
from .controller import Mission3
from .handoff import HoverHandoff
from .settings import load,controller_settings


class Handoff(unittest.TestCase):
    def setUp(self):
        self.cfg=controller_settings(load());self.entry=self.cfg["entry"]["positions"][0]
        self.settings={"target_height_m":1.,"height_tolerance_m":.025,"position_xy_tolerance_m":.05,
                       "vertical_speed_limit_m_s":.05,"horizontal_speed_limit_m_s":.02,"tilt_limit_deg":5.,
                       "heading_tolerance_deg":3.,"stable_duration_s":2.,"max_duration_s":15.,
                       "position_gain_per_s":1.2,"velocity_limit_m_s":.3}
        self.own={"position_world_m":[*self.entry["xy_m"],1.],"velocity_world_m_s":[0.,0.,0.],
                  "rpy_rad":[0.,0.,math.radians(self.entry["heading_deg"])],
                  "heading_rad":math.radians(self.entry["heading_deg"]),"position_std_m":.1,
                  "range_accurate":True,"flow_fusion_reason":"floor_patch_clear"}

    def test_wait_search_and_timeout_start_at_handoff_not_engine_zero(self):
        mission=Mission3(self.cfg,self.entry,start_time_s=8.)
        for now in (8.,8.02,22.98):self.assertEqual(mission.update(now,self.own,None).phase,"WAIT")
        self.assertEqual(mission.update(23.,self.own,None).phase,"EXPLORE")
        self.assertEqual(mission.search_deadline_s,308.)
        self.assertEqual(mission.update(308.,self.own,None).failure,"search_timeout_without_metric_pose")
        cfg=deepcopy(self.cfg);cfg["control"]["mission_timeout_s"]=20.
        mission=Mission3(cfg,self.entry,start_time_s=8.)
        self.assertEqual(mission.update(27.98,self.own,None).failure,None)
        self.assertEqual(mission.update(28.,self.own,None).failure,"mission_timeout")
        self.assertEqual(mission.terminal_safety_reason(28.,self.own,None),"mission_timeout")

    def test_start_origin_must_be_finite_nonnegative(self):
        for origin in (-1.,float("nan"),float("inf")):
            with self.assertRaises(ValueError):Mission3(self.cfg,self.entry,origin)

    def test_stable_sensor_window_preserves_actual_own_input(self):
        handoff=HoverHandoff(self.entry,self.settings);original=deepcopy(self.own)
        self.assertFalse(handoff.update(0.,self.own)[1])
        self.assertFalse(handoff.update(1.98,self.own)[1])
        command,ready=handoff.update(2.,self.own)
        self.assertTrue(ready);self.assertEqual(command.velocity_world_m_s,[0.,0.,0.])
        self.assertEqual(self.own,original);self.assertFalse(command.motors_off)

    def test_bad_surface_motion_or_height_resets_readiness(self):
        for delta in ({"range_accurate":False},{"flow_fusion_reason":"pad_patch_only"},
                      {"velocity_world_m_s":[.021,0.,0.]},
                      {"position_world_m":[*self.entry["xy_m"],.97]},
                      {"rpy_rad":[.1,0.,0.]}):
            h=HoverHandoff(self.entry,self.settings);h.update(0.,self.own)
            bad=deepcopy(self.own);bad.update(delta)
            self.assertFalse(h.update(1.98,bad)[1]);self.assertIsNone(h.since)
            self.assertFalse(h.update(2.,self.own)[1])
            self.assertTrue(h.update(4.,self.own)[1])

    def test_failed_preroll_and_nonfinite_estimate_do_not_claim_ready(self):
        h=HoverHandoff(self.entry,self.settings)
        bad=deepcopy(self.own);bad["range_accurate"]=False
        c,ready=h.update(15.,bad)
        self.assertFalse(ready);self.assertTrue(c.done);self.assertEqual(c.failure,"handoff_preroll_not_stable")
        h=HoverHandoff(self.entry,self.settings);bad["position_world_m"][2]=float("nan")
        c,ready=h.update(0.,bad)
        self.assertFalse(ready);self.assertTrue(c.done);self.assertEqual(c.failure,"handoff_nonfinite_estimate")

    def test_brief_static_floor_quality_gap_retains_window_but_cannot_trigger_handoff(self):
        self.settings["floor_quality_grace_s"]=.1
        h=HoverHandoff(self.entry,self.settings);h.update(0.,self.own);h.update(1.9,self.own)
        gap=deepcopy(self.own);gap.update(flow_fusion_reason="floor_patch_overlap_or_invalid",
                                       surface_hypothesis="static_floor_candidate")
        self.assertFalse(h.update(2.,gap)[1]);self.assertEqual(h.since,0.)
        self.assertTrue(h.latest["floor_quality_grace_active"])
        self.assertTrue(h.update(2.02,self.own)[1])

    def test_grace_never_extends_large_gap_moving_surface_or_range_failure(self):
        for delta,stamp in (({},.12),({"surface_hypothesis":"moving_pad_candidate"},.08),
                            ({"range_accurate":False},.08),({"flow_fusion_reason":"pad_patch_only"},.08)):
            self.settings["floor_quality_grace_s"]=.1
            h=HoverHandoff(self.entry,self.settings);h.update(0.,self.own)
            gap=deepcopy(self.own);gap.update(flow_fusion_reason="floor_patch_overlap_or_invalid",
                                            surface_hypothesis="static_floor_candidate");gap.update(delta)
            self.assertFalse(h.update(stamp,gap)[1]);self.assertIsNone(h.since)

if __name__=="__main__":unittest.main()
