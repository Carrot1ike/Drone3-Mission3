"""정적 관측점·영상 획득·유한 시간 탐색 계약. 물리 착륙 시험 아님."""
from copy import deepcopy
import math
import unittest

import cv2
import numpy as np

from .controller import Mission3
from .estimator import SelfMotion
from .scene import R_HEADING_CAMERA, R_VEHICLE_MARKER
from .settings import controller_settings, load
from .track import rotation, wrap
from .vision import Observer


class Search(unittest.TestCase):
    def setUp(self):
        self.full = load()
        self.cfg = controller_settings(self.full)
        self.entry = self.cfg["entry"]["positions"][0]

    def own(self, mission, pos=None, yaw=None):
        return {"position_world_m": (mission.search_target.copy() if pos is None else np.asarray(pos)).tolist(),
                "velocity_world_m_s": [0.,0.,0.], "position_std_m": .1,
                "heading_rad": mission.search_gaze if yaw is None else yaw}

    def watching(self, variant="fixed_point", entry=None):
        cfg = deepcopy(self.cfg)
        cfg["control"]["search_variant"] = variant
        mission = Mission3(cfg, entry or self.entry)
        mission.enter("WATCH", 20., "search_fixture")
        return mission

    def projection(self, position, heading, pad):
        camera_f = 160./math.tan(math.radians(self.cfg["camera"]["vertical_fov_deg"])/2)
        K = np.array([[camera_f,0.,160.],[0.,camera_f,160.],[0.,0.,1.]])
        side = self.cfg["marker"]["code_side_m"]/2
        points = np.array([[-side,side,0.],[side,side,0.],[side,-side,0.],[-side,-side,0.]])
        marker = np.asarray(pad)+self.cfg["marker"]["known_marker_to_pad_vehicle_m"]
        rcm = R_HEADING_CAMERA.T@rotation(heading).T@R_VEHICLE_MARKER
        tcm = R_HEADING_CAMERA.T@rotation(heading).T@(marker-position)
        corners = cv2.projectPoints(points,cv2.Rodrigues(rcm)[0],tcm,K,np.zeros(5))[0].reshape(4,2)
        marker_image = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),0,180)
        transform = cv2.getPerspectiveTransform(np.float32([[0,0],[179,0],[179,179],[0,179]]),np.float32(corners))
        gray = cv2.warpPerspective(marker_image,transform,(320,320),flags=cv2.INTER_NEAREST,borderValue=255)
        return corners, gray

    def test_default_station_and_inward_gaze_are_static_map_priors(self):
        lower = Mission3(self.cfg,self.entry)
        upper = Mission3(self.cfg,self.cfg["entry"]["positions"][1])
        np.testing.assert_allclose(lower.search_target,[17.,5.9,.70])
        np.testing.assert_allclose(upper.search_target,[33.,14.1,.70])
        self.assertAlmostEqual(lower.search_gaze,math.radians(20.))
        self.assertAlmostEqual(wrap(upper.search_gaze-math.pi),math.radians(20.))
        self.assertFalse(lower.search_scan_enabled)

    def test_nearest_static_station_shortens_unobserved_far_entry_travel(self):
        cfg = deepcopy(self.cfg)
        cfg["control"]["search_station_policy"] = "nearest_static"
        for entry_id, target_x in ((1,17.),(2,17.),(3,17.),(4,33.),
                                   (5,33.),(6,33.),(7,33.),(8,17.)):
            entry = cfg["entry"]["positions"][entry_id-1]
            near = Mission3(cfg,entry)
            old = Mission3(self.cfg,entry)
            self.assertEqual(near.search_target[0],target_x)
            self.assertLessEqual(np.linalg.norm(near.search_target[:2]-entry["xy_m"]),
                                 np.linalg.norm(old.search_target[:2]-entry["xy_m"]))
            self.assertEqual(near.c["odometry_uncertainty_limit_m"],1.)
            self.assertFalse(near.search_scan_enabled)
        changed = deepcopy(self.full)
        changed["control"]["search_station_policy"] = "nearest_static"
        changed["vehicle_plant_only"]["speed_m_s"] = 9.
        np.testing.assert_equal(Mission3(controller_settings(changed),cfg["entry"]["positions"][1]).search_target,
                                Mission3(cfg,cfg["entry"]["positions"][1]).search_target)

    def test_unknown_static_station_policy_is_rejected(self):
        self.cfg["control"]["search_station_policy"] = "live_vehicle_position"
        with self.assertRaises(ValueError):
            Mission3(self.cfg,self.entry)

    def test_entry_near_downstream_end_still_selects_upstream_station(self):
        entry = {"xy_m":[49.,2.],"heading_deg":180.}
        fixed = self.watching(entry=entry)
        legacy = self.watching("legacy_sweep",entry)
        self.assertEqual(fixed.search_target[0],17.)
        self.assertEqual(legacy.search_target[0],33.)

    def test_fixed_watch_never_commands_lateral_or_forward_sine_motion(self):
        mission = self.watching()
        own = self.own(mission)
        for i in range(400):
            now = 20.+i*.02
            command = mission.update(now,own,None)
            self.assertEqual(command.phase,"WATCH")
            self.assertEqual(command.velocity_world_m_s,[0.,0.,0.])
            self.assertLessEqual(abs(command.yaw_rate_rad_s),1.2*math.radians(5.)+1e-9)
            self.assertIsNone(mission.discovery_time)

    def test_legacy_sweep_remains_explicit_comparison_option(self):
        mission = self.watching("legacy_sweep")
        own = self.own(mission,yaw=0.)
        mission.last_time = 33.98
        command = mission.update(34.,own,None)
        self.assertGreater(command.velocity_world_m_s[1],0.)
        self.assertTrue(mission.search_scan_enabled)
        self.assertIsNone(mission.discovery_time)

    def test_fifteen_second_wait_has_no_scheduled_motion(self):
        mission = Mission3(self.cfg,self.entry)
        own = self.own(mission,[1.,2.,1.],0.)
        for now in (0.,1.,14.98):
            command = mission.update(now,own,None)
            self.assertEqual(command.phase,"WAIT")
            self.assertEqual(command.velocity_world_m_s,[0.,0.,0.])
            self.assertEqual(command.yaw_rate_rad_s,0.)
        self.assertEqual(mission.update(15.,own,None).phase,"EXPLORE")

    def test_explore_goes_directly_to_watch_without_full_scan(self):
        mission = Mission3(self.cfg,self.entry)
        mission.enter("EXPLORE",15.,"search_fixture")
        command = mission.update(40.,self.own(mission),None)
        self.assertEqual(command.phase,"WATCH")
        self.assertEqual(command.yaw_rate_rad_s,0.)
        self.assertEqual(mission.events[-1]["reason"],"static_observation_point_reached")
        self.assertIsNone(mission.discovery_time)

    def test_scan_can_be_enabled_for_bounded_diagnostic_only(self):
        self.cfg["control"]["search_scan_enabled"] = True
        mission = Mission3(self.cfg,self.entry)
        mission.enter("EXPLORE",15.,"search_fixture")
        own = self.own(mission)
        self.assertEqual(mission.update(40.,own,None).phase,"SCAN")
        self.assertEqual(mission.update(64.02,own,None).phase,"WATCH")
        self.assertIsNone(mission.discovery_time)

    def test_search_deadline_does_not_extend_mission_or_uncertainty_budget(self):
        mission = self.watching()
        self.assertEqual(mission.search_deadline_s,300.)
        self.assertEqual(mission.c["mission_timeout_s"],360.)
        self.assertEqual(mission.c["odometry_uncertainty_limit_m"],1.)
        command = mission.update(300.,self.own(mission),None)
        self.assertTrue(command.done)
        self.assertEqual(command.failure,"search_timeout_without_metric_pose")
        self.cfg["control"].update(search_timeout_s=600.,mission_timeout_s=90.)
        mission = Mission3(self.cfg,self.entry)
        self.assertEqual(mission.search_deadline_s,90.)

    def test_optional_shorter_search_budget_applies_to_coarse_only_guidance(self):
        self.cfg["control"]["search_timeout_s"] = 45.
        mission = Mission3(self.cfg,self.entry)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.1,"bearing_world_rad":.2}
        own = self.own(mission)
        self.assertEqual(mission.update(44.,own,coarse).phase,"COARSE_ACQUIRE")
        command = mission.update(45.,own,coarse)
        self.assertTrue(command.done)
        self.assertEqual(command.failure,"search_timeout_without_metric_pose")
        self.assertIsNone(mission.discovery_time)

    def test_fixed_station_keeps_near_and_coarse_marker_corners_in_view(self):
        mission = Mission3(self.cfg,self.entry)
        for along in (.5,.8,1.2,1.8):
            for gaze_deg in (15.,20.,25.):
                with self.subTest(along=along,gaze_deg=gaze_deg):
                    pad = [mission.search_target[0]+along,6.25,.30]
                    corners,_ = self.projection(mission.search_target,math.radians(gaze_deg),pad)
                    self.assertTrue(np.all(corners>0.) and np.all(corners<320.))

    def test_large_lateral_sweep_can_leave_same_candidate_outside_viewport(self):
        mission = Mission3(self.cfg,self.entry)
        pad = [17.8,6.25,.3]
        fixed,_ = self.projection(mission.search_target,mission.search_gaze,pad)
        # Legacy t_watch+14s: y += 2m, gaze sine is zero. Static candidate only.
        legacy_position = mission.search_target+np.array([0.,2.,0.])
        legacy,_ = self.projection(legacy_position,0.,pad)
        self.assertTrue(np.all(fixed>0.) and np.all(fixed<320.))
        self.assertFalse(np.all(legacy>0.) and np.all(legacy<320.))

    def test_native_coarse_images_trigger_approach_without_metric_ff(self):
        mission = self.watching()
        own = SelfMotion(self.cfg,self.entry)
        own.position = mission.search_target.copy()
        own.heading = mission.search_gaze
        own.history.extend((t,own.position.copy(),own.heading) for t in (0.,.05))
        observer = Observer(self.cfg,own)
        pad = own.position.copy()+[1.8,.35,0.]
        pad[2] = .3
        _,gray = self.projection(own.position,own.heading,pad)
        first = observer.frame(gray,0.,.05)
        second = observer.frame(gray,.05,.10)
        self.assertEqual(first["reason"],"marker_too_small",first)
        self.assertTrue(second.get("coarse_guidance_available"),second)
        state = observer.state(.10)
        command = mission.update(20.,self.own(mission),state)
        self.assertEqual(command.phase,"COARSE_ACQUIRE")
        self.assertGreater(np.linalg.norm(command.velocity_world_m_s[:2]),0.)
        self.assertIsNone(mission.discovery_time)
        self.assertIsNone(mission.last_vehicle)
        self.assertFalse(command.motors_off)
        self.assertFalse(command.physical_success)

    def test_private_vehicle_speed_and_seed_cannot_choose_station(self):
        other = deepcopy(self.full)
        other["vehicle_plant_only"].update(speed_m_s=99.,seed=91,phase=4.)
        mission = Mission3(controller_settings(other),self.entry)
        reference = Mission3(self.cfg,self.entry)
        np.testing.assert_equal(mission.search_target,reference.search_target)
        self.assertEqual(mission.search_gaze,reference.search_gaze)
        self.assertEqual(mission.search_deadline_s,reference.search_deadline_s)
        with self.assertRaises(ValueError):
            Mission3(other,self.entry)

    def test_coarse_turn_can_finish_and_approach_after_images_briefly_disappear(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.1,
                  "bearing_world_rad":math.radians(40.)}
        command = mission.update(20.,own,coarse)
        self.assertEqual(command.velocity_world_m_s,[0.,0.,0.])
        for i in range(1,151):
            now = 20.+i*.02
            command = mission.update(now,own,None)
            own["heading_rad"] = wrap(own["heading_rad"]+command.yaw_rate_rad_s*.02)
            own["position_world_m"] = (np.asarray(own["position_world_m"])+.02*np.asarray(command.velocity_world_m_s)).tolist()
            self.assertEqual(command.phase,"COARSE_ACQUIRE")
            self.assertIsNone(mission.last_vehicle)
            self.assertIsNone(mission.discovery_time)
        self.assertGreater(np.linalg.norm(command.velocity_world_m_s[:2]),0.)
        command = mission.update(25.,own,None)
        self.assertEqual(command.phase,"WATCH")

    def test_coarse_motion_has_finite_own_odometry_distance_even_with_repeated_bearings(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        origin = np.asarray(own["position_world_m"]).copy()
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.1,"bearing_world_rad":0.}
        for i in range(1000):
            command = mission.update(20.+i*.02,own,coarse)
            own["position_world_m"] = (np.asarray(own["position_world_m"])+.02*np.asarray(command.velocity_world_m_s)).tolist()
            self.assertIsNone(mission.discovery_time)
            self.assertIsNone(mission.last_vehicle)
        self.assertLessEqual(np.linalg.norm(np.asarray(own["position_world_m"])[:2]-origin[:2]),1.501)
        self.assertLess(np.linalg.norm(command.velocity_world_m_s[:2]),1e-5)

    def test_aged_coarse_state_does_not_refresh_image_hold_clock(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.1,"bearing_world_rad":0.}
        mission.update(20.,own,coarse)
        coarse["age_s"] = .6
        mission.update(20.5,own,coarse)
        self.assertAlmostEqual(mission.coarse_last,19.9)
        self.assertEqual(mission.update(24.92,own,None).phase,"WATCH")

    def test_fresh_pixel_guard_can_approach_beyond_base_path_but_never_total_limit(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                  "bearing_world_rad":0.,"min_side_px":9.,"max_side_px":12.}
        for i in range(951):
            command = mission.update(20.+i*.02,own,coarse)
            own["position_world_m"] = (np.asarray(own["position_world_m"])+.02*np.asarray(command.velocity_world_m_s)).tolist()
            self.assertIsNone(mission.last_vehicle)
            self.assertIsNone(mission.discovery_time)
        self.assertGreater(mission.coarse_total_distance_m,1.5)
        self.assertLessEqual(mission.coarse_total_distance_m,4.001)
        self.assertEqual(command.coarse_guard["policy"],"fresh_pixel_extension")
        self.assertLess(command.coarse_guard["desired_forward_m_s"],.001)

    def test_extension_stops_immediately_when_image_size_quality_is_stale(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                  "bearing_world_rad":0.,"min_side_px":9.,"max_side_px":12.}
        for i in range(251):
            command = mission.update(20.+i*.02,own,coarse)
            own["position_world_m"] = (np.asarray(own["position_world_m"])+.02*np.asarray(command.velocity_world_m_s)).tolist()
        self.assertGreater(mission.coarse_total_distance_m,1.5)
        command = mission.update(25.1,own,None)
        self.assertEqual(command.coarse_guard["desired_forward_m_s"],0.)
        self.assertEqual(command.coarse_guard["policy"],"extension_requires_fresh_pixels")
        self.assertEqual(command.phase,"COARSE_ACQUIRE")
        self.assertEqual(mission.update(30.,own,None).phase,"WATCH")

    def test_missing_max_side_and_aged_pixels_cannot_extend_base_budget(self):
        for variant in ({"min_side_px":9.}, {"min_side_px":9.,"max_side_px":12.,"age_s":.3},
                        {"min_side_px":19.,"max_side_px":24.}, {"min_side_px":9.,"max_side_px":float("nan")}):
            with self.subTest(variant=variant):
                mission = self.watching()
                own = self.own(mission,yaw=0.)
                coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                          "bearing_world_rad":0.,**variant}
                mission.update(20.,own,coarse)
                mission.coarse_distance_m=mission.coarse_total_distance_m=1.5
                command = mission.update(20.02,own,coarse)
                self.assertEqual(command.coarse_guard["desired_forward_m_s"],0.)

    def test_max_side_pixel_proximity_blocks_motion_even_when_min_side_is_coarse(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                  "bearing_world_rad":0.,"min_side_px":9.,"max_side_px":60.}
        command = mission.update(20.,own,coarse)
        self.assertEqual(command.coarse_guard["policy"],"pixel_proximity_stop")
        self.assertEqual(command.coarse_guard["desired_forward_m_s"],0.)
        self.assertEqual(command.velocity_world_m_s,[0.,0.,0.])
        command = mission.update(20.2,own,None)
        self.assertEqual(command.coarse_guard["policy"],"pixel_proximity_stop")

    def test_total_path_budget_cannot_reset_after_coarse_loss_and_new_id(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                  "bearing_world_rad":0.,"min_side_px":9.,"max_side_px":12.}
        mission.update(20.,own,coarse)
        mission.coarse_total_distance_m=4.
        self.assertEqual(mission.update(25.,own,None).phase,"WATCH")
        command = mission.update(25.02,own,coarse)
        self.assertEqual(command.phase,"COARSE_ACQUIRE")
        self.assertEqual(command.coarse_guard["policy"],"total_path_exhausted")
        self.assertEqual(command.coarse_guard["desired_forward_m_s"],0.)

    def test_fresh_pixel_images_cannot_make_episode_motion_unlimited(self):
        mission = self.watching()
        own = self.own(mission,yaw=0.)
        coarse = {"kind":"coarse_image_bearing_only","valid":True,"age_s":.06,
                  "bearing_world_rad":0.,"min_side_px":9.,"max_side_px":12.}
        mission.update(20.,own,coarse)
        command = mission.update(40.,own,coarse)
        self.assertEqual(command.coarse_guard["policy"],"episode_motion_timeout")
        self.assertEqual(command.coarse_guard["desired_forward_m_s"],0.)

    def test_unknown_search_variant_is_rejected(self):
        self.cfg["control"]["search_variant"] = "live_vehicle_phase"
        with self.assertRaises(ValueError):
            Mission3(self.cfg,self.entry)


if __name__ == "__main__":
    unittest.main()
