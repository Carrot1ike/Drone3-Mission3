import math
import tempfile
import unittest
import numpy as np
from .track import make_track, Stadium
from .vehicle_motion import VehicleMotion
from .passage import PassageStop
from .scene import R_HEADING_CAMERA
from .settings import load, controller_settings, ROOT
from .controller import Mission3


class StraightScenarioTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load(ROOT/'config/mission3-30x5-straight-v1.json')
        self.track=make_track(self.cfg['track'])

    def test_finite_track_does_not_wrap_and_margin_uses_estimate(self):
        np.testing.assert_array_equal(self.track.pose(100)[0],[29,2.5])
        np.testing.assert_array_equal(self.track.pose(-1)[0],[1,2.5])
        self.assertTrue(self.track.safe_straight_margin([4,2.5],0,6))
        self.assertFalse(self.track.safe_straight_margin([27,2.5],0,2))
        self.assertFalse(self.track.safe_straight_margin([4,2.5],math.pi,6))
        self.assertFalse(self.track.safe_straight_margin([4,4.5],0,6))

    def test_waiting_trigger_and_exact_stop(self):
        motion=VehicleMotion(self.cfg['vehicle_plant_only'],self.track,0.)
        self.assertEqual(motion.sample(1000),(0.,0.,'waiting_for_hover'))
        self.assertTrue(motion.start(3.));self.assertFalse(motion.start(7.))
        self.assertAlmostEqual(motion.duration,95+1/3)
        self.assertEqual(motion.sample(200),(28.,0.,'stopped_at_endpoint'))
        self.assertEqual(motion.start_time,3.)

    def test_velocity_is_derivative_of_position_across_both_ramps(self):
        m=VehicleMotion(self.cfg['vehicle_plant_only'],self.track,0.);m.start(2.)
        for elapsed in (.1,1.,2.,20.,m.duration-2.,m.duration-1.,m.duration):
            t=2.+elapsed;h=1e-5
            derivative=(m.sample(t+h)[0]-m.sample(t-h)[0])/(2*h)
            self.assertAlmostEqual(derivative,m.sample(t)[1],delta=2e-6)
        for t in np.linspace(2,2+m.duration,100):
            self.assertLessEqual(m.sample(t)[1],.3+1e-10)

    def test_historical_motion_is_unchanged(self):
        cfg=load();track=Stadium.from_config(cfg['track'])
        motion=VehicleMotion(cfg['vehicle_plant_only'],track,24.)
        for t in (0.,.02,50.,360.):
            arc,v,_=motion.sample(t)
            self.assertEqual(arc,24.+cfg['vehicle_plant_only']['speed_m_s']*t)
            self.assertEqual(v,cfg['vehicle_plant_only']['speed_m_s'])

    def test_no_yaw_search_or_translation_after_15_seconds_without_image(self):
        cfg=controller_settings(self.cfg);entry=cfg['entry']['positions'][0]
        controller=Mission3(cfg,entry)
        own={'position_world_m':[2.,1.25,1.], 'velocity_world_m_s':[0.,0.,0.],
             'heading_rad':math.radians(70), 'position_std_m':.1}
        # Use real existing own-state contract from the sensor estimator.
        from .estimator import SelfMotion
        motion=SelfMotion(cfg,entry)
        for t in np.arange(0.,50.01,.02):
            own=motion.update({'stamp_s':t,'gyro_z_rad_s':0.,'accel_heading_m_s2':[0.,0.,0.],
                              'range_m':1.,'range_valid':True,'range_accurate':True,
                              'flow_angular_rad_s':[0.,0.],'flow_valid':True})
            own['position_world_m']=[2.,1.25,1.];own['heading_rad']=math.radians(70)
            own['position_std_m']=.1
            result=controller.update(t,own,None)
            self.assertEqual(result.phase,'WAIT')
            self.assertAlmostEqual(result.yaw_rate_rad_s,0.)
            np.testing.assert_allclose(result.velocity_world_m_s,[0.,0.,0.],atol=1e-10)

    def test_live_settings_exclude_departure_and_nominal_speed(self):
        public=controller_settings(self.cfg)
        self.assertNotIn('vehicle_plant_only',public)
        self.assertNotIn('distance_sweep',public)
        self.assertNotIn('trial_termination',public)

    def test_passage_needs_entire_board_exit_and_transit_grace(self):
        plant=type('Fixture',(),{})()
        plant.physical={'mounts':{'camera_position_body_m':[0.,0.,0.]}}
        plant.position=np.array([2.,1.25,1.]);plant.rotation=np.array([
            [math.cos(math.radians(70)),-math.sin(math.radians(70)),0.],
            [math.sin(math.radians(70)),math.cos(math.radians(70)),0.],[0.,0.,1.]])
        plant.camera_rotation=R_HEADING_CAMERA;plant.vehicle_heading=0.
        guard=PassageStop(self.cfg)
        plant.pad=np.array([2.5,2.5,.3])
        self.assertIsNone(guard.update(plant,0.,False))
        self.assertTrue(guard.seen_horizontal_overlap)
        plant.pad=np.array([10.,2.5,.3])
        self.assertIsNone(guard.update(plant,1.,False))
        self.assertIsNone(guard.update(plant,1.249,False))
        self.assertEqual(guard.update(plant,1.251,False),'unrecognized_marker_passed_camera_fov')
        self.assertIsNone(guard.update(plant,2.,True))


if __name__=='__main__':unittest.main()
