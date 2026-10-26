"""예방적 감속은 원 hardgate/영상손실/접촉 조건을 보존해야 한다."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .controller import Mission3
from .settings import ROOT,load,controller_settings


class UncertaintyObservation(unittest.TestCase):
    def fixture(self, fraction=0.):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-entry-sector-raster-v2.json'))
        cfg['control']['uncertainty_observation']={
            'enabled':True,'soft_position_std_m':.6,'resume_position_std_m':.9,
            'resume_heading_std_deg':5.,'max_observation_s':15.,'vehicle_ff_fraction':fraction,
            'minimum_straight_heading_updates':12}
        mission=Mission3(cfg,cfg['entry']['positions'][0],start_time_s=9.)
        mission.phase='LINEUP';mission.discovery_time=250.;mission.last_good=256.
        own={'position_world_m':[6.,2.25,.7],'velocity_world_m_s':[.4,0.,0.],
             'heading_rad':0.,'position_std_m':.65,'heading_std_deg':13.,'qualified_straight_heading_updates':0}
        vehicle={'valid':True,'age_s':.05,'capture_stamp_s':255.95,'samples':40,
                 'pad_world_m':[7.,2.25,.3],'relative_pad_world_m':[1.,0.,-.4],
                 'relative_velocity_world_m_s':[-.15,0.,0.],
                 'vehicle_velocity_world_m_s':[.25,0.,0.],
                 'vehicle_heading_rad':0.,'vehicle_heading_rate_rad_s':.06,
                 'last_image_quality':{'min_side_px':25.,'marker_optical_depth_m':1.2}}
        return mission,own,vehicle

    def test_early_braking_preserves_slew_and_never_marks_touchdown(self):
        mission,own,vehicle=self.fixture();mission.last_v=np.array([.4,0,0.])
        previous=mission.last_v.copy()
        for i in range(150):
            command=mission.update(256.+.02*i,own,vehicle)
            self.assertEqual(command.phase,'UNCERTAINTY_OBSERVE')
            v=np.array(command.velocity_world_m_s)
            self.assertLessEqual(np.linalg.norm(v-previous)/.02,.15+1e-9)
            self.assertFalse(command.motors_off);self.assertFalse(command.physical_success)
            self.assertAlmostEqual(v[2],0.);previous=v
        np.testing.assert_allclose(previous,0.,atol=1e-10)

    def test_coast_uses_observed_vehicle_velocity_only(self):
        mission,own,vehicle=self.fixture(fraction=1.)
        vehicle['vehicle_velocity_world_m_s']=[.21,0.,0.]
        for i in range(130):command=mission.update(256.+.02*i,own,vehicle)
        self.assertEqual(command.phase,'UNCERTAINTY_OBSERVE')
        np.testing.assert_allclose(command.velocity_world_m_s,[.21,0.,0.])

    def test_resume_requires_fresh_new_heading_evidence_and_confirmed_straight(self):
        mission,own,vehicle=self.fixture();mission.update(256.,own,vehicle)
        own['heading_std_deg']=4.;own['qualified_straight_heading_updates']=12
        command=mission.update(256.02,own,vehicle)
        self.assertEqual(command.phase,'UNCERTAINTY_OBSERVE')
        vehicle['vehicle_heading_rate_rad_s']=0.
        own['qualified_straight_heading_updates']=11
        for i in range(100):command=mission.update(256.04+.02*i,own,vehicle)
        self.assertEqual(command.phase,'UNCERTAINTY_OBSERVE')
        own['qualified_straight_heading_updates']=12
        command=mission.update(258.06,own,vehicle)
        self.assertEqual(command.phase,'LINEUP')
        self.assertIsNone(mission.uncertainty_hold_started)

    def test_original_hard_gate_and_visual_loss_still_abort(self):
        for reason in ('odometry_uncertainty','visual_loss_after_discovery'):
            mission,own,vehicle=self.fixture();mission.update(256.,own,vehicle)
            if reason=='odometry_uncertainty':own['position_std_m']=1.001
            else:vehicle=None
            command=mission.update(259.,own,vehicle)
            self.assertEqual(command.phase,'ABORT');self.assertEqual(command.failure,reason)

    def test_hold_timeout_and_missing_uncertainty_evidence_are_failures(self):
        mission,own,vehicle=self.fixture();mission.update(256.,own,vehicle)
        self.assertEqual(mission.update(271.,own,vehicle).failure,'uncertainty_observation_timeout')
        mission,own,vehicle=self.fixture();own.pop('qualified_straight_heading_updates')
        self.assertEqual(mission.update(256.,own,vehicle).failure,'uncertainty_observation_evidence_unavailable')

    def test_configuration_cannot_weaken_original_hard_gate(self):
        mission,_,_=self.fixture()
        for key,value in [('resume_position_std_m',1.1),('soft_position_std_m',1.1),
                          ('vehicle_ff_fraction',-1.),('max_observation_s',float('nan'))]:
            cfg=deepcopy(mission.cfg);cfg['control']['uncertainty_observation'][key]=value
            with self.assertRaises(ValueError):Mission3(cfg,cfg['entry']['positions'][0])


if __name__=='__main__':unittest.main()
