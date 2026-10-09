"""Curve/downstream following must retain height until landing preparation fits."""
from copy import deepcopy
import math
import unittest
import numpy as np
from .controller import Mission3
from .settings import load, controller_settings


class StraightPreparation(unittest.TestCase):
    def fixture(self,x=25.,y=6.25,heading=0.,rate=0.,enabled=True):
        cfg=controller_settings(load())
        cfg['control'].update(lineup_lowering_requires_straight=enabled,
                              lineup_preparation_budget_s=12.,terminal_max_prediction_s=12.)
        mission=Mission3(cfg,cfg['entry']['positions'][0]);mission.phase='LINEUP'
        mission.discovery_time=10.;mission.last_good=20.
        forward=np.array([math.cos(heading),math.sin(heading),0.])
        own={'position_world_m':(np.array([x,y,.7])-.65*forward).tolist(),
             'velocity_world_m_s':(.25*forward).tolist(),'heading_rad':heading,'position_std_m':.8}
        vehicle={'valid':True,'age_s':.05,'capture_stamp_s':19.95,'samples':30,
                 'pad_world_m':[x,y,.3],'relative_pad_world_m':(.65*forward+np.array([0,0,-.4])).tolist(),
                 'relative_velocity_world_m_s':[0,0,0],'vehicle_velocity_world_m_s':(.25*forward).tolist(),
                 'vehicle_heading_rad':heading,'vehicle_heading_rate_rad_s':rate,
                 'last_image_quality':{'min_side_px':30.,'marker_optical_depth_m':.8}}
        return mission,own,vehicle

    def ticks(self,mission,own,vehicle,start=20.,count=136):
        commands=[]
        for i in range(count):
            command=mission.update(start+i*.02,own,vehicle);commands.append(command)
            self.assertFalse(command.motors_off)
            self.assertFalse(command.physical_success)
        return commands

    def test_curve_and_downstream_straight_keep_lineup_height(self):
        for args in ({'rate':math.radians(3.8)}, {'x':16.,'y':13.75,'heading':math.pi},
                     {'x':34.}, {'x':11.25,'y':10.,'heading':-math.pi/2}):
            with self.subTest(args=args):
                mission,own,vehicle=self.fixture(**args)
                commands=self.ticks(mission,own,vehicle)
                self.assertTrue(all(c.phase=='LINEUP' for c in commands))
                self.assertTrue(all(abs(c.velocity_world_m_s[2])<1e-10 for c in commands))

    def test_new_straight_requires_confirmation_and_lineup_hold_before_lowering(self):
        mission,own,vehicle=self.fixture(rate=math.radians(3.8))
        self.ticks(mission,own,vehicle,count=20)
        vehicle['vehicle_heading_rate_rad_s']=0.
        commands=self.ticks(mission,own,vehicle,start=21.)
        self.assertTrue(all(c.phase=='LINEUP' for c in commands[:124]))
        self.assertEqual(commands[-1].phase,'LOWER_ALIGN')

    def test_preparation_uses_measured_speed_and_strict_straight_margin(self):
        for x,allowed in ((27.76,False),(27.74,True)):
            mission,own,vehicle=self.fixture(x=x)
            self.assertEqual(self.ticks(mission,own,vehicle)[-1].phase,'LOWER_ALIGN' if allowed else 'LINEUP')
        mission,own,vehicle=self.fixture(x=25.)
        vehicle['vehicle_velocity_world_m_s']=[.4,0.,0.]
        own['velocity_world_m_s']=[.4,0.,0.]
        self.assertEqual(self.ticks(mission,own,vehicle)[-1].phase,'LINEUP')

    def test_disabled_policy_preserves_original_lowering_transition(self):
        mission,own,vehicle=self.fixture(rate=math.radians(3.8),enabled=False)
        self.assertEqual(self.ticks(mission,own,vehicle)[-1].phase,'LOWER_ALIGN')


if __name__=='__main__':unittest.main()
