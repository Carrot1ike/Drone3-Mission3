"""Reject unavailable telemetry before state mutation, preserve valid held data."""
from collections import deque
from copy import deepcopy
import json
import unittest
import numpy as np
from .estimator import SelfMotion
from .physical_estimator import PhysicalMotion
from .scene import R_HEADING_CAMERA
from .settings import ROOT, load, controller_settings


class SensorChronology(unittest.TestCase):
    def mission(self, physical=False):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-entry-scan15-imu12-v14.json'))
        if not physical:return SelfMotion(cfg,cfg['entry']['positions'][0])
        profile=json.loads((ROOT/'config/mission3-rpyt-35x12-entry-hold-limited.json').read_text())
        own=PhysicalMotion(cfg,cfg['entry']['positions'][0],profile,R_HEADING_CAMERA)
        own.imu(10,{'stamp_s':.01,'gyro_body_rad_s':[0.,0.,0.],
                    'specific_force_body_m_s2':[0.,0.,9.81]})
        return own

    def sample(self,stamp,producer=None,new=True):
        return {'stamp_s':stamp,'gyro_z_rad_s':0.,'accel_heading_m_s2':[0.,0.,0.],
                'range_m':.96,'range_valid':True,'range_accurate':True,
                'range_new':new,'range_stamp_s':stamp if producer is None else producer,
                'flow_angular_rad_s':[0.,0.],'flow_valid':False}

    def deck(self,stamp,producer=None,new=True):
        return {**self.sample(stamp,producer,new),'flow_raw_dpixel':[0.,0.],
                'flow_dt_s':.01,'flow_stamp_s':stamp}

    def snapshot(self,own):
        names=('position','velocity','heading','last_stamp','last_range','surface',
               'history','velocity_history','range_heights','vertical_imu','events',
               'vertical_fit_fallbacks')
        state={k:deepcopy(getattr(own,k)) for k in names}
        state['producer_latch']=own._last_range_input_stamp
        state['uncertainty']=deepcopy(vars(own.uncertainty)) if own.uncertainty else None
        if isinstance(own,PhysicalMotion):
            state.update(firmware=own.core.state(),pc=own.pc_core.state(),
                         counts=deepcopy(own.onboard_measurement_counts),
                         control_acc=own.control_acc.copy(),control_count=own.control_count,
                         attitude_history=deepcopy(own.attitude_history))
        return repr(state)

    def test_future_range_rejected_before_height_or_onboard_fusion(self):
        for physical in (False,True):
            for producer in (.03,np.nextafter(.02,np.inf)):
                with self.subTest(physical=physical,producer=producer):
                    own=self.mission(physical);before=self.snapshot(own)
                    packet=(self.deck if physical else self.sample)(.02,producer)
                    with self.assertRaises(ValueError):
                        (own.state_from_deck if physical else own.update)(packet)
                    self.assertEqual(before,self.snapshot(own))

    def test_duplicate_backward_nonfinite_and_gapped_consumers_are_atomic(self):
        for physical in (False,True):
            for stamp in (.02,.019,float('nan'),float('inf'),.221):
                with self.subTest(physical=physical,stamp=stamp):
                    own=self.mission(physical);fn=own.state_from_deck if physical else own.update
                    packet=self.deck if physical else self.sample
                    fn(packet(.02));before=self.snapshot(own)
                    with self.assertRaises(ValueError):fn(packet(stamp))
                    self.assertEqual(before,self.snapshot(own))

    def test_new_range_requires_monotonic_producer_but_held_value_is_usable(self):
        for physical in (False,True):
            for producer in (.02,.019,float('nan')):
                with self.subTest(physical=physical,producer=producer):
                    own=self.mission(physical);fn=own.state_from_deck if physical else own.update
                    packet=self.deck if physical else self.sample
                    fn(packet(.02));before=self.snapshot(own)
                    with self.assertRaises(ValueError):fn(packet(.03,producer,True))
                    self.assertEqual(before,self.snapshot(own))
            own=self.mission(physical);fn=own.state_from_deck if physical else own.update
            packet=self.deck if physical else self.sample
            fn(packet(.02));heights=list(own.range_heights)
            counts=deepcopy(own.onboard_measurement_counts) if physical else None
            fn(packet(.03,.02,False))
            self.assertEqual(list(own.range_heights),heights)
            self.assertEqual(own.last_stamp,.03)
            if physical:self.assertEqual(own.onboard_measurement_counts,counts)

    def test_future_imu_and_flow_stamps_rejected_before_onboard_mutation(self):
        for field in ('imu','flow'):
            own=self.mission(True);packet=self.deck(.02)
            if field=='imu':own.latest_imu['stamp_s']=.03
            else:packet['flow_stamp_s']=.03
            before=self.snapshot(own)
            with self.assertRaises(ValueError):own.state_from_deck(packet)
            self.assertEqual(before,self.snapshot(own))

    def test_invalid_fit_history_cannot_fall_back_to_linear_future_data(self):
        t=np.arange(0.,.401,.05);ranges=list(zip(t,.7-.03*t))
        imu=list(zip(np.arange(0.,.401,.01),np.zeros(41)))
        bad_cases=((ranges,imu,.3),(ranges,imu+[(.41,0.)],.4),
                   (ranges+[(.4,.6)],imu,.4),(ranges,imu+[(.4,0.)],.4),
                   (ranges[:-1]+[(.4,float('nan'))],imu,.4))
        for r,i,now in bad_cases:
            own=self.mission();own.range_heights=deque(r);own.vertical_imu=deque(i)
            before=self.snapshot(own)
            with self.assertRaises(ValueError):own.fitted_range_velocity(now)
            self.assertEqual(before,self.snapshot(own))

    def test_valid_history_with_missing_acceleration_coverage_keeps_explicit_fallback(self):
        own=self.mission();t=np.arange(0.,.401,.05)
        own.range_heights.extend(zip(t,.7-.03*t))
        own.vertical_imu.extend(zip(np.arange(.10,.401,.01),np.zeros(31)))
        self.assertAlmostEqual(own.fitted_range_velocity(.4),-.03)
        self.assertEqual(own.vertical_fit_fallbacks,1)

    def test_malformed_internal_history_rejected_before_full_update(self):
        valid=[(.001,1.),(.005,1.),(.009,1.),(.013,1.),(.017,1.),(.02,1.)]
        cases=(('range_heights',valid+[(.04,1.)]),
               ('range_heights',valid+[(.02,1.)]),
               ('range_heights',valid+[(.019,1.)]),
               ('range_heights',valid[:-1]+[(.02,float('nan'))]),
               ('vertical_imu',[(.01,0.),(.04,0.)]),
               ('vertical_imu',[(.01,0.),(.01,0.)]),
               ('vertical_imu',[(.02,0.),(.01,0.)]),
               ('vertical_imu',[(.01,float('inf'))]))
        for physical in (False,True):
            for field,history in cases:
                with self.subTest(physical=physical,field=field,history=history):
                    own=self.mission(physical)
                    fn=own.state_from_deck if physical else own.update
                    packet=self.deck if physical else self.sample
                    fn(packet(.02))
                    setattr(own,field,deque(history))
                    before=self.snapshot(own)
                    with self.assertRaises(ValueError):fn(packet(.03,.025))
                    self.assertEqual(before,self.snapshot(own))

    def test_prospective_history_append_rejected_before_mutation(self):
        for physical in (False,True):
            for field,history in (('range_heights',[(.025,1.)]),
                                  ('vertical_imu',[(.03,0.)])):
                with self.subTest(physical=physical,field=field):
                    own=self.mission(physical)
                    fn=own.state_from_deck if physical else own.update
                    packet=self.deck if physical else self.sample
                    fn(packet(.02));setattr(own,field,deque(history))
                    before=self.snapshot(own)
                    with self.assertRaises(ValueError):fn(packet(.03,.025))
                    self.assertEqual(before,self.snapshot(own))


if __name__=='__main__':unittest.main()
