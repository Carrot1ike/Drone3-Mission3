"""실제 C 추정기에 전달되는 원거리 경계와 갱신 소비 계약."""
import json
import unittest
from .physical_estimator import PhysicalMotion
from .scene import R_HEADING_CAMERA
from .settings import ROOT, load, controller_settings


class OnboardRangeGate(unittest.TestCase):
    def mission(self,minimum=0.):
        cfg=controller_settings(load(ROOT/'config/mission3-35x12-entry-forward-gaze-v11.json'))
        physical=json.loads((ROOT/'config/mission3-rpyt-35x12-entry-hold-limited.json').read_text())
        physical['control_architecture']['onboard_tof_min_raw_m']=minimum
        m=PhysicalMotion(cfg,cfg['entry']['positions'][0],physical,R_HEADING_CAMERA)
        m.imu(10,{'stamp_s':.01,'gyro_body_rad_s':[0.,0.,0.],'specific_force_body_m_s2':[0.,0.,9.81]})
        return m

    def packet(self,range_m,stamp,accurate,new=True):
        return {'stamp_s':stamp,'range_m':range_m,'range_valid':True,'range_accurate':accurate,
                'range_new':new,'range_stamp_s':stamp,'flow_raw_dpixel':[0.,0.],
                'flow_dt_s':.01,'flow_valid':False}

    def test_gate_uses_raw_measurement_not_synthetic_accuracy_and_does_not_reconsume(self):
        m=self.mission(.04)
        # Even an ideal-ray-derived True flag cannot admit a raw3cm packet.
        m.state_from_deck(self.packet(.03,.02,True))
        self.assertEqual(m.onboard_measurement_counts['tof'],0)
        self.assertEqual(m.onboard_measurement_counts['tof_near_raw_rejected'],1)
        pc=m.pc_core.state()
        # Exact boundary accepted independently of the synthetic flag.
        m.state_from_deck(self.packet(.04,.03,False))
        self.assertEqual(m.onboard_measurement_counts['tof'],1)
        m.state_from_deck(self.packet(.03,.04,True,False))
        m.state_from_deck(self.packet(.10,.05,True,False))
        self.assertEqual(m.onboard_measurement_counts['tof'],1)
        self.assertEqual(m.onboard_measurement_counts['tof_near_raw_rejected'],1)
        self.assertEqual(pc,m.pc_core.state())

    def test_default_keeps_stock_like_raw_interval_and_invalid_configuration_rejected(self):
        m=self.mission()
        for i,value in enumerate((-.001,0.,.02,.04,4.999,5.,float('nan')),2):
            m.state_from_deck(self.packet(value,i*.01,False))
        self.assertEqual(m.onboard_measurement_counts['tof'],4)
        self.assertNotIn('tof_near_raw_rejected',m.onboard_measurement_counts)
        for minimum in (-.01,5.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):self.mission(minimum)


if __name__=='__main__':unittest.main()
