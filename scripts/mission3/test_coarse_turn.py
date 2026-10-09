"""Optional image-qualified translation during coarse gaze alignment."""
import unittest
import math
import numpy as np
from .settings import load,controller_settings
from .controller import Mission3


class CoarseTurn(unittest.TestCase):
    def fixture(self, pixels=True, age=.05):
        cfg=controller_settings(load());cfg["control"].update(coarse_translate_turn_error_deg=40.,coarse_turn_translation_speed_m_s=.15)
        m=Mission3(cfg,cfg["entry"]["positions"][0]);own={"position_world_m":[17.,5.9,.7],"velocity_world_m_s":[0,0,0],"heading_rad":0.,"position_std_m":.1}
        v={"kind":"coarse_image_bearing_only","valid":True,"capture_stamp_s":20-age,"age_s":age,"bearing_world_rad":math.radians(30.),"min_side_px":8.}
        if pixels:v["max_side_px"]=20.
        return m,own,v

    def test_turn_translation_uses_fresh_pixels_and_has_no_metric_ff(self):
        m,own,v=self.fixture();c=m.update(20.,own,v)
        self.assertGreater(np.linalg.norm(c.velocity_world_m_s[:2]),0.)
        self.assertLessEqual(np.linalg.norm(c.velocity_world_m_s[:2]),.15)
        self.assertEqual(c.velocity_world_m_s[2],0.)
        self.assertIsNone(m.last_vehicle);self.assertIsNone(m.discovery_time)
        self.assertFalse(c.motors_off)

    def test_missing_or_old_pixel_quality_cannot_open_turn_translation(self):
        for pixels,age in ((False,.05),(True,.5)):
            m,own,v=self.fixture(pixels,age);c=m.update(20.,own,v)
            self.assertEqual(c.velocity_world_m_s,[0.,0.,0.])

    def test_near_large_or_outside_view_bearing_stops_translation(self):
        for side,angle in ((40.,30.),(20.,45.)):
            m,own,v=self.fixture();v["max_side_px"]=side;v["bearing_world_rad"]=math.radians(angle)
            self.assertEqual(m.update(20.,own,v).velocity_world_m_s,[0.,0.,0.])


if __name__=="__main__":unittest.main()
