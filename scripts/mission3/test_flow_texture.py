"""Generic virtual image quality counterexamples, not PMW calibration tests."""
import unittest
import cv2
import numpy as np
import json
import tempfile
from .settings import ROOT,load
from .physical_scene import PhysicalPlant
from .flow_texture import FlowTextureProbe

class FlowTextureTests(unittest.TestCase):
    def probe(self):return FlowTextureProbe({"virtual_resolution_px":64,"virtual_fov_deg":42.})

    def test_white_pad_has_no_metric_flow_even_with_perfect_mathematical_velocity(self):
        probe=self.probe();gray=np.full((64,64),217,np.uint8)
        self.assertFalse(probe.measure(gray,0.)["usable"])
        result=probe.measure(gray,.01)
        self.assertFalse(result["usable"]);self.assertIsNone(result["diagnostic_angular_rad_s"])

    def test_single_edge_cannot_constrain_two_axis_flow(self):
        image=np.full((64,64),100,np.uint8);image[:,32:]=220
        self.assertFalse(self.probe().measure(image,0.)["usable"])

    def test_two_axis_texture_translation_has_correct_sign(self):
        image=np.random.default_rng(1).integers(0,255,(64,64),dtype=np.uint8)
        shifted=cv2.warpAffine(image,np.float32([[1,0,-2],[0,1,1]]),(64,64),borderMode=cv2.BORDER_WRAP)
        probe=self.probe();self.assertTrue(probe.measure(image,0.)["usable"])
        result=probe.measure(shifted,.01)
        self.assertTrue(result["usable"],result)
        np.testing.assert_allclose(result["shift_px"],[-2,1],atol=.15)
        self.assertGreater(result["diagnostic_angular_rad_s"][0],0.)
        self.assertGreater(result["diagnostic_angular_rad_s"][1],0.)

    def test_invalid_format_and_repeated_stamp_rejected(self):
        probe=self.probe()
        for gray in (np.zeros((64,64,3),np.uint8),np.zeros((64,64),np.float32),np.zeros((32,32),np.uint8)):
            with self.assertRaises(ValueError):probe.measure(gray,0.)
        probe.measure(np.zeros((64,64),np.uint8),1.)
        with self.assertRaises(ValueError):probe.measure(np.zeros((64,64),np.uint8),1.)

    def test_near_white_pad_does_not_disappear_through_renderer_clipping(self):
        physical=json.loads((ROOT/"config/mission3-flow-texture-sensitivity.json").read_text())
        with tempfile.TemporaryDirectory() as out:
            plant=PhysicalPlant(load(),physical,out,1,1,initial_arc_m=10.,initial_position=[25,6.25,.36])
            try:
                gray=plant.flow_probe_image()
                self.assertEqual(gray.shape,(64,64))
                self.assertGreater(float(gray.mean()),240.)
                self.assertFalse(plant.flow_texture_probe.measure(gray,0.)["usable"])
            finally:plant.close()

if __name__=="__main__":unittest.main()
