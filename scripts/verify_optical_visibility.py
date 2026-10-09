#!/usr/bin/env python3
"""화각·픽셀 크기 후보영역의 좌표 부호를 독립 OpenCV 투영과 대조."""
import copy
import math
import unittest
import cv2
import numpy as np
from optical_visibility import project_marker, horizontal_fov_deg
from optical_landing_observer import R_REFERENCE_CAMERA_CV
from yaw_core import rotation_rpy


# Isolated ideal fixture: no generated run or real hardware calibration required.
FOCAL = 320 / (2 * math.tan(math.radians(87.) / 2))
PROFILE = {"width": 320, "height": 320,
    "K": [[FOCAL, 0., 160.], [0., FOCAL, 160.], [0., 0., 1.]],
    "marker_length_m": .15,
    "R_world_marker": [[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]]}


class VisibilityChecks(unittest.TestCase):
    def test_no_arbitrary_point_six_meter_cutoff(self):
        self.assertAlmostEqual(horizontal_fov_deg(PROFILE), 87.)
        self.assertTrue(project_marker([.8, 0., 0.], 0., PROFILE, 24.)["geometric_candidate"])
        far = project_marker([1.2, 0., 0.], 0., PROFILE, 24.)
        self.assertTrue(far["all_corners_in_frame"])
        self.assertFalse(far["geometric_candidate"])
        self.assertLess(far["min_side_px"], 24.)

    def test_center_in_fov_does_not_mean_whole_marker_in_frame(self):
        result = project_marker([.3, 0., -.3], 0., PROFILE, 24.)
        self.assertTrue(result["center_in_horizontal_fov"])
        self.assertFalse(result["all_corners_in_frame"])
        self.assertFalse(result["geometric_candidate"])

    def test_backside_is_rejected_even_when_camera_looks_at_it(self):
        result = project_marker([-.8, 0., 0.], math.pi, PROFILE, 24.)
        self.assertTrue(result["all_corners_in_frame"])
        self.assertFalse(result["front_facing"])
        self.assertFalse(result["geometric_candidate"])

    def test_projection_and_rotation_match_opencv(self):
        half = PROFILE["marker_length_m"]/2
        points = np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
        centers = np.array([[.8,.05,-.15],[.5,.1,-.05]])
        heading = math.radians(20.)
        Rcw = R_REFERENCE_CAMERA_CV.T @ rotation_rpy(0.,0.,heading).T
        Rcm = Rcw @ np.asarray(PROFILE["R_world_marker"])
        measured = project_marker(centers, heading, PROFILE, 24.)
        for i, center in enumerate(centers):
            projected = cv2.projectPoints(points, cv2.Rodrigues(Rcm)[0], Rcw@center,
                np.asarray(PROFILE["K"]), np.zeros(5))[0].reshape(4,2)
            np.testing.assert_allclose(measured["pixel_corners"][i], projected, atol=1e-9)
        turned = copy.deepcopy(PROFILE)
        R90 = rotation_rpy(0.,0.,math.pi/2)
        turned["R_world_marker"] = (R90@np.asarray(PROFILE["R_world_marker"])).tolist()
        self.assertTrue(project_marker([0.,.8,0.], math.pi/2, turned, 24.)["geometric_candidate"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
