import math
import unittest
import numpy as np
from .map_constraint_geometry import circle_constraint, straight_constraint
from .track import wrap


class MapConstraintGeometryTests(unittest.TestCase):
    def test_circle_jacobian_and_exact_common_rotation_gauge(self):
        rng = np.random.default_rng(721)
        for _ in range(50):
            center = rng.uniform(-5, 5, 2)
            x = np.r_[center+rng.uniform(1, 4, 2), rng.uniform(-2, 2)]
            q = rng.uniform(-1, 1, 2); beta = rng.uniform(-1, 1)
            r, H, null = circle_constraint(x, q, beta, center, 3.75, 1)
            np.testing.assert_allclose(H@null, 0., atol=1e-12)
            J = np.zeros((2, 3)); epsilon = 1e-6
            for i in range(3):
                d = np.eye(3)[i]*epsilon
                a = circle_constraint(x+d, q, beta, center, 3.75, 1)[0]
                b = circle_constraint(x-d, q, beta, center, 3.75, 1)[0]
                J[:, i] = [(a[0]-b[0])/(2*epsilon), wrap(a[1]-b[1])/(2*epsilon)]
            np.testing.assert_allclose(H, J, atol=2e-8)
            delta = .35; c, s = math.cos(delta), math.sin(delta)
            shifted = np.r_[center+np.array([[c, -s], [s, c]])@(x[:2]-center), x[2]+delta]
            np.testing.assert_allclose(circle_constraint(shifted, q, beta, center, 3.75, 1)[0], r, atol=1e-12)

    def test_single_observation_adds_no_information_along_gauge(self):
        _, H, null = circle_constraint([4., 2., .4], [.9, -.1], -.3, [7.5, 6.], 3.75, 1)
        covariance = np.diag([.2**2, math.radians(5.)**2])
        information = H.T @ np.linalg.solve(covariance, H)
        self.assertEqual(np.linalg.matrix_rank(information, tol=1e-8), 2)
        np.testing.assert_allclose(information@null, 0., atol=1e-12)

    def test_straight_does_not_claim_longitudinal_position(self):
        x = [4., 2., .1]; q = [.5, .2]
        r, H, null = straight_constraint(x, q, .2, 2.25, 0.)
        np.testing.assert_allclose(H@null, 0.)
        np.testing.assert_allclose(straight_constraint([104., 2., .1], q, .2, 2.25, 0.)[0], r)
        self.assertEqual(np.linalg.matrix_rank(H), 2)

    def test_undefined_or_nonfinite_measurements_rejected(self):
        for kwargs in [{'radius': 0.}, {'radius': 3.75, 'direction': 0}, {'radius': float('nan')}]:
            args = dict(radius=3.75, direction=1); args.update(kwargs)
            with self.assertRaises(ValueError):
                circle_constraint([0., 0., 0.], [.2, .3], 0., [1., 1.], **args)
        with self.assertRaises(ValueError):circle_constraint([0., 0., 0.], [0., 0.], 0., [0., 0.], 3.75, 1)
        with self.assertRaises(ValueError):straight_constraint([0., 0., float('nan')], [.2, .3], 0., 2.25, 0.)
