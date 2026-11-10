import os, sys, unittest
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import bs_copula as C

class T(unittest.TestCase):
    def test_icc_mc(self):
        rng = np.random.default_rng(0)
        for p in (0.1, 0.6):
            for icc in (0.3, 0.6, 0.8):
                r = C.latent_for_icc(p, icc)
                n = 400000
                z = rng.standard_normal(n)
                e1, e2 = rng.standard_normal((2, n))
                t = C.probit(p)
                a = np.sqrt(r) * z + np.sqrt(1 - r) * e1 < t
                b = np.sqrt(r) * z + np.sqrt(1 - r) * e2 < t
                self.assertAlmostEqual(a.mean(), p, delta=0.005)
                self.assertAlmostEqual(np.corrcoef(a, b)[0, 1], icc, delta=0.012)
    def test_marginal_and_indep(self):
        self.assertAlmostEqual(C.count_lik(0.3, 0.0, 1, 1), 0.3 * 0.7, places=9)
        for r in (0.2, 0.7):
            self.assertAlmostEqual(C.count_lik(0.3, r, 1, 0), 0.3, places=4)
if __name__ == '__main__':
    unittest.main()
