import os, sys, unittest, itertools
from dataclasses import replace
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import bs_exact as X, bs_copula as C


class T(unittest.TestCase):
    def test_rho0_belief_independent(self):
        I = X.XI(n=2, prior=(0.5, 0.5), rho=0.0)
        m = X.Model(I, 0.0)
        cnt = [(0, 0)] * 6
        cnt[0] = (2, 1)
        P = m.posterior(tuple(cnt), 0)
        # direct product-form: target at 0 vs 1, decoys marginalised
        pT = lambda h, mm: I.pT ** h * (1 - I.pT) ** mm
        Ebg = lambda h, mm: I.alpha ** h * (1 - I.alpha) ** mm
        Ed = lambda h, mm: (1 - I.pid) * Ebg(h, mm) + I.pid * I.fd ** h * (1 - I.fd) ** mm
        w0 = 0.5 * pT(2, 1) * Ed(0, 0)
        w1 = 0.5 * Ed(2, 1) * pT(0, 0)
        self.assertAlmostEqual(P[0], w0 / (w0 + w1), places=6)

    def test_predictive_sums(self):
        I = X.XI(n=3, prior=(0.2, 0.3, 0.5), rho=0.6)
        m = X.Model(I, 0.6)
        cnt = [(0, 0)] * 9
        cnt[1] = (1, 1); cnt[4] = (1, 0)
        cnt = tuple(cnt)
        z1 = m.p_alert(cnt, 0, 1, 0)
        self.assertTrue(0 < z1 < 1)
        # posterior sums to 1
        self.assertAlmostEqual(m.posterior(cnt, 0).sum(), 1.0)

    def test_optimal_beats_or_equals_rule_and_indep_plan(self):
        I = X.XI(n=2, prior=(0.5, 0.5), rho=0.6, cFR=40, cmiss=40, H=5)
        a = X.solve(I)
        b = X.solve(I, rb=0.0)
        self.assertLessEqual(a['J'], b['J'] + 1e-9)
        self.assertAlmostEqual(a['J'], a['plan_J'], places=6)      # evaluation under true model == plan value
        for th in (0.5, 0.8):
            r = X.solve_rule(I, 0.6, th, 'same')
            Jr = r['Tc'] + (1 - r['succ']) * (I.H + I.cmiss) + I.cFR * r['FR']
            self.assertLessEqual(a['J'], Jr + 1e-9)

    def test_rho0_planners_coincide(self):
        I = X.XI(n=2, prior=(0.5, 0.5), rho=0.0, H=5)
        self.assertAlmostEqual(X.solve(I)['J'], X.solve(I, rb=0.0)['J'], places=9)

    def test_indep_view_useless_at_rho0(self):
        I = X.XI(n=2, prior=(0.5, 0.5), rho=0.0, s=1, H=5, cFR=40, cmiss=40)
        self.assertAlmostEqual(X.solve(I)['J'], X.solve(replace(I, allow_indep=False))['J'], places=9)

    def test_partner_never_hurts(self):
        I = X.XI(n=2, prior=(0.5, 0.5), rho=0.6, H=5, cFR=40, cmiss=40, d=2)
        self.assertLessEqual(X.solve(replace(I, allow_partner=True))['J'], X.solve(I)['J'] + 1e-9)

if __name__ == '__main__':
    unittest.main()
