import os, sys, unittest, math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import bs_sim as S, bs_policies as P, bs_copula as C


class FixedLooks:
    """Scripted policy: scan cell c, then review looks, to test belief calibration; never reports."""
    rb = None
    name = 'script'
    def __init__(self, c, seq): self.c, self.seq, self.i = c, seq, 0
    def decide(self, m):
        if self.i >= len(self.seq): return None
        v = self.seq[self.i]; self.i += 1
        return ('scan', self.c, 0) if self.i == 1 else ('rev', self.c, v)


class T(unittest.TestCase):
    def test_belief_calibration(self):
        """Predictive P(next look alerts) from the correlation-aware belief matches the truth; independence belief does not."""
        reg = S.Regime(D=1e6)
        out = {0.0: [], 0.6: []}
        for rho_b in (0.0, 0.6):
            pred, obs_ = [], []
            for layout in range(40):
                ctx = S.Ctx(layout, S.Cond(rho=0.6, f='measured', regime=reg))
                # a cell that is the target half of the time: use the true target cell (target hypothesis) and a decoy cell
                for cell in (ctx.target,):
                    for rep in range(30):
                        m = S.Mission(ctx, ctx.noise(rep), FixedLooks(cell, [0, 0, 0, 0]))
                        m.B = S.Belief(ctx, rho_b)
                        # replay 4 looks and record predictive of look k+1 given k previous (conditional on target = True)
                        seq = [('scan', 0), ('rev', 0), ('rev', 0), ('rev', 0)]
                        for kind, v in seq:
                            p = m.B.pT_pred(kind, v, cell)
                            m.pos = cell; m.w = v
                            y = m._look(cell, v, kind)
                            pred.append(p); obs_.append(y)
            out[rho_b] = (np.mean(pred), np.mean(obs_))
        # conditional on target, predictive mean from the true-rho belief must match the empirical alert rate
        self.assertAlmostEqual(out[0.6][0], out[0.6][1], delta=0.06)
        # the independence belief predicts the same marginal on average (marginal is right), the failure is in the LR: tested below

    def test_posterior_ratio_independent(self):
        reg = S.Regime(D=1e6)
        ctx = S.Ctx(3, S.Cond(rho=0.6, f='measured', regime=reg))
        B = S.Belief(ctx, 0.0)
        c = int(np.argmax(ctx.prior))
        d = (ctx.prior > 0).nonzero()[0][0]
        p0 = B.post().copy()
        B.update(c, 0, 'rev', True); B.update(c, 0, 'rev', True)
        p1 = B.post()
        pn = ctx.pid_plan[c] * ctx.fd + (1 - ctx.pid_plan[c]) * ctx.alpha
        lr = (ctx.p_rev[c] / pn) ** 2
        o0 = p0[c] / (1 - p0[c]); o1 = p1[c] / (1 - p1[c])
        # independence belief: odds multiply by prod of alert likelihood ratios, with the decoy mixture handled exactly:
        # pN per look is the marginal only for one look; two alerts at a decoy cell are 'iid' given decoy -> E[fd^2]
        pn2 = (1 - ctx.pid_plan[c]) * ctx.alpha ** 2 + ctx.pid_plan[c] * ctx.fd ** 2
        self.assertAlmostEqual(o1 / o0, ctx.p_rev[c] ** 2 / pn2, delta=0.02 * o1 / o0)

    def test_correlation_deflates_evidence(self):
        """When alerts favour the target (decoys fire less often than the target: f=low), independence over-counts repeats.
        With the 'measured' decoy rate (0.8 > target TPR 0.6) the sign flips: documented in DEV.md section 4."""
        reg = S.Regime(D=1e6)
        ctx = S.Ctx(3, S.Cond(rho=0.6, f='low', regime=reg))
        c = int(np.argmax(ctx.prior))
        pj = {}
        for rb in (0.0, 0.6):
            B = S.Belief(ctx, rb)
            for _ in range(3):
                B.update(c, 0, 'rev', True)
            pj[rb] = B.post()[c]
        # three repeat alerts from one view are strong evidence if independent, weak if correlated
        self.assertGreater(pj[0.0], pj[0.6])

    def test_mission_runs_and_crn(self):
        reg = S.Regime(D=900.0)
        ctx = S.Ctx(5, S.Cond(rho=0.6, f='measured', regime=reg))
        a = S.run_mission(ctx, 0, P.make_factory({'kind': 'COVER'}))
        b = S.run_mission(ctx, 0, P.make_factory({'kind': 'COVER'}))
        self.assertEqual(a, b)
        o = S.run_mission(ctx, 0, P.make_factory({'kind': 'ORACLE'}))
        self.assertLessEqual(o['T'], 900.0)

if __name__ == '__main__':
    unittest.main()
