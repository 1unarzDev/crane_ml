"""3-DOF Fossen surface vessel with fixed vectored thrusters: the marine pack's family.

The dynamics are the continuous ODE of crane's ``Tools/Characterization/plant.py::ode_rhs`` (the
Unity ``MarinePlanarModel`` twin), batched over particles:

    M nudot = tau - C_RB(nu) nu - C_A(nu_r) nu_r - D(nu_r) + M_A nudot_c,   etadot = R(psi) nu

State [x, y, psi, u, v, r] (world ENU pose, body FLU velocity at the CG). Input: one normalised
command q_i in [-1, 1] per thruster (PWM = 1500 + 400 q). Thrust f_i = k_fwd q|q| (k_rev when
q < 0) with k_fwd = 2 k_ref / (1 + rev_ratio), k_rev = rev_ratio k_fwd: the forward/reverse split
moves, their mean k_ref does not (so rev_ratio is not confounded with the thrust gauge in tests
that run two thrusters forward and two in reverse). Delayed by ``delay_s`` (lumped dead time +
spin-up); tau = B f.

Parameters are **effective** (thrust-normalised, sim2real.md sec. 3.2 gauge): with an unknown
thrust scale s, motion identifies m_jj/s and d/s, which is the NLC study's theta_i. The card says
``gauge: effective`` until a bollard pull anchors s; views then fix s = 1 and flag it.
"""

import math

import numpy as np

from plantcard.family import Family, ParamSpec, delayed_inputs

PWM_NEUTRAL, PWM_SPAN = 1500.0, 400.0


class Fossen3(Family):
    name = 'fossen3'
    version = '1'
    state = ['x', 'y', 'psi', 'u', 'v', 'r']
    position = (0, 1)
    heading = 2
    delay_param = 'delay_s'
    params = [
        ParamSpec('m11', 'kg (per unit thrust scale)', (5.0, None), 'dyn.surge'),
        ParamSpec('d1_u', 'N s/m', (0.0, None), 'dyn.surge'),
        ParamSpec('d2_u', 'N s2/m2', (0.0, None), 'dyn.surge'),
        ParamSpec('m22', 'kg', (5.0, None), 'dyn.sway'),
        ParamSpec('d1_v', 'N s/m', (0.0, None), 'dyn.sway'),
        ParamSpec('d2_v', 'N s2/m2', (0.0, None), 'dyn.sway'),
        ParamSpec('m66', 'kg m2', (0.2, None), 'dyn.yaw'),
        ParamSpec('d1_r', 'N m s/rad', (0.0, None), 'dyn.yaw'),
        ParamSpec('d2_r', 'N m s2/rad2', (0.0, None), 'dyn.yaw'),
        ParamSpec('delay_s', 's', (0.0, None), 'act'),
        ParamSpec('rev_ratio', '1', (0.1, 1.5), 'act'),
        ParamSpec('deadband_q', '1 (of the 400 us span)', (0.0, 0.3), 'act'),
    ]
    blocks = {'dyn.surge': ['m11', 'd1_u', 'd2_u'], 'dyn.sway': ['m22', 'd1_v', 'd2_v'],
              'dyn.yaw': ['m66', 'd1_r', 'd2_r']}
    grid_params = ['delay_s', 'rev_ratio', 'deadband_q']
    # a block learns only from segments whose commanded wrench excites its axis (axis-specific
    # experiments): on 10-18 a pure sway pulse yaws the boat at -0.2 rad/s in BOTH directions
    # (thruster-hull interaction, not representable here), which would corrupt the yaw block.
    EXCITES = (10.0, 10.0, 0.5)   # |X| N, |Y| N, |N| N m of the nominal commanded wrench
    views = {'unity': 'plantcard.marine.views:unity_profile', 'nlc': 'plantcard.marine.views:nlc_params',
             'control': 'plantcard.marine.views:control', 'fastsim': 'plantcard.marine.views:fastsim_spec'}

    def __init__(self, config=None):
        super().__init__(config)
        th = self.config['thrusters']
        self.inputs = ['q_' + t['name'] for t in th]
        a = np.radians([t['alphaDeg'] for t in th])
        xy = np.array([[t['x'], t['y']] for t in th], float)
        self.B = np.stack([np.cos(a), np.sin(a), xy[:, 0] * np.sin(a) - xy[:, 1] * np.cos(a)])  # 3 x n_thr
        self.k_ref = float(self.config['k_ref'])
        self.m_rb = float(self.config['m_rb'])
        self.izz_rb = float(self.config['izz_rb'])
        self.a26 = float(self.config.get('a26', 0.0))

    # ---- dynamics -------------------------------------------------------------------------
    def thrust(self, q, rev_ratio, deadband_q=0.0):
        """Body wrench [..., 3] from commands q [..., n_thr]:
        f = k sign(q) e|e|,  e = max(|q| - db, 0) / (1 - db)  (full command still gives k)."""
        q = np.clip(q, -1.0, 1.0)
        rho = np.asarray(rev_ratio, float)[..., None]
        db = np.asarray(deadband_q, float)[..., None]
        e = np.maximum(np.abs(q) - db, 0.0) / (1.0 - db)
        f = self.k_ref * np.sign(q) * e * e * 2.0 / (1.0 + rho)   # gauge: (k_fwd + k_rev) / 2 = k_ref
        f = np.where(q < 0, f * rho, f)
        return f @ self.B.T

    def rhs(self, x, u, p, w):
        tau = self.thrust(u, p['rev_ratio'], p['deadband_q'] if 'deadband_q' in p else 0.0)
        return self.rhs_wrench(x, tau, p, w)

    def rhs_wrench(self, x, tau, p, w):
        m, a26 = self.m_rb, self.a26
        m11, m22, m66 = p['m11'], p['m22'], p['m66']
        a11, a22 = m11 - m, m22 - m
        psi, u, v, r = x[:, 2], x[:, 3], x[:, 4], x[:, 5]
        c, s = np.cos(psi), np.sin(psi)
        cx = w['cx'] if w else 0.0
        cy = w['cy'] if w else 0.0
        uc, vc = c * cx + s * cy, -s * cx + c * cy
        ur, vr = u - uc, v - vc
        ndc0, ndc1 = r * vc, -r * uc
        D0 = (p['d1_u'] + p['d2_u'] * np.abs(ur)) * ur
        D1 = (p['d1_v'] + p['d2_v'] * np.abs(vr)) * vr
        D2 = (p['d1_r'] + p['d2_r'] * np.abs(r)) * r
        ww = a22 * vr + a26 * r
        ca0, ca1, ca2 = -ww * r, a11 * ur * r, ww * ur - a11 * ur * vr
        crb0, crb1 = -m * v * r, m * u * r
        f0 = tau[:, 0] - crb0 - ca0 - D0 + a11 * ndc0
        f1 = tau[:, 1] - crb1 - ca1 - D1 + a22 * ndc1
        f2 = tau[:, 2] - ca2 - D2 + a26 * ndc1
        if w and 'fx' in w:
            f0, f1, f2 = f0 + w['fx'], f1 + w['fy'], f2 + w['fn']
        det = m22 * m66 - a26 * a26
        return np.stack([c * u - s * v, s * u + c * v, r, f0 / m11,
                         (m66 * f1 - a26 * f2) / det, (m22 * f2 - a26 * f1) / det], -1)

    def substeps_for(self, p, dt, v_max=(3.0, 3.0, 2.0), limit=0.5, cap=64):
        """RK4 substeps so dt J / m stays under ``limit`` for every particle (stiff draws)."""
        x = 0.0
        for k, m, vm in zip('uvr', ('m11', 'm22', 'm66'), v_max):
            J = np.abs(p['d1_' + k]) + 2 * np.abs(p['d2_' + k]) * vm
            x = max(x, float(np.max(dt * J / np.asarray(p[m]))))
        return int(min(cap, max(1, np.ceil(x / limit))))

    def rollout(self, x0, t, u, p, w=None, substeps=None):
        if substeps is None:
            dt = float(np.max(np.diff(t))) if len(t) > 1 else 0.0
            substeps = max(self.substeps_for(p, dt), int(np.ceil(dt / 0.025 - 1e-9)))   # delay resolved to 25 ms
        return super().rollout(x0, t, u, p, w, substeps)

    def disturbance(self, n, rng, seg=None, dist=None):
        """Per-rollout constant current (around the segment's baseline estimate) and a constant
        body force bias (unmodelled wind/waves), from the card's ``dist`` block."""
        dist = dist or {}
        c0 = (seg.meta.get('current', (0.0, 0.0)) if seg is not None else (0.0, 0.0))
        csd = float(dist.get('current_sd', 0.0))
        fsd = dist.get('force_sd', [0.0, 0.0, 0.0])
        return {'cx': c0[0] + csd * rng.standard_normal(n), 'cy': c0[1] + csd * rng.standard_normal(n),
                'fx': fsd[0] * rng.standard_normal(n), 'fy': fsd[1] * rng.standard_normal(n),
                'fn': fsd[2] * rng.standard_normal(n)}

    def wrap_state(self, x):
        return x

    # ---- identification -------------------------------------------------------------------
    def _base(self, seg, ref, window_s, step_s, sub):
        """Grid-independent parts of the integral regression (cached per segment and ref)."""
        key = ('_fossen3_base', window_s, step_s, sub, ref['m11'], ref['m22'])
        hit = seg.meta.get('_cache')
        if hit and hit[0] == key:
            return hit[1]
        t, X = seg.t, seg.x
        dt = float(np.median(np.diff(t)))
        w, st = max(1, int(round(window_s / dt))), max(1, int(round(step_s / dt)))
        lo, hi = seg.meta.get('fit_window', (t[0], t[-1]))
        psi, u, v, r = X[:, 2], X[:, 3], X[:, 4], X[:, 5]
        c, s = np.cos(psi), np.sin(psi)
        cx, cy = seg.meta.get('current', (0.0, 0.0))
        uc, vc = c * cx + s * cy, -s * cx + c * cy
        ur, vr = u - uc, v - vc
        m = self.m_rb
        a11, a22, a26 = ref['m11'] - m, ref['m22'] - m, self.a26
        ww = a22 * vr + a26 * r
        corr = np.stack([-m * v * r - ww * r - a11 * r * vc,
                         m * u * r + a11 * ur * r + a22 * r * uc,
                         ww * ur - a11 * ur * vr + a26 * r * uc], -1)
        rel = np.stack([ur, vr, r], -1)

        def cum(a):
            return np.concatenate([np.zeros((1,) + a.shape[1:]), np.cumsum(0.5 * (a[1:] + a[:-1]) * dt, 0)])
        cc, cr, cq = cum(corr), cum(rel), cum(np.abs(rel) * rel)
        k1 = np.arange(0, len(t) - w, st)
        k1 = k1[(t[k1] >= lo) & (t[k1 + w] <= hi)]
        k2 = k1 + w
        nu = np.stack([u, v, r], -1)
        tf = np.arange(t[0], t[-1] + 1e-9, dt / sub)
        base = {'k1': k1, 'k2': k2, 'dnu': nu[k2] - nu[k1], 'ir': cr[k2] - cr[k1], 'iq': cq[k2] - cq[k1],
                'icorr': cc[k2] - cc[k1], 'tf': tf, 'dt': dt, 'sub': sub}
        seg.meta['_cache'] = (key, base)
        return base

    def regressors(self, seg, g, ref, window_s=1.0, step_s=0.2, sub=10, select=True):
        """Integral form (no differentiated velocity: limits the errors-in-variables bias of
        NLC MATH 4.2). Over each window [t1, t2] and per axis j:

            int (tau_j(t - delay) - c_j(nu)) dt = m_jj [nu_j(t2) - nu_j(t1)] + d1 int nu_r,j dt
                                                  + d2 int |nu_r,j| nu_r,j dt

        c(nu) = C_RB(nu) nu + C_A(nu_r) nu_r - M_A nudot_c with the ``ref`` masses. A block gets
        rows only from segments that excite its axis (``EXCITES``)."""
        B = self._base(seg, ref, window_s, step_s, sub)
        t, tf, k1, k2 = seg.t, B['tf'], B['k1'], B['k2']
        q = seg.u
        idx = np.clip(np.searchsorted(t, tf - g['delay_s'] + 1e-9, side='right') - 1, 0, len(t) - 1)
        qf = q[idx]
        qf[tf - g['delay_s'] < t[0]] = q[0]
        tauf = self.thrust(qf, np.array([g['rev_ratio']]), np.array([g.get('deadband_q', 0.0)]))
        h = B['dt'] / B['sub']
        cumf = np.concatenate([np.zeros((1, 3)), np.cumsum(0.5 * (tauf[1:] + tauf[:-1]) * h, 0)])
        cum = np.stack([np.interp(t, tf, cumf[:, j]) for j in range(3)], -1)
        y = cum[k2] - cum[k1] - B['icorr']
        excited = np.abs(self.thrust(q, np.array([1.0]))).max(0) >= np.asarray(self.EXCITES)
        out = {}
        for j, b in enumerate(self.blocks):
            Phi = np.stack([B['dnu'][:, j], B['ir'][:, j], B['iq'][:, j]], -1)
            keep = slice(None) if excited[j] or not select else slice(0, 0)
            out[b] = (Phi[keep], y[keep, j])
        return out

    def envelope_features(self, seg):
        nus = seg.x[:, 3:6]
        cx, cy = seg.meta.get('current', (0.0, 0.0))
        c, s = np.cos(seg.x[:, 2]), np.sin(seg.x[:, 2])
        ur, vr = nus[:, 0] - (c * cx + s * cy), nus[:, 1] - (-s * cx + c * cy)
        eff = np.abs(seg.u).max(1)
        return np.stack([ur, vr, nus[:, 2], eff], -1), ['u_r', 'v_r', 'r', 'effort_abs']


def profile_config(profile):
    """Family config (robot constants) from a characterization profile (profiles/*.json)."""
    th = profile['thrusters']
    return {'thrusters': [{'name': t['name'], 'x': t['x'], 'y': t['y'], 'alphaDeg': t['alphaDeg']} for t in th],
            'k_ref': float(np.mean([0.5 * (t['kForward'] + t['kReverse']) for t in th])), 'm_rb': float(profile['mass']),
            'izz_rb': float(profile['izz']), 'a26': float(profile['addedMass']['a26']),
            'pwm': {'neutral': PWM_NEUTRAL, 'span': PWM_SPAN}}


def profile_params(profile):
    """Parameter dict (sign-averaged damping, mean thrust lag) from a characterization profile."""
    am = profile['addedMass']
    th = profile['thrusters']
    p = {'m11': profile['mass'] + am['a11'], 'm22': profile['mass'] + am['a22'], 'm66': profile['izz'] + am['a66']}
    for ax, k in (('surge', 'u'), ('sway', 'v'), ('yaw', 'r')):
        p['d1_' + k] = 0.5 * (profile[ax]['d1Pos'] + profile[ax]['d1Neg'])
        p['d2_' + k] = 0.5 * (profile[ax]['d2Pos'] + profile[ax]['d2Neg'])
    p['rev_ratio'] = float(np.mean([t['kReverse'] for t in th])) / float(np.mean([t['kForward'] for t in th]))
    p['delay_s'] = float(np.mean([t['deadTime'] + t['tauT'] for t in th]))
    p['deadband_q'] = float(np.mean([t['deadbandUs'] for t in th])) / PWM_SPAN
    return {k: float(v) for k, v in p.items()}


def pwm_to_q(pwm):
    return np.clip((np.asarray(pwm, float) - PWM_NEUTRAL) / PWM_SPAN, -1.0, 1.0)


def angle_wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi
