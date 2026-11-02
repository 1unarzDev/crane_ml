"""Planar quadrotor (x-z plane, two rotors): the extension proof that the core is robot-agnostic.

State [x, z, theta, vx, vz, omega]; input u = [u1, u2] (rotor commands, thrust k_ref u_i, delayed).

    m vxdot = -(F1 + F2) sin(theta) - dx vx
    m vzdot =  (F1 + F2) cos(theta) - dz vz - W          (W: weight, identified with m)
    I wdot  =  l (F2 - F1) - dth omega

Three NIG blocks (integral regression over 1 s windows) and a grid over the rotor delay. Nothing in
``plantcard/*.py`` outside ``families/`` and ``marine/`` knows this robot exists.
"""

import numpy as np

from plantcard.family import Family, ParamSpec


class Quad2D(Family):
    name = 'quad2d'
    version = '1'
    state = ['x', 'z', 'theta', 'vx', 'vz', 'omega']
    inputs = ['u1', 'u2']
    position = (0, 1)
    heading = 2
    delay_param = 'delay_s'
    params = [ParamSpec('m_x', 'kg', (0.05, None), 'dyn.x'), ParamSpec('dx', 'N s/m', (0.0, None), 'dyn.x'),
              ParamSpec('m_z', 'kg', (0.05, None), 'dyn.z'), ParamSpec('dz', 'N s/m', (0.0, None), 'dyn.z'),
              ParamSpec('W', 'N', (0.0, None), 'dyn.z'),
              ParamSpec('I', 'kg m2', (1e-4, None), 'dyn.th'), ParamSpec('dth', 'N m s', (0.0, None), 'dyn.th'),
              ParamSpec('delay_s', 's', (0.0, None), 'act')]
    blocks = {'dyn.x': ['m_x', 'dx'], 'dyn.z': ['m_z', 'dz', 'W'], 'dyn.th': ['I', 'dth']}
    grid_params = ['delay_s']
    views = {}

    def __init__(self, config=None):
        super().__init__(config or {'k_ref': 5.0, 'arm': 0.15})
        self.k_ref, self.arm = float(self.config['k_ref']), float(self.config['arm'])

    def rhs(self, x, u, p, w):
        F1, F2 = self.k_ref * u[:, 0], self.k_ref * u[:, 1]
        th, vx, vz, om = x[:, 2], x[:, 3], x[:, 4], x[:, 5]
        F = F1 + F2
        ax = (-F * np.sin(th) - p['dx'] * vx + (w['fx'] if w else 0.0)) / p['m_x']
        az = (F * np.cos(th) - p['dz'] * vz - p['W'] + (w['fz'] if w else 0.0)) / p['m_z']
        al = (self.arm * (F2 - F1) - p['dth'] * om) / p['I']
        return np.stack([vx, vz, om, ax, az, al], -1)

    def disturbance(self, n, rng, seg=None, dist=None):
        sd = (dist or {}).get('force_sd', [0.0, 0.0, 0.0])
        return {'fx': sd[0] * rng.standard_normal(n), 'fz': sd[1] * rng.standard_normal(n)}

    def regressors(self, seg, g, ref, window_s=1.0, step_s=0.2, select=True):
        t, X = seg.t, seg.x
        dt = float(np.median(np.diff(t)))
        w, st = int(round(window_s / dt)), max(1, int(round(step_s / dt)))
        idx = np.clip(np.searchsorted(t, t - g['delay_s'] + 1e-9, side='right') - 1, 0, len(t) - 1)
        U = seg.u[idx]
        F1, F2 = self.k_ref * U[:, 0], self.k_ref * U[:, 1]
        th, vx, vz, om = X[:, 2], X[:, 3], X[:, 4], X[:, 5]
        f = np.stack([-(F1 + F2) * np.sin(th), (F1 + F2) * np.cos(th), self.arm * (F2 - F1), vx, vz, om,
                      np.ones_like(t)], -1)
        cum = np.concatenate([np.zeros((1, f.shape[1])), np.cumsum(0.5 * (f[1:] + f[:-1]) * dt, 0)])
        k1 = np.arange(0, len(t) - w, st)
        k2 = k1 + w
        I = cum[k2] - cum[k1]
        return {'dyn.x': (np.stack([vx[k2] - vx[k1], I[:, 3]], -1), I[:, 0]),
                'dyn.z': (np.stack([vz[k2] - vz[k1], I[:, 4], I[:, 6]], -1), I[:, 1]),
                'dyn.th': (np.stack([om[k2] - om[k1], I[:, 5]], -1), I[:, 2])}

    def envelope_features(self, seg):
        return np.stack([seg.x[:, 3], seg.x[:, 4], seg.x[:, 5], seg.u.sum(1)], -1), ['vx', 'vz', 'omega', 'u_sum']
