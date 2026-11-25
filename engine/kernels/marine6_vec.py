"""``VecDynamics``: the sub platform's batched-plant interface (N vehicles, one parameter set, per-call NED
current) over the marine6 kernel, so code written against it runs on main's kernel unchanged.

    dyn = VecDynamics(params, thrusters, thruster_gain=1.0)     # marine6_ref.VehicleParams, [Thruster]
    f = dyn.allocate(tau)                                       # [N, 6] -> [N, n]
    pos, q, nu, thr, nu_dot = dyn.step(pos, q, nu, thr, dt, tau_ext, f_tgt, cur_n)   # one RK4 step of dt
    nu_dot = dyn.acceleration(q, nu, thr, tau_ext, cur_n)

Provenance: the interface (names, attributes, return values) is the sub platform branch's
``crane_sub/plant/dynamics.py`` (commit 04c0066c, the same owner, GPL-3.0); the numbers are the kernel's.
``thruster_gain`` scales the realised thrust (``T``); the allocation keeps the nominal matrix (``T_alloc``).
"""

import numpy as np

from engine.kernels import marine6 as K, marine6_ref as REF


class VecDynamics:
    def __init__(self, params, thrusters, thruster_gain: float = 1.0):
        self.params = params
        self.thrusters = list(thrusters)
        self.n_thr = n = len(self.thrusters)
        self.M_rb = params.m_rb()
        self.M_a = params.added_mass.copy()
        self.M_inv = np.linalg.inv(self.M_rb + self.M_a)
        self.T_alloc = REF.allocation_matrix(self.thrusters)
        self.T = self.T_alloc * float(thruster_gain)
        self.T_pinv = np.linalg.pinv(self.T_alloc, rcond=REF.ALLOC_RCOND)
        self.f_max = np.array([t.max_forward for t in self.thrusters])
        self.f_min = -np.array([t.max_reverse for t in self.thrusters])
        self.tc = np.array([t.time_constant for t in self.thrusters])
        self.lag = self.tc > 0.0
        self.inv_tc = np.where(self.lag, 1.0 / np.where(self.lag, self.tc, 1.0), 0.0)
        self.Dl = params.linear_damping.copy()
        self.Dq = params.quadratic_damping.copy()
        self.r_g, self.r_b = params.r_g.copy(), params.r_b.copy()
        self.W, self.B = params.weight, params.buoyancy
        self.p = K.pack_params(params, self.thrusters, (0.0, 0.0, 0.0), thruster_gain)
        self._pad = K.NT - n

    def _rows(self, N, cur_n):
        P = np.repeat(self.p[None], N, axis=0)
        P[:, K.CUR:K.CUR + 3] = cur_n
        return P

    def _state(self, pos, q, nu, thr):
        N = q.shape[0]
        return np.concatenate([pos, q, nu, thr, np.zeros((N, self._pad))], axis=1)

    def _force(self, f):
        return np.concatenate([f, np.zeros((f.shape[0], self._pad))], axis=1)

    def allocate(self, tau: np.ndarray) -> np.ndarray:
        """(N,6) wrench -> (N,n) forces: min-norm, then uniform scaling into the limits (marine6_ref.allocate)."""
        return K.allocate(np.asarray(tau, float), self.p[None])[:, :self.n_thr]

    @staticmethod
    def rot(q):
        return K.rot(np, q)

    def _c_nu(self, M, nu):
        return K._c_nu(np, M, nu)

    def acceleration(self, q, nu, thr, tau_ext, cur_n, R=None):
        x = self._state(np.zeros((q.shape[0], 3)), q, nu, thr)
        return K.deriv(np, x, x[:, K.THR:], tau_ext, self._rows(q.shape[0], cur_n))[2]

    def deriv(self, pos, q, nu, thr, tau_ext, f_tgt, cur_n):
        P = self._rows(q.shape[0], cur_n)
        f = np.clip(self._force(f_tgt), P[:, K.FMIN:K.FMIN + K.NT], P[:, K.FMAX:K.FMAX + K.NT])
        posdot, qdot, nudot, fdot = K.deriv(np, self._state(pos, q, nu, thr), f, tau_ext, P)
        return posdot, qdot, nudot, fdot[:, :self.n_thr]

    def step(self, pos, q, nu, thr, dt, tau_ext, f_tgt, cur_n):
        """RK4 step. Returns pos, q, nu, thr, nu_dot (acceleration at the step start, after the snap)."""
        N, n = q.shape[0], self.n_thr
        P = self._rows(N, cur_n)
        u = np.concatenate([tau_ext, self._force(f_tgt)], axis=1)
        snapped = np.where(self.lag[None], thr, np.clip(f_tgt, self.f_min, self.f_max))
        x0 = self._state(pos, q, nu, snapped)
        nu_dot = K.deriv(np, x0, x0[:, K.THR:], tau_ext, P)[2]
        x = K.step(x0, u, P, dt)
        return x[:, :3], x[:, K.QUAT:K.QUAT + 4], x[:, K.NU:K.NU + 6], x[:, K.THR:K.THR + n], nu_dot
