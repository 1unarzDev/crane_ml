"""Pure-numpy 6-DOF Fossen reference model for underwater vehicles.

This module is an *analytic oracle* for CRANE underwater work.  It is not wired
into Unity and it does not replace any existing water component.  It exists so
that a future, flag-gated Unity implementation can be checked against an
independent, test-covered reference.

Conventions (Fossen 2011, "Handbook of Marine Craft Hydrodynamics and Motion
Control", ch. 2-3, 6 and 10):

* Inertial frame {n}: NED (x north, y east, z down).
* Body frame {b}: FRD (x forward, y starboard, z down), origin CO.
* nu = [u, v, w, p, q, r] body-frame velocities about CO.
* Attitude: unit quaternion q = [qw, qx, qy, qz] rotating {b} -> {n}.
  Euler ZYX helpers are provided for fixtures only.
* Added mass and damping are stored as *positive* matrices, i.e.
  M_A = -diag(X_udot, Y_vdot, ...) with the SNAME derivatives being negative.
  The hydrodynamic added-mass force is therefore -M_A * nudot_r (Fossen 6.2).
  This is the sign that FossenDynamics.cs:94 gets wrong (inventory V1).

Equation of motion with an irrotational current nu_c (Fossen 2011, eq. 10.1):

    M_RB nudot + C_RB(nu) nu + M_A nudot_r + C_A(nu_r) nu_r
        + D(nu_r) nu_r + g(eta) = tau + tau_ext

with nu_r = nu - nu_c and, for a current that is constant in {n},
nudot_c = [-S(omega) v_c^b, 0].  Both C matrices use the skew-symmetric
parameterisation of Fossen 2011 Thm 3.2 (eq. 3.46 / 6.43), so
nu^T C(nu) nu = 0 for any nu.  That property is what makes the energy
invariants in the tests exact (up to integration error).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

RHO_FRESH = 997.0
RHO_SEA = 1025.0
G = 9.80665


# --------------------------------------------------------------------------- #
# Small linear-algebra helpers
# --------------------------------------------------------------------------- #
def skew(a: Sequence[float]) -> np.ndarray:
    """S(a) with S(a) b = a x b."""
    a1, a2, a3 = (float(x) for x in a)
    return np.array([[0.0, -a3, a2], [a3, 0.0, -a1], [-a2, a1, 0.0]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero quaternion")
    q = q / n
    return q if q[0] >= 0.0 else -q


def quat_unit(q: np.ndarray) -> np.ndarray:
    """Normalise WITHOUT touching the sign.  Safe inside RK4 stages (no hemisphere flip)."""
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero quaternion")
    return q / n


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """R_b^n for q = [w, x, y, z] (Fossen eq. 2.69)."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_rate_matrix(q: np.ndarray) -> np.ndarray:
    """T(q) with qdot = T(q) omega_b (Fossen eq. 2.72)."""
    w, x, y, z = q
    return 0.5 * np.array([[-x, -y, -z], [w, -z, y], [z, w, -x], [-y, x, w]])


def euler_to_quat(phi: float, theta: float, psi: float) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles to quaternion."""
    cphi, sphi = np.cos(phi / 2), np.sin(phi / 2)
    cth, sth = np.cos(theta / 2), np.sin(theta / 2)
    cpsi, spsi = np.cos(psi / 2), np.sin(psi / 2)
    return quat_normalize(
        np.array(
            [
                cphi * cth * cpsi + sphi * sth * spsi,
                sphi * cth * cpsi - cphi * sth * spsi,
                cphi * sth * cpsi + sphi * cth * spsi,
                cphi * cth * spsi - sphi * sth * cpsi,
            ]
        )
    )


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    R = quat_to_rot(q)
    phi = np.arctan2(R[2, 1], R[2, 2])
    theta = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    psi = np.arctan2(R[1, 0], R[0, 0])
    return np.array([phi, theta, psi])


# --------------------------------------------------------------------------- #
# Vehicle parameters
# --------------------------------------------------------------------------- #
@dataclass
class VehicleParams:
    """Rigid-body and hydrodynamic parameters (SI, FRD body frame, about CO).

    ``inertia_cg`` is the 3x3 inertia about the centre of gravity.  ``added_mass``
    (6x6), ``linear_damping`` (6x6) and ``quadratic_damping`` (6,) are positive.
    ``weight`` W = m g.  ``buoyancy`` B = rho g displaced_volume.
    """

    mass: float
    inertia_cg: np.ndarray
    r_g: np.ndarray
    r_b: np.ndarray
    buoyancy: float
    added_mass: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))
    linear_damping: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))
    quadratic_damping: np.ndarray = field(default_factory=lambda: np.zeros(6))
    gravity: float = G
    name: str = "vehicle"

    def __post_init__(self) -> None:
        self.inertia_cg = np.asarray(self.inertia_cg, dtype=float).reshape(3, 3)
        self.r_g = np.asarray(self.r_g, dtype=float).reshape(3)
        self.r_b = np.asarray(self.r_b, dtype=float).reshape(3)
        self.added_mass = np.asarray(self.added_mass, dtype=float).reshape(6, 6)
        self.linear_damping = np.asarray(self.linear_damping, dtype=float).reshape(6, 6)
        self.quadratic_damping = np.asarray(self.quadratic_damping, dtype=float).reshape(6)
        self.validate()

    @property
    def weight(self) -> float:
        return self.mass * self.gravity

    def validate(self) -> None:
        if not self.mass > 0.0:
            raise ValueError("mass must be positive")
        if not np.allclose(self.inertia_cg, self.inertia_cg.T):
            raise ValueError("inertia_cg must be symmetric")
        if np.min(np.linalg.eigvalsh(self.inertia_cg)) <= 0.0:
            raise ValueError("inertia_cg must be positive definite")
        if not np.allclose(self.added_mass, self.added_mass.T, atol=1e-12):
            # Fossen 2011 sec. 6.3.1: M_A is symmetric for an ideal fluid at rest.
            raise ValueError("added_mass must be symmetric")
        if np.min(np.linalg.eigvalsh(self.added_mass)) < -1e-12:
            raise ValueError("added_mass must be positive semi-definite")
        sym_d = 0.5 * (self.linear_damping + self.linear_damping.T)
        if np.min(np.linalg.eigvalsh(sym_d)) < -1e-12:
            raise ValueError("linear_damping must be dissipative (D + D^T >= 0)")
        if np.any(self.quadratic_damping < 0.0):
            raise ValueError("quadratic_damping must be non-negative")
        if not self.buoyancy >= 0.0:
            raise ValueError("buoyancy must be non-negative")

    # Rigid-body mass matrix about CO (Fossen eq. 3.44).
    def m_rb(self) -> np.ndarray:
        m = self.mass
        S = skew(self.r_g)
        I_o = self.inertia_cg - m * S @ S  # parallel-axis theorem (eq. 3.37)
        M = np.zeros((6, 6))
        M[:3, :3] = m * np.eye(3)
        M[:3, 3:] = -m * S
        M[3:, :3] = m * S
        M[3:, 3:] = I_o
        return M

    def mass_matrix(self) -> np.ndarray:
        return self.m_rb() + self.added_mass

    @property
    def bg(self) -> float:
        """Metacentric height of a submerged body: vertical CG-below-CB distance."""
        return float(self.r_g[2] - self.r_b[2])


def coriolis_from_mass(M: np.ndarray, nu: np.ndarray) -> np.ndarray:
    """Skew-symmetric C(nu) for a symmetric mass matrix (Fossen Thm 3.2)."""
    M11, M12, M21, M22 = M[:3, :3], M[:3, 3:], M[3:, :3], M[3:, 3:]
    nu1, nu2 = nu[:3], nu[3:]
    a = M11 @ nu1 + M12 @ nu2
    b = M21 @ nu1 + M22 @ nu2
    C = np.zeros((6, 6))
    C[:3, 3:] = -skew(a)
    C[3:, :3] = -skew(a)
    C[3:, 3:] = -skew(b)
    return C


def restoring(params: VehicleParams, q: np.ndarray) -> np.ndarray:
    """g(eta): restoring generalised force on the LHS (Fossen eq. 4.6)."""
    R = quat_to_rot(q)
    W, B = params.weight, params.buoyancy
    f_g = R.T @ np.array([0.0, 0.0, W])  # weight in body
    f_b = -R.T @ np.array([0.0, 0.0, B])  # buoyancy in body
    return -np.concatenate([f_g + f_b, np.cross(params.r_g, f_g) + np.cross(params.r_b, f_b)])


def damping_force(params: VehicleParams, nu_r: np.ndarray) -> np.ndarray:
    """D(nu_r) nu_r with diagonal quadratic terms (Fossen eq. 6.67)."""
    return params.linear_damping @ nu_r + params.quadratic_damping * np.abs(nu_r) * nu_r


def potential_energy(params: VehicleParams, z: float, q: np.ndarray) -> float:
    """U such that -dU/deta (mapped to body) equals -g(eta)."""
    R = quat_to_rot(q)
    z_g = z + (R @ params.r_g)[2]
    z_b = z + (R @ params.r_b)[2]
    return float(-params.weight * z_g + params.buoyancy * z_b)


# --------------------------------------------------------------------------- #
# Thrusters, allocation and tether
# --------------------------------------------------------------------------- #
@dataclass
class Thruster:
    position: np.ndarray  # body FRD, metres, relative to CO
    direction: np.ndarray  # unit thrust direction for positive command
    max_forward: float  # N
    max_reverse: float  # N (positive number)
    time_constant: float = 0.0  # first-order lag, s (0 = instantaneous)
    name: str = ""

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        d = np.asarray(self.direction, dtype=float).reshape(3)
        self.direction = d / np.linalg.norm(d)


def allocation_matrix(thrusters: Sequence[Thruster]) -> np.ndarray:
    """T (6 x n): tau = T f (Fossen eq. 11.10)."""
    cols = [np.concatenate([t.direction, np.cross(t.position, t.direction)]) for t in thrusters]
    return np.array(cols).T


def allocate(T: np.ndarray, tau: np.ndarray, f_min: np.ndarray, f_max: np.ndarray) -> np.ndarray:
    """Unconstrained least-norm allocation, then uniform scaling into limits.

    Uniform scaling keeps the wrench *direction* (no cross-coupling), unlike
    per-thruster clipping or allocating in shaft speed (inventory H2).
    """
    f = np.linalg.pinv(T) @ tau
    scale = 1.0
    for fi, lo, hi in zip(f, f_min, f_max):
        if fi > hi > 0:
            scale = min(scale, hi / fi)
        elif fi < lo < 0:
            scale = min(scale, lo / fi)
    return f * scale


@dataclass
class Tether:
    """Taut-only elastic tether between a {n} anchor and a body attachment point.

    Zero force while slack; spring-damper along the line when taut.  This is a
    deliberately minimal fixture model, not a cable-dynamics model.
    """

    anchor_n: np.ndarray
    attach_b: np.ndarray
    length: float
    stiffness: float
    damping: float = 0.0

    def wrench(self, pos_n: np.ndarray, q: np.ndarray, nu: np.ndarray) -> np.ndarray:
        R = quat_to_rot(q)
        p_att = pos_n + R @ np.asarray(self.attach_b, float)
        d = np.asarray(self.anchor_n, float) - p_att
        dist = float(np.linalg.norm(d))
        if dist <= self.length or dist == 0.0:
            return np.zeros(6)
        e = d / dist
        v_att_n = R @ (nu[:3] + np.cross(nu[3:], self.attach_b))
        stretch_rate = -float(np.dot(v_att_n, e))
        mag = self.stiffness * (dist - self.length) + self.damping * max(stretch_rate, 0.0)
        f_b = R.T @ (mag * e)
        return np.concatenate([f_b, np.cross(self.attach_b, f_b)])


# --------------------------------------------------------------------------- #
# Simulator
# --------------------------------------------------------------------------- #
@dataclass
class State:
    pos: np.ndarray  # NED position of CO
    q: np.ndarray  # quaternion b->n
    nu: np.ndarray  # body velocities
    thrust: np.ndarray = field(default_factory=lambda: np.zeros(0))  # per-thruster force

    def copy(self) -> "State":
        return State(self.pos.copy(), self.q.copy(), self.nu.copy(), self.thrust.copy())

    @staticmethod
    def at_rest(pos=(0.0, 0.0, 0.0), euler=(0.0, 0.0, 0.0), n_thrusters: int = 0) -> "State":
        return State(np.array(pos, float), euler_to_quat(*euler), np.zeros(6), np.zeros(n_thrusters))


@dataclass
class Simulator:
    params: VehicleParams
    current_n: np.ndarray = field(default_factory=lambda: np.zeros(3))
    thrusters: Sequence[Thruster] = ()
    tether: Optional[Tether] = None
    enable_added_mass: bool = True
    enable_coriolis: bool = True
    enable_damping: bool = True
    enable_restoring: bool = True

    def __post_init__(self) -> None:
        self.current_n = np.asarray(self.current_n, float).reshape(3)
        self._M_rb = self.params.m_rb()
        self._M_a = self.params.added_mass if self.enable_added_mass else np.zeros((6, 6))
        self._M_inv = np.linalg.inv(self._M_rb + self._M_a)
        self._T = allocation_matrix(self.thrusters) if self.thrusters else np.zeros((6, 0))

    # --- forces ------------------------------------------------------------
    def relative_velocity(self, q: np.ndarray, nu: np.ndarray) -> np.ndarray:
        R = quat_to_rot(q)
        nu_c = np.concatenate([R.T @ self.current_n, np.zeros(3)])
        return nu - nu_c

    def acceleration(self, s: State, tau: np.ndarray) -> np.ndarray:
        R = quat_to_rot(s.q)
        v_c_b = R.T @ self.current_n
        nu_r = s.nu - np.concatenate([v_c_b, np.zeros(3)])
        # d/dt of the body-frame current for a current constant in {n}.
        nudot_c = np.concatenate([-np.cross(s.nu[3:], v_c_b), np.zeros(3)])
        rhs = np.array(tau, float).reshape(6).copy()
        if self.enable_coriolis:
            rhs -= coriolis_from_mass(self._M_rb, s.nu) @ s.nu
            rhs -= coriolis_from_mass(self._M_a, nu_r) @ nu_r
        if self.enable_damping:
            rhs -= damping_force(self.params, nu_r)
        if self.enable_restoring:
            rhs -= restoring(self.params, s.q)
        if self.tether is not None:
            rhs += self.tether.wrench(s.pos, s.q, s.nu)
        rhs += self._M_a @ nudot_c
        return self._M_inv @ rhs

    def _deriv(self, s: State, tau_ext: np.ndarray, f_cmd: Optional[np.ndarray]):
        tau = np.array(tau_ext, float).reshape(6).copy()
        fdot = np.zeros_like(s.thrust)
        if len(self.thrusters):
            tau += self._T @ s.thrust
            if f_cmd is not None:
                for i, t in enumerate(self.thrusters):
                    target = float(np.clip(f_cmd[i], -t.max_reverse, t.max_forward))
                    if t.time_constant > 0.0:
                        fdot[i] = (target - s.thrust[i]) / t.time_constant
        posdot = quat_to_rot(s.q) @ s.nu[:3]
        qdot = quat_rate_matrix(s.q) @ s.nu[3:]
        nudot = self.acceleration(s, tau)
        return posdot, qdot, nudot, fdot

    def step(self, s: State, dt: float, tau_ext=None, f_cmd=None) -> State:
        """One classical RK4 step.  Instantaneous thrusters snap to the command."""
        tau_ext = np.zeros(6) if tau_ext is None else np.asarray(tau_ext, float)
        if f_cmd is not None and len(self.thrusters):
            s = s.copy()
            for i, t in enumerate(self.thrusters):
                if t.time_constant <= 0.0:
                    s.thrust[i] = float(np.clip(f_cmd[i], -t.max_reverse, t.max_forward))

        def add(base: State, k, h: float, canonical: bool = False) -> State:
            # Stage states must keep the sign of the step's initial quaternion: flipping q -> -q inside a
            # stage flips that stage's qdot and corrupts the RK4 combination near w = 0 (>180 deg rotations).
            # The hemisphere (w >= 0) is canonicalised only after the complete step.
            q_new = base.q + h * k[1]
            return State(
                base.pos + h * k[0],
                quat_normalize(q_new) if canonical else quat_unit(q_new),
                base.nu + h * k[2],
                base.thrust + h * k[3],
            )

        k1 = self._deriv(s, tau_ext, f_cmd)
        k2 = self._deriv(add(s, k1, dt / 2), tau_ext, f_cmd)
        k3 = self._deriv(add(s, k2, dt / 2), tau_ext, f_cmd)
        k4 = self._deriv(add(s, k3, dt), tau_ext, f_cmd)
        comb = [(a + 2 * b + 2 * c + d) / 6.0 for a, b, c, d in zip(k1, k2, k3, k4)]
        return add(s, comb, dt, canonical=True)

    def simulate(
        self,
        s0: State,
        dt: float,
        steps: int,
        tau_fn: Optional[Callable[[float, State], np.ndarray]] = None,
        f_cmd_fn: Optional[Callable[[float, State], np.ndarray]] = None,
    ) -> list[State]:
        out = [s0.copy()]
        s = s0.copy()
        for k in range(steps):
            t = k * dt
            tau = tau_fn(t, s) if tau_fn else None
            f_cmd = f_cmd_fn(t, s) if f_cmd_fn else None
            s = self.step(s, dt, tau, f_cmd)
            out.append(s)
        return out

    # --- invariants -------------------------------------------------------
    def kinetic_energy(self, s: State) -> float:
        """Relative kinetic energy 0.5 nu^T M_RB nu + 0.5 nu_r^T M_A nu_r."""
        nu_r = self.relative_velocity(s.q, s.nu)
        return float(0.5 * s.nu @ self._M_rb @ s.nu + 0.5 * nu_r @ self._M_a @ nu_r)

    def total_energy(self, s: State) -> float:
        e = self.kinetic_energy(s)
        if self.enable_restoring:
            e += potential_energy(self.params, s.pos[2], s.q)
        return e

    def dissipated_power(self, s: State) -> float:
        nu_r = self.relative_velocity(s.q, s.nu)
        return float(nu_r @ damping_force(self.params, nu_r)) if self.enable_damping else 0.0
