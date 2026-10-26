"""Pure-python rigid-body multirotor reference model for CRANE (numpy only).

Conventions: world frame W is right-handed z-up (ENU/FLU-world), body frame B is FLU
(x forward, y left, z up).  Quaternions are Hamilton [w, x, y, z] rotating B -> W.
Body rates `omega` are expressed in B.  Rotor `spin` is +1 when the rotor turns
counter-clockwise seen from above (+z_B); its aerodynamic reaction torque on the body
is then -spin * km * T about the rotor axis (same law as gz-sim MulticopterMotorModel:
dragTorque = -turningDirection * thrust * momentConstant).

Force/torque terms (each can be disabled through the profile):
  * thrust          T_i = kf * Omega_i^2 * ground_effect(h_i) along rotor axis
  * reaction torque -spin_i * km * T_i about rotor axis
  * rotor drag      -|Omega_i| * lambda1 * v_perp,i   (Martin & Salaun 2010 form; gz-sim)
  * rolling moment  -|Omega_i| * mu1 * v_perp,i       (gz-sim)
  * body drag       -D_lin (.) v_air_B - c_quad * |v_air| * v_air   (rotation-consistent)
  * gravity, rigid-body Euler equations with full inertia tensor
  * motors: asymmetric first-order lag (tau_up / tau_down), exact ZOH update per step
  * battery: Thevenin source V = Voc(soc) - I R; Omega_max scales with V / V_nominal;
             shaft power P_i = km * kf * Omega_i^3 drawn through efficiency eta
  * ground effect: Cheeseman-Bennett T_IGE/T_OGE = 1 / (1 - (R / 4h)^2), clamped.

Integration: motors are advanced exactly over dt, then the rigid body is advanced with
classical RK4 holding rotor speeds constant over the step.
"""

from dataclasses import dataclass, field
import math

import numpy as np


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_matrix(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.concatenate([[math.cos(angle / 2.0)], math.sin(angle / 2.0) * axis])


def matrix_to_quat(r):
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (r[k, j] - r[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (r[j, i] + r[i, j]) / s
        q[1 + k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


@dataclass
class Rotor:
    position: np.ndarray            # body FLU [m]
    spin: int                       # +1 CCW seen from above, -1 CW
    kf: float                       # N / (rad/s)^2
    km: float                       # m (reaction torque / thrust)
    omega_max: float                # rad/s at nominal voltage
    omega_min: float = 0.0
    tau_up: float = 0.02
    tau_down: float = 0.02
    rotor_drag: float = 0.0         # lambda1 [N s / (m rad)]
    rolling_moment: float = 0.0     # mu1
    radius: float = 0.0             # m, used by ground effect
    axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))


@dataclass
class Battery:
    v_full: float
    v_empty: float
    v_nominal: float
    resistance: float               # ohm
    capacity_ah: float
    efficiency: float = 0.7         # shaft power / electrical power

    def open_circuit(self, soc):
        return self.v_empty + (self.v_full - self.v_empty) * float(np.clip(soc, 0.0, 1.0))


@dataclass
class Vehicle:
    name: str
    mass: float
    inertia: np.ndarray
    rotors: list
    gravity: float = 9.81
    drag_linear: np.ndarray = field(default_factory=lambda: np.zeros(3))
    drag_quadratic: float = 0.0
    ground_effect: bool = False
    ground_effect_min_height_ratio: float = 0.5   # clamp h >= ratio * R
    ground_height: float = 0.0
    battery: Battery = None


@dataclass
class State:
    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    omega: np.ndarray
    rotor_speed: np.ndarray
    soc: float = 1.0
    t: float = 0.0

    def copy(self):
        return State(self.p.copy(), self.v.copy(), self.q.copy(), self.omega.copy(),
                     self.rotor_speed.copy(), self.soc, self.t)


def initial_state(vehicle, p=(0, 0, 1), rotor_speed=None, q=(1, 0, 0, 0)):
    n = len(vehicle.rotors)
    speeds = np.zeros(n) if rotor_speed is None else np.broadcast_to(
        np.asarray(rotor_speed, dtype=float), (n,)).copy()
    return State(np.array(p, dtype=float), np.zeros(3), np.array(q, dtype=float),
                 np.zeros(3), speeds)


def hover_rotor_speed(vehicle):
    """Equal-speed hover (requires symmetric thrust axes along +z_B)."""
    kf_sum = sum(r.kf for r in vehicle.rotors)
    return math.sqrt(vehicle.mass * vehicle.gravity / kf_sum)


def ground_effect_factor(vehicle, rotor, height):
    if not vehicle.ground_effect or rotor.radius <= 0.0:
        return 1.0
    h = max(height, vehicle.ground_effect_min_height_ratio * rotor.radius)
    ratio = rotor.radius / (4.0 * h)
    return 1.0 / (1.0 - ratio * ratio)


def battery_voltage(vehicle, state, current):
    b = vehicle.battery
    if b is None:
        return None
    return b.open_circuit(state.soc) - current * b.resistance


def electrical_current(vehicle, state):
    """Fixed-point solve of I = P / (eta V(I)) for the current rotor speeds."""
    b = vehicle.battery
    if b is None:
        return 0.0
    power = sum(r.km * r.kf * abs(w) ** 3 for r, w in zip(vehicle.rotors, state.rotor_speed))
    power /= b.efficiency
    voc = b.open_circuit(state.soc)
    disc = voc * voc - 4.0 * b.resistance * power
    if disc < 0.0:
        return voc / (2.0 * b.resistance)   # maximum-power point: source collapses
    return (voc - math.sqrt(disc)) / (2.0 * b.resistance)


def rotor_speed_limit(vehicle, rotor, state):
    b = vehicle.battery
    if b is None:
        return rotor.omega_max
    current = electrical_current(vehicle, state)
    volts = max(0.0, battery_voltage(vehicle, state, current))
    return rotor.omega_max * min(1.0, volts / b.v_nominal)


def wrench(vehicle, state, wind_world=np.zeros(3)):
    """Body-frame force and torque (excluding gravity)."""
    r_wb = quat_to_matrix(state.q)
    v_air_b = r_wb.T @ (state.v - np.asarray(wind_world, dtype=float))
    force = np.zeros(3)
    torque = np.zeros(3)
    for rotor, speed in zip(vehicle.rotors, state.rotor_speed):
        axis = rotor.axis / np.linalg.norm(rotor.axis)
        hub_world = state.p + r_wb @ rotor.position
        height = hub_world[2] - vehicle.ground_height
        thrust = rotor.kf * speed * speed * ground_effect_factor(vehicle, rotor, height)
        f = thrust * axis
        v_hub = v_air_b + np.cross(state.omega, rotor.position)
        v_perp = v_hub - np.dot(v_hub, axis) * axis
        f = f - abs(speed) * rotor.rotor_drag * v_perp
        force += f
        torque += np.cross(rotor.position, f)
        torque += -rotor.spin * rotor.km * thrust * axis
        torque += -abs(speed) * rotor.rolling_moment * v_perp
    speed_air = np.linalg.norm(v_air_b)
    force += -np.asarray(vehicle.drag_linear) * v_air_b - vehicle.drag_quadratic * speed_air * v_air_b
    return force, torque


def _derivative(vehicle, state, wind_world):
    force_b, torque_b = wrench(vehicle, state, wind_world)
    r_wb = quat_to_matrix(state.q)
    acc = r_wb @ force_b / vehicle.mass + np.array([0.0, 0.0, -vehicle.gravity])
    inertia = np.asarray(vehicle.inertia)
    omega = state.omega
    omega_dot = np.linalg.solve(inertia, torque_b - np.cross(omega, inertia @ omega))
    q_dot = 0.5 * quat_mul(state.q, np.concatenate([[0.0], omega]))
    return state.v.copy(), acc, q_dot, omega_dot


def specific_force_body(vehicle, state, wind_world=np.zeros(3)):
    """What an ideal accelerometer measures (FLU body): (F_nongravitational)/m."""
    force_b, _ = wrench(vehicle, state, wind_world)
    return force_b / vehicle.mass


def update_motors(vehicle, state, command, dt):
    command = np.asarray(command, dtype=float)
    for i, rotor in enumerate(vehicle.rotors):
        limit = rotor_speed_limit(vehicle, rotor, state)
        target = float(np.clip(command[i], rotor.omega_min, limit))
        tau = rotor.tau_up if target >= state.rotor_speed[i] else rotor.tau_down
        alpha = 1.0 if tau <= 0.0 else 1.0 - math.exp(-dt / tau)
        state.rotor_speed[i] += (target - state.rotor_speed[i]) * alpha


def step(vehicle, state, command, dt, wind_world=np.zeros(3), contact=False):
    """Advance one fixed step. `command` is per-rotor target speed [rad/s]."""
    update_motors(vehicle, state, command, dt)
    s0 = state
    k1 = _derivative(vehicle, s0, wind_world)

    def shifted(k, h):
        s = s0.copy()
        s.p = s0.p + h * k[0]
        s.v = s0.v + h * k[1]
        s.q = s0.q + h * k[2]
        s.q /= np.linalg.norm(s.q)
        s.omega = s0.omega + h * k[3]
        return s

    k2 = _derivative(vehicle, shifted(k1, dt / 2), wind_world)
    k3 = _derivative(vehicle, shifted(k2, dt / 2), wind_world)
    k4 = _derivative(vehicle, shifted(k3, dt), wind_world)
    new = s0.copy()
    new.p = s0.p + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
    new.v = s0.v + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    new.q = s0.q + dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
    new.q /= np.linalg.norm(new.q)
    new.omega = s0.omega + dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
    if vehicle.battery is not None:
        current = electrical_current(vehicle, s0)
        new.soc = s0.soc - current * dt / (3600.0 * vehicle.battery.capacity_ah)
    if contact and new.p[2] < vehicle.ground_height:
        new.p[2] = vehicle.ground_height
        new.v[2] = max(0.0, new.v[2])
    new.t = s0.t + dt
    return new


def mechanical_energy(vehicle, state):
    inertia = np.asarray(vehicle.inertia)
    return (0.5 * vehicle.mass * state.v @ state.v + 0.5 * state.omega @ inertia @ state.omega
            + vehicle.mass * vehicle.gravity * state.p[2])
