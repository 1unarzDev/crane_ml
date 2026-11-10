# SPDX-License-Identifier: GPL-3.0-or-later
"""Point-mass 7-inch quad at constant altitude with tilt/thrust, jerk and tracking-lag limits.

Horizontal thrust acceleration |a_T| <= a_h_max = min(g tan(tilt_max), g sqrt((TW u_max)^2 - 1)), with TW from the
AOS7 profile (Tools/Aerial/profiles/aos7_v5_f90_6s.json, derived.thrust_to_weight; IMPLEMENTED, not calibrated).
Aerodynamic drag (rotor drag 0.35/s linear + quadratic airframe drag, same profile) acts on the air-relative velocity,
so braking authority is direction dependent: decelerating gets a_T + drag, accelerating gets a_T - drag, and holding
cruise consumes part of the tilt budget that is then unavailable for lateral avoidance.
The velocity controller is a P loop (time constant tau_track, PX4-like) whose output slews at <= jerk_max.
Gusts: Ornstein-Uhlenbeck wind velocity (Dryden-like first-order spectrum) per horizontal axis.
Velocity command (m/s) and force/acceleration terms (N, m/s^2) are kept in separate fields and never mixed.
"""
import json
import math
import os

G = 9.80665
_PROFILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Aerial",
                                         "profiles", "aos7_v5_f90_6s.json"))

WIND = {  # under-canopy wind velocity statistics [U: canopy attenuates open-field wind ~3-5x]
    "calm": (0.0, 0.0, 2.0),
    "light_breeze": (0.4, 0.5, 2.0),     # mean m/s, sigma m/s, correlation time s
    "gusty": (0.8, 1.2, 1.5),
    "moderate_gusty": (1.2, 1.6, 1.5),
}


def load_params(profile_path=_PROFILE, tilt_max_deg=40.0, u_max=0.8, tau_track=0.30, jerk_max=30.0,
                r_body=0.25, yaw_rate_max=3.0):
    p = {"mass_kg": 1.3832, "thrust_to_weight": 6.38, "rotor_drag_per_s": 0.35, "drag_quadratic": 0.00858,
         "profile_id": None, "profile_label": "fallback constants"}
    if os.path.exists(profile_path):
        with open(profile_path) as f:
            pr = json.load(f)
        p["mass_kg"] = float(pr["parameters"]["mass"]["value"])
        p["thrust_to_weight"] = float(pr["derived"]["thrust_to_weight"])
        p["drag_quadratic"] = float(pr["parameters"]["drag_quadratic"]["value"])
        p["profile_id"] = pr.get("id")
        p["profile_label"] = pr.get("evidenceLabel")
    a_tilt = G * math.tan(math.radians(tilt_max_deg))
    a_tw = G * math.sqrt(max((p["thrust_to_weight"] * u_max) ** 2 - 1.0, 0.0))
    p.update({"tilt_max_deg": tilt_max_deg, "u_max": u_max, "a_h_max": min(a_tilt, a_tw),
              "a_h_tilt": a_tilt, "a_h_thrust": a_tw, "tau_track": tau_track, "jerk_max": jerk_max,
              "r_body": r_body, "yaw_rate_max": yaw_rate_max})
    return p


def drag_accel(p, vrx, vry):
    """Drag acceleration (m/s^2) for air-relative velocity (m/s)."""
    sp = math.hypot(vrx, vry)
    k = p["rotor_drag_per_s"] + p["drag_quadratic"] * sp / p["mass_kg"]
    return -k * vrx, -k * vry


def braking_authority(p, speed, direction="brake"):
    """Available deceleration (m/s^2) along/against the velocity at `speed` in still air."""
    ad = p["rotor_drag_per_s"] * speed + p["drag_quadratic"] * speed * speed / p["mass_kg"]
    if direction == "brake":
        return p["a_h_max"] + ad
    if direction == "accelerate":
        return p["a_h_max"] - ad
    return math.sqrt(max(p["a_h_max"] ** 2 - ad * ad, 0.0))  # lateral while holding speed


class Vehicle:
    __slots__ = ("p", "x", "y", "vx", "vy", "atx", "aty", "yaw", "wx", "wy", "w_mean", "w_sig", "w_tau",
                 "ax", "ay", "yaw_rate")

    def __init__(self, p, x=0.0, y=0.0, yaw=0.0, wind="calm"):
        self.p = p
        self.x, self.y, self.vx, self.vy = x, y, 0.0, 0.0
        self.atx = self.aty = 0.0           # thrust-horizontal acceleration (m/s^2)
        self.ax = self.ay = 0.0             # total coordinate acceleration (m/s^2)
        self.yaw, self.yaw_rate = yaw, 0.0
        self.w_mean, self.w_sig, self.w_tau = WIND[wind]
        self.wx, self.wy = self.w_mean, 0.0

    def step(self, dt, vcx, vcy, yaw_cmd, nx, ny):
        """Advance dt with velocity command (vcx, vcy) [m/s], yaw command [rad], unit normals nx, ny for the gust."""
        p = self.p
        # OU wind (exact discretisation)
        a = math.exp(-dt / self.w_tau)
        s = self.w_sig * math.sqrt(1.0 - a * a)
        self.wx = self.w_mean + a * (self.wx - self.w_mean) + s * nx
        self.wy = a * self.wy + s * ny
        dax, day = drag_accel(p, self.vx - self.wx, self.vy - self.wy)
        # velocity P loop + drag feedforward of the still-air drag (the controller does not know the wind)
        ffx, ffy = drag_accel(p, self.vx, self.vy)
        kdx = (vcx - self.vx) / p["tau_track"] - ffx
        kdy = (vcy - self.vy) / p["tau_track"] - ffy
        n = math.hypot(kdx, kdy)
        amax = p["a_h_max"]
        if n > amax:
            kdx, kdy = kdx * amax / n, kdy * amax / n
        # jerk limit on the thrust acceleration
        jx, jy = kdx - self.atx, kdy - self.aty
        jn = math.hypot(jx, jy)
        jl = p["jerk_max"] * dt
        if jn > jl:
            jx, jy = jx * jl / jn, jy * jl / jn
        self.atx += jx
        self.aty += jy
        self.ax, self.ay = self.atx + dax, self.aty + day
        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        e = (yaw_cmd - self.yaw + math.pi) % (2 * math.pi) - math.pi
        lim = p["yaw_rate_max"] * dt
        e = max(-lim, min(lim, e))
        self.yaw_rate = e / dt
        self.yaw = (self.yaw + e + math.pi) % (2 * math.pi) - math.pi

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)
