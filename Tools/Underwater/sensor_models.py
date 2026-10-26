"""Reference sensor models for underwater emulation (oracles for future C# sensors).

All quantities are SI, body FRD / world NED as in fossen6dof.py.  Noise is
optional and seeded so that fixtures are reproducible.  The numbers in the
defaults are *engineering priors* (label UNKNOWN) unless a datasheet is cited
in the profile that configures them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from fossen6dof import G, RHO_FRESH, quat_to_rot

P_ATM = 101325.0


@dataclass
class PressureSensor:
    """Absolute pressure at a body-fixed port: p = p_atm + rho g depth."""

    port_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rho: float = RHO_FRESH
    sigma_pa: float = 0.0
    resolution_pa: float = 0.0
    surface_z: float = 0.0

    def measure(self, pos_n, q, rng: Optional[np.random.Generator] = None) -> float:
        z_port = pos_n[2] + (quat_to_rot(q) @ np.asarray(self.port_b, float))[2]
        depth = max(z_port - self.surface_z, 0.0)
        p = P_ATM + self.rho * G * depth
        if rng is not None and self.sigma_pa > 0:
            p += rng.normal(0.0, self.sigma_pa)
        if self.resolution_pa > 0:
            p = np.round(p / self.resolution_pa) * self.resolution_pa
        return float(p)

    def depth_from_pressure(self, p: float) -> float:
        return (p - P_ATM) / (self.rho * G)


def janus_beams(tilt_deg: float = 22.5, yaw_offset_deg: float = 45.0) -> np.ndarray:
    """Unit beam directions (body FRD, pointing down-and-out) of a 4-beam Janus DVL."""
    t = np.deg2rad(tilt_deg)
    out = []
    for k in range(4):
        a = np.deg2rad(yaw_offset_deg + 90.0 * k)
        out.append([np.sin(t) * np.cos(a), np.sin(t) * np.sin(a), np.cos(t)])
    return np.array(out)


@dataclass
class DVL:
    """Doppler velocity log with bottom-track and water-track modes.

    Each beam measures the projection of the *relative* velocity of the
    transducer along the beam.  Bottom track uses the seabed (ground velocity),
    water track uses a water layer (current-relative velocity).  A beam is
    invalid if its slant range to a flat seabed at ``seabed_z`` exceeds
    ``max_range`` or is below ``min_range``.  The velocity is solved by least
    squares, which needs at least 3 valid beams.
    """

    beams_b: np.ndarray = field(default_factory=janus_beams)
    mount_b: np.ndarray = field(default_factory=lambda: np.zeros(3))
    sigma_beam: float = 0.0
    min_range: float = 0.05
    max_range: float = 50.0

    def beam_ranges(self, pos_n, q, seabed_z: float) -> np.ndarray:
        R = quat_to_rot(q)
        p = pos_n + R @ self.mount_b
        dz = seabed_z - p[2]
        down = (self.beams_b @ R.T)[:, 2]
        with np.errstate(divide="ignore"):
            r = np.where(down > 1e-6, dz / down, np.inf)
        return np.where(r > 0, r, np.inf)

    def measure(self, pos_n, q, nu, seabed_z: float, current_n=None, mode: str = "bottom", rng=None):
        R = quat_to_rot(q)
        v_mount = nu[:3] + np.cross(nu[3:], self.mount_b)  # transducer velocity, body
        if mode == "water":
            c = np.zeros(3) if current_n is None else np.asarray(current_n, float)
            v_mount = v_mount - R.T @ c
        elif mode != "bottom":
            raise ValueError("mode must be 'bottom' or 'water'")
        ranges = self.beam_ranges(pos_n, q, seabed_z)
        valid = (ranges >= self.min_range) & (ranges <= self.max_range)
        if mode == "water":
            valid = np.ones(len(self.beams_b), bool)
        radial = self.beams_b @ v_mount
        if rng is not None and self.sigma_beam > 0:
            radial = radial + rng.normal(0.0, self.sigma_beam, size=radial.shape)
        if valid.sum() < 3:
            return None, radial, valid
        v, *_ = np.linalg.lstsq(self.beams_b[valid], radial[valid], rcond=None)
        return v, radial, valid


@dataclass
class USBL:
    """Ideal ultra-short-baseline fix: range and bearing from a {n} transceiver."""

    transceiver_n: np.ndarray = field(default_factory=lambda: np.zeros(3))
    sound_speed: float = 1500.0
    sigma_range: float = 0.0
    sigma_angle_rad: float = 0.0

    def measure(self, pos_n, rng=None):
        d = np.asarray(pos_n, float) - self.transceiver_n
        rng_m = float(np.linalg.norm(d))
        az = float(np.arctan2(d[1], d[0]))
        el = float(np.arcsin(d[2] / rng_m)) if rng_m > 0 else 0.0
        if rng is not None:
            rng_m += rng.normal(0.0, self.sigma_range) if self.sigma_range else 0.0
            az += rng.normal(0.0, self.sigma_angle_rad) if self.sigma_angle_rad else 0.0
            el += rng.normal(0.0, self.sigma_angle_rad) if self.sigma_angle_rad else 0.0
        return {"range": rng_m, "azimuth": az, "elevation": el, "latency": rng_m / self.sound_speed * 2.0}


def beer_lambert(radiance: np.ndarray, distance: float, attenuation_per_m: np.ndarray, backscatter: np.ndarray) -> np.ndarray:
    """Underwater image formation per channel (Jaffe-McGlamery / Akkaynak-Treibitz simplified):

    I = J exp(-c d) + B_inf (1 - exp(-c d)).
    """
    t = np.exp(-np.asarray(attenuation_per_m, float) * distance)
    return np.asarray(radiance, float) * t + np.asarray(backscatter, float) * (1.0 - t)
