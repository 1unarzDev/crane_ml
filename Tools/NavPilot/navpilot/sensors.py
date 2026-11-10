# SPDX-License-Identifier: GPL-3.0-or-later
"""In-plane sensor models: OAK-D-like stereo depth, Unitree-L2-like LiDAR, compute stalls, person detection.

All parameters are [U] priors from vendor sheets (OAK-D S2: 80 deg HFOV stereo, 7.5 cm baseline; L2: 360x96 deg,
64 kpts/s, 10 Hz, +-2 cm) and stereo/LiDAR first principles; none is calibrated against this airframe.

Ray casting uses an angular z-buffer: each circle covers the bearing bins its angular interval spans (always at least
the bin of its centre, so thin branches register), the nearest surface per bin wins. Thin-structure dropouts are drawn
per object per frame (stereo: one matched/not-matched event per branch) or per bin (LiDAR: Poisson point hits).
"""
import math

import numpy as np

STEREO = {"hfov_deg": 80.0, "width_px": 640, "baseline_m": 0.075, "disp_sigma_px": 0.15, "range_max": 15.0,
          "rate_hz": 15.0, "latency_s": 0.045, "exposure_s": 0.008, "w50_px": 3.0, "w_scale_px": 0.8,
          "invalid_p": 0.03, "n_bins": 96, "free_evidence_m": 8.0}
LIDAR = {"range_max": 20.0, "rate_hz": 10.0, "latency_s": 0.065, "pts_per_s": 64000.0, "vfov_deg": 96.0,
         "range_sigma": 0.02, "n_bins": 720, "stem_band_m": 2.0}
RGB = {"hfov_deg": 66.0, "width_px": 640, "rate_hz": 10.0, "exposure_s": 0.008, "range_max": 30.0,
       "h50_px": 18.0, "h_scale_px": 5.0, "blur_scale_px": 6.0, "p_max": 0.9, "confirm": 2}

# Compute-time model per planning cycle: lognormal body + Bernoulli-gated Pareto stall.
LATENCY_TAILS = {
    "nominal": {"median_s": 0.025, "sigma": 0.25, "p_stall": 0.003, "stall_min_s": 0.04, "alpha": 2.5},
    "heavy": {"median_s": 0.030, "sigma": 0.35, "p_stall": 0.04, "stall_min_s": 0.06, "alpha": 1.8},
    "severe": {"median_s": 0.035, "sigma": 0.45, "p_stall": 0.10, "stall_min_s": 0.10, "alpha": 1.4},
}


def compute_time(tail, u1, u2, u3):
    """Inverse-transform sample from three uniforms (common random numbers across tail settings)."""
    c = LATENCY_TAILS[tail]
    z = math.sqrt(2.0) * _erfinv(2.0 * u1 - 1.0)
    t = c["median_s"] * math.exp(c["sigma"] * z)
    if u2 < c["p_stall"]:
        t += c["stall_min_s"] * max(u3, 1e-12) ** (-1.0 / c["alpha"])
    return min(t, 3.0)


def _erfinv(y):
    # Giles (2010) single-precision approximation; adequate for sampling.
    y = max(min(y, 1 - 1e-12), -1 + 1e-12)
    w = -math.log((1.0 - y) * (1.0 + y))
    if w < 5.0:
        w -= 2.5
        p = 2.81022636e-08
        for c in (3.43273939e-07, -3.5233877e-06, -4.39150654e-06, 0.00021858087, -0.00125372503,
                  -0.00417768164, 0.246640727, 1.50140941):
            p = c + p * w
    else:
        w = math.sqrt(w) - 3.0
        p = -0.000200214257
        for c in (0.000100950558, 0.00134934322, -0.00367342844, 0.00573950773, -0.0076224613,
                  0.00943887047, 1.00167406, 2.83297682):
            p = c + p * w
    return p * y


def latency_quantiles(tail, n=20000, seed=0):
    r = np.random.default_rng(seed)
    u = r.random((n, 3))
    t = np.array([compute_time(tail, a, b, c) for a, b, c in u])
    return {"p50": float(np.quantile(t, 0.5)), "p95": float(np.quantile(t, 0.95)),
            "p99": float(np.quantile(t, 0.99)), "p999": float(np.quantile(t, 0.999))}


# ---------------------------------------------------------------- stereo / lidar detection probabilities
def stereo_p_hit(diam_m, range_m, v_lat=0.0, yaw_rate=0.0, cfg=STEREO):
    """P(stereo returns valid depth on an object of diameter D at range r), with motion blur.

    Pixel width w = f D / r; blur b = f (v_lat / r + |yaw_rate|) t_exp; effective width w_eff = w^2 / (w + b)
    (blur spreads the object over more pixels and drops its contrast); p = logistic((w_eff - w50) / s).
    Monotone: decreasing in range, blur and speed, increasing in diameter.
    """
    f = 0.5 * cfg["width_px"] / math.tan(math.radians(0.5 * cfg["hfov_deg"]))
    r = np.maximum(np.asarray(range_m, float), 0.05)
    w = f * np.asarray(diam_m, float) / r
    b = f * (np.abs(v_lat) / r + abs(yaw_rate)) * cfg["exposure_s"]
    weff = w * w / (w + b + 1e-9)
    return 1.0 / (1.0 + np.exp(-(weff - cfg["w50_px"]) / cfg["w_scale_px"]))


def lidar_expected_hits(diam_m, len_m, range_m, cfg=LIDAR):
    """Expected L2 points per frame on an object: rho [pts/sr] x solid angle (D x L / r^2)."""
    sr = 2 * math.pi * 2 * math.sin(math.radians(0.5 * cfg["vfov_deg"]))
    rho = cfg["pts_per_s"] / cfg["rate_hz"] / sr
    r = np.maximum(np.asarray(range_m, float), 0.05)
    return rho * np.asarray(diam_m, float) * np.asarray(len_m, float) / (r * r)


def lidar_p_hit(diam_m, len_m, range_m, cfg=LIDAR):
    return 1.0 - np.exp(-lidar_expected_hits(diam_m, len_m, range_m, cfg))


# ---------------------------------------------------------------- z-buffer
def _zbuffer(d, th, R, th0, dth, nb, wrap):
    """Expand circles to bearing bins. Returns (bin, range, circle_local_idx) for every covered pair."""
    alpha = np.arcsin(np.clip(R / np.maximum(d, 1e-6), 0.0, 1.0))
    bc = np.floor((th - th0) / dth).astype(np.int64)
    lo = np.minimum(np.ceil((th - alpha - th0) / dth - 0.5).astype(np.int64), bc)
    hi = np.maximum(np.floor((th + alpha - th0) / dth - 0.5).astype(np.int64), bc)
    n = hi - lo + 1
    tot = int(n.sum())
    rep = np.repeat(np.arange(len(d)), n)
    b = np.repeat(lo, n) + (np.arange(tot) - np.repeat(np.cumsum(n) - n, n))
    phi = th0 + (b + 0.5) * dth
    delta = phi - th[rep]
    lat = d[rep] * np.sin(delta)
    disc = R[rep] ** 2 - lat * lat
    rng_ = np.where(disc > 0, d[rep] * np.cos(delta) - np.sqrt(np.maximum(disc, 0.0)), d[rep] - R[rep])
    if wrap:
        b = b % nb
    else:
        ok = (b >= 0) & (b < nb)
        b, rng_, rep = b[ok], rng_[ok], rep[ok]
    return b, np.maximum(rng_, 0.01), rep


def _nearest_per_bin(b, r, rep):
    if len(b) == 0:
        return b, r, rep
    o = np.lexsort((r, b))
    b, r, rep = b[o], r[o], rep[o]
    first = np.ones(len(b), bool)
    first[1:] = b[1:] != b[:-1]
    return b[first], r[first], rep[first]


class Frame:
    __slots__ = ("sensor", "t_acq", "ox", "oy", "angles", "ranges", "hit", "carve", "visible_obj", "detected_obj")

    def __init__(self, sensor, t_acq, ox, oy, angles, ranges, hit, carve, visible_obj, detected_obj):
        """carve[i]: free-space evidence length along ray i (m); <= 0 means no information (invalid pixel)."""
        self.sensor, self.t_acq, self.ox, self.oy = sensor, t_acq, ox, oy
        self.angles, self.ranges, self.hit, self.carve = angles, ranges, hit, carve
        self.visible_obj, self.detected_obj = visible_obj, detected_obj


def stereo_frame(world, t, x, y, yaw, vx, vy, yaw_rate, rng, cfg=STEREO):
    nb = cfg["n_bins"]
    half = math.radians(0.5 * cfg["hfov_deg"])
    dth = 2 * half / nb
    R = cfg["range_max"]
    idx = world.query(x, y, R)
    dx, dy = world.cx[idx] - x, world.cy[idx] - y
    d = np.hypot(dx, dy)
    th = (np.arctan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
    m = (d < R + 0.5) & (np.abs(th) < half + 0.2)
    idx, d, th, dx, dy = idx[m], d[m], th[m], dx[m], dy[m]
    angles = yaw - half + (np.arange(nb) + 0.5) * dth
    if len(idx) == 0:
        inv = rng.random(nb) < cfg["invalid_p"]
        return Frame("stereo", t, x, y, angles, np.full(nb, np.nan), np.zeros(nb, bool),
                     np.where(inv, -1.0, cfg["free_evidence_m"]), np.zeros(0, np.int64), np.zeros(0, np.int64))
    b, r, rep = _zbuffer(d, th, world.cr_sense[idx], -half, dth, nb, False)
    bv, rv, repv = _nearest_per_bin(b, r, rep)
    obj = world.cobj[idx]
    visible = np.unique(obj[repv])
    # per-object detection draw
    uo, inv = np.unique(obj, return_inverse=True)
    dmin = np.full(len(uo), np.inf)
    np.minimum.at(dmin, inv, d)
    vlat = np.abs(vx * dy - vy * dx) / np.maximum(d, 1e-6)
    vl = np.zeros(len(uo))
    np.maximum.at(vl, inv, vlat)
    p = stereo_p_hit(world.odiam[uo], dmin, vl, yaw_rate, cfg)
    det_o = rng.random(len(uo)) < p
    keep = det_o[inv[rep]]
    bh, rh, reph = _nearest_per_bin(b[keep], r[keep], rep[keep])
    ranges = np.full(nb, np.nan)
    hit = np.zeros(nb, bool)
    f = 0.5 * cfg["width_px"] / math.tan(half)
    sig = rh * rh * cfg["disp_sigma_px"] / (f * cfg["baseline_m"])
    rr = rh + sig * rng.standard_normal(len(rh))
    valid = (rr > 0.1) & (rr < R)
    ranges[bh[valid]] = rr[valid]
    hit[bh[valid]] = True
    # Rays with no obstacle return still see ground/background texture: free evidence to free_evidence_m.
    # Invalid pixels (texture-less, saturation) carry no information.
    inv = rng.random(nb) < cfg["invalid_p"]
    carve = np.where(hit, np.nan_to_num(ranges) - 0.15, cfg["free_evidence_m"])
    carve = np.where(inv, -1.0, carve)
    hit &= ~inv
    detected = np.unique(obj[reph[valid]]) if valid.any() else np.zeros(0, np.int64)
    return Frame("stereo", t, x, y, angles, ranges, hit, carve, visible, detected)


def lidar_frame(world, t, x, y, yaw, rng, cfg=LIDAR):
    nb = cfg["n_bins"]
    dth = 2 * math.pi / nb
    R = cfg["range_max"]
    idx = world.query(x, y, R)
    dx, dy = world.cx[idx] - x, world.cy[idx] - y
    d = np.hypot(dx, dy)
    m = d < R
    idx, d, dx, dy = idx[m], d[m], dx[m], dy[m]
    angles = -math.pi + (np.arange(nb) + 0.5) * dth
    ranges = np.full(nb, np.nan)
    hit = np.zeros(nb, bool)
    if len(idx) == 0:
        return Frame("lidar", t, x, y, angles, ranges, hit, np.full(nb, 12.0), np.zeros(0, np.int64), np.zeros(0, np.int64))
    th = np.arctan2(dy, dx)
    b, r, rep = _zbuffer(d, th, world.cr_sense[idx], -math.pi, dth, nb, True)
    bv, rv, repv = _nearest_per_bin(b, r, rep)
    obj = world.cobj[idx]
    visible = np.unique(obj[repv])
    # per-object expected hits, spread over the bins the object covers (non-repetitive scan ~ Poisson)
    uo, inv = np.unique(obj, return_inverse=True)
    dmin = np.full(len(uo), np.inf)
    np.minimum.at(dmin, inv, d)
    ln = np.where(world.okind[uo] == 0, cfg["stem_band_m"], world.olen[uo])
    lam = lidar_expected_hits(world.odiam[uo], ln, dmin, cfg)
    nbins_o = np.bincount(inv[rep], minlength=len(uo)).astype(float)
    p_bin = 1.0 - np.exp(-lam / np.maximum(nbins_o, 1.0))
    keep = rng.random(len(b)) < p_bin[inv[rep]]
    bh, rh, reph = _nearest_per_bin(b[keep], r[keep], rep[keep])
    rr = rh + cfg["range_sigma"] * rng.standard_normal(len(rh))
    ranges[bh] = rr
    hit[bh] = True
    detected = np.unique(obj[reph])
    carve = np.where(hit, np.nan_to_num(ranges) - 0.15, 12.0)   # no return within range = free
    return Frame("lidar", t, x, y, angles, ranges, hit, carve, visible, detected)


# ---------------------------------------------------------------- person detection
def detection_p(range_m, size_m, vis_frac, v_lat, yaw_rate, cfg=RGB):
    """Per-frame P(detect person): pixel height logistic x visibility x blur attenuation."""
    f = 0.5 * cfg["width_px"] / math.tan(math.radians(0.5 * cfg["hfov_deg"]))
    r = max(range_m, 0.3)
    h = f * size_m / r
    blur = f * (abs(v_lat) / r + abs(yaw_rate)) * cfg["exposure_s"]
    ph = 1.0 / (1.0 + math.exp(-(h - cfg["h50_px"]) / cfg["h_scale_px"]))
    pv = min(1.0, max(0.0, (vis_frac - 0.2) / 0.5))
    return cfg["p_max"] * ph * pv * math.exp(-blur / cfg["blur_scale_px"])


def target_visibility(world, x, y, tx, ty, size_m, n_lines=5):
    """Fraction of n_lines sight lines (spread across the target width) not blocked by any circle."""
    dx, dy = tx - x, ty - y
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return 1.0
    ux, uy = dx / L, dy / L
    mx, my = 0.5 * (x + tx), 0.5 * (y + ty)
    idx = world.query(mx, my, 0.5 * L + 0.5)
    if len(idx) == 0:
        return 1.0
    offs = np.linspace(-0.5 * size_m, 0.5 * size_m, n_lines)
    cx, cy, cr = world.cx[idx], world.cy[idx], world.cr[idx]
    free = 0
    for o in offs:
        ex, ey = tx - uy * o, ty + ux * o
        sx, sy = ex - x, ey - y
        ll = math.hypot(sx, sy)
        sx, sy = sx / ll, sy / ll
        along = (cx - x) * sx + (cy - y) * sy
        lat = np.abs((cx - x) * sy - (cy - y) * sx)
        blocked = np.any((along > 0) & (along < ll - 0.3) & (lat < cr))
        free += 0 if blocked else 1
    return free / n_lines
