# SPDX-License-Identifier: GPL-3.0-or-later
"""Calibrated person-observation model for the CRANE mission simulator (Tools/MissionSim).

Evidence label: HSUR-calibrated (real aerial imagery, NOMAD CC BY 4.0, location-held-out validation; synthetic blur;
COCO-pretrained RT-DETR-R18, Apache-2.0, tiled 6x640 on the 1920x1080 OAK-D RGB frame). Parameters live in
obs_params_tau05.json (alert threshold 0.5, default) or obs_params_tau03.json next to this file (written by fit_obs.py); nothing here is hand-tuned except the items marked PRIOR.

Two APIs:
1. Physical, per frame:
     p_detect(h_px, visible=1, view_deg=45, blur_px=0, lighting='sunny', low_sun=False) -> P(detect) in one frame
     sample_score / conf_pdf      detector score of a true detection or of a false alert
     false_alarms_per_frame(blur_px), false_alarm_rate_m2(gsd_m)
     p_any(p, n, rho)             >= 1 detection in n correlated looks (beta-binomial, ICC rho = REPEAT_RHO)
     loc_error(alt_m, ground_range_m, h_px)  -> (sigma_along_m, sigma_cross_m) pixel-to-ground projection error
     apparent_height_px(size_m, slant_m, view_deg, pose), blur_px(speed, slant_m, exposure_s, yaw_rate)
2. MissionSim drop-in: class ObsModel has every method of missionsim.obs_model.PlaceholderObsModel with the same
   signatures (p_view_good, stand_visibility, p_detect_pass, p_glimpse_bad, p_false_alert, p_decoy_detect,
   sample_conf, loc_sigma, log_lr, expected_g, expected_pd), so `missionsim.obs_model.load(prefer_vision=True)`
   returns it.

Known limits (state them when using the model):
* View angle: NOMAD is oblique (~45 deg look-down; distances set "horizontally and vertically"). No nadir or
  near-horizontal frames were benchmarked, so the view angle enters ONLY through apparent size
  (apparent_height_px) plus VIEW_PRIOR (PRIOR, multiplicative factor 1.0 at 30-60 deg, not calibrated).
* Forest stand visibility (p_view_good, stand_visibility) is scene geometry, not vision: kept from the placeholder.
* False-alert rate is an upper bound (NOMAD labels only the actor). Clutter dependence on stand density is PRIOR.
* The detector is not fine-tuned on aerial data; a fine-tuned detector would move the size curve left.
"""
import json
import math
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.join(_HERE, 'obs_params_tau05.json')          # default alert threshold: score >= 0.5
PARAMS_PATH_TAU03 = os.path.join(_HERE, 'obs_params_tau03.json')    # more sensitive, ~5x more false alerts

# OAK-D (IMX378) RGB stream as rendered by CRANE (Assets/Scripts/Theater/Aerial/RenderProfiles.cs): 1920x1080, HFOV 69 deg
FRAME_W, FRAME_H, HFOV_DEG = 1920, 1080, 69.0
F_PX = 0.5 * FRAME_W / math.tan(math.radians(0.5 * HFOV_DEG))   # ~1398 px
EXPOSURE_S = 0.004        # PRIOR: daylight under canopy, auto-exposure ~ 1/250 s
SYNC_S = 0.02             # PRIOR: image-pose timestamp error (s), enters loc_sigma via speed
ATT_SIGMA_DEG = 0.5       # PRIOR: roll/pitch error of the camera ray (deg)
YAW_SIGMA_DEG = 1.0       # PRIOR: heading error, GPS-denied VIO (deg)
ALT_SIGMA_M = 0.3         # PRIOR: height-above-ground error (m)
MISSION_ALT_M = 3.0       # PRIOR: under-canopy flight height for MissionSim calls (m)
PERSON_SIZE_M = 0.8       # MissionSim convention: lying / crouching person extent
V = 4


def _sig(x):
    return 1.0 / (1.0 + math.exp(-x))


def _beta_pdf(c, a, b):
    c = min(max(c, 1e-4), 1 - 1e-4)
    return math.exp((a - 1) * math.log(c) + (b - 1) * math.log(1 - c) + math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b))


def view_prior(view_deg):
    """PRIOR (uncalibrated) multiplicative detectability factor for the camera depression angle to the person:
    1.0 inside the benchmarked 30-60 deg band, tapering to 0.85 at nadir (top-down silhouettes are less person-like)
    and to 0.9 at grazing angles (more canopy and ground clutter in the line of sight)."""
    v = float(view_deg)
    if v > 60:
        return 1.0 - 0.15 * min(1.0, (v - 60) / 30.0)
    if v < 30:
        return 1.0 - 0.10 * min(1.0, (30 - v) / 30.0)
    return 1.0


def apparent_height_px(size_m, slant_m, view_deg=45.0, pose='standing'):
    """Box height (px, 1920 frame) of a person at slant range, seen at depression angle view_deg.
    standing: height 1.7 m foreshortened by cos(view) plus body depth 0.3 m by sin(view);
    lying: length size_m along a random ground direction: mean along-range extent (2/pi)*L foreshortened by sin(view),
    plus body thickness 0.3 m by cos(view)."""
    v = math.radians(view_deg); r = max(slant_m, 0.3)
    if pose == 'standing':
        ext = (size_m or 1.7) * math.cos(v) + 0.3 * math.sin(v)
    else:
        ext = (2.0 / math.pi) * (size_m or 1.7) * math.sin(v) + 0.3 * math.cos(v)
    return F_PX * ext / r


def blur_px(speed_mps, slant_m, exposure_s=EXPOSURE_S, yaw_rate=0.0):
    """Motion-blur length (px): translation parallax f*v*t/R plus rotation f*omega*t."""
    return F_PX * (abs(speed_mps) / max(slant_m, 0.3) + abs(yaw_rate)) * exposure_s


class ObsModel:
    name = 'vision-rtdetr-r18-nomad-v1'
    frame_model = 'RT-DETR-R18 tiled 6x640 on 1920x1080, NOMAD-calibrated (Tools/Vision/obs_params_tau05.json)'
    V = V
    # MissionSim compatibility constants (planner families); overwritten from params in __init__
    CONF_TRUE = (6.0, 2.0)
    CONF_GLIMPSE = (2.5, 2.5)
    CONF_DECOY = (3.0, 3.0)
    CONF_FA = (2.0, 4.0)

    def __init__(self, vis_table=None, params_path=PARAMS_PATH, exposure_s=EXPOSURE_S, altitude_m=MISSION_ALT_M, scene_offset=None):
        """scene_offset None: population-average model over scenes (pooled fit, leave-one-location-out calibrated).
        A float: a specific scene, logit offset u on top of the median-scene mixed model (u ~ N(0, SIGMA_SCENE^2));
        use draw_scene(rng) to sample one per mission (truth side) while planners keep the population average."""
        with open(params_path) as f:
            P = json.load(f)
        self.P = P
        self.vis_table = vis_table
        self.exposure_s = exposure_s
        self.altitude_m = altitude_m
        self.SIGMA_SCENE = P['detect_scene']['sigma_scene']
        self.scene_offset = scene_offset
        t = P['detect_pooled']['theta'] if scene_offset is None else P['detect_scene']['theta']
        self._th = [t['logit_pmax'], t['b0'], t['b_log2h'], t['b_vis'], t['b_blur'], t['b_overcast'], t['b_lowsun']]
        if scene_offset is not None:
            self._th[1] += float(scene_offset)
        self.tau = P['detect']['tau']
        ct, cf = P['conf_true'], P['conf_false']
        self._ct = (ct['a0'], ct['a1'], ct['b']); self._cf = (cf['a'], cf['b'])
        self.CONF_FA = (cf['a'], cf['b']); self.CONF_DECOY = (cf['a'], cf['b'])
        self.CONF_TRUE = (ct['a0'] + ct['a1'] * 0.8, ct['b'])
        self.REPEAT_RHO = P['repeat_view']['rho']
        fa = P['false_alerts']
        self._fa_blur = sorted((int(k), v) for k, v in fa['per_frame_by_blur'].items())
        self.persistent_fraction = fa['persistent_fraction']
        ce = P['centre_error']; self._ce = (ce['c0'], ce['c1'])

    @classmethod
    def draw_scene(cls, rng, **kw):
        """A model for one random scene (truth side): logit offset ~ N(0, sigma_scene^2) from the 9 NOMAD sites."""
        with open(kw.get('params_path', PARAMS_PATH)) as f:
            sig = json.load(f)['detect_scene']['sigma_scene']
        return cls(scene_offset=float(rng.normal(0.0, sig)), **kw)

    # ------------------------------------------------------------------ physical per-frame API
    def p_detect(self, h_px, visible=1.0, view_deg=45.0, blur_px=0.0, lighting='sunny', low_sun=False):
        """P(the detector fires on the person in one frame) at score >= tau, 'loc' criterion."""
        th = self._th; he = max(float(h_px), 4.0)
        z = (th[1] + th[2] * (math.log2(he) - 5.0) + th[3] * (float(visible) - 1.0) + th[4] * math.log1p(max(blur_px, 0.0) / he)
             + th[5] * (1.0 if lighting == 'overcast' else 0.0) + th[6] * (1.0 if low_sun else 0.0))
        p = _sig(th[0]) * _sig(z) * view_prior(view_deg)
        if float(h_px) < 4.0:  # below 4 px nothing is resolvable; fade linearly to 0 at 1 px
            p *= max(0.0, (float(h_px) - 1.0) / 3.0)
        return float(min(max(p, 0.0), 1.0))

    def score_params(self, kind='true', q=0.8):
        """Beta parameters of u = (score - tau) / (1 - tau)."""
        if kind == 'true':
            return (self._ct[0] + self._ct[1] * q, self._ct[2])
        return self._cf

    def sample_score(self, rng, kind='true', q=0.8):
        a, b = self.score_params(kind, q)
        return float(self.tau + (1 - self.tau) * rng.beta(a, b))

    def conf_pdf(self, s, kind='true', q=0.8):
        if s < self.tau:
            return 0.0
        a, b = self.score_params(kind, q)
        return _beta_pdf((s - self.tau) / (1 - self.tau), a, b) / (1 - self.tau)

    def false_alarms_per_frame(self, blur_px=0.0):
        xs, ys = zip(*self._fa_blur)
        r = np.interp(blur_px, xs, ys)
        return float(r) if np.ndim(r) == 0 else r

    def false_alarm_rate_m2(self, gsd_m, blur_px=0.0):
        """False alerts per m^2 of imaged ground at ground sample distance gsd_m (m/px)."""
        return self.false_alarms_per_frame(blur_px) / (FRAME_W * FRAME_H * np.square(gsd_m))

    def p_any(self, p, n, rho=None):
        """P(>= 1 detection in n looks with mean per-look p and intra-pass correlation rho (beta-binomial)."""
        rho = self.REPEAT_RHO if rho is None else rho
        p = min(max(p, 1e-9), 1 - 1e-9); n = max(1, int(round(n)))
        if rho <= 1e-6:
            return 1.0 - (1.0 - p) ** n
        s = (1.0 - rho) / rho  # a + b
        a, b = p * s, (1.0 - p) * s
        # E[(1-P)^n] = B(a, b+n) / B(a, b)
        lg = math.lgamma
        return float(1.0 - math.exp(lg(b + n) + lg(a + b) - lg(b) - lg(a + b + n)))

    def n_eff(self, n, rho=None):
        rho = self.REPEAT_RHO if rho is None else rho
        return n / (1.0 + (n - 1) * rho)

    def centre_sigma_px(self, h_px):
        return self._ce[0] + self._ce[1] * float(h_px)

    def loc_error(self, alt_m, ground_range_m, h_px=30.0, att_deg=ATT_SIGMA_DEG, yaw_deg=YAW_SIGMA_DEG, alt_sigma_m=ALT_SIGMA_M):
        """1-sigma ground position error (along-range, cross-range) in m of a flat-ground ray projection from a camera
        at height alt_m to a person at ground range: pixel centre error (from the benchmark) + attitude + height."""
        H = max(alt_m, 0.1); D = max(ground_range_m, 0.0); R = math.hypot(H, D)
        s_pix = self.centre_sigma_px(h_px) / F_PX
        s_el = math.hypot(s_pix, math.radians(att_deg))
        along = math.hypot(R * R / H * s_el, alt_sigma_m * D / H)
        # cross-range: an image-plane angle error moves the ray by R*angle; a heading error rotates it about the
        # vertical through the camera, moving the ground point by D*angle
        cross = math.hypot(R * s_pix, D * math.radians(yaw_deg))
        return along, cross

    # ------------------------------------------------------------------ MissionSim drop-in API
    def _frame_p(self, range_m, speed, vis=1.0):
        H = self.altitude_m
        slant = math.hypot(range_m, H)
        view = math.degrees(math.atan2(H, max(range_m, 0.1)))
        h = apparent_height_px(PERSON_SIZE_M, slant, view, pose='lying')
        return self.p_detect(h, vis, view, blur_px(speed, slant, self.exposure_s))

    def p_view_good(self, rho, person_occl):
        """Scene geometry (not vision): kept from the placeholder."""
        return float(np.clip(self.stand_visibility(rho) * (1.0 - 0.7 * person_occl), 0.02, 0.98))

    def stand_visibility(self, rho):
        sph = 400.0 + 1000.0 * rho
        if self.vis_table is not None:
            xs, ys = self.vis_table
            return float(np.interp(sph, xs, ys))
        return float(math.exp(-sph / 1e4 * 0.25 * 7.0))

    def p_detect_pass(self, range_m, speed, rho, r_eff, frame_hz=10.0):
        """P(detect | good view) in one pass: per-frame p over the frames inside r_eff, correlated by REPEAT_RHO."""
        pf = self._frame_p(range_m, speed)
        chord = 2.0 * math.sqrt(max(r_eff * r_eff - min(range_m, r_eff) ** 2, 1.0))
        n_frames = max(1.0, chord / max(speed, 0.3) * frame_hz)
        return self.p_any(pf, n_frames)

    def p_glimpse_bad(self, p_good_pass):
        """Bad (mostly occluded) view: per-frame model at 20% visibility relative to a full view, as a ratio."""
        r = self.p_detect(60.0, 0.2) / max(self.p_detect(60.0, 1.0), 1e-9)
        return float(r * p_good_pass)

    def p_false_alert(self, rho, speed, area_m2):
        """P(>= 1 background false alert while scanning area_m2): measured per-frame rate converted to ground area at
        the mission GSD (slant ~ r_eff/2 at altitude_m), times a PRIOR clutter factor (1 + rho)."""
        slant = math.hypot(6.0, self.altitude_m)
        gsd = slant / F_PX
        lam = self.false_alarm_rate_m2(gsd, blur_px(speed, slant, self.exposure_s)) * area_m2 * (1.0 + rho)
        return float(min(0.5, 1.0 - math.exp(-lam)))

    def p_decoy_detect(self, speed):
        """Persistent decoy fires on a pass: measured FP persistence across consecutive frames, reduced by blur."""
        b = blur_px(speed, math.hypot(6.0, self.altitude_m), self.exposure_s)
        f0 = self.false_alarms_per_frame(0.0)
        return float(self.persistent_fraction * min(1.0, self.false_alarms_per_frame(b) / max(f0, 1e-9)))

    def sample_conf(self, kind, quality, u):
        """kind: true / glimpse / decoy / fa; u: numpy Generator. Returns a detector score in [tau, 1]."""
        if kind == 'true':
            return self.sample_score(u, 'true', quality)
        if kind == 'glimpse':
            return self.sample_score(u, 'true', 0.2)
        return self.sample_score(u, 'false')

    def loc_sigma(self, range_m, speed):
        """Isotropic 1-sigma relative-position error (m): RMS of loc_error at altitude_m plus speed x sync error."""
        h = apparent_height_px(PERSON_SIZE_M, math.hypot(range_m, self.altitude_m),
                               math.degrees(math.atan2(self.altitude_m, max(range_m, 0.1))), 'lying')
        a, c = self.loc_error(self.altitude_m, range_m, h)
        return float(math.sqrt(0.5 * (a * a + c * c) + (speed * SYNC_S) ** 2))

    def log_lr(self, c):
        c = max(c, self.tau + 1e-4)
        f1 = 0.85 * self.conf_pdf(c, 'true', 0.8) + 0.15 * self.conf_pdf(c, 'true', 0.2)
        f0 = self.conf_pdf(c, 'false')
        return math.log(max(f1, 1e-9) / max(f0, 1e-9))

    def expected_g(self, rho):
        return self.p_view_good(rho, 1.0 / 3.0)

    def expected_pd(self, rho, speed, r_eff, mode='scan'):
        if mode == 'review':
            return self.p_detect_pass(4.0, speed, rho, r_eff)
        d = 0.25 * 2.0 * r_eff
        return self.p_detect_pass(math.hypot(d, 2.0), speed, rho, r_eff)


# ====================================================================== spatial map-update API (RoleSpec / MissionSim)
class SearchMaps:
    """Ground-grid evidence for the recon / verify roles, driven by ObsModel.

    Two maps, kept separate on purpose:
      * DETECTION map: per-cell log-odds that a person is in the cell (posterior()), and candidate survivor locations
        (candidates()) whose CONFIDENCE (posterior probability) is reported separately from their LOCATION covariance
        (projection error from loc_error + frame-alignment error sigma_align_m + spread of the supporting evidence).
      * COVERAGE-CONFIDENCE map: per cell, P(we would have detected a person there by now) given every past view's
        apparent size, blur, occlusion and view angle (coverage()), plus the best pixel size and number of distinct
        view classes; verify_needed() marks cells whose posterior is ambiguous or whose coverage is low.

    Per look (one frame or one pass treated as a look), for each cell c in the camera footprint (pinhole, flat ground):
      p_c   = per-look detection probability from ObsModel.p_detect at the cell's apparent size / blur / view / visibility,
              made INCREMENTAL for correlated repeat looks from the same view class (beta-binomial with REPEAT_RHO):
              p_inc = 1 - P_miss(n+1)/P_miss(n) within that (cell, view class); different view classes are independent.
      LR_c  = (1 - p_inc) + p_inc * sum_j f1(s_j) N(z_j; c, S_j) / (lambda_fa * f0(s_j))     (Bernoulli-target LR)
              with f1/f0 the score densities of true / false detections and lambda_fa the false-alert density per m^2
              per look. Positive evidence from repeat looks in the same view class is tempered by the n_eff increment,
              because false alerts are 80 % persistent decoys (measured).
      L_c  += log LR_c.
    View classes: n_view_bins bearings from the person to the camera (default 4, as MissionSim's V).
    """

    def __init__(self, model, bounds, cell_m=2.0, prior=1e-3, n_view_bins=4, sigma_align_m=0.5, pose='lying'):
        self.m = model
        x0, y0, x1, y1 = bounds
        self.cell = float(cell_m)
        self.xs = np.arange(x0 + cell_m / 2, x1, cell_m)
        self.ys = np.arange(y0 + cell_m / 2, y1, cell_m)
        self.X, self.Y = np.meshgrid(self.xs, self.ys, indexing='xy')
        shp = self.X.shape
        self.L = np.zeros(shp)                          # log-likelihood-ratio evidence (prior added in posterior())
        self.logit_prior = math.log(prior / (1 - prior))
        self.V = int(n_view_bins)
        self.n = np.zeros(shp + (self.V,))              # looks per (cell, view class)
        self.psum = np.zeros(shp + (self.V,))           # sum of per-look p per (cell, view class)
        self.best_px = np.zeros(shp)
        self.sigma_align = float(sigma_align_m)
        self.pose = pose

    # ---------------------------------------------------------------- geometry
    def _view(self, cam_xy, alt_m, yaw, pitch_deg, hfov_deg=HFOV_DEG, vfov_deg=None, r_max=None):
        """Per-cell (in_view mask, slant, ground range, depression deg, view-class index) for a pinhole camera at
        cam_xy, height alt_m, heading yaw (rad, ENU, 0 = +x), pitch_deg below the horizon."""
        vfov = vfov_deg if vfov_deg is not None else math.degrees(2 * math.atan(math.tan(math.radians(HFOV_DEG / 2)) * FRAME_H / FRAME_W))
        dx, dy = self.X - cam_xy[0], self.Y - cam_xy[1]
        D = np.hypot(dx, dy); H = max(alt_m, 0.1)
        # camera frame: forward f, left l, up u
        cy, sy = math.cos(yaw), math.sin(yaw); cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
        fwd_h = dx * cy + dy * sy; left = -dx * sy + dy * cy; up = -H * np.ones_like(D)
        f = fwd_h * cp - up * sp; u = fwd_h * sp + up * cp
        with np.errstate(divide='ignore', invalid='ignore'):
            ok = (f > 0.3) & (np.abs(left / f) <= math.tan(math.radians(hfov_deg / 2))) & (np.abs(u / f) <= math.tan(math.radians(vfov / 2)))
        if r_max is not None:
            ok &= D <= r_max
        slant = np.hypot(D, H); dep = np.degrees(np.arctan2(H, np.maximum(D, 1e-3)))
        bearing = np.arctan2(-dy, -dx)                  # from person to camera
        vbin = ((bearing + math.pi) / (2 * math.pi) * self.V).astype(int) % self.V
        return ok, slant, D, dep, vbin

    def _p_cells(self, slant, dep, speed, yaw_rate, visible, lighting, low_sun):
        h = np.vectorize(lambda r, v: apparent_height_px(None, r, v, self.pose))(slant, dep)
        b = F_PX * (abs(speed) / np.maximum(slant, 0.3) + abs(yaw_rate)) * self.m.exposure_s
        vis = np.broadcast_to(visible, slant.shape)
        p = np.vectorize(lambda hh, vv, dd, bb: self.m.p_detect(hh, vv, dd, bb, lighting, low_sun))(h, vis, dep, b)
        return p, h

    def _p_miss(self, psum, n):
        out = np.ones_like(psum)
        k = n > 0
        if k.any():
            pbar = np.clip(psum[k] / n[k], 1e-6, 1 - 1e-6)
            out[k] = 1.0 - np.vectorize(self.m.p_any)(pbar, n[k])
        return out

    # ---------------------------------------------------------------- update
    def update(self, cam_xy, alt_m, yaw, pitch_deg, detections=(), speed=0.0, yaw_rate=0.0, visible=1.0,
               lighting='sunny', low_sun=False, r_max=None):
        """Fold one look into both maps. detections: iterable of dicts {x, y, score, cov (optional 2x2, m^2)} in the
        ground frame (already projected; use project_detection to make them). visible: scalar or per-cell array of
        the person's visible fraction (canopy occlusion along the sight line). Returns the footprint mask."""
        ok, slant, D, dep, vbin = self._view(cam_xy, alt_m, yaw, pitch_deg, r_max=r_max)
        if not ok.any():
            return ok
        p, h = self._p_cells(slant[ok], dep[ok], speed, yaw_rate,
                             visible[ok] if np.ndim(visible) else visible, lighting, low_sun)
        iy, ix = np.nonzero(ok); vb = vbin[ok]
        n0 = self.n[iy, ix, vb]; s0 = self.psum[iy, ix, vb]
        miss0 = self._p_miss(s0, n0); miss1 = self._p_miss(s0 + p, n0 + 1)
        p_inc = np.clip(1.0 - miss1 / np.maximum(miss0, 1e-12), 0.0, 1.0)
        temper = (n0 + 1) / (1.0 + n0 * self.m.REPEAT_RHO) - n0 / (1.0 + np.maximum(n0 - 1, 0) * self.m.REPEAT_RHO)
        temper = np.where(n0 > 0, temper, 1.0)
        lam = self.m.false_alarm_rate_m2(np.maximum(slant[ok], 0.3) / F_PX,
                                         F_PX * (abs(speed) / np.maximum(slant[ok], 0.3) + abs(yaw_rate)) * self.m.exposure_s)
        lam = np.maximum(lam, 1e-6)
        ratio = np.zeros_like(p)
        cx, cy_ = self.X[ok], self.Y[ok]
        for d in detections:
            S = np.asarray(d.get('cov', np.eye(2) * 1.0), float) + np.eye(2) * self.sigma_align ** 2
            Si = np.linalg.inv(S); det = np.linalg.det(S)
            ex, ey = d['x'] - cx, d['y'] - cy_
            g = np.exp(-0.5 * (Si[0, 0] * ex * ex + 2 * Si[0, 1] * ex * ey + Si[1, 1] * ey * ey)) / (2 * math.pi * math.sqrt(det))
            f1 = self.m.conf_pdf(d['score'], 'true', 0.5); f0 = max(self.m.conf_pdf(d['score'], 'false'), 1e-9)
            ratio += f1 * g / (lam * f0)
        LR = (1.0 - p_inc) + p_inc * ratio
        self.L[iy, ix] += np.where(ratio > 0, temper, 1.0) * np.log(np.maximum(LR, 1e-12))
        self.n[iy, ix, vb] = n0 + 1; self.psum[iy, ix, vb] = s0 + p
        self.best_px[iy, ix] = np.maximum(self.best_px[iy, ix], h)
        return ok

    def project_detection(self, cam_xy, alt_m, yaw, ground_range_m, bearing_off_rad, score, h_px=30.0):
        """Ground position and covariance (along/cross-range from ObsModel.loc_error, rotated to ENU) for a detection
        at ground range and bearing offset from the camera heading."""
        th = yaw + bearing_off_rad
        x = cam_xy[0] + ground_range_m * math.cos(th); y = cam_xy[1] + ground_range_m * math.sin(th)
        a, c = self.m.loc_error(alt_m, ground_range_m, h_px)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        return dict(x=x, y=y, score=score, cov=R @ np.diag([a * a, c * c]) @ R.T)

    # ---------------------------------------------------------------- outputs
    def posterior(self):
        """DETECTION map: P(person in cell | all looks)."""
        return 1.0 / (1.0 + np.exp(-(self.L + self.logit_prior)))

    def coverage(self):
        """COVERAGE-CONFIDENCE map: P(a person in the cell would have been detected at least once so far)."""
        miss = np.ones(self.L.shape)
        for v in range(self.V):
            miss *= self._p_miss(self.psum[..., v], self.n[..., v])
        return 1.0 - miss

    def view_classes(self):
        return (self.n > 0).sum(-1)

    def verify_needed(self, p_lo=0.05, p_hi=0.9, cov_min=0.6):
        """Cells a verifier should visit: ambiguous posterior, or a person could still be hiding (low coverage)."""
        post, cov = self.posterior(), self.coverage()
        return ((post > p_lo) & (post < p_hi)) | (cov < cov_min)

    def candidates(self, p_min=0.2, radius_m=4.0):
        """Survivor candidates: local maxima of the posterior above p_min. Each has a CONFIDENCE (posterior mass in the
        neighbourhood, capped at 1) and a separate LOCATION mean and 2x2 covariance (posterior-weighted spread, which
        already includes projection and alignment error through the evidence kernels)."""
        post = self.posterior(); out = []
        used = np.zeros(post.shape, bool)
        r = max(1, int(round(radius_m / self.cell)))
        for flat in np.argsort(-post, axis=None):
            iy, ix = np.unravel_index(flat, post.shape)
            if post[iy, ix] < p_min:
                break
            if used[iy, ix]:
                continue
            sl = (slice(max(iy - r, 0), iy + r + 1), slice(max(ix - r, 0), ix + r + 1))
            w = post[sl] * ~used[sl]
            if w.sum() <= 0:
                continue
            xs, ys = self.X[sl], self.Y[sl]
            mx, my = (w * xs).sum() / w.sum(), (w * ys).sum() / w.sum()
            dx, dy = xs - mx, ys - my
            C = np.array([[(w * dx * dx).sum(), (w * dx * dy).sum()], [(w * dx * dy).sum(), (w * dy * dy).sum()]]) / w.sum()
            C += np.eye(2) * self.cell ** 2 / 12.0
            out.append(dict(x=float(mx), y=float(my), confidence=float(min(1.0, w.sum())), peak=float(post[iy, ix]),
                            cov=C.tolist(), coverage=float(self.coverage()[iy, ix]), views=int(self.view_classes()[iy, ix])))
            used[sl] = True
        return out


def new_maps(bounds, cell_m=2.0, prior=1e-3, model=None, **kw):
    """Convenience: SearchMaps with the default (population-average, tau 0.5) ObsModel."""
    return SearchMaps(model or ObsModel(), bounds, cell_m, prior, **kw)
