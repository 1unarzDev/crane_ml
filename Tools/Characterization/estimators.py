"""Gate and identification estimators. Each returns a dict with ``status`` in {"OK", "ABSTAIN"}
and, on ABSTAIN, a ``reason`` and NO point estimate. A confident wrong estimate is a test failure.

Everything here recovers parameters of the SIMULATOR fixture (sim-truth recovery) or checks data
quality for the gate (design section 4). None of it is a physical calibration.

Degenerate cases never pass (design section 4):
  * zero residual variance -> abstain DEGENERATE
  * Ljung-Box with n <= 2h -> abstain DEGENERATE (``ljung_box``); ``ljung_box_q`` refuses that case
  * chi2_crit is never called with dof <= 0; callers abstain UNDERDETERMINED instead

The current-vs-body-bias estimator is a crane reimplementation of the idea behind the
characterize_samples drift split (world current rotates with heading, body bias does not). No
MHSeals code is imported or copied; it is a numpy least-squares written from the stated scenario.
"""
from __future__ import annotations

import math

import numpy as np

import plant as P

MOUNT_LIMIT = math.radians(45.0)  # validated mount-yaw range; excludes the sign-vs-180deg alias
GOF_Z = 3.09                      # p < 0.001 (Wilson-Hilferty approximation)
SIGN_GAP_CHI2 = 25.0              # second-best sign pattern must be worse by > 25 in calibrated chi2
ABSTAIN = "ABSTAIN"
OK = "OK"
DEGENERATE = "DEGENERATE"
UNDERDETERMINED = "UNDERDETERMINED"


def abstain(reason, **extra):
    return {"status": ABSTAIN, "reason": reason, **extra}


def ok(**extra):
    return {"status": OK, **extra}


def min_sv_vs_noise(X, E, k=3.0, hard_cond=1e6):
    """Identifiability: columns of X are scaled to unit norm; X is identifiable if its smallest
    singular value exceeds k * ||E||_F (the noise-induced perturbation of the scaled matrix) and
    cond < hard_cond. k=3 is a conventional 3-sigma margin. Returns (ok, smin, noise_norm, cond)."""
    X = np.asarray(X, float)
    E = np.zeros_like(X) if E is None else np.asarray(E, float)
    norms = np.linalg.norm(X, axis=0)
    norms[norms == 0] = 1.0
    sv = np.linalg.svd(X / norms, compute_uv=False)
    noise = float(np.linalg.norm(E / norms))
    cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else math.inf
    return bool(sv[-1] > k * noise and cond < hard_cond), float(sv[-1]), noise, cond


def chi2_crit(dof, z):
    """Wilson-Hilferty chi-square quantile (z = standard-normal quantile). Approximation.
    dof <= 0 is a caller bug (it divides by zero): callers abstain UNDERDETERMINED first."""
    if dof <= 0:
        raise ValueError(f"chi2_crit called with dof={dof}; abstain UNDERDETERMINED instead")
    a = 2.0 / (9.0 * dof)
    return dof * (1 - a + z * math.sqrt(a)) ** 3


def ljung_box_q(res, h):
    """Ljung-Box Q. Refuses (ValueError) when n <= 2h or the residual variance is zero; use
    ``ljung_box`` to get the DEGENERATE abstain instead of a silent 0."""
    res = np.asarray(res, float) - np.mean(res)
    n = len(res)
    if h < 1 or n <= 2 * h:
        raise ValueError(f"Ljung-Box needs n > 2h (n={n}, h={h})")
    d = float(res @ res)
    if d == 0:
        raise ValueError("Ljung-Box on zero-variance residuals")
    q = 0.0
    for k in range(1, h + 1):
        rk = float(res[:-k] @ res[k:]) / d
        q += rk * rk / (n - k)
    return n * (n + 2) * q


def zero_variance(x, scale=None):
    """True when the scatter of x is zero to rounding: RMS deviation <= 1e-12 of the data scale."""
    x = np.asarray(x, float)
    if len(x) == 0:
        return True
    ref = float(np.max(np.abs(x))) if scale is None else float(scale)
    rms = float(np.sqrt(np.mean((x - x.mean()) ** 2)))
    return rms <= 1e-12 * ref or rms == 0.0


def ljung_box(res, h):
    """``ok(q=...)`` or ``abstain(DEGENERATE)`` for n <= 2h or zero residual variance."""
    res = np.asarray(res, float)
    n = len(res)
    if h < 1 or n <= 2 * h:
        return abstain(DEGENERATE, why="ljung_box_n_le_2h", n=int(n), h=int(h))
    if zero_variance(res):
        return abstain(DEGENERATE, why="zero_residual_variance", n=int(n))
    return ok(q=float(ljung_box_q(res, h)), n=int(n), h=int(h))


def lag1(x):
    """Lag-1 autocorrelation (0 for fewer than 3 samples or zero variance)."""
    x = np.asarray(x, float)
    if len(x) < 3:
        return 0.0
    x = x - x.mean()
    d = float(x @ x)
    return float(x[:-1] @ x[1:]) / d if d > 0 else 0.0


def decimate_white(x, r_max=0.2):
    """Decimate until the lag-1 autocorrelation is below r_max (the wave correlation time)."""
    x = np.asarray(x, float)
    stride = 1
    y = x
    while len(y) >= 3 and lag1(y) >= r_max:
        stride += 1
        y = x[::stride]
    return y, stride


class Estimators:
    """Namespace so a test can subclass a single method."""

    # ---------------------------------------------------------------- pulse wrench measurement
    def measure_pulse_wrench(self, obs, t_on, t_hold_end, tau=P.T200_SPINUP, dt=P.DT, win=0.5):
        """Wrench (Fx,Fy,Nz) at the commanded steady state from window-mean momentum balance.

        M (vB - vA) = integral(W + drag + Coriolis) between window centres, with the NOMINAL
        mass, inertia and hull drag known. The time profile of W is the discrete spool squared.
        Returns (W, sigma_W)."""
        t = obs["t"]
        mA = (t >= t_on - win) & (t < t_on)
        mB = (t > t_hold_end - win) & (t <= t_hold_end)
        nA, nB = int(mA.sum()), int(mB.sum())
        if nA < 10 or nB < 10:
            return None
        vA = np.array([obs[c][mA].mean() for c in "uvr"])
        vB = np.array([obs[c][mB].mean() for c in "uvr"])
        cA, cB = t[mA].mean(), t[mB].mean()
        sel = (t >= cA) & (t <= cB)
        ts, u, v, r = t[sel], obs["u"][sel], obs["v"][sel], obs["r"][sel]
        m = P.MASS
        resist = np.vstack([np.array([P.drag(x, P.DRAG_SURGE) for x in u]) + m * v * r,
                            np.array([P.drag(x, P.DRAG_SWAY) for x in v]) - m * u * r,
                            np.array([P.drag(x, P.DRAG_YAW) for x in r])])
        D = np.trapezoid(resist, ts, axis=1)
        M = np.array([m, m, P.IZZ])
        imp = M * (vB - vA) - D
        alpha = P.clamp01(dt / max(tau, dt))
        J = int(round((cB - t_on) / dt))
        s = 1 - (1 - alpha) ** np.arange(1, J + 1)
        Teff = float(np.sum(s * s) * dt)
        W = imp / Teff
        sv, sr = obs["sigma_v"], obs["sigma_r"]
        dts = np.gradient(ts) if len(ts) > 1 else np.array([dt])
        dsu = np.array([P.DRAG_SURGE[0] + 2 * P.DRAG_SURGE[1] * abs(x) for x in u])
        dsv = np.array([P.DRAG_SWAY[0] + 2 * P.DRAG_SWAY[1] * abs(x) for x in v])
        dsr = np.array([P.DRAG_YAW[0] + 2 * P.DRAG_YAW[1] * abs(x) for x in r])
        var_D = np.array([
            np.sum(((dsu * sv) ** 2 + (m * r * sv) ** 2 + (m * v * sr) ** 2) * dts ** 2),
            np.sum(((dsv * sv) ** 2 + (m * r * sv) ** 2 + (m * u * sr) ** 2) * dts ** 2),
            np.sum((dsr * sr) ** 2 * dts ** 2)])
        var_win = M ** 2 * (1 / nA + 1 / nB) * np.array([sv ** 2, sv ** 2, sr ** 2])
        sig = np.sqrt(var_win + var_D) / Teff
        return W, sig

    # ---------------------------------------------------------------- mixer sign / mount
    @staticmethod
    def _pred_wrench(C, strengths, angles_rad, signs):
        C = np.asarray(C, float)
        ceff = np.clip(C, -1, 1) * np.asarray(signs, float)
        ratio = P.T200_REV / P.T200_FWD
        f = P.T200_FWD * np.asarray(strengths) * ceff * np.abs(ceff) * np.where(ceff < 0, ratio, 1.0)
        W = np.zeros((C.shape[0], 3))
        for i, spec in enumerate(P.NOMINAL_THRUSTERS):
            a = math.radians(spec.axis_deg) + angles_rad[i]
            col = np.array([math.cos(a), math.sin(a), spec.x * math.sin(a) - spec.y * math.cos(a)])
            W += f[:, i:i + 1] * col[None, :]
        return W

    def mixer_jacobian(self, C, theta):
        C = np.asarray(C, float)
        cols = []
        for j in range(8):
            h = 1e-6
            tp, tm = np.array(theta, float), np.array(theta, float)
            tp[j] += h
            tm[j] -= h
            wp = self._pred_wrench(C, tp[:4], tp[4:], (1, 1, 1, 1))
            wm = self._pred_wrench(C, tm[:4], tm[4:], (1, 1, 1, 1))
            cols.append(((wp - wm) / (2 * h)).ravel())
        return np.array(cols).T

    def estimate_mixer(self, experiments, rank_tol=1e-6, cond_max=1e6):
        """experiments: list of (cmd4, W_hat3, sigma3). Fits 4 signs, 4 strengths, 4 mount yaws.

        ABSTAIN when the commands do not excite all 8 continuous parameters (Fisher rank < 8 or
        cond > cond_max), when there are no spare residuals (UNDERDETERMINED), or when the sign
        pattern is ambiguous."""
        C = np.array([e[0] for e in experiments], float)
        Wh = np.array([e[1] for e in experiments], float)
        S = np.array([e[2] for e in experiments], float)
        J0 = self.mixer_jacobian(C, [1, 1, 1, 1, 0, 0, 0, 0])
        Jw = (J0.reshape(len(C), 3, 8) / S[:, :, None]).reshape(-1, 8)
        sv = np.linalg.svd(Jw / np.maximum(np.linalg.norm(Jw, axis=0), 1e-300), compute_uv=False)
        rank = int(np.sum(sv > rank_tol * sv[0]))
        cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else math.inf
        if rank < 8 or cond > cond_max:
            return abstain("rank_deficient_fisher", rank=rank, cond=cond)
        dof = Wh.size - 8
        if dof <= 0:
            return abstain(UNDERDETERMINED, dof=dof)
        results = []
        for signs in np.array(np.meshgrid(*[[1, -1]] * 4)).reshape(4, -1).T:
            th = np.array([1, 1, 1, 1, 0, 0, 0, 0], float)
            lam = 1e-3
            for _ in range(40):
                res = ((Wh - self._pred_wrench(C, th[:4], th[4:], signs)) / S).ravel()
                cost = float(res @ res)
                Jm = np.zeros((res.size, 8))
                for j in range(8):
                    h = 1e-6
                    tp, tm = th.copy(), th.copy()
                    tp[j] += h
                    tm[j] -= h
                    Jm[:, j] = ((self._pred_wrench(C, tp[:4], tp[4:], signs)
                                 - self._pred_wrench(C, tm[:4], tm[4:], signs)) / (2 * h) / S).ravel()
                A = Jm.T @ Jm + lam * np.eye(8)
                step = np.linalg.solve(A, Jm.T @ res)
                new = th + step
                new[:4] = np.clip(new[:4], 0.01, 4)
                new[4:] = np.clip(new[4:], -MOUNT_LIMIT, MOUNT_LIMIT)
                nres = ((Wh - self._pred_wrench(C, new[:4], new[4:], signs)) / S).ravel()
                if nres @ nres < cost:
                    th, lam = new, lam * 0.3
                    if np.linalg.norm(step) < 1e-9:
                        break
                else:
                    lam *= 10
                    if lam > 1e12:
                        break
            res = ((Wh - self._pred_wrench(C, th[:4], th[4:], signs)) / S).ravel()
            results.append((float(res @ res), tuple(int(x) for x in signs), th.copy()))
        results.sort(key=lambda x: x[0])
        best, second = results[0], results[1]
        if best[0] == 0.0:
            return abstain(DEGENERATE, why="zero_residual_variance")
        if best[0] > chi2_crit(dof, GOF_Z):
            return abstain("poor_fit_chi2", chi2=best[0], dof=dof)
        if second[0] - best[0] < SIGN_GAP_CHI2:
            return abstain("sign_pattern_ambiguous", best_chi2=best[0], second_chi2=second[0])
        th = best[2]
        return {"status": OK, "signs": best[1], "strengths": th[:4].tolist(),
                "angles_deg": np.degrees(th[4:]).tolist(), "chi2": best[0], "dof": dof,
                "rank": rank, "cond": cond}

    # ---------------------------------------------------------------- current vs body bias
    def separate_current_bias(self, headings, drift_xy, sigma, yaw_rate=None, max_std=0.01,
                              min_geometry=0.1, whiteness_alpha_z=GOF_Z, lags=5):
        """drift_xy[k] = R(psi_k)^T c + b + noise (body frame). Unknown c (world), b (body).

        Gates, in order, each with its own reason:
          rank_deficient_heading_diversity   structural (fewer than 4 independent equations)
          poor_heading_geometry              smin(A/sqrt(n)) < min_geometry: depends on the SPREAD of
                                             headings only, not on noise or sample count
          insufficient_precision             predicted std > max_std (noise / count dependent)
          UNDERDETERMINED                    no residual degrees of freedom (dof <= 0)
          DEGENERATE                         zero residual variance, or too few headings for even one
                                             Ljung-Box lag
          poor_fit_chi2                      residual chi2 against sigma too large (model misspecified,
                                             e.g. a ramping current, in ANY heading order)
          residual_not_white                 Ljung-Box on the residuals in visit order
        The Ljung-Box lag count is min(lags, (n-1)//2) so it always runs with n > 2h.
        """
        psi = np.asarray(headings, float)
        d = np.asarray(drift_xy, float)
        n = len(psi)
        A = np.zeros((2 * n, 4))
        A[0::2, 0], A[0::2, 1], A[0::2, 2] = np.cos(psi), np.sin(psi), 1.0
        A[1::2, 0], A[1::2, 1], A[1::2, 3] = -np.sin(psi), np.cos(psi), 1.0
        y = d.reshape(-1)
        sv = np.linalg.svd(A, compute_uv=False)
        if sv.size < 4 or sv[-1] < 1e-9 * sv[0]:
            return abstain("rank_deficient_heading_diversity", separated=False)
        geometry = float(np.linalg.svd(A / math.sqrt(n), compute_uv=False)[-1])
        if geometry < min_geometry:
            return abstain("poor_heading_geometry", separated=False, geometry=geometry)
        cov = sigma ** 2 * np.linalg.inv(A.T @ A)
        std = np.sqrt(np.diag(cov))
        if np.max(std) > max_std:
            return abstain("insufficient_precision", separated=False, predicted_std=float(np.max(std)))
        dof = 2 * n - 4
        if dof <= 0:
            return abstain(UNDERDETERMINED, separated=False, dof=dof)
        theta = np.linalg.solve(A.T @ A, A.T @ y)
        res = (y - A @ theta).reshape(n, 2)
        chi2 = float(np.sum(res ** 2) / sigma ** 2)
        if zero_variance(res.ravel(), scale=np.max(np.abs(y))):
            return abstain(DEGENERATE, separated=False, why="zero_residual_variance")
        if chi2 > chi2_crit(dof, whiteness_alpha_z):
            return abstain("poor_fit_chi2", separated=False, chi2=chi2, dof=dof)
        h = min(lags, (n - 1) // 2)
        q = 0.0
        for k in range(2):
            lb = ljung_box(res[:, k], h)
            if lb["status"] != OK:
                return abstain(DEGENERATE, separated=False, why=lb["why"], n=n, h=h)
            q += lb["q"]
        if q > chi2_crit(2 * h, whiteness_alpha_z):
            return abstain("residual_not_white", separated=False, ljung_box_q=float(q))
        out = {"status": OK, "separated": True, "current": theta[:2].tolist(),
               "bias_xy": theta[2:].tolist(), "std": std.tolist(), "ljung_box_q": float(q),
               "chi2": chi2, "dof": dof, "geometry": geometry}
        if yaw_rate is not None:
            out["bias_r"] = float(np.mean(yaw_rate))
        return out

    # ---------------------------------------------------------------- delay / latency
    def fit_delay(self, t_ref, channels_ref, t_obs, channels_obs, sigmas, max_delay=0.5, grid=0.001,
                  max_std=0.005):
        """Delay d minimising sum (obs(t) - ref(t - d))^2. ref is noise free. ABSTAIN if the
        Fisher information of d (from ref slope) gives std > max_std (5 ms)."""
        ds = np.arange(0.0, max_delay + 1e-12, grid)
        sse = np.zeros(len(ds))
        for j, d in enumerate(ds):
            for ref, obs, sg in zip(channels_ref, channels_obs, sigmas):
                sse[j] += np.sum(((obs - np.interp(t_obs - d, t_ref, ref)) / sg) ** 2)
        j = int(np.argmin(sse))
        d = float(ds[j])
        info = 0.0
        for ref, sg in zip(channels_ref, sigmas):
            slope = np.interp(t_obs - d, t_ref[:-1], np.diff(ref) / np.diff(t_ref))
            info += float(np.sum((slope / sg) ** 2))
        std = 1.0 / math.sqrt(info) if info > 0 else math.inf
        if std > max_std:
            return abstain("insufficient_excitation", delay_std_s=std)
        return {"status": OK, "delay_s": d, "std_s": std}

    def latency_split_fisher(self, pred_log, obs, total_s, truth_log=None):
        """Fisher information of (actuator dead time, observation stamp lag). Observation stream:
        obs(t) = pred(t - d_a - d_o), so both parameters have the SAME derivative column (rank 1).
        An independent truth stream truth(t) = pred(t - d_a) adds a row that depends on d_a only.
        Returns (identifiable, smin_normalised, cond)."""
        t = obs["t"]
        cols_a, cols_o = [], []
        for ch, sg in (("u", obs["sigma_v"]), ("r", obs["sigma_r"])):
            ref = getattr(pred_log, ch)
            slope = np.interp(t - total_s, pred_log.t[:-1], np.diff(ref) / np.diff(pred_log.t))
            g = -slope / sg
            cols_a.append(g)
            cols_o.append(g)
        J = np.column_stack([np.concatenate(cols_a), np.concatenate(cols_o)])
        if truth_log is not None:
            tt = truth_log.t
            rows = []
            for ch, sg in (("u", obs["sigma_v"]), ("r", obs["sigma_r"])):
                ref = getattr(pred_log, ch)
                slope = np.interp(tt, pred_log.t[:-1], np.diff(ref) / np.diff(pred_log.t))
                rows.append(-slope / sg)
            g = np.concatenate(rows)
            J = np.vstack([J, np.column_stack([g, np.zeros_like(g)])])
        Jn = J / np.maximum(np.linalg.norm(J, axis=0), 1e-300)
        sv = np.linalg.svd(Jn, compute_uv=False)
        cond = float(sv[0] / sv[-1]) if sv[-1] > 0 else math.inf
        return bool(cond < 1e6), float(sv[-1]), cond

    def estimate_latency(self, pred_log, obs, truth_log=None):
        """Total command->observation latency from the nominal undelayed prediction. The split into
        actuator dead time + observation stamp lag is reported only if the Fisher information of the
        two parameters is full rank, which needs an independent truth stream."""
        sg = [obs["sigma_v"], obs["sigma_r"]]
        tot = self.fit_delay(pred_log.t, [pred_log.u, pred_log.r], obs["t"], [obs["u"], obs["r"]], sg)
        if tot["status"] != OK:
            return {"status": ABSTAIN, "reason": tot["reason"],
                    "split": {"status": "UNIDENTIFIABLE", "reason": tot["reason"]}}
        out = {"status": OK, "total_latency_s": tot["delay_s"], "total_std_s": tot["std_s"]}
        ident, smin, cond = self.latency_split_fisher(pred_log, obs, tot["delay_s"], truth_log)
        out["fisher"] = {"identifiable": ident, "smin": smin, "cond": cond}
        if not ident:
            out["split"] = {"status": "UNIDENTIFIABLE", "reason": "fisher_rank_deficient", "cond": cond}
            return out
        ob = self.fit_delay(truth_log.t, [truth_log.u, truth_log.r], obs["t"], [obs["u"], obs["r"]], sg)
        if ob["status"] != OK:
            out["split"] = {"status": "UNIDENTIFIABLE", "reason": ob["reason"]}
            return out
        out["split"] = {"status": OK, "obs_stamp_lag_s": ob["delay_s"],
                        "actuator_dead_time_s": tot["delay_s"] - ob["delay_s"]}
        return out

    # ---------------------------------------------------------------- drag
    def fit_drag(self, speeds, forces, sigma_v, model="linear+quadratic"):
        """Steady holds: |F| = a|v| + b v|v|. model='linear' is the constant-gain drag."""
        v = np.asarray(speeds, float)
        F = np.asarray(forces, float)
        if model == "linear":
            X = v[:, None]
            E = np.full_like(X, sigma_v)
        else:
            X = np.column_stack([v, v * np.abs(v)])
            E = np.column_stack([np.full_like(v, sigma_v), 2 * np.abs(v) * sigma_v])
        ok_, smin, noise, cond = min_sv_vs_noise(X, E)
        if not ok_:
            return abstain("unidentifiable_drag_terms", smin=smin, noise=noise, cond=cond,
                           distinct_speeds=int(len(np.unique(np.round(np.abs(v), 6)))))
        th = np.linalg.lstsq(X, F, rcond=None)[0]
        return {"status": OK, "coeffs": th.tolist(), "cond": cond}

    def fit_thrust_drag_scale(self, A, B, y, sigma_B):
        """m*dv = s*A - kd*B (A: nominal thrust integral, B: nominal drag integral, y: m*dv).
        Steady-only data have y=0 and A parallel to B: only s/kd is determined -> ABSTAIN."""
        X = np.column_stack([A, -np.asarray(B)])
        E = np.column_stack([np.zeros(len(A)), sigma_B])
        ok_, smin, noise, cond = min_sv_vs_noise(X, E)
        if not ok_:
            return abstain("only_ratio_identifiable", smin=smin, noise=noise, cond=cond)
        th = np.linalg.lstsq(X, y, rcond=None)[0]
        return {"status": OK, "thrust_scale": float(th[0]), "drag_scale": float(th[1])}

    # ---------------------------------------------------------------- timing / stream checks
    def estimate_clock_skew(self, sensor_stamps, truth_stamps, max_std_s=0.006):
        """Linear fit sensor = (1+eps) truth + o. Returns skew ppm and the offset accrued at the end."""
        s = np.asarray(sensor_stamps, float)
        t = np.asarray(truth_stamps, float)
        if len(t) < 30 or np.ptp(t) <= 0:
            return abstain("too_few_samples")
        A = np.column_stack([t, np.ones_like(t)])
        th, res, *_ = np.linalg.lstsq(A, s, rcond=None)
        rms = float(np.sqrt(np.mean((s - A @ th) ** 2)))
        end = float((th[0] - 1) * (t[-1] - t[0]))
        std_end = rms * math.sqrt(12.0 / len(t)) * 2.0  # slope std * span, approximate
        if std_end > max_std_s:
            return abstain("noisy_clock", std_end_s=std_end)
        return {"status": OK, "skew_ppm": float((th[0] - 1) * 1e6), "offset_at_end_s": end}

    def detect_frozen(self, values, period=None, max_repeat_periods=2):
        """Index of the first sample at which the value has repeated for more than
        ``max_repeat_periods`` consecutive periods (fresh stamps, held value), else None."""
        v = np.asarray(values, float)
        run = 0
        for i in range(1, len(v)):
            run = run + 1 if v[i] == v[i - 1] else 0
            if run >= max_repeat_periods:
                return i
        return None

    def window_validity(self, stamps, t0, t1, max_gap=0.25, min_samples=30):
        """No interpolation across gaps. A window is usable only if it has >= min_samples and no
        gap longer than max_gap."""
        s = np.asarray(stamps, float)
        s = s[(s >= t0) & (s <= t1)]
        if len(s) < min_samples:
            return abstain("too_few_samples", n=int(len(s)))
        gaps = np.diff(np.concatenate([[t0], s, [t1]]))
        if gaps.max() > max_gap:
            return abstain("gap_exceeds_limit", max_gap_s=float(gaps.max()))
        return {"status": OK, "n": int(len(s))}

    def read_stamps(self, stamps):
        s = np.asarray(stamps, float)
        if np.any(np.diff(s) <= 0):
            raise ValueError("non-monotonic or duplicate stamps")
        return s

    def gyro_bias(self, t, r_obs, rest_mask, cmd_norm, sigma_r, settle_s=3.0, starts_at_rest=False):
        """Gyro bias = mean r over a REST window. The window is accepted only if the recorded command
        was zero from ``settle_s`` before the window to its end (``starts_at_rest`` waives the look-back
        before the first sample) AND r is consistent with a constant (chi-square on the scatter)."""
        t = np.asarray(t, float)
        m = np.asarray(rest_mask, bool)
        r = np.asarray(r_obs, float)
        if m.sum() < 30:
            return abstain("no_rest_window")
        t0, t1 = t[m].min(), t[m].max()
        lo = t0 - settle_s
        if lo < t[0] and not starts_at_rest:
            return abstain("rest_window_lacks_settle_history")
        if np.any(np.asarray(cmd_norm, float)[(t >= lo) & (t <= t1)] != 0.0):
            return abstain("rest_window_not_at_rest", detail="nonzero command in or before the window")
        x = r[m]
        if zero_variance(x):
            return abstain(DEGENERATE, why="zero_residual_variance")
        stat = float(np.sum((x - x.mean()) ** 2)) / sigma_r ** 2
        if stat > chi2_crit(len(x) - 1, GOF_Z):
            return abstain("rest_window_not_constant", chi2=stat)
        return {"status": OK, "bias_r": float(x.mean()), "n": int(len(x))}
