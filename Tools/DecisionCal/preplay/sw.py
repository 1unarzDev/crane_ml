"""PREPLAY stopwatch extractor. Differences from baseline_selector._step/_cross: single pulse (no reversal
'flip'), onset = first non-neutral PWM message log time (t=0), non-uniform raw samples, per-trial
Monte-Carlo noise uncertainty, no 'tau' (rise) output; otherwise the same rule: first sustained (2 samples)
crossing of max(10% of peak, 3 baseline SD), interpolated, minus onset."""
import numpy as np

def smooth3(y):
    return np.convolve(np.pad(y, 1, mode='edge'), np.ones(3) / 3, 'valid')

def cross(t, y, thr, start):
    for i in range(max(start, 1), len(y) - 1):
        if y[i] >= thr and y[i + 1] >= thr:
            lo = y[i - 1]
            f = 0.0 if y[i] == lo else np.clip((thr - lo) / (y[i] - lo), 0.0, 1.0)
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
    return np.nan

def prep(t, y, t_off, base=(-10.0, -0.3), smooth=True, sd_from='raw'):
    """-> dict(t, ys sign-aligned baseline-removed (smoothed), sd, sgn, y0)"""
    bm = (t >= base[0]) & (t < base[1])
    A = np.stack([np.ones(bm.sum()), t[bm]], 1)
    coef = np.linalg.lstsq(A, y[bm], rcond=None)[0]
    res_raw = y[bm] - A @ coef
    y0 = float(np.median(y[bm]))
    ys = (smooth3(y - y0) if smooth else y - y0)
    sd_raw = float(res_raw.std())
    sd_sm = float((smooth3(res_raw) if smooth else res_raw).std())
    win = (t >= 0.6 * t_off) & (t <= t_off + 0.3)
    sgn = 1.0 if np.median(ys[win]) >= 0 else -1.0
    return dict(t=t, ys=sgn * ys, sd=sd_raw if sd_from == 'raw' else sd_sm, sd_raw=sd_raw, sd_sm=sd_sm, sgn=sgn, y0=y0,
                res=res_raw, base=bm)

def step(t, ys, sd, t_off, post=1.5, frac=0.1, k=3.0):
    m = (t >= 0) & (t <= t_off + post)
    if not m.any():
        return dict(peak=np.nan, delay=np.nan, t_peak=np.nan)
    ip = np.flatnonzero(m)[np.argmax(ys[m])]
    level = float(ys[ip])
    out = dict(peak=level, delay=np.nan, t_peak=float(t[ip]))
    if level < 6 * sd or level <= 0:
        return out
    thr = max(frac * level, k * sd)
    if thr > 0.5 * level:
        return out
    s = int(np.searchsorted(t, 0.0 - 1e-9))
    t10 = cross(t, ys, thr, s)
    out['delay'] = t10 if np.isfinite(t10) else np.nan
    out['thr'] = thr
    return out

def stopwatch(t, y, t_off, mc=300, seed=0, smooth=True, sd_from='raw', frac=0.1, quant=None):
    p = prep(t, y, t_off, smooth=smooth, sd_from=sd_from)
    r = step(t, p['ys'], p['sd'], t_off, frac=frac)
    r['sd_noise'] = p['sd']; r['sgn'] = p['sgn']
    # noise MC: add circularly shifted baseline residual (keeps autocorrelation) to the signal
    rng = np.random.default_rng(seed)
    res = p['res']; n = len(t); ds = []
    if mc and np.isfinite(r['delay']):
        for _ in range(mc):
            sh = rng.integers(0, len(res)); noise = np.resize(np.roll(res, sh), n)
            yy = y - p['y0'] + noise   # baseline already removed by y0 in prep; add fresh noise
            pp = prep(t, yy + p['y0'], t_off, smooth=smooth, sd_from=sd_from)
            ds.append(step(t, pp['ys'] * (1 if pp['sgn'] == p['sgn'] else -1) if False else pp['ys'], pp['sd'], t_off, frac=frac)['delay'])
        ds = np.array(ds)
        r['mc_sd'] = float(np.nanstd(ds)); r['mc_fail'] = float(np.mean(~np.isfinite(ds)))
    else:
        r['mc_sd'] = np.nan; r['mc_fail'] = np.nan
    q = quant if quant is not None else float(np.median(np.diff(t)))
    r['q_sd'] = q / np.sqrt(12)
    r['se'] = float(np.hypot(r['mc_sd'], r['q_sd'])) if np.isfinite(r['mc_sd']) else np.nan
    return r
