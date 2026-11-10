# SPDX-License-Identifier: GPL-3.0-or-later
"""Fit the calibrated observation model (obs_params_tau*.json) to bench.py output and validate it on held-out LOCATIONS.

Per-frame detection model (one GT person, tiled mode, 'loc' criterion, score >= tau):
    p = p_max * sigmoid(b0 + b1*(log2 h - 5) + b2*(vis - 1) + b3*log1p(blur/h_eff) + b4*overcast + b5*low_sun)
  h = apparent person box height (px in the 1920x1080 frame), vis = visible fraction (NOMAD label / 100),
  blur = motion-blur length (px), h_eff = max(h, 4).
Confidence: detector score s of a detection, u = (s - tau) / (1 - tau) ~ Beta(a0 + a1*q, b) for true detections
(q = model p), Beta(af, bf) for false alerts.
False alerts: per-frame Poisson rate (tiled, tau) vs blur; persistent fraction (decoys).
Repeat views: residual lag-1 correlation of outcomes given the model (beta-binomial ICC rho).
Localisation: box-centre error sigma_px = c0 + c1*h.

Validation (held-out locations, never seen in the fit): log loss, Brier, ECE vs two baselines (marginal rate and
the MissionSim/NavPilot placeholder curve), reliability diagram (calibration.png/.pdf).

Usage: python -I fit_obs.py BENCH_DIR METRICS_JSON OUT_PARAMS_JSON FIG_DIR [--tau 0.3]
"""
import argparse, csv, json, math, sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.special import betaln, expit, logit
from scipy.stats import beta as beta_dist

sys.path.insert(0, str(Path(__file__).resolve().parent))


def features(h, vis, blur, overcast, low_sun):
    h = np.asarray(h, float); he = np.maximum(h, 4.0)
    return np.stack([np.ones_like(h), np.log2(he) - 5.0, np.asarray(vis, float) - 1.0,
                     np.log1p(np.asarray(blur, float) / he), np.asarray(overcast, float), np.asarray(low_sun, float)], 1)


def predict(theta, X):
    return expit(theta[0]) * expit(X @ theta[1:])


def nll(theta, X, y, l2=1e-3):
    p = np.clip(predict(theta, X), 1e-6, 1 - 1e-6)
    return -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p)) + l2 * np.sum(theta[2:] ** 2)


def fit_detect(X, y):
    th0 = np.array([2.0, 0.0, 1.5, 2.0, -1.0, 0.0, 0.0])
    r = minimize(nll, th0, args=(X, y), method='L-BFGS-B')
    return r.x


GH_X, GH_W = np.polynomial.hermite_e.hermegauss(15)
GH_W = GH_W / GH_W.sum()


def predict_pa(theta, X, sigma):
    """Population-average P over a scene offset u ~ N(0, sigma^2) on the logit (Gauss-Hermite)."""
    z = X @ theta[1:]
    return expit(theta[0]) * (expit(z[:, None] + sigma * GH_X[None, :]) @ GH_W)


def fit_scene_effects(X, y, loc, l2_scene=0.5):
    """Fixed effects + one ridge-penalised logit offset per scene (location). Returns theta, offsets dict, sigma."""
    locs = sorted(set(loc)); idx = np.array([locs.index(l) for l in loc]); K = len(locs)
    def f(t):
        th, u = t[:7], t[7:]
        p = np.clip(expit(th[0]) * expit(X @ th[1:] + u[idx]), 1e-6, 1 - 1e-6)
        return -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p)) + 1e-3 * np.sum(th[2:] ** 2) + l2_scene * np.mean(u ** 2) / len(y) * K
    t0 = np.concatenate([fit_detect(X, y), np.zeros(K)])
    r = minimize(f, t0, method='L-BFGS-B')
    th, u = r.x[:7], r.x[7:]
    th = th.copy(); th[1] += u.mean(); u = u - u.mean()   # centre offsets: b0 is the median scene
    return th, dict(zip(locs, map(float, u))), float(np.sqrt(np.mean(u ** 2) * K / max(K - 1, 1)))


def metrics(p, y, nb=10):
    p = np.clip(p, 1e-6, 1 - 1e-6)
    ll = -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p)); br = np.mean((p - y) ** 2)
    edges = np.linspace(0, 1, nb + 1); ece = 0.0; rel = []
    for i in range(nb):
        k = (p >= edges[i]) & (p < edges[i + 1] if i < nb - 1 else p <= 1)
        if k.any():
            ece += k.mean() * abs(p[k].mean() - y[k].mean())
            rel.append((float(p[k].mean()), float(y[k].mean()), int(k.sum())))
    return dict(logloss=float(ll), brier=float(br), ece=float(ece), reliability=rel, n=int(len(y)), base_rate=float(y.mean()))


def placeholder_curve(h, vis, blur):
    """MissionSim/NavPilot-style closed form (h50 18 px, scale 5 px, p_max 0.9, visibility ramp, blur scale 6 px),
    rescaled from its 640-px frame to the 1920 frame (x3) for comparison."""
    h640 = np.asarray(h) / 3.0; b640 = np.asarray(blur) / 3.0
    pv = np.clip((np.asarray(vis) - 0.2) / 0.5, 0, 1)
    return 0.9 / (1 + np.exp(-(h640 - 18.0) / 5.0)) * pv * np.exp(-b640 / 6.0)


def beta_mle(u, w=None):
    u = np.clip(u, 1e-4, 1 - 1e-4)
    f = lambda t: -np.sum((np.exp(t[0]) - 1) * np.log(u) + (np.exp(t[1]) - 1) * np.log(1 - u) - betaln(np.exp(t[0]), np.exp(t[1])))
    r = minimize(f, [0.0, 0.5], method='Nelder-Mead')
    return float(np.exp(r.x[0])), float(np.exp(r.x[1]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bench'); ap.add_argument('metrics'); ap.add_argument('out'); ap.add_argument('figdir')
    ap.add_argument('--tau', type=float, default=0.3)
    a = ap.parse_args(); tau = a.tau
    bench = Path(a.bench); figdir = Path(a.figdir); figdir.mkdir(parents=True, exist_ok=True)
    M = json.load(open(a.metrics))
    rows = [r for r in csv.DictReader(open(bench / 'gt.csv')) if r['mode'] == 'tiled']
    def arr(key, rs, f=float):
        return np.array([f(r[key]) for r in rs])
    out = {}
    split = {}
    for s in ('fit', 'heldout'):
        rs = [r for r in rows if r['split'] == s]
        X = features(arr('h_px', rs), arr('visibility', rs) / 100.0, arr('blur_px', rs),
                     np.array([r['sky'] == 'overcast' for r in rs]), np.array([r['sun'] == 'low_sun' for r in rs]))
        y = (arr('scoreloc', rs) >= tau).astype(float)
        split[s] = (rs, X, y)
    rs, X, y = split['fit']
    theta = fit_detect(X, y)
    # bootstrap over FIT actors (cluster bootstrap) for parameter uncertainty
    actors = np.array([int(r['actor']) for r in rs]); ua = np.unique(actors); rng = np.random.default_rng(20261109)
    boots = []
    for _ in range(60):
        pick = rng.choice(ua, len(ua))
        idx = np.concatenate([np.where(actors == q)[0] for q in pick])
        boots.append(minimize(nll, theta, args=(X[idx], y[idx]), method='L-BFGS-B').x)
    boots = np.array(boots)
    names = ['logit_pmax', 'b0', 'b_log2h', 'b_vis', 'b_blur', 'b_overcast', 'b_lowsun']
    out['detect'] = dict(form='p = sigmoid(logit_pmax) * sigmoid(b0 + b_log2h*(log2(max(h,4))-5) + b_vis*(vis-1) + b_blur*log1p(blur/max(h,4)) + b_overcast*overcast + b_lowsun*low_sun)',
                         theta=dict(zip(names, map(float, theta))),
                         theta_sd_actor_bootstrap=dict(zip(names, map(float, boots.std(0)))),
                         h_units='px, person box height in the 1920x1080 frame', tau=tau, criterion='loc (IoU>=0.25 or centre in box)',
                         mode='tiled 6x640', n_fit=int(len(y)))
    # validation
    rh, Xh, yh = split['heldout']
    ph = predict(theta, Xh)
    pm = np.full_like(yh, y.mean())
    pp = placeholder_curve(arr('h_px', rh), arr('visibility', rh) / 100.0, arr('blur_px', rh))
    out['validation_heldout_locations'] = dict(model=metrics(ph, yh), marginal=metrics(pm, yh), placeholder=metrics(pp, yh),
                                              fit_split_in_sample=metrics(predict(theta, X), y),
                                              heldout_locations=sorted({r['location'] for r in rh}),
                                              fit_locations=sorted({r['location'] for r in rs}))
    # per-blur held-out calibration check
    bl = arr('blur_px', rh)
    out['validation_by_blur'] = {str(int(L)): dict(pred=float(ph[bl == L].mean()), obs=float(yh[bl == L].mean()), n=int((bl == L).sum()))
                                 for L in np.unique(bl)}
    hh = arr('h_px', rh)
    out['validation_by_height'] = {}
    for lo, hi in ((0, 16), (16, 32), (32, 64), (64, 128), (128, 1e9)):
        k = (hh >= lo) & (hh < hi)
        if k.any():
            out['validation_by_height'][f'{lo}-{hi:g}'] = dict(pred=float(ph[k].mean()), obs=float(yh[k].mean()), n=int(k.sum()))

    # ---- scene (location) random effect: leave-one-location-out CV over all 9 scenes, population-average predictions
    allr_ = rows
    Xall = features(arr('h_px', allr_), arr('visibility', allr_) / 100.0, arr('blur_px', allr_),
                    np.array([r['sky'] == 'overcast' for r in allr_]), np.array([r['sun'] == 'low_sun' for r in allr_]))
    yall = (arr('scoreloc', allr_) >= tau).astype(float); lall = np.array([r['location'] for r in allr_])
    pcv = np.zeros_like(yall); pcv_fixed = np.zeros_like(yall); pmarg = np.zeros_like(yall); lolo = {}
    for L_ in sorted(set(lall)):
        tr, te = lall != L_, lall == L_
        th_, offs_, sg_ = fit_scene_effects(Xall[tr], yall[tr], lall[tr])
        pcv[te] = predict_pa(th_, Xall[te], sg_)
        pcv_fixed[te] = predict(fit_detect(Xall[tr], yall[tr]), Xall[te])
        pmarg[te] = yall[tr].mean()
        lolo[L_] = dict(n=int(te.sum()), obs=float(yall[te].mean()), pred_pa=float(pcv[te].mean()), sigma_train=sg_)
    pph = placeholder_curve(arr('h_px', allr_), arr('visibility', allr_) / 100.0, arr('blur_px', allr_))
    out['validation_lolo'] = dict(model_scene_pa=metrics(pcv, yall), model_no_scene=metrics(pcv_fixed, yall),
                                  marginal=metrics(pmarg, yall), placeholder=metrics(pph, yall), per_location=lolo,
                                  note='each location predicted by a model fitted on the other 8 (population average over the scene effect)')
    ball = arr('blur_px', allr_); hall = arr('h_px', allr_)
    out['validation_lolo_by_blur'] = {str(int(L)): dict(pred=float(pcv_fixed[ball == L].mean()), obs=float(yall[ball == L].mean()),
                                                        n=int((ball == L).sum())) for L in np.unique(ball)}
    out['validation_lolo_by_height'] = {}
    for lo, hi in ((0, 16), (16, 32), (32, 64), (64, 128), (128, 1e9)):
        k_ = (hall >= lo) & (hall < hi)
        if k_.any():
            out['validation_lolo_by_height'][f'{lo}-{hi:g}'] = dict(pred=float(pcv_fixed[k_].mean()), obs=float(yall[k_].mean()), n=int(k_.sum()))
    thF, offsF, sgF = fit_scene_effects(Xall, yall, lall)
    out['detect_scene'] = dict(theta=dict(zip(names, map(float, thF))), sigma_scene=sgF, scene_offsets=offsF,
                               form='as detect, plus a per-scene logit offset u ~ N(0, sigma_scene^2); theta is the median scene; '
                                    'fitted on all 9 locations (final model used by obs_model.py)', n=int(len(yall)))
    thP = fit_detect(Xall, yall)
    out['detect_pooled'] = dict(theta=dict(zip(names, map(float, thP))), n=int(len(yall)),
                                form=out['detect']['form'],
                                note='FINAL per-frame model: fixed effects fitted on all 9 locations (population average over scenes); '
                                     'its out-of-scene calibration is the validation_lolo.model_no_scene entry')
    theta_final = thP

    # confidence distributions (all splits pooled for the TP family by q; FP from dets)
    allr = [r for r in rows]
    Xa = features(arr('h_px', allr), arr('visibility', allr) / 100.0, arr('blur_px', allr),
                  np.array([r['sky'] == 'overcast' for r in allr]), np.array([r['sun'] == 'low_sun' for r in allr]))
    sa = arr('scoreloc', allr); qa = predict(theta_final, Xa); k = sa >= tau
    u = np.clip((sa[k] - tau) / (1 - tau), 1e-4, 1 - 1e-4); q = qa[k]
    f = lambda t: -np.sum(beta_dist.logpdf(u, np.exp(t[0]) + np.exp(t[1]) * q, np.exp(t[2])))
    r = minimize(f, [0.0, 0.0, 0.5], method='Nelder-Mead', options=dict(maxiter=4000))
    out['conf_true'] = dict(a0=float(np.exp(r.x[0])), a1=float(np.exp(r.x[1])), b=float(np.exp(r.x[2])),
                            form='u=(s-tau)/(1-tau) ~ Beta(a0 + a1*q, b), q = per-frame P(detect)', n=int(k.sum()))
    D = np.load(bench / 'dets.npz')
    fp_s = []; fp_counts = {}
    for key in D.files:
        if not key.endswith('|scores'):
            continue
        f_, mode, L, _ = key.split('|')
        if mode != 'tiled':
            continue
        s_ = D[key]; tp = D[key.replace('|scores', '|tploc')]
        m = (s_ >= tau) & ~tp
        fp_counts.setdefault(int(L), []).append(int(m.sum()))
        if int(L) == 0:
            fp_s += list(s_[m])
    uf = np.clip((np.array(fp_s) - tau) / (1 - tau), 1e-4, 1 - 1e-4)
    af, bf = beta_mle(uf)
    out['conf_false'] = dict(a=af, b=bf, n=int(len(uf)))
    out['false_alerts'] = dict(per_frame_by_blur={str(L): float(np.mean(v)) for L, v in sorted(fp_counts.items())},
                               per_frame=float(np.mean(fp_counts[0])), frame_px=[1920, 1080],
                               persistent_fraction=M['fp_persistence_lag1']['frac'], tau=tau,
                               note='NOMAD frames annotate only the actor; bystanders would count as false alerts (upper bound).')
    out['repeat_view'] = dict(rho=float(max(0.0, M['repeat_view']['residual_corr_lag1_given_height_vis'])),
                              icc_within_run_raw=M['repeat_view']['icc_within_run'],
                              icc_runs_within_flight=M['repeat_view']['icc_runs_within_flight'],
                              p_next_given_det=M['repeat_view']['p_next_given_det'], p_next_given_miss=M['repeat_view']['p_next_given_miss'],
                              frame_gap_s='<=0.4 (<=12 frames at 30 fps)',
                              use='beta-binomial: P(>=1 detect in n looks) = 1 - E[(1-P)^n], P ~ Beta(mean p, ICC rho)')
    le = M['centre_error_px']
    hmid = {'<16': 12, '16-32': 24, '32-64': 45, '64-128': 90, '>=128': 160}
    hx = np.array([hmid[x['stratum']] for x in le]); sx = np.array([math.sqrt(0.5 * (x['rms_x'] ** 2 + x['rms_y'] ** 2)) for x in le])
    c1, c0 = np.polyfit(hx, sx, 1, w=np.sqrt([x['n'] for x in le]))
    out['centre_error'] = dict(c0=float(max(c0, 0.5)), c1=float(max(c1, 0.0)), form='sigma_px (per axis) = c0 + c1*h_px', table=le)
    out['provenance'] = dict(detector='RT-DETR-R18 (PekingU/rtdetr_r18vd, Apache-2.0), COCO-pretrained, not fine-tuned',
                             data='NOMAD (CC BY 4.0), location-split subset, frames resized 5472x3078 -> 1920x1080',
                             bench_dir=str(bench), evidence='HSUR: real aerial imagery, simulated blur; not CRANE renders')
    json.dump(out, open(a.out, 'w'), indent=1)
    calib_fig(out, figdir)
    print('LOLO', json.dumps({k: {kk: out['validation_lolo'][k][kk] for kk in ('logloss', 'brier', 'ece')} for k in ('model_scene_pa', 'model_no_scene', 'marginal', 'placeholder')}, indent=1))
    print('sigma_scene', sgF, offsF)
    v = out['validation_heldout_locations']
    print(json.dumps({k: {kk: v[k][kk] for kk in ('logloss', 'brier', 'ece', 'n')} for k in ('model', 'marginal', 'placeholder')}, indent=1))
    print(json.dumps(out['detect']['theta'], indent=1))


def calib_fig(out, figdir):
    import matplotlib
    matplotlib.use('Agg'); matplotlib.rcParams['pdf.use14corefonts'] = True  # Type 1 core fonts in PDFs
    import matplotlib.pyplot as plt
    v = out['validation_heldout_locations']
    fig, ax = plt.subplots(1, 3, figsize=(12.6, 3.6))
    for key, lab, c in (('model', 'fitted model', '#1f5fa8'), ('placeholder', 'MissionSim placeholder', '#c2571a')):
        rel = np.array(v[key]['reliability'])
        ax[0].scatter(rel[:, 0], rel[:, 1], s=8 + 40 * rel[:, 2] / rel[:, 2].max(), color=c, label=f"{lab} (ECE {v[key]['ece']:.3f})")
        ax[0].plot(rel[:, 0], rel[:, 1], color=c, lw=1)
    ax[0].plot([0, 1], [0, 1], color='0.6', ls='--', lw=1)
    ax[0].set_xlabel('predicted P(detect)'); ax[0].set_ylabel('observed frequency'); ax[0].set_title('pre-declared split: 4 fit -> 5 held-out locations', fontsize=9)
    ax[0].legend(frameon=False, fontsize=8)
    w = out['validation_lolo']
    for key, lab, c in (('model_no_scene', 'final model (pooled fit)', '#1f5fa8'), ('placeholder', 'MissionSim placeholder', '#c2571a')):
        rel = np.array(w[key]['reliability'])
        ax[2].scatter(rel[:, 0], rel[:, 1], s=8 + 40 * rel[:, 2] / rel[:, 2].max(), color=c, label=f"{lab} (ECE {w[key]['ece']:.3f})")
        ax[2].plot(rel[:, 0], rel[:, 1], color=c, lw=1)
    ax[2].plot([0, 1], [0, 1], color='0.6', ls='--', lw=1)
    ax[2].set_xlabel('predicted P(detect)'); ax[2].set_title('leave-one-location-out (9 folds)', fontsize=9); ax[2].legend(frameon=False, fontsize=8)
    vb = out['validation_lolo_by_blur']; Ls = sorted(vb, key=int)
    ax[1].plot([int(L) for L in Ls], [vb[L]['obs'] for L in Ls], 'o-', color='0.2', label='observed')
    ax[1].plot([int(L) for L in Ls], [vb[L]['pred'] for L in Ls], 's--', color='#1f5fa8', label='final model, LOLO')
    ax[1].set_xlabel('motion blur (px)'); ax[1].set_ylabel('mean P(detect)'); ax[1].set_title('by blur, leave-one-location-out', fontsize=9); ax[1].legend(frameon=False)
    for a_ in ax:
        a_.spines[['top', 'right']].set_visible(False); a_.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(figdir / 'calibration.png', dpi=150); fig.savefig(figdir / 'calibration.pdf'); plt.close(fig)


if __name__ == '__main__':
    main()
