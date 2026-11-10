# SPDX-License-Identifier: GPL-3.0-or-later
"""Single-frame vs temporal vs evidence-accumulation vs adaptive inference on NOMAD runs, at matched compute.

A run = 4 consecutive annotated frames (<= 0.4 s apart) of one actor at one distance (the repeat-view unit).
The detector is frozen (COCO-pretrained RT-DETR-R18, never tuned on NOMAD). Every evidence parameter (score densities,
track prior, per-frame detection model) is fitted on the FIT sites only and evaluated on the HELD-OUT sites.

Methods (all report image-space alerts; compute in 640x640 tile inferences per run):
  single     : every detection >= tau_hi in any frame is an alert (OR over frames)                     24 tiles
  track2     : tracking-by-detection: detections >= tau_lo associated across frames (40 px gate);
               a track is reported when it has >= 2 hits                                               24 tiles
  evidence   : per track, log-odds = prior + sum over hits of tempered log f1(s)/f0(s) + sum over misses of
               log(1 - p_inc) (p from the fit-site detection model at the track's box height, correlated
               repeat views via beta-binomial rho); reported when posterior >= 0.5; posterior is calibrated  24 tiles
  full4      : full-frame 640 letterbox on all 4 frames, single-frame rule                              4 tiles
  tiled1     : tiled on 1 of 4 frames (frame skip), single-frame rule                                  6 tiles
  adaptive   : full-frame when the expected person height (known from the flight geometry; here the distance
               stratum's median box height on FIT sites) >= 64 px, else tiled on all frames             4 or 24
Usage: python -I compare_temporal.py BENCH_DIR PARAMS_JSON OUT_DIR [--blur 0]
"""
import argparse, csv, json, math, sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fit_obs as F  # noqa: E402

TAU_HI, TAU_LO, GATE = 0.5, 0.3, 40.0


def tracks(frames):
    """frames: list of (boxes, scores, tploc) with score >= TAU_LO kept. Greedy nearest-centre association."""
    T = []
    for t, (b, s, tp) in enumerate(frames):
        c = 0.5 * (b[:, :2] + b[:, 2:]); h = b[:, 3] - b[:, 1]
        used = set()
        for tr in T:
            if len(c) == 0:
                break
            d = np.hypot(*(c - tr['c']).T); d[list(used)] = 1e9
            j = int(np.argmin(d))
            if d[j] < GATE:
                tr['hits'].append((t, float(s[j]), bool(tp[j]), float(h[j]))); tr['c'] = c[j]; used.add(j)
        for j in range(len(c)):
            if j not in used:
                T.append(dict(c=c[j], start=t, hits=[(t, float(s[j]), bool(tp[j]), float(h[j]))]))
    for tr in T:
        tr['true'] = np.mean([x[2] for x in tr['hits']]) >= 0.5
    return T


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bench'); ap.add_argument('params'); ap.add_argument('out')
    ap.add_argument('--blur', type=int, default=0)
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    P = json.load(open(a.params)); th = P['detect']['theta']           # FIT-SITE-ONLY detection model
    theta = np.array([th[k] for k in ('logit_pmax', 'b0', 'b_log2h', 'b_vis', 'b_blur', 'b_overcast', 'b_lowsun')])
    rho = P['repeat_view']['rho']
    gt = [r for r in csv.DictReader(open(Path(a.bench) / 'gt.csv')) if r['gt_idx'] == '0' and r['blur_px'] == str(a.blur)]
    D = np.load(Path(a.bench) / 'dets.npz')
    meta = {}
    for r in gt:
        meta.setdefault((r['file'], r['mode']), r)
    # runs
    runs = defaultdict(list)
    for (f, mode), r in meta.items():
        if mode == 'tiled':
            runs[r['folder']].append(r)
    R = []
    for k, v in runs.items():
        v.sort(key=lambda r: int(r['frame']))
        cur = [v[0]]
        for x, y in zip(v, v[1:]):
            if int(y['frame']) - int(x['frame']) <= 12:
                cur.append(y)
            else:
                R.append(cur); cur = [y]
        R.append(cur)
    R = [g for g in R if len(g) == 4]

    def frame(f, mode, lo=TAU_LO):
        key = f'{f}|{mode}|{a.blur}'
        if key + '|scores' not in D.files:      # full mode was benchmarked at blur 0 only
            return np.zeros((0, 4)), np.zeros(0), np.zeros(0, bool)
        s = D[key + '|scores']; k = s >= lo
        return D[key + '|boxes'][k], s[k], D[key + '|tploc'][k]

    # expected size per distance stratum from FIT sites (flight geometry proxy)
    hs = defaultdict(list)
    for g in R:
        if g[0]['split'] == 'fit':
            hs[g[0]['dist_m']] += [float(r['h_px']) for r in g]
    exp_h = {d: float(np.median(v)) for d, v in hs.items()}
    # fit-site score densities for track hits (true vs false), Beta on u=(s-TAU_LO)/(1-TAU_LO)
    s1, s0, ntrue, nall = [], [], 0, 0
    for g in R:
        if g[0]['split'] != 'fit':
            continue
        for tr in tracks([frame(r['file'], 'tiled') for r in g]):
            nall += 1; ntrue += tr['true']
            (s1 if tr['true'] else s0).extend(x[1] for x in tr['hits'])
    u = lambda s: np.clip((np.array(s) - TAU_LO) / (1 - TAU_LO), 1e-4, 1 - 1e-4)
    b1, b0 = F.beta_mle(u(s1)), F.beta_mle(u(s0))
    prior = ntrue / max(nall, 1)
    from scipy.stats import beta as B

    def llr_hit(s):
        x = float(u([s])[0]); return float(B.logpdf(x, *b1) - B.logpdf(x, *b0))

    def p_frame(h):
        X = F.features(np.array([h]), np.array([0.5]), np.array([a.blur]), np.array([0.0]), np.array([0.0]))
        return float(F.predict(theta, X)[0])

    def evidence(tr, n_frames=4):
        L = math.log(prior / (1 - prior)); hits = {x[0]: x for x in tr['hits']}
        h = float(np.mean([x[3] for x in tr['hits']])); p = p_frame(h)
        n_prev = 0
        for t in range(tr['start'], n_frames):
            kappa = 1.0 if n_prev == 0 else (n_prev + 1) / (1 + n_prev * rho) - n_prev / (1 + max(n_prev - 1, 0) * rho)
            if t in hits:
                L += kappa * llr_hit(hits[t][1])
            else:
                L += kappa * math.log(max(1 - p, 1e-6))
            n_prev += 1
        return 1 / (1 + math.exp(-L))

    res = {m: dict(found=0, fa=0, tiles=0, n=0) for m in ('single', 'track2', 'evidence', 'full4', 'tiled1', 'adaptive')}
    rel_p, rel_y = [], []
    for g in R:
        if g[0]['split'] != 'heldout':
            continue
        fr_t = [frame(r['file'], 'tiled', TAU_LO) for r in g]
        fr_f = [frame(r['file'], 'full', TAU_LO) for r in g]

        def single(frs):
            T = tracks([(b[s >= TAU_HI], s[s >= TAU_HI], tp[s >= TAU_HI]) for b, s, tp in frs])
            return any(t['true'] for t in T), sum(not t['true'] for t in T)
        for m, (fd, fa, tiles) in {
            'single': (*single(fr_t), 24), 'full4': (*single(fr_f), 4), 'tiled1': (*single(fr_t[:1]), 6),
        }.items():
            res[m]['found'] += fd; res[m]['fa'] += fa; res[m]['tiles'] += tiles; res[m]['n'] += 1
        T = tracks(fr_t)
        rep = [t for t in T if len(t['hits']) >= 2]
        res['track2']['found'] += any(t['true'] for t in rep); res['track2']['fa'] += sum(not t['true'] for t in rep)
        res['track2']['tiles'] += 24; res['track2']['n'] += 1
        post = [(evidence(t), t['true']) for t in T]
        rel_p += [p for p, _ in post]; rel_y += [float(y) for _, y in post]
        res['evidence']['found'] += any(p >= 0.5 and y for p, y in post); res['evidence']['fa'] += sum(p >= 0.5 and not y for p, y in post)
        res['evidence']['tiles'] += 24; res['evidence']['n'] += 1
        use_full = exp_h.get(g[0]['dist_m'], 0) >= 64
        fd, fa = single(fr_f) if use_full else single(fr_t)
        res['adaptive']['found'] += fd; res['adaptive']['fa'] += fa; res['adaptive']['tiles'] += 4 if use_full else 24; res['adaptive']['n'] += 1
    table = {m: dict(person_recall=v['found'] / v['n'], false_reports_per_run=v['fa'] / v['n'],
                     tiles_per_run=v['tiles'] / v['n'], runs=v['n']) for m, v in res.items()}
    if a.blur != 0:
        for m in ('full4', 'adaptive'):
            table[m] = 'not benchmarked (full mode ran at blur 0 only)'
    cal = F.metrics(np.array(rel_p), np.array(rel_y))
    summary = dict(blur_px=a.blur, heldout_runs=res['single']['n'], methods=table, evidence_track_calibration=cal,
                   fit_site_score_beta=dict(true=b1, false=b0), track_prior=prior, expected_h_by_distance=exp_h,
                   rho=rho, tau_hi=TAU_HI, tau_lo=TAU_LO, gate_px=GATE)
    json.dump(summary, open(out / f'temporal_blur{a.blur}.json', 'w'), indent=1)
    print(json.dumps(dict(blur=a.blur, methods=table, ece=cal['ece'], logloss=cal['logloss'], n_tracks=cal['n'],
                          base=cal['base_rate']), indent=1))
    import matplotlib
    matplotlib.use('Agg'); matplotlib.rcParams['pdf.use14corefonts'] = True
    import matplotlib.pyplot as plt
    rel = np.array(cal['reliability'])
    fig, ax = plt.subplots(figsize=(4, 3.6))
    ax.plot([0, 1], [0, 1], ls='--', color='0.6', lw=1)
    ax.plot(rel[:, 0], rel[:, 1], 'o-', color='#1f5fa8')
    ax.set_xlabel('track posterior P(person)'); ax.set_ylabel('observed fraction true')
    ax.set_title(f"evidence accumulation, held-out sites, blur {a.blur} (ECE {cal['ece']:.3f})", fontsize=8)
    ax.spines[['top', 'right']].set_visible(False); ax.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(out / f'evidence_reliability_blur{a.blur}.png', dpi=150); fig.savefig(out / f'evidence_reliability_blur{a.blur}.pdf')


if __name__ == '__main__':
    main()
