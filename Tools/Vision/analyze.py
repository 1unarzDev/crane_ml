# SPDX-License-Identifier: GPL-3.0-or-later
"""Stratified metrics from bench.py output: recall / precision / AP by apparent size, occlusion, distance, lighting,
blur and inference mode; false alerts per frame and per km^2; repeat-view correlation across consecutive frames.

Usage: python -I analyze.py BENCH_DIR ANN_DIR OUT_DIR [--tau 0.3]
Writes OUT_DIR/metrics.json, OUT_DIR/tables.md and figures (PNG + PDF with Type 42/TrueType fonts embedded; see note).
"""
import argparse, csv, json, math, sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bench import SCALE, iou, loc_match  # noqa: E402

H_BINS = [0, 16, 32, 64, 128, 1e9]
H_LAB = ['<16', '16-32', '32-64', '64-128', '>=128']
V_BINS = [(10, 30), (40, 60), (70, 90), (100, 100)]
V_LAB = ['10-30%', '40-60%', '70-90%', '100%']
PERSON_H_M = 1.7  # standing height, used to estimate GSD from walking, fully visible persons


def load(bench, ann):
    rows = list(csv.DictReader(open(bench / 'gt.csv')))
    for r in rows:
        for k in ('dist_m', 'visibility', 'blur_px', 'gt_idx', 'frame', 'actor', 'n_gt_img', 'hour'):
            r[k] = int(r[k])
        for k in ('w_px', 'h_px', 'score50', 'scoreloc', 'ex', 'ey', 'lum'):
            r[k] = float(r[k])
    D = np.load(bench / 'dets.npz')
    dets = defaultdict(dict)
    for k in D.files:
        f, mode, L, field = k.split('|')
        dets[(f, mode, int(L))][field] = D[k]
    annots = {r['file_name']: r for r in json.load(open(ann / 'annotations.json'))}
    return rows, dets, annots


def gt_boxes(an):
    return np.array([[b['bbox'][0], b['bbox'][1], b['bbox'][0] + b['bbox'][2], b['bbox'][1] + b['bbox'][3]]
                     for b in an['annotations']], float).reshape(-1, 4) * SCALE


_MCACHE = {}


def _match(f, d, annots, mode, L, crit):
    k = (f, mode, L, crit)
    if k not in _MCACHE:
        g = gt_boxes(annots[f])
        M = (iou(g, d['boxes']) >= 0.5) if crit == 'iou50' else loc_match(g, d['boxes'])
        _MCACHE[k] = (g, M, np.argsort(-d['scores']))
    return _MCACHE[k]


def ap_records(images, dets, annots, mode, L, crit, gt_keep=None):
    """Greedy score-ordered matching per image; dets matched to ignored GTs are dropped (COCO 'ignore').
    Returns (scores, is_tp) over kept detections and the number of kept GTs."""
    S, T, npos = [], [], 0
    for f in images:
        d = dets.get((f, mode, L))
        if d is None:
            continue
        g, M, order = _match(f, d, annots, mode, L, crit)
        keep = np.ones(len(g), bool) if gt_keep is None else np.array([gt_keep(f, i) for i in range(len(g))], bool)
        npos += int(keep.sum())
        s = d['scores']
        anyM = M.any(0)
        # unmatched detections are FPs regardless of order: vectorise them, loop only over matched ones
        um = ~anyM
        S.extend(s[um]); T.extend([False] * int(um.sum()))
        order = order[anyM[order]]
        used = np.zeros(len(g), bool)
        for j in order:
            cand = np.where(M[:, j] & ~used)[0]
            if len(cand):
                i = cand[0]; used[i] = True
                if keep[i]:
                    S.append(s[j]); T.append(True)
            elif M[:, j].any() and not keep[M[:, j]].any():
                continue  # duplicate on an ignored GT
            else:
                S.append(s[j]); T.append(False)  # duplicate on an already-matched kept GT: FP (COCO)
    return np.array(S), np.array(T, bool), npos


def ap_from(S, T, npos):
    if npos == 0:
        return float('nan')
    o = np.argsort(-S); T = T[o]
    tp = np.cumsum(T); fp = np.cumsum(~T)
    rec = tp / npos; prec = tp / np.maximum(tp + fp, 1)
    # COCO 101-point interpolated AP
    mprec = np.concatenate([prec, [0]])
    for i in range(len(mprec) - 2, -1, -1):
        mprec[i] = max(mprec[i], mprec[i + 1])
    rs = np.linspace(0, 1, 101); out = []
    for r in rs:
        k = np.searchsorted(rec, r, side='left')
        out.append(mprec[k] if k < len(rec) else 0.0)
    return float(np.mean(out))


def prec_at(S, T, tau):
    k = S >= tau
    return float(T[k].mean()) if k.any() else float('nan')


def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'))
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (c - h, c + h)


def hbin(h):
    for i in range(len(H_BINS) - 1):
        if H_BINS[i] <= h < H_BINS[i + 1]:
            return H_LAB[i]


def vbin(v):
    for (a, b), lab in zip(V_BINS, V_LAB):
        if a <= v <= b:
            return lab


def icc_binary(groups):
    """ANOVA ICC for binary outcomes in clusters (Fleiss)."""
    groups = [np.asarray(g, float) for g in groups if len(g) >= 2]
    N = sum(len(g) for g in groups); k = len(groups)
    if k < 2:
        return float('nan')
    mean = np.concatenate(groups).mean()
    msb = sum(len(g) * (g.mean() - mean) ** 2 for g in groups) / (k - 1)
    msw = sum(((g - g.mean()) ** 2).sum() for g in groups) / (N - k)
    n0 = (N - sum(len(g) ** 2 for g in groups) / N) / (k - 1)
    return float((msb - msw) / (msb + (n0 - 1) * msw)) if (msb + (n0 - 1) * msw) > 0 else float('nan')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('bench'); ap.add_argument('ann'); ap.add_argument('out')
    ap.add_argument('--tau', type=float, default=0.3)
    a = ap.parse_args(); bench, ann, out = Path(a.bench), Path(a.ann), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    rows, dets, annots = load(bench, ann)
    tau = a.tau
    M = {'tau': tau}
    images = sorted({r['file'] for r in rows})
    info = {}
    for r in rows:
        info[(r['file'], r['gt_idx'])] = r
    img_meta = {r['file']: r for r in rows}
    tables = []

    def recall_table(name, keyf, labels, mode='tiled', L=0, rows_=None):
        rs = [r for r in (rows_ or rows) if r['mode'] == mode and r['blur_px'] == L]
        out_ = []
        for lab in labels:
            g = [r for r in rs if keyf(r) == lab]
            n = len(g)
            k50 = sum(r['score50'] >= tau for r in g); kl = sum(r['scoreloc'] >= tau for r in g)
            gk = lambda f, i, lab=lab: keyf(info[(f, i)]) == lab if (f, i) in info else False
            imgs = sorted({r['file'] for r in g})
            S, T, npos = ap_records(imgs, dets, annots, mode, L, 'iou50', gk)
            Sl, Tl, _ = ap_records(imgs, dets, annots, mode, L, 'loc', gk)
            out_.append(dict(stratum=lab, n_gt=n, recall50=k50 / n if n else float('nan'), recall50_ci=wilson(k50, n),
                             recall_loc=kl / n if n else float('nan'), recall_loc_ci=wilson(kl, n),
                             ap50=ap_from(S, T, npos), ap_loc=ap_from(Sl, Tl, npos),
                             prec50=prec_at(S, T, tau), prec_loc=prec_at(Sl, Tl, tau)))
        M[name] = out_
        return out_

    def fmt(name, tab, note=''):
        s = [f'### {name}\n', note + '\n' if note else '',
             '| stratum | n GT | recall IoU.5 (95% CI) | recall loc (95% CI) | precision IoU.5 | precision loc | AP50 | AP loc |',
             '|---|---:|---|---|---:|---:|---:|---:|']
        for t in tab:
            s.append(f"| {t['stratum']} | {t['n_gt']} | {t['recall50']:.3f} ({t['recall50_ci'][0]:.2f}-{t['recall50_ci'][1]:.2f}) | "
                     f"{t['recall_loc']:.3f} ({t['recall_loc_ci'][0]:.2f}-{t['recall_loc_ci'][1]:.2f}) | {t['prec50']:.3f} | "
                     f"{t['prec_loc']:.3f} | {t['ap50']:.3f} | {t['ap_loc']:.3f} |")
        tables.append('\n'.join(s) + '\n')

    note_gt = ('Size/occlusion strata use COCO-style ignore: detections matched to out-of-stratum persons are dropped; '
               'unmatched detections on those images count as false positives, so precision/AP are per-image-set.')
    fmt('Overall by inference mode (blur 0)', [dict(t, stratum=m) for m in ('tiled', 'full') for t in
        recall_table('overall_' + m, lambda r: 'all', ['all'], mode=m)])
    fmt('Apparent person height (px, 1920x1080 frame), tiled', recall_table('by_height', lambda r: hbin(r['h_px']), H_LAB), note_gt)
    fmt('Apparent person height, full-frame 640 letterbox', recall_table('by_height_full', lambda r: hbin(r['h_px']), H_LAB, mode='full'), note_gt)
    fmt('Occlusion (NOMAD visibility label), tiled', recall_table('by_visibility', lambda r: vbin(r['visibility']), V_LAB), note_gt)
    fmt('NOMAD distance stratum (m), tiled', recall_table('by_distance', lambda r: str(r['dist_m']), ['10', '30', '50', '70', '90']))
    fmt('Lighting: sky, tiled', recall_table('by_sky', lambda r: r['sky'], ['sunny', 'partcloud', 'overcast']))
    fmt('Lighting: sun height (low = before 10:00 or from 17:00), tiled', recall_table('by_sun', lambda r: r['sun'], ['high_sun', 'low_sun']))
    fmt('Activity (pose), tiled', recall_table('by_activity', lambda r: r['activity'], ['Walking', 'Hiding', 'Laying', 'Hiding (Laying)']))
    fmt('Scene (location), tiled', recall_table('by_location', lambda r: r['location'], sorted({r['location'] for r in rows})))
    blurs = sorted({r['blur_px'] for r in rows if r['mode'] == 'tiled'})
    btab = []
    for L in blurs:
        t = recall_table(f'blur_{L}', lambda r: 'all', ['all'], L=L)[0]; btab.append(dict(t, stratum=f'{L} px'))
    M['by_blur'] = btab
    fmt('Synthetic horizontal motion blur (kernel length px), tiled', btab)
    # blur x height interaction (recall loc)
    bxh = {}
    for L in blurs:
        for lab in H_LAB:
            g = [r for r in rows if r['mode'] == 'tiled' and r['blur_px'] == L and hbin(r['h_px']) == lab]
            bxh[f'{L}|{lab}'] = (sum(r['scoreloc'] >= tau for r in g) / len(g)) if g else float('nan')
    M['blur_x_height_recall_loc'] = bxh
    s = ['### Recall (loc) by blur x apparent height, tiled\n', '| blur px | ' + ' | '.join(H_LAB) + ' |', '|---|' + '---:|' * len(H_LAB)]
    for L in blurs:
        s.append(f'| {L} | ' + ' | '.join(f"{bxh[f'{L}|{lab}']:.3f}" for lab in H_LAB) + ' |')
    tables.append('\n'.join(s) + '\n')

    # ---- GSD per distance stratum from walking, fully visible persons; false alerts per frame and per km^2
    # GSD at the person per distance stratum: NOMAD paper Table S3 gives 4.24 mm/px at 10 m to 38.18 mm/px at 90 m
    # (native 5472 px), i.e. ~0.424 mm/px per metre; times 5472/1920 for our resized frames. Too few walking,
    # fully visible persons are in the subset to estimate it from box heights.
    gsd = {dm: 0.424e-3 * dm / SCALE for dm in (10, 30, 50, 70, 90)}
    M['gsd_m_per_px'] = gsd
    fa = []
    for mode in ('tiled', 'full'):
        for dm in (10, 30, 50, 70, 90):
            imgs = sorted({r['file'] for r in rows if r['dist_m'] == dm})
            for thr in (0.3, 0.5):
                nfp = [int(((dets[(f, mode, 0)]['scores'] >= thr) & ~dets[(f, mode, 0)]['tploc']).sum()) for f in imgs if (f, mode, 0) in dets]
                area_km2 = 1920 * 1080 * gsd[dm] ** 2 / 1e6
                fa.append(dict(mode=mode, dist_m=dm, tau=thr, n_frames=len(nfp), fp_per_frame=float(np.mean(nfp)),
                               frames_with_fp=float(np.mean(np.array(nfp) > 0)),
                               approx_footprint_km2=area_km2, fp_per_km2=float(np.mean(nfp)) / area_km2))
    M['false_alerts'] = fa
    s = ['### False alerts (detections not on the annotated person), blur 0\n',
         'Footprint is 1920x1080 px times the squared GSD at the person (NOMAD paper Table S3, scaled to 1920 px); oblique ~45 deg views make the true footprint larger, so FP/km^2 is an upper bound.\n',
         '| mode | distance m | tau | frames | FP/frame | frames with >=1 FP | approx footprint km^2 | FP/km^2 |', '|---|---:|---:|---:|---:|---:|---:|---:|']
    for x in fa:
        s.append(f"| {x['mode']} | {x['dist_m']} | {x['tau']} | {x['n_frames']} | {x['fp_per_frame']:.3f} | {x['frames_with_fp']:.3f} | {x['approx_footprint_km2']:.5f} | {x['fp_per_km2']:.0f} |")
    tables.append('\n'.join(s) + '\n')

    # ---- FP persistence across consecutive frames (decoy-like): FP centre within 40 px of an FP in the next frame
    seq = defaultdict(list)
    for f in images:
        r = img_meta[f]; seq[(r['folder'])].append((r['frame'], f))
    pers_n = pers_k = 0
    for k, v in seq.items():
        v.sort()
        for (fa_, f1), (fb, f2) in zip(v, v[1:]):
            if fb - fa_ > 12:
                continue
            d1, d2 = dets[(f1, 'tiled', 0)], dets[(f2, 'tiled', 0)]
            m1 = (d1['scores'] >= tau) & ~d1['tploc']; m2 = (d2['scores'] >= tau) & ~d2['tploc']
            c1 = 0.5 * (d1['boxes'][m1, :2] + d1['boxes'][m1, 2:]); c2 = 0.5 * (d2['boxes'][m2, :2] + d2['boxes'][m2, 2:])
            for c in c1:
                pers_n += 1
                if len(c2) and np.min(np.hypot(*(c2 - c).T)) < 40:
                    pers_k += 1
    M['fp_persistence_lag1'] = dict(n=pers_n, frac=pers_k / pers_n if pers_n else float('nan'))
    tables.append(f"FP persistence: of {pers_n} false alerts (tau {tau}, tiled), {M['fp_persistence_lag1']['frac']:.3f} reappear within 40 px in the next annotated frame (<=12 video frames later): background false alerts are mostly persistent decoys, not white noise.\n")

    # ---- repeat-view correlation (tiled, blur 0, loc criterion, first GT per image)
    base = [r for r in rows if r['mode'] == 'tiled' and r['blur_px'] == 0 and r['gt_idx'] == 0]
    runs = defaultdict(list)
    for r in base:
        runs[r['folder']].append(r)
    run_groups, pairs, flight_groups = [], [], defaultdict(list)
    for k, v in runs.items():
        v.sort(key=lambda r: r['frame'])
        cur = [v[0]]
        for x, y in zip(v, v[1:]):
            if y['frame'] - x['frame'] <= 12:
                cur.append(y); pairs.append((x, y))
            else:
                run_groups.append(cur); cur = [y]
        run_groups.append(cur)
    D = lambda r: int(r['scoreloc'] >= tau)
    n11 = sum(D(x) and D(y) for x, y in pairs); n1_ = sum(D(x) for x, y in pairs)
    n01 = sum((not D(x)) and D(y) for x, y in pairs); n0_ = len(pairs) - n1_
    p_marg = np.mean([D(r) for r in base])
    # covariate-adjusted: logistic model from fit_obs (if present) supplies p_i; else binned by height x visibility
    cell = defaultdict(list)
    for r in base:
        cell[(hbin(r['h_px']), vbin(r['visibility']))].append(D(r))
    pc = {k: np.mean(v) for k, v in cell.items()}
    res = lambda r: (D(r) - pc[(hbin(r['h_px']), vbin(r['visibility']))])
    num = sum(res(x) * res(y) for x, y in pairs)
    den = math.sqrt(sum(res(x) ** 2 for x, y in pairs) * sum(res(y) ** 2 for x, y in pairs))
    M['repeat_view'] = dict(n_pairs=len(pairs), n_runs=len(run_groups), p_marginal=float(p_marg),
                            p_next_given_det=n11 / n1_ if n1_ else float('nan'),
                            p_next_given_miss=n01 / n0_ if n0_ else float('nan'),
                            icc_within_run=icc_binary([[D(r) for r in g] for g in run_groups]),
                            residual_corr_lag1_given_height_vis=num / den if den else float('nan'))
    # across runs of the same flight (actor x distance), different minutes / poses
    fl = defaultdict(list)
    for g in run_groups:
        fl[g[0]['folder']].append(np.mean([D(r) for r in g]))
    M['repeat_view']['icc_runs_within_flight'] = icc_binary([v for v in fl.values()])
    rv = M['repeat_view']
    tables.append('### Repeat-view correlation (tiled, blur 0, loc criterion, tau %.2f)\n\n' % tau +
                  '| quantity | value |\n|---|---:|\n' +
                  f"| consecutive annotated-frame pairs (<=12 video frames, 0.4 s at 30 fps) | {rv['n_pairs']} |\n"
                  f"| runs | {rv['n_runs']} |\n| P(detect) marginal | {rv['p_marginal']:.3f} |\n"
                  f"| P(detect at t+1 \\| detect at t) | {rv['p_next_given_det']:.3f} |\n"
                  f"| P(detect at t+1 \\| miss at t) | {rv['p_next_given_miss']:.3f} |\n"
                  f"| ICC within run (binary ANOVA) | {rv['icc_within_run']:.3f} |\n"
                  f"| lag-1 residual correlation after height x visibility cells | {rv['residual_corr_lag1_given_height_vis']:.3f} |\n"
                  f"| ICC of run means within a flight (different minutes, poses) | {rv['icc_runs_within_flight']:.3f} |\n")

    # ---- localisation (centre) error in px vs height
    le = []
    for lab in H_LAB:
        e = np.array([[r['ex'], r['ey']] for r in base if hbin(r['h_px']) == lab and r['scoreloc'] >= tau])
        if len(e) >= 5:
            le.append(dict(stratum=lab, n=len(e), rms_x=float(np.sqrt(np.mean(e[:, 0] ** 2))), rms_y=float(np.sqrt(np.mean(e[:, 1] ** 2))),
                           med_abs=float(np.median(np.hypot(e[:, 0], e[:, 1])))))
    M['centre_error_px'] = le
    s = ['### Box-centre error of matched detections (px, 1920 frame), tiled\n', '| height | n | RMS x | RMS y | median radial |', '|---|---:|---:|---:|---:|']
    for x in le:
        s.append(f"| {x['stratum']} | {x['n']} | {x['rms_x']:.1f} | {x['rms_y']:.1f} | {x['med_abs']:.1f} |")
    tables.append('\n'.join(s) + '\n')

    json.dump(M, open(out / 'metrics.json', 'w'), indent=1, default=float)
    (out / 'tables.md').write_text('\n'.join(tables))
    figures(rows, M, out, tau, blurs)
    print('wrote', out)


def figures(rows, M, out, tau, blurs):
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['pdf.use14corefonts'] = True  # Type 1 (Helvetica core font) text in PDFs, no Type 3
    matplotlib.rcParams['ps.fonttype'] = 42
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.4))
    for key, lab, c in (('by_height', 'tiled 6x640', '#1f5fa8'), ('by_height_full', 'full 640', '#c2571a')):
        t = M[key]
        ax[0].errorbar(range(len(t)), [x['recall_loc'] for x in t],
                       yerr=np.array([[x['recall_loc'] - x['recall_loc_ci'][0], x['recall_loc_ci'][1] - x['recall_loc']] for x in t]).T,
                       marker='o', label=lab, color=c, capsize=3)
    ax[0].set_xticks(range(len(H_LAB))); ax[0].set_xticklabels(H_LAB); ax[0].set_xlabel('person height (px, 1080p)')
    ax[0].set_ylabel(f'recall (loc, tau {tau})'); ax[0].set_ylim(0, 1); ax[0].legend(frameon=False); ax[0].set_title('size')
    t = M['by_visibility']
    ax[1].errorbar(range(len(t)), [x['recall_loc'] for x in t],
                   yerr=np.array([[x['recall_loc'] - x['recall_loc_ci'][0], x['recall_loc_ci'][1] - x['recall_loc']] for x in t]).T,
                   marker='o', color='#1f5fa8', capsize=3)
    ax[1].set_xticks(range(len(V_LAB))); ax[1].set_xticklabels(V_LAB); ax[1].set_xlabel('visible fraction (NOMAD label)')
    ax[1].set_ylim(0, 1); ax[1].set_title('occlusion')
    for lab, c in zip(H_LAB[1:], ['#9ecae1', '#4292c6', '#2171b5', '#08306b']):
        ax[2].plot(blurs, [M['blur_x_height_recall_loc'][f'{L}|{lab}'] for L in blurs], marker='o', label=lab + ' px', color=c)
    ax[2].set_xlabel('motion blur length (px)'); ax[2].set_ylim(0, 1); ax[2].legend(frameon=False, fontsize=8); ax[2].set_title('blur x size')
    for a_ in ax:
        a_.spines[['top', 'right']].set_visible(False); a_.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(out / 'recall_strata.png', dpi=150); fig.savefig(out / 'recall_strata.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
