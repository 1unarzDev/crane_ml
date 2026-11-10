"""Crude model-free sluggishness rule vs fitted delay / joint (M3 + M4 stored boats). Pure numpy, one process.

    cd /home/lunarz/worktrees/decisioncal && nice -n 19 python3 -I -W ignore Tools/DecisionCal/analysis/crude_rule.py
Features are cached in /tmp/dcr/feats.json (regenerated probe logs: ~1 s per boat). Creates no other files.
"""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.dont_write_bytecode = True
sys.path.insert(0, str(HERE))
import numpy as np  # noqa: E402

import hidden  # noqa: E402
import probes  # noqa: E402
from baseline_selector import FEATURES, extract_features  # noqa: E402
from study import _seed  # noqa: E402

CACHE = Path.home() / '.cache/decisioncal'
RUNS = {'m3': (CACHE / 'm3-20261113/plants.jsonl', 20261113), 'm4': (CACHE / 'm4-20261114/plants.jsonl', 20261114)}
CAP = 60.0
LIBS = ('v2', 'v3')
LV3 = {'rigid': 0.0, 'rigid@d0.16': .16, 'rigid@d0.25': .25, 'rigid@d0.35': .35, 'rigid@d0.47': .47, 'rigid@d0.62': .62}
OUT = []


def P(*a):
    s = ' '.join(str(x) for x in a)
    print(s)
    OUT.append(s)


def load(name):
    return [json.loads(l) for l in RUNS[name][0].read_text().splitlines()]


def feats_for(name, rows, nverify=5):
    fp = Path('/tmp/dcr/feats.json')
    cache = json.loads(fp.read_text()) if fp.exists() else {}
    seed = RUNS[name][1]
    nbad = 0
    cnt = {}
    for r in rows:
        key = f'{name}/{r["id"]}'
        st, i = r['stratum'], int(r['id'].split('-')[1])
        spec = hidden.sample_hidden(len({q['id'] for q in rows if q['stratum'] == st}), seed, st).take([i])
        if key not in cache:
            ev, _ = probes.generate(spec, _seed(seed, r['id'], 'probe'))
            cache[key] = {'f': extract_features(ev).tolist(), 'sha': ev.provenance['sha256']}
        nbad += cache[key]['sha'] != r['budget']['evidence_sha']
    fp.write_text(json.dumps(cache))
    P(f'[verify] {name}: {len(rows)} regenerated logs, evidence_sha mismatches = {nbad}')
    return np.array([cache[f'{name}/{r["id"]}']['f'] for r in rows])


def build(rows, F):
    """Dict of arrays."""
    fi = {k: i for i, k in enumerate(FEATURES)}
    f = lambda k: F[:, fi[k]]
    d = {'stratum': np.array([r['stratum'] for r in rows]), 'idx': np.array([int(r['id'].split('-')[1]) for r in rows]),
         'true_delay': np.array([r['spec']['dead_time'] + r['spec']['odom_latency'] for r in rows]),
         'tau_up': np.array([r['spec']['tau_up'] for r in rows]),
         'fit_delay': np.array([r['fits']['rigid+delay']['map']['delay_s'] for r in rows])}
    d['S1'], d['S2'] = f('delay_yaw'), f('delay_yaw') + f('rise_yaw') / 2
    d['S3'], d['S4'] = f('delay_surge') + f('rise_surge') / 2, f('rev_surge_t')
    d['S5'] = f('rev_yaw_t')
    return d


def impute(d_list):
    """NaN -> median of the finite values pooled over the M3 boats (fixed constants, no M4 information)."""
    return None


def arr(rows, arm, key):
    out = {}
    for lib in LIBS:
        if key == 'regret':
            out[lib] = np.array([min(r['libs'][lib]['scores']['arms'][arm]['regret'], CAP) for r in rows])
        else:
            out[lib] = np.array([r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][arm]['sel']] for r in rows])
    return out


def mats(rows, arms):
    """regret[lib] [n, A], dock[lib] [n, A]."""
    R = {lib: np.stack([arr(rows, a, 'regret')[lib] for a in arms], 1) for lib in LIBS}
    D = {lib: np.stack([arr(rows, a, 'dock')[lib] for a in arms], 1) for lib in LIBS}
    return R, D


def corr(x, y):
    m = np.isfinite(x) & np.isfinite(y)
    return float(np.corrcoef(x[m], y[m])[0, 1]) if m.sum() > 3 and x[m].std() > 0 and y[m].std() > 0 else float('nan')


def choose(S, ths):
    """level index 0..len(ths) from sorted thresholds."""
    return np.searchsorted(np.asarray(ths), S, side='right')


def fit_two(S, Rlist, a_hi, a_lo=0):
    """theta minimising mean (over libs) regret of 'arm a_lo if S<theta else a_hi'. Rlist: list of [n, A] per lib."""
    u = np.unique(S)
    cands = np.concatenate([[u[0] - 1e-6], (u[:-1] + u[1:]) / 2, [u[-1] + 1e-6]])
    best = None
    for th in cands:
        pick = np.where(S < th, a_lo, a_hi)
        v = np.mean([R[np.arange(len(S)), pick].mean() for R in Rlist])
        if best is None or v < best[0] - 1e-12:
            best = (v, th)
    return best[1], best[0]


def fit_multi(S, Rlist, th1=None, grid=None):
    """Arms indexed 0..3 = levels (0,.35,.47,.62). Search th2<=th3 (and th1 if None) by exhaustive grid."""
    grid = np.quantile(S, np.linspace(0, 1, 41)) if grid is None else grid
    grid = np.unique(grid)
    best = None
    for a in (grid if th1 is None else [th1]):
        for b in grid[grid >= a]:
            for c in grid[grid >= b]:
                pick = choose(S, (a, b, c))
                v = np.mean([R[np.arange(len(S)), pick].mean() for R in Rlist])
                if best is None or v < best[0] - 1e-12:
                    best = (v, (a, b, c))
    return best[1], best[0]


def boot_ci(d, rng, nb=4000):
    idx = rng.integers(0, len(d), (nb, len(d)))
    m = d[idx].mean(1)
    return float(d.mean()), float(np.quantile(m, .05)), float(np.quantile(m, .95))


def main():
    rows3, rows4 = load('m3'), load('m4')
    Path('/tmp/dcr').mkdir(exist_ok=True)
    F3, F4 = feats_for('m3', rows3), feats_for('m4', rows4)
    d3, d4 = build(rows3, F3), build(rows4, F4)
    rng = np.random.default_rng(0)

    # verification 2: refit on 5 regenerated logs reproduces stored rigid+delay MAP (only if evidence_sha not enough: done above)
    # NaN handling
    P('\n## NaN counts (M3 / M4) per candidate')
    for k in ('S1', 'S2', 'S3', 'S4', 'S5'):
        P(f'  {k}: {np.isnan(d3[k]).sum()}/{len(rows3)}  {np.isnan(d4[k]).sum()}/{len(rows4)}')
    # impute NaN (no response detected = very sluggish or unmeasurable) with the M3 max finite value
    for k in ('S1', 'S2', 'S3', 'S4', 'S5'):
        fill = np.nanmax(d3[k])
        for d in (d3, d4):
            d[k] = np.where(np.isfinite(d[k]), d[k], fill)
        P(f'  {k} NaN fill (M3 max finite): {fill:.2f}')

    # ---------------- 1. correlations
    P('\n## 1. Correlation of crude sluggishness candidates (Pearson r; Spearman in brackets)')
    def sp(x, y):
        return corr(np.argsort(np.argsort(x)).astype(float), np.argsort(np.argsort(y)).astype(float))
    for nm, d in (('M3', d3), ('M4', d4)):
        for st in ['all'] + sorted(set(d['stratum'])):
            m = np.ones(len(d['idx']), bool) if st == 'all' else d['stratum'] == st
            P(f'{nm} {st:9s} n={m.sum():3d}  ' + ' | '.join(
                f'{k}: true_delay {corr(d[k][m], d["true_delay"][m]):+.2f}, tau_up {corr(d[k][m], d["tau_up"][m]):+.2f}, fit {corr(d[k][m], d["fit_delay"][m]):+.2f}'
                for k in ('S1', 'S2', 'S3', 'S4')))
    P('Pooled M3+M4, Spearman with fitted delay / true delay / tau_up:')
    for k in ('S1', 'S2', 'S3', 'S4', 'S5'):
        x = np.concatenate([d3[k], d4[k]])
        P(f'  {k}: fit {sp(x, np.concatenate([d3["fit_delay"], d4["fit_delay"]])):+.2f}  true {sp(x, np.concatenate([d3["true_delay"], d4["true_delay"]])):+.2f}'
          f'  tau_up {sp(x, np.concatenate([d3["tau_up"], d4["tau_up"]])):+.2f}')
    P('Medians by M4 stratum: ' + '; '.join(f'{st}: ' + ' '.join(f'{k}={np.median(d4[k][d4["stratum"] == st]):.2f}' for k in ('S1', 'S2', 'S3', 'S4'))
                                         for st in sorted(set(d4['stratum']))))
    P('Medians by M3 stratum: ' + '; '.join(f'{st}: ' + ' '.join(f'{k}={np.median(d3[k][d3["stratum"] == st]):.2f}' for k in ('S1', 'S2', 'S3', 'S4'))
                                         for st in sorted(set(d3['stratum']))))

    # ---------------- arms
    A3 = ['rigid', 'rigid@d0.16', 'rigid@d0.25', 'rigid@d0.35']
    R3, D3 = mats(rows3, A3)
    LV4 = ['rigid', 'rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62']
    ALL4 = LV4 + ['rigid+delay', 'joint']
    R4, D4 = mats(rows4, ALL4)

    # ---------------- 2. first threshold from M3 only; pick S by M3 criterion
    P('\n## 2. Threshold on M3 only ("rigid if S<theta else assumed 0.35"), both libs pooled, 80 boats (IN-SAMPLE on M3)')
    base3 = np.mean([R3[l][:, 0].mean() for l in LIBS]); all35 = np.mean([R3[l][:, 3].mean() for l in LIBS])
    orc3 = np.mean([R3[l].min(1).mean() for l in LIBS])
    P(f'  M3 mean capped regret: rigid {base3:.1f}, assumed0.35 {all35:.1f}, oracle-of-4-levels {orc3:.1f}')
    th3 = {}
    for k in ('S1', 'S2', 'S3', 'S4', 'S5'):
        th, v = fit_two(d3[k], [R3[l] for l in LIBS], 3)
        th3[k] = (th, v)
        P(f'  {k}: theta={th:.3f}  M3 in-sample mean regret {v:.1f}')
    # M3 fitted-delay reference
    Rf3 = {l: np.array([min(r['libs'][l]['scores']['arms']['rigid+delay']['regret'], CAP) for r in rows3]) for l in LIBS}
    P(f'  M3 fitted delay (rigid+delay) mean capped regret: {np.mean([Rf3[l].mean() for l in LIBS]):.1f}')
    Sbest = min(th3, key=lambda k: th3[k][1])
    P(f'  Primary score chosen by M3 in-sample fit: {Sbest} (theta {th3[Sbest][0]:.3f})')

    # ---------------- 3. M4 evaluation
    def evalrule(pick, R, D, mask=None):
        n = len(pick)
        mask = np.ones(n, bool) if mask is None else mask
        return {l: (R[l][np.arange(n), pick][mask], D[l][np.arange(n), pick][mask]) for l in LIBS}

    def cell(x):
        r, dk = x
        return f'{r.mean():5.1f}/{np.median(r):4.1f}/{dk.mean():.2f}'

    strata = ['timing', 'transient', 'steady']
    n4 = len(rows4)
    odd = (d4['idx'] % 2 == 1)
    S = d4[Sbest]
    out_rules = {}
    for Sk in sorted({Sbest, 'S1', 'S2', 'S3', 'S4'}, key=lambda s: s != Sbest):
        S = d4[Sk]
        th1 = th3[Sk][0]
        # two-level, M3 theta, arm 0.47 (idx2) : fully OOS
        two = np.where(S < th1, 0, 2)
        # multi-level: cross-fit halves for th2/th3 (th1 from M3 fixed)
        multi = np.zeros(n4, int)
        ths_f = {}
        for fit_mask, ev_mask, tag in ((odd, ~odd, 'odd->even'), (~odd, odd, 'even->odd')):
            ths, v = fit_multi(S[fit_mask], [R4[l][fit_mask] for l in LIBS], th1=th1)
            multi[ev_mask] = choose(S[ev_mask], ths)
            ths_f[tag] = (ths, v)
        # in-sample multi, all boats, th1 free
        ths_is, v_is = fit_multi(S, [R4[l] for l in LIBS])
        multi_is = choose(S, ths_is)
        out_rules[Sk] = dict(two=two, multi=multi, multi_is=multi_is, ths_f=ths_f, ths_is=ths_is, th1=th1)

    # fixed-arm reference picks
    def pick_arm(j):
        return np.full(n4, j)
    singles = {a: pick_arm(j) for j, a in enumerate(ALL4)}
    # best single assumed level (in-sample) among 4 levels, pooled libs
    bl = int(np.argmin([np.mean([R4[l][:, j].mean() for l in LIBS]) for j in range(4)]))
    orc_pick = {l: np.argmin(R4[l][:, :4], 1) for l in LIBS}

    def table(title, mask_fn, picks):
        P(f'\n{title}  (cells: mean capped regret s / median s / dock rate of pick)')
        hdr = f'{"rule":42s}' + ''.join(f'{lib+" "+st:>24s}' for st in strata + ['pooled'] for lib in LIBS)
        P(hdr)
        for name, pk in picks:
            line = f'{name:42s}'
            for st in strata + ['pooled']:
                m = np.ones(n4, bool) if st == 'pooled' else d4['stratum'] == st
                m = m & mask_fn
                for lib in LIBS:
                    p = pk[lib] if isinstance(pk, dict) else pk
                    line += f'{cell((R4[lib][np.arange(n4), p][m], D4[lib][np.arange(n4), p][m])):>24s}'
            P(line)

    allm = np.ones(n4, bool)
    ALLp = lambda pk: pk
    pk_list = [('rigid (0 s)', singles['rigid']), (f'best single level in-sample ({LV4[bl]}) [IN-SAMPLE]', singles[LV4[bl]]),
               ('assumed 0.47 blanket', singles['rigid@d0.47']), ('fitted delay (rigid+delay)', singles['rigid+delay']),
               ('joint', singles['joint']),
               (f'crude 2-level {Sbest}, theta(M3) -> 0/0.47 [OOS]', out_rules[Sbest]['two']),
               (f'crude multi-level {Sbest}, th1 M3, th2/3 cross-fit [OOS x-fit]', out_rules[Sbest]['multi']),
               (f'crude multi-level {Sbest}, all thr fit on M4 [IN-SAMPLE]', out_rules[Sbest]['multi_is']),
               ('oracle level (best of 0/.35/.47/.62) [UPPER BOUND]', orc_pick)]
    table(f'### 3a. M4 all 160 boats; primary score {Sbest}', allm, pk_list)
    P(f'  th1 (M3)={out_rules[Sbest]["th1"]:.3f}; cross-fit (th1,th2,th3): ' + '; '.join(
        f'{t}: ' + ','.join(f'{x:.2f}' for x in v[0]) for t, v in out_rules[Sbest]['ths_f'].items()) +
      '; in-sample all: ' + ','.join(f'{x:.2f}' for x in out_rules[Sbest]['ths_is']))
    table('### 3b. M4 even-index boats only (80; multi-level thresholds fit on odd boats; strictly OOS for the cross-fit rule)', ~odd, pk_list)

    P('\n### 3c. Other candidate scores, M4 pooled 160 (mean capped regret v2 / v3; dock v2/v3)')
    for Sk in ('S1', 'S2', 'S3', 'S4'):
        o = out_rules[Sk]
        for nm, pk in (('2-level theta(M3) [OOS]', o['two']), ('multi x-fit [OOS x-fit]', o['multi']), ('multi in-sample', o['multi_is'])):
            v = [R4[l][np.arange(n4), pk].mean() for l in LIBS]
            dk = [D4[l][np.arange(n4), pk].mean() for l in LIBS]
            P(f'  {Sk} {nm:26s} {v[0]:5.1f} / {v[1]:5.1f}   dock {dk[0]:.2f}/{dk[1]:.2f}   theta1={o["th1"]:.2f}')
    P(f'  reference: fitted delay {R4["v2"][:, 4].mean():.1f}/{R4["v3"][:, 4].mean():.1f}; joint {R4["v2"][:, 5].mean():.1f}/{R4["v3"][:, 5].mean():.1f}; rigid {R4["v2"][:, 0].mean():.1f}/{R4["v3"][:, 0].mean():.1f}')

    # paired differences
    P('\n### 3d. Paired differences in capped regret, crude - X (negative = crude better), bootstrap 90% CI, 160 boats')
    for nm, pk in (('2-level theta(M3) [OOS]', out_rules[Sbest]['two']), ('multi x-fit [OOS x-fit]', out_rules[Sbest]['multi'])):
        for ref, j in (('fitted delay', 4), ('joint', 5), ('assumed 0.47', 2)):
            for st in strata + ['pooled']:
                m = np.ones(n4, bool) if st == 'pooled' else d4['stratum'] == st
                line = f'  {nm:26s} - {ref:13s} {st:9s}'
                for lib in LIBS:
                    dd = (R4[lib][np.arange(n4), pk] - R4[lib][:, j])[m]
                    mu, lo, hi = boot_ci(dd, rng)
                    line += f'  {lib}: {mu:+5.1f} [{lo:+5.1f},{hi:+5.1f}]'
                P(line)

    # ---------------- 4. robustness
    P('\n## 4. Robustness (primary score ' + Sbest + ')')
    th1 = out_rules[Sbest]['th1']
    S = d4[Sbest]
    P(f'Noise: S*(1+N(0,0.3)) on M4, theta(M3) fixed, 2-level rule; 200 reps. Mean capped regret (pooled 160) mean [5-95% over reps]; multi-level uses cross-fit thresholds fit on noiseless scores')
    ms = out_rules[Sbest]['ths_f']
    for lib in LIBS:
        v2l, vml = [], []
        for _ in range(200):
            Sn = S * (1 + rng.normal(0, .3, n4))
            v2l.append(R4[lib][np.arange(n4), np.where(Sn < th1, 0, 2)].mean())
            pk = np.zeros(n4, int)
            pk[~odd] = choose(Sn[~odd], ms['odd->even'][0])
            pk[odd] = choose(Sn[odd], ms['even->odd'][0])
            vml.append(R4[lib][np.arange(n4), pk].mean())
        q = lambda v: f'{np.mean(v):.1f} [{np.quantile(v, .05):.1f},{np.quantile(v, .95):.1f}]'
        P(f'  {lib}: 2-level noisy {q(v2l)}  (noiseless {R4[lib][np.arange(n4), out_rules[Sbest]["two"]].mean():.1f});  multi noisy {q(vml)} (noiseless {R4[lib][np.arange(n4), out_rules[Sbest]["multi"]].mean():.1f});'
          f'  fitted {R4[lib][:, 4].mean():.1f}, joint {R4[lib][:, 5].mean():.1f}')
    P('Sensitivity to theta (2-level 0/0.47 on M4 pooled; theta as multiple of theta(M3); this uses M4 outcomes, descriptive):')
    P('  mult   theta   frac->0.47   v2 regret   v3 regret   dock v2/v3')
    for mult in (0.5, 0.7, 0.85, 1.0, 1.15, 1.3, 1.5, 2.0, 3.0):
        th = th1 * mult
        pk = np.where(S < th, 0, 2)
        P(f'  {mult:4.2f}  {th:6.3f}   {np.mean(pk == 2):.2f}        {R4["v2"][np.arange(n4), pk].mean():6.1f}      {R4["v3"][np.arange(n4), pk].mean():6.1f}     '
          f'{D4["v2"][np.arange(n4), pk].mean():.2f}/{D4["v3"][np.arange(n4), pk].mean():.2f}')
    # theta fitted on M3 per library separately
    for lib in LIBS:
        th, v = fit_two(d3[Sbest], [R3[lib]], 3)
        pk = np.where(S < th, 0, 2)
        P(f'  theta fit on M3 {lib} only: {th:.3f} -> M4 {lib} regret {R4[lib][np.arange(n4), pk].mean():.1f}')
    # M4-only best 2-level threshold (in-sample), for context
    th_is, v_is = fit_two(S, [R4[l] for l in LIBS], 2)
    pk = np.where(S < th_is, 0, 2)
    P(f'  [IN-SAMPLE] best M4 2-level theta {th_is:.3f}: regret v2 {R4["v2"][np.arange(n4), pk].mean():.1f}, v3 {R4["v3"][np.arange(n4), pk].mean():.1f}')
    # crude score vs fitted delay as score (does the fitted delay as thresholded score do the same?)
    thf, _ = fit_two(d3['fit_delay'], [R3[l] for l in LIBS], 3)
    pk = np.where(d4['fit_delay'] < thf, 0, 2)
    P(f'  Fitted-delay MAP as the score, theta fit on M3 ({thf:.3f}), 2-level 0/0.47 [OOS]: v2 {R4["v2"][np.arange(n4), pk].mean():.1f}, v3 {R4["v3"][np.arange(n4), pk].mean():.1f}')
    # per-boat agreement of oracle level
    P('Oracle level distribution in M4 (v2 / v3; fraction of boats whose best level is 0, .35, .47, .62):')
    for lib in LIBS:
        P('  ' + lib + ': ' + ' '.join(f'{np.mean(orc_pick[lib] == j):.2f}' for j in range(4)))
    Path('/tmp/dcr/out.txt').write_text('\n'.join(OUT))


if __name__ == '__main__':
    main()
