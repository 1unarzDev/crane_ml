"""M5 report: excess-delay figure, M2 channel x regime map, tables. Hand-written SVG (no matplotlib).

    nice -n 10 python3 -I report_m5.py --m5 DIR --m4 DIR --m2 DIR --out OUT

Reads plants.jsonl (+ summary.json for M5) of each run. Writes fig_excess.{svg,png}, fig_map.{svg,png},
tables_m5.md. No simulation. Reuses the SVG primitives of report_m1.py. Statistics are recomputed here
independently of study_m5.analyse and compared with summary.json (mismatches are printed as WARNING).
Everything uses the cap of 60 s on regret, as the pre-registration does.
"""

import argparse
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from report_m1 import C, Svg, Axes, boot_ci, md_table, nice_ticks, to_png  # noqa: E402

CAP, MARGIN, ALPHA, WINDOW = 60.0, 3.0, 0.10, 2.0
EXCESS = (-0.10, -0.05, 0.0, 0.05, 0.10, 0.20)
LIBS = ('v2', 'v3')
LIB_TITLE = {'v2': 'library v2 (primary)', 'v3': 'library v3 (random draws)'}
REFS = ('rigid', 'rigid+delay', 'joint')
BODY = {'fit': (C['blue'], 'circle', None, 'fitted body, delay L+e (fit:e)'),
        'true': (C['black'], 'square', '6 4', 'true body, delay L+e (true:e)')}
REF_STYLE = {'rigid': (C['sky'], 'diamond'), 'rigid+delay': (C['verm'], 'tridown'), 'joint': (C['pink'], 'plus')}
M4_ARMS = ('rigid', 'rigid@d0.35', 'rigid@d0.47', 'rigid@d0.62', 'rigid+delay', 'joint')
M4_STYLE = {'rigid': (C['sky'], 'diamond'), 'rigid@d0.35': (C['orange'], 'tri'), 'rigid@d0.47': (C['green'], 'square'),
            'rigid@d0.62': (C['black'], 'cross'), 'rigid+delay': (C['verm'], 'tridown'), 'joint': (C['pink'], 'plus')}
M4_LABEL = {'rigid': 'rigid (no delay)', 'rigid@d0.35': 'assumed 0.35 s', 'rigid@d0.47': 'assumed 0.47 s',
            'rigid@d0.62': 'assumed 0.62 s', 'rigid+delay': 'fitted delay', 'joint': 'joint'}
M2_ARMS = ('rigid', 'rigid+delay', 'rigid+lag', 'joint')
M2_STYLE = {'rigid': (C['sky'], 'diamond'), 'rigid+delay': (C['verm'], 'tridown'), 'rigid+lag': (C['green'], 'cross'),
            'joint': (C['pink'], 'plus')}
M2_DASH = {'rigid': None, 'rigid+delay': '5 3', 'rigid+lag': '2 3', 'joint': '8 3 2 3'}
ekey = lambda b, e: f'{b}:{e:+.2f}'


# ------------------------------------------------------------------------------------------ data
def load(d):
    d = Path(d).expanduser()
    rows = [json.loads(l) for l in (d / 'plants.jsonl').read_text().splitlines() if l.strip()]
    rows.sort(key=lambda r: r['id'])      # file order is completion order; the study analyses in id order
    summ = json.loads((d / 'summary.json').read_text()) if (d / 'summary.json').exists() else None
    return rows, summ


def reg(r, lib, arm):
    return min(r['libs'][lib]['scores']['arms'][arm]['regret'], CAP)


def dock(r, lib, arm):
    return r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][arm]['sel']]


def mat(rows, lib, arms, f=reg):
    return np.array([[f(r, lib, a) for a in arms] for r in rows], float)


def dock_mean(rows, lib, arm):
    return float(np.mean([dock(r, lib, arm) for r in rows]))


def fitted_excess(rows, arm='rigid+delay'):
    return np.array([r['fits'][arm]['map']['delay_s'] - r['L'] for r in rows])


# ------------------------------------------------------------------------- independent statistics
def sign_flip_p(d, n_perm=20000, seed=0):
    n = len(d)
    obs = abs(d.mean())
    if n <= 16:
        s = ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1) * 2 - 1
    else:
        s = np.random.default_rng(seed).choice([-1, 1], size=(n_perm, n))
    return float(np.mean(np.abs((s * d).mean(1)) >= obs - 1e-12))


def holm(p):
    order = sorted(p, key=p.get)
    out, run = {}, 0.0
    for i, k in enumerate(order):
        run = max(run, min(1.0, (len(order) - i) * p[k]))
        out[k] = run
    return out


def stat(d):
    d = np.asarray(d, float)
    idx = np.random.default_rng(0).integers(0, len(d), (4000, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return {'n': len(d), 'mean': d.mean(), 'median': np.median(d), 'ci90': [lo, hi], 'pos': int((d > 2).sum()),
            'neg': int((d < -2).sum()), 'p': sign_flip_p(d), 'equivalent': bool(lo > -MARGIN and hi < MARGIN)}


def m5_tests(rows, lib):
    d = lambda a, b: np.array([reg(r, lib, a) - reg(r, lib, b) for r in rows])
    t = {'M5-1': ('cliff: fitted body, regret at e=-0.10 minus e=+0.05 (pred. > 0)', stat(d('fit:-0.10', 'fit:+0.05'))),
         'M5-2': ('over-estimation: fitted body, e=+0.20 minus e=+0.05 (pred. > 0)', stat(d('fit:+0.20', 'fit:+0.05'))),
         'M5-3': ('body at e=+0.10: fitted minus true (pred. equivalent within 3 s)', stat(d('fit:+0.10', 'true:+0.10'))),
         'M5-4': ('fitted lumped delay minus fitted body at e=+0.05', stat(d('rigid+delay', 'fit:+0.05')))}
    adj = holm({k: v[1]['p'] for k, v in t.items()})
    for k in t:
        t[k][1]['p_holm'] = adj[k]
    desc = {'fitted minus true body at e=0': (stat(d('fit:+0.00', 'true:+0.00')), 'body at edge'),
            'fitted minus true body at e=+0.05': (stat(d('fit:+0.05', 'true:+0.05')), 'body at e=+0.05'),
            'fitted lumped delay minus true body at e=0': (stat(d('rigid+delay', 'true:+0.00')), 'fitted lumped delay minus true'),
            'joint minus true body at e=0': (stat(d('joint', 'true:+0.00')), 'joint minus true'),
            'joint minus fitted lumped delay': (stat(d('joint', 'rigid+delay')), 'joint minus fitted')}
    return t, desc


def check_against_summary(rows, summ):
    """Compare independently recomputed statistics with summary.json; returns (mismatches, n_compared)."""
    bad, n_chk = [], 0
    if summ is None:
        return ['no summary.json'], 0
    if summ.get('n_rows') != len(rows):
        bad.append(f'n_rows: plants.jsonl {len(rows)} vs summary {summ.get("n_rows")}')
    close = lambda a, b: abs(a - b) <= 1e-7 * max(1.0, abs(b))
    for lib in LIBS:
        fam = summ['families'][lib]
        tests, desc = m5_tests(rows, lib)
        pairs = []
        for k, (_, v) in tests.items():
            pairs.append((k, v, fam['tests'][next(s for s in fam['tests'] if s.startswith(k))]))
        for k, (v, prefix) in desc.items():
            pairs.append((k, v, fam['descriptive'][next(s for s in fam['descriptive'] if s.startswith(prefix))]))
        for name, mine, th in pairs:
            for f in ('n', 'mean', 'median', 'pos', 'neg', 'p'):
                n_chk += 1
                if not close(float(mine[f]), float(th[f])):
                    bad.append(f'{lib} {name} {f}: recomputed {mine[f]} vs summary {th[f]}')
            for i in (0, 1):
                n_chk += 1
                if not close(float(mine['ci90'][i]), float(th['ci90'][i])):
                    bad.append(f'{lib} {name} ci90[{i}]: {mine["ci90"][i]} vs {th["ci90"][i]}')
            n_chk += 1
            if bool(mine['equivalent']) != bool(th['equivalent']):
                bad.append(f'{lib} {name} equivalent flag')
            if 'p_holm' in mine:
                n_chk += 2
                if not close(mine['p_holm'], th['p_holm']):
                    bad.append(f'{lib} {name} p_holm: {mine["p_holm"]} vs {th["p_holm"]}')
                if (mine['p_holm'] < ALPHA) != bool(th['significant']):
                    bad.append(f'{lib} {name} significant flag')
        for a, c in fam['curve'].items():
            x = np.array([reg(r, lib, a) for r in rows])
            dk = np.array([dock(r, lib, a) for r in rows])
            for f, v in (('regret', x.mean()), ('median_regret', np.median(x)), ('dock', dk.mean()),
                         ('p_regret_gt2', (x > 2).mean())):
                n_chk += 1
                if not close(float(v), float(c[f])):
                    bad.append(f'{lib} curve {a} {f}: {v} vs {c[f]}')
    fe, s = fitted_excess(rows), summ['fitted_delay_excess']
    L = np.array([r['L'] for r in rows])
    for f, v in (('mean', fe.mean()), ('q10', np.quantile(fe, .1)), ('q90', np.quantile(fe, .9)),
                 ('corr_with_L', np.corrcoef(fe + L, L)[0, 1])):
        n_chk += 1
        if not close(float(v), float(s[f])):
            bad.append(f'fitted_delay_excess {f}: {v} vs {s[f]}')
    return bad, n_chk


# --------------------------------------------------------------------------------------- helpers
def window(xs, ys, tol=WINDOW):
    """Excess values whose mean regret is within tol of the curve minimum; None unless that is a proper subset
    (a window only exists if some points fall outside it)."""
    ys = np.asarray(ys)
    ok = ys <= ys.min() + tol
    if ok.all():
        return None
    return [x for x, o in zip(xs, ok) if o]


def legend_item(s, x, y, color, marker, label, dash=None, line=True, size=11):
    if line:
        s.line(x, y, x + 26, y, color, 1.8, dash)
    s.marker(marker, x + 13, y, 4.5, color)
    s.text(x + 32, y + 4, label, size)
    return x + 32 + 6.2 * len(label) + 18


def hbar(s, ax, y, mean, lo, hi, color, marker, r=5.5):
    s.line(ax.px(lo), y, ax.px(hi), y, color, 2.0)
    s.line(ax.px(lo), y - 4, ax.px(lo), y + 4, color, 2.0)
    s.line(ax.px(hi), y - 4, ax.px(hi), y + 4, color, 2.0)
    s.marker(marker, ax.px(mean), y, r, color)


# --------------------------------------------------------------------------------------- fig_excess
def excess_panel(s, box_main, box_strip, rows, lib, rng, label):
    x0, y0, x1, y1 = box_main
    fx = {a: fitted_excess(rows, a) for a in ('rigid+delay', 'joint')}
    L = np.array([r['L'] for r in rows])
    ref_x = {a: (fx[a].mean(), *np.quantile(fx[a], [.1, .9])) for a in fx}
    ref_x['rigid'] = (-L.mean(), -np.quantile(L, .9), -np.quantile(L, .1))
    curves, docks = {}, {}
    for b in ('fit', 'true'):
        arms = [ekey(b, e) for e in EXCESS]
        curves[b] = boot_ci(mat(rows, lib, arms), rng)
        docks[b] = boot_ci(mat(rows, lib, arms, dock), rng)
    refm = {a: (float(mat(rows, lib, [a])[:, 0].mean()), dock_mean(rows, lib, a)) for a in REFS}
    xlo = min(EXCESS) - 0.04
    xhi = max(max(EXCESS) + 0.04, ref_x['rigid+delay'][2] + 0.03, ref_x['joint'][2] + 0.03)
    ymax_data = max(max(c[2].max() for c in curves.values()), refm['rigid+delay'][0], refm['joint'][0])
    ytop = max(5.0, min(CAP, ymax_data * 1.12))
    ax = Axes(s, box_main, (xlo, xhi), (0, ytop), xticks=nice_ticks(xlo, xhi, 8), ylabel='mean capped regret (s)',
              title=label, xtick_labels=[''] * len(nice_ticks(xlo, xhi, 8)))
    for i, b in enumerate(('fit', 'true')):             # good window: only where the data show one
        w = window(EXCESS, curves[b][0])
        if w:
            col, pad = BODY[b][0], 0.012
            wx, ww = ax.px(min(w) - pad), ax.px(max(w) + pad) - ax.px(min(w) - pad)
            s.rect(wx, y0 + 3 + 7 * i, ww, 5, col, col, 0.5)
            if b == 'fit':
                s.add(f'<rect x="{wx:.1f}" y="{y0:.1f}" width="{ww:.1f}" height="{y1 - y0:.1f}" fill="{col}" fill-opacity="0.07"/>')
    for b, (col, mk, dash, _) in BODY.items():
        m, lo, hi = curves[b]
        xs = [ax.px(e) for e in EXCESS]
        cl = lambda v: ax.py(min(v, ytop))
        s.band([(x, cl(v)) for x, v in zip(xs, hi)], [(x, cl(v)) for x, v in zip(xs, lo)], col, 0.18)
        s.poly([(x, cl(v)) for x, v in zip(xs, m)], col, 2.0, dash)
        for x, v in zip(xs, m):
            s.marker(mk, x, cl(v), 4.5, col)
    s.line(ax.px(0), y0, ax.px(0), y1, C['grey'], 0.8, '2 3')
    for a in ('rigid+delay', 'joint', 'rigid'):         # reference arms at their own mean excess
        col, mk = REF_STYLE[a]
        m, lo, hi = ref_x[a]
        yv = refm[a][0]
        yy = ax.py(min(yv, ytop))
        if m < xlo + 0.01 or m > xhi:
            xe = x0 + 14
            s.add(f'<path d="M{xe + 16:.1f} {yy:.1f}L{xe - 6:.1f} {yy:.1f}M{xe:.1f} {yy - 5:.1f}L{xe - 6:.1f} {yy:.1f}L{xe:.1f} {yy + 5:.1f}" '
                  f'stroke="{col}" stroke-width="2.4" fill="none"/>')
            s.marker(mk, xe + 22, yy, 5.5, col)
            s.text(xe + 32, yy - 7, f'{a} at -L = {m:+.2f} s: {yv:.1f} s', 11)
        else:
            hbar(s, ax, yy, m, max(lo, xlo), min(hi, xhi), col, mk)
    sx0, sy0, sx1, sy1 = box_strip                       # dock-rate strip
    dmin = min(min(d[1].min() for d in docks.values()), refm['rigid+delay'][1], refm['joint'][1])
    dlo = float(np.floor(max(0.0, dmin - 0.03) * 20) / 20)
    sa = Axes(s, box_strip, (xlo, xhi), (dlo, 1.0), xlabel='excess delay e = twin delay - boat L (s)',
              ylabel='dock rate of pick', xticks=nice_ticks(xlo, xhi, 8), yticks=nice_ticks(dlo, 1.0, 3))
    for b, (col, mk, dash, _) in BODY.items():
        m, lo, hi = docks[b]
        xs = [sa.px(e) for e in EXCESS]
        s.band([(x, sa.py(v)) for x, v in zip(xs, hi)], [(x, sa.py(v)) for x, v in zip(xs, lo)], col, 0.18)
        s.poly([(x, sa.py(v)) for x, v in zip(xs, m)], col, 1.8, dash)
        for x, v in zip(xs, m):
            s.marker(mk, x, sa.py(v), 3.8, col)
    for a in ('rigid+delay', 'joint'):
        col, mk = REF_STYLE[a]
        m, lo, hi = ref_x[a]
        hbar(s, sa, sa.py(refm[a][1]), m, max(lo, xlo), min(hi, xhi), col, mk, 4.5)
    s.line(sa.px(0), sy0, sa.px(0), sy1, C['grey'], 0.8, '2 3')


def m4_panel(s, box, rows4, lib, rng, label):
    x0, y0, x1, y1 = box
    strata = ('timing', 'transient', 'steady')
    data = {}
    for st in strata:
        R = [r for r in rows4 if r['stratum'] == st]
        data[st] = boot_ci(mat(R, lib, M4_ARMS), rng) if R else None
    top = max([d[2].max() for d in data.values() if d is not None] + [5.0]) * 1.08
    ax = Axes(s, box, (0, 3), (0, top), title=label, ylabel='mean capped regret (s)', xticks=[0.5, 1.5, 2.5],
              xtick_labels=[f'{st} (n={sum(r["stratum"] == st for r in rows4)})' for st in strata])
    for k in (1, 2):
        s.line(ax.px(k), y0, ax.px(k), y1, C['lgrey'], 0.8, '3 3')
    n = len(M4_ARMS)
    for i, st in enumerate(strata):
        if data[st] is None:
            continue
        m, lo, hi = data[st]
        for j, a in enumerate(M4_ARMS):
            x = ax.px(i + 0.5 + (j - (n - 1) / 2) * 0.125)
            col, mk = M4_STYLE[a]
            s.line(x, ax.py(lo[j]), x, ax.py(hi[j]), col, 2.0)
            s.line(x - 3, ax.py(lo[j]), x + 3, ax.py(lo[j]), col, 1.5)
            s.line(x - 3, ax.py(hi[j]), x + 3, ax.py(hi[j]), col, 1.5)
            s.marker(mk, x, ax.py(m[j]), 4.8, col)


def fig_excess(rows5, rows4, out, seed=0):
    rng = np.random.default_rng(seed)
    W, H0, RH = 1340, 110, 560
    s = Svg(W, H0 + 2 * RH + 10)
    s.text(20, 24, 'Excess delay: regret and dock rate of the picked controller (a: M5 sweep, 90% bootstrap bands; '
                   'b: M4 blanket pessimism)', 14, weight='bold')
    x = 70
    for key in ('fit', 'true'):
        col, mk, dash, txt = BODY[key]
        x = legend_item(s, x, 48, col, mk, txt, dash)
    x = 70
    for a, txt in (('rigid+delay', 'rigid+delay: at mean (fitted delay - L), 10-90% range'), ('joint', 'joint: at its fitted delay - L'),
                   ('rigid', 'rigid: at -L (arrow if off-scale)')):
        col, mk = REF_STYLE[a]
        s.line(x, 70, x + 26, 70, col, 1.5)
        s.marker(mk, x + 13, 70, 5, col)
        s.text(x + 32, 74, txt, 11)
        x += 32 + 5.8 * len(txt) + 14
    s.text(70, 94, 'Shaded bar at top of panel = excess where mean capped regret is within 2 s of that curve\'s minimum '
                   '(drawn only if some points fall outside it).', 11, fill=C['grey'])
    for i, a in enumerate(M4_ARMS):
        col, mk = M4_STYLE[a]
        lx, ly = 860 + (i % 3) * 150, 48 + (i // 3) * 22
        s.marker(mk, lx + 8, ly, 4.8, col)
        s.text(lx + 20, ly + 4, M4_LABEL[a], 11)
    for ri, lib in enumerate(LIBS):
        top = H0 + ri * RH
        excess_panel(s, (70, top + 40, 720, top + 360), (70, top + 385, 720, top + 460), rows5, lib, rng,
                     f'({chr(97 + 2 * ri)}) M5 excess sweep, {len(rows5)} timing boats, {LIB_TITLE[lib]}')
        m4_panel(s, (860, top + 40, 1300, top + 360), rows4, lib, rng, f'({chr(98 + 2 * ri)}) M4 by stratum, {LIB_TITLE[lib]}')
    p = out / 'fig_excess.svg'
    s.save(p)
    return p


# ------------------------------------------------------------------------------------------ fig_map
def fig_map(rows2, out, seed=1):
    rng = np.random.default_rng(seed)
    s = Svg(1100, 760)
    s.text(20, 24, 'M2 channel x regime map: which calibration channel helps on which boats (90% bootstrap CIs)', 14, weight='bold')
    x = 70
    for a in M2_ARMS:
        col, mk = M2_STYLE[a]
        x = legend_item(s, x, 48, col, mk, a, M2_DASH[a])
    strata = ('timing', 'transient')
    for ci_, lib in enumerate(LIBS):
        for ri, (f, ylab, ttl) in enumerate(((reg, 'mean capped regret (s)', 'capped regret'),
                                             (dock, 'dock rate of pick', 'dock rate'))):
            bx0, by0 = 80 + ci_ * 520, 100 + ri * 320
            data = {st: boot_ci(mat([r for r in rows2 if r['stratum'] == st], lib, M2_ARMS, f), rng) for st in strata}
            if ri == 0:
                ylim = (0, max(d[2].max() for d in data.values()) * 1.08)
            else:
                ylim = (max(0.0, float(np.floor((min(d[1].min() for d in data.values()) - 0.03) * 20) / 20)), 1.0)
            ax = Axes(s, (bx0, by0, bx0 + 430, by0 + 240), (0, 2), ylim, title=f'{ttl}, {LIB_TITLE[lib]}', ylabel=ylab,
                      xticks=[0.5, 1.5],
                      xtick_labels=[f'{st} boats (n={sum(r["stratum"] == st for r in rows2)})' for st in strata])
            for j, a in enumerate(M2_ARMS):
                col, mk = M2_STYLE[a]
                pts = []
                for i, st in enumerate(strata):
                    m, lo, hi = data[st]
                    xx = ax.px(i + 0.5 + (j - 1.5) * 0.07)
                    s.line(xx, ax.py(lo[j]), xx, ax.py(hi[j]), col, 2.0)
                    pts.append((xx, ax.py(m[j])))
                s.poly(pts, col, 1.6, M2_DASH[a])
                for p in pts:
                    s.marker(mk, p[0], p[1], 5, col)
    p = out / 'fig_map.svg'
    s.save(p)
    return p


# ---------------------------------------------------------------------------------------------- tables
def ci(v, d=1):
    return f'[{v[0]:+.{d}f}, {v[1]:+.{d}f}]'


def tables(rows):
    L = np.array([r['L'] for r in rows])
    out = ['# M5 tables (excess-delay sweep; regret capped at 60 s; generated by report_m5.py)\n',
           f'n = {len(rows)} timing boats; L (true dead time + odometry latency): mean {L.mean():.3f} s, '
           f'range [{L.min():.3f}, {L.max():.3f}] s.\n']
    for lib in LIBS:
        tests, desc = m5_tests(rows, lib)
        out.append(f'\n## Tests, {LIB_TITLE[lib]}\n')
        out.append(md_table(['test (paired, regret a minus b)', 'mean (s)', '90% CI', 'median', 'w/l (>2 s)', 'raw p', 'Holm p',
                             'equiv. +-3 s'],
                            [[f'{k} {txt}', f'{v["mean"]:+.1f}', ci(v['ci90']), f'{v["median"]:+.1f}', f'{v["pos"]}/{v["neg"]}',
                              f'{v["p"]:.4f}', f'{v["p_holm"]:.4f}' + (' *' if v['p_holm'] < ALPHA else ''),
                              'yes' if v['equivalent'] else 'no'] for k, (txt, v) in tests.items()]))
        out.append('\nHolm over the 4 tests; * = Holm p < 0.10; w/l = boats with difference > +2 s / < -2 s.\n\nDescriptive (not tested):\n')
        out.append(md_table(['contrast', 'mean (s)', '90% CI', 'median', '>2 / <-2', 'raw p', 'equiv. +-3 s'],
                            [[k, f'{v["mean"]:+.1f}', ci(v['ci90']), f'{v["median"]:+.1f}', f'{v["pos"]}/{v["neg"]}',
                              f'{v["p"]:.4f}', 'yes' if v['equivalent'] else 'no'] for k, (v, _) in desc.items()]))
    out.append('\n## Full curves (all boats)\n')
    for lib in LIBS:
        out.append(f'\n{LIB_TITLE[lib]}: mean capped regret / median / dock rate of pick / P(regret > 2 s)\n')
        body = []
        for a in [ekey(b, e) for b in ('fit', 'true') for e in EXCESS] + list(REFS):
            x = mat(rows, lib, [a])[:, 0]
            body.append([a + (' (reference)' if a in REFS else ''), f'{x.mean():.1f}', f'{np.median(x):.1f}',
                         f'{dock_mean(rows, lib, a):.3f}', f'{(x > 2).mean():.2f}'])
        out.append(md_table(['arm', 'regret', 'median', 'dock', 'P(>2 s)'], body))
    out.append('\n## Excess of the realisable delay estimates (fitted delay minus L, s)\n')
    body = []
    for a in ('rigid+delay', 'joint'):
        fe = fitted_excess(rows, a)
        q = np.quantile(fe, [0, .1, .25, .5, .75, .9, 1])
        body.append([a] + [f'{v:+.3f}' for v in q] + [f'{fe.mean():+.3f}',
                                                      f'{np.corrcoef(fe + L, L)[0, 1]:+.2f}' if len(L) > 2 else 'n/a'])
    out.append(md_table(['arm', 'min', 'q10', 'q25', 'median', 'q75', 'q90', 'max', 'mean', 'corr(fitted, L)'], body))
    out.append('\n## Curve by tercile of L (does the cliff sit at the same excess for low- and high-latency boats?)\n')
    terc = np.array_split(np.argsort(L, kind='stable'), 3)
    for lib in LIBS:
        out.append(f'\n{LIB_TITLE[lib]}: mean capped regret (s) / dock rate per excess; "first ok e" = smallest e whose mean '
                   f'regret is within 2 s of that tercile\'s minimum (resolution 0.05 s)\n')
        body = []
        for ti, idx in enumerate(terc):
            R = [rows[i] for i in idx]
            for b in ('fit', 'true'):
                ms = [mat(R, lib, [ekey(b, e)])[:, 0].mean() for e in EXCESS]
                ds = [np.mean([dock(r, lib, ekey(b, e)) for r in R]) for e in EXCESS]
                w = window(EXCESS, ms)
                body.append([f'T{ti + 1} L {L[idx].min():.2f}-{L[idx].max():.2f} (n={len(idx)})', b] +
                            [f'{m:.1f} / {d:.2f}' for m, d in zip(ms, ds)] + ['all within 2 s' if w is None else f'{min(w):+.2f}'])
        out.append(md_table(['tercile', 'body'] + [f'e={e:+.2f}' for e in EXCESS] + ['first ok e'], body))
    return '\n'.join(out) + '\n'


# ----------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser()
    for k in ('--m5', '--m4', '--m2', '--out'):
        ap.add_argument(k, required=True)
    a = ap.parse_args(argv)
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    rows5, summ5 = load(a.m5)
    rows4, _ = load(a.m4)
    rows2, _ = load(a.m2)
    bad, n = check_against_summary(rows5, summ5)
    if bad:
        print(f'WARNING: {len(bad)} of {n} recomputed M5 statistics do NOT match summary.json:')
        for b in bad[:40]:
            print('  ', b)
    else:
        print(f'check OK: {n} recomputed M5 statistics match summary.json')
    p1, p2 = fig_excess(rows5, rows4, out), fig_map(rows2, out)
    (out / 'tables_m5.md').write_text(tables(rows5))
    wrote = [p1, p2, out / 'tables_m5.md']
    for p in (p1, p2):
        q = to_png(p)
        if q:
            wrote.append(Path(q))
    for p in wrote:
        print('wrote', p)


if __name__ == '__main__':
    main()
