"""M1 report: figures (hand-written SVG; matplotlib is not installed) and tables.

    nice -n 10 python3 -I report_m1.py RESULTS_DIR --out OUT_DIR

Reads RESULTS_DIR/plants.jsonl and summary.json written by study_m1.py. Writes fig_main.svg,
fig_generalization.svg (plus .png when rsvg-convert / inkscape / magick is on PATH) and tables.md.
No simulation is run. Pure numpy + stdlib; imports score_m1 only for fixed_regrets (the no-twin baseline).
"""

import argparse
import json
import math
import shutil
import subprocess
import sys
from html import escape
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

import score_m1  # noqa: E402

K_REF = 51.5   # thrust gauge of twin.py / plantcard: (k_fwd + k_rev) / 2 = k_ref for every twin
MAIN = ('steady', 'transient', 'timing', 'mixed')
STRATA = MAIN + ('lagsweep',)

ARM_DESC = {
    'nominal': 'prior card, no calibration',
    'broad': 'wide random prior, no data',
    'steady': 'steady windows: thrust/deadband + damping, inertia at prior',
    'rigid': 'steady + inertia',
    'rigid+delay': 'steady + inertia + delay',
    'rigid+lag': 'steady + inertia + thruster lag / reversal hold',
    'joint': 'steady + inertia + delay + lag (all channels)',
    'timing': 'M0b: delay only, everything else at prior',
    'joint-transient': 'M0b: steady + damping + delay, inertia at prior',
}


def arm_label(a, short=False):
    pre = 'sample-based fit, ' if a.startswith('abc:') else ''
    base = a[4:] if a.startswith('abc:') else a
    if short:
        return a
    return f'{a}: {pre}{ARM_DESC.get(base, "")}'.rstrip(': ')


# Okabe-Ito colours; every series also has its own marker shape and (where lines) dash, so greyscale works.
C = {'black': '#000000', 'orange': '#E69F00', 'sky': '#56B4E9', 'green': '#009E73', 'yellow': '#F0E442',
     'blue': '#0072B2', 'verm': '#D55E00', 'pink': '#CC79A7', 'grey': '#767676', 'lgrey': '#BBBBBB'}
ARM_STYLE = {   # arm -> (colour, marker)
    'nominal': (C['grey'], 'circle'), 'fixed': (C['black'], 'square'), 'steady': (C['orange'], 'tri'),
    'rigid': (C['sky'], 'diamond'), 'rigid+delay': (C['verm'], 'tridown'), 'rigid+lag': (C['green'], 'cross'),
    'joint': (C['pink'], 'plus'), 'timing': (C['blue'], 'circle'),
}
FAM_STYLE = {'v1/reg': (C['blue'], 'circle'), 'v2/reg': (C['verm'], 'square'),
             'v1/abc': (C['green'], 'tri'), 'v2/abc': (C['pink'], 'diamond')}


# ---------------------------------------------------------------------------------------------
# tiny SVG toolkit
# ---------------------------------------------------------------------------------------------

def nice_ticks(lo, hi, n=5):
    if not np.isfinite(lo) or not np.isfinite(hi):
        return [0.0, 1.0]
    if hi - lo < 1e-12:
        hi, lo = hi + 0.5, lo - 0.5
    raw = (hi - lo) / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10) if s * mag >= raw), default=10 * mag)
    t = math.ceil(lo / step - 1e-9) * step
    out = []
    while t <= hi + 1e-9 * step:
        out.append(round(t, 12))
        t += step
    return out


def fmt(v):
    if abs(v) >= 100 or float(v).is_integer():
        return f'{v:.0f}'
    return f'{v:.3g}'


class Svg:
    def __init__(self, w, h):
        self.w, self.h, self.e = w, h, []

    def add(self, s):
        self.e.append(s)

    def line(self, x1, y1, x2, y2, stroke='#000', w=1.0, dash=None, op=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ''
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" '
                 f'stroke-width="{w}"{d} stroke-opacity="{op}"/>')

    def poly(self, pts, stroke='#000', w=1.5, dash=None, op=1.0):
        if len(pts) < 2:
            return
        d = f' stroke-dasharray="{dash}"' if dash else ''
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.add(f'<polyline points="{p}" fill="none" stroke="{stroke}" stroke-width="{w}"{d} '
                 f'stroke-opacity="{op}" stroke-linejoin="round"/>')

    def band(self, top, bot, fill, op=0.25):
        pts = list(top) + list(reversed(bot))
        if len(pts) < 3:
            return
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.add(f'<polygon points="{p}" fill="{fill}" fill-opacity="{op}" stroke="none"/>')

    def rect(self, x, y, w, h, fill='none', stroke='#000', sw=1.0):
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" fill="{fill}" '
                 f'stroke="{stroke}" stroke-width="{sw}"/>')

    def text(self, x, y, s, size=12, anchor='start', weight='normal', rot=None, fill='#000', style='normal'):
        t = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot is not None else ''
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" '
                 f'font-style="{style}" fill="{fill}"{t}>{escape(str(s))}</text>')

    def marker(self, shape, x, y, r=5, color='#000', filled=True, sw=1.6):
        f = color if filled else '#fff'
        st = f'fill="{f}" stroke="{color}" stroke-width="{sw}"'
        if shape == 'circle':
            self.add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" {st}/>')
        elif shape == 'square':
            self.add(f'<rect x="{x - r:.1f}" y="{y - r:.1f}" width="{2 * r}" height="{2 * r}" {st}/>')
        elif shape in ('tri', 'tridown'):
            s = 1 if shape == 'tri' else -1
            p = f'{x:.1f},{y + s * -1.2 * r:.1f} {x - r * 1.1:.1f},{y + s * r * 0.9:.1f} {x + r * 1.1:.1f},{y + s * r * 0.9:.1f}'
            self.add(f'<polygon points="{p}" {st}/>')
        elif shape == 'diamond':
            p = f'{x:.1f},{y - 1.3 * r:.1f} {x + 1.3 * r:.1f},{y:.1f} {x:.1f},{y + 1.3 * r:.1f} {x - 1.3 * r:.1f},{y:.1f}'
            self.add(f'<polygon points="{p}" {st}/>')
        elif shape == 'cross':
            d = r * 0.95
            self.add(f'<path d="M{x - d:.1f} {y - d:.1f}L{x + d:.1f} {y + d:.1f}M{x - d:.1f} {y + d:.1f}L{x + d:.1f} {y - d:.1f}" '
                     f'stroke="{color}" stroke-width="2.4" fill="none"/>')
        elif shape == 'plus':
            d = r * 1.2
            self.add(f'<path d="M{x - d:.1f} {y:.1f}L{x + d:.1f} {y:.1f}M{x:.1f} {y - d:.1f}L{x:.1f} {y + d:.1f}" '
                     f'stroke="{color}" stroke-width="2.6" fill="none"/>')

    def save(self, path):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}" font-family="Helvetica, Arial, sans-serif">'
                f'<rect width="100%" height="100%" fill="#ffffff"/>')
        Path(path).write_text(head + '\n'.join(self.e) + '</svg>\n')


class Axes:
    """Linear axes inside pixel box (x0, y0, x1, y1)."""

    def __init__(self, svg, box, xlim, ylim, title='', xlabel='', ylabel='', xticks=None, yticks=None,
                 xtick_labels=None, grid=True, ylabel_right=None, rlim=None):
        self.s, (self.x0, self.y0, self.x1, self.y1) = svg, box
        self.xlim, self.ylim = xlim, ylim
        s = svg
        yt = yticks if yticks is not None else nice_ticks(*ylim, 5)
        xt = xticks if xticks is not None else nice_ticks(*xlim, 6)
        if grid:
            for t in yt:
                if ylim[0] <= t <= ylim[1]:
                    s.line(self.x0, self.py(t), self.x1, self.py(t), C['lgrey'], 0.6, op=0.7)
        s.rect(self.x0, self.y0, self.x1 - self.x0, self.y1 - self.y0)
        for t in yt:
            if ylim[0] - 1e-9 <= t <= ylim[1] + 1e-9:
                s.line(self.x0 - 4, self.py(t), self.x0, self.py(t))
                s.text(self.x0 - 7, self.py(t) + 4, fmt(t), 11, 'end')
        for i, t in enumerate(xt):
            if xlim[0] - 1e-9 <= t <= xlim[1] + 1e-9:
                s.line(self.px(t), self.y1, self.px(t), self.y1 + 4)
                lab = xtick_labels[i] if xtick_labels else fmt(t)
                s.text(self.px(t), self.y1 + 17, lab, 11, 'middle')
        if title:
            s.text(self.x0, self.y0 - 10, title, 13, 'start', 'bold')
        if xlabel:
            s.text((self.x0 + self.x1) / 2, self.y1 + 36, xlabel, 12, 'middle')
        if ylabel:
            s.text(self.x0 - 44, (self.y0 + self.y1) / 2, ylabel, 12, 'middle', rot=-90)
        if rlim is not None:                                  # right axis
            self.rlim = rlim
            for t in nice_ticks(*rlim, 5):
                if rlim[0] - 1e-9 <= t <= rlim[1] + 1e-9:
                    s.line(self.x1, self.pr(t), self.x1 + 4, self.pr(t))
                    s.text(self.x1 + 7, self.pr(t) + 4, fmt(t), 11, 'start', fill=C['verm'])
            if ylabel_right:
                s.text(self.x1 + 46, (self.y0 + self.y1) / 2, ylabel_right, 12, 'middle', rot=90, fill=C['verm'])

    def px(self, x):
        return self.x0 + (x - self.xlim[0]) / (self.xlim[1] - self.xlim[0]) * (self.x1 - self.x0)

    def py(self, y):
        return self.y1 - (y - self.ylim[0]) / (self.ylim[1] - self.ylim[0]) * (self.y1 - self.y0)

    def pr(self, y):
        return self.y1 - (y - self.rlim[0]) / (self.rlim[1] - self.rlim[0]) * (self.y1 - self.y0)

    def zero(self, dash='4 3'):
        if self.ylim[0] < 0 < self.ylim[1]:
            self.s.line(self.x0, self.py(0), self.x1, self.py(0), C['black'], 1.0, dash)


def padded(lo, hi, frac=0.08, include=None):
    if include is not None:
        lo, hi = min(lo, include), max(hi, include)
    if hi - lo < 1e-9:
        hi, lo = hi + 0.5, lo - 0.5
    p = (hi - lo) * frac
    return lo - p, hi + p


def boot_ci(M, rng, n_boot=4000, q=(0.05, 0.95)):
    """M: [n_units, ...]. Bootstrap over units of the mean; returns (mean, lo, hi) arrays (NaN-aware)."""
    M = np.asarray(M, float)
    n = M.shape[0]
    if n == 0:
        return None
    idx = rng.integers(0, n, (n_boot, n))
    bm = np.nanmean(M[idx], axis=1)
    return np.nanmean(M, 0), np.nanquantile(bm, q[0], axis=0), np.nanquantile(bm, q[1], axis=0)


def to_png(svg_path):
    for tool, args in (('rsvg-convert', ['-o']), ('inkscape', ['-o']), ('magick', [])):
        exe = shutil.which(tool)
        if exe:
            out = str(svg_path)[:-4] + '.png'
            try:
                if tool == 'magick':
                    subprocess.run([exe, str(svg_path), out], check=True, timeout=120)
                elif tool == 'inkscape':
                    subprocess.run([exe, str(svg_path), '-o', out], check=True, timeout=120)
                else:
                    subprocess.run([exe, '-o', out, str(svg_path)], check=True, timeout=120)
                return out
            except Exception:
                continue
    return None


# ---------------------------------------------------------------------------------------------
# data access
# ---------------------------------------------------------------------------------------------

def reg(r, lib, arm, rule='R0'):
    return r['libs'][lib]['scores']['arms'][arm]['regret' if rule == 'R0' else 'regret_abst']


def load(res):
    rows = [json.loads(line) for line in (res / 'plants.jsonl').read_text().splitlines() if line.strip()]
    summ = json.loads((res / 'summary.json').read_text())
    return rows, summ


# ---------------------------------------------------------------------------------------------
# fig_main
# ---------------------------------------------------------------------------------------------

def panel_a(svg, box, rows, rng):
    L = [r for r in rows if r['stratum'] == 'lagsweep']
    if not L:
        svg.text(box[0], box[1] + 20, '(a) no lagsweep boats in this run', 13)
        return
    levels = sorted({r['lag_level'] for r in L})
    bases = sorted({r['lag_base'] for r in L})
    D = np.full((len(bases), len(levels)), np.nan)
    F = np.full_like(D, np.nan)
    for r in L:
        i, j = bases.index(r['lag_base']), levels.index(r['lag_level'])
        D[i, j] = reg(r, 'v1', 'rigid+delay') - reg(r, 'v1', 'rigid')
        F[i, j] = r['fits']['rigid+delay']['map']['delay_s']
    mD, loD, hiD = boot_ci(D, rng)
    mF, loF, hiF = boot_ci(F, rng)
    ylo, yhi = padded(min(np.nanmin(D), np.nanmin(loD)), max(np.nanmax(D), np.nanmax(hiD)), 0.08, 0.0)
    xl = (levels[0] - 0.03, levels[-1] + 0.03)
    rhi = max(np.nanmax(F), np.nanmax(hiF), levels[-1]) * 1.1 + 1e-6
    ax = Axes(svg, box, xl, (ylo, yhi), '(a) Lag dose-response, library v1, regression fitter',
              'true thruster lag level  tau_up  (s)', 'regret(rigid+delay) - regret(rigid)  (s)',
              xticks=levels, ylabel_right='fitted delay of rigid+delay (s)', rlim=(0, rhi))
    ax.zero()
    for i in range(len(bases)):
        ax.s.poly([(ax.px(x), ax.py(y)) for x, y in zip(levels, D[i]) if np.isfinite(y)], C['grey'], 0.8, op=0.55)
    X = [ax.px(x) for x in levels]
    svg.band(list(zip(X, [ax.py(v) for v in hiD])), list(zip(X, [ax.py(v) for v in loD])), C['blue'], 0.22)
    svg.poly(list(zip(X, [ax.py(v) for v in mD])), C['blue'], 3.2)
    for x, v in zip(X, mD):
        svg.marker('circle', x, ax.py(v), 5, C['blue'])
    # fitted delay on the right axis
    svg.band(list(zip(X, [ax.pr(v) for v in hiF])), list(zip(X, [ax.pr(v) for v in loF])), C['verm'], 0.18)
    svg.poly(list(zip(X, [ax.pr(v) for v in mF])), C['verm'], 2.6, '7 4')
    for x, v in zip(X, mF):
        svg.marker('square', x, ax.pr(v), 4.5, C['verm'], False)
    svg.line(ax.px(levels[0]), ax.pr(levels[0]), ax.px(levels[-1]), ax.pr(levels[-1]), C['verm'], 1.0, '1 3')
    ly = box[3] + 56
    svg.line(box[0], ly, box[0] + 28, ly, C['grey'], 0.8)
    svg.text(box[0] + 34, ly + 4, f'one base boat ({len(bases)})', 11)
    svg.line(box[0] + 150, ly, box[0] + 178, ly, C['blue'], 3.2)
    svg.text(box[0] + 184, ly + 4, 'mean, bootstrap 90% band (left axis)', 11)
    ly2 = ly + 16
    svg.line(box[0], ly2, box[0] + 28, ly2, C['verm'], 2.6, '7 4')
    svg.marker('square', box[0] + 14, ly2, 4.5, C['verm'], False)
    svg.text(box[0] + 34, ly2 + 4, 'fitted delay, mean and 90% band (right axis); dotted: delay = lag', 11)


def panel_b(svg, box, rows):
    T = [r for r in rows if r['stratum'] == 'transient']
    if not T:
        svg.text(box[0], box[1] + 20, '(b) no transient boats in this run', 13)
        return
    r0 = np.array([reg(r, 'v1', 'timing') for r in T])
    r1 = np.array([reg(r, 'v1', 'timing', 'R1') for r in T])
    ab = np.array([r['libs']['v1']['scores']['arms']['timing']['abstained'] for r in T])
    ylo, yhi = padded(min(r0.min(), r1.min()), max(r0.max(), r1.max()), 0.08, 0.0)
    ax = Axes(svg, box, (-0.5, 1.5), (ylo, yhi), '(b) Abstention, transient stratum, library v1',
              '', 'regret of M0b "timing" arm (delay only)  (s)', xticks=[0, 1],
              xtick_labels=['R0: argmin', 'R1: abstain, else nominal pick'])
    ax.zero()
    jit = np.linspace(-0.12, 0.12, len(T))[np.random.default_rng(3).permutation(len(T))]
    for i in range(len(T)):
        xa, xb = ax.px(0 + jit[i]), ax.px(1 + jit[i])
        if ab[i]:
            svg.line(xa, ax.py(r0[i]), xb, ax.py(r1[i]), C['verm'], 1.8)
        else:
            svg.line(xa, ax.py(r0[i]), xb, ax.py(r1[i]), C['grey'], 0.9, op=0.7)
    for i in range(len(T)):
        xa, xb = ax.px(0 + jit[i]), ax.px(1 + jit[i])
        svg.marker('circle', xa, ax.py(r0[i]), 4, C['black'], False, 1.3)
        if ab[i]:
            svg.marker('diamond', xb, ax.py(r1[i]), 4.5, C['verm'], True, 1.3)
        else:
            svg.marker('circle', xb, ax.py(r1[i]), 4, C['black'], False, 1.3)
    for k, v in ((0, r0.mean()), (1, r1.mean())):
        svg.line(ax.px(k - 0.25), ax.py(v), ax.px(k + 0.25), ax.py(v), C['black'], 3.0)
    ly = box[3] + 56
    svg.marker('circle', box[0] + 6, ly, 4, C['black'], False, 1.3)
    svg.text(box[0] + 16, ly + 4, 'boat, twin did not abstain', 11)
    svg.marker('diamond', box[0] + 190, ly, 4.5, C['verm'], True, 1.3)
    svg.text(box[0] + 202, ly + 4, f'abstained (fell back to nominal pick): {int(ab.sum())} of {len(T)}', 11)
    svg.line(box[0], ly + 16, box[0] + 22, ly + 16, C['black'], 3.0)
    svg.text(box[0] + 28, ly + 20, f'mean over boats (R0 {r0.mean():.1f} s, R1 {r1.mean():.1f} s)', 11)


def panel_c(svg, box, rows, legend_x, rng):
    per, loo, _ = score_m1.fixed_regrets(rows, 'v1')
    arms = ['nominal', 'fixed', 'steady', 'rigid', 'rigid+delay', 'rigid+lag', 'joint']
    strata = [s for s in STRATA if any(r['stratum'] == s for r in rows)]
    stats = {}
    for st in strata:
        ix = [i for i, r in enumerate(rows) if r['stratum'] == st]
        for a in arms:
            v = loo[ix] if a == 'fixed' else np.array([reg(rows[i], 'v1', a, 'R1') for i in ix])
            stats[st, a] = boot_ci(v[:, None], rng)
    lo = min(float(stats[k][1][0]) for k in stats)
    hi = max(float(stats[k][2][0]) for k in stats)
    ylo, yhi = padded(lo, hi, 0.08, 0.0)
    ax = Axes(svg, box, (-0.5, len(strata) - 0.5), (ylo, yhi), '(c) Mean regret by arm and stratum, library v1, '
              'regression fitter, abstain rule R1', '', 'mean regret (s), bootstrap 90% CI', xticks=list(range(len(strata))),
              xtick_labels=[f'{s} (n={sum(r["stratum"] == s for r in rows)})' for s in strata])
    ax.zero()
    for k in range(1, len(strata)):
        svg.line(ax.px(k - 0.5), box[1], ax.px(k - 0.5), box[3], C['lgrey'], 0.8, '2 3')
    w = 0.8 / len(arms)
    for g, st in enumerate(strata):
        for j, a in enumerate(arms):
            m, l, h = (float(x[0]) for x in stats[st, a])
            x = ax.px(g - 0.4 + w * (j + 0.5))
            col, mk = ARM_STYLE[a]
            svg.line(x, ax.py(l), x, ax.py(h), col, 2.0)
            svg.line(x - 3, ax.py(l), x + 3, ax.py(l), col, 2.0)
            svg.line(x - 3, ax.py(h), x + 3, ax.py(h), col, 2.0)
            svg.marker(mk, x, ax.py(m), 4.6, col, a != 'nominal')
    lab = {'fixed': 'fixed: best single controller on all other main-stratum boats (leave-one-out, no twin)',
           'nominal': 'nominal: prior card, no calibration', 'steady': 'steady: steady windows, inertia at prior',
           'rigid': 'rigid: steady + inertia', 'rigid+delay': 'rigid+delay: steady + inertia + delay',
           'rigid+lag': 'rigid+lag: steady + inertia + lag / reversal hold', 'joint': 'joint: all channels'}
    for j, a in enumerate(arms):
        y = box[1] + 18 + 26 * j
        col, mk = ARM_STYLE[a]
        svg.line(legend_x, y, legend_x + 24, y, col, 2.0)
        svg.marker(mk, legend_x + 12, y, 4.6, col, a != 'nominal')
        svg.text(legend_x + 32, y + 4, lab[a], 12)
    svg.text(legend_x, box[1] + 18 + 26 * len(arms) + 6, 'Arms listed in plotting order, left to right within a stratum.', 11,
             fill=C['grey'])


def fig_main(rows, out):
    rng = np.random.default_rng(11)
    W, H = 1500, 1040
    svg = Svg(W, H)
    svg.text(W / 2, 28, 'DecisionCal M1: lag dose-response, abstention and regret by arm', 17, 'middle', 'bold')
    panel_a(svg, (90, 80, 590, 400), rows, rng)
    panel_b(svg, (880, 80, 1380, 400), rows)
    panel_c(svg, (90, 610, 900, 960), rows, 960, rng)
    p = out / 'fig_main.svg'
    svg.save(p)
    return p


# ---------------------------------------------------------------------------------------------
# fig_generalization
# ---------------------------------------------------------------------------------------------

TEST_TITLE = {'T1': 'T1 transient: timing arm vs nominal (R0)', 'T2': 'T2 transient: rigid+delay vs rigid (R0)',
              'T3': 'T3 transient: timing arm, R1 vs R0', 'T4': 'T4 timing: joint vs nominal (R1)',
              'T5': 'T5 pooled: joint (R1) vs fixed LOO', 'T6': 'T6 lagsweep: slope of harm on true lag'}
TEST_UNIT = {'T6': 'regret slope (s per s of lag)'}


def fig_generalization(summ, out):
    fams = [f for f in score_m1.FAMILIES if f in summ['families']]
    W, H = 1600, 1000
    svg = Svg(W, H)
    svg.text(W / 2, 28, 'Pre-registered gains (positive = first-named arm selects better), mean with bootstrap 90% CI',
             16, 'middle', 'bold')
    svg.text(W / 2, 48, 'filled marker and * = significant after Holm (alpha 0.10) within the family; open marker = not '
             'significant; T1/T3 exist only for the regression fitter', 12, 'middle', fill=C['grey'])
    pw, ph, gx, gy = 390, 270, 120, 85
    for k in range(6):
        tid = f'T{k + 1}'
        c, r = k % 3, k // 3
        x0, y0 = 120 + c * (pw + gx), 100 + r * (ph + gy)
        ent = {}
        for f in fams:
            for name, t in summ['families'][f]['tests'].items():
                if name.startswith(tid + ' '):
                    ent[f] = t
        lo = min([t['ci90'][0] for t in ent.values()] + [0.0])
        hi = max([t['ci90'][1] for t in ent.values()] + [0.0])
        xl = padded(lo, hi, 0.1)
        ax = Axes(svg, (x0, y0, x0 + pw, y0 + ph), xl, (0, len(fams)), TEST_TITLE[tid],
                  TEST_UNIT.get(tid, 'gain (s of regret)'), '', yticks=[], grid=False)
        if xl[0] < 0 < xl[1]:
            svg.line(ax.px(0), y0, ax.px(0), y0 + ph, C['black'], 1.0, '4 3')
        for i, f in enumerate(fams):
            y = y0 + (i + 0.5) / len(fams) * ph
            col, mk = FAM_STYLE[f]
            svg.text(x0 - 8, y + 4, f, 12, 'end', fill='#000')
            if f not in ent:
                svg.text(ax.px(xl[0]) + 8, y + 4, 'n/a (arm does not exist)', 11, fill=C['grey'], style='italic')
                continue
            t = ent[f]
            l, h = t['ci90']
            svg.line(ax.px(l), y, ax.px(h), y, col, 2.6)
            svg.line(ax.px(l), y - 5, ax.px(l), y + 5, col, 2.0)
            svg.line(ax.px(h), y - 5, ax.px(h), y + 5, col, 2.0)
            svg.marker(mk, ax.px(t['mean_gain']), y, 6, col, bool(t['significant']))
            if t['significant']:
                svg.text(ax.px(h) + 5, y + 5, '*', 18, 'start', 'bold')
            svg.text(x0 + pw + 4, y + 4, f'n={t["n"]} pH={t["p_holm"]:.3f}', 10, 'start', fill=C['grey'])
    # legend
    lx, ly = 140, H - 38
    for f in fams:
        col, mk = FAM_STYLE[f]
        svg.line(lx, ly, lx + 26, ly, col, 2.6)
        svg.marker(mk, lx + 13, ly, 6, col, True)
        lib, fit = f.split('/')
        svg.text(lx + 34, ly + 4, f'{f}: library {lib}, ' + ('integral-regression fitter' if fit == 'reg' else
                                                            'sample-based (ABC) fitter'), 12)
        lx += 330
    p = out / 'fig_generalization.svg'
    svg.save(p)
    return p


# ---------------------------------------------------------------------------------------------
# tables
# ---------------------------------------------------------------------------------------------

def md_table(head, body):
    out = ['| ' + ' | '.join(head) + ' |', '|' + '|'.join(['---'] * len(head)) + '|']
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in body]
    return '\n'.join(out) + '\n'


def f1(v, d=1):
    return 'n/a' if v is None or not np.isfinite(v) else f'{v:.{d}f}'


def truth(spec):
    kf, kr = np.asarray(spec['k_fwd'], float), np.asarray(spec['k_rev'], float)
    m_eff = (spec['mass'] + spec['a11']) * K_REF / float(np.mean(0.5 * (kf + kr)))
    return {'delay': spec['dead_time'] + spec['odom_latency'], 'tau': spec['tau_up'], 'm11': m_eff}


def fitted(fit):
    """delay, tau, effective m11 of one fit report (regression: MAP grid point + surge-block mean; abc: best)."""
    if 'map' in fit:
        return {'delay': fit['map']['delay_s'], 'tau': fit['map']['tau_s'], 'm11': fit['block_map']['surge'][0]}
    if 'best' in fit:
        b = fit['best']
        return {'delay': b['delay_s'], 'tau': b['tau_s'], 'm11': b['m11']}
    return None


def corr(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
        return float('nan')
    return float(np.corrcoef(x, y)[0, 1])


def tables(rows, summ, libs=('v1', 'v2')):
    o = ['# DecisionCal M1 report tables', '',
         f'Boats: {len(rows)} ({", ".join(f"{s} {sum(r["stratum"] == s for r in rows)}" for s in STRATA)}). '
         'Regret = cross-fit selection regret in seconds of mission score J (positive gain = better selection). '
         'R0 = argmin, R1 = abstain-and-fall-back-to-nominal. CIs are bootstrap 90% over boats. '
         'Boat counts are small in smoke runs; treat as format check only.', '']
    # (i) tests
    o += ['## (i) Pre-registered tests, by family', '',
          'Gain = regret(B) - regret(A) for the first-named arm A (T6: mean per-base-boat slope). Holm over the family; '
          'significant at Holm p < 0.10. Primary family: v1/reg.', '']
    for fam, body in summ['families'].items():
        o += [f'### {fam}' + (' (primary)' if fam == score_m1.PRIMARY else ' (secondary)'), '']
        b = []
        for k, t in body['tests'].items():
            b.append([k, t['n'], f1(t['mean_gain'], 2), f'[{t["ci90"][0]:.2f}, {t["ci90"][1]:.2f}]', f1(t['median_gain'], 2),
                      f'{t["p"]:.4f}', f'{t["p_holm"]:.4f}', 'yes' if t['significant'] else 'no'])
        o += [md_table(['test', 'n', 'mean gain', '90% CI', 'median', 'raw p', 'Holm p', 'significant'], b)]
    # (ii) regret tables
    o += ['## (ii) Mean regret R0 / R1 and abstain rate', '',
          'Cell = mean regret under R0 / mean regret under R1 / abstain rate. fixed:loo is the leave-one-boat-out '
          'best single controller (no twin; not an abstaining rule).', '']
    for lib in libs:
        tab = summ['tables'][lib]
        o += [f'### library {lib}', '']
        arms = [a for a in rows[0]['libs'][lib]['scores']['arms']]
        sts = [s for s in list(MAIN) + ['main-pooled', 'lagsweep'] if s in tab]
        b = []
        for a in arms + ['fixed:loo']:
            cells = []
            for s in sts:
                t = tab[s].get(a)
                if t is None:
                    cells.append('n/a')
                elif a == 'fixed:loo':
                    cells.append(f'{t["mean_regret"]:.1f} / - / -')
                else:
                    cells.append(f'{t["mean_regret"]:.1f} / {t["mean_regret_abst"]:.1f} / {t["abstain_rate"]:.2f}')
            b.append([a] + cells)
        b.append(['*(n boats)*'] + [tab[s]['n'] for s in sts])
        o += [md_table(['arm'] + sts, b)]
    # (iii) recovery
    o += ['## (iii) Parameter recovery vs hidden truth', '',
          'Truth: delay = dead_time + odom_latency; lag = tau_up; effective inertia m11 = (mass + a11) * K_REF / '
          f'mean_thrusters(0.5 (k_fwd + k_rev)), K_REF = {K_REF} (the twin gauge fixes (k_fwd + k_rev)/2 = k_ref; '
          'twin.to_spec sets m11 = m_rb + a11 as the total). Fitted values: regression = MAP grid point (delay, tau) and '
          'surge-block posterior mean (m11); abc = best proposal. Columns: median signed error (fit - truth) in s for '
          'delay and tau, median ratio fit/true for inertia, and Pearson correlation (inertia: of logs) across boats in the '
          'stratum. "const" = the fitted value is identical on every boat (pinned or fixed by the grid).', '']
    arm_list = ['rigid', 'rigid+delay', 'rigid+lag', 'joint']
    b = []
    for pre, nm in (('', 'reg'), ('abc:', 'abc')):
        for a in arm_list:
            for s in list(STRATA) + ['all']:
                R = [r for r in rows if (s == 'all' or r['stratum'] == s) and pre + a in r['fits']]
                F = [fitted(r['fits'][pre + a]) for r in R]
                T = [truth(r['spec']) for r in R]
                if not R or any(f is None for f in F):
                    continue
                col = lambda L, k: np.array([d[k] for d in L], float)
                cells = [nm, a, s, len(R)]
                for k in ('delay', 'tau'):
                    fv, tv = col(F, k), col(T, k)
                    c = 'const' if np.std(fv) < 1e-12 else f1(corr(fv, tv), 2)
                    cells += [f1(np.median(fv - tv), 3), c]
                fv, tv = col(F, 'm11'), col(T, 'm11')
                c = 'const' if np.std(fv) < 1e-12 else f1(corr(np.log(fv), np.log(tv)), 2)
                cells += [f1(np.median(fv / tv), 2), c]
                b.append(cells)
    o += [md_table(['fitter', 'arm', 'stratum', 'n', 'delay med err (s)', 'delay corr', 'tau med err (s)', 'tau corr',
                    'inertia med ratio', 'inertia corr(log)'], b)]
    # (iv) diversity
    o += ['## (iv) Selection diversity and oracle concentration', '',
          'Cell = number of distinct controllers selected under R0 (R1 in brackets) / share of boats choosing the most '
          'common one under R0. "oracle" row: the hidden-boat oracle choice (cross-fit-free argmin of mean J).', '']
    for lib in libs:
        C_ = len(rows[0]['libs'][lib]['J_h'])
        o += [f'### library {lib} ({C_} controllers)', '']
        sts = [s for s in list(MAIN) + ['lagsweep'] if any(r['stratum'] == s for r in rows)] + ['main-pooled']
        arms = list(rows[0]['libs'][lib]['scores']['arms'])
        b = []
        for a in arms + ['oracle']:
            cells = []
            for s in sts:
                R = [r for r in rows if r['stratum'] == s or (s == 'main-pooled' and r['stratum'] in MAIN)]
                sc = [r['libs'][lib]['scores'] for r in R]
                if a == 'oracle':
                    sel = [x['oracle'] for x in sc]
                    cells.append(f'{len(set(sel))} / {max(sel.count(c) for c in set(sel)) / len(sel):.2f}')
                else:
                    sel = [x['arms'][a]['sel'] for x in sc]
                    sa = [x['arms'][a]['sel_abst'] for x in sc]
                    cells.append(f'{len(set(sel))} ({len(set(sa))}) / {max(sel.count(c) for c in set(sel)) / len(sel):.2f}')
            b.append([a] + cells)
        b.append(['*(n boats)*'] + [sum(r['stratum'] == s or (s == 'main-pooled' and r['stratum'] in MAIN) for r in rows)
                                    for s in sts])
        o += [md_table(['arm'] + sts, b)]
    # (v) all-fail
    o += ['## (v) Twins that predict every controller fails', '',
          'pred_dock_frac_any = fraction of the arm\'s particles on which at least one library controller docks. Cell = '
          'mean pred_dock_frac_any / share of boats with pred_dock_frac_any = 0 (every particle predicts all-fail) / '
          'share of boats that abstained (R1: median particle score of the pick >= 180).', '']
    for lib in libs:
        o += [f'### library {lib}', '']
        sts = [s for s in list(MAIN) + ['lagsweep'] if any(r['stratum'] == s for r in rows)]
        b = []
        for a in rows[0]['libs'][lib]['scores']['arms']:
            cells = []
            for s in sts:
                A = [r['libs'][lib]['scores']['arms'][a] for r in rows if r['stratum'] == s]
                cells.append(f'{np.mean([x["pred_dock_frac_any"] for x in A]):.2f} / '
                             f'{np.mean([x["pred_dock_frac_any"] == 0 for x in A]):.2f} / '
                             f'{np.mean([x["abstained"] for x in A]):.2f}')
            b.append([a] + cells)
        o += [md_table(['arm'] + sts, b)]
    return '\n'.join(o)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('results')
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    res, out = Path(a.results).expanduser(), Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    rows, summ = load(res)
    if len(rows) != summ.get('n_rows', len(rows)):
        print(f'warning: plants.jsonl has {len(rows)} rows, summary.json says {summ["n_rows"]}', file=sys.stderr)
    written = [fig_main(rows, out), fig_generalization(summ, out)]
    (out / 'tables.md').write_text(tables(rows, summ))
    written.append(out / 'tables.md')
    for p in list(written[:2]):
        q = to_png(p)
        if q:
            written.append(q)
    for p in written:
        print('wrote', p)


if __name__ == '__main__':
    main()
