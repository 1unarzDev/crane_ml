#!/usr/bin/env python3
"""Paired snap-on vs snap-off comparison of the snap_sensitivity.py runs (PARITY.md, task A3). EXPLORATORY.

    nice -n 19 python3 -I Tools/DecisionCal/parity/compare_sensitivity.py
Reads ~/.cache/decisioncal/parity/sens-{m5,m2}-snap{on,off}/plants.jsonl; pairs boats by id (only boats present in
both runs). 90% bootstrap CIs over boats (4000 resamples, seed 0), regret capped at 60 s as in M5/M2.
"""

import json
import sys
from pathlib import Path

import numpy as np

DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))

P = Path.home() / '.cache' / 'decisioncal' / 'parity'
CAP = 60.0
LIBS = ('v2', 'v3')


def rows(design, snap):
    f = P / f'sens-{design}-snap{snap}' / 'plants.jsonl'
    if not f.exists():
        return {}
    return {r['id']: r for r in map(json.loads, f.read_text().splitlines())}


def reg(r, lib, arm):
    return min(r['libs'][lib]['scores']['arms'][arm]['regret'], CAP)


def dock(r, lib, arm):
    return r['libs'][lib]['hidden_docked'][r['libs'][lib]['scores']['arms'][arm]['sel']]


def stat(d):
    d = np.asarray(d, float)
    if len(d) == 0:
        return None
    idx = np.random.default_rng(0).integers(0, len(d), (4000, len(d)))
    lo, hi = np.quantile(d[idx].mean(1), [0.05, 0.95])
    return {'n': int(len(d)), 'mean': float(d.mean()), 'ci90': [float(lo), float(hi)], 'pos': int((d > 2).sum()),
            'neg': int((d < -2).sum())}


def fmt(s):
    return 'n/a' if s is None else f"{s['mean']:+6.1f} [{s['ci90'][0]:+.1f}, {s['ci90'][1]:+.1f}] (+{s['pos']}/-{s['neg']}, n={s['n']})"


def contrast(on, off, ids, f):
    """f(row) -> per-boat value; returns stats under on, off, and the paired difference on - off."""
    a = np.array([f(on[i]) for i in ids])
    b = np.array([f(off[i]) for i in ids])
    return {'on': stat(a), 'off': stat(b), 'on_minus_off': stat(a - b)}


def m5():
    on, off = rows('m5', 'on'), rows('m5', 'off')
    ids = sorted(set(on) & set(off))
    out = {'n_paired': len(ids)}
    if not ids:
        return out
    E = (-0.10, -0.05, 0.0, 0.05, 0.10, 0.20)
    for lib in LIBS:
        L = {}
        for body in ('fit', 'true'):
            lo, hi = f'{body}:-0.10', f'{body}:+0.05'
            L[f'{body} cliff regret(-0.10) - regret(+0.05)'] = contrast(on, off, ids, lambda r: reg(r, lib, lo) - reg(r, lib, hi))
            L[f'{body} cliff dock(+0.05) - dock(-0.10)'] = contrast(on, off, ids, lambda r: dock(r, lib, hi) - dock(r, lib, lo))
            L[f'{body} curve'] = {f'{e:+.2f}': {s: {'regret': float(np.mean([reg(R[i], lib, f'{body}:{e:+.2f}') for i in ids])),
                                                     'dock': float(np.mean([dock(R[i], lib, f'{body}:{e:+.2f}') for i in ids]))}
                                                 for s, R in (('on', on), ('off', off))} for e in E}
        L['hidden oracle J (mean over boats)'] = {s: float(np.mean([R[i]['libs'][lib]['scores']['J_oracle'] for i in ids]))
                                                   for s, R in (('on', on), ('off', off))}
        L['oracle pick same on/off (share)'] = float(np.mean([on[i]['libs'][lib]['scores']['oracle'] == off[i]['libs'][lib]['scores']['oracle'] for i in ids]))
        out[lib] = L
    return out


def m2():
    on, off = rows('m2', 'on'), rows('m2', 'off')
    ids = sorted(set(on) & set(off))
    out = {'n_paired': len(ids)}
    if not ids:
        return out
    for lib in LIBS:
        L = {'M2-2 gain rigid+lag over rigid': contrast(on, off, ids, lambda r: reg(r, lib, 'rigid') - reg(r, lib, 'rigid+lag')),
             'M2-3 gain rigid+lag over rigid+delay': contrast(on, off, ids, lambda r: reg(r, lib, 'rigid+delay') - reg(r, lib, 'rigid+lag')),
             'gain joint over rigid': contrast(on, off, ids, lambda r: reg(r, lib, 'rigid') - reg(r, lib, 'joint')),
             'gain rigid+delay over rigid': contrast(on, off, ids, lambda r: reg(r, lib, 'rigid') - reg(r, lib, 'rigid+delay'))}
        L['arms'] = {a: {s: {'regret': float(np.mean([reg(R[i], lib, a) for i in ids])),
                             'dock': float(np.mean([dock(R[i], lib, a) for i in ids]))} for s, R in (('on', on), ('off', off))}
                     for a in ('nominal', 'rigid', 'rigid+delay', 'rigid+lag', 'joint')}
        L['hidden oracle J (mean over boats)'] = {s: float(np.mean([R[i]['libs'][lib]['scores']['J_oracle'] for i in ids]))
                                                   for s, R in (('on', on), ('off', off))}
        out[lib] = L
    return out


def main():
    res = {'m5_timing': m5(), 'm2_transient': m2()}
    (P / 'sensitivity_summary.json').write_text(json.dumps(res, indent=1))
    for design, body in res.items():
        print(f'== {design}: {body["n_paired"]} paired boats')
        for lib in LIBS:
            if lib not in body:
                continue
            print(f'  library {lib}')
            for k, v in body[lib].items():
                if isinstance(v, dict) and 'on_minus_off' in v:
                    print(f'    {k:44s} on {fmt(v["on"])} | off {fmt(v["off"])} | on-off {fmt(v["on_minus_off"])}')
                elif k.endswith('curve'):
                    print(f'    {k:44s} ' + ' '.join(f'{e}: {c["on"]["regret"]:.1f}/{c["on"]["dock"]:.2f}->{c["off"]["regret"]:.1f}/{c["off"]["dock"]:.2f}' for e, c in v.items()))
                elif k == 'arms':
                    print('    arms (on -> off, regret/dock): ' + ' '.join(f'{a}: {c["on"]["regret"]:.1f}/{c["on"]["dock"]:.2f}->{c["off"]["regret"]:.1f}/{c["off"]["dock"]:.2f}' for a, c in v.items()))
                else:
                    print(f'    {k:44s} {v}')


if __name__ == '__main__':
    main()
