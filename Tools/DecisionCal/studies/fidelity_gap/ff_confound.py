#!/usr/bin/env python3
"""EXPLORATORY, POST HOC (written after the model results were seen). Is the dock disagreement anything more
than one controller factor on one boat? Reads out/paired_table.json.

    nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/ff_confound.py
"""

import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fg_common as F  # noqa: E402


def main():
    T = F.load_table()
    L = []
    for b in F.BOATS:
        m = T['boat'] == b
        uf = T['u_docked'][m] == 0
        ff = T['c_ff'][m] == 1
        L.append(f'{b}: Unity fails {int(uf.sum())}/{int(m.sum())}; feed-forward controllers {int(uf[ff].sum())}/{int(ff.sum())}, PI-only {int(uf[~ff].sum())}/{int((~ff).sum())}')
    m = T['boat'] == 'transient'
    uf = T['u_docked'][m] == 0
    ff = T['c_ff'][m] == 1
    k, n = int(uf[ff].sum()), int(ff.sum())
    lo, hi = F.clopper_pearson(k, n)
    L.append(f'transient, feed-forward: Unity fail rate {k}/{n} [{lo:.2f}, {hi:.2f}] (24 missions = 8 controllers x 3 seeds, not 24 independent units); '
             f'PI-only 0/24 [0.00, {F.clopper_pearson(0, 24)[1]:.2f}]')
    L.append('transient boat, AUC of a single quantity for "Unity fails the dock" (all 48 missions | the 24 feed-forward missions only):')
    L.append(f"   {'c_ff (library factor)':28s} {F.auc(T['c_ff'][m], uf):.2f} | n/a")
    for name in F.RAW_CTRL:
        if name != 'c_ff':
            L.append(f"   {name:28s} {F.auc(T[name][m], uf):.2f} | {F.auc(T[name][m][ff], uf[ff]):.2f}")
    for fast in ('rep', 'hid'):
        rows = []
        for name in F.MECH:
            x = T[f'{fast}_{name}'][m]
            if x.std() == 0:
                continue
            rows.append((abs(F.auc(x, uf) - 0.5), name, F.auc(x, uf), F.auc(x[ff], uf[ff]), F.spearman(x, T['c_ff'][m])))
        L.append(f'   interaction features from the {fast} fast layer (top 6 by pooled AUC; last column = Spearman with c_ff):')
        for r in sorted(rows, reverse=True)[:6]:
            L.append(f'   {r[1]:28s} {r[2]:.2f} | {r[3]:.2f} | {r[4]:+.2f}')
    txt = '\n'.join(L)
    (F.OUT / 'results_ff_confound.txt').write_text(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()
