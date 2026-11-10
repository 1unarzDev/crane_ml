#!/usr/bin/env python3
"""EXPLORATORY. Leave-TWO-controllers-out check of the pairwise classifier (reads out/paired_table.json).

models.py's leave-one-controller-out pairwise fold tests the pairs that involve controller c but trains on
pairs that involve the OTHER member of each tested pair, so what that member does on Unity leaks into
training. Here each of the 120 controller pairs {c, d} is predicted (on all 8 blocks) by a classifier
trained only on pairs that contain neither c nor d. The boat is still in the training data (this is a
known-boat, new-controllers estimate). Ridge strength fixed at 10 (declared, not tuned).

Feature sets: gap only | gap + raw controller descriptors of both members + boat descriptors (the
"smooth residual on raw parameters" analogue) | gap + mechanism interaction features of both members.
Null for the last: interaction features shuffled among controllers inside each (boat, seed) block.

    ulimit -v 1500000; nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/pair_l2co.py
"""

import itertools
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fg_common as F  # noqa: E402
import models as M  # noqa: E402

LAM = 10.0
N_PERM = 40


def l2co(R, X):
    lab = R['wrong']
    p = np.full(len(lab), np.nan)
    for c, d in itertools.combinations(range(16), 2):
        has = lambda k: (R['pick'] == k) | (R['other'] == k)
        te = np.where(has(c) & has(d))[0]
        tr = np.where(~has(c) & ~has(d))[0]
        S = F.Standardizer(X[tr])
        p[te] = F.logit_fit(S(X[tr]), lab[tr], LAM)(S(X[te]))
    return p


def summarize(p, R):
    lab = R['wrong']
    out = {'auc': F.auc(p, lab), 'brier': F.brier(p, lab), 'brier_base': F.brier(np.full(len(lab), lab.mean()), lab),
           'within_boat_auc': {b: F.auc(p[R['boat_id'] == i], lab[R['boat_id'] == i]) for i, b in enumerate(F.BOATS)}}
    # regret left when the top 25 % by score are escalated (same-run truth), against the same per-boat mix at random
    n = len(lab)
    k = n // 4
    esc = np.zeros(n, bool)
    esc[np.argsort(-p, kind='stable')[:k]] = True
    out['regret_top25'] = float(np.where(esc, 0.0, R['regret']).mean())
    out['regret_none'] = float(R['regret'].mean())
    rng = np.random.default_rng(7)
    st = []
    for _ in range(1000):
        e2 = np.zeros(n, bool)
        for i in range(4):
            m = np.where(R['boat_id'] == i)[0]
            e2[rng.choice(m, int(esc[m].sum()), replace=False)] = True
        st.append(np.where(e2, 0.0, R['regret']).mean())
    out['regret_top25_same_boat_mix_random'] = float(np.mean(st))
    out['p_vs_same_boat_mix'] = float((1 + (np.array(st) <= out['regret_top25'] + 1e-12).sum()) / 1001)
    return out


def main():
    T = F.load_table()
    rng = np.random.default_rng(20261113)
    res, L = {}, []
    raw_c = F.cols(T, F.RAW_CTRL)
    raw_b = F.cols(T, F.RAW_BOAT)
    for fast in ('rep', 'hid'):
        R = M.pair_table(T, fast)
        ip, io = R['i_pick'].astype(int), R['i_other'].astype(int)
        sets = {'gap_only': R['log_gap'][:, None],
                'gap+raw_params': np.column_stack([R['log_gap'], raw_c[ip], raw_c[io], raw_b[ip]]),
                'gap+mechanism': np.column_stack([R['log_gap'], R['X_mech']])}
        r = {k: summarize(l2co(R, X), R) for k, X in sets.items()}
        nul = []
        for _ in range(N_PERM):
            Rn = M.pair_table(T, fast, perm=F.within_block_perm(T['block'], rng))
            nul.append(F.auc(l2co(Rn, np.column_stack([Rn['log_gap'], Rn['X_mech']])), R['wrong']))
        nul = np.array(nul)
        r['perm'] = {'n_perm': N_PERM, 'null_auc_median': float(np.median(nul)), 'null_auc_max': float(nul.max()),
                     'p_auc': float((1 + (nul >= r['gap+mechanism']['auc'] - 1e-12).sum()) / (1 + N_PERM))}
        res[fast] = r
        L.append(f'[{fast}] leave-two-controllers-out, {int(R["wrong"].sum())}/{len(R["wrong"])} wrong picks, ridge {LAM}')
        for k in sets:
            v = r[k]
            L.append(f"   {k:15s} AUC {v['auc']:.3f}  Brier {v['brier']:.4f} (base {v['brier_base']:.4f})  within-boat AUC "
                     + ' '.join(f'{b} {x:.2f}' for b, x in v['within_boat_auc'].items())
                     + f"  | escalate top 25 %: regret {v['regret_top25']:.2f} s (none {v['regret_none']:.2f}; same boat mix at random {v['regret_top25_same_boat_mix_random']:.2f}, p {v['p_vs_same_boat_mix']:.3f})")
        L.append(f"   mechanism features shuffled within block ({N_PERM} perms): null AUC median {r['perm']['null_auc_median']:.3f}, max {r['perm']['null_auc_max']:.3f}, p {r['perm']['p_auc']:.3f}")
    F.jdump(res, F.OUT / 'results_l2co.json')
    txt = '\n'.join(L)
    (F.OUT / 'results_l2co.txt').write_text(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()
