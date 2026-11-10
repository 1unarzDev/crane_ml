#!/usr/bin/env python3
"""EXPLORATORY. Follow-up checks of the fidelity-gap study (reads out/paired_table.json; no simulation):

1. Permutation test of the one positive-looking number in models.py: the leave-one-controller-out pairwise
   classifier with mechanism features (AUC above the gap-only classifier). Null: the interaction features
   shuffled among the 16 controllers inside each (boat, seed) block, which keeps everything that is constant
   on a boat (so the null model still knows which boat it is on).
2. The repeat structure the stored data offers: per (boat, controller), dock counts over the 3 noise seeds in
   each layer (transient and mixed boats), and the stored Unity repeats of transient c00 seed 1 (parity).

    ulimit -v 1500000; nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/extra_checks.py
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fg_common as F  # noqa: E402
import models as M  # noqa: E402

N_PERM = {'rep': 60, 'hid': 40}
PARITY = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'closed'


def main():
    T = F.load_table()
    rng = np.random.default_rng(20261112)
    res = {'loco_pair_perm': {}}
    L = []
    for fast in ('rep', 'hid'):
        R = M.pair_table(T, fast)
        p, _ = M.pair_classifier(R, 'loco', True)
        pg, _ = M.pair_classifier(R, 'loco', False)
        a0, ag = F.auc(p, R['wrong']), F.auc(pg, R['wrong'])
        wb = {b: F.auc(p[R['boat_id'] == i], R['wrong'][R['boat_id'] == i]) for i, b in enumerate(F.BOATS)}
        nul, nul_wb = [], []
        for _ in range(N_PERM[fast]):
            Rn = M.pair_table(T, fast, perm=F.within_block_perm(T['block'], rng))
            pn, _ = M.pair_classifier(Rn, 'loco', True)
            nul.append(F.auc(pn, R['wrong']))
            nul_wb.append(F.auc(pn[R['boat_id'] == 2], R['wrong'][R['boat_id'] == 2]))
        nul, nul_wb = np.array(nul), np.array(nul_wb)
        res['loco_pair_perm'][fast] = {
            'auc_mech+gap': a0, 'auc_gap_only': ag, 'within_boat_auc': wb, 'n_perm': N_PERM[fast],
            'null_auc_median': float(np.median(nul)), 'null_auc_max': float(nul.max()),
            'p_auc': float((1 + (nul >= a0 - 1e-12).sum()) / (1 + len(nul))),
            'transient_null_auc_median': float(np.median(nul_wb)),
            'p_transient_auc': float((1 + (nul_wb >= wb['transient'] - 1e-12).sum()) / (1 + len(nul_wb)))}
        r = res['loco_pair_perm'][fast]
        L.append(f"[{fast}] LOCO pairwise classifier: AUC mech+gap {a0:.3f}, gap only {ag:.3f}; features shuffled within block "
                 f"({N_PERM[fast]} perms): null median {r['null_auc_median']:.3f}, max {r['null_auc_max']:.3f}, p {r['p_auc']:.3f}; "
                 f"transient-boat AUC {wb['transient']:.3f} vs null median {r['transient_null_auc_median']:.3f}, p {r['p_transient_auc']:.3f}; "
                 f"within-boat AUC " + ' '.join(f'{b} {v:.2f}' for b, v in wb.items()))
    # seed structure
    res['seed_counts'] = {}
    for boat in ('transient', 'mixed'):
        rows = []
        for c in range(16):
            m = (T['boat'] == boat) & (T['controller'] == c)
            rows.append({'controller': c, 'unity_docks_of_3': int(T['u_docked'][m].sum()), 'rep_docks_of_3': int(T['rep_docked'][m].sum()),
                         'hid_docks_of_3': int(T['hid_docked'][m].sum()), 'unity_J_range': float(np.ptp(T['u_J'][m])),
                         'rep_J_range': float(np.ptp(T['rep_J'][m])), 'hid_J_range': float(np.ptp(T['hid_J'][m]))})
        res['seed_counts'][boat] = rows
        L.append(f'{boat}: docks of 3 seeds per controller (unity / representable / hidden): '
                 + ' '.join(f"c{r['controller']:02d} {r['unity_docks_of_3']}/{r['rep_docks_of_3']}/{r['hid_docks_of_3']}" for r in rows))
        for k in ('unity', 'rep', 'hid'):
            mixed = sum(0 < r[f'{k}_docks_of_3'] < 3 for r in rows)
            L.append(f"   {k}: controllers with a seed-dependent dock outcome {mixed}/16; median seed range of J {np.median([r[f'{k}_J_range'] for r in rows]):.1f} s")
    reps = []
    for d in sorted(PARITY.glob('u-transient-c00-s1-*')):
        m = json.loads((d / 'meta.json').read_text())
        mm = m.get('metrics')
        reps.append({'label': d.name, 'env': m['env'], 'deadband0': m['deadband0'], 'complete': m['complete'],
                     'docked': None if not mm else mm['docked'], 't_dock': None if not mm or mm['t_dock'] != mm['t_dock'] else mm['t_dock']})
    res['stored_unity_repeats_transient_c00_s1'] = reps
    L.append('stored Unity repeats, transient c00 seed 1 (75 s cap; + the 120 s X1 run: no dock): '
             + '; '.join(f"{r['label'][len('u-transient-c00-s1-'):]} {'incomplete' if not r['complete'] else ('dock ' + format(r['t_dock'], '.1f') if r['docked'] else 'no dock')}" for r in reps))
    F.jdump(res, F.OUT / 'results_extra.json')
    txt = '\n'.join(L)
    (F.OUT / 'results_extra.txt').write_text(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()
