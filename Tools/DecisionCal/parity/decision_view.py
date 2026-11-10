#!/usr/bin/env python3
"""The decision view on the four X1/X2 boats (PARITY.md, task E): does the same controller win in the Unity layer
and the NumPy layer, and what does it cost when one layer picks for the other?

Unity J: the stored X1 (seed 1, 4 boats) and X2 (seeds 2, 3, transient and mixed) missions, unchanged.
NumPy J: the representable boat (unity_check.representable_numpy, what X1/X2 compared against) with the snap on
(as stored in the summaries, re-computed here and checked equal) and with the snap off; and the full hidden boat.

    nice -n 19 python3 -I Tools/DecisionCal/parity/decision_view.py
"""

import json
import sys
from pathlib import Path

import numpy as np

DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))

import _paths  # noqa: E402,F401
import mission  # noqa: E402
import plant  # noqa: E402
import unity_check as U  # noqa: E402
from library import LIBRARY  # noqa: E402

UC = Path.home() / '.cache' / 'decisioncal' / 'unity-check'
OUT = Path.home() / '.cache' / 'decisioncal' / 'parity' / 'decision_view.json'
BOATS = ('steady', 'timing', 'transient', 'mixed')
C = len(LIBRARY)


def unity_J(boat):
    """{seed: J [16]} from the stored summaries (complete and water-valid only)."""
    out = {}
    for seed, pat in ((1, f'x1-{boat}-c{{:02d}}'), (2, f'x2-{boat}-s2-c{{:02d}}'), (3, f'x2-{boat}-s3-c{{:02d}}')):
        J, Jr, ok = np.full(C, np.nan), np.full(C, np.nan), True
        for c in range(C):
            f = UC / pat.format(c) / 'summary.json'
            if not f.is_file():
                ok = False
                break
            d = json.loads(f.read_text())
            if d.get('complete') and not d.get('water_invalid_searches'):
                J[c] = d['metrics']['J']
            Jr[c] = d['numpy_representable']['J']
        if ok:
            out[seed] = {'unity': J, 'stored_rep_snap': Jr,
                         'docked_unity': None}
    return out


def numpy_J(spec, seed, snap):
    old = plant.SNAP_FRAC
    plant.SNAP_FRAC = 1e-3 if snap else 0.0
    try:
        m, _ = mission.run(spec.repeat(C), np.arange(C), np.full(C, seed, np.uint64))
    finally:
        plant.SNAP_FRAC = old
    return m['J'], m['docked']


def spearman(a, b):
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    return float(np.corrcoef(ra, rb)[0, 1])


def cross(Ja, Jb):
    """Pick on layer a (argmin), regret on layer b, and vice versa."""
    pa, pb = int(np.argmin(Ja)), int(np.argmin(Jb))
    return {'pick_a': pa, 'pick_b': pb, 'same': pa == pb, 'regret_on_b_of_a_pick': float(Jb[pa] - Jb.min()),
            'regret_on_a_of_b_pick': float(Ja[pb] - Ja.min()), 'rho': spearman(Ja, Jb)}


def main():
    res = {}
    for boat in BOATS:
        hid = U.get_boat(4242, boat, 0)
        rep = U.representable_numpy(hid)
        uj = unity_J(boat)
        seeds = sorted(uj)
        L = {'unity': [], 'np_rep_snap': [], 'np_rep_nosnap': [], 'np_hidden_snap': [], 'np_hidden_nosnap': []}
        D = {k: [] for k in L}
        for s in seeds:
            L['unity'].append(uj[s]['unity'])
            for k, sp, sn in (('np_rep_snap', rep, True), ('np_rep_nosnap', rep, False),
                              ('np_hidden_snap', hid, True), ('np_hidden_nosnap', hid, False)):
                J, dk = numpy_J(sp, s, sn)
                L[k].append(J)
                D[k].append(dk)
            assert np.allclose(L['np_rep_snap'][-1], uj[s]['stored_rep_snap'], atol=1e-9), (boat, s)
        L = {k: np.array(v) for k, v in L.items()}                  # [seeds, 16]
        mean = {k: v.mean(0) for k, v in L.items()}
        row = {'seeds': seeds, 'J_mean': {k: v.tolist() for k, v in mean.items()},
               'J': {k: v.tolist() for k, v in L.items()},
               'np_docked': {k: np.array(v).tolist() for k, v in D.items() if v}}
        for k in L:
            if k == 'unity':
                continue
            row[f'unity_vs_{k}'] = {'mean': cross(mean['unity'], mean[k]),
                                    'per_seed': [cross(L['unity'][i], L[k][i]) for i in range(len(seeds))]}
        # seed-to-seed noise of the cost of a wrong pick inside each layer: pick on seed i, regret on the mean of
        # the other seeds (3 seeds only)
        if len(seeds) >= 3:
            for k in ('unity', 'np_rep_snap', 'np_rep_nosnap'):
                regs = []
                for i in range(len(seeds)):
                    pick = int(np.argmin(L[k][i]))
                    other = np.delete(L[k], i, 0).mean(0)
                    regs.append(float(other[pick] - other.min()))
                row[f'within_{k}_loo_regret'] = regs
        res[boat] = row
        u = row['unity_vs_np_rep_snap']['mean']
        v = row['unity_vs_np_rep_nosnap']['mean']
        print(f"{boat:9s} seeds {seeds}: unity best c{u['pick_a']:02d}; np-rep snap best c{u['pick_b']:02d} "
              f"(rho {u['rho']:.2f}, regret on Unity {u['regret_on_a_of_b_pick']:.1f} s, on NumPy of Unity's {u['regret_on_b_of_a_pick']:.1f} s); "
              f"snap off best c{v['pick_b']:02d} (rho {v['rho']:.2f}, regret on Unity {v['regret_on_a_of_b_pick']:.1f} s, "
              f"on NumPy {v['regret_on_b_of_a_pick']:.1f} s)", flush=True)
    OUT.write_text(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
