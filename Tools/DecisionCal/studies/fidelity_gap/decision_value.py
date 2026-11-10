#!/usr/bin/env python3
"""EXPLORATORY. Step 3 of the fidelity-gap study: what a selector that escalates to Unity when the
discrepancy model predicts disagreement would have achieved on the stored data, against fast-only, full-only
(everything in Unity) and escalate-at-random. Reads out/paired_table.json and out/results_models.json.

Nesting: every model score used on a boat comes from the leave-one-boat-out fit that never saw that boat
(regularisation chosen inside the three training boats). The feature list was fixed before any fit and no
threshold is tuned: escalation is compared at fixed budgets (the top 10 / 25 / 50 % of decisions by score).

Truth, two versions:
  same-run   regret measured on the very Unity runs an escalation would look at (all 8 blocks). Any policy
             that looks at Unity is scored on the run it picked from, and Unity is not repeatable after reset,
             so this is OPTIMISTIC for full-only and for every escalation policy.
  held-out   transient and mixed boats only (3 seeds): the policy sees seed s (fast and Unity), regret is
             measured on the mean Unity J of the other two seeds.

Decisions: (A) every controller pair (120 per block); (B) the full 16-controller library, with top-k
screening; (C) every 4-controller sub-library (1820 per block).

    ulimit -v 1500000; nice -n 19 python3 -I Tools/DecisionCal/studies/fidelity_gap/decision_value.py
"""

import itertools
import json
import sys
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import fg_common as F  # noqa: E402
import models as M  # noqa: E402

BUDGETS = (0.10, 0.25, 0.50)
N_RANDOM = 5000
N_STRAT = 1000
SEED = 20261111


def heldout_truth(T):
    """Per row: mean Unity J of the same boat and controller on the OTHER seeds (nan for one-seed boats)."""
    out = np.full(len(T['u_J']), np.nan)
    for i in range(len(out)):
        m = (T['boat_id'] == T['boat_id'][i]) & (T['controller'] == T['controller'][i]) & (T['seed'] != T['seed'][i])
        if m.any():
            out[i] = T['u_J'][m].mean()
    return out


# ------------------------------------------------------------------ A: pairs

def pair_decisions(T, fast, oof, rng):
    R = M.pair_table(T, fast)
    yh = heldout_truth(T)
    ip, io = R['i_pick'].astype(int), R['i_other'].astype(int)
    u = T['u_J']
    unity_pick_is_fastpick = u[ip] <= u[io]
    out = {}
    scores = {'mech+gap': np.array(oof['p_pair_wrong_mech']), 'gap_only': np.array(oof['p_pair_wrong_gap']),
              'oracle': R['regret'] + 1e-9 * rng.random(len(R['regret']))}
    # disagreement predicted by the mechanism-indexed J model (d): escalate where its predicted order differs
    # from the fast order, then where its predicted gap is smallest
    jd = np.array(oof['J_pred']['d_ridge'])
    dgap = jd[io] - jd[ip]
    scores['d_ridge_order'] = -dgap
    for truth in ('same_run', 'held_out'):
        if truth == 'same_run':
            keep = np.ones(len(ip), bool)
            ta, tb = u[ip], u[io]
        else:
            keep = ~np.isnan(yh[ip])
            ta, tb = yh[ip], yh[io]
        best = np.minimum(ta, tb)
        r_fast = (ta - best)[keep]
        r_full = np.where(unity_pick_is_fastpick, ta, tb)[keep] - best[keep]
        n = int(keep.sum())
        row = {'n_pairs': n, 'fast_only': {'regret': float(r_fast.mean()), 'unity_runs_per_decision': 0.0},
               'full_only': {'regret': float(r_full.mean()), 'unity_runs_per_decision': 2.0,
                             'removed_per_run': float((r_fast.mean() - r_full.mean()) / 2.0)},
               'wrong_gt2s_fast': int((r_fast > M.MARGIN_S).sum()), 'budgets': {}}
        for q in BUDGETS:
            k = int(round(q * n))
            rand = np.array([np.where(np.isin(np.arange(n), rng.choice(n, k, replace=False)), r_full, r_fast).mean()
                             for _ in range(N_RANDOM)])
            b = {'escalated': k, 'unity_runs_per_decision': 2.0 * k / n,
                 'random': {'regret': float((1 - k / n) * r_fast.mean() + k / n * r_full.mean()),
                            'removed_per_run': float((r_fast.mean() - r_full.mean()) / 2.0)}}
            for name, s in scores.items():
                esc = np.zeros(n, bool)
                esc[np.argsort(-s[keep], kind='stable')[:k]] = True
                reg = np.where(esc, r_full, r_fast).mean()
                # random escalation with the SAME number of escalations on each boat: removes what the score
                # gets from knowing which boat is the bad one, leaves what it knows inside a boat
                bid = R['boat_id'][keep]
                strat = np.empty(N_STRAT)
                for it in range(N_STRAT):
                    e2 = np.zeros(n, bool)
                    for i in np.unique(bid):
                        m = np.where(bid == i)[0]
                        e2[rng.choice(m, int(esc[m].sum()), replace=False)] = True
                    strat[it] = np.where(e2, r_full, r_fast).mean()
                b[name] = {'regret': float(reg), 'removed_per_run': float((r_fast.mean() - reg) / (2.0 * k / n)),
                           'p_vs_random': float((1 + (rand <= reg + 1e-12).sum()) / (1 + N_RANDOM)),
                           'same_boat_mix_random': float(strat.mean()),
                           'p_vs_same_boat_mix': float((1 + (strat <= reg + 1e-12).sum()) / (1 + N_STRAT)),
                           'share_of_escalations_by_boat': {bn: float(esc[R['boat_id'][keep] == i].sum() / max(k, 1))
                                                            for i, bn in enumerate(F.BOATS)},
                           'regret_by_boat': {bn: float(np.where(esc, r_full, r_fast)[R['boat_id'][keep] == i].mean())
                                              for i, bn in enumerate(F.BOATS) if (R['boat_id'][keep] == i).any()}}
            row['budgets'][f'{q:.2f}'] = b
        row['fast_regret_by_boat'] = {bn: float(r_fast[R['boat_id'][keep] == i].mean())
                                      for i, bn in enumerate(F.BOATS) if (R['boat_id'][keep] == i).any()}
        out[truth] = row
    return out


# ------------------------------------------------------------------ B, C: libraries

def _expected_random_topk(sel_unity, truth, k):
    """Expected truth-regret when k of the candidates are drawn at random, run in Unity and the Unity-best of
    them is picked (exact over all subsets; candidates are few)."""
    n = len(truth)
    regs = [truth[list(c)][int(np.argmin(sel_unity[list(c)]))] - truth.min() for c in itertools.combinations(range(n), k)]
    return float(np.mean(regs))


def library_decisions(T, fast, oof):
    yh = heldout_truth(T)
    f, u = T[f'{fast}_J'], T['u_J']
    rankers = {'fast': f, 'c_ridge': np.array(oof['J_pred']['c_ridge']), 'd_ridge': np.array(oof['J_pred']['d_ridge']),
               'd_hurdle': np.array(oof['J_pred']['d_hurdle']), 'cd_ridge': np.array(oof['J_pred']['cd_ridge'])}
    full = {}
    for truth in ('same_run', 'held_out'):
        rows = []
        for b in np.unique(T['block']):
            idx = np.where(T['block'] == b)[0]
            tr = u[idx] if truth == 'same_run' else yh[idx]
            if np.isnan(tr).any():
                continue
            r = {'block': int(b), 'boat': str(T['boat'][idx[0]]), 'seed': int(T['seed'][idx[0]]),
                 'unity_best': int(np.argmin(tr)), 'screen': {}}
            for name, s in rankers.items():
                order = np.argsort(s[idx], kind='stable')
                r['screen'][name] = {str(k): float(tr[order[:k]][int(np.argmin(u[idx][order[:k]]))] - tr.min()) if k > 1
                                     else float(tr[order[0]] - tr.min()) for k in (1, 2, 3, 4, 16)}
                r['screen'][name]['pick'] = int(order[0])
            r['screen']['random'] = {str(k): _expected_random_topk(u[idx], tr, k) for k in (1, 2, 3)}
            rows.append(r)
        agg = {name: {k: float(np.mean([r['screen'][name][k] for r in rows])) for k in ('1', '2', '3', '4', '16')}
               for name in rankers}
        agg['random'] = {k: float(np.mean([r['screen']['random'][k] for r in rows])) for k in ('1', '2', '3')}
        full[truth] = {'blocks': rows, 'mean_regret_by_k': agg, 'n_blocks': len(rows)}
    # C: all 4-controller sub-libraries
    combos = np.array(list(itertools.combinations(range(16), 4)))
    sub = {}
    for truth in ('same_run', 'held_out'):
        acc = {}
        per_block = []
        for b in np.unique(T['block']):
            idx = np.where(T['block'] == b)[0]
            tr = u[idx] if truth == 'same_run' else yh[idx]
            if np.isnan(tr).any():
                continue
            ub = u[idx]
            Tc, Uc = tr[combos], ub[combos]
            best = Tc.min(1)
            ar = np.arange(len(combos))
            row = {'block': int(b), 'boat': str(T['boat'][idx[0]])}
            row['full_only'] = float((Tc[ar, Uc.argmin(1)] - best).mean())           # 4 Unity runs
            for name, s in rankers.items():
                Sc = s[idx][combos]
                o = np.argsort(Sc, axis=1, kind='stable')
                row[f'{name}_k0'] = float((Tc[ar, o[:, 0]] - best).mean())           # 0 Unity runs: the ranker's pick
                top2 = o[:, :2]
                u2 = Uc[ar[:, None], top2]
                pick = top2[ar, u2.argmin(1)]
                row[f'{name}_k2'] = float((Tc[ar, pick] - best).mean())              # 2 Unity runs
            # random 2 of 4, exact
            regs = []
            for a_, b_ in itertools.combinations(range(4), 2):
                pair = np.stack([np.full(len(combos), a_), np.full(len(combos), b_)], 1)
                u2 = Uc[ar[:, None], pair]
                pick = pair[ar, u2.argmin(1)]
                regs.append((Tc[ar, pick] - best).mean())
            row['random_k2'] = float(np.mean(regs))
            row['random_k0'] = float((Tc.mean(1) - best).mean())
            per_block.append(row)
        keys = [k for k in per_block[0] if k not in ('block', 'boat')]
        acc = {k: float(np.mean([r[k] for r in per_block])) for k in keys}
        by_boat = {bn: {k: float(np.mean([r[k] for r in per_block if r['boat'] == bn])) for k in keys}
                   for bn in F.BOATS if any(r['boat'] == bn for r in per_block)}
        cmp_ = {}
        for a_, b_ in (('d_ridge_k0', 'fast_k0'), ('d_hurdle_k0', 'fast_k0'), ('c_ridge_k0', 'fast_k0'), ('cd_ridge_k0', 'fast_k0'),
                       ('d_ridge_k2', 'fast_k2'), ('d_hurdle_k2', 'fast_k2'), ('fast_k2', 'random_k2')):
            d = np.array([r[a_] - r[b_] for r in per_block])
            cmp_[f'{a_}_minus_{b_}'] = {'mean': float(d.mean()), 'blocks_better': int((d < 0).sum()), 'n_blocks': len(d),
                                        'p_signflip_blocks': F.sign_flip_exact(d)}
        sub[truth] = {'n_sublibraries_per_block': len(combos), 'n_blocks': len(per_block), 'mean_regret': acc,
                      'by_boat': by_boat, 'comparisons': cmp_}
    return full, sub


def main():
    T = F.load_table()
    oofs = json.loads((F.OUT / 'results_models.json').read_text())
    rng = np.random.default_rng(SEED)
    res = {}
    for fast in ('rep', 'hid'):
        oof = oofs[fast]['lobo']['oof']
        A = pair_decisions(T, fast, oof, rng)
        B, C = library_decisions(T, fast, oof)
        res[fast] = {'pairs': A, 'full_library': B, 'sub_libraries_of_4': C,
                     'pairs_loco_scores': pair_decisions(T, fast, oofs[fast]['loco']['oof'], rng)}
    F.jdump(res, F.OUT / 'results_decision.json')
    L = []
    for fast in ('rep', 'hid'):
        r = res[fast]
        L.append(f'=== fast layer = NumPy {"representable" if fast == "rep" else "full hidden"} boat (EXPLORATORY; LOBO-nested scores) ===')
        for truth in ('same_run', 'held_out'):
            a = r['pairs'][truth]
            L.append(f"-- A pairs, truth {truth} (n {a['n_pairs']}; fast pick wrong by > 2 s: {a['wrong_gt2s_fast']})")
            L.append(f"   fast-only regret {a['fast_only']['regret']:.2f} s (0 runs); full-only {a['full_only']['regret']:.2f} s (2 runs/decision; removes {a['full_only']['removed_per_run']:.2f} s per run); fast regret by boat "
                     + ', '.join(f'{k} {v:.2f}' for k, v in a['fast_regret_by_boat'].items()))
            for q, b in a['budgets'].items():
                L.append(f"   budget {q} ({b['unity_runs_per_decision']:.2f} runs/decision): random {b['random']['regret']:.2f} ({b['random']['removed_per_run']:.2f}/run)")
                for name in ('gap_only', 'mech+gap', 'd_ridge_order', 'oracle'):
                    v = b[name]
                    L.append(f"      {name:14s} regret {v['regret']:.2f} s, removes {v['removed_per_run']:.2f} s/run, p vs random {v['p_vs_random']:.4f}; same-boat-mix random {v['same_boat_mix_random']:.2f} (p {v['p_vs_same_boat_mix']:.3f}); escalations by boat "
                             + ' '.join(f'{k[:5]} {x:.2f}' for k, x in v['share_of_escalations_by_boat'].items()))
            a = r['pairs_loco_scores'][truth]
            L.append(f'-- A2 pairs, truth {truth}, scores from LEAVE-ONE-CONTROLLER-OUT fits (the boat is in the training data: not a new-boat estimate)')
            for q, b in a['budgets'].items():
                for name in ('gap_only', 'mech+gap'):
                    v = b[name]
                    L.append(f"   budget {q} {name:9s} regret {v['regret']:.2f} s (random {b['random']['regret']:.2f}), removes {v['removed_per_run']:.2f} s/run, p vs random {v['p_vs_random']:.4f}; same-boat-mix random {v['same_boat_mix_random']:.2f} (p {v['p_vs_same_boat_mix']:.3f}); escalations by boat "
                             + ' '.join(f'{k[:5]} {x:.2f}' for k, x in v['share_of_escalations_by_boat'].items()))
            fl = r['full_library'][truth]
            L.append(f"-- B full library, truth {truth} ({fl['n_blocks']} blocks): mean regret (s) when the top k by a ranker are run in Unity")
            for name, v in fl['mean_regret_by_k'].items():
                L.append(f"   {name:9s} " + '  '.join(f'k={k}: {x:.2f}' for k, x in v.items()))
            L.append('   per block (k=1 pick, regret): ' + '; '.join(
                f"{x['boat'][:5]}s{x['seed']} best c{x['unity_best']:02d} | " + ' '.join(f"{n[:6]} c{x['screen'][n]['pick']:02d}/{x['screen'][n]['1']:.1f}" for n in ('fast', 'd_ridge', 'd_hurdle'))
                for x in fl['blocks']))
            s = r['sub_libraries_of_4'][truth]
            L.append(f"-- C all {s['n_sublibraries_per_block']} four-controller sub-libraries, truth {truth} ({s['n_blocks']} blocks): mean regret (s)")
            L.append('   ' + '  '.join(f'{k} {v:.2f}' for k, v in s['mean_regret'].items()))
            for bn, v in s['by_boat'].items():
                L.append(f'     {bn:9s} ' + '  '.join(f'{k} {x:.2f}' for k, x in v.items()))
            for k, v in s['comparisons'].items():
                L.append(f"   {k}: {v['mean']:+.2f} s; better in {v['blocks_better']}/{v['n_blocks']} blocks; sign-flip p {v['p_signflip_blocks']:.3f}")
        L.append('')
    txt = '\n'.join(L)
    (F.OUT / 'results_decision.txt').write_text(txt + '\n')
    print(txt)


if __name__ == '__main__':
    main()
