"""Controller selection across records: oracle, picks, cross-fit regret with uncertainty, "same
winner?", the cost of a wrong pick, rank agreement against its noise ceiling, and screening.

Unit of analysis: one scenario (a boat) holds, per arm (a backend layer, or a twin arm used as a
selector), a score matrix J [C controllers, K repeats]; repeats are common-random-number seeds, so
column k of every controller is the same noise. Lower J is better.

The study's definitions (``Tools/DecisionCal/score.py``, pre-registered 2026-11-09) are re-implemented
here and pinned to that file's outputs by ``judge/tests/test_selection.py``:

* selection: argmin over controllers of the mean of finite scores (non-finite -> J_FAIL = 300)
* cross-fit regret: split the K repeats into halves; the oracle is the best on one half, both are
  scored on the other; average the two splits (removes the min-of-noisy-means optimism; can be < 0)
* noise band: 2 SD_K(J[oracle]) / sqrt(K / 2), the resolution of a half-split comparison
* sign-flip permutation p, Holm step-down, argsort Spearman, tau-a Kendall, bootstrap of a mean

On top (this module's own, deterministic for a fixed ``seed``):

* uncertainty of a boat's regret: bootstrap over the selector's and the target's repeats jointly
  (the pick is re-made on each resample); over boats: bootstrap over boats (``judge.stats``)
* the runs that drive it: per boat, the repeats whose paired difference J[pick, k] - J[oracle, k]
  contributes most to the mean; across boats, the boats contributing most
* ties: controllers within max(noise band, margin) of the target's best are tied winners
* rank agreement: Spearman (tie-aware) and Kendall tau-b between arms' mean scores, with the
  within-arm split-half (seed-to-seed) rank agreement as the noise ceiling sqrt(r_A r_B)
* screening (A for B): top-k containment of B's winners in A's top k, missed-failure rate (A says
  the controller docks, B says it fails), false-rejection rate (A says fail, B docks)
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Callable, Mapping, Optional, Sequence

import numpy as np

from judge import stats
from judge.insight.layers import header_block

J_FAIL = 300.0          # score.J_FAIL
J_ABSTAIN = 180.0       # score.J_ABSTAIN: a mission that never docks scores >= 120 + 60
MARGIN_S = 2.0          # score.MARGIN_S practical-equivalence margin (study amendment M0b)
FAIL_RATE = 0.5         # a controller "fails" on an arm when at least half its repeats fail
SCREEN_K = 3


# ------------------------------------------------------------------ the study's definitions

def finite_scores(J):
    J = np.asarray(J, float)
    return np.where(np.isfinite(J), J, J_FAIL)


def selection(J):
    """argmin_c mean_k J[c, k] of finite scores."""
    return int(np.argmin(finite_scores(J).mean(axis=1)))


def cross_fit_regret(J_h, sel):
    J_h = np.asarray(J_h, float)
    K = J_h.shape[1]
    A, B = np.arange(0, K // 2), np.arange(K // 2, 2 * (K // 2))
    out = []
    for a, b in ((A, B), (B, A)):
        oracle = int(np.argmin(J_h[:, a].mean(1)))
        out.append(J_h[sel, b].mean() - J_h[oracle, b].mean())
    return float(np.mean(out))


def noise_band(J_h):
    """2 SD(J[oracle]) / sqrt(K // 2) (0 for a single repeat)."""
    J_h = finite_scores(J_h)
    K = J_h.shape[1]
    oracle = int(np.argmin(J_h.mean(1)))
    return float(2 * J_h[oracle].std(ddof=1) / np.sqrt(max(K // 2, 1))) if K > 1 else 0.0


def sign_flip_p(d, n_perm=20000, seed=0):
    d = np.asarray(d, float)
    n = len(d)
    obs = abs(d.mean())
    if n <= 16:
        signs = ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1) * 2 - 1
    else:
        signs = np.random.default_rng(seed).choice([-1, 1], size=(n_perm, n))
    return float(np.mean(np.abs((signs * d).mean(1)) >= obs - 1e-12))


def holm(pvals):
    return stats.holm(pvals)


def spearman_study(a, b):
    """The study's Spearman (argsort ranks, ties broken by order; nan when a side is constant)."""
    ra, rb = np.argsort(np.argsort(a)), np.argsort(np.argsort(b))
    if ra.std() == 0 or rb.std() == 0:
        return float('nan')
    return float(np.corrcoef(ra, rb)[0, 1])


def kendall_study(x, y):
    c = d = 0
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            p = (x[i] - x[j]) * (y[i] - y[j])
            c += p > 0
            d += p < 0
    return (c - d) / max(c + d, 1)


def bootstrap_mean_ci(x, n_boot=2000, seed=0, q=(0.05, 0.95)):
    """The study's bootstrap of a mean: idx = default_rng(seed).integers(0, n, (n_boot, n))."""
    x = np.asarray(x, float)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(x), (n_boot, len(x)))
    return [float(v) for v in np.quantile(x[idx].mean(1), q)]


# ------------------------------------------------------------------ inputs

@dataclass
class Arm:
    """One arm of one scenario: J [C, K] over ``controllers`` x ``seeds``."""
    controllers: tuple
    J: np.ndarray
    failed: Optional[np.ndarray] = None      # [C, K] bool; None -> J >= J_ABSTAIN
    run_ids: Optional[list] = None           # [C][K]
    seeds: tuple = ()
    notes: list = field(default_factory=list)

    def fail_matrix(self):
        if self.failed is not None:
            return np.asarray(self.failed, bool)
        return finite_scores(self.J) >= J_ABSTAIN

    def run(self, c, k):
        return self.run_ids[c][k] if self.run_ids else f'{self.controllers[c]}#{k}'


def _scenario_of(rec):
    s = rec.header.scenario
    pk = s.pair_key or ''
    suf = f':s{s.seed}'
    return pk[:-len(suf)] if s.seed is not None and pk.endswith(suf) else (pk or s.family or rec.run_id)


def _condition_of(rec):
    c = rec.header.scenario.condition or {}
    return str(c.get('stratum') or c.get('condition') or 'all')


def _arm_of(rec):
    b = header_block(rec, 'backend')
    cal = header_block(rec, 'calibration')
    name = b.get('name') or b.get('layer') or rec.header.tier
    return f'{name}/{cal["arm"]}' if cal.get('arm') else name


def _controller_of(rec):
    return header_block(rec, 'controller').get('id') or rec.header.robot.id


def table_from_records(records, score: Callable, failed: Optional[Callable] = None, scenario_of=_scenario_of,
                       condition_of=_condition_of, arm_of=_arm_of, controller_of=_controller_of,
                       repeat_of=lambda r: r.header.scenario.seed):
    """{scenario: {'condition': str, 'arms': {arm: Arm}, 'skipped': [...]}}.

    ``score(rec)`` -> float or None (None: unscorable, the run is listed in 'skipped' and the
    repeat column dropped for that arm); ``failed(rec)`` -> bool. Repeats are matched by
    ``repeat_of`` (the CRN seed); only seeds every controller of an arm has are kept."""
    cells, cond, skipped = {}, {}, {}
    for r in records:
        sc = scenario_of(r)
        cond.setdefault(sc, condition_of(r))
        v = score(r)
        if v is None:
            skipped.setdefault(sc, []).append(r.run_id)
            continue
        f = bool(failed(r)) if failed else None
        cells.setdefault(sc, {}).setdefault(arm_of(r), {}).setdefault(controller_of(r), {})[repeat_of(r)] = (
            float(v), f, r.run_id)
    out = {}
    for sc, arms in sorted(cells.items()):
        res = {}
        for arm, by_c in sorted(arms.items()):
            ctrls = tuple(sorted(by_c))
            seeds = sorted(set.intersection(*[set(by_c[c]) for c in ctrls]), key=lambda s: (s is None, s))
            notes = []
            dropped = sorted({s for c in ctrls for s in by_c[c]} - set(seeds), key=str)
            if dropped:
                notes.append(f'seeds {dropped} not run for every controller: dropped')
            if not seeds:
                continue
            J = np.array([[by_c[c][s][0] for s in seeds] for c in ctrls])
            fl = (np.array([[by_c[c][s][1] for s in seeds] for c in ctrls], bool)
                  if failed else None)
            rid = [[by_c[c][s][2] for s in seeds] for c in ctrls]
            res[arm] = Arm(ctrls, J, fl, rid, tuple(seeds), notes)
        out[sc] = {'condition': cond[sc], 'arms': res, 'skipped': skipped.get(sc, [])}
    return out


# ------------------------------------------------------------------ one scenario

def _common_controllers(a: Arm, b: Arm):
    ids = [c for c in a.controllers if c in b.controllers]
    ia = [a.controllers.index(c) for c in ids]
    ib = [b.controllers.index(c) for c in ids]
    return ids, ia, ib


def split_half_rank(J, n_splits=50, seed=0):
    """Mean Spearman (tie-aware) between controller means on two random halves of the repeats: the
    within-arm, seed-to-seed rank agreement (nan for fewer than 2 repeats)."""
    J = finite_scores(J)
    K = J.shape[1]
    if K < 2 or J.shape[0] < 3:
        return float('nan')
    rng = np.random.default_rng(seed)
    vals = []
    for _ in range(n_splits):
        p = rng.permutation(K)
        h = K // 2
        r = stats.spearman(J[:, p[:h]].mean(1).tolist(), J[:, p[h:2 * h]].mean(1).tolist())
        if not math.isnan(r):
            vals.append(r)
    return float(np.mean(vals)) if vals else float('nan')


def pick_cost(sel_arm: Arm, tgt_arm: Arm, n_boot=1000, seed=0, k_drivers=3, margin=0.0):
    """What it costs on ``tgt_arm`` to use ``sel_arm``'s pick (one boat)."""
    ids, ia, ib = _common_controllers(sel_arm, tgt_arm)
    Js, Jt = finite_scores(sel_arm.J)[ia], finite_scores(tgt_arm.J)[ib]
    pick = int(np.argmin(Js.mean(1)))
    mean_t = Jt.mean(1)
    oracle = int(np.argmin(mean_t))
    band = noise_band(Jt)
    tol = max(band, margin)
    ties = [i for i in range(len(ids)) if mean_t[i] - mean_t[oracle] <= tol + 1e-12]
    reg = cross_fit_regret(Jt, pick)
    raw = float(mean_t[pick] - mean_t[oracle])
    rng = np.random.default_rng(seed)
    Ks, Kt = Js.shape[1], Jt.shape[1]
    boot = np.empty(n_boot)
    for b in range(n_boot):
        js = Js[:, rng.integers(0, Ks, Ks)]
        jt = Jt[:, rng.integers(0, Kt, Kt)]
        p = int(np.argmin(js.mean(1)))
        boot[b] = cross_fit_regret(jt, p) if Kt >= 2 else float(jt[p].mean() - jt.mean(1).min())
    lo, hi = np.quantile(boot, [0.05, 0.95])
    d = Jt[pick] - Jt[oracle]                       # paired over CRN repeats
    order = np.argsort(-d, kind='stable')[:k_drivers]
    drivers = [{'repeat': int(k), 'seed': tgt_arm.seeds[k] if tgt_arm.seeds else int(k),
                'pick_run': tgt_arm.run(ib[pick], k), 'oracle_run': tgt_arm.run(ib[oracle], k),
                'J_pick': float(Jt[pick, k]), 'J_oracle': float(Jt[oracle, k]), 'contribution': float(d[k] / Kt)}
               for k in order if d[k] != 0]
    return {'pick': ids[pick], 'oracle': ids[oracle], 'tied_winners': [ids[i] for i in ties], 'band': band,
            'tie_tolerance': tol, 'regret': reg, 'regret_raw': raw, 'regret_ci90': [float(lo), float(hi)],
            'same_winner': ids[pick] == ids[oracle], 'same_winner_within_band': pick in ties,
            'n_runs': int(Js.size + Jt.size), 'K_selector': Ks, 'K_target': Kt, 'drivers': drivers}


def rank_agreement(a: Arm, b: Arm, seed=0):
    ids, ia, ib = _common_controllers(a, b)
    ma, mb = finite_scores(a.J)[ia].mean(1), finite_scores(b.J)[ib].mean(1)
    ra, rb = split_half_rank(finite_scores(a.J)[ia], seed=seed), split_half_rank(finite_scores(b.J)[ib], seed=seed + 1)
    ceil = (math.sqrt(max(ra, 0.0) * max(rb, 0.0)) if not (math.isnan(ra) or math.isnan(rb)) else float('nan'))
    sp = stats.spearman(ma.tolist(), mb.tolist()) if len(ids) >= 3 else float('nan')
    kt = stats.kendall_tau(ma.tolist(), mb.tolist()) if len(ids) >= 2 else float('nan')
    return {'n_controllers': len(ids), 'spearman': sp, 'kendall_tau_b': kt, 'spearman_study': spearman_study(ma, mb),
            'ceiling_split_half': {'a': ra, 'b': rb, 'sqrt_product': ceil},
            'relative_to_ceiling': (sp / ceil) if ceil and not math.isnan(sp) and ceil > 0 else float('nan')}


def screen(a: Arm, b: Arm, ks=(1, SCREEN_K, 5), margin=0.0):
    """Can ``a`` screen for ``b`` on one boat: top-k containment of b's (tied) winners, missed
    failures and false rejections."""
    ids, ia, ib = _common_controllers(a, b)
    Ja, Jb = finite_scores(a.J)[ia], finite_scores(b.J)[ib]
    ma, mb = Ja.mean(1), Jb.mean(1)
    tol = max(noise_band(Jb), margin)
    winners = {i for i in range(len(ids)) if mb[i] - mb.min() <= tol + 1e-12}
    order = list(np.argsort(ma, kind='stable'))
    contain = {int(k): bool(winners & set(order[:k])) for k in ks}
    fa = a.fail_matrix()[ia].mean(1) >= FAIL_RATE
    fb = b.fail_matrix()[ib].mean(1) >= FAIL_RATE
    missed = [ids[i] for i in range(len(ids)) if not fa[i] and fb[i]]
    false_rej = [ids[i] for i in range(len(ids)) if fa[i] and not fb[i]]
    return {'top_k_contains_winner': contain, 'winners_b': [ids[i] for i in sorted(winners)],
            'n_fail_b': int(fb.sum()), 'n_safe_b': int((~fb).sum()), 'missed_failures': missed,
            'false_rejections': false_rej}


def analyze_scenario(arms: Mapping[str, Arm], n_boot=1000, seed=0, margin=0.0, k_drivers=3):
    names = sorted(arms)
    per_arm = {}
    for n in names:
        a = arms[n]
        Jf = finite_scores(a.J)
        m = Jf.mean(1)
        per_arm[n] = {'controllers': list(a.controllers), 'K': int(Jf.shape[1]), 'mean_J': m.tolist(),
                      'oracle': a.controllers[int(np.argmin(m))], 'band': noise_band(Jf),
                      'fail_rate': a.fail_matrix().mean(1).tolist(), 'notes': list(a.notes),
                      'within_arm_rank_split_half': split_half_rank(Jf, seed=seed)}
    pairs = {}
    for i, s in enumerate(names):
        for j, t in enumerate(names):
            if s == t:
                continue
            key = f'{s}->{t}'
            cost = pick_cost(arms[s], arms[t], n_boot, seed + 7 * i + j, k_drivers, margin)
            pairs[key] = {'selector': s, 'target': t, **cost, 'rank': rank_agreement(arms[s], arms[t], seed),
                          'screen': screen(arms[s], arms[t], margin=margin)}
    return {'arms': per_arm, 'pairs': pairs}


# ------------------------------------------------------------------ across boats

def _rate(k, n):
    if n == 0:
        return {'k': 0, 'n': 0, 'rate': None, 'wilson95': None}
    return {'k': int(k), 'n': int(n), 'rate': k / n, 'wilson95': list(stats.wilson(k, n))}


def screening_verdict(contain_frac, missed, n_fail, k=SCREEN_K):
    """'can_screen' when B's winner is in A's top k on every boat and A misses none of B's failures
    (with at least one failure to miss); 'can_screen_rank_only' when there are no B failures to
    test; else 'cannot_screen' with the reasons."""
    reasons = []
    if contain_frac is None or contain_frac < 1.0:
        reasons.append(f"B's winner outside A's top {k} on {0 if contain_frac is None else 1 - contain_frac:.0%} of boats")
    if missed:
        reasons.append(f'A calls {missed} failing-on-B controller(s) safe')
    if reasons:
        return 'cannot_screen', reasons
    if n_fail == 0:
        return 'can_screen_rank_only', ['no controller fails on B: missed-failure rate untested']
    return 'can_screen', []


def analyze(table, n_boot=1000, seed=0, margin=0.0, k_drivers=3):
    """Per scenario ``analyze_scenario`` plus, per condition (and 'all'), per selector->target:
    mean cross-fit regret with a bootstrap CI over boats, n boats / runs, the boats and runs
    driving it, winner agreement (exact / within band, Wilson CI), rank agreement vs ceiling, and
    the screening verdict."""
    per = {sc: analyze_scenario(v['arms'], n_boot, seed, margin, k_drivers) for sc, v in sorted(table.items())}
    groups = {}
    for sc, v in table.items():
        groups.setdefault(v['condition'], []).append(sc)
    groups['all'] = sorted(table)
    agg = {}
    for cond, scs in sorted(groups.items()):
        keys = sorted({k for sc in scs for k in per[sc]['pairs']})
        agg[cond] = {}
        for key in keys:
            rows = [(sc, per[sc]['pairs'][key]) for sc in sorted(scs) if key in per[sc]['pairs']]
            reg = [r['regret'] for _, r in rows]
            n = len(reg)
            est, ci = stats.bootstrap_ci(reg, lambda xs: sum(xs) / len(xs), B=2000, alpha=0.1, seed=seed)
            contrib = sorted(((r['regret'] / n, sc, r) for sc, r in rows), key=lambda x: -x[0])[:k_drivers]
            drivers = [{'scenario': sc, 'regret': r['regret'], 'contribution': c,
                        'top_run': (r['drivers'][0] if r['drivers'] else None)} for c, sc, r in contrib if c > 0]
            same = sum(r['same_winner'] for _, r in rows)
            same_b = sum(r['same_winner_within_band'] for _, r in rows)
            sp = [r['rank']['spearman'] for _, r in rows if not math.isnan(r['rank']['spearman'])]
            ceil = [r['rank']['ceiling_split_half']['sqrt_product'] for _, r in rows
                    if not math.isnan(r['rank']['ceiling_split_half']['sqrt_product'])]
            cont = {k: float(np.mean([r['screen']['top_k_contains_winner'][k] for _, r in rows]))
                    for k in rows[0][1]['screen']['top_k_contains_winner']}
            missed = sum(len(r['screen']['missed_failures']) for _, r in rows)
            nfail = sum(r['screen']['n_fail_b'] for _, r in rows)
            frej = sum(len(r['screen']['false_rejections']) for _, r in rows)
            nsafe = sum(r['screen']['n_safe_b'] for _, r in rows)
            verdict, why = screening_verdict(cont.get(SCREEN_K), missed, nfail)
            agg[cond][key] = {
                'selector': rows[0][1]['selector'], 'target': rows[0][1]['target'],
                'n_boats': n, 'n_runs': int(sum(r['n_runs'] for _, r in rows)),
                'mean_regret': est, 'mean_regret_ci90_boats': list(ci),
                'median_regret': float(np.median(reg)), 'per_boat_ci90': {sc: r['regret_ci90'] for sc, r in rows},
                'drivers': drivers,
                'same_winner': _rate(same, n), 'same_winner_within_band': _rate(same_b, n),
                'spearman_median': float(np.median(sp)) if sp else float('nan'),
                'ceiling_median': float(np.median(ceil)) if ceil else float('nan'),
                'screen': {'top_k_containment': cont, 'missed_failure': _rate(missed, nfail),
                           'false_rejection': _rate(frej, nsafe), 'verdict': verdict, 'why': why}}
    return {'scenarios': per, 'by_condition': agg,
            'definitions': {'regret': 'cross-fit (score.cross_fit_regret)', 'band': '2 SD(J[oracle]) / sqrt(K/2)',
                            'tie_tolerance': f'max(band, margin={margin})', 'fail': f'fail rate >= {FAIL_RATE}',
                            'screen_k': SCREEN_K, 'ci': '90% percentile bootstrap'}}


def table_text(result):
    """Plain-text tables of ``analyze`` output."""
    L = []
    for cond, rows in result['by_condition'].items():
        L.append(f'[{cond}]')
        L.append(f'{"selector -> target":44s} {"boats":>5s} {"runs":>5s} {"regret":>8s} {"90% CI":>17s} '
                 f'{"same":>5s} {"~same":>5s} {"rho":>6s} {"ceil":>6s} {"top3":>5s} {"missF":>7s} verdict')
        for key, r in rows.items():
            ci = r['mean_regret_ci90_boats']
            mf = r['screen']['missed_failure']
            mfs = '-' if mf['rate'] is None else f'{mf["rate"]:.2f}'
            top = r['screen']['top_k_containment'].get(SCREEN_K, float('nan'))
            L.append(f'{key:44s} {r["n_boats"]:5d} {r["n_runs"]:5d} {r["mean_regret"]:8.2f} '
                     f'[{ci[0]:7.2f},{ci[1]:7.2f}] {r["same_winner"]["rate"]:5.2f} '
                     f'{r["same_winner_within_band"]["rate"]:5.2f} {r["spearman_median"]:6.2f} '
                     f'{r["ceiling_median"]:6.2f} {top:5.2f} {mfs:>7s} {r["screen"]["verdict"]}')
            for d in r['drivers']:
                tr = d['top_run']
                L.append(f'    driver {d["scenario"]}: regret {d["regret"]:.2f} (+{d["contribution"]:.2f} to the mean)'
                         + (f'; worst repeat {tr["pick_run"]} J {tr["J_pick"]:.1f} vs oracle {tr["oracle_run"]} '
                            f'J {tr["J_oracle"]:.1f}' if tr else ''))
    return '\n'.join(L)
