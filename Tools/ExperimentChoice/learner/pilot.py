"""The learner pilot (EXPLORATORY, SURROGATE): arms, repeats, scoring, report.

Arms are named <design>-<inputs>:
  designs  sobol (space-filling in the raw box, snapped to the pool grid), random (uniform over the
           pool), straddle, sur (see acquire.py; both start from the first 10 Sobol courses of the
           same repeat and then add batches of 5)
  inputs   raw (w, e, d) | mech (mechanics features) | barn (BARN-style map metrics)

Same GP (gpmodel.GP), same target (sign-preserving log of the paired gap by default), same
budgets, same per-repeat seeds for every arm: repeat k fixes the Sobol shift, the random order
and which stored seed of each pool course is served first.

Replication policy, identical for every arm: each selected course is run ``r`` times (default 1)
and is never selected again. Budgets count paired runs.

Scoring: on the held-out courses only (and zero-shot on the narrow_tandem transfer grid), by
R = mean |Delta| 1[recommended winner wrong], with the recommendation = sign of the GP mean.

Transfer (the headline comparison asked for by LIT_representation.md, prediction P1): every model is
also scored zero-shot on narrow_tandem. Inputs compared: raw only, raw + robot-dimension vector
('rawdim'), slack, mechanics features. Two training regimes:
  1 footprint   the arms above, trained on compact_diff only. The dimension vector is then constant in
                training; its lengthscales are not identified and are set to DIM_LS (reported for two values).
  3 footprints  arms 'mf3-sobol-<inputs>': a third of each budget on each of compact_diff, synth_small,
                synth_xl (truth.SYNTHETIC), Sobol within each footprint's slack box. In-distribution
                score = compact_diff held-out set; held-out footprint = narrow_tandem.

CLI:  pilot.py run [--pairs ...] [--reps 20] [--r 1] [--transform slog] [--budget-s N]
      pilot.py report            (prints markdown tables, writes pilot_curves.csv and pilot_summary.json)
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_HERE)
for _p in (_PARENT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import course as C            # noqa: E402
import robots as R            # noqa: E402
import surrogate as S         # noqa: E402
import acquire as A           # noqa: E402
import features as F          # noqa: E402
import gpmodel as GPM         # noqa: E402
import truth as T             # noqa: E402

PILOT_VERSION = 1
STORE = os.path.join(S.CACHE_DIR, "learner_pilot_v%d.jsonl" % PILOT_VERSION)
CHECKPOINTS = (20, 30, 50, 75, 100, 150, 200)
N_INIT, BATCH = 10, 5
ARMS = ("sobol-raw", "sobol-mech", "sobol-barn", "random-raw", "random-mech",
        "straddle-raw", "straddle-mech", "sur-raw", "sur-mech")
MF_INPUTS = ("raw", "rawdim", "slack", "mech")
MF_ARMS = tuple("mf3-sobol-%s" % k for k in MF_INPUTS)
TRANSFER_INPUTS = {"raw": ("raw", "slack"), "mech": ("mech",), "barn": ("barn",)}
DIM_LS = (0.8, 3.2)        # lengthscales tried for the unidentified dimension-vector inputs (natural units)
SLACK_BOX = np.array([[0.08, 0.48], [0.0, 1.50], [0.04, 0.76]])      # w - W, e, d - L (m)


def _course(a):
    return C.make_course(a["w_cm"] / 100, a["e_cm"] / 100, a["d_cm"] / 100, t=a["t_cm"] / 100)


class Data:
    """Everything a run needs for one stand-in pair: pool oracle, held-out and transfer truth, features."""

    def __init__(self, pair, rows=None):
        rows = T.load_rows() if rows is None else rows
        self.pair = pair
        self.margin = T.margins_from_pool(rows)[pair]
        comp, tand = R.load_robot("compact_diff"), R.load_robot("narrow_tandem")
        self.robots = {"pool": comp, "heldout": comp, "transfer": tand}
        # courses outside the frozen box's side-clearance rule are dropped (pool and transfer grids overshoot it)
        self.aggs = {s: [a for a in T.aggregate(rows, s, pair) if T.admissible(_course(a))]
                     for s in ("pool", "heldout", "transfer")}
        self.courses = {s: [_course(a) for a in self.aggs[s]] for s in self.aggs}
        pool = self.aggs["pool"]
        seeds = T.POOL_SEEDS
        self.gaps = np.array([[a["gap"][a["seeds"].index(s)] for s in seeds] for a in pool], float)
        self.P = len(pool)
        self.delta = {s: np.array([float(np.mean(a["gap"])) for a in self.aggs[s]]) for s in self.aggs}
        self.labels = {s: np.array([T.label(a, self.margin) for a in self.aggs[s]]) for s in self.aggs}
        # features, scaled by the pool's range
        self.X = {}
        self.scalers = {}
        for kind in ("raw", "mech", "barn"):
            ref = F.matrix(kind, self.courses["pool"], comp)
            sc = F.Scaler(ref)
            self.scalers[kind] = sc
            self.X[(kind, "pool")] = sc(ref)
            self.X[(kind, "heldout")] = sc(F.matrix(kind, self.courses["heldout"], comp))
            if self.courses["transfer"]:
                self.X[(kind, "transfer")] = sc(F.matrix(kind, self.courses["transfer"], tand))
        if self.courses["transfer"]:
            # 'slack': the tandem course mapped to the compact course with the same absolute slacks
            sl = F.matrix("slack", self.courses["transfer"], tand) + np.array([comp.W, 0.0, comp.L])
            self.X[("slack", "transfer")] = self.scalers["raw"](sl)
            # 'rawdim': raw + (L, W, v_max, omega_max). Constant over the pool, so it enters only at test time.
            self.dim_train = np.array(F.dims(comp), float)
            self.dim_test = np.array(F.dims(tand), float)
        self.near = A.near_boundary(self.X[("raw", "heldout")], self.labels["heldout"],
                                    X_ref=self.X[("raw", "pool")], labels_ref=self.labels["pool"])

    # per-repeat randomness shared by all arms
    def seed_perm(self, rep):
        rng = np.random.Generator(np.random.PCG64([int(rep), 101]))
        return np.array([rng.permutation(self.gaps.shape[1]) for _ in range(self.P)])

    def sobol_order(self, rep, n=256):
        """Sobol points in the raw unit cube, each snapped to the nearest pool course not yet used."""
        pts = GPM.sobol(n, 3, seed=rep)
        Xp = self.X[("raw", "pool")]
        free = np.ones(self.P, bool)
        out = []
        for p in pts:
            d = ((Xp - p) ** 2).sum(1)
            d[~free] = np.inf
            k = int(np.argmin(d))
            free[k] = False
            out.append(k)
        return out

    def random_order(self, rep):
        rng = np.random.Generator(np.random.PCG64([int(rep), 202]))
        return [int(k) for k in rng.permutation(self.P)]

    def observe(self, chosen, perm, r):
        idx = np.repeat(np.asarray(chosen, int), r)
        y = np.concatenate([self.gaps[c, perm[c, :r]] for c in chosen])
        return idx, y

    def rule_recs(self, set_name):
        rb = self.robots[set_name]
        return {name: np.array([f(c, rb) for c in self.courses[set_name]]) for name, f in F.RULES.items()}


def _transform(name):
    return A.slog if name == "slog" else (lambda x: np.asarray(x, float))


def evaluate(D, gp, inputs):
    """Score one fitted model on the held-out set and zero-shot on the transfer set."""
    mu, _ = gp.posterior(D.X[(inputs, "heldout")])
    rec = A.recommend(mu)
    out = {"R": A.wrong_cost(D.delta["heldout"], D.labels["heldout"], rec),
           "acc": A.sign_accuracy(D.labels["heldout"], rec),
           "acc_near": A.sign_accuracy(D.labels["heldout"], rec, D.near)}
    if D.courses["transfer"]:
        for kind in TRANSFER_INPUTS[inputs]:
            mu_t, _ = gp.posterior(D.X[(kind, "transfer")])
            rec_t = A.recommend(mu_t)
            out["Rt_" + kind] = A.wrong_cost(D.delta["transfer"], D.labels["transfer"], rec_t)
            out["acct_" + kind] = A.sign_accuracy(D.labels["transfer"], rec_t)
        if inputs == "raw":
            Xt = D.X[("raw", "transfer")]
            Xt = np.hstack([Xt, np.repeat(D.dim_test[None, :], Xt.shape[0], axis=0)])
            for ls in DIM_LS:
                g2 = gp.augmented(D.dim_train, [ls] * D.dim_train.size)
                mu_t, _ = g2.posterior(Xt)
                rec_t = A.recommend(mu_t)
                tag = "rawdim" if ls == DIM_LS[0] else "rawdim_ls%g" % ls
                out["Rt_" + tag] = A.wrong_cost(D.delta["transfer"], D.labels["transfer"], rec_t)
                out["acct_" + tag] = A.sign_accuracy(D.labels["transfer"], rec_t)
    return out


class MFData:
    """Three training footprints (compact_diff, synth_small, synth_xl); narrow_tandem held out."""

    TRAIN = (("pool", "compact_diff"), ("mf_small", "synth_small"), ("mf_xl", "synth_xl"))

    def __init__(self, pair, rows=None):
        rows = T.load_rows() if rows is None else rows
        self.pair = pair
        self.margin = T.margins_from_pool(rows)[pair]
        self.train = []
        for set_name, rname in self.TRAIN:
            rb = T.load_robot(rname)
            aggs = [a for a in T.aggregate(rows, set_name, pair) if T.admissible(_course(a))]
            cs = [_course(a) for a in aggs]
            seeds = T.base_seeds(set_name)
            gaps = np.array([[a["gap"][a["seeds"].index(s)] for s in seeds] for a in aggs], float).reshape(len(aggs), len(seeds))
            sl = F.matrix("slack", cs, rb) if cs else np.zeros((0, 3))
            self.train.append({"set": set_name, "robot": rb, "courses": cs, "gaps": gaps,
                               "snap": (sl - SLACK_BOX[:, 0]) / (SLACK_BOX[:, 1] - SLACK_BOX[:, 0])})
        self.offset = np.cumsum([0] + [len(t["courses"]) for t in self.train])
        self.gaps = np.concatenate([t["gaps"] for t in self.train])
        comp, tand = R.load_robot("compact_diff"), R.load_robot("narrow_tandem")
        ev = {"heldout": comp, "transfer": tand}
        aggs = {s: [a for a in T.aggregate(rows, s, pair) if T.admissible(_course(a))] for s in ev}
        cs = {s: [_course(a) for a in aggs[s]] for s in ev}
        self.delta = {s: np.array([float(np.mean(a["gap"])) for a in aggs[s]]) for s in ev}
        self.labels = {s: np.array([T.label(a, self.margin) for a in aggs[s]]) for s in ev}
        self.X = {}
        for kind in MF_INPUTS:
            ref = np.concatenate([F.matrix(kind, t["courses"], t["robot"]) for t in self.train])
            sc = F.Scaler(ref)
            self.X[(kind, "train")] = sc(ref)
            for s in ev:
                self.X[(kind, s)] = sc(F.matrix(kind, cs[s], ev[s]))

    def ready(self):
        return all(len(t["courses"]) > 0 for t in self.train) and len(self.delta["transfer"]) > 0

    def order(self, f, rep, n=128):
        """Sobol in footprint f's slack box, snapped to its grid; indices into the stacked training set."""
        t = self.train[f]
        pts = GPM.sobol(n, 3, seed=1000 * (f + 1) + rep)
        free = np.ones(len(t["courses"]), bool)
        out = []
        for p in pts:
            if not free.any():
                break
            d = ((t["snap"] - p) ** 2).sum(1)
            d[~free] = np.inf
            k = int(np.argmin(d))
            free[k] = False
            out.append(int(self.offset[f]) + k)
        return out


def run_mf(D, arm, rep, transform="slog", checkpoints=CHECKPOINTS):
    """One repeat of a three-footprint Sobol arm: {n: {R (compact held-out), Rt (narrow_tandem), ...}}."""
    inputs = arm.split("-")[2]
    tf = _transform(transform)
    rng = np.random.Generator(np.random.PCG64([int(rep), 303]))
    perm = np.array([rng.permutation(D.gaps.shape[1]) for _ in range(D.gaps.shape[0])])
    orders = [D.order(f, rep) for f in range(len(D.train))]
    X = D.X[(inputs, "train")]
    results = {}
    for n in checkpoints:
        chosen = []
        for f, o in enumerate(orders):
            chosen += o[:n // 3 + (1 if f < n % 3 else 0)]
        y = np.array([D.gaps[c, perm[c, 0]] for c in chosen])
        gp = GPM.GP().fit(X[chosen], tf(y))
        m = {"n_actual": len(chosen), "ls": [float(v) for v in gp.ls], "noise": gp.noise}
        for s, tag in (("heldout", "R"), ("transfer", "Rt")):
            mu, _ = gp.posterior(D.X[(inputs, s)])
            rec = A.recommend(mu)
            m[tag] = A.wrong_cost(D.delta[s], D.labels[s], rec)
            m["acc" if s == "heldout" else "acct"] = A.sign_accuracy(D.labels[s], rec)
        results[n] = m
    return results


def run_arm(D, arm, rep, r=1, transform="slog", checkpoints=CHECKPOINTS, n_init=N_INIT, batch=BATCH):
    """One repeat of one arm. Returns {n: metrics} at the checkpoints (n = paired runs spent)."""
    design, inputs = arm.split("-")
    tf = _transform(transform)
    perm = D.seed_perm(rep)
    Xp = D.X[(inputs, "pool")]
    sob = D.sobol_order(rep)
    results = {}
    hyper = None

    def fit(chosen):
        nonlocal hyper
        idx, y = D.observe(chosen, perm, r)
        gp = GPM.GP().fit(Xp[idx], tf(y), init=hyper)
        hyper = (list(gp.ls), gp.noise)
        return gp

    if design in ("sobol", "random"):
        seq = sob if design == "sobol" else D.random_order(rep)
        for n in checkpoints:
            hyper = None
            gp = fit(seq[:max(n // r, 2)])
            results[n] = dict(evaluate(D, gp, inputs), ls=[float(v) for v in gp.ls], noise=gp.noise)
        return results

    chosen = list(sob[:max(n_init // r, 2)])
    q = max(batch // r, 1)
    pending = sorted(checkpoints)
    while pending:
        gp = fit(chosen)
        n = len(chosen) * r
        while pending and n >= pending[0]:
            results[pending.pop(0)] = dict(evaluate(D, gp, inputs), n_actual=n, ls=[float(v) for v in gp.ls],
                                           noise=gp.noise)
        if not pending:
            break
        mu, cov = gp.posterior(Xp, full_cov=True)
        avail = np.ones(D.P, bool)
        avail[chosen] = False
        chosen += A.select_batch(design, mu, cov, gp.noise_var / r, avail, q)
    results["chosen"] = [int(c) for c in chosen]
    return results


# --------------------------------------------------------------------------- store

def _key(pair, arm, rep, r, transform):
    return "p%d|%s|%s|%d|r%d|%s" % (PILOT_VERSION, pair, arm, rep, r, transform)


def load_runs(path=None):
    out = []
    path = path or STORE
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue
    return out


def run_all(pairs, arms, reps, r, transform, budget_s, path=None, log=print):
    path = path or STORE
    os.makedirs(os.path.dirname(path), exist_ok=True)
    done = {x["key"] for x in load_runs(path)}
    rows = T.load_rows()
    t0, n_new = time.time(), 0
    for pair in pairs:
        D = Data(pair, rows)
        DM = MFData(pair, rows) if any(a.startswith("mf3-") for a in arms) else None
        if DM is not None and not DM.ready():
            log("%s: multi-footprint truth incomplete, mf3 arms skipped" % pair)
            DM = None
        for rep in range(reps):
            for arm in arms:
                key = _key(pair, arm, rep, r, transform)
                if key in done or (arm.startswith("mf3-") and (DM is None or r != 1)):
                    continue
                if time.time() - t0 > budget_s:
                    log("budget reached after %d runs; rerun to resume" % n_new)
                    return n_new
                t1 = time.time()
                if arm.startswith("mf3-"):
                    res = run_mf(DM, arm, rep, transform=transform)
                else:
                    res = run_arm(D, arm, rep, r=r, transform=transform)
                rec = {"key": key, "label": "exploratory-surrogate", "pair": pair, "arm": arm, "rep": rep, "r": r,
                       "transform": transform, "chosen": res.pop("chosen", None),
                       "results": {str(k): v for k, v in res.items()}, "wall_s": round(time.time() - t1, 2)}
                with open(path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, sort_keys=True) + "\n")
                n_new += 1
            log("%s rep %d done, %.0f s" % (pair, rep, time.time() - t0))
    return n_new


# --------------------------------------------------------------------------- report

def _fmt(ci, scale=1.0, nd=2):
    m, lo, hi = ci
    return ("%.*f [%.*f, %.*f]" % (nd, m * scale, nd, lo * scale, nd, hi * scale)) if np.isfinite(m) else "n/a"


def collect(runs, pair, r, transform):
    """{arm: {n: {metric: [per-repeat values, ordered by rep]}}}"""
    out = {}
    for x in sorted(runs, key=lambda z: z["rep"]):
        if x["pair"] != pair or x["r"] != r or x["transform"] != transform:
            continue
        for n, m in x["results"].items():
            slot = out.setdefault(x["arm"], {}).setdefault(int(n), {})
            for k, v in m.items():
                if isinstance(v, (int, float)):
                    slot.setdefault(k, []).append(float(v))
    return out


def tie_band(D):
    """How wide the undecided region is, measured on the pool grid along d, per (w, e) column."""
    by = {}
    for a, lab in zip(D.aggs["pool"], D.labels["pool"]):
        by.setdefault((a["w_cm"], a["e_cm"]), []).append((a["d_cm"], lab))
    widths, flips, n_cols, n_two_sided = [], [], 0, 0
    for col in by.values():
        col.sort()
        n_cols += 1
        dA = [d for d, lab in col if lab == "A_wins"]
        dB = [d for d, lab in col if lab == "B_wins"]
        if dA and dB:
            n_two_sided += 1
            # band between the last B-side course and the first A-side course above it
            lo = max(dB)
            above = [d for d in dA if d > lo]
            if above:
                widths.append((min(above) - lo) / 100.0)
                flips.append(0.5 * (min(above) + lo) / 100.0)
    return {"columns": n_cols, "columns_with_both_winners": n_two_sided,
            "gap_between_last_B_and_first_A_m": widths, "flip_d_m": flips}


def report(r=1, transform="slog", out_dir=None, log=print):
    out_dir = out_dir or _HERE
    runs = load_runs()
    rows = T.load_rows()
    csv = ["pair,arm,kind,n,metric,mean,ci_lo,ci_hi,n_repeats"]
    summary = {"label": "exploratory-surrogate", "r": r, "transform": transform, "pairs": {}}
    md = []
    for pair in T.PAIRS:
        D = Data(pair, rows)
        if D.P == 0 or len(D.aggs["heldout"]) == 0:
            continue
        res = collect(runs, pair, r, transform)
        if not res:
            continue
        S_ = summary["pairs"].setdefault(pair, {})
        labs = D.labels["heldout"]
        S_["margin_s"] = D.margin
        S_["heldout_labels"] = {k: int((labs == k).sum()) for k in ("A_wins", "B_wins", "indifferent", "both_fail", "unresolved")}
        S_["pool_labels"] = {k: int((D.labels["pool"] == k).sum()) for k in ("A_wins", "B_wins", "indifferent", "both_fail", "unresolved")}
        S_["heldout_near_boundary_decided"] = int((D.near & np.isin(labs, ("A_wins", "B_wins"))).sum())
        dec = np.isin(labs, ("A_wins", "B_wins"))
        S_["heldout_mean_abs_gap_decided"] = float(np.abs(D.delta["heldout"][dec]).mean()) if dec.any() else 0.0
        S_["tie_band"] = tie_band(D)
        md.append("\n### Pair: %s  (margin %.2f s; held-out labels %s)\n" % (pair, D.margin, S_["heldout_labels"]))
        # rules
        S_["rules"] = {}
        md.append("| no-learning rule | R held-out (s) | sign acc. | acc. near boundary | R transfer (s) | sign acc. transfer |")
        md.append("|---|---|---|---|---|---|")
        recs_h = D.rule_recs("heldout")
        recs_t = D.rule_recs("transfer") if D.courses["transfer"] else {}
        for name in F.RULES:
            Rh = A.wrong_cost(D.delta["heldout"], labs, recs_h[name])
            ah = A.sign_accuracy(labs, recs_h[name])
            an = A.sign_accuracy(labs, recs_h[name], D.near)
            Rt = A.wrong_cost(D.delta["transfer"], D.labels["transfer"], recs_t[name]) if recs_t else float("nan")
            at = A.sign_accuracy(D.labels["transfer"], recs_t[name]) if recs_t else float("nan")
            S_["rules"][name] = {"R": Rh, "acc": ah, "acc_near": an, "Rt": Rt, "acct": at}
            md.append("| %s | %.3f | %.3f | %.3f | %.3f | %.3f |" % (name, Rh, ah, an, Rt, at))
            for n in CHECKPOINTS:
                csv.append("%s,%s,rule,%d,R,%.5f,%.5f,%.5f,0" % (pair, name, n, Rh, Rh, Rh))
        # learners
        S_["arms"] = {}
        for metric, title in (("R", "R on held-out courses (s), mean over repeats [95% bootstrap CI]"),
                              ("acc_near", "sign accuracy near the boundary (held-out)")):
            md.append("\n%s\n" % title)
            md.append("| arm | " + " | ".join("n=%d" % n for n in CHECKPOINTS) + " |")
            md.append("|---|" + "---|" * len(CHECKPOINTS))
            for arm in ARMS:
                if arm not in res:
                    continue
                cells = []
                for n in CHECKPOINTS:
                    vals = res[arm].get(n, {}).get(metric, [])
                    ci = A.bootstrap_ci(vals)
                    cells.append(_fmt(ci, nd=2 if metric == "R" else 2))
                    S_["arms"].setdefault(arm, {}).setdefault(str(n), {})[metric] = {"mean": ci[0], "lo": ci[1], "hi": ci[2], "n": len(vals)}
                    csv.append("%s,%s,learner,%d,%s,%.5f,%.5f,%.5f,%d" % (pair, arm, n, metric, ci[0], ci[1], ci[2], len(vals)))
                md.append("| %s | " % arm + " | ".join(cells) + " |")
        # paired contrasts (same repeat = same seeds and same initial design)
        md.append("\npaired differences in R (s), first minus second, mean over repeats [95% bootstrap CI]; negative = first is better\n")
        md.append("| contrast | " + " | ".join("n=%d" % n for n in CHECKPOINTS) + " |")
        md.append("|---|" + "---|" * len(CHECKPOINTS))
        S_["contrasts"] = {}
        contrasts = [("sobol-mech", "sobol-raw"), ("straddle-mech", "straddle-raw"), ("sur-mech", "sur-raw"),
                     ("straddle-raw", "sobol-raw"), ("sur-raw", "sobol-raw"), ("straddle-mech", "sobol-mech"),
                     ("sur-mech", "sobol-mech"), ("sobol-raw", "random-raw"), ("sobol-barn", "sobol-raw")]
        for a, b in contrasts:
            if a not in res or b not in res:
                continue
            cells = []
            for n in CHECKPOINTS:
                va, vb = res[a].get(n, {}).get("R", []), res[b].get(n, {}).get("R", [])
                k = min(len(va), len(vb))
                ci = A.bootstrap_ci(np.array(va[:k]) - np.array(vb[:k])) if k else (float("nan"),) * 3
                cells.append(_fmt(ci))
                S_["contrasts"].setdefault("%s minus %s" % (a, b), {})[str(n)] = {"mean": ci[0], "lo": ci[1], "hi": ci[2]}
            md.append("| %s - %s | " % (a, b) + " | ".join(cells) + " |")
        # best arm vs sobol-raw per n
        md.append("\nbest arm at each budget and sobol-raw's distance from it (paired, s)\n")
        md.append("| n | best arm | its R | sobol-raw minus best [95% CI] |")
        md.append("|---|---|---|---|")
        S_["best"] = {}
        for n in CHECKPOINTS:
            means = {a: float(np.mean(res[a][n]["R"])) for a in res if n in res[a] and res[a][n].get("R")}
            if not means or "sobol-raw" not in means:
                continue
            best = min(means, key=means.get)
            va, vb = res["sobol-raw"][n]["R"], res[best][n]["R"]
            k = min(len(va), len(vb))
            ci = A.bootstrap_ci(np.array(va[:k]) - np.array(vb[:k]))
            S_["best"][str(n)] = {"arm": best, "R": means[best], "sobol_raw_minus_best": {"mean": ci[0], "lo": ci[1], "hi": ci[2]}}
            md.append("| %d | %s | %.3f | %s |" % (n, best, means[best], _fmt(ci)))
        # transfer
        if D.courses["transfer"]:
            tl = D.labels["transfer"]
            S_["transfer_labels"] = {k: int((tl == k).sum()) for k in ("A_wins", "B_wins", "indifferent", "both_fail", "unresolved")}
            md.append("\nzero-shot transfer to narrow_tandem: R (s) on %d courses (labels %s)\n" % (len(tl), S_["transfer_labels"]))
            md.append("| arm / inputs at test time | " + " | ".join("n=%d" % n for n in CHECKPOINTS) + " |")
            md.append("|---|" + "---|" * len(CHECKPOINTS))
            S_["transfer"] = {}
            for arm in ARMS:
                if arm not in res:
                    continue
                kinds = TRANSFER_INPUTS[arm.split("-")[1]]
                if arm.split("-")[1] == "raw":
                    kinds = kinds + ("rawdim", "rawdim_ls%g" % DIM_LS[1])
                for kind in kinds:
                    cells = []
                    for n in CHECKPOINTS:
                        vals = res[arm].get(n, {}).get("Rt_" + kind, [])
                        ci = A.bootstrap_ci(vals)
                        cells.append(_fmt(ci))
                        S_["transfer"].setdefault("%s/%s" % (arm, kind), {})[str(n)] = {"mean": ci[0], "lo": ci[1], "hi": ci[2]}
                        csv.append("%s,%s/%s,transfer,%d,R,%.5f,%.5f,%.5f,%d" % (pair, arm, kind, n, ci[0], ci[1], ci[2], len(vals)))
                    md.append("| %s / %s | " % (arm, kind) + " | ".join(cells) + " |")
            md += headline_transfer(res, S_, csv, pair)
    tag = "" if (r == 1 and transform == "slog") else "_r%d_%s" % (r, transform)
    with open(os.path.join(out_dir, "pilot_curves%s.csv" % tag), "w", encoding="utf-8") as f:
        f.write("\n".join(csv) + "\n")
    with open(os.path.join(out_dir, "pilot_summary%s.json" % tag), "w", encoding="utf-8") as f:
        f.write(json.dumps(summary, sort_keys=True, indent=1) + "\n")
    text = "\n".join(md)
    log(text)
    return summary, text


def _ratio_ci(num, den, n_boot=4000, seed=0):
    """1 - mean(num)/mean(den) with a paired bootstrap over repeats."""
    num, den = np.asarray(num, float), np.asarray(den, float)
    k = min(num.size, den.size)
    if k == 0 or den[:k].mean() <= 0:
        return (float("nan"),) * 3
    num, den = num[:k], den[:k]
    rng = np.random.Generator(np.random.PCG64(seed))
    idx = rng.integers(0, k, (n_boot, k))
    dm = den[idx].mean(1)
    val = 1.0 - num[idx].mean(1) / np.where(dm > 0, dm, np.nan)
    val = val[np.isfinite(val)]
    lo, hi = np.percentile(val, [2.5, 97.5])
    return 1.0 - num.mean() / den.mean(), float(lo), float(hi)


def headline_transfer(res, S_, csv, pair):
    """Mechanics features vs raw + dimension vector vs raw only; in distribution and on the held-out footprint."""
    md = ["\n**Headline comparison (P1): in distribution (compact_diff held-out) vs held-out footprint (narrow_tandem); Sobol designs; R in s**\n"]
    # (label, arm, in-distribution metric, held-out-footprint metric)
    table = [("1 footprint", "raw only", "sobol-raw", "R", "Rt_raw"),
             ("1 footprint", "raw + dims (ls %g)" % DIM_LS[0], "sobol-raw", "R", "Rt_rawdim"),
             ("1 footprint", "raw + dims (ls %g)" % DIM_LS[1], "sobol-raw", "R", "Rt_rawdim_ls%g" % DIM_LS[1]),
             ("1 footprint", "slack", "sobol-raw", "R", "Rt_slack"),
             ("1 footprint", "mechanics", "sobol-mech", "R", "Rt_mech"),
             ("3 footprints", "raw only", "mf3-sobol-raw", "R", "Rt"),
             ("3 footprints", "raw + dims", "mf3-sobol-rawdim", "R", "Rt"),
             ("3 footprints", "slack", "mf3-sobol-slack", "R", "Rt"),
             ("3 footprints", "mechanics", "mf3-sobol-mech", "R", "Rt")]
    for where, idx in (("in distribution", 3), ("held-out footprint", 4)):
        md.append("%s\n" % where)
        md.append("| training | inputs | " + " | ".join("n=%d" % n for n in CHECKPOINTS) + " |")
        md.append("|---|---|" + "---|" * len(CHECKPOINTS))
        for row in table:
            if row[2] not in res:
                continue
            cells = []
            for n in CHECKPOINTS:
                ci = A.bootstrap_ci(res[row[2]].get(n, {}).get(row[idx], []))
                cells.append(_fmt(ci))
                csv.append("%s,%s|%s,headline_%s,%d,R,%.5f,%.5f,%.5f,%d" % (
                    pair, row[0], row[1].replace(",", ""), "in" if idx == 3 else "out", n, ci[0], ci[1], ci[2],
                    len(res[row[2]].get(n, {}).get(row[idx], []))))
            md.append("| %s | %s | " % (row[0], row[1]) + " | ".join(cells) + " |")
        md.append("")
    md.append("P1 check: reduction of R by mechanics features relative to raw + dims, 1 - R_mech / R_rawdim [95% paired bootstrap CI]; and the in-distribution difference R_mech - R_rawdim (s)\n")
    md.append("| training | n | held-out footprint: reduction | in distribution: reduction | in distribution: difference (s) |")
    md.append("|---|---|---|---|---|")
    S_["P1"] = {}
    for regime, a_mech, a_dim, m_out_mech, m_out_dim in (
            ("1 footprint", "sobol-mech", "sobol-raw", "Rt_mech", "Rt_rawdim"),
            ("3 footprints", "mf3-sobol-mech", "mf3-sobol-rawdim", "Rt", "Rt")):
        if a_mech not in res or a_dim not in res:
            continue
        for n in CHECKPOINTS:
            om, od = res[a_mech].get(n, {}).get(m_out_mech, []), res[a_dim].get(n, {}).get(m_out_dim, [])
            im, idm = res[a_mech].get(n, {}).get("R", []), res[a_dim].get(n, {}).get("R", [])
            k = min(len(im), len(idm))
            out_red, in_red = _ratio_ci(om, od), _ratio_ci(im, idm)
            in_diff = A.bootstrap_ci(np.array(im[:k]) - np.array(idm[:k])) if k else (float("nan"),) * 3
            S_["P1"].setdefault(regime, {})[str(n)] = {
                "heldout_footprint_reduction": dict(zip(("mean", "lo", "hi"), out_red)),
                "in_distribution_reduction": dict(zip(("mean", "lo", "hi"), in_red)),
                "in_distribution_difference_s": dict(zip(("mean", "lo", "hi"), in_diff)),
                "R_out_mech": float(np.mean(om)) if om else float("nan"), "R_out_rawdim": float(np.mean(od)) if od else float("nan")}
            md.append("| %s | %d | %s | %s | %s |" % (regime, n, _fmt(out_red, 100, 0) + " %", _fmt(in_red, 100, 0) + " %", _fmt(in_diff)))
    return md


def main(argv=None):
    ap = argparse.ArgumentParser(description="EXPLORATORY learner pilot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r_ = sub.add_parser("run")
    r_.add_argument("--pairs", default="constructed,independent")
    r_.add_argument("--arms", default=",".join(ARMS + MF_ARMS))
    r_.add_argument("--reps", type=int, default=20)
    r_.add_argument("--r", type=int, default=1)
    r_.add_argument("--transform", default="slog", choices=("slog", "raw"))
    r_.add_argument("--budget-s", type=float, default=540.0)
    p = sub.add_parser("report")
    p.add_argument("--r", type=int, default=1)
    p.add_argument("--transform", default="slog", choices=("slog", "raw"))
    a = ap.parse_args(argv)
    if a.cmd == "run":
        n = run_all(a.pairs.split(","), a.arms.split(","), a.reps, a.r, a.transform, a.budget_s)
        print("wrote %d runs to %s" % (n, STORE))
    else:
        report(a.r, a.transform)
    return 0


if __name__ == "__main__":
    sys.exit(main())
