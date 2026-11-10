"""EXPLORATORY: can a COORDINATED health beacon restore the beacon's benefit at N >= 5?

nscale.py (frozen, not edited) showed that plain HEALTH (one 12 B death frame per peer) is
harmful at N = 10.  Hypothesis (untested): all N-1 survivors learn of the death at once, all
re-plan greedily on the same belief and rush into the dead agent's area.

New protocols (same death frame, NO extra messages; receivers only change how they read it):

  HEALTH_PART[:rule]  the survivors split the dead agent's orphaned nominal tasks (those with
                      start >= death stamp) by a deterministic common-knowledge rule computed
                      from the default plan.  Receiver a drops from its belief ONLY the orphans
                      assigned to a; orphans assigned to another survivor stay counted as
                      covered (as under IMPLICIT), so only the assignee goes for each cell.
      near     nearest survivor (Manhattan on grid coordinates) to the orphan's cell, measured
               from the survivor's default-plan position at the orphan's start; ties -> lighter
               load, then lower index.
      nearbal  as near, but each survivor takes at most ceil(M / survivors) orphans.
      rr       round robin in priority (= nominal) order over the survivors, cyclic from the
               first index above the dead agent.
  HEALTH_ONE[:rule]   a single survivor takes ALL the orphans; the others ignore the death.
      low      lowest-index survivor.
      near     survivor whose default-plan position at the stamp is nearest the dead agent's.

Diagnostics (plain HEALTH vs IMPLICIT, ...): episodes run the full horizon with the target
removed (Truth.target = -1: only false hints, no detection) so there is no early-stop
truncation; duplicated looks are counted from the decision log (see diag_metrics).

Tune ONLY on dev seeds 1000-1015; evaluate ONCE on fresh seeds 6000-6015 (refused without
results/nbeacon_frozen.json, and refused if the output already exists).

CLI (always under fsrun):
  cd Tools && fsrun python3 -m ForestSearch.nbeacon dev --E 20 --out-dir /tmp/nbeacon
  cd Tools && fsrun python3 -m ForestSearch.nbeacon fresh --E 60
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import os
import sys
import time

import numpy as np

from .model import Truth
from .nscale import (FIELDS, LINKS, NSEpisode, boot_means, build, fault_spec, inst_t, load,
                     make_strategies, nominal_plan_ns, t_cdf)

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results")

PART_RULES = ("near", "nearbal", "rr")
ONE_RULES = ("low", "near")
# Frozen after the dev run (results/nbeacon_frozen.json); defaults only used for dev / tests.
DEFAULT_PART, DEFAULT_ONE = "near", "low"
NS = (3, 5, 10)
TRAVELS = (0, 1)
# horizons from PREREG_SCALING.md Amendment 1 (dev rule): travel 1 -> 24 (N<=5), 32 (N=10);
# travel 0 -> 12 (N<=5), 16 (N=10)
HZ = {(3, 1): 24, (5, 1): 24, (10, 1): 32, (3, 0): 12, (5, 0): 12, (10, 0): 16,
      (2, 1): 24, (2, 0): 12}
CELL_LINKS = ("ideal", "burst30_d1")


def proto_spec(name):
    """'HEALTH_PART:near' -> ('part', 'near'); 'HEALTH' -> (None, None) base protocols."""
    if name.startswith("HEALTH_PART"):
        return "part", (name.split(":")[1] if ":" in name else DEFAULT_PART)
    if name.startswith("HEALTH_ONE"):
        return "one", (name.split(":")[1] if ":" in name else DEFAULT_ONE)
    return None, None


def base_name(name):
    return "HEALTH" if proto_spec(name)[0] else name


# ---------------------------------------------------------------- assignment rules
def _pos_at(plan, start_cell, t):
    cell = start_cell
    for tk in plan:
        if tk.finish <= t:
            cell = tk.cell
        else:
            break
    return cell


def _man(coords, a, b):
    return int(abs(coords[a, 0] - coords[b, 0]) + abs(coords[a, 1] - coords[b, 1]))


def assign_orphans(inst, nominal, starts, dead, td, survivors, variant, rule):
    """Deterministic common-knowledge map {orphan task seq -> taking survivor}.

    Depends only on the default plan, the stamp and the survivor set."""
    surv = sorted(survivors)
    orph = [tk for tk in nominal[dead] if tk.start >= td]
    if not surv or not orph:
        return {}
    if variant == "one":
        if rule == "low":
            who = surv[0]
        elif rule == "near":
            pd_ = _pos_at(nominal[dead], starts[dead], td)
            who = min(surv, key=lambda s: (_man(inst.coords, _pos_at(nominal[s], starts[s], td), pd_), s))
        else:
            raise ValueError(rule)
        return {tk.seq: who for tk in orph}
    out, load_ = {}, {s: 0 for s in surv}
    if rule == "rr":
        rot = [s for s in surv if s > dead] + [s for s in surv if s < dead]
        for k, tk in enumerate(orph):
            out[tk.seq] = rot[k % len(rot)]
        return out
    cap = math.ceil(len(orph) / len(surv)) if rule == "nearbal" else 10 ** 9
    if rule not in ("near", "nearbal"):
        raise ValueError(rule)
    for tk in orph:
        cand = [s for s in surv if load_[s] < cap]
        who = min(cand, key=lambda s: (_man(inst.coords, _pos_at(nominal[s], starts[s], tk.start), tk.cell),
                                       load_[s], s))
        out[tk.seq] = who
        load_[who] += 1
    return out


# ---------------------------------------------------------------- episode
class NBEpisode(NSEpisode):
    """NSEpisode plus coordinated reading of the death frame (variant 'part' / 'one') and a
    decision log.  variant=None is plain HEALTH / IMPLICIT / ORACLE, bit-identical to NSEpisode."""

    def __init__(self, inst, strats, link, rng, nominal=None, fault=None, variant=None,
                 rule=None, diag=False):
        super().__init__(inst, strats, link, rng, nominal, fault=fault)
        self.variant, self.rule = variant, rule
        self.decisions = []
        self._amap = {}
        if diag:
            self.truth = Truth(-1, self.truth.good)       # no target: no detection, full horizon

    def _choose(self, a, t):
        c, v = NSEpisode._choose(self, a, t)
        self.decisions.append((t, a.idx, c, v))
        return c, v

    def _assignment(self, dead, td, known_dead):
        k = (dead, td, known_dead)
        if k not in self._amap:
            surv = [s for s in range(self.N) if s not in known_dead]
            self._amap[k] = assign_orphans(self.inst, self.nominal, self.starts, dead, td, surv,
                                           self.variant, self.rule)
        return self._amap[k]

    def _partner_counts(self, a, t):
        if (self.variant and not self.oracle and self.nominal is not None
                and self.hkind == "beacon" and self.p_dead[a.idx]):
            pd = self.p_dead[a.idx]
            kd = frozenset(pd)
            amap = {j: self._assignment(j, td, kd) for j, td in pd.items()}
            return self._filtered(
                a, t, lambda j, tk: j not in pd or tk.start < pd[j] or amap[j].get(tk.seq) != a.idx)
        return NSEpisode._partner_counts(self, a, t)


def run_nb(inst, N, proto, link, ep, S, nominal, fault=None, diag=False, keep=False):
    f = None
    if fault:
        f = dict(fault)
        f["seed"] = 10_000 + ep
    variant, rule = proto_spec(proto)
    e = NBEpisode(inst, (S[base_name(proto)],) * N, link, np.random.default_rng(ep), nominal,
                  fault=f, variant=variant, rule=rule, diag=diag)
    r = e.run()
    return (r, e) if keep else r


# ---------------------------------------------------------------- diagnostics
DIAG_FIELDS = ["n_orph", "n_post", "dup_post", "orph_dec", "orph_dup", "orph_cov", "own_missed",
               "own_total", "first_orph", "n_surv", "orph_coll", "distinct_post"]


def diag_metrics(ep, nominal, H):
    """Per-episode duplication metrics after the (single) death.  (c, v) = a look target.

      n_post        survivor decisions at t >= td
      dup_post      post-death survivor decisions whose (c, v) was already chosen earlier by ANY
                    agent whose look is valid (survivors at any time, the dead agent before td)
      orph_dec      post-death survivor decisions on an orphan (c, v) (dead's nominal task with
                    start >= td)
      orph_dup      orph_dec minus the number of DISTINCT orphan (c, v) chosen  (= wasted repeats)
      orph_cov      distinct orphan (c, v) chosen post-death
      own_missed    survivors' own nominal tasks (start in [td, H)) whose (c, v) nobody chose post-death
      own_total     all such own tasks
      first_orph    survivors whose first post-death decision is an orphan (c, v)  (rush onset)
      orph_coll     same-step same-(c, v) survivor pairs on orphans
      distinct_post distinct (c, v) among post-death survivor decisions
    """
    dead = [i for i in range(ep.N) if ep.t_dead[i] is not None]
    if not dead:
        return None
    i = dead[0]
    td = ep.t_dead[i]
    surv = [s for s in range(ep.N) if s != i]
    orph = {(tk.cell, tk.view) for tk in nominal[i] if tk.start >= td}
    claimed, post = set(), []
    for (t, s, c, v) in sorted(ep.decisions):
        valid = (s != i) or t < td
        if s != i and t >= td:
            post.append((t, s, c, v, (c, v) in claimed))
        if valid:
            claimed.add((c, v))
    n_post = len(post)
    dup_post = sum(1 for p in post if p[4])
    ocv = [(p[2], p[3]) for p in post if (p[2], p[3]) in orph]
    dist_post = {(p[2], p[3]) for p in post}
    own = [(tk.cell, tk.view) for s in surv for tk in nominal[s] if td <= tk.start < H]
    first = {}
    for p in post:
        first.setdefault(p[1], (p[2], p[3]))
    coll = {}
    for p in post:
        if (p[2], p[3]) in orph:
            coll[(p[0], p[2], p[3])] = coll.get((p[0], p[2], p[3]), 0) + 1
    return dict(n_orph=len(orph), n_post=n_post, dup_post=dup_post, orph_dec=len(ocv),
                orph_dup=len(ocv) - len(set(ocv)), orph_cov=len(set(ocv)),
                own_missed=sum(1 for x in own if x not in dist_post), own_total=len(own),
                first_orph=sum(1 for s in surv if first.get(s) in orph), n_surv=len(surv),
                orph_coll=sum(k * (k - 1) // 2 for k in coll.values()), distinct_post=len(dist_post))


# ---------------------------------------------------------------- grid runners
def _write(path, header, rows):
    with gzip.open(path, "wt", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


def run_main(seeds, Ns, travels, protos, E, out, links=CELL_LINKS, faults=("none", "perm1"), lam=1):
    """Same row format / CRN as nscale.run_grid (episode seed = ep, fault seed = 10000 + ep)."""
    t0 = time.time()
    S = make_strategies("greedy")
    rows = []
    LF = {"ORACLE", "INDEP", "IMPLICIT"}
    for N in Ns:
        for travel in travels:
            for seed in seeds:
                inst = build(N, seed, lam, travel, HZ[(N, travel)])
                H = inst.horizon
                nom = nominal_plan_ns(inst, N, S)
                for fname in faults:
                    fs = fault_spec(fname, H, N)
                    for p in protos:
                        lks = [links[0]] if (p in LF or fs is None) else list(links)
                        for ln in lks:
                            for ep in range(E):
                                r = run_nb(inst, N, p, LINKS[ln], ep, S, nom, fs)
                                rec = (N, travel, lam, "greedy", fname, ln, p, seed, ep,
                                       int(r.detected), r.ttd, H, r.bytes_sent, r.collisions)
                                rows.append(rec)
                                if len(lks) == 1:
                                    for ln2 in links[1:]:
                                        rows.append(rec[:5] + (ln2,) + rec[6:])
                print(f"[{time.time()-t0:7.0f}s] main N={N} travel={travel} seed={seed}",
                      file=sys.stderr, flush=True)
    _write(out, FIELDS, rows)
    print(f"wrote {out} ({len(rows)} rows, {time.time()-t0:.0f}s)", file=sys.stderr)


def run_diag(seeds, Ns, travels, protos, E, out, links=CELL_LINKS, lam=1):
    t0 = time.time()
    S = make_strategies("greedy")
    rows = []
    LF = {"ORACLE", "INDEP", "IMPLICIT"}
    for N in Ns:
        for travel in travels:
            for seed in seeds:
                inst = build(N, seed, lam, travel, HZ[(N, travel)])
                H = inst.horizon
                nom = nominal_plan_ns(inst, N, S)
                fs = fault_spec("perm1", H, N)
                for p in protos:
                    lks = [links[0]] if p in LF else list(links)
                    for ln in lks:
                        for ep in range(E):
                            r, e = run_nb(inst, N, p, LINKS[ln], ep, S, nom, fs, diag=True, keep=True)
                            m = diag_metrics(e, nom, H)
                            if m is None:
                                continue
                            rec = (N, travel, ln, p, seed, ep, e.t_dead[[i for i in range(N) if e.t_dead[i] is not None][0]]) \
                                + tuple(m[k] for k in DIAG_FIELDS)
                            rows.append(rec)
                            if len(lks) == 1:
                                for ln2 in links[1:]:
                                    rows.append(rec[:2] + (ln2,) + rec[3:])
                print(f"[{time.time()-t0:7.0f}s] diag N={N} travel={travel} seed={seed}",
                      file=sys.stderr, flush=True)
    _write(out, ["N", "travel", "link", "proto", "seed", "ep", "td"] + DIAG_FIELDS, rows)
    print(f"wrote {out} ({len(rows)} rows, {time.time()-t0:.0f}s)", file=sys.stderr)


# ---------------------------------------------------------------- analysis
def load_diag(path):
    raw = {}
    with gzip.open(path, "rt") as fh:
        for r in csv.DictReader(fh):
            k = (int(r["N"]), int(float(r["travel"])), r["link"], r["proto"])
            raw.setdefault(k, {}).setdefault(int(r["seed"]), {})[int(r["ep"])] = [float(r[f]) for f in DIAG_FIELDS]
    D = {}
    for k, by in raw.items():
        seeds = sorted(by)
        eps = sorted(set.intersection(*[set(by[s]) for s in seeds]))
        D[k] = np.array([[by[s][e] for e in eps] for s in seeds])       # (S, E, F)
    return D


def pooled_boot(Xs, B, rng):
    """Cluster bootstrap of the mean over a list of (S, E) per-episode arrays (cells weighted
    equally).  Instances (seed index) are resampled JOINTLY across cells, then episodes
    independently within each (cell, instance)."""
    S_ = Xs[0].shape[0]
    out = np.empty(B)
    for b in range(B):
        ss = rng.integers(S_, size=S_)
        acc = 0.0
        for X in Xs:
            E = X.shape[1]
            acc += X[ss[:, None], rng.integers(E, size=(S_, E))].mean()
        out[b] = acc / len(Xs)
    return out


def ci_str(X, B=2000, seed=0):
    rng = np.random.default_rng(seed)
    bs = boot_means(X, B, rng)
    lo, hi = np.percentile(bs, [2.5, 97.5])
    star = "*" if lo > 0 or hi < 0 else ""
    return f"{X.mean():+.3f} [{lo:+.3f},{hi:+.3f}]{star}", float(X.mean()), float(lo), float(hi)


def inst_p(X):
    """Two-sided instance-level t p-value (unadjusted)."""
    m, se, df = inst_t(X.mean(1))
    if se == 0:
        return 0.0 if m != 0 else 1.0
    t = m / se
    return float(2 * (1 - t_cdf(abs(t), df)))


def ctr(D, key, a, b, metric="det"):
    return D[key + (a,)][metric] - D[key + (b,)][metric]


def analyze_main(path, protos, out_lines, B=2000, title=""):
    D = load([path])
    L = out_lines
    keys = sorted({k[:6] for k in D})
    fam = [p for p in protos if p.startswith("HEALTH")]
    L += [f"### {title} per-cell results (lam=1, greedy)", "",
          "P(detect) of IMPLICIT and paired differences vs IMPLICIT, mean [95% cluster-bootstrap CI]; "
          "* = CI excludes 0. p = unadjusted instance-level t.", ""]
    part = [p for p in fam if p.startswith("HEALTH_PART")]
    one = [p for p in fam if p.startswith("HEALTH_ONE")]
    head = ["travel", "fault", "link", "N", "P IMPL", "ORACLE-IMPL", "HEALTH-IMPL"] + \
           [f"{p}-IMPL" for p in part + one] + [f"{p}-HEALTH" for p in part + one]
    L += ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for tr in TRAVELS:
        for fa in ("none", "perm1"):
            for ln in CELL_LINKS:
                for N in NS:
                    k = (N, float(tr), 1.0, "greedy", fa, ln)
                    if k + ("IMPLICIT",) not in D:
                        continue
                    row = [str(tr), fa, ln, str(N), f"{D[k+('IMPLICIT',)]['det'].mean():.3f}"]
                    for p in ["ORACLE", "HEALTH"] + part + one:
                        if k + (p,) in D:
                            row.append(ci_str(ctr(D, k, p, "IMPLICIT"), B)[0])
                        else:
                            row.append("n/a")
                    for p in part + one:
                        row.append(ci_str(ctr(D, k, p, "HEALTH"), B)[0] if k + (p,) in D else "n/a")
                    L.append("| " + " | ".join(row) + " |")
    L.append("")
    return D


def pooled_table(D, protos, L, B=2000, title=""):
    """Pooled over the 4 perm1 cells (travel x link) per N, equal weights."""
    part = [p for p in protos if p.startswith("HEALTH_PART")]
    one = [p for p in protos if p.startswith("HEALTH_ONE")]
    L += [f"### {title} perm1 pooled over travel {{0,1}} x link {{ideal, burst30_d1}} (4 cells, equal weight)", "",
          "Mean paired difference [95% bootstrap CI; instances resampled jointly across the 4 cells]. "
          "Healthy cells are not pooled in (the beacon protocols are silent there; see the healthy rows above).", ""]
    cols = ["ORACLE-IMPL", "HEALTH-IMPL"] + [f"{p}-IMPL" for p in part + one] + \
           [f"{p}-HEALTH" for p in part + one]
    L += ["| N | " + " | ".join(cols) + " | recovery of ORACLE gain (PART) |", "|" + "---|" * (len(cols) + 2)]
    res = {}
    for N in NS:
        cells = [(N, float(tr), 1.0, "greedy", "perm1", ln) for tr in TRAVELS for ln in CELL_LINKS]
        if any(c + ("IMPLICIT",) not in D for c in cells):
            continue
        row = [str(N)]
        rng = np.random.default_rng(7 + N)
        for (a, b) in [("ORACLE", "IMPLICIT"), ("HEALTH", "IMPLICIT")] + [(p, "IMPLICIT") for p in part + one] + \
                [(p, "HEALTH") for p in part + one]:
            Xs = [ctr(D, c, a, b) for c in cells]
            bs = pooled_boot(Xs, B, rng)
            m = float(np.mean([X.mean() for X in Xs]))
            lo, hi = np.percentile(bs, [2.5, 97.5])
            res[(N, a, b)] = (m, float(lo), float(hi))
            row.append(f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]{'*' if lo > 0 or hi < 0 else ''}")
        o = res[(N, "ORACLE", "IMPLICIT")][0]
        rec = [f"{res[(N, p, 'IMPLICIT')][0]/o:.2f}" if abs(o) > 1e-9 else "n/a" for p in part[:1]]
        row += rec
        L.append("| " + " | ".join(row) + " |")
    L.append("")
    return res


def healthy_check(D, protos, L):
    bad = 0
    n = 0
    for k in sorted({k[:6] for k in D}):
        if k[4] != "none":
            continue
        for p in protos:
            if p.startswith("HEALTH") and k + (p,) in D and k + ("IMPLICIT",) in D:
                n += 1
                if not np.array_equal(D[k + (p,)]["det"], D[k + ("IMPLICIT",)]["det"]):
                    bad += 1
    L += [f"Healthy-cell check: {n - bad}/{n} (cell, protocol) pairs have detections identical to IMPLICIT "
          "episode for episode (beacon silent when nobody dies).", ""]


DIAG_SHOW = [("n_post", "decisions after death"), ("dup_post", "dup looks (any)"),
             ("orph_dec", "looks on orphan (c,v)"), ("orph_dup", "dup looks on orphans"),
             ("orph_cov", "distinct orphans covered"), ("own_missed", "own nominal tasks skipped"),
             ("first_orph", "survivors rushing to orphan first"), ("orph_coll", "same-step orphan collisions")]


def diag_tables(DG, protos, L, B=1000):
    fi = {f: i for i, f in enumerate(DIAG_FIELDS)}
    L += ["### Duplicated-effort diagnostic (perm1, target removed, full horizon)", "",
          "Per-episode counts after the death, averaged over episodes and instances; per survivor = / (N-1). "
          "Pooled over travel {0,1} x link {ideal, burst30_d1} (4 cells, equal weight); orphans = the dead agent's "
          "nominal tasks with start >= death stamp.", ""]
    for N in NS:
        cells = [(N, tr, ln) for tr in TRAVELS for ln in CELL_LINKS]
        ps = [p for p in protos if all(c + (p,) in DG for c in cells)]
        if not ps:
            continue
        no = np.mean([DG[c + (ps[0],)][..., fi["n_orph"]].mean() for c in cells])
        ns = N - 1
        L += [f"**N = {N}** ({ns} survivors; mean orphan tasks per episode = {no:.2f})", "",
              "| metric (per survivor) | " + " | ".join(ps) + " |", "|---|" + "---|" * len(ps)]
        for f, lab in DIAG_SHOW:
            row = [lab]
            for p in ps:
                Xs = [DG[c + (p,)][..., fi[f]] / (ns if f not in ("orph_cov", "orph_coll") else 1) for c in cells]
                bs = pooled_boot(Xs, B, np.random.default_rng(3))
                m = float(np.mean([X.mean() for X in Xs]))
                lo, hi = np.percentile(bs, [2.5, 97.5])
                row.append(f"{m:.3f} [{lo:.3f},{hi:.3f}]")
            L.append("| " + " | ".join(row) + " |")
        L.append("")
        # paired HEALTH - IMPLICIT on dup metrics
        if "HEALTH" in ps and "IMPLICIT" in ps:
            row = []
            for f in ("dup_post", "orph_dup", "own_missed"):
                Xs = [(DG[c + ("HEALTH",)][..., fi[f]] - DG[c + ("IMPLICIT",)][..., fi[f]]) / ns for c in cells]
                bs = pooled_boot(Xs, B, np.random.default_rng(5))
                m = float(np.mean([X.mean() for X in Xs]))
                lo, hi = np.percentile(bs, [2.5, 97.5])
                row.append(f"{f}: {m:+.3f} [{lo:+.3f},{hi:+.3f}]{'*' if lo > 0 or hi < 0 else ''}")
            L += ["Paired HEALTH - IMPLICIT per survivor: " + "; ".join(row), ""]
    return L


# ---------------------------------------------------------------- commands
def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def cmd_dev(a):
    os.makedirs(a.out_dir, exist_ok=True)
    seeds = list(range(1000, 1016))
    protos = ["ORACLE", "IMPLICIT", "HEALTH"] + [f"HEALTH_PART:{r}" for r in PART_RULES] + \
             [f"HEALTH_ONE:{r}" for r in ONE_RULES]
    Ns = [int(x) for x in a.Ns.split(",")]
    run_main(seeds, Ns, TRAVELS, protos, a.E, os.path.join(a.out_dir, "nbeacon_dev_main.csv.gz"),
             faults=("perm1",))
    run_diag(seeds, Ns, TRAVELS, ["IMPLICIT", "HEALTH", f"HEALTH_PART:{DEFAULT_PART}",
                                  f"HEALTH_ONE:{DEFAULT_ONE}"], a.Ed,
             os.path.join(a.out_dir, "nbeacon_dev_diag.csv.gz"))


def cmd_fresh(a):
    fr = os.path.join(RES, "nbeacon_frozen.json")
    if not os.path.exists(fr):
        sys.exit("fresh seeds 6000-6015 need results/nbeacon_frozen.json (written after dev)")
    F = json.load(open(fr))
    out = os.path.join(RES, "data", "nbeacon_fresh_main.csv.gz")
    outd = os.path.join(RES, "data", "nbeacon_fresh_diag.csv.gz")
    if os.path.exists(out) or os.path.exists(outd):
        sys.exit("fresh output exists: the fresh seeds are used once only")
    seeds = list(range(6000, 6016))
    protos = ["ORACLE", "IMPLICIT", "HEALTH", f"HEALTH_PART:{F['part']}", f"HEALTH_ONE:{F['one']}"]
    run_main(seeds, NS, TRAVELS, protos, a.E, out + ".part")
    os.replace(out + ".part", out)
    run_diag(seeds, NS, TRAVELS, [p for p in protos if p != "ORACLE"], a.Ed, outd + ".part")
    os.replace(outd + ".part", outd)


def cmd_analyze(a):
    L = [f"# {a.title}", ""]
    D = analyze_main(a.main, a.protos.split(","), L, title=a.title)
    healthy_check(D, a.protos.split(","), L)
    pooled_table(D, a.protos.split(","), L, title=a.title)
    if a.diag:
        diag_tables(load_diag(a.diag), a.dprotos.split(","), L)
    open(a.out, "w").write("\n".join(L) + "\n")
    print("\n".join(L))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dev")
    d.add_argument("--out-dir", default="/tmp/nbeacon")
    d.add_argument("--E", type=int, default=20)
    d.add_argument("--Ed", type=int, default=6)
    d.add_argument("--Ns", default="3,5,10")
    f = sub.add_parser("fresh")
    f.add_argument("--E", type=int, default=60)
    f.add_argument("--Ed", type=int, default=10)
    z = sub.add_parser("analyze")
    z.add_argument("--main", required=True)
    z.add_argument("--diag", default=None)
    z.add_argument("--protos", required=True)
    z.add_argument("--dprotos", default="IMPLICIT,HEALTH")
    z.add_argument("--title", default="nbeacon")
    z.add_argument("--out", required=True)
    a = ap.parse_args()
    {"dev": cmd_dev, "fresh": cmd_fresh, "analyze": cmd_analyze}[a.cmd](a)


if __name__ == "__main__":
    main()
