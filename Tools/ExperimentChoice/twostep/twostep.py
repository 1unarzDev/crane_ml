#!/usr/bin/env python3
"""Exactly solvable toy: sequential (course, tier) experiment selection.

When does the optimal adaptive policy beat strong short-horizon rules?

Model
-----
* n <= 6 courses x = 0..n-1 along a 1-D parameter, course weights p(x).
* Two configurations A/B.  Latent reversal boundary b in {0..n}: the true
  full-tier paired gap has sign +1 (A better) for x < b and -1 for x >= b,
  magnitude mag[x].
* Binary latent m ("cheap tier omits a mechanism"), prior P(m=1) = q.  On
  courses of type V the cheap tier's sign is inverted when m = 1.
* Query (x, full): cost 1, returns sign flipped w.p. e_hi; repeatable (seeds).
* Query (x, cheap): cost c_lo, returns the cheap-tier sign flipped w.p. e_lo.
  At most once per course: the cheap tier is a deterministic model, so its
  "noise" is per-course model error and re-running it returns the same answer.
* Budget: total cost.  Terminal loss = sum_x p(x) mag[x] 1[wrong recommendation].

All values are exact expectations (no Monte Carlo).  Beliefs are kept as
unnormalised joint weights w[h] = prior[h] * likelihood(history | h), so the
expectimax recursion is V(s) = min_a V(s, a, +) + V(s, a, -) with no
normalisation.  A belief state is identified by (cheap outcome per course,
full-tier +/- counts per course), which is a sufficient statistic.

Costs are integers in thousandths of a full-tier unit.
"""
import itertools
import math
import sys
import time

import numpy as np

FULL = 1000
EPS = 1e-12


class Instance:
    def __init__(self, n, c_lo, budget, e_lo, e_hi, q, V=None, prior_b=None,
                 p=None, mag=None):
        self.n = n
        self.c = int(round(c_lo * FULL))
        self.budget = int(round(budget * FULL))
        self.e_lo, self.e_hi, self.q = float(e_lo), float(e_hi), float(q)
        self.V = tuple(sorted(V)) if V is not None else tuple(range(n // 2, n))
        self.prior_b = (np.full(n + 1, 1.0 / (n + 1)) if prior_b is None
                        else np.asarray(prior_b, float))
        self.p = np.full(n, 1.0 / n) if p is None else np.asarray(p, float)
        self.mag = np.ones(n) if mag is None else np.asarray(mag, float)

    def label(self):
        f = lambda a: "[" + ",".join("%.3g" % v for v in a) + "]"
        return ("n=%d c_lo=%g budget=%g e_lo=%.3g e_hi=%.3g q=%.3g V=%s "
                "prior_b=%s p=%s mag=%s" % (
                    self.n, self.c / FULL, self.budget / FULL, self.e_lo,
                    self.e_hi, self.q, list(self.V), f(self.prior_b),
                    f(self.p), f(self.mag)))


class Solver:
    def __init__(self, inst):
        self.inst = inst
        n = self.n = inst.n
        # hypotheses h = (b, m)
        hyp = [(b, m) for b in range(n + 1) for m in (0, 1)]
        H = len(hyp)
        prior = np.array([inst.prior_b[b] * (inst.q if m else 1 - inst.q)
                          for b, m in hyp])
        sgn = np.array([[1 if x < b else -1 for x in range(n)] for b, m in hyp])
        csgn = sgn.copy()
        for h, (b, m) in enumerate(hyp):
            if m:
                for x in inst.V:
                    csgn[h, x] *= -1
        # likelihood of outcome '+' per hypothesis
        Lf = np.where(sgn > 0, 1 - inst.e_hi, inst.e_hi)      # (H, n)
        Lc = np.where(csgn > 0, 1 - inst.e_lo, inst.e_lo)
        wx = inst.p * inst.mag
        Mp = (sgn > 0) * wx[None, :]   # loss weight if truth is + (recommend B wrong)
        Mm = (sgn < 0) * wx[None, :]
        self.wx = wx

        # cheap status: base-3 digits (0 unqueried, 1 '+', 2 '-')
        C = self.C = 3 ** n
        self.pow3 = [3 ** x for x in range(n)]
        LC = np.ones((C, H))
        self.nch = [0] * C
        for cs in range(C):
            for x in range(n):
                d = (cs // self.pow3[x]) % 3
                if d == 1:
                    LC[cs] *= Lc[:, x]
                elif d == 2:
                    LC[cs] *= 1 - Lc[:, x]
                if d:
                    self.nch[cs] += 1
        # full status: counts (k+_x, k-_x), total <= K
        K = self.K = inst.budget // FULL
        cfgs = []

        def rec(i, left, cur):
            if i == 2 * n:
                cfgs.append(tuple(cur))
                return
            for k in range(left + 1):
                cur.append(k)
                rec(i + 1, left - k, cur)
                cur.pop()
        rec(0, K, [])
        idx = {cf: i for i, cf in enumerate(cfgs)}
        F = self.F = len(cfgs)
        self.cfgs = cfgs
        LF = np.ones((F, H))
        self.nf = [sum(cf) for cf in cfgs]
        self.fnext = []
        for i, cf in enumerate(cfgs):
            for x in range(n):
                if cf[2 * x]:
                    LF[i] *= Lf[:, x] ** cf[2 * x]
                if cf[2 * x + 1]:
                    LF[i] *= (1 - Lf[:, x]) ** cf[2 * x + 1]
            row = []
            for x in range(n):
                if self.nf[i] < K:
                    a = list(cf); a[2 * x] += 1
                    b = list(cf); b[2 * x + 1] += 1
                    row.append((idx[tuple(a)], idx[tuple(b)]))
                else:
                    row.append(None)
            self.fnext.append(row)
        W = prior[None, None, :] * LC[:, None, :] * LF[None, :, :]
        U = np.minimum(W @ Mp, W @ Mm)          # (C, F, n) per-course expected loss
        self.U = U.reshape(C * F, n)
        self.T2 = U.sum(-1)                     # (C, F)
        self.term = self.T2.reshape(-1).tolist()
        self.mass = W.sum(-1).reshape(-1).tolist()
        self.prior_loss = self.term[0]
        self._kids = {}
        self._memo = {}

    # ---- state graph -------------------------------------------------
    def rem(self, s):
        cs, fs = divmod(s, self.F)
        return self.inst.budget - self.nch[cs] * self.inst.c - FULL * self.nf[fs]

    def kids(self, s):
        """Affordable actions: (a, cost, state_if_plus, state_if_minus);
        a < n is cheap@a, a >= n is full@(a-n).  Ordered cheap first."""
        k = self._kids.get(s)
        if k is not None:
            return k
        k = []
        if self.mass[s] > 0.0:
            n, F, c = self.n, self.F, self.inst.c
            cs, fs = divmod(s, F)
            r = self.inst.budget - self.nch[cs] * c - FULL * self.nf[fs]
            if r >= c:
                for x in range(n):
                    p3 = self.pow3[x]
                    if (cs // p3) % 3 == 0:
                        k.append((x, c, s + p3 * F, s + 2 * p3 * F))
            if r >= FULL:
                base = cs * F
                fn = self.fnext[fs]
                for x in range(n):
                    k.append((n + x, FULL, base + fn[x][0], base + fn[x][1]))
        self._kids[s] = k
        return k

    def aname(self, a):
        return "cheap@%d" % a if a < self.n else "full@%d" % (a - self.n)

    def _m(self, name):
        return self._memo.setdefault(name, {})

    # ---- optimal (backward induction over belief states) --------------
    def v_opt(self, s):
        m = self._m("opt")
        v = m.get(s)
        if v is None:
            k = self.kids(s)
            if not k:
                v = self.term[s]
            else:
                v = min(self.v_opt(sp) + self.v_opt(sm) for _, _, sp, sm in k)
            m[s] = v
        return v

    def act_opt(self, s):
        best, ba = None, None
        for kid in self.kids(s):
            v = self.v_opt(kid[2]) + self.v_opt(kid[3])
            if best is None or v < best - EPS:
                best, ba = v, kid
        return ba

    # ---- one-step greedy: expected loss reduction per unit cost --------
    def act_greedy(self, s):
        m = self._m("ag")
        if s in m:
            return m[s]
        t = self.term
        best, ba = -1.0, None
        for kid in self.kids(s):      # cheap first => ties go to the cheaper query
            r = (t[s] - t[kid[2]] - t[kid[3]]) * FULL / kid[1]
            if r > best + 1e-9:
                best, ba = r, kid
        m[s] = ba
        return ba

    # ---- k-step lookahead with greedy rollout --------------------------
    def look(self, k, s):
        """Loss of: k optimally chosen (adaptive) queries, then greedy to the
        end of the budget.  look(0, .) is the greedy policy value."""
        if k == 0:
            return self.evaluate("greedy", self.act_greedy, s)
        m = self._m(("L", k))
        v = m.get(s)
        if v is None:
            kd = self.kids(s)
            if not kd:
                v = self.term[s]
            else:
                v = min(self.look(k - 1, sp) + self.look(k - 1, sm)
                        for _, _, sp, sm in kd)
            m[s] = v
        return v

    def act_look(self, k):
        def act(s):
            best, ba = None, None
            for kid in self.kids(s):
                v = self.look(k - 1, kid[2]) + self.look(k - 1, kid[3])
                if best is None or v < best - EPS:
                    best, ba = v, kid
            return ba
        return act

    # ---- cheap screen first, then escalate the most uncertain ----------
    def _screen(self, s):
        """Next cheap query of the unconditional screen (keeps one full unit
        in reserve if a full query is still affordable), else None."""
        kd = self.kids(s)
        r = self.rem(s)
        cheap = [kid for kid in kd if kid[0] < self.n]
        if cheap and (r - self.inst.c >= FULL or r < FULL):
            return max(cheap, key=lambda kid: (self.wx[kid[0]], -kid[0]))
        return None

    def act_cheapfirst(self, s):
        kd = self.kids(s)
        if not kd:
            return None
        a = self._screen(s)
        if a is not None:
            return a
        full = [kid for kid in kd if kid[0] >= self.n]
        if not full:
            return kd[0]
        u = self.U[s]
        return max(full, key=lambda kid: (round(u[kid[0] - self.n], 12),
                                          self.wx[kid[0] - self.n], -kid[0]))

    def act_screen_then(self, base):
        def act(s):
            a = self._screen(s)
            return a if a is not None else base(s)
        return act

    # ---- exact policy evaluation ---------------------------------------
    def evaluate(self, name, act, s=0):
        m = self._m(("ev", name))
        v = m.get(s)
        if v is None:
            kid = act(s) if self.kids(s) else None
            if kid is None:
                v = self.term[s]
            else:
                v = self.evaluate(name, act, kid[2]) + self.evaluate(name, act, kid[3])
            m[s] = v
        return v

    def v_random(self, s=0):
        m = self._m("rand")
        v = m.get(s)
        if v is None:
            kd = self.kids(s)
            if not kd:
                v = self.term[s]
            else:
                v = sum(self.v_random(sp) + self.v_random(sm)
                        for _, _, sp, sm in kd) / len(kd)
            m[s] = v
        return v

    # ---- best non-adaptive design --------------------------------------
    def best_fixed(self):
        n, c, B = self.n, self.inst.c, self.inst.budget
        # full designs grouped by per-course counts
        groups = {}
        for i, cf in enumerate(self.cfgs):
            mt = tuple(cf[2 * x] + cf[2 * x + 1] for x in range(n))
            coef = 1.0
            for x in range(n):
                coef *= math.comb(mt[x], cf[2 * x])
            g = groups.setdefault(mt, ([], []))
            g[0].append(i); g[1].append(coef)
        groups = {k: (np.array(v[0]), np.array(v[1])) for k, v in groups.items()}
        best, bd = None, None
        for j in range(n + 1):
            if j * c > B:
                break
            Kj = (B - j * c) // FULL
            fulls = [(mt, g) for mt, g in groups.items() if sum(mt) == Kj]
            for S in itertools.combinations(range(n), j):
                csl = [0]
                for x in S:
                    csl = [v + d * self.pow3[x] for v in csl for d in (1, 2)]
                rows = self.T2[np.array(csl)]
                for mt, (fi, coef) in fulls:
                    v = float((rows[:, fi] @ coef).sum())
                    if best is None or v < best - EPS:
                        best, bd = v, (S, mt)
        return best, bd

    # ---- everything -----------------------------------------------------
    def all_values(self, deep=3):
        r = {"prior": self.prior_loss, "opt": self.v_opt(0)}
        r["greedy"] = self.evaluate("greedy", self.act_greedy)
        for k in range(1, deep + 1):
            r["look%d" % k] = self.evaluate("look%d" % k, self.act_look(k))
        r["fixed"] = self.best_fixed()[0]
        r["cheapfirst"] = self.evaluate("cheapfirst", self.act_cheapfirst)
        r["screen+greedy"] = self.evaluate(
            "sg", self.act_screen_then(self.act_greedy))
        r["random"] = self.v_random()
        return r

    # ---- explanation ------------------------------------------------------
    def describe(self, out=print, depth=3):
        t = self.term
        out("    " + self.inst.label())
        r = self.all_values()
        out("    losses: " + "  ".join("%s=%.5f" % kv for kv in r.items()))
        out("    one-step gains at the root (abs, per unit cost): " + ", ".join(
            "%s:%.4f/%.3f" % (self.aname(k[0]), t[0] - t[k[2]] - t[k[3]],
                              (t[0] - t[k[2]] - t[k[3]]) * FULL / k[1])
            for k in self.kids(0)))
        for nm, act in (("optimal", self.act_opt), ("greedy", self.act_greedy),
                        ("two-step", self.act_look(2)),
                        ("three-step", self.act_look(3))):
            out("    %s plan:" % nm)
            self._tree(act, 0, depth, out, "      ")

    def _tree(self, act, s, depth, out, ind):
        if depth == 0 or not self.kids(s):
            return
        kid = act(s)
        if kid is None:
            return
        g = self.term[s] - self.term[kid[2]] - self.term[kid[3]]
        out("%s%s (immediate gain %.4f)" % (ind, self.aname(kid[0]), g))
        for sym, c in (("+", kid[2]), ("-", kid[3])):
            if self.mass[c] > 0 and self.kids(c) and depth > 1:
                out("%s  saw %s (prob %.3f, loss now %.4f):" % (
                    ind, sym, self.mass[c] / self.mass[s], self.term[c] / self.mass[c]))
                self._tree(act, c, depth - 1, out, ind + "    ")


# ======================================================================
# sweeps
# ======================================================================
POL = ["greedy", "look1", "look2", "look3", "screen+greedy", "cheapfirst",
       "fixed", "random"]
C_LOS = [0.5, 0.2, 0.05, 0.001]


def solve(inst):
    return Solver(inst).all_values()


def grid_instances(ns=(3, 4), Bs=(2, 3)):
    for n in ns:
        for B in Bs:
            for c in C_LOS:
                for mode in ("hard", "slack"):
                    for e_lo in (0.0, 0.2):
                        for q in (0.0, 0.2, 0.5):
                            for e_hi in (0.0, 0.1):
                                bud = B + (n * c if mode == "slack" else 0)
                                yield (dict(n=n, B=B, c_lo=c, mode=mode, e_lo=e_lo,
                                            q=q, e_hi=e_hi),
                                       Instance(n, c, bud, e_lo, e_hi, q))


def random_instance(rng):
    n = int(rng.choice([3, 3, 4, 4, 5]))
    B = 2 if n == 5 else int(rng.choice([2, 3, 4] if n == 3 else [2, 3]))
    c = float(rng.choice(C_LOS))
    mode = str(rng.choice(["hard", "slack"]))
    V = [x for x in range(n) if rng.random() < 0.5]
    q = 0.0 if rng.random() < 0.2 else float(rng.uniform(0, 0.6))
    e_lo = float(rng.uniform(0, 0.4))
    e_hi = 0.0 if rng.random() < 0.3 else float(rng.uniform(0, 0.3))
    bud = B + (n * c if mode == "slack" else 0)
    inst = Instance(n, c, bud, e_lo, e_hi, q, V=V,
                    prior_b=rng.dirichlet(np.ones(n + 1)),
                    p=rng.dirichlet(2 * np.ones(n)), mag=rng.uniform(0.2, 1, n))
    return dict(n=n, B=B, c_lo=c, mode=mode, e_lo=e_lo, q=q, e_hi=e_hi), inst


def gapstats(rows, pol):
    a = np.array([r[1][pol] - r[1]["opt"] for r in rows])
    f = np.array([(r[1][pol] - r[1]["opt"]) / r[1]["prior"] for r in rows])
    return a, f


def table(rows, key, title, out, pols=("greedy", "look2", "look3")):
    out("\n**%s** (regret = policy loss - optimal loss; `frac` = regret / prior loss)\n" % title)
    hdr = "| %s | N |" % key
    for p in pols:
        hdr += " %s mean abs | %s mean frac | %s max frac |" % (p, p, p)
    out(hdr)
    out("|" + "---|" * (2 + 3 * len(pols)))
    levels = sorted({r[0][key] for r in rows}, key=lambda v: (str(type(v)), v))
    for lv in levels:
        sub = [r for r in rows if r[0][key] == lv]
        line = "| %s | %d |" % (lv, len(sub))
        for p in pols:
            a, f = gapstats(sub, p)
            line += " %.5f | %.2f%% | %.2f%% |" % (a.mean(), 100 * f.mean(), 100 * f.max())
        out(line)


def pct_table(rows, title, out):
    out("\n**%s**\n" % title)
    out("| policy | median frac | mean frac | p90 frac | p99 frac | max frac | "
        "share with frac > 1% | p90 abs | max abs |")
    out("|---|---|---|---|---|---|---|---|---|")
    for p in POL:
        a, f = gapstats(rows, p)
        out("| %s | %.3f%% | %.3f%% | %.3f%% | %.3f%% | %.2f%% | %.1f%% | %.5f | %.5f |" % (
            p, 100 * np.median(f), 100 * f.mean(), 100 * np.quantile(f, .9),
            100 * np.quantile(f, .99), 100 * f.max(), 100 * (f > 0.01).mean(),
            np.quantile(a, .9), a.max()))


def top(rows, pol, k, filt=None):
    rs = [r for r in rows if filt is None or filt(r[0])]
    rs = sorted(rs, key=lambda r: -(r[1][pol] - r[1]["opt"]) / r[1]["prior"])
    return rs[:k]


def main(argv):
    quick = "--quick" in argv
    out = print
    t0 = time.time()
    out("# twostep sweep output (exact enumeration, no Monte Carlo)")

    # ---------------- structured grid ----------------
    rows = []
    for meta, inst in grid_instances(ns=(3,) if quick else (3, 4),
                                     Bs=(2,) if quick else (2, 3)):
        rows.append((meta, solve(inst), inst))
    out("\n## 1. Structured grid (%d instances; uniform boundary prior, uniform p, "
        "mag=1, V = upper half of the courses)" % len(rows))
    out("\nmode `hard`: budget = B full units.  mode `slack`: budget = B + n*c_lo "
        "(the whole cheap screen is affordable on top of B full queries).")
    pct_table(rows, "All grid instances, every policy", out)
    for key in ("c_lo", "mode", "e_lo", "e_hi", "q", "B", "n"):
        table(rows, key, "Grid by " + key, out)
    for mode in ("hard", "slack"):
        table([r for r in rows if r[0]["mode"] == mode], "c_lo",
              "Grid by c_lo, mode=%s" % mode, out,
              pols=("greedy", "look2", "look3", "screen+greedy"))

    # ---------------- larger course sets ----------------
    if not quick:
        big = []
        for n, B, ehs in ((5, 2, (0.0, 0.1)), (6, 2, (0.0, 0.1)),
                          (3, 4, (0.1, 0.2)), (4, 4, (0.1, 0.2))):
            for c in (0.2, 0.001):
                for e_hi in ehs:
                    meta = dict(n=n, B=B, c_lo=c, mode="slack", e_lo=0.2, q=0.3,
                                e_hi=e_hi, nB="n=%d,B=%d" % (n, B))
                    inst = Instance(n, c, B + n * c, 0.2, e_hi, 0.3)
                    big.append((meta, solve(inst), inst))
        out("\n## 2. Larger course sets and longer budgets (slack budget, e_lo=0.2, "
            "q=0.3; e_hi in {0,0.1} for B=2 and {0.1,0.2} for B=4)")
        table(big, "nB", "larger n / larger B", out,
              pols=("greedy", "look2", "look3", "screen+greedy"))
        rows_all = rows + big
    else:
        rows_all = rows

    # ---------------- random family ----------------
    rng = np.random.default_rng(20261109)
    N = 40 if quick else 300
    rnd = []
    for _ in range(N):
        meta, inst = random_instance(rng)
        rnd.append((meta, solve(inst), inst))
    out("\n## 3. Broad random sample (seed 20261109, N=%d): n in {3,4,5}, B in {2,3} (also 4 when n=3; only 2 when n=5), "
        "Dirichlet boundary prior and course weights, mag ~ U(0.2,1), random V, "
        "q ~ U(0,0.6) (20%% q=0), e_lo ~ U(0,0.4), e_hi ~ U(0,0.3) (30%% e_hi=0), "
        "c_lo uniform over the four ratios, hard/slack budget" % N)
    pct_table(rnd, "Random sample, every policy", out)
    for mode in ("hard", "slack"):
        table([r for r in rnd if r[0]["mode"] == mode], "c_lo",
              "Random sample by c_lo, mode=%s" % mode, out,
              pols=("greedy", "look2", "look3", "screen+greedy"))
    real = [r for r in rnd if r[0]["c_lo"] == 0.001 and r[0]["mode"] == "slack"]
    pct_table(real, "Random sample restricted to the realistic case "
              "(c_lo=0.001, slack budget), N=%d" % len(real), out)

    # ---------------- largest-gap instances ----------------
    out("\n## 4. Largest-gap instances\n\n```")
    pool = rows_all + rnd
    seen = set()
    for title, pol, filt in (
            ("largest optimal-vs-two-step gap (any instance)", "look2", None),
            ("largest optimal-vs-two-step gap at c_lo=0.001, slack budget", "look2",
             lambda m: m["c_lo"] == 0.001 and m["mode"] == "slack"),
            ("largest optimal-vs-greedy gap (any instance)", "greedy", None),
            ("largest optimal-vs-greedy gap at c_lo=0.001, slack budget", "greedy",
             lambda m: m["c_lo"] == 0.001 and m["mode"] == "slack"),
            ("largest optimal-vs-three-step gap (any instance)", "look3", None)):
        for meta, res, inst in top(pool, pol, 1, filt):
            out("\n=== %s: regret %.5f = %.2f%% of prior loss ===" % (
                title, res[pol] - res["opt"], 100 * (res[pol] - res["opt"]) / res["prior"]))
            if id(inst) in seen:
                out("    (same instance as above) " + inst.label())
                continue
            seen.add(id(inst))
            Solver(inst).describe(out)
    out("```")

    # ---------------- adversarial checks ----------------
    out("\n## 5. Adversarial checks")
    # (1) realistic ratio: is screen-then-optimal exactly optimal?
    worst = 0.0
    cnt = 0
    for meta, res, inst in pool:
        if meta["c_lo"] == 0.001 and meta["mode"] == "slack" and meta["n"] <= 4:
            s = Solver(inst)
            v = s.evaluate("so", s.act_screen_then(s.act_opt))
            worst = max(worst, v - res["opt"]); cnt += 1
    out("\n(1) c_lo=0.001, slack budget: max over %d instances of "
        "[loss(run every cheap test first, then optimal) - loss(optimal)] = %.2e" % (cnt, worst))
    pct_table([r for r in pool if r[0]["c_lo"] == 0.001 and r[0]["mode"] == "slack"],
              "(1) all instances with c_lo=0.001, slack budget (grid + random)", out)
    pct_table([r for r in pool if r[0]["c_lo"] == 0.001 and r[0]["mode"] == "hard"],
              "(1b) all instances with c_lo=0.001, hard integer budget (grid + random)", out)

    # (2) three-step / four-step
    out("\n(2) deeper lookahead on the top-5 two-step-gap instances "
        "(regret as % of prior loss):\n")
    out("| instance | greedy | look1 | look2 | look3 | look4 |")
    out("|---|---|---|---|---|---|")
    for meta, res, inst in top(pool, "look2", 5):
        s = Solver(inst)
        r = s.all_values(deep=4)
        out("| n=%d B=%d c=%g %s e_lo=%.2f e_hi=%.2f q=%.2f | " % (
            meta["n"], meta["B"], meta["c_lo"], meta["mode"], meta["e_lo"],
            meta["e_hi"], meta["q"]) + " | ".join(
                "%.2f%%" % (100 * (r[p] - r["opt"]) / r["prior"])
                for p in ("greedy", "look1", "look2", "look3", "look4")) + " |")

    # (3) noise sweeps on the top instances
    out("\n(3) flip-probability sweeps: the top two-step-gap instance and the top "
        "realistic-ratio greedy-gap instance, varying one flip probability at a time "
        "(regret as % of prior loss)\n")
    cases = [("top look2", top(pool, "look2", 1)[0]),
             ("top greedy @ c=0.001 slack", top(
                 pool, "greedy", 1,
                 lambda m: m["c_lo"] == 0.001 and m["mode"] == "slack")[0])]
    grid_e = [0.0, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]
    for nm, (meta, res, inst) in cases:
        for which in ("e_hi", "e_lo"):
            out("\n%s, sweeping %s (base e_lo=%.3f e_hi=%.3f):\n" % (
                nm, which, inst.e_lo, inst.e_hi))
            out("| %s | prior | opt | greedy frac | look2 frac | look3 frac |" % which)
            out("|---|---|---|---|---|---|")
            for e in grid_e:
                i2 = Instance(inst.n, inst.c / FULL, inst.budget / FULL,
                              e if which == "e_lo" else inst.e_lo,
                              e if which == "e_hi" else inst.e_hi, inst.q, V=inst.V,
                              prior_b=inst.prior_b, p=inst.p, mag=inst.mag)
                r = solve(i2)
                out("| %.2f | %.4f | %.4f | %.2f%% | %.2f%% | %.2f%% |" % (
                    e, r["prior"], r["opt"],
                    100 * (r["greedy"] - r["opt"]) / r["prior"],
                    100 * (r["look2"] - r["opt"]) / r["prior"],
                    100 * (r["look3"] - r["opt"]) / r["prior"]))
    # gap vs e_hi over the whole random sample
    out("\nRandom sample binned by full-tier flip probability e_hi:\n")
    out("| e_hi bin | N | greedy mean frac | greedy max frac | look2 mean frac | "
        "look2 max frac | look3 max frac |")
    out("|---|---|---|---|---|---|---|")
    for lo, hi in ((0, 1e-9), (1e-9, 0.1), (0.1, 0.2), (0.2, 0.3)):
        sub = [r for r in rnd if lo <= r[0]["e_hi"] < hi]
        if not sub:
            continue
        g = gapstats(sub, "greedy")[1]; l2 = gapstats(sub, "look2")[1]
        l3 = gapstats(sub, "look3")[1]
        out("| [%.1f,%.1f) | %d | %.2f%% | %.2f%% | %.3f%% | %.2f%% | %.2f%% |" % (
            lo, hi, len(sub), 100 * g.mean(), 100 * g.max(), 100 * l2.mean(),
            100 * l2.max(), 100 * l3.max()))
    out("\nelapsed %.1f s" % (time.time() - t0))


if __name__ == "__main__":
    main(sys.argv[1:])
