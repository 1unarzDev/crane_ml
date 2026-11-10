# SPDX-License-Identifier: GPL-3.0-or-later
"""Aggregate results/raw/run_*.jsonl into the DEV tables (cluster bootstrap over layouts, paired by construction because
all policies share (layout, rep) noise).  Writes tables to stdout / results/raw/tables.json; DEV.md is assembled by hand
from these plus EXACT.md and the tuning log."""
import glob, json, math, os, sys
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
B = 400
FAMS = {  # family -> (policy name prefix, kind)
    "IND_BAYES": "IND_BAYES@", "CORR": "CORR@", "CORR_SAMEVIEW": "CORR_SAMEVIEW@", "CORR_PARTNER": "CORR_PARTNER@",
    "SPRT": "SPRT@", "SPRT_DEFL": "SPRT_DEFL@", "SPRT_DEFL_ALT": "SPRT_DEFL_ALT@", "CI_THR": "CI_THR@", "FIXK": "FIXK",
}
CHEAP = ("SPRT_DEFL", "SPRT_DEFL_ALT", "CI_THR")
LAMS = (100, 300, 900)
PLAN = ("CORR", "CORR_SAMEVIEW", "CORR_PARTNER")


def load(pattern):
    rows, seen = [], set()
    for f in sorted(g for pat in pattern.split(",") for g in glob.glob(pat)):
        for line in open(f):
            r = json.loads(line)
            k = (r["rho"], r["f"], r["regime"], r["policy"], r["layout"], r["rep"])
            if k not in seen:
                seen.add(k)
                rows.append(r)
    return rows


def fam_of(name):
    for fam in ("CORR_SAMEVIEW", "CORR_PARTNER", "SPRT_DEFL_ALT", "SPRT_DEFL", "IND_BAYES", "CORR", "CI_THR", "SPRT", "FIXK"):
        if name.startswith(fam):
            return fam
    return name


def cell_arrays(rows):
    """policy -> array (nL, 4) of per-layout means [T_cens, FR, ok, verifs] and the layout list."""
    by = defaultdict(lambda: defaultdict(list))
    for r in rows:
        by[r["policy"]][r["layout"]].append((r["T"], r["FR"], r["ok"], r["verifs"]))
    layouts = sorted({l for p in by.values() for l in p})
    out = {}
    for p, d in by.items():
        out[p] = np.array([np.mean(d[l], axis=0) for l in layouts])
    return out, layouts


def curve(arr, names, idx):
    """points (T, FR) for the policies `names` using layouts idx."""
    return np.array([arr[n][idx, :2].mean(0) for n in names if n in arr])


def interp_T_at_FR(c, fr):
    o = np.argsort(c[:, 1])
    x, y = c[o, 1], c[o, 0]
    if fr < x[0] - 1e-12 or fr > x[-1] + 1e-12:
        return np.nan
    return float(np.interp(fr, x, y))


def interp_FR_at_T(c, t):
    o = np.argsort(c[:, 0])
    x, y = c[o, 0], c[o, 1]
    if t < x[0] - 1e-12 or t > x[-1] + 1e-12:
        return np.nan
    return float(np.interp(t, x, y))


def h1_ratio(ci, cc):
    """geometric-mean FR(independence)/FR(correlation-aware) at matched T over IND points inside the CORR T range."""
    rs = []
    for T, fr in ci:
        f2 = interp_FR_at_T(cc, T)
        if np.isnan(f2):
            continue
        rs.append(math.log(max(fr, 0.01) / max(f2, 0.01)))
    return float(math.exp(np.mean(rs))) if rs else np.nan


def matched_fr_gain(cA, cB, n=6):
    """mean over common FR levels of (T_B - T_A)/T_B  (A = candidate, B = reference); positive = A faster."""
    lo, hi = max(cA[:, 1].min(), cB[:, 1].min()), min(cA[:, 1].max(), cB[:, 1].max())
    if hi <= lo + 1e-9:
        return np.nan
    gs = []
    for fr in np.linspace(lo, hi, n):
        ta, tb = interp_T_at_FR(cA, fr), interp_T_at_FR(cB, fr)
        if not (np.isnan(ta) or np.isnan(tb)):
            gs.append((tb - ta) / tb)
    return float(np.mean(gs)) if gs else np.nan


def stats_for(arr, layouts, names_by_fam, rng):
    """point estimates + bootstrap CI for H1 ratio, H2 gains, kill-check ratio."""
    nL = len(layouts)
    def one(idx):
        cs = {fam: curve(arr, nm, idx) for fam, nm in names_by_fam.items() if len(nm) > 0}
        r = {}
        if "IND_BAYES" in cs and "CORR" in cs:
            r["H1"] = h1_ratio(cs["IND_BAYES"], cs["CORR"])
        if "CORR" in cs and "FIXK" in cs:
            r["H2_fixk"] = matched_fr_gain(cs["CORR"], cs["FIXK"])
        plan = [cs[f] for f in PLAN if f in cs]
        cheap = [cs[f] for f in CHEAP if f in cs]
        if plan and cheap:
            P = np.vstack(plan); Ch = np.vstack(cheap)
            # lower envelopes in (FR, T): for each FR keep the min T among families
            lo = max(min(c[:, 1].min() for c in plan), min(c[:, 1].min() for c in cheap))
            hi = min(max(c[:, 1].max() for c in plan), max(c[:, 1].max() for c in cheap))
            if hi > lo + 1e-9:
                g = []
                for fr in np.linspace(lo, hi, 6):
                    tp = np.nanmin([interp_T_at_FR(c, fr) for c in plan])
                    tc = np.nanmin([interp_T_at_FR(c, fr) for c in cheap])
                    g.append(tc / tp - 1.0)
                r["cheap_over_best"] = float(np.mean(g))
            if "SPRT_DEFL" in cs and "CORR" in cs:
                r["H2_defl"] = matched_fr_gain(cs["CORR"], cs["SPRT_DEFL"])
            if "IND_BAYES" in cs and "CORR" in cs:
                r["H2_ind"] = matched_fr_gain(cs["CORR"], cs["IND_BAYES"])
        if "CORR_PARTNER" in cs and "CORR" in cs:
            r["partner_gain"] = matched_fr_gain(cs["CORR_PARTNER"], cs["CORR"])
        # matched nominal threshold: geometric mean over theta of FR(IND)/FR(CORR)
        ni = sorted(n for n in names_by_fam.get("IND_BAYES", []))
        nc = [n.replace("IND_BAYES", "CORR") for n in ni]
        if ni and all(n in arr for n in nc):
            rr = [math.log(max(arr[a][idx, 1].mean(), 0.01) / max(arr[b][idx, 1].mean(), 0.01)) for a, b in zip(ni, nc)]
            r["H1_theta"] = float(math.exp(np.mean(rr)))
        # Lagrangian objective J = T_cens + lam * FR, best parameter per family (no interpolation)
        def bestJ(fams, lam):
            v = [arr[n][idx, 0].mean() + lam * arr[n][idx, 1].mean() for f_ in fams for n in names_by_fam.get(f_, []) if n in arr]
            return min(v) if v else np.nan
        for lam in LAMS:
            J = {k: bestJ(v, lam) for k, v in {"corr": ["CORR"], "corr_p": ["CORR_PARTNER"], "corr_s": ["CORR_SAMEVIEW"],
                                               "ind": ["IND_BAYES"], "fixk": ["FIXK"], "sprt": ["SPRT"],
                                               "cheap": ["SPRT_DEFL", "SPRT_DEFL_ALT", "CI_THR"]}.items()}
            J["cover"] = arr["COVER"][idx, 0].mean() + lam * arr["COVER"][idx, 1].mean() if "COVER" in arr else np.nan
            others = [J[k] for k in ("ind", "fixk", "sprt", "cheap", "cover")]
            best_plan = np.nanmin([J["corr"], J["corr_p"], J["corr_s"]])
            r[f"gain_vs_fixk@{lam}"] = 1 - J["corr"] / J["fixk"]
            r[f"gain_vs_ind@{lam}"] = 1 - J["corr"] / J["ind"]
            r[f"gain_vs_cheap@{lam}"] = 1 - J["corr"] / J["cheap"]
            r[f"gain_vs_best_other@{lam}"] = 1 - best_plan / np.nanmin(others)
            r[f"viewclass@{lam}"] = 1 - J["corr"] / J["corr_s"]
            r[f"partner@{lam}"] = 1 - J["corr_p"] / J["corr"]
            r[f"cheap_over_best@{lam}"] = J["cheap"] / np.nanmin([best_plan, J["ind"], J["fixk"], J["sprt"]]) - 1
            if idx is not None and len(idx) == nL and np.array_equal(idx, np.arange(nL)):
                r[f"J@{lam}"] = {k: float(v) for k, v in J.items()}
        return r
    point = one(np.arange(nL))
    bs = defaultdict(list)
    for _ in range(B):
        idx = rng.integers(0, nL, nL)
        for k, v in one(idx).items():
            if not isinstance(v, dict):
                bs[k].append(v)
    ci = {}
    for k, v in bs.items():
        v = np.array(v, float)
        v = v[~np.isnan(v)]
        ci[k] = (float(np.percentile(v, 2.5)), float(np.percentile(v, 97.5))) if len(v) > 20 else (np.nan, np.nan)
    return point, ci


def fmt(x, ci=None, d=2, pct=False):
    if x is None or (isinstance(x, float) and np.isnan(x)):
        return "n/a"
    s = f"{x*100:.0f}%" if pct else f"{x:.{d}f}"
    if ci is not None and not np.isnan(ci[0]):
        s += f" [{ci[0]*100:.0f}, {ci[1]*100:.0f}]%" if pct else f" [{ci[0]:.{d}f}, {ci[1]:.{d}f}]"
    return s


def main(pattern):
    rows = load(pattern)
    cells = sorted({(r["regime"], r["f"], r["rho"]) for r in rows})
    rng = np.random.default_rng(0)
    out = {"H": [], "policy": [], "cause": [], "cost": defaultdict(float), "n": defaultdict(int)}
    for (reg, f, rho) in cells:
        rr = [r for r in rows if r["regime"] == reg and r["f"] == f and r["rho"] == rho]
        arr, layouts = cell_arrays(rr)
        names = sorted(arr)
        nbf = {fam: [n for n in names if (n.startswith(pre) if fam != "CORR" else (n.startswith("CORR@")))] for fam, pre in FAMS.items()}
        nbf["FIXK"] = [n for n in names if n.startswith("FIXK")]
        nbf["SPRT"] = [n for n in names if n.startswith("SPRT@")]
        nbf["SPRT_DEFL"] = [n for n in names if n.startswith("SPRT_DEFL@")]
        pt, ci = stats_for(arr, layouts, nbf, rng)
        out["H"].append(dict(regime=reg, f=f, rho=rho, point=pt, ci=ci, nL=len(layouts)))
        for r in rr:
            out["cost"][r["policy"]] += r["cpu"]; out["n"][r["policy"]] += 1
        # per-policy table (mean over layouts) and cause counts
        for p in names:
            a = arr[p]
            sub = [r for r in rr if r["policy"] == p]
            causes = defaultdict(int)
            for r in sub:
                if not r["ok"]:
                    causes[r["cause"]] += 1
            nver = sum(r["verifs"] for r in sub)
            nok = sum(r["ok"] for r in sub)
            bs = []
            for _ in range(200):
                i = rng.integers(0, len(layouts), len(layouts))
                bs.append(a[i].mean(0))
            bs = np.array(bs)
            out["policy"].append(dict(regime=reg, f=f, rho=rho, policy=p, ok=float(a[:, 2].mean()),
                                      ok_ci=[float(np.percentile(bs[:, 2], 2.5)), float(np.percentile(bs[:, 2], 97.5))],
                                      T=float(a[:, 0].mean()), T_ci=[float(np.percentile(bs[:, 0], 2.5)), float(np.percentile(bs[:, 0], 97.5))],
                                      FR=float(a[:, 1].mean()), FR_ci=[float(np.percentile(bs[:, 1], 2.5)), float(np.percentile(bs[:, 1], 97.5))],
                                      verifs_per_true=(nver / nok if nok else None), n=len(sub), causes=dict(causes)))
    # reliability of the stated report posterior (belief policies only): nominal vs realised precision
    cal = defaultdict(lambda: defaultdict(lambda: [0, 0.0, 0]))
    for r in rows:
        fam = fam_of(r["policy"])
        if fam not in ("IND_BAYES", "CORR", "CORR_PARTNER", "CORR_SAMEVIEW") or not r.get("rlog"):
            continue
        for p_, ok in r["rlog"]:
            b = min(int(p_ * 5), 4)
            c = cal[(r["regime"], r["f"], r["rho"], fam)][b]
            c[0] += 1; c[1] += p_; c[2] += ok
    out["calibration"] = [dict(regime=k[0], f=k[1], rho=k[2], fam=k[3],
                               bins=[(b, v[0], v[1] / v[0] if v[0] else None, v[2] / v[0] if v[0] else None) for b, v in sorted(d.items())])
                          for k, d in cal.items()]
    # false-report anatomy
    fa = defaultdict(lambda: [0, 0])
    for r in rows:
        k = (r["regime"], r["f"], r["rho"], fam_of(r["policy"]))
        fa[k][0] += r["FR"]; fa[k][1] += r.get("fr_decoy", 0)
    out["fr_decoy"] = [dict(regime=k[0], f=k[1], rho=k[2], fam=k[3], FR=v[0], decoy=v[1]) for k, v in fa.items()]
    json.dump(out, open(os.path.join(HERE, "results", "raw", "tables.json"), "w"), default=float)
    return out


if __name__ == "__main__":
    o = main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "raw", "run_*of*.jsonl"))
    for h in o["H"]:
        print(h["regime"], h["f"], h["rho"], {k: (round(v, 3) if (not isinstance(v, dict) and v == v) else None) for k, v in h["point"].items() if not isinstance(v, dict)})
