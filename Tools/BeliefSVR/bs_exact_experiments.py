# SPDX-License-Identifier: GPL-3.0-or-later
"""Exact minimal examples (a)(b)(c) for the correlated search/verify/report problem.  Writes results/EXACT.md.
Run: python exact_experiments.py   (about 4 CPU-min; no seeds, fully deterministic)."""
import json, os, sys, time
from dataclasses import replace
import numpy as np
import bs_exact as X

HERE = os.path.dirname(os.path.abspath(__file__))
_CACHE = os.path.join(HERE, "results", "raw", "exact_cache.json")
_C = json.load(open(_CACHE)) if os.path.exists(_CACHE) else {}


def _solve(I, rb=None):
    """Memoised X.solve (JSON file, resumable; key = exact.py hash + instance + rb)."""
    import hashlib
    h = hashlib.sha256(open(os.path.join(HERE, "bs_exact.py"), "rb").read()).hexdigest()[:12]
    key = h + repr(I) + repr(rb)
    if key not in _C:
        _C[key] = X.solve(I, rb)
        os.makedirs(os.path.dirname(_CACHE), exist_ok=True)
        json.dump(_C, open(_CACHE, "w"))
        print("solved", time.strftime("%H:%M:%S"), flush=True)
    return _C[key]
BASE = X.XI(n=2, prior=(0.5, 0.5), pT=0.6, pP=0.6, fd=0.2, pid=0.3, alpha=0.02, rho=0.6, s=1, d=2, cFR=40, H=6, cmiss=40)
f = lambda x, k=2: ("%." + str(k) + "f") % x
RHOS = (0.0, 0.3, 0.6, 0.8)


def table(h, rows):
    return "\n".join(["| " + " | ".join(h) + " |", "|" + "---|" * len(h)] + ["| " + " | ".join(map(str, r)) + " |" for r in rows])


def calib(I, rho):
    """Belief that cell 0 holds the target after k same-class alerts (no other evidence), independence vs correlated."""
    out = []
    for k in (1, 2, 3):
        cnt = [(0, 0)] * (3 * I.n)
        cnt[0] = (k, 0)
        cnt = tuple(cnt)
        pi = X.Model(I, 0.0).posterior(cnt, 0)[0]
        pc = X.Model(I, rho).posterior(cnt, 0)[0]
        out.append((k, pi, pc))
    return out


def main():
    t0 = time.time()
    L = ["# Exact minimal examples (MissionExact extension, `Tools/BeliefSVR/exact.py`)", "",
         "Evidence label: IMPLEMENTED (synthetic; deterministic exact expectimax, no seeds). TPR/FPR levels are *inputs* "
         "chosen inside the range of `Tools/Vision/obs_model.py` (pass-level TPR ~0.6, decoy marginal fire 0.2, = the MissionSim/Vision 0.8 fire x 0.35 per-view visibility rounded down); "
         "they are not fitted here.", "",
         "Model: one target among n cells; each non-target cell holds a persistent decoy w.p. pid; per (cell, view class) a "
         "Gaussian-copula latent shared by repeat looks, ICC rho (the Vision ICC); independent across view classes; decoys "
         "alert w.p. fd with the same ICC; empty cells alert iid w.p. alpha. Actions: look same view / look from the other "
         "own view class (+s steps), move, partner look (class 3, free for the drone, result after d steps), wait, report. "
         "J = E[time of the correct report] + (1-P(succ))(H+cmiss) + cFR E[#false reports]; a false report rules the cell "
         "out and the mission continues. Base instance: n=2, prior uniform, pT=pP=0.6, fd=0.2, pid=0.3, alpha=0.02, "
         "H=6, cFR=cmiss=40, s=1, d=2 (instance A).", ""]
    I = BASE
    # ---------------- (a)
    L += ["## (a) An independence-assuming belief over-reports", "",
          "Belief calibration: posterior that cell 0 is the target after k repeat alerts from one view class "
          "(base instance). Over-confidence = ratio of odds.", ""]
    rows = []
    for rho in (0.3, 0.6, 0.8):
        for k, pi, pc in calib(replace(I, rho=rho), rho):
            rows.append((rho, k, f(pi, 3), f(pc, 3), f((pi / (1 - pi)) / (pc / (1 - pc)), 2)))
    L += [table(["rho", "k alerts", "P independent", "P correlated (true)", "odds inflation"], rows), ""]
    L += ["Exact optimal policy planned with the true rho vs planned assuming independence (rho_b=0) but run in the "
          "true-rho world:", ""]
    rows = []
    for rho in RHOS:
        Ir = replace(I, rho=rho)
        a = _solve(Ir); b = _solve(Ir, rb=0.0)
        rows.append((rho, f(a["J"]), f(b["J"]), f(a["FR"], 3), f(b["FR"], 3),
                     f(b["FR"] / a["FR"], 2) if a["FR"] > 1e-9 else "n/a", f(a["succ"], 3), f(b["succ"], 3),
                     f(a["T_given_succ"]), f(b["T_given_succ"])))
    L += [table(["rho", "J opt", "J indep-planner", "E[FR] opt", "E[FR] indep", "FR ratio", "succ opt", "succ indep",
                 "T|succ opt", "T|succ indep"], rows), ""]
    # a second, larger instance where the effect is bigger (n=3)
    I3 = X.XI(n=2, prior=(0.5, 0.5), pT=0.8, pP=0.8, fd=0.2, pid=0.6, alpha=0.02, rho=0.6, s=1, d=2, cFR=40, H=6, cmiss=40)
    rows = []
    for rho in RHOS:
        Ir = replace(I3, rho=rho)
        a = _solve(Ir); b = _solve(Ir, rb=0.0)
        rows.append((rho, f(a["J"]), f(b["J"]), f(a["FR"], 3), f(b["FR"], 3),
                     f(b["FR"] / a["FR"], 2) if a["FR"] > 1e-9 else "n/a", f(a["succ"], 2), f(b["succ"], 2),
                     f(a["T_given_succ"]), f(b["T_given_succ"])))
    L += ["Second instance B (n=2, pT=0.8, fd=0.2, pid=0.6, cFR=cmiss=40, H=6):", "",
          table(["rho", "J opt", "J indep-planner", "E[FR] opt", "E[FR] indep", "FR ratio", "succ opt", "succ indep",
                 "T|succ opt", "T|succ indep"], rows), ""]
    # fixed threshold rule: report when belief >= theta, repeat looks. (belief rho_b = 0 vs true rho)
    L += ["Fixed rule (look at the best cell from the same view, report when belief >= theta), instance B, true rho=0.6. "
          "T_cens = E[min(T_report, H)] counts unreported missions as H.", ""]
    rows = []
    for th in (0.9, 0.95, 0.97, 0.99):
        for rb in (0.0, 0.6):
            r = X.solve_rule(replace(I3, rho=0.6), rb, th, "same")
            rows.append((th, "independent" if rb == 0 else "correlated", f(r["FR"], 3), f(r["succ"], 3),
                         f(r["Tc"] + (1 - r["succ"]) * I3.H, 2)))
    L += [table(["theta", "belief", "E[FR]", "P(succ)", "T_cens"], rows), ""]
    # ---------------- (b)
    L += ["## (b) Independent-view verification beats a repeat look", "",
          "Gain = J(repeat/own-view-only) - J(full action set). n_indep / n_repeat are expected counts under the optimal "
          "policy. 'after alert' is the optimal action at the first-look-alert state.", ""]
    rows = []
    for rho in RHOS:
        for s in (0, 1, 2):
            Ir = replace(I, rho=rho, s=s)
            a = _solve(Ir); b = _solve(replace(Ir, allow_indep=False))
            act, _ = X.first_action_after(Ir, None, [("L", 0, 1)])
            actm, _ = X.first_action_after(Ir, None, [("L", 0, 0)])
            rows.append((rho, s, f(a["J"]), f(b["J"]), f(b["J"] - a["J"]), f(a["n_indep"]), f(a["n_repeat"]), f(a["FR"], 3),
                         f(b["FR"], 3), str(act), str(actm)))
    L += [table(["rho", "s", "J full", "J repeat-only", "gain", "n_indep", "n_repeat", "E[FR] full", "E[FR] repeat-only",
                 "after alert", "after miss"], rows), ""]
    L += ["In instance A the optimal move after one alert is to report (P=0.89); independent views are used after a MISS "
          "(a repeat miss from a bad view is nearly uninformative at high rho). Instance C (pT=0.6, fd=0.35, pid=0.6) is a "
          "case where an alert alone is not enough to report, so the verify-view choice is visible directly:", ""]
    IC = replace(I, fd=0.35, pid=0.6)
    rows = []
    for rho in RHOS:
        for s in (0, 1, 2):
            Ir = replace(IC, rho=rho, s=s)
            a = _solve(Ir); b = _solve(replace(Ir, allow_indep=False))
            act, q = X.first_action_after(Ir, None, [("L", 0, 1)])
            rows.append((rho, s, f(a["J"]), f(b["J"]), f(b["J"] - a["J"]), f(a["n_indep"]), f(a["n_repeat"]), str(act)))
    L += [table(["rho", "s", "J full", "J repeat-only", "gain", "n_indep", "n_repeat", "after alert"], rows), ""]
    L += ["Prediction P3 (fraction of non-first same-cell looks that are independent-view, over rho and s):", ""]
    rows = []
    for rho in RHOS:
        r = []
        for s in (0, 1, 2):
            a = _solve(replace(I, rho=rho, s=s))
            tot = a["n_indep"] + a["n_repeat"]
            r.append(f(a["n_indep"] / tot, 2) if tot > 1e-9 else "no verification")
        rows.append([rho] + r)
    L += [table(["rho", "s=0", "s=1", "s=2"], rows), ""]
    # ---------------- (c)
    L += ["## (c) Partner verification vs solo verification as a function of delay", "",
          "Substitution comparison. J_repeat: own repeat looks only. J_solo(s): own repeat + own independent view (cost s). "
          "J_partner(d): own repeat + partner look (class 3, free for the drone, result after d steps, TPR pP), NO own "
          "independent view. d* = largest delay at which partner-only is at least as good as solo (J_partner <= J_solo). "
          "'both' = everything allowed (gain over J_solo).", ""]
    rows = []
    for pP in (0.6, 0.8):
        for s in (0, 1, 2):
            Ir = replace(I, s=s, pP=pP)
            rep = _solve(replace(Ir, allow_indep=False))["J"]
            solo = _solve(Ir)["J"]
            line, dstar = [], 0
            for d in (1, 2, 3, 4, 5):
                jp = _solve(replace(Ir, allow_indep=False, allow_partner=True, d=d))["J"]
                line.append(f(jp))
                if jp <= solo + 1e-9:
                    dstar = d
            both = _solve(replace(Ir, allow_partner=True, d=2))["J"]
            rows.append([pP, s, f(rep), f(solo)] + line + [dstar, f(solo - both)])
    L += [table(["pP", "s", "J_repeat", "J_solo", "J_partner d=1", "d=2", "d=3", "d=4", "d=5", "d*", "both(d=2) gain over solo"], rows), ""]
    L += ["Partner-only gain over repeat-only vs rho (pP=0.6, s=1; compare with the solo gain from (b)):", ""]
    rows = []
    for rho in RHOS:
        Ir = replace(I, rho=rho)
        rep = _solve(replace(Ir, allow_indep=False))["J"]
        rows.append([rho, f(rep - _solve(Ir)["J"])] + [f(rep - _solve(replace(Ir, allow_indep=False, allow_partner=True, d=d))["J"]) for d in (1, 2, 3, 4)])
    L += [table(["rho", "solo gain (s=1)", "partner gain d=1", "d=2", "d=3", "d=4"], rows), ""]
    L += [f"Wall time {time.time() - t0:.0f} s (CPU, single process)."]
    with open(os.path.join(HERE, "results", "EXACT.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
