"""Regenerate results/hypotheses.md from the confirmatory CSVs (preregistered method).

Method: analyze_curve.paired (cluster bootstrap, instances then episodes within
instance), B=2000.  Bootstrap RNG: one np.random.default_rng(--seed) per
(grid, travel) file, shared across its cells in table order (the default seed 0 is
analyze_curve.paired's own default).  PREREG.md fixes B and the function but not
the RNG seed; the original generating script was not saved, so exact digits of
bootstrap endpoints can differ by ~0.001-0.003 from the committed file.

If DATA lacks the raw CSVs, results/data/*.csv.gz is used instead.

Usage (from repo root Tools/):
  python3 -m ForestSearch.make_hypotheses [--data DIR] [--out FILE] [--seed 0]
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from .analyze_curve import load, paired

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATA = os.environ.get("FS_DATA", os.path.join(HERE, "..", "..", "PerformanceResults", "forest_search"))
LAMS = (0.0, 1.0, 2.0, 3.0)


def resolve(data, name):
    """Path to `name` under `data`; falls back to results/data/<name>.gz (decompressed to a temp file)."""
    p = os.path.join(data, name)
    if os.path.exists(p):
        return p
    gz = os.path.join(HERE, "results", "data", name + ".gz")
    import gzip, shutil, tempfile
    tmp = os.path.join(tempfile.mkdtemp(prefix="fs_"), name)
    with gzip.open(gz, "rb") as i, open(tmp, "wb") as o:
        shutil.copyfileobj(i, o)
    return tmp

# (label, link, strategy, reference, lams, hypothesis, travel-0 only)
ROWS = [
    ("H1 ORACLE−IMPLICIT, d0_loss0", "d0_loss0", "ORACLE", "IMPLICIT", LAMS, "H1", False),
    ("H2 HYBRID−IMPLICIT, d1_loss0", "d1_loss0", "HYBRID", "IMPLICIT", LAMS[1:], "H2", True),
    ("H2 HYBRID−IMPLICIT, d1_burst30", "d1_burst30", "HYBRID", "IMPLICIT", LAMS[1:], "H2", True),
    ("H3 HYBRID_P−HYBRID, d1_loss0", "d1_loss0", "HYBRID_P", "HYBRID", LAMS[1:], "H3", True),
    ("(descr.) HYBRID_P−IMPLICIT, d1_loss0", "d1_loss0", "HYBRID_P", "IMPLICIT", LAMS[1:], "descr", True),
    ("H4 FULL−IMPLICIT, d1_loss0", "d1_loss0", "FULL", "IMPLICIT", (0.0,), "H4", False),
    ("H5 INDEP−IMPLICIT, d0_loss0", "d0_loss0", "INDEP", "IMPLICIT", LAMS, "H5", False),
]


def passes(h, m, lo, hi):
    return {"H1": lo >= -0.03 and hi <= 0.03, "H2": hi < 0, "H3": lo > 0, "H4": hi < 0, "H5": hi < 0}[h]


def fmt(m, lo, hi):
    return f"{m:+.3f} [{lo:+.3f},{hi:+.3f}]"


def build(data, seed=0, B=2000):
    out = ["# Preregistered hypotheses: held-out verdicts (seeds 200–207)", "",
           "Paired cluster bootstrap with 2000 resamples (`analyze_curve.paired`). Rules are in "
           "`Tools/ForestSearch/PREREG.md`.", ""]
    h1_fired = []
    for g in ("5x5", "6x6"):
        for t in (0, 1):
            D, _ = load(resolve(data, f"confirm_{g}_t{t}.csv"))
            rng = np.random.default_rng(seed)
            out += [f"## {g}, travel {t}", "", "| test | λ=0 | λ=1 | λ=2 | λ=3 | verdict |", "|---|---|---|---|---|---|"]
            for label, link, s, ref, lams, h, t0only in ROWS:
                if t0only and t != 0:
                    continue
                cells, ok = [], []
                for lam in LAMS:
                    if lam not in lams:
                        cells.append("–")
                        continue
                    m, lo, hi = paired(D, lam, link, s, ref=ref, B=B, rng=rng)
                    cells.append(fmt(m, lo, hi))
                    if h != "descr":
                        ok.append(passes(h, m, lo, hi))
                        if h == "H1" and not ok[-1]:
                            h1_fired.append((g, t, lam))
                verdict = "–" if h == "descr" else ("PASS" if all(ok) else "FAIL")
                out.append(f"| {label} | " + " | ".join(cells) + f" | {verdict} |")
            out.append("")
    out.append("H1 decision rule: it fired (" + "; ".join(f"{g} travel {t}, λ={l:g}" for g, t, l in h1_fired) +
               "). That cell is non-equivalent but shows no significant positive value. The rule conflated the two; "
               "this is disclosed in the paper." if h1_fired else "H1 decision rule: did not fire.")
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=DEFAULT_DATA)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "hypotheses.md"))
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    txt = build(a.data, a.seed)
    open(a.out, "w").write(txt)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
