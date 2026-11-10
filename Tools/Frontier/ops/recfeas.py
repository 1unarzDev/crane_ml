# SPDX-License-Identifier: GPL-3.0-or-later
"""Staged (successive-halving) recfeas sweep helper.

Cell = speed (5, 6, 8 m/s). A "seed" = one flight = (mission, run_seed). Flight units are ordered
u_k = (MISSIONS[k % 12], run_seed 1 + k // 12); stage 1 = units 0-7, stage 2 adds 8-15, stage 3 adds 16-31 (cumulative 8/16/32).
  gen STAGE            write specs/recfeas_{staged,legacy}_s<STAGE>[<part>].json for the units new at that stage and for the speeds in --speeds
  analyze DIR... --md  paired (fix - legacy) collision-rate difference per speed, mission-cluster bootstrap 95% CI, decision for next stage
  costs OUT_DIR TAG CPU_FRAC  append per-run costs from results rows to Tools/Frontier/costs.jsonl
"""
import argparse, glob, json, os, random, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
SPECS = os.path.join(REPO, "Tools", "CraneHarness", "specs")
COSTS = os.path.join(REPO, "Tools", "Frontier", "costs.jsonl")
MISSIONS = ["cross_%d" % i for i in range(10)] + ["corridor", "corridor_rev"]
NEW_UNITS = {1: range(0, 8), 2: range(8, 16), 3: range(16, 32)}


def groups(stage):
    g = {}
    for k in NEW_UNITS[stage]:
        g.setdefault(1 + k // 12, []).append(MISSIONS[k % 12])
    return g  # run_seed -> missions


def gen(stage, speeds):
    for arm, src in (("staged", "recfeas_sweep.json"), ("legacy", "recfeas_sweep_legacy.json")):
        base = json.load(open(os.path.join(SPECS, src)))
        for part, (rs, ms) in enumerate(sorted(groups(stage).items())):
            s = json.loads(json.dumps(base))
            tag = "recfeas_%s_s%d%s" % (arm, stage, "" if part == 0 else "_p%d" % part)
            s["name"] = tag
            s["speeds"] = speeds
            s["run_seeds"] = [rs]
            s["missions"] = {"cross": 10, "corridor": True, "only": ms}
            s["_note"] = ("Reduced staged recfeas sweep, stage %d (units new at this stage; cell=speed, flight=(mission,run_seed)); "
                          "speeds >=5 only (<=4 m/s: 0 collisions in 97 flights). Derived from %s. " % (stage, src)) + base.get("_note", "")[:0]
            json.dump(s, open(os.path.join(SPECS, tag + ".json"), "w"), indent=1)
            print(tag)


def load(dirs):
    rows = []
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, "results*.jsonl"))):
            rows += [json.loads(l) for l in open(f) if l.strip()]
    return rows


def boot(pairs, B=4000, seed=7):
    """pairs: {cluster: [(c_fix, c_leg)]}; percentile CI of mean paired diff, resampling clusters."""
    cl = list(pairs)
    tot = [sum(a - b for a, b in v) for v in pairs.values()]
    n = [len(v) for v in pairs.values()]
    rnd = random.Random(seed)
    est = []
    for _ in range(B):
        ix = [rnd.randrange(len(cl)) for _ in cl]
        d = sum(n[i] for i in ix)
        est.append(sum(tot[i] for i in ix) / d)
    est.sort()
    return est[int(0.025 * B)], est[int(0.975 * B)]


def analyze(dirs, md):
    rows = load(dirs)
    by = {}
    for r in rows:
        arm = "fix" if r["config"].get("RecursiveFeasibility") else "leg"
        by.setdefault(r["speed"], {}).setdefault(r["pair_key"], {})[arm] = r
    out = ["| speed | pairs | fix coll | legacy coll | diff (pp) | 95% CI (pp) | fix succ | legacy succ | decision |", "|---|---|---|---|---|---|---|---|---|"]
    res = {}
    for v in sorted(by):
        prs = {k: x for k, x in by[v].items() if "fix" in x and "leg" in x}
        n = len(prs)
        if n == 0:
            continue
        pairs = {}
        for k, x in prs.items():
            pairs.setdefault(x["fix"]["cluster"], []).append((int(bool(x["fix"]["r_Collision"])), int(bool(x["leg"]["r_Collision"]))))
        cf = sum(a for p in pairs.values() for a, b in p); cl_ = sum(b for p in pairs.values() for a, b in p)
        sf = sum(bool(x["fix"]["r_Success"]) for x in prs.values()); sl = sum(bool(x["leg"]["r_Success"]) for x in prs.values())
        lo, hi = boot(pairs)
        d = (cf - cl_) / n
        excl0 = lo > 0 or hi < 0
        within = -0.02 <= lo and hi <= 0.02
        dec = "resolved (CI excludes 0)" if excl0 else ("resolved (CI within +-2pp)" if within else "CONTINUE")
        res[v] = dec
        out.append("| %g | %d | %d | %d | %+.1f | [%+.1f, %+.1f] | %d | %d | %s |" % (v, n, cf, cl_, 100 * d, 100 * lo, 100 * hi, sf, sl, dec))
    txt = "\n".join(out)
    print(txt)
    if md:
        open(md, "w").write(txt + "\n")
    json.dump(res, open((md or "/dev/null") + ".decision.json", "w")) if md else None


def costs(outdir, tag, cpu_frac):
    n = 0
    with open(COSTS, "a") as f:
        for r in load([outdir]):
            f.write(json.dumps({"kind": "run", "sim": "crane_dotnet_harness", "fidelity": "high", "config": "%s|v%g|%s" % (tag, r["speed"], r["mission"]),
                                "seed": int(r["run_seed"]), "wall_s": round(r["wall_s"], 3), "cpu_s": round(r["wall_s"] * cpu_frac, 3)}) + "\n")
            n += 1
    print("costs appended", n)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd"); ap.add_argument("args", nargs="*"); ap.add_argument("--md"); ap.add_argument("--speeds", default="5,6,8")
    a = ap.parse_args()
    if a.cmd == "gen":
        gen(int(a.args[0]), [float(x) for x in a.speeds.split(",")])
    elif a.cmd == "analyze":
        analyze(a.args, a.md)
    elif a.cmd == "costs":
        costs(a.args[0], a.args[1], float(a.args[2]))
