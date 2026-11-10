# SPDX-License-Identifier: GPL-3.0-or-later
"""Successive-halving driver for the staged recfeas sweep (run inside ONE fsrun job).
Stage 1 = 8 flights/cell, 2 = 16, 3 = 32 (cumulative). A cell (speed) continues only if its paired 95% CI on collision rate
(fix - legacy; mission-cluster bootstrap) neither excludes 0 nor lies within +-2 pp. Orchestrator revision (Tier A): stages 1-2 only, all of 5/6/8 m/s run unconditionally; cap 1.3 core-h (4680 s): before each stage the next cost is projected from measured cpu_s/run and the stage is skipped if it would exceed the cap."""
import glob, json, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
O = os.path.expanduser("~/.cache/frontier/crane/recfeas"); CAP = 4680.0  # 1.3 core-h (orchestrator Tier A approval 2026-11-10)
sys.path.insert(0, HERE)
import recfeas  # noqa: E402


def spent():
    return sum(json.load(open(f))["cpu_s"] for f in glob.glob(O + "/*.time.json"))


def twin_seed2():
    # NavPilot 2.5D twin, legacy-equivalent MAP_RHP, seed 2 on cross_0-3 so the twin covers the same 14 cross units as the .NET sweep
    # (seed 1 exists from run_pending stage 2). The twin cannot express the recfeas flags (they are .NET AutonomyConfig only).
    H = os.path.expanduser("~/.cache/frontier/crane/harness")
    subprocess.run([sys.executable, os.path.join(REPO, "Tools/CraneHarness/navpilot_bridge_timed.py"), "--stands", H + "/stands", "--speeds", "5,6,8",
                    "--seeds", "2", "--only", "cross_0,cross_1,cross_2,cross_3", "--out", H + "/navpilot_twin_seed2.jsonl"], check=False)


def main():
    twin_seed2()
    speeds = [5.0, 6.0, 8.0]
    nruns = lambda: sum(1 for f in glob.glob(O + "/recfeas_*/results*.jsonl") for _ in open(f))  # noqa: E731
    for st in (1, 2):  # Tier A: stages 1-2 only, all three speeds unconditionally; stage 3 NOT approved
        recfeas.gen(st, speeds)
        n_new = len(speeds) * len(recfeas.NEW_UNITS[st]) * 2
        per = spent() / max(nruns(), 1) if nruns() else 40.0
        proj = per * n_new
        print("stage %d speeds %s: %d runs, projected %.0f s, spent %.0f s, cap %.0f" % (st, speeds, n_new, proj, spent(), CAP), flush=True)
        if spent() + proj > CAP:
            print("CAP: stage %d skipped (would exceed 1.2 core-h)" % st); break
        subprocess.run([os.path.join(HERE, "run_recfeas_stage.sh"), str(st)], check=False)
        dirs = glob.glob(O + "/recfeas_*_s*") ; dirs = [d for d in dirs if os.path.isdir(d)]
        md = os.path.join(O, "paired_after_stage%d.md" % st)
        recfeas.analyze(dirs, md)
        dec = json.load(open(md + ".decision.json"))
        print('(informational) cells still unresolved:', [v for v, d in dec.items() if d == 'CONTINUE'])
        print("stage %d decision: %s -> continue %s" % (st, dec, speeds), flush=True)


def pending_1_5():
    """run_pending.sh stages 1 (12 counterfactual replays) and 5 (attrib relabel); stage 4 (dev_dr_demo) deferred. Same fsrun hold."""
    t0 = time.time()
    r = subprocess.run([os.path.join(REPO, "Tools/CraneHarness/run_pending_stages.sh"), "1 5"], check=False)
    json.dump({"start": time.strftime("%F %T", time.localtime(t0)), "end": time.strftime("%F %T"), "wall_s": time.time() - t0, "rc": r.returncode},
              open(os.path.join(O, "pending_1_5.time.json"), "w"))


if __name__ == "__main__":
    main()
    pending_1_5()
    subprocess.run([sys.executable, os.path.join(HERE, "oplog_append.py")], check=False)
