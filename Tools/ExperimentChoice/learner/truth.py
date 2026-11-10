"""Truth tables for the learner pilot (EXPLORATORY, SURROGATE): pool grid, held-out set, transfer set.

Three course sets, two stand-in pairs, one resumable JSONL store (~/.cache/expchoice/learner_truth_v1.jsonl):

  pool      compact_diff, regular grid over the frozen box in (w, e, d), t = 0.20 m, 3 seeds.
            The learners may only query these courses (one stored seed = one paired run).
  heldout   compact_diff, courses drawn uniformly from the same box with a separate seed,
            4 seeds, topped up to 7 and 10 where the sign of the mean gap is not settled.
            Scoring uses only this set. Its hash is written to heldout_manifest.json first.
  transfer  narrow_tandem, a smaller grid with the same absolute slacks (w - W, e, d - L),
            3 seeds. (The frozen box itself does not fit this footprint at all: d <= 1.60 < L.)

Pairs:  'constructed'  = surrogate.py's rotate_then_follow vs stroke_shimmy (flip built in)
        'independent'  = latticenav.py's lattice_forward vs lattice_reverse

CLI:  truth.py manifest | run --budget-s N | topup --budget-s N | summarize
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
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
import mechanics as M         # noqa: E402
import robots as R            # noqa: E402
import surrogate as S         # noqa: E402
import latticenav as LN       # noqa: E402

TRUTH_VERSION = 1
STORE = os.path.join(S.CACHE_DIR, "learner_truth_v%d.jsonl" % TRUTH_VERSION)
MANIFEST = os.path.join(_HERE, "heldout_manifest.json")

T_CM = 20
BOX = dict(C.ADMISSIBLE_BOX_V0, name="offset_doorway_box_v0_t20", t_cm=[T_CM, T_CM])   # frozen box, t fixed

PAIRS = {"constructed": S.ARMS["differential"], "independent": LN.INDEPENDENT_ARMS}

POOL_W = [80, 90, 100, 110, 120]
POOL_E = [0, 21, 43, 64, 86, 107, 129, 150]
POOL_D = [84, 90, 96, 102, 107, 113, 119, 125, 131, 137, 142, 148, 154, 160]
POOL_SEEDS = [1000, 1001, 1002]

HELDOUT_N = 150
HELDOUT_SAMPLER_SEED = 20261110
HELDOUT_SEEDS = [5000, 5001, 5002, 5003]
HELDOUT_TOPUPS = ([5004, 5005, 5006], [5007, 5008, 5009])

# Transfer: narrow_tandem (L 1.76, W 0.66). Same absolute slacks as the compact box:
# w - W in [0.08, 0.48], e in [0, 1.50], d - L in [0.04, 0.76].
TRANSFER_W = [74, 94, 114]
TRANSFER_E = [0, 50, 100, 150]
TRANSFER_D = [180, 186, 190, 194, 198, 206, 220, 252]      # cut from 150 to 96 courses: the host is loaded
TRANSFER_SEEDS = [7000, 7001, 7002]

# Extra TRAINING footprints (synthetic, differential drive) for the multi-footprint comparison asked for by
# the prior-work review (LIT_representation.md, prediction P1): with one training footprint an explicit
# robot-dimension vector is constant and cannot be learned from. narrow_tandem (1.76 x 0.66, 0.65 m/s,
# 0.75 rad/s, 0.55 m/s^2) is bracketed in every entry by compact_diff/synth_small on one side and
# synth_xl on the other. Limits are invented; nothing is measured.
SYNTHETIC = {"synth_small": dict(L=0.60, W=0.50, v_max=1.0, omega_max=1.8, acc=1.3),
             "synth_xl": dict(L=2.20, W=0.80, v_max=0.55, omega_max=0.6, acc=0.45)}
MF_W_SLACK = [8, 28, 48]                          # cm, w - W
MF_E = [0, 50, 100, 150]
MF_D_SLACK = [4, 10, 14, 18, 22, 30, 44, 76]      # cm, d - L   (the transfer grid uses the same slacks)
MF_SEEDS = POOL_SEEDS

SETS = {"pool": "compact_diff", "heldout": "compact_diff", "transfer": "narrow_tandem",
        "mf_small": "synth_small", "mf_xl": "synth_xl"}


def load_robot(name):
    if name in SYNTHETIC:
        return R.Robot(name=name, drive="differential", provenance={"source": "synthetic, learner/truth.py"},
                       **SYNTHETIC[name])
    return R.load_robot(name)


def admissible(course, box=None):
    """The frozen box's side-clearance rule: |e| + w/2 <= B/2 - side_clearance."""
    box = box or BOX
    return abs(course.e_cm) + course.w_cm / 2 <= course.B_cm / 2 - box["side_clearance_cm"]


def pool_courses():
    return [C.make_course(w / 100, e / 100, d / 100, t=T_CM / 100) for w in POOL_W for e in POOL_E for d in POOL_D]


def heldout_courses():
    return C.sample_courses(HELDOUT_N, HELDOUT_SAMPLER_SEED, BOX)


def transfer_courses():
    return [C.make_course(w / 100, e / 100, d / 100, t=T_CM / 100)
            for w in TRANSFER_W for e in TRANSFER_E for d in TRANSFER_D]


def mf_courses(robot_name):
    """Slack-matched grid for a synthetic training footprint (only courses inside the side-clearance rule)."""
    rb = SYNTHETIC[robot_name]
    Lc, Wc = int(round(rb["L"] * 100)), int(round(rb["W"] * 100))
    cs = [C.make_course((Wc + w) / 100, e / 100, (Lc + d) / 100, t=T_CM / 100)
          for w in MF_W_SLACK for e in MF_E for d in MF_D_SLACK]
    return [c for c in cs if admissible(c)]


def courses_of(set_name):
    if set_name.startswith("mf_"):
        return mf_courses(SETS[set_name])
    return {"pool": pool_courses, "heldout": heldout_courses, "transfer": transfer_courses}[set_name]()


def base_seeds(set_name):
    if set_name.startswith("mf_"):
        return MF_SEEDS
    return {"pool": POOL_SEEDS, "heldout": HELDOUT_SEEDS, "transfer": TRANSFER_SEEDS}[set_name]


def ids_hash(courses):
    return hashlib.sha256(",".join(c.course_id for c in courses).encode()).hexdigest()


def manifest():
    """Everything that defines the scored sets; written and committed before any learner runs."""
    out = {"label": "exploratory", "truth_version": TRUTH_VERSION, "box": BOX, "box_sha256": C.box_hash(BOX),
           "pairs": {k: list(v) for k, v in PAIRS.items()},
           "surrogate_version": S.SURROGATE_VERSION, "lattice_version": LN.LATTICE_VERSION,
           "lattice_cfg": LN.LATTICE_CFG, "sets": {}}
    out["synthetic_training_footprints"] = SYNTHETIC
    for name in SETS:
        cs = courses_of(name)
        robot = load_robot(SETS[name])
        verdicts = [M.certificate(c, robot)["verdict"] for c in cs]
        out["sets"][name] = {"robot": SETS[name], "n": len(cs), "course_ids_sha256": ids_hash(cs),
                             "seeds": base_seeds(name), "n_inside_side_clearance": sum(admissible(c) for c in cs),
                             "certificate_counts": {v: verdicts.count(v) for v in sorted(set(verdicts))}}
    out["sets"]["heldout"].update(sampler_seed=HELDOUT_SAMPLER_SEED, topup_seeds=[list(s) for s in HELDOUT_TOPUPS])
    tandem = R.load_robot("narrow_tandem")
    fb = [M.certificate(c, tandem)["verdict"] for c in pool_courses()]
    out["frozen_box_on_narrow_tandem"] = {v: fb.count(v) for v in sorted(set(fb))}
    return out


# --------------------------------------------------------------------------- store

def _key(set_name, pair, course, seeds):
    ver = S.SURROGATE_VERSION if pair == "constructed" else "L%d" % LN.LATTICE_VERSION
    return "t%d|%s|%s|%s|%s|%s" % (TRUTH_VERSION, set_name, pair, ver, course.course_id, ",".join(map(str, seeds)))


def load_rows(path=None):
    path = path or STORE
    rows = []
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    continue                                   # torn last line
    return rows


def run_one(set_name, pair, course, robot, seeds):
    t0 = time.time()
    g = S.paired_gap(course, robot, seeds, arms=PAIRS[pair])
    return {"key": _key(set_name, pair, course, seeds), "label": "exploratory-surrogate", "set": set_name,
            "pair": pair, "robot": robot.name, "course_id": course.course_id,
            "w_cm": course.w_cm, "e_cm": course.e_cm, "d_cm": course.d_cm, "t_cm": course.t_cm,
            "seeds": g["seeds"], "J_A": g["J_A"], "J_B": g["J_B"], "status_A": g["status_A"],
            "status_B": g["status_B"], "strokes_B": g["strokes_B"], "reversals_B": g["reversals_B"],
            "backups_B": g["backups_B"], "arm_A": g["arm_A"], "arm_B": g["arm_B"],
            "wall_s": round(time.time() - t0, 2)}


def _append(path, rec):
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, sort_keys=True) + "\n")
        f.flush()
        os.fsync(f.fileno())


def run_jobs(jobs, budget_s, path=None, log=print):
    """jobs: list of (set, pair, course, seeds). Appends missing rows until the budget is spent."""
    path = path or STORE
    os.makedirs(os.path.dirname(path), exist_ok=True)
    done = {r["key"] for r in load_rows(path)}
    robots = {n: load_robot(n) for n in set(SETS.values())}
    t0, n_new = time.time(), 0
    for set_name, pair, course, seeds in jobs:
        if _key(set_name, pair, course, seeds) in done:
            continue
        if time.time() - t0 > budget_s:
            log("budget reached after %d new rows; rerun to resume" % n_new)
            return n_new, False
        robot = robots[SETS[set_name]]
        if M.certificate(course, robot)["verdict"] == "does_not_fit":
            continue
        rec = run_one(set_name, pair, course, robot, seeds)
        _append(path, rec)
        done.add(rec["key"])
        n_new += 1
        if n_new % 25 == 0:
            log("%s %s: %d new rows, %.0f s" % (set_name, pair, n_new, time.time() - t0))
    return n_new, True


def base_jobs(order=None):
    order = order or [("pool", "independent"), ("heldout", "independent"), ("pool", "constructed"),
                      ("heldout", "constructed"), ("transfer", "independent"), ("transfer", "constructed"),
                      ("mf_small", "independent"), ("mf_xl", "independent"),
                      ("mf_small", "constructed"), ("mf_xl", "constructed")]
    return [(s, p, c, base_seeds(s)) for s, p in order for c in courses_of(s)]


# --------------------------------------------------------------------------- aggregation and labels

def aggregate(rows, set_name, pair):
    """Merge rows per course. Returns a list of dicts (course params, per-seed arrays), in course order."""
    by = {}
    for r in rows:
        if r["set"] != set_name or r["pair"] != pair:
            continue
        a = by.setdefault(r["course_id"], {"course_id": r["course_id"], "w_cm": r["w_cm"], "e_cm": r["e_cm"],
                                           "d_cm": r["d_cm"], "t_cm": r["t_cm"], "robot": r["robot"], "seeds": [],
                                           "J_A": [], "J_B": [], "status_A": [], "status_B": [], "strokes_B": []})
        for s, ja, jb, sa, sb, nb in zip(r["seeds"], r["J_A"], r["J_B"], r["status_A"], r["status_B"], r["strokes_B"]):
            if s not in a["seeds"]:
                a["seeds"].append(s)
                a["J_A"].append(ja)
                a["J_B"].append(jb)
                a["status_A"].append(sa)
                a["status_B"].append(sb)
                a["strokes_B"].append(nb)
    out = []
    for c in courses_of(set_name):
        if c.course_id in by:
            a = by[c.course_id]
            a["gap"] = [x - y for x, y in zip(a["J_A"], a["J_B"])]
            out.append(a)
    return out


def noise_margin(aggs):
    """Indifference margin from seed noise: max(1 s, 2 x median seed sd over courses where every run succeeds)."""
    sds = [float(np.std(a["gap"], ddof=1)) for a in aggs if len(a["gap"]) > 1
           and all(s == "success" for s in a["status_A"] + a["status_B"])]
    med = float(np.median(sds)) if sds else float("nan")
    return (max(1.0, 2 * med) if sds else 1.0), med


def sign_settled(gap):
    """True when the sign of the mean gap is known: |mean| > 2 standard errors (or no spread at all)."""
    g = np.asarray(gap, float)
    if g.size < 2:
        return False
    se = float(g.std(ddof=1)) / math.sqrt(g.size)
    return abs(float(g.mean())) > 2.0 * se


def label(agg, margin):
    """Five-way: both_fail, indifferent, A_wins, B_wins, unresolved (|mean| >= margin, sign not settled)."""
    if not any(s == "success" for s in agg["status_A"] + agg["status_B"]):
        return "both_fail"
    g = float(np.mean(agg["gap"]))
    if abs(g) < margin:
        return "indifferent"
    if not sign_settled(agg["gap"]):
        return "unresolved"
    return "B_wins" if g > 0 else "A_wins"


def topup_jobs(rows, margins):
    jobs = []
    by_id = {c.course_id: c for c in heldout_courses()}
    for pair in PAIRS:
        for a in aggregate(rows, "heldout", pair):
            if label(a, margins[pair]) != "unresolved":
                continue
            for extra in HELDOUT_TOPUPS:
                if extra[0] not in a["seeds"]:
                    jobs.append(("heldout", pair, by_id[a["course_id"]], list(extra)))
                    break
    return jobs


def margins_from_pool(rows):
    return {pair: noise_margin(aggregate(rows, "pool", pair))[0] for pair in PAIRS}


# --------------------------------------------------------------------------- text summary

def text_map(aggs, margin, w_cm, e_list, d_list):
    sym = {"A_wins": "A", "B_wins": "B", "indifferent": "=", "both_fail": "x", "unresolved": "?"}
    cell = {(a["w_cm"], a["e_cm"], a["d_cm"]): a for a in aggs}
    lines = ["w = %.2f m       d (m): " % (w_cm / 100) + " ".join("%4.2f" % (d / 100) for d in d_list)]
    for e in reversed(e_list):
        lines.append("  e = %4.2f              " % (e / 100) + " ".join(
            "   %s" % (sym[label(cell[(w_cm, e, d)], margin)] if (w_cm, e, d) in cell else " ") for d in d_list))
    return "\n".join(lines)


def summarize(rows):
    out = []
    margins = margins_from_pool(rows)
    for pair in PAIRS:
        for set_name, (ws, es, ds) in (("pool", (POOL_W, POOL_E, POOL_D)), ("heldout", (None, None, None)),
                                       ("transfer", (TRANSFER_W, TRANSFER_E, TRANSFER_D)),
                                       ("mf_small", (None, None, None)), ("mf_xl", (None, None, None))):
            aggs = aggregate(rows, set_name, pair)
            if not aggs:
                continue
            m = margins[pair]
            labs = [label(a, m) for a in aggs]
            wall = sum(r["wall_s"] for r in rows if r["set"] == set_name and r["pair"] == pair)
            nep = sum(2 * len(a["seeds"]) for a in aggs)
            stat = {}
            for a in aggs:
                for arm in "AB":
                    for s in a["status_" + arm]:
                        stat[arm + ":" + s] = stat.get(arm + ":" + s, 0) + 1
            out.append("== %s / %s: %d courses, %d episodes, %.0f s wall; margin %.2f s (seed sd median %.2f s)" % (
                pair, set_name, len(aggs), nep, wall, m, noise_margin(aggs)[1]))
            out.append("   labels: " + ", ".join("%s %d" % (k, labs.count(k)) for k in
                                                 ("A_wins", "B_wins", "indifferent", "both_fail", "unresolved")))
            out.append("   statuses: " + ", ".join("%s %d" % kv for kv in sorted(stat.items())))
            if ws:
                for w in ws:
                    out.append(text_map(aggs, m, w, es, ds))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="EXPLORATORY truth tables for the learner pilot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("manifest")
    r = sub.add_parser("run")
    r.add_argument("--budget-s", type=float, default=540.0)
    t = sub.add_parser("topup")
    t.add_argument("--budget-s", type=float, default=540.0)
    sub.add_parser("summarize")
    a = ap.parse_args(argv)
    if a.cmd == "manifest":
        m = manifest()
        with open(MANIFEST, "w", encoding="utf-8") as f:
            f.write(json.dumps(m, sort_keys=True, indent=1) + "\n")
        print(json.dumps(m["sets"], sort_keys=True, indent=1))
        print("frozen box on narrow_tandem:", m["frozen_box_on_narrow_tandem"])
    elif a.cmd == "run":
        n, complete = run_jobs(base_jobs(), a.budget_s)
        print("wrote %d rows; complete=%s" % (n, complete))
    elif a.cmd == "topup":
        t0, total = time.time(), 0
        for _ in range(len(HELDOUT_TOPUPS)):
            rows = load_rows()
            jobs = topup_jobs(rows, margins_from_pool(rows))
            if not jobs:
                break
            n, complete = run_jobs(jobs, a.budget_s - (time.time() - t0))
            total += n
            if not complete:
                break
        print("top-up wrote %d rows" % total)
    else:
        print(summarize(load_rows()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
