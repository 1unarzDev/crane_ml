"""scenario v1 + task v1 -> an mf TaskSpec and ConfigRef, so one scene is evaluated at L0 (game grid), L1 (capplan
truth), the fastsim tier and L2 (CRANE: a pending job request).

    python3 -m scenes.mf_adapter --family F [--split S] [--seed N] [--plan P|all] [--levels L0,L1,fastsim,L2]

The mf API is imported read-only from $FLEET_CAPPLAN_ROOT (crane_ml Tools/Fleet); records are evaluated with
``cache=False`` (nothing is written to the shared mf cache). mf v0's transport task is ONE assembly's route (legs),
one door width, one payload and handling counts, so the adapter takes the plan's DELIVERING unit as the configuration
(the assembly a dock/undock produces, by its catalogue key) and the narrowest door that unit's legs cross. What the mf
levels cannot see (doglegs, slots, friction patches, traffic, bays, faults mid-route, visibility) is listed per task
in ``unmodelled``; that is exactly where L0/L1 and fastsim are expected to disagree (and where mf.select should spend
CRANE runs).
"""
import argparse
import json
import math
import os
import sys

from .common import CAPPLAN_ROOT, FRICTION_CLASSES, door_frame, rect_centre, sha, zone
from .families import GENERATORS, generate
from . import catalog

_MF = {}


def mf():
    if not _MF:
        if str(CAPPLAN_ROOT) not in sys.path:
            sys.path.append(str(CAPPLAN_ROOT))
        os.environ.setdefault("FLEET_MF_CACHE", os.path.expanduser("~/.cache/fleet-mf-scenes"))
        import mf as m  # noqa: E402  (read-only)
        from mf import defs  # noqa: E402
        _MF.update(m=m, defs=defs)
    return _MF


def delivering(scn, task, plan):
    """(unit id, catalogue key, its start/handover pose, payload id, delivery zone) of the plan's last deliver step."""
    keys = {u["id"]: u["catalog"] for u in scn["fleet"]["units"]}
    pos = {u["id"]: u["start"] for u in scn["fleet"]["units"]}
    ops = {"dock": 0, "undock": 0, "lift": 0, "deliver": 0, "handoff": 0}
    last = None
    for s in task["plans"][plan]["steps"]:
        k = s["kind"]
        if k == "dock":
            keys[s["as"]] = s.get("as_catalog") or keys[s["target"]]
            pos[s["as"]] = pos[s["target"]]
            ops["dock"] += 1
        elif k == "undock":
            cats = s.get("catalogs") or [None, None]
            for nid, c in zip(s["as"], cats):
                keys[nid] = c or "carrier"
                pos[nid] = pos[s["unit"]]
            ops["undock"] += 1
        elif k == "pickup":
            ops["lift"] += 1
        elif k == "handoff":
            ops["handoff"] += 1
            pos[s["to"]] = pos[s["from"]]
        elif k == "deliver":
            ops["deliver"] += 1
            last = s
    if last is None:
        return None
    return last["unit"], keys[last["unit"]], pos[last["unit"]], last["payload"], last["zone"], ops


def crossings(scn, a, b):
    """Doors whose wall the straight segment a->b crosses (ordered along it), with their crossing points."""
    out = []
    for d in scn["doors"]:
        cx, cy, normal, w = door_frame(scn, d)
        if normal == "x" and (a[0] - cx) * (b[0] - cx) < 0:
            out.append((abs(a[0] - cx), d, (cx, cy)))
        elif normal == "y" and (a[1] - cy) * (b[1] - cy) < 0:
            out.append((abs(a[1] - cy), d, (cx, cy)))
    return [(d, p) for _, d, p in sorted(out, key=lambda t: t[0])]


def unmodelled(scn, task, plan):
    out = []
    if len({d["wall"] for d in scn["doors"]}) > 1:
        out.append("several doors in series (doglegs): mf has one door width")
    if any(z["kind"] == "slot" for z in scn["zones"]):
        out.append("slot pose with required yaw (lateral-only access)")
    if scn["floor_patches"]:
        out.append("floor patches (mf has one floor)")
    if any(z["kind"] == "aisle" for z in scn["zones"]) or len(scn["fleet"]["units"]) > 2:
        out.append("multi-robot interaction (traffic, queues)")
    if any(z["kind"] == "charging" for z in scn["zones"]):
        out.append("battery and charging bays")
    if scn["fleet"]["faults"]:
        out.append("mid-mission faults (mf health is static)")
    if any(not p["spawn"].get("pose_known", True) for p in scn["payloads"]):
        out.append("payload search / visibility")
    if any(s["kind"] == "dock" for s in task["plans"][plan]["steps"]):
        out.append("docking-face approach corridors")
    return out


def taskspec(scn, task, plan):
    """The mf TaskSpec of one (scene, plan) and the delivering unit's catalogue key (None if the plan delivers nothing)."""
    D = mf()["defs"]
    dv = delivering(scn, task, plan)
    floor = scn["arena"]["floor"]
    prof = FRICTION_CLASSES[floor]["profile"] or "hard_tile"
    src = {"scenario_sha": sha(scn, 64), "scenario_id": scn["scenario_id"], "family": scn["family"]["id"],
           "split": scn["family"]["split"], "seed": scn["family"]["seed"], "plan": plan, "adapter": "scenes.mf_adapter",
           "unmodelled": unmodelled(scn, task, plan)}
    if dv is None:
        return D.TaskSpec("transport", task_id="%s/%s" % (task["task_id"], plan), scenario_id=scn["scenario_id"],
                          floor=prof, deadline_s=float(task["deadline_s"]), source=src,
                          world_overrides=D._floor_overrides(prof)), None
    uid, key, start, pid, zid, ops = dv
    p = next(x for x in scn["payloads"] if x["id"] == pid)
    pts = [tuple(start[:2])]
    if not p["spawn"].get("on_unit"):
        pts.append(tuple(p["spawn"]["pose"][:2]))
    goal = rect_centre(zone(scn, zid)["rect"])
    route, door_w = [pts[-1]], None
    for d, (cx, cy) in crossings(scn, pts[-1], goal):
        _, _, normal, _ = door_frame(scn, d)
        off = (0.6, 0.0) if normal == "x" else (0.0, 0.6)
        sgn = 1 if (goal[0] - cx if normal == "x" else goal[1] - cy) > 0 else -1
        route += [(cx - sgn * off[0], cy - sgn * off[1]), (cx + sgn * off[0], cy + sgn * off[1])]
        door_w = d["width"] if door_w is None else min(door_w, d["width"])
    pts = pts[:-1] + route + [goal]
    ts = D.TaskSpec("transport", task_id="%s/%s" % (task["task_id"], plan), scenario_id=scn["scenario_id"], floor=prof,
                    door_width=door_w, payload={"type": p["type"], "mass": float(p["mass_kg"])}, legs=D._legs(pts),
                    ops={k: v for k, v in ops.items() if v}, deadline_s=float(task["deadline_s"]),
                    world_overrides=D._floor_overrides(prof), source=src)
    return ts, key


def config_ref(key):
    m = mf()["m"]
    ref = catalog.unit(key)["ref"]
    if "manifest" in ref:
        return m.ConfigRef.from_manifest(ref["manifest"], ref["config_hash"])
    g, _ = catalog._graph(ref)
    return m.ConfigRef.from_graph(g)


def evaluate(scn, task, plan, levels=("L0", "L1", "fastsim", "L2"), world_seed=0, fastsim_out=None):
    rows = {}
    ts, key = taskspec(scn, task, plan)
    if key is None:
        return {"plan": plan, "config": None, "note": "the plan delivers no payload: not an mf transport task",
                "unmodelled": ts.source["unmodelled"]}
    cfg = config_ref(key)
    for lv in levels:
        if lv in ("L0", "L1"):
            rec = mf()["m"].evaluate(lv, cfg, ts, world_seed=world_seed, cache=False, study="scenes_v1")
            o = rec["outcome"]
            rows[lv] = {"status": rec["status"], "feasible": o.get("feasible"), "success": o.get("success"),
                        "time_s": o.get("time_s"), "reason": o.get("reason")}
        elif lv == "fastsim":
            from .fastsim_run import run
            import tempfile
            r = run(scn["family"]["id"], scn["family"]["split"], plan, [scn["family"]["seed"]],
                    fastsim_out or tempfile.mkdtemp(prefix="scenes_mf_"))[0]
            rows[lv] = {"status": "ok", "feasible": None, "success": r["outcome"] == "SUCCEEDED",
                        "time_s": r["t_end"], "reason": r["reason"]}
        elif lv == "L2":
            from .crane_compile import compile_scene
            import tempfile
            c = compile_scene(scn, task, tempfile.mkdtemp(prefix="scenes_l2_"), [plan])
            rows[lv] = {"status": "pending" if c["jobs"] else "unsupported", "job": (c["jobs"] or [None])[0],
                        "reason": None if c["jobs"] else "; ".join(g["detail"] for g in c["plans"][plan]["gaps"]
                                                                if g["status"] == "unsupported")}
    return {"plan": plan, "config": {"key": key, "manifest": cfg.manifest, "config_hash": cfg.config_hash},
            "task_hash": ts.task_hash, "door_width": ts.door_width, "route_m": round(ts.route_length(), 3),
            "unmodelled": ts.source["unmodelled"], "levels": rows}


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.mf_adapter")
    ap.add_argument("--family", required=True)
    ap.add_argument("--split", default="train")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plan", default="all")
    ap.add_argument("--levels", default="L0,L1,fastsim,L2")
    a = ap.parse_args(argv)
    scn, task = generate(a.family, a.seed, a.split)
    plans = list(task["plans"]) if a.plan == "all" else [a.plan]
    for p in plans:
        print(json.dumps(evaluate(scn, task, p, tuple(a.levels.split(","))), default=str))


if __name__ == "__main__":
    main()
