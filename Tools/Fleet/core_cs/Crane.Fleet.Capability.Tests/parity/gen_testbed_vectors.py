#!/usr/bin/env python3
"""Parity vectors whose expectations come from the capplan testbed (Tools/Fleet/capplan/capability.py), i.e. the Python
implementation the orchestrator named as the counterpart of Crane.Fleet.Capability.

Random graphs of testbed library modules (mecanum_pod, diff_pod, caster_module, cargo_deck, lifter) are placed at random
rigid poses, random actuator health (dead, locked, weak) is injected, and capplan.capability.Capability supplies the expected
values. The same wheels are exported in the C# test-vector format. Nothing from the C# is used here.

    python3 -I gen_testbed_vectors.py [--n 240] [--seed 7] [--out testbed_vectors.json]

Note on the drivable subspace: when the number of commandable directions da is smaller than the degree of mobility dm
(a free dof the wheels cannot command), the testbed picks the drivable subspace by minimum passive dissipation (an
ASSUMPTION in capability.py) while the C# picks the Euclidean row space of the actuation map. Those vectors are flagged
`subspace_comparable=false`; only dm, da, J, C and limits are compared on them.
"""
import argparse, json, math, os, sys
sys.dont_write_bytecode = True
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, FLEET)
from capplan import modgraph as mg, capability as cap  # noqa: E402

TYPES = ["mecanum_pod", "diff_pod", "caster_module", "cargo_deck", "lifter"]


def export(graph, c):
    mods = []
    for m in graph.modules.values():
        ws = []
        for k, w in enumerate(m.spec()["wheels"]):
            h, g = m.act_health[k], m.act_gain[k]
            mode = {"ok": "ok", "dead": "dead_free", "locked": "dead_locked", "weak": "weak"}[h]
            if w["kind"] == "wheel_mecanum":
                dx, dy = w["d"]
                roller, drive_yaw = math.atan2(dy, dx), 0.0
                kind = "wheel_mecanum"
            elif w["kind"] == "wheel_diff":
                roller, drive_yaw, kind = 0.0, math.atan2(w["e"][1], w["e"][0]), "wheel_diff"
            else:
                roller, drive_yaw, kind = 0.0, 0.0, "wheel_caster"
            ws.append({"id": f"{m.id}.w{k}", "kind": kind, "x": w["x"], "y": w["y"], "drive_yaw": drive_yaw,
                       "radius": mg.R_W, "roller_angle": roller, "max_speed": mg.PHI_MAX, "driven": True,
                       "health": {"mode": mode, "gain": g}, "lateral": "hard", "normal_load": 0.0, "lateral_mu": 0.0})
        mods.append({"id": m.id, "x": m.x, "y": m.y, "yaw": m.yaw, "wheels": ws})
    return {"id": "testbed", "modules": mods}


def build(rng, i):
    n = int(rng.integers(1, 5)); mods = []
    x = 0.0
    for k in range(n):
        t = str(rng.choice(TYPES, p=[0.35, 0.3, 0.15, 0.1, 0.1]))
        yaw = float(rng.choice([0.0, math.pi, math.pi / 2, -math.pi / 2, rng.uniform(-math.pi, math.pi)], p=[0.35, 0.15, 0.1, 0.1, 0.3]))
        m = mg.Module(f"m{k}", t, x, float(rng.uniform(-0.1, 0.1)), yaw)
        nw = len(m.spec()["wheels"])
        for j, w in enumerate(m.spec()["wheels"]):
            if w["kind"] == "wheel_caster":
                continue
            if rng.random() < 0.28:
                h = str(rng.choice(["dead", "locked", "weak"]))
                m.act_health[j] = h
                m.act_gain[j] = float(rng.uniform(0.2, 0.9)) if h == "weak" else 1.0
        mods.append(m); x += 0.36
    return mg.ModuleGraph(mods, [])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=240)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default=os.path.join(HERE, "testbed_vectors.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed); vecs = []; skipped = 0; i = 0
    while len(vecs) < a.n and i < a.n * 5:
        g = build(rng, i); i += 1
        c = cap.Capability(g)
        # borderline guard on rank decisions
        bad = False
        for M in (c.C, c.J @ c.Nadm if len(c.J) else None):
            if M is not None and np.size(M):
                s = np.linalg.svd(np.atleast_2d(M), compute_uv=False)
                if s[0] > 0 and np.any((s / s[0] > 1e-9) & (s / s[0] < 1e-3)): bad = True
        if bad:
            skipped += 1; continue
        asm = export(g, c)
        comparable = (c.da == c.dm)
        drive_ids = [f"{c.wheels[j]['module']}.w{c.wheels[j]['index']}" for j in c.drive_wheel]
        exp = {"rank_c": 3 - c.dm, "mobility": c.dm, "actuated_rank": c.da, "driven_ids": drive_ids,
               "A": c.J.tolist(), "limits": c.phimax.tolist(),
               "gram_hard": (c.C.T @ c.C).tolist() if len(c.C) else np.zeros((3, 3)).tolist(),
               "projector_mobility": (c.Nadm @ c.Nadm.T).tolist(), "subspace_comparable": bool(comparable)}
        if comparable:
            Q = c.Q if c.k else np.zeros((3, 0))
            exp["projector_controlled"] = (Q @ Q.T).tolist()
            exp["B"] = c.B.tolist() if c.B.size else np.zeros((3, len(c.J))).tolist()
            ax = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
            exp["axis_bounds"] = [float(c.ray_limit(np.array(u, float), use_caps=False)) if c.in_drivable(np.array(u, float)) else 0.0 for u in ax]
            dirs = [rng.normal(size=3) for _ in range(3)] + [(Q @ Q.T) @ rng.normal(size=3) for _ in range(5)]
            dirs = [u / np.linalg.norm(u) for u in dirs if np.linalg.norm(u) > 1e-9]
            exp["rays"] = [{"u": u.tolist(), "s": float(c.ray_limit(u, use_caps=False))} for u in dirs]
            V = c.polytope_vertices()
            sup = [rng.normal(size=3) for _ in range(6)]; sup = [u / np.linalg.norm(u) for u in sup]
            exp["support"] = [{"u": u.tolist(), "h": float(max(0.0, (V @ u).max())) if len(V) else 0.0} for u in sup]
        vecs.append({"id": f"testbed#{len(vecs)}", "assembly": asm, "expect": exp})
    doc = {"format": "crane.fleet.capability_motion_vectors/v0", "source": "capplan.capability.Capability (use_caps=False)",
           "seed": a.seed, "tolerances": {"scalar": 1e-9, "matrix": 1e-9, "polytope": 1e-8}, "n_skipped_borderline": skipped,
           "n_subspace_comparable": sum(1 for v in vecs if v["expect"]["subspace_comparable"]), "vectors": vecs}
    json.dump(doc, open(a.out, "w"), default=float)
    print(f"wrote {len(vecs)} vectors ({doc['n_subspace_comparable']} subspace-comparable, {skipped} borderline skipped)", file=sys.stderr)


if __name__ == "__main__":
    main()
