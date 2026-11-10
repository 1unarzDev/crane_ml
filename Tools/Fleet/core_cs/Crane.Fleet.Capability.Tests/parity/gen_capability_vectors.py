#!/usr/bin/env python3
"""Generate parity vectors for Crane.Fleet.Capability (motion part) with an independent NumPy implementation.

The reference below is written separately from the C# (SVD for ranks and projectors, numpy pinv for B, np.linalg.solve vertex
enumeration for the polytope). If the capplan testbed gains `capability.py`, `--testbed` cross-checks this reference against it
through `testbed_adapter` (adapter is a stub until that API exists). Deterministic: fixed seed.

    python3 -I gen_capability_vectors.py [--n 240] [--seed 20261110] [--out capability_vectors.json]
"""
import argparse, itertools, json, math, os, sys
import numpy as np

FORMAT = "crane.fleet.capability_motion_vectors/v0"
G = math.pi / 4


# ---------------------------------------------------------------- reference model (numpy)
def wheel_rows(asm):
    """Returns driven rows (ids, A, limits), hard rows, soft rows. All in assembly frame."""
    ids, A, L, hard, soft = [], [], [], [], []
    for m in asm["modules"]:
        c, s = math.cos(m["yaw"]), math.sin(m["yaw"])
        for w in m["wheels"]:
            if w["kind"] == "wheel_caster":
                continue
            px = m["x"] + c * w["x"] - s * w["y"]
            py = m["y"] + s * w["x"] + c * w["y"]
            al = m["yaw"] + w["drive_yaw"]
            d = np.array([math.cos(al), math.sin(al)]); n = np.array([-d[1], d[0]])
            mode = w["health"]["mode"]; gain = w["health"]["gain"]
            if not w["driven"]:
                mode = "dead_free"
            if mode == "intermittent":
                mode = "ok"  # expected-authority convention (CapOptions.IntermittentWorstCase = false)
            if mode not in ("weak", "ok"):
                gain = 1.0
            if w["kind"] == "wheel_mecanum":
                a = np.array([math.cos(al + w["roller_angle"]), math.sin(al + w["roller_angle"])]); k = w["radius"] * math.cos(w["roller_angle"])
            else:
                a = d; k = w["radius"]
            row = lambda v: np.array([v[0], v[1], v[1] * px - v[0] * py])
            if mode == "dead_locked":
                hard.append(row(a))
            elif mode != "dead_free":
                ids.append(w["id"]); A.append(row(a) / k); L.append(w["max_speed"] * min(max(gain, 0), 1))
            if w["kind"] in ("wheel_diff", "wheel_conventional"):
                (soft if w["lateral"] == "skid" else hard).append(row(n))
    return ids, np.array(A).reshape(-1, 3), np.array(L), np.array(hard).reshape(-1, 3), np.array(soft).reshape(-1, 3)


def rank_basis(M, rcond=1e-6):
    if M.shape[0] == 0:
        return 0, np.eye(3), None
    U, S, Vt = np.linalg.svd(M)
    r = int((S > rcond * S[0]).sum()) if S[0] > 0 else 0
    return r, Vt[r:].T, S


def reference(asm, dirs):
    ids, A, L, Ch, _ = wheel_rows(asm)
    rc, N, Sc = rank_basis(Ch)
    if A.shape[0] and N.shape[1]:
        AN = A @ N
        U, S, Vt = np.linalg.svd(AN, full_matrices=False)
        r = int((S > 1e-6 * S[0]).sum()) if S[0] > 0 else 0
        T = N @ Vt[:r].T
    else:
        r = 0; T = np.zeros((3, 0))
    Pc = T @ T.T; Pn = N @ N.T
    B = T @ np.linalg.pinv(A @ T, rcond=1e-6) if (A.shape[0] and T.shape[1]) else np.zeros((3, A.shape[0]))
    # vertices
    d = T.shape[1]
    verts = []
    if d == 0:
        verts = [np.zeros(3)]
    else:
        Cz = A @ T
        for pick in itertools.combinations(range(A.shape[0]), d):
            for signs in itertools.product((1, -1), repeat=d):
                M = Cz[list(pick)]
                if abs(np.linalg.det(M)) < 1e-10: continue
                z = np.linalg.solve(M, np.array(signs) * L[list(pick)])
                if np.all(np.abs(Cz @ z) <= L + 1e-9 * np.maximum(1, L)):
                    verts.append(T @ z)
    V = np.array(verts)
    def ray(u):
        u = np.array(u, float)
        if np.linalg.norm(u - Pc @ u) > 1e-9 * np.linalg.norm(u): return 0.0
        au = np.abs(A @ u); m = au > 1e-12
        return float(np.min(L[m] / au[m])) if m.any() else float("inf")
    ax = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    return {
        "rank_c": rc, "mobility": 3 - rc, "actuated_rank": r,
        "driven_ids": ids, "A": A.tolist(), "limits": L.tolist(), "B": B.tolist(),
        "gram_hard": (Ch.T @ Ch).tolist() if Ch.shape[0] else np.zeros((3, 3)).tolist(),
        "projector_mobility": Pn.tolist(), "projector_controlled": Pc.tolist(),
        "axis_bounds": [ray(u) for u in ax],
        "rays": [{"u": list(map(float, u)), "s": ray(u)} for u in dirs["ray"]],
        "support": [{"u": list(map(float, u)), "h": float(max(0.0, (V @ np.array(u)).max()))} for u in dirs["support"]],
        "n_vertices": int(len(V)),
    }, (Sc, S if A.shape[0] and N.shape[1] else None, Pc)


# ---------------------------------------------------------------- configuration generation
def wheel(wid, kind, x, y, drive_yaw=0.0, radius=0.05, roller=G, vmax=20.0, driven=True, mode="ok", gain=1.0,
          lateral="hard", load=0.0, mu=0.0):
    return {"id": wid, "kind": kind, "x": x, "y": y, "drive_yaw": drive_yaw, "radius": radius, "roller_angle": roller,
            "max_speed": vmax, "driven": driven, "health": {"mode": mode, "gain": gain}, "lateral": lateral,
            "normal_load": load, "lateral_mu": mu}


def mecanum_rover(mid, x, y, yaw, lx, ly, r, vmax):
    ws = [wheel(f"{mid}.fl", "wheel_mecanum", lx, ly, 0, r, -G, vmax), wheel(f"{mid}.fr", "wheel_mecanum", lx, -ly, 0, r, G, vmax),
          wheel(f"{mid}.rl", "wheel_mecanum", -lx, ly, 0, r, G, vmax), wheel(f"{mid}.rr", "wheel_mecanum", -lx, -ly, 0, r, -G, vmax)]
    return {"id": mid, "x": x, "y": y, "yaw": yaw, "wheels": ws}


def diff_pod(mid, x, y, yaw, lateral="hard", ht=0.07, r=0.055, vmax=24.0, load=0.0, mu=0.0):
    return {"id": mid, "x": x, "y": y, "yaw": yaw, "wheels": [
        wheel(f"{mid}.l", "wheel_diff", 0, ht, 0, r, vmax=vmax, lateral=lateral, load=load, mu=mu),
        wheel(f"{mid}.r", "wheel_diff", 0, -ht, 0, r, vmax=vmax, lateral=lateral, load=load, mu=mu)]}


SLOTS = {"compact_diff": [(0.23, 0.2), (0.23, -0.2), (-0.23, 0.2), (-0.23, -0.2), (0, 0.2), (0, -0.2)],
         "narrow_tandem": [(0.71, 0.2), (0.71, -0.2), (-0.71, 0.2), (-0.71, -0.2), (0.35, 0.2), (-0.35, -0.2)]}


def six_pod(name, lateral):
    load = (5.0 + 6 * 2.1) * 9.81 / 12
    return {"id": f"gz_{name}_{lateral}", "modules": [diff_pod(f"pod_{i}", sx, sy, 0.0, lateral, load=load, mu=0.08) for i, (sx, sy) in enumerate(SLOTS[name])]}


def random_health(rng, asm, nfail):
    ws = [w for m in asm["modules"] for w in m["wheels"] if w["kind"] != "wheel_caster"]
    rng.shuffle(ws)
    for w in ws[:nfail]:
        mode = rng.choice(["dead_free", "dead_locked", "weak", "intermittent"])
        w["health"] = {"mode": mode, "gain": float(rng.uniform(0.2, 0.9)) if mode in ("weak", "intermittent") else 1.0}
    return asm


def random_assembly(rng, i):
    nm = int(rng.integers(1, 5)); mods = []
    for k in range(nm):
        mx, my, myaw = rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-math.pi, math.pi)
        ws = []
        for j in range(int(rng.integers(2, 5))):
            kind = rng.choice(["wheel_mecanum", "wheel_diff", "wheel_omni", "wheel_caster"], p=[0.35, 0.35, 0.2, 0.1])
            ws.append(wheel(f"m{k}.w{j}", str(kind), rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-math.pi, math.pi),
                            rng.uniform(0.03, 0.1), float(rng.choice([-G, G, rng.uniform(0.3, 1.2) * rng.choice([-1, 1])])),
                            rng.uniform(10, 30), bool(rng.random() > 0.05), lateral=str(rng.choice(["hard", "hard", "skid"])),
                            load=rng.uniform(5, 40), mu=rng.uniform(0.05, 0.8)))
        mods.append({"id": f"m{k}", "x": mx, "y": my, "yaw": myaw, "wheels": ws})
    return {"id": f"random_{i}", "modules": mods}


def configurations(n, seed):
    rng = np.random.default_rng(seed); out = []
    # 1. mecanum rovers with single and double faults
    for fault in [None] + [(w, m) for w in range(4) for m in ("dead_free", "dead_locked", "weak")]:
        a = {"id": f"mecanum_{fault}", "modules": [mecanum_rover("r", 0, 0, 0, 0.15, 0.12, 0.05, 20.0)]}
        if fault:
            wh = a["modules"][0]["wheels"][fault[0]]
            wh["health"] = {"mode": fault[1], "gain": 0.5 if fault[1] == "weak" else 1.0}
        out.append(a)
    for k in range(10):
        a = {"id": f"mecanum_double_{k}", "modules": [mecanum_rover("r", 0, 0, 0, 0.15, 0.12, 0.05, 20.0)]}
        out.append(random_health(rng, a, 2))
    # 2. docked mecanum pairs/triples (side by side), with faults
    for k in range(14):
        n_r = 2 + k % 2
        a = {"id": f"mecanum_chain_{k}", "modules": [mecanum_rover(f"r{j}", 0.3 * j, 0.0, 0.0, 0.15, 0.12, 0.05, 20.0) for j in range(n_r)]}
        out.append(random_health(rng, a, k % 4))
    # 3. differential pods, hard and skid, and Gazebo platforms
    for lat in ("hard", "skid"):
        a = {"id": f"diff_pod_{lat}", "modules": [diff_pod("p", 0, 0, 0, lat, mu=0.08, load=20)]}
        out.append(a)
    for name in SLOTS:
        for lat in ("hard", "skid"):
            out.append(six_pod(name, lat))
            for f in range(6):
                out.append(random_health(rng, six_pod(name, lat), 1 + f % 3))
    # 4. generic random
    i = 0
    while len(out) < n:
        a = random_assembly(rng, i); i += 1
        out.append(random_health(rng, a, int(rng.integers(0, 4))))
    return out[:n]


def directions(rng, Pc):
    ray = [rng.normal(size=3) for _ in range(3)]
    ray += [Pc @ rng.normal(size=3) for _ in range(5)]
    ray = [u / np.linalg.norm(u) for u in ray if np.linalg.norm(u) > 1e-9]
    sup = [rng.normal(size=3) for _ in range(6)]
    sup = [u / np.linalg.norm(u) for u in sup]
    return {"ray": ray, "support": sup}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=240)
    ap.add_argument("--seed", type=int, default=20261110)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "capability_vectors.json"))
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed + 1)
    vecs = []; skipped = 0
    for asm in configurations(a.n + 40, a.seed):
        if len(vecs) >= a.n: break
        # first pass for the controlled projector, to draw in-subspace directions
        _, (Sc, S, Pc) = reference(asm, {"ray": [], "support": []})
        # reject borderline rank decisions (would make rank implementation-dependent)
        bad = False
        for sv in (Sc, S):
            if sv is not None and len(sv) and sv[0] > 0:
                ratio = sv / sv[0]
                if np.any((ratio > 1e-9) & (ratio < 1e-3)): bad = True
        if bad:
            skipped += 1; continue
        dirs = directions(rng, Pc)
        exp, _ = reference(asm, dirs)
        vecs.append({"id": asm["id"] + f"#{len(vecs)}", "assembly": asm, "expect": exp})
    doc = {"format": FORMAT, "seed": a.seed, "generator": "gen_capability_vectors.py (independent numpy reference)",
           "tolerances": {"scalar": 1e-9, "matrix": 1e-9, "polytope": 1e-8}, "n_skipped_borderline": skipped, "vectors": vecs}
    json.dump(doc, open(a.out, "w"), indent=0, default=float)
    print(f"wrote {len(vecs)} vectors ({skipped} borderline skipped) -> {a.out}", file=sys.stderr)
    if os.path.exists(os.path.join(os.path.dirname(__file__), "..", "..", "..", "capplan", "capability.py")):
        print("NOTE: capplan/capability.py exists; implement testbed_adapter() to cross-check this reference against it.", file=sys.stderr)


if __name__ == "__main__":
    main()
