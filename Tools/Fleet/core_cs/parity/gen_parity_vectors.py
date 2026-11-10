#!/usr/bin/env python3
"""Dump terminal-pose test vectors from Tools/Fleet/docking_mc/model.py for the C# parity test.

Usage: python3 -I gen_parity_vectors.py OUT.json
Each vector carries everything the C# side needs (guide g, acceptance angle a, magnet reach r, tolerances,
terminal lateral y / yaw psi, total height h incl. pitch arm, pitch, dead-band draws) plus the Python verdict:
  py_code: 0 missed, 1 captured only, 2 one hook, 3 both hooks;  py_cap: capture boolean.
It also records the Python constants the model uses (ell, hook_half_span) so the C# parity test can assert
its own assumptions agree with them instead of silently duplicating them.
"""
import json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, "..", "..", "docking_mc")))
import model as m  # noqa: E402

P = m.Params()
rng = np.random.default_rng(20261109)
vecs = []


def add(y, psi, ok, S, g, a, r, ty, tp, source):
    out = m.classify(y, psi, ok, S, g, a, r, ty, tp, P)
    h, pitch = m.nuisance(S, P)
    for i in range(len(out)):
        vecs.append(dict(
            source=source, g=float(g), a=float(a), r=float(r), tol_y=float(ty), tol_psi=float(tp),
            y=float(y[i]) if ok[i] else None, psi=float(psi[i]) if ok[i] else None, ok=bool(ok[i]),
            h=float(h[i]), pitch=float(pitch[i]),
            band_y=float(S["band"][i, 0] * P.band_y), band_h=float(S["band"][i, 1] * P.band_h),
            band_psi=float(S["band"][i, 2] * P.band_psi),
            py_code=int(out[i]), py_cap=bool(out[i] >= 1)))


# A) real terminal poses from the Monte Carlo (encoder-only mecanum => wide error spread, plus tag => tight spread)
cells = [("tile", "enc", "mecanum", 0.05), ("rubber", "enc", "mecanum", 0.02), ("carpet", "tag", "mecanum", 0.10), ("tile", "obs", "diff", 0.05)]
for floor, fb, drive, v in cells:
    res = m.simulate(floor, fb, drive, v, seed=7, N=24)
    for g, a, r, ty, tp in [(m.GUIDE_G[0], m.GUIDE_A[1], m.MAG_R[0], m.TOL_Y[1], m.TOL_PSI[1]),
                            (m.GUIDE_G[1], m.GUIDE_A[1], m.MAG_R[1], m.TOL_Y[0], m.TOL_PSI[0]),
                            (m.GUIDE_G[2], m.GUIDE_A[2], m.MAG_R[1], m.TOL_Y[1], m.TOL_PSI[1])]:
        add(res["y"], res["psi"], res["ok"], res["S"], g, a, r, ty, tp, f"mc:{floor}/{fb}/{drive}/{v}")

# B) synthetic, aimed at the capture boundary and the hook-spacing (yaw lever arm) zone
N = 160
S = dict(h0=rng.standard_normal(N), pitch=rng.standard_normal(N), band=np.abs(rng.standard_normal((N, 3))))
y = rng.uniform(-0.03, 0.03, N) * rng.choice([0.1, 0.3, 1.0], N)
psi = rng.uniform(-1, 1, N) * np.radians(rng.choice([1.0, 2.0, 4.0, 8.0], N))
ok = np.ones(N, bool)
g = rng.choice(m.GUIDE_G, N); a = rng.choice(m.GUIDE_A, N); r = rng.choice(m.MAG_R, N)
ty = rng.choice(m.TOL_Y, N); tp = rng.choice(m.TOL_PSI, N)
# add() takes scalar config, so group by config
for gi in m.GUIDE_G:
    for ai in m.GUIDE_A:
        for ri in m.MAG_R:
            for tyi in m.TOL_Y:
                for tpi in m.TOL_PSI:
                    sel = (g == gi) & (a == ai) & (r == ri) & (ty == tyi) & (tp == tpi)
                    if sel.any():
                        Ssel = {k: v[sel] for k, v in S.items()}
                        add(y[sel], psi[sel], ok[sel], Ssel, gi, ai, ri, tyi, tpi, "synthetic")

# C) hand-built hook-spacing cases: weak centring (big bands) so residual == error, lateral offset balanced against yaw lever arm
for ty_i in m.TOL_Y:
    for tp_i in m.TOL_PSI:
        for yy, pp in [(0.0, 0.0), (0.0015, 0.025), (-0.0015, 0.025), (0.001, -0.018), (0.002, 0.0), (0.0, 0.03), (0.0012, 0.03), (0.004, 0.0), (0.0, 0.05)]:
            S1 = dict(h0=np.zeros(1), pitch=np.zeros(1), band=np.full((1, 3), 1e9))
            S1["pitch"] = np.array([-P.pitch_mu / P.pitch_sd])  # pitch exactly 0 rad
            add(np.array([yy]), np.array([pp]), np.array([True]), S1, 0.010, 0.087, 0.008, ty_i, tp_i, "handbuilt-weak-centring")

meta = dict(py_ell=P.ell, py_hook_half_span=P.hook_half_span, py_h_sd=P.h_sd, py_pitch_arm=P.pitch_arm,
            py_band_y_sd=P.band_y, py_band_h_sd=P.band_h, py_band_psi_sd=P.band_psi, count=len(vecs))
json.dump(dict(meta=meta, vectors=vecs), open(sys.argv[1], "w"), indent=0)
print("wrote", len(vecs), "vectors; python codes:", {c: sum(v["py_code"] == c for v in vecs) for c in range(4)})
