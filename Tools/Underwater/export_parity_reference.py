"""Export reference trajectories from fossen6dof.py for the C# parity tests.

Usage: python -I Tools/Underwater/export_parity_reference.py [out.json]

Writes Tools/Underwater/golden/parity_reference.json (and the copy consumed by
the Unity EditMode tests).  The vehicle is always profiles/bluerov2_heavy.json;
the file's SHA-256 is recorded so the C# side can prove it loaded the same bytes.
All numbers are float64 and printed with repr() round-trip precision.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import vehicle_profile  # noqa: E402
from fossen6dof import (  # noqa: E402
    Simulator, State, allocate, allocation_matrix, euler_to_quat,
)

PROFILE = HERE / "profiles" / "bluerov2_heavy.json"
DT = 0.01
SAMPLE_EVERY = 10


def sample(s: State, t: float) -> dict:
    return {"t": t, "pos": s.pos.tolist(), "q": s.q.tolist(), "nu": s.nu.tolist(), "thrust": s.thrust.tolist()}


def run_case(name, params, thrusters, *, seconds, euler=(0, 0, 0), pos=(0, 0, 2.0), nu0=None,
             current=(0, 0, 0), tau_ext=None, f_cmd=None, use_thrusters=False):
    tau_ext = np.zeros(6) if tau_ext is None else np.asarray(tau_ext, float)
    sim = Simulator(params, current_n=np.asarray(current, float), thrusters=thrusters if use_thrusters else ())
    s0 = State.at_rest(pos=pos, euler=euler, n_thrusters=len(thrusters) if use_thrusters else 0)
    if nu0 is not None:
        s0.nu = np.asarray(nu0, float)
    steps = int(round(seconds / DT))
    fcmd = None if f_cmd is None else np.asarray(f_cmd, float)
    traj = [sample(s0, 0.0)]
    s = s0
    for k in range(steps):
        s = sim.step(s, DT, tau_ext, fcmd)
        if (k + 1) % SAMPLE_EVERY == 0:
            traj.append(sample(s, (k + 1) * DT))
    return {
        "name": name, "dt": DT, "steps": steps, "sampleEvery": SAMPLE_EVERY,
        "useThrusters": use_thrusters, "current": list(current), "tauExt": tau_ext.tolist(),
        "fCmd": None if fcmd is None else fcmd.tolist(),
        "q0": s0.q.tolist(), "pos0": s0.pos.tolist(), "nu0": s0.nu.tolist(),
        "thrust0": s0.thrust.tolist(), "samples": traj,
    }


def main() -> None:
    data = vehicle_profile.load(PROFILE)
    params = vehicle_profile.to_params(data)
    thr = vehicle_profile.to_thrusters(data)
    wb = params.weight - params.buoyancy
    hover_tau = [0, 0, -wb, 0, 0, 0]
    cases = [
        run_case("hover", params, thr, seconds=10, tau_ext=hover_tau),
        run_case("surge_step", params, thr, seconds=12, tau_ext=hover_tau, use_thrusters=True,
                 f_cmd=[20, 20, -20, -20, 0, 0, 0, 0]),
        run_case("roll_release", params, thr, seconds=10, euler=(np.radians(20), 0, 0)),
        run_case("yaw_spin_down", params, thr, seconds=10, nu0=[0, 0, 0, 0, 0, 1.5], tau_ext=hover_tau),
        run_case("current_drift", params, thr, seconds=20, current=(0.3, -0.2, 0.05), tau_ext=hover_tau),
        run_case("coupled_free", params, thr, seconds=8, euler=(0.3, -0.2, 0.7),
                 nu0=[0.3, 0.1, -0.1, 0.2, -0.15, 0.3], current=(0.1, 0.2, -0.05)),
    ]
    # Thrust allocation cases: full 8-thruster Heavy and a rank-deficient 4-horizontal-only subset.
    layouts = {"heavy8": list(range(8)), "horizontal4": [0, 1, 2, 3]}
    taus = [
        [30, 0, 0, 0, 0, 0], [0, 25, 0, 0, 0, 0], [0, 0, -40, 0, 0, 0], [0, 0, 0, 3, 0, 0],
        [0, 0, 0, 0, 4, 0], [0, 0, 0, 0, 0, 5], [20, -10, 15, 1, -1, 2],
        [200, 0, 0, 0, 0, 0],            # saturates
        [90, 80, -150, 8, 8, 8],         # saturates, coupled
        [0, 0, 120, 0, 0, 0],            # reverse-limited vertical push (down)
    ]
    alloc = []
    for lname, idx in layouts.items():
        ts = [thr[i] for i in idx]
        T = allocation_matrix(ts)
        fmin = np.array([-t.max_reverse for t in ts])
        fmax = np.array([t.max_forward for t in ts])
        for tau in taus:
            tau = np.asarray(tau, float)
            f_un = np.linalg.pinv(T) @ tau
            f = allocate(T, tau, fmin, fmax)
            alloc.append({"layout": lname, "indices": idx, "tau": tau.tolist(), "T": T.tolist(),
                          "fUnconstrained": f_un.tolist(), "f": f.tolist(), "wrench": (T @ f).tolist(),
                          "fMin": fmin.tolist(), "fMax": fmax.tolist()})
    out = {
        "schema": "crane.underwater.parity_reference/v1",
        "generator": "Tools/Underwater/export_parity_reference.py",
        "oracle": "Tools/Underwater/fossen6dof.py (RK4, float64)",
        "profile": "bluerov2_heavy.json",
        "profileSha256": hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
        "numpy": np.__version__,
        "cases": cases, "allocation": alloc,
    }
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "golden" / "parity_reference.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(out, indent=1))
    print(f"wrote {target} ({len(cases)} trajectories, {len(alloc)} allocation cases)")


if __name__ == "__main__":
    main()
