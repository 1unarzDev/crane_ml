"""Export oracle trajectories for the C# CraneRotorModelV2 EditMode parity tests.

Run:  python -I Tools/Aerial/export_parity_trajectories.py [output.json]
Writes Assets/Tests/Editor/AerialParity/rotor_parity_trajectories.json by default.
Pure computation; no network, no Unity, no autopilot.
"""

import hashlib
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import aerial_model as am  # noqa: E402
from aerial_profiles import PROFILE_DIR, load_vehicle  # noqa: E402

DEFAULT_OUT = HERE.parent.parent / "Assets/Tests/Editor/AerialParity/rotor_parity_trajectories.json"


def quat(axis, angle):
    return am.quat_from_axis_angle(axis, angle)


SCENARIOS = [
    # name, profile, battery, ground effect, dt, p0, q0, omega0, wind, schedule of
    # (steps, per-rotor speed fraction of omega_max)
    dict(name="x500_ige_battery_wind", profile="x500_px4_gz", battery=True, ground=True,
         dt=0.0025, p=[0.0, 0.0, 0.25], q=[1, 0, 0, 0], omega=[0, 0, 0],
         wind=[2.0, -1.0, 0.3], hover_frac=None,
         schedule=[(200, [0.78] * 4), (200, [0.80, 0.78, 0.76, 0.78]), (200, [0.9, 0.9, 0.7, 0.7]),
                   (200, [1.0, 0.6, 1.0, 0.6])]),
    dict(name="x500_tumble_no_battery", profile="x500_px4_gz", battery=False, ground=False,
         dt=0.002, p=[1.0, 2.0, 30.0], q=list(quat([1, 2, 3], 0.9)), omega=[2.0, -1.5, 1.0],
         wind=[0.0, 0.0, 0.0], hover_frac=None,
         schedule=[(300, [0.7, 0.8, 0.7, 0.8]), (300, [0.5, 0.5, 0.9, 0.9])]),
    dict(name="crazyflie_battery_ige", profile="crazyflie21", battery=True, ground=True,
         dt=0.0025, p=[0, 0, 0.03], q=list(quat([0, 1, 0], 0.15)), omega=[0.2, 0.0, -0.3],
         wind=[0.5, 0.5, 0.0], hover_frac=None,
         schedule=[(250, [0.65] * 4), (250, [0.75, 0.55, 0.75, 0.55]), (300, [0.6, 0.7, 0.6, 0.7])]),
    dict(name="crane_reference_normalized", profile="crane_reference_quad", battery=False, ground=False,
         dt=0.02, p=[0, 0, 5.0], q=[1, 0, 0, 0], omega=[0, 0, 0], wind=[1.0, 0.0, 0.0],
         hover_frac=None, schedule=[(100, [0.65] * 4), (100, [0.8, 0.6, 0.8, 0.6])]),
    dict(name="hex_firefly_battery", profile="rotors_firefly_hex", battery=True, ground=False,
         dt=0.0025, p=[0, 0, 3.0], q=list(quat([1, 1, 0], 0.2)), omega=[0.1, -0.2, 0.3],
         wind=[0.0, 1.5, 0.0], hover_frac=None,
         schedule=[(300, [0.8] * 6), (300, [0.9, 0.8, 0.7, 0.9, 0.8, 0.7])]),
]


def run(sc):
    v = load_vehicle(sc["profile"], with_battery=sc["battery"], ground_effect=sc["ground"])
    s = am.initial_state(v, p=sc["p"], q=sc["q"])
    s.omega = np.array(sc["omega"], dtype=float)
    # start the rotors at the first commanded speed so the lag is exercised from there
    first = np.array(sc["schedule"][0][1]) * np.array([r.omega_max for r in v.rotors]) * 0.9
    s.rotor_speed = first.copy()
    init = dict(p=s.p.tolist(), v=s.v.tolist(), q=s.q.tolist(), omega=s.omega.tolist(),
                rotorSpeed=s.rotor_speed.tolist(), soc=s.soc)
    wind = np.array(sc["wind"], dtype=float)
    samples = []
    step_index = 0
    commands = []
    for steps, frac in sc["schedule"]:
        cmd = (np.array(frac, dtype=float) * np.array([r.omega_max for r in v.rotors])).tolist()
        commands.append(dict(steps=steps, command=cmd))
        for _ in range(steps):
            s = am.step(v, s, cmd, sc["dt"], wind, contact=sc["ground"])
            step_index += 1
            if step_index % 10 == 0:
                samples.append(dict(
                    step=step_index, p=s.p.tolist(), v=s.v.tolist(), q=s.q.tolist(),
                    omega=s.omega.tolist(), rotorSpeed=s.rotor_speed.tolist(), soc=s.soc,
                    specificForce=am.specific_force_body(v, s, wind).tolist()))
    path = PROFILE_DIR / f"{sc['profile']}.json"
    return dict(name=sc["name"], profile=sc["profile"], profileSha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                battery=sc["battery"], groundEffect=sc["ground"], contact=sc["ground"], dt=sc["dt"], wind=sc["wind"],
                initial=init, commands=commands, samples=samples)


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_OUT
    out.parent.mkdir(parents=True, exist_ok=True)
    doc = dict(schema="crane-aerial-rotor-parity-v1", oracle="Tools/Aerial/aerial_model.py",
               scenarios=[run(sc) for sc in SCENARIOS])
    out.write_text(json.dumps(doc, indent=0, allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {out} ({out.stat().st_size} bytes, {len(doc['scenarios'])} scenarios)")


if __name__ == "__main__":
    main()
