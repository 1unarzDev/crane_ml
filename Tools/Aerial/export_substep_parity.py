"""Reference trajectories for the C# sub-stepped plant (CraneAerialPlantCore) parity tests.

Run:  python -I Tools/Aerial/export_substep_parity.py [--report]
Writes Tools/Aerial/parity/aos7_substep_parity.json.

For each scenario the per-rotor speed command is a function of time evaluated at control ticks
t_k = k * control_dt and held (ZOH) until the next tick, identically in Python and C#:
  * reference : aerial_model.step at REF_DT = 1e-4 s (10 kHz)
  * plant     : aerial_model.step at PLANT_DT = 1e-3 s (1 kHz), the scheme the C# core runs inside
                each Unity FixedUpdate; C# must reproduce this to ~1e-9 (implementation identity)
  * naive     : one step per Unity fixed step (50 Hz), for the error table only
Pure computation; no network, no Unity.
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

OUT = HERE / "parity" / "aos7_substep_parity.json"
PROFILE = "aos7_v5_f90_6s_v1"  # parity fixture pinned to v1
REF_DT = 1e-4
PLANT_DT = 1e-3
UNITY_DT = 0.02
SAMPLE_DT = 0.01


def schedule(name, t, w_h, w_max):
    """Per-rotor speed command [rad/s] in profile order FR, RL, FL, RR. Mirrored in C# tests."""
    if name == "step":
        if t < 0.5:
            return [w_h] * 4
        if t < 1.0:
            return [1.15 * w_h] * 4
        if t < 1.3:   # roll: right rotors (FR, RR) up, left (RL, FL) down -> negative roll torque
            return [1.06 * w_h, 0.94 * w_h, 0.94 * w_h, 1.06 * w_h]
        return [w_h] * 4
    if name == "chirp":
        f0, f1, T = 0.5, 10.0, 3.0
        tt = min(t, T)
        phase = 2 * math.pi * (f0 * tt + 0.5 * (f1 - f0) / T * tt * tt)
        d = 0.04 * w_h * math.sin(phase)
        # pitch: front (FR, FL) minus rear (RL, RR)
        return [w_h + d, w_h - d, w_h + d, w_h - d]
    if name == "saturation":
        if t < 0.6:
            return [1.3 * w_max] * 4              # above the (battery-sagged) limit: clamped
        if t < 0.9:
            return [w_max, w_max, 0.2 * w_max, 0.2 * w_max]   # CCW pair max, CW pair low: yaw
        if t < 1.2:
            return [0.0] * 4                      # idle (omega_min = 0)
        return [w_h] * 4
    if name == "pitch_flip":
        if t < 0.3:
            return [w_h] * 4
        if t < 0.38:   # nose-down pitch torque: rear (RL, RR) high, front (FR, FL) low
            return [0.35 * w_max, 0.9 * w_max, 0.35 * w_max, 0.9 * w_max]
        if t < 0.62:
            return [0.45 * w_max] * 4
        if t < 0.70:   # brake
            return [0.9 * w_max, 0.35 * w_max, 0.9 * w_max, 0.35 * w_max]
        return [1.25 * w_h] * 4
    raise KeyError(name)


SCENARIOS = [
    dict(name="step", duration=2.0, control_dt=0.002, p=[0.0, 0.0, 0.30], wind=[1.0, -0.5, 0.0],
         ground=True, contact=True, battery=True),
    dict(name="chirp", duration=3.0, control_dt=0.002, p=[0.0, 0.0, 5.0], wind=[0.0, 0.0, 0.0],
         ground=False, contact=False, battery=True),
    dict(name="saturation", duration=1.6, control_dt=0.002, p=[0.0, 0.0, 5.0], wind=[0.0, 2.0, 0.0],
         ground=False, contact=False, battery=True),
    dict(name="pitch_flip", duration=1.4, control_dt=0.002, p=[0.0, 0.0, 10.0], wind=[0.0, 0.0, 0.0],
         ground=False, contact=False, battery=True),
]


def simulate(v, sc, dt, w_h, w_max, scheme="exact_stage"):
    s = am.initial_state(v, p=sc["p"], rotor_speed=w_h)
    wind = np.array(sc["wind"], dtype=float)
    n_total = int(round(sc["duration"] / dt))
    ctrl_every = max(1, int(round(sc["control_dt"] / dt))) if dt < sc["control_dt"] else 1
    sample_every = int(round(SAMPLE_DT / dt)) if dt <= SAMPLE_DT else None
    out = []
    cmd = None
    k_ctrl = 0
    for n in range(n_total):
        if dt < sc["control_dt"]:
            if n % ctrl_every == 0:
                cmd = schedule(sc["name"], k_ctrl * sc["control_dt"], w_h, w_max)
                k_ctrl += 1
        else:
            cmd = schedule(sc["name"], n * dt, w_h, w_max)
        s = am.step(v, s, cmd, dt, wind, contact=sc["contact"], motor_scheme=scheme)
        if sample_every and (n + 1) % sample_every == 0:
            out.append(snapshot(v, s, wind))
        elif sample_every is None:
            out.append(snapshot(v, s, wind))
    return out


def snapshot(v, s, wind):
    return dict(t=round(s.t, 9), p=s.p.tolist(), v=s.v.tolist(), q=s.q.tolist(), omega=s.omega.tolist(),
                rotorSpeed=s.rotor_speed.tolist(), soc=s.soc)


def quat_angle(a, b):
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def errors(test, ref, stride=1):
    """RMS errors of `test` against `ref` (ref sampled `stride` times denser)."""
    ep, ev, ea, ew, er = [], [], [], [], []
    for i, a in enumerate(test):
        b = ref[(i + 1) * stride - 1]
        ep.append(np.linalg.norm(np.subtract(a["p"], b["p"])))
        ev.append(np.linalg.norm(np.subtract(a["v"], b["v"])))
        ea.append(quat_angle(np.array(a["q"]), np.array(b["q"])))
        ew.append(np.linalg.norm(np.subtract(a["omega"], b["omega"])))
        er.append(np.linalg.norm(np.subtract(a["rotorSpeed"], b["rotorSpeed"])) / 2.0)
    rms = lambda x: float(np.sqrt(np.mean(np.square(x))))
    return dict(pos=rms(ep), vel=rms(ev), att=rms(ea), rate=rms(ew), rotor=rms(er),
                pos_max=float(np.max(ep)), att_max=float(np.max(ea)))


def main():
    v = load_vehicle(PROFILE, with_battery=True, ground_effect=False)
    w_h = am.hover_rotor_speed(v)
    w_max = v.rotors[0].omega_max
    scenarios, table = [], []
    for sc in SCENARIOS:
        v = load_vehicle(PROFILE, with_battery=sc["battery"], ground_effect=sc["ground"])
        ref = simulate(v, sc, REF_DT, w_h, w_max)
        plant = simulate(v, sc, PLANT_DT, w_h, w_max)
        naive = simulate(v, sc, UNITY_DT, w_h, w_max)
        legacy = simulate(v, sc, PLANT_DT, w_h, w_max, scheme="zoh_first")
        e_plant = errors(plant, ref)
        e_legacy = errors(legacy, ref)
        e_naive = errors(naive, ref[int(round(UNITY_DT / SAMPLE_DT)) - 1::int(round(UNITY_DT / SAMPLE_DT))])
        table.append((sc["name"], e_plant, e_naive, e_legacy))
        scenarios.append(dict(name=sc["name"], duration=sc["duration"], controlDt=sc["control_dt"],
                              p0=sc["p"], wind=sc["wind"], groundEffect=sc["ground"], contact=sc["contact"],
                              battery=sc["battery"], hoverRotorSpeed=w_h, omegaMax=w_max,
                              reference=ref, plant1kHz=plant,
                              pythonPlantVsReferenceRms=e_plant, pythonNaive50HzVsReferenceRms=e_naive,
                              pythonZohFirst1kHzVsReferenceRms=e_legacy))
        if sc["name"] == "pitch_flip":
            # report the flip: max pitch excursion via body x axis z-component going negative
            pitch_min = min(am.quat_to_matrix(np.array(s["q"]))[2, 2] for s in ref)
            print(f"pitch_flip: min body-z up component {pitch_min:.3f} (negative = inverted)")
    path = PROFILE_DIR / f"{PROFILE}.json"
    doc = dict(schema="crane-aerial-substep-parity-v1", oracle="Tools/Aerial/aerial_model.py",
               profile=PROFILE, profileSha256=hashlib.sha256(path.read_bytes()).hexdigest(),
               referenceDt=REF_DT, plantDt=PLANT_DT, unityDt=UNITY_DT, sampleDt=SAMPLE_DT, scenarios=scenarios)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(doc, separators=(",", ":"), allow_nan=False) + "\n", encoding="utf-8")
    print(f"wrote {OUT} ({OUT.stat().st_size} bytes)")
    print("| scenario | 1 kHz exact-stage: pos [m] | att [rad] | rate [rad/s] | rotor [rad/s] |"
          " 1 kHz zoh-first: pos | att | naive 50 Hz: pos | att | rotor |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for name, a, b, c in table:
        print(f"| {name} | {a['pos']:.1e} | {a['att']:.1e} | {a['rate']:.1e} | {a['rotor']:.1e} | "
              f"{c['pos']:.1e} | {c['att']:.1e} | {b['pos']:.1e} | {b['att']:.1e} | {b['rotor']:.1e} |")


if __name__ == "__main__":
    main()
