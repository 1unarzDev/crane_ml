"""Generate parity/cases.json: deterministic inputs and expected states of the exact twin
(plant.ProfilePlant) for the C# MarinePlanarModel EditMode parity test (design section 7).

    python3 -I Tools/Characterization/parity.py            # rewrite parity/cases.json
    python3 -I Tools/Characterization/parity.py --check    # exit 1 if the file is stale

JSON layout (Unity JsonUtility compatible: objects, and arrays of numbers or of objects only):

    {
      "schema": "crane.parity/1",            string
      "profileFile": "profiles/ideal.json",  string, relative to Tools/Characterization
      "profileSha256": "<hex>",              string, sha256 of that file's bytes when generated
      "relTolerance": 1e-4,                  number
      "absTolerance": 1e-6,                  number; a component passes if
                                             |actual - expected| <= absTolerance + relTolerance*|expected|
      "stateLayout": "x,y,psi,vx,vy,r",      string, world ENU m, rad (CCW from +x), m/s, rad/s
      "cases": [                             array of Case objects
        {
          "name": "steps_dt0.02",            string
          "dt": 0.02,                        number, s (fixed step of every tick)
          "steps": 500,                      int, N ticks
          "currentX": 0.0, "currentY": 0.0,  numbers, constant world current m/s
          "initialState": [6 numbers],       state before the first tick, stateLayout order
          "pwm": [4*N numbers],              flat, tick-major: pwm[4*k + i] is thruster i's pulse (us)
                                             at tick k, thrusters in profile order
                                             (front_left, front_right, rear_right, rear_left)
          "expectedTimes": [K numbers],      s, sample times (every 0.5 s, 0.5 .. 10.0)
          "expectedTicks": [K ints],         ticks completed at each sample (= round(t/dt))
          "expected": [6*K numbers]          flat: expected[6*j + c] is component c of the state
                                             after expectedTicks[j] ticks
        }
      ]
    }

Each tick: thrust from all thrusters in profile order (beta = 1, V = vRef), then one plant step
with that thrust. The plant and thrusters start reset (zero lag state, zero delay buffers).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # python3 -I drops the script dir
import plant as P  # noqa: E402

OUT = os.path.join(P.HERE, "parity", "cases.json")
DURATION, SAMPLE_EVERY = 10.0, 0.5
DTS = (0.02, 0.01)


def prog_steps(k, t):
    if t < 3.0:
        return P.pwm_for_effort(surge=0.5)
    if t < 6.0:
        return P.pwm_for_effort(surge=1.0)
    return P.pwm_for_effort()


def prog_turn(k, t):
    return P.pwm_for_effort(surge=0.6, yaw=0.3) if t < 7.0 else P.pwm_for_effort(sway=-0.4)


def prog_reverse(k, t):
    if t < 4.0:
        return P.pwm_for_effort(surge=-1.0)
    if t < 7.0:
        return P.pwm_for_effort(sway=0.7, yaw=-0.2)
    return P.pwm_for_effort()


def prog_flip(k, t):
    """Full sway and half yaw, flipping sign every tick, after a 2 s surge start."""
    if t < 2.0:
        return P.pwm_for_effort(surge=0.8)
    s = 1.0 if k % 2 == 0 else -1.0
    return P.pwm_for_effort(surge=0.3, sway=s, yaw=0.5 * s)


PROGRAMS = (("steps", prog_steps, (0.0, 0.0)),
            ("turn", prog_turn, (0.0, 0.0)),
            ("reverse", prog_reverse, (0.0, 0.0)),
            ("flip", prog_flip, (0.0, 0.0)),
            ("current", prog_steps, (0.3, -0.2)))


def make_case(profile, name, fn, current, dt):
    n = int(round(DURATION / dt))
    pl = P.ProfilePlant(profile, dt, current=current)
    sample_ticks = [int(round(SAMPLE_EVERY * (j + 1) / dt)) for j in range(int(round(DURATION / SAMPLE_EVERY)))]
    wanted = set(sample_ticks)
    pwm, expected, times = [], [], []
    for k in range(n):
        p = [float(x) for x in fn(k, k * dt)]
        pwm.extend(p)
        pl.step_pwm(p)
        if (k + 1) in wanted:
            times.append(round((k + 1) * dt, 12))
            expected.extend(pl.state)
    return {"name": f"{name}_dt{dt}", "dt": dt, "steps": n, "currentX": current[0], "currentY": current[1],
            "initialState": [0.0] * 6, "pwm": pwm, "expectedTimes": times, "expectedTicks": sample_ticks,
            "expected": expected}


def build():
    with open(P.IDEAL_PATH, "rb") as f:
        raw = f.read()
    profile = json.loads(raw)
    cases = [make_case(profile, name, fn, cur, dt) for dt in DTS for name, fn, cur in PROGRAMS]
    return {"schema": "crane.parity/1", "profileFile": "profiles/ideal.json",
            "profileSha256": hashlib.sha256(raw).hexdigest(), "relTolerance": 1e-4, "absTolerance": 1e-6,
            "stateLayout": "x,y,psi,vx,vy,r", "cases": cases}


def dumps(doc):
    return json.dumps(doc, separators=(",", ":"), allow_nan=False) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if parity/cases.json is stale")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    text = dumps(build())
    if a.check:
        try:
            with open(a.out) as f:
                same = f.read() == text
        except OSError:
            same = False
        print("parity cases up to date" if same else f"parity cases stale: regenerate {a.out}")
        return 0 if same else 1
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write(text)
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
