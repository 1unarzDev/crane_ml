"""parity/cases.json is current, uses only JsonUtility-friendly shapes, and replays on the twin."""
import json
import math

import parity
import plant as P


def load():
    with open(parity.OUT) as f:
        return f.read()


def test_cases_file_is_current_and_deterministic():
    assert load() == parity.dumps(parity.build()), "regenerate: python3 -I Tools/Characterization/parity.py"


def test_layout_is_jsonutility_friendly():
    doc = json.loads(load())

    def no_nested_arrays(node):
        if isinstance(node, list):
            assert all(not isinstance(x, list) for x in node)
            for x in node:
                no_nested_arrays(x)
        elif isinstance(node, dict):
            for v in node.values():
                no_nested_arrays(v)
    no_nested_arrays(doc)
    names = {c["name"] for c in doc["cases"]}
    assert names == {f"{n}_dt{dt}" for dt in (0.02, 0.01) for n in ("steps", "turn", "reverse", "flip", "current")}
    for c in doc["cases"]:
        n, k = c["steps"], len(c["expectedTimes"])
        assert len(c["pwm"]) == 4 * n and len(c["expected"]) == 6 * k and len(c["expectedTicks"]) == k
        assert abs(n * c["dt"] - 10.0) < 1e-9 and k == 20
        assert all(abs(t - tk * c["dt"]) < 1e-9 for t, tk in zip(c["expectedTimes"], c["expectedTicks"]))
    cur = next(c for c in doc["cases"] if c["name"] == "current_dt0.02")
    assert (cur["currentX"], cur["currentY"]) == (0.3, -0.2)


def test_replay_matches_expected():
    doc = json.loads(load())
    prof = P.load_profile()
    for c in doc["cases"]:
        pl = P.ProfilePlant(prof, c["dt"], current=(c["currentX"], c["currentY"]))
        pl.reset(c["initialState"])
        want = {tk: c["expected"][6 * j:6 * j + 6] for j, tk in enumerate(c["expectedTicks"])}
        for k in range(c["steps"]):
            pl.step_pwm(c["pwm"][4 * k:4 * k + 4])
            if k + 1 in want:
                for a, b in zip(pl.state, want[k + 1]):
                    assert abs(a - b) <= doc["absTolerance"] + doc["relTolerance"] * abs(b)
                    assert a == b  # same code, same inputs: bit-identical
        assert all(math.isfinite(x) for x in pl.state)
