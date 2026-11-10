"""Seed/disturbance split lookup and the held-out guard (twin of DroneSplitGuard.cs; same splits.json)."""
import json
import os
import sys

HELDOUT = "heldout"
ALLOW_FLAG = "--allow-heldout"
DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "splits.json")


class HeldoutRefused(Exception):
    pass


def load(path=DEFAULT_PATH):
    with open(path) as f:
        s = json.load(f)
    if s.get("schema") != "crane-drone-splits/1":
        raise ValueError("not a crane-drone-splits/1 file")
    validate(s)
    return s


def validate(s):
    """Seed ranges must be well formed and disjoint."""
    rs = sorted(s["seed_ranges"], key=lambda r: r["start"])
    for r in rs:
        if r["start"] > r["end"]:
            raise ValueError("bad range %r" % (r,))
    for a, b in zip(rs, rs[1:]):
        if b["start"] <= a["end"]:
            raise ValueError("overlapping ranges %r %r" % (a, b))
    seen = {}
    for d in s.get("disturbance", []):
        for v in d["values"]:
            key = (d["family"], v)
            if key in seen and seen[key] != d["split"] and HELDOUT in (seen[key], d["split"]):
                raise ValueError("held-out disturbance %r also in %s" % (key, seen[key]))
            seen[key] = d["split"]


def split_of_seed(s, seed):
    for r in s["seed_ranges"]:
        if r["start"] <= seed <= r["end"]:
            return r["split"]
    return "unassigned"


def split_of_disturbance(s, family, value):
    for d in s.get("disturbance", []):
        if d["family"] == family and value in d["values"]:
            if d["split"] == HELDOUT:
                return HELDOUT
    for d in s.get("disturbance", []):
        if d["family"] == family and value in d["values"]:
            return d["split"]
    return "unassigned"


def enforce(s, seed, wind=None, lighting=None, allow_heldout=False):
    """Return the seed's split or raise HeldoutRefused (held-out or unassigned) unless allowed."""
    split = split_of_seed(s, seed)
    bad = None
    if split == HELDOUT:
        bad = "seed %d is HELD-OUT" % seed
    elif split == "unassigned":
        bad = "seed %d is in no split" % seed
    elif wind is not None and split_of_disturbance(s, "wind", wind) == HELDOUT:
        bad = "wind %r is HELD-OUT" % wind
    elif lighting is not None and split_of_disturbance(s, "lighting", lighting) == HELDOUT:
        bad = "lighting %r is HELD-OUT" % lighting
    if bad and not allow_heldout:
        raise HeldoutRefused("%s; refusing (pass %s for the final evaluation only)" % (bad, ALLOW_FLAG))
    if bad:
        print("CRANE_DRONE_HELDOUT_ALLOWED " + bad, file=sys.stderr)
    return split


def enforce_argv(argv, path=DEFAULT_PATH):
    """For Python capture/logging entry points: reads --drone-seed/--drone-wind/--drone-lighting/--allow-heldout."""
    def arg(n):
        return argv[argv.index(n) + 1] if n in argv and argv.index(n) + 1 < len(argv) else None
    seed = arg("--drone-seed")
    if seed is None:
        raise HeldoutRefused("--drone-seed missing; refusing (seed must be declared)")
    return enforce(load(path), int(seed), arg("--drone-wind"), arg("--drone-lighting"), ALLOW_FLAG in argv)


if __name__ == "__main__":
    try:
        print(enforce_argv(sys.argv[1:]))
    except HeldoutRefused as e:
        print(e, file=sys.stderr)
        sys.exit(2)
