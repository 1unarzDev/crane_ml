"""The unit catalogue: which capplan module graphs scenes may put in a fleet, with their resolved capability summary.

Scenes reference units by catalogue key; each key resolves to a capplan graph, either an item of a frozen manifest
(``{"manifest": "train_v1", "config_hash": ...}``) or a composition of TRAIN-pool module types
(``{"compose": {"types": [...], "topology": "single|chain|side|L"}}``). Only train-pool types and train_v1 items are
used, so every scene split (train or held-out) runs train configurations: the capplan H7 control (environment
generalization is not confounded with configuration generalization).

    python3 -m scenes.catalog build      # needs capplan (read-only import from $FLEET_CAPPLAN_ROOT); writes catalog/units_v1.json
    python3 -m scenes.catalog check      # recomputes every entry with capplan and compares (skips without capplan)

At run time nothing imports capplan: fastsim and the compilers read the frozen JSON.
"""
import argparse
import json
import math
import sys

from .common import CAPPLAN_ROOT, HERE, sha, canonical

CATALOG_PATH = HERE / "catalog" / "units_v1.json"

# key -> (graph reference, CRANE profile role or None)
SPEC = {
    "carrier": ({"manifest": "train_v1", "config_hash": "d0b9ad389bac696b"}, "cargo"),
    "carrier_v3": ({"manifest": "train_v1", "config_hash": "f39a546b111760ea"}, None),
    "diff_carrier": ({"manifest": "train_v1", "config_hash": "dd124db40db92d29"}, None),
    "lifter": ({"manifest": "train_v1", "config_hash": "ae7a9c321cbf8b21"}, "lifter"),
    "scout": ({"compose": {"types": ["scout"], "topology": "single"}}, "scout"),
    "pair_chain": ({"compose": {"types": ["carrier", "carrier"], "topology": "chain"}}, None),
    "pair_side": ({"compose": {"types": ["carrier", "carrier"], "topology": "side"}}, None),
    "l3_carrier": ({"compose": {"types": ["carrier", "carrier", "carrier"], "topology": "L"}}, None),
}
# health variants computed for faults: name -> {module: [wheel health]} (wheel order fl, fr, rl, rr)
HEALTH = {
    "fl_dead_locked": ["dead_locked", "ok", "ok", "ok"],
    "fl_dead_free": ["dead_free", "ok", "ok", "ok"],
    "fl_weak": ["weak", "ok", "ok", "ok"],
    "all_weak": ["weak", "weak", "weak", "weak"],
}
WHEEL_NAMES = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
# perception stand-ins per type family (camera height m, hfov deg, range m); assumptions, not from capplan
CAMERAS = {"scout": (0.45, 90.0, 6.0), "lifter": (0.15, 70.0, 4.0), "carrier": (0.15, 70.0, 4.0),
           "diff_carrier": (0.15, 70.0, 3.0)}


def _capplan():
    if str(CAPPLAN_ROOT) not in sys.path:
        sys.path.append(str(CAPPLAN_ROOT))
    from capplan import configs, modgraph  # noqa: F401  (read-only import)
    return configs, modgraph


def _graph(ref):
    configs, mg = _capplan()
    if "manifest" in ref:
        man = json.loads((CAPPLAN_ROOT / "capplan" / "manifests" / (ref["manifest"] + ".json")).read_text())
        item = next(i for i in man["items"] if i["config_hash"] == ref["config_hash"])
        return mg.ModuleGraph.from_dict(item["graph"]), {"manifest_hash": man["manifest_hash"], "split": man["split"]}
    c = ref["compose"]
    t, topo = c["types"], c["topology"]
    if topo == "single":
        g = mg.single(t[0])
    elif topo == "chain":
        g = mg.chain(t, "front")
    elif topo == "side":
        g = mg.chain(t, "left")
    elif topo == "L":
        g = mg.single(t[0])
        mg.attach(g, "m0", "front", t[1], "rear", new_id="m1")
        mg.attach(g, "m0", "left", t[2], "right", new_id="m2")
        mg.recentre(g)
    else:
        raise ValueError(topo)
    return g, {"types_from": "train pool (capplan.configs.train_pool)"}


def _family(tname):
    base = tname.split(".")[0].split("+")[0]
    return base


def _summary(g):
    configs, mg = _capplan()
    d = configs.descriptor(g)
    f = d["features"]
    vm = {k: round(float(v), 4) for k, v in d["motion"]["vmax"]["value"].items()}
    mods = []
    for m in g.modules.values():
        s = m.spec()
        mods.append({"id": m.id, "type": m.type, "x": round(m.x, 4), "y": round(m.y, 4), "yaw": round(m.yaw, 4),
                     "size": [round(v, 4) for v in s["size"]], "height": round(float(s.get("height", 0.14)), 4),
                     "faces": {k: {"x": round(v["x"], 4), "y": round(v["y"], 4), "yaw": round(v["yaw"], 4),
                                   "type": v["type"], "approach_len": v.get("approach_len", 0.1)}
                               for k, v in s["faces"].items()},
                     "camera": CAMERAS.get(_family(m.type), CAMERAS["carrier"])})
    used = {(c.a, c.face_a) for c in g.couplings} | {(c.b, c.face_b) for c in g.couplings}
    for m in mods:
        for fn in list(m["faces"]):
            m["faces"][fn]["free"] = (m["id"], fn) not in used
    corners = []
    for m in mods:
        c, s_ = math.cos(m["yaw"]), math.sin(m["yaw"])
        hx, hy = m["size"][0] / 2, m["size"][1] / 2
        corners += [(m["x"] + c * u - s_ * v, m["y"] + s_ * u + c * v) for u in (-hx, hx) for v in (-hy, hy)]
    aabb = [round(float(min(p[0] for p in corners)), 4), round(float(min(p[1] for p in corners)), 4),
            round(float(max(p[0] for p in corners)), 4), round(float(max(p[1] for p in corners)), 4)]
    xs, ys = [max(-aabb[0], aabb[2])], [max(-aabb[1], aabb[3])]
    circ = float(max(math.hypot(*p) for p in corners))
    return {"config_hash": g.config_hash(), "mobility_class": f["mobility_class"],
            "lateral": vm["left"] > 1e-6 and vm["right"] > 1e-6, "vmax": vm, "min_width": f["min_width"],
            "payloads": f["payloads"], "tools": f["tools"], "mass": f["mass"], "n_modules": f["n_modules"],
            "modules": mods, "couplings": [[c.a, c.face_a, c.b, c.face_b] for c in g.couplings],
            "half_extent": [round(float(max(xs)), 4), round(float(max(ys)), 4)], "aabb": aabb,
            "circumradius": round(circ, 4),
            "descriptor_version": d["descriptor_version"]}


def build():
    configs, mg = _capplan()
    units = {}
    for key, (ref, role) in SPEC.items():
        g, prov = _graph(ref)
        s = _summary(g)
        s.update(key=key, ref=ref, provenance=prov, crane_role=role)
        variants = {}
        if s["n_modules"] == 1:
            for name, hs in HEALTH.items():
                gd = g.to_dict()
                gd["modules"][0]["act_health"] = list(hs)
                gd["modules"][0]["act_gain"] = [0.5 if h == "weak" else 1.0 for h in hs]
                gv = mg.ModuleGraph.from_dict(gd)
                v = _summary(gv)
                variants[name] = {k: v[k] for k in ("config_hash", "mobility_class", "lateral", "vmax", "min_width",
                                                    "payloads")}
        s["health_variants"] = variants
        units[key] = s
    doc = {"schema": "crane.fleet_scene_catalog/v1", "note": __doc__.split("\n")[0],
           "capplan_root": str(CAPPLAN_ROOT), "units": units}
    doc["catalog_hash"] = sha(units)
    CATALOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CATALOG_PATH.write_text(json.dumps(doc, indent=1) + "\n")
    return doc


_CACHE = {}


def load():
    if "doc" not in _CACHE:
        _CACHE["doc"] = json.loads(CATALOG_PATH.read_text())
    return _CACHE["doc"]


def unit(key):
    return load()["units"][key]


def check():
    """Problems between the frozen catalogue and a fresh capplan computation ([] when equal)."""
    frozen = load()["units"]
    problems = []
    for key, (ref, role) in SPEC.items():
        g, _ = _graph(ref)
        s = _summary(g)
        f = frozen.get(key)
        if f is None:
            problems.append("%s missing" % key)
            continue
        for k in ("config_hash", "mobility_class", "lateral", "vmax", "min_width", "payloads", "modules"):
            if canonical(f[k]) != canonical(s[k]):
                problems.append("%s.%s differs" % (key, k))
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.catalog")
    ap.add_argument("cmd", choices=["build", "check", "show"])
    a = ap.parse_args(argv)
    if a.cmd == "build":
        doc = build()
        for k, u in doc["units"].items():
            print(k, u["config_hash"], u["mobility_class"], u["vmax"], u["min_width"], u["payloads"], u["half_extent"])
    elif a.cmd == "check":
        p = check()
        print("\n".join(p) or "catalogue matches capplan")
        return 1 if p else 0
    else:
        print(json.dumps(load(), indent=1)[:4000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
