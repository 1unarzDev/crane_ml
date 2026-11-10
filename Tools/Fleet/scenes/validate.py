"""Validate scenario v1 / task v1 documents: the JSON Schema (a built-in subset validator; jsonschema is not
installed here) and the semantic rules a schema cannot state.

    python3 -m scenes.validate FILE.json [FILE.json ...]      # scenario, task or bundle files

Semantic rules: unique ids per kind and across solids; walls axis-aligned and inside the arena; doors on an
existing wall, inside it, not overlapping each other; zones, patches, payload spawns and unit starts inside the
arena; unit start footprints clear of driving solids; catalogue units match their capplan reference; payload types
exist; faults name units; task: the scenario hash matches, every plan's step ids are unique, dependencies exist and
are acyclic, step references (units created by earlier docks/undocks, payloads, zones) resolve, the default plan
exists, outcome rules name existing payloads, zones and units.
"""
import json
import math
import sys
from pathlib import Path

from .common import HERE, BUNDLE_SCHEMA, SCENARIO_SCHEMA, TASK_SCHEMA, lower, sha, wall_axis
from . import catalog

SCHEMAS = {SCENARIO_SCHEMA: HERE / "schema" / "scenario_v1.schema.json", TASK_SCHEMA: HERE / "schema" / "task_v1.schema.json"}
_TYPES = {"object": dict, "array": list, "string": str, "boolean": bool, "null": type(None)}


def _type_ok(v, t):
    if isinstance(t, list):
        return any(_type_ok(v, x) for x in t)
    if t == "number":
        return isinstance(v, (int, float)) and not isinstance(v, bool)
    if t == "integer":
        return isinstance(v, int) and not isinstance(v, bool)
    return isinstance(v, _TYPES[t])


def schema_errors(doc, schema, root=None, path="$"):
    root = root or schema
    if "$ref" in schema:
        name = schema["$ref"].split("/")[-1]
        return schema_errors(doc, root["$defs"][name], root, path)
    errs = []
    if "type" in schema and not _type_ok(doc, schema["type"]):
        return ["%s: expected %s" % (path, schema["type"])]
    if "const" in schema and doc != schema["const"]:
        errs.append("%s: must be %r" % (path, schema["const"]))
    if "enum" in schema and doc not in schema["enum"]:
        errs.append("%s: %r not in %s" % (path, doc, schema["enum"]))
    if isinstance(doc, (int, float)) and not isinstance(doc, bool):
        if "minimum" in schema and doc < schema["minimum"]:
            errs.append("%s: below %s" % (path, schema["minimum"]))
        if "maximum" in schema and doc > schema["maximum"]:
            errs.append("%s: above %s" % (path, schema["maximum"]))
        if "exclusiveMinimum" in schema and doc <= schema["exclusiveMinimum"]:
            errs.append("%s: must exceed %s" % (path, schema["exclusiveMinimum"]))
    if isinstance(doc, list):
        if "minItems" in schema and len(doc) < schema["minItems"]:
            errs.append("%s: fewer than %d items" % (path, schema["minItems"]))
        if "maxItems" in schema and len(doc) > schema["maxItems"]:
            errs.append("%s: more than %d items" % (path, schema["maxItems"]))
        if "items" in schema:
            for i, v in enumerate(doc):
                errs += schema_errors(v, schema["items"], root, "%s[%d]" % (path, i))
    if isinstance(doc, dict):
        for k in schema.get("required", []):
            if k not in doc:
                errs.append("%s: missing %s" % (path, k))
        props = schema.get("properties", {})
        if "minProperties" in schema and len(doc) < schema["minProperties"]:
            errs.append("%s: empty" % path)
        for k, v in doc.items():
            if k in props:
                errs += schema_errors(v, props[k], root, "%s.%s" % (path, k))
            elif schema.get("additionalProperties") is False:
                errs.append("%s: unknown key %s" % (path, k))
            elif isinstance(schema.get("additionalProperties"), dict):
                errs += schema_errors(v, schema["additionalProperties"], root, "%s.%s" % (path, k))
    if "oneOf" in schema:
        ok = [s for s in schema["oneOf"] if not schema_errors(doc, s, root, path)]
        if len(ok) != 1:
            errs.append("%s: matches %d of oneOf (need exactly 1)" % (path, len(ok)))
    return errs


def _schema(name):
    return json.loads(SCHEMAS[name].read_text())


def scenario_problems(scn):
    errs = schema_errors(scn, _schema(SCENARIO_SCHEMA))
    if errs:
        return errs
    X, Y = scn["arena"]["size_xy"]

    def inside(p, m=0.0):
        return -1e-9 <= p[0] - m and p[0] + m <= X + 1e-9 and -1e-9 <= p[1] - m and p[1] + m <= Y + 1e-9
    for kind in ("walls", "doors", "obstacles", "floor_patches", "zones", "tags", "overhead_cameras", "payloads"):
        ids = [e["id"] for e in scn[kind]]
        if len(set(ids)) != len(ids):
            errs.append("%s: duplicate ids" % kind)
    walls = {w["id"]: w for w in scn["walls"]}
    for w in scn["walls"]:
        try:
            wall_axis(w)
        except ValueError as e:
            errs.append(str(e))
        if not (inside(w["from"]) and inside(w["to"])):
            errs.append("wall %s leaves the arena" % w["id"])
    for d in scn["doors"]:
        if d["wall"] not in walls:
            errs.append("door %s: unknown wall %s" % (d["id"], d["wall"]))
            continue
        _, _, lo, hi = wall_axis(walls[d["wall"]])
        if d["centre"] - d["width"] / 2 < lo - 1e-9 or d["centre"] + d["width"] / 2 > hi + 1e-9:
            errs.append("door %s outside its wall" % d["id"])
    for wid in walls:
        ds = sorted((d for d in scn["doors"] if d["wall"] == wid), key=lambda d: d["centre"])
        for a, b in zip(ds, ds[1:]):
            if a["centre"] + a["width"] / 2 > b["centre"] - b["width"] / 2:
                errs.append("doors %s and %s overlap" % (a["id"], b["id"]))
    if errs:
        return errs   # the lowering needs valid walls and doors
    sol = lower(scn)
    sids = [s.id for s in sol] + [p["id"] for p in scn["payloads"]]
    if len(set(sids)) != len(sids):
        errs.append("solid/payload ids collide: %s" % sorted({i for i in sids if sids.count(i) > 1}))
    for kind in ("zones", "floor_patches"):
        for z in scn[kind]:
            r = z["rect"]
            if not (r[0] < r[2] and r[1] < r[3]):
                errs.append("%s %s: empty rect" % (kind, z["id"]))
            if not (inside(r[:2]) and inside(r[2:])):
                errs.append("%s %s leaves the arena" % (kind, z["id"]))
    zones = {z["id"] for z in scn["zones"]}
    units = {u["id"]: u for u in scn["fleet"]["units"]}
    cat = catalog.load()["units"]
    drive = [s for s in sol if s.drive_block]
    for u in scn["fleet"]["units"]:
        c = cat.get(u["catalog"])
        if c is None:
            errs.append("unit %s: unknown catalogue key %s" % (u["id"], u["catalog"]))
            continue
        if u["config_hash"] != c["config_hash"] or u["module_graph"] != c["ref"]:
            errs.append("unit %s: module graph does not match catalogue entry %s" % (u["id"], u["catalog"]))
        if not inside(u["start"]):
            errs.append("unit %s starts outside the arena" % u["id"])
        # start footprint clear of driving solids (AABB test, conservative for yawed obstacles)
        x0, y0, x1, y1 = c["aabb"]
        yaw = u["start"][2]
        q = int(round(yaw / (math.pi / 2))) % 4
        for _ in range(q):
            x0, y0, x1, y1 = -y1, x0, -y0, x1
        ux0, uy0, ux1, uy1 = u["start"][0] + x0, u["start"][1] + y0, u["start"][0] + x1, u["start"][1] + y1
        for s in drive:
            pts = s.footprint()
            sx0, sy0 = min(p[0] for p in pts), min(p[1] for p in pts)
            sx1, sy1 = max(p[0] for p in pts), max(p[1] for p in pts)
            if ux0 < sx1 and sx0 < ux1 and uy0 < sy1 and sy0 < uy1:
                errs.append("unit %s starts inside %s" % (u["id"], s.id))
    boxes = {}
    for u in scn["fleet"]["units"]:
        c = cat.get(u["catalog"])
        if c is None:
            continue
        x0, y0, x1, y1 = c["aabb"]
        for _ in range(int(round(u["start"][2] / (math.pi / 2))) % 4):
            x0, y0, x1, y1 = -y1, x0, -y0, x1
        boxes[u["id"]] = (u["start"][0] + x0, u["start"][1] + y0, u["start"][0] + x1, u["start"][1] + y1)
    ids = sorted(boxes)
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            A, B = boxes[a], boxes[b]
            if A[0] < B[2] and B[0] < A[2] and A[1] < B[3] and B[1] < A[3]:
                errs.append("units %s and %s start overlapping" % (a, b))
    for p in scn["payloads"]:
        if p["spawn"].get("zone") and p["spawn"]["zone"] not in zones:
            errs.append("payload %s: unknown zone %s" % (p["id"], p["spawn"]["zone"]))
        if p["spawn"].get("on_unit") and p["spawn"]["on_unit"] not in units:
            errs.append("payload %s: unknown unit %s" % (p["id"], p["spawn"]["on_unit"]))
        if not inside(p["spawn"]["pose"]):
            errs.append("payload %s spawns outside the arena" % p["id"])
    for f in scn["fleet"]["faults"]:
        if f["unit"] not in units:
            errs.append("fault on unknown unit %s" % f["unit"])
    return errs


def task_problems(task, scn):
    errs = schema_errors(task, _schema(TASK_SCHEMA))
    if errs:
        return errs
    if task["scenario_id"] != scn["scenario_id"]:
        errs.append("task names scenario %s, not %s" % (task["scenario_id"], scn["scenario_id"]))
    if task["scenario_sha256"] != sha(scn, 64):
        errs.append("task.scenario_sha256 does not match the scenario")
    if task["default_plan"] not in task["plans"]:
        errs.append("default_plan %s is not a plan" % task["default_plan"])
    zones = {z["id"] for z in scn["zones"]}
    payloads = {p["id"] for p in scn["payloads"]}
    for name, plan in task["plans"].items():
        units = {u["id"] for u in scn["fleet"]["units"]}
        ego = plan.get("ego", task["ego"])
        if ego not in units:
            errs.append("plan %s: ego %s is not a unit" % (name, ego))
        ids = [s["id"] for s in plan["steps"]]
        if len(set(ids)) != len(ids):
            errs.append("plan %s: duplicate step ids" % name)
        seen = set()
        for s in plan["steps"]:
            for a in s.get("after", []):
                if a not in ids:
                    errs.append("plan %s step %s: unknown dependency %s" % (name, s["id"], a))
                elif a not in seen:
                    errs.append("plan %s step %s: depends on a later step %s (order steps topologically)" % (name, s["id"], a))
            for k in ("unit", "docker", "target", "from", "to"):
                if k in s and s[k] not in units:
                    errs.append("plan %s step %s: unit %s does not exist at this point" % (name, s["id"], s[k]))
            if "payload" in s and s["payload"] not in payloads:
                errs.append("plan %s step %s: unknown payload %s" % (name, s["id"], s["payload"]))
            for k in ("zone", "then_zone"):
                if k in s and s[k] not in zones:
                    errs.append("plan %s step %s: unknown zone %s" % (name, s["id"], s[k]))
            for b in s.get("bays", []) + ([s["bay"]] if "bay" in s else []):
                if b not in zones:
                    errs.append("plan %s step %s: unknown bay %s" % (name, s["id"], b))
            if s["kind"] == "dock":
                units -= {s["docker"], s["target"]}
                units.add(s["as"])
            if s["kind"] == "undock":
                units.discard(s["unit"])
                units |= set(s["as"])
            seen.add(s["id"])
    for rule in task["outcomes"]["success"]:
        r = rule.get("payload_in_zone")
        if r and (r["payload"] not in payloads or r["zone"] not in zones):
            errs.append("success rule %s names unknown payload/zone" % rule)
        r = rule.get("unit_in_zone")
        if r and r["zone"] not in zones:
            errs.append("success rule %s names an unknown zone" % rule)
    return errs


def validate_pair(scn, task):
    return scenario_problems(scn) + task_problems(task, scn)


def validate_file(path):
    d = json.loads(Path(path).read_text())
    if d.get("schema") == BUNDLE_SCHEMA:
        return validate_pair(d["scenario"], d["task"])
    if d.get("schema") == SCENARIO_SCHEMA:
        return scenario_problems(d)
    if d.get("schema") == TASK_SCHEMA:
        sp = Path(path).with_name(Path(path).name.replace(".task.json", ".scenario.json"))
        if not sp.exists():
            return ["task file: scenario %s not found next to it" % sp.name]
        return task_problems(d, json.loads(sp.read_text()))
    return ["unknown schema %r" % d.get("schema")]


def main(argv=None):
    bad = 0
    for p in (argv if argv is not None else sys.argv[1:]):
        errs = validate_file(p)
        bad += bool(errs)
        print("%s: %s" % (p, "ok" if not errs else "; ".join(errs)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
