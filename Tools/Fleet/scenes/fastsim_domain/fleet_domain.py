"""The fastsim fleet domain: a scenario.py format adapter for scene bundles, the DomainAdapter, and the plant plugin.

A bundle (``crane.fleet_scene_bundle/v1``, written by ``gen_scenarios.py``) holds one scenario v1 and one task v1
verbatim plus the plan to run. ``FleetBundleFormat.parse`` lowers the scenario with ``scenes.common.lower`` (the same
lowering the CRANE compiler uses) into fastsim elements: every driving-blocking solid as a box, every payload as a
labelled element; the start pose is the ego unit's start. ``FleetAdapter`` runs ``scenes.fleetsim.FleetEngine`` as the
controller. ``FleetPlant`` (YAML ``vehicle.ref``) is the ego's plant; the engine builds the same class for the others.

Nothing in fastsim's core is changed; see Docs/Fleet/SCENES_v1.md for the core proposals.
"""
import json
import sys
from pathlib import Path

_FLEET = Path(__file__).resolve().parents[2]          # Tools/Fleet
_ROOT = Path(__file__).resolve().parents[4]           # the crane root (fastsim/)
for _p in (_ROOT, _FLEET):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fastsim.controller import DomainAdapter  # noqa: E402
from fastsim.scenario import Element, Scenario, ScenarioFormat  # noqa: E402

from scenes import catalog  # noqa: E402
from scenes.common import BUNDLE_SCHEMA, HERE as SCENES, lower, payload_type, sha  # noqa: E402
from scenes import fleetsim  # noqa: E402
from scenes.fleetsim import FIDELITY, FleetController, FleetEngine, FleetPlant, RUN, limits_for  # noqa: E402,F401


def code_hash():
    files = sorted(SCENES.glob("*.py")) + [Path(__file__)]
    return sha(b"".join(f.read_bytes() for f in files))


def elements_of(scn):
    els = []
    for s in lower(scn):
        if not s.drive_block:
            continue
        els.append(Element(s.id, s.cx, s.cy, 0.0, "", None, (s.sx, s.sy, s.yaw)))
    for p in scn["payloads"]:
        L, W, _ = payload_type(p["type"])["size_lwh"]
        x, y, yaw = p["spawn"]["pose"]
        els.append(Element(p["id"], x, y, 0.0, "payload", None, None))
    return els


class FleetBundleFormat(ScenarioFormat):
    filename = "fleet_scene.json"

    def parse(self, data):
        b = json.loads(data)
        if b.get("schema") != BUNDLE_SCHEMA:
            raise ValueError("not a %s file" % BUNDLE_SCHEMA)
        scn, task, plan = b["scenario"], b["task"], b["plan"]
        ego = task["plans"][plan].get("ego", task["ego"])
        start = next(u["start"] for u in scn["fleet"]["units"] if u["id"] == ego)
        X, Y = scn["arena"]["size_xy"]
        ident = {"pair_key": "%s/%s" % (scn["scenario_id"], plan), "truth_sha256": sha({"s": scn, "t": task}, 64),
                 "seed": scn["family"]["seed"], "family": scn["family"]["id"], "split": scn["family"]["split"]}
        sc = Scenario(scn["scenario_id"], tuple(float(v) for v in start), elements_of(scn), data,
                      [[0.0, 0.0], [X, 0.0], [X, Y], [0.0, Y]], [], ident, self.filename, b)
        RUN.clear()
        RUN["bundle"] = b
        return sc

    def check(self, scenario):
        """Refuse an edited bundle, and a stale one (the family generator no longer produces it: regenerate)."""
        b = scenario.raw
        if b["task"]["scenario_sha256"] != sha(b["scenario"], 64):
            return ["task.scenario_sha256 does not match the embedded scenario (edited file?)"]
        f = b["scenario"]["family"]
        if f.get("split") in ("train", "heldout"):
            from scenes.families import generate
            scn, task = generate(f["id"], f["seed"], f["split"])
            if sha(scn, 64) != sha(b["scenario"], 64) or sha(task) != sha(b["task"]):
                return ["stale bundle: the generator now produces a different scene (delete it and regenerate)"]
        return []


def ego_footprint(b):
    task, plan = b["task"], b["plan"]
    ego = task["plans"][plan].get("ego", task["ego"])
    key = next(u["catalog"] for u in b["scenario"]["fleet"]["units"] if u["id"] == ego)
    x0, y0, x1, y1 = catalog.unit(key)["aabb"]
    return [(x1, y1), (x1, y0), (x0, y0), (x0, y1)]


class FleetAdapter(DomainAdapter):
    def identity(self):
        return {"domain": "crane_fleet_scenes_v1", "catalog_hash": catalog.load()["catalog_hash"],
                "code_sha": code_hash(), "options": self.options}

    def limits(self):
        return limits_for(ego_footprint(RUN["bundle"]))

    def overrides(self):
        return {"fidelity": FIDELITY}

    def controller(self, ctx):
        b = ctx.scenario.raw
        task, plan = dict(b["task"]), b["plan"]
        task["ego"] = task["plans"][plan].get("ego", task["ego"])
        ego_plant = RUN.get("ego_plant")
        if ego_plant is None:
            raise RuntimeError("the ego plant is not a FleetPlant: set vehicle.ref to fleet_domain.py:FleetPlant")
        eng = FleetEngine(b["scenario"], task, plan, ctx.world, ctx.clock, ctx.rng, ego_plant, ctx.navigator, ctx.emit,
                          gain=float(self.options.get("gain", 2.0)))
        self.engine = eng
        return FleetController(eng)

    def time_limit(self, ctx):
        return float(ctx.scenario.raw["task"]["deadline_s"]) + 5.0

    def condition(self, ctx):
        b = ctx.scenario.raw
        return {"scenario_id": b["scenario"]["scenario_id"], "plan": b["plan"], "task_id": b["task"]["task_id"]}

    def finalize(self, run_dir, result):
        s = self.engine.summary()
        s["core_outcome"] = result["outcome"]
        s["scenario_id"] = self.engine.scn["scenario_id"]
        s["family"] = self.engine.scn["family"]
        (Path(run_dir) / "fleet.json").write_text(json.dumps(s, indent=1) + "\n")
        self.write_fleet_traj(Path(run_dir) / "fleet.ctraj", result)

    def write_fleet_traj(self, path, result):
        """Every unit's 2 Hz pose as one crane-traj/1 file (agents = unit ids; a unit's samples are held before it
        exists and after it is merged away), with the engine's events; the core's run.ctraj has the ego only."""
        from theater import traj as ctraj
        tr = self.engine.traces
        times = sorted({t for rows in tr.values() for t, *_ in rows})
        if not times:
            return
        cols = {}
        for uid, rows in sorted(tr.items()):
            by = {t: (x, y, a) for t, x, y, a in rows}
            cur = rows[0][1:]
            xs, ys, yaws = [], [], []
            for t in times:
                cur = by.get(t, cur)
                xs.append(cur[0]); ys.append(cur[1]); yaws.append(cur[2])
            cols.update({"%s.x" % uid: xs, "%s.y" % uid: ys, "%s.yaw" % uid: yaws})
        evs = [{**{k: v for k, v in e.items() if k not in ("t", "event")}, "t": e["t"], "kind": str(e["event"])}
               for e in result["events"] if isinstance(e, dict) and "t" in e]
        sc = result["scenario"]
        out = {"SUCCEEDED": "finished", "TIMEOUT": "timeout"}.get(result["outcome"], "failed")
        header = {"run_id": result["identity"].get("run_sha256"), "seed": result["seed"], "pose_dims": "planar",
                  "robot": {"model": "crane_fleet_scenes_v1", "agents": sorted(tr)}, "frame": {"type": "enu", "origin": "scenario"},
                  "course": {"name": sc["name"], "pair_key": sc["pair_key"], "truth_sha256": sc["truth_sha256"]},
                  "build": {"recorder": "fastsim fleet domain"}, "water": {"mode": "none"}, "outcome": out, "valid": True,
                  "t0_s": times[0]}
        ctraj.write(path, header, cols, sorted(evs, key=lambda e: e["t"]), hz=2.0)

    def metrics(self, run_dir):
        f = json.loads((Path(run_dir) / "fleet.json").read_text())
        return {"fleet": {"success": 1.0 if f["outcome"] == "SUCCEEDED" else 0.0, "t_end": f["t_end"],
                          "collisions": len(f["contacts"]),
                          "steps_done": sum(s["state"] == "done" for s in f["steps"]),
                          "steps_failed": sum(s["state"] == "failed" for s in f["steps"])}}
