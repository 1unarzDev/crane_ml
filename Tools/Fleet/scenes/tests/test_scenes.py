"""scenes v1 tests: schema + semantics, lowering, planner, splits, CRANE compile, parity, fastsim smoke, mf adapter, previews.

    cd Tools/Fleet && nice -n 15 python3 -m pytest -q scenes/tests -p no:cacheprovider
"""
import copy
import json
import math
import sys
from pathlib import Path

import pytest

FLEET = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(FLEET))

from scenes import catalog, common, families, plan, splits, validate  # noqa: E402
from scenes.families import GENERATORS, generate  # noqa: E402

SEEDS = (1, 2)


def capplan_available():
    try:
        catalog._capplan()
        return True
    except Exception:  # noqa: BLE001
        return False


@pytest.mark.parametrize("family", sorted(GENERATORS))
@pytest.mark.parametrize("split", ["train", "heldout"])
def test_every_family_validates_and_lands_in_its_split(family, split):
    for seed in SEEDS:
        scn, task = generate(family, seed, split)
        assert validate.validate_pair(scn, task) == []
        assert families.in_split(family, scn["family"]["params"]) == split
        assert task["default_plan"] in task["plans"] and len(task["plans"]) >= 2, "a family offers a decision"


def test_generation_is_deterministic():
    a, _ = generate("clutter_observer", 3, "train")
    b, _ = generate("clutter_observer", 3, "train")
    assert common.sha(a) == common.sha(b)


def test_validator_catches_semantic_errors():
    scn, task = generate("narrow_door", 1, "train")
    bad = copy.deepcopy(scn)
    bad["doors"][0]["wall"] = "nope"
    assert any("unknown wall" in e for e in validate.scenario_problems(bad))
    bad = copy.deepcopy(scn)
    bad["fleet"]["units"][0]["start"] = [5.0, 1.0, 0.0]          # inside wall_a
    assert any("starts inside" in e for e in validate.scenario_problems(bad))
    bad = copy.deepcopy(scn)
    bad["zones"][0]["kind"] = "garage"
    assert any("not in" in e for e in validate.scenario_problems(bad))
    t = copy.deepcopy(task)
    t["plans"]["single"]["steps"][1]["after"] = ["nope"]
    assert any("unknown dependency" in e for e in validate.task_problems(t, scn))
    t = copy.deepcopy(task)
    t["scenario_sha256"] = "0" * 64
    assert any("does not match" in e for e in validate.task_problems(t, scn))


def test_lowering_cuts_doors_with_exact_widths():
    scn, _ = generate("narrow_door", 1, "train")
    sol = {s.id: s for s in common.lower(scn)}
    for d in scn["doors"]:
        w = d["wall"]
        segs = sorted((s for s in sol.values() if s.source == w and s.kind == "wall"), key=lambda s: s.cy)
        gaps = [(b.cy - b.sy / 2) - (a.cy + a.sy / 2) for a, b in zip(segs, segs[1:])]
        assert any(abs(g - d["width"]) < 1e-9 for g in gaps)
        assert sol[d["id"] + "_lintel"].drive_block is False
        if d["threshold_height"] > 0:
            assert sol[d["id"] + "_threshold"].drive_block is False


def test_planner_dogleg_admits_a_single_but_not_a_pair():
    scn, _ = generate("narrow_door", 1, "train")
    boxes = []
    for s in common.lower(scn):
        if s.drive_block:
            p = s.footprint()
            boxes.append((min(q[0] for q in p), min(q[1] for q in p), max(q[0] for q in p), max(q[1] for q in p)))
    g = plan.Grid(scn["arena"]["size_xy"], boxes, align=(8.75, 3.5))
    v = catalog.unit("carrier")["vmax"]
    single = plan.Unitshape([(0, 0, 0.21, 0.2)], 0.29, True, v, 0.025)
    chain = plan.Unitshape([(-0.21, 0, 0.21, 0.2), (0.21, 0, 0.21, 0.2)], 0.465, True, catalog.unit("pair_chain")["vmax"], 0.025)
    assert plan.plan(g, single, (1.2, 1.0, 0.0), (8.75, 3.5))[0] is not None
    assert plan.plan(g, chain, (1.2, 1.0, 0.0), (8.75, 3.5))[0] is None


def test_planner_lateral_slot_needs_strafing():
    scn, task = generate("lateral_slot", 1, "train")
    boxes = []
    for s in common.lower(scn):
        if s.drive_block:
            p = s.footprint()
            boxes.append((min(q[0] for q in p), min(q[1] for q in p), max(q[0] for q in p), max(q[1] for q in p)))
    z = common.zone(scn, "slot")
    goal = common.rect_centre(z["rect"])
    g = plan.Grid(scn["arena"]["size_xy"], boxes, align=goal)
    mec = plan.Unitshape([(0, 0, 0.21, 0.2)], 0.29, True, catalog.unit("carrier")["vmax"], 0.025)
    dif = plan.Unitshape([(0, 0, 0.1999, 0.194)], 0.279, False, catalog.unit("diff_carrier")["vmax"], 0.025)
    assert plan.plan(g, mec, (1.0, 1.0, 0.0), goal, z["required_yaw"])[0] is not None
    assert plan.plan(g, dif, (1.0, 2.0, 0.0), goal, z["required_yaw"])[0] is None


def test_catalogue_is_train_only_and_consistent():
    for k, u in catalog.load()["units"].items():
        assert u["ref"].get("manifest") in (None, "train_v1"), k
    if capplan_available():
        assert catalog.check() == []


def test_frozen_splits_have_no_leakage():
    if not splits.FROZEN.exists():
        pytest.skip("splits not frozen yet")
    res = splits.check()
    for k, v in res.items():
        assert [x for x in v if not x.startswith("(")] == [], k


def test_split_rules_are_disjoint_on_fresh_samples():
    for f in GENERATORS:
        tr = {json.dumps(generate(f, s, "train")[0]["family"]["params"], sort_keys=True) for s in range(1, 6)}
        ho = {json.dumps(generate(f, s, "heldout")[0]["family"]["params"], sort_keys=True) for s in range(1, 6)}
        assert not tr & ho, f


def test_crane_compile_emits_v0_files(tmp_path):
    from scenes import crane_compile
    scn, task = generate("narrow_door", 1, "train")
    r = crane_compile.compile_scene(scn, task, tmp_path)
    s = json.loads(Path(r["scenario"]).read_text())
    assert s["schema"] == "crane.fleet_scenario/v0" and s["walls"] == [] and "delivery" in s["zones"]
    assert all(o["type"] == "box" for o in s["obstacles"])
    assert {"width", "centre_y", "height", "in_wall"} <= set(s["doorway"])
    m = json.loads(Path(r["plans"]["single"]["mission"]).read_text())
    ids = [t["id"] for t in m["tasks"]]
    assert len(ids) == len(set(ids))
    for t in m["tasks"]:
        assert all(a in ids for a in t.get("after", [])), t
    assert set(m["crane_start"]) == {"scout_1", "lifter_1", "cargo_1", "cargo_2"}
    assert m["crane_payload"]["mass_kg"] == scn["payloads"][0]["mass_kg"]


def test_crane_compile_reports_unsupported_fleets(tmp_path):
    from scenes import crane_compile
    scn, task = generate("lateral_slot", 1, "train")
    r = crane_compile.compile_scene(scn, task, tmp_path)
    # the differential carrier cannot be built in CRANE: its plan has no mission; the mecanum plan leaves it out (idle)
    assert r["plans"]["diffdrive"]["mission"] is None
    assert any(g["status"] == "unsupported" and "diff" in g["field"] for g in r["plans"]["diffdrive"]["gaps"])
    assert r["plans"]["mecanum"]["mission"] is not None
    assert any(g["status"] == "omitted" and "diff" in g["field"] for g in r["plans"]["mecanum"]["gaps"])


@pytest.mark.parametrize("family", sorted(GENERATORS))
def test_fastsim_and_crane_geometry_parity(family, tmp_path):
    from scenes import consistency
    scn, task = generate(family, 1, "train")
    probs, info = consistency.check(scn, task, None, tmp_path)
    assert probs == [], probs


def test_fastsim_smoke_narrow_door(tmp_path):
    from scenes.fastsim_run import run
    ok = run("narrow_door", "train", "single", [1], tmp_path)[0]
    bad = run("narrow_door", "train", "couple_chain", [1], tmp_path)[0]
    assert ok["outcome"] == "SUCCEEDED"
    assert bad["outcome"] == "FAILED" and "no_path" in bad["reason"]
    run_json = json.loads((Path(ok["dir"]) / "run.json").read_text())
    assert run_json["schema"] == "crane.fastsim.run/1" and "fleet tier" in run_json["fidelity"]


def test_fastsim_runs_are_reproducible(tmp_path):
    from scenes.fastsim_run import run
    a = run("lateral_slot", "train", "mecanum", [2], tmp_path / "a")[0]
    b = run("lateral_slot", "train", "mecanum", [2], tmp_path / "b")[0]
    ra = json.loads((Path(a["dir"]) / "run.json").read_text())
    rb = json.loads((Path(b["dir"]) / "run.json").read_text())
    assert ra["identity"]["run_sha256"] == rb["identity"]["run_sha256"]
    assert (Path(a["dir"]) / "track.jsonl").read_bytes() == (Path(b["dir"]) / "track.jsonl").read_bytes()


def test_mf_taskspec_from_scene():
    if not capplan_available():
        pytest.skip("capplan / mf not importable")
    from scenes import mf_adapter
    scn, task = generate("long_payload_door", 1, "train")
    ts, key = mf_adapter.taskspec(scn, task, "reconfigure")
    assert key == "pair_chain" and ts.kind == "transport"
    assert abs(ts.door_width - scn["doors"][0]["width"]) < 1e-9
    assert ts.payload["type"] == "box_long" and ts.ops.get("undock") == 1
    ts2, key2 = mf_adapter.taskspec(scn, task, "keep")
    assert key2 == "l3_carrier" and ts2.task_hash != ts.task_hash


def test_previews_exist_and_are_small():
    prev = FLEET / "scenes" / "previews"
    if not prev.exists():
        pytest.skip("previews not rendered")
    for f in GENERATORS:
        p = prev / ("%s.jpg" % f)
        assert p.exists() and p.stat().st_size <= 300 * 1024, p
