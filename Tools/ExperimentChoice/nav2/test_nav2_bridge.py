"""Live agreement between course.py and the Nav2-side doorway family (EXPLORATORY).

The modular-study worktree keeps frozen copies of a few manifests in its own tests; this test runs the
two generators side by side on fresh courses instead. Skipped when the worktree is not on this machine.

  ulimit -v 1500000; OPENBLAS_NUM_THREADS=1 nice -n 19 python3 -I -m pytest Tools/ExperimentChoice/nav2 -q -p no:cacheprovider
"""
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
MRMN = os.environ.get("EXPCHOICE_MRMN_WORKTREE", "/home/lunarz/worktrees/mrmn-expchoice")
sys.path.insert(0, os.path.dirname(HERE))
import course as C      # noqa: E402
import robots as R      # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.isdir(os.path.join(MRMN, "src/modular_robot_benchmarks")),
                                reason="modular-study worktree not present")


@pytest.fixture(scope="module")
def nav2_side():
    for sub in ("src/morphology_planner", "src/modular_robot_benchmarks"):
        sys.path.insert(0, os.path.join(MRMN, sub))
    sys.dont_write_bytecode = True
    from modular_robot_benchmarks import doorway_scenarios, sdf_export
    return doorway_scenarios, sdf_export


def courses():
    out = C.sample_courses(40, seed=20261110)
    rob = R.load_robot("compact_diff")
    out += [C.course_from_ratios(rob, eps, e) for eps in (0.06, 0.38, 1.0) for e in (0.1, 1.0, 2.0)]
    out += [C.make_course(w=0.9, e=-0.7, d=0.85), C.make_course(w=1.0, e=2.0, d=1.2, t=0.05)]
    return out


def boxes_of(sdf_text):
    walls = {}
    for model in ET.fromstring(re.sub(r"^<\?xml[^>]*\?>", "", sdf_text)).find("world").findall("model"):
        if model.get("name") in ("ground", "ground_plane"):
            continue
        x, y = (float(v) for v in model.find("pose").text.split()[:2])
        sx, sy = (float(v) for v in model.find("link/collision/geometry/box/size").text.split()[:2])
        walls[model.get("name")] = (round(x - sx / 2, 4), round(x + sx / 2, 4), round(y - sy / 2, 4), round(y + sy / 2, 4))
    return walls


def test_manifest_world_and_map_agree(nav2_side, tmp_path):
    scen, sdf_export = nav2_side
    rob = R.load_robot("compact_diff")
    catalog = os.path.join(MRMN, "src/modular_robot_description/config/morphologies.yaml")
    from pathlib import Path
    for crs in courses():
        manifest = json.loads(C.manifest_json(crs, rob))
        theirs = scen.course_from_manifest(manifest)          # checks id, parameters, walls, start, goal
        assert theirs.course_id == crs.course_id
        world, _ = sdf_export.export_doorway_sdf(theirs, Path(tmp_path) / crs.course_id / "world.sdf", catalog, None)
        assert boxes_of(world.read_text()) == boxes_of(C.to_sdf(crs))
        # Map: the 0.01 m raster must be exactly "every cell that overlaps a wall of course.py",
        # recomputed here from course.py's own metre boxes. (course.py's 0.025 m grid is a
        # different conservative raster of the same boxes and is not what Nav2 is given.)
        fine = scen.rasterize(theirs)
        occ_fine = np.frombuffer(b"".join(fine.rows), np.uint8).reshape(fine.height, fine.width).astype(bool)
        expect = np.zeros_like(occ_fine)
        for xmin, xmax, ymin, ymax in C.to_boxes(crs):
            ix0 = int(np.floor((xmin - fine.origin[0]) / 0.01 + 1e-6))
            ix1 = int(np.ceil((xmax - fine.origin[0]) / 0.01 - 1e-6))
            iy0 = int(np.floor((ymin - fine.origin[1]) / 0.01 + 1e-6))
            iy1 = int(np.ceil((ymax - fine.origin[1]) / 0.01 - 1e-6))
            expect[max(iy0, 0):iy1, max(ix0, 0):ix1] = True
        assert (occ_fine == expect).all()
        assert scen.pgm_bytes(fine).endswith(np.where(expect[::-1], 0, 254).astype(np.uint8).tobytes())
        measured = scen.measure_map(theirs, fine)
        assert abs(measured["bay_depth_m"] - crs.d) < 1e-9
        assert abs(measured["door_centre_m"] - crs.e) < 1e-9
        assert abs(measured["door_wall_thickness_m"] - crs.t) < 1e-9
        assert 0.0 <= crs.w - measured["door_width_m"] <= (0.0 if crs.w_cm % 2 == 0 else 0.01) + 1e-9


def test_committed_batch_is_what_the_design_runs(nav2_side):
    """courses_v0 here, the copies in the modular worktree and its frozen design name the same courses."""
    scen, _ = nav2_side
    ours = {name: open(os.path.join(HERE, "courses_v0", name), "rb").read()
            for name in sorted(os.listdir(os.path.join(HERE, "courses_v0")))}
    study = os.path.join(MRMN, "studies/expchoice_doorway_v0")
    theirs = {name: open(os.path.join(study, "courses", name), "rb").read()
              for name in sorted(os.listdir(os.path.join(study, "courses")))}
    assert ours == theirs
    design = json.load(open(os.path.join(study, "design.json")))["design"]
    assert {t["course_id"] for t in design["trials"]} == {n[len("course_"):-len(".json")] for n in ours}
    batch = json.load(open(os.path.join(HERE, "batch_v0.json")))
    assert {c["course_id"] for c in batch["courses"]} == {t["course_id"] for t in design["trials"]}
    canary = [c["course_id"] for c in batch["courses"] if c["canary"]]
    assert [t["course_id"] for t in design["trials"][:2]] == canary * 2


def test_committed_batch_v1_is_what_the_v1_design_runs(nav2_side):
    ours = {name: open(os.path.join(HERE, "courses_v1", name), "rb").read()
            for name in sorted(os.listdir(os.path.join(HERE, "courses_v1")))}
    study = os.path.join(MRMN, "studies/expchoice_doorway_v1")
    theirs = {name: open(os.path.join(study, "courses", name), "rb").read()
              for name in sorted(os.listdir(os.path.join(study, "courses")))}
    assert ours == theirs and len(ours) == 10
    design = json.load(open(os.path.join(study, "design.json")))["design"]
    batch = json.load(open(os.path.join(HERE, "batch_v1.json")))
    assert {c["course_id"] for c in batch["courses"]} == {t["course_id"] for t in design["trials"]}
    canary = [c for c in batch["courses"] if c["canary"]]
    assert len(canary) == 1 and canary[0]["d_m"] == max(c["d_m"] for c in batch["courses"])
    assert [t["course_id"] for t in design["trials"][:2]] == [canary[0]["course_id"]] * 2
    assert all(c["in_admissible_box_v0"] and c["footprints"]["physical"]["certificate"] == "fits"
               for c in batch["courses"])
    jobs = os.path.expanduser("~/.cache/expchoice/jobs/v1")
    if os.path.isdir(jobs):
        seeds = sorted(s for name in ("canary", "rep0", "rep1")
                       for j in json.load(open(os.path.join(jobs, name + ".json"))) for s in j["seeds"])
        assert seeds == list(range(40))
