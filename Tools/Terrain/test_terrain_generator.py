import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

import terrain_generator as gen

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def course(kind, params, seed=1, **kw):
    return gen.generate_course(f"t-{kind}", kind, params, seed, **kw)


@pytest.mark.parametrize("angle", [0, 5, 15, 25, 35])
def test_ramp_measured_slope_matches_analytic(angle):
    m, _ = course("ramp", {"angleDeg": angle})
    assert m["groundTruth"]["measuredMaxSlopeDeg"] == pytest.approx(angle, abs=1e-6 + 0.5 * (angle > 0))
    assert m["groundTruth"]["analyticSlopeDeg"] == angle


def test_ramp_rejects_out_of_range_angle_and_bad_resolution():
    with pytest.raises(gen.CourseError):
        course("ramp", {"angleDeg": 36})
    with pytest.raises(gen.CourseError):
        course("ramp", {"angleDeg": 5}, resolution=100)


def test_cross_slope_is_planar_everywhere():
    m, _ = course("cross_slope", {"angleDeg": 10})
    gt = m["groundTruth"]
    assert gt["measuredMaxSlopeDeg"] == pytest.approx(10, abs=1e-9)
    assert gt["measuredMeanSlopeDeg"] == pytest.approx(10, abs=1e-9)


def test_mogul_max_slope_converges_to_closed_form():
    params = {"amplitudeM": 0.1, "wavelengthXM": 4.0, "wavelengthZM": 4.0}
    coarse, _ = course("moguls", params, resolution=65)
    fine, _ = course("moguls", params, resolution=257)
    analytic = math.degrees(math.atan(2 * math.pi * 0.1 / 4.0))
    assert fine["groundTruth"]["analyticSlopeDeg"] == pytest.approx(analytic)
    e_coarse = abs(coarse["groundTruth"]["measuredMaxSlopeDeg"] - analytic)
    e_fine = abs(fine["groundTruth"]["measuredMaxSlopeDeg"] - analytic)
    assert e_fine < e_coarse and e_fine < 0.2


def test_rock_field_is_seed_deterministic_and_density_scaled():
    p = {"rocksPerSquareMeter": 0.2, "rockRadiusRangeM": [0.03, 0.15]}
    a, raw_a = course("rock_field", p, seed=7)
    b, raw_b = course("rock_field", p, seed=7)
    c, _ = course("rock_field", p, seed=8)
    assert raw_a == raw_b and a["heightmapSha256"] == b["heightmapSha256"]
    assert a["heightmapSha256"] != c["heightmapSha256"]
    expected = a["groundTruth"]["expectedRockCount"]
    counts = [len(course("rock_field", p, seed=s)[0]["groundTruth"]["rocks"]) for s in range(20)]
    assert abs(sum(counts) / len(counts) - expected) < 3 * math.sqrt(expected / len(counts))
    for rock in a["groundTruth"]["rocks"]:
        assert rock["heightM"] <= rock["radiusM"]


def test_mixed_friction_reuses_campus_surface_ids():
    m, _ = course("mixed_friction", {"patches": [{"surface": "normal-concrete", "zRangeM": [0, 8]},
                                                 {"surface": "compacted-dirt", "zRangeM": [8, 16]}]})
    campus = json.loads((ROOT / "Assets/Resources/ReferenceEnvironments/industrial_logistics_campus_v1.json").read_text())
    ids = {s["id"] for s in campus["surfaceProfiles"]}
    assert set(gen.CAMPUS_SURFACE_IDS) == ids
    assert [s["surface"] for s in m["surfaces"]] == ["normal-concrete", "compacted-dirt"]
    with pytest.raises(gen.CourseError):
        course("mixed_friction", {"patches": [{"surface": "ice", "zRangeM": [0, 1]}]})


def test_deformable_patch_is_marked_ablation_with_cited_soil():
    m, _ = course("deformable", {"soil": "chrono-lete-sand", "patchStartM": 4, "patchLengthM": 6})
    assert m["ablation"] is True
    soil = [s for s in m["surfaces"] if s.get("ablation")]
    assert len(soil) == 1 and "chrono" in soil[0]["soil"]["source"]
    assert soil[0]["soil"]["elasticStiffnessPaPerM"] > soil[0]["soil"]["bekkerKphi"]


def test_cli_writes_catalog(tmp_path):
    subprocess.run([sys.executable, "-I", str(HERE / "terrain_generator.py"), "--out", str(tmp_path),
                    "--resolution", "33", "--raw"], check=True)
    cat = json.loads((tmp_path / "catalog.json").read_text())
    assert cat["schema"] == "crane-land-terrain-catalog-v1"
    ids = {c["courseId"] for c in cat["courses"]}
    assert {"ramp-00deg", "ramp-35deg", "cross-slope-10deg", "soil-lete-sand-ablation"} <= ids
    assert (tmp_path / "ramp-20deg.r16").stat().st_size == 33 * 33 * 2
