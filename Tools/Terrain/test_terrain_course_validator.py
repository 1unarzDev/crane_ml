import json
import subprocess
import sys
from pathlib import Path

import pytest

import terrain_course_validator as val
import terrain_generator as gen

HERE = Path(__file__).resolve().parent
LEGACY = ["--crane-land-nav2", "--crane-campus"]
FORCE = LEGACY + ["--crane-campus-model"]
CONTACT = FORCE + ["--crane-campus-contact-wheels"]
SLOPE_AWARE = FORCE + ["--crane-campus-slope-traction"]


def ramp(angle):
    return gen.generate_course(f"ramp-{angle}", "ramp", {"angleDeg": angle}, 1, resolution=33)


def test_plant_inference_matches_campus_flags():
    assert val.infer_plant(LEGACY) == "legacy-diff-drive"
    assert val.infer_plant(FORCE) == "campus-force-branch"
    assert val.infer_plant(CONTACT) == "campus-contact-wheels"
    assert val.infer_plant(SLOPE_AWARE) == "slope-aware-traction"


@pytest.mark.parametrize("args", [LEGACY, FORCE])
def test_force_and_legacy_runs_above_two_degrees_are_invalid_for_slope(args):
    course, _ = ramp(5)
    result = val.label_run(course, args, claimed_label="LIVE_SIM_VALIDATED")
    assert result["label"] == "INVALID_FOR_SLOPE" and result["evidenceUsable"] is False


@pytest.mark.parametrize("args", [LEGACY, FORCE])
def test_two_degree_boundary_is_strict(args):
    at, _ = ramp(2)
    above, _ = ramp(2.01)
    assert val.label_run(at, args)["label"] == "NOT_RUN"
    assert val.label_run(above, args)["label"] == "INVALID_FOR_SLOPE"


@pytest.mark.parametrize("args", [CONTACT, SLOPE_AWARE])
def test_slope_capable_plants_keep_their_claimed_label(args):
    course, _ = ramp(20)
    assert val.label_run(course, args, claimed_label="LIVE_SIM_VALIDATED")["label"] == "LIVE_SIM_VALIDATED"


def test_flat_mixed_friction_course_is_valid_for_every_plant():
    course, _ = gen.generate_course("mf", "mixed_friction", {"patches": [{"surface": "normal-concrete", "zRangeM": [0, 16]}]}, 1, resolution=33)
    for args in (LEGACY, FORCE, CONTACT):
        assert val.label_run(course, args)["evidenceUsable"] is True


def test_rock_and_mogul_courses_count_as_sloped():
    moguls, _ = gen.generate_course("m", "moguls", {"amplitudeM": 0.05, "wavelengthXM": 4.0, "wavelengthZM": 4.0}, 1, resolution=65)
    assert val.label_run(moguls, FORCE)["label"] == "INVALID_FOR_SLOPE"


def test_heightmap_tamper_and_bad_inputs_fail_closed():
    course, raw = ramp(10)
    assert val.label_run(course, CONTACT, heightmap=raw)["heightmapVerified"] is True
    with pytest.raises(val.ValidationError):
        val.label_run(course, CONTACT, heightmap=raw[:-2] + b"\x00\x01")
    with pytest.raises(val.ValidationError):
        val.label_run(dict(course, schema="other"), CONTACT)
    with pytest.raises(val.ValidationError):
        val.label_run(dict(course, groundTruth={}), CONTACT)
    with pytest.raises(val.ValidationError):
        val.label_run(course, None)
    with pytest.raises(val.ValidationError):
        val.label_run(course, CONTACT, claimed_label="PROBABLY_FINE")


def test_ablation_course_is_flagged():
    course, _ = gen.generate_course("s", "deformable", {"soil": "chrono-lete-sand", "patchStartM": 4, "patchLengthM": 6}, 1, resolution=33)
    result = val.label_run(course, CONTACT)
    assert any("ablation" in r for r in result["reasons"])


def test_cli_exit_codes(tmp_path):
    course, raw = ramp(10)
    path = tmp_path / "ramp.json"
    path.write_text(json.dumps(course))
    (tmp_path / "ramp.r16").write_bytes(raw)
    run = lambda *a: subprocess.run([sys.executable, "-I", str(HERE / "terrain_course_validator.py"), "--course", str(path), *a],
                                    capture_output=True, text=True)
    bad = run("--run-args", " ".join(FORCE))
    assert bad.returncode == 1 and json.loads(bad.stdout)["label"] == "INVALID_FOR_SLOPE"
    good = run("--run-args", " ".join(CONTACT), "--heightmap", str(tmp_path / "ramp.r16"))
    assert good.returncode == 0 and json.loads(good.stdout)["heightmapVerified"] is True
    assert run("--plant", "nonsense").returncode == 2
