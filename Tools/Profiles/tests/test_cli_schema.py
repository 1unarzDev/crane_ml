import json
import subprocess
import sys
from pathlib import Path

import pytest

from crane_profiles import model
from helpers import good_physical, good_prior, mutate

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "crane_profiles"


def run(*args):
    """Isolated mode (-I) drops PYTHONPATH and cwd, so the package directory is executed directly."""
    return subprocess.run([sys.executable, "-I", str(PKG), *args], capture_output=True, text=True, timeout=60)


def test_cli_validate_ok_emits_json(tmp_path):
    f = tmp_path / "p.json"
    f.write_text(json.dumps(good_prior()))
    r = run("validate", str(f))
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout)
    assert out["ok"] is True and out["errors"] == [] and out["calibrationState"] == "UNCALIBRATED_PRIOR"
    assert out["labelCeiling"] == "LIVE_SIM_VALIDATED"


def test_cli_validate_failure_exit_code_and_rules(tmp_path):
    f = tmp_path / "bad.json"
    f.write_text(json.dumps(mutate(good_prior(), lambda d: d["dynamics"]["massKg"].pop("unit"))))
    r = run("validate", str(f))
    assert r.returncode == 1
    out = json.loads(r.stdout)
    assert out["ok"] is False and {e["rule"] for e in out["errors"]} == {"R1"}
    assert out["errors"][0]["path"] == "dynamics.massKg"


def test_cli_unreadable_file(tmp_path):
    r = run("validate", str(tmp_path / "nope.json"))
    assert r.returncode == 2 and json.loads(r.stdout)["ok"] is False


@pytest.mark.parametrize("name", sorted(p.name for p in (ROOT / "migrated").glob("*.json")))
def test_cli_validates_every_migrated_file(name):
    r = run("validate", str(ROOT / "migrated" / name))
    assert r.returncode == 0, r.stdout
    assert json.loads(r.stdout)["ok"]


def test_cli_physical_good(tmp_path):
    f = tmp_path / "ph.json"
    f.write_text(json.dumps(good_physical()))
    assert json.loads(run("validate", str(f)).stdout)["evidenceLabel"] == "PHYSICALLY_CALIBRATED"


def _schema(name):
    return json.loads((ROOT / "schema" / name).read_text())


def test_schema_enums_match_validator_constants():
    p, m = _schema("profile_v2.schema.json"), _schema("dataset_manifest_v2.schema.json")
    props = p["properties"]
    assert props["calibrationState"]["enum"] == list(model.STATES)
    assert props["evidenceLabel"]["enum"] == list(model.LABELS)
    assert props["schema"]["const"] == model.PROFILE_SCHEMA and m["properties"]["schema"]["const"] == model.MANIFEST_SCHEMA
    q = p["$defs"]["quantity"]["properties"]
    assert q["provenance"]["enum"] == list(model.PROVENANCE)
    assert q["identifiability"]["properties"]["status"]["enum"] == list(model.ID_STATUSES)
    assert props["datasets"]["items"]["properties"]["origin"]["enum"] == list(model.ORIGINS)
    assert m["properties"]["origin"]["enum"] == list(model.ORIGINS)


def test_schema_required_fields_present_in_every_migrated_file():
    p = _schema("profile_v2.schema.json")
    for f in (ROOT / "migrated").glob("*.json"):
        doc = json.loads(f.read_text())
        assert set(p["required"]) <= set(doc), f.name
        assert doc["schema"] == p["properties"]["schema"]["const"]


def test_schema_documents_are_draft_2020_12_objects():
    for n in ("profile_v2.schema.json", "dataset_manifest_v2.schema.json"):
        s = _schema(n)
        assert s["$schema"].endswith("2020-12/schema") and s["type"] == "object" and s["required"]


def test_contracts_labels_match_contracts_md():
    text = (ROOT.parents[1] / "Docs" / "ResearchOrchestration" / "CONTRACTS.md").read_text()
    line = next(l for l in text.splitlines() if l.startswith("- Evidence labels:"))
    assert set(x.strip(" .") for x in line.split(":", 1)[1].split("/")) == set(model.LABELS)
