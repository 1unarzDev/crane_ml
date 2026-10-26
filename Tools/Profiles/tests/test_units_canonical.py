import json
import math

import pytest

from crane_profiles import canonical, units


def test_dimension_algebra():
    P = units.parse_unit
    assert P("N").dim == P("kg m/s^2").dim
    assert P("N s/m").dim == P("kg/s").dim
    assert P("N/(rad/s)^2").dim == (P("N") / (P("rad") / P("s")) ** 2).dim
    assert P("N s/(m rad)").dim == (P("N") * P("s") / (P("m") * P("rad"))).dim
    assert P("V").dim == P("kg m^2/(s^3 A)").dim
    assert P("ohm").dim == (P("V") / P("A")).dim
    assert P("kg m^2").dim == (P("kg") * P("m") ** 2).dim
    assert P("Ah").dim == P("A s").dim and P("Ah").scale == 3600.0
    assert P("1").dim == P("-").dim == units.ONE.dim
    assert P("N/norm^2").dim[-1] == -2
    assert P("N m s^2/rad^2").dim == (P("N") * P("m") * P("s") ** 2 / P("rad") ** 2).dim


def test_division_is_left_associative():
    assert units.parse_unit("N s/m").dim == (units.parse_unit("N") * units.parse_unit("s") / units.parse_unit("m")).dim
    assert units.parse_unit("N/(m rad)").dim == (units.parse_unit("N") / (units.parse_unit("m") * units.parse_unit("rad"))).dim


@pytest.mark.parametrize("bad", ["", "  ", "furlong", "N/", "(m", "m)", "m^", "kg ?", None, 3])
def test_bad_units_rejected(bad):
    with pytest.raises(units.UnitError):
        units.parse_unit(bad)


def test_conversion():
    assert units.convert(180.0, "deg", "rad") == pytest.approx(math.pi)
    assert units.convert(250.0, "mm", "m") == pytest.approx(0.25)
    assert units.convert(1.0, "Ah", "A s") == 3600.0
    with pytest.raises(units.UnitError):
        units.convert(1.0, "m", "s")


@pytest.mark.parametrize("key,suffix", [("massKg", "Kg"), ("inertiaKgM2", "KgM2"), ("wheelbaseM", "M"), ("peakWheelTorqueNm", "Nm"),
                                        ("tauUpS", "S"), ("accelerationMps2", "Mps2"), ("batteryResistanceOhm", "Ohm"),
                                        ("motorLagSeconds", "Seconds"), ("massPerLengthKgPerM", None), ("gazeboMu", None),
                                        ("latencyScans", None), ("Xu", None), ("kf", None)])
def test_key_suffix(key, suffix):
    assert units.key_suffix(key) == suffix


def test_hash_ignores_key_order_and_own_field():
    a = {"b": 1, "a": [1, 2, {"z": 1, "y": 2}]}
    b = {"a": [1, 2, {"y": 2, "z": 1}], "b": 1}
    assert canonical.content_hash(a) == canonical.content_hash(b)
    sealed = canonical.seal(a)
    assert canonical.content_hash(sealed) == sealed["contentHash"]
    assert canonical.seal(sealed) == sealed


def test_hash_changes_with_any_value():
    a = {"x": 1.0}
    assert canonical.content_hash(a) != canonical.content_hash({"x": 1.0000000000000002})


def test_canonical_rejects_nan():
    with pytest.raises(ValueError):
        canonical.canonical_json({"x": float("nan")})


def test_written_file_roundtrips_hash(tmp_path):
    h = canonical.write_sealed({"k": [1, 2, 3], "ü": "x"}, tmp_path / "d" / "f.json")
    doc = json.loads((tmp_path / "d" / "f.json").read_text(encoding="utf-8"))
    assert doc["contentHash"] == h == canonical.content_hash(doc)
