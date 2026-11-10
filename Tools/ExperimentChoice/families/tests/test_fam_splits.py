"""Held-out splits are deterministic, hashed and non-degenerate (EXPLORATORY)."""
import json

from families import envs as E, robotspace as RS, splits as S


def test_spec_hashes_match_the_recorded_ones():
    assert set(S.RECORDED_SPEC_HASHES) == set(S.SPLITS_V0)
    for name in S.SPLITS_V0:
        assert S.spec_hash(name) == S.RECORDED_SPEC_HASHES[name], "split %s changed after it was frozen" % name
        assert S.SPLITS_V0[name]["kind"] in ("interpolation", "extrapolation", "both")


def test_assignment_hash_on_the_reference_sample_matches_the_recorded_one():
    pairs = S.reference_pairs()
    assert len(pairs) == 432
    assert S.assignment_hash(pairs) == S.assignment_hash() == S.RECORDED_ASSIGNMENT_HASH


def test_required_axes_are_present_and_each_has_train_and_test_pairs():
    for name in ("family", "param_range", "robot_size", "module_combination", "sensor", "drive"):
        assert name in S.SPLITS_V0
    counts = S.counts()
    for name, c in counts.items():
        assert c.get(S.TRAIN, 0) >= 80, (name, c)
        kind = S.SPLITS_V0[name]["kind"]
        if kind in ("extrapolation", "both"):
            assert c.get(S.EXTRAP, 0) >= 30, (name, c)
        if kind in ("interpolation", "both"):
            assert c.get(S.INTERP, 0) >= 20, (name, c)


def test_split_rules_hold_on_every_reference_pair():
    sp = S.SPLITS_V0
    assert sorted(sp["family"]["train_families"] + sp["family"]["test_families"]) == sorted(E.FAMILIES)
    arr = sp["module_combination"]
    assert sorted(arr["train"] + arr["interp"] + arr["extrap"]) == sorted(RS.ARRANGEMENTS)
    assert set(sp["param_range"]["keys"]) == set(E.FAMILIES)
    for name, k in sp["param_range"]["keys"].items():                   # extrap band touches an end of the box, interp is interior
        key = "a_cm" if k["key"].startswith("min") else k["key"]
        lo, hi = E.FAMILIES[name].box[key]
        assert k["extrap"][0] == lo or k["extrap"][1] == hi
        assert lo < k["interp"][0] <= k["interp"][1] < hi
        assert k["interp"][1] < k["extrap"][0] or k["extrap"][1] < k["interp"][0]
    for co, rob in S.reference_pairs():
        a = S.assign_all(co, rob)
        assert json.dumps(a)                                              # plain strings
        if a["robot_size"] == S.TRAIN:
            assert rob.L <= 1.0 and rob.W <= 0.72
        if a["robot_size"] == S.EXTRAP:
            assert rob.L >= 1.1 or rob.W >= 0.78
        assert (a["drive"] == S.TRAIN) == (rob.drive == "differential")
        assert (a["family"] == S.TRAIN) == (co.family in sp["family"]["train_families"])
        if a["sensor"] == S.TRAIN:
            assert rob.lidar_fov_deg > 120 and rob.lidar_range_mm > 4000
        assert S.leave_one_family_out(co.family, co) == S.EXTRAP
