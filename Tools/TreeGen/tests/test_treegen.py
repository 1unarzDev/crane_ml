# SPDX-License-Identifier: GPL-3.0-or-later
import json
import time

import numpy as np
import pytest

from treegen import colliders as col
from treegen import metrics
from treegen.export import export_tree
from treegen.forest import grow_stand, layout, DEFAULT_STAND
from treegen.glb import read_glb, write_glb
from treegen.growth import grow_tree
from treegen.mesh import build_lods, leaf_cards
from treegen.rng import stream
from treegen.skeleton import Skeleton
from treegen.species import load_species
from treegen.variants import make_log, make_snag

SMALL = {"seasons": 14}
# Golden skeleton hashes (forest_seed=11, tree_id=3, seasons=14). Regenerate deliberately with
#   python -c "from tests.test_treegen import print_golden; print_golden()"  (from Tools/TreeGen)
GOLDEN = {
    "beech": "2d0a0e7fc2578500e14734d81f23e096f4c281e08bfed90e4ac1313a8abc2c30",
    "spruce": "9a554082942c755b219271b8c634616029ac6d4d6dd3b4d590841eb5203f2caf",
    "birch": "1070900c58f69652572b6b5a9a6979b02edbcd137d9df3bf41fc90f510f4c8c9",
}


def _small(name, seed=11, tid=3):
    return grow_tree(load_species(name, SMALL), seed, tid)


@pytest.fixture(scope="module")
def trees():
    return {n: _small(n) for n in ("beech", "spruce", "birch")}


def print_golden():
    for n in ("beech", "spruce", "birch"):
        print(n, _small(n).topology_hash())


# ------------------------------------------------------------------ determinism
def test_rng_streams_independent():
    a = stream(1, 2, "growth").random(4)
    b = stream(1, 2, "growth").random(4)
    c = stream(1, 2, "leaves").random(4)
    d = stream(1, 3, "growth").random(4)
    assert np.array_equal(a, b) and not np.array_equal(a, c) and not np.array_equal(a, d)


@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_golden_hash(trees, name):
    h = trees[name].topology_hash()
    assert h == _small(name).topology_hash(), "regrowth not bit-identical"
    if GOLDEN[name] != "__FILL__":
        assert h == GOLDEN[name], f"golden hash changed for {name}: {h}"


def test_seed_changes_tree(trees):
    assert _small("birch", tid=4).topology_hash() != trees["birch"].topology_hash()


def test_leaf_stream_does_not_touch_topology(trees):
    sk = trees["birch"]
    sp = load_species("birch")
    h0 = sk.topology_hash()
    sp2 = load_species("birch", {"leaf": {"cards_per_node": 7}})
    leaf_cards(sk, sp2, 11, 3)
    assert sk.topology_hash() == h0 == _small("birch").topology_hash()


def test_json_roundtrip(trees, tmp_path):
    sk = trees["spruce"]
    p = tmp_path / "s.json"
    sk.save_json(p)
    sk2 = Skeleton.load_json(p)
    d = json.loads(p.read_text())
    assert d["skeleton_hash"] == sk.topology_hash()
    assert d["params_hash"] and d["forest_seed"] == 11 and d["tree_id"] == 3
    assert np.allclose(sk2.pos, sk.pos, atol=1e-5) and np.array_equal(sk2.parent, sk.parent)


# ------------------------------------------------------------------ structure
@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_topology_valid(trees, name):
    sk = trees[name]
    assert sk.parent[0] == -1 and np.all(sk.parent[1:] < np.arange(1, sk.n)) and np.all(sk.parent[1:] >= 0)
    mc_count = np.bincount(sk.parent[(sk.parent >= 0) & sk.is_main], minlength=sk.n)
    assert mc_count.max() <= 1
    assert np.all(sk.radius > 0)


@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_pipe_model_consistency(trees, name):
    """r_p^n == sum_children r_c^n + shed contributions + tip term, on the unflared pipe radii."""
    sk = trees[name]
    e = sk.meta["n_pipe"]
    r = sk.pipe_radius
    lhs = r ** e
    rhs = np.zeros(sk.n)
    np.add.at(rhs, sk.parent[1:], r[1:] ** e)
    rhs += sk.shed_pipe
    tips = sk.tips()
    rhs[tips] += sk.meta["r_tip"] ** e
    check = sk.alive
    rel = np.abs(lhs[check] - rhs[check]) / lhs[check]
    assert np.median(rel) < 1e-9
    assert np.mean(rel < 1e-6) > 0.99, f"pipe violations {np.mean(rel >= 1e-6):.3%}"
    # children never thicker than parents
    assert np.all(r[1:] <= r[sk.parent[1:]] * (1 + 1e-9))


def test_pipe_exponent_fit_matches_preset(trees):
    for name, sk in trees.items():
        n_fit, ns = metrics.pipe_exponent_fit(sk, sk.pipe_radius)
        assert len(ns) > 5
        # junction fit ignores shed and tip terms, so it can only over-estimate a little
        assert abs(n_fit - sk.meta["n_pipe"]) < 0.6, (name, n_fit)


def test_tropism_by_order(trees):
    sk = trees["spruce"]
    ang = metrics.branching_angles(sk)
    assert 55 < np.median(ang[1]) < 100   # spruce laterals near-horizontal
    sk = trees["birch"]
    assert np.median(metrics.branching_angles(sk)[1]) < 70


def test_shedding_and_stubs():
    sk = grow_tree(load_species("spruce", {"seasons": 24}), 5, 1)
    assert (~sk.alive).sum() > 0, "spruce should retain dead stubs"
    assert (sk.shed_pipe > 0).sum() > 0, "some branches should have been shed"


# ------------------------------------------------------------------ colliders
@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_collider_coverage(trees, name):
    sk = trees[name]
    caps, softs, cfg = col.build_colliders(sk)
    cov, missing = col.coverage_report(sk, caps)
    assert cov == 1.0, f"{len(missing)} hard internodes uncovered"
    # band rule: a 3 cm branch at 2 m must be hard, at 6 m soft
    assert col.segment_threshold(np.array([2.0]), np.array([2.2]), cfg)[0] == 0.02
    assert col.segment_threshold(np.array([6.0]), np.array([6.2]), cfg)[0] == 0.05
    hard = col.hard_mask(sk)
    kid = np.nonzero(sk.parent >= 0)[0]
    assert np.all(hard[kid] == (2 * sk.radius[kid] >= col.segment_threshold(
        sk.pos[sk.parent[kid], 2], sk.pos[kid, 2], cfg)))


def test_colliders_json_unity_fields(trees):
    sk = trees["beech"]
    caps, softs, cfg = col.build_colliders(sk)
    j = col.colliders_json(sk, caps, softs, cfg)
    c = j["capsules"][0]
    for k in ("a", "b", "center", "axis", "radius", "height"):
        assert k in c
    assert c["height"] >= 2 * c["radius"]
    assert j["skeleton_hash"] == sk.topology_hash()


# ------------------------------------------------------------------ meshes
def _check_part(part):
    assert len(part.pos) and len(part.tri)
    assert part.tri.max() < len(part.pos) and part.tri.min() >= 0
    assert np.all(np.isfinite(part.pos)) and np.all(np.isfinite(part.uv)) and np.all(np.isfinite(part.nrm))
    a, b, c = (part.pos[part.tri[:, i]] for i in range(3))
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    assert np.mean(area > 1e-12) > 0.999


@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_mesh_validity_and_lods(trees, name, tmp_path):
    sk = trees[name]
    sp = load_species(name)
    lods, centres, fol = build_lods(sk, sp, 11, 3)
    tri = []
    for lod, (bark, leaves) in lods.items():
        _check_part(bark)
        if not leaves.empty:
            _check_part(leaves)
        tri.append(len(bark.tri))
        write_glb(tmp_path / f"l{lod}.glb", [bark, leaves])
        g = read_glb(tmp_path / f"l{lod}.glb")
        assert g["json"]["asset"]["version"] == "2.0"
        assert len(g["meshes"]["bark"]["POSITION"]) == len(bark.pos)
        assert g["meshes"]["bark"]["indices"].max() < len(bark.pos)
    assert tri[0] >= tri[1] >= tri[2]
    # outward-facing bark: face normals agree with stored radial normals
    bark = lods[0][0]
    a, b, c = (bark.pos[bark.tri[:, i]] for i in range(3))
    fn = np.cross(b - a, c - a)
    vn = bark.nrm[bark.tri[:, 0]] + bark.nrm[bark.tri[:, 1]] + bark.nrm[bark.tri[:, 2]]
    good = (fn * vn).sum(1) > 0
    assert good.mean() > 0.97


def test_bark_uv_texel_density(trees):
    sk = trees["beech"]
    sp = load_species("beech")
    lods, _, _ = build_lods(sk, sp, 11, 3, lods=(0,), with_leaves=False)
    bark = lods[0][0]
    # along-axis density: dV/dlength == 1/bark_tile on the stem's first segment
    a, b = bark.pos[0], bark.pos[sum(1 for _ in range(0))]
    tile = sp["bark_tile_m"]
    k1 = np.nonzero(np.abs(bark.uv[:, 0]) < 1e-9)[0][:2]   # u = 0 seam column of rings 0 and 1
    dl = np.linalg.norm(bark.pos[k1[1]] - bark.pos[k1[0]])
    dv = abs(bark.uv[k1[1], 1] - bark.uv[k1[0], 1])
    assert dl > 0 and abs(dv * tile / dl - 1) < 0.15


# ------------------------------------------------------------------ variants, stand, export
def test_variants(trees):
    sk = trees["beech"]
    sn = make_snag(sk, 11, 3)
    assert not sn.alive.any() and sn.n < sk.n and sn.pos[:, 2].max() < sk.pos[:, 2].max()
    lg = make_log(sk, 11, 3)
    assert lg.pos[:, 2].max() < 2.0 and np.min(lg.pos[:, 2] - lg.radius) >= -1e-9


def test_stand_layout_deterministic():
    a = layout({**DEFAULT_STAND, "n_trees": 15}, 3)
    b = layout({**DEFAULT_STAND, "n_trees": 15}, 3)
    assert a == b and len(a) == 15
    xy = np.array([[t["x"], t["y"]] for t in a])
    d = np.hypot(*(xy[:, None] - xy[None]).transpose(2, 0, 1)) + np.eye(len(xy)) * 1e9
    assert d.min() >= DEFAULT_STAND["min_spacing_m"]


def test_stand_competition_and_export(tmp_path):
    cfg = {"n_trees": 6, "width_m": 12, "depth_m": 12, "min_spacing_m": 2.5, "p_snag": 0.0, "p_log": 0.0,
           "species_mix": {"birch": 1.0}, "age_spread_seasons": 0}
    t = time.time()
    r = grow_stand(cfg, 2, str(tmp_path), write_obj_files=False, log=None)
    dt = time.time() - t
    man = json.loads((tmp_path / "stand_manifest.json").read_text())
    assert len(man["trees"]) == 6
    for tr in man["trees"]:
        d = tmp_path / tr["dir"]
        assert (d / "skeleton.json").exists() and (d / "colliders.json").exists() and (d / "lod0.glb").exists()
    # shared light field: the same trees grown alone (same seeds) carry more live wood than in the stand
    alone = [grow_tree(load_species("birch"), 2, it["tree_id"]).alive.sum() for it in r["items"]]
    crowded = [s.alive.sum() for s in r["skeletons"]]
    assert np.sum(crowded) < np.sum(alone)
    print(f"stand of 6 birches: {dt:.2f}s")


@pytest.mark.parametrize("name", ["beech", "spruce", "birch"])
def test_performance_full_tree(name, tmp_path):
    sp = load_species(name)
    t = time.time()
    sk = grow_tree(sp, 7, 0)
    tg = time.time() - t
    t = time.time()
    export_tree(sk, sp, str(tmp_path), 7, 0, write_obj_files=False)
    te = time.time() - t
    print(f"PERF {name}: nodes={sk.n} grow={tg:.2f}s export(glb+json)={te:.2f}s")
    assert tg < 30 and te < 30
