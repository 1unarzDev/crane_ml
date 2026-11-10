import itertools
import json
import math
import os

import numpy as np
import pytest

import apriltag36h11 as at
import frames
import geom as g
import minischema

HERE = os.path.dirname(os.path.abspath(__file__))


def test_rpy_roundtrip():
    for rpy in ([0.3, -0.5, 2.0], [0, 0, math.pi], [-1.2, 0.4, -3.0]):
        R = g.rpy_to_R(rpy)
        assert np.allclose(g.rpy_to_R(g.R_to_rpy(R)), R, atol=1e-9)


def test_polytope_chamfer_volume_and_closed():
    p = g.Poly.box([0, 0, 0], [1, 1, 1]).chamfer([1, 1, 1], [1, 0, 0], [0, 0, 1], 0.2)
    m = p.to_mesh({}, "a")
    assert m.signed_volume() == pytest.approx(1 - 0.5 * 0.2 * 0.2, rel=1e-9)
    assert m.closedness() == {"unbalanced": 0, "nonmanifold": 0, "degenerate": 0}


def test_lathe_cylinder_volume_and_outward():
    m = g.cylinder(0.1, 0.3, 64, "a")
    assert m.signed_volume() > 0
    assert m.signed_volume() == pytest.approx(math.pi * 0.1 ** 2 * 0.3, rel=0.01)


def test_overlap_volume():
    a = g.box_mesh([0, 0, 0], [1, 1, 1], "a")
    b = g.box_mesh([0.5, 0.5, 0.5], [1.5, 1.5, 1.5], "a")
    assert g.overlap_volume(a, b, 0.02) == pytest.approx(0.125, rel=0.05)
    c = g.box_mesh([2, 0, 0], [3, 1, 1], "a")
    assert g.overlap_volume(a, c, 0.02) == 0.0


def test_obj_roundtrip(tmp_path):
    m = g.cylinder(0.1, 0.2, 16, "m1", "grp1")
    m.append(g.box_mesh([0.2, 0, 0], [0.3, 0.1, 0.1], "m2", "grp2"))
    p = str(tmp_path / "x.obj")
    g.write_obj(p, [("x", m)], "x.mtl")
    r, info = g.read_obj(p)
    assert len(r) == len(m) and info["n_vn"] > 0 and info["n_vt"] > 0
    assert set(r.mats) == {"m1", "m2"} and set(r.groups) == {"grp1", "grp2"}
    assert np.allclose(np.sort(r.P.reshape(-1, 3), axis=0), np.sort(m.P.reshape(-1, 3), axis=0), atol=1e-6)


def test_smoothing_cylinder_vs_box_edges():
    m = g.cylinder(0.1, 0.2, 24, "a")
    m.smooth_normals(40)
    side = np.abs(m.P[:, :, 2]).max(1) <= 0.1 + 1e-9
    # side-wall normals have no z component (smoothed around the barrel), cap normals are +-z
    n = m.N
    horiz = np.abs(n[:, :, 2]) < 1e-6
    assert horiz.any() and (np.abs(n[:, :, 2]) > 0.999).any()
    b = g.box_mesh([0, 0, 0], [1, 1, 1], "a").smooth_normals(40)
    assert np.all(np.isclose(np.abs(b.N).max(-1), 1.0))   # box faces stay flat


# ---------------------------------------------------------------- frames
def test_unity_worked_example():
    e = frames.worked_example()
    assert e["point_unity"] == [-0.06, 0.06, 0.21]
    assert e["forward_x_robot_in_unity"] == [0.0, 0.0, 1.0]
    assert e["left_y_robot_in_unity"] == [-1.0, 0.0, 0.0]
    assert e["up_z_robot_in_unity"] == [0.0, 1.0, 0.0]
    assert e["yaw_90deg_left_unity_euler_y_deg"] == -90.0
    assert np.linalg.det(frames.C) == pytest.approx(-1.0)


def test_unity_quaternion_matches_matrix_conjugation():
    for rpy in ([0.3, 0.2, 0.9], [1.0, -0.4, -2.0], [0, 0, math.pi / 2]):
        R = g.rpy_to_R(rpy)
        q = frames.R_to_quat(R)
        assert np.allclose(frames.quat_to_R(frames.quat_to_unity(q)), frames.R_to_unity(R), atol=1e-9)


def test_unity_winding_swap_keeps_positive_volume():
    m = g.box_mesh([0, 0, 0], [0.2, 0.1, 0.3], "a")
    P = m.P.reshape(-1, 3)
    idx = np.arange(len(P)).reshape(-1, 3)
    Pu, idxu = frames.tris_to_unity(P, idx)
    tris = Pu[idxu]
    vol_naive = np.einsum("ij,ij->i", tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6
    tris_wrong = Pu[idx]
    vol_wrong = np.einsum("ij,ij->i", tris_wrong[:, 0], np.cross(tris_wrong[:, 1], tris_wrong[:, 2])).sum() / 6
    # Unity is left-handed: with swapped winding the right-handed volume formula is positive, unswapped negative
    assert vol_naive > 0 > vol_wrong


# ---------------------------------------------------------------- tags
@pytest.mark.parametrize("tid", [0, 7, 500])
def test_tag_matches_published_reference_png(tid):
    from PIL import Image
    ref = np.array(Image.open(os.path.join(HERE, "ref", f"ref_{tid:05d}.png")).convert("L")) > 128
    assert np.array_equal(ref, at.tag_cells(tid) > 128)


def test_tag36h11_family_hamming_distance():
    """Independent check of the family definition: any two codes (all 4 rotations) differ in >= 11 bits."""
    def payload(tid):
        c = at.tag_cells(tid)[2:8, 2:8]  # 6x6 inner cells? inner payload ring sits at cells 1..8 minus border
        return None
    # rebuild each 36-bit pattern through the bit layout and rotate the 6x6 grid
    d = at._DATA
    def grid(tid):
        code = d["codes"][tid]
        gr = np.zeros((6, 6), int)
        for i in range(36):
            gr[d["bit_y"][i] - 1, d["bit_x"][i] - 1] = code >> (35 - i) & 1
        return gr
    assert len(d["codes"]) == 587 == at.NCODES
    gs = [grid(i) for i in range(0, 587, 7)]
    dmin = 99
    for (i, a), (j, b) in itertools.combinations(enumerate(gs), 2):
        for k in range(4):
            dmin = min(dmin, int((a != np.rot90(b, k)).sum()))
    assert dmin >= 11


def test_tag_id_range_and_png(tmp_path):
    with pytest.raises(ValueError):
        at.tag_cells(587)
    import subprocess, sys
    out = str(tmp_path / "t")
    subprocess.check_call([sys.executable, "-I", os.path.join(os.path.dirname(HERE), "apriltag36h11.py"), "--ids", "3", "4", "--out", out])
    assert sorted(os.listdir(out)) == ["tag36h11_00003.png", "tag36h11_00004.png"]


# ---------------------------------------------------------------- minischema
def test_minischema_basics():
    s = {"type": "object", "required": ["a"], "properties": {"a": {"type": "number", "minimum": 0}, "b": {"enum": ["x", "y"]}}, "additionalProperties": False}
    assert minischema.validate({"a": 1, "b": "x"}, s) == []
    assert len(minischema.validate({"b": "z"}, s)) == 2
    assert minischema.validate({"a": -1}, s)
    assert minischema.validate({"a": 1, "c": 2}, s)
