#!/usr/bin/env python3
"""Independent (numpy) evaluation of the C# perception parity vectors.

Usage:  python3 -I check_parity.py [perception_parity_v0.json] [--no-compare]

Three independent pieces, none sharing code with the C# library:
  1. visibility (projection, FOV, pixel size, view angle, range, occlusion) -- the occlusion test is a slab clip
     written differently from the C# one AND cross-checked by dense point sampling along each corner ray;
  2. first-order pose-error scale from a *numerical* Jacobian (central differences) instead of the analytic one;
  3. an informational comparison against the existing Python fast models (docking_mc tag-noise law and the
     observer_value visibility constants), which are different models by design; differences are reported, not asserted.
Exit status 1 when visibility or error-scale parity fails.
"""
import json, os, sys, importlib.util
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
args = [a for a in sys.argv[1:] if not a.startswith("--")]
PATH = args[0] if args else os.path.join(HERE, "perception_parity_v0.json")
COMPARE = "--no-compare" not in sys.argv


def rot_exp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + skew(w)
    k = w / th
    K = skew(k)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0.0]])


def corners_local(s):
    h = s / 2
    return np.array([[-h, -h, 0], [h, -h, 0], [h, h, 0], [-h, h, 0.0]])


def seg_hits_box_slab(p0, p1, c, half, yaw, eps):
    """Segment (last eps metres removed) vs yawed box; per-axis interval intersection with explicit parallel handling."""
    d = p1 - p0
    L = np.linalg.norm(d)
    if L > 0:
        p1 = p0 + d * (1.0 - min(eps / L, 0.5))
    cs, sn = np.cos(yaw), np.sin(yaw)
    Rt = np.array([[cs, sn, 0], [-sn, cs, 0], [0, 0, 1.0]])           # world -> box
    a, b = Rt @ (p0 - c), Rt @ (p1 - c)
    lo, hi = 0.0, 1.0
    for ax in range(3):
        da = b[ax] - a[ax]
        if abs(da) < 1e-12:
            if abs(a[ax]) > half[ax]:
                return False
            continue
        t1, t2 = (-half[ax] - a[ax]) / da, (half[ax] - a[ax]) / da
        lo, hi = max(lo, min(t1, t2)), min(hi, max(t1, t2))
        if lo > hi:
            return False
    return True


def seg_hits_box_sampled(p0, p1, c, half, yaw, eps, n=20001):
    d = p1 - p0
    L = np.linalg.norm(d)
    p1 = p0 + d * (1.0 - min(eps / L, 0.5)) if L > 0 else p1
    t = np.linspace(0, 1, n)[:, None]
    pts = p0 + (p1 - p0) * t
    cs, sn = np.cos(yaw), np.sin(yaw)
    loc = (pts - c) @ np.array([[cs, -sn, 0], [sn, cs, 0], [0, 0, 1.0]])   # rows: R_box^T (p - c)
    return bool(np.any(np.all(np.abs(loc) <= half + 1e-9, axis=1)))


def visibility(case, sampled=False):
    cam, tag, P = case["camera"], case["tag"], case["params"]
    W, H, hf = cam["width"], cam["height"], cam["hfov"]
    f = W / (2 * np.tan(hf / 2))
    Rc, pc = np.array(cam["R"]).reshape(3, 3), np.array(cam["position"])
    Rt, ct = np.array(tag["R"]).reshape(3, 3), np.array(tag["centre"])
    s = tag["size"]
    X = corners_local(s) @ Rt.T + ct                                 # world corners
    n = Rt[:, 2]
    to_cam = pc - ct
    rng = np.linalg.norm(to_cam)
    cosv = float(n @ to_cam) / rng
    out = dict(range=rng, view=np.arccos(np.clip(cosv, -1, 1)), blocked=0, pix=float("nan"))
    if cosv <= 0 or out["view"] > P["max_view_angle"]:
        return "TooOblique", out
    pcam = (X - pc) @ Rc                                              # rows: Rc^T (X - pc)
    if np.any(pcam[:, 2] < P["near_plane"]):
        return "Behind", out
    u = f * pcam[:, 0] / pcam[:, 2] + W / 2
    v = f * pcam[:, 1] / pcam[:, 2] + H / 2
    if np.any((u < 0) | (u > W) | (v < 0) | (v > H)):
        return "OutOfFov", out
    x, y = u, v
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))
    out["pix"] = float(np.sqrt(area))
    if rng < P["min_range"]:
        return "TooClose", out
    if out["pix"] < P["min_pixels"]:
        return "TooSmall", out
    hit = seg_hits_box_sampled if sampled else seg_hits_box_slab
    for k in range(4):
        if any(hit(pc, X[k], np.array(b["centre"]), np.array(b["half"]), b["yaw"], P["occlusion_eps"]) for b in case["boxes"]):
            out["blocked"] += 1
    return ("Occluded" if out["blocked"] else "Visible"), out


def error_scale_numeric(case, sigma_px=1.0):
    cam, tag = case["camera"], case["tag"]
    W, H, hf = cam["width"], cam["height"], cam["hfov"]
    f = W / (2 * np.tan(hf / 2))
    Rc, pc = np.array(cam["R"]).reshape(3, 3), np.array(cam["position"])
    Rt, ct = np.array(tag["R"]).reshape(3, 3), np.array(tag["centre"])
    Rct, tct = Rc.T @ Rt, Rc.T @ (ct - pc)                          # T_cam_tag
    Xl = corners_local(tag["size"])

    def proj(R, t):
        pcm = Xl @ R.T + t
        return np.concatenate([[f * q[0] / q[2] + W / 2, f * q[1] / q[2] + H / 2] for q in pcm])

    J = np.zeros((8, 6))
    h = 1e-6
    for k in range(6):
        e = np.zeros(6)
        e[k] = h
        up = proj(rot_exp(e[3:]) @ Rct, tct + e[:3])
        e[k] = -h
        dn = proj(rot_exp(e[3:]) @ Rct, tct + e[:3])
        J[:, k] = (up - dn) / (2 * h)
    S = np.linalg.inv(J.T @ J) * sigma_px ** 2
    return float(np.sqrt(np.trace(S[:3, :3]))), float(np.sqrt(np.trace(S[3:, 3:]))), S, Rc, Rt, tct


def load_py_models():
    """docking_mc / observer_value models, if importable (informational only)."""
    try:
        sys.dont_write_bytecode = True
        p = os.path.join(FLEET, "observer_value", "model.py")
        spec = importlib.util.spec_from_file_location("observer_value_model", p)
        m = importlib.util.module_from_spec(spec)
        sys.modules["observer_value_model"] = m
        spec.loader.exec_module(m)
        return m
    except Exception as e:  # pragma: no cover
        print("  (observer_value model not importable: %s)" % e)
        return None


def main():
    data = json.load(open(PATH))
    cases = data["cases"]
    print("cases: %d   file: %s" % (len(cases), PATH))
    dis, cat = [], {}
    sampled_dis = []
    scale_rel = []
    for c in cases:
        reason, info = visibility(c)
        reason_s, _ = visibility(c, sampled=True)
        e = c["expected"]
        ok = reason == e["reason"] and (reason == "Visible") == e["visible"]
        cat.setdefault(c["category"], [0, 0])
        cat[c["category"]][0] += 1
        cat[c["category"]][1] += ok
        if not ok:
            dis.append((c["id"], e["reason"], reason, info))
        if reason_s != reason:
            sampled_dis.append((c["id"], reason, reason_s))
        if ok and e["visible"]:
            sp, sr, *_ = error_scale_numeric(c)
            scale_rel.append((abs(sp - e["sigma_pos_m"]) / e["sigma_pos_m"], abs(sr - e["sigma_rot_rad"]) / e["sigma_rot_rad"], c["id"]))
        if ok and e["blocked_corners"] != info["blocked"] and reason == "Occluded":
            dis.append((c["id"], "blocked_corners=%d" % e["blocked_corners"], "blocked_corners=%d" % info["blocked"], info))
    n = len(cases)
    agree = n - len([d for d in dis if not str(d[1]).startswith("blocked")])
    print("\nVISIBILITY PARITY (C# vs independent numpy): %d / %d cases agree (%.1f%%)" % (agree, n, 100.0 * agree / n))
    for k, (t, a) in sorted(cat.items()):
        print("   %-11s %3d / %3d" % (k, a, t))
    print("   slab clip vs dense sampling inside Python: %d disagreements" % len(sampled_dis))
    for d in sampled_dis[:10]:
        print("     ", d)
    vis = [c for c in cases if c["expected"]["visible"]]
    mp = max(s[0] for s in scale_rel) if scale_rel else float("nan")
    mr = max(s[1] for s in scale_rel) if scale_rel else float("nan")
    print("\nERROR-SCALE PARITY on %d visible cases (numerical vs analytic Jacobian): max rel diff  position %.2e   rotation %.2e" % (len(scale_rel), mp, mr))
    if dis:
        print("\nDISAGREEMENTS (%d):" % len(dis))
        for d in dis:
            print("   %s  C#=%s  python=%s   range=%.4f view=%.2fdeg pix=%.3f" % (d[0], d[1], d[2], d[3]["range"], np.degrees(d[3]["view"]), d[3]["pix"]))
    else:
        print("\nNo disagreements.")
    if COMPARE:
        compare_with_models(cases)
    fail = bool(dis) or mp > 1e-4 or mr > 1e-4
    print("\nRESULT:", "FAIL" if fail else "PASS")
    return 1 if fail else 0


def compare_with_models(cases):
    ov = load_py_models()
    if ov is None:
        return
    DM = ov.DM
    Pm, Q = DM.Params(), ov.OP()
    print("\nINFORMATIONAL COMPARISON with the existing Python fast models (different models by design)")
    # (a) visibility constants of observer_value: min_px 20, max_alpha 65 deg, rmin_cam 0.35, ideal pinhole, no occluders
    f_px = ov.f_px
    tot = agree = 0
    flip = {"ov_visible_cs_not": 0, "cs_visible_ov_not": 0}
    for c in cases:
        if c["boxes"]:
            continue
        rng, view = None, None
        _, info = visibility(c)
        s, r, alpha = c["tag"]["size"], info["range"], info["view"]
        cam_ok = (r >= Q.rmin_cam) & (r <= s * f_px(c["camera"]["hfov"], c["camera"]["width"]) / Q.min_px) & (alpha <= Q.max_alpha)
        # observer_value has no FOV test for rover cameras in this function; use C#'s FOV verdict to isolate the pixel/angle/range rules
        cs_ok = c["expected"]["reason"] in ("Visible", "TooSmall") and c["expected"]["view_angle"] <= c["params"]["max_view_angle"]
        cs_ok_full = c["expected"]["visible"]
        if c["expected"]["reason"] in ("OutOfFov", "Behind"):
            continue
        tot += 1
        if bool(cam_ok) == bool(cs_ok_full):
            agree += 1
        elif cam_ok and not cs_ok_full:
            flip["ov_visible_cs_not"] += 1
        else:
            flip["cs_visible_ov_not"] += 1
    print("  observer_value cam_range_ok (min_px=20, alpha<=65 deg) vs C# (min_px=32 default) on %d box-free in-FOV cases: agree %d, disagree %d  %s" % (tot, agree, tot - agree, flip))
    print("     -> disagreements are the N-pixels parameter: 20 px (observer_value ASSUMPTION) vs 32 px (cited community threshold, 4 px/cell); the C# value is a parameter.")
    # (b) docking_mc / observer_value tag noise law vs the geometric first-order covariance (face-sized 5 cm tags, visible)
    rows = []
    for c in cases:
        e = c["expected"]
        if not e["visible"] or c["boxes"]:
            continue
        sp, sr, S, Rc, Rt, tct = error_scale_numeric(c)
        alpha = e["view_angle"]
        # effective corner sigma of the C# default model: 1 px * (1 + 0.5 (1/cos a - 1))
        k = (1 + 0.5 * (1 / np.cos(alpha) - 1))
        Sg = S * k * k
        lat_cam = Rc.T @ Rt[:, 0]
        up_cam = Rc.T @ Rt[:, 1]
        sy_cs = float(np.sqrt(lat_cam @ Sg[:3, :3] @ lat_cam))
        spsi_cs = float(np.sqrt(up_cam @ Sg[3:, 3:] @ up_cam))
        pc_, ct_ = np.array(c["camera"]["position"]), np.array(c["tag"]["centre"])
        u = (ct_ - pc_) / np.linalg.norm(ct_ - pc_)
        ut = float(u @ Rt[:, 0])
        sy_py, sp_py = ov.helper_sigma(np.array(e["range"]), np.array(alpha), np.array(ut), c["tag"]["size"], Pm, Q)
        rows.append((e["range"], c["tag"]["size"], np.degrees(alpha), sy_cs, float(sy_py), spsi_cs, float(sp_py), float(np.sqrt(np.trace(Sg[:3, :3])))))
    rows = np.array(rows)
    print("  %d visible box-free cases: lateral (tag-x) position sigma and tag-yaw sigma, C# geometric vs docking_mc/observer_value law" % len(rows))
    for lo, hi in ((0, 0.6), (0.6, 1.0), (1.0, 2.0), (2.0, 9.0)):
        m = (rows[:, 0] >= lo) & (rows[:, 0] < hi)
        if m.sum() == 0:
            continue
        ry = rows[m, 3] / rows[m, 4]
        rp = rows[m, 5] / rows[m, 6]
        print("    range %.1f-%.1f m (n=%3d): sigma_y ratio C#/py median %.2f [%.2f..%.2f];  sigma_psi ratio median %.2f [%.2f..%.2f]" % (lo, hi, m.sum(), np.median(ry), ry.min(), ry.max(), np.median(rp), rp.min(), rp.max()))
    # anchor: 16.4 cm tag, 2 m frontal, the literature point docking_mc was scaled from (~2 cm)
    c = {"camera": {"width": 1280, "height": 800, "hfov": 1.40, "position": [0, 0, 0.3], "R": [0, 0, 1, -1, 0, 0, 0, -1, 0]},
         "tag": {"size": 0.164, "centre": [2.0, 0, 0.3], "R": [0, 1, -1, 0, 0, 0, 0, 0, 0]}}
    # tag normal = -x (toward camera), x right (world -y), y up (+z)
    Rt = np.column_stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])
    c["tag"]["R"] = list(Rt.flatten())
    Rc = np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    c["camera"]["R"] = list(Rc.flatten())
    sp, sr, S, *_ = error_scale_numeric(c)
    depth = float(np.sqrt(S[2, 2]))
    print("  literature anchor (16.4 cm tag, 2 m, frontal, 1 px corners): C# first-order sigma along line of sight %.1f mm, cross-range %.1f mm, 3-D RMS %.1f mm; docking_mc scaling target ~20 mm." % (1000 * depth, 1000 * np.sqrt(S[0, 0]), 1000 * sp))
    print("     (single-frame first-order values; the C# model additionally has frontal pose ambiguity that the first-order covariance omits.)")


if __name__ == "__main__":
    sys.exit(main())
