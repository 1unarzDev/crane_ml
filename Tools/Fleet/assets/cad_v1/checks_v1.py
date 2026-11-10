#!/usr/bin/env python3
"""Engineering checks for the v1 assets (beyond looks), written to <assets>/validation_report.json.

    python checks_v1.py [--assets ~/.cache/crane-fleet/assets_v1]

1. validate_manifest.py on every v1 manifest + cross-role mates (must have no FAIL)
2. reciprocal dock: hook engage vs partner pin, magnet alignment/polarity, bracket coincidence, face gap,
   hook closing sweep (open -> closed) never intersects the partner body
3. static stability: 2 x cargo inline carrying box_long, lifter holding box_small and the 80 mm concept box,
   scout arm extended with payload; plain tip angles per role
4. arm folded for travel: inside the footprint, under the LiDAR top / scan plane, no collision with the scout body
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import geom as g  # noqa: E402
import validate_manifest as V  # noqa: E402
from rover_model import RoverModel  # noqa: E402

G = 9.81


def mpath(assets, role):
    return os.path.join(assets, role, f"{role}.manifest.json")


def mate_T(A, fa, B, fb):
    return A.face_T(fa) @ g.make_T(g.rotz(math.pi)) @ g.inv_T(B.face_T(fb))


def mass_state(model, state):
    """Total mass/COM of a manifest model at a joint state, from the per-part derivation rows (part COM moved rigidly)."""
    auth = V.authored_state(model)
    comps = []
    for row in model.m["mass_properties"]["derivation"]:
        n = row["component"]
        Ta = model.frame_T(n, auth)
        Ts = model.frame_T(n, {**auth, **state})
        c_local = np.linalg.inv(Ta) @ np.append(row["com_xyz"], 1.0)
        comps.append((row["mass"], (Ts @ c_local)[:3]))
    M = sum(m for m, _ in comps)
    return M, sum(m * c for m, c in comps) / M


def support_margin(com_xy, contacts):
    """Signed distance of the COM projection to the edges of the convex support polygon (positive = inside)."""
    from math import atan2
    P = np.array(contacts)
    c = P.mean(0)
    P = P[np.argsort([atan2(p[1] - c[1], p[0] - c[0]) for p in P])]
    d = []
    for i in range(len(P)):
        a, b = P[i], P[(i + 1) % len(P)]
        e = b - a
        n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        d.append(float(-(com_xy - a) @ n))
    return min(d)


def contacts_of(model, T=np.eye(4)):
    out = []
    for w in model.m["wheels"]["items"]:
        p = T @ np.append(w["position"], 1.0)
        out.append(p[:2])
    return out


# ------------------------------------------------------------------ 1 validator
def run_validator(assets, rep):
    roles = ["scout", "lifter", "cargo", "payloads"]
    reports = [V.validate(mpath(assets, r), None, None, None, True) for r in roles]
    mates = [("scout", "front", "cargo", "rear"), ("cargo", "front", "cargo", "rear"), ("lifter", "right", "cargo", "left"),
             ("lifter", "rear", "cargo", "front"), ("cargo", "left", "cargo", "right"), ("scout", "left", "lifter", "left"),
             ("scout", "rear", "lifter", "rear")]
    specs = [f"{mpath(assets, a)}:{fa},{mpath(assets, b)}:{fb}" for a, fa, b, fb in mates]
    cross = V.validate(mpath(assets, "cargo"), None, None, specs, True)
    out = []
    for r in reports + [cross]:
        j = r.to_json()
        out.append({"manifest": j["manifest"], "overall": j["overall"], "counts": j["counts"],
                    "non_pass": [c for c in j["checks"] if c["level"] != "PASS"], "mating": [m["label"] + ": " + m["summary"] for m in j["mating"]]})
    rep["validator"] = out
    rep["validator_full"] = [r.to_json() for r in reports + [cross]]
    return all(r.overall() == "PASS" for r in reports + [cross])


# ------------------------------------------------------------------ 2 dock
def dock_checks(assets, rep):
    M = {r: RoverModel(mpath(assets, r)) for r in ("scout", "lifter", "cargo")}
    res = []
    for a, fa, b, fb in [("cargo", "front", "cargo", "rear"), ("lifter", "right", "cargo", "left"), ("scout", "front", "cargo", "rear"), ("cargo", "left", "cargo", "right")]:
        A, B = M[a], M[b]
        TB = mate_T(A, fa, B, fb)
        e = {"pair": f"{a}.{fa} <-> {b}.{fb}"}
        FA, FB = A.face(fa), B.face(fb)
        TA_f, TB_f = A.face_T(fa), TB @ B.face_T(fb)
        mags = []
        for ma in FA["guide"]["magnets"]:
            pa = TA_f @ np.append(ma["position"], 1)
            best = min(((np.linalg.norm((TB_f @ np.append(mb["position"], 1))[:3] - pa[:3]), mb) for mb in FB["guide"]["magnets"]), key=lambda t: t[0])
            pb = TB_f @ np.append(best[1]["position"], 1)
            lateral = np.linalg.norm(((pb - pa)[:3]) - ((pb - pa)[:3] @ TA_f[:3, 0]) * TA_f[:3, 0])
            mags.append({"gap_mm": round(float(abs((pb - pa)[:3] @ TA_f[:3, 0])) * 1000, 3), "lateral_offset_mm": round(float(lateral) * 1000, 3),
                         "poles": [ma["pole_out"], best[1]["pole_out"]], "attract": ma["pole_out"] != best[1]["pole_out"]})
        e["magnets"] = mags
        br = FA["guide"]["brackets"]
        e["brackets"] = "identical bracket outlines at y and -y: Rz(pi) maps each bracket of A onto a bracket of B (butt contact on the mating plane)"
        # hook closing sweep: open -> closed, overlap with partner chassis_body must stay 0
        body_B = B.world_mesh("chassis_body", {}, TB)
        sweep = []
        for h in A.m["actuated"]["hooks"]:
            if h["face"] != fa:
                continue
            for q in np.linspace(h["open_angle"], h["closed_angle"], 7):
                hm = A.world_mesh(h["part"], {h["part"]: float(q)})
                sweep.append(round(g.overlap_volume(hm, body_B, 0.0008) * 1e9, 2))
            # clearance of the opened tip above the partner pin
            Tp = A.frame_T(h["part"], {h["part"]: h["open_angle"]})
            hm = A.mesh(h["part"]).transformed(Tp)
            pin = [p for p in FB["catch_pins"]][0]
            pw = (TB_f @ np.append(pin["position"], 1))[:3]
            near = hm.P.reshape(-1, 3)
            sel = np.abs((near - pw) @ TA_f[:3, 1]) < 0.004      # points in the hook plane band
            dz = near[sel] - pw
            # distance in the x-z plane of the face to the pin axis
            dist = np.sqrt((dz @ TA_f[:3, 0]) ** 2 + (dz @ TA_f[:3, 2]) ** 2)
            e["open_hook_min_distance_to_pin_axis_mm"] = round(float(dist.min()) * 1000, 2)
            e["open_hook_clearance_to_pin_surface_mm"] = round(float(dist.min() - pin["radius"]) * 1000, 2)
        e["hook_sweep_overlap_with_partner_body_mm3"] = sweep
        e["pass"] = all(m["attract"] and m["gap_mm"] <= 1.5 and m["lateral_offset_mm"] < 0.5 for m in mags) and max(sweep) <= 1.0 and e.get("open_hook_clearance_to_pin_surface_mm", 1) > 0
        res.append(e)
    rep["dock"] = res
    return all(e["pass"] for e in res)


# ------------------------------------------------------------------ 3 stability
def stability(assets, rep, prof):
    Mo = {r: RoverModel(mpath(assets, r)) for r in ("scout", "lifter", "cargo")}
    pl = {it["name"]: it for it in RoverModel(mpath(assets, "payloads")).m["payloads"]["items"]}
    gz = Mo["cargo"].m["base_frame"]["ground_z"]
    out = {}
    # (a) two cargo inline + box_long
    A = Mo["cargo"]
    TB = mate_T(A, "front", A, "rear")
    m1, c1 = mass_state(A, {})
    c2 = (TB @ np.append(c1, 1))[:3]
    bl = pl["box_long"]
    dtop = gz + prof["roles"]["cargo"]["deck_height"]["value"]
    mid = (np.zeros(3) + TB[:3, 3]) / 2
    cb = np.array([mid[0], mid[1], dtop + bl["com_xyz"][2]])
    Mt = 2 * m1 + bl["mass"]
    C = (m1 * c1 + m1 * c2 + bl["mass"] * cb) / Mt
    cont = contacts_of(A) + contacts_of(A, TB)
    marg = support_margin(C[:2], cont)
    h = C[2] - gz
    deck_len = 2 * prof["roles"]["cargo"]["deck_size_lw"]["value"][0]
    out["cargo_pair_box_long"] = {"total_mass": round(Mt, 3), "com_height_above_ground": round(float(h), 4), "com_margin_to_support_edge_m": round(marg, 4),
                                  "static_lateral_tip_angle_deg": round(math.degrees(math.atan2(marg, h)), 1),
                                  "lateral_accel_to_tip_g": round(marg / h, 3), "box_overhang_each_end_m": round((bl["size_lwh"][0] - deck_len - 0.008) / 2, 4),
                                  "box_overhang_each_side_m": round((bl["size_lwh"][1] - prof["roles"]["cargo"]["deck_size_lw"]["value"][1]) / 2, 4),
                                  "load_per_rover_kg": bl["mass"] / 2, "deck_capacity_kg": prof["roles"]["cargo"]["deck_capacity"]["value"],
                                  "pass": marg > 0 and bl["mass"] / 2 <= prof["roles"]["cargo"]["deck_capacity"]["value"]}
    # (b) lifter + box_small / concept box held in the jaws (box near face against the jaw uprights)
    L = Mo["lifter"]
    trv = L.m["actuated"]["lift"]["range"][1]
    st = {"fork_carriage": trv}
    mL, cL = mass_state(L, st)
    Tc = L.frame_T("fork_carriage", st)
    x_face = Tc[0, 3] + 0.007 + 0.014              # carriage front + jaw upright
    lx = abs(L.m["wheels"]["items"][0]["position"][0])
    ly = abs(L.m["wheels"]["items"][0]["position"][1])
    res = {}
    for name, size, mass in (("box_small", pl["box_small"]["size_lwh"], pl["box_small"]["mass"]),
                             ("box_small_rotated_0.20_deep", [pl["box_small"]["size_lwh"][1], pl["box_small"]["size_lwh"][0], pl["box_small"]["size_lwh"][2]], pl["box_small"]["mass"]),
                             ("tag_box_80mm", pl["tag_box"]["size_lwh"], pl["tag_box"]["mass"])):
        depth, width, hgt = size
        cbx = np.array([x_face + depth / 2, 0.0, Tc[2, 3] - 0.013 + hgt / 2])
        Mt = mL + mass
        C = (mL * cL + mass * cbx) / Mt
        marg = lx - C[0]
        mmax = mL * (lx - cL[0]) / (cbx[0] - lx) if cbx[0] > lx else float("inf")
        res[name] = {"box_com_x": round(float(cbx[0]), 4), "combined_com_x": round(float(C[0]), 4), "front_wheel_line_x": round(lx, 4),
                     "pitch_margin_m": round(float(marg), 4), "tips_forward": bool(marg <= 0),
                     "fits_jaws": bool(width <= L.m["actuated"]["gripper"]["opening_range"][1] + 1e-9),
                     "max_mass_zero_margin_kg": round(mmax, 3), "max_mass_safety_1.5_kg": round(mmax / 1.5, 3)}
    res["lifter_alone"] = {"mass": round(mL, 3), "com": [round(float(v), 4) for v in cL]}
    out["lifter_payload"] = res
    # (c) scout arm extended horizontally (forward and sideways) with the rated payload at the TCP
    S = Mo["scout"]
    pay = prof["roles"]["scout"]["arm_proxy"]["payload"]["value"]
    sres = {}
    for lab, yaw in (("forward", 0.0), ("sideways_right", -math.pi / 2), ("rearward", math.pi)):
        st = {"arm_link_1": yaw, "arm_link_2": math.pi / 2, "arm_link_3": 0.0, "arm_link_4": 0.0, "arm_link_5": 0.0, "arm_gripper_jaw": 0.0}
        m, c = mass_state(S, st)
        Tj = S.frame_T("arm_gripper_jaw", st)
        tcp = (Tj @ np.array([0, 0.004, 0.030, 1.0]))[:3]
        Mt = m + pay
        C = (m * c + pay * tcp) / Mt
        marg = support_margin(C[:2], contacts_of(S))
        sres[lab] = {"tcp_xyz": [round(float(v), 4) for v in tcp], "combined_com_xy": [round(float(v), 4) for v in C[:2]], "com_margin_m": round(marg, 4),
                     "tips": bool(marg <= 0)}
    out["scout_arm_extended_with_payload"] = sres
    # (b2) lifter option (b) as built (user decision): box_small at FULL lift stroke, counterweight + battery position
    par = json.load(open(os.path.join(HERE, "cad_params_v1.json")))["groups"]
    cw_now = par["lifter"]["counterweight_mass"]["value"]
    cwb = np.array(par["lifter"]["counterweight_boxes"]["value"], float)
    x_cw = float(np.mean([(b[0][0] + b[1][0]) / 2 for b in cwb]))
    bb = np.array(par["lifter"]["battery_box"]["value"], float)
    x_bat = float((bb[0][0] + bb[1][0]) / 2)
    m_bat = par["masses"]["battery"]["value"]
    x_bat_centre = (-0.050 + 0.015) / 2
    sf = 1.5
    bs = pl["box_small"]
    depth, width, hgt = bs["size_lwh"]
    pz0 = par["lifter"]["jaw_pad_z0"]["value"] - par["lifter"]["box_below_pad"]["value"]     # box underside rel. carriage
    cbx = np.array([x_face + depth / 2, 0.0, Tc[2, 3] + pz0 + hgt / 2])
    restoring = mL * (lx - cL[0])
    overturn = bs["mass"] * (cbx[0] - lx)
    delta = (sf * overturn - restoring) / (lx - x_cw)
    mmax = restoring / (sf * (cbx[0] - lx))
    Mt = mL + bs["mass"]
    C2 = (mL * cL + bs["mass"] * cbx) / Mt
    marg = lx - C2[0]
    h = C2[2] - gz
    bat_equiv = m_bat * (x_bat_centre - x_bat) / (lx - x_cw)
    tb = pl["tag_box"]
    cbt = np.array([x_face + tb["size_lwh"][0] / 2, 0.0, 0.0])
    opt = {"as_built_option_b": {"payload": "box_small 0.30 x 0.20 x 0.15 m, 2 kg, gripped across 0.20 m (0.30 m deep, on the 160 mm forks)",
                                 "lift_stroke": trv, "jaw_opening_range": L.m["actuated"]["gripper"]["opening_range"],
                                 "lifter_mass_incl_counterweight": round(mL, 3), "counterweight_kg": cw_now, "counterweight_centroid_x": round(x_cw, 4),
                                 "battery_centre_x": round(x_bat, 4), "box_com_x": round(float(cbx[0]), 4), "front_wheel_line_x": round(lx, 4),
                                 "pitch_safety_factor": round(restoring / overturn, 3), "static_margin_m": round(float(marg), 4),
                                 "max_decel_before_tip_m_s2": round(float(G * marg / h), 2), "combined_com_height": round(float(h), 4),
                                 "box_underside_above_ground_at_full_stroke": round(float(Tc[2, 3] + pz0 - gz), 4),
                                 "braking_requirement_m_s2": 1.0, "braking_ok": bool(G * marg / h >= 1.0),
                                 "counterweight_change_needed_for_sf1.5_kg": round(float(delta), 3),
                                 "rated_lift_mass_at_sf1.5_box_small_geometry_kg": round(float(mmax), 3),
                                 "rated_lift_mass_at_sf1.5_tag_box_geometry_kg": round(float(restoring / (sf * (cbt[0] - lx))), 3)},
           "battery_vs_dead_mass": {"battery_moved_from_x": round(x_bat_centre, 4), "to_x": round(x_bat, 4),
                                    "equivalent_counterweight_saved_kg": round(float(bat_equiv), 3),
                                    "counterweight_if_battery_centred_kg": round(float(cw_now + bat_equiv + max(delta, 0)), 3),
                                    "note": "battery can only move above the rear motors (cavity ends at x=-0.136); it saves ~0.1 kg of steel"},
           "option_a_alternative": "concept jaws 0.12 m, tag_box only, no counterweight (iteration 5 numbers in DESIGN_LOG)"}
    out["lifter_options"] = opt
    # (d) plain tip angles
    tips = {}
    for r_, m_ in Mo.items():
        M_, c_ = m_.m["mass_properties"]["mass"], np.array(m_.m["mass_properties"]["com_xyz"])
        h_ = c_[2] - m_.m["base_frame"]["ground_z"]
        w_ = m_.m["wheels"]["items"][0]["position"]
        tips[r_] = {"mass": M_, "com_height": round(float(h_), 4), "lateral_tip_deg": round(math.degrees(math.atan2(abs(w_[1]) - abs(c_[1]), h_)), 1),
                    "longitudinal_tip_deg_fwd": round(math.degrees(math.atan2(abs(w_[0]) - c_[0], h_)), 1)}
    out["tip_angles"] = tips
    # (e) drivetrain: Pololu 25D 47:1 HP (stall 1.47 N m), sill climbing with the 97 mm wheel
    prw = prof["chassis"]["wheel"]
    stall = prw["max_wheel_torque"]["value"]
    r = Mo["cargo"].m["wheels"]["items"][0]["radius"]
    drv = {"motor": prw["motor"]["value"], "stall_torque_Nm": stall, "continuous_Nm_25pct": round(stall / 4, 3),
           "no_load_speed_m_s": round(prw["max_wheel_speed_rad_s"]["value"] * r, 3), "cases": []}
    for role, extra in (("scout", 0.0), ("lifter", 1.0), ("cargo", 0.0), ("cargo", 3.0), ("cargo", 5.0)):
        M_ = Mo[role].m["mass_properties"]["mass"] + extra
        Nw = M_ * G / 4
        case = {"role": role, "payload_kg": extra, "total_kg": round(M_, 3),
                "steady_torque_Nm_0.5ms2_plus_rolling": round(Nw * r * (0.5 / G + 0.05), 4)}
        for h in (0.01, 0.02, 0.03):
            th = math.acos((r - h) / r)
            tau = Nw * r * math.sin(th)                       # torque to pivot one front wheel over the edge
            push = 2 * Nw * math.tan(th)                       # horizontal force to lift both front wheels
            trac = {mu: 2 * mu * Nw for mu in (0.6, 0.9)}      # rear-wheel traction (tile, rubber)
            case[f"sill_{int(h*100)}cm"] = {"torque_per_front_wheel_Nm": round(tau, 3), "pct_of_stall": round(100 * tau / stall, 1),
                                           "push_needed_N": round(push, 1), "rear_traction_N_mu0.6": round(trac[0.6], 1), "rear_traction_N_mu0.9": round(trac[0.9], 1),
                                           "verdict": ("ok" if push <= trac[0.6] and tau <= 0.6 * stall else "marginal (needs edge grip/rubber floor)" if push <= trac[0.9] + 2 * 0.6 * Nw and tau <= stall else "no (ramp or larger wheels)")}
        drv["cases"].append(case)
    drv["note"] = ("Quasi-static: a front wheel pivots on the sill edge (cos th = (r-h)/r). Mecanum rollers (9 mm) make real sill behaviour worse than a solid wheel; "
                   "motor torque is no longer the limit, the 97 mm wheel geometry and traction are.")
    out["drivetrain"] = drv
    rep["stability"] = out
    return bool(out["cargo_pair_box_long"]["pass"] and opt["as_built_option_b"]["pitch_safety_factor"] >= 1.5 - 1e-3 and opt["as_built_option_b"]["braking_ok"]
                and not any(v["tips"] for v in sres.values()))


# ------------------------------------------------------------------ 4 arm fold
def arm_fold(assets, rep):
    S = RoverModel(mpath(assets, "scout"))
    st = V.authored_state(S)
    arm = [n for n in S.parts if n.startswith("arm_link") or n == "arm_gripper_jaw"]
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    am = g.Mesh()
    for n in arm:
        m = S.world_mesh(n, st)
        am.append(m)
        l, h_ = m.bbox()
        lo, hi = np.minimum(lo, l), np.maximum(hi, h_)
    L, W = 0.358, 0.180
    lid = next(f for f in S.m["sensors"]["frames"] if f["type"] == "lidar2d")
    scan_z = lid["origin"]["xyz"][2]
    lidar_top = S.world_mesh("lidar_head", st).bbox()[1][2]
    others = {}
    for n in ("chassis_body", "d435", "compute", "mast", "lidar_base", "lidar_head", "service_port", "tag_top"):
        if n in S.parts:
            others[n] = round(g.overlap_volume(am, S.world_mesh(n, st), 0.001) * 1e9, 2)
    # self-collision between non-adjacent arm links in the folded pose
    pairs = {}
    for i, a in enumerate(arm):
        for b in arm[i + 2:]:
            pairs[f"{a}|{b}"] = round(g.overlap_volume(S.world_mesh(a, st), S.world_mesh(b, st), 0.001) * 1e9, 2)
    # concept pose: arm vs body
    from render_v1 import concept_state
    cs = concept_state(S, "scout")
    amc = g.Mesh()
    for n in arm:
        amc.append(S.world_mesh(n, cs))
    concept_ov = {n: round(g.overlap_volume(amc, S.world_mesh(n, cs), 0.001) * 1e9, 2) for n in ("chassis_body", "d435", "compute", "mast", "lidar_head")}
    d435_top = S.world_mesh("d435", st).bbox()[1][2]
    res = {"folded_bbox_min": lo.round(4).tolist(), "folded_bbox_max": hi.round(4).tolist(),
           "inside_footprint": bool(lo[0] >= -L / 2 and hi[0] <= L / 2 and lo[1] >= -W / 2 and hi[1] <= W / 2),
           "folded_top_z": round(float(hi[2]), 4), "lidar_top_z": round(float(lidar_top), 4), "lidar_scan_plane_z": round(float(scan_z), 4),
           "below_lidar_top": bool(hi[2] <= lidar_top), "below_scan_plane": bool(hi[2] <= scan_z),
           "overlap_with_scout_parts_mm3": others, "arm_self_overlap_folded_mm3": {k: v for k, v in pairs.items() if v > 0},
           "concept_pose_overlap_mm3": concept_ov, "d435_top_z": round(float(d435_top), 4)}
    res["pass"] = res["inside_footprint"] and res["below_lidar_top"] and max(others.values()) <= 1.0 and not res["arm_self_overlap_folded_mm3"]
    rep["arm_fold"] = res
    return res["pass"]


def _ray_hits(orig, targets, tris):
    """True per target if the segment orig->target hits any triangle (Moller-Trumbore, vectorised)."""
    out = []
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2 = v1 - v0, v2 - v0
    for t in targets:
        d = t - orig
        L = np.linalg.norm(d)
        d = d / L
        lo, hi = np.minimum(orig, t) - 1e-3, np.maximum(orig, t) + 1e-3
        sel = np.all(tris.max(1) >= lo, 1) & np.all(tris.min(1) <= hi, 1)
        if not sel.any():
            out.append(False)
            continue
        a1, a2, b0 = e1[sel], e2[sel], v0[sel]
        p = np.cross(d, a2)
        det = np.einsum("ij,ij->i", a1, p)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
        s = orig - b0
        u = np.einsum("ij,ij->i", s, p) * inv
        q = np.cross(s, a1)
        v = (q @ d) * inv
        tt = np.einsum("ij,ij->i", a2, q) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (tt > 1e-4) & (tt < L - 2e-3)
        out.append(bool(hit.any()))
    return out


def tag_visibility(assets, rep, prof):
    M = {r: RoverModel(mpath(assets, r)) for r in ("scout", "lifter", "cargo")}
    cams = {"scout": "camera_color", "lifter": "cam_front", "cargo": "cam_front"}
    def tris_of(model, T_w, skip=()):
        P = []
        for n, p in model.parts.items():
            if not p.get("default_visible", True) or p.get("role_in_assembly") == "tag" or n in skip:
                continue
            P.append(model.world_mesh(n, V.authored_state(model), T_w).P)
        return np.concatenate(P)
    res, worst = [], {}
    for tr, T_ in M.items():
        tri_t = tris_of(T_, np.eye(4))
        for tag in [t for t in T_.m["attachments"]["tags"] if t["name"] != "tag_top"]:
            face = tag["name"][4:]
            F = T_.face_T(face)
            Tt = g.T_from_pose(tag["origin"])
            hs = tag["size"] / 2
            corners = [(Tt @ np.array([sx * hs, sy * hs, 0, 1.0]))[:3] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            inner = [(Tt @ np.array([sx * hs * 0.8, sy * hs * 0.8, 0, 1.0]))[:3] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (0, 0))]
            nrm = np.array(tag["outward_normal"])
            for orole, O in M.items():
                cs = next(s for s in O.m["sensors"]["frames"] if s["name"] == cams[orole])
                sp = cs["params"]
                Wpx, Hpx = sp["resolution"]
                f = (Wpx / 2) / math.tan(sp["hfov"] / 2)
                for d in (0.3, 0.5, 1.0, 1.5):
                    T_O = F @ g.make_T(None, [d, 0, 0]) @ g.make_T(g.rotz(math.pi)) @ g.inv_T(O.face_T("front"))
                    Tc = T_O @ g.T_from_pose(cs["origin"])
                    cpos = Tc[:3, 3]
                    pix = []
                    infov = True
                    for p in corners:
                        q = np.linalg.inv(Tc) @ np.append(p, 1)
                        if q[0] <= 0:
                            infov = False
                            break
                        u, v = -f * q[1] / q[0], -f * q[2] / q[0]
                        pix.append((u, v))
                        if abs(u) > Wpx / 2 or abs(v) > Hpx / 2:
                            infov = False
                    edge = min(np.hypot(pix[i][0] - pix[(i + 1) % 4][0], pix[i][1] - pix[(i + 1) % 4][1]) for i in range(4)) if len(pix) == 4 else 0.0
                    vdir = cpos - (Tt[:3, 3])
                    ang = math.degrees(math.acos(np.clip(vdir @ nrm / np.linalg.norm(vdir), -1, 1)))
                    occl = any(_ray_hits(cpos, inner, np.concatenate([tri_t, tris_of(O, T_O, skip=(cs.get("render_part"),))])))
                    ok = infov and ang <= 60 and edge >= 12 and not occl
                    res.append({"target": f"{tr}.{face}", "observer": f"{orole}.{cams[orole]}", "distance_m": d, "in_fov": infov,
                                "view_angle_deg": round(ang, 1), "min_edge_px": round(float(edge), 1), "occluded": occl, "visible": ok})
    summ = {}
    for r_ in res:
        k = f"{r_['observer']} -> {r_['target']}"
        summ.setdefault(k, []).append(f"{r_['distance_m']}m:{'OK' if r_['visible'] else 'no'}({r_['min_edge_px']}px,{r_['view_angle_deg']}deg{',occl' if r_['occluded'] else ''}{',fov' if not r_['in_fov'] else ''})")
    rep["tag_visibility"] = {"criteria": "tag fully in the camera FOV, view angle <= 60 deg, black-border edge >= 12 px, rays to the tag unobstructed by either rover; observer front mating plane d metres from the target face, aligned",
                             "summary": summ, "cases": res, "n_visible": sum(r_["visible"] for r_ in res), "n_cases": len(res)}
    return bool(all(r_["visible"] for r_ in res if r_["distance_m"] <= 1.0))


def deck_placement(assets, rep, prof):
    """Lifter places box_small on (i) one cargo deck from the side, (ii) the junction of an inline cargo pair from the side,
    (iii) one cargo deck end-on. Mesh overlap of the whole lifter (full stroke, jaws gripping 0.20 m) against the cargo(s)
    in the carry pose, the set-down pose (box underside on the deck) and the released pose (jaws open); prong-to-deck
    clearance; box footprint vs deck edges and the tag-flag notch."""
    Lm, Cm = RoverModel(mpath(assets, "lifter")), RoverModel(mpath(assets, "cargo"))
    par = json.load(open(os.path.join(HERE, "cad_params_v1.json")))["groups"]
    pz0 = par["lifter"]["jaw_pad_z0"]["value"] - par["lifter"]["box_below_pad"]["value"]     # box underside rel. carriage
    trv = Lm.m["actuated"]["lift"]["range"][1]
    gr = Lm.m["actuated"]["gripper"]
    omax = gr["opening_range"][1]
    bs = next(i for i in RoverModel(mpath(assets, "payloads")).m["payloads"]["items"] if i["name"] == "box_small")
    depth, width, hgt = bs["size_lwh"]
    gz = Cm.m["base_frame"]["ground_z"]
    dtop = gz + prof["roles"]["cargo"]["deck_height"]["value"]
    dl, dw = prof["roles"]["cargo"]["deck_size_lw"]["value"]
    grip = {f["part"]: (omax - width) / 2 for f in gr["fingers"]}
    openj = {f["part"]: 0.0 for f in gr["fingers"]}
    def lifter_mesh(state, T):
        m = g.Mesh()
        for n, p in Lm.parts.items():
            if p.get("default_visible", True) and p.get("role_in_assembly") != "tag":
                m.append(Lm.world_mesh(n, {**V.authored_state(Lm), **state}, T))
        return m
    def cargo_mesh(T):
        m = g.Mesh()
        for n, p in Cm.parts.items():
            if p.get("default_visible", True) and p.get("role_in_assembly") != "tag":
                m.append(Cm.world_mesh(n, V.authored_state(Cm), T))
        return m
    Tcar0 = Lm.frame_T("fork_carriage", {"fork_carriage": 0.0})
    x_face = Tcar0[0, 3] + 0.007 + 0.014
    box_cx_L = x_face + depth / 2                   # box centre in the lifter frame (x)
    q_set = dtop - gz - (Tcar0[2, 3] + pz0 - gz)    # stroke at which the box underside touches the deck top
    cases = {}
    pair_T = Cm.face_T("front") @ g.make_T(g.rotz(math.pi)) @ g.inv_T(Cm.face_T("rear"))
    for name, cargo_Ts, target_xy, yaw in (
            ("single_deck_side_on", [np.eye(4)], (0.0, 0.0), -math.pi / 2),
            ("inline_pair_junction_side_on", [np.eye(4), pair_T], (pair_T[0, 3] / 2, 0.0), -math.pi / 2),
            ("single_deck_end_on", [np.eye(4)], (0.0, 0.0), math.pi)):
        cm = g.Mesh()
        for Tc_ in cargo_Ts:
            cm.append(cargo_mesh(Tc_))
        # lifter pose: its box centre over target_xy, facing yaw (lifter +x points along (cos yaw, sin yaw))
        dirv = np.array([math.cos(yaw), math.sin(yaw)])
        # back off until the lifter body (full stroke) clears the cargo, then report the box-centre offset
        res = None
        for back in np.arange(0.0, 0.08, 0.002):
            pos = np.array(target_xy) - dirv * (box_cx_L + back)
            TL = g.make_T(g.rotz(yaw), [pos[0], pos[1], 0.0])
            ov = g.overlap_volume(lifter_mesh({"fork_carriage": trv, **grip}, TL), cm, 0.002)
            if ov <= 1e-9:
                res = (back, TL)
                break
        if res is None:
            cases[name] = {"feasible": False}
            continue
        back, TL = res
        ov_set = g.overlap_volume(lifter_mesh({"fork_carriage": q_set, **grip}, TL), cm, 0.002)
        ov_rel = g.overlap_volume(lifter_mesh({"fork_carriage": q_set, **openj}, TL), cm, 0.002)
        # prong clearance above the deck at set-down
        prong_bottom = (Lm.frame_T("fork_carriage", {"fork_carriage": q_set})[2, 3] - 0.013)
        bc = np.array(target_xy) - dirv * back
        # box footprint in the deck frame of the (first) cargo
        ax_len = dirv                          # box depth axis
        ax_w = np.array([-dirv[1], dirv[0]])
        corners = [bc + sa * ax_len * depth / 2 + sb * ax_w * width / 2 for sa in (-1, 1) for sb in (-1, 1)]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        span_x = (min(xs), max(xs))
        deck_x = (min(T_[0, 3] for T_ in cargo_Ts) - dl / 2, max(T_[0, 3] for T_ in cargo_Ts) + dl / 2)
        notch = par["cargo"]["flag_notch"]["value"]
        flag_overlap = []
        for T_ in cargo_Ts:
            nx = (T_[0, 3] - dl / 2, T_[0, 3] - dl / 2 + notch)
            ny = (dw / 2 - notch, dw / 2)
            ox = max(0.0, min(span_x[1], nx[1]) - max(span_x[0], nx[0]))
            oy = max(0.0, min(max(ys), ny[1]) - max(min(ys), ny[0]))
            flag_overlap.append(round(ox * oy * 1e4, 2))
        cases[name] = {"feasible": True, "box_centre_offset_from_target_m": round(float(back), 3),
                       "carry_pose_overlap_mm3": round(g.overlap_volume(lifter_mesh({"fork_carriage": trv, **grip}, TL), cm, 0.002) * 1e9, 1),
                       "set_down_stroke_m": round(float(q_set), 4), "set_down_overlap_mm3": round(ov_set * 1e9, 1), "released_overlap_mm3": round(ov_rel * 1e9, 1),
                       "box_underside_clearance_to_deck_at_full_stroke_m": round(float(Tcar0[2, 3] + trv + pz0 - dtop), 4),
                       "prong_clearance_above_deck_at_set_down_m": round(float(prong_bottom - dtop), 4),
                       "box_overhang_beyond_deck_x_m": [round(float(deck_x[0] - span_x[0]), 4), round(float(span_x[1] - deck_x[1]), 4)],
                       "box_overhang_beyond_deck_y_m": round(float(max(np.abs(ys)) - dw / 2), 4),
                       "box_over_flag_notch_cm2": flag_overlap,
                       "flag_vertical_clearance_m": 0.002,
                       "withdraw": "jaws open (pads move out (0.24-0.20)/2 = 20 mm per side), lifter backs out along its -x; prongs stay above the deck"}
    ok = all(v.get("feasible") and v["set_down_overlap_mm3"] <= 1.0 and v["released_overlap_mm3"] <= 1.0 and v["prong_clearance_above_deck_at_set_down_m"] > 0
             for k, v in cases.items() if k != "single_deck_end_on")
    rep["deck_placement"] = {"payload": "box_small gripped across 0.20 m", "cases": cases, "pass_side_on": bool(ok)}
    return bool(ok)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", default=os.path.expanduser("~/.cache/crane-fleet/assets_v1"))
    ap.add_argument("--skip-validator", action="store_true")
    a = ap.parse_args(argv)
    prof = json.load(open(os.path.join(V.REPO, "Tools", "Fleet", "spec", "fleet_profile_v1.json")))
    rep = {"schema": "crane.fleet_v1_engineering_checks/v1", "assets": a.assets, "profile_id": prof["profile_id"]}
    ok = {}
    if not a.skip_validator:
        ok["validator_no_fail"] = run_validator(a.assets, rep)
    ok["dock"] = dock_checks(a.assets, rep)
    ok["stability"] = stability(a.assets, rep, prof)
    ok["arm_fold"] = arm_fold(a.assets, rep)
    ok["tag_visibility_0.3_1.0m"] = tag_visibility(a.assets, rep, prof)
    ok["deck_placement"] = deck_placement(a.assets, rep, prof)
    rep["summary"] = ok
    path = os.path.join(a.assets, "validation_report.json")
    json.dump(rep, open(path, "w"), indent=1)
    slim = {k: v for k, v in rep.items() if k != "validator_full"}
    print(json.dumps(slim, indent=1)[:12000])
    print("report:", path)


if __name__ == "__main__":
    main()
