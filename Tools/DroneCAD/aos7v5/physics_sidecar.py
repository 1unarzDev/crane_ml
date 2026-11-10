"""Derives the multirotor sidecar (aos7v5_multirotor.yaml) and a proposed CRANE profile patch from CAD mass properties
plus the T-Motor F90 KV1300 / HQ7040 6S thrust table already fitted in Tools/Aerial/fit_aos7_profile.py."""
import json, math, sys
from pathlib import Path
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "Tools" / "Aerial"))
import fit_aos7_profile as fit  # noqa: E402  (reuses the datasheet table and the LSQ fits; no copy of the numbers)
import params as P  # noqa: E402

G = 9.81


def clean(o):
    if isinstance(o, dict):
        return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return clean(o.tolist())
    return o


def derive(props, report):
    rot = fit.fit_rotor()
    m = report["mass_kg"]
    com = np.array(report["com_m"])
    I = np.array(report["inertia_com"])
    kf, km, kq, wmax = rot["kf"], rot["km"], rot["kq"], rot["omega_max"]
    w_h = math.sqrt(m * G / (4 * kf))
    j_rot = props["rotor_0"]["I"][2, 2]          # kg m^2 about the spin axis, CAD (bell + 3 blades + hub + nut)
    kt = 60.0 / (2 * math.pi * P.MOTOR["kv"])
    tau = j_rot / (kt * kt / P.MOTOR["resistance_ohm"] + 2 * kq * w_h)
    drag_per_m = fit.DRAG_PER_MASS
    lam1 = drag_per_m * m / (4 * w_h)
    area = 0.5 * (report["frontal_area_x_m2"] + report["frontal_area_y_m2"])
    cdq = 0.5 * P.AIR["rho"] * P.AIR["cd_body"] * area
    rows = np.array(fit.F90_KV1300_HQ7040)
    hover_g = m * 1000 / 4 * G / fit.G
    thr_h = float(np.interp(hover_g, rows[:, 1], rows[:, 0], left=np.nan))
    if np.isnan(thr_h):
        thr_h = float(rows[0, 0] + (hover_g - rows[0, 1]) * (rows[1, 0] - rows[0, 0]) / (rows[1, 1] - rows[0, 1]))
    rotors = []
    for i in range(4):
        a = np.array(props[f"rotor_{i}"]["anchor_m"])
        rotors.append(dict(index=i, px4_motor=i + 1, crane_motor=f"M{i+1}", name=P.ROTOR_NAMES[i], link=f"rotor_{i}",
                           spin=P.SPIN[i], rotation="CCW seen from above (+z)" if P.SPIN[i] > 0 else "CW seen from above",
                           position_base_link_m=[round(float(x), 6) for x in a],
                           position_rel_com_m=[round(float(x), 6) for x in (a - com)]))
    return dict(rot=rot, m=m, com=com, I=I, w_h=w_h, j_rot=j_rot, tau=tau, lam1=lam1, cdq=cdq, area=area,
                thr_h=thr_h, rotors=rotors, kt=kt)


def write(path, links, props, report):
    anchors = {l.name: [x * 1e-3 for x in l.anchor] for l in links}
    for n in props:
        props[n]["anchor_m"] = anchors[n]
    d = derive(props, report)
    rot = d["rot"]
    def e(v, unit, status, source, note=""):
        return dict(value=v, unit=unit, status=status, source=source, **({"note": note} if note else {}))
    old = json.loads((REPO / "Tools" / "Aerial" / "profiles" / "aos7_v5_f90_6s.json").read_text())
    doc = {
        "schema": "crane-aos7v5-multirotor-sidecar-v1",
        "urdf": "aos7v5.urdf",
        "evidence_label": "IMPLEMENTED (CAD-derived mass properties; thrust data from the T-Motor F90 table; NOT PHYSICALLY_CALIBRATED)",
        "frames": {"body": "FLU (REP-103), origin = base_link = frame centre", "spin": "+1 = CCW seen from above (+z)",
                   "com_offset_from_base_link_m": [round(float(x), 6) for x in d["com"]]},
        "mapping": {"px4_quad_x": {"motor1": "rotor_0 front-right CCW", "motor2": "rotor_1 rear-left CCW",
                                   "motor3": "rotor_2 front-left CW", "motor4": "rotor_3 rear-right CW"},
                    "crane_profile_order": "FR, RL, FL, RR = M1..M4 = rotor_0..rotor_3 (identity mapping)"},
        "rotors": d["rotors"],
        "mass": e(round(d["m"], 5), "kg", "DERIVED", "CAD sum of per-link masses (SPEC.md table)"),
        "inertia_com": e([[float(f"{x:.6e}") for x in row] for row in d["I"]], "kg m^2", "DERIVED",
                         "CAD B-rep mass properties about the CoM, FLU axes, includes motors/props"),
        "thrust_coefficient_kf": e(rot["kf"], "N/(rad/s)^2", "MEASURED_DATASHEET", "T-Motor F90 KV1300 + HQ7040 table, 6S ~24 V; relative LSQ T=kf w^2 (max resid 8.3%)"),
        "torque_coefficient_kq": e(rot["kq"], "N m/(rad/s)^2", "ESTIMATED", "shaft Q = eta P/w with eta = 0.8 assumed (same method as the CRANE profile)"),
        "moment_constant_km": e(rot["km"], "m", "ESTIMATED", "kq/kf; reaction torque = -spin * km * T"),
        "omega_max": e(rot["omega_max"], "rad/s", "MEASURED_DATASHEET", "19617 rpm at 100% throttle, 23.48 V"),
        "thrust_max_per_motor": e(rot["thrust_max_n"], "N", "MEASURED_DATASHEET", "2360.6 gf table row at 100% throttle (kf omega_max^2)"),
        "motor_time_constant": e(round(d["tau"], 5), "s", "ESTIMATED", "J/(Kt^2/R + 2 kq w_h) with J = CAD rotor Izz and w_h for the CAD mass",
                                 "tau_up = tau_down; PX4 gz x500 default is 12.5/25 ms"),
        "rotor_inertia": e(float(f"{d['j_rot']:.5e}"), "kg m^2", "DERIVED", "CAD Izz of rotor_i (bell + prop); prop mass 9 g is an ESTIMATE"),
        "rotor_radius": e(P.PROP["diameter"] / 2000, "m", "MEASURED_DATASHEET", "7 in prop"),
        "rotor_drag_lambda1": e(d["lam1"], "N s/(m rad)", "ESTIMATED", "0.35 1/s x mass / (4 w_hover); Faessler 2018 order of magnitude"),
        "rolling_moment": e(0.0, "N s/rad", "ESTIMATED", "neglected"),
        "drag_quadratic": e(d["cdq"], "N s^2/m^2", "ESTIMATED", f"0.5 rho Cd A, Cd = {P.AIR['cd_body']} (ESTIMATE), A = {d['area']:.4f} m^2 mean of CAD frontal/side silhouette (props excluded)"),
        "drag_linear": e([0.0, 0.0, 0.0], "N s/m", "ESTIMATED", "carried by rotor drag"),
        "battery": {"capacity_ah": e(P.BATTERY["capacity_ah"], "Ah", "DATASHEET", "Tattu 6S 3500 mAh (user)"),
                    "v_full": e(25.2, "V", "LITERATURE", "6 x 4.20 V"), "v_empty": e(21.0, "V", "ESTIMATED", "6 x 3.50 V"),
                    "v_nominal_for_omega_max": e(23.48, "V", "MEASURED_DATASHEET", "bench voltage of the 100% throttle row"),
                    "resistance": e(P.BATTERY["resistance_ohm"], "ohm", "ESTIMATED", "6 x ~8 mOhm 25C-class cells + leads"),
                    "efficiency": e(0.8, "-", "ESTIMATED", "as CRANE profile"),
                    "note": "4 motors at 100% draw 4 x 45.1 A = 180 A (datasheet) against a 25C x 3.5 Ah = 87 A rating: sag limits real peak thrust"},
        "derived": {"hover_rotor_speed_rad_s": d["w_h"], "hover_rpm": d["w_h"] * 30 / math.pi,
                    "thrust_to_weight": 4 * rot["thrust_max_n"] / (d["m"] * G), "hover_thrust_per_rotor_n": d["m"] * G / 4,
                    "hover_throttle_datasheet": d["thr_h"], "hover_thrust_fraction_model": (d["w_h"] / rot["omega_max"]) ** 2},
        "reconciliation_with_crane_aos7_profile": {
            "profile": "Tools/Aerial/profiles/aos7_v5_f90_6s.json (id aos7-v5-f90-1300kv-6s)",
            "same": ["kf", "km", "omega_max", "rotor_radius", "motor kv/R", "battery_v_nominal", "spin order FR RL FL RR"],
            "changed": {"mass_kg": [old["parameters"]["mass"]["value"], round(d["m"], 4)],
                        "rotor_xy": "old: true-X square 0.318 m wheelbase (+-0.11243 m); CAD: 254 x 184 mm stretched X (+-0.127, +-0.092 m, DATASHEET)",
                        "battery_capacity_ah": [old["parameters"]["battery_capacity_ah"]["value"], P.BATTERY["capacity_ah"]]}},
    }
    Path(path).write_text("# AOS 7 V5 multirotor physics sidecar. Generated by Tools/DroneCAD/aos7v5/physics_sidecar.py; do not edit by hand.\n"
                          + yaml.safe_dump(clean(doc), sort_keys=False, width=140))
    # proposed CRANE profile (NOT installed): same schema as the existing JSON with CAD mass properties
    new = json.loads(json.dumps(old))
    pr = new["parameters"]
    def setp(k, v, status, loc):
        pr[k] = dict(value=v, unit=pr[k].get("unit", ""), status=status, source=pr[k].get("source"), locator=loc)
    setp("mass", round(d["m"], 5), "ESTIMATED", "Tools/DroneCAD/aos7v5 CAD sum (SPEC.md); frame 325 g is DATASHEET, most others ESTIMATE")
    setp("inertia", d["I"].round(9).tolist(), "ESTIMATED", "CAD B-rep inertia about the CoM")
    setp("tau_up", round(d["tau"], 5), "ESTIMATED", "recomputed with CAD rotor inertia and CAD-mass hover speed")
    setp("tau_down", round(d["tau"], 5), "ESTIMATED", "same")
    setp("rotor_inertia", float(f"{d['j_rot']:.5e}"), "ESTIMATED", "CAD rotor Izz")
    setp("rotor_drag", d["lam1"], "ESTIMATED", "0.35 1/s x CAD mass / (4 w_hover)")
    setp("drag_quadratic", d["cdq"], "ESTIMATED", "CAD frontal area")
    setp("battery_capacity_ah", P.BATTERY["capacity_ah"], "DATASHEET", "Tattu 6S 3500 mAh")
    setp("battery_resistance", P.BATTERY["resistance_ohm"], "ESTIMATED", "25C-class pack")
    new["rotors"] = [dict(name=r["name"], position=r["position_rel_com_m"], spin=r["spin"], source="CAD",
                          locator="aos7v5.urdf rotor link origin minus CAD CoM") for r in d["rotors"]]
    new["derived"]["com_offset_from_frame_center_flu"] = [round(float(x), 6) for x in d["com"]]
    new["id"] = "aos7-v5-f90-1300kv-6s-cad"
    new["description"] = "PROPOSED (not installed): AOS 7 V5 profile with CAD mass properties. " + old["description"]
    new["massBudget"] = [dict(item=n, mass_kg=round(p["mass"], 5), status="see SPEC.md", position_flu=[round(float(x), 5) for x in p["com"]])
                         for n, p in props.items()]
    new["derived"]["thrust_to_weight"] = 4 * rot["thrust_max_n"] / (d["m"] * G)
    new["derived"]["hover_rotor_speed"] = d["w_h"]
    (HERE / "proposed_profile_aos7_cad.json").write_text(json.dumps(new, indent=1))
