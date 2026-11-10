#!/usr/bin/env python3
"""Verification of the AOS 7 V5 model: URDF vs CAD mass properties, inertia sanity, rotor geometry vs sidecar, meshes,
RK4 plant (Tools/Aerial) hover / step / attitude / saturation with these properties vs the CRANE 1.383 kg profile,
and an optional pybullet drop test.   Run:  ~/.cache/research/cad-venv/bin/python test_model.py [--urdf-dir DIR]
No Unity.  Exit code 1 on any failed check.  Writes test_results.json next to this file."""
import argparse, json, math, os, sys, tempfile, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh, yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "Tools" / "Aerial"))
import aerial_model as am  # noqa: E402
import aerial_allocation as alloc  # noqa: E402
from aerial_profiles import load_vehicle  # noqa: E402

RES = {"checks": [], "sim": {}}
FAILS = []


def check(name, ok, detail=""):
    RES["checks"].append(dict(name=name, ok=bool(ok), detail=detail))
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILS.append(name)


def vec(s):
    return np.array([float(x) for x in s.split()])


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        inr = l.find("inertial")
        d = dict(mass=None, com=None, I=None, visuals=[l_.find("geometry/mesh").get("filename") for l_ in l.findall("visual")],
                 collisions=l.findall("collision"))
        if inr is not None:
            d["mass"] = float(inr.find("mass").get("value"))
            d["com"] = vec(inr.find("origin").get("xyz"))
            i = inr.find("inertia")
            g = lambda k: float(i.get(k))
            d["I"] = np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]])
        links[l.get("name")] = d
    for j in root.findall("joint"):
        joints[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                                      xyz=vec(j.find("origin").get("xyz")), rpy=vec(j.find("origin").get("rpy")),
                                      axis=vec(j.find("axis").get("xyz")) if j.find("axis") is not None else None)
    return root, links, joints


def world_pos(name, joints, cache):
    if name in cache:
        return cache[name]
    p = np.zeros(3) if name not in joints else world_pos(joints[name]["parent"], joints, cache) + joints[name]["xyz"]
    cache[name] = p
    return p


def total_from_urdf(links, joints):
    cache = {}
    items = [(l["mass"], world_pos(n, joints, cache) + l["com"], l["I"]) for n, l in links.items() if l["mass"]]
    m = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / m
    I = np.zeros((3, 3))
    for mi, c, Il in items:
        d = c - com
        I += Il + mi * (d @ d * np.eye(3) - np.outer(d, d))
    return m, com, I, cache


def inertia_ok(I):
    w = np.linalg.eigvalsh(I)
    a, b, c = np.diag(I)
    tri = (a + b >= c * (1 - 1e-9)) and (a + c >= b * (1 - 1e-9)) and (b + c >= a * (1 - 1e-9))
    return bool(w.min() > 0), bool(tri), w


# ------------------------------------------------------------------ plant helpers
def make_vehicle(mass, I, rotors_rel_com, sc, with_battery=True):
    g = lambda k: sc[k]["value"]
    rot = [am.Rotor(position=np.array(r["position_rel_com_m"], float), spin=int(r["spin"]), kf=g("thrust_coefficient_kf"),
                    km=g("moment_constant_km"), omega_max=g("omega_max"), tau_up=g("motor_time_constant"),
                    tau_down=g("motor_time_constant"), rotor_drag=g("rotor_drag_lambda1"), rolling_moment=0.0,
                    radius=g("rotor_radius"), inertia=g("rotor_inertia")) for r in rotors_rel_com]
    b = sc["battery"]
    bat = am.Battery(v_full=b["v_full"]["value"], v_empty=b["v_empty"]["value"], v_nominal=b["v_nominal_for_omega_max"]["value"],
                     resistance=b["resistance"]["value"], capacity_ah=b["capacity_ah"]["value"], efficiency=b["efficiency"]["value"]) if with_battery else None
    return am.Vehicle(name="aos7v5-cad", mass=mass, inertia=I, rotors=rot, drag_linear=np.zeros(3),
                      drag_quadratic=g("drag_quadratic"), battery=bat)


def trim_speeds(v):
    """Rotor speeds that give exactly m g and zero torque (the CoM is not exactly on the rotor centroid)."""
    B = alloc.effectiveness_matrix(v)
    u = np.linalg.solve(B, np.array([v.mass * v.gravity, 0, 0, 0]))
    return np.sqrt(u / np.array([r.kf for r in v.rotors]))


def euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def run_attitude(v, target_roll, wn, zeta=0.8, t_end=1.5, dt=0.002, collective_boost=1.0):
    umax = alloc.max_thrusts(v)
    B = alloc.effectiveness_matrix(v)
    kf = np.array([r.kf for r in v.rotors])
    w_h = trim_speeds(v)
    s = am.initial_state(v, p=(0, 0, 50.0), rotor_speed=w_h)
    log = []
    sat_steps = 0
    n = int(t_end / dt)
    for k in range(n):
        roll, pitch = euler(s.q)
        tau = v.inertia @ np.array([wn ** 2 * (target_roll - roll) - 2 * zeta * wn * s.omega[0],
                                    wn ** 2 * (0 - pitch) - 2 * zeta * wn * s.omega[1], -2 * zeta * wn * s.omega[2]])
        T = v.mass * v.gravity * collective_boost / max(0.5, math.cos(roll) * math.cos(pitch))
        u, ach, exact = alloc.allocate(B, umax, np.array([T, *tau]))
        sat_steps += (not exact)
        s = am.step(v, s, np.sqrt(np.maximum(u, 0) / kf), dt)
        log.append((s.t, roll, s.rotor_speed.max(), s.p[2]))
    log = np.array(log)
    r = log[:, 1]
    final = target_roll
    def t_at(frac):
        idx = np.argmax(r >= frac * final)
        return float(log[idx, 0]) if r.max() >= frac * final else float("nan")
    return dict(t10=t_at(0.1), t90=t_at(0.9), overshoot_pct=float((r.max() - final) / final * 100),
                final_roll_deg=float(math.degrees(r[-1])), peak_rotor_frac=float(log[:, 2].max() / v.rotors[0].omega_max),
                saturated_step_frac=sat_steps / n, altitude_change_m=float(log[-1, 3] - 50.0))


def run_collective_step(v, factor=1.5, t_end=0.6, dt=0.001):
    kf = v.rotors[0].kf
    w_h = trim_speeds(v)
    s = am.initial_state(v, p=(0, 0, 50.0), rotor_speed=w_h)
    cmd = w_h * math.sqrt(factor)
    az = []
    for k in range(int(t_end / dt)):
        s = am.step(v, s, cmd, dt)
        f, _ = am.wrench(v, s)
        az.append((s.t, f[2] / v.mass - v.gravity))
    az = np.array(az)
    a_final = factor * v.gravity - v.gravity
    idx = np.argmax(az[:, 1] >= 0.632 * a_final)
    return dict(az_final=float(az[-1, 1]), az_target=a_final, t63_ms=float(az[idx, 0] * 1e3))


def run_full_throttle(v, t_end=1.0, dt=0.001):
    s = am.initial_state(v, p=(0, 0, 50.0), rotor_speed=trim_speeds(v))
    cmd = np.full(4, v.rotors[0].omega_max)
    out = []
    for k in range(int(t_end / dt)):
        s = am.step(v, s, cmd, dt)
        if k % 100 == 99:
            f, _ = am.wrench(v, s)
            cur = am.electrical_current(v, s)
            out.append((s.t, f[2] / (v.mass * v.gravity), cur, am.battery_voltage(v, s, cur)))
    o = np.array(out)
    return dict(tw_at_1s=float(o[-1, 1]), tw_peak=float(o[:, 1].max()), current_a=float(o[-1, 2]), pack_v=float(o[-1, 3]))


def run_yaw_saturation(v):
    umax = alloc.max_thrusts(v)
    B = alloc.effectiveness_matrix(v)
    w_req = np.array([v.mass * v.gravity, 0, 0, 1.0])     # 1 N m yaw at hover collective: beyond what hover headroom gives?
    u, ach, exact = alloc.allocate(B, umax, w_req)
    feas = alloc.wrench_feasible(B, umax, w_req)
    # largest yaw torque at hover collective
    lo, hi = 0.0, 5.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if alloc.wrench_feasible(B, umax, np.array([v.mass * v.gravity, 0, 0, mid])):
            lo = mid
        else:
            hi = mid
    # max roll torque at hover collective
    lo2, hi2 = 0.0, 50.0
    for _ in range(40):
        mid = (lo2 + hi2) / 2
        if alloc.wrench_feasible(B, umax, np.array([v.mass * v.gravity, mid, 0, 0])):
            lo2 = mid
        else:
            hi2 = mid
    return dict(max_yaw_torque_nm=lo, max_roll_torque_nm=lo2, yaw1nm_feasible=feas)


def sim_suite(v):
    r = {}
    w_h = am.hover_rotor_speed(v)
    umax = alloc.max_thrusts(v)
    r["hover_rotor_rad_s"] = float(w_h)
    r["hover_rpm"] = float(w_h * 30 / math.pi)
    r["hover_thrust_fraction"] = float((w_h / v.rotors[0].omega_max) ** 2)
    r["thrust_to_weight_nominal_V"] = float(umax.sum() / (v.mass * v.gravity))
    w_t = trim_speeds(v)
    r["trim_speed_spread_pct"] = float((w_t.max() - w_t.min()) / w_t.mean() * 100)
    r["equal_speed_hover_pitch_accel_rad_s2"] = float(abs(am.wrench(v, am.initial_state(v, p=(0, 0, 50.0), rotor_speed=w_h))[1][1] / v.inertia[1, 1]))
    s = am.initial_state(v, p=(0, 0, 50.0), rotor_speed=w_t)
    for _ in range(1000):
        s = am.step(v, s, w_t, 0.001)
    r["hover_drift_1s_m"] = float(np.linalg.norm(s.p - np.array([0, 0, 50.0])))
    r["collective_step_x1.5"] = run_collective_step(v)
    r["attitude_roll20_wn25"] = run_attitude(v, math.radians(20), 25.0)
    r["attitude_roll30_wn45_saturating"] = run_attitude(v, math.radians(30), 45.0, t_end=1.0)
    r["full_throttle_with_battery"] = run_full_throttle(v)
    r["wrench_limits_at_hover"] = run_yaw_saturation(v)
    B = alloc.effectiveness_matrix(v)
    r["roll_accel_per_nm_rad_s2"] = float(1.0 / v.inertia[0, 0])
    r["pitch_accel_per_nm_rad_s2"] = float(1.0 / v.inertia[1, 1])
    return r


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf-dir", default=str(REPO / "Assets" / "Robot Descriptions" / "aos7v5"))
    ap.add_argument("--skip-sim", action="store_true")
    a = ap.parse_args()
    udir = Path(a.urdf_dir)
    rep = json.loads((REPO / "Docs" / "DroneVisuals" / "model" / "cad_report.json").read_text())
    sc = yaml.safe_load((udir / "aos7v5_multirotor.yaml").read_text())
    root, links, joints = parse_urdf(udir / "aos7v5.urdf")

    # 1. mass properties vs CAD
    m, com, I, wp = total_from_urdf(links, joints)
    rm, rc, rI = rep["mass_kg"], np.array(rep["com_m"]), np.array(rep["inertia_com"])
    check("URDF total mass == CAD (1%)", abs(m / rm - 1) < 0.01, f"{m:.4f} vs {rm:.4f} kg")
    check("URDF CoM == CAD (1 mm)", np.linalg.norm(com - rc) < 1e-3, f"{np.round(com*1e3,2)} vs {np.round(rc*1e3,2)} mm")
    # CoM above is in base_link coordinates; the CAD report is in the same frame (anchor 0)
    dI = np.abs(np.diag(I) / np.diag(rI) - 1).max()
    check("URDF inertia diag == CAD (1%)", dI < 0.01, f"max rel {dI:.2e}")
    check("URDF inertia off-diagonals == CAD (1% of max diag)", np.abs(I - rI).max() < 0.01 * np.diag(rI).max(),
          f"max abs {np.abs(I-rI).max():.2e}")
    RES["mass_props"] = dict(mass_kg=m, com_m=com.tolist(), inertia=I.tolist())
    # mesh-based independent check on uniform-density links
    uniform = ["battery_link", "l2_link", "jetson_link", "usb_eth_link", "esc_link", "harness_link", "rx_link", "motor_0"]
    worst = 0.0
    for n in uniform:
        ms = [trimesh.load(udir / "meshes" / "visual" / Path(f).name) for f in links[n]["visuals"]]
        mesh = trimesh.util.concatenate(ms)
        dens = links[n]["mass"] / mesh.volume
        Im = mesh.moment_inertia * dens
        com_m = mesh.center_mass
        e_i = np.abs(np.diag(Im) / np.diag(links[n]["I"]) - 1).max()
        e_c = np.linalg.norm(com_m - links[n]["com"])
        worst = max(worst, e_i)
        check(f"mesh-recomputed inertia {n} (1%)", e_i < 0.01 and e_c < 5e-4, f"I err {e_i:.2e}, com err {e_c*1e3:.2f} mm")
    # 2. per-link and total inertia sanity
    bad = []
    for n, l in links.items():
        if l["mass"]:
            pd_, tri, w = inertia_ok(l["I"])
            if not (pd_ and tri):
                bad.append(n)
    check("every link inertia positive-definite and triangle-inequality OK", not bad, f"bad={bad}")
    pd_, tri, w = inertia_ok(I)
    check("total inertia positive-definite + triangle inequality", pd_ and tri, f"eig {np.round(w,6)}")
    # 3. rotors vs sidecar
    sx = {r["index"]: r for r in sc["rotors"]}
    for i in range(4):
        ln = f"rotor_{i}"
        pos = world_pos(ln, joints, {})
        check(f"{ln}: continuous joint on +z", joints[ln]["type"] == "continuous" and np.allclose(joints[ln]["axis"], [0, 0, 1]))
        check(f"{ln} position == sidecar", np.allclose(pos, sx[i]["position_base_link_m"], atol=1e-6), f"{np.round(pos,4)}")
        check(f"{ln} rel-CoM position == sidecar", np.allclose(pos - com, sx[i]["position_rel_com_m"], atol=2e-5))
        mesh = trimesh.util.concatenate([trimesh.load(udir / "meshes" / "visual" / Path(f).name) for f in links[ln]["visuals"]])
        rad = np.linalg.norm(mesh.vertices[:, :2], axis=1).max()
        check(f"{ln} prop radius == sidecar rotor_radius (1 mm)", abs(rad - sc["rotor_radius"]["value"]) < 1e-3, f"{rad*1e3:.2f} mm")
    spins = [sx[i]["spin"] for i in range(4)]
    check("spin order PX4/CRANE (FR,RL,FL,RR) = +,+,-,-", spins == [1, 1, -1, -1])
    pos = np.array([world_pos(f"rotor_{i}", joints, {}) for i in range(4)])
    check("quad-X geometry: FR(+,-) RL(-,+) FL(+,+) RR(-,-)",
          all(np.sign(pos[i, :2]) .tolist() == s for i, s in enumerate([[1, -1], [-1, 1], [1, 1], [-1, -1]])))
    check("diagonal 313.6 mm (AOS 7 V5 datasheet)", abs(np.linalg.norm(pos[0] - pos[1]) - 0.3136) < 5e-4, f"{np.linalg.norm(pos[0]-pos[1])*1e3:.1f} mm")
    for i, (ref) in enumerate(sc["rotors"]):
        pass
    j_cad = links["rotor_0"]["I"][2, 2]
    check("sidecar rotor_inertia == URDF rotor_0 Izz", abs(sc["rotor_inertia"]["value"] / j_cad - 1) < 1e-3)
    check("sidecar mass == URDF mass (0.1%)", abs(sc["mass"]["value"] / m - 1) < 1e-3)
    check("kf*omega_max^2 == thrust_max_per_motor", abs(sc["thrust_coefficient_kf"]["value"] * sc["omega_max"]["value"] ** 2 / sc["thrust_max_per_motor"]["value"] - 1) < 1e-6)
    # sensor frames
    for f in ["oak_d_left_camera_frame", "oak_d_right_camera_frame", "oak_d_rgb_camera_frame", "oak_d_left_camera_optical_frame",
              "oak_d_right_camera_optical_frame", "oak_d_rgb_camera_optical_frame", "l2_lidar_frame", "l2_imu_frame", "cube_imu_link"]:
        check(f"frame {f} present", f in links)
    check("OAK-D stereo baseline 75 mm", abs(world_pos("oak_d_left_camera_frame", joints, {})[1] - world_pos("oak_d_right_camera_frame", joints, {})[1] - 0.075) < 1e-6)
    ro = joints["oak_d_rgb_camera_optical_frame"]["rpy"]
    check("optical frame rpy = (-pi/2, 0, -pi/2) REP-103", np.allclose(ro, [-math.pi / 2, 0, -math.pi / 2], atol=1e-6))
    # 4. meshes
    missing, notwt = [], []
    for n, l in links.items():
        for f in l["visuals"]:
            p = udir / "meshes" / "visual" / Path(f).name
            if not p.exists():
                missing.append(str(p)); continue
            mesh = trimesh.load(p)
            if not mesh.is_watertight:
                notwt.append(p.name)
    check("all URDF mesh files exist", not missing, str(missing[:3]))
    check("all visual meshes watertight", not notwt, f"{len(notwt)} not watertight: {notwt[:4]}")
    check("collision hull exists and is watertight", trimesh.load(udir / "meshes" / "collision" / "aos7v5_hull.stl").is_watertight)
    check("prop-disc clearance of every non-prop body >= 1 mm", rep["prop_clearance_min_mm"] >= 1.0, f"{rep['prop_clearance_min_mm']:.1f} mm")
    check("no unexplained CAD interferences", len(rep["interferences"]) == 0, str(rep["interferences"][:3]))
    hull_vol = trimesh.load(udir / "meshes" / "collision" / "aos7v5_hull.stl").volume
    RES["hull_volume_m3"] = hull_vol

    # 5. RK4 plant
    if not a.skip_sim:
        rotors_rel = sc["rotors"]
        v_new = make_vehicle(m, I, rotors_rel, sc)
        v_old = load_vehicle("aos7_v5_f90_6s", with_battery=True)
        sims = {"cad_1p843kg": sim_suite(v_new), "crane_profile_1p383kg": sim_suite(v_old)}
        # also isolate the cause: old profile geometry with CAD mass only is covered by comparing rows
        RES["sim"] = sims
        n_, o_ = sims["cad_1p843kg"], sims["crane_profile_1p383kg"]
        check("plant trimmed hover is steady (< 1 mm in 1 s)", n_["hover_drift_1s_m"] < 1e-3, f"{n_['hover_drift_1s_m']:.2e} m")
        check("T/W > 2.5 (CRANE flag threshold)", n_["thrust_to_weight_nominal_V"] > 2.5, f"{n_['thrust_to_weight_nominal_V']:.2f}")
        check("collective step reaches target accel", abs(n_["collective_step_x1.5"]["az_final"] / n_["collective_step_x1.5"]["az_target"] - 1) < 0.05)
        check("roll 20 deg step settles (final within 1 deg)", abs(n_["attitude_roll20_wn25"]["final_roll_deg"] - 20) < 1.0, str(n_["attitude_roll20_wn25"]))
        check("saturating roll case is flagged saturated", n_["attitude_roll30_wn45_saturating"]["saturated_step_frac"] > 0 or n_["attitude_roll30_wn45_saturating"]["peak_rotor_frac"] > 0.99,
              str(n_["attitude_roll30_wn45_saturating"]))
        print(json.dumps(sims, indent=1))
        # 6. pybullet
        try:
            import pybullet as pb
            tmp = Path(os.environ.get("TMPDIR", str(Path.home() / ".cache" / "cad-tmp"))) / "aos7v5_pb"
            tmp.mkdir(parents=True, exist_ok=True)
            txt = (udir / "aos7v5.urdf").read_text().replace("package://aos7v5/", str(udir) + "/")
            (tmp / "aos7v5_abs.urdf").write_text(txt)
            cid = pb.connect(pb.DIRECT)
            pb.setGravity(0, 0, -9.81)
            pb.setTimeStep(1 / 480)
            plane = pb.createMultiBody(0, pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[5, 5, 0.05]), basePosition=[0, 0, -0.05])
            body = pb.loadURDF(str(tmp / "aos7v5_abs.urdf"), basePosition=[0, 0, 0.6])
            nj = pb.getNumJoints(body)
            for j in range(nj):   # frames without <inertial> get mass 1 in pybullet; the URDF convention (as the boats) is no inertial
                if pb.getJointInfo(body, j)[12].decode() in [n for n, l in links.items() if l["mass"] is None]:
                    pb.changeDynamics(body, j, mass=1e-9)
            mtot = pb.getDynamicsInfo(body, -1)[0] + sum(pb.getDynamicsInfo(body, j)[0] for j in range(nj))
            check("pybullet loads URDF, total mass == URDF", abs(mtot / m - 1) < 1e-3, f"{mtot:.4f} kg, {nj} joints")
            for _ in range(480 * 3):
                pb.stepSimulation()
            pos_b, orn_b = pb.getBasePositionAndOrientation(body)
            vel = np.linalg.norm(pb.getBaseVelocity(body)[0])
            lowest = min(pb.getAABB(body, j)[0][2] for j in range(-1, nj))
            tilt = math.degrees(math.acos(np.clip(1 - 2 * (orn_b[0] ** 2 + orn_b[1] ** 2), -1, 1)))
            check("pybullet drop test: comes to rest upright on the pack", vel < 0.02 and tilt < 10 and abs(lowest) < 2e-3,
                  f"v={vel:.3f} m/s tilt={tilt:.1f} deg lowest_aabb={lowest*1e3:.1f} mm, base z={pos_b[2]*1e3:.1f} mm (pack bottom at +49.5 + contact margin)")
            RES["pybullet"] = dict(mass=mtot, rest_base_z_m=pos_b[2], tilt_deg=tilt)
            pb.disconnect()
        except ImportError:
            print("pybullet not installed; skipped")
    (HERE / "test_results.json").write_text(json.dumps(RES, indent=1, default=float))
    print(f"\n{len(RES['checks'])-len(FAILS)}/{len(RES['checks'])} checks passed")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
