"""Compile scenario v1 + task v1 into what CRANE consumes TODAY, plus the v1 fields CRANE cannot use yet.

    python3 -m scenes.crane_compile --family F --split S --seed N [--plan P|all] [--out DIR]
    python3 -m scenes.crane_compile --library          # one train + one held-out scene per family -> crane_out/

Outputs per scene (DIR defaults to Tools/Fleet/scenes/crane_out):
  scenarios/<name>.json         crane.fleet_scenario/v0 as FleetSpec/FleetWorld/Scenario.cs/AutonomyMission read it, plus
                                an ``x_v1`` block (doors, thresholds, floor patches, tags, v1 zones, units, faults, gaps)
                                that the current C# ignores and the v1 loader (Assets/Scripts/Fleet/Core/ScenarioV1.cs) reads
  missions/<name>.<plan>.json   crane.fleet_mission/v0 for the profile fleet (scout_1, lifter_1, cargo_1, cargo_2)
  jobs_scenes_v1.json           a FleetHarness jobs file over every compiled (scene, plan)

How v1 maps (what CRANE builds today, no C# change):
  walls + doors   lowered by scenes.common.lower into box obstacles (wall segments, lintels). v0 has ONE doorway in a
                  wall of constant x (Scenario.cs) and AutonomyMission.BuildMap only maps the doorway wall, so every
                  wall goes in as obstacles instead; ``walls`` is empty and ``doorway`` is a stub (the primary door's
                  width, recorded in the run manifest; no geometry).
  outer walls     built by FleetWorld from arena.size_xy and the overlay thickness (0.15 m, = v1 outer_wall_thickness)
  zones           dict keyed by v1 id; ``delivery`` is the task's primary delivery zone (AutonomyMission's truth check)
  payload         the first payload as mission crane_payload (mass_kg = the true mass)
  fleet           units with a CRANE profile role are mapped onto scout_1 / lifter_1 / cargo_1 / cargo_2 (crane_start);
                  unused profile rovers are parked in free spots; other module graphs are a gap
  faults          the jobs entry's ``failures`` string (rover:wheel_fl:dead_locked@t)
"""
import argparse
import json
import math
from pathlib import Path

from .common import HERE, OUTER_WALL_THICKNESS, FRICTION_CLASSES, dump, lower, rect_centre, sha, zone, payload_type
from .families import GENERATORS, generate
from . import catalog

OUT = HERE / "crane_out"
PROFILE_ROVERS = {"scout": ["scout_1"], "lifter": ["lifter_1"], "cargo": ["cargo_1", "cargo_2"]}
DEFAULT_ROLE = {"scout_1": "scout", "lifter_1": "lifter", "cargo_1": "cargo", "cargo_2": "cargo"}
# demo-derived handling numbers (missions/autonomy_demo_v2.json); provisional for other payloads
GRASP = {"box_long": 0.237}
HOLD = {"box_long": 0.637}


def gap(field, status, detail):
    return {"field": field, "status": status, "detail": detail}


def compile_scenario(scn, task):
    gaps = []
    X, Y = scn["arena"]["size_xy"]
    floor = scn["arena"]["floor"]
    prof = FRICTION_CLASSES[floor]["profile"]
    if prof is None:
        gaps.append(gap("arena.floor", "approximated", "%s is not in fleet_profile_v0 floors; hard_tile used" % floor))
        prof = "hard_tile"
    if abs(scn["arena"].get("outer_wall_thickness", OUTER_WALL_THICKNESS) - OUTER_WALL_THICKNESS) > 1e-9:
        gaps.append(gap("arena.outer_wall_thickness", "ignored", "FleetWorld uses the overlay's 0.15 m"))
    obstacles = []
    for s in lower(scn):
        if s.kind in ("outer_wall", "threshold"):
            continue
        if abs(s.yaw) > 1e-9:
            gaps.append(gap("obstacles.%s.yaw" % s.id, "unsupported", "FleetWorld boxes are axis-aligned"))
        obstacles.append({"id": s.id, "type": "box", "centre": [round(s.cx, 6), round(s.cy, 6), round(s.cz, 6)],
                          "size": [round(s.sx, 6), round(s.sy, 6), round(s.sz, 6)], "v1_kind": s.kind, "v1_source": s.source})
    doors = scn["doors"]
    if doors:
        d0 = min(doors, key=lambda d: d["width"])
        stub = {"in_wall": "(lowered: no wall carries a door; geometry is in obstacles)", "centre_y": d0["centre"],
                "width": {"default": d0["width"]}, "height": d0["height"], "v1_door": d0["id"]}
        gaps.append(gap("doors", "approximated", "%d door(s) lowered to wall-segment and lintel boxes; v0 doorway is a "
                        "stub (narrowest door %s)" % (len(doors), d0["id"])))
    else:
        stub = {"in_wall": "(none)", "centre_y": Y / 2, "width": {"default": 1.0}, "height": 2.0}
    th = [s.to_dict() for s in lower(scn) if s.kind == "threshold"]
    if th:
        gaps.append(gap("doors.threshold_height", "ignored", "%d threshold(s) in x_v1.thresholds only; CRANE builds no "
                        "sill (needs the v1 loader: a traversable solid, excluded from PlanMap)" % len(th)))
    if scn["floor_patches"]:
        gaps.append(gap("floor_patches", "ignored", "CRANE has one floor per arena (FleetSpec.Floor); patches are in "
                        "x_v1.floor_patches (v1 loader parses them; the tractive plant does not look them up yet)"))
    zones = {z["id"]: {"rect": z["rect"], "kind": z["kind"]} for z in scn["zones"]}
    deliv = primary_delivery(task)
    if deliv and deliv != "delivery":
        zones["delivery"] = {"rect": zone(scn, deliv)["rect"], "kind": "delivery", "alias_of": deliv}
    if sum(1 for z in scn["zones"] if z["kind"] in ("delivery", "slot")) > 1:
        gaps.append(gap("zones", "approximated", "the CRANE truth check uses zones.delivery only (%s)" % deliv))
    if any(z.get("required_yaw") is not None for z in scn["zones"]):
        gaps.append(gap("zones.required_yaw", "ignored", "the truth check tests the payload centre only"))
    if any(z["kind"] == "charging" for z in scn["zones"]):
        gaps.append(gap("zones.charging", "approximated", "a charging zone is a floor patch; CRANE has no battery model"))
    if any(z["kind"] == "aisle" for z in scn["zones"]):
        gaps.append(gap("zones.aisle", "ignored", "no aisle reservation in Crane.Fleet.Autonomy"))
    cams = [{"id": c["id"], "position": c["position"], "look_at": c["look_at"]} for c in scn["overhead_cameras"]]
    if any(c["mode"] != "working" or c.get("mode_declared") for c in scn["overhead_cameras"]):
        gaps.append(gap("overhead_cameras.mode", "approximated", "per-camera modes are run options in CRANE "
                        "(overhead_modes), not scenario fields; 'occluded' is geometry (a banner box) in v1"))
    if any(not p["spawn"].get("pose_known", True) for p in scn["payloads"]):
        gaps.append(gap("payloads.spawn.pose_known", "approximated", "CRANE's autonomy finds the payload with an explore "
                        "task; the fastsim 'unknown until seen' rule is not a CRANE field"))
    if len(scn["payloads"]) > 1:
        gaps.append(gap("payloads", "unsupported", "one crane_payload per mission"))
    out = {"schema": "crane.fleet_scenario/v0", "scenario_id": scn["scenario_id"],
           "note": "compiled from %s by scenes.crane_compile; the v1 source is authoritative" % scn["scenario_id"],
           "arena": {"size_xy": [X, Y], "wall_height": scn["arena"]["wall_height"], "floor": prof},
           "walls": [], "doorway": stub, "zones": zones, "obstacles": obstacles, "overhead_cameras": cams,
           "overhead_modes": ["working", "occluded", "disabled"],
           "payload_spawn": {"zone": (scn["payloads"][0]["spawn"].get("zone") if scn["payloads"] else None),
                             "types": sorted({p["type"] for p in scn["payloads"]})},
           "mission_default": task["stresses"]["decision"] if task.get("stresses") else "",
           "outcomes": {"delivered": "payload centre inside zones.delivery, at rest, supported, within the deadline"},
           "deadline_s": task["deadline_s"],
           "x_v1": {"source_sha256": sha(scn, 64), "family": scn["family"], "doors": doors, "thresholds": th,
                    "floor_patches": [dict(p, profile_floor=FRICTION_CLASSES[p["friction_class"]]["profile"])
                                      for p in scn["floor_patches"]],
                    "tags": scn["tags"], "zones": scn["zones"], "overhead_cameras": scn["overhead_cameras"],
                    "payloads": scn["payloads"], "units": scn["fleet"]["units"], "faults": scn["fleet"]["faults"]}}
    return out, gaps


def primary_delivery(task):
    for r in task["outcomes"]["success"]:
        if "payload_in_zone" in r:
            return r["payload_in_zone"]["zone"]
    return None


def free_spots(scn, n, avoid=()):
    """n parking poses (x, y, 0) clear of every solid, zone and start by 0.35 m (a coarse scan)."""
    X, Y = scn["arena"]["size_xy"]
    boxes = []
    for s in lower(scn):
        if s.drive_block:
            pts = s.footprint()
            boxes.append((min(p[0] for p in pts), min(p[1] for p in pts), max(p[0] for p in pts), max(p[1] for p in pts)))
    for z in scn["zones"]:
        r = z["rect"]
        if z["kind"] in ("staging", "aisle") or (r[2] - r[0]) * (r[3] - r[1]) > 6.0:
            continue   # halls and aisles are space to park in, not destinations
        boxes.append(tuple(r))
    for p in list(avoid):
        boxes.append((p[0] - 0.3, p[1] - 0.3, p[0] + 0.3, p[1] + 0.3))
    for p in scn["payloads"]:
        x, y, _ = p["spawn"]["pose"]
        boxes.append((x - 0.5, y - 0.5, x + 0.5, y + 0.5))
    out = []
    y = Y - 0.45
    while y > 0.4 and len(out) < n:
        x = 0.45
        while x < X - 0.4 and len(out) < n:
            r = (x - 0.6, y - 0.6, x + 0.6, y + 0.6)
            if all(r[2] <= b[0] or r[0] >= b[2] or r[3] <= b[1] or r[1] >= b[3] for b in boxes):
                out.append((round(x, 3), round(y, 3), 0.0))
                boxes.append(r)
            x += 0.3
        y -= 0.3
    return out


def compile_mission(scn, task, plan_name):
    """(mission dict or None, rover map, gaps, failures string)."""
    gaps = []
    plan = task["plans"][plan_name]
    units = scn["fleet"]["units"]
    rmap, free = {}, {k: list(v) for k, v in PROFILE_ROVERS.items()}
    used = set()
    for st in plan["steps"]:
        used |= {st[k] for k in ("unit", "docker", "target", "from", "to") if k in st} | set(st.get("units", []))
    motion_only = {u["id"] for u in units if all(st["kind"] in ("goto", "wait") for st in plan["steps"]
                                                  if u["id"] in {st.get(k) for k in ("unit", "docker", "target", "from", "to")})}
    kept = []
    for u in units:
        role = catalog.unit(u["catalog"]).get("crane_role")
        if role and free.get(role):
            rmap[u["id"]] = free[role].pop(0)
        elif role and u["id"] in motion_only and (free["scout"] or free["lifter"]):
            sub = (free["lifter"] or free["scout"]).pop(0)
            rmap[u["id"]] = sub
            gaps.append(gap("fleet.units.%s" % u["id"], "approximated", "role substitution: %s (%s) runs as %s (same mecanum "
                            "chassis; role mass and sensors differ); it only drives in this plan" % (u["id"], role, sub)))
        elif u["id"] not in used:
            gaps.append(gap("fleet.units.%s" % u["id"], "omitted", "%s (%s) is idle in plan %s and not buildable in CRANE: "
                            "left out (it is still an obstacle in fastsim)" % (u["id"], u["catalog"], plan_name)))
            continue
        else:
            gaps.append(gap("fleet.units.%s" % u["id"], "unsupported",
                            "%s (%s, %s) has no CRANE profile role or no free profile rover; autonomy missions spawn only "
                            "scout_1, lifter_1, cargo_1, cargo_2 (module graphs spawn only in probe missions, uncoupled)"
                            % (u["id"], u["catalog"], catalog.unit(u["catalog"])["mobility_class"])))
        kept.append(u)
    if len(rmap) < len(kept):
        return None, rmap, gaps, ""
    units = [u for u in units if u["id"] in rmap]
    starts = {rmap[u["id"]]: u["start"] for u in units}
    parked = [r for r in DEFAULT_ROLE if r not in starts]
    spots = free_spots(scn, len(parked), [s[:2] for s in starts.values()])
    for r, sp in zip(parked, spots):
        starts[r] = list(sp)
    if len(spots) < len(parked):
        gaps.append(gap("fleet", "unsupported", "no free parking spot for the unused profile rovers"))
    pl = scn["payloads"][0] if scn["payloads"] else None
    m = {"schema": "crane.fleet_mission/v0", "mission_id": "%s/%s" % (scn["scenario_id"], plan_name),
         "note": "compiled by scenes.crane_compile from task %s plan %s; crane_* keys are CRANE setup" % (task["task_id"], plan_name),
         "crane_start": {r: [round(v, 4) for v in p] for r, p in starts.items()},
         "crane_unit_map": rmap, "crane_max_duration_s": round(float(task["deadline_s"]) + 30.0, 1), "tasks": []}
    if pl:
        if pl["spawn"].get("on_unit"):
            gaps.append(gap("payloads.spawn.on_unit", "approximated", "payload starts on a unit: mission initial_payloads"))
            sup = [rmap[pl["spawn"]["on_unit"]]]
            m["initial_payloads"] = [{"id": pl["id"], "type": pl["type"], "supporters": sup, "com": pl["spawn"]["pose"][:2]}]
        m["crane_payload"] = {"id": pl["id"], "type": pl["type"], "pose": pl["spawn"]["pose"], "pallet_height": pl.get("pallet_height", 0.03),
                              "mass_kg": pl["mass_kg"]}
    tasks = m["tasks"]
    tasks.append({"id": "settle", "type": "wait", "agents": sorted(starts), "duration": 1.5})
    members = {u: [rmap[u]] for u in rmap}          # v1 unit -> CRANE rovers (assemblies after docks)
    couplings = {}
    last_pose = {u["id"]: u["start"] for u in units}
    dropped = []
    lifter = "lifter_1"
    for s in plan["steps"]:
        if s.get("when"):
            dropped.append(s["id"])
            continue
        after = [a for a in s.get("after", []) if a not in dropped] or ["settle"]
        k = s["kind"]
        base = {"id": s["id"], "after": after}
        if k == "goto":
            g = list(s["goal"]) if "goal" in s else list(rect_centre(zone(scn, s["zone"])["rect"])) + [
                zone(scn, s["zone"]).get("required_yaw") or 0.0]
            t = dict(base, type="goto", agent=members[s["unit"]][0], goal=[round(v, 4) for v in g])
            if s.get("via"):
                t["route"] = s["via"]
            tasks.append(t)
            last_pose[s["unit"]] = g
        elif k == "pickup":
            p = next(x for x in scn["payloads"] if x["id"] == s["payload"])
            who = members[s["unit"]]
            src = next((u for u in units if u["id"] == s["unit"]), None)
            if src is not None and "lift" in catalog.unit(src["catalog"])["tools"]:
                tasks.append(dict(base, type="lift", agent=who[0], object=p["id"], grasp_offset=GRASP.get(p["type"], 0.1),
                                  height=0.15))
            else:
                gaps.append(gap("task.%s" % s["id"], "approximated", "a deck pickup is a lifter lift + handoff in CRANE "
                                "(fastsim loads the deck kinematically); %s" % ("provisional grasp/hold offsets" if p["type"] not in GRASP else "demo offsets")))
                tasks.append(dict(base, id=s["id"] + "_lift", type="lift", agent=lifter, object=p["id"],
                                  grasp_offset=GRASP.get(p["type"], round(payload_type(p["type"])["size_lwh"][0] / 2 - 0.163, 3)), height=0.15))
                tasks.append({"id": s["id"], "type": "handoff", "from": lifter, "supporters": who, "payload": p["id"],
                              "payload_type": p["type"], "hold_offset": HOLD.get(p["type"], 0.5), "after": [s["id"] + "_lift"] + after,
                              "place_on": who[0], "place_offset": [-0.17 if len(who) > 1 else 0.0, 0.0, math.pi]})
                spot = free_spots(scn, 1, [x[:2] for x in starts.values()])
                tasks.append({"id": s["id"] + "_clear", "type": "goto", "agent": lifter,
                              "goal": list(spot[0]) if spot else [0.5, 0.5, 0.0], "after": [s["id"]]})
        elif k == "deliver":
            z = zone(scn, s["zone"])
            g = list(rect_centre(z["rect"])) + [z.get("required_yaw") or 0.0]
            t = dict(base, type="transport", agent=members[s["unit"]][0], goal=[round(v, 4) for v in g], payload=s["payload"],
                     tol=[0.05, 0.08])
            if any(x["id"].endswith("_clear") for x in tasks):
                t["after"] = t["after"] + [x["id"] for x in tasks if x["id"].endswith("_clear")]
            tasks.append(t)
        elif k == "dock":
            d, tg = members[s["docker"]], members[s["target"]]
            if len(d) > 1 or len(tg) > 1:
                gaps.append(gap("task.%s" % s["id"], "unsupported", "docking an assembly to an assembly"))
            tp = last_pose.get(s["target"], [0, 0, 0])
            tasks.append(dict(base, id=s["id"] + "_meet", type="rendezvous", target=tg[0], docker=d[0],
                              target_face=s["target_face"], docker_face=s["docker_face"],
                              meeting_pose=[round(v, 4) for v in tp], retries=1))
            tasks.append({"id": s["id"], "type": "dock", "docker": d[0], "docker_face": s["docker_face"], "target": tg[0],
                          "target_face": s["target_face"], "after": [s["id"] + "_meet"], "retries": 2})
            members[s["as"]] = tg + d
            couplings[s["as"]] = (d[0], s["docker_face"], tg[0], s["target_face"])
        elif k == "undock":
            if s["unit"] not in couplings:
                gaps.append(gap("task.%s" % s["id"], "unsupported", "undocking a pre-assembled unit (%s): CRANE cannot "
                                "spawn it" % s["unit"]))
                return None, rmap, gaps, ""
            dd, df, tt, tf = couplings[s["unit"]]
            tasks.append(dict(base, type="separate", docker=dd, docker_face=df, target=tt, target_face=tf))
            for nid in s["as"]:
                members[nid] = [tt] if nid == s["as"][0] else [dd]
        elif k == "charge":
            bay = s.get("bay") or (s.get("bays") or [z["id"] for z in scn["zones"] if z["kind"] == "charging"])[0]
            z = zone(scn, bay)
            g = list(rect_centre(z["rect"])) + [z.get("required_yaw") or 0.0]
            uid = members[s["unit"]][0]
            soc = next(u["soc"] for u in units if u["id"] == s["unit"])
            dur = max(0.0, (float(s.get("target_soc", 0.8)) - soc) / 0.02)
            tasks.append(dict(base, id=s["id"] + "_go", type="goto", agent=uid, goal=[round(v, 4) for v in g]))
            tasks.append({"id": s["id"], "type": "wait", "agents": [uid], "duration": round(dur, 1), "after": [s["id"] + "_go"]})
            if s.get("then_zone"):
                z2 = zone(scn, s["then_zone"])
                tasks.append({"id": s["id"] + "_then", "type": "goto", "agent": uid,
                              "goal": [round(v, 4) for v in list(rect_centre(z2["rect"])) + [0.0]], "after": [s["id"]]})
            gaps.append(gap("task.%s" % s["id"], "approximated", "charge = goto bay + wait (no battery, no bay "
                            "reservation; %s policy is fixed at compile time)" % s.get("policy", "assigned")))
        elif k == "observe":
            p = next(x for x in scn["payloads"] if x["id"] == s["payload"])
            reg = zone(scn, p["spawn"]["zone"])["rect"] if p["spawn"].get("zone") else None
            vps = [v for vp in s["viewpoints"] for v in vp]
            tasks.append(dict(base, type="explore", agent=members[s["unit"]][0], region=reg, target=p["id"],
                              viewpoints=vps, dwell=s.get("dwell_s", 1.5), retries=1))
        elif k == "handoff":
            gaps.append(gap("task.%s" % s["id"], "unsupported", "deck-to-deck handoff (CRANE handoff is lifter -> decks)"))
            dropped.append(s["id"])
        elif k == "wait":
            tasks.append(dict(base, type="wait", agents=[members[x][0] for x in s.get("units", [s.get("unit")]) if x],
                              duration=s.get("duration_s", 1.0)))
    if dropped:
        gaps.append(gap("task.when", "unsupported", "conditional steps (on a fault) have no CRANE equivalent; dropped: %s"
                        % dropped))
    if plan.get("traffic") in ("reserved", "none"):
        gaps.append(gap("plans.traffic", "ignored", "CRANE's executive has its own deconfliction; policy %s not mapped" % plan["traffic"]))
    failures = ";".join("%s:%s:%s@%s" % (rmap[f["unit"]], f["target"], f["mode"], f["t_s"]) for f in scn["fleet"]["faults"]
                        if f["unit"] in rmap)
    return m, rmap, gaps, failures


def approx_preassembled(scn, task, plan_name):
    """CRANE cannot spawn a pre-coupled assembly (the 3-carrier L of long_payload_door). For a plan that undocks it into a
    chain that then delivers, emit the closest CRANE mission: the chain's two modules start as cargo_1 / cargo_2 at their
    module poses, dock front-to-rear at t0, the lifter loads the payload from a pallet beside them (as in the demo), and
    the chain transports it through the door. The third module and the 'keep' plan have no CRANE equivalent."""
    plan = task["plans"][plan_name]
    und = next((s for s in plan["steps"] if s["kind"] == "undock"), None)
    dl = next((s for s in plan["steps"] if s["kind"] == "deliver"), None)
    if und is None or dl is None:
        return None, []
    u = next(x for x in scn["fleet"]["units"] if x["id"] == und["unit"])
    cat = catalog.unit(u["catalog"])
    stay = [m for m in cat["modules"] if "%s/%s" % (u["id"], m["id"]) not in und["part"]]
    if len(stay) != 2:
        return None, []
    X, Y, T = u["start"]
    c, s_ = math.cos(T), math.sin(T)
    poses = sorted(((X + c * m["x"] - s_ * m["y"], Y + s_ * m["x"] + c * m["y"], T) for m in stay), key=lambda p: p[0])
    rear, front = poses[0], poses[1]
    gap_m = 0.15   # start the docker 15 cm behind its mated pose
    starts = {"cargo_1": [round(front[0], 4), round(front[1], 4), T], "cargo_2": [round(rear[0] - gap_m, 4), round(rear[1], 4), T]}
    pl = scn["payloads"][0]
    L = payload_type(pl["type"])["size_lwh"][0]
    pallet = [round(front[0] + 0.1, 4), round(front[1] + 1.1, 4), 0.0]
    spots = free_spots(scn, 2, [p[:2] for p in starts.values()] + [pallet[:2]])
    starts["lifter_1"] = [spots[0][0], spots[0][1], 0.0] if spots else [0.5, 0.5, 0.0]
    starts["scout_1"] = [spots[1][0], spots[1][1], 0.0] if len(spots) > 1 else [0.5, 1.2, 0.0]
    z = zone(scn, dl["zone"])
    goal = list(rect_centre(z["rect"])) + [0.0]
    m = {"schema": "crane.fleet_mission/v0", "mission_id": "%s/%s/crane_approx" % (scn["scenario_id"], plan_name),
         "note": approx_preassembled.__doc__.split("\n")[0], "crane_start": starts,
         "crane_unit_map": {"chain": ["cargo_1", "cargo_2"]}, "crane_max_duration_s": round(float(task["deadline_s"]) + 150.0, 1),
         "crane_payload": {"id": pl["id"], "type": pl["type"], "pose": [pallet[0] + L / 2 - 0.4, pallet[1], math.pi], "pallet_height": 0.03,
                           "mass_kg": pl["mass_kg"]},
         "tasks": [{"id": "settle", "type": "wait", "agents": sorted(starts), "duration": 1.5},
                   {"id": "couple", "type": "dock", "docker": "cargo_2", "docker_face": "front", "target": "cargo_1",
                    "target_face": "rear", "after": ["settle"], "retries": 2},
                   {"id": "pick", "type": "lift", "agent": "lifter_1", "object": pl["id"], "grasp_offset": GRASP.get(pl["type"], 0.237),
                    "height": 0.15, "after": ["settle"]},
                   {"id": "give", "type": "handoff", "from": "lifter_1", "supporters": ["cargo_1", "cargo_2"], "payload": pl["id"],
                    "payload_type": pl["type"], "hold_offset": HOLD.get(pl["type"], 0.637), "after": ["pick", "couple"],
                    "place_on": "cargo_1", "place_offset": [-0.17, 0.0, math.pi]},
                   {"id": "lifter_clear", "type": "goto", "agent": "lifter_1", "goal": starts["lifter_1"], "after": ["give"]},
                   {"id": dl["id"], "type": "transport", "agent": "cargo_1", "goal": [round(v, 4) for v in goal], "payload": pl["id"],
                    "tol": [0.05, 0.08], "after": ["give", "lifter_clear"]}]}
    gaps = [gap("fleet.units.%s" % u["id"], "approximated", "pre-assembled %s (%d modules) is not spawnable in CRANE; the "
                "chain's modules start as cargo_1/cargo_2 and dock at t0, the third module is omitted" % (u["catalog"], len(cat["modules"]))),
            gap("payloads.spawn.on_unit", "approximated", "the payload starts on a pallet beside the chain and is loaded by "
                "lifter_1 (demo v2 handoff); in v1 it starts on the assembly")]
    return m, gaps


def name_of(scn):
    f = scn["family"]
    return "%s_%s_s%d" % (f["id"], f["split"], f["seed"])


def compile_scene(scn, task, out=OUT, plans=None):
    name = name_of(scn)
    crane_scn, gaps = compile_scenario(scn, task)
    plans = plans or list(task["plans"])
    jobs, per_plan = [], {}
    for p in plans:
        m, rmap, mgaps, failures = compile_mission(scn, task, p)
        if m is None:
            am, agaps = approx_preassembled(scn, task, p)
            if am is not None:
                m, mgaps, failures = am, mgaps + agaps, ""
        per_plan[p] = {"mission": None, "gaps": mgaps, "rover_map": rmap}
        if m is not None:
            mp = dump(m, Path(out) / "missions" / ("%s.%s.json" % (name, p)))
            per_plan[p]["mission"] = str(mp)
            jobs.append({"id": "%s.%s" % (name, p), "seed": scn["family"]["seed"], "variant": "scenes_v1_%s" % p,
                         "scenario": str((Path(out) / "scenarios" / (name + ".json")).resolve()),
                         "mission_file": str(mp.resolve()), "failures": failures,
                         "labels": {"scenes_v1_family": scn["family"]["id"], "split": scn["family"]["split"], "plan": p,
                                    "expected_fastsim": task["plans"][p].get("expected")}})
    crane_scn["x_v1"]["gaps"] = gaps
    crane_scn["x_v1"]["plan_gaps"] = {p: v["gaps"] for p, v in per_plan.items()}
    dump(crane_scn, Path(out) / "scenarios" / (name + ".json"))
    return {"name": name, "scenario": str(Path(out) / "scenarios" / (name + ".json")), "gaps": gaps, "plans": per_plan,
            "jobs": jobs}


def library(out=OUT, seeds=((1, "train"), (1, "heldout"))):
    rows, jobs = [], []
    for f in GENERATORS:
        for seed, split in seeds:
            scn, task = generate(f, seed, split)
            r = compile_scene(scn, task, out)
            rows.append(r)
            jobs += r["jobs"]
    dump({"note": "scenes v1 in CRANE (compiled by scenes.crane_compile --library). Run with Tools/Fleet/fleet.sh jobs "
                  "<this file> after the integration agent has built and tested the v1 loader; the v0 subset needs no C#.",
          "out": "~/.cache/crane-fleet/jobs/scenes_v1".replace("~", str(Path.home())), "divergence_tol": 0.001, "jobs": jobs},
         Path(out) / "jobs_scenes_v1.json")
    summary = {r["name"]: {"scene_gaps": [g["field"] + ":" + g["status"] for g in r["gaps"]],
                           "plans": {p: {"mission": bool(v["mission"]), "gaps": [g["field"] + ":" + g["status"] for g in v["gaps"]]}
                                     for p, v in r["plans"].items()}} for r in rows}
    dump(summary, Path(out) / "transfer_summary.json")
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scenes.crane_compile")
    ap.add_argument("--family")
    ap.add_argument("--split", default="train")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plan", default="all")
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--library", action="store_true")
    a = ap.parse_args(argv)
    if a.library:
        for r in library(Path(a.out)):
            ok = [p for p, v in r["plans"].items() if v["mission"]]
            print("%-34s missions %-28s scene gaps %d" % (r["name"], ",".join(ok) or "-", len(r["gaps"])))
        return
    scn, task = generate(a.family, a.seed, a.split)
    r = compile_scene(scn, task, Path(a.out), None if a.plan == "all" else [a.plan])
    print(json.dumps({k: v for k, v in r.items() if k != "jobs"}, indent=1))


if __name__ == "__main__":
    main()
