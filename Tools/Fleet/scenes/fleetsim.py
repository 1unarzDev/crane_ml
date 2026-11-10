"""The fleet engine behind the fastsim fleet domain: units, plants, a task executive, perception and outcome rules.

fastsim's core drives ONE plant (the run's ego). A fleet scene has several units, so this engine:
  - drives the ego unit through the core's Navigator (its track, contacts and identity are the core's), and
  - steps every other unit itself with the same core classes (fastsim.controller.Navigator, fastsim.plant.GoalTracker
    and the FleetPlant below, a KinematicPlant subclass), at the core's dt, inside the controller tick.
Coupled assemblies move as one rigid unit (the lead's plant, the assembly's catalogue limits, the union of the module
rectangles as footprint). Perception is fastsim's ConeCamera per unit plus a 3-D box occlusion test (the core cone
only lets labelled disks occlude); overhead cameras are cones from their pose. Contacts never stop a plant (fastsim
semantics) but any contact deeper than the scene's collision tolerance fails the run (the outcome rules).

What this does NOT model is in FIDELITY (recorded with every run).
"""
import math
import sys

import numpy as np

from .common import FLEET, floor_props, in_rect, lower, rect_centre, zone, payload_type
from . import catalog, plan as planner

_ROOT = FLEET.parent.parent          # the crane root that holds fastsim/
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from fastsim.controller import Controller, Navigator  # noqa: E402
from fastsim.perception import ConeCamera  # noqa: E402
from fastsim.plant import GoalTracker, KinematicPlant, Limits  # noqa: E402
from fastsim.world import to_polygon, wrap  # noqa: E402

FIDELITY = ("fastsim fleet tier (scenes v1): kinematic plants with per-axis capability caps from the capplan descriptor, "
            "floor patches as speed/accel/lateral-efficiency scales and multiplicative slip noise; coupled assemblies are "
            "rigid (no coupling compliance, no hook/magnet mechanics, docking = reach the mated pose then a fixed latch "
            "time); payloads are carried kinematically (no tipping, no support or friction dynamics: support is the "
            "capplan payload set); door thresholds are NOT modelled (traversable); wheel faults change the capplan "
            "descriptor's speed bounds instantly (no torque or drag dynamics); batteries are a linear distance/time drain; "
            "perception is truth labels in a cone with 3-D box occlusion (no detector noise, no tag pose noise, no "
            "latency); the planner is a Manhattan C-space lattice (not CRANE's planner); multi-robot coordination is "
            "priority yielding plus optional aisle reservations, or none (not a WoMAPF solver); contacts are geometry and never stop a plant")

DT = 0.05
HANDLING = {"dock": 20.0, "undock": 6.0, "pickup": 6.0, "deliver": 4.0, "handoff": 12.0}   # mf.levels.HANDLING_S
BATTERY = {"per_m": 0.01, "per_s": 0.0005, "charge_per_s": 0.02}
TILE = floor_props("hard_tile")

# limits shared by every unit's tracker; the capability caps live in FleetPlant
BASE_LIMITS = dict(vmax=(0.5, 0.5, 1.5), vmin=(-0.5, -0.5, -1.5), accel=(0.6, 0.6, 2.0), decel=(-1.0, -1.0, -3.0),
                   goal_tol=(0.03, 0.05), precise_tol=(0.012, 0.03), rotate_above=0.35, rotate_wz=1.2,
                   progress_radius=0.04, progress_s=8.0, pass_radius=0.05)
RUN = {}   # per-process registry: the scene the format just parsed and the ego plant the core built


def limits_for(footprint):
    return Limits(**BASE_LIMITS, footprint=tuple(tuple(p) for p in footprint))


class FleetPlant(KinematicPlant):
    """KinematicPlant with capability caps (vx fwd/back, vy, wz), floor-patch scaling, slip noise and a holonomic
    leg (strafe): while ``override`` holds a pose (plant frame) the plant tracks it with the yaw held, ignoring the
    tracker's target (the core GoalTracker always faces its direction of travel)."""

    def __init__(self, limits, pose=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0), delay_s=(0.0, 0.0, 0.0), speed_scale=1.0,
                 slip=True, register=True):
        super().__init__(limits, pose, vel, delay_s, speed_scale)
        self.caps = {"fwd": 0.5, "back": 0.5, "left": 0.5, "right": 0.5, "ccw": 1.5, "cw": 1.5}
        self.lateral = True
        self.floor = None             # callable (x, y) in the plant frame -> floor props
        self.override = None
        self.slip, self.rng = bool(slip), None
        self.travelled = 0.0
        if register:
            RUN["ego_plant"] = self

    def set_caps(self, vmax, lateral):
        self.caps = {k: float(vmax.get(k, 0.0)) for k in ("fwd", "back", "left", "right", "ccw", "cw")}
        self.lateral = bool(lateral)

    def velocity(self, target, dt):
        x, y, yaw = self.pose
        if self.override is not None:
            mode, (tx, ty, tyaw) = self.override
            dx, dy = tx - x, ty - y
            d = math.hypot(dx, dy)
            c, s = math.cos(yaw), math.sin(yaw)
            if mode == "holo":     # translate straight at the target with the yaw held (omnidirectional units)
                sp = min(0.5, math.sqrt(2 * 0.8 * d)) if d > 1e-4 else 0.0
                bx, by = ((c * dx + s * dy) / d * sp, (-s * dx + c * dy) / d * sp) if d > 1e-4 else (0.0, 0.0)
                target = (bx, by, 2.0 * wrap(tyaw - yaw))
            else:                  # "diff": turn to the target, drive, then turn to the final yaw (no lateral)
                if d > 0.02:
                    err = wrap(math.atan2(dy, dx) - yaw)
                    if abs(err) > math.pi / 2:          # behind: back up instead of turning round
                        err = wrap(err - math.pi)
                        sgn = -1.0
                    else:
                        sgn = 1.0
                    if abs(err) > 0.35:
                        target = (0.0, 0.0, math.copysign(min(1.2, 2.0 * abs(err)), err))
                    else:
                        target = (sgn * min(0.5, math.sqrt(2 * 0.8 * d)), 0.0, 2.0 * err)
                else:
                    target = (0.0, 0.0, 2.0 * wrap(tyaw - yaw))
        f = self.floor(x, y) if self.floor else TILE
        lat_scale = f["lateral_efficiency"] / TILE["lateral_efficiency"]
        acc_scale = min(1.0, f["mu_long"] / TILE["mu_long"])
        vx, vy, wz = target
        k = self.speed_scale * f.get("speed_scale", 1.0)
        vx = max(-self.caps["back"], min(self.caps["fwd"], vx * k))
        vy = 0.0 if not self.lateral else max(-self.caps["right"], min(self.caps["left"], vy * k)) * min(1.0, lat_scale)
        wz = max(-self.caps["cw"], min(self.caps["ccw"], wz))
        out = []
        for v, want, up, down in zip(self.vel, (vx, vy, wz), self.lim.accel, self.lim.decel):
            grow = abs(want) > abs(v) and want * v >= 0
            rate = (up if grow else -down) * acc_scale
            out.append(v + max(-rate * dt, min(rate * dt, want - v)))
        if self.slip and self.rng is not None and f["slip_sigma_frac"] > 0:
            n = self.rng.normal(0.0, f["slip_sigma_frac"], 3)
            out = [v * (1.0 + e) for v, e in zip(out, n)]
        return tuple(out)

    def step(self, target, dt):
        moved = super().step(target, dt)
        self.travelled += math.hypot(*moved)
        return moved


# ---------------------------------------------------------------- geometry helpers
def module_world(unit_pose, m):
    X, Y, T = unit_pose
    c, s = math.cos(T), math.sin(T)
    return (X + c * m["x"] - s * m["y"], Y + s * m["x"] + c * m["y"], wrap(T + m["yaw"]))


def rect_poly(x, y, yaw, lx, ly):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * u - s * v, y + s * u + c * v) for u, v in
                     ((lx / 2, ly / 2), (lx / 2, -ly / 2), (-lx / 2, -ly / 2), (-lx / 2, ly / 2))])


def poly_clearance(a, b):
    """Signed clearance between two convex polygons (negative: overlap depth estimate)."""
    return float(min(to_polygon(a, b).min(), to_polygon(b, a).min()))


def segment_box_entry(p0, p1, solid):
    """3-D slab test of segment p0->p1 against a yawed box (Solid); entry parameter or -1 (port of Scenario.cs Obb)."""
    c, s = math.cos(solid.yaw), math.sin(solid.yaw)

    def local(p):
        dx, dy = p[0] - solid.cx, p[1] - solid.cy
        return (c * dx + s * dy, -s * dx + c * dy, p[2] - solid.cz)
    a, b = local(p0), local(p1)
    h = (solid.sx / 2, solid.sy / 2, solid.sz / 2)
    t0, t1 = 0.0, 1.0
    for k in range(3):
        d = b[k] - a[k]
        if abs(d) < 1e-12:
            if a[k] < -h[k] or a[k] > h[k]:
                return -1.0
            continue
        u, v = (-h[k] - a[k]) / d, (h[k] - a[k]) / d
        if u > v:
            u, v = v, u
        t0, t1 = max(t0, u), min(t1, v)
        if t0 > t1:
            return -1.0
    return t0


def faces_world(unit, mid, face):
    m = next(x for x in unit.modules if x["id"] == mid)
    mx, my, myaw = module_world(unit.pose, m)
    f = m["faces"][face]
    c, s = math.cos(myaw), math.sin(myaw)
    return (mx + c * f["x"] - s * f["y"], my + s * f["x"] + c * f["y"], wrap(myaw + f["yaw"]))


# ---------------------------------------------------------------- units
class Unit:
    def __init__(self, uid, key, modules, pose, engine, plant=None, nav=None, is_ego=False):
        self.id, self.key, self.modules, self.engine = uid, key, modules, engine
        self.is_ego = is_ego
        cat = catalog.unit(key) if key in catalog.load()["units"] else None
        self.cat = cat
        self.vmax = dict(cat["vmax"]) if cat else {}
        self.lateral = bool(cat["lateral"]) if cat else False
        self.payloads_ok = list(cat["payloads"]) if cat else []
        self.tools = list(cat["tools"]) if cat else []
        self.health = "ok"
        self.payload = None
        self.soc = 1.0
        self.busy = None
        self.dead = None             # reason the unit cannot move (fault, battery)
        self.plan_queue, self.seg = [], None
        self.waiting_for = None
        self.paused, self.leg, self.replanned_t, self.needs_plan = False, None, -1e9, False
        self.trace = []
        if plant is None:
            lim = engine.lim
            plant = FleetPlant(lim, pose=tuple(pose), register=False)
            plant.floor = engine.floor_at
            plant.rng = engine.rng
            nav = Navigator(lim, engine.clock)
            self.tracker = GoalTracker(lim, engine.gain)
        else:
            self.tracker = None
        self.plant, self.nav = plant, nav
        self.refresh_caps()

    # pose in the world frame
    @property
    def pose(self):
        return self.engine.to_world(self.plant.pose) if self.is_ego else self.plant.pose

    def shape(self):
        rects = []
        for m in self.modules:
            q = int(round(m["yaw"] / (math.pi / 2))) % 2
            lx, ly = m["size"] if q == 0 else m["size"][::-1]
            rects.append((m["x"], m["y"], lx / 2, ly / 2))
        R = max(math.hypot(abs(x) + hx, abs(y) + hy) for x, y, hx, hy in rects)
        return planner.Unitshape(rects, R, self.lateral, self.caps_now(), margin=self.engine.margin)

    def polys(self, pose=None):
        pose = pose or self.pose
        out = []
        for m in self.modules:
            x, y, yaw = module_world(pose, m)
            out.append(rect_poly(x, y, yaw, m["size"][0], m["size"][1]))
        return out

    def aabb(self, inflate=0.0):
        pts = np.vstack(self.polys())
        return (pts[:, 0].min() - inflate, pts[:, 1].min() - inflate, pts[:, 0].max() + inflate, pts[:, 1].max() + inflate)

    def front_extent(self):
        """Distance from the unit origin to its front (+x) edge."""
        return max(x + hx for x, y, hx, hy in self.shape().rects)

    def caps_now(self):
        if self.dead:
            return {k: 0.0 for k in ("fwd", "back", "left", "right", "ccw", "cw")}
        return self.vmax

    def refresh_caps(self):
        self.plant.set_caps(self.caps_now(), self.lateral)

    def moving(self):
        return self.seg is not None or bool(self.plan_queue)

    # one segment at a time through the navigator
    def start_segment(self, seg):
        e = self.engine
        x, y, yaw = self.pose
        self.plant.override = None
        mode = seg["mode"]
        if mode == "rotate":
            tgt = (x, y, seg["yaw"])
        else:
            tgt = (seg["x"], seg["y"], seg["yaw"])
        if mode == "settle":
            mode = "strafe" if self.lateral else "drive"
            if math.hypot(tgt[0] - x, tgt[1] - y) < 0.02:
                tgt, mode = (x, y, tgt[2]), "rotate"
        # every leg goes to the core Navigator as a goal (status, progress checker, the ego's goal events); the
        # plant plugin tracks it: holonomically for omnidirectional units, turn-drive-turn for the others (the core
        # GoalTracker always faces its direction of travel and corrects small offsets by turning in place)
        if mode in ("strafe", "reverse") and not self.lateral and mode == "strafe":
            mode = "drive"
        mt = e.to_map_pose(tgt) if self.is_ego else tgt
        self.nav.send([mt], precise=self.lateral, why=mode, quiet=not self.is_ego)
        self.plant.override = ("holo" if self.lateral else "diff", mt)
        self.seg = dict(seg, started=e.clock["now"], target=tgt, exec_mode=mode)

    def update_motion(self):
        """Advance the segment queue; returns 'done' | 'failed' | None (moving)."""
        e = self.engine
        if self.paused:
            return None
        if self.needs_plan:
            now = e.clock["now"]
            if now - self.replanned_t >= 2.0 and self.leg is not None:
                self.replanned_t = now
                goal, gyaw, ignore = self.leg
                segs, info = e.plan_for(self, goal, gyaw, ignore)
                if segs is not None:
                    self.plan_queue, self.needs_plan = segs, False
            return None
        if self.seg is None:
            if not self.plan_queue:
                return "done"
            if self.engine.blocked_by_traffic(self, self.plan_queue[0]):
                return None
            self.start_segment(self.plan_queue.pop(0))
            return None
        g = self.nav.goal
        if g is not None and g.done:
            self.plant.override = None
            status = g.status
            seg = self.seg
            self.seg = None
            if status != "SUCCEEDED":
                seg["retries"] = seg.get("retries", 0) + 1
                if seg["retries"] <= 2 and not self.dead:
                    self.plan_queue.insert(0, seg)
                    return None
                self.plan_queue = []
                e.emit("segment_failed", unit=self.id, mode=seg["mode"], status=status)
                return "failed"
            return None if self.plan_queue else "done"
        return None

    def pause(self):
        """Stop the live segment and keep it to resume (traffic yield)."""
        if self.seg is not None:
            seg = {k: v for k, v in self.seg.items() if k in ("mode", "x", "y", "yaw")}
            self.plan_queue.insert(0, seg)
        self.seg = None
        self.plant.override = None
        self.nav.hold()
        self.paused = True

    def stop(self):
        self.plan_queue, self.seg = [], None
        self.plant.override = None
        self.nav.hold()

    def step_own(self, now):
        """Non-ego units: two core dt steps per controller tick."""
        if self.is_ego:
            return
        for k in range(2):
            t = now + k * DT
            cmd = self.tracker.command(self.nav.goal, self.plant.pose, t, self.nav.cap)
            moved = self.plant.step(cmd, DT)
            self.nav.advance(self.plant.pose, moved, t)


# ---------------------------------------------------------------- steps
class Step:
    def __init__(self, spec, engine):
        self.spec, self.e = spec, engine
        self.id, self.kind = spec["id"], spec["kind"]
        self.after = list(spec.get("after", []))
        self.state, self.reason = "pending", None
        self.t0 = self.t1 = None
        self.phase = 0
        self.when = spec.get("when")             # e.g. {"fault": "cargo_1"}: activated only if that happens
        self.optional = self.when is not None

    def units(self):
        s = self.spec
        out = [s[k] for k in ("unit", "docker", "target", "from", "to") if k in s]
        return out + list(s.get("units", []))

    def ready(self, steps):
        if self.when and not self.e.happened(self.when):
            return False
        for a in self.after:
            st = steps[a]
            if st.state in ("pending", "running"):
                return False
            if st.state == "failed":
                self.finish("failed", "dependency %s failed" % a)
                return False
            if st.state == "skipped" and not st.optional:
                return False
        return all(self.e.unit(u) is not None and self.e.unit(u).busy in (None, self.id) for u in self.units()
                   if self.kind not in ("dock",) or True)

    def start(self):
        self.state, self.t0 = "running", self.e.clock["now"]
        for u in self.units():
            un = self.e.unit(u)
            if un is not None:
                un.busy = self.id
        self.e.emit("step_start", step=self.id, kind=self.kind)

    def finish(self, state, reason=None):
        if self.state in ("done", "failed", "skipped"):
            return
        self.state, self.reason, self.t1 = state, reason, self.e.clock["now"]
        for u in list(self.e.units.values()):
            if u.busy == self.id:
                u.busy = None
        self.e.emit("step_" + state, step=self.id, kind=self.kind, reason=reason)

    # drive helper: plan and queue; returns False (and fails the step) when no path
    def go(self, u, goal, goal_yaw=None, ignore=()):
        """Plan and queue a leg. When the leg is blocked only by other units (feasible with them removed), wait and
        retry every 2 s (the stuck rule ends a wait that never clears); otherwise the step fails."""
        now = self.e.clock["now"]
        if now < getattr(self, "next_try", -1.0):
            return False
        segs, info = self.e.plan_for(u, goal, goal_yaw, ignore)
        u.leg = (goal, goal_yaw, tuple(ignore))
        if segs is None:
            others = tuple(v.id for v in self.e.units.values() if v is not u)
            if self.e.plan_for(u, goal, goal_yaw, tuple(ignore) + others, quiet=True)[0] is not None:
                self.next_try = now + 2.0
                if not getattr(self, "waiting", False):
                    self.e.emit("wait_for_units", step=self.id, unit=u.id, reason=info)
                self.waiting = True
                return False
            self.finish("failed", "%s: %s to (%.2f, %.2f)" % (u.id, info, goal[0], goal[1]))
            return False
        self.waiting = False
        u.plan_queue = segs
        return True

    def zone_goal(self, u, zid):
        """A free point of the zone for u: the centre, else the nearest free point of a 0.1 m scan (zone yaw kept)."""
        z = zone(self.e.scn, zid)
        cx, cy = rect_centre(z["rect"])
        if self.e.point_free(u, (cx, cy)):
            return (cx, cy)
        r = z["rect"]
        pts = [(x, y) for x in np.arange(r[0] + 0.1, r[2] - 0.05, 0.1) for y in np.arange(r[1] + 0.1, r[3] - 0.05, 0.1)]
        for p in sorted(pts, key=lambda p: math.dist(p, (cx, cy))):
            if self.e.point_free(u, p):
                return (float(p[0]), float(p[1]))
        return (cx, cy)

    def update(self):
        return getattr(self, "_" + self.kind)()

    # ------------------------------------------------------------------ kinds
    def _wait(self):
        if self.e.clock["now"] - self.t0 >= float(self.spec.get("duration_s", 1.0)):
            self.finish("done")

    def _goto(self):
        u = self.e.unit(self.spec["unit"])
        legs = [(v[0], v[1], None) for v in self.spec.get("via", [])]
        if "zone" in self.spec:
            if not hasattr(self, "_zgoal"):
                self._zgoal = self.zone_goal(u, self.spec["zone"])
            cx, cy = self._zgoal
            legs.append((cx, cy, zone(self.e.scn, self.spec["zone"]).get("required_yaw", self.spec.get("yaw"))))
        else:
            legs.append(tuple(self.spec["goal"]))
        k, sub = divmod(self.phase, 2)
        if sub == 0:
            x, y, yaw = legs[k]
            if self.go(u, (x, y), yaw):
                self.phase += 1
            return
        r = u.update_motion()
        if r == "done":
            if k + 1 < len(legs):
                self.phase += 1
            else:
                self.finish("done")
        elif r == "failed":
            self.finish("failed", "%s motion failed" % u.id)

    def _approach_payload(self, u, p):
        """A pose beside the payload (deck/gripper side) clear of it."""
        px, py, pyaw = p["pose"]
        L = payload_type(p["type"])["size_lwh"]
        best = None
        for k in range(4):
            a = pyaw + k * math.pi / 2
            reach = (L[0] if k % 2 == 0 else L[1]) / 2 + u.front_extent() + 0.06
            gx, gy = px - math.cos(a) * reach, py - math.sin(a) * reach
            yaw = wrap(round(a / (math.pi / 2)) * math.pi / 2)
            dist = math.hypot(gx - u.pose[0], gy - u.pose[1])
            if best is None or dist < best[0]:
                best = (dist, (gx, gy), yaw)
        return best[1], best[2]

    def _pickup(self):
        e, u = self.e, self.e.unit(self.spec["unit"])
        p = e.payload(self.spec["payload"])
        if self.phase == 0:
            if p["carrier"] is not None:
                return self.finish("failed", "payload already carried by %s" % p["carrier"])
            if not p["known"]:
                return None   # waits for an observer; the stuck rule ends it
            spec = payload_type(p["type"])
            ok_type = p["type"] in u.payloads_ok or "lift" in u.tools
            if not ok_type:
                return self.finish("failed", "unsupported_payload: %s cannot carry %s (capplan payload set %s)" % (
                    u.id, p["type"], u.payloads_ok))
            cap = 5.0 if ("lift" in u.tools and p["type"] not in u.payloads_ok) else 8.0 * max(1, len(u.modules))
            if p["mass_kg"] > cap:
                return self.finish("failed", "overload: true mass %.1f kg > capacity %.1f kg (revealed at pickup)" % (
                    p["mass_kg"], cap))
            goal, yaw = self._approach_payload(u, p)
            if self.go(u, goal, yaw, ignore=()):
                self.phase = 1
            return
        if self.phase == 1:
            r = u.update_motion()
            if r == "failed":
                return self.finish("failed", "%s could not reach the payload" % u.id)
            if r == "done":
                self.phase, self.t_hand = 2, e.clock["now"]
            return
        if e.clock["now"] - self.t_hand >= HANDLING["pickup"]:
            e.attach(p, u)
            self.finish("done")

    def _deliver(self):
        e, u = self.e, self.e.unit(self.spec["unit"])
        p = e.payload(self.spec["payload"])
        z = zone(e.scn, self.spec["zone"])
        if self.phase == 0:
            if p["carrier"] != u.id:
                return self.finish("failed", "%s does not carry %s" % (u.id, p["id"]))
            cx, cy = self.zone_goal(u, z["id"]) if z["kind"] != "slot" else rect_centre(z["rect"])
            if self.go(u, (cx, cy), z.get("required_yaw")):
                self.phase = 1
            return
        if self.phase == 1:
            r = u.update_motion()
            if r == "failed":
                return self.finish("failed", "%s could not reach %s" % (u.id, z["id"]))
            if r == "done":
                self.phase, self.t_hand = 2, e.clock["now"]
            return
        if e.clock["now"] - self.t_hand >= HANDLING["deliver"]:
            e.release(p, placed=True)
            if in_rect(p["pose"][:2], z["rect"]):
                self.finish("done")
            else:
                self.finish("failed", "payload released outside %s" % z["id"])

    def _dock(self):
        e = self.e
        d, t = e.unit(self.spec["docker"]), e.unit(self.spec["target"])
        if self.phase == 0:
            tm, tf = self.spec.get("target_module", t.modules[0]["id"]), self.spec["target_face"]
            dm, df = self.spec.get("docker_module", d.modules[0]["id"]), self.spec["docker_face"]
            fx, fy, fyaw = faces_world(t, tm, tf)
            dmod = next(m for m in d.modules if m["id"] == dm)
            dface = dmod["faces"][df]
            # docker unit pose so its face (in the unit frame) sits on the target face, opposed
            fu = module_world((0.0, 0.0, 0.0), dmod)
            c0, s0 = math.cos(fu[2]), math.sin(fu[2])
            ffx, ffy, ffyaw = fu[0] + c0 * dface["x"] - s0 * dface["y"], fu[1] + s0 * dface["x"] + c0 * dface["y"], wrap(fu[2] + dface["yaw"])
            T = wrap(fyaw + math.pi - ffyaw)
            c, s = math.cos(T), math.sin(T)
            mx, my = fx - (c * ffx - s * ffy), fy - (s * ffx + c * ffy)
            back = float(dface.get("approach_len", 0.1)) + 0.12
            ax, ay = mx + math.cos(fyaw) * back, my + math.sin(fyaw) * back
            self.mated, self.approach = (mx, my, T), (ax, ay, T)
            if not d.lateral and df in ("left", "right"):
                return self.finish("failed", "dock_needs_lateral: %s.%s needs a sideways approach" % (d.id, df))
            # the approach corridor must be clear of everything but the two units
            if e.corridor_blocked((ax, ay), (mx, my), d, ignore=(d.id, t.id)):
                return self.finish("failed", "face_blocked: %s.%s approach corridor obstructed" % (t.id, tf))
            if self.go(d, (ax, ay), T):
                self.phase = 1
            return
        if self.phase == 1:
            r = d.update_motion()
            if r == "failed":
                return self.finish("failed", "face_blocked: %s cannot reach the approach pose of %s.%s" % (
                    d.id, t.id, self.spec["target_face"]))
            if r == "done":
                df = self.spec["docker_face"]
                mode = {"front": "drive", "rear": "reverse", "left": "strafe", "right": "strafe"}[df]
                d.plan_queue = [{"mode": mode, "x": self.mated[0], "y": self.mated[1], "yaw": self.mated[2]}]
                e.contact_exempt.add(frozenset((d.id, t.id)))
                self.phase = 2
            return
        if self.phase == 2:
            r = d.update_motion()
            if r == "failed":
                return self.finish("failed", "latch approach failed")
            if r == "done":
                self.phase, self.t_hand = 3, e.clock["now"]
            return
        if e.clock["now"] - self.t_hand >= HANDLING["dock"]:
            e.merge(d, t, self.spec["as"], self.spec.get("as_catalog"))
            self.finish("done")

    def _undock(self):
        e = self.e
        u = e.unit(self.spec["unit"])
        if self.phase == 0:
            self.phase, self.t_hand = 1, e.clock["now"]
            return
        if e.clock["now"] - self.t_hand >= HANDLING["undock"]:
            e.split(u, self.spec["part"], self.spec["as"], self.spec.get("catalogs", [None, None]))
            self.finish("done")

    def _handoff(self):
        e = self.e
        a, b = e.unit(self.spec["from"]), e.unit(self.spec["to"])
        p = e.payload(self.spec["payload"])
        if self.phase == 0:
            if p["carrier"] != a.id:
                return self.finish("failed", "%s does not carry %s" % (a.id, p["id"]))
            if p["type"] not in b.payloads_ok:
                return self.finish("failed", "unsupported_payload: %s cannot carry %s" % (b.id, p["type"]))
            ax, ay, ayaw = a.pose
            best = None
            for k in range(4):
                ang = ayaw + k * math.pi / 2
                gx, gy = ax + math.cos(ang) * 0.47, ay + math.sin(ang) * 0.47
                if e.point_free(b, (gx, gy), ignore=(a.id, b.id)):
                    dd = math.hypot(gx - b.pose[0], gy - b.pose[1])
                    if best is None or dd < best[0]:
                        best = (dd, (gx, gy), wrap(round(ang / (math.pi / 2)) * math.pi / 2))
            if best is None:
                return self.finish("failed", "no free handoff side around %s" % a.id)
            e.contact_exempt.add(frozenset((a.id, b.id)))
            if self.go(b, best[1], best[2], ignore=(a.id,)):
                self.phase = 1
            return
        if self.phase == 1:
            r = b.update_motion()
            if r == "failed":
                return self.finish("failed", "%s could not reach %s" % (b.id, a.id))
            if r == "done":
                self.phase, self.t_hand = 2, e.clock["now"]
            return
        if e.clock["now"] - self.t_hand >= HANDLING["handoff"]:
            e.release(p)
            e.attach(p, b)
            e.contact_exempt.discard(frozenset((a.id, b.id)))
            self.finish("done")

    def _observe(self):
        e = self.e
        u = e.unit(self.spec["unit"])
        p = e.payload(self.spec["payload"])
        if p["known"]:
            u.stop()
            return self.finish("done")
        vps = self.spec["viewpoints"]
        if self.phase >= 2 * len(vps):
            return self.finish("failed", "not_found: %s did not see %s from %d viewpoints" % (u.id, p["id"], len(vps)))
        k, sub = divmod(self.phase, 2)
        if sub == 0:
            vx, vy, vyaw = vps[k]
            if not self.go(u, (vx, vy), vyaw):
                self.state, self.reason = "running", None   # unreachable viewpoint: skip it
                self.phase += 2
                return
            self.phase += 1
            self.t_dw = None
            return
        r = u.update_motion()
        if r == "failed":
            self.phase += 1
        elif r == "done":
            if self.t_dw is None:
                self.t_dw = e.clock["now"]
            elif e.clock["now"] - self.t_dw >= float(self.spec.get("dwell_s", 1.5)):
                self.phase += 1

    def _charge(self):
        """0 choose a bay; 1 wait (in place) until it is free and acquire it; 2 drive in; 3 charge to target_soc;
        4 drive to then_zone (the bay is released by the engine once the unit has left it)."""
        e = self.e
        u = e.unit(self.spec["unit"])
        if self.phase == 0:
            bays = self.spec.get("bays") or [z["id"] for z in e.scn["zones"] if z["kind"] == "charging"]
            self.bay = self.spec.get("bay") or min(
                bays, key=lambda b: math.dist(rect_centre(zone(e.scn, b)["rect"]), u.pose[:2]))
            self.phase = 1
            return
        if self.phase == 1:
            if e.bays.get(self.bay) in (None, u.id):
                e.bays[self.bay] = u.id
                z = zone(e.scn, self.bay)
                cx, cy = rect_centre(z["rect"])
                e.emit("bay_acquired", unit=u.id, bay=self.bay)
                if self.go(u, (cx, cy), z.get("required_yaw")):
                    self.phase = 2
                else:
                    e.bays[self.bay] = None
            return
        if self.phase == 2:
            r = u.update_motion()
            if r == "failed":
                e.bays[self.bay] = None
                return self.finish("failed", "%s could not enter %s" % (u.id, self.bay))
            if r == "done":
                self.phase = 3
            return
        if self.phase == 3:
            u.soc = min(1.0, u.soc + BATTERY["charge_per_s"] * 0.1)
            if u.soc >= float(self.spec.get("target_soc", 0.8)):
                if u.dead == "battery_empty":
                    u.dead = None
                    u.refresh_caps()
                e.leaving[self.bay] = u.id
                then = self.spec.get("then_zone")
                if then is None:
                    return self.finish("done")
                cx, cy = rect_centre(zone(e.scn, then)["rect"])
                if self.go(u, (cx, cy), zone(e.scn, then).get("required_yaw")):
                    self.phase = 4
            return
        r = u.update_motion()
        if r == "done":
            self.finish("done")
        elif r == "failed":
            self.finish("failed", "%s could not leave %s" % (u.id, self.bay))


# ---------------------------------------------------------------- the engine (one run)
class FleetEngine:
    def __init__(self, scn, task, plan_name, world, clock, rng, ego_plant, ego_nav, emit, gain=2.0):
        self.scn, self.task, self.plan_name = scn, task, plan_name
        self.world, self.clock, self.rng, self.emit_fn = world, clock, rng, emit
        self.gain = gain
        self.margin = float(task.get("planner_margin_m", 0.025))
        self.solids = lower(scn)
        self.drive = [s for s in self.solids if s.drive_block]
        X, Y = scn["arena"]["size_xy"]
        self.grid_static = [self._aabb(s) for s in self.drive]
        self.lim = limits_for([(0.21, 0.2), (0.21, -0.2), (-0.21, -0.2), (-0.21, 0.2)])
        self.patches = scn.get("floor_patches", [])
        self.base_floor = floor_props(scn["arena"].get("floor", "hard_tile"))
        self.tol = float(task["outcomes"].get("collision_tol_m", 0.01))
        self.contact_exempt = set()
        self.events, self.contacts = [], []
        self.bays, self.leaving = {}, {}
        self.aisle_holder = {}
        self.policy = task["plans"][plan_name].get("traffic", task.get("traffic", "yield"))
        self.happenings = set()
        self.payloads = {}
        for p in scn["payloads"]:
            self.payloads[p["id"]] = {"id": p["id"], "type": p["type"], "mass_kg": float(p["mass_kg"]),
                                      "unknown_mass": bool(p.get("unknown_mass")), "pose": list(p["spawn"]["pose"]),
                                      "known": bool(p["spawn"].get("pose_known", True)), "carrier": None,
                                      "offset": None, "seen_t": 0.0 if p["spawn"].get("pose_known", True) else None,
                                      "size": payload_type(p["type"])["size_lwh"], "z0": float(p.get("pallet_height", 0.03))}
        ego = task["ego"]
        self.units = {}
        for i, su in enumerate(scn["fleet"]["units"]):
            key = su["catalog"]
            mods = []
            for m in catalog.unit(key)["modules"]:
                mods.append(dict(m, id="%s/%s" % (su["id"], m["id"]), home=su["id"]))
            if su["id"] == ego:
                u = Unit(su["id"], key, mods, su["start"], self, plant=ego_plant, nav=ego_nav, is_ego=True)
                ego_plant.floor = lambda x, y: self.floor_at(*self.to_world((x, y, 0.0))[:2])
                ego_plant.rng = rng
            else:
                u = Unit(su["id"], key, mods, su["start"], self)
            u.soc = float(su.get("soc", 1.0))
            u.priority = i
            self.units[su["id"]] = u
        for c in scn["fleet"].get("initial_couplings", []):
            pass  # pre-assembled units are catalogue assemblies (e.g. l3_carrier); couplings are recorded only
        for p in scn["payloads"]:
            if p["spawn"].get("on_unit"):
                self.attach(self.payloads[p["id"]], self.units[p["spawn"]["on_unit"]])
        self.faults = sorted(scn["fleet"].get("faults", []), key=lambda f: f["t_s"])
        steps = task["plans"][plan_name]["steps"]
        self.steps = {s["id"]: Step(s, self) for s in steps}
        self.order = [s["id"] for s in steps]
        self.cams = []
        for c in scn.get("overhead_cameras", []):
            self.cams.append(c)
        self.cones = {}
        self.payload_index = {p_id: self.world.ids.index(p_id) for p_id in self.payloads if p_id in self.world.ids}
        self.last_progress = 0.0
        self.outcome = None
        self.reason = None
        self.trace_every = 5
        self.ticks = 0
        self.traces = {}          # every unit id that ever existed -> [(t, x, y, yaw)] at 2 Hz

    # frames (the core's map frame is the ego's start pose)
    def to_world(self, pose):
        return self.world.to_world(pose)

    def to_map_pose(self, pose):
        x0, y0, yaw0 = self.world.start
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = pose[0] - x0, pose[1] - y0
        return (c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - yaw0))

    def emit(self, event, **f):
        self.emit_fn(event, **f)

    def happened(self, when):
        return any(("fault", when.get("fault")) == h for h in self.happenings)

    @staticmethod
    def _aabb(s):
        pts = np.array(s.footprint())
        return (pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max())

    def unit(self, uid):
        return self.units.get(uid)

    def payload(self, pid):
        return self.payloads[pid]

    def floor_at(self, x, y):
        f = self.base_floor
        for p in self.patches:
            if in_rect((x, y), p["rect"]):
                f = floor_props(p["friction_class"])
        return f

    # ------------------------------------------------------------------ planning
    def obstacles_for(self, u, ignore=()):
        extra = []
        for v in self.units.values():
            if v is u or v.id in ignore or v.seg is not None:
                continue
            if frozenset((u.id, v.id)) in self.contact_exempt:
                continue  # its partner (just undocked / handing off / docking): not an obstacle
            extra.append(v.aabb())
        for z in self.scn["zones"]:
            # charging bays are keep-out for every unit that does not hold them
            if z["kind"] == "charging" and self.bays.get(z["id"]) != u.id and not in_rect(u.pose[:2], z["rect"]):
                extra.append(tuple(z["rect"]))
        for p in self.payloads.values():
            if p["carrier"] is None and not p.get("placed") and p["id"] not in ignore:
                L, W, _ = p["size"]
                poly = rect_poly(p["pose"][0], p["pose"][1], p["pose"][2], L, W)
                extra.append((poly[:, 0].min(), poly[:, 1].min(), poly[:, 0].max(), poly[:, 1].max()))
        return extra

    def plan_for(self, u, goal, goal_yaw=None, ignore=(), quiet=False):
        grid = planner.Grid(self.scn["arena"]["size_xy"], self.grid_static, align=goal)
        shape = u.shape()
        x, y, yaw = u.pose
        segs, info = planner.plan(grid, shape, (x, y, yaw), goal, goal_yaw, self.obstacles_for(u, ignore))
        if quiet:
            return segs, info
        if segs is None:
            self.emit("no_path", unit=u.id, reason=info, goal=[round(goal[0], 3), round(goal[1], 3)])
            return None, info
        self.emit("plan", unit=u.id, segments=len(segs), cost_s=round(info, 2), goal=[round(goal[0], 3), round(goal[1], 3)])
        return segs, info

    def point_free(self, u, xy, ignore=()):
        grid = planner.Grid(self.scn["arena"]["size_xy"], self.grid_static)
        extra = self.obstacles_for(u, ignore)
        i, j = grid.cell(*xy)
        return any(grid.free(u.shape().rects, h, self.margin, extra)[i, j] for h in range(4))

    def corridor_blocked(self, a, b, u, ignore=()):
        """True if the straight sweep of u's footprint from a to b hits a static solid or a free payload."""
        n = max(2, int(math.dist(a, b) / 0.05))
        polys0 = u.polys((0.0, 0.0, 0.0))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        for k in range(n + 1):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            for s in self.drive:
                if math.hypot(s.cx - x, s.cy - y) > math.hypot(s.sx, s.sy) / 2 + 0.6:
                    continue
                box = np.array(s.footprint())
                for poly in polys0:
                    if poly_clearance(poly + (x, y), box) < 0.0:
                        return True
            for p in self.payloads.values():
                if p["carrier"] is None:
                    pb = rect_poly(p["pose"][0], p["pose"][1], p["pose"][2], p["size"][0], p["size"][1])
                    for poly in polys0:
                        if poly_clearance(poly + (x, y), pb) < 0.0:
                            return True
        _ = yaw
        return False

    # ------------------------------------------------------------------ traffic
    def aisles(self):
        return [z for z in self.scn["zones"] if z["kind"] == "aisle"]

    def blocked_by_traffic(self, u, seg):
        if self.policy != "reserved":
            return False
        x, y, _ = u.pose
        tx, ty = seg.get("x", x), seg.get("y", y)
        for z in self.aisles():
            r = z["rect"]
            inside_now = in_rect((x, y), r)
            enters = any(in_rect((x + (tx - x) * k / 10, y + (ty - y) * k / 10), r) for k in range(11))
            holder = self.aisle_holder.get(z["id"])
            if enters and not inside_now:
                if holder not in (None, u.id):
                    if u.waiting_for != z["id"]:
                        u.waiting_for = z["id"]
                        self.emit("aisle_wait", unit=u.id, aisle=z["id"], holder=holder)
                    return True
                self.aisle_holder[z["id"]] = u.id
                u.waiting_for = None
        return False

    def lookahead(self, u, dist=None):
        if u.seg is None:
            return None
        if dist is None:   # stopping distance at the current speed, a tick of latency and a head-on partner
            dist = 0.25 + 0.8 * math.hypot(*u.plant.vel[:2])
        x, y, _ = u.pose
        tx, ty, _ = u.seg["target"]
        d = math.hypot(tx - x, ty - y)
        if d < 0.03 or u.seg["exec_mode"] == "rotate":
            R = u.shape().R
            return [np.array([(x + R, y + R), (x + R, y - R), (x - R, y - R), (x - R, y + R)])]
        k = min(dist, d) / d
        return [p + ((tx - x) * k, (ty - y) * k) for p in u.polys()]

    def resolve_traffic(self, now):
        """Priority yielding (policies yield, reserved). For a unit whose next 0.3 m runs into another unit v:
        v moving with higher priority -> pause; v moving with lower priority -> go on only if v's own look-ahead
        hits this unit (v will pause), else pause; v waiting for a higher-priority unit -> pause; v stationary
        otherwise -> replan around it (at most every 3 s). A paused unit resumes when its way is clear.
        Policy 'none': nothing (independent motion)."""
        if self.policy == "none":
            return
        for u in sorted(self.units.values(), key=lambda v: v.priority):
            if u.paused:
                ahead = u.polys()
                if u.plan_queue:
                    nxt = u.plan_queue[0]
                    x, y, _ = u.pose
                    dx, dy = nxt.get("x", x) - x, nxt.get("y", y) - y
                    d = math.hypot(dx, dy)
                    if nxt.get("mode") == "rotate":
                        R = u.shape().R
                        ahead = [np.array([(x + R, y + R), (x + R, y - R), (x - R, y - R), (x - R, y + R)])]
                    elif d > 0.03:
                        k = min(0.3, d) / d
                        ahead = [p + (dx * k, dy * k) for p in u.polys()]
                v = self.blocker(u, ahead)
                act = "go" if v is None else self.decide(u, v)
                if act == "go":
                    u.paused = False
                elif act == "replan":
                    u.paused = False
                    self.replan(u, now)
                continue
            ahead = self.lookahead(u)
            if ahead is None:
                continue
            v = self.blocker(u, ahead)
            if v is None:
                continue
            act = self.decide(u, v)
            if act == "pause":
                u.pause()
                self.emit("yield", unit=u.id, to=v.id)
            elif act == "replan":
                self.replan(u, now)

    def decide(self, u, v):
        if v.paused:
            return "pause" if v.priority < u.priority else "replan"
        if v.moving():
            # the swept-area test is symmetric, so the lower-priority unit of a conflicting pair always pauses
            return "pause" if v.priority < u.priority else "go"
        return "replan"

    def blocker(self, u, polys):
        """The first unit whose footprint, or swept look-ahead when it is moving, comes within 0.015 m of ``polys``."""
        for v in self.units.values():
            if v is u or frozenset((u.id, v.id)) in self.contact_exempt:
                continue
            if math.dist(u.pose[:2], v.pose[:2]) > 2.5:
                continue
            area = v.polys() + ((self.lookahead(v) or []) if (v.seg is not None and not v.paused) else [])
            if min(poly_clearance(a, b) for a in polys for b in area) < 0.015:   # < the planner margin
                return v
        return None

    def replan(self, u, now):
        if u.leg is None or now - u.replanned_t < 3.0:
            if not u.paused:
                u.pause()
            return
        u.replanned_t = now
        goal, gyaw, ignore = u.leg
        u.stop()
        segs, info = self.plan_for(u, goal, gyaw, ignore)
        if segs is not None:
            u.plan_queue = segs
            self.emit("replan", unit=u.id)
        else:
            u.paused, u.needs_plan = True, True
            # deadlock breaking: a nearby waiting lower-priority unit moves on around this one
            for w in sorted(self.units.values(), key=lambda w: -w.priority):
                if w is not u and w.paused and w.priority > u.priority and w.leg is not None and \
                        math.dist(w.pose[:2], u.pose[:2]) < 1.5 and now - w.replanned_t >= 3.0:
                    w.paused = False
                    self.replan(w, now)
                    self.emit("give_way", unit=w.id, to=u.id)
                    break

    def release_aisles(self):
        for z in self.aisles():
            h = self.aisle_holder.get(z["id"])
            if h is None or h not in self.units:
                continue
            u = self.units[h]
            ax0, ay0, ax1, ay1 = u.aabb()
            r = z["rect"]
            overlap = not (ax1 < r[0] or ax0 > r[2] or ay1 < r[1] or ay0 > r[3])
            if not overlap:
                self.aisle_holder[z["id"]] = None

    # ------------------------------------------------------------------ assembly changes
    def attach(self, p, u):
        x, y, yaw = u.pose
        px, py, pyaw = p["pose"]
        c, s = math.cos(yaw), math.sin(yaw)
        if p["carrier"] is None:
            # the payload goes onto the unit's deck centre (kinematic stand-in for the lift/handoff)
            ox, oy = 0.0, 0.0
        else:
            dx, dy = px - x, py - y
            ox, oy = c * dx + s * dy, -s * dx + c * dy
        p["carrier"], p["offset"] = u.id, (ox, oy, wrap(pyaw - yaw))
        u.payload = p["id"]
        self.emit("payload_attached", payload=p["id"], unit=u.id)
        self.sync_payloads()

    def release(self, p, placed=False):
        if p["carrier"] and p["carrier"] in self.units:
            self.units[p["carrier"]].payload = None
        self.emit("payload_released", payload=p["id"], unit=p["carrier"], pose=[round(v, 3) for v in p["pose"]],
                  placed=placed)
        p["carrier"], p["offset"], p["placed"] = None, None, placed

    def sync_payloads(self):
        for p in self.payloads.values():
            if p["carrier"] is not None and p["carrier"] in self.units:
                x, y, yaw = self.units[p["carrier"]].pose
                ox, oy, oyaw = p["offset"]
                c, s = math.cos(yaw), math.sin(yaw)
                p["pose"] = [x + c * ox - s * oy, y + s * ox + c * oy, wrap(yaw + oyaw)]

    def merge(self, d, t, new_id, cat_key=None):
        lead = d if d.is_ego else t
        other = t if lead is d else d
        X, Y, T = lead.pose
        c, s = math.cos(T), math.sin(T)
        mods = []
        for u in (lead, other):
            for m in u.modules:
                mx, my, myaw = module_world(u.pose, m)
                dx, dy = mx - X, my - Y
                mods.append(dict(m, x=round(c * dx + s * dy, 4), y=round(-s * dx + c * dy, 4), yaw=wrap(myaw - T)))
        lead.modules = mods
        old_ids = (lead.id, other.id)
        carried = [p for p in self.payloads.values() if p["carrier"] in old_ids]
        for p in carried:
            self.release(p)
        other.stop()
        del self.units[other.id]
        del self.units[lead.id]
        lead.id = new_id
        if cat_key:
            cat = catalog.unit(cat_key)
            lead.key, lead.vmax, lead.lateral = cat_key, dict(cat["vmax"]), bool(cat["lateral"])
            lead.payloads_ok, lead.tools = list(cat["payloads"]), list(cat["tools"])
        else:
            lead.vmax = {k: min(lead.vmax.get(k, 0), other.vmax.get(k, 0)) for k in lead.vmax}
            lead.lateral = lead.lateral and other.lateral
            lead.payloads_ok = sorted(set(lead.payloads_ok) | set(other.payloads_ok))
            lead.tools = sorted(set(lead.tools) | set(other.tools))
        lead.soc = min(lead.soc, other.soc)
        lead.busy = None
        self.units[new_id] = lead
        lead.refresh_caps()
        for p in carried:
            self.attach_keep(p, lead)
        self.contact_exempt = {fs for fs in self.contact_exempt if not (fs & set(old_ids))}
        self.emit("docked", unit=new_id, members=list(old_ids), modules=len(mods))

    def attach_keep(self, p, u):
        x, y, yaw = u.pose
        px, py, pyaw = p["pose"]
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = px - x, py - y
        p["carrier"], p["offset"] = u.id, (c * dx + s * dy, -s * dx + c * dy, wrap(pyaw - yaw))
        u.payload = p["id"]

    def split(self, u, part, new_ids, cats):
        """part: module ids that leave as new_ids[1]; the rest becomes new_ids[0]."""
        X, Y, T = u.pose
        leave = [m for m in u.modules if m["id"] in part]
        stay = [m for m in u.modules if m["id"] not in part]
        if not leave or not stay:
            raise ValueError("undock: part must be a proper subset of %s's modules" % u.id)
        carried = [p for p in self.payloads.values() if p["carrier"] == u.id]
        for p in carried:
            self.release(p)
        # the leaving part: a new plant at its centre
        wpos = [module_world(u.pose, m) for m in leave]
        cx, cy = np.mean([w[0] for w in wpos]), np.mean([w[1] for w in wpos])
        c, s = math.cos(T), math.sin(T)
        new_mods = []
        for m, w in zip(leave, wpos):
            dx, dy = w[0] - cx, w[1] - cy
            new_mods.append(dict(m, x=round(c * dx + s * dy, 4), y=round(-s * dx + c * dy, 4), yaw=wrap(w[2] - T)))
        del self.units[u.id]
        u.modules = stay
        u.id = new_ids[0]
        k0 = cats[0] or ("carrier" if len(stay) == 1 else None)
        if k0:
            cat = catalog.unit(k0)
            u.key, u.vmax, u.lateral, u.payloads_ok, u.tools = k0, dict(cat["vmax"]), cat["lateral"], list(cat["payloads"]), list(cat["tools"])
        u.busy = None
        self.units[u.id] = u
        u.refresh_caps()
        k1 = cats[1] or ("carrier" if len(leave) == 1 else u.key)
        v = Unit(new_ids[1], k1, new_mods, (float(cx), float(cy), T), self)
        v.soc, v.priority = u.soc, u.priority + 100
        self.units[v.id] = v
        for p in carried:
            # the payload stays with the part whose deck is under its centre
            on_v = any(np.all(to_polygon(np.array([p["pose"][:2]]), poly) <= 0) for poly in v.polys())
            self.attach_keep(p, v if on_v else u)
        self.contact_exempt.add(frozenset((u.id, v.id)))
        self.emit("undocked", unit=u.id, part=v.id)

    # ------------------------------------------------------------------ perception
    def see(self):
        """Mark payloads seen by any unit camera or working overhead camera."""
        unseen = [p for p in self.payloads.values() if not p["known"]]
        if not unseen:
            return
        blockers = [s for s in self.solids if s.view_block]
        for p in unseen:
            px, py, _ = p["pose"]
            top = (px, py, p["z0"] + p["size"][2])
            seen_by = None
            for u in self.units.values():
                h, hfov, rng = max((m["camera"] for m in u.modules), key=lambda c: c[0])
                key = (hfov, rng)
                if key not in self.cones:
                    self.cones[key] = ConeCamera(self.world, hfov_deg=hfov, range_m=rng, occlusion=False)
                idx = self.payload_index.get(p["id"])
                if idx is None:
                    continue
                cam = self.cones[key]
                # fastsim's cone on the core element array (an unseen payload has never moved: its element is valid)
                _, _, inview = cam.cone(self.to_map_pose(u.pose))
                if not inview[idx]:
                    continue
                ux, uy, _ = u.pose
                eye = (ux, uy, h)
                if not any(segment_box_entry(eye, top, s) >= 0 and not s.id == p["id"] for s in blockers):
                    seen_by = u.id
                    break
            if seen_by is None:
                for c in self.cams:
                    if c["mode"] == "disabled":
                        continue
                    pos, look = c["position"], c["look_at"]
                    v = np.subtract(top, pos)
                    w = np.subtract(look, pos)
                    ang = math.acos(max(-1.0, min(1.0, float(v @ w / (np.linalg.norm(v) * np.linalg.norm(w) + 1e-12)))))
                    if np.linalg.norm(v) > c.get("range_m", 12.0) or ang > math.radians(c.get("hfov_deg", 90.0)) / 2:
                        continue
                    if not any(segment_box_entry(pos, top, s) >= 0 for s in blockers):
                        seen_by = c["id"]
                        break
            if seen_by is not None:
                p["known"], p["seen_t"] = True, self.clock["now"]
                self.emit("payload_seen", payload=p["id"], by=seen_by)

    # ------------------------------------------------------------------ contacts
    def check_contacts(self):
        for u in self.units.values():
            polys = u.polys()
            ux, uy, _ = u.pose
            for s in self.drive:
                if math.hypot(s.cx - ux, s.cy - uy) > math.hypot(s.sx, s.sy) / 2 + 1.2:
                    continue
                box = np.array(s.footprint())
                cl = min(poly_clearance(pp, box) for pp in polys)
                if cl < -self.tol:
                    self.contact(u.id, s.id, cl)
            for p in self.payloads.values():
                if p["carrier"] is None and not p.get("placed"):
                    pb = rect_poly(p["pose"][0], p["pose"][1], p["pose"][2], p["size"][0], p["size"][1])
                    cl = min(poly_clearance(pp, pb) for pp in polys)
                    if cl < -self.tol:
                        self.contact(u.id, p["id"], cl)
        ids = sorted(self.units)
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                if frozenset((a, b)) in self.contact_exempt:
                    continue
                ua, ub = self.units[a], self.units[b]
                if math.dist(ua.pose[:2], ub.pose[:2]) > 2.0:
                    continue
                cl = min(poly_clearance(pa, pb) for pa in ua.polys() for pb in ub.polys())
                if cl < -self.tol:
                    self.contact(a, b, cl)

    def contact(self, a, b, cl):
        key = (a, b)
        if any(c["pair"] == list(key) and self.clock["now"] - c["t"] < 2.0 for c in self.contacts):
            return
        self.contacts.append({"t": self.clock["now"], "pair": [a, b], "clearance_m": round(cl, 4)})
        self.emit("collision", unit=a, other=b, clearance_m=round(cl, 4))

    # ------------------------------------------------------------------ the tick
    def tick(self):
        now = self.clock["now"]
        self.ticks += 1
        for f in self.faults:
            if not f.get("_done") and now >= f["t_s"]:
                f["_done"] = True
                self.apply_fault(f)
        for u in self.units.values():
            u.step_own(now)
            moved = u.plant.travelled - getattr(u, "_last_travel", 0.0)
            u._last_travel = u.plant.travelled
            u.soc -= BATTERY["per_m"] * moved + BATTERY["per_s"] * 0.1
            if u.soc <= 0 and u.dead is None:
                u.soc, u.dead = 0.0, "battery_empty"
                u.refresh_caps()
                self.emit("battery_empty", unit=u.id)
            if self.ticks % self.trace_every == 0:
                x, y, yaw = u.pose
                u.trace.append((round(now, 2), round(x, 3), round(y, 3), round(yaw, 3)))
                self.traces.setdefault(u.id, []).append(u.trace[-1])
        self.sync_payloads()
        self.see()
        self.check_contacts()
        self.release_aisles()
        self.resolve_traffic(now)
        for bay, uid in list(self.leaving.items()):
            u = self.units.get(uid)
            r = zone(self.scn, bay)["rect"]
            if u is None or not any(in_rect(c, r) for c in np.vstack(u.polys())):
                self.bays[bay] = None
                del self.leaving[bay]
                self.emit("bay_released", unit=uid, bay=bay)
        progressed = False
        for sid in self.order:
            st = self.steps[sid]
            if st.state == "pending" and st.ready(self.steps):
                st.start()
                progressed = True
            if st.state == "running":
                before = (st.phase, st.state)
                st.update()
                progressed |= (st.phase, st.state) != before
        if progressed or any(u.moving() and math.hypot(*u.plant.vel[:2]) > 0.02 for u in self.units.values()):
            self.last_progress = now
        return self.judge(now)

    def apply_fault(self, f):
        uid = next((u.id for u in self.units.values() if any(m["home"] == f["unit"] for m in u.modules)), None)
        if uid is None:
            return
        u = self.units[uid]
        variant = "%s_%s" % (f["target"].replace("wheel_", ""), f["mode"])
        cat = catalog.unit(u.key)
        hv = cat.get("health_variants", {}).get(variant)
        if hv is None:   # assemblies: conservative rule (no descriptor for this health)
            hv = {"vmax": {k: (0.0 if f["mode"] == "dead_locked" else v * 0.58) for k, v in u.vmax.items()},
                  "lateral": u.lateral and f["mode"] != "dead_locked", "mobility_class": "assumed"}
        u.vmax, u.lateral = dict(hv["vmax"]), bool(hv["lateral"])
        if max(u.vmax.get("fwd", 0), u.vmax.get("back", 0)) <= 1e-6:
            u.dead = "fault:%s:%s" % (f["target"], f["mode"])
        u.refresh_caps()
        self.happenings.add(("fault", f["unit"]))
        self.emit("fault", unit=u.id, module=f.get("module", "m0"), target=f["target"], mode=f["mode"],
                  vmax=hv["vmax"], mobility=hv.get("mobility_class"))
        for st in self.steps.values():
            if st.state == "running" and st.spec.get("abort_on_fault") == f["unit"]:
                for un in st.units():
                    if self.unit(un):
                        self.unit(un).stop()
                st.finish("skipped", "aborted on fault of %s" % f["unit"])

    def judge(self, now):
        oc = self.task["outcomes"]
        fail_on = set(oc.get("fail_on", ["collision", "deadline", "stuck", "step_failed"]))
        if "collision" in fail_on and self.contacts:
            c = self.contacts[0]
            return self.end("FAILED", "collision %s/%s at t=%.1f (%.3f m)" % (c["pair"][0], c["pair"][1], c["t"], c["clearance_m"]))
        for sid in self.order:
            st = self.steps[sid]
            if st.state == "failed" and "step_failed" in fail_on:
                return self.end("FAILED", "step %s: %s" % (sid, st.reason))
        req = [s for s in self.steps.values() if not s.optional]
        if all(s.state in ("done", "skipped") for s in req) and all(
                s.state != "running" for s in self.steps.values()):
            ok, why = self.success_rules()
            return self.end("SUCCEEDED" if ok else "FAILED", why)
        if "deadline" in fail_on and now > float(self.task["deadline_s"]):
            return self.end("FAILED", "deadline %.0f s" % float(self.task["deadline_s"]))
        if "stuck" in fail_on and now - self.last_progress > float(self.task.get("stuck_s", 60.0)):
            running = [s.id for s in self.steps.values() if s.state == "running"]
            dead = [u.id + ":" + u.dead for u in self.units.values() if u.dead]
            return self.end("FAILED", "stuck %.0f s (running %s; %s)" % (now - self.last_progress, running, dead))
        return None

    def success_rules(self):
        for rule in self.task["outcomes"]["success"]:
            if "payload_in_zone" in rule:
                r = rule["payload_in_zone"]
                p = self.payloads[r["payload"]]
                if p["carrier"] is not None or not in_rect(p["pose"][:2], zone(self.scn, r["zone"])["rect"]):
                    return False, "payload %s not at rest in %s" % (p["id"], r["zone"])
            if "unit_in_zone" in rule:
                r = rule["unit_in_zone"]
                u = self.units.get(r["unit"])
                if u is None or not in_rect(u.pose[:2], zone(self.scn, r["zone"])["rect"]):
                    return False, "unit %s not in %s" % (r["unit"], r["zone"])
            if "min_soc" in rule:
                r = rule["min_soc"]
                lows = [u.id for u in self.units.values() if u.soc < r]
                if lows:
                    return False, "soc below %.2f: %s" % (r, lows)
        return True, "all success rules hold"

    def end(self, outcome, reason):
        self.outcome, self.reason = outcome, reason
        self.emit("outcome", outcome=outcome, reason=reason)
        return outcome

    def summary(self):
        return {"outcome": self.outcome, "reason": self.reason, "plan": self.plan_name, "t_end": self.clock["now"],
                "steps": [{"id": s.id, "kind": s.kind, "state": s.state, "reason": s.reason,
                           "t0": s.t0, "t1": s.t1} for s in (self.steps[i] for i in self.order)],
                "contacts": self.contacts,
                "payloads": {p["id"]: {"pose": [round(v, 4) for v in p["pose"]], "carrier": p["carrier"],
                                       "seen_t": p["seen_t"], "mass_kg": p["mass_kg"]} for p in self.payloads.values()},
                "units": {u.id: {"key": u.key, "modules": [m["id"] for m in u.modules], "soc": round(u.soc, 4),
                                 "dead": u.dead, "pose": [round(v, 4) for v in u.pose], "trace": u.trace}
                          for u in self.units.values()}}


class FleetController(Controller):
    def __init__(self, engine):
        self.e = engine

    def tick(self, obs):
        return self.e.tick()

    def close(self):
        for u in self.e.units.values():
            u.plant.override = None
