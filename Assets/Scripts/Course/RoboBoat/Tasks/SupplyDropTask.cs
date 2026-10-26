using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class SupplyDropParams : ITaskParams {
        [CourseParam("H", "p.51 <= 3")] public ParamRange ball_hulls = ParamRange.Int(1, 3);
        [CourseParam("D", "hulls docked in free red slips (never green, never in or beside a moored boat's slip); the rest scattered among the tasks")] public ParamRange min_docked = ParamRange.Int(1, 2);
        [CourseParam("D", "a free hull's nearest other-task object at most this far (T4.near_task)")] public ParamRange near_max = ParamRange.Fixed(25);
        [CourseParam("D", "free hull from the object it is scattered by")] public ParamRange reach = ParamRange.Uniform(4, 20);
        [CourseParam("D", "free hulls at least this far apart, centre to centre (T4.spacing)")] public ParamRange spacing = ParamRange.Fixed(8);
        [CourseParam("H/S", "yellow (H) vs orange (S) kept")] public ParamRange water_hull_color = ParamRange.Choice("yellow", "orange");
        [CourseParam("H", "p.51 <= 3")] public ParamRange water_hulls = ParamRange.Int(1, 3);
    }

    /// <summary>Task 4, supply drop (H p.51): up to three water-target and three ball-target
    /// hulls, min_docked of them in free red slips and the rest scattered one by one among the
    /// other tasks: each free hull near a different task where it can be (T4.scatter), within
    /// near_max of another task's object and spacing from the other free hulls. The free hulls
    /// have no common footprint; each keeps its own clearance from objects and footprints.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Supply Drop")]
    public sealed class SupplyDropTask : CourseTaskDefinition<SupplyDropParams> {
        public override string TaskType => "supply_drop";
        public override string Cite => "H p.51";
        const double Margin = 1.5;  // a free hull's own footprint margin (D)

        /// <summary>No region: the hulls scatter in Generate.</summary>
        public override Pose2 Place(PlacementContext c) => new Pose2(0, 0, 0);

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout { Scattered = true };
            var p = c.Params;
            var rng = c.Rng;
            string id = TaskId;
            string waterKind = p.Str(nameof(SupplyDropParams.water_hull_color)) == "orange" ? Kinds.HullWaterOrange : Kinds.HullWaterYellow;
            double spacing = p.Num(nameof(SupplyDropParams.spacing)), nearMax = p.Num(nameof(SupplyDropParams.near_max));
            var water = new List<PlacedObject>();
            var ball = new List<PlacedObject>();
            var slips = new List<object>();

            // Docked hulls first: random free red slips (a red slip is never the target, and a
            // green slip never holds a stationary boat, H p.54), not holding or beside a moored
            // boat on the same pier, each a random kind with quota left.
            var marina = c.Placement.ByType("marina");
            var bays = marina?.Bays ?? new List<BaySpec>();
            var free = bays.Where(b => b.IndicatorColor == "red" && !b.Occupied &&
                                       !bays.Any(o => o.Occupied && o.Pier == b.Pier && Math.Abs(o.Slip - b.Slip) == 1)).ToList();
            int waterCount = p.Int(nameof(SupplyDropParams.water_hulls)), ballCount = p.Int(nameof(SupplyDropParams.ball_hulls));
            for (int k = 0; k < p.Int(nameof(SupplyDropParams.min_docked)) && free.Count > 0; k++) {
                var bay = free[rng.Int(0, free.Count - 1)];
                free.Remove(bay);
                bool wet = water.Count < waterCount && (ball.Count >= ballCount || rng.Chance(0.5));
                if (!wet && ball.Count >= ballCount) break;
                var list = wet ? water : ball;
                list.Add(l.Add($"{id}.{(wet ? "water" : "ball")}{list.Count + 1}", "slip_hull", wet ? waterKind : Kinds.HullBall, bay.At, bay.Yaw + Math.PI));
                slips.Add(bay.Id);
            }

            // Free hulls: each by an object of a task not yet used (round robin over a shuffled
            // task order), reach off it, keeping spacing, near_max and free water.
            var tasks = c.Placement.Placed.Keys.OrderBy(k => k, StringComparer.Ordinal).ToList();
            for (int i = tasks.Count - 1; i > 0; i--) { int j = rng.Int(0, i); (tasks[i], tasks[j]) = (tasks[j], tasks[i]); }
            var others = c.Placement.Placed.Values.SelectMany(t => t.Objects).ToList();
            P2 start = c.Placement.ByType("entry_gates")?.Anchors["start"] is JObj st ? new P2(st.Num("x"), st.Num("y")) : new P2(0, 0);
            var freeHulls = new List<P2>();
            var reachRange = new SupplyDropParams().reach;
            int next = 0;
            foreach (var (name, kind, count, list) in new[] {
                         ("water", waterKind, waterCount, water), ("ball", Kinds.HullBall, ballCount, ball) }) {
                while (list.Count < count && tasks.Count > 0) {
                    double radius = c.Catalog.Radius(kind);
                    P2? at = null;
                    for (int k = 0; k < 300 && at == null; k++) {
                        var near = c.Placement.Placed[tasks[(next + k / 60) % tasks.Count]].Objects;
                        if (near.Count == 0) continue;
                        var by = near[rng.Int(0, near.Count - 1)];
                        double d = k == 0 ? p.Num(nameof(SupplyDropParams.reach)) : rng.Range(reachRange.min, reachRange.max);
                        P2 q = by.At + P2.Dir(rng.Range(-Math.PI, Math.PI)) * d;
                        if (freeHulls.Any(h => P2.Dist(h, q) < spacing + 0.01) || P2.Dist(q, start) < 4) continue;
                        if (!others.Any(o => P2.Dist(o.At, q) <= nearMax - 0.01)) continue;
                        if (!c.Occupancy.Free(q, radius + 0.5, 1.5) || !c.Occupancy.FootprintFree(Square(q, radius + Margin))) continue;
                        at = q;
                    }
                    if (at == null) break;
                    next++;
                    freeHulls.Add(at.Value);
                    list.Add(l.Add($"{id}.{name}{list.Count + 1}", "hull", kind, at.Value, rng.Range(-Math.PI, Math.PI)));
                }
            }
            l.Footprint = new List<P2>();
            // Approach: 2.5 m off the first free hull, away from its nearest object; with only
            // docked hulls, in the aisle off the first docked slip's mouth.
            if (freeHulls.Count > 0) {
                P2 first = freeHulls[0];
                P2 away = others.Count > 0 ? first - others.OrderBy(o => P2.Dist(o.At, first)).First().At : new P2(1, 0);
                l.Anchor("approach", first + (away.Length > 1e-6 ? away * (1 / away.Length) : new P2(1, 0)) * 2.5);
            }
            else if (slips.Count > 0) {
                var bay = bays.First(b => b.Id == (string)slips[0]);
                l.Anchor("approach", bay.At + P2.Dir(bay.Yaw) * (bay.Depth / 2 + 1.5));
            }
            l.Semantics.Set("water", Layouts.IdList(water)).Set("ball", Layouts.IdList(ball))
                .Set("water_color", p.Str(nameof(SupplyDropParams.water_hull_color))).Set("slip", slips.Count > 0 ? slips[0] : null).Set("slips", slips);
            return l;
        }

        static List<P2> Square(P2 q, double r) =>
            new List<P2> { q + new P2(-r, -r), q + new P2(r, -r), q + new P2(r, r), q + new P2(-r, r) };
    }
}
