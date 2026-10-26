using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class SupplyDropParams : ITaskParams {
        [CourseParam("H", "p.51 <= 3")] public ParamRange ball_hulls = ParamRange.Int(1, 3);
        [CourseParam("D", "between hulls")] public ParamRange clearance = ParamRange.Fixed(3);
        [CourseParam("D", "hulls docked in free red slips (never green, never beside a moored boat); the rest in free water")] public ParamRange min_docked = ParamRange.Int(1, 2);
        [CourseParam("D", "cluster radius")] public ParamRange spread = ParamRange.Uniform(4, 7);
        [CourseParam("H/S", "yellow (H) vs orange (S) kept")] public ParamRange water_hull_color = ParamRange.Choice("yellow", "orange");
        [CourseParam("H", "p.51 <= 3")] public ParamRange water_hulls = ParamRange.Int(1, 3);
    }

    /// <summary>Task 4, supply drop (H p.51): up to three water-target and three ball-target
    /// hulls in free water on the side away from S, and min_docked of them in free red slips.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Supply Drop")]
    public sealed class SupplyDropTask : CourseTaskDefinition<SupplyDropParams> {
        public override string TaskType => "supply_drop";
        public override string Cite => "H p.51";

        public override Pose2 Place(PlacementContext c) {
            // Reserve the square the hulls and their footprint margin can reach.
            double r = c.Params.Num(nameof(SupplyDropParams.spread)) * Math.Sqrt(2) + CourseGenerator.FootprintMargin + 0.6;
            for (int i = 0; i < 200; i++) {
                var at = new P2(c.Rng.Range(5, 90), -c.Side * c.Rng.Range(6, c.Boundary.HalfWidth - r));
                var square = new List<P2> { at + new P2(-r, -r), at + new P2(r, -r), at + new P2(r, r), at + new P2(-r, r) };
                if (c.Boundary.Contains(at, r) && c.Occupancy.FootprintFree(square)) return new Pose2(at.X, at.Y, 0);
            }
            return new Pose2(40, -c.Side * 30, 0);  // rejected by the footprint row, then re-forked
        }

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            var rng = c.Rng;
            string id = TaskId;
            string waterKind = p.Str(nameof(SupplyDropParams.water_hull_color)) == "orange" ? Kinds.HullWaterOrange : Kinds.HullWaterYellow;
            double spread = p.Num(nameof(SupplyDropParams.spread)), spacing = p.Num(nameof(SupplyDropParams.clearance));
            var keep = new List<(P2, double)>();
            var water = new List<PlacedObject>();
            var ball = new List<PlacedObject>();
            var slips = new List<object>();

            // Docked hulls first: random free red slips (a red slip is never the target, and a
            // green slip never holds a stationary boat, H p.54), each a random kind with quota left.
            var marina = c.Placement.ByType("marina");
            var free = marina?.Bays.Where(b => b.IndicatorColor == "red" && !b.Occupied).ToList() ?? new List<BaySpec>();
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
            foreach (var (name, kind, count, list) in new[] {
                         ("water", waterKind, waterCount, water), ("ball", Kinds.HullBall, ballCount, ball) }) {
                while (list.Count < count) {
                    P2? at = Layouts.Scatter(rng, c.Anchor, spread, spread, c.Occupancy, keep, c.Catalog.Radius(kind));
                    if (at == null) break;
                    keep.Add((at.Value, spacing));
                    list.Add(l.Add($"{id}.{name}{list.Count + 1}", "hull", kind, at.Value, rng.Range(-Math.PI, Math.PI)));
                }
            }
            var scattered = l.Objects.Where(o => o.Role == "hull").Select(o => o.At).ToList();
            l.Footprint = scattered.Count > 0 ? Geometry.Hull(scattered, CourseGenerator.FootprintMargin) : new List<P2>();
            l.Anchor("approach", c.Anchor.At(-spread - 2, 0));
            l.Semantics.Set("water", Layouts.IdList(water)).Set("ball", Layouts.IdList(ball))
                .Set("water_color", p.Str(nameof(SupplyDropParams.water_hull_color))).Set("slip", slips.Count > 0 ? slips[0] : null).Set("slips", slips);
            return l;
        }
    }
}
