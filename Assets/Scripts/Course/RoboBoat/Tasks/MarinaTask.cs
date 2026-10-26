using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class MarinaParams : ITaskParams {
        [CourseParam("D", "red slips kept free of moored boats (room for a docked supply hull)")] public ParamRange free_red_min = ParamRange.Int(1, 1);
        [CourseParam("D", "chance a slip shows green")] public ParamRange green_chance = ParamRange.Fixed(0.5, "p");
        [CourseParam("D", "near pier to the entry line")] public ParamRange lateral_offset = ParamRange.Uniform(4, 10);
        [CourseParam("D", "slip numbers ascend from or toward the shore")] public ParamRange numbering = ParamRange.Choice("from_shore", "toward_shore");
        [CourseParam("H", "p.54 0-2 stationary boats in red slips")] public ParamRange occupied = ParamRange.Int(0, 2);
        [CourseParam("H", "p.55 fig 340 in")] public ParamRange pier_length = ParamRange.Fixed(8.64);
        [CourseParam("D", "spine shore end to the edge")] public ParamRange shore_gap = ParamRange.Uniform(0, 2);
        [CourseParam("H", "p.55 fig 60 in")] public ParamRange slip_clear = ParamRange.Fixed(1.52);
        [CourseParam("D", "20 in cube")] public ParamRange spine_width = ParamRange.Fixed(0.508);
        [CourseParam("D", "tine-tip gap; fig p.55 about 3.9 m")] public ParamRange tine_gap = ParamRange.Uniform(3.0, 8.0);
        [CourseParam("H", "p.56 table 4-7 ft vs fig 120 in")] public ParamRange tine_length = ParamRange.Uniform(1.22, 3.05);
        [CourseParam("D", "20 in cube")] public ParamRange tine_width = ParamRange.Fixed(0.508);
        [CourseParam("D", "pier axis off the shore normal; the marina turns rigidly")] public ParamRange yaw_deg = ParamRange.Angle(-90, 90);
    }

    /// <summary>Task 5, navigate the marina (H pp.54–56): two piers (N, S) of three slips
    /// running out from the shore on side S, slips opening toward each other, a number sign
    /// and a red/green indicator per slip, red slips possibly holding a boat. The target is
    /// the lowest green number, unique across both piers.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Marina")]
    public sealed class MarinaTask : CourseTaskDefinition<MarinaParams> {
        public override string TaskType => "marina";
        public override string Cite => "H pp.54-56";

        static double HalfSpan(ParamValues p) => p.Num(nameof(MarinaParams.tine_gap)) / 2 +
            p.Num(nameof(MarinaParams.tine_length)) + p.Num(nameof(MarinaParams.spine_width));

        /// <summary>The marina turns rigidly by yaw_deg about its shore end; it then slides so its
        /// shore-most spine corner sits shore_gap off the shore and its near side lateral_offset
        /// off the entry line. At yaw 0 both spine roots lie on the shore.</summary>
        public override Pose2 Place(PlacementContext c) {
            var p = c.Params;
            double yaw = Layouts.Deg(p.Num(nameof(MarinaParams.yaw_deg))), half = HalfSpan(p), length = p.Num(nameof(MarinaParams.pier_length));
            double cos = Math.Cos(yaw), sin = Math.Sin(yaw);
            double minX = double.MaxValue, minY = double.MaxValue, maxY = double.MinValue;
            foreach (var (u, v) in new[] { (0.0, half), (0.0, -half), (length, half), (length, -half) }) {
                minX = Math.Min(minX, u * cos - v * sin);
                double y = u * sin + v * cos;
                minY = Math.Min(minY, y); maxY = Math.Max(maxY, y);
            }
            double lateral = p.Num(nameof(MarinaParams.lateral_offset));
            double anchorY = c.Side > 0 ? lateral - minY : -lateral - maxY;
            // At least 5 mm off, so 1 mm truth rounding never puts a turned spine corner past the edge.
            double gap = yaw == 0 ? p.Num(nameof(MarinaParams.shore_gap)) : Math.Max(p.Num(nameof(MarinaParams.shore_gap)), 0.005);
            return new Pose2(c.Boundary.ShoreX + gap - minX, anchorY, yaw);
        }

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            var rng = c.Rng;
            string id = TaskId;
            double length = p.Num(nameof(MarinaParams.pier_length)), clear = p.Num(nameof(MarinaParams.slip_clear));
            double tine = p.Num(nameof(MarinaParams.tine_length)), tw = p.Num(nameof(MarinaParams.tine_width));
            double sw = p.Num(nameof(MarinaParams.spine_width)), gap = p.Num(nameof(MarinaParams.tine_gap));
            bool fromShore = p.Str(nameof(MarinaParams.numbering)) == "from_shore";
            double x0 = (length - (4 * tw + 3 * clear)) / 2 + tw / 2;

            // Colours: resample until ≥1 green and the lowest green number is unique (H p.54, D).
            var colors = new string[6];
            int freeRed = p.Int(nameof(MarinaParams.free_red_min));
            for (int attempt = 0; attempt < 100; attempt++) {
                for (int i = 0; i < 6; i++) colors[i] = rng.Chance(p.Num(nameof(MarinaParams.green_chance))) ? "green" : "red";
                var greens = Enumerable.Range(0, 6).Where(i => colors[i] == "green").Select(i => Number(i % 3, fromShore)).ToList();
                if (greens.Count > 0 && greens.Count(n => n == greens.Min()) == 1 && 6 - greens.Count >= freeRed) break;
            }
            var reds = Enumerable.Range(0, 6).Where(i => colors[i] == "red").ToList();
            var occupied = new HashSet<int>();
            for (int k = 0; k < p.Int(nameof(MarinaParams.occupied)) && reds.Count > freeRed; k++) {
                int pick = reds[rng.Int(0, reds.Count - 1)];
                reds.Remove(pick);
                occupied.Add(pick);
            }

            var corners = new List<P2>();
            var bays = new List<BaySpec>();
            foreach (var (pier, sign) in new[] { ("N", -1.0), ("S", 1.0) }) {
                double spineY = sign * (gap / 2 + tine + sw / 2), slipY = sign * (gap / 2 + tine / 2);
                var spine = l.Add($"{id}.{pier}.spine", "pier_spine", Kinds.DockSpine, c.Anchor.At(length / 2, spineY), c.Anchor.Yaw);
                spine.Length = length; spine.Width = sw;
                Corners(corners, spine);
                for (int t = 0; t < 4; t++) {
                    var o = l.Add($"{id}.{pier}.tine{t + 1}", "pier_tine", Kinds.DockTine, c.Anchor.At(x0 + t * (tw + clear), slipY),
                        c.Anchor.YawOf(Math.PI / 2));
                    o.Length = tine; o.Width = tw;
                    Corners(corners, o);
                }
                for (int s = 0; s < 3; s++) {
                    int index = (pier == "N" ? 0 : 3) + s, number = Number(s, fromShore);
                    double x = x0 + s * (tw + clear) + (tw + clear) / 2;
                    string bay = $"{id}.{pier}{s + 1}";
                    var signObj = l.Add($"{bay}.sign", "dock_sign", Kinds.DockSign, c.Anchor.At(x, spineY), c.Anchor.YawOf(-sign * Math.PI / 2));
                    signObj.Z = 1.0;
                    Layouts.State(l, signObj, "sign_face", Programs.Fixed(number.ToString()));
                    var light = l.Add($"{bay}.indicator", "dock_indicator", Kinds.LightBeacon, c.Anchor.At(x + 0.45, spineY), signObj.Yaw);
                    light.Z = 0.5;
                    Layouts.State(l, light, "indicator", Programs.Fixed(colors[index]));
                    string vessel = null;
                    if (occupied.Contains(index)) {
                        vessel = l.Add($"{bay}.vessel", "vessel", Kinds.MooredBoat, c.Anchor.At(x, slipY), c.Anchor.YawOf(sign * Math.PI / 2)).Id;
                    }
                    bays.Add(new BaySpec {
                        Id = bay, Pier = pier, Slip = s + 1, Number = number, At = c.Anchor.At(x, slipY),
                        Yaw = c.Anchor.YawOf(-sign * Math.PI / 2), Width = clear, Depth = tine, SignId = signObj.Id,
                        IndicatorId = light.Id, IndicatorColor = colors[index], Occupied = vessel != null, Vessel = vessel
                    });
                }
            }
            l.Bays.AddRange(bays);
            var green = bays.Where(b => b.IndicatorColor == "green").OrderBy(b => b.Number).ToList();
            var target = green.FirstOrDefault();
            l.Semantics.Set("target", target == null ? null : new JObj().Set("pier", target.Pier).Set("slip", (double)target.Number).Set("bay", target.Id))
                .Set("tiers", new JObj()
                    .Set("unoccupied", bays.Where(b => !b.Occupied).Select(b => (object)b.Id).ToList())
                    .Set("green", green.Select(b => (object)b.Id).ToList())
                    .Set("lowest_green", target?.Id))
                .Set("numbering", fromShore ? "from_shore" : "toward_shore");
            l.Anchor("approach", c.Anchor.At(length + 2.5, 0));
            l.Anchor("outer_end", c.Anchor.At(length, 0));
            l.Anchor("tips_N", c.Anchor.At(length / 2, -gap / 2));
            l.Anchor("tips_S", c.Anchor.At(length / 2, gap / 2));
            l.Footprint = Geometry.Hull(corners, 1.0);
            return l;
        }

        static int Number(int slipFromShore, bool fromShore) => fromShore ? slipFromShore + 1 : 3 - slipFromShore;

        static void Corners(List<P2> corners, PlacedObject box) {
            P2 f = P2.Dir(box.Yaw), left = f.Left;
            foreach (var (u, v) in new[] { (1, 1), (1, -1), (-1, 1), (-1, -1) })
                corners.Add(box.At + f * (u * box.Length / 2) + left * (v * box.Width / 2));
        }
    }
}
