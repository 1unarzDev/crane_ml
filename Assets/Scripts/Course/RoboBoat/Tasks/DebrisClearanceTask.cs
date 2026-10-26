using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class DebrisClearanceParams : ITaskParams {
        [CourseParam("D", "bend away from side S, or either way")] public ParamRange bend = ParamRange.Choice("away", "either");
        [CourseParam("D")] public ParamRange channel_blacks = ParamRange.Int(0, 2);
        [CourseParam("D")] public ParamRange channel_gates = ParamRange.Int(2, 5);
        [CourseParam("D", "1/m")] public ParamRange curvature = ParamRange.Uniform(0, 0.04, "1/m");
        [CourseParam("D")] public ParamRange field_blacks = ParamRange.Int(3, 8);
        [CourseParam("D", "last gate to field edge")] public ParamRange field_gap = ParamRange.Uniform(3, 6);
        [CourseParam("D", "square side")] public ParamRange field_size = ParamRange.Uniform(15, 30);
        [CourseParam("D", "entry gate 2 to first channel gate")] public ParamRange gap_after_entry = ParamRange.Uniform(3, 8);
        [CourseParam("D", "channel heading off entry gate 2's normal (heading continuity)")] public ParamRange heading_deg = ParamRange.Uniform(-15, 15, "deg");
        [CourseParam("H", "p.48 6-10 ft, read as c-c (D)")] public ParamRange gate_cc = ParamRange.Uniform(1.83, 3.05);
        [CourseParam("D")] public ParamRange gate_spacing = ParamRange.Uniform(3, 8);
        [CourseParam("H", "p.47 >= 1 each; count D")] public ParamRange green_beacons = ParamRange.Int(1, 2);
        [CourseParam("H", "p.47 >= 1 each; count D")] public ParamRange red_beacons = ParamRange.Int(1, 2);
    }

    /// <summary>Task 2, debris clearance (H pp.47–49): a channel of A-0 gates past entry
    /// gate 2 that bends away from side S, then a field of black debris and red/green light
    /// buoys (rescue green with a full circle, avoid red).</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Debris Clearance")]
    public sealed class DebrisClearanceTask : CourseTaskDefinition<DebrisClearanceParams> {
        public override string TaskType => "debris_clearance";
        public override string Cite => "H pp.47-49";

        public override Pose2 Place(PlacementContext c) {
            var entry = c.ByType("entry_gates") ?? throw new InvalidOperationException("debris_clearance follows entry_gates");
            var g2 = entry.Gates.First(g => g.Order == 2);
            // A turned first gate swings a post back by cc/2·|sin h| and its 3 m lead-in (P1) sideways
            // by 3·|sin h|; stand off that much more so both stay clear of entry gate 2.
            double h = Layouts.Deg(c.Params.Num(nameof(DebrisClearanceParams.heading_deg)));
            double gap = c.Params.Num(nameof(DebrisClearanceParams.gap_after_entry)) + (c.Params.Num(nameof(DebrisClearanceParams.gate_cc)) / 2 + 3) * Math.Abs(Math.Sin(h));
            P2 at = Layouts.Mid(entry, g2) + P2.Dir(g2.NormalYaw) * gap;
            return new Pose2(at.X, at.Y, g2.NormalYaw + h);
        }

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            string id = TaskId;
            var rng = c.Rng;
            int n = p.Int(nameof(DebrisClearanceParams.channel_gates));
            double spacing = p.Num(nameof(DebrisClearanceParams.gate_spacing)), cc = p.Num(nameof(DebrisClearanceParams.gate_cc));
            double bend = p.Str(nameof(DebrisClearanceParams.bend)) == "away" ? -c.Side : rng.Sign();
            double k = bend * p.Num(nameof(DebrisClearanceParams.curvature));

            // Centreline: an arc of curvature k from the anchor, heading k·s at arc length s.
            Pose2 At(double s) {
                double th = k * s;
                double x = Math.Abs(k) < 1e-9 ? s : Math.Sin(th) / k, y = Math.Abs(k) < 1e-9 ? 0 : (1 - Math.Cos(th)) / k;
                P2 w = c.Anchor.At(x, y);
                return new Pose2(w.X, w.Y, c.Anchor.YawOf(th));
            }

            var mids = new List<Pose2>();
            for (int i = 0; i < n; i++) {
                Pose2 g = At(i * spacing);
                mids.Add(g);
                Layouts.Gate(l, id, $"gate{i + 1}", i + 1, g.Position, g.Yaw, cc, Kinds.A0Red, Kinds.A0Green);
            }
            int blacks = n > 1 ? p.Int(nameof(DebrisClearanceParams.channel_blacks)) : 0;
            for (int b = 0; b < blacks; b++) {
                P2? at = Layouts.Clear(l, c.Catalog, Kinds.A0Black, 1.2, () => {
                    Pose2 g = At((rng.Int(0, n - 2) + rng.Range(0.3, 0.7)) * spacing);
                    return g.At(0, rng.Range(-0.5, 0.5) * cc);
                });
                if (at != null) l.Add($"{id}.channel_black{b + 1}", "obstacle", Kinds.A0Black, at.Value);
            }

            double size = p.Num(nameof(DebrisClearanceParams.field_size));
            Pose2 end = At((n - 1) * spacing);
            P2 centre = end.At(p.Num(nameof(DebrisClearanceParams.field_gap)) + size / 2, 0);
            var field = new Pose2(centre.X, centre.Y, end.Yaw);
            var keep = new List<(P2, double)>();
            var rescue = new List<PlacedObject>();
            var avoid = new List<PlacedObject>();
            var report = new List<PlacedObject>();
            void Place(string name, string role, string kind, string color, int count) {
                for (int i = 0; i < count; i++) {
                    // Greens keep room for the 360° rescue circle (P3); the rest 2 m apart.
                    double spacingFor = color == "green" ? 3.5 : 2.5;
                    // A green's rescue circle (turn radius + beam) plus the hull must fit inside.
                    double room = color == "green" ? c.Vessel.TurnRadius + 1.5 * c.Vessel.Beam + 0.3 : c.Catalog.Radius(kind);
                    P2? at = Layouts.Scatter(rng, field, size / 2 - 1, size / 2 - 1, c.Occupancy,
                        keep.Select(q => (q.Item1, Math.Max(q.Item2, spacingFor))), room);
                    if (at == null) continue;
                    var o = l.Add($"{id}.{name}{i + 1}", role, kind, at.Value, field.Yaw);
                    if (color != null) Layouts.State(l, o, "indicator", Programs.Fixed(color));
                    keep.Add((at.Value, spacingFor));
                    report.Add(o);
                    if (color == "green") rescue.Add(o);
                    if (color == "red") avoid.Add(o);
                }
            }
            Place("green_beacon", "beacon", Kinds.LightBuoy, "green", p.Int(nameof(DebrisClearanceParams.green_beacons)));
            Place("red_beacon", "beacon", Kinds.LightBuoy, "red", p.Int(nameof(DebrisClearanceParams.red_beacons)));
            Place("black", "debris", Kinds.A0Black, null, p.Int(nameof(DebrisClearanceParams.field_blacks)));

            l.Anchor("approach", c.Anchor.At(-3, 0));
            l.Anchor("exit", end.At(2, 0));
            l.Anchor("field", centre, size / 2);
            l.Semantics.Set("rescue", Layouts.IdList(rescue)).Set("avoid", Layouts.IdList(avoid))
                .Set("report", Layouts.IdList(report)).Set("bend", bend > 0 ? "port" : "starboard");
            return l;
        }
    }
}
