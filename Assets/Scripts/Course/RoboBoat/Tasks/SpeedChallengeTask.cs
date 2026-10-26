using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class SpeedChallengeParams : ITaskParams {
        [CourseParam("D", "gate at least this far beyond entry gate 2 along its normal (course progression)")] public ParamRange beyond_gate2 = ParamRange.Fixed(3);
        [CourseParam("H", "p.49 marker sets the direction (L Jan 14); sign opt-in")] public ParamRange direction_source = ParamRange.Default("marker", "sign");
        [CourseParam("D", "gate from the preceding task's exit (channel exit, else entry gate 2)")] public ParamRange gate_distance = ParamRange.Uniform(10, 45);
        [CourseParam("H", "p.50 6-10 ft, read as c-c (D)")] public ParamRange gate_cc = ParamRange.Uniform(1.83, 3.05);
        [CourseParam("D", "lateral <= 3 m")] public ParamRange mark_lateral = ParamRange.Uniform(-3, 3);
        [CourseParam("D", "fraction of the leg")] public ParamRange marker_at = ParamRange.Uniform(0.3, 0.7, "frac");
        [CourseParam("H", "p.49 red ccw, green cw")] public ParamRange marker_color = ParamRange.Choice("red", "green");
        [CourseParam("D", "marker off the leg line")] public ParamRange marker_lateral = ParamRange.Uniform(1.0, 2.5);
        [CourseParam("L", "Jan 14 separate marker (default); H p.49 on the mark")] public ParamRange marker_variant = ParamRange.Default("separate", "on_mark");
        [CourseParam("D")] public ParamRange obstacles = ParamRange.Int(0, 2);
        [CourseParam("D")] public ParamRange sign_color = ParamRange.Choice("red", "green");
        [CourseParam("D", "fixed (H: set before the run); toggle opt-in")] public ParamRange sign_program = ParamRange.Default("fixed", "toggle");
        [CourseParam("D")] public ParamRange toggle_period_s = ParamRange.Uniform(5, 20, "s");
        [CourseParam("H", "p.50 40-100 ft")] public ParamRange turn_distance = ParamRange.Uniform(12.19, 30.48);
        [CourseParam("D", "gate normal off the approach bearing, scaled so it stays within 60 deg of the bearing from both the channel exit and entry gate 2 (H p.40 map: the boat runs straight in)")] public ParamRange yaw_deg = ParamRange.Angle(-60, 60);
    }

    /// <summary>Task 3, emergency response sprint (H pp.49–51, L Jan 14): one A-2 gate on
    /// side S beyond entry gate 2, facing the boat's approach (within 60 deg), a yellow turn mark 12–30 m out, a separate red/green light
    /// buoy between them (or the indicator on the mark), a light sign before the gate. The
    /// direction source's colour sets the circling direction: red ccw, green cw.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Speed Challenge")]
    public sealed class SpeedChallengeTask : CourseTaskDefinition<SpeedChallengeParams> {
        public override string TaskType => "speed_challenge";
        public override string Cite => "H pp.49-51; L Jan 14";

        public const double ApproachMaxDeg = 60;  // D, rows T3.approach_exit / T3.approach_gate2

        /// <summary>The gate goes gate_distance from the preceding task's exit (the channel exit,
        /// else entry gate 2) at a random bearing, on side S, at least beyond_gate2 past entry
        /// gate 2 along its normal. Its normal faces the approach: the mean of the bearings from the
        /// channel exit and from gate 2, turned by yaw_deg scaled so it stays within 60 deg of
        /// both. The leg (gate to mark) and its reach must lie on side S, inside the course and
        /// clear of every footprint; else the distance and bearing are redrawn within range.</summary>
        public override Pose2 Place(PlacementContext c) {
            var p = c.Params;
            var entry = c.ByType("entry_gates") ?? throw new InvalidOperationException("speed_challenge follows entry_gates");
            var g2 = entry.Gates.OrderBy(g => g.Order).Last();
            P2 hub = Layouts.Mid(entry, g2);
            P2 exit = c.ByType("debris_clearance")?.Anchors["exit"] is JObj e ? new P2(e.Num("x"), e.Num("y")) : hub;
            double leg = p.Num(nameof(SpeedChallengeParams.turn_distance)), reach = leg / 2 + 4.5;
            double beyond = p.Num(nameof(SpeedChallengeParams.beyond_gate2)), turn = p.Num(nameof(SpeedChallengeParams.yaw_deg));
            var range = new SpeedChallengeParams().gate_distance;
            double dist = p.Num(nameof(SpeedChallengeParams.gate_distance));
            Pose2 best = new Pose2(hub.X + beyond, hub.Y, g2.NormalYaw);
            for (int k = 0; k < 400; k++) {
                if (k > 0) dist = c.Rng.Range(range.min, range.max);
                P2 gate = exit + P2.Dir(c.Rng.Range(-Math.PI, Math.PI)) * dist;
                if (P2.Dot(gate - hub, P2.Dir(g2.NormalYaw)) < beyond + 0.05) continue;
                double b1 = Bearing(exit, gate), b2 = Bearing(hub, gate), half = Math.Abs(CourseFrame.Wrap(b1 - b2)) / 2;
                double room = Layouts.Deg(ApproachMaxDeg - 1) - half;
                if (room < Layouts.Deg(5)) continue;
                double yaw = b2 + CourseFrame.Wrap(b1 - b2) / 2 + Layouts.Deg(turn) * room / Layouts.Deg(ApproachMaxDeg);
                P2 centre = gate + P2.Dir(yaw) * (leg / 2);
                if (c.Side * centre.Y < reach || c.Side * gate.Y < 4) continue;
                if (!c.Boundary.Contains(centre, reach) || !c.Boundary.Contains(gate - P2.Dir(yaw) * 4, 1)) continue;
                if (!c.Occupancy.FootprintFree(Disc(centre, reach + 1.5)) || !c.Occupancy.FootprintFree(Disc(gate - P2.Dir(yaw) * 2.5, 3))) continue;
                return new Pose2(gate.X, gate.Y, CourseFrame.Wrap(yaw));
            }
            return best;  // rejected by the rows, then re-forked
        }

        static double Bearing(P2 from, P2 to) => Math.Atan2(to.Y - from.Y, to.X - from.X);

        static List<P2> Disc(P2 centre, double r) =>
            Enumerable.Range(0, 12).Select(i => centre + P2.Dir(i * Math.PI / 6) * (r / Math.Cos(Math.PI / 12))).ToList();

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            var rng = c.Rng;
            string id = TaskId;
            double cc = p.Num(nameof(SpeedChallengeParams.gate_cc)), leg = p.Num(nameof(SpeedChallengeParams.turn_distance));
            var gate = Layouts.Gate(l, id, "gate", 1, c.Anchor.Position, c.Anchor.Yaw, cc, Kinds.A2Red, Kinds.A2Green);
            var mark = l.Add($"{id}.mark", "mark", Kinds.A2Yellow, c.Anchor.At(leg, p.Num(nameof(SpeedChallengeParams.mark_lateral))));

            string markerColor = p.Str(nameof(SpeedChallengeParams.marker_color));
            PlacedObject marker;
            if (p.Str(nameof(SpeedChallengeParams.marker_variant)) == "separate") {
                double side = rng.Sign();
                marker = l.Add($"{id}.marker", "marker", Kinds.LightBuoy,
                    c.Anchor.At(p.Num(nameof(SpeedChallengeParams.marker_at)) * leg, side * p.Num(nameof(SpeedChallengeParams.marker_lateral))));
            }
            else {
                marker = l.Add($"{id}.marker", "marker_on_mark", Kinds.LightIndicator, mark.At);
                marker.Z = c.Catalog.Height(Kinds.A2Yellow);
            }
            Layouts.State(l, marker, "indicator", Programs.Fixed(markerColor));

            string signColor = p.Str(nameof(SpeedChallengeParams.sign_color));
            var sign = l.Add($"{id}.sign", "light_sign", Kinds.LightSign, c.Anchor.At(-2.5, rng.Sign() * (cc / 2 + 2.0)), c.Anchor.YawOf(Math.PI));
            JObj signProgram = p.Str(nameof(SpeedChallengeParams.sign_program)) == "toggle"
                ? Programs.Toggle(signColor, signColor == "red" ? "green" : "red", p.Num(nameof(SpeedChallengeParams.toggle_period_s)), 0)
                : Programs.Fixed(signColor);
            Layouts.State(l, sign, "light_sign", signProgram);

            for (int i = 0; i < p.Int(nameof(SpeedChallengeParams.obstacles)); i++) {
                P2? at = Layouts.Clear(l, c.Catalog, Kinds.A0Black, 1.5,
                    () => c.Anchor.At(rng.Range(0.25, 0.85) * leg, rng.Sign() * rng.Range(1.5, 3.5)));
                if (at != null) l.Add($"{id}.obstacle{i + 1}", "obstacle", Kinds.A0Black, at.Value);
            }

            string source = p.Str(nameof(SpeedChallengeParams.direction_source));
            string sourceColor = source == "sign" ? signColor : markerColor;
            l.Semantics
                .Set("sign", new JObj().Set("id", sign.Id).Set("program", signProgram))
                .Set("marker", new JObj().Set("id", marker.Id).Set("color", markerColor))
                .Set("direction_source", source).Set("source_color", sourceColor)
                .Set("direction", Layouts.Direction(sourceColor)).Set("turn_mark", mark.Id).Set("gate", gate.Id)
                .Set("variant", p.Str(nameof(SpeedChallengeParams.marker_variant)));
            l.Anchor("approach", c.Anchor.At(-4, 0));
            l.Anchor("mark", mark.At);
            return l;
        }
    }
}
