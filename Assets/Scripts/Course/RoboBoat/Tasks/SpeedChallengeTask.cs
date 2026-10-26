using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class SpeedChallengeParams : ITaskParams {
        [CourseParam("D", "beyond the marina's far end")] public ParamRange along_offset = ParamRange.Uniform(12, 20);
        [CourseParam("H", "p.49 marker sets the direction (L Jan 14); sign opt-in")] public ParamRange direction_source = ParamRange.Default("marker", "sign");
        [CourseParam("D", "gate clear of the tasks toward side S")] public ParamRange gate_offset = ParamRange.Uniform(5, 10);
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
        [CourseParam("D", "gate normal off facing the interior")] public ParamRange yaw_deg = ParamRange.Angle(-180, 180);
    }

    /// <summary>Task 3, emergency response sprint (H pp.49–51, L Jan 14): one A-2 gate on
    /// side S facing the interior, a yellow turn mark 12–30 m out, a separate red/green light
    /// buoy between them (or the indicator on the mark), a light sign before the gate. The
    /// direction source's colour sets the circling direction: red ccw, green cw.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Speed Challenge")]
    public sealed class SpeedChallengeTask : CourseTaskDefinition<SpeedChallengeParams> {
        public override string TaskType => "speed_challenge";
        public override string Cite => "H pp.49-51; L Jan 14";

        /// <summary>The leg (gate to mark) turns by yaw_deg about its centre. The centre goes beyond
        /// the marina's far end on side S, gate_offset plus the leg's reach clear of whatever already
        /// reaches toward side S; if a disc of that reach is not free there, at a random free spot on
        /// side S. yaw_deg 0 faces the interior.</summary>
        public override Pose2 Place(PlacementContext c) {
            var p = c.Params;
            var marina = c.ByType("marina");
            double leg = p.Num(nameof(SpeedChallengeParams.turn_distance)), reach = leg / 2 + 4.5;
            double x = (marina != null ? Layouts.MaxX(marina) : 0) + p.Num(nameof(SpeedChallengeParams.along_offset)) + reach;
            double edge = Layouts.EdgeInSlab(c.Placed.Values.Where(l => l != marina), x - reach, x + reach, c.Side);
            double y = Math.Min(edge + p.Num(nameof(SpeedChallengeParams.gate_offset)) + reach, c.Boundary.HalfWidth - reach - 1);
            var centre = new P2(x, c.Side * y);
            for (int k = 0; k < 200 && !(c.Boundary.Contains(centre, reach) && c.Occupancy.FootprintFree(Disc(centre, reach + 1.5))); k++)
                centre = new P2(c.Rng.Range(reach, c.Boundary.Length - c.Boundary.Back - reach), c.Side * c.Rng.Range(reach, c.Boundary.HalfWidth - reach));
            double yaw = c.Side * Math.PI / 2 + Layouts.Deg(p.Num(nameof(SpeedChallengeParams.yaw_deg)));
            P2 gate = centre - P2.Dir(yaw) * (leg / 2);
            return new Pose2(gate.X, gate.Y, yaw);
        }

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
