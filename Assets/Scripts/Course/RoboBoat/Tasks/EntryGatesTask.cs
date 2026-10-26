using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class EntryGatesParams : ITaskParams {
        [CourseParam("H", "p.46 6-10 ft, read as c-c (D)")] public ParamRange gate_cc = ParamRange.Uniform(1.83, 3.05);
        [CourseParam("H", "p.46 6-10 ft, read as c-c (D)")] public ParamRange gate2_cc = ParamRange.Uniform(1.83, 3.05);
        [CourseParam("D", "gate 2 yaw")] public ParamRange gate2_yaw_deg = ParamRange.Uniform(-15, 15, "deg");
        [CourseParam("H", "p.45 25-100 ft")] public ParamRange row_separation = ParamRange.Uniform(7.62, 30.48);
        [CourseParam("H", "p.45 >= 6 ft; upper bound D")] public ParamRange standoff = ParamRange.Uniform(1.83, 4.0);
    }

    /// <summary>Task 1, evacuation route and return (H pp.45–47): two gates of tall Taylor
    /// markers ahead of the start, red to port. The exit is the same gates in reverse.</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Entry Gates")]
    public sealed class EntryGatesTask : CourseTaskDefinition<EntryGatesParams> {
        public override string TaskType => "entry_gates";
        public override string Cite => "H pp.45-47";

        public override Pose2 Place(PlacementContext c) => new Pose2(0, 0, 0);

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            string id = TaskId;
            double x1 = p.Num(nameof(EntryGatesParams.standoff)), x2 = x1 + p.Num(nameof(EntryGatesParams.row_separation));
            double yaw2 = c.Anchor.YawOf(Layouts.Deg(p.Num(nameof(EntryGatesParams.gate2_yaw_deg))));
            var g1 = Layouts.Gate(l, id, "gate1", 1, c.Anchor.At(x1, 0), c.Anchor.Yaw, p.Num(nameof(EntryGatesParams.gate_cc)), Kinds.TaylorRed, Kinds.TaylorGreen);
            var g2 = Layouts.Gate(l, id, "gate2", 2, c.Anchor.At(x2, 0), yaw2, p.Num(nameof(EntryGatesParams.gate2_cc)), Kinds.TaylorRed, Kinds.TaylorGreen);
            l.Anchor("start", c.Anchor.Position);
            l.Anchor("approach", c.Anchor.Position);
            l.Anchor("exit", c.Anchor.At(x2, 0) + P2.Dir(yaw2) * 3);
            l.Semantics.Set("gates", new List<object> { g1.Id, g2.Id }).Set("exit", "reverse");
            return l;
        }
    }
}
