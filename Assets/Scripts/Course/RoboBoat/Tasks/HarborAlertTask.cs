using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    [Serializable]
    public sealed class HarborAlertParams : ITaskParams {
        [CourseParam("H", "p.56 600/800/1000 Hz; 1200 opt-in")] public ParamRange base_hz = ParamRange.Choice("600", "800", "1000");
        [CourseParam("H", "p.56 +5%")] public ParamRange freq_offset = ParamRange.Uniform(0, 0.05, "frac");
        [CourseParam("H", "p.57 one 1 s blast (zone 1) or two 0.5 s (zone 2)")] public ParamRange pattern = ParamRange.Choice("one", "two");
        [CourseParam("D")] public ParamRange repeats = ParamRange.Int(1, 3);
        [CourseParam("D", "sim seconds after the course is ready")] public ParamRange trigger_s = ParamRange.Uniform(60, 240, "s");
        [CourseParam("D", "zone 2 buoy bearing off the marina axis at the mouth")] public ParamRange zone2_bearing_deg = ParamRange.Uniform(-120, 120, "deg");
        [CourseParam("D", "zone 2 buoy off the marina mouth (H p.40 map; T6.zone2)")] public ParamRange zone2_offset = ParamRange.Uniform(7, 12);
        [CourseParam("D", "zone 2 buoy from the start and the entry lane (T6.zone2_start, T6.zone2_lane)")] public ParamRange zone2_clear = ParamRange.Fixed(15);
        [CourseParam("D")] public ParamRange zone_radius = ParamRange.Fixed(3);
    }

    /// <summary>Task 6, harbor alert (H pp.56–58): one blast sends the boat to zone 1 at the
    /// sprint's turn mark, two blasts to zone 2, a yellow buoy 7-12 m off the marina mouth (H p.40
    /// map) and at least zone2_clear from the start and the entry lane (D).</summary>
    [CreateAssetMenu(menuName = "Course/RoboBoat/Harbor Alert")]
    public sealed class HarborAlertTask : CourseTaskDefinition<HarborAlertParams> {
        public override string TaskType => "harbor_alert";
        public override string Cite => "H pp.56-58";

        public override Pose2 Place(PlacementContext c) {
            var marina = c.ByType("marina") ?? throw new InvalidOperationException("harbor_alert follows marina");
            JObj end = (JObj)marina.Anchors["outer_end"], approach = (JObj)marina.Anchors["approach"];
            double axis = Math.Atan2(approach.Num("y") - end.Num("y"), approach.Num("x") - end.Num("x"));
            return new Pose2(end.Num("x"), end.Num("y"), axis);
        }

        public override TaskLayout Generate(TaskContext c) {
            var l = new TaskLayout();
            var p = c.Params;
            string id = TaskId;
            double r = p.Num(nameof(HarborAlertParams.zone_radius));
            var speed = c.Placement.ByType("speed_challenge") ?? throw new InvalidOperationException("harbor_alert follows speed_challenge");
            P2 zone1 = speed.Objects.First(o => o.Role == "mark").At;
            // Off the marina mouth at the sampled bearing and offset; if that spot or its approach is
            // taken, or it is within zone2_clear of the start or the entry lane (start, gate 1,
            // gate 2), redraw both within their ranges (D). None fits: keep the sampled spot and
            // let the rows reject it.
            var range = new HarborAlertParams();
            double m = CourseGenerator.FootprintMargin, clear = p.Num(nameof(HarborAlertParams.zone2_clear));
            var entry = c.Placement.ByType("entry_gates");
            var lane = new List<P2> { new P2(0, 0) };
            if (entry != null) {
                if (entry.Anchors["start"] is JObj st) lane[0] = new P2(st.Num("x"), st.Num("y"));
                lane.AddRange(entry.Gates.OrderBy(g => g.Order).Select(g => Layouts.Mid(entry, g)));
            }
            double sampledDeg = p.Num(nameof(HarborAlertParams.zone2_bearing_deg)), sampledOff = p.Num(nameof(HarborAlertParams.zone2_offset));
            bool Fits(double deg, double off) {
                double yaw = c.Anchor.YawOf(Layouts.Deg(deg));
                P2 q = c.Anchor.Position + P2.Dir(yaw) * off;
                var square = new List<P2> { q + new P2(-m, -m), q + new P2(m, -m), q + new P2(m, m), q + new P2(-m, m) };
                return P2.Dist(q, lane[0]) >= clear + 0.01 && Layouts.LaneDistance(lane, q) >= clear + 0.01 &&
                       c.Occupancy.Free(q, 0.25, 1.0) && c.Occupancy.Free(q + P2.Dir(yaw) * 2.5, 0.5, 1.0) && c.Occupancy.FootprintFree(square);
            }
            // The sampled spot, then 24 redraws, then the whole range on a 5 deg x 0.5 m grid nearest
            // the sampled spot first.
            var tries = new List<(double, double)> { (sampledDeg, sampledOff) };
            for (int k = 0; k < 24; k++)
                tries.Add((c.Rng.Range(range.zone2_bearing_deg.min, range.zone2_bearing_deg.max), c.Rng.Range(range.zone2_offset.min, range.zone2_offset.max)));
            var grid = new List<(double, double)>();
            for (double d = range.zone2_bearing_deg.min; d <= range.zone2_bearing_deg.max + 1e-9; d += 5)
                for (double o = range.zone2_offset.min; o <= range.zone2_offset.max + 1e-9; o += 0.5) grid.Add((d, o));
            tries.AddRange(grid.OrderBy(g => Math.Abs(g.Item1 - sampledDeg) / 240 + Math.Abs(g.Item2 - sampledOff) / 5)
                .ThenBy(g => g.Item1).ThenBy(g => g.Item2));
            var (bearingDeg, offset) = tries.FirstOrDefault(x => Fits(x.Item1, x.Item2));
            if (bearingDeg == 0 && offset == 0) (bearingDeg, offset) = (sampledDeg, sampledOff);
            double bearing = c.Anchor.YawOf(Layouts.Deg(bearingDeg));
            P2 at = c.Anchor.Position + P2.Dir(bearing) * offset;
            var buoy = l.Add($"{id}.zone2_buoy", "zone_buoy", Kinds.A2Yellow, at);
            double hz = double.Parse(p.Str(nameof(HarborAlertParams.base_hz)), CultureInfo.InvariantCulture) *
                        (1 + p.Num(nameof(HarborAlertParams.freq_offset)));
            hz = Math.Round(hz, 1);
            string pattern = p.Str(nameof(HarborAlertParams.pattern));
            var program = Programs.Horn(hz, pattern, p.Int(nameof(HarborAlertParams.repeats)), Math.Round(p.Num(nameof(HarborAlertParams.trigger_s)), 1));
            string hornId = $"{id}.horn";
            l.States.Add(new StateSpec { Id = hornId, Kind = "horn", Initial = "off", Program = program });
            l.Anchor("zone1", zone1, r);
            l.Anchor("zone2", buoy.At, r);
            l.Anchor("approach", buoy.At + P2.Dir(bearing) * 2.5);
            l.Semantics.Set("zone1", "zone1").Set("zone2", "zone2").Set("zone2_bearing_deg", Math.Round(bearingDeg, 2)).Set("zone2_offset", Math.Round(offset, 3))
                .Set("horn", new JObj().Set("id", hornId).Set("freq_hz", hz).Set("pattern", pattern)
                    .Set("program", program).Set("target_zone", pattern == "one" ? "zone1" : "zone2"));
            return l;
        }
    }
}
