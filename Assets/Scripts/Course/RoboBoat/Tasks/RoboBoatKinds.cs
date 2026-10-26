using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Course.RoboBoat {
    /// <summary>Catalog kinds the RoboBoat tasks place (Appendix B lists the prefabs).</summary>
    public static class Kinds {
        public const string TaylorRed = "roboboat/taylor_red", TaylorGreen = "roboboat/taylor_green";
        public const string A0Red = "roboboat/a0_red", A0Green = "roboboat/a0_green", A0Black = "roboboat/a0_black";
        public const string A2Red = "roboboat/a2_red", A2Green = "roboboat/a2_green", A2Yellow = "roboboat/a2_yellow";
        public const string LightBuoy = "roboboat/light_buoy", LightIndicator = "roboboat/light_indicator";
        public const string LightSign = "roboboat/light_sign", LightBeacon = "roboboat/light_beacon";
        public const string DockSpine = "roboboat/dock_spine", DockTine = "roboboat/dock_tine", DockSign = "roboboat/dock_sign";
        public const string MooredBoat = "roboboat/moored_boat";
        public const string HullWaterYellow = "roboboat/hull_water_yellow", HullWaterOrange = "roboboat/hull_water_orange";
        public const string HullBall = "roboboat/hull_ball_black";
    }

    internal static class Layouts {
        /// <summary>A gate: red to port of the travel direction (H), green to starboard.</summary>
        public static GateSpec Gate(TaskLayout l, string task, string name, int order, P2 mid, double yaw, double cc,
            string red, string green) {
            P2 left = P2.Dir(yaw).Left;
            var port = l.Add($"{task}.{name}.port", "gate", red, mid + left * (cc / 2), yaw);
            var starboard = l.Add($"{task}.{name}.starboard", "gate", green, mid - left * (cc / 2), yaw);
            var g = new GateSpec { Id = $"{task}.{name}", Port = port.Id, Starboard = starboard.Id, Order = order, NormalYaw = yaw };
            l.Gates.Add(g);
            return g;
        }

        public static void State(TaskLayout l, PlacedObject o, string kind, JObj program) {
            o.StateId = o.Id;
            l.States.Add(new StateSpec { Id = o.Id, Kind = kind, Initial = Programs.Initial(program), Program = program });
        }

        public static P2 Mid(TaskLayout l, GateSpec g) =>
            (l.Objects.First(o => o.Id == g.Port).At + l.Objects.First(o => o.Id == g.Starboard).At) * 0.5;

        /// <summary>Up to <paramref name="tries"/> uniform draws in a box until one keeps
        /// <paramref name="spacing"/> from the given points and the occupancy is free.</summary>
        public static P2? Scatter(CourseRng rng, Pose2 frame, double halfX, double halfY, IOccupancy occ,
            IEnumerable<(P2 at, double spacing)> keep, double radius, int tries = 200) {
            var avoid = keep.ToList();
            for (int i = 0; i < tries; i++) {
                P2 p = frame.At(rng.Range(-halfX, halfX), rng.Range(-halfY, halfY));
                if (avoid.All(k => P2.Dist(k.at, p) >= k.spacing) && occ.Free(p, radius, 1.0)) return p;
            }
            return null;
        }

        /// <summary>The furthest any footprint reaches toward <paramref name="side"/> (in y)
        /// within the x-slab [x0, x1]; edges are clipped to the slab.</summary>
        public static double EdgeInSlab(IEnumerable<TaskLayout> layouts, double x0, double x1, double side) {
            double edge = 0;
            foreach (var l in layouts)
                for (int i = 0; i < l.Footprint.Count; i++) {
                    P2 a = l.Footprint[i], b = l.Footprint[(i + 1) % l.Footprint.Count];
                    double lo = 0, hi = 1;
                    if (Math.Abs(b.X - a.X) < 1e-12) { if (a.X < x0 || a.X > x1) continue; }
                    else {
                        double t0 = (x0 - a.X) / (b.X - a.X), t1 = (x1 - a.X) / (b.X - a.X);
                        lo = Math.Max(0, Math.Min(t0, t1)); hi = Math.Min(1, Math.Max(t0, t1));
                        if (lo > hi) continue;
                    }
                    edge = Math.Max(edge, Math.Max(side * (a.Y + (b.Y - a.Y) * lo), side * (a.Y + (b.Y - a.Y) * hi)));
                }
            return edge;
        }

        /// <summary>A local draw repeated until it keeps <paramref name="gap"/> (surface to
        /// surface) from every object already in the layout; null after 50 tries.</summary>
        public static P2? Clear(TaskLayout l, CourseObjectCatalog catalog, string kind, double gap, Func<P2> draw) {
            for (int i = 0; i < 50; i++) {
                P2 p = draw();
                if (l.Objects.All(o => Geometry.SurfaceDistance(p, o, catalog.Radius(o.Kind)) - catalog.Radius(kind) >= gap)) return p;
            }
            return null;
        }

        public static double MaxX(TaskLayout l) => l.Footprint.Count > 0 ? l.Footprint.Max(p => p.X) : l.Objects.Max(o => o.At.X);

        public static List<object> IdList(IEnumerable<PlacedObject> objects) => objects.Select(o => (object)o.Id).ToList();

        public static string Direction(string color) => color == "red" ? "ccw" : "cw";  // H p.49

        public static double Deg(double d) => d * Math.PI / 180;
    }
}
