// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Deterministic mission sets for a forest manifest. Identical in Unity and in the .NET harness.
    ///  * corridor_fwd / corridor_rev: the manifest's CorridorWaypoint polyline (cleared of trunks within
    ///    corridor_clear_radius_m, branches and mid-story NOT cleared), forward and reversed;
    ///  * cross_k: straight-line routes between seeded random interior points &gt;= 35 m apart, start/goal with &gt;= 1.5 m hard
    ///    clearance, route points every 5 m at ground + CruiseAgl (the mission planner's DEM = manifest terrain model).
    ///    These cut through unmodified forest: the planner has to find its own way.
    /// </summary>
    public static class MissionCatalog {
        public static List<MissionSpec> Build(ForestManifestWorld w, AutonomyConfig c, int crossCount, bool corridor = true) {
            var o = new List<MissionSpec>();
            if (corridor && w.Corridor.Count >= 2) {
                var fwd = new List<Vec3>(w.Corridor);
                o.Add(Make("corridor_fwd", w, fwd));
                var rev = new List<Vec3>(w.Corridor); rev.Reverse();
                o.Add(Make("corridor_rev", w, rev));
            }
            var rng = new Rng((ulong)w.Seed, 77);
            double half = Math.Min(Math.Min(-w.MinX, w.MaxX), Math.Min(-w.MinY, w.MaxY)) - 12.0;
            int made = 0, tries = 0;
            while (made < crossCount && tries++ < 5000) {
                var a = new Vec3((2 * rng.Uniform() - 1) * half, (2 * rng.Uniform() - 1) * half, 0);
                var b = new Vec3((2 * rng.Uniform() - 1) * half, (2 * rng.Uniform() - 1) * half, 0);
                if ((b - a).NormXY < 35.0) continue;
                a.Z = w.GroundHeight(a.X, a.Y) + c.CruiseAgl; b.Z = w.GroundHeight(b.X, b.Y) + c.CruiseAgl;
                if (w.Clearance(a, 2.0, true, out _, out _) < 1.5 || w.Clearance(b, 2.0, true, out _, out _) < 1.5) continue;
                var route = new List<Vec3>();
                double L = (b - a).NormXY; int n = Math.Max(1, (int)Math.Ceiling(L / 5.0));
                for (int i = 0; i <= n; i++) { var p = Vec3.Lerp(a, b, (double)i / n); p.Z = w.GroundHeight(p.X, p.Y) + c.CruiseAgl; route.Add(p); }
                o.Add(Make("cross_" + made, w, route));
                made++;
            }
            return o;
        }

        static MissionSpec Make(string name, ForestManifestWorld w, List<Vec3> route) {
            var m = new MissionSpec { Name = name, ForestSeed = w.Seed, Route = route, Start = route[0] };
            m.StartYaw = Math.Atan2(route[1].Y - route[0].Y, route[1].X - route[0].X);
            return m;
        }
    }
}
