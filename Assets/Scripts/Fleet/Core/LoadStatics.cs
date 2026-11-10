using System;
using System.Collections.Generic;

namespace Crane.Fleet {
    /// <summary>
    /// Quasi-static tipping check of a loaded rover or rigid assembly on level ground (engine-free). The support polygon is the convex hull of the
    /// wheel contact points; the combined centre of mass of every body (rovers and payload) is projected on the floor. Reported, per run, for
    /// the worst edge:
    /// <list type="bullet">
    /// <item><c>MarginStatic</c>: distance from the CoM to the nearest edge (m) and <c>RelStatic</c> = that distance / the distance from the polygon
    /// centroid to the same edge (1 = centred, 0 = on the edge, negative = outside: it tips);</item>
    /// <item><c>RelAtAccel</c>: the same after a horizontal acceleration <c>a</c> in the worst direction shifts the effective CoM by h a / g;</item>
    /// <item><c>TipAccel</c>: g d / h for the weakest edge (the horizontal acceleration that tips it).</item>
    /// </list>
    /// Rigid bodies, no suspension, no dynamic load transfer beyond the static shift (an upper bound on stability, labelled so).
    /// </summary>
    public static class LoadStatics {
        public struct Mass { public double M, X, Y, Z; public Mass(double m, double x, double y, double z) { M = m; X = x; Y = y; Z = z; } }

        public struct Result {
            public double ComX, ComY, ComZ, MarginStatic, RelStatic, RelAtAccel, TipAccel, Accel;
            public int Supports;
            public bool Flagged(double minRel) => RelAtAccel < minRel;
        }

        public static Result Evaluate(IList<(double x, double y)> contacts, IList<Mass> masses, double accel, double g) {
            double m = 0, cx = 0, cy = 0, cz = 0;
            foreach (var b in masses) { m += b.M; cx += b.M * b.X; cy += b.M * b.Y; cz += b.M * b.Z; }
            cx /= m; cy /= m; cz /= m;
            var hull = Hull(contacts);
            double gx = 0, gy = 0;
            foreach (var p in hull) { gx += p.x; gy += p.y; }
            gx /= hull.Count; gy /= hull.Count;
            double minD = double.PositiveInfinity, rel = double.PositiveInfinity, relA = double.PositiveInfinity;
            double shift = cz * accel / g;
            for (int i = 0; i < hull.Count; i++) {
                var a = hull[i]; var b = hull[(i + 1) % hull.Count];
                double ex = b.x - a.x, ey = b.y - a.y, len = Math.Sqrt(ex * ex + ey * ey);
                if (len < 1e-12) continue;
                // inward normal of a counter-clockwise hull: (-ey, ex)/len
                double nx = -ey / len, ny = ex / len;
                double d = (cx - a.x) * nx + (cy - a.y) * ny;          // CoM distance inside this edge
                double dref = (gx - a.x) * nx + (gy - a.y) * ny;       // centroid distance inside this edge
                if (d < minD) minD = d;
                rel = Math.Min(rel, d / dref);
                relA = Math.Min(relA, (d - shift) / dref);
            }
            return new Result { ComX = cx, ComY = cy, ComZ = cz, MarginStatic = minD, RelStatic = rel, RelAtAccel = relA,
                TipAccel = cz > 0 ? g * minD / cz : double.PositiveInfinity, Accel = accel, Supports = contacts.Count };
        }

        /// <summary>Convex hull (counter-clockwise, Andrew's monotone chain).</summary>
        public static List<(double x, double y)> Hull(IList<(double x, double y)> pts) {
            var p = new List<(double x, double y)>(pts);
            p.Sort((u, v) => u.x != v.x ? u.x.CompareTo(v.x) : u.y.CompareTo(v.y));
            if (p.Count < 3) return p;
            double Cross((double x, double y) o, (double x, double y) a, (double x, double y) b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
            var h = new List<(double x, double y)>();
            foreach (var q in p) { while (h.Count >= 2 && Cross(h[h.Count - 2], h[h.Count - 1], q) <= 0) h.RemoveAt(h.Count - 1); h.Add(q); }
            int lower = h.Count + 1;
            for (int i = p.Count - 2; i >= 0; i--) { var q = p[i]; while (h.Count >= lower && Cross(h[h.Count - 2], h[h.Count - 1], q) <= 0) h.RemoveAt(h.Count - 1); h.Add(q); }
            h.RemoveAt(h.Count - 1);
            return h;
        }
    }
}
