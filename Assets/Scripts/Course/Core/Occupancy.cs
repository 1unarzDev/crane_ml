using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Course {
    /// <summary>What a task may not overlap: placed objects, task footprints and the course
    /// boundary.</summary>
    public interface IOccupancy {
        CourseBoundary Boundary { get; }
        /// <summary>A circle of <paramref name="radius"/> at <paramref name="p"/> keeps
        /// <paramref name="clearance"/> from every placed object and lies inside the boundary.</summary>
        bool Free(P2 p, double radius, double clearance);
        /// <summary>The polygon overlaps no placed task footprint.</summary>
        bool FootprintFree(IReadOnlyList<P2> polygon);
    }

    public sealed class Occupancy : IOccupancy {
        readonly List<(PlacedObject o, double r)> objects = new List<(PlacedObject, double)>();
        readonly List<List<P2>> footprints = new List<List<P2>>();
        public CourseBoundary Boundary { get; }

        public Occupancy(CourseBoundary boundary) { Boundary = boundary; }

        public void Add(TaskLayout layout, CourseObjectCatalog catalog) {
            foreach (var o in layout.Objects) objects.Add((o, catalog.Radius(o.Kind)));
            if (layout.Footprint.Count > 2) footprints.Add(layout.Footprint);
        }

        public bool Free(P2 p, double radius, double clearance) {
            if (!Boundary.Contains(p, radius)) return false;
            foreach (var (o, r) in objects)
                if (Geometry.SurfaceDistance(p, o, r) - radius < clearance) return false;
            return true;
        }

        public bool FootprintFree(IReadOnlyList<P2> polygon) =>
            footprints.All(f => !Geometry.ConvexOverlap(f, polygon));
    }

    /// <summary>Plane geometry shared by occupancy, the constraint evaluator and path checks.
    /// validate.py mirrors every function here line for line.</summary>
    public static class Geometry {
        /// <summary>Signed distance from p to a box (negative inside).</summary>
        public static double BoxDistance(P2 p, P2 center, double yaw, double length, double width) {
            P2 d = p - center;
            double c = Math.Cos(yaw), s = Math.Sin(yaw);
            double lx = Math.Abs(c * d.X + s * d.Y) - length / 2, ly = Math.Abs(-s * d.X + c * d.Y) - width / 2;
            if (lx <= 0 && ly <= 0) return Math.Max(lx, ly);
            return Math.Sqrt(Math.Max(lx, 0) * Math.Max(lx, 0) + Math.Max(ly, 0) * Math.Max(ly, 0));
        }

        public static double SurfaceDistance(P2 p, PlacedObject o, double radius) =>
            o.IsBox ? BoxDistance(p, o.At, o.Yaw, o.Length, o.Width) : P2.Dist(p, o.At) - radius;

        /// <summary>Convex hull (monotone chain), counter-clockwise, grown by margin as a
        /// 16-gon around each input point, so a footprint covers its objects with room.</summary>
        public static List<P2> Hull(IEnumerable<P2> points, double margin) {
            var pts = new List<P2>();
            foreach (P2 p in points)
                for (int k = 0; k < 16; k++) {
                    double a = 2 * Math.PI * k / 16;
                    pts.Add(new P2(CourseFrame.Mm(p.X + margin * Math.Cos(a)), CourseFrame.Mm(p.Y + margin * Math.Sin(a))));
                }
            pts = pts.OrderBy(p => p.X).ThenBy(p => p.Y).ToList();
            if (pts.Count < 3) return pts;
            var hull = new List<P2>();
            for (int pass = 0; pass < 2; pass++) {
                int start = hull.Count;
                foreach (P2 p in pts) {
                    while (hull.Count >= start + 2 && Cross(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0)
                        hull.RemoveAt(hull.Count - 1);
                    hull.Add(p);
                }
                hull.RemoveAt(hull.Count - 1);
                pts.Reverse();
            }
            return hull;
        }

        public static double Cross(P2 o, P2 a, P2 b) => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

        /// <summary>Separating-axis test for two convex polygons; touching is not overlap.</summary>
        public static bool ConvexOverlap(IReadOnlyList<P2> a, IReadOnlyList<P2> b) {
            foreach (var poly in new[] { a, b })
                for (int i = 0; i < poly.Count; i++) {
                    P2 e = poly[(i + 1) % poly.Count] - poly[i];
                    P2 n = new P2(-e.Y, e.X);
                    double amin = double.MaxValue, amax = double.MinValue, bmin = double.MaxValue, bmax = double.MinValue;
                    foreach (P2 p in a) { double d = P2.Dot(p, n); amin = Math.Min(amin, d); amax = Math.Max(amax, d); }
                    foreach (P2 p in b) { double d = P2.Dot(p, n); bmin = Math.Min(bmin, d); bmax = Math.Max(bmax, d); }
                    if (amax <= bmin + 1e-9 || bmax <= amin + 1e-9) return false;
                }
            return true;
        }

        /// <summary>Signed distance of p inside a convex counter-clockwise polygon (negative outside).</summary>
        public static double InsideMargin(IReadOnlyList<P2> poly, P2 p) {
            double best = double.MaxValue;
            for (int i = 0; i < poly.Count; i++) {
                P2 a = poly[i], b = poly[(i + 1) % poly.Count];
                P2 e = b - a;
                best = Math.Min(best, Cross(a, b, p) / e.Length);
            }
            return best;
        }
    }
}
