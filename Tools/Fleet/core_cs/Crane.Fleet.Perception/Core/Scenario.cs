using System;
using System.Collections.Generic;
using System.IO;

namespace Crane.Fleet.Perception
{
    /// <summary>Oriented box (yaw about +z) with a full 3-D extent, so height range matters: an elevated beam only blocks rays through it.</summary>
    public sealed class Obb
    {
        public string Id; public Vec3 Centre; public double Yaw; public Vec3 Half;
        public Obb(string id, Vec3 centre, Vec3 half, double yaw = 0) { Id = id; Centre = centre; Half = half; Yaw = yaw; }
        public double ZMin => Centre.Z - Half.Z;
        public double ZMax => Centre.Z + Half.Z;

        private void ToLocal(Vec3 p, out double x, out double y, out double z)
        {
            double c = Math.Cos(Yaw), s = Math.Sin(Yaw), dx = p.X - Centre.X, dy = p.Y - Centre.Y;
            x = c * dx + s * dy; y = -s * dx + c * dy; z = p.Z - Centre.Z;
        }

        /// <summary>Slab test of segment p0->p1 against the box; returns the entry parameter in [0,1] or -1 for a miss.</summary>
        public double SegmentEntry(Vec3 p0, Vec3 p1)
        {
            ToLocal(p0, out double ax, out double ay, out double az);
            ToLocal(p1, out double bx, out double by, out double bz);
            double[] a = { ax, ay, az }, dv = { bx - ax, by - ay, bz - az }, h = { Half.X, Half.Y, Half.Z };
            double t0 = 0, t1 = 1;
            for (int k = 0; k < 3; k++)
            {
                if (Math.Abs(dv[k]) < 1e-12) { if (a[k] < -h[k] || a[k] > h[k]) return -1; continue; }
                double u = (-h[k] - a[k]) / dv[k], v = (h[k] - a[k]) / dv[k];
                if (u > v) { double tmp = u; u = v; v = tmp; }
                if (u > t0) t0 = u; if (v < t1) t1 = v;
                if (t0 > t1) return -1;
            }
            return t0;
        }

        /// <summary>True if the segment, with its last <paramref name="eps"/> metres removed (the target sits on a surface), intersects the box.</summary>
        public bool Blocks(Vec3 from, Vec3 to, double eps)
        {
            Vec3 d = to - from; double L = d.Norm;
            Vec3 end = L > 0 ? from + d * (1.0 - Math.Min(eps / L, 0.5)) : to;
            return SegmentEntry(from, end) >= 0;
        }

        public bool Contains(Vec3 p)
        {
            ToLocal(p, out double x, out double y, out double z);
            return Math.Abs(x) <= Half.X && Math.Abs(y) <= Half.Y && Math.Abs(z) <= Half.Z;
        }
    }

    public sealed class OverheadCameraSpec { public string Id; public Vec3 Position, LookAt; }

    /// <summary>Static scene (warehouse_a_v0): obstacles, divider wall split at the doorway, outer walls, overhead camera poses.</summary>
    public sealed class Scenario
    {
        public string ScenarioId; public double ArenaX, ArenaY, WallHeight;
        public List<Obb> Boxes = new List<Obb>();
        public List<Obb> ElevatedOnly = new List<Obb>();   // boxes that do not block driving (beam)
        public List<OverheadCameraSpec> OverheadCameras = new List<OverheadCameraSpec>();
        public List<string> OverheadModes = new List<string>();

        public static Scenario Load(string path, double outerWallThickness = 0.10) => Parse(File.ReadAllText(path), outerWallThickness);

        public static Scenario Parse(string json, double outerWallThickness = 0.10)
        {
            var d = (Dictionary<string, object>)MiniJson.Parse(json);
            var sc = new Scenario { ScenarioId = (string)d["scenario_id"] };
            var arena = (Dictionary<string, object>)d["arena"];
            var xy = Vec2((List<object>)arena["size_xy"]); sc.ArenaX = xy[0]; sc.ArenaY = xy[1]; sc.WallHeight = (double)arena["wall_height"];
            double wh = sc.WallHeight;
            // outer walls (outside the arena rectangle)
            double t = outerWallThickness;
            sc.Boxes.Add(new Obb("wall_south", new Vec3(sc.ArenaX / 2, -t / 2, wh / 2), new Vec3(sc.ArenaX / 2 + t, t / 2, wh / 2)));
            sc.Boxes.Add(new Obb("wall_north", new Vec3(sc.ArenaX / 2, sc.ArenaY + t / 2, wh / 2), new Vec3(sc.ArenaX / 2 + t, t / 2, wh / 2)));
            sc.Boxes.Add(new Obb("wall_west", new Vec3(-t / 2, sc.ArenaY / 2, wh / 2), new Vec3(t / 2, sc.ArenaY / 2, wh / 2)));
            sc.Boxes.Add(new Obb("wall_east", new Vec3(sc.ArenaX + t / 2, sc.ArenaY / 2, wh / 2), new Vec3(t / 2, sc.ArenaY / 2, wh / 2)));
            var door = (Dictionary<string, object>)d["doorway"];
            var wd = door["width"]; double dw = wd is double dd ? dd : (double)((Dictionary<string, object>)wd)["default"];
            double dc = (double)door["centre_y"], dh = (double)door["height"];
            foreach (var wo in (List<object>)d["walls"])
            {
                var w = (Dictionary<string, object>)wo; var f = Vec2((List<object>)w["from"]); var to = Vec2((List<object>)w["to"]);
                if (f[0] != to[0]) throw new FormatException("only walls of constant x are supported");
                double th = (double)w["thickness"]; string id = (string)w["id"];
                double lo = dc - dw / 2, hi = dc + dw / 2;
                sc.Boxes.Add(new Obb(id + "_s", new Vec3(f[0], (f[1] + lo) / 2, wh / 2), new Vec3(th / 2, (lo - f[1]) / 2, wh / 2)));
                sc.Boxes.Add(new Obb(id + "_n", new Vec3(f[0], (hi + to[1]) / 2, wh / 2), new Vec3(th / 2, (to[1] - hi) / 2, wh / 2)));
                sc.Boxes.Add(new Obb(id + "_lintel", new Vec3(f[0], dc, (dh + wh) / 2), new Vec3(th / 2, dw / 2, (wh - dh) / 2)));
            }
            foreach (var oo in (List<object>)d["obstacles"])
            {
                var o = (Dictionary<string, object>)oo;
                var c = Vec3L((List<object>)o["centre"]); var s = Vec3L((List<object>)o["size"]);
                var box = new Obb((string)o["id"], c, s * 0.5);
                sc.Boxes.Add(box);
                if (box.ZMin >= 0.5) sc.ElevatedOnly.Add(box);
            }
            foreach (var oo in (List<object>)d["overhead_cameras"])
            {
                var o = (Dictionary<string, object>)oo;
                sc.OverheadCameras.Add(new OverheadCameraSpec { Id = (string)o["id"], Position = Vec3L((List<object>)o["position"]), LookAt = Vec3L((List<object>)o["look_at"]) });
            }
            foreach (var m in (List<object>)d["overhead_modes"]) sc.OverheadModes.Add((string)m);
            return sc;
        }
        private static double[] Vec2(List<object> l) => new[] { (double)l[0], (double)l[1] };
        private static Vec3 Vec3L(List<object> l) => new Vec3((double)l[0], (double)l[1], (double)l[2]);
    }
}
