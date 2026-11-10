// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.IO;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Analytic world from a Drone Forest manifest (schema crane.drone_forest.manifest/1): every flight-band capsule
    /// (trunk segments and branches of each tree, mid-story stems), fallen logs (capsules) and rocks (spheres), plus a
    /// terrain height field interpolated from the manifest's ground-anchored points (tree bases, trail, props). Frames: the
    /// manifest is Unity world (x, y up, z); everything here is plant ENU = (Unity x, Unity z, Unity y).
    /// Used as (a) the sensor ray world outside Unity, (b) the ground-truth clearance / collision oracle in every run.
    /// Approximation: terrain between anchor points is inverse-distance interpolated (the Unity terrain is not in the
    /// manifest); in Unity runs the collider world replaces it for sensing.
    /// </summary>
    public sealed class ForestManifestWorld : IRayWorld, IGroundTruthWorld {
        // capsules: a(3) b(3) r, owner
        public float[] Ax, Ay, Az, Bx, By, Bz, R; public int[] Owner;
        public string[] OwnerNames;
        public int Count;
        public int Seed;
        public List<Vec3> Corridor = new List<Vec3>();
        public Vec3 Spawn; public double SpawnYawEnu;
        public double CorridorClearRadius;
        public double HardRadius = 0.01;
        // spatial grid (CSR)
        private double cell = 0.5, gx0, gy0, gz0; private int nx, ny, nz; private int[] start, items;
        private int[] stamp; private int stampId;
        // terrain
        private double hRes = 1.0, hx0, hy0; private int hnx, hny; private float[] h;
        public double MinX, MaxX, MinY, MaxY;

        public static ForestManifestWorld Load(string path) => FromJson(File.ReadAllText(path));

        public static ForestManifestWorld FromJson(string json) {
            var root = MiniJson.O(MiniJson.Parse(json));
            var w = new ForestManifestWorld();
            w.Seed = (int)MiniJson.D(MiniJson.Get(root, "seed"), 0);
            w.CorridorClearRadius = MiniJson.D(MiniJson.Get(root, "corridor_clear_radius_m"), 0);
            var ax = new List<float>(); var ay = new List<float>(); var az = new List<float>(); var bx = new List<float>(); var by = new List<float>(); var bz = new List<float>();
            var rr = new List<float>(); var own = new List<int>(); var names = new List<string>();
            var anchors = new List<(double x, double y, double z)>();
            void Cap(double[] s, int o) {   // Unity [ax,ay,az,bx,by,bz,r] -> ENU
                ax.Add((float)s[0]); ay.Add((float)s[2]); az.Add((float)s[1]); bx.Add((float)s[3]); by.Add((float)s[5]); bz.Add((float)s[4]); rr.Add((float)s[6]); own.Add(o);
            }
            foreach (var to in MiniJson.L(MiniJson.Get(root, "trees"))) {
                var t = MiniJson.O(to); var p = (double[])MiniJson.Get(t, "pos");
                anchors.Add((p[0], p[2], p[1]));
            }
            var fb = MiniJson.O(MiniJson.Get(root, "flight_band_branches"));
            foreach (var pto in MiniJson.L(MiniJson.Get(fb, "per_tree"))) {
                var pt = MiniJson.O(pto); int tid = (int)MiniJson.D(MiniJson.Get(pt, "tree"));
                int o = names.Count; names.Add("tree_" + tid);
                foreach (var so in MiniJson.L(MiniJson.Get(pt, "segments"))) if (so is double[] s && s.Length >= 7) Cap(s, o);
            }
            var mid = MiniJson.O(MiniJson.Get(root, "midstory"));
            foreach (var mo in MiniJson.L(MiniJson.Get(mid, "items"))) {
                var m = MiniJson.O(mo); int o = names.Count; names.Add("mid_" + (int)MiniJson.D(MiniJson.Get(m, "id")) + "_" + MiniJson.Get(m, "kind"));
                var p = MiniJson.Get(m, "pos") as double[]; if (p != null) anchors.Add((p[0], p[2], p[1]));
                foreach (var so in MiniJson.L(MiniJson.Get(m, "segments"))) if (so is double[] s && s.Length >= 7) Cap(s, o);
            }
            int pi = 0;
            foreach (var po in MiniJson.L(MiniJson.Get(root, "props"))) {
                var pr = MiniJson.O(po); var p = (double[])MiniJson.Get(pr, "pos"); string kind = (string)MiniJson.Get(pr, "kind");
                double size = MiniJson.D(MiniJson.Get(pr, "size_m"), 0.5), yaw = MiniJson.D(MiniJson.Get(pr, "yaw_deg"), 0) * Math.PI / 180;
                int o = names.Count; names.Add("prop_" + pi++ + "_" + kind);
                if (kind == "log") {
                    double rad = 0.25; string note = MiniJson.Get(pr, "note") as string ?? "";
                    int k = note.IndexOf("radius="); if (k >= 0) double.TryParse(note.Substring(k + 7).Split(' ', ',', ';')[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out rad);
                    double dx = Math.Sin(yaw) * size * 0.5, dz = Math.Cos(yaw) * size * 0.5;   // Unity yaw about +y: forward = (sin, cos) in (x, z)
                    Cap(new[] { p[0] - dx, p[1] + rad, p[2] - dz, p[0] + dx, p[1] + rad, p[2] + dz, rad }, o);
                } else {
                    Cap(new[] { p[0], p[1], p[2], p[0], p[1] + 1e-3, p[2], size * 0.5 }, o);
                    anchors.Add((p[0], p[2], p[1]));
                }
            }
            foreach (var to in MiniJson.L(MiniJson.Get(root, "trail"))) if (to is double[] p) anchors.Add((p[0], p[2], p[1]));
            foreach (var co in MiniJson.L(MiniJson.Get(root, "corridor_waypoints"))) if (co is double[] p) w.Corridor.Add(new Vec3(p[0], p[2], p[1]));
            var sp = MiniJson.O(MiniJson.Get(root, "spawn"));
            if (sp != null) {
                var p = (double[])MiniJson.Get(sp, "position"); w.Spawn = new Vec3(p[0], p[2], p[1]); anchors.Add((p[0], p[2], p[1]));
                double yu = MiniJson.D(MiniJson.Get(sp, "yaw_deg"), 0) * Math.PI / 180;   // Unity heading (from +z toward +x)
                w.SpawnYawEnu = Math.PI / 2 - yu;
            }
            w.Ax = ax.ToArray(); w.Ay = ay.ToArray(); w.Az = az.ToArray(); w.Bx = bx.ToArray(); w.By = by.ToArray(); w.Bz = bz.ToArray();
            w.R = rr.ToArray(); w.Owner = own.ToArray(); w.OwnerNames = names.ToArray(); w.Count = w.R.Length;
            w.BuildGrid(); w.BuildTerrain(anchors);
            return w;
        }

        /// <summary>Build from explicit capsules (tests and synthetic worlds). Ground is flat at groundZ.</summary>
        public static ForestManifestWorld FromCapsules(IList<(Vec3 a, Vec3 b, double r)> caps, double groundZ, double halfExtent) {
            var w = new ForestManifestWorld(); int n = caps.Count;
            w.Ax = new float[n]; w.Ay = new float[n]; w.Az = new float[n]; w.Bx = new float[n]; w.By = new float[n]; w.Bz = new float[n]; w.R = new float[n]; w.Owner = new int[n];
            w.OwnerNames = new string[n];
            for (int i = 0; i < n; i++) {
                w.Ax[i] = (float)caps[i].a.X; w.Ay[i] = (float)caps[i].a.Y; w.Az[i] = (float)caps[i].a.Z;
                w.Bx[i] = (float)caps[i].b.X; w.By[i] = (float)caps[i].b.Y; w.Bz[i] = (float)caps[i].b.Z; w.R[i] = (float)caps[i].r; w.Owner[i] = i; w.OwnerNames[i] = "cap_" + i;
            }
            w.Count = n;
            var anchors = new List<(double, double, double)> { (-halfExtent, -halfExtent, groundZ), (halfExtent, -halfExtent, groundZ), (-halfExtent, halfExtent, groundZ), (halfExtent, halfExtent, groundZ) };
            w.BuildGrid(-halfExtent, halfExtent, -halfExtent, halfExtent); w.BuildTerrain(anchors);
            return w;
        }

        void BuildGrid(double fx0 = double.NaN, double fx1 = 0, double fy0 = 0, double fy1 = 0) {
            double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue, z0 = double.MaxValue, z1 = double.MinValue;
            for (int i = 0; i < Count; i++) {
                x0 = Math.Min(x0, Math.Min(Ax[i], Bx[i]) - R[i]); x1 = Math.Max(x1, Math.Max(Ax[i], Bx[i]) + R[i]);
                y0 = Math.Min(y0, Math.Min(Ay[i], By[i]) - R[i]); y1 = Math.Max(y1, Math.Max(Ay[i], By[i]) + R[i]);
                z0 = Math.Min(z0, Math.Min(Az[i], Bz[i]) - R[i]); z1 = Math.Max(z1, Math.Max(Az[i], Bz[i]) + R[i]);
            }
            if (Count == 0) { x0 = y0 = z0 = -1; x1 = y1 = z1 = 1; }
            if (!double.IsNaN(fx0)) { x0 = Math.Min(x0, fx0); x1 = Math.Max(x1, fx1); y0 = Math.Min(y0, fy0); y1 = Math.Max(y1, fy1); }
            MinX = x0; MaxX = x1; MinY = y0; MaxY = y1;
            gx0 = x0 - 1; gy0 = y0 - 1; gz0 = z0 - 1;
            nx = (int)Math.Ceiling((x1 - gx0 + 1) / cell); ny = (int)Math.Ceiling((y1 - gy0 + 1) / cell); nz = (int)Math.Ceiling((z1 - gz0 + 1) / cell);
            var cnt = new int[nx * ny * nz + 1];
            const double pad = 0.1;
            for (int pass = 0; pass < 2; pass++) {
                for (int i = 0; i < Count; i++) {
                    double r = R[i] + pad;
                    int i0 = Cx(Math.Min(Ax[i], Bx[i]) - r), i1 = Cx(Math.Max(Ax[i], Bx[i]) + r);
                    int j0 = Cy(Math.Min(Ay[i], By[i]) - r), j1 = Cy(Math.Max(Ay[i], By[i]) + r);
                    int k0 = Cz(Math.Min(Az[i], Bz[i]) - r), k1 = Cz(Math.Max(Az[i], Bz[i]) + r);
                    for (int a = i0; a <= i1; a++) for (int b = j0; b <= j1; b++) for (int c = k0; c <= k1; c++) {
                        int id = (a * ny + b) * nz + c;
                        if (pass == 0) cnt[id + 1]++; else items[cnt[id]++] = i;
                    }
                }
                if (pass == 0) {
                    for (int k = 1; k < cnt.Length; k++) cnt[k] += cnt[k - 1];
                    start = (int[])cnt.Clone(); items = new int[cnt[cnt.Length - 1]];
                } else start = RestoreStart(cnt);
            }
            stamp = new int[Count];
        }

        int[] RestoreStart(int[] filledEnds) {
            // after fill, filledEnds[id] == original start[id+1]; rebuild start = [0, ends...]
            var s = new int[filledEnds.Length]; s[0] = 0;
            for (int k = 1; k < s.Length; k++) s[k] = filledEnds[k - 1];
            return s;
        }

        int Cx(double x) => Math.Max(0, Math.Min(nx - 1, (int)Math.Floor((x - gx0) / cell)));
        int Cy(double y) => Math.Max(0, Math.Min(ny - 1, (int)Math.Floor((y - gy0) / cell)));
        int Cz(double z) => Math.Max(0, Math.Min(nz - 1, (int)Math.Floor((z - gz0) / cell)));

        void BuildTerrain(List<(double x, double y, double z)> anchors) {
            double x0 = MinX - 5, x1 = MaxX + 5, y0 = MinY - 5, y1 = MaxY + 5;
            foreach (var a in anchors) { x0 = Math.Min(x0, a.x - 5); x1 = Math.Max(x1, a.x + 5); y0 = Math.Min(y0, a.y - 5); y1 = Math.Max(y1, a.y + 5); }
            hx0 = x0; hy0 = y0; hnx = (int)Math.Ceiling((x1 - x0) / hRes) + 1; hny = (int)Math.Ceiling((y1 - y0) / hRes) + 1;
            h = new float[hnx * hny];
            // bucket anchors (4 m) and IDW (power 2) over the nearest buckets, widening until >= 4 samples
            double B = 4.0; int bnx = (int)Math.Ceiling((x1 - x0) / B) + 1, bny = (int)Math.Ceiling((y1 - y0) / B) + 1;
            var buckets = new List<int>[bnx * bny];
            for (int i = 0; i < anchors.Count; i++) {
                int bi = (int)((anchors[i].x - x0) / B), bj = (int)((anchors[i].y - y0) / B);
                var l = buckets[bi * bny + bj] ??= new List<int>(); l.Add(i);
            }
            for (int i = 0; i < hnx; i++)
                for (int j = 0; j < hny; j++) {
                    double x = x0 + i * hRes, y = y0 + j * hRes; int bi = (int)((x - x0) / B), bj = (int)((y - y0) / B);
                    double sw = 0, sz = 0; int n = 0;
                    for (int rad = 1; rad <= 12 && n < 4; rad++) {
                        sw = sz = 0; n = 0;
                        for (int a = Math.Max(0, bi - rad); a <= Math.Min(bnx - 1, bi + rad); a++)
                            for (int b = Math.Max(0, bj - rad); b <= Math.Min(bny - 1, bj + rad); b++) {
                                var l = buckets[a * bny + b]; if (l == null) continue;
                                foreach (int k in l) {
                                    double d2 = (anchors[k].x - x) * (anchors[k].x - x) + (anchors[k].y - y) * (anchors[k].y - y) + 0.25;
                                    double wgt = 1.0 / d2; sw += wgt; sz += wgt * anchors[k].z; n++;
                                }
                            }
                    }
                    h[i * hny + j] = n > 0 ? (float)(sz / sw) : 0f;
                }
        }

        /// <summary>Optional replacement terrain (e.g. Unity Terrain.SampleHeight in ENU). Null = manifest interpolation.</summary>
        public Func<double, double, double> GroundProvider;

        public double GroundHeight(double x, double y) {
            if (GroundProvider != null) return GroundProvider(x, y);
            double fx = (x - hx0) / hRes, fy = (y - hy0) / hRes;
            int i = (int)Math.Floor(fx), j = (int)Math.Floor(fy);
            i = Math.Max(0, Math.Min(hnx - 2, i)); j = Math.Max(0, Math.Min(hny - 2, j));
            double u = Math.Max(0, Math.Min(1, fx - i)), v = Math.Max(0, Math.Min(1, fy - j));
            return (1 - u) * (1 - v) * h[i * hny + j] + u * (1 - v) * h[(i + 1) * hny + j] + (1 - u) * v * h[i * hny + j + 1] + u * v * h[(i + 1) * hny + j + 1];
        }

        public string ObjectName(int id) => id < 0 ? "terrain" : id < Count ? OwnerNames[Owner[id]] + "/seg" + id : "unknown";
        public string OwnerName(int capsule) => capsule < 0 ? "terrain" : OwnerNames[Owner[capsule]];

        // ------------------------------------------------------------------ geometry helpers
        /// <summary>Squared distance from p to segment i and the segment parameter.</summary>
        public double SegDist(int i, double px, double py, double pz) {
            double ux = Bx[i] - Ax[i], uy = By[i] - Ay[i], uz = Bz[i] - Az[i];
            double wx = px - Ax[i], wy = py - Ay[i], wz = pz - Az[i];
            double uu = ux * ux + uy * uy + uz * uz;
            double t = uu > 1e-12 ? Math.Max(0, Math.Min(1, (wx * ux + wy * uy + wz * uz) / uu)) : 0;
            double dx = wx - t * ux, dy = wy - t * uy, dz = wz - t * uz;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>Ray (o, d unit) vs capsule i with radius inflated by beam*t. Returns hit distance or +inf.</summary>
        double RayCapsule(int i, Vec3 o, Vec3 d, double beam) {
            double ux = Bx[i] - Ax[i], uy = By[i] - Ay[i], uz = Bz[i] - Az[i];
            double L = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            double r0 = R[i];
            double wx = o.X - Ax[i], wy = o.Y - Ay[i], wz = o.Z - Az[i];
            double t, s;   // ray param, segment param [0, L]
            if (L < 1e-9) { s = 0; t = -(wx * d.X + wy * d.Y + wz * d.Z); }
            else {
                ux /= L; uy /= L; uz /= L;
                double b = d.X * ux + d.Y * uy + d.Z * uz;     // cos angle
                double dw = d.X * wx + d.Y * wy + d.Z * wz, uw = ux * wx + uy * wy + uz * wz;
                double den = 1 - b * b;
                if (den < 1e-9) { s = Math.Max(0, Math.Min(L, uw)); t = s * b - dw; }
                else {
                    t = (b * uw - dw) / den; s = Math.Max(0, Math.Min(L, uw + b * t));
                    t = s * b - dw;   // re-project onto the ray for the clamped segment point
                }
            }
            if (t < 0) t = 0;
            double sx = Ax[i] + (L < 1e-9 ? 0 : ux * s), sy = Ay[i] + (L < 1e-9 ? 0 : uy * s), sz = Az[i] + (L < 1e-9 ? 0 : uz * s);
            double qx = o.X + d.X * t - sx, qy = o.Y + d.Y * t - sy, qz = o.Z + d.Z * t - sz;
            double dist = Math.Sqrt(qx * qx + qy * qy + qz * qz);
            double rr = r0 + beam * t;
            if (dist >= rr) return double.PositiveInfinity;
            // entry point: back off along the ray (exact for a sphere; divide by sin(angle) for cylinder sides)
            double sinA = L < 1e-9 ? 1.0 : Math.Sqrt(Math.Max(0.04, 1 - Math.Pow(d.X * ux + d.Y * uy + d.Z * uz, 2)));
            double th = t - Math.Sqrt(rr * rr - dist * dist) / sinA;
            return Math.Max(th, 0.0);
        }

        public bool Raycast(Vec3 o, Vec3 d, double maxRange, double beam, int ignore, out RayHit hit) {
            hit = new RayHit { Distance = double.PositiveInfinity, ObjectId = -2, Diameter = double.PositiveInfinity };
            stampId++; if (stampId == int.MaxValue) { Array.Clear(stamp, 0, stamp.Length); stampId = 1; }
            // 3D DDA over the capsule grid
            double best = maxRange; int bestId = -2;
            double ox = (o.X - gx0) / cell, oy = (o.Y - gy0) / cell, oz = (o.Z - gz0) / cell;
            int i = (int)Math.Floor(ox), j = (int)Math.Floor(oy), k = (int)Math.Floor(oz);
            int si = d.X > 0 ? 1 : -1, sj = d.Y > 0 ? 1 : -1, sk = d.Z > 0 ? 1 : -1;
            double tdx = Math.Abs(d.X) > 1e-12 ? cell / Math.Abs(d.X) : double.PositiveInfinity;
            double tdy = Math.Abs(d.Y) > 1e-12 ? cell / Math.Abs(d.Y) : double.PositiveInfinity;
            double tdz = Math.Abs(d.Z) > 1e-12 ? cell / Math.Abs(d.Z) : double.PositiveInfinity;
            double tmx = Math.Abs(d.X) > 1e-12 ? ((d.X > 0 ? i + 1 - ox : ox - i) * cell) / Math.Abs(d.X) : double.PositiveInfinity;
            double tmy = Math.Abs(d.Y) > 1e-12 ? ((d.Y > 0 ? j + 1 - oy : oy - j) * cell) / Math.Abs(d.Y) : double.PositiveInfinity;
            double tmz = Math.Abs(d.Z) > 1e-12 ? ((d.Z > 0 ? k + 1 - oz : oz - k) * cell) / Math.Abs(d.Z) : double.PositiveInfinity;
            double tEnter = 0;
            while (tEnter <= best) {
                if (i >= 0 && i < nx && j >= 0 && j < ny && k >= 0 && k < nz) {
                    int id = (i * ny + j) * nz + k;
                    for (int q = start[id]; q < start[id + 1]; q++) {
                        int c = items[q];
                        if (stamp[c] == stampId) continue;
                        stamp[c] = stampId;
                        if (c == ignore) continue;
                        double th = RayCapsule(c, o, d, beam);
                        if (th < best) { best = th; bestId = c; }
                    }
                } else if ((i < 0 && si < 0) || (i >= nx && si > 0) || (j < 0 && sj < 0) || (j >= ny && sj > 0) || (k < 0 && sk < 0) || (k >= nz && sk > 0)) break;
                if (tmx < tmy && tmx < tmz) { tEnter = tmx; tmx += tdx; i += si; }
                else if (tmy < tmz) { tEnter = tmy; tmy += tdy; j += sj; }
                else { tEnter = tmz; tmz += tdz; k += sk; }
            }
            // terrain
            double tg = TerrainRay(o, d, Math.Min(best, maxRange));
            if (tg < best) { best = tg; bestId = -1; }
            if (bestId == -2) return false;
            hit.Distance = best; hit.ObjectId = bestId; hit.Diameter = bestId >= 0 ? 2.0 * R[bestId] : double.PositiveInfinity;
            return true;
        }

        double TerrainRay(Vec3 o, Vec3 d, double maxT) {
            // sphere-tracing style march: step = max(0.2, 0.6 * height above terrain) (terrain slopes < ~55 deg), then bisection
            double f0 = o.Z - GroundHeight(o.X, o.Y);
            if (f0 <= 0) return 0;
            double prev = 0, t = 0, f = f0;
            while (t < maxT) {
                double step = Math.Max(0.2, 0.6 * f);
                double tt = Math.Min(t + step, maxT);
                var p = o + d * tt; f = p.Z - GroundHeight(p.X, p.Y);
                if (f <= 0) {
                    double a = t, b = tt;
                    for (int it = 0; it < 12; it++) { double m = 0.5 * (a + b); var q = o + d * m; if (q.Z - GroundHeight(q.X, q.Y) > 0) a = m; else b = m; }
                    return 0.5 * (a + b);
                }
                prev = t; t = tt;
            }
            return double.PositiveInfinity;
        }

        public double Clearance(Vec3 p, double search, bool hardOnly, out int objectId, out double objectRadius) {
            double best = p.Z - GroundHeight(p.X, p.Y); objectId = -1; objectRadius = double.PositiveInfinity;
            int i0 = Cx(p.X - search), i1 = Cx(p.X + search), j0 = Cy(p.Y - search), j1 = Cy(p.Y + search), k0 = Cz(p.Z - search), k1 = Cz(p.Z + search);
            stampId++; if (stampId == int.MaxValue) { Array.Clear(stamp, 0, stamp.Length); stampId = 1; }
            for (int a = i0; a <= i1; a++) for (int b = j0; b <= j1; b++) for (int c = k0; c <= k1; c++) {
                int id = (a * ny + b) * nz + c;
                for (int q = start[id]; q < start[id + 1]; q++) {
                    int s = items[q]; if (stamp[s] == stampId) continue; stamp[s] = stampId;
                    if (hardOnly && R[s] < HardRadius) continue;
                    double dd = SegDist(s, p.X, p.Y, p.Z) - R[s];
                    if (dd < best) { best = dd; objectId = s; objectRadius = R[s]; }
                }
            }
            return best;
        }

        /// <summary>Capsule indices whose AABB cell overlaps the box (used to rasterize priors).</summary>
        public void ForEachCapsuleIn(double x0, double x1, double y0, double y1, double z0, double z1, Action<int> f) {
            stampId++; if (stampId == int.MaxValue) { Array.Clear(stamp, 0, stamp.Length); stampId = 1; }
            for (int a = Cx(x0); a <= Cx(x1); a++) for (int b = Cy(y0); b <= Cy(y1); b++) for (int c = Cz(z0); c <= Cz(z1); c++) {
                int id = (a * ny + b) * nz + c;
                for (int q = start[id]; q < start[id + 1]; q++) { int s = items[q]; if (stamp[s] == stampId) continue; stamp[s] = stampId; f(s); }
            }
        }
    }
}
