// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// M3 local map: sparse log-odds voxels (16^3 chunks) in the estimator's local frame. Depth pixels and LiDAR
    /// points are inserted at the ESTIMATED pose (registration error = estimator drift). Free space is carved by exact
    /// voxel traversal from the sensor to (hit - CarveBackoffM), or to the free-evidence length for no-return rays,
    /// applying the NavPilot conservative miss update (l_miss -0.1 vs l_hit +0.85, clamp [-1, 3.5]) at most once per
    /// voxel per frame. LiDAR points are deskewed with the estimator's pose at each point's own stamp.
    /// </summary>
    public sealed class SparseVoxelMap : IOccupancyMap {
        const int CB = 4, CS = 1 << CB, CM = CS - 1, CN = CS * CS * CS;
        sealed class Chunk { public readonly float[] L = new float[CN]; public readonly int[] Seen = new int[CN]; public readonly bool[] Obs = new bool[CN]; }
        private readonly Dictionary<long, Chunk> chunks = new Dictionary<long, Chunk>();
        private readonly AutonomyConfig c;
        private readonly double res, inv;
        private int frameId;
        private long lastKey = long.MinValue; private Chunk lastChunk;
        public double Resolution => res;
        public long OccupiedVoxels { get; private set; }
        public long KnownVoxels { get; private set; }
        public long Inserts { get; private set; }

        public SparseVoxelMap(AutonomyConfig cfg) { c = cfg; res = cfg.MapRes; inv = 1.0 / res; }

        public void Clear() { chunks.Clear(); lastChunk = null; lastKey = long.MinValue; OccupiedVoxels = KnownVoxels = 0; }

        static long Key(int cx, int cy, int cz) => ((long)(cx & 0x1FFFFF) << 42) | ((long)(cy & 0x1FFFFF) << 21) | (long)(cz & 0x1FFFFF);

        Chunk Get(int ix, int iy, int iz, bool create) {
            int cx = ix >> CB, cy = iy >> CB, cz = iz >> CB; long k = Key(cx, cy, cz);
            if (k == lastKey && lastChunk != null) return lastChunk;
            if (!chunks.TryGetValue(k, out var ch)) { if (!create) return null; ch = new Chunk(); chunks[k] = ch; }
            lastKey = k; lastChunk = ch; return ch;
        }
        static int Idx(int ix, int iy, int iz) => ((ix & CM) << (2 * CB)) | ((iy & CM) << CB) | (iz & CM);

        public int Vx(double x) => (int)Math.Floor(x * inv);

        public float LogOddsAt(int ix, int iy, int iz) { var ch = Get(ix, iy, iz, false); return ch == null ? 0f : ch.L[Idx(ix, iy, iz)]; }
        public bool ObservedAt(int ix, int iy, int iz) { var ch = Get(ix, iy, iz, false); return ch != null && ch.Obs[Idx(ix, iy, iz)]; }
        public float LogOdds(Vec3 p) => LogOddsAt(Vx(p.X), Vx(p.Y), Vx(p.Z));
        public bool Known(Vec3 p) => ObservedAt(Vx(p.X), Vx(p.Y), Vx(p.Z));
        public bool Occupied(Vec3 p) => LogOdds(p) > c.LOcc;

        void Update(int ix, int iy, int iz, float delta, bool once) {
            var ch = Get(ix, iy, iz, true); int i = Idx(ix, iy, iz);
            if (once) { if (ch.Seen[i] == frameId) return; ch.Seen[i] = frameId; }
            float before = ch.L[i];
            if (!ch.Obs[i]) { ch.Obs[i] = true; KnownVoxels++; }
            float after = Math.Max(c.LMin, Math.Min(c.LMax, before + delta));
            ch.L[i] = after;
            if (before <= c.LOcc && after > c.LOcc) OccupiedVoxels++; else if (before > c.LOcc && after <= c.LOcc) OccupiedVoxels--;
        }

        /// <summary>Carve free space from o along unit d for length len (exact voxel traversal), then optionally mark a hit at end.</summary>
        void Ray(Vec3 o, Vec3 d, double carveLen, Vec3? hit) {
            if (carveLen > 0) {
                int ix = Vx(o.X), iy = Vx(o.Y), iz = Vx(o.Z);
                int sx = d.X > 0 ? 1 : -1, sy = d.Y > 0 ? 1 : -1, sz = d.Z > 0 ? 1 : -1;
                double tdx = Math.Abs(d.X) > 1e-12 ? res / Math.Abs(d.X) : 1e30, tdy = Math.Abs(d.Y) > 1e-12 ? res / Math.Abs(d.Y) : 1e30, tdz = Math.Abs(d.Z) > 1e-12 ? res / Math.Abs(d.Z) : 1e30;
                double fx = o.X * inv - ix, fy = o.Y * inv - iy, fz = o.Z * inv - iz;
                double tmx = Math.Abs(d.X) > 1e-12 ? (d.X > 0 ? 1 - fx : fx) * tdx : 1e30, tmy = Math.Abs(d.Y) > 1e-12 ? (d.Y > 0 ? 1 - fy : fy) * tdy : 1e30, tmz = Math.Abs(d.Z) > 1e-12 ? (d.Z > 0 ? 1 - fz : fz) * tdz : 1e30;
                double t = 0;
                while (t < carveLen) {
                    Update(ix, iy, iz, c.LMiss, true);
                    if (tmx < tmy && tmx < tmz) { t = tmx; tmx += tdx; ix += sx; }
                    else if (tmy < tmz) { t = tmy; tmy += tdy; iy += sy; }
                    else { t = tmz; tmz += tdz; iz += sz; }
                }
            }
            if (hit.HasValue) { var h = hit.Value; Update(Vx(h.X), Vx(h.Y), Vx(h.Z), c.LHit, false); }
        }

        public void InsertDepth(DepthFrame f, Pose cam, long nowNs) {
            frameId++; Inserts++;
            const double carveCap = 12.0;
            for (int v = 0; v < f.Height; v++)
                for (int u = 0; u < f.Width; u++) {
                    float z = f.Depth[v * f.Width + u];
                    if (float.IsNaN(z)) continue;
                    var dOpt = new Vec3((u + 0.5 - f.CxPx) / f.FxPx, (v + 0.5 - f.CyPx) / f.FyPx, 1);
                    double scale = dOpt.Norm; var dU = dOpt * (1.0 / scale);
                    var d = cam.Rotate(dU);
                    if (float.IsPositiveInfinity(z)) { Ray(cam.P, d, Math.Min(f.FreeEvidenceM, carveCap), null); continue; }
                    double r = z * scale;
                    Ray(cam.P, d, Math.Min(r - c.CarveBackoffM, carveCap), cam.P + d * r);
                }
        }

        public void InsertLidar(LidarScan s, Func<long, Pose?> bodyPoseAt, long nowNs) {
            frameId++; Inserts++;
            const double carveCap = 12.0;
            Pose? cached = null; long cachedT = long.MinValue;
            Pose SensorAt(long t) {
                long tq = t - t % c.TickNs;   // tick granularity (the sensor casts per tick)
                if (tq != cachedT) { cachedT = tq; var b = bodyPoseAt(tq); cached = b.HasValue ? DepthRaycastSensor.Compose(b.Value, s.MountInBody) : (Pose?)null; }
                return cached ?? default;
            }
            for (int i = 0; i < s.Count; i++) {
                var sp = SensorAt(s.StartNs + s.OffsetNs[i]); if (cached == null) continue;
                var pS = new Vec3(s.X[i], s.Y[i], s.Z[i]); double r = pS.Norm; if (r < 1e-3) continue;
                var d = sp.Rotate(pS * (1.0 / r));
                Ray(sp.P, d, Math.Min(r - c.CarveBackoffM, carveCap), sp.P + d * r);
            }
            for (int i = 0; i < s.MissX.Length; i++) {
                var sp = SensorAt(s.StartNs + s.MissOffsetNs[i]); if (cached == null) continue;
                var d = sp.Rotate(new Vec3(s.MissX[i], s.MissY[i], s.MissZ[i]));
                Ray(sp.P, d, Math.Min(s.RangeMax, carveCap), null);
            }
        }

        /// <summary>Copy a box of voxels into dense arrays (occupied / observed). i in [ix0, ix0+nx), etc.</summary>
        public void Extract(int ix0, int iy0, int iz0, int nx, int ny, int nz, bool[] occ, bool[] obs) {
            Array.Clear(occ, 0, nx * ny * nz); Array.Clear(obs, 0, nx * ny * nz);
            int cx0 = ix0 >> CB, cx1 = (ix0 + nx - 1) >> CB, cy0 = iy0 >> CB, cy1 = (iy0 + ny - 1) >> CB, cz0 = iz0 >> CB, cz1 = (iz0 + nz - 1) >> CB;
            for (int cx = cx0; cx <= cx1; cx++) for (int cy = cy0; cy <= cy1; cy++) for (int cz = cz0; cz <= cz1; cz++) {
                if (!chunks.TryGetValue(Key(cx, cy, cz), out var ch)) continue;
                int bx = cx << CB, by = cy << CB, bz = cz << CB;
                for (int a = Math.Max(0, ix0 - bx); a < Math.Min(CS, ix0 + nx - bx); a++)
                    for (int b = Math.Max(0, iy0 - by); b < Math.Min(CS, iy0 + ny - by); b++)
                        for (int q = Math.Max(0, iz0 - bz); q < Math.Min(CS, iz0 + nz - bz); q++) {
                            int li = (a << (2 * CB)) | (b << CB) | q;
                            if (!ch.Obs[li]) continue;
                            int wi = ((bx + a - ix0) * ny + (by + b - iy0)) * nz + (bz + q - iz0);
                            obs[wi] = true; if (ch.L[li] > c.LOcc) occ[wi] = true;
                        }
            }
        }
    }

    /// <summary>
    /// Dense planning window around the vehicle (local frame): occupied / observed flags copied from the voxel map and an
    /// exact Euclidean distance transform (Felzenszwalb-Huttenlocher) to the nearest occupied voxel, i.e. a window ESDF
    /// (positive side). Outside the window counts as blocked.
    /// </summary>
    public sealed class PlanningWindow {
        public int Nx, Ny, Nz, Ix0, Iy0, Iz0; public double Res;
        public bool[] Occ, Obs; public float[] Dist;   // Dist in metres to nearest occupied voxel centre
        private double[] f, d; private int[] v; private double[] z;

        public void Build(SparseVoxelMap map, Vec3 center, double halfXY, double zDown, double zUp, IPriorMap prior = null, MapAlignment? align = null) {
            Res = map.Resolution;
            int hx = (int)Math.Ceiling(halfXY / Res), zd = (int)Math.Ceiling(zDown / Res), zu = (int)Math.Ceiling(zUp / Res);
            Nx = Ny = 2 * hx + 1; Nz = zd + zu + 1;
            Ix0 = map.Vx(center.X) - hx; Iy0 = map.Vx(center.Y) - hx; Iz0 = map.Vx(center.Z) - zd;
            int n = Nx * Ny * Nz;
            if (Occ == null || Occ.Length != n) { Occ = new bool[n]; Obs = new bool[n]; Dist = new float[n]; }
            map.Extract(Ix0, Iy0, Iz0, Nx, Ny, Nz, Occ, Obs);
            if (prior != null && align.HasValue) {
                var off = align.Value.Offset;
                for (int i = 0; i < Nx; i++) for (int j = 0; j < Ny; j++) for (int k = 0; k < Nz; k++) {
                    var p = CellCenter(i, j, k) + off;
                    if (prior.Occupied(p)) { int w = (i * Ny + j) * Nz + k; Occ[w] = true; Obs[w] = true; }
                }
            }
            Edt();
        }

        public Vec3 CellCenter(int i, int j, int k) => new Vec3((Ix0 + i + 0.5) * Res, (Iy0 + j + 0.5) * Res, (Iz0 + k + 0.5) * Res);

        public bool Index(Vec3 p, out int w) {
            int i = (int)Math.Floor(p.X / Res) - Ix0, j = (int)Math.Floor(p.Y / Res) - Iy0, k = (int)Math.Floor(p.Z / Res) - Iz0;
            w = -1;
            if (i < 0 || j < 0 || k < 0 || i >= Nx || j >= Ny || k >= Nz) return false;
            w = (i * Ny + j) * Nz + k; return true;
        }

        /// <summary>Truncated exact distance (voxel-centre metric) up to TruncCells voxels: each occupied voxel stamps its
        /// ball; farther cells get TruncCells * Res + Res. Same answers as a full EDT wherever the planner thresholds it.</summary>
        public int TruncCells = 3;
        private int[] ballOff; private float[] ballDist; private int ballNy = -1, ballNz = -1, ballK = -1;
        void Edt() {
            int n = Nx * Ny * Nz; float far = (float)((TruncCells + 1) * Res);
            for (int w = 0; w < n; w++) Dist[w] = far;
            int K = TruncCells;
            if (ballNy != Ny || ballNz != Nz || ballK != K) {
                var o = new List<int>(); var dd = new List<float>(); var di = new List<int>(); 
                for (int a = -K; a <= K; a++) for (int b = -K; b <= K; b++) for (int q = -K; q <= K; q++) {
                    int r2 = a * a + b * b + q * q; if (r2 > K * K) continue;
                    o.Add((a * Ny + b) * Nz + q); dd.Add((float)(Math.Sqrt(r2) * Res)); di.Add(a); di.Add(b); di.Add(q);
                }
                ballOff = o.ToArray(); ballDist = dd.ToArray(); ballIjk = di.ToArray(); ballNy = Ny; ballNz = Nz; ballK = K;
            }
            for (int i = 0; i < Nx; i++) for (int j = 0; j < Ny; j++) for (int k = 0; k < Nz; k++) {
                int w = (i * Ny + j) * Nz + k; if (!Occ[w]) continue;
                bool interior = i >= K && j >= K && k >= K && i < Nx - K && j < Ny - K && k < Nz - K;
                for (int m = 0; m < ballOff.Length; m++) {
                    if (!interior) {
                        int a = i + ballIjk[3 * m], b = j + ballIjk[3 * m + 1], q = k + ballIjk[3 * m + 2];
                        if (a < 0 || b < 0 || q < 0 || a >= Nx || b >= Ny || q >= Nz) continue;
                    }
                    int x = w + ballOff[m]; if (ballDist[m] < Dist[x]) Dist[x] = ballDist[m];
                }
            }
        }
        private int[] ballIjk;

        void Dt1(int n) {
            int k = 0; v[0] = 0; z[0] = -1e30; z[1] = 1e30;
            for (int q = 1; q < n; q++) {
                double s;
                while (true) {
                    s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
                    if (s <= z[k] && k > 0) k--; else break;
                }
                if (s <= z[k]) { v[0] = q; z[0] = -1e30; z[1] = 1e30; k = 0; continue; }
                k++; v[k] = q; z[k] = s; z[k + 1] = 1e30;
            }
            k = 0;
            for (int q = 0; q < n; q++) { while (z[k + 1] < q) k++; double dq = q - v[k]; d[q] = dq * dq + f[v[k]]; }
        }
    }
}
