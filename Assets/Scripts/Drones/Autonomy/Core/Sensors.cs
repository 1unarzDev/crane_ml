// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// OAK-D-S2-like stereo depth as a raycast depth image (M1). A decimated ray grid (default 80x50 over 80x55 deg)
    /// is cast against an <see cref="IRayWorld"/>; each ray carries its pixel footprint so structures thinner than a
    /// decimated pixel still register (as NavPilot's angular z-buffer). Stereo artefacts (NavPilot sensors.py):
    ///  * thin-structure dropout per struck structure per frame: P(valid) = logistic((w_eff - w50)/s) with native pixel
    ///    width w = f D / r and motion blur b = f (v_lat / r + |yaw rate|) t_exp, w_eff = w^2 / (w + b). A dropped
    ///    structure is seen THROUGH (the background behind it is returned: free space is carved through the branch);
    ///  * disparity noise: sigma_z = z^2 sigma_d / (f B);
    ///  * invalid pixels (texture-less, saturation) with probability InvalidP carry no information (NaN);
    ///  * rays without a return inside RangeMax carry free evidence to FreeEvidenceM (+inf).
    /// Timing: acquired at the tick where t >= next acquisition (rate DepthRateHz), available at acq + DepthLatencyS.
    /// The pose used is the truth pose at acquisition (no rolling shutter).
    /// </summary>
    public sealed class DepthRaycastSensor : IDepthSensor {
        private readonly AutonomyConfig c; private readonly IRayWorld world; private readonly Rng rng; private readonly ulong seed;
        private long nextNs; private long frameIdx;
        public readonly Pose Mount;
        public readonly double Fx, Fy, Cx, Cy, FNative, Beam;

        public DepthRaycastSensor(AutonomyConfig cfg, IRayWorld world, ulong seed, long firstNs = 0) {
            c = cfg; this.world = world; this.seed = seed; rng = new Rng(seed, 11); nextNs = firstNs;
            Fx = 0.5 * c.DepthWidth / Math.Tan(c.DepthHfovDeg * Math.PI / 360);
            Fy = 0.5 * c.DepthHeight / Math.Tan(c.DepthVfovDeg * Math.PI / 360);
            Cx = 0.5 * c.DepthWidth; Cy = 0.5 * c.DepthHeight;
            FNative = 0.5 * c.DepthNativeWidth / Math.Tan(c.DepthHfovDeg * Math.PI / 360);
            Beam = 0.5 * (c.DepthHfovDeg * Math.PI / 180) / c.DepthWidth;
            Mount = OpticalMount(new Vec3(c.DepthMountForward, 0, c.DepthMountUp), c.DepthPitchDeg * Math.PI / 180);
        }

        /// <summary>Optical frame (+x right, +y down, +z forward) to body FLU, pitched down by pitch (rad, + = nose down view).</summary>
        public static Pose OpticalMount(Vec3 offsetBody, double pitchDown) {
            // R columns: optical x -> body -y, optical y -> body -z, optical z -> body +x; then pitch about body y
            double cp = Math.Cos(pitchDown), sp = Math.Sin(pitchDown);
            var ox = new Vec3(0, -1, 0); var oy = new Vec3(-sp, 0, -cp); var oz = new Vec3(cp, 0, -sp);
            return new Pose { P = offsetBody, Qw = 1 }.WithAxes(ox, oy, oz);
        }

        public DepthFrame Tick(long tNs, Pose body, Vec3 vel, Vec3 omegaBody) {
            if (tNs < nextNs) return null;
            nextNs += (long)Math.Round(1e9 / c.DepthRateHz);
            frameIdx++;
            var camWorld = Compose(body, Mount);
            var f = new DepthFrame {
                AcqNs = tNs, AvailNs = tNs + (long)Math.Round(c.DepthLatencyS * 1e9), Width = c.DepthWidth, Height = c.DepthHeight,
                FxPx = Fx, FyPx = Fy, CxPx = Cx, CyPx = Cy, Depth = new float[c.DepthWidth * c.DepthHeight],
                RangeMax = c.DepthRangeMax, FreeEvidenceM = c.FreeEvidenceM, TruthPose = camWorld, MountInBody = Mount
            };
            var fwd = camWorld.Rotate(new Vec3(0, 0, 1));
            double yawRate = Math.Abs(omegaBody.Z);
            var dropCache = new Dictionary<int, bool>();
            int dropped = 0;
            for (int v = 0; v < c.DepthHeight; v++)
                for (int u = 0; u < c.DepthWidth; u++) {
                    int idx = v * c.DepthWidth + u;
                    if (rng.Uniform() < c.InvalidP) { f.Depth[idx] = float.NaN; f.InvalidPixels++; continue; }
                    var dOpt = new Vec3((u + 0.5 - Cx) / Fx, (v + 0.5 - Cy) / Fy, 1).Normalized;
                    var d = camWorld.Rotate(dOpt);
                    double tAcc = 0; int ignore = -2; bool got = false; RayHit h = default;
                    for (int pass = 0; pass < 4; pass++) {
                        var o = camWorld.P + d * tAcc;
                        if (!world.Raycast(o, d, c.DepthRangeMax - tAcc, Beam, ignore, out h)) break;
                        h.Distance += tAcc;
                        if (h.ObjectId < 0 || double.IsInfinity(h.Diameter)) { got = true; break; }
                        if (!dropCache.TryGetValue(h.ObjectId, out bool drop)) {
                            double r = Math.Max(h.Distance, 0.05);
                            var vLatV = vel - d * vel.Dot(d); double vLat = vLatV.Norm;
                            double w = FNative * h.Diameter / r, b = FNative * (vLat / r + yawRate) * c.ExposureS;
                            double weff = w * w / (w + b + 1e-9);
                            double p = 1.0 / (1.0 + Math.Exp(-(weff - c.ThinW50Px) / c.ThinWScalePx));
                            drop = Hash01(seed, (ulong)frameIdx, (ulong)h.ObjectId) >= p;
                            dropCache[h.ObjectId] = drop; if (drop) dropped++;
                        }
                        if (!drop) { got = true; break; }
                        tAcc = h.Distance + 1e-3; ignore = h.ObjectId;
                        if (tAcc >= c.DepthRangeMax) break;
                    }
                    if (!got || h.Distance > c.DepthRangeMax) { f.Depth[idx] = float.PositiveInfinity; continue; }
                    double z = h.Distance * d.Dot(fwd);
                    double sig = z * z * c.DispSigmaPx / (FNative * c.BaselineM);
                    z += sig * rng.Normal();
                    if (z < c.DepthRangeMin || z > c.DepthRangeMax) { f.Depth[idx] = z < c.DepthRangeMin ? float.NaN : float.PositiveInfinity; continue; }
                    f.Depth[idx] = (float)z; f.TruthHits++;
                }
            f.DroppedObjects = dropped;
            return f;
        }

        public static Pose Compose(Pose a, Pose b) {
            // a * b (rotation then translation): world = a.P + a.R (b.P + b.R x)
            var r = new Pose {
                P = a.P + a.Rotate(b.P),
                Qw = a.Qw * b.Qw - a.Qx * b.Qx - a.Qy * b.Qy - a.Qz * b.Qz,
                Qx = a.Qw * b.Qx + a.Qx * b.Qw + a.Qy * b.Qz - a.Qz * b.Qy,
                Qy = a.Qw * b.Qy - a.Qx * b.Qz + a.Qy * b.Qw + a.Qz * b.Qx,
                Qz = a.Qw * b.Qz + a.Qx * b.Qy - a.Qy * b.Qx + a.Qz * b.Qw };
            return r;
        }

        public static double Hash01(ulong a, ulong b, ulong c) {
            ulong z = a * 0x9E3779B97F4A7C15UL ^ (b + 0x632BE59BD9B4E019UL) * 0xBF58476D1CE4E5B9UL ^ (c + 0x94D049BB133111EBUL) * 0xD6E8FEB86659FD93UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL; z ^= z >> 31;
            return (z >> 11) * (1.0 / 9007199254740992.0);
        }
    }

    public static class PoseExt {
        /// <summary>Pose with rotation whose columns are the given orthonormal axes (body coordinates of the child axes).</summary>
        public static Pose WithAxes(this Pose p, Vec3 x, Vec3 y, Vec3 z) {
            double m00 = x.X, m01 = y.X, m02 = z.X, m10 = x.Y, m11 = y.Y, m12 = z.Y, m20 = x.Z, m21 = y.Z, m22 = z.Z;
            double tr = m00 + m11 + m22, qw, qx, qy, qz;
            if (tr > 0) { double s = Math.Sqrt(tr + 1) * 2; qw = 0.25 * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s; }
            else if (m00 > m11 && m00 > m22) { double s = Math.Sqrt(1 + m00 - m11 - m22) * 2; qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s; }
            else if (m11 > m22) { double s = Math.Sqrt(1 + m11 - m00 - m22) * 2; qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s; }
            else { double s = Math.Sqrt(1 + m22 - m00 - m11) * 2; qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s; }
            return new Pose { P = p.P, Qw = qw, Qx = qx, Qy = qy, Qz = qz };
        }
    }

    /// <summary>
    /// Unitree-L2-like LiDAR (M1): non-repetitive scan (golden-angle azimuth, equal-area elevation between
    /// LidarElevMinDeg and LidarElevMaxDeg, rotated every sweep), LidarPointsPerS points, LidarRateHz sweeps.
    /// Each point is cast at its own sample time from the truth pose at that tick (5 ms granularity), and stored in the
    /// sensor frame of that instant: the scan carries real motion skew; consumers must deskew with per-point stamps.
    /// Range noise sigma 2 cm, random per-point dropout; thin structures are hit only as often as the beam pattern
    /// actually intersects them (no extra dropout). No return within RangeMax -> a miss direction (free evidence).
    /// </summary>
    public sealed class LidarL2Sensor : ILidarSensor {
        private readonly AutonomyConfig c; private readonly IRayWorld world; private readonly Rng rng;
        private long sweepStart; private long sweepNs; private int perSweep; private int k; private long sweepIdx;
        private List<float> x = new List<float>(), y = new List<float>(), z = new List<float>(), mx = new List<float>(), my = new List<float>(), mz = new List<float>();
        private List<int> off = new List<int>(), moff = new List<int>();
        public readonly Pose Mount;

        public LidarL2Sensor(AutonomyConfig cfg, IRayWorld world, ulong seed, long firstNs = 13_000_000) {
            c = cfg; this.world = world; rng = new Rng(seed, 12);
            sweepNs = (long)Math.Round(1e9 / c.LidarRateHz); perSweep = (int)Math.Round(c.LidarPointsPerS / c.LidarRateHz);
            sweepStart = firstNs - (firstNs % c.TickNs);
            Mount = new Pose { P = new Vec3(0, 0, c.LidarMountUp), Qw = 1 };
        }

        public LidarScan Tick(long tNs, long tickNs, Pose body) {
            if (tNs < sweepStart) return null;
            var sensor = DepthRaycastSensor.Compose(body, Mount);
            long tickEnd = tNs + tickNs;
            double s0 = Math.Sin(c.LidarElevMinDeg * Math.PI / 180), s1 = Math.Sin(c.LidarElevMaxDeg * Math.PI / 180);
            const double golden = 2.399963229728653;   // 2 pi (1 - 1/phi)
            double rot = sweepIdx * 0.7853981;
            while (k < perSweep) {
                long tp = sweepStart + (long)((double)k / perSweep * sweepNs);
                if (tp >= tickEnd) break;
                double az = k * golden + rot;
                double se = s0 + (s1 - s0) * ((k * 0.7548776662466927 + sweepIdx * 0.5698402909980532) % 1.0);
                double ce = Math.Sqrt(1 - se * se);
                var dS = new Vec3(ce * Math.Cos(az), ce * Math.Sin(az), se);
                int o = (int)(tp - sweepStart);
                k++;
                if (rng.Uniform() < c.LidarDropP) continue;
                var dW = sensor.Rotate(dS);
                if (world.Raycast(sensor.P, dW, c.LidarRangeMax, 0.0, -2, out RayHit h) && h.Distance >= c.LidarRangeMin) {
                    double r = h.Distance + c.LidarRangeSigma * rng.Normal();
                    x.Add((float)(dS.X * r)); y.Add((float)(dS.Y * r)); z.Add((float)(dS.Z * r)); off.Add(o);
                } else { mx.Add((float)dS.X); my.Add((float)dS.Y); mz.Add((float)dS.Z); moff.Add(o); }
            }
            if (k < perSweep) return null;
            var scan = new LidarScan {
                StartNs = sweepStart, EndNs = sweepStart + sweepNs, AvailNs = sweepStart + sweepNs + (long)Math.Round(c.LidarLatencyS * 1e9),
                X = x.ToArray(), Y = y.ToArray(), Z = z.ToArray(), OffsetNs = off.ToArray(),
                MissX = mx.ToArray(), MissY = my.ToArray(), MissZ = mz.ToArray(), MissOffsetNs = moff.ToArray(), MountInBody = Mount, RangeMax = c.LidarRangeMax };
            x.Clear(); y.Clear(); z.Clear(); off.Clear(); mx.Clear(); my.Clear(); mz.Clear(); moff.Clear();
            sweepStart += sweepNs; k = 0; sweepIdx++;
            return scan;
        }
    }
}
