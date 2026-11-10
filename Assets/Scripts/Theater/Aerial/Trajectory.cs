using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Crane.Theater.Aerial {
    /// <summary>One pose of the quad, Unity axes (+x right, +y up, +z forward), metres. Rotor speeds are rad/s,
    /// ArduPilot Quad-X order (M1 FR, M2 RL, M3 FL, M4 RR), as DroneVisualRig.SetRotorSpeeds expects.</summary>
    public struct TrajSample {
        public double t;
        public Vector3 pos, vel;
        public Quaternion rot;
        public float[] rotorRadS;
    }

    /// <summary>A flight as a pure function of time. Every source (scripted, recorded, plant-baked) ends up as a
    /// <see cref="TrajectoryTable"/> so a frame is a pure function of its index.</summary>
    public interface ITrajectory {
        double Duration { get; }
        TrajSample Sample(double t);
        IReadOnlyDictionary<string, double> Marks { get; }
        string Describe { get; }
    }

    /// <summary>Plant seam. The default factory returns <see cref="CranePlantAdapter"/> (the real aerial-v2 plant, Crane.Aerial.Physics);
    /// <see cref="KinematicStubPlant"/> is used only if the profile cannot be loaded.</summary>
    public interface IAerialPlantFollower {
        void Reset(Vector3 pos, Quaternion rot, int seed);
        /// <summary>Advance dt seconds towards the reference (pose and velocity); returns the plant's state.</summary>
        TrajSample Step(double dt, TrajSample reference);
        string Name { get; }
    }

    public static class AerialPlants {
        public static Func<IAerialPlantFollower> Factory = () => CranePlantAdapter.TryCreate() ?? (IAerialPlantFollower)new KinematicStubPlant();
    }

    /// <summary>Stand-in plant: critically damped position tracking (wn 5 rad/s), reference attitude. Visual only.</summary>
    public sealed class KinematicStubPlant : IAerialPlantFollower {
        Vector3 p, v;
        public string Name => "kinematic-stub (NOT the physics plant)";
        public void Reset(Vector3 pos, Quaternion rot, int seed) { p = pos; v = Vector3.zero; }
        public TrajSample Step(double dt, TrajSample r) {
            float d = (float)dt, wn = 5f;
            Vector3 a = wn * wn * (r.pos - p) - 2f * wn * v;
            v += a * d; p += v * d;
            return new TrajSample { t = r.t + dt, pos = p, vel = v, rot = r.rot, rotorRadS = r.rotorRadS };
        }
    }

    /// <summary>Uniformly sampled flight.</summary>
    public sealed class TrajectoryTable : ITrajectory {
        public readonly double Dt;
        readonly Vector3[] pos, vel;
        readonly Quaternion[] rot;
        readonly float[][] rotor;
        public IReadOnlyDictionary<string, double> Marks { get; }
        public string Describe { get; }
        public double Duration => (pos.Length - 1) * Dt;
        public int Count => pos.Length;

        public TrajectoryTable(double dt, Vector3[] p, Vector3[] v, Quaternion[] q, float[][] w, IReadOnlyDictionary<string, double> marks, string describe) {
            Dt = dt; pos = p; vel = v; rot = q; rotor = w; Marks = marks ?? new Dictionary<string, double>(); Describe = describe;
            if (p.Length < 2) throw new ArgumentException("trajectory needs at least 2 samples");
        }

        public TrajSample Sample(double t) {
            double x = Math.Max(0, Math.Min(Duration, t)) / Dt;
            int i = Math.Min((int)x, pos.Length - 2);
            float u = (float)(x - i);
            var s = new TrajSample {
                t = t, pos = Vector3.Lerp(pos[i], pos[i + 1], u), vel = Vector3.Lerp(vel[i], vel[i + 1], u),
                rot = Quaternion.Slerp(rot[i], rot[i + 1], u), rotorRadS = new float[4]
            };
            for (int k = 0; k < 4; k++) s.rotorRadS[k] = Mathf.Lerp(rotor[i][k], rotor[i + 1][k], u);
            return s;
        }

        /// <summary>Resample a reference through a plant (deterministic for a seed) into a new table.</summary>
        public static TrajectoryTable BakeThroughPlant(ITrajectory reference, IAerialPlantFollower plant, int seed, double dt = 1.0 / 200) {
            int n = (int)Math.Ceiling(reference.Duration / dt) + 1;
            var p = new Vector3[n]; var v = new Vector3[n]; var q = new Quaternion[n]; var w = new float[n][];
            var s0 = reference.Sample(0);
            plant.Reset(s0.pos, s0.rot, seed);
            for (int i = 0; i < n; i++) {
                var r = reference.Sample(i * dt);
                var s = i == 0 ? s0 : plant.Step(dt, r);
                p[i] = s.pos; v[i] = s.vel; q[i] = s.rot; w[i] = s.rotorRadS ?? r.rotorRadS;
            }
            return new TrajectoryTable(dt, p, v, q, w, reference.Marks, reference.Describe + " -> " + plant.Name);
        }

        /// <summary>CSV: header names columns; needs t, px, py, pz, qx, qy, qz, qw; optional w1..w4 (rad/s). A line
        /// "# frame=unity" (default) or "# frame=enu_flu" (ENU world, FLU body, x east y north z up) selects the convention.
        /// Resampled to a uniform 120 Hz table.</summary>
        public static TrajectoryTable FromCsv(string text, string name) {
            bool enu = false; string[] head = null;
            var rows = new List<double[]>();
            foreach (var raw in text.Split('\n')) {
                string line = raw.Trim();
                if (line.Length == 0) continue;
                if (line[0] == '#') { if (line.Replace(" ", "").Contains("frame=enu_flu")) enu = true; continue; }
                var f = line.Split(',');
                if (head == null) { head = f; for (int i = 0; i < head.Length; i++) head[i] = head[i].Trim().ToLowerInvariant(); continue; }
                var r = new double[f.Length];
                for (int i = 0; i < f.Length; i++) r[i] = double.Parse(f[i], CultureInfo.InvariantCulture);
                rows.Add(r);
            }
            if (head == null || rows.Count < 2) throw new InvalidDataException("trajectory csv: no data");
            int Col(string c, bool must = true) { int i = Array.IndexOf(head, c); if (i < 0 && must) throw new InvalidDataException("trajectory csv: missing column " + c); return i; }
            int ct = Col("t"), cx = Col("px"), cy = Col("py"), cz = Col("pz"), qx = Col("qx"), qy = Col("qy"), qz = Col("qz"), qw = Col("qw");
            int[] cw = { Col("w1", false), Col("w2", false), Col("w3", false), Col("w4", false) };
            int n = rows.Count;
            var t = new double[n]; var P = new Vector3[n]; var Q = new Quaternion[n]; var W = new float[n][];
            for (int i = 0; i < n; i++) {
                var r = rows[i]; t[i] = r[ct];
                if (i > 0 && t[i] <= t[i - 1]) throw new InvalidDataException("trajectory csv: time not increasing at row " + i);
                var pe = new Vector3((float)r[cx], (float)r[cy], (float)r[cz]);
                var qe = new Quaternion((float)r[qx], (float)r[qy], (float)r[qz], (float)r[qw]);
                if (enu) { P[i] = EnuToUnity(pe); Q[i] = EnuFluToUnity(qe); } else { P[i] = pe; Q[i] = qe; }
                W[i] = new float[4];
                for (int k = 0; k < 4; k++) W[i][k] = cw[k] >= 0 ? (float)r[cw[k]] : TrajectoryBaker.HoverOmega;
            }
            double dt = 1.0 / 120; double dur = t[n - 1] - t[0];
            int m = (int)Math.Floor(dur / dt) + 1;
            if (m < 2) throw new InvalidDataException("trajectory csv: too short");
            var p = new Vector3[m]; var v = new Vector3[m]; var q = new Quaternion[m]; var w = new float[m][];
            int j = 0;
            for (int i = 0; i < m; i++) {
                double ti = t[0] + i * dt;
                while (j < n - 2 && t[j + 1] < ti) j++;
                float u = Mathf.Clamp01((float)((ti - t[j]) / (t[j + 1] - t[j])));
                p[i] = Vector3.Lerp(P[j], P[j + 1], u); q[i] = Quaternion.Slerp(Q[j], Q[j + 1], u);
                v[i] = (P[j + 1] - P[j]) / (float)(t[j + 1] - t[j]);
                w[i] = new float[4];
                for (int k = 0; k < 4; k++) w[i][k] = Mathf.Lerp(W[j][k], W[j + 1][k], u);
            }
            return new TrajectoryTable(dt, p, v, q, w, null, "recorded " + name);
        }

        public static Vector3 EnuToUnity(Vector3 e) => new Vector3(e.x, e.z, e.y);

        /// <summary>Body FLU in ENU world to Unity body axes in Unity world: R_u = Cw R Cb^T.</summary>
        public static Quaternion EnuFluToUnity(Quaternion q) {
            var cw = new Matrix4x4(); cw.SetRow(0, new Vector4(1, 0, 0, 0)); cw.SetRow(1, new Vector4(0, 0, 1, 0)); cw.SetRow(2, new Vector4(0, 1, 0, 0)); cw.SetRow(3, new Vector4(0, 0, 0, 1));
            var cb = new Matrix4x4(); cb.SetRow(0, new Vector4(0, -1, 0, 0)); cb.SetRow(1, new Vector4(0, 0, 1, 0)); cb.SetRow(2, new Vector4(1, 0, 0, 0)); cb.SetRow(3, new Vector4(0, 0, 0, 1));
            var r = Matrix4x4.Rotate(q);
            var ru = cw * r * cb.transpose;
            return ru.rotation;
        }
    }

    /// <summary>Centripetal Catmull-Rom spline (Barry-Goldman) through control points, arclength-parameterised.</summary>
    public sealed class SplinePath {
        public readonly Vector3[] Pts;
        readonly float[] cum;      // arclength at LUT samples
        readonly Vector3[] lut;
        const int PerSeg = 48;
        public readonly float[] PointS;   // arclength at each control point
        public float Length => cum[cum.Length - 1];

        public SplinePath(IList<Vector3> pts) {
            if (pts.Count < 2) throw new ArgumentException("spline needs 2+ points");
            Pts = new Vector3[pts.Count]; pts.CopyTo(Pts, 0);
            int segs = Pts.Length - 1;
            lut = new Vector3[segs * PerSeg + 1]; cum = new float[lut.Length]; PointS = new float[Pts.Length];
            for (int s = 0; s < segs; s++)
                for (int k = 0; k < PerSeg; k++) lut[s * PerSeg + k] = Segment(s, k / (float)PerSeg);
            lut[lut.Length - 1] = Pts[Pts.Length - 1];
            for (int i = 1; i < lut.Length; i++) cum[i] = cum[i - 1] + Vector3.Distance(lut[i], lut[i - 1]);
            for (int s = 0; s <= segs; s++) PointS[s] = cum[s * PerSeg];
        }

        Vector3 P(int i) {
            if (i < 0) return 2 * Pts[0] - Pts[1];
            if (i >= Pts.Length) return 2 * Pts[Pts.Length - 1] - Pts[Pts.Length - 2];
            return Pts[i];
        }

        Vector3 Segment(int s, float u) => Centripetal(P(s - 1), P(s), P(s + 1), P(s + 2), u);

        public static Vector3 Centripetal(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float u) {
            float t0 = 0, t1 = t0 + Knot(p0, p1), t2 = t1 + Knot(p1, p2), t3 = t2 + Knot(p2, p3);
            float t = Mathf.Lerp(t1, t2, u);
            Vector3 a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1;
            Vector3 a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2;
            Vector3 a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3;
            Vector3 b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2;
            Vector3 b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3;
            return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2;
        }
        static float Knot(Vector3 a, Vector3 b) => Mathf.Max(1e-3f, Mathf.Sqrt(Vector3.Distance(a, b)));

        /// <summary>Position at arclength s (clamped).</summary>
        public Vector3 At(float s) {
            s = Mathf.Clamp(s, 0, Length);
            int lo = 0, hi = cum.Length - 1;
            while (hi - lo > 1) { int mid = (lo + hi) / 2; if (cum[mid] <= s) lo = mid; else hi = mid; }
            float d = cum[hi] - cum[lo];
            return d < 1e-6f ? lut[lo] : Vector3.Lerp(lut[lo], lut[hi], (s - cum[lo]) / d);
        }

        /// <summary>Position at normalised parameter 0..1 of arclength.</summary>
        public Vector3 AtNorm(float u) => At(u * Length);
    }

    /// <summary>Monotone cubic (Fritsch-Carlson) interpolant through (t_i, s_i) with zero end slopes: a flight
    /// time schedule that stops at the ends and never runs backwards.</summary>
    public sealed class MonotoneSchedule {
        readonly double[] t, s; readonly double[] m;
        public MonotoneSchedule(double[] times, double[] arclen) {
            t = times; s = arclen; int n = t.Length;
            for (int i = 1; i < n; i++) if (t[i] <= t[i - 1]) throw new ArgumentException("schedule times must increase");
            m = new double[n];
            var d = new double[n - 1];
            for (int i = 0; i < n - 1; i++) d[i] = (s[i + 1] - s[i]) / (t[i + 1] - t[i]);
            for (int i = 1; i < n - 1; i++) m[i] = d[i - 1] * d[i] <= 0 ? 0 : 0.5 * (d[i - 1] + d[i]);
            for (int i = 0; i < n - 1; i++) {
                if (Math.Abs(d[i]) < 1e-12) { m[i] = 0; m[i + 1] = 0; continue; }
                double a = m[i] / d[i], b = m[i + 1] / d[i], h = a * a + b * b;
                if (h > 9) { double k = 3 / Math.Sqrt(h); m[i] = k * a * d[i]; m[i + 1] = k * b * d[i]; }
            }
        }
        public double Eval(double x) {
            int n = t.Length;
            if (x <= t[0]) return s[0];
            if (x >= t[n - 1]) return s[n - 1];
            int i = 0; while (i < n - 2 && t[i + 1] < x) i++;
            double h = t[i + 1] - t[i], u = (x - t[i]) / h, u2 = u * u, u3 = u2 * u;
            return (2 * u3 - 3 * u2 + 1) * s[i] + (u3 - 2 * u2 + u) * h * m[i] + (-2 * u3 + 3 * u2) * s[i + 1] + (u3 - u2) * h * m[i + 1];
        }
    }

    /// <summary>Builds a flight from timed waypoints: spline path, monotone arclength schedule, attitude from the
    /// acceleration (thrust direction) with yaw following the velocity (or a hold-yaw point while hovering).</summary>
    public static class TrajectoryBaker {
        public const float HoverOmega = 560f;   // rad/s, nominal 7-inch tri-blade hover, visual only
        public struct Waypoint { public Vector3 pos; public double t; public string mark; }

        public static TrajectoryTable Bake(IList<Waypoint> wps, Vector3? hoverLookAt, double dt, string describe, double hoverYawBlendS = 1.5) {
            var pts = new List<Vector3>(); foreach (var w in wps) pts.Add(w.pos);
            var path = new SplinePath(pts);
            var times = new double[wps.Count]; var arcs = new double[wps.Count];
            var marks = new Dictionary<string, double>();
            for (int i = 0; i < wps.Count; i++) {
                times[i] = wps[i].t - wps[0].t; arcs[i] = path.PointS[i];
                if (!string.IsNullOrEmpty(wps[i].mark)) marks[wps[i].mark] = times[i];
            }
            // strictly increasing arclength is not required (hovers), only monotone: nudge duplicates for the spline LUT
            var sched = new MonotoneSchedule(times, arcs);
            double dur = times[times.Length - 1] - times[0];
            int n = (int)Math.Round(dur / dt) + 1;
            var P = new Vector3[n]; var V = new Vector3[n]; var A = new Vector3[n];
            for (int i = 0; i < n; i++) P[i] = path.At((float)sched.Eval(times[0] + i * dt));
            for (int i = 0; i < n; i++) {
                int a = Mathf.Max(0, i - 1), b = Mathf.Min(n - 1, i + 1);
                V[i] = (P[b] - P[a]) / (float)((b - a) * dt);
            }
            // low-pass the velocity a little (the spline LUT is piecewise linear), then differentiate
            var Vs = new Vector3[n];
            for (int i = 0; i < n; i++) {
                Vector3 acc = Vector3.zero; float wsum = 0;
                for (int k = -3; k <= 3; k++) { int j = Mathf.Clamp(i + k, 0, n - 1); float wk = 4 - Mathf.Abs(k); acc += V[j] * wk; wsum += wk; }
                Vs[i] = acc / wsum;
            }
            for (int i = 0; i < n; i++) {
                int a = Mathf.Max(0, i - 1), b = Mathf.Min(n - 1, i + 1);
                A[i] = (Vs[b] - Vs[a]) / (float)((b - a) * dt);
            }
            // thrust-lag smoothing of the acceleration (tau 0.2 s)
            float alpha = (float)(dt / (0.2 + dt));
            for (int i = 1; i < n; i++) A[i] = Vector3.Lerp(A[i - 1], A[i], alpha);

            var Q = new Quaternion[n]; var W = new float[n][];
            Vector3 heading = Vector3.forward; bool haveHeading = false;
            float yawSmooth = 0; bool yawInit = false;
            for (int i = 0; i < n; i++) {
                Vector3 hv = new Vector3(Vs[i].x, 0, Vs[i].z);
                float hs = hv.magnitude;
                double tt = times[0] + i * dt;
                float targetYaw;
                if (hs > 0.6f) { heading = hv / hs; haveHeading = true; }
                else if (hoverLookAt.HasValue) {
                    Vector3 d = hoverLookAt.Value - P[i]; d.y = 0;
                    if (d.sqrMagnitude > 1e-4f) { heading = d.normalized; haveHeading = true; }
                }
                targetYaw = haveHeading ? Mathf.Atan2(heading.x, heading.z) * Mathf.Rad2Deg : 0f;
                if (!yawInit) { yawSmooth = targetYaw; yawInit = true; }
                else yawSmooth = Mathf.MoveTowardsAngle(yawSmooth, targetYaw, 120f * (float)dt * Mathf.Min(1f, 0.25f + 0.5f * Mathf.Abs(Mathf.DeltaAngle(yawSmooth, targetYaw)) / 20f));
                Vector3 thrust = A[i] + 9.80665f * Vector3.up;
                Vector3 zb = thrust.normalized;
                float tilt = Vector3.Angle(zb, Vector3.up);
                if (tilt > 35f) zb = Vector3.Slerp(Vector3.up, zb, 35f / tilt);
                Vector3 hd = Quaternion.Euler(0, yawSmooth, 0) * Vector3.forward;
                Vector3 fwd = (hd - Vector3.Dot(hd, zb) * zb).normalized;
                Q[i] = Quaternion.LookRotation(fwd, zb);
                float coll = Mathf.Sqrt(Mathf.Clamp(thrust.magnitude / 9.80665f, 0.3f, 2.2f));
                Vector3 lp = Quaternion.Inverse(Q[i]) * (A[i]);   // body-frame accel for rotor asymmetry
                float pitchK = Mathf.Clamp(lp.z * 0.02f, -0.1f, 0.1f), rollK = Mathf.Clamp(lp.x * 0.02f, -0.1f, 0.1f);
                // M1 FR, M2 RL, M3 FL, M4 RR; forward accel loads the rear motors, rightward accel the left ones
                float[] w = { HoverOmega * coll * (1 - pitchK + rollK * -1f), HoverOmega * coll * (1 + pitchK + rollK), HoverOmega * coll * (1 - pitchK + rollK), HoverOmega * coll * (1 + pitchK - rollK) };
                W[i] = w;
            }
            // shift marks to table-relative time: the table starts at flight time 0 (waypoint 0 must be t=0)
            return new TrajectoryTable(dt, P, Vs, Q, W, marks, describe);
        }
    }
}
