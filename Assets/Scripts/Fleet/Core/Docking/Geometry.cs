using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Engine-independent 2D vector (metres).</summary>
    public readonly struct Vec2
    {
        public readonly double X, Y;
        public Vec2(double x, double y) { X = x; Y = y; }
        public static Vec2 operator +(Vec2 a, Vec2 b) => new Vec2(a.X + b.X, a.Y + b.Y);
        public static Vec2 operator -(Vec2 a, Vec2 b) => new Vec2(a.X - b.X, a.Y - b.Y);
        public static Vec2 operator *(Vec2 a, double s) => new Vec2(a.X * s, a.Y * s);
        public double Length => Math.Sqrt(X * X + Y * Y);
        public static double Cross(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;
        public override string ToString() => $"({X:0.####}, {Y:0.####})";
    }

    /// <summary>Planar pose: position (m) and yaw (rad, CCW, x forward, y left).</summary>
    public readonly struct Pose2
    {
        public readonly double X, Y, Yaw;
        public Pose2(double x, double y, double yaw) { X = x; Y = y; Yaw = yaw; }
        public static Pose2 Identity => new Pose2(0, 0, 0);
        public Vec2 Position => new Vec2(X, Y);

        public Vec2 TransformPoint(Vec2 p)
        {
            double c = Math.Cos(Yaw), s = Math.Sin(Yaw);
            return new Vec2(X + c * p.X - s * p.Y, Y + s * p.X + c * p.Y);
        }

        /// <summary>this * other (apply other in this pose's frame).</summary>
        public Pose2 Compose(Pose2 o)
        {
            Vec2 p = TransformPoint(o.Position);
            return new Pose2(p.X, p.Y, Yaw + o.Yaw);
        }

        public Pose2 Inverse()
        {
            double c = Math.Cos(Yaw), s = Math.Sin(Yaw);
            return new Pose2(-(c * X + s * Y), -(-s * X + c * Y), -Yaw);
        }
    }

    /// <summary>Convex hull helpers used by the static support check.</summary>
    public static class Hull2
    {
        /// <summary>Andrew monotone chain; counter-clockwise, collinear points dropped.</summary>
        public static List<Vec2> Convex(IList<Vec2> pts)
        {
            var p = new List<Vec2>(pts);
            p.Sort((a, b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y));
            if (p.Count < 3) return p;
            var h = new List<Vec2>();
            for (int pass = 0; pass < 2; pass++)
            {
                int start = h.Count;
                for (int i = 0; i < p.Count; i++)
                {
                    while (h.Count >= start + 2 &&
                           Vec2.Cross(h[h.Count - 1] - h[h.Count - 2], p[i] - h[h.Count - 2]) <= 1e-12)
                        h.RemoveAt(h.Count - 1);
                    h.Add(p[i]);
                }
                h.RemoveAt(h.Count - 1);
                p.Reverse();
            }
            return h;
        }

        /// <summary>Signed margin: smallest distance from q to any hull edge line (positive inside). Degenerate hull returns -infinity.</summary>
        public static double InsideMargin(IList<Vec2> hullCcw, Vec2 q)
        {
            if (hullCcw.Count < 3) return double.NegativeInfinity;
            double m = double.PositiveInfinity;
            for (int i = 0; i < hullCcw.Count; i++)
            {
                Vec2 a = hullCcw[i], b = hullCcw[(i + 1) % hullCcw.Count];
                Vec2 e = b - a;
                double len = e.Length;
                if (len < 1e-12) continue;
                m = Math.Min(m, Vec2.Cross(e, q - a) / len);
            }
            return m;
        }
    }

    /// <summary>
    /// Small deterministic RNG (SplitMix64 + Box-Muller) so results are identical across .NET, Mono and IL2CPP;
    /// System.Random(seed) is not specified to be stable across runtimes.
    /// </summary>
    public sealed class DeterministicRng
    {
        private ulong _s;
        public DeterministicRng(ulong seed) { _s = seed; }

        public ulong NextU64()
        {
            unchecked
            {
                _s += 0x9E3779B97F4A7C15UL;
                ulong z = _s;
                z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
                z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
                return z ^ (z >> 31);
            }
        }

        /// <summary>Uniform in [0,1).</summary>
        public double NextDouble() => (NextU64() >> 11) * (1.0 / 9007199254740992.0);

        /// <summary>Standard normal (always consumes exactly two uniforms).</summary>
        public double NextGaussian()
        {
            double u1 = NextDouble(), u2 = NextDouble();
            if (u1 < 1e-300) u1 = 1e-300;
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }
    }
}
