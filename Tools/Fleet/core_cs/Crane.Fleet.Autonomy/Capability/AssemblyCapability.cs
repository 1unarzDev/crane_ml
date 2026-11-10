using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Oriented rectangle in a parent frame (assembly frame for footprints, world for obstacles).</summary>
    public struct Box2
    {
        public double Cx, Cy, Yaw, Hx, Hy;
        public string Id;
        public Box2(string id, double cx, double cy, double yaw, double hx, double hy) { Id = id; Cx = cx; Cy = cy; Yaw = yaw; Hx = hx; Hy = hy; }
        public double Radius => Math.Sqrt(Hx * Hx + Hy * Hy);
        public Box2 Transformed(Pose2 parent)
        {
            double c = Math.Cos(parent.Yaw), s = Math.Sin(parent.Yaw);
            return new Box2(Id, parent.X + c * Cx - s * Cy, parent.Y + s * Cx + c * Cy, parent.Yaw + Yaw, Hx, Hy);
        }
        public Box2 Inflated(double m) => new Box2(Id, Cx, Cy, Yaw, Hx + m, Hy + m);

        /// <summary>Separating-axis overlap test of two oriented rectangles.</summary>
        public static bool Overlap(in Box2 a, in Box2 b)
        {
            double dx = b.Cx - a.Cx, dy = b.Cy - a.Cy;
            double rr = a.Radius + b.Radius;
            if (dx * dx + dy * dy > rr * rr) return false;
            double ca = Math.Cos(a.Yaw), sa = Math.Sin(a.Yaw), cb = Math.Cos(b.Yaw), sb = Math.Sin(b.Yaw);
            // axes: a.x, a.y, b.x, b.y
            for (int k = 0; k < 4; k++)
            {
                double ux, uy;
                switch (k) { case 0: ux = ca; uy = sa; break; case 1: ux = -sa; uy = ca; break; case 2: ux = cb; uy = sb; break; default: ux = -sb; uy = cb; break; }
                double pa = a.Hx * Math.Abs(ca * ux + sa * uy) + a.Hy * Math.Abs(-sa * ux + ca * uy);
                double pb = b.Hx * Math.Abs(cb * ux + sb * uy) + b.Hy * Math.Abs(-sb * ux + cb * uy);
                if (Math.Abs(dx * ux + dy * uy) > pa + pb) return false;
            }
            return true;
        }
    }

    /// <summary>One rover module placed in an assembly, with the health of its four wheels (fl, fr, rl, rr).</summary>
    public sealed class ModuleInstance
    {
        public RoverSpec Spec;
        public Pose2 InAssembly;
        public readonly CapHealth[] Health = { CapHealth.Ok, CapHealth.Ok, CapHealth.Ok, CapHealth.Ok };
        public ModuleInstance Clone()
        {
            var m = new ModuleInstance { Spec = Spec, InAssembly = InAssembly };
            Array.Copy(Health, m.Health, 4);
            return m;
        }
    }

    /// <summary>
    /// The autonomy's MODULE GRAPH view of one rigid assembly (a single rover is an assembly of one): modules with poses in the
    /// assembly frame (the first module's base frame), wheel health, and extra footprint (a carried payload). Capability is
    /// computed from it (capability_descriptor_v0: planners consume C, not the graph).
    /// </summary>
    public sealed class AssemblyModel
    {
        public string Id;
        public readonly List<ModuleInstance> Modules = new List<ModuleInstance>();
        public readonly List<Box2> ExtraFootprint = new List<Box2>();
        /// <summary>Incremented on every change (health, attach, detach, payload): consumers replan when it changes.</summary>
        public int Version;

        public static AssemblyModel Single(RoverSpec s)
        {
            var a = new AssemblyModel { Id = s.Id };
            a.Modules.Add(new ModuleInstance { Spec = s, InAssembly = Pose2.Identity });
            return a;
        }

        public int IndexOf(string roverId)
        {
            for (int i = 0; i < Modules.Count; i++) if (Modules[i].Spec.Id == roverId) return i;
            return -1;
        }

        public bool Contains(string roverId) => IndexOf(roverId) >= 0;

        public int WheelCount => 4 * Modules.Count;

        /// <summary>Pose of member <paramref name="roverId"/>'s base in the assembly frame.</summary>
        public Pose2 ModulePose(string roverId) => Modules[IndexOf(roverId)].InAssembly;

        public List<Box2> Footprint()
        {
            var l = new List<Box2>();
            foreach (var m in Modules)
                l.Add(new Box2(m.Spec.Id, m.InAssembly.X, m.InAssembly.Y, m.InAssembly.Yaw, m.Spec.BodyLength / 2, Math.Max(m.Spec.OverallWidth, m.Spec.BodyWidth) / 2));
            l.AddRange(ExtraFootprint);
            return l;
        }

        public AssemblyModel Clone()
        {
            var a = new AssemblyModel { Id = Id, Version = Version };
            foreach (var m in Modules) a.Modules.Add(m.Clone());
            a.ExtraFootprint.AddRange(ExtraFootprint);
            return a;
        }

        /// <summary>Pose of the newcomer's base when its face <paramref name="faceNew"/> mates the existing member's face
        /// (same rule as Crane.Fleet.Graph.ModuleGraph.MateNew): T_new = T_existing * F_existing * Rot(pi) * F_new^-1.</summary>
        public static Pose2 MatedPose(Pose2 existing, FaceSpec faceExisting, FaceSpec faceNew)
            => Se2.Compose(Se2.Compose(Se2.Compose(existing, faceExisting.InBody), new Pose2(0, 0, Math.PI)), Se2.Inverse(faceNew.InBody));

        /// <summary>Merge: assembly b joins a, b's member <paramref name="roverB"/> face mating a's member <paramref name="roverA"/> face.
        /// The result keeps a's frame.</summary>
        public static AssemblyModel Merge(AssemblyModel a, string roverA, string faceA, AssemblyModel b, string roverB, string faceB, string newId)
        {
            var r = a.Clone(); r.Id = newId; r.Version = Math.Max(a.Version, b.Version) + 1;
            var ma = a.Modules[a.IndexOf(roverA)];
            var mb = b.Modules[b.IndexOf(roverB)];
            Pose2 bInA = MatedPose(ma.InAssembly, ma.Spec.Face(faceA), mb.Spec.Face(faceB));   // pose of roverB base in a's frame
            Pose2 bFrameInA = Se2.Compose(bInA, Se2.Inverse(mb.InAssembly));                     // pose of b's assembly frame in a's frame
            foreach (var m in b.Modules)
            {
                var c = m.Clone(); c.InAssembly = Se2.Compose(bFrameInA, m.InAssembly); r.Modules.Add(c);
            }
            foreach (var e in b.ExtraFootprint) r.ExtraFootprint.Add(e.Transformed(bFrameInA));
            return r;
        }

        /// <summary>Remove the listed members (detach); returns the remainder, re-based on its first module.</summary>
        public AssemblyModel Without(ICollection<string> roverIds, string newId, out Pose2 newFrameInOld)
        {
            var r = new AssemblyModel { Id = newId, Version = Version + 1 };
            foreach (var m in Modules) if (!roverIds.Contains(m.Spec.Id)) r.Modules.Add(m.Clone());
            newFrameInOld = r.Modules.Count > 0 ? r.Modules[0].InAssembly : Pose2.Identity;
            var inv = Se2.Inverse(newFrameInOld);
            foreach (var m in r.Modules) m.InAssembly = Se2.Compose(inv, m.InAssembly);
            return r;
        }
    }

    /// <summary>
    /// The motion and geometry fields of the capability descriptor that the planner and controllers consume. Implementations are
    /// built from an <see cref="AssemblyModel"/> by an <see cref="ICapabilityBuilder"/>; the Crane.Fleet.Capability library
    /// (MotionAnalyzer) is the default source, <see cref="AnalyticCapabilityBuilder"/> an engine-free fallback with the same contract.
    /// Twists are [vx, vy, wz] in the assembly frame at the assembly origin. Wheel index = 4 * module index + (fl, fr, rl, rr).
    /// </summary>
    public interface ICapability
    {
        string AssemblyId { get; }
        int Version { get; }
        int WheelCount { get; }
        /// <summary>Health mode of wheel i (Ok, DeadFree, DeadLocked, Weak, Intermittent).</summary>
        CapHealthMode WheelMode(int i);
        /// <summary>Effective speed limit (rad/s) of wheel i after health; 0 for dead wheels.</summary>
        double WheelLimit(int i);
        /// <summary>Kinematic wheel speed of wheel i for a twist (rad/s), including dead wheels (what a free or forced wheel would turn at).</summary>
        double WheelRate(int i, in Twist2 xi);
        /// <summary>Dimension of the commandable twist subspace (3 = holonomic).</summary>
        int ActuatedRank { get; }
        /// <summary>Orthogonal projection onto the commandable subspace (respects locked-wheel constraints).</summary>
        Twist2 Project(in Twist2 xi);
        /// <summary>Orthonormal basis vector k (k &lt; ActuatedRank) of the commandable subspace.</summary>
        Twist2 Basis(int k);
        /// <summary>Largest s with s*u feasible (wheel limits, body limits); 0 if u leaves the commandable subspace.</summary>
        double MaxAlong(in Twist2 u);
        bool Feasible(in Twist2 xi, double tol = 1e-6);
        double MaxSpeedXy { get; }
        double MaxYawRate { get; }
        double MaxLinAccel { get; }
        double MaxYawAccel { get; }
        /// <summary>Footprint rectangles in the assembly frame (modules plus carried payload).</summary>
        IReadOnlyList<Box2> Footprint { get; }
        /// <summary>Provenance of the motion field: "capability_library" or "analytic_fallback".</summary>
        string Provenance { get; }
    }

    public interface ICapabilityBuilder
    {
        ICapability Build(AssemblyModel model, AutonomyAssumptions assume);
    }

    /// <summary>Concrete capability; both builders fill it.</summary>
    public sealed class AssemblyCapability : ICapability
    {
        public string AssemblyId { get; internal set; }
        public int Version { get; internal set; }
        public int WheelCount { get; internal set; }
        public int ActuatedRank { get; internal set; }
        public double MaxSpeedXy { get; internal set; }
        public double MaxYawRate { get; internal set; }
        public double MaxLinAccel { get; internal set; }
        public double MaxYawAccel { get; internal set; }
        public IReadOnlyList<Box2> Footprint { get; internal set; }
        public string Provenance { get; internal set; }
        internal double[] Rows;      // WheelCount x 3, kinematic row of every wheel (assembly twist -> wheel rad/s)
        internal double[] Limits;    // WheelCount
        internal CapHealthMode[] Modes;
        internal double[] P = new double[9];   // projector onto commandable subspace
        internal double[] BasisRows = new double[9];

        public CapHealthMode WheelMode(int i) => Modes[i];
        public double WheelLimit(int i) => Limits[i];
        public double WheelRate(int i, in Twist2 xi) => Rows[3 * i] * xi.Vx + Rows[3 * i + 1] * xi.Vy + Rows[3 * i + 2] * xi.Wz;
        public void Row(int i, out double a, out double b, out double c) { a = Rows[3 * i]; b = Rows[3 * i + 1]; c = Rows[3 * i + 2]; }

        public Twist2 Project(in Twist2 x) => new Twist2(
            P[0] * x.Vx + P[1] * x.Vy + P[2] * x.Wz, P[3] * x.Vx + P[4] * x.Vy + P[5] * x.Wz, P[6] * x.Vx + P[7] * x.Vy + P[8] * x.Wz);

        public Twist2 Basis(int k) => new Twist2(BasisRows[3 * k], BasisRows[3 * k + 1], BasisRows[3 * k + 2]);

        public bool Driven(int i) => Modes[i] != CapHealthMode.DeadFree && Modes[i] != CapHealthMode.DeadLocked && Limits[i] > 0;

        public double MaxAlong(in Twist2 u)
        {
            double n = u.Norm(1.0);
            if (n == 0) return double.PositiveInfinity;
            var pu = Project(u);
            if ((u - pu).Norm(1.0) > 1e-7 * n) return 0;
            double s = double.PositiveInfinity;
            for (int i = 0; i < WheelCount; i++)
            {
                if (!Driven(i)) continue;
                double w = Math.Abs(WheelRate(i, u));
                if (w > 1e-12) s = Math.Min(s, Limits[i] / w);
            }
            double lin = u.LinearSpeed;
            if (lin > 1e-12) s = Math.Min(s, MaxSpeedXy / lin);
            if (Math.Abs(u.Wz) > 1e-12) s = Math.Min(s, MaxYawRate / Math.Abs(u.Wz));
            return s;
        }

        public bool Feasible(in Twist2 xi, double tol = 1e-6)
        {
            if ((xi - Project(xi)).Norm(1.0) > tol) return false;
            for (int i = 0; i < WheelCount; i++)
            {
                if (Modes[i] == CapHealthMode.DeadLocked) { if (Math.Abs(WheelRate(i, xi)) > tol) return false; continue; }
                if (!Driven(i)) continue;
                if (Math.Abs(WheelRate(i, xi)) > Limits[i] + tol) return false;
            }
            return xi.LinearSpeed <= MaxSpeedXy + tol && Math.Abs(xi.Wz) <= MaxYawRate + tol;
        }

        /// <summary>Project onto the commandable subspace, then scale uniformly into the twist set (direction preserved).</summary>
        public Twist2 Saturate(in Twist2 xi, out double scale)
        {
            var p = Project(xi);
            double m = MaxAlong(p);
            scale = (m >= 1 || double.IsPositiveInfinity(m)) ? 1 : m;
            return p * scale;
        }

        /// <summary>Fill rows/projector/basis common helpers.</summary>
        internal static AssemblyCapability Skeleton(AssemblyModel model, AutonomyAssumptions assume, string provenance)
        {
            int n = model.WheelCount;
            var c = new AssemblyCapability
            {
                AssemblyId = model.Id, Version = model.Version, WheelCount = n, Provenance = provenance,
                Rows = new double[3 * n], Limits = new double[n], Modes = new CapHealthMode[n], Footprint = model.Footprint(),
            };
            double vmax = double.PositiveInfinity, wmax = double.PositiveInfinity;
            for (int m = 0; m < model.Modules.Count; m++)
            {
                var mod = model.Modules[m]; var s = mod.Spec;
                vmax = Math.Min(vmax, s.MaxSpeedXy); wmax = Math.Min(wmax, s.MaxYawRate);
                double cth = Math.Cos(mod.InAssembly.Yaw), sth = Math.Sin(mod.InAssembly.Yaw), px = mod.InAssembly.X, py = mod.InAssembly.Y;
                for (int j = 0; j < 4; j++)
                {
                    int i = 4 * m + j;
                    s.WheelRow(j, out double ax, out double ay, out double aw);
                    c.Rows[3 * i] = ax * cth - ay * sth;
                    c.Rows[3 * i + 1] = ax * sth + ay * cth;
                    c.Rows[3 * i + 2] = ax * (-cth * py + sth * px) + ay * (sth * py + cth * px) + aw;
                    var h = mod.Health[j];
                    c.Modes[i] = h.Mode;
                    double gain = h.Mode == CapHealthMode.Weak || h.Mode == CapHealthMode.Intermittent ? Math.Max(0, Math.Min(1, h.Gain)) : 1;
                    c.Limits[i] = (h.Mode == CapHealthMode.DeadFree || h.Mode == CapHealthMode.DeadLocked) ? 0 : s.MaxWheelSpeed * gain;
                }
            }
            c.MaxSpeedXy = vmax; c.MaxYawRate = wmax;
            // a coupled group accelerates no faster than one module (traction scales with mass); conservative for assemblies
            c.MaxLinAccel = assume.MaxLinAccel; c.MaxYawAccel = assume.MaxYawAccel / Math.Max(1, model.Modules.Count);
            return c;
        }
    }
}
