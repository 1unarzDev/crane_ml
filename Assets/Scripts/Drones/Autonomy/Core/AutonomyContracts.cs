// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;

namespace Crane.Drones.Autonomy {
    /// <summary>Double-precision 3-vector, world ENU (x east, y north, z up) unless a field says otherwise.</summary>
    public struct Vec3 {
        public double X, Y, Z;
        public Vec3(double x, double y, double z) { X = x; Y = y; Z = z; }
        public static Vec3 operator +(Vec3 a, Vec3 b) => new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator -(Vec3 a) => new Vec3(-a.X, -a.Y, -a.Z);
        public static Vec3 operator *(Vec3 a, double s) => new Vec3(a.X * s, a.Y * s, a.Z * s);
        public static Vec3 operator *(double s, Vec3 a) => new Vec3(a.X * s, a.Y * s, a.Z * s);
        public double Dot(Vec3 b) => X * b.X + Y * b.Y + Z * b.Z;
        public Vec3 Cross(Vec3 b) => new Vec3(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
        public double Norm => Math.Sqrt(X * X + Y * Y + Z * Z);
        public double NormXY => Math.Sqrt(X * X + Y * Y);
        public Vec3 Normalized { get { double n = Norm; return n > 1e-12 ? this * (1.0 / n) : new Vec3(); } }
        public static Vec3 FromArray(double[] a) => new Vec3(a[0], a[1], a[2]);
        public double[] ToArray() => new[] { X, Y, Z };
        public static Vec3 Lerp(Vec3 a, Vec3 b, double u) => a + (b - a) * u;
        public override string ToString() => $"({X:0.###},{Y:0.###},{Z:0.###})";
    }

    /// <summary>Pose: position (ENU) and yaw/pitch/roll as a quaternion (w,x,y,z), body FLU.</summary>
    public struct Pose {
        public Vec3 P; public double Qw, Qx, Qy, Qz;
        public static Pose FromYaw(Vec3 p, double yaw) => new Pose { P = p, Qw = Math.Cos(yaw / 2), Qz = Math.Sin(yaw / 2) };
        public static Pose FromQuat(Vec3 p, double[] q) => new Pose { P = p, Qw = q[0], Qx = q[1], Qy = q[2], Qz = q[3] };
        public double Yaw => Math.Atan2(2 * (Qw * Qz + Qx * Qy), 1 - 2 * (Qy * Qy + Qz * Qz));
        /// <summary>Rotate a body-frame vector into the world frame.</summary>
        public Vec3 Rotate(Vec3 v) {
            double w = Qw, x = Qx, y = Qy, z = Qz;
            double tx = 2 * (y * v.Z - z * v.Y), ty = 2 * (z * v.X - x * v.Z), tz = 2 * (x * v.Y - y * v.X);
            return new Vec3(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
        }
        public Vec3 InverseRotate(Vec3 v) { var c = this; c.Qx = -Qx; c.Qy = -Qy; c.Qz = -Qz; return c.Rotate(v); }
        public Vec3 ToWorld(Vec3 body) => P + Rotate(body);
        /// <summary>Linear position, normalized-lerp rotation (adequate for 5 ms intervals).</summary>
        public static Pose Interpolate(Pose a, Pose b, double u) {
            double d = a.Qw * b.Qw + a.Qx * b.Qx + a.Qy * b.Qy + a.Qz * b.Qz, s = d < 0 ? -1 : 1;
            var r = new Pose { P = Vec3.Lerp(a.P, b.P, u),
                Qw = a.Qw + u * (s * b.Qw - a.Qw), Qx = a.Qx + u * (s * b.Qx - a.Qx), Qy = a.Qy + u * (s * b.Qy - a.Qy), Qz = a.Qz + u * (s * b.Qz - a.Qz) };
            double n = Math.Sqrt(r.Qw * r.Qw + r.Qx * r.Qx + r.Qy * r.Qy + r.Qz * r.Qz);
            r.Qw /= n; r.Qx /= n; r.Qy /= n; r.Qz /= n; return r;
        }
    }

    // ------------------------------------------------------------------ world (simulation side only)

    public struct RayHit {
        public double Distance;
        /// <summary>Stable object id (manifest capsule owner) or -1 for terrain/unknown.</summary>
        public int ObjectId;
        /// <summary>Characteristic diameter of the struck structure [m] (drives thin-structure dropout); +inf for terrain.</summary>
        public double Diameter;
    }

    /// <summary>Sensor-side ray queries against the simulated world. Implementations: ForestManifestWorld (analytic
    /// capsules + terrain), Unity collider world (Crane.Drones.Autonomy.Unity). The stack never sees this.</summary>
    public interface IRayWorld {
        /// <param name="beamHalfAngle">ray footprint half-angle [rad]: a structure is hit if the ray passes within
        /// r + beamHalfAngle * t of its axis (pixel footprint for decimated depth images). 0 = ideal ray.</param>
        /// <param name="ignoreObject">object id to see through (stereo thin-structure dropout); -2 = none</param>
        bool Raycast(Vec3 origin, Vec3 dirUnit, double maxRange, double beamHalfAngle, int ignoreObject, out RayHit hit);
    }

    /// <summary>Ground-truth geometry for metrics only (clearance, collision). Never visible to the stack.</summary>
    public interface IGroundTruthWorld {
        /// <summary>Nearest surface distance from p, minus nothing (body radius is applied by the caller).</summary>
        /// <param name="hardOnly">ignore soft twigs (radius below the configured hard-structure radius)</param>
        double Clearance(Vec3 p, double searchRadius, bool hardOnly, out int objectId, out double objectRadius);
        double GroundHeight(double x, double y);
        string ObjectName(int id);
    }

    // ------------------------------------------------------------------ messages (timestamps: sim clock, int64 ns)

    public sealed class DepthFrame {
        public string Sensor = "oakd_depth";
        public long AcqNs, AvailNs;
        public int Width, Height;
        public double FxPx, FyPx, CxPx, CyPx;
        /// <summary>Row-major, metres along the optical axis (z); NaN = invalid (no information); +inf = no return within
        /// range (free evidence up to FreeEvidenceM).</summary>
        public float[] Depth;
        public double RangeMax, FreeEvidenceM;
        /// <summary>Sensor pose at acquisition in the world (truth; for logging/metrics only, not for the stack).</summary>
        public Pose TruthPose;
        /// <summary>Camera optical frame: +x right, +y down, +z forward; Mount maps optical -> body FLU.</summary>
        public Pose MountInBody;
        public int TruthHits, DroppedObjects, InvalidPixels;
    }

    public sealed class LidarScan {
        public string Sensor = "l2";
        public long StartNs, EndNs, AvailNs;
        /// <summary>Points in the sensor frame at their own sample time (motion skew NOT removed).</summary>
        public float[] X, Y, Z;
        public int[] OffsetNs;
        /// <summary>Directions of rays with no return (sensor frame, unit) and their offsets: free evidence to RangeMax.</summary>
        public float[] MissX, MissY, MissZ; public int[] MissOffsetNs;
        public Pose MountInBody;
        public double RangeMax;
        public int Count => X.Length;
    }

    public sealed class StateEstimate {
        public long TNs;
        /// <summary>Local odometry frame (drifts). Position, velocity, attitude.</summary>
        public Vec3 P, V; public Pose Pose;
        public double SigmaP, SigmaYaw;
        public bool Valid = true;
        /// <summary>Age of the newest estimator input when this estimate was produced [ns].</summary>
        public long InputAgeNs;
    }

    /// <summary>Believed transform from the local odometry frame to the global (prior map / mission) frame: g = l + Offset.</summary>
    public struct MapAlignment {
        public Vec3 Offset; public double Sigma; public long TNs;
    }

    /// <summary>Kinematic setpoint (PX4 TrajectorySetpoint semantics, ENU). NaN fields are unset.
    /// Units: m, m/s, m/s^2, rad, rad/s. No force or torque appears here.</summary>
    public sealed class TrajectorySetpoint {
        public long TNs, ValidUntilNs;
        public Vec3 Position = new Vec3(double.NaN, double.NaN, double.NaN);
        public Vec3 Velocity, Acceleration;
        public double Yaw = double.NaN, YawRate;
        public string Source = "planner";
        /// <summary>Hold the current position (PX4 Hold/Loiter semantics) instead of tracking Velocity.</summary>
        public bool Hold;
        public long SensorAcqNs = -1;   // newest sensor acquisition that influenced it (for perception-to-command age)
    }

    public sealed class FcStatus {
        public bool Armed = true, OffboardActive = true, Failsafe;
        public long LastSetpointNs = -1;
    }

    public sealed class PlanDecision {
        public long TNs, ApplyNs;
        public string Planner = "";
        public bool FeasibleAny;
        public int FeasibleCount, Candidates;
        public double Heading, Climb, VCmd, DFree, StopDist, TauAge, Progress;
        public Vec3 VelocityCmd;
        public string Note = "";
        /// <summary>Heading change of the chosen primitive vs the current velocity [rad], committed arc [m] and the arc
        /// [m] before which a stop along Path is validated (d_free - margin).</summary>
        public double DHeading, CommitArc, SafeArc;
        /// <summary>The validated path (local frame) and its cumulative arc length; null when no primitive was feasible.</summary>
        public Vec3[] Path; public double[] PathArc;
    }

    // ------------------------------------------------------------------ module interfaces

    public interface IDepthSensor {
        /// <summary>Called every scheduler tick; returns a frame when one is acquired at tNs (available later at AvailNs).</summary>
        DepthFrame Tick(long tNs, Pose truthBodyPose, Vec3 truthVel, Vec3 truthOmegaBody);
    }

    public interface ILidarSensor {
        /// <summary>Called every tick with the truth pose; casts the points whose sample time falls in this tick from
        /// that pose (per-point motion skew). Returns a completed sweep when one ends.</summary>
        LidarScan Tick(long tNs, long tickNs, Pose truthBodyPose);
    }

    public interface IStateEstimator {
        void Reset(Pose truth0, Vec3 v0, long tNs, ulong seed);
        /// <summary>Advance with the truth state (simulated estimator) or with real sensor input (future estimators).</summary>
        void Update(long tNs, Pose truthBodyPose, Vec3 truthVel);
        StateEstimate Current { get; }
        /// <summary>Estimated body pose at an earlier time (for frame registration and deskew). False if outside history.</summary>
        bool PoseAt(long tNs, out Pose pose);
        MapAlignment Alignment { get; }
        /// <summary>Truth minus estimate, for metrics only.</summary>
        Vec3 LocalError { get; }
    }

    public interface IOccupancyMap {
        void InsertDepth(DepthFrame f, Pose sensorPoseEst, long nowNs);
        void InsertLidar(LidarScan s, Func<long, Pose?> bodyPoseEstAt, long nowNs);
        /// <summary>Log-odds of the voxel containing p (0 = unknown).</summary>
        float LogOdds(Vec3 p);
        bool Known(Vec3 p);
        bool Occupied(Vec3 p);
        double Resolution { get; }
        long OccupiedVoxels { get; }
        long KnownVoxels { get; }
    }

    public interface IPriorMap {
        /// <summary>True if the prior claims an obstacle within the flight band at (x,y) in the GLOBAL frame.</summary>
        bool BlockedColumn(double x, double y);
        /// <summary>True if the prior claims a hard obstacle surface near p (3D, global frame).</summary>
        bool Occupied(Vec3 p);
        double Resolution { get; }
        string Describe { get; }
    }

    public interface IGlobalRouter {
        /// <summary>Waypoints (local frame) from the current position to the goal.</summary>
        IReadOnlyList<Vec3> Route { get; }
        /// <summary>Cost-to-go [m] in the local frame; +inf blocked.</summary>
        double CostToGo(double x, double y);
        void Update(long tNs, StateEstimate est, MapAlignment align, IOccupancyMap map);
    }

    public interface ILocalPlanner {
        string Name { get; }
        PlanDecision Plan(long tNs, StateEstimate est, Vec3 goalLocal, double vCap, IOccupancyMap map, IGlobalRouter router, StopRule rule, double targetAltitudeLocal);
    }

    public interface IFlightControllerLink {
        void Submit(TrajectorySetpoint sp);
        FcStatus Status { get; }
    }

    public sealed class CommsMessage {
        public string From, To, Kind; public long SentNs, DeliverNs; public byte[] Payload; public int Seq;
    }

    public interface ICommsLink {
        void Send(CommsMessage m);
        /// <summary>Messages whose DeliverNs &lt;= tNs for this endpoint.</summary>
        IEnumerable<CommsMessage> Receive(string endpoint, long tNs);
    }

    /// <summary>Single tap for the experiment harness: ground truth, sensor outputs, decisions, arbiter actions and comms.
    /// Fields are plain dictionaries so any sink (crane-drone-log/1 events.jsonl, JSONL, in-memory) can store them.</summary>
    public interface IAutonomyRecorder {
        void Event(long tNs, string kind, Dictionary<string, object> fields);
    }

    public sealed class NullRecorder : IAutonomyRecorder {
        public static readonly NullRecorder Instance = new NullRecorder();
        public void Event(long tNs, string kind, Dictionary<string, object> fields) { }
    }

    public sealed class MemoryRecorder : IAutonomyRecorder {
        public readonly List<(long t, string kind, Dictionary<string, object> f)> Events = new List<(long, string, Dictionary<string, object>)>();
        public void Event(long tNs, string kind, Dictionary<string, object> fields) => Events.Add((tNs, kind, fields));
    }

    /// <summary>Single-drone no-op link; also a loss/delay bus for multi-drone tests.</summary>
    public sealed class LocalCommsBus : ICommsLink {
        private readonly List<CommsMessage> q = new List<CommsMessage>();
        public IAutonomyRecorder Recorder = NullRecorder.Instance;
        public void Send(CommsMessage m) {
            q.Add(m);
            Recorder.Event(m.SentNs, "comms_send", new Dictionary<string, object> { ["from"] = m.From, ["to"] = m.To, ["kind"] = m.Kind, ["seq"] = m.Seq, ["deliver_ns"] = m.DeliverNs, ["bytes"] = m.Payload?.Length ?? 0 });
        }
        public IEnumerable<CommsMessage> Receive(string endpoint, long tNs) {
            var o = new List<CommsMessage>();
            for (int i = q.Count - 1; i >= 0; i--)
                if (q[i].DeliverNs <= tNs && (q[i].To == endpoint || q[i].To == "*")) { o.Add(q[i]); q.RemoveAt(i); }
            o.Reverse();
            foreach (var m in o) Recorder.Event(tNs, "comms_recv", new Dictionary<string, object> { ["from"] = m.From, ["to"] = endpoint, ["kind"] = m.Kind, ["seq"] = m.Seq, ["sent_ns"] = m.SentNs });
            return o;
        }
    }
}
