using System;
using System.Collections.Generic;

// EVALUATOR-ONLY namespace. Ground truth lives here. Rover-side code (everything outside this namespace) must never reference these types.
namespace Crane.Fleet.Perception.Evaluator
{
    /// <summary>True state of one rover at one instant.</summary>
    public sealed class RoverTruthState
    {
        public Pose Pose;                       // T_world_base
        public Vec3 LinVelWorld, AngVelWorld;   // for motion blur
        public double Vx, Vy, Wz;               // body-frame twist (for wheel/gyro truth)
    }

    public delegate RoverTruthState Trajectory(double t);

    public static class Trajectories
    {
        public static Trajectory Static(Pose p) => t => new RoverTruthState { Pose = p };
        /// <summary>Constant body-frame twist (vx forward, vy left, wz yaw rate) from a start pose at t = 0 (exact arc integration).</summary>
        public static Trajectory ConstantTwist(Pose start, double vx, double vy, double wz)
        {
            return t =>
            {
                double yaw0 = start.Yaw, yaw = yaw0 + wz * t, x, y;
                if (Math.Abs(wz) < 1e-9) { x = start.P.X + (vx * Math.Cos(yaw0) - vy * Math.Sin(yaw0)) * t; y = start.P.Y + (vx * Math.Sin(yaw0) + vy * Math.Cos(yaw0)) * t; }
                else
                {
                    // integral of R(yaw0 + wz s) [vx, vy] ds
                    double s1 = (Math.Sin(yaw) - Math.Sin(yaw0)) / wz, c1 = -(Math.Cos(yaw) - Math.Cos(yaw0)) / wz;
                    x = start.P.X + vx * s1 - vy * c1; y = start.P.Y + vx * c1 + vy * s1;
                }
                var pose = new Pose(Mat3.Rz(yaw), new Vec3(x, y, start.P.Z));
                var lin = new Vec3(vx * Math.Cos(yaw) - vy * Math.Sin(yaw), vx * Math.Sin(yaw) + vy * Math.Cos(yaw), 0);
                return new RoverTruthState { Pose = pose, LinVelWorld = lin, AngVelWorld = new Vec3(0, 0, wz), Vx = vx, Vy = vy, Wz = wz };
            };
        }
    }

    public enum OverheadMode { Working, Occluded, Disabled }

    /// <summary>All ground truth the evaluator holds. Not reachable from <c>Crane.Fleet.Perception.Rover</c>.</summary>
    public sealed class EvaluatorTruth
    {
        private readonly Dictionary<string, Trajectory> _rovers = new Dictionary<string, Trajectory>();
        private readonly Dictionary<string, OverheadMode> _overhead = new Dictionary<string, OverheadMode>();
        public IReadOnlyCollection<string> RoverIds => _rovers.Keys;
        public void SetTrajectory(string roverId, Trajectory tr) { _rovers[roverId] = tr; }
        public RoverTruthState At(string roverId, double t) => _rovers[roverId](t);
        public bool Has(string roverId) => _rovers.ContainsKey(roverId);
        public void SetOverheadMode(string cameraId, OverheadMode m) { _overhead[cameraId] = m; }
        public OverheadMode OverheadModeOf(string cameraId) => _overhead.TryGetValue(cameraId, out var m) ? m : OverheadMode.Working;
    }

    /// <summary>Scores observations against truth (evaluator-side).</summary>
    public static class ObservationScorer
    {
        public static string WorldFrame = "world";
        public static string BaseFrame(string roverId) => roverId + "/base_link";

        /// <summary>True pose of the observation's child in its parent frame at the acquisition stamp.</summary>
        public static Pose TruePose(EvaluatorTruth truth, RelPoseObservation o)
        {
            Pose child = truth.At(RoverOf(o.FrameChild), o.StampAcquired).Pose;
            if (o.FrameParent == WorldFrame) return child;
            Pose parent = truth.At(RoverOf(o.FrameParent), o.StampAcquired).Pose;
            return parent.Inverse().Compose(child);
        }
        public static string RoverOf(string frame) { int i = frame.IndexOf('/'); return i < 0 ? frame : frame.Substring(0, i); }

        /// <summary>Error twist [dp; dtheta] of the observation pose relative to truth, in parent axes.</summary>
        public static double[] Error(EvaluatorTruth truth, RelPoseObservation o) => o.Pose.ErrorFrom(TruePose(truth, o));

        /// <summary>Normalised estimation error squared (6 dof expected mean 6 if calibrated).</summary>
        public static double Nees(EvaluatorTruth truth, RelPoseObservation o) => o.Covariance.Mahalanobis(Error(truth, o));
    }
}
