using System;

namespace Crane.Fleet.Perception
{
    public enum ObservationSourceKind { OwnCamera, PeerObserver, FixedStation, Overhead }

    /// <summary>
    /// A relative pose measurement as a rover would receive it. Pose is T_parent_child (child expressed in parent); Covariance is the
    /// 6x6 covariance of the error twist [dp; dtheta] expressed in PARENT axes (pose.Perturbed(xi) convention). The only time information
    /// a consumer may rely on is StampAcquired (when the world was sampled) and StampDelivered (when it reached the consumer).
    /// </summary>
    public sealed class RelPoseObservation
    {
        public double StampAcquired, StampDelivered;
        public string SourceId; public ObservationSourceKind SourceKind;
        public string FrameParent, FrameChild;
        public Pose Pose; public Matrix Covariance;
        /// <summary>Age at the time of the last ageing operation (now - StampAcquired); 0 for a freshly created observation.</summary>
        public double Age;
        public int TagId = -1;
        public bool AmbiguousPose;

        public RelPoseObservation Clone() => (RelPoseObservation)MemberwiseClone();
    }

    /// <summary>Growth rate of the consumer's own dead-reckoning uncertainty (variance per second of age).</summary>
    public sealed class DeadReckoningDrift
    {
        public double PositionVarRate;   // m^2 / s
        public double YawVarRate;        // rad^2 / s
        public DeadReckoningDrift(double posVarRate, double yawVarRate) { PositionVarRate = posVarRate; YawVarRate = yawVarRate; }
        /// <summary>Random-walk drift from wheel slip: speed-proportional slip sigma fraction per sqrt(s) and a gyro bias/noise rate.</summary>
        public static DeadReckoningDrift FromSlip(double slipSigmaFrac, double speed, double gyroNoiseDensity, double gyroBiasInstability, double age = 1.0)
        {
            double pos = Math.Pow(slipSigmaFrac * speed, 2);
            double yaw = gyroNoiseDensity * gyroNoiseDensity + gyroBiasInstability * gyroBiasInstability * age;
            return new DeadReckoningDrift(pos, yaw);
        }
    }

    public static class ObservationAging
    {
        /// <summary>
        /// Age an observation to time <paramref name="now"/>. Covariance grows by the consumer's dead-reckoning drift accumulated since acquisition
        /// (position and yaw random walks; yaw drift also moves a lever-arm-length of position error), and, if the consumer moved since the
        /// acquisition, the observation is re-expressed in the consumer's current base frame using that motion
        /// (<paramref name="consumerMotionSinceAcquisition"/> = T_base_at_acquisition_base_now).
        /// </summary>
        public static RelPoseObservation Age(RelPoseObservation obs, double now, DeadReckoningDrift drift, Pose? consumerMotionSinceAcquisition = null)
        {
            if (now < obs.StampAcquired) throw new ArgumentException("now precedes acquisition");
            double dt = now - obs.StampAcquired;
            var o = obs.Clone(); o.Age = dt;
            var cov = obs.Covariance.Clone();
            Pose pose = obs.Pose;
            Vec3 lever = obs.Pose.P;
            if (consumerMotionSinceAcquisition.HasValue)
            {
                Pose d = consumerMotionSinceAcquisition.Value, inv = d.Inverse();
                // parent axes rotate: new_parent_axes = inv.R * old_parent_axes; position uncertainty of the new parent is added below
                var Rm = Matrix.Block6(inv.R, new Mat3(), new Mat3(), inv.R);
                cov = Rm * cov * Rm.T;
                pose = inv.Compose(pose);
                lever = pose.P;
            }
            double qp = drift.PositionVarRate * dt, qy = drift.YawVarRate * dt;
            Mat3 sk = Mat3.Skew(lever);
            // child position error from a parent yaw error w: dp = -[lever]x w (+ translation drift directly)
            Mat3 I = Mat3.Identity;
            var A = Matrix.Block6(I, sk * -1.0, new Mat3(), I);
            var Qd = new Matrix(6, 6);
            for (int i = 0; i < 2; i++) Qd[i, i] = qp;     // planar ground vehicle: x,y translation and yaw only
            Qd[5, 5] = qy;
            cov = cov + A * Qd * A.T;
            o.Covariance = cov; o.Pose = pose;
            return o;
        }
    }

    /// <summary>First-order covariance algebra on rigid transforms with the [dp; dtheta] parent-axes convention.</summary>
    public static class PoseUncertainty
    {
        public static Matrix BlockRot(Mat3 R) => Matrix.Block6(R, new Mat3(), new Mat3(), R);

        /// <summary>Covariance of T_a ∘ T_b where T_a has error cov Sa (in a's parent axes) and T_b has error cov Sb (in a's frame axes, i.e. b's parent axes).</summary>
        public static Matrix Compose(Pose a, Matrix Sa, Pose b, Matrix Sb)
        {
            Vec3 r = a.R * b.P;
            var A1 = Matrix.Block6(Mat3.Identity, Mat3.Skew(r) * -1.0, new Mat3(), Mat3.Identity);
            var Rb = BlockRot(a.R);
            var s2 = Rb * Sb * Rb.T;
            return A1 * Sa * A1.T + s2;
        }

        /// <summary>Covariance of a rigid shift T_t ∘ T_m: the body frame b = t ∘ m where m (e.g. -mount) is exactly known. Output in same (parent) axes.</summary>
        public static Matrix RigidShift(Pose t, Pose m, Matrix St)
        {
            Vec3 r = t.R * m.P;
            var A = Matrix.Block6(Mat3.Identity, Mat3.Skew(r) * -1.0, new Mat3(), Mat3.Identity);
            return A * St * A.T;
        }

        /// <summary>Camera-axes covariance -> axes of the frame the camera pose is expressed in.</summary>
        public static Matrix RotateInto(Mat3 R, Matrix S) { var B = BlockRot(R); return B * S * B.T; }
    }
}
