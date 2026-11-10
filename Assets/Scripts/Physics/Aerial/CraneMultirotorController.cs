using System;
using static Sim.Physics.Aerial.CraneRotorModelV2;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Cascaded position -> velocity -> attitude -> rate controller for demo flight (world ENU,
    /// body FLU). Kinematic gains (1/s, 1/s^2) are kept separate from the force/torque conversion:
    /// acceleration -> force uses the profile mass; angular acceleration -> torque uses the profile
    /// inertia plus omega x J omega. Torques/thrust go through the priority mixer (yaw gives way
    /// first, then collective; roll/pitch ratio preserved) to per-rotor speed commands.
    /// Uses truth state (no estimator). Replaceable by any IAerialRotorCommandSource, e.g. a PX4
    /// lockstep bridge (Docs/DroneVisuals/PX4_LOCKSTEP_PLAN.md).
    /// </summary>
    public sealed class CraneMultirotorController : IAerialRotorCommandSource {
        public sealed class Gains {
            // kinematic gains
            public double PositionP = 1.2;            // [1/s]   velocity per position error
            public double VelocityP = 3.0;            // [1/s]   acceleration per velocity error
            public double VelocityI = 0.8;            // [1/s^2]
            public double AttitudeP = 9.0;            // [1/s]   body rate per attitude error (roll/pitch)
            public double YawP = 3.0;                 // [1/s]
            public double RateP = 22.0;               // [1/s]   angular acceleration per rate error (roll/pitch)
            public double RateI = 8.0;                // [1/s^2]
            public double YawRateP = 8.0;             // [1/s]
            // limits
            public double MaxSpeedXY = 10.0, MaxSpeedUp = 4.0, MaxSpeedDown = 2.5;   // [m/s]
            public double MaxAccelXY = 8.0;           // [m/s^2]
            public double MaxTilt = 0.7;              // [rad]
            public double MaxRateRP = 7.0, MaxRateYaw = 3.0;   // [rad/s]
            public double IntegratorLimit = 3.0;      // [m/s^2] / [rad/s^2 * s]
        }

        public enum Mode { Idle, Position, Velocity }

        public readonly Gains G = new Gains();
        public Mode CurrentMode { get; private set; } = Mode.Idle;
        public CraneTrajectoryFollower Follower;      // when set and active, overrides the setpoint
        public double[] PositionSetpoint = new double[3], VelocitySetpoint = new double[3], AccelFeedforward = new double[3];
        public double YawSetpoint, YawRateSetpoint;
        public CraneRotorMixerV2.Result LastMix { get; private set; }

        private readonly VehicleSpecV2 spec;
        private readonly CraneRotorMixerV2 mixer;
        private readonly double dt;
        private readonly double tMax, dRoll, dPitch, dYaw;
        private readonly double[] velI = new double[3], rateI = new double[3];
        private double lastTime = double.NaN;

        public CraneMultirotorController(VehicleSpecV2 spec, double controlDt) {
            this.spec = spec; dt = controlDt;
            mixer = new CraneRotorMixerV2(spec.Rotors);
            var r0 = spec.Rotors[0];
            tMax = r0.Kf * r0.OmegaMax * r0.OmegaMax;
            for (int i = 0; i < spec.Rotors.Length; i++) {
                var r = spec.Rotors[i];
                dRoll += mixer.RollCoeff[i] * r.Position[1];
                dPitch += mixer.PitchCoeff[i] * -r.Position[0];
                dYaw += mixer.YawCoeff[i] * -r.Spin * r.Km;
            }
            dRoll *= tMax; dPitch *= tMax; dYaw *= tMax;
        }

        public void HoldPosition(double[] pEnu, double yaw) {
            CurrentMode = Mode.Position; Array.Copy(pEnu, PositionSetpoint, 3);
            Array.Clear(VelocitySetpoint, 0, 3); Array.Clear(AccelFeedforward, 0, 3);
            YawSetpoint = yaw; YawRateSetpoint = 0;
        }

        public void SetVelocity(double[] vEnu, double yawRate) {
            CurrentMode = Mode.Velocity; Array.Copy(vEnu, VelocitySetpoint, 3);
            Array.Clear(AccelFeedforward, 0, 3); YawRateSetpoint = yawRate;
        }

        public void ResetIntegrators() { Array.Clear(velI, 0, 3); Array.Clear(rateI, 0, 3); lastTime = double.NaN; }

        public double[] RotorCommands(StateV2 s, double time) {
            int n = spec.Rotors.Length;
            if (Follower != null && Follower.Active) {
                Follower.Sample(time, out double[] p, out double[] v, out double[] a, out double yaw);
                CurrentMode = Mode.Position;
                Array.Copy(p, PositionSetpoint, 3); Array.Copy(v, VelocitySetpoint, 3); Array.Copy(a, AccelFeedforward, 3);
                YawSetpoint = yaw;
            }
            if (CurrentMode == Mode.Idle) return new double[n];
            double yawNow = CraneAerialFrames.YawEnu(s.Q);
            if (CurrentMode == Mode.Velocity) {
                if (double.IsNaN(lastTime)) YawSetpoint = yawNow;
                YawSetpoint = WrapPi(YawSetpoint + YawRateSetpoint * dt);
            }
            lastTime = time;

            // position -> velocity
            double[] vSp = (double[])VelocitySetpoint.Clone();
            if (CurrentMode == Mode.Position)
                for (int i = 0; i < 3; i++) vSp[i] += G.PositionP * (PositionSetpoint[i] - s.P[i]);
            double hs = Math.Sqrt(vSp[0] * vSp[0] + vSp[1] * vSp[1]);
            if (hs > G.MaxSpeedXY) { vSp[0] *= G.MaxSpeedXY / hs; vSp[1] *= G.MaxSpeedXY / hs; }
            vSp[2] = Math.Max(-G.MaxSpeedDown, Math.Min(G.MaxSpeedUp, vSp[2]));

            // velocity -> acceleration (kinematic), anti-windup clamp
            double[] aSp = new double[3];
            for (int i = 0; i < 3; i++) {
                double e = vSp[i] - s.V[i];
                velI[i] = Math.Max(-G.IntegratorLimit, Math.Min(G.IntegratorLimit, velI[i] + G.VelocityI * e * dt));
                aSp[i] = AccelFeedforward[i] + G.VelocityP * e + velI[i];
            }
            double ah = Math.Sqrt(aSp[0] * aSp[0] + aSp[1] * aSp[1]);
            if (ah > G.MaxAccelXY) { aSp[0] *= G.MaxAccelXY / ah; aSp[1] *= G.MaxAccelXY / ah; }
            // tilt limit: horizontal accel bounded by tan(maxTilt) * (g + a_z)
            double az = Math.Max(aSp[2] + spec.Gravity, 0.2 * spec.Gravity);
            double ahMax = Math.Tan(G.MaxTilt) * az;
            ah = Math.Sqrt(aSp[0] * aSp[0] + aSp[1] * aSp[1]);
            if (ah > ahMax) { aSp[0] *= ahMax / ah; aSp[1] *= ahMax / ah; }

            // acceleration -> thrust vector (force: mass) and desired attitude
            double[] fDes = { spec.Mass * aSp[0], spec.Mass * aSp[1], spec.Mass * az };
            double[,] r = QuatToMatrix(s.Q);
            double[] zB = { r[0, 2], r[1, 2], r[2, 2] };
            double thrust = Math.Max(0.0, Dot(fDes, zB));
            double[] zD = Scale(fDes, 1.0 / Norm(fDes));
            double[] xC = { Math.Cos(YawSetpoint), Math.Sin(YawSetpoint), 0 };
            double[] yD = Cross(zD, xC); yD = Scale(yD, 1.0 / Norm(yD));
            double[] xD = Cross(yD, zD);
            var rd = new[,] { { xD[0], yD[0], zD[0] }, { xD[1], yD[1], zD[1] }, { xD[2], yD[2], zD[2] } };

            // attitude error e_R = 0.5 vee(Rd^T R - R^T Rd) (Lee et al. 2010) -> body-rate setpoint
            var m = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    double a = 0, b = 0;
                    for (int k = 0; k < 3; k++) { a += rd[k, i] * r[k, j]; b += r[k, i] * rd[k, j]; }
                    m[i, j] = a - b;
                }
            double[] eR = { 0.5 * m[2, 1], 0.5 * m[0, 2], 0.5 * m[1, 0] };
            double[] wSp = { -G.AttitudeP * eR[0], -G.AttitudeP * eR[1], -G.YawP * eR[2] };
            wSp[0] = Clamp(wSp[0], G.MaxRateRP); wSp[1] = Clamp(wSp[1], G.MaxRateRP);
            wSp[2] = Clamp(wSp[2] + (CurrentMode == Mode.Velocity ? YawRateSetpoint : 0.0), G.MaxRateYaw);

            // rate -> angular acceleration (kinematic) -> torque (inertia)
            double[] alpha = new double[3];
            for (int i = 0; i < 3; i++) {
                double e = wSp[i] - s.Omega[i];
                double kp = i == 2 ? G.YawRateP : G.RateP;
                rateI[i] = Math.Max(-G.IntegratorLimit, Math.Min(G.IntegratorLimit, rateI[i] + G.RateI * e * dt));
                alpha[i] = kp * e + rateI[i];
            }
            double[] tau = Add(MatVec(spec.Inertia, alpha), Cross(s.Omega, MatVec(spec.Inertia, s.Omega)));

            // thrust/torque -> normalized mixer inputs -> rotor speed commands
            double collective = thrust / (n * tMax);
            var mix = mixer.Mix(collective, tau[0] / dRoll, tau[1] / dPitch, tau[2] / dYaw);
            LastMix = mix;
            var cmd = new double[n];
            for (int i = 0; i < n; i++) cmd[i] = spec.Rotors[i].OmegaMax * Math.Sqrt(Math.Max(0.0, mix.Output[i]));
            return cmd;
        }

        private static double Clamp(double x, double lim) => Math.Max(-lim, Math.Min(lim, x));
        public static double WrapPi(double a) {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            return a;
        }
    }
}
