using System;
using System.Globalization;
using System.Text;
using Crane.Drones.Logging;
using Sim.Physics.Aerial;

namespace Crane.Drones.Integration {
    /// <summary>
    /// Logging adapter for the real aerial plant (Crane.Aerial.Physics, CraneAerialPlantCore). It reads the plant
    /// without changing it: ground truth comes from <see cref="CraneAerialPlantCore.State"/>, the IMU rows from the
    /// plant's own <see cref="CraneAerialImu"/> queue (500 Hz by default).
    /// Frames: the plant's world ENU is E = Unity +x, N = Unity +z, U = Unity +y (CraneAerialFrames). That is NOT the
    /// stub's DroneFrames mapping, so obstacles and anything else in the same log must use the plant mapping
    /// (<see cref="EnuFromUnity"/>).
    /// </summary>
    public sealed class AerialPlantLogSource : IDroneStateSource, IDroneImuSource {
        public const string FrameNote = "world ENU = (Unity x, Unity z, Unity y) as CraneAerialFrames; body FLU";
        public readonly CraneAerialPlantCore Plant;
        public int RotorCount => Plant.Spec.Rotors.Length;
        public long ImuSamples { get; private set; }

        public AerialPlantLogSource(CraneAerialPlantCore plant) {
            Plant = plant ?? throw new ArgumentNullException(nameof(plant));
            if (plant.Imu == null) throw new ArgumentException("attach the plant IMU (CraneAerialPlantCore.AttachImu) before logging");
        }

        public static double[] EnuFromUnity(double ux, double uy, double uz) => CraneAerialFrames.UnityToEnu(new[] { ux, uy, uz });

        public void Sample(DroneStateSample s) {
            var st = Plant.State;
            Array.Copy(st.P, s.Pos, 3);
            double sign = st.Q[0] < 0 ? -1.0 : 1.0;   // log convention: canonical w >= 0
            for (int i = 0; i < 4; i++) s.Quat[i] = sign * st.Q[i];
            Array.Copy(st.V, s.Vel, 3);
            Array.Copy(st.Omega, s.AngRateBody, 3); // plant omega is body FLU (the IMU integrates the same field as its gyro)
            for (int i = 0; i < 3; i++) s.AccBody[i] = double.NaN; // recorder differentiates the logged velocity
            var spec = Plant.Spec;
            for (int i = 0; i < spec.Rotors.Length; i++) {
                s.RotorRpm[i] = Math.Abs(st.RotorSpeed[i]) * 60.0 / (2.0 * Math.PI);
                double wmax = spec.Rotors[i].OmegaMax;
                s.MotorCmd[i] = wmax > 0 ? Math.Max(0.0, Math.Min(1.0, Plant.RotorCommand[i] / wmax)) : double.NaN;
            }
            var b = spec.Battery;
            if (b != null) {
                double amps = CraneRotorModelV2.ElectricalCurrent(spec, st);
                s.BattAmp = amps;
                s.BattVolt = b.OpenCircuit(st.Soc) - amps * b.Resistance;
                s.BattSoc = st.Soc;
            } else { s.BattAmp = s.BattVolt = s.BattSoc = double.NaN; }
        }

        /// <summary>The plant IMU sample produced by the sub-step just taken. Fails loudly if the IMU did not emit a
        /// sample at exactly the current plant time (rates misaligned), so imu.csv can never carry a stale sample.</summary>
        public DroneImuSample Sample() {
            var imu = Plant.Imu;
            if (imu.Count != 1 || !imu.TryDequeue(out CraneImuSample q))
                throw new InvalidOperationException($"plant IMU queue holds {imu.Count} samples at t={Plant.State.T}; expected exactly one (rate misaligned)");
            if (q.Time != Plant.State.T)
                throw new InvalidOperationException($"plant IMU sample time {q.Time} != plant time {Plant.State.T}");
            ImuSamples++;
            int flags = 0;
            if (imu.GyroNoiseDensity > 0 || imu.AccelNoiseDensity > 0) flags |= 1;
            if (imu.GyroBiasWalk > 0 || imu.AccelBiasWalk > 0) flags |= 2;
            return new DroneImuSample {
                Gx = q.GyroFlu[0], Gy = q.GyroFlu[1], Gz = q.GyroFlu[2],
                Ax = q.SpecificForceFlu[0], Ay = q.SpecificForceFlu[1], Az = q.SpecificForceFlu[2],
                NoiseFlags = flags
            };
        }

        /// <summary>Manifest fields owned by the plant: substep/IMU rates, noise, physics profile and the params hash.</summary>
        public void FillManifest(DroneRunManifest m, CraneAerialProfile profile, CraneMultirotorController ctl) {
            long subNs = (long)Math.Round(Plant.SubstepDt * 1e9);
            if (m.FixedDtNs % subNs != 0) throw new ArgumentException("fixed tick must be a whole number of plant sub-steps");
            m.Substeps = (int)(m.FixedDtNs / subNs);
            m.StateEverySubsteps = 1;
            m.RotorCount = RotorCount;
            m.ImuRateHz = Plant.Imu.RateHz;
            m.ImuNoise["gyro_noise_density"] = Plant.Imu.GyroNoiseDensity;
            m.ImuNoise["accel_noise_density"] = Plant.Imu.AccelNoiseDensity;
            m.ImuNoise["gyro_bias_walk"] = Plant.Imu.GyroBiasWalk;
            m.ImuNoise["accel_bias_walk"] = Plant.Imu.AccelBiasWalk;
            m.ImuNoise["source"] = "CraneAerialImu (plant, integrated over the sample interval)";
            m.PhysicsProfile = profile != null ? profile.Id : "";
            string canon = PhysicsParamsCanonical(profile, ctl);
            m.PhysicsParamsSha256 = Sha256(canon);
            m.Extra["physics_params_canonical"] = canon;
            m.Extra["frames_note"] = FrameNote;
            m.Controller = ctl != null ? "CraneMultirotorController (cascaded P/PI, 500 Hz) + CraneTrajectoryFollower" : "";
        }

        /// <summary>Everything that determines the plant's response, as one canonical string: profile file hash, the
        /// runtime overrides (ground height, battery, ground effect, rates), IMU settings, controller gains.</summary>
        public string PhysicsParamsCanonical(CraneAerialProfile profile, CraneMultirotorController ctl) {
            var ci = CultureInfo.InvariantCulture;
            var sb = new StringBuilder();
            var spec = Plant.Spec;
            sb.Append("profile_sha256=").Append(profile != null ? profile.Sha256 : "none");
            sb.Append(";battery=").Append(spec.Battery != null ? 1 : 0);
            sb.Append(";ground_effect=").Append(spec.GroundEffect ? 1 : 0);
            sb.Append(";ground_height=").Append(spec.GroundHeight.ToString("R", ci));
            sb.Append(";substep_hz=").Append((1.0 / Plant.SubstepDt).ToString("R", ci));
            sb.Append(";control_every=").Append(Plant.ControlEvery);
            sb.Append(";contact=").Append(Plant.Contact ? 1 : 0);
            sb.Append(";mean_wind=").Append(string.Join(",", Array.ConvertAll(Plant.MeanWindWorld, v => v.ToString("R", ci))));
            sb.Append(";turbulence=").Append(Plant.Turbulence != null ? "dryden" : "none");
            var imu = Plant.Imu;
            sb.Append(";imu_hz=").Append(imu.RateHz.ToString("R", ci)).Append(";imu_noise=")
              .Append(imu.GyroNoiseDensity.ToString("R", ci)).Append(',').Append(imu.AccelNoiseDensity.ToString("R", ci)).Append(',')
              .Append(imu.GyroBiasWalk.ToString("R", ci)).Append(',').Append(imu.AccelBiasWalk.ToString("R", ci));
            if (ctl != null) {
                var g = ctl.G;
                // controller gains: velocity-loop (PositionP, VelocityP/I) and attitude/rate-loop gains kept as named fields
                foreach (var f in typeof(CraneMultirotorController.Gains).GetFields())
                    if (f.FieldType == typeof(double)) sb.Append(";gain.").Append(f.Name).Append('=').Append(((double)f.GetValue(g)).ToString("R", ci));
            }
            return sb.ToString();
        }

        public static string Sha256(string s) => Sha256(Encoding.UTF8.GetBytes(s));
        public static string Sha256(byte[] bytes) {
            using var h = System.Security.Cryptography.SHA256.Create();
            return BitConverter.ToString(h.ComputeHash(bytes)).Replace("-", "").ToLowerInvariant();
        }
    }

    /// <summary>
    /// Steps the plant one 1 ms sub-step at a time (Advance(SubstepDt) is exactly one RK4 sub-step) and calls
    /// <see cref="DroneRunSession.OnSubstep"/> after each, stamped with the index of the sub-step's END:
    /// sub-step k of tick n ends at (n, k+1), wrapping to (n+1, 0). The first logged row is therefore t = 1 sub-step
    /// (there is no IMU sample at t = 0). Pure C#: no Unity time, so the log does not depend on Time.fixedDeltaTime.
    /// </summary>
    public sealed class AerialLoggedFlight {
        public readonly DroneRunSession Session;
        public readonly AerialPlantLogSource Source;
        public long Tick { get; private set; }
        private readonly int substeps;
        private readonly long subNs;

        public AerialLoggedFlight(DroneRunSession session, AerialPlantLogSource source) {
            Session = session; Source = source;
            var m = session.Writer.Manifest;
            substeps = m.Substeps;
            subNs = m.FixedDtNs / m.Substeps;
            if (Math.Abs(source.Plant.SubstepDt * 1e9 - subNs) > 1e-3)
                throw new ArgumentException($"plant sub-step {source.Plant.SubstepDt} s != log sub-step {subNs} ns");
        }

        /// <summary>Advance one Unity-style tick (Substeps plant sub-steps). Returns the tick index now reached.</summary>
        public long StepTick() {
            var plant = Source.Plant;
            for (int sub = 0; sub < substeps; sub++) {
                plant.Advance(plant.SubstepDt);
                int nsub = sub + 1; long ntick = Tick;
                if (nsub == substeps) { nsub = 0; ntick++; }
                long t = Session.TimeNs(ntick, nsub);
                if (Math.Abs(plant.State.T * 1e9 - t) > 1000.0)
                    throw new InvalidOperationException($"plant time {plant.State.T} s drifted from log time {t} ns");
                Session.OnSubstep(ntick, nsub);
            }
            return ++Tick;
        }
    }
}
