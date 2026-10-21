using System;
using System.IO;
using UnityEngine;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Opt-in rotor plant. Active only with --crane-aerial-profile=&lt;name|path&gt;. It disables
    /// the legacy MultirotorDynamics on the same object (which stays numerically untouched) and
    /// applies CraneRotorModelV2 forces and torques to the Rigidbody; PhysX integrates the rigid
    /// body, this class integrates rotor speeds and battery state.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public sealed class CraneAerialRotorV2Plant : MonoBehaviour {
        private static readonly double[,] MfluFromRuf = { { 0, 0, 1 }, { -1, 0, 0 }, { 0, 1, 0 } };
        private static readonly double[,] MenuFromRuf = { { 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } };

        public CraneAerialProfile Profile { get; private set; }
        public CraneRotorMixerV2 Mixer { get; private set; }
        private VehicleSpecV2 spec;
        private StateV2 state;
        private double[] targetSpeed;
        private Rigidbody body;

        public int RotorCount => spec?.Rotors.Length ?? 0;
        public double[] RotorSpeeds => state?.RotorSpeed;
        public double BatterySoc => state?.Soc ?? 1.0;

        /// <summary>Adds and configures the plant when the profile flag is present; else returns null.</summary>
        public static CraneAerialRotorV2Plant TryInstall(GameObject target, string[] args) {
            string name = CraneAerialOptions.ReadAerialProfile(args);
            if (string.IsNullOrEmpty(name)) return null;
            var plant = target.GetComponent<CraneAerialRotorV2Plant>();
            if (plant == null) plant = target.AddComponent<CraneAerialRotorV2Plant>();
            plant.Initialize(name);
            return plant;
        }

        private void Initialize(string nameOrPath) {
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles"));
            Profile = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve(nameOrPath, dir),
                withBattery: true, groundEffect: true);
            spec = Profile.Vehicle;
            Mixer = new CraneRotorMixerV2(spec.Rotors);
            body = GetComponent<Rigidbody>();
            var legacy = GetComponent<MultirotorDynamics>();
            if (legacy != null) legacy.enabled = false;
            body.mass = (float)spec.Mass;
            body.centerOfMass = Vector3.zero;
            // FLU (Ixx, Iyy, Izz) -> Unity body (x right = Iyy, y up = Izz, z forward = Ixx).
            body.inertiaTensorRotation = Quaternion.identity;
            body.inertiaTensor = new Vector3((float)spec.Inertia[1, 1], (float)spec.Inertia[2, 2],
                (float)spec.Inertia[0, 0]);
            state = new StateV2(spec.Rotors.Length);
            targetSpeed = new double[spec.Rotors.Length];
            Debug.Log($"CRANE_AERIAL_PROFILE id={Profile.Id} sha256={Profile.Sha256}");
        }

        /// <summary>Normalized thrust-fraction commands in profile rotor order.</summary>
        public void SetNormalizedCommands(double[] thrustFraction) {
            for (int i = 0; i < targetSpeed.Length; i++) {
                double u = Math.Max(0.0, Math.Min(1.0, thrustFraction[i]));
                targetSpeed[i] = spec.Rotors[i].OmegaMax * Math.Sqrt(u);
            }
        }

        private void FixedUpdate() {
            if (spec == null) return;
            double dt = Time.fixedDeltaTime;
            Quaternion q = body.rotation;
            double[,] rU = CraneRotorModelV2.QuatToMatrix(new double[] { q.w, q.x, q.y, q.z });
            double[,] rWb = Mul(Mul(MenuFromRuf, rU), Transpose(MfluFromRuf));
            Vector3 w = body.angularVelocity, vel = body.linearVelocity, pos = body.position;
            double[] wBodyRuf = CraneRotorModelV2.MatTVec(rU, new double[] { w.x, w.y, w.z });
            double[] omegaFlu = Neg(MulV(MfluFromRuf, wBodyRuf));
            double[] pEnu = { pos.x, pos.z, pos.y }, vEnu = { vel.x, vel.z, vel.y };
            state.P = pEnu; state.V = vEnu; state.Omega = omegaFlu;
            CraneRotorModelV2.UpdateMotors(spec, state, targetSpeed, dt);
            CraneRotorModelV2.Wrench(spec, rWb, pEnu, vEnu, omegaFlu, state.RotorSpeed,
                new double[3], out double[] fFlu, out double[] tFlu);
            double[] fRuf = MulV(Transpose(MfluFromRuf), fFlu);
            double[] tRuf = Neg(MulV(Transpose(MfluFromRuf), tFlu));
            body.AddRelativeForce((float)fRuf[0], (float)fRuf[1], (float)fRuf[2], ForceMode.Force);
            body.AddRelativeTorque((float)tRuf[0], (float)tRuf[1], (float)tRuf[2], ForceMode.Force);
            if (spec.Battery != null)
                state.Soc -= CraneRotorModelV2.ElectricalCurrent(spec, state) * dt /
                    (3600.0 * spec.Battery.CapacityAh);
        }

        private static double[,] Transpose(double[,] m) => new[,] {
            { m[0, 0], m[1, 0], m[2, 0] }, { m[0, 1], m[1, 1], m[2, 1] }, { m[0, 2], m[1, 2], m[2, 2] } };
        private static double[,] Mul(double[,] a, double[,] b) {
            var r = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    r[i, j] = a[i, 0] * b[0, j] + a[i, 1] * b[1, j] + a[i, 2] * b[2, j];
            return r;
        }
        private static double[] MulV(double[,] m, double[] v) => CraneRotorModelV2.MatVec(m, v);
        private static double[] Neg(double[] v) => new[] { -v[0], -v[1], -v[2] };
    }
}
