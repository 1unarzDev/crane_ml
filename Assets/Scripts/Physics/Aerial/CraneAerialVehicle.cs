using System;
using System.IO;
using UnityEngine;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Scene component for a multirotor. The physics is owned by CraneAerialPlantCore (double
    /// precision, &gt;= 1 kHz RK4 sub-steps inside each FixedUpdate); this component only drives the
    /// transform / kinematic Rigidbody to the integrated pose (MovePosition/MoveRotation), so PhysX
    /// never integrates the airframe and the result is independent of Time.fixedDeltaTime.
    /// Collisions: the plant has a flat ground plane at GroundHeightUnityY; PhysX contacts are not
    /// fed back (known gap, see Docs/DroneVisuals/PHYSICS.md).
    /// Director API: SetTrajectory, SetVelocitySetpoint, HoldPosition, ResetEpisode.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class CraneAerialVehicle : MonoBehaviour {
        [Tooltip("Profile name in Tools/Aerial/profiles or an absolute path")]
        public string profile = "aos7_v5_f90_6s";
        public double substepHz = 1000.0;
        public double controlHz = 500.0;
        public double imuHz = 500.0;
        public bool batteryEnabled = true;
        public bool groundEffect = true;
        public float groundHeightUnityY = 0f;
        public Vector3 meanWindUnity = Vector3.zero;
        [Tooltip("Dryden W20 [m/s]; 0 disables turbulence")]
        public float turbulenceW20 = 0f;
        public long seed = 1;
        public bool hoverOnStart = true;

        public CraneAerialProfile Profile { get; private set; }
        public CraneAerialPlantCore Plant { get; private set; }
        public CraneMultirotorController Controller { get; private set; }
        public CraneTrajectoryFollower Follower { get; private set; }
        public CraneAerialImu Imu => Plant?.Imu;
        public double[] RotorSpeeds => Plant?.State.RotorSpeed;
        private Rigidbody body;

        private void Awake() {
            string dir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Tools", "Aerial", "profiles"));
            Profile = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve(profile, dir), batteryEnabled, groundEffect);
            var spec = Profile.Vehicle;
            spec.GroundHeight = groundHeightUnityY;
            Plant = new CraneAerialPlantCore(spec, substepHz, controlHz);
            Plant.AttachImu(new CraneAerialImu(imuHz, spec.Gravity, (ulong)seed));
            Controller = new CraneMultirotorController(spec, Plant.ControlDt);
            Follower = new CraneTrajectoryFollower();
            Controller.Follower = Follower;
            Plant.CommandSource = Controller;
            body = GetComponent<Rigidbody>();
            if (body != null) { body.isKinematic = true; body.interpolation = RigidbodyInterpolation.Interpolate; }
            Debug.Log($"CRANE_AERIAL_PROFILE id={Profile.Id} sha256={Profile.Sha256} substepHz={substepHz}");
            ResetEpisode(seed);
        }

        /// <summary>Restart from the current transform pose with rotors at hover speed.</summary>
        public void ResetEpisode(long episodeSeed) {
            seed = episodeSeed;
            var spec = Plant.Spec;
            var s = CraneAerialPlantCore.HoverState(spec, ToEnu(transform.position));
            s.Q = CraneAerialFrames.QuatFromUnityAxes(V(transform.forward), V(transform.up));
            Plant.MeanWindWorld = ToEnu(meanWindUnity);
            Plant.Turbulence = turbulenceW20 > 0f ? new CraneDrydenTurbulence(turbulenceW20, (ulong)seed) : null;
            Plant.Reset(s, (ulong)seed);
            Controller.ResetIntegrators();
            Follower.Stop();
            if (hoverOnStart) Controller.HoldPosition(s.P, CraneAerialFrames.YawEnu(s.Q));
        }

        public void HoldPosition(Vector3 positionUnity, float yawDegUnity) {
            Follower.Stop();
            Controller.HoldPosition(ToEnu(positionUnity), UnityYawToEnu(yawDegUnity));
        }

        /// <summary>Velocity in Unity world [m/s]; yaw rate [deg/s], positive clockwise from above (Unity +y).</summary>
        public void SetVelocitySetpoint(Vector3 velocityUnity, float yawRateDegUnity = 0f) {
            Follower.Stop();
            Controller.SetVelocity(ToEnu(velocityUnity), -yawRateDegUnity * Mathf.Deg2Rad);
        }

        /// <summary>Fly straight segments from the current position through the waypoints (Unity world).</summary>
        public void SetTrajectory(Vector3[] waypointsUnity, float[] speeds) {
            var pts = new double[waypointsUnity.Length + 1][];
            pts[0] = (double[])Plant.State.P.Clone();
            for (int i = 0; i < waypointsUnity.Length; i++) pts[i + 1] = ToEnu(waypointsUnity[i]);
            var sp = new double[speeds.Length == 1 ? 1 : pts.Length - 1];
            for (int i = 0; i < sp.Length; i++) sp[i] = speeds[Math.Min(i, speeds.Length - 1)];
            Follower.SetTrajectory(pts, sp, CraneAerialFrames.YawEnu(Plant.State.Q));
        }

        private void FixedUpdate() {
            if (Plant == null) return;
            Plant.Advance(Time.fixedDeltaTime);
            var st = Plant.State;
            CraneAerialFrames.UnityLookAxes(st.Q, out double[] f, out double[] u);
            Vector3 pos = ToUnity(st.P);
            Quaternion rot = Quaternion.LookRotation(new Vector3((float)f[0], (float)f[1], (float)f[2]),
                new Vector3((float)u[0], (float)u[1], (float)u[2]));
            if (body != null) { body.MovePosition(pos); body.MoveRotation(rot); }
            else transform.SetPositionAndRotation(pos, rot);
        }

        private static double[] V(Vector3 v) => new double[] { v.x, v.y, v.z };
        public static double[] ToEnu(Vector3 u) => CraneAerialFrames.UnityToEnu(V(u));
        public static Vector3 ToUnity(double[] enu) {
            double[] u = CraneAerialFrames.EnuToUnity(enu);
            return new Vector3((float)u[0], (float)u[1], (float)u[2]);
        }
        /// <summary>Unity yaw (deg, clockwise from +z/north seen from above) to ENU yaw (rad, CCW from east).</summary>
        public static double UnityYawToEnu(float yawDeg) => Math.PI / 2 - yawDeg * Mathf.Deg2Rad;
    }
}
