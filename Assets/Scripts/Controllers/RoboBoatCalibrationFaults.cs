using System;
using System.IO;
using Sim.Actuators.Motors;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>Opt-in physical faults for held-out characterization tests. No controller
    /// or sensor is told the injected values. Default worker physics is unchanged.</summary>
    internal sealed class RoboBoatCalibrationFaults : MonoBehaviour {
        [Serializable] private sealed class Faults {
            public float[] strengths = { 1, 1, 1, 1 }; // FL, FR, RR, RL, force multipliers
            public float[] angles = { 0, 0, 0, 0 };    // mount yaw error, degrees
            public float[] drag = { 1, 1, 1 };          // surge, sway, yaw multipliers
            public float[] dragOffset = { 0, 0 };      // bow, port metres, extra drag only
            // Plant variants (absent fields keep the contract physics):
            public float mass = RoboBoatPhysics.Mass;                 // kg, total
            public float izz = RoboBoatPhysics.BoatInertia.z;         // kg m^2 about the CoM
            public float[] reverse = {                                 // reverse / forward thrust
                RoboBoatPhysics.T200Reverse / RoboBoatPhysics.T200Forward,
                RoboBoatPhysics.T200Reverse / RoboBoatPhysics.T200Forward,
                RoboBoatPhysics.T200Reverse / RoboBoatPhysics.T200Forward,
                RoboBoatPhysics.T200Reverse / RoboBoatPhysics.T200Forward };
            public float[] com = { 0, 0 };                             // bow, port metres CoM shift
            public float spinup = RoboBoatPhysics.T200SpinUp;          // s, thruster first-order lag
            public int deadtime_ticks;                                 // physics ticks of PWM delay
        }

        private string sha256;

        private Faults faults;
        private ArticulationBody body;
        private UnityEngine.Rendering.HighDefinition.WaterSurface water;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, "--crane-calibration-faults");
            if (i < 0) return;
            if (i + 1 >= args.Length) throw new ArgumentException("fault JSON path required");
            string text = File.ReadAllText(args[i + 1]);
            Faults data = JsonUtility.FromJson<Faults>(text);
            Validate(data.strengths, 4, 0, 4, "strengths");   // 0: a dead thruster
            Validate(data.angles, 4, -45, 45, "angles");
            Validate(data.drag, 3, 1, 20, "drag");
            Validate(data.dragOffset, 2, -1, 1, "dragOffset");
            Validate(new[] { data.mass }, 1, 10, 80, "mass");
            Validate(new[] { data.izz }, 1, 0.5f, 20, "izz");
            Validate(data.reverse, 4, 0.2f, 1.5f, "reverse");
            Validate(data.com, 2, -0.3f, 0.3f, "com");
            Validate(new[] { data.spinup }, 1, 0.001f, 2, "spinup");
            Validate(new[] { (float)data.deadtime_ticks }, 1, 0, 100, "deadtime_ticks");
            string digest;
            using (var hash = System.Security.Cryptography.SHA256.Create())
                digest = BitConverter.ToString(hash.ComputeHash(System.Text.Encoding.UTF8.GetBytes(text)))
                    .Replace("-", "").ToLowerInvariant();
            ROSThrusterPwmCommand.DelayTicks = data.deadtime_ticks;
            SceneManager.sceneLoaded += (_, _) => {
                var boat = FindAnyObjectByType<OmniXController>();
                if (boat == null) return;
                var instance = boat.gameObject.AddComponent<RoboBoatCalibrationFaults>();
                instance.faults = data;
                instance.sha256 = digest;
                instance.body = RoboBoatPhysics.Root(boat);
            };
        }

        private static void Validate(float[] values, int length, float min, float max, string name) {
            if (values == null || values.Length != length)
                throw new ArgumentException($"{name} requires {length} values");
            foreach (float value in values)
                if (float.IsNaN(value) || float.IsInfinity(value) || value < min || value > max)
                    throw new ArgumentException($"{name} outside [{min}, {max}]");
        }

        private void Start() {
            // Start runs after all sceneLoaded callbacks, including contract physics setup.
            var boat = GetComponent<OmniXController>();
            Thruster[] thrusters = { boat.frontLeft, boat.frontRight, boat.rearRight, boat.rearLeft };
            for (int i = 0; i < 4; i++) {
                float forward = RoboBoatPhysics.T200Forward * faults.strengths[i];
                thrusters[i].SetThrustCurve(forward, forward * faults.reverse[i], faults.spinup,
                    faults.angles[i]);
            }
            Vector3 inertia = RoboBoatPhysics.BoatInertia;
            RoboBoatPhysics.SetMassLayout(boat, faults.mass, new Vector3(inertia.x, inertia.y, faults.izz),
                faults.com[0], faults.com[1]);
            water = FindAnyObjectByType<UnityEngine.Rendering.HighDefinition.WaterSurface>();
            Debug.Log($"CRANE_CALIBRATION_FAULTS {JsonUtility.ToJson(faults)}");
            Debug.Log($"CRANE_PLANT_APPLIED sha256={sha256} {JsonUtility.ToJson(faults)}");
        }

        private void FixedUpdate() {
            Quaternion frame = RoboBoatRosFrame.Rotation(body.transform.rotation);
            Vector3 bow = frame * Vector3.forward, up = frame * Vector3.up;
            Vector3 port = Vector3.Cross(bow, Vector3.up);
            Vector3 current = Vector3.zero;
            if (water != null) {
                float angle = water.largeOrientationValue * Mathf.Deg2Rad;
                current = water.largeCurrentSpeedValue * RoboBoatPhysics.Disturbance(Time.timeSinceLevelLoad) *
                          new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            }
            Vector3 offset = bow * faults.dragOffset[0] + port * faults.dragOffset[1];
            Vector3 relative = body.linearVelocity + Vector3.Cross(body.angularVelocity, offset) - current;
            Vector3 force = bow * (faults.drag[0] - 1) *
                            RoboBoatHull.Drag(Vector3.Dot(relative, bow), RoboBoatHull.Surge) +
                            port * (faults.drag[1] - 1) *
                            RoboBoatHull.Drag(Vector3.Dot(relative, port), RoboBoatHull.Sway);
            body.AddForceAtPosition(force, body.worldCenterOfMass + offset);
            body.AddTorque(up * (faults.drag[2] - 1) *
                RoboBoatHull.Drag(Vector3.Dot(body.angularVelocity, up), RoboBoatHull.Yaw));
        }
    }
}
