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
        }

        private Faults faults;
        private ArticulationBody body;
        private UnityEngine.Rendering.HighDefinition.WaterSurface water;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            int i = Array.IndexOf(args, "--crane-calibration-faults");
            if (i < 0) return;
            if (i + 1 >= args.Length) throw new ArgumentException("fault JSON path required");
            Faults data = JsonUtility.FromJson<Faults>(File.ReadAllText(args[i + 1]));
            Validate(data.strengths, 4, 0.01f, 4, "strengths");
            Validate(data.angles, 4, -45, 45, "angles");
            Validate(data.drag, 3, 1, 20, "drag");
            Validate(data.dragOffset, 2, -1, 1, "dragOffset");
            SceneManager.sceneLoaded += (_, _) => {
                var boat = FindAnyObjectByType<OmniXController>();
                if (boat == null) return;
                var instance = boat.gameObject.AddComponent<RoboBoatCalibrationFaults>();
                instance.faults = data;
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
                thrusters[i].SetThrustCurve(RoboBoatPhysics.T200Forward * faults.strengths[i],
                    RoboBoatPhysics.T200Reverse * faults.strengths[i], RoboBoatPhysics.T200SpinUp,
                    faults.angles[i]);
            }
            water = FindAnyObjectByType<UnityEngine.Rendering.HighDefinition.WaterSurface>();
            Debug.Log($"CRANE_CALIBRATION_FAULTS {JsonUtility.ToJson(faults)}");
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
