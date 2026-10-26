using System;
using Sim.Physics.Water.Boat;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// RoboBoat scene extras that are not plant physics: under the sensor contract the LiDAR
    /// becomes a VLP-16, and ``--crane-wind`` attaches <see cref="RoboBoatWind"/>. The plant itself
    /// is applied by Sim.Physics.Water.Boat.RoboBoatPhysics.
    /// </summary>
    internal static class RoboBoatSceneExtras {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            bool contract = Array.IndexOf(args, CraneRoboBoatSensorContractBootstrap.Flag) >= 0;
            bool wind = Array.IndexOf(args, RoboBoatWind.Flag) >= 0;
            bool legacy = Array.IndexOf(args, RoboBoatPhysics.LegacyFlag) >= 0;
            if ((!contract || legacy) && !wind) return;
            SceneManager.sceneLoaded += (_, _) => {
                try {
                    OmniXController boat = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                        FindObjectsInactive.Exclude);
                    if (boat == null) return;
                    if (contract && !legacy) ConfigureLidar(RoboBoatPhysics.Root(boat));
                    if (wind) RoboBoatWind.Attach(boat, args);
                }
                catch (Exception error) {
                    Debug.LogError($"CRANE_ROBOBOAT_PHYSICS_FAILED {error}");
                }
            };
        }

        /// <summary>The LiDAR as a Velodyne VLP-16: 16 rings 2 degrees apart from +15 to -15
        /// degrees and 0.2 degree steps over 360 degrees, at the prefab's rate (10 Hz). Its mount
        /// follows the imported hull's axes, so the cloud gets the same turn into REP-103 as the
        /// body (x forward in lidar_link, as the real driver gives).</summary>
        private static void ConfigureLidar(ArticulationBody root) {
            foreach (var lidar in root.GetComponentsInChildren<Sim.Sensors.Lidar.Lidar3D>()) {
                lidar.Configure(1800, 16, 30f * Mathf.Deg2Rad, 100f, spanVerticalFov: true,
                    horizontalFieldOfView: 2f * Mathf.PI);
                float[] rings = lidar.RingElevationsDegrees();
                Debug.Log($"CRANE_ROBOBOAT_LIDAR rings={rings.Length} " +
                          $"elevation_deg={rings[0]:F2}..{rings[^1]:F2} azimuth_steps=1800");
                lidar.SetFrameCorrection(Quaternion.Inverse(
                    Quaternion.Inverse(lidar.transform.rotation) *
                    RoboBoatRosFrame.Rotation(root.transform.rotation)));
            }
        }
    }

    /// <summary>
    /// Wind as a world-fixed air velocity with gusts: aerodynamic drag on the hull's
    /// above-water area, F = 1/2 rho Cd A(beta) |u| u with u the wind relative to the boat,
    /// applied above and slightly ahead of the centre of mass so it also turns the boat.
    /// ``--crane-wind speed,toward_deg`` (toward, counter-clockwise from world +x, like the
    /// current); gusts are seeded by ``--crane-seed``.
    /// </summary>
    internal sealed class RoboBoatWind : MonoBehaviour {
        internal const string Flag = "--crane-wind";
        internal const float AirDensity = 1.225f;      // kg/m^3
        internal const float DragCoefficient = 1.0f;   // bluff hulls, deck box and mast
        internal const float FrontArea = 0.30f;        // m^2 above water, seen from ahead
        internal const float SideArea = 0.40f;         // m^2 above water, seen abeam
        internal const float GustIntensity = 0.25f;    // standard deviation / mean speed
        internal const float GustTime = 8f;            // s, gust correlation time
        internal const float VeerDeg = 10f;            // standard deviation of direction
        internal const float VeerTime = 30f;           // s
        private static readonly Vector3 PressureCentre = new(0f, 0.25f, 0f);

        private ArticulationBody body;
        private float meanSpeed, meanToward, gust, veer;
        private System.Random random;

        internal static void Attach(OmniXController boat, string[] args) {
            int index = Array.IndexOf(args, Flag);
            string value = index + 1 < args.Length ? args[index + 1] : "";
            if (!RoboBoatCourse.ParseCurrent(value, out float speed, out float toward))
                throw new ArgumentException($"{Flag} expects speed,toward_deg; got {value}");
            ArticulationBody root = RoboBoatPhysics.Root(boat);
            var wind = root.gameObject.AddComponent<RoboBoatWind>();
            wind.body = root;
            wind.meanSpeed = speed;
            wind.meanToward = toward * Mathf.Deg2Rad;
            wind.random = new System.Random((int)RoboBoatPhysics.ReadFloat(args, "--crane-seed", 0f));
            Debug.Log($"CRANE_ROBOBOAT_WIND speed={speed:R} toward_deg={toward:R} " +
                      $"gust={GustIntensity:R} veer_deg={VeerDeg:R}");
        }

        /// <summary>Above-water area facing a relative wind at ``beta`` from the bow.</summary>
        internal static float ProjectedArea(float beta) =>
            FrontArea * Mathf.Abs(Mathf.Cos(beta)) + SideArea * Mathf.Abs(Mathf.Sin(beta));

        /// <summary>Drag of a relative air velocity (world-ROS, m/s) on a boat heading
        /// ``heading``; returns world-ROS newtons.</summary>
        internal static Vector2 Force(Vector2 relative, float heading) {
            float speed = relative.magnitude;
            if (speed < 1e-4f) return Vector2.zero;
            float beta = Mathf.Atan2(relative.y, relative.x) - heading;
            return 0.5f * AirDensity * DragCoefficient * ProjectedArea(beta) * speed * relative;
        }

        // Ornstein-Uhlenbeck step: mean-reverting noise with the given spread and memory.
        private float Wander(float value, float spread, float time, float dt) {
            double u1 = 1.0 - random.NextDouble(), u2 = random.NextDouble();
            float normal = (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2));
            return value - value * dt / time + spread * Mathf.Sqrt(2f * dt / time) * normal;
        }

        private void FixedUpdate() {
            float dt = Time.fixedDeltaTime;
            gust = Wander(gust, GustIntensity, GustTime, dt);
            veer = Wander(veer, VeerDeg * Mathf.Deg2Rad, VeerTime, dt);
            float speed = Mathf.Max(0f, meanSpeed * (1f + gust)) *
                          RoboBoatPhysics.Disturbance(Time.timeSinceLevelLoad);
            float toward = meanToward + veer;
            var air = new Vector2(speed * Mathf.Cos(toward), speed * Mathf.Sin(toward));
            Vector2 boat = RoboBoatCourse.UnityToRos(body.linearVelocity);
            float heading = RoboBoatCourse.RosYawOfUnityDirection(
                RoboBoatRosFrame.Rotation(body.transform.rotation) * Vector3.forward);
            Vector2 force = Force(air - boat, heading);
            Vector3 bow = RoboBoatRosFrame.Rotation(body.transform.rotation) * Vector3.forward;
            Vector3 at = body.worldCenterOfMass + PressureCentre + bow * 0.05f;
            body.AddForceAtPosition(RoboBoatCourse.RosToUnity(force, 0f), at);
        }
    }
}
