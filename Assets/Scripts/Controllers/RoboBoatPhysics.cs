using System;
using System.Globalization;
using System.Linq;
using Sim.Actuators.Motors;
using Sim.Physics.Water.Dynamics;
using Sim.Utils;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// The real boat's physical numbers, applied over the scene's estimates whenever the
    /// RoboBoat contract runs: 30 kg loaded, four Blue Robotics T200 at 16 V, hull drag
    /// relative to the water (so a current carries the boat), no artificial Unity damping,
    /// and an optional world-fixed wind. ``--crane-roboboat-legacy-physics`` keeps the
    /// scene as authored.
    /// </summary>
    internal static class RoboBoatPhysics {
        internal const float Mass = 30f;               // kg, loaded
        internal const float T200Forward = 51.5f;      // N, 5.25 kgf at 16 V
        internal const float T200Reverse = 40.2f;      // N, 4.1 kgf at 16 V
        internal const float T200SpinUp = 0.1f;        // s, a T200 from rest to commanded speed
        // Whole-boat inertia about the centre of mass, kg m^2, roll / pitch / yaw. Two hulls
        // carry 60 % of the 30 kg as slender 1.06 m bodies 0.33 m either side of the centre
        // line; the deck, electronics and batteries carry the rest as a 0.5 x 0.4 x 0.25 m
        // box: roll 2 * 9 * 0.33^2 + 12 * (0.4^2 + 0.25^2) / 12 = 2.2, pitch
        // 2 * 9 * 1.06^2 / 12 + 12 * (0.5^2 + 0.25^2) / 12 = 2.0, yaw
        // 2 * 9 * (1.06^2 / 12 + 0.33^2) + 12 * (0.5^2 + 0.4^2) / 12 = 4.1.
        internal static readonly Vector3 BoatInertia = new(2.2f, 2.0f, 4.1f);
        // Unity damping is not water; keep a trace for numerical calm only.
        internal const float ResidualLinearDamping = 0.05f;
        internal const float ResidualAngularDamping = 0.2f;

        /// <summary>How much of the wind and current is on: 0 until
        /// ``--crane-disturbance-after`` seconds of the scene, then ramping to 1 over 10 s,
        /// so a run can start from a boat sitting where it was placed.</summary>
        internal static float DisturbanceAfter;
        internal static float Disturbance(float sceneTime) =>
            Mathf.Clamp01((sceneTime - DisturbanceAfter) / 10f);

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            DisturbanceAfter = ReadFloat(args, "--crane-disturbance-after", -10f);
            bool contract = Array.IndexOf(args, CraneRoboBoatSensorContractBootstrap.Flag) >= 0;
            bool wind = Array.IndexOf(args, RoboBoatWind.Flag) >= 0;
            bool legacy = Array.IndexOf(args, "--crane-roboboat-legacy-physics") >= 0;
            if ((!contract || legacy) && !wind) return;
            SceneManager.sceneLoaded += (_, _) => {
                try {
                    OmniXController boat = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                        FindObjectsInactive.Exclude);
                    if (boat == null) return;
                    if (contract && !legacy) Apply(boat);
                    if (wind) RoboBoatWind.Attach(boat, args);
                }
                catch (Exception error) {
                    Debug.LogError($"CRANE_ROBOBOAT_PHYSICS_FAILED {error}");
                }
            };
        }

        internal static ArticulationBody Root(OmniXController boat) {
            ArticulationBody body = boat.GetComponentInParent<ArticulationBody>() ??
                                    boat.GetComponentInChildren<ArticulationBody>();
            if (body == null) throw new MissingComponentException("boat has no articulation");
            while (!body.isRoot) body = body.transform.parent.GetComponentInParent<ArticulationBody>();
            return body;
        }

        /// <summary>Scale every link's mass by the same factor so the total is ``total``.</summary>
        internal static float[] ScaledMasses(float[] masses, float total) {
            float sum = masses.Sum();
            return masses.Select(mass => mass * total / sum).ToArray();
        }

        private static void Apply(OmniXController boat) {
            ArticulationBody root = Root(boat);
            ArticulationBody[] all = root.GetComponentsInChildren<ArticulationBody>();
            // Thruster rotors spin on their own joints with tiny authored inertia; only the
            // hull and frame links (fixed to the root) carry the boat's mass.
            ArticulationBody[] links = all.Where(link => link.isRoot ||
                link.jointType == ArticulationJointType.FixedJoint).ToArray();
            float rotors = all.Except(links).Sum(link => link.mass);
            float[] masses = ScaledMasses(links.Select(link => link.mass).ToArray(), Mass - rotors);
            for (int i = 0; i < links.Length; i++) links[i].mass = masses[i];
            Vector3 before = Composite(root, all);
            SetInertia(root, all, BoatInertia);
            foreach (ArticulationBody link in all) {
                link.linearDamping = ResidualLinearDamping;
                link.angularDamping = ResidualAngularDamping;
            }
            // Thrust from the ESC command: the rotor joints top out near 150 rad/s, well
            // short of the 500 rad/s full throttle maps to, so their speed is no measure.
            foreach (Thruster thruster in root.GetComponentsInChildren<Thruster>())
                thruster.SetThrustCurve(T200Forward, T200Reverse, T200SpinUp);
            // One hull model replaces the scene's three overlapping ones: GeneralDynamics and
            // the per-face current drag were tuned for the old 3.5 kN thrusters, and Fossen's
            // damping uses ground velocity in the imported mesh's axes.
            foreach (GeneralDynamics extra in root.GetComponentsInChildren<GeneralDynamics>())
                extra.enabled = false;
            foreach (Current extra in root.GetComponentsInChildren<Current>()) extra.enabled = false;
            foreach (FossenDynamics extra in root.GetComponentsInChildren<FossenDynamics>())
                extra.enabled = false;
            var hull = root.gameObject.AddComponent<RoboBoatHull>();
            hull.body = root;
            hull.links = all;
            // The LiDAR as a Velodyne VLP-16: 16 rings 2 degrees apart over +-15 degrees and
            // 0.2 degree steps at 10 Hz. (The pattern spaces rings FOV / count apart.)
            // Its mount follows the imported hull's axes, so the cloud gets the same turn
            // into REP-103 as the body (x forward in lidar_link, as the real driver gives).
            foreach (var lidar in root.GetComponentsInChildren<Sim.Sensors.Lidar.Lidar3D>()) {
                lidar.Configure(1800, 16, 32f * Mathf.Deg2Rad, 100f);
                lidar.SetFrameCorrection(Quaternion.Inverse(
                    Quaternion.Inverse(lidar.transform.rotation) *
                    RoboBoatRosFrame.Rotation(root.transform.rotation)));
            }
            Vector3 inertia = Composite(root, all);
            Debug.Log($"CRANE_ROBOBOAT_PHYSICS mass={all.Sum(link => link.mass):R} " +
                      $"links={links.Length}+{all.Length - links.Length} thrust={T200Forward:R}/{T200Reverse:R}N " +
                      $"top_speed={RoboBoatHull.TopSpeed(4 * T200Forward * Mathf.Sqrt(0.5f), RoboBoatHull.Surge):R} " +
                      $"scene_inertia_rpy=({before.x:F2},{before.y:F2},{before.z:F2}) inertia_rpy=({inertia.x:R},{inertia.y:R}," +
                      $"{inertia.z:R})");
        }

        /// <summary>Body axes (bow, port, up) of the hull in world space.</summary>
        private static (Vector3, Vector3, Vector3) Axes(ArticulationBody root) {
            Quaternion frame = RoboBoatRosFrame.Rotation(root.transform.rotation);
            return (frame * Vector3.forward, Vector3.Cross(frame * Vector3.forward, frame * Vector3.up),
                frame * Vector3.up);
        }

        private static Vector3 CentreOfMass(ArticulationBody[] links) =>
            links.Aggregate(Vector3.zero, (sum, link) => sum + link.mass * link.worldCenterOfMass) /
            links.Sum(link => link.mass);

        /// <summary>Moment of inertia of one link about ``axis`` through ``point``.</summary>
        private static float About(ArticulationBody link, Vector3 axis, Vector3 point) {
            Vector3 local = Quaternion.Inverse(link.transform.rotation * link.inertiaTensorRotation) * axis;
            float own = Vector3.Dot(Vector3.Scale(local, local), link.inertiaTensor);
            Vector3 d = link.worldCenterOfMass - point;
            return own + link.mass * (d.sqrMagnitude - Mathf.Pow(Vector3.Dot(d, axis), 2));
        }

        /// <summary>The whole boat's roll, pitch and yaw inertia about its centre of mass.</summary>
        internal static Vector3 Composite(ArticulationBody root, ArticulationBody[] links) {
            var (bow, port, up) = Axes(root);
            Vector3 com = CentreOfMass(links);
            return new Vector3(links.Sum(l => About(l, bow, com)), links.Sum(l => About(l, port, com)),
                links.Sum(l => About(l, up, com)));
        }

        /// <summary>Give the boat ``target`` (roll, pitch, yaw) about its centre of mass: the
        /// other links keep small tensors and their mass offsets, the root takes the rest.</summary>
        private static void SetInertia(ArticulationBody root, ArticulationBody[] links, Vector3 target) {
            foreach (var link in links.Where(link => !link.isRoot)) {
                link.automaticInertiaTensor = false;
                link.inertiaTensor = Vector3.one * Mathf.Max(1e-6f, 1e-3f * link.mass);
                link.inertiaTensorRotation = Quaternion.identity;
            }
            // PhysX ignores a zero tensor, so start the root from a small one.
            const float floor = 0.05f;
            root.automaticInertiaTensor = false;
            root.inertiaTensorRotation = Quaternion.identity;
            root.inertiaTensor = Vector3.one * floor;
            var (bow, port, up) = Axes(root);
            Vector3 com = CentreOfMass(links);
            Vector3 rest = new Vector3(links.Sum(l => About(l, bow, com)),
                links.Sum(l => About(l, port, com)), links.Sum(l => About(l, up, com))) -
                Vector3.one * floor;
            // Where the links' own spread already exceeds the target (roll: the hulls sit
            // wide and low), that is the boat's; the root only adds what is missing.
            Vector3 need = Vector3.Max(target - rest, Vector3.one * floor);  // roll, pitch, yaw
            // Into the root's local axes, whichever way the imported mesh points them.
            Vector3 tensor = Vector3.zero;
            for (int k = 0; k < 3; k++) {
                Vector3 axis = root.transform.rotation * new Vector3(k == 0 ? 1 : 0, k == 1 ? 1 : 0, k == 2 ? 1 : 0);
                float[] fit = { Mathf.Abs(Vector3.Dot(axis, bow)), Mathf.Abs(Vector3.Dot(axis, port)),
                    Mathf.Abs(Vector3.Dot(axis, up)) };
                int best = fit[0] >= fit[1] && fit[0] >= fit[2] ? 0 : fit[1] >= fit[2] ? 1 : 2;
                tensor[k] = need[best];
            }
            root.inertiaTensor = tensor;
            Debug.Log($"CRANE_ROBOBOAT_INERTIA rest_rpy=({rest.x:F2},{rest.y:F2},{rest.z:F2}) " +
                      $"root_local=({tensor.x:F2},{tensor.y:F2},{tensor.z:F2})");
        }

        internal static float ReadFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length &&
                   float.TryParse(args[index + 1], NumberStyles.Float,
                       CultureInfo.InvariantCulture, out float value) ? value : fallback;
        }
    }

    /// <summary>
    /// Hull drag in the REP-103 body frame, on the velocity relative to the water:
    /// X = -(Xu + Xuu |u|) u for surge, likewise sway and yaw (Fossen's linear plus
    /// quadratic form). Surge is set so four T200 at full surge (146 N along the bow) reach
    /// the boat's measured 1.8 m/s; sway and yaw keep Fossen's defaults, a catamaran being
    /// draggier abeam. A current therefore carries the boat, as real water does. Heave,
    /// roll and pitch are damped too, or buoyancy alone would rock the hull ever harder.
    /// </summary>
    internal sealed class RoboBoatHull : MonoBehaviour {
        internal static readonly Vector2 Surge = new(30f, 27f);   // N/(m/s), N/(m/s)^2
        internal static readonly Vector2 Sway = new(40f, 60f);
        internal static readonly Vector2 Yaw = new(20f, 30f);     // N m/(rad/s), N m/(rad/s)^2
        internal static readonly Vector2 Heave = new(150f, 40f);
        internal static readonly Vector2 Roll = new(30f, 10f);    // twin hulls resist rolling
        internal static readonly Vector2 Pitch = new(30f, 10f);
        internal ArticulationBody body;
        internal ArticulationBody[] links;
        private UnityEngine.Rendering.HighDefinition.WaterSurface water;
        private int steps;

        internal static float Drag(float velocity, Vector2 coefficients) =>
            -(coefficients.x + coefficients.y * Mathf.Abs(velocity)) * velocity;

        /// <summary>Steady speed where ``force`` balances the drag.</summary>
        internal static float TopSpeed(float force, Vector2 c) =>
            (-c.x + Mathf.Sqrt(c.x * c.x + 4f * c.y * force)) / (2f * c.y);

        private void Start() =>
            water = FindAnyObjectByType<UnityEngine.Rendering.HighDefinition.WaterSurface>();

        private void FixedUpdate() {
            if (++steps == 50 && links != null) {
                Vector3 i = RoboBoatPhysics.Composite(body, links);
                Debug.Log($"CRANE_ROBOBOAT_INERTIA_CHECK rpy=({i.x:F2},{i.y:F2},{i.z:F2}) " +
                          $"root=({body.inertiaTensor.x:F2},{body.inertiaTensor.y:F2},{body.inertiaTensor.z:F2}) " +
                          $"root_rotation={body.inertiaTensorRotation.eulerAngles} auto={body.automaticInertiaTensor}");
            }
            Vector3 current = Vector3.zero;
            if (water != null) {
                float angle = water.largeOrientationValue * Mathf.Deg2Rad;
                current = water.largeCurrentSpeedValue *
                          RoboBoatPhysics.Disturbance(Time.timeSinceLevelLoad) *
                          new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
            }
            Quaternion frame = RoboBoatRosFrame.Rotation(body.transform.rotation);
            Vector3 bow = frame * Vector3.forward, port = Vector3.Cross(bow, Vector3.up);
            Vector3 relative = body.linearVelocity - current;
            Vector3 up = frame * Vector3.up;
            float u = Vector3.Dot(relative, bow), v = Vector3.Dot(relative, port);
            float w = Vector3.Dot(relative, up);
            body.AddForce(bow * Drag(u, Surge) + port * Drag(v, Sway) + up * Drag(w, Heave));
            // Damping opposes the rate about each body axis, whatever the handedness.
            Vector3 spin = body.angularVelocity;
            body.AddTorque(up * Drag(Vector3.Dot(spin, up), Yaw) +
                           bow * Drag(Vector3.Dot(spin, bow), Roll) +
                           port * Drag(Vector3.Dot(spin, port), Pitch));
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
