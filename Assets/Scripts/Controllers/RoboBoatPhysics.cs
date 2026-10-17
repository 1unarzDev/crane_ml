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
        // Unity damping is not water; keep a trace for numerical calm only.
        internal const float ResidualLinearDamping = 0.05f;
        internal const float ResidualAngularDamping = 0.2f;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
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
            for (int i = 0; i < links.Length; i++) {
                links[i].mass = masses[i];
                // The authored tensors were placeholders (10, 15, 10 kg m^2 on a 3 kg link);
                // let PhysX integrate the colliders at the new masses instead.
                links[i].automaticInertiaTensor = true;
            }
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
            Vector3 inertia = links.Aggregate(Vector3.zero, (sum, link) => sum + link.inertiaTensor);
            Debug.Log($"CRANE_ROBOBOAT_PHYSICS mass={all.Sum(link => link.mass):R} " +
                      $"links={links.Length}+{all.Length - links.Length} thrust={T200Forward:R}/{T200Reverse:R}N " +
                      $"top_speed={RoboBoatHull.TopSpeed(4 * T200Forward * Mathf.Sqrt(0.5f), RoboBoatHull.Surge):R} link_inertia_sum=({inertia.x:R},{inertia.y:R}," +
                      $"{inertia.z:R})");
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
    /// draggier abeam. A current therefore carries the boat, as real water does.
    /// </summary>
    internal sealed class RoboBoatHull : MonoBehaviour {
        internal static readonly Vector2 Surge = new(30f, 27f);   // N/(m/s), N/(m/s)^2
        internal static readonly Vector2 Sway = new(40f, 60f);
        internal static readonly Vector2 Yaw = new(20f, 30f);     // N m/(rad/s), N m/(rad/s)^2
        internal ArticulationBody body;
        private UnityEngine.Rendering.HighDefinition.WaterSurface water;

        internal static float Drag(float velocity, Vector2 coefficients) =>
            -(coefficients.x + coefficients.y * Mathf.Abs(velocity)) * velocity;

        /// <summary>Steady speed where ``force`` balances the drag.</summary>
        internal static float TopSpeed(float force, Vector2 c) =>
            (-c.x + Mathf.Sqrt(c.x * c.x + 4f * c.y * force)) / (2f * c.y);

        private void Start() =>
            water = FindAnyObjectByType<UnityEngine.Rendering.HighDefinition.WaterSurface>();

        private void FixedUpdate() {
            Vector3 current = Vector3.zero;
            if (water != null) {
                float angle = water.largeOrientationValue * Mathf.Deg2Rad;
                current = water.largeCurrentSpeedValue *
                          new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
            }
            Quaternion frame = RoboBoatRosFrame.Rotation(body.transform.rotation);
            Vector3 bow = frame * Vector3.forward, port = Vector3.Cross(bow, Vector3.up);
            Vector3 relative = body.linearVelocity - current;
            float u = Vector3.Dot(relative, bow), v = Vector3.Dot(relative, port);
            float r = -body.angularVelocity.y;  // Unity turns clockwise about +y
            body.AddForce(bow * Drag(u, Surge) + port * Drag(v, Sway));
            body.AddTorque(-Vector3.up * Drag(r, Yaw));
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
            float speed = Mathf.Max(0f, meanSpeed * (1f + gust));
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
