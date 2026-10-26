using System;
using System.Globalization;
using System.Linq;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Dynamics;
using Sim.Physics.Water.Statics;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Water.Boat {
    /// <summary>
    /// The RoboBoat plant bootstrap. On every scene with a thruster-driven articulation it applies a
    /// <see cref="MarineSurfaceProfile"/> (<c>--crane-boat-profile</c>, default Ideal; JSON override
    /// <c>--crane-boat-profile-json</c>), then the optional error layer
    /// (<c>--crane-calibration-faults</c>): mass layout, zero Unity link damping, the legacy water
    /// components (Fossen, GeneralDynamics, Current) off, fixedDeltaTime, and one
    /// <see cref="MarineSurfacePlant"/> on the hull. The scene and prefab stay as authored;
    /// <c>--crane-roboboat-legacy-physics</c> keeps their physics.
    /// </summary>
    public static class RoboBoatPhysics {
        public const string LegacyFlag = "--crane-roboboat-legacy-physics";
        /// <summary>Visible bow is hull-local −X: the same offset as the controllers' RoboBoatRosFrame.</summary>
        public static readonly Quaternion BodyFromFlu = Quaternion.Euler(0f, -90f, 0f);

        /// <summary>How much of the wind and current is on: 0 until
        /// ``--crane-disturbance-after`` seconds of the scene, then ramping to 1 over 10 s,
        /// so a run can start from a boat sitting where it was placed.</summary>
        public static float DisturbanceAfter = -10f;
        public static float Disturbance(float sceneTime) =>
            Mathf.Clamp01((sceneTime - DisturbanceAfter) / 10f);

        /// <summary>The plant applied in this scene, if any.</summary>
        public static MarineSurfacePlant Plant { get; private set; }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            DisturbanceAfter = ReadFloat(args, "--crane-disturbance-after", -10f);
            if (Array.IndexOf(args, LegacyFlag) >= 0) {
                Debug.Log("CRANE_PLANT_LEGACY scene physics kept as authored");
                return;
            }
            SceneManager.sceneLoaded += (_, _) => {
                try {
                    Thruster thruster = UnityEngine.Object.FindAnyObjectByType<Thruster>(FindObjectsInactive.Exclude);
                    if (thruster != null) Apply(Root(thruster), args);
                }
                catch (Exception error) {
                    Debug.LogError($"CRANE_ROBOBOAT_PHYSICS_FAILED {error}");
                }
            };
        }

        /// <summary>The articulation root of the tree that carries <paramref name="part"/>.</summary>
        public static ArticulationBody Root(Component part) {
            ArticulationBody body = part.GetComponentInParent<ArticulationBody>() ??
                                    part.GetComponentInChildren<ArticulationBody>();
            if (body == null) throw new MissingComponentException("boat has no articulation");
            while (!body.isRoot) body = body.transform.parent.GetComponentInParent<ArticulationBody>();
            return body;
        }

        public static MarineSurfacePlant Apply(ArticulationBody root, string[] args) {
            MarineSurfaceProfile profile = ProfileBootstrap.Load<MarineSurfaceProfile>(args);
            var (errors, errorsSha) = RoboBoatCalibrationFaults.Load(args);
            return Apply(root, profile, errors, errorsSha);
        }

        public static MarineSurfacePlant Apply(ArticulationBody root, MarineSurfaceProfile profile,
            ActuatorErrorProfile errors, string errorsSha = "none") {
            MarineSurfaceParameters nominal = profile.Parameters;
            float dt = nominal.fixedDeltaTime;
            MarineSurfaceParameters p = errors?.Apply(nominal, dt) ?? nominal.Clone();
            ProfileValidation validation = p.Validate(dt);
            if (!validation.Ok) {
                Debug.LogError($"CRANE_PLANT_REJECTED profile={profile.ProfileName} {validation}");
                return null;
            }
            foreach (string warning in validation.Warnings)
                Debug.LogWarning($"CRANE_PLANT_WARNING profile={profile.ProfileName} {warning}");

            Time.fixedDeltaTime = dt;
            ArticulationBody[] all = root.GetComponentsInChildren<ArticulationBody>();
            Thruster[] thrusters = root.GetComponentsInChildren<Thruster>();
            string missing = string.Join(",", p.thrusters.Where(s => !thrusters.Any(t => MarineSurfacePlant.Matches(t, s.name)))
                .Select(s => s.name));
            if (missing.Length > 0) {
                Debug.LogError($"CRANE_PLANT_REJECTED profile={profile.ProfileName} missing_thrusters={missing}");
                return null;
            }
            SetMassLayout(root, p.mass, new Vector3(p.ixx, p.iyy, p.izz), p.comBow, p.comPort);
            foreach (ArticulationBody link in all) {
                link.linearDamping = 0f;
                link.angularDamping = 0f;
            }
            root.maxLinearVelocity = Mathf.Min(root.maxLinearVelocity, Mathf.Max(10f, 4f * p.envelope.surge));
            foreach (FossenDynamics legacy in root.GetComponentsInChildren<FossenDynamics>()) legacy.enabled = false;
            foreach (GeneralDynamics legacy in root.GetComponentsInChildren<GeneralDynamics>()) legacy.enabled = false;
            foreach (Current legacy in root.GetComponentsInChildren<Current>()) legacy.enabled = false;

            var medium = new WaterMedium(UnityEngine.Object.FindAnyObjectByType<WaterSurface>()) { Ramp = Disturbance };
            float rigidMass = all.Sum(link => link.mass);
            Vector3 inertia = Composite(root, all);
            Plant = root.gameObject.GetComponent<MarineSurfacePlant>() ?? root.gameObject.AddComponent<MarineSurfacePlant>();
            Plant.Configure(p, root, new BodyFrame(root.transform, BodyFromFlu), medium, thrusters,
                root.GetComponentInChildren<Buoyancy>(), new[] { errors?.DragModel(p.PlanarModel(rigidMass, inertia.z)) },
                rigidMass, inertia.z);
            Debug.Log($"CRANE_ROBOBOAT_PHYSICS mass={rigidMass:R} inertia_rpy=({inertia.x:F3},{inertia.y:F3},{inertia.z:F3}) " +
                      $"dt={dt:R} thrusters={string.Join(",", p.thrusters.Select(t => t.name))} " +
                      $"step_ratio=({p.StepRatio(0, dt):F2},{p.StepRatio(1, dt):F2},{p.StepRatio(2, dt):F2})");
            Debug.Log(ProfileBootstrap.AppliedLine(profile, $"errors_sha256={errorsSha}"));
            return Plant;
        }

        /// <summary>Scale every link's mass by the same factor so the total is ``total``.</summary>
        public static float[] ScaledMasses(float[] masses, float total) {
            float sum = masses.Sum();
            return masses.Select(mass => mass * total / sum).ToArray();
        }

        /// <summary>Total ``mass`` (kg), whole-boat ``inertia`` (roll, pitch, yaw, kg m^2) about
        /// the centre of mass, and the centre of mass moved ``comBow`` and ``comPort`` metres.
        /// Thruster rotors keep their small authored mass; the hull and frame links (fixed to the
        /// root) carry the rest.</summary>
        public static void SetMassLayout(ArticulationBody root, float mass, Vector3 inertia,
            float comBow, float comPort) {
            ArticulationBody[] all = root.GetComponentsInChildren<ArticulationBody>();
            ArticulationBody[] links = all.Where(link => link.isRoot ||
                link.jointType == ArticulationJointType.FixedJoint).ToArray();
            float rotors = all.Except(links).Sum(link => link.mass);
            float[] masses = ScaledMasses(links.Select(link => link.mass).ToArray(), mass - rotors);
            for (int i = 0; i < links.Length; i++) links[i].mass = masses[i];
            if (comBow != 0 || comPort != 0) {
                // Only the root moves its own centre, by enough to move the whole boat's.
                var (bow, port, _) = Axes(root);
                Vector3 shift = (bow * comBow + port * comPort) * links.Sum(l => l.mass) / root.mass;
                root.automaticCenterOfMass = false;
                root.centerOfMass += Quaternion.Inverse(root.transform.rotation) * shift;
            }
            SetInertia(root, all, inertia);
            Vector3 result = Composite(root, all);
            Debug.Log($"CRANE_ROBOBOAT_MASS_LAYOUT mass={all.Sum(link => link.mass):R} " +
                      $"com_shift=({comBow:R},{comPort:R}) inertia_rpy=({result.x:R},{result.y:R},{result.z:R})");
        }

        /// <summary>Body axes (bow, port, up) of the hull in world space.</summary>
        private static (Vector3, Vector3, Vector3) Axes(ArticulationBody root) {
            Quaternion frame = root.transform.rotation * BodyFromFlu;
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
        public static Vector3 Composite(ArticulationBody root, ArticulationBody[] links) {
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

        public static float ReadFloat(string[] args, string key, float fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length &&
                   float.TryParse(args[index + 1], NumberStyles.Float,
                       CultureInfo.InvariantCulture, out float value) ? value : fallback;
        }
    }
}
