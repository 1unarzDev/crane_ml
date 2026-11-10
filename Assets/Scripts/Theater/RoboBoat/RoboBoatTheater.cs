using System;
using System.Collections;
using System.Collections.Generic;
using Sim.Controllers;
using Sim.Course;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater.RoboBoat {
    /// <summary>RoboBoat plug-ins for the ghost theater. The world is the player's own course, rebuilt from the
    /// manifest's truth by the course bootstrap (<c>--crane-course-replay</c>, passed by the launcher from the
    /// manifest's world.args); the trajectory frame is /crane/odom (ENU), Unity = (-y, z, x). The robot is the
    /// scene boat: a catamaran ghost mesh, and the boat's own visuals (cloned with nothing live) for heroes.</summary>
    static class RoboBoatTheaterPlugins {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Register() {
            TheaterPlugins.Worlds["roboboat"] = () => new RoboBoatTheaterWorld();
            TheaterPlugins.Robots["roboboat"] = () => new RoboBoatTheaterRobot();
        }

        /// <summary>The scene boat's body, its visual template; found once, before the live boat is switched off.</summary>
        public static Transform Body;
    }

    public sealed class RoboBoatTheaterWorld : ITheaterWorld {
        WaterSurface[] waters = Array.Empty<WaterSurface>();
        float waterY, waterT0;
        string waterMode = "still";
        Bounds bounds;
        readonly List<Vector3> line = new List<Vector3>();
        string course = "none";

        public IEnumerator Prepare(TheaterManifest m) {
            for (int i = 0; i < 600 && UnityEngine.Object.FindAnyObjectByType<CourseStateRegistry>() == null; i++) yield return null;
            var registry = UnityEngine.Object.FindAnyObjectByType<CourseStateRegistry>();
            course = registry != null ? registry.transform.root.name : "none (no course built: pass the manifest's world.args)";
            RoboBoatTheaterPlugins.Body = RoboBoatStartReset.FindBoatBody();
            waters = UnityEngine.Object.FindObjectsByType<WaterSurface>(FindObjectsInactive.Exclude);
            Array.Sort(waters, (x, y) => string.CompareOrdinal(x.name, y.name));
            waterY = waters.Length > 0 ? waters[0].transform.position.y : RoboBoatTheaterPlugins.Body != null ? RoboBoatTheaterPlugins.Body.position.y : 0f;
            waterMode = TJson.Str(m.Water, "mode", "still");
            waterT0 = (float)TJson.Num(m.Water, "t0_s", 0);
            foreach (var w in waters) { w.timeMultiplier = 0f; w.simulationTime = waterT0; }
            DisableLive();
            var b = TJson.Arr(m.World, "bounds");
            if (b != null && b.Count == 4) {
                Vector3 a = ToUnity((float)(double)b[0], (float)(double)b[1], 0), c = ToUnity((float)(double)b[2], (float)(double)b[3], 0);
                bounds = new Bounds((a + c) * 0.5f, new Vector3(Mathf.Abs(c.x - a.x), 2f, Mathf.Abs(c.z - a.z)));
            }
            else bounds = new Bounds(new Vector3(0, waterY, 40), new Vector3(80, 2, 80));
            foreach (var p in TJson.Arr(m.World, "centreline") ?? new List<object>())
                if (p is List<object> xy && xy.Count >= 2) line.Add(ToUnity((float)(double)xy[0], (float)(double)xy[1], 0));
        }

        /// <summary>Everything that would simulate or publish is switched off; bodies are kinematic; the scene boat
        /// is hidden (heroes are clones of it).</summary>
        static void DisableLive() {
            foreach (var behaviour in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include)) {
                string n = behaviour.GetType().FullName ?? "";
                if (n.StartsWith("Sim.Physics.") || n.StartsWith("Sim.Actuators.") || n.StartsWith("Sim.Controllers.") || n.StartsWith("Sim.Sensors."))
                    behaviour.enabled = false;
            }
            foreach (var body in UnityEngine.Object.FindObjectsByType<Rigidbody>(FindObjectsInactive.Include)) body.isKinematic = true;
            foreach (var body in UnityEngine.Object.FindObjectsByType<ArticulationBody>(FindObjectsInactive.Include))
                if (body.isRoot) body.immovable = true;
            var boat = RoboBoatTheaterPlugins.Body;
            if (boat != null) boat.gameObject.SetActive(false);
        }

        public Vector3 ToUnity(float x, float y, float z) => new Vector3(-y, waterY + z, x);
        public float UnityYaw(float yawDeg) => -yawDeg;
        public float SurfaceY(Vector3 p) => waterY;
        public Bounds Bounds => bounds;
        public IReadOnlyList<Vector3> CentreLine => line;

        public void SetTime(double t) {
            float st = waterMode == "still" ? waterT0 : waterT0 + (float)t;
            foreach (var w in waters) { w.timeMultiplier = 0f; w.simulationTime = st; }
        }

        public string Describe => $"roboboat course={course} water={waterMode} waters={waters.Length} waterline={waterY:0.###}";
    }

    public sealed class RoboBoatTheaterRobot : ITheaterRobot {
        static readonly Quaternion BodyFromPose = Quaternion.Euler(0f, 90f, 0f);   // RoboBoatRosFrame: ros = body * Euler(0,-90,0)
        Mesh ghost;

        public Mesh GhostMesh() => ghost ??= MeshKit.Catamaran(1.06f, 0.9f, 0.26f, 0.32f);
        public float Length => 1.06f;
        public string Surface => "water";
        public Quaternion HeroRotation(Quaternion pose) => pose * BodyFromPose;

        public GameObject CreateHero(string name) {
            var body = RoboBoatTheaterPlugins.Body;
            if (body == null) return new ArrowRobot().CreateHero(name);
            bool was = body.gameObject.activeSelf;
            body.gameObject.SetActive(false);                      // the clone is born inactive: nothing in it wakes
            var hero = UnityEngine.Object.Instantiate(body.gameObject);
            body.gameObject.SetActive(was);
            hero.name = name;
            Strip(hero);
            hero.SetActive(true);
            return hero;
        }

        /// <summary>Keep only what draws: deepest components first, scripts before bodies, until nothing else is left.</summary>
        static void Strip(GameObject root) {
            for (int pass = 0; pass < 6; pass++) {
                var doomed = new List<Component>();
                foreach (var c in root.GetComponentsInChildren<Component>(true)) {
                    if (c == null || c is Transform || c is MeshFilter || c is MeshRenderer || c is SkinnedMeshRenderer || c is LODGroup) continue;
                    doomed.Add(c);
                }
                if (doomed.Count == 0) return;
                doomed.Sort((a, b) => Rank(a).CompareTo(Rank(b)) != 0 ? Rank(a).CompareTo(Rank(b)) : Depth(b.transform).CompareTo(Depth(a.transform)));
                foreach (var c in doomed) {
                    if (c == null) continue;
                    try { UnityEngine.Object.DestroyImmediate(c); } catch (Exception) { }
                }
            }
        }

        // scripts that [RequireComponent] another go before the rest, so a required component (e.g. Submersion, which
        // Buoyancy/Current/GeneralDynamics require) is never destroyed while a dependent still exists (Unity logs an error)
        static int Rank(Component c) => c is MonoBehaviour ? (Attribute.IsDefined(c.GetType(), typeof(RequireComponent), true) ? -1 : 0) : c is Joint ? 1 : c is Collider ? 2 : c is Rigidbody || c is ArticulationBody ? 3 : 4;
        static int Depth(Transform t) { int d = 0; while (t.parent != null) { d++; t = t.parent; } return d; }
    }
}
