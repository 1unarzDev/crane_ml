using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using Crane.Drones.Environment;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Crane.Theater.Aerial {
    /// <summary>What the Drone Forest scene offers: spawn pose, corridor waypoints, person target, camera rigs.
    /// Found through the environment agent's <see cref="DroneForestMarker"/> components; when absent the demo
    /// falls back to a procedural stub forest and says so in the report.</summary>
    public sealed class ForestMarkers {
        public Vector3 spawnPos; public Quaternion spawnRot = Quaternion.identity;
        public readonly List<Vector3> corridor = new List<Vector3>();
        public Vector3? target;
        public readonly List<Transform> cameraRigs = new List<Transform>();
        public bool hasSpawn, isStubScene;
        public string Describe => $"spawn={(hasSpawn ? "marker" : "default")} corridor={corridor.Count} target={(target.HasValue ? "marker" : "spec")} camera_rigs={cameraRigs.Count} stub_scene={isStubScene}";

        public static ForestMarkers Scan() {
            var m = new ForestMarkers();
            var all = UnityEngine.Object.FindObjectsByType<DroneForestMarker>(FindObjectsInactive.Exclude);
            var wps = new List<DroneForestMarker>();
            foreach (var k in all) {
                switch (k.kind) {
                    case DroneForestMarker.Kind.Spawn: m.spawnPos = k.transform.position; m.spawnRot = Quaternion.Euler(0, k.transform.eulerAngles.y, 0); m.hasSpawn = true; break;
                    case DroneForestMarker.Kind.CorridorWaypoint: wps.Add(k); break;
                    case DroneForestMarker.Kind.Target: m.target = k.transform.position; break;
                    case DroneForestMarker.Kind.CameraRig: m.cameraRigs.Add(k.transform); break;
                }
            }
            wps.Sort((a, b) => string.CompareOrdinal(a.name, b.name));   // names carry the order (wp00, wp01, ...)
            foreach (var w in wps) m.corridor.Add(w.transform.position);
            return m;
        }
    }

    /// <summary>The Drone Forest as a Theater world: ENU metres relative to the spawn point (x east = Unity +x,
    /// y north = Unity +z, z up = Unity +y).</summary>
    public sealed class AerialWorld : ITheaterWorld {
        public ForestMarkers Markers { get; private set; } = new ForestMarkers();
        Bounds bounds = new Bounds(Vector3.zero, new Vector3(200, 60, 200));
        readonly List<Vector3> line = new List<Vector3>();
        public GameObject StubRoot { get; private set; }

        public IEnumerator Prepare(TheaterManifest manifest) {
            for (int i = 0; i < 3; i++) yield return null;
            Markers = ForestMarkers.Scan();
            bool hasGeometry = false;
            var b = new Bounds(); bool first = true;
            foreach (var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsInactive.Exclude)) {
                if (r is ParticleSystemRenderer) continue;
                if (first) { b = r.bounds; first = false; } else b.Encapsulate(r.bounds);
                hasGeometry = true;
            }
            if (!hasGeometry || (!Markers.hasSpawn && Markers.corridor.Count == 0)) {
                if (!hasGeometry) { StubRoot = StubForest.Build(1, 40f); Markers.isStubScene = true; b = new Bounds(new Vector3(0, 10, 20), new Vector3(120, 40, 120)); }
            }
            bounds = b;
            line.Clear(); line.AddRange(Markers.corridor);
        }

        Vector3 Origin => Markers.spawnPos;
        public Vector3 ToUnity(float x, float y, float z) => Origin + Markers.spawnRot * new Vector3(x, z, y);
        public float UnityYaw(float yawDeg) => 90f - yawDeg + Markers.spawnRot.eulerAngles.y;
        public float SurfaceY(Vector3 p) =>
            Physics.Raycast(p + Vector3.up * 200f, Vector3.down, out var hit, 400f) ? hit.point.y : Origin.y;
        public Bounds Bounds => bounds;
        public IReadOnlyList<Vector3> CentreLine => line;
        public void SetTime(double t) { }   // wind and foliage motion are scene-side and not time-driven here
        public string Describe => "aerial-forest " + Markers.Describe;
    }

    /// <summary>Access to the quad prefab's visual rig without a compile-time reference (the prefab scripts live in the default
    /// assembly): cameraMount, SetRotorSpeeds, AdvanceRotors, ResetVisuals by reflection.</summary>
    public sealed class QuadHandle {
        public readonly GameObject Root;
        public readonly Transform CameraMount;
        readonly Component rig;
        readonly MethodInfo setSpeeds, advance, reset;
        public bool HasRig => rig != null;

        public QuadHandle(GameObject root) {
            Root = root;
            foreach (var c in root.GetComponentsInChildren<MonoBehaviour>(true))
                if (c != null && c.GetType().Name == "DroneVisualRig") { rig = c; break; }
            if (rig != null) {
                var t = rig.GetType();
                setSpeeds = t.GetMethod("SetRotorSpeeds"); advance = t.GetMethod("AdvanceRotors"); reset = t.GetMethod("ResetVisuals");
                var f = t.GetField("cameraMount");
                CameraMount = f?.GetValue(rig) as Transform;
                var rb = rig as MonoBehaviour; if (rb != null) { var af = t.GetField("autoAdvance"); af?.SetValue(rig, false); }
                reset?.Invoke(rig, null);
            }
            if (CameraMount == null) {   // stub: OAK-D RGB position from the prefab builder (FLU 0.1195 m forward, 1.75 mm below the body origin)
                var go = new GameObject("Mount_Camera_OAKD_RGB (stub)");
                go.transform.SetParent(root.transform, false); go.transform.localPosition = new Vector3(0, -0.00175f, 0.1195f);
                CameraMount = go.transform;
            }
        }

        public void Pose(TrajSample s, double dt) {
            Root.transform.SetPositionAndRotation(s.pos, s.rot);
            if (rig == null) return;
            setSpeeds?.Invoke(rig, new object[] { s.rotorRadS });
            advance?.Invoke(rig, new object[] { (float)dt });
        }
    }

    /// <summary>The AOS 7 V5 research quad as a Theater robot. The prefab is injected by <see cref="PrefabLoader"/> (the editor
    /// capture sets it from AssetDatabase); without it a primitive stub is used.</summary>
    public sealed class AerialRobot : ITheaterRobot {
        public static Func<GameObject> PrefabLoader;
        public Mesh GhostMesh() {
            var parts = new List<CombineInstance>();
            parts.Add(new CombineInstance { mesh = MeshKit.Box(new Vector3(0.12f, 0.06f, 0.26f)), transform = Matrix4x4.identity });
            parts.Add(new CombineInstance { mesh = MeshKit.Box(new Vector3(0.36f, 0.015f, 0.02f)), transform = Matrix4x4.Rotate(Quaternion.Euler(0, 45, 0)) });
            parts.Add(new CombineInstance { mesh = MeshKit.Box(new Vector3(0.36f, 0.015f, 0.02f)), transform = Matrix4x4.Rotate(Quaternion.Euler(0, -45, 0)) });
            var m = new Mesh { name = "aerial ghost" }; m.CombineMeshes(parts.ToArray(), true, true); m.RecalculateBounds(); return m;
        }
        public GameObject CreateHero(string name) {
            GameObject go = null;
            var src = PrefabLoader?.Invoke();
            if (src != null) { go = UnityEngine.Object.Instantiate(src); go.name = name; }
            else go = StubForest.StubQuad(name);
            foreach (var c in go.GetComponentsInChildren<Collider>(true)) UnityEngine.Object.DestroyImmediate(c);
            foreach (var r in go.GetComponentsInChildren<Rigidbody>(true)) UnityEngine.Object.DestroyImmediate(r);
            return go;
        }
        public Quaternion HeroRotation(Quaternion q) => q;
        public float Length => 0.38f;
        public string Surface => "none";
    }

    /// <summary>Procedural stand-ins used only while the real forest scene or quad prefab are missing: trunks with
    /// crowns on a ground disc, a capsule person, a box quad. Seeded, so frames stay reproducible.</summary>
    public static class StubForest {
        public static GameObject Build(int seed, float canopyHeight) {
            var root = new GameObject("StubForest");
            var rnd = new System.Random(seed);
            var ground = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            ground.name = "ground"; ground.transform.SetParent(root.transform);
            ground.transform.localScale = new Vector3(260, 0.05f, 260); ground.transform.position = new Vector3(0, -0.05f, 20);
            Tint(ground, new Color(0.22f, 0.2f, 0.12f));
            for (int i = 0; i < 160; i++) {
                float x = (float)(rnd.NextDouble() * 160 - 80), z = (float)(rnd.NextDouble() * 160 - 40);
                if (Mathf.Abs(x) < 3f && z > -5 && z < 50) continue;   // keep a flight corridor along +z
                float h = canopyHeight * (0.7f + 0.4f * (float)rnd.NextDouble()), r = 0.15f + 0.15f * (float)rnd.NextDouble();
                var t = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                t.transform.SetParent(root.transform); t.transform.position = new Vector3(x, h * 0.5f, z); t.transform.localScale = new Vector3(r * 2, h * 0.5f, r * 2);
                Tint(t, new Color(0.25f, 0.17f, 0.1f));
                var c = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                c.transform.SetParent(root.transform); c.transform.position = new Vector3(x, h, z); c.transform.localScale = new Vector3(7, 4.5f, 7);
                Tint(c, new Color(0.08f, 0.22f, 0.07f));
            }
            var p = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            p.name = "stub person"; p.transform.SetParent(root.transform); p.transform.position = new Vector3(0, 0.9f, 40); p.transform.localScale = new Vector3(0.5f, 0.9f, 0.5f);
            Tint(p, new Color(0.85f, 0.25f, 0.15f));
            var sun = new GameObject("StubSun").AddComponent<Light>();
            sun.type = LightType.Directional; sun.transform.SetParent(root.transform); sun.transform.rotation = Quaternion.Euler(38, 30, 0);
            sun.intensity = 80000f;
            return root;
        }

        public static GameObject StubQuad(string name) {
            var go = new GameObject(name);
            var body = GameObject.CreatePrimitive(PrimitiveType.Cube); body.transform.SetParent(go.transform, false); body.transform.localScale = new Vector3(0.12f, 0.06f, 0.26f);
            Tint(body, new Color(0.1f, 0.1f, 0.1f));
            for (int i = 0; i < 4; i++) {
                var d = GameObject.CreatePrimitive(PrimitiveType.Cylinder); d.transform.SetParent(go.transform, false);
                d.transform.localPosition = new Vector3(i % 2 == 0 ? 0.13f : -0.13f, 0.03f, i < 2 ? 0.13f : -0.13f); d.transform.localScale = new Vector3(0.18f, 0.003f, 0.18f);
                Tint(d, new Color(0.3f, 0.3f, 0.3f));
            }
            return go;
        }

        static void Tint(GameObject go, Color c) {
            var r = go.GetComponent<Renderer>();
            var sh = Shader.Find("HDRP/Lit");
            var m = sh != null ? new Material(sh) : new Material(r.sharedMaterial);
            m.SetColor("_BaseColor", c); m.color = c;
            r.sharedMaterial = m;
        }
    }

    static class AerialPluginRegistration {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Register() {
            TheaterPlugins.Worlds["aerial-forest"] = () => new AerialWorld();
            TheaterPlugins.Robots["aos7-quad"] = () => new AerialRobot();
        }
    }
}
