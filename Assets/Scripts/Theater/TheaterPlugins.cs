using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Theater {
    /// <summary>The world a fleet ran in: builds or finds it in the loaded scene, maps the trajectory frame to
    /// Unity, and owns its time-driven state (water, sky). A world must take every time source from
    /// <see cref="SetTime"/> so a frame is a pure function of its index.</summary>
    public interface ITheaterWorld {
        /// <summary>Make the world ready (course built, live simulation off); yields frames while waiting.</summary>
        IEnumerator Prepare(TheaterManifest manifest);
        /// <summary>A world-frame position (ENU metres) in Unity.</summary>
        Vector3 ToUnity(float x, float y, float z);
        /// <summary>Unity yaw (degrees about +y) of a world-frame yaw (degrees CCW from +x).</summary>
        float UnityYaw(float yawDeg);
        /// <summary>Height a planar pose is seated at (the water or the ground), in Unity y.</summary>
        float SurfaceY(Vector3 unityPosition);
        /// <summary>Unity bounds of the course (cameras frame it; ghosts outside are reported).</summary>
        Bounds Bounds { get; }
        /// <summary>Points along the course in Unity (cameras fly along it).</summary>
        IReadOnlyList<Vector3> CentreLine { get; }
        /// <summary>Set every time-driven part of the world to time t (s since the runs' start).</summary>
        void SetTime(double t);
        string Describe { get; }
    }

    /// <summary>The robot a fleet is made of: a light ghost mesh for thousands of instances and the full visuals
    /// for the hero boats, plus how its body frame sits on a pose.</summary>
    public interface ITheaterRobot {
        /// <summary>Few hundred triangles; +z forward, +y up, origin at the pose point; metres.</summary>
        Mesh GhostMesh();
        /// <summary>The robot's full visuals with nothing live (no physics, scripts, sensors or colliders).</summary>
        GameObject CreateHero(string name);
        /// <summary>Rotation of a hero instance for a pose's Unity rotation (body-frame offset).</summary>
        Quaternion HeroRotation(Quaternion poseRotation);
        float Length { get; }
        /// <summary>planar ghosts (z, pitch, roll unrecorded) are seated on the world's surface.</summary>
        string Surface { get; }   // "water", "ground" or "none"
    }

    public static class TheaterPlugins {
        public static readonly Dictionary<string, Func<ITheaterWorld>> Worlds = new Dictionary<string, Func<ITheaterWorld>>();
        public static readonly Dictionary<string, Func<ITheaterRobot>> Robots = new Dictionary<string, Func<ITheaterRobot>>();
    }

    /// <summary>A flat-plane world for tests and for runs with no world plug-in: z up becomes Unity y, x east
    /// is Unity +x, y north is Unity +z.</summary>
    public sealed class PlaneWorld : ITheaterWorld {
        readonly List<Vector3> line = new List<Vector3>();
        Bounds bounds;
        public IEnumerator Prepare(TheaterManifest m) {
            var b = TJson.Arr(m.World, "bounds");
            if (b != null && b.Count == 4) {
                Vector3 a = ToUnity((float)(double)b[0], (float)(double)b[1], 0), c = ToUnity((float)(double)b[2], (float)(double)b[3], 0);
                bounds = new Bounds((a + c) / 2, new Vector3(Mathf.Abs(c.x - a.x), 1f, Mathf.Abs(c.z - a.z)));
            }
            else bounds = new Bounds(Vector3.zero, new Vector3(100, 1, 100));
            foreach (var p in TJson.Arr(m.World, "centreline") ?? new List<object>())
                if (p is List<object> xy && xy.Count >= 2) line.Add(ToUnity((float)(double)xy[0], (float)(double)xy[1], 0));
            yield break;
        }
        public Vector3 ToUnity(float x, float y, float z) => new Vector3(x, z, y);
        public float UnityYaw(float yawDeg) => 90f - yawDeg;
        public float SurfaceY(Vector3 p) => 0f;
        public Bounds Bounds => bounds;
        public IReadOnlyList<Vector3> CentreLine => line;
        public void SetTime(double t) { }
        public string Describe => "plane";
    }

    /// <summary>A procedural robot for tests and unknown plug-ins: a small arrow-shaped hull.</summary>
    public sealed class ArrowRobot : ITheaterRobot {
        public Mesh GhostMesh() => MeshKit.Hull(1.0f, 0.5f, 0.25f, 12, 6);
        public GameObject CreateHero(string name) {
            var go = new GameObject(name);
            go.AddComponent<MeshFilter>().sharedMesh = GhostMesh();
            go.AddComponent<MeshRenderer>();
            return go;
        }
        public Quaternion HeroRotation(Quaternion q) => q;
        public float Length => 1f;
        public string Surface => "ground";
    }

    /// <summary>Procedural meshes.</summary>
    public static class MeshKit {
        /// <summary>A pointed hull (+z bow), <paramref name="ring"/> sides, <paramref name="rings"/> stations.</summary>
        public static Mesh Hull(float length, float beam, float height, int ring, int rings) {
            var v = new List<Vector3>(); var n = new List<Vector3>(); var idx = new List<int>();
            for (int s = 0; s <= rings; s++) {
                float u = s / (float)rings;                       // 0 stern .. 1 bow
                float z = (u - 0.5f) * length;
                float w = beam * 0.5f * Mathf.Sqrt(Mathf.Clamp01(1f - Mathf.Pow(Mathf.Max(0, u - 0.35f) / 0.65f, 2f))) * (u < 0.08f ? 0.85f + 1.9f * u : 1f);
                float h = height * (0.6f + 0.4f * Mathf.Sin(Mathf.PI * Mathf.Min(1, u * 1.2f)));
                for (int k = 0; k < ring; k++) {
                    float a = 2 * Mathf.PI * k / ring;
                    float cx = Mathf.Cos(a), cy = Mathf.Sin(a);
                    var p = new Vector3(cx * w, cy > 0 ? cy * h * 0.6f : cy * h * 0.4f, z);
                    v.Add(p);
                    n.Add(new Vector3(cx, cy * 0.6f, (u - 0.5f) * 0.3f).normalized);
                }
            }
            for (int s = 0; s < rings; s++)
                for (int k = 0; k < ring; k++) {
                    int a = s * ring + k, b = s * ring + (k + 1) % ring, c = a + ring, d = b + ring;
                    idx.AddRange(new[] { a, c, b, b, c, d });
                }
            CapEnd(v, n, idx, 0, ring, false);
            CapEnd(v, n, idx, rings * ring, ring, true);
            var m = new Mesh { name = "theater hull" };
            m.SetVertices(v); m.SetNormals(n); m.SetTriangles(idx, 0);
            m.RecalculateBounds();
            return m;
        }

        static void CapEnd(List<Vector3> v, List<Vector3> n, List<int> idx, int start, int ring, bool bow) {
            var c = Vector3.zero;
            for (int k = 0; k < ring; k++) c += v[start + k];
            c /= ring;
            int ci = v.Count;
            v.Add(c); n.Add(bow ? Vector3.forward : Vector3.back);
            for (int k = 0; k < ring; k++) {
                int a = start + k, b = start + (k + 1) % ring;
                if (bow) idx.AddRange(new[] { a, b, ci }); else idx.AddRange(new[] { b, a, ci });
            }
        }

        /// <summary>A catamaran: two hulls and a deck bridge (about 300 triangles).</summary>
        public static Mesh Catamaran(float length, float beam, float hullBeam, float height) {
            var hull = Hull(length, hullBeam, height, 8, 8);
            var combine = new List<CombineInstance>();
            float off = (beam - hullBeam) * 0.5f;
            combine.Add(new CombineInstance { mesh = hull, transform = Matrix4x4.Translate(new Vector3(-off, 0, 0)) });
            combine.Add(new CombineInstance { mesh = hull, transform = Matrix4x4.Translate(new Vector3(off, 0, 0)) });
            var deck = Box(new Vector3(beam * 0.9f, height * 0.25f, length * 0.55f));
            combine.Add(new CombineInstance { mesh = deck, transform = Matrix4x4.Translate(new Vector3(0, height * 0.55f, -length * 0.05f)) });
            var mast = Box(new Vector3(beam * 0.18f, height * 1.6f, beam * 0.18f));
            combine.Add(new CombineInstance { mesh = mast, transform = Matrix4x4.Translate(new Vector3(0, height * 1.3f, length * 0.05f)) });
            var m = new Mesh { name = "theater catamaran" };
            m.CombineMeshes(combine.ToArray(), true, true);
            m.RecalculateBounds();
            return m;
        }

        public static Mesh Box(Vector3 size) {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            var src = go.GetComponent<MeshFilter>().sharedMesh;
            var m = UnityEngine.Object.Instantiate(src);
            if (Application.isPlaying) UnityEngine.Object.Destroy(go); else UnityEngine.Object.DestroyImmediate(go);
            var v = m.vertices;
            for (int i = 0; i < v.Length; i++) v[i] = Vector3.Scale(v[i], size);
            m.vertices = v;
            m.RecalculateBounds();
            return m;
        }
    }
}
