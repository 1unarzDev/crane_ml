using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace Sim.Physics.LandTerrain
{
    // Builds a Unity Terrain + TerrainCollider from a Tools/Terrain `crane-land-terrain-course-v1`
    // manifest and its 16-bit RAW heightmap. Fails closed on a hash, size or schema mismatch.
    // No scene is created: call Build from EditMode tests or an editor menu. Opt-in flag for a later
    // runtime bootstrap would be --crane-land-terrain-course <id> (not implemented here).
    public static class CraneLandTerrainCourse
    {
        public const string SchemaId = "crane-land-terrain-course-v1";

        [Serializable] public class SurfacePatch { public string surface; public float[] xRangeM; public float[] zRangeM; }
        [Serializable] public class GroundTruth { public double measuredMaxSlopeDeg; public double measuredMeanSlopeDeg; }
        [Serializable]
        public class Manifest
        {
            public string schema, generatorVersion, courseId, kind, heightmapSha256, evidenceLabel;
            public int seed, heightmapResolution;
            public float sizeMeters;
            public double[] heightRangeMeters;
            public SurfacePatch[] surfaces;
            public GroundTruth groundTruth;
            public bool ablation;
        }

        public sealed class CourseIntegrityException : Exception
        {
            public CourseIntegrityException(string message) : base(message) { }
        }

        public sealed class BuildResult
        {
            public Manifest manifest;
            public string computedSha256;
            public GameObject root;
            public Terrain terrain;
            public TerrainCollider collider;
            public string[] warnings = Array.Empty<string>();

            // One JSON object suitable for embedding in a run header (provenance of the terrain).
            public string ToRunHeaderJson()
            {
                return "{\"terrainCourseId\":\"" + manifest.courseId + "\",\"schema\":\"" + manifest.schema +
                       "\",\"generatorVersion\":\"" + manifest.generatorVersion + "\",\"seed\":" + manifest.seed +
                       ",\"heightmapSha256\":\"" + computedSha256 + "\",\"hashVerified\":true" +
                       ",\"evidenceLabel\":\"" + manifest.evidenceLabel + "\"}";
            }
        }

        public static string Sha256Hex(byte[] data)
        {
            using (var sha = SHA256.Create())
            {
                byte[] h = sha.ComputeHash(data);
                var sb = new StringBuilder(h.Length * 2);
                foreach (byte b in h) sb.Append(b.ToString("x2"));
                return sb.ToString();
            }
        }

        public static Manifest ParseAndVerify(string manifestJson, byte[] raw16)
        {
            Manifest m = JsonUtility.FromJson<Manifest>(manifestJson);
            if (m == null || m.schema != SchemaId) throw new CourseIntegrityException("manifest schema is not " + SchemaId);
            int r = m.heightmapResolution;
            if (r < 33 || ((r - 1) & (r - 2)) != 0) throw new CourseIntegrityException("heightmapResolution must be 2^n+1, got " + r);
            if (raw16 == null || raw16.Length != r * r * 2)
                throw new CourseIntegrityException("heightmap byte length " + (raw16 == null ? -1 : raw16.Length) + " != " + (r * r * 2));
            string h = Sha256Hex(raw16);
            if (!string.Equals(h, m.heightmapSha256, StringComparison.OrdinalIgnoreCase))
                throw new CourseIntegrityException("heightmap SHA-256 mismatch: manifest " + m.heightmapSha256 + " computed " + h);
            if (m.heightRangeMeters == null || m.heightRangeMeters.Length != 2) throw new CourseIntegrityException("heightRangeMeters missing");
            return m;
        }

        public static BuildResult BuildFromFiles(string manifestPath, string raw16Path, Func<string, PhysicsMaterial> materialForSurface = null)
        {
            return Build(File.ReadAllText(manifestPath), File.ReadAllBytes(raw16Path), materialForSurface);
        }

        // The terrain origin is the (x=0, z=0) corner; its y offset is the manifest minimum height.
        public static BuildResult Build(string manifestJson, byte[] raw16, Func<string, PhysicsMaterial> materialForSurface = null)
        {
            Manifest m = ParseAndVerify(manifestJson, raw16);
            int r = m.heightmapResolution;
            double span = m.heightRangeMeters[1] - m.heightRangeMeters[0];
            var heights = new float[r, r];
            for (int z = 0; z < r; z++)
                for (int x = 0; x < r; x++)
                {
                    int i = (z * r + x) * 2;
                    heights[z, x] = (raw16[i] | (raw16[i + 1] << 8)) / 65535f;
                }
            var data = new TerrainData { heightmapResolution = r };
            data.size = new Vector3(m.sizeMeters, (float)Math.Max(span, 1e-3), m.sizeMeters);
            data.SetHeights(0, 0, heights);
            data.name = "terrain-" + m.courseId;

            var go = new GameObject("TerrainCourse-" + m.courseId);
            go.transform.position = new Vector3(0f, (float)m.heightRangeMeters[0], 0f);
            var terrain = go.AddComponent<Terrain>();
            terrain.terrainData = data;
            var col = go.AddComponent<TerrainCollider>();
            col.terrainData = data;

            var warnings = new System.Collections.Generic.List<string>();
            if (m.surfaces != null && m.surfaces.Length > 0)
            {
                if (materialForSurface != null) col.sharedMaterial = materialForSurface(m.surfaces[0].surface);
                if (m.surfaces.Length > 1)
                    warnings.Add("mixed-surface course: a TerrainCollider holds one material; patch tiles are not implemented, using surface '" + m.surfaces[0].surface + "' everywhere");
            }
            if (m.ablation) warnings.Add("ablation course: PhysX rigid contact cannot represent deformable soil; no evidence claims");
            return new BuildResult { manifest = m, computedSha256 = Sha256Hex(raw16), root = go, terrain = terrain, collider = col, warnings = warnings.ToArray() };
        }

#if UNITY_EDITOR
        [UnityEditor.MenuItem("CRANE/Terrain/Build Course From Files...")]
        static void BuildFromMenu()
        {
            string manifest = UnityEditor.EditorUtility.OpenFilePanel("Terrain course manifest", "", "json");
            if (string.IsNullOrEmpty(manifest)) return;
            string raw = Path.ChangeExtension(manifest, ".r16");
            var result = BuildFromFiles(manifest, raw);
            Debug.Log("CRANE terrain course built: " + result.ToRunHeaderJson());
            foreach (string w in result.warnings) Debug.LogWarning(w);
        }
#endif
    }
}
