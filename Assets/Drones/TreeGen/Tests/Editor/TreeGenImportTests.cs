// SPDX-License-Identifier: GPL-3.0-or-later
using System.Collections.Generic;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEngine;
using Crane.Drones.TreeGenImport.Editor;

namespace Crane.Drones.TreeGenImport.Tests
{
    /// <summary>Orientation and collider checks for the TreeGen importer against a committed TreeGen fixture
    /// (Fixtures~ is ignored by the asset database; regenerate with Tools/TreeGen, see Docs/DroneVisuals/treegen/README.md).</summary>
    public class TreeGenImportTests
    {
        static string Dir => Path.Combine(Application.dataPath, "Drones/TreeGen/Tests/Editor/Fixtures~");
        static TgManifest Man() => TgJson.Parse<TgManifest>(File.ReadAllText(Path.Combine(Dir, "library_manifest.json")));

        static Bounds BarkBounds(string glb)
        {
            var bark = TreeGenGlb.Read(glb).First(p => p.name == "bark");
            var b = new Bounds(bark.pos[0], Vector3.zero); foreach (var v in bark.pos) b.Encapsulate(v); return b;
        }

        [Test]
        public void ImportedBoundsMatchContractBoundsUnityLocal()
        {
            var man = Man(); Assert.That(man.trees.Length, Is.GreaterThan(0));
            foreach (var t in man.trees)
            {
                Vector3 cmin = TgJson.V(t.bounds_unity_local.min), cmax = TgJson.V(t.bounds_unity_local.max);
                var b = BarkBounds(Path.Combine(Dir, t.dir, "lod0.glb"));
                float tol = 0.06f + 0.02f * (cmax - cmin).magnitude; // contract pads node bounds by the max radius
                for (int k = 0; k < 3; k++)
                {
                    Assert.That(b.min[k], Is.EqualTo(cmin[k]).Within(tol), "tree " + t.tree_id + " min axis " + k);
                    Assert.That(b.max[k], Is.EqualTo(cmax[k]).Within(tol), "tree " + t.tree_id + " max axis " + k);
                }
                // the fixture leans: an un-negated (mirrored) import would swap and negate min.x/max.x and fail above
                Assert.That(Mathf.Abs(cmin.x + cmax.x), Is.GreaterThan(2 * tol), "fixture must be asymmetric in x to detect a mirror");
                Assert.That(Mathf.Abs(-b.max.x - cmin.x), Is.GreaterThan(tol), "mirror detection would be vacuous");
            }
        }

        [Test]
        public void WindingStaysOutwardAfterXNegation()
        {
            foreach (var t in Man().trees)
            {
                var bark = TreeGenGlb.Read(Path.Combine(Dir, t.dir, "lod0.glb")).First(p => p.name == "bark");
                int good = 0, n = 0;
                for (int i = 0; i + 2 < bark.tri.Length; i += 3)
                {
                    Vector3 a = bark.pos[bark.tri[i]], b = bark.pos[bark.tri[i + 1]], c = bark.pos[bark.tri[i + 2]];
                    // Unity is left-handed: clockwise front faces, face normal = cross(b-a, c-a)
                    Vector3 fn = Vector3.Cross(b - a, c - a); if (fn.sqrMagnitude < 1e-14f) continue;
                    Vector3 vn = bark.nrm[bark.tri[i]] + bark.nrm[bark.tri[i + 1]] + bark.nrm[bark.tri[i + 2]];
                    n++; if (Vector3.Dot(fn, vn) > 0) good++;
                }
                Assert.That((float)good / n, Is.GreaterThan(0.97f));
            }
        }

        [Test]
        public void MeshHasBarkAndLeafSubmeshes()
        {
            var t = Man().trees[0];
            var m = TreeGenGlb.Load(Path.Combine(Dir, t.dir, "lod0.glb"));
            try
            {
                Assert.That(m.subMeshCount, Is.EqualTo(2));
                Assert.That(m.GetIndexCount(0), Is.GreaterThan(0)); Assert.That(m.GetIndexCount(1), Is.GreaterThan(0));
                var uv = new List<Vector2>(); m.GetUVs(0, uv);
                var leafIdx = m.GetIndices(1);
                // birch leaf cards use atlas tile 2: u in [0,0.5], v in [0.5,1] after the glTF V flip
                foreach (int i in leafIdx.Take(400)) { Assert.That(uv[i].x, Is.InRange(-1e-4f, 0.5001f)); Assert.That(uv[i].y, Is.InRange(0.4999f, 1.0001f)); }
            }
            finally { Object.DestroyImmediate(m); }
        }

        [Test]
        public void CapsulesFollowContractOrientation()
        {
            foreach (var t in Man().trees)
            {
                var col = TgJson.Parse<TgColliders>(File.ReadAllText(Path.Combine(Dir, t.dir, "colliders.json")));
                Assert.That(col.capsules.Length, Is.GreaterThan(0));
                var root = new GameObject("fixture");
                try
                {
                    root.transform.SetPositionAndRotation(new Vector3(3, 1, -2), Quaternion.Euler(0, 37, 0));
                    var cr = TreeGenImporter.AddCapsules(root.transform, col);
                    Assert.That(cr.childCount, Is.EqualTo(col.capsules.Length));
                    for (int i = 0; i < col.capsules.Length; i++)
                    {
                        var c = col.capsules[i]; var cc = cr.GetChild(i).GetComponent<CapsuleCollider>();
                        Assert.That(cc.direction, Is.EqualTo(1)); Assert.That(cc.gameObject.layer, Is.EqualTo(TreeGenImporter.EnvLayer));
                        float half = cc.height * 0.5f - cc.radius;
                        Vector3 p0 = cc.transform.TransformPoint(Vector3.up * half), p1 = cc.transform.TransformPoint(-Vector3.up * half);
                        Vector3 a = root.transform.TransformPoint(c.a.V), b = root.transform.TransformPoint(c.b.V);
                        float d = Mathf.Min((p0 - b).magnitude + (p1 - a).magnitude, (p0 - a).magnitude + (p1 - b).magnitude);
                        Assert.That(d, Is.LessThan(2e-3f), "capsule " + i);
                    }
                }
                finally { Object.DestroyImmediate(root); }
            }
        }

        [Test]
        public void ProviderChoosesSpeciesAndClosestHeight()
        {
            var pool = new List<TreeGenImporter.Entry>();
            float[] h = { 8f, 15f, 21f, 23f };
            for (int i = 0; i < h.Length; i++) pool.Add(new TreeGenImporter.Entry { tree = new TgTreeEntry { tree_id = i, species = "beech", metrics = new TgMetrics { height_m = h[i] } } });
            pool.Add(new TreeGenImporter.Entry { tree = new TgTreeEntry { tree_id = 9, species = "spruce", metrics = new TgMetrics { height_m = 16f } } });
            Assert.That(new[] { 0, 1 }, Does.Contain(TreeGenForestProvider.Choose(pool, "beech", 9f, 0).tree.tree_id));
            Assert.That(TreeGenForestProvider.Choose(pool, "beech", 24f, 0).tree.species, Is.EqualTo("beech"));
            Assert.That(new[] { 2, 3 }, Does.Contain(TreeGenForestProvider.Choose(pool, "beech", 24f, 1).tree.tree_id));
            Assert.That(TreeGenForestProvider.Choose(pool, "spruce", 24f, 5).tree.tree_id, Is.EqualTo(9));
            Assert.That(TreeGenForestProvider.Choose(pool, "birch", 9f, 0), Is.Null);
        }
    }
}
