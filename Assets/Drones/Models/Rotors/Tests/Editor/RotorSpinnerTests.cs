using System.Globalization;
using System.IO;
using System.Xml.Linq;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

namespace Crane.Drones.Models.Tests
{
    /// <summary>
    /// EditMode checks for the AOS 7 V5 rotors: URDF axes / spin signs / positions (parsed from the URDF text, no Unity importer needed),
    /// the spinner turning transforms about the shaft with the right sign, and (once the prefab exists) the imported rotor links.
    /// Run: unity-drones LOG -runTests -testPlatform EditMode -assemblyNames Crane.Drones.Models.Rotors.Tests
    /// </summary>
    public class RotorSpinnerTests
    {
        const string UrdfPath = "Assets/Robot Descriptions/aos7v5/aos7v5.urdf";
        static readonly float[][] Expected =   // FLU positions of the shafts: FR, RL, FL, RR (PX4 quad-X motors 1..4)
        {
            new[] { +0.127f, -0.092f, 0.040f }, new[] { -0.127f, +0.092f, 0.040f },
            new[] { +0.127f, +0.092f, 0.040f }, new[] { -0.127f, -0.092f, 0.040f },
        };
        static readonly int[] ExpectedSpin = { +1, +1, -1, -1 };

        static float[] V(string s) { var p = s.Split(new[] { ' ' }, System.StringSplitOptions.RemoveEmptyEntries); return System.Array.ConvertAll(p, x => float.Parse(x, CultureInfo.InvariantCulture)); }

        [Test]
        public void UrdfRotorJointsAreContinuousAboutZAtTheShafts()
        {
            Assert.IsTrue(File.Exists(UrdfPath), UrdfPath);
            var doc = XDocument.Load(UrdfPath);
            foreach (var j in doc.Root.Elements("joint"))
            {
                string child = (string)j.Element("child").Attribute("link");
                if (!child.StartsWith("rotor_")) continue;
                int i = int.Parse(child.Substring(6));
                Assert.AreEqual("continuous", (string)j.Attribute("type"), child);
                var axis = V((string)j.Element("axis").Attribute("xyz"));
                CollectionAssert.AreEqual(new[] { 0f, 0f, 1f }, axis, child + " axis");
                Assert.AreEqual("motor_" + i, (string)j.Element("parent").Attribute("link"), child + " parent");
            }
        }

        [Test]
        public void UrdfRotorWorldPositionsMatchQuadXOrder()
        {
            var doc = XDocument.Load(UrdfPath);
            var parent = new System.Collections.Generic.Dictionary<string, (string, float[])>();
            foreach (var j in doc.Root.Elements("joint"))
                parent[(string)j.Element("child").Attribute("link")] = ((string)j.Element("parent").Attribute("link"), V((string)j.Element("origin").Attribute("xyz")));
            for (int i = 0; i < 4; i++)
            {
                var p = new float[3]; string n = "rotor_" + i;
                while (parent.ContainsKey(n)) { var (pl, o) = parent[n]; for (int k = 0; k < 3; k++) p[k] += o[k]; n = pl; }
                Assert.AreEqual("base_link", n);
                for (int k = 0; k < 3; k++) Assert.AreEqual(Expected[i][k], p[k], 1e-4f, $"rotor_{i}[{k}]");
            }
        }

        static RotorSpinner MakeRig(out Transform[] arms)
        {
            var root = new GameObject("rig");
            var sp = root.AddComponent<RotorSpinner>();
            sp.autoAdvance = false;
            sp.rotors = new RotorSpinner.Rotor[4];
            arms = new Transform[4];
            for (int i = 0; i < 4; i++)
            {
                var hub = new GameObject("rotor_" + i).transform;
                hub.SetParent(root.transform, false);
                hub.localPosition = new Vector3(-Expected[i][1], Expected[i][2], Expected[i][0]);   // FLU -> Unity RUF
                hub.localRotation = Quaternion.Euler(0f, 17f * i, 0f);                              // arbitrary rest yaw: spin must compose with it
                var tip = new GameObject("blade_tip").transform;
                tip.SetParent(hub, false);
                tip.localPosition = new Vector3(0.0889f, 0f, 0f);
                arms[i] = tip;
                sp.rotors[i] = new RotorSpinner.Rotor { link = "rotor_" + i, motorIndex = i + 1, spinSign = ExpectedSpin[i], hub = hub };
            }
            sp.Capture();
            return sp;
        }

        [Test]
        public void SpinnerRotatesAboutTheShaftWithPx4Signs()
        {
            var sp = MakeRig(out var tips);
            try
            {
                var hubPos = new Vector3[4];
                for (int i = 0; i < 4; i++) hubPos[i] = sp.rotors[i].hub.position;
                sp.SetRotorSpeeds(new[] { 500f, 500f, 500f, 500f });
                sp.AdvanceRotors(0.01f);                                   // 286.5 deg: the hub must stay put and the tip must stay on its circle
                for (int i = 0; i < 4; i++)
                {
                    Assert.AreEqual(0f, Vector3.Distance(hubPos[i], sp.rotors[i].hub.position), 1e-6f, "hub moved: the pivot is not on the shaft");
                    Assert.AreEqual(0.0889f, Vector3.Distance(sp.rotors[i].hub.position, tips[i].position), 1e-5f, "tip left its circle");
                    Assert.AreEqual(sp.rotors[i].hub.position.y, tips[i].position.y, 1e-5f, "tip left the rotor plane");
                }
                // sign test from a clean rest pose
                for (int i = 0; i < 4; i++) { sp.rotors[i].angleDeg = 0f; sp.rotors[i].hub.localRotation = sp.rotors[i].rest; }
                var before = new Vector3[4];
                for (int i = 0; i < 4; i++) before[i] = tips[i].position - sp.rotors[i].hub.position;
                sp.SetRotorSpeeds(new[] { 10f, 10f, 10f, 10f });
                sp.AdvanceRotors(0.01f);                                   // 0.1 rad = 5.73 deg
                for (int i = 0; i < 4; i++)
                {
                    Vector3 a = before[i], b = tips[i].position - sp.rotors[i].hub.position;
                    float ccwDeg = -Vector3.SignedAngle(a, b, Vector3.up);   // Unity SignedAngle about +Y is clockwise seen from above
                    Assert.AreEqual(ExpectedSpin[i] * 5.7296f, ccwDeg, 0.01f, $"rotor_{i} spin sign/magnitude (CCW positive)");
                }
            }
            finally { Object.DestroyImmediate(sp.gameObject); }
        }

        [Test]
        public void SpinnerKeepsAngleBoundedAndIgnoresNegativeSpeeds()
        {
            var sp = MakeRig(out _);
            try
            {
                sp.SetRotorSpeeds(new[] { -300f, 300f, 300f, 300f });
                for (int k = 0; k < 1000; k++) sp.AdvanceRotors(0.002f);
                foreach (var r in sp.rotors) Assert.That(r.angleDeg, Is.InRange(0f, 360f));
                Assert.AreEqual(sp.rotors[0].spinSign, 1);                  // the sign comes from the joint order, not from the speed sign
                Assert.GreaterOrEqual(sp.rotors[0].speed, 0f);
            }
            finally { Object.DestroyImmediate(sp.gameObject); }
        }

        [Test]
        public void ImportedPrefabKeepsRotorLinksAsSeparateTransforms()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(AOS7V5PrefabAdapter.SourcePrefab);
            if (prefab == null) Assert.Ignore("aos7v5.prefab not synced yet (run Crane.Urdf.UrdfSync.Batch on the URDF)");
            for (int i = 0; i < 4; i++)
            {
                var t = Find(prefab.transform, "rotor_" + i);
                Assert.IsNotNull(t, "rotor_" + i + " link transform missing (the importer must not merge it into the motor)");
                Assert.AreNotEqual(prefab.transform, t);
                Assert.IsNotNull(t.parent, "rotor_" + i + " has no parent");
                Assert.AreEqual("motor_" + i, t.parent.name);
                Assert.Less(Vector3.Angle(t.up, prefab.transform.up), 0.1f, "shaft axis (local up) must be body up");
                Assert.Greater(t.GetComponentsInChildren<MeshFilter>(true).Length, 0, "rotor_" + i + " has no visual mesh of its own");
            }
        }

        static Transform Find(Transform t, string n)
        {
            if (t.name == n) return t;
            for (int i = 0; i < t.childCount; i++) { var r = Find(t.GetChild(i), n); if (r != null) return r; }
            return null;
        }
    }
}
