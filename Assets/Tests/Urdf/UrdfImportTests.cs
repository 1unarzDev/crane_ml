using System.IO;
using System.Linq;
using System.Xml.Linq;
using Crane.Urdf;
using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Sensors.Lidar;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.TestTools;

namespace Crane.Urdf.Tests
{
    public class UrdfImportTests
    {
        const string Blastoise = "Assets/Robot Descriptions/blastoise/blastoise.urdf";
        const string Scene = "Assets/Scenes/Roboboat Course.unity";
        // The shipped example is the fixture: synced in place (a re-sync must not change it), restored afterwards.
        const string Minimal = "Assets/Robot Descriptions/minimal_boat/minimal_boat.urdf";
        const string MinimalPrefab = "Assets/Robot Descriptions/minimal_boat/minimal_boat.prefab";
        static readonly string[] Restore = { MinimalPrefab, RobotCatalog.AssetPath };
        byte[][] saved;

        [OneTimeSetUp]
        public void Save() => saved = Restore.Select(File.ReadAllBytes).ToArray();

        static UrdfSync.Report Run(string urdf, bool check)
        {
            var r = UrdfSync.Run(urdf, check);
            r.Log();
            Assert.That(r.errors, Is.Empty, string.Join("\n", r.errors));
            return r;
        }

        [Test]
        public void BlastoiseMatchesTheRealBoatUrdf()
        {
            var r = Run(Blastoise, true);
            Assert.That(r.worstPosM, Is.LessThan(0.001f));
            Assert.That(r.worstAngDeg, Is.LessThan(0.1f));
        }

        [Test]
        public void SceneBoatMatchesTheRealBoatUrdf()
        {
            // The boat as it stands in the course scene: prefab plus the scene's overrides, frames and joint anchors.
            var r = UrdfSync.CheckScene(Blastoise, Scene);
            r.Log();
            Assert.That(r.errors, Is.Empty, string.Join("\n", r.errors));
            Assert.That(r.instancesChecked, Is.GreaterThanOrEqualTo(1), "the course scene holds a Blastoise");
            Assert.That(r.worstPosM, Is.LessThan(UrdfSync.MaxPosErrorM));
            Assert.That(r.worstAngDeg, Is.LessThan(UrdfSync.MaxAngErrorDeg));
        }

        [Test]
        public void SceneAnchorOverrideFailsTheCheck()
        {
            const string path = "Assets/UrdfAnchorOverrideTest.unity";
            Run(Minimal, false);
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            try
            {
                var boat = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(MinimalPrefab), scene);
                var ab = boat.GetComponentsInChildren<ArticulationBody>().Single(a => a.name == "left_thruster_link");
                var so = new SerializedObject(ab);
                so.FindProperty("m_ParentAnchorPosition").vector3Value += new Vector3(0.01f, 0, 0);
                so.ApplyModifiedPropertiesWithoutUndo();
                PrefabUtility.RecordPrefabInstancePropertyModifications(ab);
                EditorSceneManager.SaveScene(scene, path);
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);   // closed: the check opens it itself
                var r = UrdfSync.Run(Minimal, true);
                LogAssert.ignoreFailingMessages = true;   // the check logs the errors this test expects
                r.Log();
                Assert.That(r.errors.Any(e => e.Contains("overrides m_ParentAnchorPosition") && e.Contains("left_thruster_link")), string.Join("\n", r.errors));
                Assert.That(r.errors.Any(e => e.Contains(path) && e.Contains("parent anchor is 10.00 mm")), "the instantiated scene boat fails the anchor check: " + string.Join("\n", r.errors));
            }
            finally
            {
                if (scene.isLoaded) EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                AssetDatabase.DeleteAsset(path);
            }
        }

        [Test]
        public void LostGeometryHashesKeepFileIds()
        {
            // Without stored hashes every link's geometry is re-imported; it must land on the same objects.
            Run(Minimal, false);
            var before = File.ReadAllText(MinimalPrefab);
            var c = PrefabUtility.LoadPrefabContents(MinimalPrefab);
            try { c.GetComponent<RobotDescription>().geometryHashes = new string[0]; PrefabUtility.SaveAsPrefabAsset(c, MinimalPrefab); }
            finally { PrefabUtility.UnloadPrefabContents(c); }
            var r = Run(Minimal, false);
            Assert.That(r.changes.Count(x => x.Contains("re-imported")), Is.GreaterThan(0));
            Assert.That(File.ReadAllText(MinimalPrefab), Is.EqualTo(before), "re-imported geometry keeps every fileID and value");
        }

        [Test]
        public void AgentUrdfImportsWithComponents()
        {
            Run(Minimal, false);
            var bytes = File.ReadAllText(MinimalPrefab);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(MinimalPrefab);
            var thrusters = prefab.GetComponentsInChildren<Thruster>(true);
            CollectionAssert.AreEquivalent(new[] { "left_propeller_link", "right_propeller_link" }, thrusters.Select(t => t.name));
            Assert.That(prefab.GetComponentsInChildren<Lidar3D>(true).Single().name, Is.EqualTo("laser_frame"));
            Assert.That(Run(Minimal, false).changes.Count(c => c.Contains(" gets ")), Is.Zero, "a second sync adds nothing");
            Assert.That(File.ReadAllText(MinimalPrefab), Is.EqualTo(bytes), "a second sync is byte-identical");
            var laser = prefab.GetComponentsInChildren<ArticulationBody>(true).Single(a => a.name == "laser_frame");
            Assert.That(laser.mass, Is.EqualTo(UrdfSync.NoInertialMassKg), "no <inertial>: near-zero mass, not the importer's 1 kg");
            var tensor = new SerializedObject(laser).FindProperty("m_InertiaTensor").vector3Value;   // as saved; the getter needs a live body
            Assert.That((tensor - Vector3.one * UrdfSync.NoInertialTensor).magnitude, Is.LessThan(1e-9f), $"tiny explicit tensor, was {tensor.x:G3}");
        }

        [Test]
        public void AgentBoatDrivesStraightAndTurns()
        {
            // The runtime path, stepped explicitly as Tests/Physics does: the plant the bootstrap applies
            // (a default profile, since Ideal names Blastoise's thrusters) turns thruster commands into thrust.
            // No player loop: its FixedUpdate count per manual Simulate follows wall-clock frame time.
            Run(Minimal, false);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(MinimalPrefab);
            Assert.That(prefab.GetComponent<RobotDescription>().rootYawDeg, Is.Zero, "a REP-103 URDF: bow = hull-local +z");
            // Alone in an empty scene: a scene an earlier test left open (the course) would be stepped with it.
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var mode = UnityEngine.Physics.simulationMode;
            var boat = Object.Instantiate(prefab);
            try
            {
                UnityEngine.Physics.simulationMode = SimulationMode.Script;
                foreach (var ab in boat.GetComponentsInChildren<ArticulationBody>()) ab.useGravity = false;   // no water here
                var root = boat.GetComponentInChildren<ArticulationBody>();
                var ideal = Resources.Load<MarineSurfaceProfile>("Ideal");
                Assert.That(RoboBoatPhysics.Fits(ideal.Parameters, boat.GetComponentsInChildren<Thruster>()), Is.False);
                var profile = RoboBoatPhysics.DefaultFor(root, ideal);
                Assert.That(profile.Provenance.kind, Is.EqualTo("default"));
                // No water here to right the hull: thrust 0.15 m below the CG would pitch the bow through vertical, where the
                // heading is undefined. Thrust at CG height keeps the test on what it checks (push direction, differential turn).
                foreach (var spec in profile.Parameters.thrusters) spec.z = 0f;
                float urdfMass = XDocument.Load(Minimal).Root.Elements("link").Select(l => l.Element("inertial")?.Element("mass")?.Attribute("value"))
                    .Where(v => v != null).Sum(v => float.Parse(v.Value, System.Globalization.CultureInfo.InvariantCulture));
                Assert.That(urdfMass, Is.EqualTo(20.84f).Within(1e-4f), "the fixture's own inertials");
                Assert.That(profile.Parameters.mass, Is.EqualTo(urdfMass).Within(0.01f), "mass from the URDF inertials; a link without <inertial> adds ~nothing");
                var plant = RoboBoatPhysics.Apply(root, profile, null);
                Assert.That(plant, Is.Not.Null);
                var left = boat.GetComponentsInChildren<Thruster>().Single(t => t.name.StartsWith("left"));
                var right = boat.GetComponentsInChildren<Thruster>().Single(t => t.name.StartsWith("right"));

                float dt = Time.fixedDeltaTime; double time = 0;
                void Drive(float l, float r) { for (int i = 0; i < 150; i++) { left.SetCommand(l); right.SetCommand(r); plant.Step(dt, time); UnityEngine.Physics.Simulate(dt); time += dt; } }
                Quaternion bowFrame = root.transform.rotation * RoboBoatPhysics.BodyFromFluOf(root);   // +z = bow at the start
                float Heading() { var b = root.transform.rotation * RoboBoatPhysics.BodyFromFluOf(root) * Vector3.forward; return Mathf.Atan2(b.x, b.z) * Mathf.Rad2Deg; }

                Vector3 p0 = root.transform.position; float yaw0 = Heading();
                float cmd = 0.6f * left.GetMaxCommand();
                Drive(cmd, cmd);
                Vector3 d = Quaternion.Inverse(bowFrame) * (root.transform.position - p0);   // z = forward, x = starboard, y = up
                float yawStraight = Mathf.Abs(Mathf.DeltaAngle(yaw0, Heading()));
                Debug.Log($"URDF_DRIVE straight d=({d.x:F3},{d.y:F3},{d.z:F3}) yaw={yawStraight:F2}");

                float yaw1 = Heading();
                Drive(-cmd, cmd);
                float turned = Mathf.Abs(Mathf.DeltaAngle(yaw1, Heading()));
                Debug.Log($"URDF_DRIVE turn yaw={turned:F2}");

                Assert.That(Mathf.Abs(d.z), Is.GreaterThan(0.3f), "equal thrust moves the boat along its URDF x axis");
                Assert.That(Mathf.Abs(d.x), Is.LessThan(0.25f * Mathf.Abs(d.z)), "and not sideways");
                Assert.That(yawStraight, Is.LessThan(5f));
                Assert.That(turned, Is.GreaterThan(5f), "opposite thrust turns the boat");
            }
            finally
            {
                UnityEngine.Physics.simulationMode = mode;
                Object.DestroyImmediate(boat);
            }
        }

        [OneTimeTearDown]
        public void Cleanup()
        {
            for (int i = 0; i < Restore.Length; i++)
                if (!File.ReadAllBytes(Restore[i]).SequenceEqual(saved[i])) { File.WriteAllBytes(Restore[i], saved[i]); AssetDatabase.ImportAsset(Restore[i]); }
        }
    }
}
