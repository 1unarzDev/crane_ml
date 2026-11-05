using System;
using System.IO;
using System.Linq;
using NUnit.Framework;
using Sim.Course.RoboBoat;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Sim.Course.Tests {
    /// <summary>The generic assemblies compile without Crane.RoboBoat (the compiler only records
    /// references that are used), and the RoboBoat side of the course keeps its tools' spellings.</summary>
    public sealed class RoboBoatBoundaryTests {
        [TestCase("Crane.Course")]
        [TestCase("ControllerAssembly")]
        [TestCase("PhysicsCoreAssembly")]
        [TestCase("PhysicsAssembly")]
        [TestCase("BoatPhysicsAssembly")]
        public void GenericAssemblyDoesNotReferenceRoboBoat(string name) {
            var assembly = AppDomain.CurrentDomain.GetAssemblies().Single(a => a.GetName().Name == name);
            Assert.That(assembly.GetReferencedAssemblies().Select(a => a.Name), Has.None.EqualTo("Crane.RoboBoat"));
        }

        [Test]
        public void RoboBoatHostKeepsItsFlagAliases() {
            ICourseHost host = new RoboBoatCourseHost();
            Assert.That(host.Alias("--crane-course-spec"), Is.EqualTo("--crane-roboboat-course"));
            Assert.That(host.Alias("--crane-course-truth"), Is.EqualTo("--crane-roboboat-course-truth"));
            Assert.That(host.Alias("--crane-course-family"), Is.Null);
        }

        [Test]
        public void FamilyLessSpecWritesTheGenericTruthFlag() {
            EditorSceneManager.OpenScene("Assets/Scenes/Roboboat Course.unity", OpenSceneMode.Single);
            string root = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            string truth = Path.Combine(Path.GetTempPath(), $"crane-course-truth-{Guid.NewGuid():N}.json");
            // Recolouring clones in edit mode logs Unity's material-leak error; the scene is discarded.
            UnityEngine.TestTools.LogAssert.ignoreFailingMessages = true;
            ICourseHost previous = CourseBootstrap.Host;
            CourseBootstrap.Host = new RoboBoatCourseHost();
            try {
                Assert.That(CourseBootstrap.Run(new[] {
                    "--crane-course-spec", Path.Combine(root, "Tools/RoboBoat/courses/roboboat_2025.json"),
                    "--crane-course-truth", truth
                }), Is.True);
                Assert.That(File.Exists(truth), "--crane-course-truth reaches the host's spec builder");
            }
            finally {
                CourseBootstrap.Host = previous;
                File.Delete(truth);
            }
        }
    }
}
