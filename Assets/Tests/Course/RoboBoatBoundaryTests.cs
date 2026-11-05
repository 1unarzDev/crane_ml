using System;
using System.Linq;
using NUnit.Framework;
using Sim.Course.RoboBoat;

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
    }
}
