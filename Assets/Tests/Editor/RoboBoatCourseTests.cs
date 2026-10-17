using NUnit.Framework;
using Sim.Controllers;
using Sim.Sensors.Vision;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatCourseTests {
        [Test]
        public void CourseSpecParsesElementsAndDockBays() {
            var spec = RoboBoatCourse.Parse(@"{
                ""name"": ""t"", ""hide"": [""Marina""], ""elements"": [
                  {""template"": ""RedBuoy0"", ""label"": ""red_buoy"", ""x"": 4, ""y"": 1},
                  {""template"": ""Dock0"", ""yaw"": 3.14, ""bays"": [""red_circle"", ""blue_plus""],
                   ""occupied"": [1]}]}");

            Assert.That(spec.elements.Length, Is.EqualTo(2));
            Assert.That(spec.elements[0].scale, Is.EqualTo(1f), "scale defaults to 1");
            Assert.That(spec.elements[1].bays, Is.EqualTo(new[] { "red_circle", "blue_plus" }));
            Assert.That(spec.elements[1].occupied, Is.EqualTo(new[] { 1 }));
            Assert.Throws<System.ArgumentException>(() =>
                RoboBoatCourse.Parse(@"{""elements"": [{""label"": ""x""}]}"));
        }

        [Test]
        public void CourseFrameIsTheStartPose() {
            // Facing +y in the world, 2 m ahead and 1 m to port is (-1, 2) from the start.
            Vector2 world = RoboBoatCourse.ToWorld(new Vector2(10f, 5f), Mathf.PI / 2, 2f, 1f);
            Assert.That(world.x, Is.EqualTo(9f).Within(1e-5f));
            Assert.That(world.y, Is.EqualTo(7f).Within(1e-5f));
            Vector3 unity = RoboBoatCourse.RosToUnity(world, 0.1f);
            Assert.That(RoboBoatCourse.UnityToRos(unity), Is.EqualTo(world));
            Assert.That(RoboBoatCourse.RosYawOfUnityDirection(Vector3.forward),
                Is.EqualTo(0f).Within(1e-6f), "Unity +z is world-ROS +x");
            Assert.That(RoboBoatCourse.RosYawOfUnityDirection(Vector3.left),
                Is.EqualTo(Mathf.PI / 2).Within(1e-6f), "Unity -x is world-ROS +y");
        }

        [Test]
        public void CurrentFlagParsesAndTurnsIntoHdrpOrientation() {
            Assert.That(RoboBoatCourse.ParseCurrent("0.15,45", out float speed, out float toward));
            Assert.That(speed, Is.EqualTo(0.15f).Within(1e-6f));
            Assert.That(toward, Is.EqualTo(45f));
            Assert.That(RoboBoatCourse.ParseCurrent("-1,0", out _, out _), Is.False);
            Assert.That(RoboBoatCourse.ParseCurrent("0.1", out _, out _), Is.False);
            // HDRP's direction is (cos, sin) over Unity (x, z): orientation 90 is Unity +z,
            // which is world-ROS +x, i.e. a current toward 0 degrees.
            Assert.That(RoboBoatCourse.HdrpOrientation(0f), Is.EqualTo(90f));
        }

        [Test]
        public void BannerShapesAreDrawnWhereExpected() {
            Assert.That(RoboBoatCourse.InShape("circle", 0.5f, 0.5f));
            Assert.That(RoboBoatCourse.InShape("circle", 0.95f, 0.95f), Is.False);
            Assert.That(RoboBoatCourse.InShape("plus", 0.5f, 0.8f));
            Assert.That(RoboBoatCourse.InShape("plus", 0.8f, 0.8f), Is.False);
            Assert.That(RoboBoatCourse.InShape("triangle", 0.5f, 0.75f));
            Assert.That(RoboBoatCourse.InShape("triangle", 0.2f, 0.75f), Is.False);
            Assert.That(RoboBoatCourse.BannerColor("blue_square"), Is.EqualTo(new Color(0.1f, 0.3f, 0.9f)));
        }

        [Test]
        public void DetectionsAreRep103InTheCameraFrame() {
            // Ahead of the camera (Unity +z) is +x, its left (Unity -x) is +y, up stays up.
            Assert.That(BoundingBox3D.UnityToROSPosition(new Vector3(0f, 0f, 5f)),
                Is.EqualTo(new Vector3(5f, 0f, 0f)));
            Assert.That(BoundingBox3D.UnityToROSPosition(new Vector3(-2f, 1f, 0f)),
                Is.EqualTo(new Vector3(0f, 2f, 1f)));
            // An object turned 90 degrees to the camera's left (Unity -90 about y) is +90 about z.
            Quaternion ros = BoundingBox3D.UnityToROSRotation(Quaternion.Euler(0f, -90f, 0f));
            Vector3 forward = ros * Vector3.right;
            Assert.That(forward.x, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(forward.y, Is.EqualTo(1f).Within(1e-5f));
        }

        [Test]
        public void PhysicsScalesTheLinksToTheLoadedMass() {
            float[] masses = RoboBoatPhysics.ScaledMasses(new[] { 3f, 8f, 8f, 1f }, 30f);
            Assert.That(masses[0] + masses[1] + masses[2] + masses[3], Is.EqualTo(30f).Within(1e-4f));
            Assert.That(masses[1] / masses[0], Is.EqualTo(8f / 3f).Within(1e-4f));
        }

        [Test]
        public void ModerateWindPushesAbeamHarderThanAhead() {
            // 8 m/s across the boat: 1/2 * 1.225 * 1.0 * 0.40 * 64 = 15.7 N, to port.
            Vector2 beam = RoboBoatWind.Force(new Vector2(0f, 8f), 0f);
            Assert.That(beam.x, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(beam.y, Is.EqualTo(15.68f).Within(0.01f));
            Vector2 ahead = RoboBoatWind.Force(new Vector2(-8f, 0f), 0f);
            Assert.That(ahead.x, Is.EqualTo(-11.76f).Within(0.01f));
            // The same wind seen from a boat turned 90 degrees is now on its bow.
            Assert.That(RoboBoatWind.Force(new Vector2(0f, 8f), Mathf.PI / 2).y,
                Is.EqualTo(11.76f).Within(0.01f));
        }

        [Test]
        public void HullReachesTheMeasuredTopSpeedOnFourT200() {
            // Four T200 at 45 degrees give 146 N along the bow at full surge.
            float thrust = 4f * RoboBoatPhysics.T200Forward * Mathf.Sqrt(0.5f);
            Assert.That(RoboBoatHull.TopSpeed(thrust, RoboBoatHull.Surge),
                Is.EqualTo(1.8f).Within(0.05f));
            Assert.That(RoboBoatHull.Drag(RoboBoatHull.TopSpeed(thrust, RoboBoatHull.Surge),
                RoboBoatHull.Surge), Is.EqualTo(-thrust).Within(0.01f));
            Assert.That(RoboBoatHull.TopSpeed(thrust, RoboBoatHull.Sway),
                Is.LessThan(RoboBoatHull.TopSpeed(thrust, RoboBoatHull.Surge)),
                "a catamaran is draggier abeam");
        }
    }
}
