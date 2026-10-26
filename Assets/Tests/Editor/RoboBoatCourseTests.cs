using NUnit.Framework;
using Sim.Controllers;
using Sim.Sensors.Vision;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatCourseTests {
        // Course spec parsing and banner shapes moved with the builder to Assets/Tests/Course.
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
        public void LidarReturnsOnlyDropsTheMisses() {
            var packed = new byte[36];
            System.Buffer.BlockCopy(new[] { 1f, 2f, 3f, float.NaN, float.NaN, float.NaN, 4f, 5f, 6f },
                0, packed, 0, 36);
            byte[] kept = Sim.Sensors.Lidar.Lidar3D.Returns(packed);
            Assert.That(kept.Length, Is.EqualTo(24));
            Assert.That(System.BitConverter.ToSingle(kept, 12), Is.EqualTo(4f));
        }
    }
}
