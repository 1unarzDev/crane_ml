using NUnit.Framework;
using Sim.Controllers;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatStartResetTests {
        private GameObject boat;
        private GameObject host;

        [TearDown]
        public void TearDown() {
            if (boat != null) Object.DestroyImmediate(boat);
            if (host != null) Object.DestroyImmediate(host);
        }

        [Test]
        public void ResetPutsADriftedRigidbodyBoatBackOnTheStartAtRest() {
            boat = new GameObject("Boat");
            var body = boat.AddComponent<Rigidbody>();
            body.useGravity = false;
            Vector3 start = new(3f, 0.1f, 7f);
            Quaternion heading = Quaternion.Euler(0f, 30f, 0f);
            boat.transform.SetPositionAndRotation(start, heading);
            host = new GameObject("Course");
            var reset = host.AddComponent<RoboBoatStartReset>();
            reset.Capture(boat.transform, new Vector2(7f, -3f), Mathf.PI / 3);  // its world-ROS pose

            // Drift 99 m with the current and keep moving.
            boat.transform.SetPositionAndRotation(start + new Vector3(70f, 0f, 70f),
                Quaternion.Euler(0f, 110f, 0f));
            body.position = boat.transform.position;
            body.rotation = boat.transform.rotation;
            body.linearVelocity = new Vector3(0.2f, 0f, 0.1f);
            body.angularVelocity = new Vector3(0f, 0.3f, 0f);

            var response = reset.Handle();

            Assert.That(response.success, Is.True, response.message);
            Assert.That(Vector3.Distance(boat.transform.position, start), Is.LessThan(1e-4f));
            Assert.That(Quaternion.Angle(boat.transform.rotation, heading), Is.LessThan(0.01f));
            Assert.That(body.linearVelocity, Is.EqualTo(Vector3.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(Vector3.zero));
            // World-ROS (7, -3), heading 60 deg: the body's 30 deg Unity yaw plus its 90 deg ROS frame.
            Assert.That(response.message, Does.StartWith("x=7 y=-3 yaw="));
            float yaw = float.Parse(response.message.Substring(response.message.IndexOf("yaw=") + 4),
                System.Globalization.CultureInfo.InvariantCulture);
            Assert.That(yaw, Is.EqualTo(Mathf.PI / 3).Within(1e-4f));
        }

        [Test]
        public void ResetUsesTheCourseTruthStartNotWhereTheBoatWasCaptured() {
            // A spawned robot can sit off the course start pose it was laid out from.
            boat = new GameObject("Boat");
            var body = boat.AddComponent<Rigidbody>();
            body.useGravity = false;
            boat.transform.SetPositionAndRotation(new Vector3(1f, 0.2f, 1f), Quaternion.Euler(5f, 80f, 0f));
            body.position = boat.transform.position;
            body.rotation = boat.transform.rotation;
            host = new GameObject("Course");
            var reset = host.AddComponent<RoboBoatStartReset>();
            reset.Capture(boat.transform, new Vector2(10f, 4f), -0.5f);

            var response = reset.Handle();

            Assert.That(response.success, Is.True, response.message);
            Assert.That(boat.transform.position.y, Is.EqualTo(0.2f).Within(1e-5f));
            Vector2 ros = RoboBoatCourse.UnityToRos(boat.transform.position);
            Assert.That(Vector2.Distance(ros, new Vector2(10f, 4f)), Is.LessThan(1e-4f));
            float yaw = float.Parse(response.message.Substring(response.message.IndexOf("yaw=") + 4),
                System.Globalization.CultureInfo.InvariantCulture);
            Assert.That(yaw, Is.EqualTo(-0.5f).Within(1e-4f));
        }

        [Test]
        public void ResetMovesAnArticulatedBoatsTransformsWithItAndRepliesTheTruthStart() {
            // TeleportRoot moves only the physics pose until the next step. The hull's submersion
            // reads the transform, so a stale one left the buoyancy centre at the drifted pose and
            // blew the plant up; the reply, read from it, named the drifted pose (20 m off).
            boat = new GameObject("Boat");
            boat.SetActive(false);
            boat.AddComponent<ArticulationBody>().useGravity = false;
            var hull = new GameObject("Hull");
            hull.transform.SetParent(boat.transform, false);
            hull.transform.localPosition = new Vector3(0f, 0f, 1f);
            var link = hull.AddComponent<ArticulationBody>();
            link.jointType = ArticulationJointType.FixedJoint;
            link.useGravity = false;
            boat.SetActive(true);
            Quaternion heading = Quaternion.Euler(0f, 30f, 0f);
            boat.transform.SetPositionAndRotation(new Vector3(3f, 0.1f, 7f), heading);
            host = new GameObject("Course");
            var reset = host.AddComponent<RoboBoatStartReset>();
            Vector2 truthStart = new(7f, -3f);
            reset.Capture(boat.transform, truthStart, Mathf.PI / 3);
            boat.transform.SetPositionAndRotation(new Vector3(23f, 0.1f, 9f), Quaternion.Euler(0f, 50f, 0f));

            var response = reset.Handle();

            Assert.That(response.success, Is.True, response.message);
            Vector3 start = RoboBoatCourse.RosToUnity(truthStart, 0.1f);
            Assert.That(Vector3.Distance(boat.transform.position, start), Is.LessThan(1e-4f));
            Assert.That(Quaternion.Angle(boat.transform.rotation, heading), Is.LessThan(0.01f));
            Assert.That(Vector3.Distance(hull.transform.position, start + heading * Vector3.forward), Is.LessThan(1e-4f));
            Assert.That(RoboBoatStartResetProbe.ReplyPose(response.message, out Vector2 at, out float yaw), Is.True);
            Assert.That(Vector2.Distance(at, truthStart), Is.LessThan(1e-4f), response.message);
            Assert.That(yaw, Is.EqualTo(Mathf.PI / 3).Within(1e-4f));
        }

        [Test]
        public void ResetWithoutABoatFailsInsteadOfClaimingSuccess() {
            host = new GameObject("Course");
            var reset = host.AddComponent<RoboBoatStartReset>();
            reset.Capture(null, Vector2.zero, 0f);
            var response = reset.Handle();
            Assert.That(response.success, Is.False);
            Assert.That(response.message, Does.Contain("no boat"));
        }

        [Test]
        public void PoseIsReportedInTheOdomFrame() {
            // Unity (x, z) = (-2, 5) is world-ROS (5, 2); the ROS frame is the body turned -90 deg.
            Quaternion bodyFacingRosX = Quaternion.Inverse(Quaternion.Euler(0f, -90f, 0f));
            string pose = RoboBoatStartReset.DescribeRosPose(new Vector3(-2f, 0f, 5f),
                bodyFacingRosX);
            Assert.That(pose, Does.StartWith("x=5 y=2 yaw="));
            float yaw = float.Parse(pose.Substring(pose.IndexOf("yaw=") + 4),
                System.Globalization.CultureInfo.InvariantCulture);
            Assert.That(yaw, Is.EqualTo(0f).Within(1e-5f));
        }
    }
}
