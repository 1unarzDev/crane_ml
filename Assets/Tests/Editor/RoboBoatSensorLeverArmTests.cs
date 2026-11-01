using NUnit.Framework;
using Sim.Controllers;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatSensorLeverArmTests {
        private GameObject boat;

        [TearDown]
        public void TearDown() {
            if (boat != null) Object.DestroyImmediate(boat);
        }

        [Test]
        public void TheImuPointSeesTheTurnRateTimesItsLeverArm() {
            boat = new GameObject("Boat");
            var body = boat.AddComponent<Rigidbody>();
            body.useGravity = false;
            body.centerOfMass = Vector3.zero;
            var imuLink = new GameObject("imu_link").transform;
            imuLink.SetParent(boat.transform, false);
            imuLink.localPosition = new Vector3(1f, 0f, 0f);
            var imu = boat.AddComponent<CraneRoboBoatSensorContract>();
            imu.Observe(body);
            body.angularVelocity = new Vector3(0f, 2f, 0f);

            Vector3 v = imu.PointVelocity(imuLink.position);
            Assert.That(Vector3.Distance(v, new Vector3(0f, 0f, -2f)), Is.LessThan(1e-4f), v.ToString("F3"));
            // The first sample is gravity; the next differentiates the imu_link point, not the body origin.
            imu.Differentiate();
            Assert.That(imu.SpecificForceWorld, Is.EqualTo(-UnityEngine.Physics.gravity));
        }
    }
}
