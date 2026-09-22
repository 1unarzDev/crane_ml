using NUnit.Framework;
using Sim.Controllers;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatFrameContractTests {
        [Test]
        public void UnityPositiveYawRateBecomesRosNegativeYawRate() {
            var actual = CraneROSNavigationState.ToRosAngularVector(Vector3.up);

            Assert.That(actual.x, Is.EqualTo(0.0).Within(1e-6));
            Assert.That(actual.y, Is.EqualTo(0.0).Within(1e-6));
            Assert.That(actual.z, Is.EqualTo(-1.0).Within(1e-6));
        }

        [Test]
        public void RosFluCommandMapsToMeasuredControllerAxes() {
            ROSOmniXCommand.ToControllerMotion(0.25f, -0.5f, 0.75f,
                out Vector3 linear, out Vector3 angular);

            Assert.That(linear.x, Is.EqualTo(0.25f).Within(1e-6f),
                "ROS surge must drive the controller axis measured as body surge");
            Assert.That(linear.y, Is.EqualTo(-0.5f).Within(1e-6f),
                "ROS sway must drive the controller axis measured as body sway");
            Assert.That(linear.z, Is.Zero.Within(1e-6f));
            Assert.That(angular.x, Is.Zero.Within(1e-6f));
            Assert.That(angular.y, Is.Zero.Within(1e-6f));
            Assert.That(angular.z, Is.EqualTo(-0.75f).Within(1e-6f),
                "positive ROS yaw must negate the controller axis measured as negative ROS yaw");
        }
    }
}
