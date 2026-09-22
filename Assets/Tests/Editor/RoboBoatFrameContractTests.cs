using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Controllers;
using UnityEditor;
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

        [Test]
        public void OmniThrusterUsesProportionalShaftVelocityCommands() {
            var config = AssetDatabase.LoadAssetAtPath<ThrusterConfig>(
                "Assets/Config/OmniThrusterConfig.asset");

            Assert.That(config, Is.Not.Null);
            Assert.That(config.controlMode, Is.EqualTo(MotorControlMode.Velocity),
                "Normalized RoboBoat inputs must select shaft speed, not saturating motor torque");
            Assert.That(config.GetMaxCommand(), Is.EqualTo(config.maxAngularVelocity),
                "Full manual/Nav2 input must retain the configured shaft-speed top end");
        }

        [Test]
        public void RosVelocityControllerUsesMeasuredBodyVelocity() {
            float accelerating = ROSOmniXCommand.CalculateNormalizedLinearEffort(
                desired: 0.2f, measured: 0.0f, scale: 1.0f,
                feedForward: 0.21f, proportionalGain: 0.1f);
            float steady = ROSOmniXCommand.CalculateNormalizedLinearEffort(
                desired: 0.2f, measured: 0.2f, scale: 1.0f,
                feedForward: 0.21f, proportionalGain: 0.1f);
            float braking = ROSOmniXCommand.CalculateNormalizedLinearEffort(
                desired: 0.0f, measured: 0.2f, scale: 1.0f,
                feedForward: 0.21f, proportionalGain: 0.1f);

            Assert.That(accelerating, Is.GreaterThan(steady));
            Assert.That(steady, Is.GreaterThan(0f));
            Assert.That(braking, Is.LessThan(0f));

            float yawSteady = ROSOmniXCommand.CalculateNormalizedYawEffort(
                desired: 0.2f, measured: 0.2f, scale: 1.0f,
                feedForward: 0.24f, proportionalGain: 0.05f);
            Assert.That(yawSteady, Is.GreaterThan(steady),
                "yaw retains its measured square-root inverse while translation is linear");

            float integral = 0f;
            float previousDesired = 0f;
            for (int tick = 0; tick < 50; tick++) {
                ROSOmniXCommand.UpdateNormalizedLinearIntegral(
                    desired: 0.2f, measured: 0.1f, scale: 1f,
                    integralGain: 0.1f, integralEffortLimit: 0.2f, deltaTime: 0.02f,
                    ref integral, ref previousDesired);
            }
            Assert.That(integral, Is.EqualTo(0.01f).Within(1e-6f));

            ROSOmniXCommand.UpdateNormalizedLinearIntegral(
                desired: 0f, measured: 0.1f, scale: 1f,
                integralGain: 0.1f, integralEffortLimit: 0.2f, deltaTime: 0.02f,
                ref integral, ref previousDesired);
            Assert.That(integral, Is.Zero, "a stop command must discard accumulated effort");
        }
    }
}
