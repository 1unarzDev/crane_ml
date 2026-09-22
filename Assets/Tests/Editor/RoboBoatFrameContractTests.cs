using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Controllers;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

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

            Assert.That(linear.x, Is.EqualTo(0.5f).Within(1e-6f),
                "ROS left sway must negate the controller axis that realizes body-local +Z");
            Assert.That(linear.y, Is.EqualTo(0.25f).Within(1e-6f),
                "ROS surge must drive the controller axis that realizes visible-bow local -X");
            Assert.That(linear.z, Is.Zero.Within(1e-6f));
            Assert.That(angular.x, Is.Zero.Within(1e-6f));
            Assert.That(angular.y, Is.Zero.Within(1e-6f));
            Assert.That(angular.z, Is.EqualTo(-0.75f).Within(1e-6f),
                "positive ROS yaw must negate the controller axis measured as negative ROS yaw");
        }

        [Test]
        public void SceneBaseRotationPublishesVisibleBowAsRosForward() {
            Quaternion sceneBaseRotation = Quaternion.Euler(0f, 90f, 0f);
            Quaternion rosFrameRotation = RoboBoatRosFrame.Rotation(sceneBaseRotation);
            Vector3 worldForward = rosFrameRotation * Vector3.forward;

            Assert.That(worldForward.x, Is.Zero.Within(1e-5f));
            Assert.That(worldForward.z, Is.EqualTo(1f).Within(1e-5f));

            Vector3 logicalVelocity = RoboBoatRosFrame.ToRosLocalUnity(Vector3.left);
            var rosVelocity = CraneROSNavigationState.ToRosVector(logicalVelocity);
            Assert.That(rosVelocity.x, Is.EqualTo(1f).Within(1e-5f));
            Assert.That(rosVelocity.y, Is.Zero.Within(1e-5f));
        }

        [Test]
        public void RosSurgeAlignsWithVisibleBowInRoboboatScene() {
            Scene scene = EditorSceneManager.OpenScene(
                "Assets/Scenes/Roboboat Course.unity", OpenSceneMode.Single);
            GameObject root = System.Array.Find(scene.GetRootGameObjects(),
                candidate => candidate.name == "Blastoise");
            Assert.That(root, Is.Not.Null);

            Transform baseLink = root.transform.Find("base_link");
            Transform chaseCamera = baseLink != null ? baseLink.Find("Camera") : null;
            Assert.That(baseLink, Is.Not.Null);
            Assert.That(chaseCamera, Is.Not.Null);

            Vector3 cameraOffset = baseLink.InverseTransformPoint(chaseCamera.position);
            cameraOffset.y = 0f;
            Vector3 visibleBow = -cameraOffset.normalized;

            ROSOmniXCommand.ToControllerMotion(1f, 0f, 0f,
                out Vector3 controllerLinear, out _);
            // OmniX controller X produces base-local +Z; controller Y produces base-local -X.
            Vector3 commandedBaseDirection = new(
                -controllerLinear.y, 0f, controllerLinear.x);

            Assert.That(Vector3.Dot(commandedBaseDirection.normalized, visibleBow),
                Is.GreaterThan(0.999f),
                "positive ROS surge must move from the behind-camera toward the visible bow");
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

        [Test]
        public void DockingEvaluatorRequiresContinuousStoppedContactFreeHullContainment() {
            double qualifiedSince = -1;
            RoboBoatDockingEvaluationMessage first = RoboBoatDockingEvaluator.Evaluate(
                x: 1f, y: -2f, yaw: 0f, surge: 0.01f, sway: 0f, yawRate: 0.01f,
                simulationTime: 10.0, contactCount: 0, ref qualifiedSince,
                targetX: 1f, targetY: -2f, targetYaw: 0f,
                positionTolerance: 0.4f, headingTolerance: 0.35f,
                stoppedSpeed: 0.05f, stoppedYawRate: 0.05f, requiredSettleSeconds: 5f,
                regionWidth: 2f, regionDepth: 3f,
                physicalHullLength: 1.063f, physicalHullBeam: 0.895f);
            Assert.That(first.hullInsideDockRegion, Is.True);
            Assert.That(first.success, Is.False);

            RoboBoatDockingEvaluationMessage settled = RoboBoatDockingEvaluator.Evaluate(
                x: 1f, y: -2f, yaw: 0f, surge: 0.01f, sway: 0f, yawRate: 0.01f,
                simulationTime: 15.1, contactCount: 0, ref qualifiedSince,
                targetX: 1f, targetY: -2f, targetYaw: 0f,
                positionTolerance: 0.4f, headingTolerance: 0.35f,
                stoppedSpeed: 0.05f, stoppedYawRate: 0.05f, requiredSettleSeconds: 5f,
                regionWidth: 2f, regionDepth: 3f,
                physicalHullLength: 1.063f, physicalHullBeam: 0.895f);
            Assert.That(settled.success, Is.True);

            RoboBoatDockingEvaluationMessage contact = RoboBoatDockingEvaluator.Evaluate(
                x: 1f, y: -2f, yaw: 0f, surge: 0f, sway: 0f, yawRate: 0f,
                simulationTime: 15.2, contactCount: 1, ref qualifiedSince,
                targetX: 1f, targetY: -2f, targetYaw: 0f,
                positionTolerance: 0.4f, headingTolerance: 0.35f,
                stoppedSpeed: 0.05f, stoppedYawRate: 0.05f, requiredSettleSeconds: 5f,
                regionWidth: 2f, regionDepth: 3f,
                physicalHullLength: 1.063f, physicalHullBeam: 0.895f);
            Assert.That(contact.success, Is.False);
            Assert.That(contact.continuousQualifiedSeconds, Is.Zero);
        }
    }
}
