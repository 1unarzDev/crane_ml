using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Controllers;
using UnityEditor;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class RoboBoatThrusterPwmTests {
        // mhseals_hardware thruster_mixer.THRUSTER_MIXER: rows FL, FR, RR, RL; columns surge,
        // sway, yaw (positive: forward, port, counterclockwise).
        private static readonly float[,] BoatMixer = {
            { -1f, 1f, 1f }, { -1f, -1f, -1f }, { 1f, -1f, 1f }, { 1f, 1f, -1f }
        };

        private readonly System.Collections.Generic.List<GameObject> created = new();

        [TearDown]
        public void DestroyCreated() {
            foreach (GameObject gameObject in created) Object.DestroyImmediate(gameObject);
            created.Clear();
        }

        private OmniXController Controller() {
            Thruster MakeThruster(string name) {
                var gameObject = new GameObject(name, typeof(Rigidbody));
                created.Add(gameObject);
                return gameObject.AddComponent<Thruster>();
            }
            var root = new GameObject("RoboBoat");
            created.Add(root);
            var controller = root.AddComponent<OmniXController>();
            controller.config = AssetDatabase.LoadAssetAtPath<ThrusterConfig>(
                "Assets/Config/OmniThrusterConfig.asset");
            controller.frontLeft = MakeThruster("FL");
            controller.frontRight = MakeThruster("FR");
            controller.rearRight = MakeThruster("RR");
            controller.rearLeft = MakeThruster("RL");
            return controller;
        }

        private static float[] Commands(OmniXController controller) => new[] {
            controller.frontLeft.GetCommand(), controller.frontRight.GetCommand(),
            controller.rearRight.GetCommand(), controller.rearLeft.GetCommand()
        };

        [Test]
        public void EscPulsesMapAroundNeutral() {
            Assert.That(EscPwm.Bidirectional(1500f, 10f), Is.Zero.Within(1e-6f));
            Assert.That(EscPwm.Bidirectional(1900f, 10f), Is.EqualTo(10f).Within(1e-5f));
            Assert.That(EscPwm.Bidirectional(1100f, 10f), Is.EqualTo(-10f).Within(1e-5f));
            Assert.That(EscPwm.Bidirectional(1700f, 10f), Is.EqualTo(5f).Within(1e-5f));
            Assert.That(EscPwm.Bidirectional(2000f, 10f), Is.EqualTo(10f).Within(1e-5f),
                "pulses beyond the ESC range saturate");
            Assert.That(EscPwm.Bidirectional(1520f, 10f, deadbandPulse: 25f), Is.Zero);
        }

        [Test]
        public void SitlLinearMappingIsUnchanged() {
            // The formula MAVROSConnection.MapPWM used before sharing the model.
            float pwm = 1300f, min = 1000f, max = 2000f, lo = -4f, hi = 6f;
            float previous = lo + (pwm - min) * (hi - lo) / (max - min);
            Assert.That(EscPwm.Linear(pwm, min, max, lo, hi), Is.EqualTo(previous).Within(1e-6f));
            Assert.That(EscPwm.Linear(900f, min, max, 0f, 1f), Is.Zero);
        }

        [Test]
        public void BoatMixerThroughPwmMatchesSimulatorMixer() {
            // The boat's mixer -> ESC pulses -> this input must drive every thruster the way the
            // simulator's own velocity path does, or tuned boat allocation would be tested wrong.
            OmniXController controller = Controller();
            float maximum = controller.config.GetMaxCommand();
            Thruster[] byOutput = ROSThrusterPwmCommand.ThrustersByOutput(
                controller, new[] { 1, 2, 3, 4 });
            float[][] commands = {
                new[] { 0.3f, 0f, 0f }, new[] { 0f, 0.3f, 0f }, new[] { 0f, 0f, 0.3f },
                new[] { 0.2f, -0.1f, 0.15f }
            };
            foreach (float[] command in commands) {
                ROSOmniXCommand.ToControllerMotion(command[0], command[1], command[2],
                    out Vector3 linear, out Vector3 angular);
                controller.SetMotion(linear, angular);
                float[] simulator = Commands(controller);

                var pulses = new ushort[4];
                for (int thruster = 0; thruster < 4; thruster++) {
                    float effort = BoatMixer[thruster, 0] * command[0] +
                                   BoatMixer[thruster, 1] * command[1] +
                                   BoatMixer[thruster, 2] * command[2];
                    pulses[thruster] = (ushort)Mathf.RoundToInt(1500f + 400f * effort);
                }
                ROSThrusterPwmCommand.ApplyPulses(byOutput, pulses, maximum);
                float[] boat = Commands(controller);
                for (int thruster = 0; thruster < 4; thruster++)
                    Assert.That(boat[thruster], Is.EqualTo(simulator[thruster])
                        .Within(maximum / 400f), $"thruster {thruster} for command " +
                        $"({command[0]}, {command[1]}, {command[2]})");
            }
        }

        [Test]
        public void PhysicalOutputsFollowTheConfiguredWiring() {
            OmniXController controller = Controller();
            Thruster[] byOutput = ROSThrusterPwmCommand.ThrustersByOutput(
                controller, new[] { 2, 1, 4, 3 });
            Assert.That(byOutput[0], Is.SameAs(controller.frontRight));
            Assert.That(byOutput[1], Is.SameAs(controller.frontLeft));
            Assert.That(byOutput[2], Is.SameAs(controller.rearLeft));
            Assert.That(byOutput[3], Is.SameAs(controller.rearRight));
            Assert.Throws<System.ArgumentException>(() =>
                ROSThrusterPwmCommand.ThrustersByOutput(controller, new[] { 1, 1, 3, 4 }));
            Assert.Throws<System.ArgumentException>(() =>
                ROSThrusterPwmCommand.ThrustersByOutput(controller, new[] { 1, 2, 3 }));
            Assert.That(ROSThrusterPwmCommandBootstrap.ParseOutputs("2, 1,4,3"),
                Is.EqualTo(new[] { 2, 1, 4, 3 }));
        }

        [Test]
        public void PwmInputNeverRunsBesideAnotherThrusterSource() {
            Assert.That(ROSThrusterPwmCommandBootstrap.Conflict(
                new[] { "--crane-ros-thruster-pwm", "--crane-disable-sitl" }), Is.Null);
            Assert.That(ROSThrusterPwmCommandBootstrap.Conflict(
                new[] { "--crane-ros-thruster-pwm" }), Does.Contain("sitl"));
            Assert.That(ROSThrusterPwmCommandBootstrap.Conflict(new[] {
                "--crane-ros-thruster-pwm", "--crane-disable-sitl", "--crane-ros-cmd-vel"
            }), Does.Contain("cmd-vel"));
        }

        [Test]
        public void ContractAccelerometerReadsGravityUpAtRest() {
            Vector3 force = CraneRoboBoatSensorContract.SpecificForce(
                Vector3.zero, Vector3.zero, 0.02f, new Vector3(0f, -9.81f, 0f));
            var ros = CraneROSNavigationState.ToRosVector(force);
            Assert.That(ros.z, Is.EqualTo(9.81).Within(1e-5));
            Vector3 accelerating = CraneRoboBoatSensorContract.SpecificForce(
                new Vector3(0f, 0f, 0.02f), Vector3.zero, 0.02f, Vector3.zero);
            Assert.That(CraneROSNavigationState.ToRosVector(accelerating).x,
                Is.EqualTo(1.0).Within(1e-5), "Unity +Z is ROS +X");
        }

        [Test]
        public void ContractGpsConvertsLocalMetresToDegrees() {
            CraneRoboBoatSensorContract.EnuToGeodetic(0, 111319.49, 0, 0,
                out double latitude, out double longitude);
            Assert.That(latitude, Is.EqualTo(1.0).Within(1e-6));
            Assert.That(longitude, Is.Zero.Within(1e-12));
            CraneRoboBoatSensorContract.EnuToGeodetic(111319.49 * 0.5, 0, 60, 10,
                out latitude, out longitude);
            Assert.That(latitude, Is.EqualTo(60.0).Within(1e-12));
            Assert.That(longitude, Is.EqualTo(11.0).Within(1e-5),
                "a degree of longitude shrinks with cos(latitude)");
        }

        [Test]
        public void ContractStreamsKeepTheirRates() {
            double next = 0;
            int published = 0;
            for (int tick = 0; tick < 50; tick++)
                if (CraneRoboBoatSensorContract.Due(tick * 0.02, 20f, ref next)) published++;
            Assert.That(published, Is.EqualTo(20), "20 Hz over one second of 50 Hz physics");
        }
    }
}
