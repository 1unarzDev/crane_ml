using NUnit.Framework;
using Sim.Utils.Viewer;
using UnityEngine;

namespace Sim.Tests.Editor {
    public sealed class ViewerCameraTests {
        private sealed class Settings : IViewerFollowSettings {
            public float PositionTime { get; set; } = 0.35f;
            public float YawTime { get; set; } = 0.6f;
            public float TiltTime { get; set; } = 0.4f;
            public float RollFraction { get; set; } = 0.25f;
        }

        /// <summary>Rotation angle [deg] via atan2; Quaternion.Angle's acos loses ~0.01° near zero.</summary>
        private static float Angle(Quaternion q) =>
            2f * Mathf.Atan2(new Vector3(q.x, q.y, q.z).magnitude, Mathf.Abs(q.w)) * Mathf.Rad2Deg;

        private static ViewerContext At(Vector3 position, float yaw = 0f, IViewerAreas areas = null) =>
            new(new ViewerFrame(position, yaw, Quaternion.identity), 60f, 16f / 9f, areas);

        [TestCase(1f / 60f)]
        [TestCase(0.25f)]
        [TestCase(2f)]
        public void DampingConvergesWithoutOvershoot(float dt) {
            const float tau = 0.35f, goal = 10f;
            float value = 0f;
            for (float t = 0f; t < 5f * tau; t += dt) {
                float next = ViewerFollow.Damp(value, goal, tau, dt);
                Assert.That(next, Is.GreaterThanOrEqualTo(value).And.LessThanOrEqualTo(goal), $"t={t}");
                value = next;
            }
            Assert.That(goal - value, Is.LessThan(0.01f * goal), "within 1 % after five time constants");
            Assert.That(ViewerFollow.Damp(3f, goal, 0f, dt), Is.EqualTo(goal), "a zero time constant is rigid");
        }

        [Test]
        public void DampingIsFrameRateIndependent() {
            Vector3 fine = Vector3.zero;
            for (int i = 0; i < 50; i++) fine = ViewerFollow.Damp(fine, new Vector3(4f, -2f, 7f), 0.35f, 0.01f);
            Vector3 coarse = ViewerFollow.Damp(Vector3.zero, new Vector3(4f, -2f, 7f), 0.35f, 0.5f);
            Assert.That(Vector3.Distance(fine, coarse), Is.LessThan(1e-4f));
        }

        [Test]
        public void HeadingDampsAlongTheShortArc() {
            float yaw = 350f;
            for (int i = 0; i < 200; i++) {
                yaw = ViewerFollow.DampAngle(yaw, 10f, 0.6f, 0.02f);
                Assert.That(yaw, Is.InRange(350f, 370f), "never swings the long way through 180°");
            }
            Assert.That(Mathf.DeltaAngle(yaw, 10f), Is.EqualTo(0f).Within(0.1f));
        }

        [Test]
        public void FollowLagsAMovingRobotByABoundAndNeverPassesIt() {
            var settings = new Settings();
            const float speed = 2f, dt = 0.02f;
            Quaternion attitude = Quaternion.identity;
            ViewerFrame frame = ViewerFollow.Snap(Vector3.zero, attitude, settings.RollFraction);
            Vector3 robot = Vector3.zero;
            for (int i = 0; i < 500; i++) {
                robot += Vector3.forward * speed * dt;
                frame = ViewerFollow.Step(frame, robot, attitude, settings, dt);
                Assert.That(frame.Position.z, Is.LessThanOrEqualTo(robot.z + 1e-5f), "never ahead of the robot");
            }
            // First order: steady lag is speed · tau, the bound the shot sits behind.
            Assert.That(robot.z - frame.Position.z, Is.EqualTo(speed * settings.PositionTime).Within(0.02f));
            for (int i = 0; i < 500; i++) {
                frame = ViewerFollow.Step(frame, robot, attitude, settings, dt);
                Assert.That(frame.Position.z, Is.LessThanOrEqualTo(robot.z + 1e-5f), "no overshoot when it stops");
            }
            Assert.That(Vector3.Distance(frame.Position, robot), Is.LessThan(1e-3f));
        }

        [Test]
        public void RollFractionScalesTheInheritedTiltAndKeepsHeading() {
            Quaternion attitude = Quaternion.Euler(0f, 30f, 0f) * Quaternion.Euler(10f, 0f, 20f);
            Quaternion none = ViewerFollow.PartialTilt(attitude, 0f, out float yaw);
            Assert.That(yaw, Is.EqualTo(30f).Within(1e-3f), "roll and pitch do not move the heading");
            Assert.That(Quaternion.Angle(none, Quaternion.identity), Is.LessThan(1e-3f));
            Quaternion full = ViewerFollow.PartialTilt(attitude, 1f, out _);
            Assert.That(Quaternion.Angle(full * Quaternion.Euler(0f, 30f, 0f), attitude), Is.LessThan(1e-2f),
                "fraction 1 is rigidly attached");
            float tilt = Angle(Quaternion.Euler(10f, 0f, 20f));
            Quaternion quarter = ViewerFollow.PartialTilt(attitude, 0.25f, out _);
            Assert.That(Angle(quarter), Is.EqualTo(0.25f * tilt).Within(1e-3f));

            var settings = new Settings();
            ViewerFrame frame = ViewerFollow.Snap(Vector3.zero, Quaternion.Euler(0f, 30f, 0f), settings.RollFraction);
            for (int i = 0; i < 300; i++) frame = ViewerFollow.Step(frame, Vector3.zero, attitude, settings, 0.02f);
            Assert.That(Angle(frame.Tilt), Is.EqualTo(0.25f * tilt).Within(0.01f));
        }

        [Test]
        public void PresetTransitionInterpolatesMonotonicallyAndArrives() {
            var chase = new FollowShot(0f, 14f, 9f, Vector3.up);
            var overhead = new FollowShot(0f, 89f, 18f, Vector3.up);
            ViewerContext context = At(new Vector3(3f, 0f, 4f), 40f);
            ViewerShot shot = new ViewerBlend(chase, overhead, 1.2f);
            ViewerOrbit start = shot.Evaluate(context);
            Assert.That(start.Pitch, Is.EqualTo(14f).Within(1e-4f));
            Assert.That(start.Yaw, Is.EqualTo(40f).Within(1e-4f), "follow yaw is relative to the heading");
            ViewerOrbit previous = start;
            for (int i = 0; i < 70; i++) {
                shot = shot.Advance(0.02f);
                ViewerOrbit now = shot.Evaluate(context);
                Assert.That(now.Pitch, Is.GreaterThanOrEqualTo(previous.Pitch - 1e-4f).And.LessThanOrEqualTo(89f + 1e-4f));
                Assert.That(now.Distance, Is.GreaterThanOrEqualTo(previous.Distance - 1e-4f).And.LessThanOrEqualTo(18f + 1e-4f));
                previous = now;
            }
            Assert.That(shot, Is.SameAs(overhead), "a finished transition collapses to its target");
            Assert.That(shot.Evaluate(context).Pitch, Is.EqualTo(89f));
        }

        [Test]
        public void TransitionYawTakesTheShortArcAndStartsAtRest() {
            var blend = new ViewerBlend(new FollowShot(170f, 10f, 8f, Vector3.zero), new FollowShot(-170f, 10f, 8f, Vector3.zero), 1f);
            ViewerContext context = At(Vector3.zero);
            for (int i = 0; i < 10; i++) {
                blend.Advance(0.1f);
                float yaw = Mathf.Repeat(blend.Evaluate(context).Yaw, 360f);
                Assert.That(yaw, Is.InRange(170f - 1e-3f, 190f + 1e-3f));
            }
            Assert.That(ViewerFollow.Ease(0.01f), Is.LessThan(0.001f), "eased: no velocity step at the start");
            Assert.That(ViewerFollow.Ease(0.5f), Is.EqualTo(0.5f));
        }

        [Test]
        public void ANewTransitionMidWayAndUnwindingOrbitDoNotJump() {
            ViewerContext context = At(Vector3.zero);
            var a = new FollowShot(0f, 14f, 9f, Vector3.up);
            var b = new FollowShot(90f, 8f, 8f, Vector3.up);
            var c = new FollowShot(0f, 89f, 18f, Vector3.up);
            ViewerShot shot = new ViewerBlend(a, b, 1f).Advance(0.4f);
            Pose before = shot.Evaluate(context).ToPose();
            shot = new ViewerBlend(shot, c, 1f);
            Pose after = shot.Evaluate(context).ToPose();
            Assert.That(Vector3.Distance(before.position, after.position), Is.LessThan(1e-4f));
            Assert.That(Quaternion.Angle(before.rotation, after.rotation), Is.LessThan(1e-2f));

            var orbited = new OffsetShot(a, 30f, 20f, 0.5f, 2f, 89f);
            ViewerOrbit held = OffsetShot.Apply(a.Evaluate(context), 30f, 20f, 0.5f, 2f, 89f);
            ViewerOrbit unwinding = new ViewerBlend(orbited, b, 1f).Evaluate(context);
            Assert.That(Vector3.Distance(unwinding.ToPose().position, held.ToPose().position), Is.LessThan(1e-4f));
            Assert.That(held.Distance, Is.EqualTo(9f * Mathf.Exp(0.5f)).Within(1e-4f));
        }

        [Test]
        public void FollowShotRocksAboutTheFocusWithTheFrameTilt() {
            var chase = new FollowShot(0f, 14f, 9f, Vector3.up * 0.5f);
            Quaternion tilt = Quaternion.Euler(0f, 0f, 5f);
            var tilted = new ViewerContext(new ViewerFrame(Vector3.zero, 0f, tilt), 60f, 1.5f, null);
            ViewerOrbit level = chase.Evaluate(At(Vector3.zero));
            ViewerOrbit rocked = chase.Evaluate(tilted);
            Assert.That(Quaternion.Angle(level.Rotation, rocked.Rotation), Is.EqualTo(5f).Within(1e-2f));
            Assert.That(Vector3.Distance(rocked.ToPose().position, rocked.Focus), Is.EqualTo(9f).Within(1e-4f));
        }

        [Test]
        public void AreaShotsFrameTheCourseAndTheNearestTask() {
            var gate = new Bounds(new Vector3(0f, 0f, 20f), new Vector3(8f, 2f, 4f));
            var marina = new Bounds(new Vector3(40f, 0f, 60f), new Vector3(20f, 3f, 12f));
            var areas = new ViewerAreaList(new[] { gate, marina });
            var fallback = new FollowShot(0f, 89f, 20f, Vector3.zero, true);
            var host = new GameObject("viewer test camera");
            try {
                Camera camera = host.AddComponent<Camera>();
                camera.fieldOfView = 60f;
                camera.aspect = 16f / 9f;
                foreach (bool nearest in new[] { false, true }) {
                    var shot = new AreaShot(nearest, 0f, 65f, 1.05f, fallback);
                    ViewerOrbit orbit = shot.Evaluate(new ViewerContext(
                        new ViewerFrame(new Vector3(1f, 0f, 18f), 0f, Quaternion.identity), 60f, 16f / 9f, areas));
                    Bounds expected = gate;
                    if (!nearest) expected.Encapsulate(marina);
                    Assert.That(Vector3.Distance(orbit.Focus, expected.center), Is.LessThan(1e-4f));
                    Pose pose = orbit.ToPose();
                    host.transform.SetPositionAndRotation(pose.position, pose.rotation);
                    for (int i = 0; i < 8; i++) {
                        Vector3 corner = expected.center + Vector3.Scale(expected.extents,
                            new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                        Vector3 viewport = camera.WorldToViewportPoint(corner);
                        Assert.That(viewport.x, Is.InRange(0f, 1f), $"nearest={nearest} corner {i}");
                        Assert.That(viewport.y, Is.InRange(0f, 1f), $"nearest={nearest} corner {i}");
                        Assert.That(viewport.z, Is.GreaterThan(0f));
                    }
                }
                ViewerOrbit none = new AreaShot(false, 0f, 65f, 1f, fallback).Evaluate(At(new Vector3(5f, 0f, 5f), 0f,
                    new ViewerAreaList(new Bounds[0])));
                Assert.That(none.Distance, Is.EqualTo(20f), "no areas: the fallback shot");
            }
            finally {
                Object.DestroyImmediate(host);
            }
        }

        [Test]
        public void EvidenceShotReproducesTheFixedSpectatorPose() {
            // CraneEvidenceCaptureBootstrap used to place the camera at boat + 0.2 up + offset and
            // LookAt boat + 0.2 up + (0, 0.1, 2); its rigid viewer shot must land on the same pose.
            Vector3 boat = new Vector3(12f, 0.1f, -30f), offset = new Vector3(5.5f, 12.8f, -13f);
            Vector3 lookAhead = new Vector3(0f, 0.1f, 2f), raised = boat + Vector3.up * 0.2f;
            FollowShot shot = FollowShot.Looking(Vector3.up * 0.2f + lookAhead, offset - lookAhead, worldYaw: true);
            var settings = new Settings { PositionTime = 0f, YawTime = 0f, TiltTime = 0f, RollFraction = 0f };
            ViewerFrame frame = ViewerFollow.Snap(Vector3.zero, Quaternion.identity, 0f);
            frame = ViewerFollow.Step(frame, boat, Quaternion.Euler(3f, 77f, -4f), settings, 0.02f);
            Pose pose = shot.Evaluate(new ViewerContext(frame, 50f, 1.5f, null)).ToPose();
            Assert.That(Vector3.Distance(pose.position, raised + offset), Is.LessThan(1e-3f));
            Quaternion lookAt = Quaternion.LookRotation(raised + lookAhead - (raised + offset));
            Assert.That(Quaternion.Angle(pose.rotation, lookAt), Is.LessThan(1e-2f));
        }

        [Test]
        public void SavedConfigLoadsAndValidates() {
            var asset = Resources.Load<ViewerCameraConfig>(ViewerCameraConfig.ResourceName);
            Assert.That(asset, Is.Not.Null, "Assets/Config/Viewer/Resources/ViewerCamera.asset");
            Assert.That(asset.Validate(), Is.Empty);
            string[] names = System.Array.ConvertAll(asset.Presets, preset => preset.name);
            Assert.That(names, Is.EqualTo(System.Array.ConvertAll(ViewerCameraConfig.DefaultPresets(), preset => preset.name)));
            Assert.That(asset.RollFraction, Is.InRange(0f, 1f));
            Assert.That(asset.Presets[asset.IndexOf("course")].ToShot(), Is.InstanceOf<AreaShot>());
            Assert.That(asset.Presets[asset.IndexOf("Chase")].ToShot(), Is.InstanceOf<FollowShot>());
            foreach (ViewerPreset preset in asset.Presets)
                Assert.That(UnityEngine.InputSystem.InputControlPath.TryGetDeviceLayout(preset.key), Is.EqualTo("Keyboard"),
                    $"{preset.name} key {preset.key}");
            Assert.That(UnityEngine.InputSystem.InputControlPath.TryGetDeviceLayout(asset.OrbitButton), Is.EqualTo("Mouse"));
            ViewerCameraConfig loaded = ViewerCameraConfig.Load();
            Assert.That(loaded, Is.Not.SameAs(asset), "runtime overrides never touch the asset");
            Object.DestroyImmediate(loaded);
        }
    }
}
