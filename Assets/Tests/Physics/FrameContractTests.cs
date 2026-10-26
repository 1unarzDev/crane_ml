using System;
using System.Linq;
using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Dynamics;
using Sim.Sensors.Nav;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Tests.Physics {
    public sealed class FrameContractTests {
        [Test]
        public void IdentityFrameMapsUnityToFlu() {
            var go = new GameObject("frame");
            try {
                var f = new BodyFrame(go.transform, Quaternion.identity);
                Assert.That(f.BodyLinear(Vector3.forward), Is.EqualTo(new Vector3(1, 0, 0)));
                Assert.That(f.BodyLinear(Vector3.left), Is.EqualTo(new Vector3(0, 1, 0)));
                Assert.That(f.BodyLinear(Vector3.up), Is.EqualTo(new Vector3(0, 0, 1)));
                // Unity +y rate turns clockwise seen from above: ROS yaw rate is negative.
                Assert.That(f.BodyAngular(Vector3.up), Is.EqualTo(new Vector3(0, 0, -1)));
                var v = new Vector3(0.3f, -1.2f, 2f);
                Assert.That((f.UnityLinear(f.BodyLinear(v)) - v).magnitude, Is.LessThan(1e-6f));
                Assert.That((f.UnityAngular(f.BodyAngular(v)) - v).magnitude, Is.LessThan(1e-6f));
                go.transform.rotation = Quaternion.Euler(0, -30, 0);  // CCW from above
                Assert.That(f.Yaw, Is.EqualTo(30 * Mathf.Deg2Rad).Within(1e-5f), "ROS +yaw is CCW");
            }
            finally {
                UnityEngine.Object.DestroyImmediate(go);
            }
        }

        [Test]
        public void BoatFrameBowIsHullLocalMinusX() {
            var go = new GameObject("hull");
            try {
                go.transform.rotation = Quaternion.Euler(0, 90, 0);  // the scene's base rotation
                var f = new BodyFrame(go.transform, RoboBoatPhysics.BodyFromFlu);
                Vector3 bow = go.transform.TransformDirection(Vector3.left);
                Assert.That((f.BodyLinear(bow) - Vector3.right).magnitude, Is.LessThan(1e-5f));
                Assert.That(RoboBoatPhysics.BodyFromFlu, Is.EqualTo(Quaternion.Euler(0, -90, 0)),
                    "must equal the controllers' RoboBoatRosFrame offset");
            }
            finally {
                UnityEngine.Object.DestroyImmediate(go);
            }
        }

        [Test]
        public void ImuGyroIsTheAxialBodyRate() {
            var gyro = Imu.AngularVelocity(new Vector3(0.1f, 0.2f, 0.3f));
            Assert.That(gyro.x, Is.EqualTo(-0.3).Within(1e-6));
            Assert.That(gyro.y, Is.EqualTo(0.1).Within(1e-6));
            Assert.That(gyro.z, Is.EqualTo(-0.2).Within(1e-6), "Unity +y (clockwise) is ROS −z");
        }

        [Test]
        public void ZeroThrustInACurrentDriftsWithItAtAnyHeading([Values(0.0, 1.0, 2.5)] double heading) {
            MarinePlanarModel m = Kit.Model();
            const double cx = 0.3, cy = -0.2;
            var s = Kit.Run(m, new MarinePlanarModel.State { Psi = heading }, _ => default, 0.02, 120, cx: cx, cy: cy);
            Assert.That(s.Vx, Is.EqualTo(cx).Within(1e-3));
            Assert.That(s.Vy, Is.EqualTo(cy).Within(1e-3));
            Assert.That(s.R, Is.EqualTo(0).Within(1e-6));
        }

        [Test]
        public void SceneThrustersPushAlongTheProfileGeometry() {
            Scene scene = EditorSceneManager.OpenScene("Assets/Scenes/Roboboat Course.unity", OpenSceneMode.Single);
            Thruster[] thrusters = scene.GetRootGameObjects().SelectMany(g => g.GetComponentsInChildren<Thruster>()).ToArray();
            Assert.That(thrusters.Length, Is.EqualTo(4));
            ArticulationBody root = RoboBoatPhysics.Root(thrusters[0]);
            var frame = new BodyFrame(root.transform, RoboBoatPhysics.BodyFromFlu);
            var config = UnityEditor.AssetDatabase.LoadAssetAtPath<ThrusterConfig>("Assets/Config/OmniThrusterConfig.asset");
            Vector3 centre = thrusters.Aggregate(Vector3.zero, (sum, t) => sum + t.transform.position) / 4;
            foreach (ThrusterSpec spec in Kit.Ideal().thrusters) {
                Thruster t = thrusters.Single(x => MarineSurfacePlant.Matches(x, spec.name));
                // The legacy thruster pushes along sign(thrustK)·(rotor axis) for a positive command.
                Vector3 axis = config.rotationAxis == Axis.Y ? Vector3.up : config.rotationAxis == Axis.X ? Vector3.right : Vector3.forward;
                Vector3 push = frame.BodyLinear(Mathf.Sign(config.thrustK) * (t.transform.rotation * axis));
                float angle = Mathf.Atan2(push.y, push.x) * Mathf.Rad2Deg;
                Vector3 at = frame.BodyLinear(t.transform.position - centre);
                UnityEngine.Debug.Log($"PHYS_GEOMETRY {spec.name} scene_alpha={angle:F1} profile_alpha={spec.alphaDeg} " +
                                               $"scene_xy=({at.x:F3},{at.y:F3}) profile_xy=({spec.x},{spec.y})");
                Assert.That(Mathf.Abs(Mathf.DeltaAngle(angle, spec.alphaDeg)), Is.LessThan(5f), spec.name);
                Assert.That(at.x, Is.EqualTo(spec.x).Within(0.03f), spec.name);
                Assert.That(at.y, Is.EqualTo(spec.y).Within(0.03f), spec.name);
            }
        }

        [Test]
        public void UnityArticulationRealizesTheModel() {
            SimulationMode mode = UnityEngine.Physics.simulationMode;
            var go = new GameObject("boat");
            try {
                UnityEngine.Physics.simulationMode = SimulationMode.Script;
                var body = go.AddComponent<ArticulationBody>();
                body.useGravity = false;
                body.linearDamping = body.angularDamping = 0;
                body.mass = 30;
                body.automaticCenterOfMass = false;
                body.centerOfMass = Vector3.zero;
                body.automaticInertiaTensor = false;
                body.inertiaTensor = new Vector3(2.2f, 4.1f, 2.0f);  // Unity y is up
                body.inertiaTensorRotation = Quaternion.identity;
                MarinePlanarModel model = Kit.Ideal().PlanarModel(30, 4.1f);
                var thrust = new Wrench(new Vector3(100, 0, 0), new Vector3(0, 0, 20));
                var plant = go.AddComponent<TestPlant>();
                var frame = new BodyFrame(go.transform, Quaternion.identity);
                plant.Setup(body, frame, new FlowMedium(), new AddedMassCoupling(model),
                    new MarineHydrodynamics(model, default, default, default, Vector3.zero), new ConstantForce { Value = thrust });
                const float dt = 0.02f;
                var s = new MarinePlanarModel.State();
                for (int k = 0; k < 500; k++) {
                    Assert.That(plant.Step(dt, k * dt), Is.True);
                    UnityEngine.Physics.Simulate(dt);
                    model.Step(ref s, new Planar(100, 0, 20), 0, 0, dt);
                }
                Vector3 p = frame.WorldEnu(go.transform.position);
                Vector3 nu = frame.BodyLinear(body.linearVelocity);
                float r = frame.BodyAngular(body.angularVelocity).z;
                UnityEngine.Debug.Log($"PHYS_UNITY_SPOT unity=({p.x:F3},{p.y:F3},{frame.Yaw:F3},{nu.x:F4},{nu.y:F4},{r:F4}) " +
                                               $"model=({s.X:F3},{s.Y:F3},{s.Psi:F3},{s.Body.X:F4},{s.Body.Y:F4},{s.R:F4})");
                Assert.That(nu.x, Is.EqualTo(s.Body.X).Within(0.05 * Math.Abs(s.Body.X)));
                Assert.That(r, Is.EqualTo(s.R).Within(0.05 * Math.Abs(s.R)));
                Assert.That(nu.y, Is.EqualTo(s.Body.Y).Within(0.05 * Math.Abs(s.Body.Y) + 1e-3));
                double dist = Math.Sqrt(s.X * s.X + s.Y * s.Y);
                Assert.That(Math.Sqrt(Math.Pow(p.x - s.X, 2) + Math.Pow(p.y - s.Y, 2)), Is.LessThan(0.05 * dist));
            }
            finally {
                UnityEngine.Physics.simulationMode = mode;
                UnityEngine.Object.DestroyImmediate(go);
            }
        }
    }
}
