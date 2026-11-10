using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>Evaluator truth of one rover (never shown to the rover's own estimators).</summary>
    public struct RoverTruth {
        public double X, Y, Z, Yaw, Pitch, Roll;
        public Twist Body;
        public double Speed;
    }

    /// <summary>
    /// One rover in Unity: a single Rigidbody built from its <see cref="RoverDescription"/> (render, collision,
    /// mass, frames each from their own field) plus the thin adapter that feeds the Core plant with the measured
    /// twist and applies its acceleration. Sensors (encoders, gyro) and the odometry estimate live here too,
    /// keyed by the rover's own id; nothing is global.
    /// </summary>
    public sealed class RoverRig {
        public readonly RoverDescription D;
        public readonly GameObject Root;
        public readonly Rigidbody Body;
        public readonly MecanumPlant Plant;
        public readonly WheelEncoders Encoders;
        public readonly YawGyro Gyro;
        public WheelOdometry Odometry;
        public QualificationScript Script;
        public Twist Command;
        public readonly Dictionary<string, Transform> Frames = new Dictionary<string, Transform>();
        public readonly List<SphereCollider> WheelColliders = new List<SphereCollider>();
        public readonly string RenderSource;
        public readonly AssetVisual Visual;   // null for primitives
        public double Lift, GripperOpening = double.NaN;
        public bool CameraLost, EncodersLost;
        public IRoverSampleSink SampleSink;
        Vector3 prevVel; bool havePrevVel;
        readonly long[] prevTicks = new long[4];
        FleetRng accelRng; double accelNoise;   // the autonomy source receives the rover's own samples as they are produced   // injected sensor failures (rover side sees no tags / frozen encoders)   // lifter state (m), render and payload carry

        readonly int imuEvery, encEvery;
        readonly double dt;

        public RoverRig(FleetSpec spec, RoverDescription d, FloorProperties floor, long seed, Transform parent, Pose2 pose) {
            D = d;
            dt = spec.Dt;
            JsonNode o = spec.O, p = spec.P;
            Root = new GameObject(d.Id);
            Root.transform.SetParent(parent, false);
            double lift = o["physics.spawn_lift"].Double;
            Root.transform.SetPositionAndRotation(FleetFrames.World(pose.X, pose.Y, d.OriginHeight + lift), FleetFrames.WorldYaw(pose.Yaw));

            // mass properties
            Body = Root.AddComponent<Rigidbody>();
            Body.mass = (float)d.Mass;
            Body.automaticCenterOfMass = false;
            Body.automaticInertiaTensor = false;
            Body.centerOfMass = FleetFrames.Local(d.ComOffset);
            // FLU diag (Ixx, Iyy, Izz) -> Unity local axes (x = -y_flu, y = z_flu, z = x_flu)
            Body.inertiaTensor = new Vector3((float)d.InertiaDiag[1], (float)d.InertiaDiag[2], (float)d.InertiaDiag[0]);
            Body.inertiaTensorRotation = Quaternion.identity;
            Body.linearDamping = 0f;
            Body.angularDamping = 0f;
            Body.interpolation = RigidbodyInterpolation.None;
            Body.collisionDetectionMode = CollisionDetectionMode.Discrete;
            Body.solverIterations = (int)o["physics.solver_iterations"].Double;
            Body.solverVelocityIterations = (int)o["physics.solver_velocity_iterations"].Double;

            // collision geometry: chassis envelope and wheel contact spheres (separate from render)
            var collision = new GameObject("collision").transform;
            collision.SetParent(Root.transform, false);
            double bf = o["physics.body_friction"].Double, wf = o["physics.wheel_friction"].Double;
            var bodyMat = new PhysicsMaterial("fleet body") { dynamicFriction = (float)bf, staticFriction = (float)bf };
            var wheelMat = new PhysicsMaterial("fleet wheel") { dynamicFriction = (float)wf, staticFriction = (float)wf,
                frictionCombine = PhysicsMaterialCombine.Minimum, bounceCombine = PhysicsMaterialCombine.Minimum };
            foreach (var part in d.CollisionParts) {
                var go = new GameObject("collision_" + part.Name);
                go.transform.SetParent(collision, false);
                go.transform.localPosition = FleetFrames.Local(part.Centre);
                go.transform.localRotation = FleetFrames.LocalYawPitch(part.Yaw, 0);
                var bc = go.AddComponent<BoxCollider>();
                bc.size = FleetFrames.LocalScale(part.Size[0], part.Size[1], part.Size[2]);
                bc.sharedMaterial = bodyMat;
                bc.providesContacts = true;
            }
            if (d.WheelContact != "sphere") throw new NotSupportedException("wheel contact " + d.WheelContact);
            foreach (var w in d.Wheels) {
                var go = new GameObject("contact_" + w.Name);
                go.transform.SetParent(collision, false);
                go.transform.localPosition = FleetFrames.LocalPosition(w.Axle);
                var sc = go.AddComponent<SphereCollider>();
                sc.radius = (float)w.Radius;
                sc.sharedMaterial = wheelMat;
                WheelColliders.Add(sc);
            }

            // render geometry: the asset manifest when present, else primitives
            if (d.MeshManifestPresent) {
                Visual = new AssetVisual(d, Root.transform, spec);
                RenderSource = d.MeshManifestPath;
            }
            else {
                var render = new GameObject("render").transform;
                render.SetParent(Root.transform, false);
                foreach (var part in d.RenderPrimitives) FleetGeometry.Part(part, render);
                RenderSource = "primitives";
            }

            // frames: wheels, docking faces (attachment points), sensors, CoM
            var frames = new GameObject("frames").transform;
            frames.SetParent(Root.transform, false);
            foreach (var w in d.Wheels) AddFrame(frames, "wheel_" + w.Name, w.Axle);
            foreach (var f in d.Faces) AddFrame(frames, "face_" + f.Name, f.Pose);
            foreach (var s in d.Sensors) AddFrame(frames, "sensor_" + s.Id, s.Pose);
            AddFrame(frames, "com", new FramePose(d.ComOffset[0], d.ComOffset[1], d.ComOffset[2]));

            // plant and the rover's own sensors (independent seeded streams per rover)
            Plant = new MecanumPlant(d.Geometry, PlantParams.From(spec, d, floor), FleetRng.For(seed, d.Id + "/slip"));
            Encoders = new WheelEncoders(d.Geometry.CountsPerRev);
            Gyro = new YawGyro(FleetRng.For(seed, d.Id + "/imu"), p["sensors.imu.gyro_noise_density"].Double,
                p["sensors.imu.gyro_bias_instability"].Double, o["plant.imu_initial_bias_sigma"].Double);
            accelRng = FleetRng.For(seed, d.Id + "/accel");
            accelNoise = o["plant.imu_accel_noise_density"].Double;
            imuEvery = Math.Max(1, (int)Math.Round(1.0 / (p["sensors.imu.rate_hz"].Double * dt)));
            encEvery = Math.Max(1, (int)Math.Round(1.0 / (p["sensors.wheel_encoder.rate_hz"].Double * dt)));
        }

        void AddFrame(Transform parent, string name, FramePose pose) {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.localPosition = FleetFrames.LocalPosition(pose);
            go.transform.localRotation = FleetFrames.Local(pose);
            Frames[name] = go.transform;
        }

        public RoverTruth Truth() {
            var t = new RoverTruth();
            FleetFrames.ToEnu(Body.position, out t.X, out t.Y, out t.Z);
            FleetFrames.Attitude(Body.rotation, out t.Yaw, out t.Pitch, out t.Roll);
            t.Body = FleetFrames.BodyTwist(Body);
            Vector3 v = Body.linearVelocity;
            t.Speed = Math.Sqrt(v.x * v.x + v.z * v.z);
            return t;
        }

        /// <summary>Plant step with the current command; applies the planar force and yaw torque at the CoM.</summary>
        public void ApplyPlant() {
            Twist measured = FleetFrames.BodyTwist(Body);
            Plant.Step(Command, measured, dt);
            float m = Body.mass;
            Body.AddRelativeForce(new Vector3((float)(-Plant.Ay * m), 0f, (float)(Plant.Ax * m)), ForceMode.Force);
            Body.AddRelativeTorque(new Vector3(0f, (float)(-Plant.Alpha * Plant.P.Izz), 0f), ForceMode.Force);
        }

        /// <summary>Sensors after the physics step <paramref name="step"/> (0-based count of steps since t = 0).</summary>
        public void Sense(long step) {
            if (step % imuEvery == 0) {
                double rate = Gyro.Sample(Truth().Body.Wz, imuEvery * dt);
                Odometry?.IntegrateGyro(rate, imuEvery * dt);
                SampleSink?.OnGyro(step * dt, rate);
                if (SampleSink != null) {
                    // body-frame linear acceleration from the rigid body (gravity removed) + white noise (overlay provisional density)
                    Vector3 v = Body.linearVelocity;
                    if (havePrevVel) {
                        Vector3 a = Quaternion.Inverse(Body.rotation) * ((v - prevVel) / (float)(imuEvery * dt));
                        double sd = accelNoise / Math.Sqrt(imuEvery * dt);
                        SampleSink.OnAccel(step * dt, a.z + sd * accelRng.Gaussian(), -a.x + sd * accelRng.Gaussian());
                    }
                    prevVel = v; havePrevVel = true;
                }
            }
            if (step % encEvery == 0 && !EncodersLost) {
                Encoders.Sample(Plant.WheelAngle);
                Odometry?.IntegrateEncoders(encEvery * dt);
                SampleSink?.OnEncoder(step * dt, (long[])Encoders.Ticks.Clone());
                if (SampleSink != null) {
                    var meas = new double[4];
                    for (int i = 0; i < 4; i++) { meas[i] = Encoders.TicksToRad(Encoders.Ticks[i] - prevTicks[i]) / (encEvery * dt); prevTicks[i] = Encoders.Ticks[i]; }
                    SampleSink.OnWheelRates(step * dt, (double[])Plant.WheelCommand.Clone(), meas);
                }
            }
        }

        /// <summary>Drive the render-only movable parts from state (wheels from the plant's wheel angles; lift; gripper).</summary>
        public void UpdateVisual() {
            if (Visual == null) return;
            Visual.SetWheels(Plant.WheelAngle);
            Visual.SetLift(Lift);
            if (!double.IsNaN(GripperOpening)) Visual.SetGripper(GripperOpening);
        }

        public Transform RenderRoot => Visual != null ? Visual.Root : Root.transform.Find("render");

        public void StartOdometry() {
            RoverTruth t = Truth();
            Encoders.Sample(Plant.WheelAngle);
            Odometry = new WheelOdometry(D.Geometry, Encoders, t.X, t.Y, t.Yaw);
            Odometry.IntegrateEncoders(dt);  // absorb ticks accumulated during settling
            Odometry.X = t.X; Odometry.Y = t.Y;
        }
    }
}
