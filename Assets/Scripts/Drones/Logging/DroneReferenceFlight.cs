using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Crane.Drones.Logging {
    /// <summary>Deterministic xorshift64* PRNG (no System.Random / UnityEngine.Random dependence).</summary>
    public sealed class DroneRng {
        private ulong s;
        public DroneRng(ulong seed) { s = seed * 0x9E3779B97F4A7C15UL + 0xD1B54A32D192ED03UL; if (s == 0) s = 1; Next(); Next(); }
        public ulong Next() { s ^= s >> 12; s ^= s << 25; s ^= s >> 27; return s * 0x2545F4914F6CDD1DUL; }
        public double Uniform() => (Next() >> 11) * (1.0 / (1UL << 53));
        public double Normal() {
            double u1 = Math.Max(Uniform(), 1e-300), u2 = Uniform();
            return Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
        }
    }

    /// <summary>
    /// STUB plant + scene: a PhysX rigid body with seeded thrust/torque/wind disturbances, one box
    /// obstacle, one detectable target, synthetic RGB/depth/LiDAR. It exists to prove the logging and
    /// reproducibility machinery before the aerial physics module lands; the physics module replaces
    /// the plant by implementing <see cref="IDroneStateSource"/>. Physics runs in Script mode at 1 kHz.
    /// </summary>
    public sealed class DroneReferenceFlight : IDroneStateSource, IDroneImuSource {
        public const double Mass = 1.2, Radius = 0.15, Hover = 1.5;
        public int RotorCount => 4;
        private readonly int seed;
        private readonly DroneRng rng;
        private GameObject root, droneGo, box, ground, targetGo, camGo;
        private Rigidbody rb;
        private BoxCollider boxCol;
        private Vector3 lastVel;
        private double windE, windN;
        private double cmdNoise, torqueX, torqueY, torqueZ, battSoc = 1;
        private double lastThrust;
        private readonly List<(long t, string other, double speed, double[] n)> contacts = new();
        private double clearance;

        public DroneReferenceFlight(int seed) {
            this.seed = seed;
            rng = new DroneRng((ulong)seed);
        }

        public static DroneRunManifest MakeManifest(string runId, int seed) {
            var m = new DroneRunManifest {
                RunId = runId, SceneSeed = seed, Scene = "stub:DroneReferenceFlight",
                PhysicsProfile = "stub-rigidbody-1kHz", PhysicsParamsSha256 = Sha("mass=1.2;radius=0.15;dt=0.001"),
                Controller = "stub-altitude-hold+seeded-disturbance",
                UnityVersion = Application.unityVersion, WallClockStartUtc = DateTime.UtcNow.ToString("o"),
                FixedDtNs = 5_000_000, Substeps = 5, StateEverySubsteps = 1, ImuRateHz = 250,
                Disturbance_Wind = "light_breeze", Disturbance_Lighting = "GoldenHour",
            };
            m.TrajectorySpec["kind"] = "seeded_disturbed_hover_v1";
            m.TrajectorySpec["init_vel_enu"] = new[] { 4.0, 0.0, 0.0 };
            m.TrajectorySpec["seed"] = seed;
            m.ImuNoise["white_gyro_std"] = 0.002; m.ImuNoise["white_accel_std"] = 0.02; m.ImuNoise["flags"] = 1;
            m.Cameras.Add(new Dictionary<string, object> { ["name"] = "oakd_rgb", ["kind"] = "Rgb", ["width"] = 64, ["height"] = 48, ["rate_hz"] = 25.0, ["profile"] = "onboard-stub" });
            m.Cameras.Add(new Dictionary<string, object> { ["name"] = "oakd_depth", ["kind"] = "Depth", ["width"] = 64, ["height"] = 48, ["rate_hz"] = 25.0, ["profile"] = "onboard-stub" });
            m.Lidars.Add(new Dictionary<string, object> { ["name"] = "lidar", ["rate_hz"] = 10.0, ["points"] = 16 });
            return m;
        }

        public static string Sha(string s) {
            using var h = System.Security.Cryptography.SHA256.Create();
            var d = h.ComputeHash(System.Text.Encoding.UTF8.GetBytes(s));
            return BitConverter.ToString(d).Replace("-", "").ToLowerInvariant();
        }

        private void Build() {
            root = new GameObject("DroneReferenceFlight");
            ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "ground"; ground.transform.SetParent(root.transform);
            ground.transform.position = new Vector3(0, -0.5f, 0); ground.transform.localScale = new Vector3(60, 1, 60);
            box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = "tree_box"; box.transform.SetParent(root.transform);
            box.transform.position = new Vector3(0.1f, 1.5f, 4f); box.transform.localScale = new Vector3(3, 3, 1);
            boxCol = box.GetComponent<BoxCollider>();
            targetGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            targetGo.name = "target0"; targetGo.transform.SetParent(root.transform);
            targetGo.transform.position = new Vector3(4f, 1.5f, 8f);
            UnityEngine.Object.DestroyImmediate(targetGo.GetComponent<Collider>());
            targetGo.AddComponent<DroneDetectableTarget>().targetId = "target0";
            droneGo = new GameObject("drone");
            droneGo.transform.SetParent(root.transform);
            droneGo.transform.position = new Vector3(0, (float)Hover, 0);
            var sc = droneGo.AddComponent<SphereCollider>(); sc.radius = (float)Radius;
            rb = droneGo.AddComponent<Rigidbody>();
            rb.mass = (float)Mass; rb.linearDamping = 0; rb.angularDamping = 0.05f; rb.useGravity = true;
            rb.interpolation = RigidbodyInterpolation.None; rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            rb.linearVelocity = new Vector3(0, 0, 4f); // ENU east
            camGo = new GameObject("oakd_rgb");
            camGo.transform.SetParent(droneGo.transform, false);
            var cam = camGo.AddComponent<Camera>(); cam.enabled = false; cam.fieldOfView = 69; cam.aspect = 4f / 3f;
            cam.nearClipPlane = 0.1f; cam.farClipPlane = 50;
            var relay = droneGo.AddComponent<DroneCollisionRelay>();
            relay.OnContact = (other, speed, n) => { relayContacts++; lastRelayT = curT; contacts.Add((curT, other, speed, new double[] { n.z, -n.x, n.y })); };
        }

        private long curT;
        private int relayContacts, seenContacts;
        private long lastRelayT = long.MinValue / 2;
        private bool touching;

        public void Sample(DroneStateSample s) {
            Transform t = droneGo.transform;
            Vector3 p = DroneFrames.Enu(rb.position);
            s.Pos[0] = p.x; s.Pos[1] = p.y; s.Pos[2] = p.z;
            DroneFrames.QuatEnuFlu(rb.rotation, s.Quat);
            Vector3 v = DroneFrames.Enu(rb.linearVelocity);
            s.Vel[0] = v.x; s.Vel[1] = v.y; s.Vel[2] = v.z;
            Vector3 wb = DroneFrames.Enu(rb.angularVelocity);       // world ENU
            double[] ww = { wb.x, wb.y, wb.z };
            DroneRunSession.RotateInverse(s.Quat, ww, s.AngRateBody);
            double perRotor = Math.Max(lastThrust, 0) / 4.0;
            double rpm = Math.Sqrt(perRotor / 1.8e-7) * 60 / (2 * Math.PI);
            for (int i = 0; i < 4; i++) { s.RotorRpm[i] = rpm; s.MotorCmd[i] = Math.Min(1, Math.Max(0, lastThrust / (2 * Mass * 9.81))); }
            s.BattVolt = 14.8 - 1.2 * (1 - battSoc); s.BattAmp = lastThrust * 0.9; s.BattSoc = battSoc;
        }

        public DroneImuSample Sample() {
            // Specific force in body = R^T (a_world - g) with a from the last substep's velocity change.
            Vector3 a = (rb.linearVelocity - lastVel) / 0.001f;
            Vector3 f = DroneFrames.Enu(a - Physics.gravity);
            double[] q = new double[4]; DroneFrames.QuatEnuFlu(rb.rotation, q);
            double[] fb = new double[3]; DroneRunSession.RotateInverse(q, new double[] { f.x, f.y, f.z }, fb);
            Vector3 w = DroneFrames.Enu(rb.angularVelocity);
            double[] wb = new double[3]; DroneRunSession.RotateInverse(q, new double[] { w.x, w.y, w.z }, wb);
            // Noise from the flight's own seeded stream (consumed in a fixed order).
            return new DroneImuSample {
                Gx = wb[0] + 0.002 * rng.Normal(), Gy = wb[1] + 0.002 * rng.Normal(), Gz = wb[2] + 0.002 * rng.Normal(),
                Ax = fb[0] + 0.02 * rng.Normal(), Ay = fb[1] + 0.02 * rng.Normal(), Az = fb[2] + 0.02 * rng.Normal(),
                NoiseFlags = 1 };
        }

        /// <summary>Runs the flight, writing a run directory. Returns the number of state rows.</summary>
        public static void Run(string runDir, int seed, double durationS, DroneSplits splits, bool allowHeldout = false) {
            var mode = Physics.simulationMode;
            var gravity = Physics.gravity;
            var flight = new DroneReferenceFlight(seed);
            DroneRunSession session = null;
            try {
                Physics.simulationMode = SimulationMode.Script;
                Physics.gravity = new Vector3(0, -9.81f, 0);
                var m = MakeManifest(Path.GetFileName(runDir.TrimEnd('/')), seed);
                session = DroneRunSession.Begin(runDir, m, splits, allowHeldout, flight, flight);
                flight.Build();
                Physics.SyncTransforms();
                flight.Loop(session, durationS);
            }
            finally {
                session?.Close();
                if (flight.root != null) UnityEngine.Object.DestroyImmediate(flight.root);
                Physics.simulationMode = mode;
                Physics.gravity = gravity;
            }
        }

        private void Loop(DroneRunSession session, double durationS) {
            session.Writer.WriteObstacles(new[] {
                new Dictionary<string, object> { ["id"] = "tree_box", ["kind"] = "box", ["x"] = 4.0, ["y"] = -0.1, ["hx"] = 0.5, ["hy"] = 1.5, ["r"] = 1.0 } });
            session.Writer.WriteEvent(0, "marker", new Dictionary<string, object> { ["name"] = "start" });
            var m = session.Writer.Manifest;
            long ticks = (long)Math.Round(durationS * 1e9 / m.FixedDtNs);
            var cam = camGo.GetComponent<Camera>();
            var dets = new List<Dictionary<string, object>>();
            lastVel = rb.linearVelocity;
            curT = 0;
            session.OnSubstep(0, 0); // initial state at t = 0
            int camEvery = 8, lidarEvery = 20; // ticks: 25 Hz, 10 Hz
            for (long tick = 0; tick < ticks; tick++) {
                for (int sub = 0; sub < m.Substeps; sub++) {
                    long gs = tick * m.Substeps + sub;
                    curT = session.TimeNs(tick, sub + 1);
                    Actuate(gs);
                    lastVel = rb.linearVelocity;
                    Physics.Simulate(0.001f);
                    // the sample taken after Simulate belongs to the end of this substep
                    int nsub = sub + 1; long ntick = tick;
                    if (nsub == m.Substeps) { nsub = 0; ntick++; }
                    session.OnSubstep(ntick, nsub);
                    foreach (var c in contacts) session.Collision(c.t, c.other, c.speed, c.n);
                    contacts.Clear();
                    UpdateClearance(session, session.TimeNs(ntick, nsub));
                }
                long nt = tick + 1;
                if (nt % camEvery == 0) EmitCamera(session, nt, cam, dets);
                if (nt % lidarEvery == 0) EmitLidar(session, nt);
            }
            session.Writer.WriteEvent(session.TimeNs(ticks, 0), "marker", new Dictionary<string, object> { ["name"] = "end" });
        }

        private void Actuate(long gs) {
            if (gs % 20 == 0) { // seeded disturbance update every 20 ms
                cmdNoise = 0.08 * rng.Normal();
                torqueX = 0.02 * rng.Normal(); torqueY = 0.02 * rng.Normal(); torqueZ = 0.01 * rng.Normal();
            }
            // Ornstein-Uhlenbeck wind (m/s) per substep, tau = 2 s, sigma ~ 1 m/s
            double a = Math.Exp(-0.001 / 2.0), b = Math.Sqrt(1 - a * a);
            windE = a * windE + b * 1.0 * rng.Normal();
            windN = a * windN + b * 1.0 * rng.Normal();
            Vector3 pos = rb.position;
            double zerr = Hover - pos.y;
            double thrust = Mass * (9.81 + 4.0 * zerr - 3.0 * rb.linearVelocity.y) * (1 + cmdNoise);
            thrust = Math.Max(0, thrust);
            lastThrust = thrust;
            rb.AddForce(rb.rotation * Vector3.up * (float)thrust);
            // wind drag-like force on horizontal relative velocity (ENU E = Unity z, N = -Unity x)
            Vector3 vrel = new Vector3((float)-windN - rb.linearVelocity.x, 0, (float)windE - rb.linearVelocity.z);
            rb.AddForce(vrel * 0.15f);
            rb.AddTorque(new Vector3((float)torqueX, (float)torqueZ, (float)torqueY));
            battSoc = Math.Max(0, battSoc - thrust * 0.9 * 0.001 / 3600.0 / 1.5);
        }

        private void UpdateClearance(DroneRunSession s, long t) {
            Vector3 closest = boxCol.ClosestPoint(rb.position);
            clearance = Vector3.Distance(closest, rb.position) - Radius;
            // Fallback: if the contact message did not fire (it is not delivered in every Editor mode), log the touch itself.
            bool touchNow = clearance < 0.002;
            if (touchNow && !touching && relayContacts == seenContacts && t - lastRelayT > 50_000_000)
                s.Collision(t, "tree_box", rb.linearVelocity.magnitude, new double[] { 0, 0, 0 });
            touching = touchNow;
            seenContacts = relayContacts;
            s.Clearance.Update(t, clearance, "tree_box", s.Writer);
        }

        private void EmitCamera(DroneRunSession s, long tick, Camera cam, List<Dictionary<string, object>> dets) {
            long acq = s.TimeNs(tick, 0), done = acq + 2 * s.Writer.Manifest.FixedDtNs;
            const int w = 64, h = 48;
            var rgb = new byte[w * h * 3]; var depth = new ushort[w * h];
            Vector3 o = camGo.transform.position;
            for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
                int i = y * w + x;
                rgb[3 * i] = (byte)(x * 4); rgb[3 * i + 1] = (byte)(y * 5); rgb[3 * i + 2] = (byte)((seed * 7 + tick) & 0xFF);
                depth[i] = (ushort)Math.Min(65535, 1000 + 10 * x + 10 * y + (tick & 0xF));
            }
            // forward range in the centre of the depth image from a real ray
            if (Physics.Raycast(o, rb.rotation * Vector3.forward, out RaycastHit hit, 50f))
                depth[(h / 2) * w + w / 2] = (ushort)Math.Min(65535, (int)Math.Round(hit.distance * 1000));
            s.PushCamera(new DroneCameraFrame { Camera = "oakd_rgb", Kind = DroneCameraKind.Rgb, Width = w, Height = h, Rgb8 = rgb, AcquisitionNs = acq, CompletionNs = done });
            s.PushCamera(new DroneCameraFrame { Camera = "oakd_depth", Kind = DroneCameraKind.Depth, Width = w, Height = h, Depth16 = depth, AcquisitionNs = acq, CompletionNs = done });
            DroneDetectionProbe.Detect(cam, 40, ~0, dets);
            foreach (var d in dets) s.Detection(acq, d);
        }

        private void EmitLidar(DroneRunSession s, long tick) {
            const int n = 16; long period = 100_000_000;
            long acq = s.TimeNs(tick, 0) - period; // sweep that just completed
            if (acq < 0) return;
            var scan = new DroneLidarScan { AcquisitionNs = acq, X = new float[n], Y = new float[n], Z = new float[n], Intensity = new float[n], TimeOffsetNs = new int[n] };
            Vector3 o = rb.position;
            for (int i = 0; i < n; i++) {
                float ang = i * 2 * Mathf.PI / n;
                Vector3 dir = new Vector3(Mathf.Sin(ang), 0, Mathf.Cos(ang));
                float r = 20f; float inten = 0;
                if (Physics.Raycast(o, dir, out RaycastHit hit, 20f)) { r = hit.distance; inten = 1; }
                scan.X[i] = dir.z * r; scan.Y[i] = -dir.x * r; scan.Z[i] = 0; scan.Intensity[i] = inten;
                scan.TimeOffsetNs[i] = (int)(i * period / n);
            }
            s.PushLidar(scan);
        }
    }
}
