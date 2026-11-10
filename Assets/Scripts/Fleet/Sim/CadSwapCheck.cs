using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// The CAD-swap regression check: one rover per role on a flat floor plus a face-to-face pair, settled under
    /// physics, then every declared frame is compared with where it ended up. Run it after changing the profile
    /// or dropping in meshes; it writes a JSON report and returns whether everything is within tolerance.
    /// </summary>
    public static class CadSwapCheck {
        public sealed class Result {
            public bool Pass;
            public Dictionary<string, object> Report;
            public List<string> Failures = new List<string>();
        }

        public static Result Run(FleetSpec spec, string reportPath, string pairFaceA = "front", string pairFaceB = "rear") {
            SimulationMode previous = Physics.simulationMode;
            FleetRun.ConfigurePhysics(spec);
            var world = new FleetWorld(spec, scenario: false);
            var res = new Result();
            try {
                JsonNode c = spec.O["check"];
                double contactTol = c["contact_tolerance"].Double, frameTol = c["frame_tolerance"].Double, yawTol = c["yaw_tolerance"].Double;
                double spacing = c["spacing"].Double;
                FloorProperties floor = spec.Floor();
                var rigs = new List<RoverRig>();
                int i = 0;
                foreach (string role in spec.Roles()) {
                    var d = spec.Describe("check_" + role, role);
                    rigs.Add(new RoverRig(spec, d, floor, 0, world.Root.transform, new Pose2(1.0 + spacing * i++, 1.5, 0)));
                }
                // face-to-face pair of the first role in the default fleet with two instances (else the first role)
                string pairRole = null;
                var counts = new Dictionary<string, int>();
                foreach (var (_, r) in spec.FleetDefault()) { counts.TryGetValue(r, out int n); counts[r] = n + 1; if (n + 1 == 2 && pairRole == null) pairRole = r; }
                pairRole ??= rigs[0].D.Role;
                var da = spec.Describe("pair_a", pairRole);
                var db = spec.Describe("pair_b", pairRole);
                Pose2 pa = new Pose2(1.0, 4.0, 0.0);
                Pose2 pb = MatingPose(pa, da.Face(pairFaceA).Pose, db.Face(pairFaceB).Pose);
                var ra = new RoverRig(spec, da, floor, 0, world.Root.transform, pa);
                var rb = new RoverRig(spec, db, floor, 0, world.Root.transform, pb);
                Physics.SyncTransforms();

                int settle = (int)Math.Round(spec.O["physics.settle_s"].Double / spec.Dt);
                var all = new List<RoverRig>(rigs) { ra, rb };
                for (int s = 0; s < settle; s++) {
                    foreach (var r in all) r.ApplyPlant();
                    Physics.Simulate((float)spec.Dt);
                }

                Collider floorCol = world.Root.transform.Find("floor").GetComponent<Collider>();
                var rovers = new Dictionary<string, object>();
                foreach (var r in all) rovers[r.D.Id] = CheckRover(r, floorCol, contactTol, frameTol, yawTol, res.Failures);

                // pair: the two connector frames must coincide, facing each other, with matching hook points
                Transform fa = ra.Frames["face_" + pairFaceA], fb = rb.Frames["face_" + pairFaceB];
                FleetFrames.Attitude(fa.rotation, out double yawA, out _, out _);
                FleetFrames.Attitude(fb.rotation, out double yawB, out _, out _);
                double yawErr = Angles.Wrap(yawB - (yawA + Math.PI));
                Vector3 delta = fb.position - fa.position;
                Vector3 local = Quaternion.Inverse(fa.rotation) * delta;  // Unity local of face A: z = normal, x = -lateral, y = up
                double halfSpacing = spec.P["docking.hook_lateral_spacing"].Double / 2;
                double hookErr = 0;
                foreach (double side in new[] { -1.0, 1.0 }) {
                    Vector3 ha = fa.TransformPoint(FleetFrames.Local(0, side * halfSpacing, 0));
                    Vector3 hb = fb.TransformPoint(FleetFrames.Local(0, -side * halfSpacing, 0));
                    hookErr = Math.Max(hookErr, (ha - hb).magnitude);
                }
                bool posOk = delta.magnitude <= frameTol, yawOk = Math.Abs(yawErr) <= yawTol, hookOk = hookErr <= frameTol;
                if (!posOk) res.Failures.Add($"pair connector distance {delta.magnitude:0.0000} > {frameTol}");
                if (!yawOk) res.Failures.Add($"pair connector yaw error {yawErr:0.0000} > {yawTol}");
                if (!hookOk) res.Failures.Add($"pair hook mismatch {hookErr:0.0000} > {frameTol}");
                var pair = new Dictionary<string, object> {
                    ["role"] = pairRole, ["faces"] = new[] { pairFaceA, pairFaceB },
                    ["distance"] = (double)delta.magnitude, ["normal_gap"] = (double)local.z, ["lateral"] = (double)-local.x, ["vertical"] = (double)local.y,
                    ["yaw_error"] = yawErr, ["hook_point_mismatch"] = hookErr, ["pass"] = posOk && yawOk && hookOk
                };

                res.Pass = res.Failures.Count == 0;
                res.Report = new Dictionary<string, object> {
                    ["schema"] = "crane.fleet_cad_swap_check/v0", ["pass"] = res.Pass,
                    ["profile_sha256"] = spec.Profile.Sha256, ["overlay_sha256"] = spec.Overlay.Sha256, ["profile_id"] = spec.P["profile_id"].String,
                    ["unity_version"] = Application.unityVersion,
                    ["tolerances"] = new Dictionary<string, object> { ["contact"] = contactTol, ["frame"] = frameTol, ["yaw"] = yawTol },
                    ["settle_s"] = settle * spec.Dt, ["rovers"] = rovers, ["pair"] = pair, ["failures"] = res.Failures
                };
                if (!string.IsNullOrEmpty(reportPath)) {
                    Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(reportPath)) ?? ".");
                    File.WriteAllText(reportPath, FleetJson.Write(res.Report, true) + "\n");
                }
                return res;
            }
            finally {
                world.Destroy();
                Physics.simulationMode = previous;
            }
        }

        /// <summary>Pose of rover B so that its face fb coincides with A's face fa, facing it (planar).</summary>
        public static Pose2 MatingPose(Pose2 a, FramePose fa, FramePose fb) {
            double ca = Math.Cos(a.Yaw), sa = Math.Sin(a.Yaw);
            double px = a.X + ca * fa.X - sa * fa.Y, py = a.Y + sa * fa.X + ca * fa.Y;
            double yawB = Angles.Wrap(a.Yaw + fa.Yaw + Math.PI - fb.Yaw);
            double cb = Math.Cos(yawB), sb = Math.Sin(yawB);
            return new Pose2(px - (cb * fb.X - sb * fb.Y), py - (sb * fb.X + cb * fb.Y), yawB);
        }

        static Dictionary<string, object> CheckRover(RoverRig r, Collider floor, double contactTol, double frameTol, double yawTol, List<string> failures) {
            RoverTruth t = r.Truth();
            var o = new Dictionary<string, object> { ["role"] = r.D.Role, ["render"] = r.RenderSource,
                ["pitch"] = t.Pitch, ["roll"] = t.Roll, ["base_height"] = t.Z };
            if (Math.Abs(t.Pitch) > yawTol || Math.Abs(t.Roll) > yawTol) failures.Add($"{r.D.Id}: not level (pitch {t.Pitch:0.0000}, roll {t.Roll:0.0000})");
            if (Math.Abs(t.Z) > contactTol) failures.Add($"{r.D.Id}: chassis origin {t.Z:0.0000} m off the floor");

            var wheels = new Dictionary<string, object>();
            for (int i = 0; i < r.WheelColliders.Count; i++) {
                SphereCollider sc = r.WheelColliders[i];
                WheelFrame w = r.D.Wheels[i];
                Vector3 centre = sc.transform.position;
                double gap = double.NaN;
                if (floor.Raycast(new Ray(centre + Vector3.up * 0.5f, Vector3.down), out RaycastHit hit, 2f))
                    gap = centre.y - hit.point.y - sc.radius;   // > 0 floating, < 0 penetrating
                Vector3 frame = r.Frames["wheel_" + w.Name].position;
                double heightErr = frame.y - w.Axle.Z;
                bool ok = !double.IsNaN(gap) && Math.Abs(gap) <= contactTol && Math.Abs(heightErr) <= frameTol;
                if (!ok) failures.Add($"{r.D.Id}: wheel {w.Name} contact gap {gap:0.0000}, axle height error {heightErr:0.0000}");
                wheels[w.Name] = new Dictionary<string, object> { ["contact_gap"] = gap, ["axle_height_error"] = heightErr, ["pass"] = ok };
            }
            o["wheels"] = wheels;

            var faces = new Dictionary<string, object>();
            foreach (var f in r.D.Faces) faces[f.Name] = FrameError(r, "face_" + f.Name, f.Pose, t, frameTol, yawTol, failures);
            o["faces"] = faces;
            var sensors = new Dictionary<string, object>();
            foreach (var s in r.D.Sensors) sensors[s.Id] = FrameError(r, "sensor_" + s.Id, s.Pose, t, frameTol, yawTol, failures);
            o["sensors"] = sensors;

            // render geometry must not dip below the floor or float far above the collision envelope
            double minY = double.PositiveInfinity;
            foreach (var mr in r.Root.transform.Find("render").GetComponentsInChildren<Renderer>()) minY = Math.Min(minY, mr.bounds.min.y);
            o["render_min_height"] = minY;
            if (minY < -contactTol) failures.Add($"{r.D.Id}: render geometry {minY:0.0000} m below the floor");
            return o;
        }

        /// <summary>Error of a frame against its declared pose, evaluated in the world from the settled rover pose
        /// projected on the floor (so sinking, tilting or a wrong origin all show up).</summary>
        static Dictionary<string, object> FrameError(RoverRig r, string frame, FramePose declared, RoverTruth t, double tol, double yawTol, List<string> failures) {
            Transform f = r.Frames[frame];
            double c = Math.Cos(t.Yaw), s = Math.Sin(t.Yaw);
            double ex = t.X + c * declared.X - s * declared.Y, ey = t.Y + s * declared.X + c * declared.Y, ez = declared.Z;
            FleetFrames.ToEnu(f.position, out double x, out double y, out double z);
            FleetFrames.Attitude(f.rotation, out double yaw, out _, out _);
            double pos = Math.Sqrt((x - ex) * (x - ex) + (y - ey) * (y - ey) + (z - ez) * (z - ez));
            double yawErr = Angles.Wrap(yaw - (t.Yaw + declared.Yaw));
            bool ok = pos <= tol && Math.Abs(yawErr) <= yawTol;
            if (!ok) failures.Add($"{r.D.Id}: {frame} off by {pos:0.0000} m, yaw {yawErr:0.0000}");
            return new Dictionary<string, object> { ["position_error"] = pos, ["height_error"] = z - ez, ["yaw_error"] = yawErr, ["pass"] = ok };
        }
    }
}
