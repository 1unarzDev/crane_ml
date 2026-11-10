using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Crane.Drones.Environment;
using Crane.Drones.Logging;
using Crane.Theater.Aerial;
using Sim.Physics.Aerial;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Drones.Integration {
    /// <summary>
    /// One logged, seeded flight through the Drone Forest corridor on the REAL aerial plant with the onboard OAK-D cameras.
    ///   unity-drones --render LOG -executeMethod Crane.Drones.Integration.DroneForestFlightCli.Run
    ///     --drone-seed 1337 --drone-out DIR [--drone-run-id ID] [--drone-git-sha SHA] [--drone-git-dirty N]
    ///     [--drone-duration 40] [--drone-speed 3.5] [--drone-camera-hz 25] [--drone-rgb-size 640x360]
    ///     [--drone-depth-size 640x400] [--drone-no-render] [--drone-wind calm] [--drone-lighting GoldenHour]
    /// The split guard runs before the scene is opened. Exits the editor with 0 on success.
    /// </summary>
    public static class DroneForestFlightCli {
        const string ScenePath = "Assets/Scenes/Drone Forest.unity";
        const string PrefabPath = "Assets/Drones/Models/AOS7Quad.prefab";
        const float DepthMinM = 0.2f, DepthMaxM = 35f;

        [Serializable] class ForestTree { public int id; public string species; public float[] pos; public float dbh_cm; public float height_m; }
        [Serializable] class ForestDoc { public string schema; public int seed; public ForestTree[] trees; public float corridor_clear_radius_m; }

        static string[] Args => System.Environment.GetCommandLineArgs();
        static string Arg(string k, string def = null) => DroneSplitGuard.ReadArg(Args, k) ?? def;
        static double ArgD(string k, double def) => double.Parse(Arg(k, def.ToString("R", CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);
        static (int, int) ArgSize(string k, string def) { var p = Arg(k, def).Split('x'); return (int.Parse(p[0]), int.Parse(p[1])); }

        public static void Run() {
            int code;
            try { code = RunInner(); }
            catch (Exception e) { Debug.LogError("CRANE_DRONE_FLIGHT_FAILED " + e); code = 3; }
            Debug.Log("CRANE_DRONE_FLIGHT_EXIT code=" + code);
            EditorApplication.Exit(code);
        }

        static void Drain(IEnumerator e) { while (e.MoveNext()) { if (e.Current is IEnumerator n) Drain(n); } }

        static int RunInner() {
            // ---- split guard BEFORE the scene is built
            string split = DroneSplitGuard.EnforceFromCommandLine(Args);
            var splits = DroneSplitGuard.Parse(File.ReadAllText(DroneSplitGuard.FindSplitsFile(Args)));
            bool allowHeldout = Array.IndexOf(Args, DroneSplitGuard.AllowFlag) >= 0;
            int seed = int.Parse(Arg("--drone-seed"), CultureInfo.InvariantCulture);
            string outDir = Path.GetFullPath(Arg("--drone-out") ?? throw new ArgumentException("--drone-out required"));
            string runId = Arg("--drone-run-id", Path.GetFileName(outDir.TrimEnd('/')));
            double duration = ArgD("--drone-duration", 40.0), speed = ArgD("--drone-speed", 3.5), camHz = ArgD("--drone-camera-hz", 25.0);
            var (rgbW, rgbH) = ArgSize("--drone-rgb-size", "640x360");
            var (dW, dH) = ArgSize("--drone-depth-size", "640x400");
            bool render = Array.IndexOf(Args, "--drone-no-render") < 0;
            string wind = Arg("--drone-wind", "calm"), lighting = Arg("--drone-lighting", "GoldenHour");
            if (wind != "calm") throw new ArgumentException("only --drone-wind calm is implemented for this flight");
            string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            Debug.Log($"CRANE_DRONE_FLIGHT_START run={runId} seed={seed} split={split} out={outDir} render={render}");

            // ---- forest manifest (provenance + obstacle geometry)
            string fmRel = $"Assets/Drones/Environment/Manifests/forest_seed{seed}_manifest.json";
            string fmPath = Path.Combine(project, fmRel);
            byte[] fmBytes = File.ReadAllBytes(fmPath);
            var forestDoc = JsonUtility.FromJson<ForestDoc>(System.Text.Encoding.UTF8.GetString(fmBytes));
            if (forestDoc == null || forestDoc.trees == null || forestDoc.seed != seed)
                throw new InvalidDataException($"forest manifest {fmRel} missing trees or seed != {seed}");

            // ---- scene
            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            var world = new AerialWorld();
            Drain(world.Prepare(null));
            var mk = world.Markers;
            if (mk.isStubScene || mk.corridor.Count < 2) throw new InvalidOperationException("Drone Forest corridor markers not found: " + mk.Describe);
            var forest = UnityEngine.Object.FindFirstObjectByType<DroneForestDirector>();
            if (forest != null) { forest.SetPreset(lighting); forest.SetDepthOfField(false); forest.SetFilmGrain(false); forest.SetMotionBlur(false); }

            // ---- plant (real AOS 7 V5 profile, as CranePlantAdapter: terrain is not flat, so the flat-ground plane is disabled)
            string profDir = Path.Combine(project, "Tools", "Aerial", "profiles");
            var profile = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve("aos7_v5_f90_6s", profDir), true, true);
            var spec = profile.Vehicle;
            spec.GroundHeight = -1000.0;
            var plant = new CraneAerialPlantCore(spec, 1000.0, 500.0);
            plant.AttachImu(new CraneAerialImu(500.0, spec.Gravity, (ulong)seed));
            var ctl = new CraneMultirotorController(spec, plant.ControlDt);
            var follower = new CraneTrajectoryFollower();
            ctl.Follower = follower;
            plant.CommandSource = ctl;

            var wps = new double[mk.corridor.Count][];
            for (int i = 0; i < wps.Length; i++) wps[i] = AerialPlantLogSource.EnuFromUnity(mk.corridor[i].x, mk.corridor[i].y, mk.corridor[i].z);
            double yaw0 = Math.Atan2(wps[1][1] - wps[0][1], wps[1][0] - wps[0][0]);
            var s0 = CraneAerialPlantCore.HoverState(spec, wps[0], yaw0);
            plant.Reset(s0, (ulong)seed);
            ctl.ResetIntegrators();
            ctl.HoldPosition(s0.P, yaw0);
            follower.SetTrajectory(wps, new[] { speed }, yaw0);
            var source = new AerialPlantLogSource(plant);

            // ---- manifest
            var m = new DroneRunManifest {
                RunId = runId, SceneSeed = seed, Scene = ScenePath, ForestManifestSha256 = AerialPlantLogSource.Sha256(fmBytes),
                CraneGitSha = Arg("--drone-git-sha", "unknown"), UnityVersion = Application.unityVersion,
                WallClockStartUtc = DateTime.UtcNow.ToString("o"), FixedDtNs = 5_000_000,
                Disturbance_Wind = wind, Disturbance_Lighting = lighting,
            };
            source.FillManifest(m, profile, ctl);
            m.TrajectorySpec["kind"] = "corridor_waypoints_follower_v1";
            m.TrajectorySpec["source"] = "DroneForestMarker CorridorWaypoint markers in the scene (sorted by name)";
            m.TrajectorySpec["waypoints_enu"] = wps;
            m.TrajectorySpec["speed_mps"] = speed;
            m.TrajectorySpec["max_accel_mps2"] = follower.MaxAccel;
            m.TrajectorySpec["face_along_track"] = follower.FaceAlongTrack;
            m.TrajectorySpec["initial_yaw_enu_rad"] = yaw0;
            m.TrajectorySpec["duration_s"] = duration;
            m.TrajectorySpec["seed"] = seed;
            m.Extra["forest_manifest_path"] = fmRel;
            m.Extra["git_dirty_paths"] = Arg("--drone-git-dirty", "unknown");
            m.Extra["plant_ground_plane"] = "disabled (GroundHeight -1000, as CranePlantAdapter); the plant has no tree or terrain contact";
            m.Extra["collision_events"] = "NOT_AVAILABLE: the aerial plant has no contact with trees/terrain (PHYSICS.md known gap); near_miss uses manifest trunk geometry";
            m.Extra["lidar"] = "NOT_AVAILABLE: no LiDAR is mounted on the aerial demo rig (the only Lidar3D is the ROS boat sensor, not part of the drone onboard set)";
            m.Extra["detection_events"] = "NOT_AVAILABLE: no DroneDetectableTarget in the Drone Forest scene";
            double droneRadius = 0;
            foreach (var r in spec.Rotors) droneRadius = Math.Max(droneRadius, Math.Sqrt(r.Position[0] * r.Position[0] + r.Position[1] * r.Position[1]) + r.Radius);
            m.Extra["clearance_model"] = $"horizontal distance to trunk axis - dbh/2 - {droneRadius:0.000} m (max rotor hub radius + prop radius); trunks only (no branches/foliage)";

            int camEvery = (int)Math.Round(1e9 / (camHz * m.FixedDtNs));
            if (Math.Abs(camEvery * m.FixedDtNs * camHz - 1e9) > 1) throw new ArgumentException("camera period must be a whole number of 5 ms ticks");

            // ---- cameras (onboard profile only)
            QuadHandle quad = null; OnboardCamera rgb = null; OnboardCamera depthCam = null;
            RenderTexture depthRt = null; RTHandle depthHandle = null; Texture2D rgbTex = null, depthTex = null;
            int aovCalls = 0;
            if (render) {
                AerialRobot.PrefabLoader = () => AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                quad = new QuadHandle(new AerialRobot().CreateHero("AOS7 Quad"));
                var sceneProfiles = new List<VolumeProfile>();
                var sv = UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsInactive.Exclude);
                Array.Sort(sv, (x, y) => y.priority.CompareTo(x.priority));
                foreach (var v in sv) if (v.sharedProfile != null && v.gameObject.layer != RenderProfiles.CinematicLayer && v.weight > 0.01f) sceneProfiles.Add(v.sharedProfile);
                var onboardProfile = RenderProfiles.BuildOnboard(sceneProfiles);
                var leak = RenderProfiles.CinematicOverrides(onboardProfile);
                if (leak.Count > 0) throw new InvalidOperationException("onboard profile contains cinematic effects: " + string.Join(",", leak));
                RenderProfiles.MakeVolume("Aerial Onboard Volume", onboardProfile, RenderProfiles.OnboardLayer, 1000f);

                rgb = new OnboardCamera("oakd_rgb", quad.CameraMount, Vector3.zero, OakD.RgbHfovDeg, rgbW, rgbH, false);
                depthCam = new OnboardCamera("oakd_depth", quad.CameraMount, new Vector3(-OakD.BaselineM * 0.5f, 0, 0), OakD.StereoHfovDeg, dW, dH, true);
                foreach (var (c, hf, w, h) in new[] { (rgb, OakD.RgbHfovDeg, rgbW, rgbH), (depthCam, OakD.StereoHfovDeg, dW, dH) }) {
                    if (forest != null) DroneForestDirector.ConfigureCamera(c.Cam);
                    RenderProfiles.ConfigureOnboardCamera(c.Cam, hf, w, h);   // after the forest defaults: post effects off again
                }
                depthRt = new RenderTexture(dW, dH, 0, RenderTextureFormat.RFloat) { name = "oakd depth aov" };
                depthRt.Create();
                depthHandle = RTHandles.Alloc(depthRt);
                var hd = depthCam.Cam.GetComponent<HDAdditionalCameraData>();
                var builder = new AOVRequestBuilder();
                builder.Add(AOVRequest.NewDefault(), id => depthHandle, null, new[] { AOVBuffers.DepthStencil }, (cmd, bufs, props) => aovCalls++);
                hd.SetAOVRequests(builder.Build());
                rgbTex = new Texture2D(rgbW, rgbH, TextureFormat.RGB24, false);
                depthTex = new Texture2D(dW, dH, TextureFormat.RFloat, false);

                string onboardNote = "RenderProfiles onboard: volume layer " + RenderProfiles.OnboardLayer + " only, no cinematic effects, AA off, instant auto-exposure, neutral tonemap";
                m.Cameras.Add(new Dictionary<string, object> {
                    ["name"] = "oakd_rgb", ["kind"] = "Rgb", ["width"] = rgbW, ["height"] = rgbH, ["rate_hz"] = camHz, ["profile"] = onboardNote,
                    ["hfov_deg"] = (double)OakD.RgbHfovDeg, ["mount"] = "DroneVisualRig.cameraMount (OAK-D S2 RGB)",
                    ["shutter"] = "instantaneous render at acq_ns (rolling shutter NOT modeled)", ["complete_ns"] = "== acq_ns (no readback latency modeled)" });
                m.Cameras.Add(new Dictionary<string, object> {
                    ["name"] = "oakd_depth", ["kind"] = "Depth", ["width"] = dW, ["height"] = dH, ["rate_hz"] = camHz, ["profile"] = onboardNote,
                    ["hfov_deg"] = (double)OakD.StereoHfovDeg, ["mount"] = "left OV9282 pose (cameraMount - baseline/2)",
                    ["range_gate_m"] = new double[] { DepthMinM, DepthMaxM },
                    ["source"] = "pending", ["note"] = "rendered ground-truth Z-depth, NOT stereo-matched (no disparity noise, no SGBM artefacts)" });
            } else m.Extra["cameras"] = "not rendered (--drone-no-render)";

            // ---- obstacles for clearance and plots (plant ENU)
            var near = new List<(string id, double e, double n, double u0, double u1, double r)>();
            var obstacles = new List<Dictionary<string, object>>();
            foreach (var t in forestDoc.trees) {
                var p = AerialPlantLogSource.EnuFromUnity(t.pos[0], t.pos[1], t.pos[2]);
                double rr = t.dbh_cm / 200.0;
                obstacles.Add(new Dictionary<string, object> {
                    ["id"] = "tree_" + t.id, ["kind"] = "trunk", ["species"] = t.species, ["x"] = p[0], ["y"] = p[1], ["z_base"] = p[2],
                    ["r"] = rr, ["height_m"] = (double)t.height_m });
                double best = double.MaxValue;
                foreach (var w in wps) best = Math.Min(best, Math.Sqrt((w[0] - p[0]) * (w[0] - p[0]) + (w[1] - p[1]) * (w[1] - p[1])));
                if (best < 30.0) near.Add(("tree_" + t.id, p[0], p[1], p[2], p[2] + t.height_m, rr));
            }

            Directory.CreateDirectory(Path.GetDirectoryName(outDir));
            var session = DroneRunSession.Begin(outDir, m, splits, allowHeldout, source, source);
            int frames = 0; double rgbMs = 0, depthMs = 0;
            string depthSource = null; bool flipDepth = true;
            try {
                session.Writer.WriteObstacles(obstacles);
                session.Writer.WriteEvent(0, "marker", new Dictionary<string, object> { ["name"] = "start" });
                var flight = new AerialLoggedFlight(session, source);
                long ticks = (long)Math.Round(duration * 1e9 / m.FixedDtNs);
                bool finishedLogged = false;
                double errSq = 0, errMax = 0; long errN = 0;

                if (render) for (int i = 0; i < 8; i++) { PoseQuad(quad, plant, 0.04); rgb.Render(); depthCam.Render(); }   // preroll, discarded

                var sw = new System.Diagnostics.Stopwatch();
                while (flight.Tick < ticks) {
                    long tick = flight.StepTick();
                    long tNs = session.TimeNs(tick, 0);
                    var st = plant.State;
                    // clearance against manifest trunks
                    double dmin = double.MaxValue; string dId = null;
                    foreach (var o in near) {
                        if (st.P[2] < o.u0 || st.P[2] > o.u1) continue;
                        double d = Math.Sqrt((st.P[0] - o.e) * (st.P[0] - o.e) + (st.P[1] - o.n) * (st.P[1] - o.n)) - o.r - droneRadius;
                        if (d < dmin) { dmin = d; dId = o.id; }
                    }
                    if (dId != null) session.Clearance.Update(tNs, dmin, dId, session.Writer);
                    double ex = st.P[0] - ctl.PositionSetpoint[0], ey = st.P[1] - ctl.PositionSetpoint[1], ez = st.P[2] - ctl.PositionSetpoint[2];
                    double e2 = ex * ex + ey * ey + ez * ez; errSq += e2; errN++; errMax = Math.Max(errMax, Math.Sqrt(e2));
                    if (follower.Finished && !finishedLogged) {
                        finishedLogged = true;
                        session.Writer.WriteEvent(tNs, "marker", new Dictionary<string, object> { ["name"] = "trajectory_finished" });
                    }
                    if (!render || tick % camEvery != 0) continue;

                    PoseQuad(quad, plant, camEvery * m.FixedDtNs * 1e-9);
                    Physics.SyncTransforms();
                    sw.Restart();
                    rgb.Render();
                    byte[] rgbBytes = ReadRgb(rgb.Rt, rgbTex);
                    rgbMs += sw.Elapsed.TotalMilliseconds;
                    session.PushCamera(new DroneCameraFrame { Camera = "oakd_rgb", Kind = DroneCameraKind.Rgb, Width = rgbW, Height = rgbH, Rgb8 = rgbBytes, AcquisitionNs = tNs, CompletionNs = tNs });

                    sw.Restart();
                    int before = aovCalls;
                    depthCam.Render();
                    float[] raw = null;
                    if (aovCalls > before) raw = ReadFloat(depthRt, depthTex);
                    if (depthSource == null) {
                        // decide once: AOV z-buffer (with the row order that matches physics raycasts) or raycast fallback
                        if (raw == null) { depthSource = "raycast_colliders_fallback (AOV callback did not fire)"; }
                        else {
                            Verify(depthCam.Cam, raw, dW, dH, out double eF, out double eK, out int nh);
                            if (nh >= 12) {
                                flipDepth = eF <= eK;
                                double best = Math.Min(eF, eK);
                                depthSource = best < 0.05 ? "hdrp_aov_depthstencil_zbuffer" : "raycast_colliders_fallback (AOV disagreed with raycasts)";
                                Debug.Log($"CRANE_DRONE_DEPTH_CHECK hits={nh} median_rel_err flip={eF:0.0000} keep={eK:0.0000} -> {depthSource} flip={flipDepth}");
                                m.Extra["depth_check"] = new Dictionary<string, object> { ["raycast_hits"] = nh, ["median_rel_err_flip"] = eF, ["median_rel_err_keep"] = eK, ["flip_rows"] = flipDepth, ["at_t_ns"] = tNs };
                            }
                        }
                        if (depthSource != null) m.Cameras[1]["source"] = depthSource;
                    }
                    ushort[] mm = depthSource != null && depthSource.StartsWith("hdrp") ? ToMm(raw, dW, dH, depthCam.Cam, flipDepth)
                        : RaycastDepth(depthCam.Cam, dW, dH);
                    depthMs += sw.Elapsed.TotalMilliseconds;
                    session.PushCamera(new DroneCameraFrame { Camera = "oakd_depth", Kind = DroneCameraKind.Depth, Width = dW, Height = dH, Depth16 = mm, AcquisitionNs = tNs, CompletionNs = tNs });
                    frames++;
                    if (frames % 100 == 0) Debug.Log($"CRANE_DRONE_FLIGHT_FRAME {frames} t={tNs * 1e-9:0.00}s rgb_ms={rgbMs / frames:0.0} depth_ms={depthMs / frames:0.0} source={depthSource}");
                }
                if (depthSource == null && render) m.Cameras[1]["source"] = "raycast_colliders_fallback (no verification hits)";
                m.Extra["tracking_error_m"] = new Dictionary<string, object> { ["rms"] = Math.Sqrt(errSq / Math.Max(1, errN)), ["max"] = errMax, ["note"] = "|plant position - follower setpoint| per tick" };
                m.Extra["trajectory_finished"] = follower.Finished;
                m.Extra["imu_dropped"] = plant.Imu.Dropped;
                session.Writer.WriteEvent(session.TimeNs(flight.Tick, 0), "marker", new Dictionary<string, object> { ["name"] = "end" });
            }
            finally { session.Close(); }
            Debug.Log($"CRANE_DRONE_FLIGHT_DONE run={runId} frames={frames} depth={depthSource} imu={source.ImuSamples} out={outDir}");
            depthHandle?.Release();
            return 0;
        }

        static void PoseQuad(QuadHandle quad, CraneAerialPlantCore plant, double dt) {
            var st = plant.State;
            CraneAerialFrames.UnityLookAxes(st.Q, out double[] f, out double[] u);
            var p = CraneAerialFrames.EnuToUnity(st.P);
            var w = new float[st.RotorSpeed.Length];
            for (int i = 0; i < w.Length; i++) w[i] = (float)Math.Abs(st.RotorSpeed[i]);
            quad.Pose(new TrajSample {
                t = st.T, pos = new Vector3((float)p[0], (float)p[1], (float)p[2]),
                rot = Quaternion.LookRotation(new Vector3((float)f[0], (float)f[1], (float)f[2]), new Vector3((float)u[0], (float)u[1], (float)u[2])),
                rotorRadS = w }, dt);
        }

        /// <summary>RGB24, rows top to bottom (Texture2D rows are bottom-up).</summary>
        static byte[] ReadRgb(RenderTexture rt, Texture2D tex) {
            var prev = RenderTexture.active; RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0, false);
            RenderTexture.active = prev;
            var src = tex.GetRawTextureData<byte>();
            int w = rt.width, h = rt.height, stride = w * 3;
            var o = new byte[stride * h];
            for (int y = 0; y < h; y++) Unity.Collections.NativeArray<byte>.Copy(src, (h - 1 - y) * stride, o, y * stride, stride);
            return o;
        }

        /// <summary>Raw float rows as stored (row 0 = texture row 0).</summary>
        static float[] ReadFloat(RenderTexture rt, Texture2D tex) {
            var prev = RenderTexture.active; RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0, false);
            RenderTexture.active = prev;
            return tex.GetRawTextureData<float>().ToArray();
        }

        static double LinearZ(float d, Camera cam) {
            double n = cam.nearClipPlane, f = cam.farClipPlane;
            if (SystemInfo.usesReversedZBuffer) return d <= 0 ? double.PositiveInfinity : n * f / (n + d * (f - n));
            return d >= 1 ? double.PositiveInfinity : n * f / (f - d * (f - n));
        }

        /// <summary>Sample (image row y, top-down) from a raw texture-row buffer.</summary>
        static float At(float[] raw, int w, int h, int x, int y, bool flip) => raw[(flip ? h - 1 - y : y) * w + x];

        static ushort[] ToMm(float[] raw, int w, int h, Camera cam, bool flip) {
            var o = new ushort[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++) {
                    double z = LinearZ(At(raw, w, h, x, y, flip), cam);
                    o[y * w + x] = z >= DepthMinM && z <= DepthMaxM ? (ushort)Math.Round(z * 1000.0) : (ushort)0;
                }
            return o;
        }

        static bool RayZ(Camera cam, int w, int h, int x, int y, out double z) {
            var ray = cam.ViewportPointToRay(new Vector3((x + 0.5f) / w, 1f - (y + 0.5f) / h, 0));
            if (Physics.Raycast(ray, out RaycastHit hit, DepthMaxM / 0.5f, ~0, QueryTriggerInteraction.Ignore)) {
                z = hit.distance * Vector3.Dot(ray.direction, cam.transform.forward); return true;
            }
            z = 0; return false;
        }

        static ushort[] RaycastDepth(Camera cam, int w, int h) {
            var o = new ushort[w * h];
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                    if (RayZ(cam, w, h, x, y, out double z) && z >= DepthMinM && z <= DepthMaxM) o[y * w + x] = (ushort)Math.Round(z * 1000.0);
            return o;
        }

        /// <summary>Median relative error of AOV depth vs collider raycasts on a 24x16 grid, for both row orders.</summary>
        static void Verify(Camera cam, float[] raw, int w, int h, out double eFlip, out double eKeep, out int hits) {
            var a = new List<double>(); var b = new List<double>();
            for (int gy = 0; gy < 16; gy++)
                for (int gx = 0; gx < 24; gx++) {
                    int x = (gx * 2 + 1) * w / 48, y = (gy * 2 + 1) * h / 32;
                    if (!RayZ(cam, w, h, x, y, out double z) || z < DepthMinM || z > DepthMaxM) continue;
                    a.Add(Math.Abs(LinearZ(At(raw, w, h, x, y, true), cam) - z) / z);
                    b.Add(Math.Abs(LinearZ(At(raw, w, h, x, y, false), cam) - z) / z);
                }
            hits = a.Count;
            a.Sort(); b.Sort();
            eFlip = hits > 0 ? a[hits / 2] : double.PositiveInfinity;
            eKeep = hits > 0 ? b[hits / 2] : double.PositiveInfinity;
        }
    }
}
