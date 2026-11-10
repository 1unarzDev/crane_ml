// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using Crane.Drones.Environment;
using Crane.Drones.Integration;
using Crane.Drones.Logging;
using Crane.Theater.Aerial;
using Sim.Physics.Aerial;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// M5/M6 batch: autonomous missions through the Drone Forest on the real aerial plant, sensors raycast against the
    /// scene colliders, ground truth (clearance, collision) from the forest manifest, every run logged as
    /// crane-drone-log/1. One Unity invocation:
    ///   unity-drones [--render] LOG -executeMethod Crane.Drones.Autonomy.AutonomyBatchCli.Run --drone-seed 1337
    ///     --autonomy-out DIR [--autonomy-missions all|a,b] [--autonomy-cross N] [--autonomy-speeds 2,2.5,3]
    ///     [--autonomy-seeds 1,2] [--autonomy-planner map_rhp|reactive] [--autonomy-set Field=Value ...]
    ///     [--autonomy-state-every 5] [--autonomy-render-mission NAME --autonomy-render-speed 2.5]
    ///     [--autonomy-render-only] [--autonomy-sweep-speeds 4,6] [--drone-git-sha SHA]
    /// The split guard runs before the scene opens. Writes DIR/summary.jsonl and one log directory per run.
    /// </summary>
    public static class AutonomyBatchCli {
        const string ScenePath = "Assets/Scenes/Drone Forest.unity";
        const string PrefabPath = "Assets/Drones/Models/AOS7Quad.prefab";
        static string[] Args => System.Environment.GetCommandLineArgs();
        static string Arg(string k, string d = null) => DroneSplitGuard.ReadArg(Args, k) ?? d;
        static bool Flag(string k) => Array.IndexOf(Args, k) >= 0;
        static readonly CultureInfo CI = CultureInfo.InvariantCulture;

        public static void Run() {
            int code;
            try { code = Inner(); }
            catch (Exception e) { Debug.LogError("CRANE_AUTONOMY_FAILED " + e); code = 3; }
            Debug.Log("CRANE_AUTONOMY_EXIT code=" + code);
            EditorApplication.Exit(code);
        }

        static int Inner() {
            string split = DroneSplitGuard.EnforceFromCommandLine(Args);
            var splits = DroneSplitGuard.Parse(File.ReadAllText(DroneSplitGuard.FindSplitsFile(Args)));
            bool allowHeldout = Flag(DroneSplitGuard.AllowFlag);
            int seed = int.Parse(Arg("--drone-seed"), CI);
            string outDir = Path.GetFullPath(Arg("--autonomy-out") ?? throw new ArgumentException("--autonomy-out required"));
            Directory.CreateDirectory(outDir);
            string project = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
            string fmRel = $"Assets/Drones/Environment/Manifests/forest_seed{seed}_manifest.json";
            byte[] fmBytes = File.ReadAllBytes(Path.Combine(project, fmRel));
            string fmSha = AerialPlantLogSource.Sha256(fmBytes);
            var world = ForestManifestWorld.FromJson(System.Text.Encoding.UTF8.GetString(fmBytes));
            if (world.Seed != seed) throw new InvalidDataException($"manifest seed {world.Seed} != {seed}");
            Debug.Log($"CRANE_AUTONOMY_START seed={seed} split={split} capsules={world.Count} out={outDir}");

            EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
            Physics.SyncTransforms();
            bool terrain = !double.IsNaN(UnityColliderRayWorld.TerrainHeight(world.Spawn.X, world.Spawn.Y));
            if (terrain) world.GroundProvider = (x, y) => { double h = UnityColliderRayWorld.TerrainHeight(x, y); return double.IsNaN(h) ? -1000 : h; };
            var sensorWorld = new UnityColliderRayWorld(world);

            var prof = CraneAerialProfile.LoadFile(CraneAerialProfile.Resolve("aos7_v5_f90_6s", Path.Combine(project, "Tools", "Aerial", "profiles")), true, true);
            var spec = prof.Vehicle; spec.GroundHeight = -1000.0;   // terrain is not flat; tree/terrain contact is judged from the manifest
            var bt = StopRule.SimulateBrakeTable(spec);

            var baseCfg = new AutonomyConfig { Planner = Arg("--autonomy-planner", "map_rhp") };
            for (int i = 0; i < Args.Length - 1; i++) if (Args[i] == "--autonomy-set") Set(baseCfg, Args[i + 1]);
            var missions = MissionCatalog.Build(world, baseCfg, int.Parse(Arg("--autonomy-cross", "0"), CI));
            string sel = Arg("--autonomy-missions", "all");
            if (sel != "all") missions = missions.Where(m => sel.Split(',').Contains(m.Name)).ToList();
            var speeds = Arg("--autonomy-speeds", "2.5").Split(',').Select(s => double.Parse(s, CI)).ToArray();
            var seeds = Arg("--autonomy-seeds", "1").Split(',').Select(s => ulong.Parse(s, CI)).ToArray();
            int stateEvery = int.Parse(Arg("--autonomy-state-every", "5"), CI);
            string sha = Arg("--drone-git-sha", "unknown");

            using (var summary = new StreamWriter(Path.Combine(outDir, "summary.jsonl"), true)) {
                if (!Flag("--autonomy-render-only"))
                    foreach (var v in speeds) foreach (var s in seeds) foreach (var ms in missions) {
                        var c = baseCfg.Clone(); c.VLimit = v; c.RunSeed = s;
                        var r = Fly(ms, c, spec, prof, world, sensorWorld, bt, outDir, fmRel, fmSha, splits, allowHeldout, stateEvery, sha, null);
                        summary.WriteLine(DroneJson.Write(r)); summary.Flush();
                    }
                string sweep = Arg("--autonomy-sweep-speeds");
                if (sweep != null && !Flag("--autonomy-render-only"))
                    foreach (var v in sweep.Split(',').Select(x => double.Parse(x, CI))) foreach (var ms in missions) {
                        var c = baseCfg.Clone(); c.VLimit = v; c.RunSeed = seeds[0];
                        var r = Fly(ms, c, spec, prof, world, sensorWorld, bt, outDir, fmRel, fmSha, splits, allowHeldout, stateEvery, sha, null);
                        r["sweep"] = true; summary.WriteLine(DroneJson.Write(r)); summary.Flush();
                    }
                string rm = Arg("--autonomy-render-mission");
                if (rm != null) {
                    var ms = MissionCatalog.Build(world, baseCfg, int.Parse(Arg("--autonomy-cross", "0"), CI)).FirstOrDefault(m => m.Name == rm)
                             ?? throw new ArgumentException("no mission " + rm);
                    var c = baseCfg.Clone(); c.VLimit = double.Parse(Arg("--autonomy-render-speed", "2.5"), CI); c.RunSeed = ulong.Parse(Arg("--autonomy-render-seed", "1"), CI);
                    var rr = new RenderRig(Path.Combine(outDir, "render_" + rm), Arg("--autonomy-still-times", "6,18,32").Split(',').Select(x => double.Parse(x, CI)).ToArray());
                    var r = Fly(ms, c, spec, prof, world, sensorWorld, bt, outDir, fmRel, fmSha, splits, allowHeldout, stateEvery, sha, rr);
                    r["render_stills"] = rr.Written;
                    summary.WriteLine(DroneJson.Write(r)); summary.Flush();
                    rr.Dispose();
                }
            }
            Debug.Log("CRANE_AUTONOMY_DONE out=" + outDir);
            return 0;
        }

        /// <summary>Autonomy events -> the run's events.jsonl. Events raised before the session exists (mission constructor)
        /// are buffered and flushed on Attach. Stamps are clamped non-decreasing (the writer enforces monotonic events).</summary>
        sealed class SessionRecorder : IAutonomyRecorder {
            DroneRunSession s; public long Last;
            readonly List<(long t, string k, Dictionary<string, object> f)> pending = new List<(long, string, Dictionary<string, object>)>();
            public void Attach(DroneRunSession session) { s = session; foreach (var e in pending) Write(e.t, e.k, e.f); pending.Clear(); }
            public void Event(long tNs, string kind, Dictionary<string, object> f) { if (s == null) pending.Add((tNs, kind, f)); else Write(tNs, kind, f); }
            void Write(long tNs, string kind, Dictionary<string, object> f) { if (tNs < Last) tNs = Last; Last = tNs; s.Writer.WriteEvent(tNs, kind, f); }
        }

        static Dictionary<string, object> Fly(MissionSpec ms, AutonomyConfig c, VehicleSpecV2 spec, CraneAerialProfile prof, ForestManifestWorld world,
                UnityColliderRayWorld sensorWorld, (double[] vs, double[] ds) bt, string outDir, string fmRel, string fmSha, DroneSplits splits, bool allowHeldout,
                int stateEvery, string sha, RenderRig render) {
            try { return FlyInner(ms, c, spec, prof, world, sensorWorld, bt, outDir, fmRel, fmSha, splits, allowHeldout, stateEvery, sha, render); }
            catch (DroneHeldoutException) { throw; }
            catch (Exception e) {   // one broken run must not lose the batch: record it and continue
                Debug.LogError($"CRANE_AUTONOMY_RUN_FAILED {ms.Name} v={c.VLimit} seed={c.RunSeed}: {e}");
                return new Dictionary<string, object> { ["Name"] = ms.Name, ["VLimit"] = c.VLimit, ["RunSeed"] = (double)c.RunSeed, ["Outcome"] = "harness_error", ["error"] = e.ToString() };
            }
        }

        static Dictionary<string, object> FlyInner(MissionSpec ms, AutonomyConfig c, VehicleSpecV2 spec, CraneAerialProfile prof, ForestManifestWorld world,
                UnityColliderRayWorld sensorWorld, (double[] vs, double[] ds) bt, string outDir, string fmRel, string fmSha, DroneSplits splits, bool allowHeldout,
                int stateEvery, string sha, RenderRig render) {
            string runId = $"auto_s{ms.ForestSeed}_{ms.Name}_{c.Planner}_v{c.VLimit.ToString("0.0", CI)}_r{c.RunSeed}" + (render != null ? "_render" : "");
            string dir = Path.Combine(outDir, runId);
            var prior = new ManifestPriorMap(world, c, c.RunSeed);
            var rule = new StopRule(bt.vs, bt.ds, c.TauPrior, c.Margin, c.AgeWindow, c.AgeQuantile);
            var rec = new SessionRecorder();
            var mission = new AutonomousMission(ms, c, spec, sensorWorld, world, world, prior, rec, rule) { PriorForBelief = prior };
            var source = new AerialPlantLogSource(mission.Plant);
            var m = new DroneRunManifest {
                RunId = runId, SceneSeed = ms.ForestSeed, Scene = ScenePath, ForestManifestSha256 = fmSha, CraneGitSha = sha,
                UnityVersion = Application.unityVersion, WallClockStartUtc = DateTime.UtcNow.ToString("o"), FixedDtNs = c.TickNs,
                Disturbance_Wind = "calm", Disturbance_Lighting = "GoldenHour" };
            source.FillManifest(m, prof, mission.Ctl);
            m.StateEverySubsteps = stateEvery;
            m.Controller = "Crane.Drones.Autonomy (" + c.Planner + ") -> SetpointArbiter -> InProcessControllerLink -> CraneMultirotorController velocity mode (500 Hz)";
            m.TrajectorySpec["kind"] = "autonomous_mission_v1";
            m.TrajectorySpec["mission"] = ms.Name;
            m.TrajectorySpec["route_global_enu"] = ms.Route.Select(p => p.ToArray()).ToList();
            m.TrajectorySpec["v_limit_mps"] = c.VLimit;
            m.TrajectorySpec["run_seed"] = (double)c.RunSeed;
            m.Extra["autonomy_config"] = c.Canonical();
            m.Extra["autonomy_config_sha256"] = AerialPlantLogSource.Sha256(c.Canonical());
            m.Extra["forest_manifest_path"] = fmRel;
            m.Extra["sensor_world"] = "unity PhysX colliders, layer Environment(9); ideal rays; manifest capsule lookup for thin-structure diameter";
            m.Extra["ground_truth"] = "manifest capsules (trunks, branches r>=" + world.HardRadius + " m hard; thinner = twig contacts) + Unity terrain height; body radius " + mission.BodyRadius.ToString("0.000", CI) + " m";
            m.Extra["lidar"] = "L2-like raycast model (Crane.Drones.Autonomy.LidarL2Sensor); scans summarised in events (autonomy_sensor), point files not written";
            m.Extra["cameras"] = "depth raycast image " + c.DepthWidth + "x" + c.DepthHeight + " summarised in events; images not written" + (render != null ? "; rendered stills in render_* dir" : "");
            m.Extra["brake_table"] = new Dictionary<string, object> { ["v"] = bt.vs, ["d"] = bt.ds };
            var session = DroneRunSession.Begin(dir, m, splits, allowHeldout, source, source);
            session.Writer.WriteEvent(0, "marker", new Dictionary<string, object> { ["name"] = "start" });
            rec.Attach(session);
            var flight = new AerialLoggedFlight(session, source);
            mission.StepPlant = () => flight.StepTick();
            render?.Begin(mission);
            var sw = System.Diagnostics.Stopwatch.StartNew();
            try {
                while (!mission.Done) {
                    mission.Step();
                    if (mission.ClearanceUpdated) { mission.ClearanceUpdated = false; session.Clearance.Update(mission.TNs, mission.LastHardClearance, mission.LastHardObject, session.Writer); }
                    render?.Tick(mission);
                    if (mission.Tick % 2000 == 0) Debug.Log($"CRANE_AUTONOMY_TICK run={runId} t={mission.TNs * 1e-9:0.0}s mode={mission.Arbiter.Mode} prog={mission.Result.ProgressM:0.0}m wall={sw.Elapsed.TotalSeconds:0.0}s");
                }
                long tEnd = Math.Max(mission.TNs, rec.Last);
                session.Writer.WriteEvent(tEnd, "marker", new Dictionary<string, object> { ["name"] = "end", ["outcome"] = mission.Result.Outcome });
                m.Extra["result"] = mission.Result.ToDict();
            } finally { session.Close(); }
            var d = mission.Result.ToDict();
            d["harness"] = "unity_collider_world"; d["run_dir"] = dir; d["wall_s_total"] = sw.Elapsed.TotalSeconds; d["ray_casts"] = sensorWorld.Casts;
            Debug.Log($"CRANE_AUTONOMY_RUN {runId} outcome={mission.Result.Outcome} t={mission.Result.TimeS:0.0} prog={mission.Result.ProgressFrac:0.00} minclr={mission.Result.MinClearanceHardM:0.00} wall={sw.Elapsed.TotalSeconds:0.0}s");
            return d;
        }

        static void Set(AutonomyConfig c, string kv) {
            var p = kv.Split('='); var f = typeof(AutonomyConfig).GetField(p[0]) ?? throw new ArgumentException("no config field " + p[0]);
            object v = f.FieldType == typeof(double) ? double.Parse(p[1], CI) : f.FieldType == typeof(int) ? int.Parse(p[1], CI) : f.FieldType == typeof(bool) ? bool.Parse(p[1])
                : f.FieldType == typeof(string) ? p[1] : f.FieldType == typeof(double[]) ? p[1].Split(':').Select(x => double.Parse(x, CI)).ToArray() : (object)ulong.Parse(p[1], CI);
            f.SetValue(c, v);
        }

        /// <summary>M6: hero quad posed from the plant, onboard OAK-D RGB (onboard volume only) and a third-person chase camera
        /// (cinematic volume only), still pairs written at the requested sim times. Rendering never feeds the stack.</summary>
        sealed class RenderRig : IDisposable {
            readonly string dir; readonly double[] times; int next; public readonly List<string> Written = new List<string>();
            QuadHandle quad; OnboardCamera rgb; Camera chase; RenderTexture chaseRt; Texture2D rgbTex, chaseTex;
            public RenderRig(string dir, double[] times) { this.dir = dir; this.times = times; Directory.CreateDirectory(dir); }

            public void Begin(AutonomousMission m) {
                AerialRobot.PrefabLoader = () => AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
                quad = new QuadHandle(new AerialRobot().CreateHero("AOS7 Quad (autonomy)"));
                foreach (var col in quad.Root.GetComponentsInChildren<Collider>(true)) col.enabled = false;   // never let the hero block sensor rays
                var forest = UnityEngine.Object.FindAnyObjectByType<DroneForestDirector>();
                var sceneProfiles = new List<VolumeProfile>();
                var sv = UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsInactive.Exclude);
                Array.Sort(sv, (x, y) => y.priority.CompareTo(x.priority));
                foreach (var v in sv) if (v.sharedProfile != null && v.gameObject.layer != RenderProfiles.CinematicLayer && v.weight > 0.01f) sceneProfiles.Add(v.sharedProfile);
                var onboard = RenderProfiles.BuildOnboard(sceneProfiles);
                var leak = RenderProfiles.CinematicOverrides(onboard);
                if (leak.Count > 0) throw new InvalidOperationException("onboard profile contains cinematic effects: " + string.Join(",", leak));
                RenderProfiles.MakeVolume("Autonomy Onboard Volume", onboard, RenderProfiles.OnboardLayer, 1000f);
                RenderProfiles.MakeVolume("Autonomy Cinematic Volume", RenderProfiles.BuildCinematic(), RenderProfiles.CinematicLayer, 1000f);
                rgb = new OnboardCamera("oakd_rgb_autonomy", quad.CameraMount, Vector3.zero, OakD.RgbHfovDeg, 960, 540, false);
                if (forest != null) DroneForestDirector.ConfigureCamera(rgb.Cam);
                RenderProfiles.ConfigureOnboardCamera(rgb.Cam, OakD.RgbHfovDeg, 960, 540);
                var go = new GameObject("Autonomy Chase Camera"); chase = go.AddComponent<Camera>(); chase.enabled = false;
                if (forest != null) DroneForestDirector.ConfigureCamera(chase);
                RenderProfiles.ConfigureCinematicCamera(chase, 1280, 720);
                chaseRt = new RenderTexture(1280, 720, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB); chaseRt.Create(); chase.targetTexture = chaseRt;
                rgbTex = new Texture2D(960, 540, TextureFormat.RGB24, false); chaseTex = new Texture2D(1280, 720, TextureFormat.RGB24, false);
                Pose(m, 0.04); for (int i = 0; i < 6; i++) { rgb.Render(); chase.Render(); }   // preroll (exposure settle), discarded
            }

            void Pose(AutonomousMission m, double dt) {
                var st = m.Plant.State;
                CraneAerialFrames.UnityLookAxes(st.Q, out double[] f, out double[] u);
                var p = CraneAerialFrames.EnuToUnity(st.P);
                var w = new float[st.RotorSpeed.Length]; for (int i = 0; i < w.Length; i++) w[i] = (float)Math.Abs(st.RotorSpeed[i]);
                quad.Pose(new TrajSample { t = st.T, pos = new Vector3((float)p[0], (float)p[1], (float)p[2]),
                    rot = Quaternion.LookRotation(new Vector3((float)f[0], (float)f[1], (float)f[2]), new Vector3((float)u[0], (float)u[1], (float)u[2])), rotorRadS = w }, dt);
            }

            public void Tick(AutonomousMission m) {
                double t = m.TNs * 1e-9;
                if (next >= times.Length || (t < times[next] && !m.Done)) return;
                next++;
                Pose(m, 0.005); Physics.SyncTransforms();
                var body = quad.Root.transform;
                var back = -body.forward; back.y = 0; back = back.sqrMagnitude > 1e-4 ? back.normalized : Vector3.back;
                chase.transform.position = body.position + back * 4.0f + Vector3.up * 1.6f;
                chase.transform.LookAt(body.position + Vector3.up * 0.2f);
                rgb.Render(); chase.Render();
                string tag = $"t{t:000.0}s";
                Save(rgb.Rt, rgbTex, Path.Combine(dir, $"onboard_rgb_{tag}.png"));
                Save(chaseRt, chaseTex, Path.Combine(dir, $"third_person_{tag}.png"));
            }

            void Save(RenderTexture rt, Texture2D tex, string path) {
                var prev = RenderTexture.active; RenderTexture.active = rt;
                tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0, false); tex.Apply(false);
                RenderTexture.active = prev;
                File.WriteAllBytes(path, tex.EncodeToPNG()); Written.Add(path);
            }

            public void Dispose() {
                rgb?.Dispose(); if (chaseRt != null) { chaseRt.Release(); UnityEngine.Object.DestroyImmediate(chaseRt); }
                if (chase != null) UnityEngine.Object.DestroyImmediate(chase.gameObject);
                if (quad != null) UnityEngine.Object.DestroyImmediate(quad.Root);
            }
        }
    }
}
