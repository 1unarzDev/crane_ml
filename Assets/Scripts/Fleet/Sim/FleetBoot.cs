using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Fleet.Sim {
    /// <summary>Command-line options shared by the editor CLI and the player.</summary>
    public static class FleetArgs {
        public static string Get(string key, string fallback = null) {
            string[] a = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < a.Length; i++) if (a[i] == key) return a[i + 1];
            return fallback;
        }

        public static bool Has(string key) => Array.IndexOf(Environment.GetCommandLineArgs(), key) >= 0;

        public static string SpecDir(string fallback) => Get("--crane-fleet-spec", fallback);

        public static FleetRunOptions RunOptions(string launcher, FleetSpec spec = null) {
            var o = new FleetRunOptions { Launcher = launcher };
            o.Seed = long.Parse(Get("--crane-seed", "1"), CultureInfo.InvariantCulture);
            o.Variant = Get("--crane-fleet-variant");
            o.OutRoot = Get("--crane-fleet-out-root");
            o.OutDir = Get("--crane-fleet-out");
            string door = Get("--crane-fleet-doorway");
            if (door != null) o.DoorwayWidth = double.Parse(door, CultureInfo.InvariantCulture);
            string dur = Get("--crane-fleet-duration");
            if (dur != null) o.MaxDuration = double.Parse(dur, CultureInfo.InvariantCulture);
            o.SourceCommit = Get("--crane-source-commit", "");
            o.Failures = Get("--crane-fleet-failures", "");
            o.SourceDirty = string.Equals(Get("--crane-source-dirty", "false"), "true", StringComparison.OrdinalIgnoreCase);
            string mission = Get("--crane-fleet-mission", "qual");
            if (spec != null && mission == "demo") {
                o.Mission = new DemoMission(spec);
                if (dur == null) o.MaxDuration = spec.O["demo.max_duration_s"].Double;
            }
            if (spec != null && mission == "autonomy") {
                string mf = Get("--crane-fleet-mission-file") ?? throw new ArgumentException("--crane-fleet-mission autonomy needs --crane-fleet-mission-file");
                o.Mission = new AutonomyMission(spec, mf);
                var root = JsonNode.FromText(System.IO.File.ReadAllText(mf));
                if (dur == null) o.MaxDuration = root.Has("crane_max_duration_s") ? root["crane_max_duration_s"].Double : 300.0;
            }
            return o;
        }
    }

    /// <summary>
    /// Player entry: <c>--crane-fleet-run</c> (seeded run, works with -nographics), <c>--crane-fleet-check REPORT.json</c> (CAD-swap check) or
    /// <c>--crane-fleet-still DIR</c> (the same seeded run with graphics: the indoor look and the overlay's shots, triggered by time or by run
    /// events, written as PNG; <c>--crane-fleet-clip</c> adds the clip frames). <c>--crane-fleet-mission demo</c> selects the demo mission.
    /// </summary>
    public sealed class FleetBoot : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Install() {
            if (!FleetArgs.Has("--crane-fleet-run") && FleetArgs.Get("--crane-fleet-check") == null && FleetArgs.Get("--crane-fleet-still") == null
                && FleetArgs.Get("--crane-fleet-jobs") == null) return;
            var go = new GameObject("CRANE fleet boot");
            DontDestroyOnLoad(go);
            go.AddComponent<FleetBoot>();
        }

        IEnumerator Start() {
            int code = 0;
            IEnumerator body = Body(c => code = c);
            while (true) {
                object cur;
                try {
                    if (!body.MoveNext()) break;
                    cur = body.Current;
                }
                catch (Exception e) {
                    Debug.LogError("CRANE_FLEET_FAILED " + e);
                    code = 3;
                    break;
                }
                yield return cur;
            }
            Debug.Log($"CRANE_FLEET_EXIT code={code}");
            Application.Quit(code);
        }

        IEnumerator Body(Action<int> exit) {
            yield return null;
            var spec = FleetSpec.LoadDirectory(FleetArgs.SpecDir(Path.Combine(Application.streamingAssetsPath, "Fleet")));
            Debug.Log($"CRANE_FLEET_SPEC profile={spec.Profile.Sha256} scenario={spec.Scenario.Sha256} overlay={spec.Overlay.Sha256} gfx={SystemInfo.graphicsDeviceType}");
            string check = FleetArgs.Get("--crane-fleet-check");
            if (check != null) {
                var r = CadSwapCheck.Run(spec, check);
                Debug.Log($"CRANE_FLEET_CHECK pass={r.Pass} report={check} failures={r.Failures.Count}");
                foreach (string f in r.Failures) Debug.Log("CRANE_FLEET_CHECK_FAIL " + f);
                exit(r.Pass ? 0 : 1);
                yield break;
            }
            string jobs = FleetArgs.Get("--crane-fleet-jobs");
            if (jobs != null) {
                string specDir = FleetArgs.SpecDir(Path.Combine(Application.streamingAssetsPath, "Fleet"));
                string commit = FleetArgs.Get("--crane-source-commit", "");
                bool dirty = FleetArgs.Get("--crane-source-dirty", "false") == "true";
                var res = FleetHarness.RunHeadless(specDir, jobs, "player", commit, dirty);
                if (SystemInfo.graphicsDeviceType != UnityEngine.Rendering.GraphicsDeviceType.Null) {
                    var root = JsonNode.FromText(File.ReadAllText(jobs));
                    var byId = new Dictionary<string, JsonNode>();
                    foreach (var j in root["jobs"].Items) byId[j["id"].String] = j;
                    foreach (var r in res)
                        if (!r.Success) yield return FleetHarness.RenderReplay(specDir, byId[r.Id], r, Path.GetFullPath(root["out"].String), "player", commit, dirty);
                }
                exit(0);
                yield break;
            }
            string still = FleetArgs.Get("--crane-fleet-still");
            var opt = FleetArgs.RunOptions("player", spec);
            if (still != null && opt.OutDir == null) opt.OutDir = Path.Combine(still, "run");
            var run = new FleetRun(spec, opt);
            var shots = still != null ? new FleetShots(spec, run, still, opt.Mission is AutonomyMission ? "autonomy_shots" : opt.Mission != null ? "demo_shots" : "stills") : null;
            run.Setup();
            if (shots != null) shots.Begin();
            Debug.Log($"CRANE_FLEET_RUN_START id={run.RunId} dir={run.Dir} physics={run.Phys.Describe}");
            while (true) {
                if (shots != null) while (shots.Due()) yield return shots.RenderNext();
                if (!run.Advance()) break;
                if (shots == null && run.Step % 4000 == 0) yield return null;
            }
            if (shots != null) while (shots.Pending) yield return shots.RenderNext(force: true);
            run.Finish();
            shots?.Finish();
            Debug.Log($"CRANE_FLEET_RUN_END id={run.RunId} reason={run.EndReason} t={run.T} steps={run.Step} dir={run.Dir}");
            exit(0);
        }
    }

    /// <summary>
    /// Shots from the overlay (<c>stills</c> or <c>demo_shots</c>): each has a trigger (<c>t</c>, or <c>event</c> with optional rover / field
    /// match and an offset in run seconds), a camera (absolute <c>position</c>/<c>look_at</c>; <c>target</c> = rover:id[:frame] or payload with
    /// world offsets; or <c>sensor</c> = rover:frame for an onboard view), the profile (cinematic or sensor), fov and aperture. Rendered
    /// offscreen through HDRP into a RenderTexture and written as PNG; a JSON line per still records time, trigger and the logged state names.
    /// </summary>
    public sealed class FleetShots {
        sealed class Shot {
            public string Name, Event, Rover, Field, Value, Profile = "cinematic";
            public double T = double.NaN, Offset, Fov = 45, Aperture, Due = double.NaN;
            public JsonNode Camera;
            public bool Clip, Done;
            public double T1, Fps, NextFrame; public int Frames;
        }

        readonly FleetSpec spec;
        readonly FleetRun run;
        readonly string dir;
        readonly List<Shot> shots = new List<Shot>();
        readonly int w, h;
        readonly StreamWriter index;
        readonly bool look;
        readonly bool clips;

        public FleetShots(FleetSpec spec, FleetRun run, string dir, string section) : this(spec, run, dir, spec.O[section]) { }

        public FleetShots(FleetSpec spec, FleetRun run, string dir, JsonNode S) {
            this.spec = spec; this.run = run; this.dir = dir;
            Directory.CreateDirectory(dir);
            double[] size = S["size"].Doubles;
            double scale = double.Parse(FleetArgs.Get("--crane-fleet-still-scale", "1"), CultureInfo.InvariantCulture);
            w = (int)(size[0] * scale); h = (int)(size[1] * scale);
            look = S.Has("look") && S["look"].Bool;
            clips = FleetArgs.Has("--crane-fleet-clip");
            foreach (var n in S["shots"].Items) {
                var s = new Shot { Name = n["name"].String, Camera = n["camera"], Fov = n.Has("fov") ? n["fov"].Double : 45, Aperture = n.Has("aperture") ? n["aperture"].Double : 0,
                    Profile = n.Has("profile") ? n["profile"].String : "cinematic", Offset = n.Has("offset") ? n["offset"].Double : 0 };
                if (n.Has("t")) s.T = s.Due = s.NextFrame = n["t"].Double;
                if (n.Has("event")) { s.Event = n["event"].String; if (n.Has("rover")) s.Rover = n["rover"].String; if (n.Has("field")) { s.Field = n["field"].String; s.Value = n["value"].String; } }
                if (n.Has("clip")) { if (!clips) continue; s.Clip = true; s.T1 = n["clip.duration_s"].Double; s.Fps = n["clip.fps"].Double; }
                shots.Add(s);
            }
            index = new StreamWriter(Path.Combine(dir, "stills.jsonl"), false) { NewLine = "\n" };
            run.EventRaised += OnEvent;
        }

        public void Begin() {
            if (look) FleetLook.Apply(run.World, spec);
            Debug.Log($"CRANE_FLEET_SHOTS n={shots.Count} size={w}x{h} look={look} clips={clips}");
        }

        void OnEvent(string kind, string rover, Dictionary<string, object> f) {
            foreach (var s in shots) {
                if (s.Event == null || !double.IsNaN(s.Due) || s.Event != kind) continue;
                if (s.Rover != null && s.Rover != rover) continue;
                if (s.Field != null && !(f.TryGetValue(s.Field, out var v) && Convert.ToString(v, CultureInfo.InvariantCulture) == s.Value)) continue;
                s.Due = run.T + s.Offset;
                if (s.Clip) s.NextFrame = s.Due;
            }
        }

        public bool Pending { get { foreach (var s in shots) if (!s.Done && !double.IsNaN(s.Due)) return true; return false; } }

        public bool Due() {
            foreach (var s in shots) if (!s.Done && !double.IsNaN(s.Due) && run.T >= (s.Clip ? s.NextFrame : s.Due) - 1e-9) return true;
            return false;
        }

        public IEnumerator RenderNext(bool force = false) {
            foreach (var s in shots) {
                if (s.Done || double.IsNaN(s.Due)) continue;
                if (!force && run.T < (s.Clip ? s.NextFrame : s.Due) - 1e-9) continue;
                if (s.Clip) {
                    string d = Path.Combine(dir, s.Name);
                    Directory.CreateDirectory(d);
                    yield return Render(s, Path.Combine(d, $"frame_{s.Frames:D5}.png"), warm: s.Frames == 0 ? 8 : 1);
                    s.Frames++;
                    s.NextFrame = s.Due + s.Frames / s.Fps;
                    if (force || s.NextFrame > s.Due + s.T1) s.Done = true;
                }
                else {
                    yield return Render(s, Path.Combine(dir, s.Name + ".png"), warm: 12);
                    s.Done = true;
                }
                yield break;
            }
        }

        IEnumerator Render(Shot s, string path, int warm) {
            var go = new GameObject("shot camera " + s.Name);
            var cam = go.AddComponent<Camera>();
            go.AddComponent<HDAdditionalCameraData>();
            cam.enabled = false;
            Vector3 p, l;
            double hfov = double.NaN;
            if (s.Camera.Has("sensor")) {
                var parts = s.Camera["sensor"].String.Split(':');
                var frame = run.Rover(parts[0]).Frames["sensor_" + parts[1]];
                p = frame.position; l = p + frame.forward;
                go.transform.SetPositionAndRotation(p, frame.rotation);
                hfov = spec.P["sensors.mono_camera_ov9281.hfov"].Double;
                var missing = FleetLook.ConfigureSensor(cam, w, h, hfov);
                if (missing.Count > 0) Debug.Log("CRANE_FLEET_SENSOR_FRAMESETTING_MISSING " + string.Join(",", missing));
            }
            else {
                Vector3 anchor = Vector3.zero;
                if (s.Camera.Has("target")) anchor = Target(s.Camera["target"].String);
                p = anchor + Off(s.Camera, "position");
                l = anchor + Off(s.Camera, "look_at");
                go.transform.SetPositionAndRotation(p, Quaternion.LookRotation(l - p, Vector3.up));
                cam.fieldOfView = (float)s.Fov;
                FleetLook.ConfigureCinematic(cam, w, h, s.Fov, s.Aperture, (l - p).magnitude);
            }
            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1 };
            rt.Create();
            cam.targetTexture = rt;
            for (int i = 0; i < warm; i++) { cam.Render(); yield return null; }
            cam.Render();
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0, false);
            tex.Apply();
            RenderTexture.active = prev;
            File.WriteAllBytes(path, tex.EncodeToPNG());
            var rec = new Dictionary<string, object> { ["shot"] = s.Name, ["file"] = Path.GetFileName(path), ["t"] = run.T, ["step"] = run.Step,
                ["trigger"] = s.Event != null ? s.Event + (s.Value != null ? ":" + s.Value : "") + "+" + s.Offset.ToString(CultureInfo.InvariantCulture) : "t",
                ["profile"] = s.Camera.Has("sensor") ? "sensor" : "cinematic", ["camera"] = new[] { (double)p.x, p.z, p.y }, ["look_at"] = new[] { (double)l.x, l.z, l.y },
                ["fov"] = s.Fov, ["size"] = new[] { w, h } };
            if (run.Opt.Mission is DemoMission dm) {
                rec["connector"] = dm.ConnectorState;
                rec["hook_pos"] = dm.HookPos;
                rec["payload_mode"] = dm.PayloadMode;
                var steps = new Dictionary<string, object>();
                foreach (var r in run.Rovers) steps[r.D.Id] = dm.StepOf(r.D.Id);
                rec["steps"] = steps;
                var ctl = new Dictionary<string, object>();
                foreach (var r in run.Rovers) ctl[r.D.Id] = dm.ControllerOf(r.D.Id);
                rec["controllers"] = ctl;
            }
            if (run.Opt.Mission is AutonomyMission au) {
                rec["controller"] = "closed_loop (Crane.Fleet.Autonomy)";
                var steps = new Dictionary<string, object>();
                foreach (var r in run.Rovers) steps[r.D.Id] = au.StepOf(r.D.Id);
                rec["tasks"] = steps;
                rec["payload_mode"] = au.PayloadMode;
                var c0 = au.Couplings.All.Count > 0 ? au.Couplings.All[0] : null;
                rec["connector"] = c0 != null ? Crane.Fleet.Docking.ConnectorStateNames.ToProfileName(c0.Machine.State) : "FREE";
            }
            if (s.Camera.Has("sensor")) rec["sensor_profile_violations"] = FleetLook.Sensor != null ? FleetLook.SensorProfileViolations(FleetLook.Sensor) : new List<string> { "no sensor profile (look off)" };
            index.WriteLine(FleetJson.Write(rec));
            index.Flush();
            if (!s.Clip) Debug.Log($"CRANE_FLEET_STILL {path} {w}x{h} t={run.T:0.00}");
            UnityEngine.Object.Destroy(tex);
            cam.targetTexture = null;
            rt.Release();
            UnityEngine.Object.Destroy(go);
        }

        Vector3 Target(string t) {
            var parts = t.Split(':');
            if (parts[0] == "payload" && run.Opt.Mission is DemoMission dm) return dm.Payload.worldCenterOfMass;
            if (parts[0] == "payload" && run.Opt.Mission is AutonomyMission am && am.Payload != null) return am.Payload.worldCenterOfMass;
            if (parts[0] == "rover") {
                var r = run.Rover(parts[1]);
                return parts.Length > 2 ? r.Frames[parts[2]].position : r.Body.position;
            }
            throw new ArgumentException("unknown camera target " + t);
        }

        static Vector3 Off(JsonNode cam, string key) { double[] v = cam[key].Doubles; return FleetFrames.World(v[0], v[1], v[2]); }

        public void Finish() {
            run.EventRaised -= OnEvent;
            index.Dispose();
            foreach (var s in shots) {
                if (!s.Clip || s.Frames == 0) continue;
                string d = Path.Combine(dir, s.Name), ff = FleetArgs.Get("--crane-fleet-ffmpeg", "ffmpeg");
                try {
                    var p = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(ff,
                        $"-y -loglevel error -framerate {s.Fps.ToString(CultureInfo.InvariantCulture)} -i {d}/frame_%05d.png -c:v libx264 -pix_fmt yuv420p -crf 20 {d}.mp4") { UseShellExecute = false });
                    p.WaitForExit();
                    Debug.Log($"CRANE_FLEET_CLIP {d}.mp4 frames={s.Frames} exit={p.ExitCode}");
                }
                catch (Exception e) { Debug.Log($"CRANE_FLEET_CLIP_FAILED {e.Message}"); }
            }
        }
    }
}
