using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater {
    /// <summary>
    /// Ghost theater: plays a manifest of recorded runs back as one deterministic video, offline.
    ///   CRANE -batchmode --crane-disable-ros --crane-disable-sitl [world args from the manifest]
    ///     --crane-theater MANIFEST --crane-theater-out DIR [--crane-theater-script ACTS.json]
    ///     [--crane-theater-frames A:B] [--crane-theater-seconds S] [--crane-theater-fps F] [--crane-theater-size WxH]
    ///     [--crane-theater-ffmpeg PATH|none] [--crane-theater-crf 18] [--crane-theater-preset medium]
    ///     [--crane-theater-preroll-s 2] [--crane-theater-quality N] [--crane-theater-stills N]
    /// Writes DIR/video.mp4 (frames A..B-1), DIR/frames.jsonl (per frame: act, state digest, pixel sha256,
    /// camera), DIR/render_report.json (checks for the inspectors) and, with --crane-theater-stills N, a PNG
    /// every N frames in DIR/stills. Every time source is the frame index: frame f is at f / fps; the world's
    /// time (water) is set per frame, motion blur is off, exposure adapts without history, the camera uses SMAA
    /// (no temporal jitter), and a chunk is preceded by preroll frames that are rendered and discarded.
    /// </summary>
    public sealed class TheaterBoot : MonoBehaviour {
        static string[] Args => Environment.GetCommandLineArgs();

        static string Arg(string key, string fallback = null) {
            var a = Args;
            for (int i = 0; i + 1 < a.Length; i++) if (a[i] == key) return a[i + 1];
            return fallback;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Install() {
            if (Arg("--crane-theater") == null) return;
            var go = new GameObject("CRANE Theater");
            DontDestroyOnLoad(go);
            go.AddComponent<TheaterBoot>();
        }

        int errors;
        readonly List<string> errorLines = new List<string>();

        void Awake() {
            Application.logMessageReceived += (msg, stack, type) => {
                if (type != LogType.Error && type != LogType.Exception && type != LogType.Assert) return;
                errors++;
                if (errorLines.Count < 40) errorLines.Add(msg.Length > 400 ? msg.Substring(0, 400) : msg);
            };
            Time.timeScale = 0f;   // nothing simulates; the theater only poses
        }

        IEnumerator Start() {
            int code = 0;
            IEnumerator run = Run(c => code = c);
            while (true) {
                object cur;
                try {
                    if (!run.MoveNext()) break;
                    cur = run.Current;
                }
                catch (Exception e) {
                    Debug.LogError($"CRANE_THEATER_FAILED {e}");
                    WriteFailure(e);
                    code = 3;
                    break;
                }
                yield return cur;
            }
            Debug.Log($"CRANE_THEATER_EXIT code={code}");
            Application.Quit(code);
        }

        string outDir;

        void WriteFailure(Exception e) {
            try {
                if (outDir == null) return;
                File.WriteAllText(Path.Combine(outDir, "render_report.json"), TJson.Write(new SortedDictionary<string, object> {
                    { "schema", "crane-theater-report/1" }, { "ok", false }, { "error", e.ToString() }, { "errors", errors },
                    { "error_lines", errorLines }
                }) + "\n");
            }
            catch (Exception) { }
        }

        IEnumerator Run(Action<int> exit) {
            string manifestPath = Path.GetFullPath(Arg("--crane-theater"));
            outDir = Path.GetFullPath(Arg("--crane-theater-out", Path.Combine(Path.GetDirectoryName(manifestPath) ?? ".", "render")));
            Directory.CreateDirectory(outDir);
            int quality = int.Parse(Arg("--crane-theater-quality", "-1"), CultureInfo.InvariantCulture);
            if (quality >= 0 && quality < QualitySettings.names.Length) QualitySettings.SetQualityLevel(quality, true);
            for (int i = 0; i < 10; i++) yield return null;   // scene start and the course bootstrap

            var wall = System.Diagnostics.Stopwatch.StartNew();
            var manifest = TheaterManifest.Load(manifestPath);
            string worldName = TJson.Str(manifest.World, "plugin", "plane"), robotName = TJson.Str(manifest.Robot, "plugin", "arrow");
            ITheaterWorld world = TheaterPlugins.Worlds.TryGetValue(worldName, out var wf) ? wf() : new PlaneWorld();
            ITheaterRobot robot = TheaterPlugins.Robots.TryGetValue(robotName, out var rf) ? rf() : new ArrowRobot();
            yield return world.Prepare(manifest);

            TheaterScript script;
            string scriptPath = Arg("--crane-theater-script");
            if (scriptPath != null) script = TheaterScript.Parse(TJson.Parse(File.ReadAllText(scriptPath)));
            else if (manifest.Script != null) script = TheaterScript.Parse(manifest.Script);
            else script = TheaterScript.Default(manifest, float.Parse(Arg("--crane-theater-seconds", "15"), CultureInfo.InvariantCulture));
            string fpsArg = Arg("--crane-theater-fps"), sizeArg = Arg("--crane-theater-size");
            if (fpsArg != null) script.Fps = float.Parse(fpsArg, CultureInfo.InvariantCulture);
            if (sizeArg != null) { var p = sizeArg.Split('x'); script.Width = int.Parse(p[0]); script.Height = int.Parse(p[1]); }
            if (string.IsNullOrEmpty(script.Title)) script.Title = manifest.Study;

            var state = new TheaterState(manifest, script, world);
            int total = state.FrameCount, a = 0, b = total;
            string frames = Arg("--crane-theater-frames");
            if (frames != null) {
                var p = frames.Split(':');
                a = Mathf.Clamp(int.Parse(p[0]), 0, total);
                b = p.Length > 1 && p[1].Length > 0 ? Mathf.Clamp(int.Parse(p[1]), a, total) : total;
            }
            int preroll = Mathf.RoundToInt(float.Parse(Arg("--crane-theater-preroll-s", "2"), CultureInfo.InvariantCulture) * script.Fps);
            int first = Mathf.Max(0, a - preroll);
            int stills = int.Parse(Arg("--crane-theater-stills", "0"), CultureInfo.InvariantCulture);
            Debug.Log($"CRANE_THEATER_START manifest={manifestPath} runs={manifest.Runs.Count} world={world.Describe} robot={robotName} " +
                      $"acts={script.Acts.Count} frames={a}:{b}/{total} fps={script.Fps} size={script.Width}x{script.Height} " +
                      $"gfx={SystemInfo.graphicsDeviceType} device={SystemInfo.graphicsDeviceName} quality={QualitySettings.names[QualitySettings.GetQualityLevel()]}");

            // determinism: no motion blur, exposure without temporal adaptation
            var volGo = new GameObject("Theater Volume");
            var vol = volGo.AddComponent<Volume>();
            vol.isGlobal = true; vol.priority = 10000;
            var profile = ScriptableObject.CreateInstance<VolumeProfile>();
            var exposure = profile.Add<Exposure>(true);
            exposure.adaptationMode.Override(AdaptationMode.Fixed);
            var blur = profile.Add<MotionBlur>(true);
            blur.intensity.Override(0f);
            vol.sharedProfile = profile;

            foreach (var c in FindObjectsByType<Camera>(FindObjectsInactive.Exclude)) c.enabled = false;
            var camGo = new GameObject("Theater Camera");
            var cam = camGo.AddComponent<Camera>();
            var hd = camGo.AddComponent<HDAdditionalCameraData>();
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            hd.SMAAQuality = HDAdditionalCameraData.SMAAQualityLevel.High;
            hd.allowDynamicResolution = false;
            cam.enabled = false;
            cam.nearClipPlane = 0.2f; cam.farClipPlane = 5000f;
            var rt = new RenderTexture(script.Width, script.Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1, name = "theater frame" };
            rt.Create();
            cam.targetTexture = rt;
            cam.aspect = script.Width / (float)script.Height;

            var renderer = new TheaterRenderer(state, robot) { Camera = cam };
            var passGo = new GameObject("Theater Pass");
            var cpv = passGo.AddComponent<CustomPassVolume>();
            cpv.isGlobal = true;
            cpv.targetCamera = cam;
            cpv.injectionPoint = CustomPassInjectionPoint.AfterPostProcess;
            cpv.customPasses.Add(new TheaterPass { R = renderer, name = "ghost theater" });
            var hud = new HudRenderer(script.Width, script.Height);

            GameObject gold = null, silver = null;
            bool heroes = false;
            foreach (var p in state.Plans) heroes |= p.hero >= 0 || p.previous >= 0;
            if (heroes) {
                gold = robot.CreateHero("Theater hero gold"); silver = robot.CreateHero("Theater hero silver");
                gold.SetActive(false); silver.SetActive(false);
            }

            string video = Path.Combine(outDir, "video.mp4");
            string ffmpeg = Arg("--crane-theater-ffmpeg", "ffmpeg");
            var capture = new TheaterCapture(ffmpeg, video, script.Width, script.Height, script.Fps,
                int.Parse(Arg("--crane-theater-crf", "18"), CultureInfo.InvariantCulture), Arg("--crane-theater-preset", "medium"));
            if (stills > 0) Directory.CreateDirectory(Path.Combine(outDir, "stills"));
            Time.captureDeltaTime = 1f / script.Fps;

            var log = new StringBuilder();
            var report = new Report(state, outDir);
            var renderMs = new List<double>();
            var sw = new System.Diagnostics.Stopwatch();
            for (int f = first; f < b; f++) {
                sw.Restart();
                var fs = state.Evaluate(f);
                float t = state.ActRunTime(fs.plan, fs.tau, out _);
                world.SetTime(t);
                PlaceHero(gold, fs, fs.plan.hero, robot);
                PlaceHero(silver, fs, fs.plan.previous, robot);
                cam.transform.SetPositionAndRotation(fs.camPos, fs.camRot);
                cam.fieldOfView = fs.fov;
                renderer.Prepare(fs);
                cam.Render();
                hud.Draw(rt, fs.hud);
                if (f >= a) {
                    string pix = capture.Write(rt, f);
                    if (stills > 0 && (f - a) % stills == 0) capture.WritePng(Path.Combine(outDir, "stills", $"frame_{f:D6}.png"));
                    string digest = fs.Digest();
                    report.Frame(fs, digest);
                    var cp = fs.camPos;
                    log.Append("{\"frame\":").Append(f).Append(",\"act\":").Append(fs.act).Append(",\"tau\":").Append(TJson.F(fs.tau))
                        .Append(",\"t\":").Append(TJson.F(t)).Append(",\"iteration\":").Append(fs.iteration)
                        .Append(",\"ghosts\":").Append(fs.ghosts.Count).Append(",\"state\":\"").Append(digest)
                        .Append("\",\"pixels\":\"").Append(pix).Append("\",\"cam\":[").Append(TJson.F(cp.x)).Append(',')
                        .Append(TJson.F(cp.y)).Append(',').Append(TJson.F(cp.z)).Append("]}\n");
                    renderMs.Add(sw.Elapsed.TotalMilliseconds);
                    if ((f - a) % 60 == 0) Debug.Log($"CRANE_THEATER_FRAME {f}/{b} act={fs.act} ghosts={fs.ghosts.Count} ms={sw.Elapsed.TotalMilliseconds:0.0}");
                }
                yield return null;
            }
            bool encoded = capture.Finish();
            File.WriteAllText(Path.Combine(outDir, "frames.jsonl"), log.ToString());
            renderMs.Sort();
            var doc = report.Build();
            doc["ok"] = encoded && capture.Frames == b - a;
            doc["frames"] = new List<object> { a, b };
            doc["frames_written"] = capture.Frames;
            doc["total_frames"] = total;
            doc["fps"] = (double)script.Fps;
            doc["size"] = new List<object> { script.Width, script.Height };
            doc["preroll_frames"] = a - first;
            doc["video"] = ffmpeg == "none" ? "frames/" : "video.mp4";
            doc["ffmpeg_log"] = capture.FfmpegLog;
            doc["gfx"] = SystemInfo.graphicsDeviceType.ToString();
            doc["device"] = SystemInfo.graphicsDeviceName;
            doc["os"] = SystemInfo.operatingSystem;
            doc["unity"] = Application.unityVersion;
            doc["quality"] = QualitySettings.names[QualitySettings.GetQualityLevel()];
            doc["world"] = world.Describe;
            doc["errors"] = errors;
            doc["error_lines"] = errorLines;
            doc["render_ms_median"] = renderMs.Count > 0 ? renderMs[renderMs.Count / 2] : 0.0;
            doc["render_ms_p95"] = renderMs.Count > 0 ? renderMs[(int)(renderMs.Count * 0.95)] : 0.0;
            doc["wall_s"] = wall.Elapsed.TotalSeconds;
            doc["script"] = ScriptDoc(script);
            foreach (var kv in renderer.Stats) doc["stat_" + kv.Key] = kv.Value;
            File.WriteAllText(Path.Combine(outDir, "render_report.json"), TJson.Write(doc) + "\n");
            Debug.Log($"CRANE_THEATER_DONE frames={capture.Frames} video={video} encoded={encoded} errors={errors} wall_s={wall.Elapsed.TotalSeconds:0.0}");
            renderer.Dispose(); hud.Dispose(); capture.Dispose();
            exit(encoded && capture.Frames == b - a ? 0 : 4);
        }

        static void PlaceHero(GameObject hero, FrameState fs, int run, ITheaterRobot robot) {
            if (hero == null) return;
            bool on = false;
            foreach (var g in fs.heroes) {
                if (g.run != run) continue;
                var q = Quaternion.Euler(g.pitch, g.yaw, g.roll);
                hero.transform.SetPositionAndRotation(g.pos, robot.HeroRotation(q));
                on = true;
            }
            if (hero.activeSelf != on) hero.SetActive(on);
        }

        static List<object> ScriptDoc(TheaterScript s) {
            var acts = new List<object>();
            foreach (var a in s.Acts)
                acts.Add(new SortedDictionary<string, object> {
                    { "type", a.type }, { "duration_s", (double)a.duration }, { "speed", (double)a.speed }, { "colour", a.colour },
                    { "trails", a.trails }, { "heat", a.heat }, { "shot", a.Shot }, { "caption", a.caption }
                });
            return acts;
        }

        /// <summary>The checks inspectors read: per act, ghosts seen, poses that are NaN, off the world, or jump
        /// further than the run could travel in a frame; whether the iteration counter only climbs; events off
        /// their run's path.</summary>
        sealed class Report {
            readonly TheaterState st;
            readonly Dictionary<int, Vector3> last = new Dictionary<int, Vector3>();
            int lastAct = -1, lastIteration = int.MinValue, lastFrame = -1;
            readonly SortedDictionary<string, object> acts = new SortedDictionary<string, object>();
            int nan, offWorld, teleports, counterBackwards, hudMismatch;
            readonly List<object> samples = new List<object>();

            public Report(TheaterState s, string dir) { st = s; }

            public void Frame(FrameState fs, string digest) {
                string key = fs.act.ToString("D2", CultureInfo.InvariantCulture);
                if (!acts.TryGetValue(key, out var o)) acts[key] = o = new SortedDictionary<string, object> {
                    { "type", fs.plan.act.type }, { "frames", 0 }, { "ghosts_min", int.MaxValue }, { "ghosts_max", 0 },
                    { "runs", fs.plan.runs.Count }, { "iterations", fs.plan.iterations.Length },
                    { "hero", fs.plan.hero >= 0 ? st.M.Runs[fs.plan.hero].id : null },
                    { "previous", fs.plan.previous >= 0 ? st.M.Runs[fs.plan.previous].id : null }
                };
                var d = (SortedDictionary<string, object>)o;
                d["frames"] = (int)d["frames"] + 1;
                d["ghosts_min"] = Math.Min((int)d["ghosts_min"], fs.ghosts.Count);
                d["ghosts_max"] = Math.Max((int)d["ghosts_max"], fs.ghosts.Count);
                bool contiguous = fs.act == lastAct && fs.frame == lastFrame + 1;
                if (!contiguous || fs.iteration != lastIteration) last.Clear();
                if (fs.plan.act.type == "evolution" && contiguous && fs.iteration < lastIteration) counterBackwards++;
                var bounds = st.World.Bounds; bounds.Expand(new Vector3(30, 1000, 30));
                float maxStep = 6f * fs.plan.act.speed / st.S.Fps + 1.5f;   // 6 m/s
                foreach (var g in fs.ghosts) {
                    if (float.IsNaN(g.pos.x) || float.IsNaN(g.pos.z) || float.IsNaN(g.yaw)) { nan++; continue; }
                    if (!bounds.Contains(g.pos)) offWorld++;
                    if (last.TryGetValue(g.run, out var prev) && Vector3.Distance(prev, g.pos) > maxStep) teleports++;
                    last[g.run] = g.pos;
                }
                if (fs.hud.running + fs.hud.finished + fs.hud.failed > fs.hud.runs) hudMismatch++;
                lastAct = fs.act; lastIteration = fs.iteration; lastFrame = fs.frame;
                if (fs.frame % Mathf.Max(1, Mathf.RoundToInt(st.S.Fps * 5)) == 0)
                    samples.Add(new SortedDictionary<string, object> {
                        { "frame", fs.frame }, { "act", fs.act }, { "iteration", fs.hud.iteration }, { "runs", fs.hud.runs },
                        { "running", fs.hud.running }, { "finished", fs.hud.finished }, { "failed", fs.hud.failed },
                        { "best", float.IsNaN(fs.hud.best) ? null : (object)(double)fs.hud.best }, { "state", digest }
                    });
            }

            public SortedDictionary<string, object> Build() {
                int offPath = 0, events = 0;
                foreach (var r in st.M.Runs)
                    foreach (var e in r.events) {
                        if (float.IsNaN(e.x)) continue;
                        events++;
                        var q = st.M.Pack.At(r.index, e.t);
                        if (Mathf.Sqrt((q.x - e.x) * (q.x - e.x) + (q.y - e.y) * (q.y - e.y)) > 3f) offPath++;
                    }
                return new SortedDictionary<string, object> {
                    { "schema", "crane-theater-report/1" }, { "acts", acts }, { "runs", st.M.Runs.Count },
                    { "checks", new SortedDictionary<string, object> {
                        { "nan_poses", nan }, { "off_world_poses", offWorld }, { "teleports", teleports },
                        { "iteration_counter_backwards", counterBackwards }, { "hud_count_mismatch", hudMismatch },
                        { "events", events }, { "events_off_path", offPath } } },
                    { "hud_samples", samples }
                };
            }
        }
    }
}
