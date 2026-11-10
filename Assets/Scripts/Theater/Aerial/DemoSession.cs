using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Crane.Drones.Environment;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater.Aerial {
    public sealed class DemoOptions {
        public string sequencePath, outDir, runId, ffmpeg = "ffmpeg", profile = "both";
        public int seed = 1, crf = 20, firstFrame = 0, lastFrame = int.MaxValue, preroll = 8;
        public float maxMegabytes = 18f, noiseSigma, readoutS;   // readoutS > 0 turns the rolling-shutter hook on
        public bool stills = true, onboardHud;
        public string masterDir;                                 // scratch for the high-quality master encode
    }

    /// <summary>Renders one demo run: the cinematic film, the onboard sensor feed, key stills and the side-by-side critique pairs.
    /// Editor-mode safe (no play mode needed): the editor entry point opens the scene and calls <see cref="Run"/>.</summary>
    public sealed class DemoSession {
        readonly DemoOptions o;
        readonly List<string> log = new List<string>();
        int errors; readonly List<string> errorLines = new List<string>();
        public int Errors => errors;

        public DemoSession(DemoOptions options) { o = options; }

        static void Drain(IEnumerator e) { while (e.MoveNext()) { } }

        public bool Run() {
            Application.logMessageReceived += OnLog;
            try { return RunInner(); }
            finally { Application.logMessageReceived -= OnLog; }
        }

        void OnLog(string msg, string stack, LogType t) {
            if (t != LogType.Error && t != LogType.Exception && t != LogType.Assert) return;
            errors++; if (errorLines.Count < 40) errorLines.Add(msg.Length > 400 ? msg.Substring(0, 400) : msg);
        }

        bool RunInner() {
            var wall = System.Diagnostics.Stopwatch.StartNew();
            bool wantCine = o.profile == "cinematic" || o.profile == "both", wantOnboard = o.profile == "onboard" || o.profile == "both";
            if (!wantCine && !wantOnboard) throw new ArgumentException("profile must be cinematic, onboard or both");
            Directory.CreateDirectory(o.outDir);
            var spec = DemoSpec.Parse(File.ReadAllText(o.sequencePath));
            if (o.seed != 0) spec.seed = o.seed;
            int W = spec.width, H = spec.height;

            // ---- world, quad, flight
            var world = new AerialWorld();
            Drain(world.Prepare(null));
            var mk = world.Markers;
            var robot = new AerialRobot();
            var quad = new QuadHandle(robot.CreateHero("AOS7 Quad"));
            bool stubQuad = !quad.HasRig;
            Vector3 person = mk.target ?? (mk.spawnPos + mk.spawnRot * spec.personOffset);
            if (mk.isStubScene) person = new Vector3(0, 0, 40);
            Physics.SyncTransforms();
            var flight = FlightBuilder.Build(spec, mk, person, spec.seed, Path.GetDirectoryName(Path.GetFullPath(o.sequencePath)), out string flightNote);
            var dir = new DemoDirector(spec, flight, person, mk);
            int total = dir.FrameCount;
            int a = Mathf.Clamp(o.firstFrame, 0, total), b = Mathf.Clamp(o.lastFrame, a, total);

            // ---- scene look
            var forest = UnityEngine.Object.FindFirstObjectByType<DroneForestDirector>();
            if (forest != null) {
                forest.SetPreset(spec.lightPreset); forest.SetDepthOfField(false); forest.SetFilmGrain(false); forest.SetMotionBlur(wantCine);
            }

            // ---- cameras and profiles
            Camera cine = null; RenderTexture rtCine = null; DepthOfField dof = null;
            Volume cineVol = null; VolumeProfile cineProfile = null;
            if (wantCine) {
                var go = new GameObject("Aerial Cinematic Camera");
                cine = go.AddComponent<Camera>(); cine.enabled = false;
                RenderProfiles.ConfigureCinematicCamera(cine, W, H);
                if (forest != null) DroneForestDirector.ConfigureCamera(cine);
                cineProfile = RenderProfiles.BuildCinematic();
                cineProfile.TryGet(out dof);
                cineProfile.TryGet<Exposure>(out var ce); if (ce != null) ce.adaptationMode.Override(AdaptationMode.Fixed);   // deterministic: no temporal adaptation
                cineVol = RenderProfiles.MakeVolume("Aerial Cinematic Volume", cineProfile, RenderProfiles.CinematicLayer, 1000f);
                rtCine = NewRt(W, H, "cinematic");
                cine.targetTexture = rtCine;
            }
            var sceneProfiles = new List<VolumeProfile>();
            var sv = UnityEngine.Object.FindObjectsByType<Volume>(FindObjectsInactive.Exclude);
            Array.Sort(sv, (x, y) => y.priority.CompareTo(x.priority));
            foreach (var v in sv) if (v.sharedProfile != null && v.gameObject.layer != RenderProfiles.CinematicLayer && v.gameObject != (cineVol != null ? cineVol.gameObject : null) && v.weight > 0.01f) sceneProfiles.Add(v.sharedProfile);
            // the global project profile is not a Volume object: add it too so sky/fog defaults carry over
            var onboardProfile = RenderProfiles.BuildOnboard(sceneProfiles);
            var onboardVol = RenderProfiles.MakeVolume("Aerial Onboard Volume", onboardProfile, RenderProfiles.OnboardLayer, 1000f);
            var leak = RenderProfiles.CinematicOverrides(onboardProfile);
            if (leak.Count > 0) throw new InvalidOperationException("onboard profile contains cinematic effects: " + string.Join(",", leak));

            var rgb = new OnboardCamera("OAK-D S2 RGB", quad.CameraMount, Vector3.zero, OakD.RgbHfovDeg, OakD.RgbW, OakD.RgbH, false);
            OnboardCamera left = null, right = null;
            if (forest != null) { DroneForestDirector.ConfigureCamera(rgb.Cam); RenderProfiles.ConfigureOnboardCamera(rgb.Cam, OakD.RgbHfovDeg, OakD.RgbW, OakD.RgbH); }
            if (o.stills) {
                left = new OnboardCamera("OAK-D S2 Left", quad.CameraMount, new Vector3(-OakD.BaselineM * 0.5f, 0, 0), OakD.StereoHfovDeg, OakD.StereoW, OakD.StereoH, true);
                right = new OnboardCamera("OAK-D S2 Right", quad.CameraMount, new Vector3(OakD.BaselineM * 0.5f, 0, 0), OakD.StereoHfovDeg, OakD.StereoW, OakD.StereoH, true);
                if (forest != null) foreach (var c in new[] { left, right }) { DroneForestDirector.ConfigureCamera(c.Cam); RenderProfiles.ConfigureOnboardCamera(c.Cam, OakD.StereoHfovDeg, OakD.StereoW, OakD.StereoH); }
            }
            var compose = new AerialCompose();
            var overlay = new DemoOverlay(W, H);
            var rtSensor = NewRt(OakD.RgbW, OakD.RgbH, "onboard sensor");
            var rtFinal = NewRt(W, H, "final");
            var rtStereo = o.stills ? NewRt(OakD.StereoW, OakD.StereoH, "stereo sensor") : null;

            // ---- outputs
            string scratch = o.masterDir ?? Path.Combine(Path.GetTempPath(), "crane_demo_" + o.runId);
            Directory.CreateDirectory(scratch);
            string stillDir = Path.Combine(o.outDir, "stills"); if (o.stills) Directory.CreateDirectory(stillDir);
            TheaterCapture capCine = wantCine ? new TheaterCapture(o.ffmpeg, Path.Combine(scratch, "cinematic_master.mp4"), W, H, spec.fps, 14, "medium") : null;
            TheaterCapture capOn = wantOnboard ? new TheaterCapture(o.ffmpeg, Path.Combine(scratch, "onboard_master.mp4"), OakD.RgbW, OakD.RgbH, spec.fps, 14, "medium") : null;
            var stillFrames = new Dictionary<int, int>();
            if (o.stills) foreach (var (shot, fr) in dir.StillFrames()) stillFrames[fr] = shot;
            Debug.Log($"AERIAL_DEMO_START run={o.runId} frames={a}:{b}/{total} fps={spec.fps} size={W}x{H} profile={o.profile} seed={spec.seed} " +
                      $"gfx={SystemInfo.graphicsDeviceType} device={SystemInfo.graphicsDeviceName} world=[{world.Describe}] flight=[{flightNote}] stubQuad={stubQuad}");

            // ---- frame loop (with preroll so exposure, volumetrics and motion-blur history settle)
            int first = Mathf.Max(0, a - o.preroll);
            var framesLog = new StringBuilder();
            var renderMs = new List<double>();
            var sw = new System.Diagnostics.Stopwatch();
            double dt = 1.0 / spec.fps;
            TrajSample prevDrone = default; bool havePrev = false;
            var model = new OverlayModel();
            int stillsWritten = 0;
            int loopStart = a == 0 ? -o.preroll : first;
            for (int fi = loopStart; fi < b; fi++) {
                int f = Mathf.Max(0, fi);
                sw.Restart();
                var df = dir.Evaluate(f);
                world.SetTime(df.flightT);
                quad.Pose(df.drone, dt);
                Physics.SyncTransforms();
                bool recording = fi >= a;
                bool still = recording && stillFrames.ContainsKey(f);
                bool needOnboard = wantOnboard || df.inset != "" || still;
                bool inRange = fi >= a;

                // onboard camera
                if (needOnboard) {
                    rgb.Render();
                    float shear = 0;
                    if (o.readoutS > 0 && havePrev) {
                        float yawRate = Mathf.DeltaAngle(Mathf.Atan2((prevDrone.rot * Vector3.forward).x, (prevDrone.rot * Vector3.forward).z) * Mathf.Rad2Deg,
                            Mathf.Atan2((df.drone.rot * Vector3.forward).x, (df.drone.rot * Vector3.forward).z) * Mathf.Rad2Deg) / (float)dt;
                        shear = -o.readoutS * yawRate / OakD.RgbHfovDeg;   // fraction of image width swept during the readout
                    }
                    compose.SensorPass(rgb.Rt, rtSensor, o.noiseSigma, shear, false, spec.seed * 1000003 + f);
                }
                prevDrone = df.drone; havePrev = true;

                // cinematic camera
                model.boxes.Clear();
                if (cine != null && (wantCine || still)) {
                    cine.transform.SetPositionAndRotation(df.cam.pos, df.cam.rot);
                    RenderProfiles.SetCinematicLens(cine, df.cam.fov, df.aperture);
                    if (dof != null) {
                        dof.focusMode.Override(df.aperture > 0 ? DepthOfFieldMode.UsePhysicalCamera : DepthOfFieldMode.Off);
                        dof.focusDistance.Override(df.cam.focus);
                    }
                    cine.Render();
                }

                if (inRange) {
                    // compose the film frame
                    if (cine != null && wantCine) {
                        Compose(df, spec, dir, cine, rgb.Cam, rtCine, rtSensor, rtFinal, compose, overlay, model, W, H, true);
                        capCine.Write(rtFinal, f);
                        if (still) SaveStill(Path.Combine(stillDir, $"{df.spec.id}_f{f:D4}_cinematic.jpg"), rtFinal, compose, 1600);
                    }
                    if (wantOnboard) {
                        capOn.Write(rtSensor, f);
                    }
                    if (still) {
                        string tag = $"{df.spec.id}_f{f:D4}";
                        SaveStill(Path.Combine(stillDir, tag + "_onboard_rgb.jpg"), rtSensor, compose, 1600);
                        left.Render(); right.Render();
                        compose.SensorPass(left.Rt, rtStereo, o.noiseSigma, 0, true, spec.seed + f);
                        SaveStill(Path.Combine(stillDir, tag + "_onboard_stereo_left.jpg"), rtStereo, compose, 1280);
                        compose.SensorPass(right.Rt, rtStereo, o.noiseSigma, 0, true, spec.seed + f + 1);
                        SaveStill(Path.Combine(stillDir, tag + "_onboard_stereo_right.jpg"), rtStereo, compose, 1280);
                        // clean (no HUD) pair for critique: third person camera beside the onboard RGB, same timestamp
                        if (cine != null) {
                            SavePair(Path.Combine(stillDir, tag + "_pair_thirdperson_vs_onboard.jpg"), rtCine, rtSensor, compose, rtFinal);
                        }
                        stillsWritten++;
                    }
                    var cp = df.cam.pos;
                    framesLog.Append("{\"frame\":").Append(f).Append(",\"shot\":\"").Append(df.spec.id).Append("\",\"t\":").Append(TJson.F(df.flightT))
                        .Append(",\"drone\":[").Append(TJson.F(df.drone.pos.x)).Append(',').Append(TJson.F(df.drone.pos.y)).Append(',').Append(TJson.F(df.drone.pos.z))
                        .Append("],\"cam\":[").Append(TJson.F(cp.x)).Append(',').Append(TJson.F(cp.y)).Append(',').Append(TJson.F(cp.z)).Append("]}\n");
                    renderMs.Add(sw.Elapsed.TotalMilliseconds);
                    if ((f - a) % 60 == 0) Debug.Log($"AERIAL_DEMO_FRAME {f}/{b} shot={df.spec.id} ms={sw.Elapsed.TotalMilliseconds:0.0}");
                }
            }

            bool encoded = true;
            string cineOut = null, onOut = null;
            if (capCine != null) { encoded &= capCine.Finish(); }
            if (capOn != null) { encoded &= capOn.Finish(); }
            double seconds = (b - a) / spec.fps;
            if (wantCine && o.ffmpeg != "none") { cineOut = Path.Combine(o.outDir, "demo_cinematic_1080p30.mp4"); encoded &= Transcode(o, capCine.Output, cineOut, seconds, W, H); }
            if (wantOnboard && o.ffmpeg != "none") { onOut = Path.Combine(o.outDir, "onboard_oakd_rgb_1080p30.mp4"); encoded &= Transcode(o, capOn.Output, onOut, seconds, OakD.RgbW, OakD.RgbH); }
            File.WriteAllText(Path.Combine(o.outDir, "frames.jsonl"), framesLog.ToString());

            var doc = new SortedDictionary<string, object> {
                { "schema", "crane-aerial-demo-report/1" }, { "run_id", o.runId }, { "ok", encoded && errors == 0 }, { "encoded", encoded },
                { "sequence", Path.GetFileName(o.sequencePath) }, { "title", spec.title }, { "seed", spec.seed }, { "profile", o.profile },
                { "frames", new List<object> { a, b } }, { "total_frames", total }, { "fps", (double)spec.fps }, { "size", new List<object> { W, H } },
                { "duration_s", (double)total / spec.fps }, { "videos", new List<object> { cineOut != null ? Path.GetFileName(cineOut) : "", onOut != null ? Path.GetFileName(onOut) : "" } },
                { "stills", stillsWritten }, { "errors", errors }, { "error_lines", errorLines },
                { "gfx", SystemInfo.graphicsDeviceType.ToString() }, { "device", SystemInfo.graphicsDeviceName }, { "unity", Application.unityVersion },
                { "world", world.Describe }, { "flight", flight.Describe }, { "flight_note", flightNote }, { "director_notes", dir.Notes },
                { "stub_scene", mk.isStubScene }, { "stub_quad", stubQuad },
                { "render_ms_median", renderMs.Count > 0 ? Median(renderMs) : 0.0 }, { "wall_s", wall.Elapsed.TotalSeconds },
                { "onboard", new SortedDictionary<string, object> {
                    { "sensor", "OAK-D S2" }, { "source", "https://docs.luxonis.com/hardware/products/OAK-D%20S2" },
                    { "rgb_hfov_deg", (double)OakD.RgbHfovDeg }, { "rgb_vfov_deg", (double)OakD.RgbVfov }, { "rgb_size", new List<object> { OakD.RgbW, OakD.RgbH } },
                    { "rgb_fx_px", (double)OakD.FocalPx(OakD.RgbHfovDeg, OakD.RgbW) },
                    { "stereo_hfov_deg", (double)OakD.StereoHfovDeg }, { "stereo_vfov_deg", (double)OakD.StereoVfov }, { "stereo_size", new List<object> { OakD.StereoW, OakD.StereoH } },
                    { "stereo_fx_px", (double)OakD.FocalPx(OakD.StereoHfovDeg, OakD.StereoW) }, { "baseline_m", (double)OakD.BaselineM },
                    { "noise_sigma", (double)o.noiseSigma }, { "rolling_shutter_readout_s", (double)o.readoutS },
                    { "cinematic_effects_in_profile", new List<object>(leak) }, { "volume_layer", RenderProfiles.OnboardLayer },
                    { "frame_settings_off", new List<object> { "DepthOfField", "MotionBlur", "Bloom", "Vignette", "ColorGrading", "ChromaticAberration", "FilmGrain", "LensDistortion", "PaniniProjection" } }
                } }
            };
            File.WriteAllText(Path.Combine(o.outDir, "render_report.json"), TJson.Write(doc) + "\n");
            Debug.Log($"AERIAL_DEMO_DONE frames={b - a} encoded={encoded} errors={errors} wall_s={wall.Elapsed.TotalSeconds:0.0}");

            capCine?.Dispose(); capOn?.Dispose(); compose.Dispose(); overlay.Dispose();
            rgb.Dispose(); left?.Dispose(); right?.Dispose();
            return encoded && errors == 0;
        }

        static double Median(List<double> l) { var c = new List<double>(l); c.Sort(); return c[c.Count / 2]; }

        static RenderTexture NewRt(int w, int h, string name) {
            var rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1, name = name };
            rt.Create(); return rt;
        }

        // ------------------------------------------------------------------ composition

        static void Compose(DirectedFrame df, DemoSpec spec, DemoDirector dir, Camera cine, Camera rgbCam, RenderTexture rtCine, RenderTexture rtSensor,
                            RenderTexture rtFinal, AerialCompose compose, DemoOverlay overlay, OverlayModel m, int W, int H, bool hud) {
            m.boxes.Clear();
            m.split = df.inset == "split";
            m.leftLabel = "THIRD PERSON  /  CINEMATIC"; m.rightLabel = $"ONBOARD  /  OAK-D S2 RGB  {OakD.RgbHfovDeg:0} DEG HFOV";
            if (df.inset == "split") {
                compose.Blit(rtCine, rtFinal, new Rect(0, 0, W * 0.5f, H), new Vector4(0.25f, 0, 0.75f, 1), 1, true);
                compose.Blit(rtSensor, rtFinal, new Rect(W * 0.5f, 0, W * 0.5f, H), new Vector4(0.25f, 0, 0.75f, 1), 1, false);
                if (df.detection) {
                    if (dir.PersonBox(cine, out var bx, out var occ)) AddCropped(m, bx, occ, W, 0.25f, 0f, "PERSON", "third person");
                    if (dir.PersonBox(rgbCam, out var bx2, out var occ2)) AddCropped(m, bx2, occ2, W, 0.25f, W * 0.5f, "PERSON", "onboard");
                }
            } else {
                compose.Blit(rtCine, rtFinal, new Rect(0, 0, W, H), new Vector4(0, 0, 1, 1), 1, true);
                if (df.inset == "pip") {
                    var r = new Rect(W - W * 0.3f - 36, 36, W * 0.3f, H * 0.3f);
                    compose.Blit(rtSensor, rtFinal, r, new Vector4(0, 0, 1, 1), 1, false);
                }
                if (df.detection && dir.PersonBox(cine, out var bx, out var occ))
                    m.boxes.Add(new DetBox { px = bx, label = "PERSON", sub = "sim ground-truth box", occluded = occ });
            }
            m.fade = df.fade; m.titleAlpha = df.titleAlpha; m.lowerAlpha = df.lowerAlpha;
            m.title = df.title?.text; m.titleSub = df.title?.sub; m.lower = df.lowerThird?.text; m.lowerSub = df.lowerThird?.sub;
            m.tagLeft = $"{df.spec.id.ToUpperInvariant()}  /  {df.spec.type.ToUpperInvariant()}";
            m.tagRight = spec.title.ToUpperInvariant(); m.clock = df.clock;
            overlay.Draw(rtFinal, m);
        }

        static void AddCropped(OverlayModel m, Rect box, bool occ, int W, float cropLeft, float xOffset, string label, string sub) {
            float c0 = cropLeft * W, half = W * 0.5f;
            float x0 = Mathf.Clamp(box.xMin - c0, 0, half), x1 = Mathf.Clamp(box.xMax - c0, 0, half);
            if (x1 - x0 < 3) return;
            m.boxes.Add(new DetBox { px = new Rect(x0 + xOffset, box.y, x1 - x0, box.height), label = label, sub = sub + " (sim ground-truth box)", occluded = occ });
        }

        // ------------------------------------------------------------------ stills and encode

        static void SaveStill(string path, RenderTexture src, AerialCompose compose, int maxDim) {
            float k = Mathf.Min(1f, maxDim / (float)Mathf.Max(src.width, src.height));
            int w = Mathf.RoundToInt(src.width * k), h = Mathf.RoundToInt(src.height * k);
            var rt = RenderTexture.GetTemporary(w, h, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            compose.Blit(src, rt, new Rect(0, 0, w, h), new Vector4(0, 0, 1, 1), 1, true);
            WriteJpg(path, rt);
            RenderTexture.ReleaseTemporary(rt);
        }

        static void SavePair(string path, RenderTexture a, RenderTexture b, AerialCompose compose, RenderTexture scratch) {
            const int w = 1600, h = 450;
            var rt = RenderTexture.GetTemporary(w, h, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            compose.Blit(a, rt, new Rect(0, 0, w / 2, h), new Vector4(0, 0, 1, 1), 1, true);
            compose.Blit(b, rt, new Rect(w / 2, 0, w / 2, h), new Vector4(0, 0, 1, 1), 1, false);
            WriteJpg(path, rt);
            RenderTexture.ReleaseTemporary(rt);
        }

        static void WriteJpg(string path, RenderTexture rt) {
            var prev = RenderTexture.active;
            RenderTexture.active = rt;
            var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0, false);
            tex.Apply(false);
            RenderTexture.active = prev;
            File.WriteAllBytes(path, tex.EncodeToJPG(88));
            UnityEngine.Object.DestroyImmediate(tex);
        }

        /// <summary>Re-encode the high-quality master to the repo-size budget (average bitrate from the byte budget).</summary>
        static bool Transcode(DemoOptions o, string master, string output, double seconds, int w, int h) {
            long bits = (long)(o.maxMegabytes * 1024 * 1024 * 8 * 0.94);
            long kbps = Math.Min(8000, Math.Max(500, (long)(bits / Math.Max(1.0, seconds) / 1000)));   // cap: short clips must not balloon
            var psi = new System.Diagnostics.ProcessStartInfo {
                FileName = o.ffmpeg,
                Arguments = $"-hide_banner -loglevel error -y -i \"{master}\" -c:v libx264 -preset slow -b:v {kbps}k -maxrate {(long)(kbps * 1.4)}k -bufsize {kbps * 2}k " +
                            $"-pix_fmt yuv420p -movflags +faststart -an \"{output}\"",
                UseShellExecute = false, CreateNoWindow = true, RedirectStandardError = true
            };
            using (var p = System.Diagnostics.Process.Start(psi)) {
                string err = p.StandardError.ReadToEnd();
                p.WaitForExit();
                if (p.ExitCode != 0) { Debug.LogError("AERIAL_DEMO transcode failed: " + err); return false; }
            }
            double mb = new FileInfo(output).Length / 1048576.0;
            Debug.Log($"AERIAL_DEMO_VIDEO {output} {mb:0.0} MB kbps={kbps}");
            if (mb > 20.0) { Debug.LogError($"AERIAL_DEMO video {mb:0.0} MB exceeds the 20 MB repo budget"); return false; }
            return true;
        }
    }
}
