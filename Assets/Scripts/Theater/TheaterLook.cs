using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater {
    /// <summary>A theater look: <c>plain</c> (the scene as the simulator renders it, the cut every render had before
    /// looks existed) or <c>theater</c> (a graded, calm presentation: tonemapping, soft bloom, ambient occlusion,
    /// vignette, deterministic grain, close-shot depth of field, a time-of-day sun and sky with fog, an inky water
    /// surface, a horizon, hero wakes, ink-bodied ghosts, the camera rig and the panel HUD).
    /// Chosen by <c>--crane-theater-look NAME</c>, else the manifest's <c>look_preset</c> (a name or
    /// <c>{"name", "time_of_day", "grain", "dof", "horizon", "wakes", "ink"}</c>), else plain.
    /// Everything here runs in the theater player only and changes pixels, never the simulation.</summary>
    public sealed class LookPreset {
        public string Name = "plain";
        public string TimeOfDay = "evening";   // morning | noon | evening | dusk | overcast
        public float Grain = 0.035f;
        public bool Dof = true, Horizon = true, Wakes = true, Ink = true, Rig = true, Panels = true;
        public bool Theater => Name == "theater";

        public static LookPreset Resolve(TheaterManifest m, Func<string, string> arg) {
            var p = new LookPreset();
            object raw = m.Raw != null && m.Raw.TryGetValue("look_preset", out var o) ? o : null;
            if (raw is string s) p.Name = s;
            else if (raw is Dictionary<string, object> d) {
                p.Name = TJson.Str(d, "name", "theater");
                p.TimeOfDay = TJson.Str(d, "time_of_day", p.TimeOfDay);
                p.Grain = (float)TJson.Num(d, "grain", p.Grain);
                p.Dof = TJson.Bool(d, "dof", p.Dof); p.Horizon = TJson.Bool(d, "horizon", p.Horizon);
                p.Wakes = TJson.Bool(d, "wakes", p.Wakes); p.Ink = TJson.Bool(d, "ink", p.Ink);
                p.Rig = TJson.Bool(d, "camera_rig", p.Rig); p.Panels = TJson.Bool(d, "panels", p.Panels);
            }
            p.Name = arg("--crane-theater-look") ?? p.Name;
            p.TimeOfDay = arg("--crane-theater-time-of-day") ?? p.TimeOfDay;
            if (p.Name != "theater") p.Name = "plain";
            return p;
        }

        public string Describe => Theater ? $"theater/{TimeOfDay}" : "plain";
    }

    /// <summary>Sun and sky for a time of day: sun elevation and azimuth (degrees), its illuminance (lux) and colour
    /// temperature (K), the soft-shadow angular diameter, and a three-stop gradient sky (relative to a white
    /// diffuse surface in full sun) with the fog's mean free path.</summary>
    struct TimeOfDay {
        public float elevation, azimuth, lux, kelvin, diameter, fogMfp, exposureComp;
        public Color top, middle, bottom;

        public static TimeOfDay Get(string name) {
            switch (name) {
                case "morning": return new TimeOfDay {
                    elevation = 24, azimuth = 140, lux = 70000, kelvin = 5200, diameter = 2.0f, fogMfp = 260, exposureComp = 0f,
                    top = Hex("#33496A"), middle = Hex("#8BA3B8"), bottom = Hex("#D3D5D0") };
                case "noon": return new TimeOfDay {
                    elevation = 58, azimuth = 200, lux = 100000, kelvin = 6000, diameter = 1.5f, fogMfp = 340, exposureComp = -0.15f,
                    top = Hex("#2E4D74"), middle = Hex("#86A6C2"), bottom = Hex("#D8DEE2") };
                case "dusk": return new TimeOfDay {
                    elevation = 5, azimuth = 250, lux = 18000, kelvin = 3500, diameter = 3.0f, fogMfp = 190, exposureComp = 0.2f,
                    top = Hex("#141C2C"), middle = Hex("#3C4A62"), bottom = Hex("#A88D79") };
                case "overcast": return new TimeOfDay {
                    elevation = 40, azimuth = 210, lux = 25000, kelvin = 6500, diameter = 12f, fogMfp = 170, exposureComp = 0.1f,
                    top = Hex("#566270"), middle = Hex("#8E99A4"), bottom = Hex("#B9BFC3") };
                default: return new TimeOfDay {   // evening: low warm sun, deep blue ink sky, a muted warm horizon
                    elevation = 15, azimuth = 235, lux = 42000, kelvin = 4500, diameter = 2.5f, fogMfp = 520, exposureComp = 0.1f,
                    top = Hex("#121B2B"), middle = Hex("#34455C"), bottom = Hex("#8E8C88") };
            }
        }

        static Color Hex(string h) { ColorUtility.TryParseHtmlString(h, out var c); return c; }
    }

    /// <summary>Applies a <see cref="LookPreset"/> to the theater's volume, sun, sky, water and camera, and draws the
    /// look's own layers (<see cref="LookScenePass"/> before post-processing: horizon, hero wakes, ghost ink;
    /// <see cref="LookFinishPass"/> after the theater's overlays: film grain). Every input is the frame index.</summary>
    public sealed class TheaterLook : IDisposable {
        public readonly LookPreset P;
        readonly TheaterState st;
        readonly ITheaterWorld world;
        readonly TheaterRenderer overlays;
        readonly Material mat;
        readonly TimeOfDay tod;
        DepthOfField dof;
        Mesh horizon;
        readonly Mesh wakes;
        readonly List<Vector3> wv = new List<Vector3>();
        readonly List<Vector4> wu = new List<Vector4>();
        readonly List<int> wi = new List<int>();
        int frame;
        float skyEv;
        Color haze, hill, foam;
        GameObject passGo;
        public readonly Dictionary<string, object> Report = new Dictionary<string, object>();

        public TheaterLook(LookPreset preset, TheaterState state, ITheaterWorld world, TheaterRenderer overlays, VolumeProfile profile, Camera cam) {
            P = preset; st = state; this.world = world; this.overlays = overlays;
            tod = TimeOfDay.Get(P.TimeOfDay);
            var shader = Resources.Load<Shader>("CraneTheaterLook");
            if (shader == null) throw new InvalidOperationException("theater: shader Resources/CraneTheaterLook missing");
            mat = new Material(shader) { name = "theater look", hideFlags = HideFlags.DontSave };
            wakes = new Mesh { name = "theater wakes", indexFormat = IndexFormat.UInt32 };
            wakes.MarkDynamic();
            if (overlays != null) { overlays.TrailGain = 0.6f; overlays.CometGain = 0.7f; }   // calmer data ribbons
            Sun();
            Post(profile);
            Water();
            if (P.Horizon) horizon = Hills();
            passGo = new GameObject("Theater Look Passes");
            var finishGo = new GameObject("Theater Look Finish");
            finishGo.transform.SetParent(passGo.transform);
            var before = passGo.AddComponent<CustomPassVolume>();
            before.isGlobal = true; before.targetCamera = cam;
            before.injectionPoint = CustomPassInjectionPoint.BeforePostProcess;
            before.customPasses.Add(new LookScenePass { L = this, name = "theater look scene" });
            var after = finishGo.AddComponent<CustomPassVolume>();
            after.isGlobal = true; after.targetCamera = cam; after.priority = -10f;   // after the theater's overlays (priority 0)
            after.injectionPoint = CustomPassInjectionPoint.AfterPostProcess;
            after.customPasses.Add(new LookFinishPass { L = this, name = "theater look finish" });
            Report["preset"] = P.Describe;
        }

        public void Dispose() { if (passGo != null) UnityEngine.Object.Destroy(passGo); }

        static T Get<T>(VolumeProfile profile) where T : VolumeComponent =>
            profile.TryGet<T>(out var c) ? c : profile.Add<T>(false);

        // ------------------------------------------------------------------ setup

        void Sun() {
            Light sun = null;
            foreach (var l in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude))
                if (l.type == LightType.Directional && (sun == null || l.intensity > sun.intensity)) sun = l;
            skyEv = Mathf.Log(tod.lux / Mathf.PI, 2f);
            if (sun == null) { Report["sun"] = "none"; return; }
            sun.transform.rotation = Quaternion.Euler(tod.elevation, tod.azimuth, 0f);
            sun.intensity = tod.lux;
            sun.useColorTemperature = true;
            sun.colorTemperature = tod.kelvin;
            sun.color = Color.white;
            sun.shadows = LightShadows.Soft;
            if (sun.TryGetComponent<HDAdditionalLightData>(out var hd)) hd.angularDiameter = tod.diameter;
            Report["sun"] = $"{sun.name} elev={tod.elevation} az={tod.azimuth} lux={tod.lux} K={tod.kelvin}";
            // what the look's own layers emit, in the same units as the sky (white diffuse in full sun = 1)
            float s = Mathf.Sin(tod.elevation * Mathf.Deg2Rad);
            var k = Mathf.CorrelatedColorTemperatureToRGB(tod.kelvin);
            foam = new Color(k.r, k.g, k.b) * (0.78f * (0.25f + 0.75f * s));
            haze = tod.bottom.linear;
            hill = Color.Lerp(tod.middle.linear, new Color(0.04f, 0.05f, 0.07f), 0.55f);
        }

        void Post(VolumeProfile profile) {
            var tm = Get<Tonemapping>(profile);
            tm.mode.Override(TonemappingMode.ACES);
            var ca = Get<ColorAdjustments>(profile);
            ca.postExposure.Override(tod.exposureComp);
            ca.contrast.Override(8f);
            ca.saturation.Override(-12f);
            var lgg = Get<LiftGammaGain>(profile);
            lgg.lift.Override(new Vector4(0.985f, 1.0f, 1.035f, -0.012f));   // shadows lean to blue ink
            lgg.gain.Override(new Vector4(1.02f, 1.0f, 0.975f, 0f));         // highlights a touch warm
            var bloom = Get<Bloom>(profile);
            bloom.intensity.Override(0.14f);
            bloom.scatter.Override(0.62f);
            bloom.threshold.Override(1.0f);
            var ao = Get<ScreenSpaceAmbientOcclusion>(profile);
            ao.intensity.Override(0.7f);
            ao.radius.Override(1.4f);
            ao.directLightingStrength.Override(0.25f);
            ao.temporalAccumulation.Override(false);   // no history: a frame is a function of its index
            var vg = Get<Vignette>(profile);
            vg.intensity.Override(0.24f);
            vg.smoothness.Override(0.48f);
            vg.roundness.Override(1f);
            vg.color.Override(new Color(0.02f, 0.025f, 0.04f));
            dof = Get<DepthOfField>(profile);
            dof.focusMode.Override(DepthOfFieldMode.Off);
            dof.coCStabilization.Override(false);
            var env = Get<VisualEnvironment>(profile);
            env.skyType.Override((int)SkyType.Gradient);
            env.skyAmbientMode.Override(SkyAmbientMode.Dynamic);
            var sky = Get<GradientSky>(profile);
            sky.top.Override(tod.top); sky.middle.Override(tod.middle); sky.bottom.Override(tod.bottom);
            sky.gradientDiffusion.Override(1.6f);
            sky.skyIntensityMode.Override(SkyIntensityMode.Exposure);
            sky.exposure.Override(skyEv);
            var fog = Get<Fog>(profile);
            fog.enabled.Override(true);
            fog.colorMode.Override(FogColorMode.SkyColor);
            fog.meanFreePath.Override(tod.fogMfp);
            fog.baseHeight.Override(world.SurfaceY(world.Bounds.center));
            fog.maximumHeight.Override(world.SurfaceY(world.Bounds.center) + 40f);
            fog.maxFogDistance.Override(6000f);
            fog.enableVolumetricFog.Override(false);
            fog.mipFogMaxMip.Override(0.6f);
        }

        /// <summary>The water's look only (colours, scattering, smoothness, foam colour); its simulation (waves, wind,
        /// time) is left as recorded.</summary>
        void Water() {
            int n = 0;
            foreach (var w in UnityEngine.Object.FindObjectsByType<WaterSurface>(FindObjectsInactive.Exclude)) {
                w.scatteringColor = new Color(0.01f, 0.075f, 0.12f);
                w.refractionColor = new Color(0.015f, 0.08f, 0.11f);
                w.absorptionDistance = 3.5f;
                w.maxRefractionDistance = 0.6f;
                w.ambientScattering = 0.15f;
                w.heightScattering = 0.18f;
                w.displacementScattering = 0.22f;
                w.directLightTipScattering = 0.5f;
                w.directLightBodyScattering = 0.3f;
                w.startSmoothness = 0.93f;
                w.endSmoothness = 0.8f;
                w.foamColor = new Color(0.92f, 0.94f, 0.95f);
                w.caustics = false;
                n++;
            }
            Report["waters"] = n;
        }

        /// <summary>Two rings of low hills around the course (deterministic noise): a near, darker range and a far,
        /// hazier one, so the horizon is a coast and not a void.</summary>
        Mesh Hills() {
            var c = world.Bounds.center;
            float y0 = world.SurfaceY(c) - 3f;
            var v = new List<Vector3>(); var u = new List<Vector4>(); var idx = new List<int>();
            void Ring(float radius, float hMin, float hMax, float layer, int seed) {
                const int n = 360;
                int b = v.Count;
                for (int i = 0; i <= n; i++) {
                    float a = 2 * Mathf.PI * i / n;
                    float h = hMin + (hMax - hMin) * Ridge(a, seed);
                    var dir = new Vector3(Mathf.Cos(a), 0, Mathf.Sin(a));
                    v.Add(c + dir * radius + Vector3.up * (y0 - c.y)); u.Add(new Vector4(0, layer, a, 0));
                    v.Add(c + dir * radius + Vector3.up * (y0 - c.y + h)); u.Add(new Vector4(1, layer, a, h));
                    if (i > 0) { int k = b + 2 * i; idx.Add(k - 2); idx.Add(k); idx.Add(k - 1); idx.Add(k - 1); idx.Add(k); idx.Add(k + 1); }
                }
            }
            Ring(1700f, 18f, 95f, 1f, 7);
            Ring(900f, 6f, 46f, 0f, 3);
            var m = new Mesh { name = "theater horizon", indexFormat = IndexFormat.UInt32 };
            m.SetVertices(v); m.SetUVs(0, u); m.SetTriangles(idx, 0);
            m.bounds = new Bounds(c, Vector3.one * 1e5f);
            return m;
        }

        /// <summary>Smooth, seeded ridge noise in 0..1 around a ring (periodic in a).</summary>
        static float Ridge(float a, int seed) {
            float h = 0, amp = 1, norm = 0;
            for (int o = 0; o < 5; o++) {
                int f = 2 + o * 3 + seed % 3;
                float ph = seed * 1.7f + o * 2.3f;
                h += amp * (0.5f + 0.5f * Mathf.Sin(f * a + ph) * Mathf.Cos((f + 1) * a * 0.5f + ph * 0.7f));
                norm += amp; amp *= 0.55f;
            }
            h /= norm;
            // a few calm gaps where the far sea shows through
            float gap = Mathf.SmoothStep(0f, 1f, Mathf.Abs(Mathf.Sin(a * 1.5f + seed)) * 1.6f);
            return Mathf.Clamp01(h * h * 1.6f) * gap;
        }

        // ------------------------------------------------------------------ per frame

        /// <summary>Before <c>cam.Render()</c>: depth of field for close shots, the heroes' wakes.</summary>
        public void Frame(FrameState fs, Camera cam) {
            frame = fs.frame;
            DofFor(fs, cam);
            if (P.Wakes) BuildWakes(fs);
        }

        public static bool CloseShot(string shot) => shot == "follow" || shot == "chase" || shot == "hero_orbit";

        void DofFor(FrameState fs, Camera cam) {
            if (!P.Dof || dof == null) return;
            string shot = fs.plan.act.Shot;
            if (!CloseShot(shot) || fs.heroes.Count == 0) { dof.focusMode.Override(DepthOfFieldMode.Off); return; }
            float d = Vector3.Distance(fs.camPos, fs.heroes[0].pos);
            dof.focusMode.Override(DepthOfFieldMode.Manual);
            dof.nearFocusStart.Override(0f);
            dof.nearFocusEnd.Override(Mathf.Max(0.5f, d * 0.3f));
            dof.farFocusStart.Override(d * 1.9f);
            dof.farFocusEnd.Override(d * 6f);
        }

        void BuildWakes(FrameState fs) {
            wv.Clear(); wu.Clear(); wi.Clear();
            var pack = st.M.Pack;
            const float span = 7f;      // run seconds of wake
            float beam = 0.55f;
            foreach (var g in fs.heroes) {
                int run = g.run;
                float dur = st.M.Runs[run].duration;
                float head = Mathf.Min(g.tr, dur);
                if (head <= 0) continue;
                int n = 36;
                int b = wv.Count;
                for (int i = 0; i <= n; i++) {
                    float age = span * i / n;
                    float t = Mathf.Max(0f, head - age);
                    var p = st.UnityPos(pack.At(run, t));
                    var q = st.UnityPos(pack.At(run, Mathf.Max(0f, t - 0.25f)));
                    var r = st.UnityPos(pack.At(run, Mathf.Min(dur, t + 0.25f)));
                    Vector3 tan = r - q; tan.y = 0;
                    float speed = tan.magnitude / 0.5f;
                    tan = tan.sqrMagnitude > 1e-6f ? tan.normalized : Vector3.forward;
                    var side = Vector3.Cross(Vector3.up, tan);
                    float w = beam + age * 0.42f;
                    float fade = Mathf.Clamp01(speed / 0.35f);   // a boat at rest leaves no wake
                    if (g.tr > dur) fade *= Mathf.Clamp01(1f - (g.tr - dur) / 3f);
                    p.y = world.SurfaceY(p) + 0.02f;
                    wv.Add(p - side * w); wu.Add(new Vector4(age / span, -1, fade, w));
                    wv.Add(p + side * w); wu.Add(new Vector4(age / span, 1, fade, w));
                    if (i > 0) { int k = b + 2 * i; wi.Add(k - 2); wi.Add(k - 1); wi.Add(k); wi.Add(k - 1); wi.Add(k + 1); wi.Add(k); }
                }
            }
            wakes.Clear();
            wakes.SetVertices(wv); wakes.SetUVs(0, wu); wakes.SetTriangles(wi, 0, false);
            wakes.bounds = new Bounds(Vector3.zero, Vector3.one * 1e5f);
        }

        // ------------------------------------------------------------------ GPU

        void Common(Camera cam) {
            var vp = GL.GetGPUProjectionMatrix(cam.projectionMatrix, true) * cam.worldToCameraMatrix;
            mat.SetMatrix("_TheaterVP", vp);
            mat.SetVector("_CamPos", cam.transform.position);
            mat.SetFloat("_Frame", frame);
        }

        public int SceneDraws, FinishDraws;

        internal void DrawScene(CommandBuffer cmd, Camera cam, RTHandle color, RTHandle depth) {
            SceneDraws++;
            Common(cam);
            float sky = Mathf.Pow(2f, skyEv);
            mat.SetVector("_Haze", haze * sky);
            mat.SetVector("_Hill", hill * sky);
            mat.SetVector("_Foam", foam * sky);
            CoreUtils.SetRenderTarget(cmd, color, depth, ClearFlag.None);
            if (horizon != null) cmd.DrawMesh(horizon, Matrix4x4.identity, mat, 0, 0);
            if (P.Ink && overlays != null && overlays.GhostCount > 0) {
                mat.SetFloat("_Ink", 0.32f);
                cmd.DrawMeshInstancedProcedural(overlays.GhostMesh, 0, mat, 2, overlays.GhostCount, overlays.GhostProps);
            }
            if (P.Wakes && wakes.vertexCount > 0) cmd.DrawMesh(wakes, Matrix4x4.identity, mat, 0, 1);
        }

        internal void DrawFinish(CommandBuffer cmd, Camera cam, RTHandle color) {
            FinishDraws++;
            if (P.Grain <= 0) return;
            Common(cam);
            mat.SetFloat("_Grain", P.Grain);
            CoreUtils.SetRenderTarget(cmd, color, ClearFlag.None);
            cmd.DrawProcedural(Matrix4x4.identity, mat, 3, MeshTopology.Triangles, 3, 1);
        }
    }

    [Serializable]
    public sealed class LookScenePass : CustomPass {
        [NonSerialized] public TheaterLook L;
        protected override void Execute(CustomPassContext ctx) {
            if (L == null) return;
            L.DrawScene(ctx.cmd, ctx.hdCamera.camera, ctx.cameraColorBuffer, ctx.cameraDepthBuffer);
        }
    }

    [Serializable]
    public sealed class LookFinishPass : CustomPass {
        [NonSerialized] public TheaterLook L;
        protected override void Execute(CustomPassContext ctx) {
            if (L == null) return;
            L.DrawFinish(ctx.cmd, ctx.hdCamera.camera, ctx.cameraColorBuffer);
        }
    }
}
