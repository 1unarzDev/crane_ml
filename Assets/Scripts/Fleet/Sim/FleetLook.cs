using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// The indoor look (crane_ml Tools/Fleet/demo/LOOK.md; adapted from theater_staging FleetStage/FleetRenderProfiles): render-only dressing
    /// of the runtime warehouse (tiled floor texture, matte walls, safety-yellow door jambs, shelf and pallet detail, tinted zone patches with
    /// tape), a ceiling and the south wall on a cut-away layer that cinematic cameras skip and sensor cameras draw, a rect-light rig instead of the
    /// sun, and two strictly separate volume profiles: cinematic (layer 30) and neutral sensor (layer 31). Nothing here has a collider, so the
    /// physics and the logs are unchanged by it.
    /// </summary>
    public static class FleetLook {
        public const int CinematicLayer = 30, SensorLayer = 31, CutawayLayer = 28;
        public static VolumeProfile Cinematic, Sensor;

        public static readonly Type[] LookEffects = {
            typeof(Bloom), typeof(DepthOfField), typeof(MotionBlur), typeof(ColorAdjustments), typeof(ColorCurves), typeof(LiftGammaGain),
            typeof(ShadowsMidtonesHighlights), typeof(SplitToning), typeof(ChannelMixer), typeof(WhiteBalance), typeof(Vignette), typeof(FilmGrain),
            typeof(ChromaticAberration), typeof(LensDistortion), typeof(PaniniProjection)
        };

        public static List<string> SensorProfileViolations(VolumeProfile p) {
            var r = new List<string>();
            foreach (var c in p.components) {
                foreach (var t in LookEffects) if (t.IsInstanceOfType(c)) r.Add(c.GetType().Name);
                if (c is Tonemapping tm && tm.mode.overrideState && tm.mode.value != TonemappingMode.Neutral) r.Add("Tonemapping:" + tm.mode.value);
            }
            return r;
        }

        public static void Apply(FleetWorld world, FleetSpec spec) {
            JsonNode s = spec.S, L = spec.O["look"];
            Transform root = world.Root.transform;
            double[] size = s["arena.size_xy"].Doubles;
            double W = size[0], H = size[1], wallH = s["arena.wall_height"].Double;
            var stage = new GameObject("look").transform;
            stage.SetParent(root, false);

            // remove the phase-1 outdoor sun and fixed exposure
            foreach (var n in new[] { "sun", "fleet volume" }) { var t = root.Find(n); if (t != null) t.gameObject.SetActive(false); }

            // floor: tiled texture, one tile per 2 m
            var floor = root.Find("floor").GetComponent<Renderer>();
            var fm = FleetGeometry.Material(Color.white);
            fm.SetTexture("_BaseColorMap", FloorTexture(L["floor_value"].Double, (int)L["floor_texture_px"].Double));
            double tile = L["floor_tile_m"].Double, margin = spec.O["scene.floor_margin"].Double;
            fm.SetTextureScale("_BaseColorMap", new Vector2((float)((W + 2 * margin) / tile), (float)((H + 2 * margin) / tile)));
            Smooth(fm, L["floor_smoothness"].Double);
            floor.sharedMaterial = fm;
            // outside the arena: a large dark ground, so there is no void beyond the walls
            FleetGeometry.Visual(PrimitiveType.Cube, "outer_ground", stage, FleetFrames.World(W / 2, H / 2, -0.06), Quaternion.identity,
                FleetFrames.WorldScale(W + 30, H + 30, 0.1), Rgb(L["outer_color"]));

            var wallMat = FleetGeometry.Material(Rgb(L["wall_color"])); Smooth(wallMat, L["wall_smoothness"].Double);
            foreach (Transform t in root) {
                if (t.name.StartsWith("wall_") || t.name.StartsWith("divider")) {
                    t.GetComponent<Renderer>().sharedMaterial = wallMat;
                    if (t.name == "wall_south") t.gameObject.layer = CutawayLayer;
                }
            }
            // ceiling: closes the room for sensor cameras; cut away for cinematic cameras
            var ceil = FleetGeometry.Visual(PrimitiveType.Cube, "ceiling", stage, FleetFrames.World(W / 2, H / 2, wallH + 0.05), Quaternion.identity,
                FleetFrames.WorldScale(W + 0.3, H + 0.3, 0.1), Rgb(L["wall_color"]));
            ceil.layer = CutawayLayer;

            // door jambs (safety yellow) so the passage reads from far away
            JsonNode door = s.TryGet("doorway");
            if (door != null) {
                foreach (var w in s["walls"].Items) {
                    if (w["id"].String != door["in_wall"].String) continue;
                    double x = w["from"].Doubles[0], th = w["thickness"].Double, cy = door["centre_y"].Double, dw = world.DoorwayWidth, dh = door["height"].Double;
                    double jw = L["jamb_width"].Double;
                    Color jc = Rgb(L["jamb_color"]);
                    foreach (double side in new[] { -1.0, 1.0 }) {
                        FleetGeometry.Visual(PrimitiveType.Cube, "jamb", stage, FleetFrames.World(x, cy + side * (dw / 2 + jw / 2), dh / 2), Quaternion.identity,
                            FleetFrames.WorldScale(th + 0.04, jw, dh), jc);
                    }
                    FleetGeometry.Visual(PrimitiveType.Cube, "jamb_head", stage, FleetFrames.World(x, cy, dh + jw / 2), Quaternion.identity,
                        FleetFrames.WorldScale(th + 0.04, dw + 2 * jw, jw), jc);
                    // floor tape through the doorway
                    FleetGeometry.Visual(PrimitiveType.Cube, "door_tape", stage, FleetFrames.World(x, cy, 0.003), Quaternion.identity,
                        FleetFrames.WorldScale(th + 0.6, dw, 0.002), Color.Lerp(Rgb(L["jamb_color"]), new Color(0.5f, 0.5f, 0.5f), 0.4f));
                }
            }

            // obstacles: shelf, pallet stack, gantry beam
            var wood = Rgb(L["wood_color"]); var steel = Rgb(L["steel_color"]); var carton = Rgb(L["carton_color"]);
            foreach (var ob in s["obstacles"].Items) {
                string id = ob["id"].String;
                double[] c = ob["centre"].Doubles, sz = ob["size"].Doubles;
                var t = root.Find(id);
                if (t == null) continue;
                if (id.StartsWith("shelf")) {
                    t.GetComponent<Renderer>().enabled = false;
                    for (int k = 0; k < 4; k++) {
                        double z = 0.12 + k * (sz[2] - 0.15) / 3;
                        FleetGeometry.Visual(PrimitiveType.Cube, id + "_board", stage, FleetFrames.World(c[0], c[1], z), Quaternion.identity, FleetFrames.WorldScale(sz[0], sz[1], 0.04), wood);
                        for (int j = 0; j < 4; j++)   // boxes on the boards
                            FleetGeometry.Visual(PrimitiveType.Cube, id + "_box", stage, FleetFrames.World(c[0], c[1] - sz[1] / 2 + (j + 0.5) * sz[1] / 4, z + 0.02 + 0.13), Quaternion.identity,
                                FleetFrames.WorldScale(sz[0] * 0.8, sz[1] / 4 * 0.8, 0.26), Color.Lerp(carton, wood, (k + j) % 3 * 0.25f));
                    }
                    foreach (double side in new[] { -1.0, 1.0 })
                        FleetGeometry.Visual(PrimitiveType.Cube, id + "_post", stage, FleetFrames.World(c[0], c[1] + side * (sz[1] / 2 - 0.02), c[2]), Quaternion.identity, FleetFrames.WorldScale(sz[0], 0.04, sz[2]), steel);
                    FleetGeometry.Visual(PrimitiveType.Cube, id + "_back", stage, FleetFrames.World(c[0] + sz[0] / 2 - 0.01, c[1], c[2]), Quaternion.identity, FleetFrames.WorldScale(0.02, sz[1], sz[2]), steel);
                }
                else if (id.StartsWith("pallet")) {
                    t.GetComponent<Renderer>().enabled = false;
                    FleetGeometry.Visual(PrimitiveType.Cube, id + "_pallet", stage, FleetFrames.World(c[0], c[1], 0.07), Quaternion.identity, FleetFrames.WorldScale(sz[0], sz[1], 0.14), wood);
                    double h = (sz[2] - 0.14) / 3;
                    for (int k = 0; k < 3; k++)
                        FleetGeometry.Visual(PrimitiveType.Cube, id + "_crate", stage, FleetFrames.World(c[0], c[1], 0.14 + h * (k + 0.5)), Quaternion.identity,
                            FleetFrames.WorldScale(sz[0] * 0.92, sz[1] * 0.92, h * 0.97), carton);
                }
                else t.GetComponent<Renderer>().sharedMaterial = FleetGeometry.Material(steel);
            }

            // zones: tinted patch (fraction toward the zone colour) and border tape
            double tint = L["zone_tint"].Double, tape = L["zone_tape_width"].Double;
            Color baseFloor = Rgb(spec.O["visual.floor_color"]);
            foreach (var kv in s["zones"].Members) {
                var z = root.Find("zone_" + kv.Key);
                if (z == null) continue;
                Color zc = Rgb(spec.O["visual.zone_colors." + kv.Key]);
                z.GetComponent<Renderer>().sharedMaterial = FleetGeometry.Material(Color.Lerp(baseFloor, zc, (float)tint));
                double[] r = kv.Value["rect"].Doubles;
                foreach (var (cx, cy, sx, sy) in new[] { ((r[0] + r[2]) / 2, r[1], r[2] - r[0], tape), ((r[0] + r[2]) / 2, r[3], r[2] - r[0], tape), (r[0], (r[1] + r[3]) / 2, tape, r[3] - r[1]), (r[2], (r[1] + r[3]) / 2, tape, r[3] - r[1]) })
                    FleetGeometry.Visual(PrimitiveType.Cube, "tape_" + kv.Key, stage, FleetFrames.World(cx, cy, 0.004), Quaternion.identity, FleetFrames.WorldScale(sx, sy, 0.002), zc);
            }

            LightRig(stage, L, W, H, wallH);
            Volumes(stage, L);
        }

        static void Smooth(Material m, double v) { if (m.HasProperty("_Smoothness")) m.SetFloat("_Smoothness", (float)v); }
        static Color Rgb(JsonNode n) => FleetFrames.Rgb(n.Doubles);

        static void LightRig(Transform stage, JsonNode L, double W, double H, double wallH) {
            JsonNode k = L["key"];
            int nx = (int)k["grid"].Doubles[0], ny = (int)k["grid"].Doubles[1];
            double z = k["height"].Double;
            for (int i = 0; i < nx; i++)
                for (int j = 0; j < ny; j++)
                    Rect(stage, $"key_{i}{j}", FleetFrames.World(W * (i + 0.5) / nx, H * (j + 0.5) / ny, z), Quaternion.Euler(90, 0, 0), k["size"].Doubles, k["lumen"].Double, k["kelvin"].Double, true);
            foreach (var n in L["extra_lights"].Items) {
                double[] p = n["position"].Doubles, e = n["euler_deg"].Doubles;
                Rect(stage, n["name"].String, FleetFrames.World(p[0], p[1], p[2]), Quaternion.Euler((float)e[0], (float)e[1], (float)e[2]), n["size"].Doubles, n["lumen"].Double, n["kelvin"].Double, n["shadows"].Bool);
            }
        }

        static void Rect(Transform parent, string name, Vector3 pos, Quaternion rot, double[] size, double lumen, double kelvin, bool shadows) {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            go.transform.SetPositionAndRotation(pos, rot);
            var l = go.AddComponent<Light>();
            l.type = LightType.Rectangle;
            if (!go.TryGetComponent<HDAdditionalLightData>(out _)) go.AddComponent<HDAdditionalLightData>();
            l.areaSize = new Vector2((float)size[0], (float)size[1]);
            l.useColorTemperature = true;
            l.colorTemperature = (float)kelvin;
            l.color = Color.white;
            l.lightUnit = LightUnit.Lumen;
            l.intensity = (float)lumen;
            l.range = 12f;
            l.shadows = shadows ? LightShadows.Soft : LightShadows.None;
        }

        static void Volumes(Transform stage, JsonNode L) {
            JsonNode e = L["exposure"];
            Cinematic = ScriptableObject.CreateInstance<VolumeProfile>(); Cinematic.name = "fleet cinematic";
            var exp = Cinematic.Add<Exposure>(false);
            exp.mode.Override(ExposureMode.Automatic); exp.meteringMode.Override(MeteringMode.CenterWeighted);
            exp.adaptationMode.Override(AdaptationMode.Fixed); exp.limitMin.Override((float)e["limit_min"].Double); exp.limitMax.Override((float)e["limit_max"].Double);
            exp.compensation.Override(0f);
            Cinematic.Add<Tonemapping>(false).mode.Override(TonemappingMode.ACES);
            var bloom = Cinematic.Add<Bloom>(false); bloom.intensity.Override(0.12f); bloom.threshold.Override(1.1f); bloom.scatter.Override(0.55f);
            var ca = Cinematic.Add<ColorAdjustments>(false); ca.contrast.Override(6f); ca.saturation.Override(-4f);
            var vg = Cinematic.Add<Vignette>(false); vg.intensity.Override(0.18f); vg.smoothness.Override(0.5f);
            Cinematic.Add<MotionBlur>(false).intensity.Override(0f);
            var dof = Cinematic.Add<DepthOfField>(false); dof.focusMode.Override(DepthOfFieldMode.UsePhysicalCamera); dof.focusDistanceMode.Override(FocusDistanceMode.Volume); dof.focusDistance.Override(2f);
            dof.active = false;
            AddLighting(Cinematic, L);
            Sensor = ScriptableObject.CreateInstance<VolumeProfile>(); Sensor.name = "fleet sensor neutral";
            var se = Sensor.Add<Exposure>(false);
            se.mode.Override(ExposureMode.Automatic); se.meteringMode.Override(MeteringMode.Average); se.adaptationMode.Override(AdaptationMode.Fixed);
            se.limitMin.Override((float)e["limit_min"].Double); se.limitMax.Override((float)e["limit_max"].Double); se.compensation.Override(0f);
            Sensor.Add<Tonemapping>(false).mode.Override(TonemappingMode.Neutral);
            AddLighting(Sensor, L);
            Volume(stage, "volume cinematic", Cinematic, CinematicLayer);
            Volume(stage, "volume sensor", Sensor, SensorLayer);
        }

        /// <summary>Geometry-lighting components shared by both profiles (not look effects): sky, ambient, AO, contact shadows, no fog.</summary>
        static void AddLighting(VolumeProfile p, JsonNode L) {
            var ao = p.Add<ScreenSpaceAmbientOcclusion>(false);
            ao.intensity.Override(0.9f); ao.radius.Override(0.5f); ao.directLightingStrength.Override(0.35f);
            var cs = p.Add<ContactShadows>(false); cs.enable.Override(true); cs.length.Override(0.15f); cs.opacity.Override(0.85f); cs.maxDistance.Override(30f);
            var ve = p.Add<VisualEnvironment>(false); ve.skyType.Override((int)SkyType.Gradient); ve.skyAmbientMode.Override(SkyAmbientMode.Dynamic);
            var sky = p.Add<GradientSky>(false);
            sky.top.Override(new Color(0.34f, 0.38f, 0.44f)); sky.middle.Override(new Color(0.30f, 0.32f, 0.35f)); sky.bottom.Override(new Color(0.22f, 0.22f, 0.22f));
            sky.skyIntensityMode.Override(SkyIntensityMode.Exposure); sky.exposure.Override((float)L["sky_exposure"].Double);
            p.Add<Fog>(false).enabled.Override(false);
        }

        static void Volume(Transform parent, string name, VolumeProfile profile, int layer) {
            var go = new GameObject(name) { layer = layer };
            go.transform.SetParent(parent, false);
            var v = go.AddComponent<Volume>();
            v.isGlobal = true; v.priority = 1000; v.weight = 1f; v.sharedProfile = profile;
        }

        // ---------------------------------------------------------------- cameras

        public static void ConfigureCinematic(Camera cam, int w, int h, double fovV, double aperture, double focus) {
            cam.usePhysicalProperties = true; cam.sensorSize = new Vector2(36f, 24f);
            cam.nearClipPlane = 0.03f; cam.farClipPlane = 80f; cam.aspect = w / (float)h;
            cam.cullingMask &= ~(1 << CutawayLayer);
            var hd = cam.GetComponent<HDAdditionalCameraData>() ?? cam.gameObject.AddComponent<HDAdditionalCameraData>();
            hd.volumeLayerMask = (1 << 0) | (1 << CinematicLayer);
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing; hd.SMAAQuality = HDAdditionalCameraData.SMAAQualityLevel.High;
            hd.allowDynamicResolution = false;
            cam.focalLength = (float)(24.0 * 0.5 / Math.Tan(fovV * 0.5 * Math.PI / 180.0));
            cam.aperture = aperture > 0 ? (float)aperture : 16f;
            if (Cinematic != null && Cinematic.TryGet<DepthOfField>(out var dof)) {
                dof.active = aperture > 0;
                dof.focusDistance.Override((float)Math.Max(0.2, focus));
            }
        }

        public static List<string> ConfigureSensor(Camera cam, int w, int h, double hfovRad) {
            cam.usePhysicalProperties = false; cam.aspect = w / (float)h;
            cam.fieldOfView = Camera.HorizontalToVerticalFieldOfView((float)(hfovRad * 180 / Math.PI), cam.aspect);
            cam.nearClipPlane = 0.02f; cam.farClipPlane = 40f;
            var hd = cam.GetComponent<HDAdditionalCameraData>() ?? cam.gameObject.AddComponent<HDAdditionalCameraData>();
            hd.volumeLayerMask = 1 << SensorLayer;
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.None; hd.allowDynamicResolution = false; hd.customRenderingSettings = true;
            var missing = new List<string>();
            foreach (var n in new[] { "DepthOfField", "MotionBlur", "Bloom", "Vignette", "ColorGrading", "ChromaticAberration", "FilmGrain", "LensDistortion", "PaniniProjection" })
                if (!SetFrameSetting(hd, n, false)) missing.Add(n);
            return missing;
        }

        static bool SetFrameSetting(HDAdditionalCameraData hd, string field, bool on) {
            if (!Enum.TryParse<FrameSettingsField>(field, out var f)) return false;
            hd.renderingPathCustomFrameSettings.SetEnabled(f, on);
            hd.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)f] = true;
            return true;
        }

        /// <summary>Deterministic tile texture: mid grey tile, 0.25 m joints, heavier joint on the tile edge, fixed-seed mottling.</summary>
        static Texture2D FloorTexture(double value, int n) {
            var tex = new Texture2D(n, n, TextureFormat.RGBA32, true) { wrapMode = TextureWrapMode.Repeat, filterMode = FilterMode.Trilinear, anisoLevel = 8, name = "fleet floor" };
            var px = new Color[n * n];
            var rng = new FleetRng(7);
            int fine = n / 8;
            for (int y = 0; y < n; y++)
                for (int x = 0; x < n; x++) {
                    float v = (float)(value + (Math.Sin(x * 0.07 + y * 0.03) + Math.Sin(y * 0.11)) * 0.012 + (rng.Uniform() - 0.5) * 0.03);
                    if (x % fine < 2 || y % fine < 2) v *= 0.78f;
                    if (x < 3 || y < 3 || x > n - 4 || y > n - 4) v *= 0.6f;
                    px[y * n + x] = new Color(v, v * 0.99f, v * 0.96f, 1f);
                }
            tex.SetPixels(px); tex.Apply(true);
            return tex;
        }
    }
}
