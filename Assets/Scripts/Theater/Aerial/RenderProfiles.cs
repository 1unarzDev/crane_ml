using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Theater.Aerial {
    /// <summary>
    /// OAK-D S2 camera parameters. Source: Luxonis OAK-D S2 product page and datasheet,
    /// https://docs.luxonis.com/hardware/products/OAK-D%20S2 (fetched 2026-11-10):
    ///   RGB    IMX378 1/2.3", 4056x3040 (12 MP), rolling shutter, fixed-focus variant HFOV 69 / VFOV 55 / DFOV 82 deg
    ///          (auto-focus variant 66 / 54 / 78 deg).
    ///   Stereo OV9282 mono 1/4", 1280x800, global shutter, fixed focus, HFOV 80 / VFOV 55 / DFOV 89.5 deg, baseline 7.5 cm.
    /// The RGB stream is rendered as 1920x1080 (the ISP's 16:9 crop of the 4:3 sensor keeps the horizontal FOV, so the
    /// vertical FOV is derived from HFOV and aspect). The focal length and f-number are not on the product page.
    /// </summary>
    public static class OakD {
        public const float RgbHfovDeg = 69f, RgbVfovSensorDeg = 55f;
        public const int RgbSensorW = 4056, RgbSensorH = 3040, RgbW = 1920, RgbH = 1080;
        public const float StereoHfovDeg = 80f, StereoVfovDeg = 55f;
        public const int StereoW = 1280, StereoH = 800;
        public const float BaselineM = 0.075f;
        public const float RgbRollingReadoutS = 0.030f;   // assumed full-frame readout, not on the datasheet; a hook default

        public static float VerticalFov(float hfovDeg, int w, int h) =>
            2f * Mathf.Atan(Mathf.Tan(hfovDeg * 0.5f * Mathf.Deg2Rad) * h / w) * Mathf.Rad2Deg;
        public static float FocalPx(float hfovDeg, int w) => 0.5f * w / Mathf.Tan(hfovDeg * 0.5f * Mathf.Deg2Rad);
        public static float RgbVfov => VerticalFov(RgbHfovDeg, RgbW, RgbH);
        public static float StereoVfov => VerticalFov(StereoHfovDeg, StereoW, StereoH);
    }

    /// <summary>The two strictly separate render profiles. Cinematic uses volume layer <see cref="CinematicLayer"/> plus
    /// the scene's own volumes; onboard uses only layer <see cref="OnboardLayer"/>, whose profile is built from the
    /// scene's NON-cinematic components (sky, fog, lighting, shadows) and never contains a cinematic effect.</summary>
    public static class RenderProfiles {
        public const int CinematicLayer = 30, OnboardLayer = 31;

        /// <summary>Every volume component type that is a look or lens effect. None may appear in an onboard profile.</summary>
        public static readonly Type[] CinematicEffects = {
            typeof(Bloom), typeof(DepthOfField), typeof(MotionBlur), typeof(ColorAdjustments), typeof(ColorCurves),
            typeof(LiftGammaGain), typeof(ShadowsMidtonesHighlights), typeof(SplitToning), typeof(ChannelMixer), typeof(WhiteBalance),
            typeof(Vignette), typeof(FilmGrain), typeof(ChromaticAberration), typeof(LensDistortion), typeof(PaniniProjection),
            typeof(ScreenSpaceLensFlare)
        };

        public static bool IsCinematicEffect(VolumeComponent c) {
            if (c == null) return false;
            foreach (var t in CinematicEffects) if (t.IsInstanceOfType(c)) return true;
            return false;
        }

        /// <summary>The cinematic effect components present in a profile (active or not). Empty for a valid onboard profile.</summary>
        public static List<string> CinematicOverrides(VolumeProfile p) {
            var r = new List<string>();
            if (p == null) return r;
            foreach (var c in p.components) if (IsCinematicEffect(c)) r.Add(c.GetType().Name);
            return r;
        }

        // ------------------------------------------------------------------ cinematic

        public static VolumeProfile BuildCinematic() {
            var p = ScriptableObject.CreateInstance<VolumeProfile>(); p.name = "Aerial Cinematic";
            var exp = p.Add<Exposure>(false);
            exp.mode.Override(ExposureMode.Automatic);
            exp.meteringMode.Override(MeteringMode.CenterWeighted);
            exp.adaptationMode.Override(AdaptationMode.Progressive);
            exp.adaptationSpeedDarkToLight.Override(2.5f); exp.adaptationSpeedLightToDark.Override(1.5f);
            exp.compensation.Override(0.9f); exp.limitMin.Override(-1f); exp.limitMax.Override(13f);
            var tm = p.Add<Tonemapping>(false); tm.mode.Override(TonemappingMode.ACES);
            var bloom = p.Add<Bloom>(false); bloom.intensity.Override(0.22f); bloom.threshold.Override(0.9f); bloom.scatter.Override(0.65f);
            var mb = p.Add<MotionBlur>(false); mb.intensity.Override(0.7f); mb.maximumVelocity.Override(250f); mb.minimumVelocity.Override(2f);
            var dof = p.Add<DepthOfField>(false);
            dof.focusMode.Override(DepthOfFieldMode.UsePhysicalCamera); dof.focusDistanceMode.Override(FocusDistanceMode.Volume);
            dof.focusDistance.Override(8f);
            var ca = p.Add<ColorAdjustments>(false);
            ca.postExposure.Override(0f); ca.contrast.Override(14f); ca.saturation.Override(10f); ca.colorFilter.Override(new Color(1f, 0.97f, 0.93f));
            var wb = p.Add<WhiteBalance>(false); wb.temperature.Override(6f); wb.tint.Override(-2f);
            var st = p.Add<SplitToning>(false); st.shadows.Override(new Color(0.42f, 0.55f, 0.62f)); st.highlights.Override(new Color(1f, 0.82f, 0.62f)); st.balance.Override(-15f);
            var vg = p.Add<Vignette>(false); vg.intensity.Override(0.2f); vg.smoothness.Override(0.6f);
            var fg = p.Add<FilmGrain>(false); fg.type.Override(FilmGrainLookup.Thin1); fg.intensity.Override(0.18f); fg.response.Override(0.8f);
            return p;
        }

        public static void ConfigureCinematicCamera(Camera cam, int w, int h) {
            cam.usePhysicalProperties = true;
            cam.sensorSize = new Vector2(36f, 24f);
            cam.nearClipPlane = 0.1f; cam.farClipPlane = 4000f;
            cam.aspect = w / (float)h;
            var hd = cam.gameObject.GetComponent<HDAdditionalCameraData>() ?? cam.gameObject.AddComponent<HDAdditionalCameraData>();
            hd.volumeLayerMask = (1 << 0) | (1 << CinematicLayer);   // the scene's own volumes plus the cinematic one
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            hd.SMAAQuality = HDAdditionalCameraData.SMAAQualityLevel.High;
            hd.allowDynamicResolution = false;
            hd.customRenderingSettings = true;
            foreach (var name in new[] { "DepthOfField", "MotionBlur", "Bloom", "Vignette", "ColorGrading", "Tonemapping", "ChromaticAberration", "FilmGrain", "ExposureControl" })
                SetFrameSetting(hd, name, true);
        }

        /// <summary>Set the lens from a horizontal-agnostic vertical FOV (physical camera: focal length from FOV and 24 mm sensor height).</summary>
        public static void SetCinematicLens(Camera cam, float vfovDeg, float aperture) {
            cam.focalLength = 24f * 0.5f / Mathf.Tan(vfovDeg * 0.5f * Mathf.Deg2Rad);
            cam.aperture = aperture > 0 ? aperture : 8f;
        }

        // ------------------------------------------------------------------ onboard

        /// <summary>
        /// The onboard profile: <paramref name="sceneProfiles"/> (highest priority first) contribute their NON-cinematic
        /// components (sky, fog, volumetrics, shadows, lighting); a sensor-like Exposure and a neutral Tonemapping
        /// complete it. A cinematic effect present in a scene profile is dropped, never disabled in place.
        /// </summary>
        public static VolumeProfile BuildOnboard(IEnumerable<VolumeProfile> sceneProfiles) {
            var p = ScriptableObject.CreateInstance<VolumeProfile>(); p.name = "Aerial Onboard Sensor";
            if (sceneProfiles != null)
                foreach (var sp in sceneProfiles) {
                    if (sp == null) continue;
                    foreach (var c in sp.components) {
                        if (c == null || IsCinematicEffect(c) || c is Exposure || c is Tonemapping) continue;
                        if (p.Has(c.GetType())) continue;
                        p.components.Add(UnityEngine.Object.Instantiate(c));
                    }
                }
            // Sensor-like exposure: the camera ISP's auto-exposure, instant (deterministic offline), no creative compensation.
            var exp = p.Add<Exposure>(false);
            exp.mode.Override(ExposureMode.Automatic); exp.meteringMode.Override(MeteringMode.Average);
            exp.adaptationMode.Override(AdaptationMode.Fixed); exp.compensation.Override(0f);
            exp.limitMin.Override(-1f); exp.limitMax.Override(14f);
            var tm = p.Add<Tonemapping>(false); tm.mode.Override(TonemappingMode.Neutral);   // ISP-like curve, no filmic look
            return p;
        }

        public static void ConfigureOnboardCamera(Camera cam, float hfovDeg, int w, int h) {
            cam.usePhysicalProperties = false;
            cam.aspect = w / (float)h;
            cam.fieldOfView = OakD.VerticalFov(hfovDeg, w, h);
            cam.nearClipPlane = 0.05f; cam.farClipPlane = 1500f;
            var hd = cam.gameObject.GetComponent<HDAdditionalCameraData>() ?? cam.gameObject.AddComponent<HDAdditionalCameraData>();
            hd.volumeLayerMask = 1 << OnboardLayer;                  // only the onboard profile, never the scene's look volumes
            hd.antialiasing = HDAdditionalCameraData.AntialiasingMode.None;   // a raw sensor is aliased
            hd.allowDynamicResolution = false;
            hd.customRenderingSettings = true;
            foreach (var name in new[] { "DepthOfField", "MotionBlur", "Bloom", "Vignette", "ColorGrading", "ChromaticAberration", "FilmGrain", "LensDistortion", "PaniniProjection" })
                SetFrameSetting(hd, name, false);
            foreach (var name in new[] { "Tonemapping", "ExposureControl" }) SetFrameSetting(hd, name, true);
        }

        /// <summary>Frame settings by name; fields a given HDRP version lacks are skipped.</summary>
        public static bool SetFrameSetting(HDAdditionalCameraData hd, string field, bool on) {
            if (!Enum.TryParse<FrameSettingsField>(field, out var f)) return false;
            hd.renderingPathCustomFrameSettings.SetEnabled(f, on);
            hd.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)f] = true;
            return true;
        }

        public static bool FrameSettingOn(HDAdditionalCameraData hd, string field) {
            if (!Enum.TryParse<FrameSettingsField>(field, out var f)) return false;
            return hd.renderingPathCustomFrameSettings.IsEnabled(f);
        }

        // ------------------------------------------------------------------ volumes

        public static Volume MakeVolume(string name, VolumeProfile profile, int layer, float priority) {
            var go = new GameObject(name) { layer = layer };
            var v = go.AddComponent<Volume>();
            v.isGlobal = true; v.priority = priority; v.weight = 1f; v.sharedProfile = profile;
            return v;
        }
    }
}
