using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Drones.Environment.Editor
{
    /// <summary>Volume profiles (base, 3 lighting presets, FX toggles), sun and director wiring. Parameters set by reflection so
    /// a renamed HDRP field logs a warning instead of breaking the build.</summary>
    public static class ForestLighting
    {
        public static bool Try(object o, string name, object val)
        {
            var t = o.GetType(); const BindingFlags f = BindingFlags.Public | BindingFlags.Instance;
            var fi = t.GetField(name, f); var pi = t.GetProperty(name, f);
            try
            {
                if (fi != null) { fi.SetValue(o, Conv(fi.FieldType, val)); return true; }
                if (pi != null && pi.CanWrite) { pi.SetValue(o, Conv(pi.PropertyType, val)); return true; }
            }
            catch (Exception e) { Debug.LogWarning("[DroneForest] set " + t.Name + "." + name + " failed: " + e.Message); return false; }
            Debug.LogWarning("[DroneForest] missing member " + t.Name + "." + name); return false;
        }
        static object Conv(Type t, object v)
        {
            if (v == null) return null;
            if (t.IsEnum) return v is string ? Enum.Parse(t, (string)v, true) : Enum.ToObject(t, Convert.ToInt32(v));
            if (t == typeof(float)) return Convert.ToSingle(v);
            if (t == typeof(int)) return Convert.ToInt32(v);
            return v;
        }
        public static void P(VolumeComponent c, string field, object val)
        {
            var fi = c.GetType().GetField(field, BindingFlags.Public | BindingFlags.Instance);
            if (fi == null) { Debug.LogWarning("[DroneForest] missing param " + c.GetType().Name + "." + field); return; }
            var par = (VolumeParameter)fi.GetValue(c); par.overrideState = true;
            var pi = par.GetType().GetProperty("value");
            try { pi.SetValue(par, Conv(pi.PropertyType, val)); }
            catch (Exception e) { Debug.LogWarning("[DroneForest] param " + c.GetType().Name + "." + field + ": " + e.Message); }
        }

        static T Add<T>(VolumeProfile prof) where T : VolumeComponent
        {
            var c = prof.Add<T>(false); c.active = true; c.name = typeof(T).Name;
            AssetDatabase.AddObjectToAsset(c, prof); return c;
        }

        static VolumeProfile NewProfile(string path)
        {
            AssetDatabase.DeleteAsset(path);
            var p = ScriptableObject.CreateInstance<VolumeProfile>(); AssetDatabase.CreateAsset(p, path); return p;
        }

        public class PresetDef
        {
            public string name; public Vector3 sunEuler; public float sunLux, sunKelvin, shadowStrength, volDimmer, expComp;
            public float meanFreePath, aniso, aerosol, satSky, cloudsPreset; public Color haze, fogAlbedo, airTint, filter, shadowTone, highlightTone, groundTint;
            public float wbTemp, wbTint, saturation, contrast;
        }

        public static List<PresetDef> Presets()
        {
            return new List<PresetDef>
            {
                // natural warm light: neutral-white sun disc (the physical sky already reddens a low sun), cool-neutral shadows, restrained split toning
                new PresetDef{ name="GoldenHour", sunEuler=new Vector3(15f,218f,0), sunLux=105000, sunKelvin=5700, shadowStrength=1f, volDimmer=1f, expComp=0.15f,
                    meanFreePath=75, aniso=0.72f, aerosol=0.03f, satSky=1f, cloudsPreset=0, fogAlbedo=new Color(0.86f,0.82f,0.74f), airTint=Color.white, filter=Color.white, haze=new Color(0.80f,0.76f,0.68f),
                    shadowTone=new Color(0.52f,0.60f,0.74f), highlightTone=new Color(0.98f,0.84f,0.64f), groundTint=new Color(0.15f,0.13f,0.10f), wbTemp=4, wbTint=0, saturation=12, contrast=8 },
                new PresetDef{ name="Overcast", sunEuler=new Vector3(52f,160f,0), sunLux=14000, sunKelvin=6800, shadowStrength=0.16f, volDimmer=0.35f, expComp=0.8f,
                    meanFreePath=100, aniso=0.2f, aerosol=0.35f, satSky=0.35f, cloudsPreset=2, fogAlbedo=new Color(0.78f,0.82f,0.84f), airTint=new Color(0.8f,0.85f,0.9f), filter=Color.white, haze=new Color(0.64f,0.68f,0.72f),
                    shadowTone=new Color(0.45f,0.55f,0.6f), highlightTone=new Color(0.8f,0.85f,0.85f), groundTint=new Color(0.15f,0.14f,0.12f), wbTemp=-4, wbTint=-3, saturation=-6, contrast=4 },
                new PresetDef{ name="Dusk", sunEuler=new Vector3(4.5f,250f,0), sunLux=7000, sunKelvin=6600, shadowStrength=0.9f, volDimmer=0.8f, expComp=1.5f,
                    meanFreePath=55, aniso=0.62f, aerosol=0.10f, satSky=1f, cloudsPreset=1, fogAlbedo=new Color(0.62f,0.62f,0.72f), airTint=new Color(0.85f,0.85f,1f), filter=Color.white, haze=new Color(0.50f,0.50f,0.60f),
                    shadowTone=new Color(0.3f,0.4f,0.7f), highlightTone=new Color(0.86f,0.76f,0.7f), groundTint=new Color(0.08f,0.07f,0.07f), wbTemp=-14, wbTint=2, saturation=-8, contrast=8 },
            };
        }

        public static void BuildProfiles(string dir, out VolumeProfile baseP, out Dictionary<string, VolumeProfile> presets, out VolumeProfile dof, out VolumeProfile grain, out VolumeProfile mb, out VolumeProfile aerial, DiffusionProfileSettings foliageProfile)
        {
            baseP = NewProfile(dir + "/Forest_Base.asset");
            var ve = Add<VisualEnvironment>(baseP); P(ve, "skyType", 4); P(ve, "skyAmbientMode", 1);
            Add<PhysicallyBasedSky>(baseP);
            var shadows = Add<HDShadowSettings>(baseP); P(shadows, "maxShadowDistance", 150f); P(shadows, "cascadeShadowSplitCount", 4);
            P(shadows, "cascadeShadowSplit0", 0.03f); P(shadows, "cascadeShadowSplit1", 0.09f); P(shadows, "cascadeShadowSplit2", 0.26f);
            P(shadows, "directionalTransmissionMultiplier", 1f);
            var cs = Add<ContactShadows>(baseP); P(cs, "enable", true); P(cs, "length", 0.14f); P(cs, "opacity", 0.8f); P(cs, "distanceScaleFactor", 0.6f); P(cs, "sampleCount", 14);
            var ao = Add<ScreenSpaceAmbientOcclusion>(baseP); P(ao, "intensity", 0.9f); P(ao, "radius", 1.0f); P(ao, "directLightingStrength", 0.25f); P(ao, "fullResolution", true); P(ao, "stepCount", 8); P(ao, "temporalAccuracy", 0.85f); P(ao, "ghostingReduction", 0.5f);
            var gi = Add<GlobalIllumination>(baseP); P(gi, "enable", true); P(gi, "fullResolutionSS", true);
            var fog = Add<Fog>(baseP); P(fog, "enabled", true); P(fog, "enableVolumetricFog", true); P(fog, "colorMode", "SkyColor");
            P(fog, "depthExtent", 200f); P(fog, "sliceDistributionUniformity", 0.7f); P(fog, "maxFogDistance", 2500f); P(fog, "globalLightProbeDimmer", 1f);
            var tm = Add<Tonemapping>(baseP); P(tm, "mode", "ACES");
            var ex = Add<Exposure>(baseP); P(ex, "mode", "AutomaticHistogram"); P(ex, "limitMin", -1.5f); P(ex, "limitMax", 11.5f);
            P(ex, "adaptationMode", "Progressive"); P(ex, "adaptationSpeedDarkToLight", 2.5f); P(ex, "adaptationSpeedLightToDark", 1.5f);
            var bl = Add<Bloom>(baseP); P(bl, "intensity", 0.14f); P(bl, "threshold", 0.9f); P(bl, "scatter", 0.62f);
            var vg = Add<Vignette>(baseP); P(vg, "intensity", 0.2f); P(vg, "smoothness", 0.5f);
            var ca = Add<ColorAdjustments>(baseP); P(ca, "contrast", 8f); P(ca, "saturation", 4f);
            var dpl = Add<DiffusionProfileList>(baseP); if (foliageProfile != null) P(dpl, "diffusionProfiles", new DiffusionProfileSettings[] { foliageProfile });
            
            EditorUtility.SetDirty(baseP);

            presets = new Dictionary<string, VolumeProfile>();
            foreach (var d in Presets())
            {
                var p = NewProfile(dir + "/Forest_" + d.name + ".asset");
                var sky = Add<PhysicallyBasedSky>(p); P(sky, "aerosolDensity", d.aerosol); P(sky, "airTint", d.airTint); P(sky, "colorSaturation", d.satSky); P(sky, "groundTint", d.groundTint);
                var f = Add<Fog>(p); P(f, "meanFreePath", d.meanFreePath); P(f, "albedo", d.fogAlbedo); P(f, "anisotropy", d.aniso);
                var e = Add<Exposure>(p); P(e, "compensation", d.expComp);
                var w = Add<WhiteBalance>(p); P(w, "temperature", d.wbTemp); P(w, "tint", d.wbTint);
                var c = Add<ColorAdjustments>(p); P(c, "saturation", d.saturation); P(c, "contrast", d.contrast);
                var s = Add<SplitToning>(p); P(s, "shadows", d.shadowTone); P(s, "highlights", d.highlightTone); P(s, "balance", d.name == "Overcast" ? 0f : 15f);
                var cl = Add<VolumetricClouds>(p); P(cl, "enable", true); P(cl, "cloudControl", "Simple"); P(cl, "cloudSimpleMode", "Performance");
                // HDRP 17.5: cloudSimpleMode is the Performance/Quality switch; the weather preset is the cloudPreset property (its setter fills the shape parameters)
                Try(cl, "cloudPreset", d.cloudsPreset == 0 ? "Sparse" : (d.cloudsPreset == 1 ? "Cloudy" : "Overcast"));
                foreach (var fi in cl.GetType().GetFields(BindingFlags.Public | BindingFlags.Instance)) { var vp = fi.GetValue(cl) as VolumeParameter; if (vp != null && fi.Name != "cloudControl") vp.overrideState = true; }
                EditorUtility.SetDirty(p); presets[d.name] = p;
            }
            dof = NewProfile(dir + "/Forest_FX_DepthOfField.asset");
            var dd = Add<DepthOfField>(dof); P(dd, "focusMode", "Manual"); P(dd, "focusDistanceMode", "Volume"); P(dd, "focusDistance", 8f);
            P(dd, "nearFocusStart", 0f); P(dd, "nearFocusEnd", 3f); P(dd, "farFocusStart", 14f); P(dd, "farFocusEnd", 55f);
            grain = NewProfile(dir + "/Forest_FX_FilmGrain.asset");
            var fg = Add<FilmGrain>(grain); P(fg, "type", "Thin1"); P(fg, "intensity", 0.3f); P(fg, "response", 0.8f);
            mb = NewProfile(dir + "/Forest_FX_MotionBlur.asset");
            var m = Add<MotionBlur>(mb); P(m, "intensity", 0.55f); P(m, "maximumVelocity", 180f);
            // aerial haze: non-volumetric exponential fog in a constant horizon colour, so distant terrain/forest fades into the sky without froxel streaking
            aerial = NewProfile(dir + "/Forest_FX_AerialHaze.asset");
            var af = Add<Fog>(aerial); P(af, "enabled", true); P(af, "enableVolumetricFog", false); P(af, "colorMode", "ConstantColor"); P(af, "color", new Color(0.8f, 0.76f, 0.68f)); P(af, "tint", new Color(0.8f, 0.76f, 0.68f));
            P(af, "meanFreePath", 1100f); P(af, "maxFogDistance", 9000f); P(af, "mipFogMaxMip", 0.4f);
            EditorUtility.SetDirty(aerial);
            foreach (var pr in new[] { dof, grain, mb }) EditorUtility.SetDirty(pr);
            AssetDatabase.SaveAssets();
        }

        public static Volume MakeVolume(Transform parent, string name, VolumeProfile prof, float priority, float weight)
        {
            var go = new GameObject(name); go.transform.SetParent(parent, false);
            var v = go.AddComponent<Volume>(); v.isGlobal = true; v.sharedProfile = prof; v.priority = priority; v.weight = weight; return v;
        }

        public static DroneForestDirector Build(Transform root, string dir, string startPreset, DiffusionProfileSettings foliageProfile)
        {
            System.IO.Directory.CreateDirectory(dir); AssetDatabase.Refresh();
            VolumeProfile bp, dof, grain, mbp, aer; Dictionary<string, VolumeProfile> pr;
            BuildProfiles(dir, out bp, out pr, out dof, out grain, out mbp, out aer, foliageProfile);
            var g = new GameObject("Lighting"); g.transform.SetParent(root, false);
            var sunGo = new GameObject("Sun"); sunGo.transform.SetParent(g.transform, false);
            var sun = sunGo.AddComponent<Light>(); sun.type = LightType.Directional; sun.shadows = LightShadows.Soft;
            var hd = sunGo.AddComponent<HDAdditionalLightData>();
            Try(sun, "lightUnit", "Lux"); sun.intensity = 90000f;
            Try(hd, "angularDiameter", 1.0f); Try(hd, "interactsWithSky", true); Try(hd, "affectsVolumetric", true); hd.volumetricDimmer = 1f;
            hd.shadowResolution.useOverride = true; hd.shadowResolution.@override = 4096;
            hd.useContactShadow.useOverride = true; hd.useContactShadow.@override = true;
            Try(hd, "shadowNearPlane", 0.2f);
            // soft PCSS-style filtering with more samples; modest slope/normal bias (alpha-clipped foliage cards otherwise show hatching)
            try { hd.SetPCSSParams(24, 36, 0.03f, 1.4f); } catch (Exception e) { Debug.LogWarning("[DroneForest] PCSS params: " + e.Message); }
            Try(hd, "slopeBias", 0.35f); Try(hd, "normalBias", 0.6f); Try(hd, "shadowFilteringQuality", "High");
            hd.EnableShadows(true);
            var vol = new GameObject("Volumes"); vol.transform.SetParent(g.transform, false);
            MakeVolume(vol.transform, "Volume_Base", bp, 0, 1);
            var dir_ = sunGo.AddComponent<DroneForestDirector>(); // director lives on the sun object for simplicity
            var list = new List<DroneForestDirector.Preset>();
            foreach (var d in Presets())
            {
                var v = MakeVolume(vol.transform, "Volume_Preset_" + d.name, pr[d.name], 1, 0);
                list.Add(new DroneForestDirector.Preset { name = d.name, volume = v, sunEuler = d.sunEuler, sunLux = d.sunLux, sunKelvin = d.sunKelvin, shadowStrength = d.shadowStrength, volumetricDimmer = d.volDimmer, hazeColor = d.haze });
            }
            dir_.sun = sun; dir_.presets = list.ToArray(); dir_.startPreset = startPreset;
            dir_.depthOfFieldVolume = MakeVolume(vol.transform, "Volume_FX_DepthOfField_HeroShots", dof, 2, 0);
            dir_.filmGrainVolume = MakeVolume(vol.transform, "Volume_FX_FilmGrain", grain, 2, 0);
            dir_.motionBlurVolume = MakeVolume(vol.transform, "Volume_FX_MotionBlur_Capture", mbp, 2, 1);
            dir_.aerialHazeVolume = MakeVolume(vol.transform, "Volume_FX_AerialHaze", aer, 3, 0);
            dir_.SetPreset(startPreset);
            RenderSettings.sun = sun;
            return dir_;
        }
    }
}
