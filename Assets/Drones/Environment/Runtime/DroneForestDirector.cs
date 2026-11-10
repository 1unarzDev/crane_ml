using System;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;

namespace Crane.Drones.Environment
{
    /// <summary>
    /// Runtime switchboard for the Drone Forest look. The camera director calls these:
    ///   SetPreset("GoldenHour"|"Overcast"|"Dusk"), SetDepthOfField(bool, focusDistance),
    ///   SetFilmGrain(bool), SetMotionBlur(bool), ConfigureCamera(camera).
    /// Presets are global Volumes (weight 1 or 0) plus a sun pose/intensity/temperature.
    /// </summary>
    public class DroneForestDirector : MonoBehaviour
    {
        [Serializable]
        public class Preset
        {
            public string name;
            public Volume volume;
            public Vector3 sunEuler;
            public float sunLux;
            public float sunKelvin;
            public float shadowStrength = 1f;
            public float volumetricDimmer = 1f;
            public Color hazeColor = new Color(0.8f, 0.76f, 0.68f);
        }

        public Light sun;
        public Preset[] presets;
        public Volume depthOfFieldVolume;   // weight 0 by default, hero shots
        public Volume filmGrainVolume;      // weight 0 by default
        public Volume motionBlurVolume;     // weight 1 by default, capture
        public Volume aerialHazeVolume;     // weight 0 by default: for shots high above the canopy (hides the edge of the world)
        public string startPreset = "GoldenHour";
        Color activeHaze = new Color(0.8f, 0.76f, 0.68f);

        void Start() { SetPreset(startPreset); }

        public void SetPreset(string presetName)
        {
            if (presets == null) return;
            foreach (var p in presets)
            {
                bool on = string.Equals(p.name, presetName, StringComparison.OrdinalIgnoreCase);
                if (p.volume != null) p.volume.weight = on ? 1f : 0f;
                if (on) activeHaze = p.hazeColor;
                if (!on || sun == null) continue;
                sun.transform.rotation = Quaternion.Euler(p.sunEuler);
                sun.intensity = p.sunLux;
                sun.useColorTemperature = true;
                sun.colorTemperature = p.sunKelvin;
                sun.shadowStrength = p.shadowStrength;
                var hd = sun.GetComponent<HDAdditionalLightData>();
                if (hd != null) hd.volumetricDimmer = p.volumetricDimmer;
            }
        }

        public void SetDepthOfField(bool on, float focusDistance = 8f)
        {
            if (depthOfFieldVolume == null) return;
            depthOfFieldVolume.weight = on ? 1f : 0f;
            if (depthOfFieldVolume.profile.TryGet(out DepthOfField dof)) dof.focusDistance.value = focusDistance;
        }

        /// <summary>Aerial / over-canopy shots: swaps volumetric fog for exponential horizon haze (weight 0..1) so the surrounding landscape fades into the sky.</summary>
        public void SetAerialHaze(float weight)
        {
            if (aerialHazeVolume == null) return;
            aerialHazeVolume.weight = Mathf.Clamp01(weight);
            if (aerialHazeVolume.profile.TryGet(out Fog fog)) { fog.color.value = activeHaze; fog.tint.value = activeHaze; }
        }

        public void SetFilmGrain(bool on) { if (filmGrainVolume != null) filmGrainVolume.weight = on ? 1f : 0f; }
        public void SetMotionBlur(bool on) { if (motionBlurVolume != null) motionBlurVolume.weight = on ? 1f : 0f; }

        /// <summary>Turns on the per-camera frame settings the forest look needs (SSGI, SSAO, volumetrics, contact shadows).</summary>
        public static void ConfigureCamera(Camera cam)
        {
            var hd = cam.GetComponent<HDAdditionalCameraData>();
            if (hd == null) hd = cam.gameObject.AddComponent<HDAdditionalCameraData>();
            cam.farClipPlane = Mathf.Max(cam.farClipPlane, 3600f); // covers the 3.5 km surrounding landscape
            hd.customRenderingSettings = true;
            Enable(hd, FrameSettingsField.SSGI);
            Enable(hd, FrameSettingsField.SSAO);
            Enable(hd, FrameSettingsField.Volumetrics);
            Enable(hd, FrameSettingsField.ContactShadows);
            Enable(hd, FrameSettingsField.MotionBlur);
            Enable(hd, FrameSettingsField.DepthOfField);
            Enable(hd, FrameSettingsField.Bloom);
            Enable(hd, FrameSettingsField.Tonemapping);
            Enable(hd, FrameSettingsField.ExposureControl);
            Enable(hd, FrameSettingsField.AfterPostprocess);
        }

        static void Enable(HDAdditionalCameraData hd, FrameSettingsField f)
        {
            hd.renderingPathCustomFrameSettings.SetEnabled(f, true);
            hd.renderingPathCustomFrameSettingsOverrideMask.mask[(uint)f] = true;
        }
    }
}
