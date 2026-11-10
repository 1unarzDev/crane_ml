using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace Crane.Drones.EditorTools
{
    /// <summary>Creates/updates the HDRP Lit materials in Assets/Drones/Materials (GUIDs stay stable across re-runs).</summary>
    public static class DroneMaterials
    {
        public const string Dir = "Assets/Drones/Materials";
        static Dictionary<string, Material> _cache = new Dictionary<string, Material>();

        static Texture2D Tex(string n) { return AssetDatabase.LoadAssetAtPath<Texture2D>(DroneTextures.Dir + "/" + n + ".png"); }

        static void Validate(Material m)
        {
            var t = Type.GetType("UnityEditor.Rendering.HighDefinition.HDShaderUtils, Unity.RenderPipelines.HighDefinition.Editor");
            var mi = t != null ? t.GetMethod("ResetMaterialKeywords", BindingFlags.Public | BindingFlags.Static, null, new[] { typeof(Material) }, null) : null;
            if (mi != null) mi.Invoke(null, new object[] { m }); else Debug.LogWarning("HDShaderUtils.ResetMaterialKeywords not found");
        }

        static Material Get(string name)
        {
            Material m;
            if (_cache.TryGetValue(name, out m) && m != null) return m;
            string path = Dir + "/" + name + ".mat";
            m = AssetDatabase.LoadAssetAtPath<Material>(path);
            var sh = Shader.Find("HDRP/Lit");
            if (sh == null) throw new Exception("HDRP/Lit shader not found");
            if (m == null) { m = new Material(sh); AssetDatabase.CreateAsset(m, path); }
            else m.shader = sh;
            _cache[name] = m; return m;
        }

        static Material Opaque(string name, Color c, float metal, float smooth)
        {
            var m = Get(name);
            m.SetColor("_BaseColor", c); m.SetFloat("_Metallic", metal); m.SetFloat("_Smoothness", smooth);
            m.SetFloat("_SmoothnessRemapMin", 0f); m.SetFloat("_SmoothnessRemapMax", smooth);
            return m;
        }

        static void SetMaps(Material m, Texture2D albedo, Texture2D normal, float normalScale, float tilePerMeter)
        {
            if (albedo != null) { m.SetTexture("_BaseColorMap", albedo); m.SetTextureScale("_BaseColorMap", Vector2.one * tilePerMeter); }
            if (normal != null)
            {
                m.SetTexture("_NormalMap", normal); m.SetFloat("_NormalScale", normalScale);
                m.SetTextureScale("_NormalMap", Vector2.one * tilePerMeter);
                m.EnableKeyword("_NORMALMAP"); m.EnableKeyword("_NORMALMAP_TANGENT_SPACE");
            }
        }

        static void Emissive(Material m, Color c, float nits)
        {
            m.SetFloat("_UseEmissiveIntensity", 1f); m.SetColor("_EmissiveColorLDR", c); m.SetFloat("_EmissiveIntensity", nits);
            m.SetFloat("_EmissiveIntensityUnit", 0f);
            m.SetColor("_EmissiveColor", c * nits); m.EnableKeyword("_EMISSIVE_COLOR_MAP");
            m.globalIlluminationFlags = MaterialGlobalIlluminationFlags.BakedEmissive;
        }

        static void Transparent(Material m)
        {
            m.SetFloat("_SurfaceType", 1f); m.SetFloat("_BlendMode", 0f);
            m.SetFloat("_SrcBlend", 1f); m.SetFloat("_DstBlend", 10f); m.SetFloat("_AlphaSrcBlend", 1f); m.SetFloat("_AlphaDstBlend", 10f);
            m.SetFloat("_ZWrite", 0f); m.SetFloat("_TransparentZWrite", 0f); m.SetFloat("_DoubleSidedEnable", 1f); m.SetFloat("_CullMode", 0f); m.SetFloat("_CullModeForward", 0f);
            m.SetFloat("_AlphaCutoffEnable", 0f); m.SetFloat("_TransparentDepthPrepassEnable", 0f);
            m.SetFloat("_RenderQueueType", 5f);
            m.renderQueue = 3000;
        }

        public static Dictionary<string, Material> BuildAll()
        {
            System.IO.Directory.CreateDirectory(Dir);
            var carbonA = Tex("carbon_twill_albedo"); var carbonN = Tex("carbon_twill_normal");
            var pcbA = Tex("pcb_green_albedo"); var brushN = Tex("brushed_normal");
            var d = new Dictionary<string, Material>();
            Material m;

            // Carbon fiber: dark weave albedo, strong tow normal, thin clear coat.
            m = Opaque("Carbon", new Color(1, 1, 1, 1), 0f, 0.62f); SetMaps(m, carbonA, carbonN, 1.0f, 1f / 0.064f);
            m.SetFloat("_CoatMask", 1f); d["Carbon"] = m;

            m = Opaque("Aluminum_Anodized", new Color(0.16f, 0.165f, 0.18f), 1f, 0.74f); SetMaps(m, null, brushN, 0.25f, 40f); d["Aluminum_Anodized"] = m;
            m = Opaque("Anodized_Red", new Color(0.52f, 0.035f, 0.03f), 1f, 0.78f); SetMaps(m, null, brushN, 0.2f, 40f); d["Anodized_Red"] = m;
            m = Opaque("Aluminum_Brushed", new Color(0.78f, 0.79f, 0.80f), 1f, 0.58f); SetMaps(m, null, brushN, 0.6f, 40f); d["Aluminum_Brushed"] = m;
            m = Opaque("Steel", new Color(0.55f, 0.56f, 0.58f), 1f, 0.55f); d["Steel"] = m;
            m = Opaque("Gold", new Color(0.83f, 0.62f, 0.2f), 1f, 0.8f); d["Gold"] = m;
            m = Opaque("Copper", new Color(0.72f, 0.36f, 0.2f), 1f, 0.6f); d["Copper"] = m;
            m = Opaque("Plastic_Black", new Color(0.025f, 0.025f, 0.028f), 0f, 0.38f); d["Plastic_Black"] = m;
            m = Opaque("Plastic_DarkGray", new Color(0.09f, 0.09f, 0.10f), 0f, 0.42f); d["Plastic_DarkGray"] = m;
            m = Opaque("Plastic_White", new Color(0.72f, 0.72f, 0.70f), 0f, 0.45f); d["Plastic_White"] = m;
            m = Opaque("Plastic_Orange", new Color(0.78f, 0.28f, 0.03f), 0f, 0.5f); d["Plastic_Orange"] = m;
            m = Opaque("Connector_Yellow", new Color(0.85f, 0.62f, 0.04f), 0f, 0.5f); d["Connector_Yellow"] = m;
            m = Opaque("Rubber", new Color(0.02f, 0.02f, 0.02f), 0f, 0.15f); d["Rubber"] = m;
            m = Opaque("Wire_Black", new Color(0.015f, 0.015f, 0.015f), 0f, 0.35f); d["Wire_Black"] = m;
            m = Opaque("Wire_Red", new Color(0.5f, 0.02f, 0.02f), 0f, 0.4f); d["Wire_Red"] = m;
            m = Opaque("PCB_Green", new Color(1, 1, 1, 1), 0f, 0.55f); SetMaps(m, pcbA, null, 1f, 1f / 0.05f); d["PCB_Green"] = m;
            m = Opaque("PCB_Black", new Color(0.018f, 0.02f, 0.022f), 0f, 0.55f); d["PCB_Black"] = m;
            m = Opaque("Prop_Plastic", new Color(0.055f, 0.057f, 0.062f), 0f, 0.5f); d["Prop_Plastic"] = m;
            m = Opaque("LiPo_Wrap", new Color(0.04f, 0.05f, 0.10f), 0f, 0.5f); d["LiPo_Wrap"] = m;
            m = Opaque("LiPo_Label", new Color(1, 1, 1, 1), 0f, 0.4f); SetMaps(m, Tex("battery_label_albedo"), null, 1f, 1f); d["LiPo_Label"] = m;
            // Lens glass: coated, nearly black, mirror-smooth, opaque (no refraction needed at this scale).
            m = Opaque("Glass_Lens", new Color(0.012f, 0.014f, 0.03f), 0f, 1.0f); d["Glass_Lens"] = m;
            // Smoked dome (L2): alpha blended.
            m = Opaque("Glass_Smoked", new Color(0.02f, 0.025f, 0.03f, 0.5f), 0f, 0.96f); Transparent(m); d["Glass_Smoked"] = m;

            m = Opaque("LED_Green", new Color(0.02f, 0.06f, 0.02f), 0f, 0.8f); Emissive(m, new Color(0.1f, 1f, 0.15f), 600f); d["LED_Green"] = m;
            m = Opaque("LED_Red", new Color(0.08f, 0.01f, 0.01f), 0f, 0.8f); Emissive(m, new Color(1f, 0.05f, 0.03f), 600f); d["LED_Red"] = m;
            m = Opaque("LED_White", new Color(0.1f, 0.1f, 0.1f), 0f, 0.8f); Emissive(m, new Color(0.9f, 0.95f, 1f), 700f); d["LED_White"] = m;
            m = Opaque("LED_Blue", new Color(0.01f, 0.02f, 0.08f), 0f, 0.8f); Emissive(m, new Color(0.1f, 0.3f, 1f), 600f); d["LED_Blue"] = m;

            // Props: opaque blades, alpha-blended ghost blades, translucent blur disc.
            m = Opaque("Prop_BladeFade", new Color(0.055f, 0.057f, 0.062f, 1f), 0f, 0.5f); Transparent(m); m.SetFloat("_TransparentDepthPrepassEnable", 0f); d["Prop_BladeFade"] = m;
            m = Opaque("Prop_BlurDisc", new Color(1f, 1f, 1f, 0f), 0f, 0.3f); SetMaps(m, Tex("prop_blur_disc"), null, 1f, 1f); Transparent(m);
            m.SetFloat("_ZWrite", 0f); d["Prop_BlurDisc"] = m;

            foreach (var kv in d) { Validate(kv.Value); EditorUtility.SetDirty(kv.Value); }
            AssetDatabase.SaveAssets();
            return d;
        }
    }
}
