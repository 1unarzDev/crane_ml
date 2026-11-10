using System;
using System.IO;
using UnityEditor;
using UnityEngine;

namespace Crane.Drones.EditorTools
{
    /// <summary>Deterministic procedural PNG textures (carbon twill weave, PCB, brushed metal, blur disc).</summary>
    public static class DroneTextures
    {
        public const string Dir = "Assets/Drones/Models/Textures";

        static void Save(string name, Color[] px, int w, int h, bool normalMap, bool srgb, TextureWrapMode wrap)
        {
            Directory.CreateDirectory(Dir);
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false, !srgb);
            tex.SetPixels(px); tex.Apply();
            string path = Dir + "/" + name + ".png";
            File.WriteAllBytes(path, tex.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(tex);
            AssetDatabase.ImportAsset(path, ImportAssetOptions.ForceUpdate);
            var imp = (TextureImporter)AssetImporter.GetAtPath(path);
            imp.textureType = normalMap ? TextureImporterType.NormalMap : TextureImporterType.Default;
            imp.sRGBTexture = srgb && !normalMap;
            imp.wrapMode = wrap; imp.filterMode = FilterMode.Bilinear; imp.anisoLevel = 8; imp.mipmapEnabled = true;
            imp.alphaIsTransparency = false;
            imp.SaveAndReimport();
        }

        static Color[] NormalFromHeight(float[] h, int w, int hgt, float strength)
        {
            var o = new Color[w * hgt];
            for (int y = 0; y < hgt; y++)
                for (int x = 0; x < w; x++)
                {
                    float l = h[y * w + (x + w - 1) % w], r = h[y * w + (x + 1) % w];
                    float d = h[((y + hgt - 1) % hgt) * w + x], u = h[((y + 1) % hgt) * w + x];
                    var n = new Vector3((l - r) * strength, (d - u) * strength, 1f).normalized;
                    o[y * w + x] = new Color(n.x * 0.5f + 0.5f, n.y * 0.5f + 0.5f, n.z * 0.5f + 0.5f, 1f);
                }
            return o;
        }

        public static void GenerateAll()
        {
            Carbon(); Pcb(); Brushed(); BlurDisc(); BatteryLabel();
        }

        // 2x2 twill, 32x32 tows per tile; tile covers 64 mm on the model.
        static void Carbon()
        {
            const int S = 256, C = 8; var h = new float[S * S]; var a = new Color[S * S];
            var rng = new System.Random(7);
            for (int y = 0; y < S; y++)
                for (int x = 0; x < S; x++)
                {
                    int cx = x / C, cy = y / C; float u = ((x % C) + 0.5f) / C, v = ((y % C) + 0.5f) / C;
                    bool warp = ((cx + cy) % 4) < 2;   // twill diagonal
                    float prof = warp ? Mathf.Sin(u * Mathf.PI) : Mathf.Sin(v * Mathf.PI);
                    float along = warp ? v : u;
                    float edge = Mathf.Pow(prof, 0.6f);
                    float tow = 0.35f + 0.65f * edge * (0.85f + 0.15f * Mathf.Cos(along * Mathf.PI * 2f));
                    h[y * S + x] = tow * (0.9f + 0.1f * (float)rng.NextDouble());
                    float lum = 0.020f + 0.075f * edge * (warp ? 1f : 0.62f) + 0.01f * (float)rng.NextDouble();
                    a[y * S + x] = new Color(lum, lum * 1.02f, lum * 1.08f, 1f);
                }
            Save("carbon_twill_albedo", a, S, S, false, true, TextureWrapMode.Repeat);
            Save("carbon_twill_normal", NormalFromHeight(h, S, S, 2.4f), S, S, true, false, TextureWrapMode.Repeat);
        }

        static void Pcb()
        {
            const int S = 256; var a = new Color[S * S]; var rng = new System.Random(11);
            var green = new Color(0.020f, 0.215f, 0.075f, 1f);
            for (int i = 0; i < a.Length; i++) { float n = 0.9f + 0.1f * (float)rng.NextDouble(); a[i] = green * n; a[i].a = 1; }
            // Manhattan traces (slightly lighter solder-mask over copper)
            for (int t = 0; t < 46; t++)
            {
                int x = rng.Next(S), y = rng.Next(S), len = 20 + rng.Next(70); bool horiz = rng.Next(2) == 0;
                for (int k = 0; k < len; k++)
                {
                    int px = horiz ? (x + k) % S : x, py = horiz ? y : (y + k) % S;
                    for (int w = 0; w < 2; w++) { int qx = horiz ? px : (px + w) % S, qy = horiz ? (py + w) % S : py; a[qy * S + qx] = green * 1.35f; a[qy * S + qx].a = 1; }
                }
            }
            // gold pads and white silkscreen ticks
            for (int t = 0; t < 70; t++)
            {
                int x = rng.Next(S), y = rng.Next(S);
                for (int dy = 0; dy < 3; dy++) for (int dx = 0; dx < 3; dx++) a[((y + dy) % S) * S + (x + dx) % S] = new Color(0.62f, 0.46f, 0.12f, 1f);
            }
            for (int t = 0; t < 30; t++)
            {
                int x = rng.Next(S), y = rng.Next(S), len = 8 + rng.Next(16);
                for (int k = 0; k < len; k++) a[y * S + (x + k) % S] = new Color(0.78f, 0.80f, 0.76f, 1f);
            }
            Save("pcb_green_albedo", a, S, S, false, true, TextureWrapMode.Repeat);
        }

        static void Brushed()
        {
            const int S = 128; var h = new float[S * S]; var rng = new System.Random(23);
            var row = new float[S]; for (int y = 0; y < S; y++) row[y] = (float)rng.NextDouble();
            for (int y = 0; y < S; y++) for (int x = 0; x < S; x++) h[y * S + x] = row[y] * 0.8f + 0.2f * (float)rng.NextDouble();
            Save("brushed_normal", NormalFromHeight(h, S, S, 0.9f), S, S, true, false, TextureWrapMode.Repeat);
        }

        // u runs hub -> tip across the texture width; alpha is the integrated blade-sweep opacity.
        static void BlurDisc()
        {
            const int W = 128, H = 8; var a = new Color[W * H]; var rng = new System.Random(5);
            for (int x = 0; x < W; x++)
            {
                float u = (x + 0.5f) / W;
                float ramp = Mathf.SmoothStep(0f, 1f, u / 0.22f);
                float tipFade = 1f - Mathf.SmoothStep(0.88f, 1f, u);
                float chord = 0.55f + 0.45f * Mathf.Sin(Mathf.PI * Mathf.Clamp01(u * 0.95f));
                float al = ramp * tipFade * chord * (0.9f + 0.1f * Mathf.Sin(u * 60f));
                float tint = 0.30f + 0.25f * u;
                for (int y = 0; y < H; y++) a[y * W + x] = new Color(tint, tint * 1.02f, tint * 1.06f, Mathf.Clamp01(al));
            }
            Save("prop_blur_disc", a, W, H, false, true, TextureWrapMode.Clamp);
        }

        static void BatteryLabel()
        {
            const int W = 256, H = 128; var a = new Color[W * H]; var rng = new System.Random(31);
            for (int y = 0; y < H; y++) for (int x = 0; x < W; x++)
                {
                    float n = 0.93f + 0.07f * (float)rng.NextDouble();
                    var c = new Color(0.82f * n, 0.84f * n, 0.86f * n, 1f);
                    if (y > 100 && y < 118 && x > 10 && x < 150) c = new Color(0.08f, 0.14f, 0.45f, 1f);
                    if (y > 60 && y < 70 && x > 10 && x < 220 && ((x / 6) % 3 != 0)) c = new Color(0.1f, 0.1f, 0.1f, 1f);
                    if (y > 30 && y < 40 && x > 10 && x < 170 && ((x / 5) % 4 != 0)) c = new Color(0.2f, 0.2f, 0.2f, 1f);
                    a[y * W + x] = c;
                }
            Save("battery_label_albedo", a, W, H, false, true, TextureWrapMode.Clamp);
        }
    }
}
