using System;
using UnityEngine;
using UnityEngine.Rendering;

namespace Crane.Theater.Aerial {
    /// <summary>Compositing and sensor-effect helper: copies a camera render into a target through the compose shader.
    /// Noise and rolling-shutter shear are the optional onboard hooks (both default off).</summary>
    public sealed class AerialCompose : IDisposable {
        readonly Material mat;
        readonly CommandBuffer cmd = new CommandBuffer { name = "aerial compose" };
        static readonly int RectId = Shader.PropertyToID("_Rect"), WinId = Shader.PropertyToID("_UvWin"),
            ParamsId = Shader.PropertyToID("_Params"), SensorId = Shader.PropertyToID("_Sensor"), TexId = Shader.PropertyToID("_MainTex");

        public AerialCompose() {
            var shader = Resources.Load<Shader>("CraneAerialCompose");
            if (shader == null) throw new InvalidOperationException("aerial: shader Resources/CraneAerialCompose missing");
            mat = new Material(shader) { hideFlags = HideFlags.DontSave };
        }

        static float Flip => SystemInfo.graphicsUVStartsAtTop ? -1f : 1f;

        /// <summary>Sensor post: src to dst (same size), with noise sigma (0 = off, gamma-space units), horizontal rolling-shutter
        /// shear (fraction of image width between the top and bottom row, 0 = off), optional monochrome.</summary>
        public void SensorPass(RenderTexture src, RenderTexture dst, float noiseSigma, float shear, bool mono, int seed) {
            mat.SetVector(RectId, new Vector4(0, 0, 1, 1)); mat.SetVector(WinId, new Vector4(0, 0, 1, 1));
            mat.SetVector(ParamsId, new Vector4(Flip, 1, noiseSigma, seed % 9973));
            mat.SetVector(SensorId, new Vector4(shear, mono ? 1 : 0, dst.width, dst.height));
            mat.SetTexture(TexId, src);
            Run(dst, 0, false);
        }

        /// <summary>Draw src into a rectangle of dst (pixels, origin bottom-left), sampling the window <paramref name="win"/>
        /// (u0,v0,u1,v1) of the source. clear fills dst black first.</summary>
        public void Blit(RenderTexture src, RenderTexture dst, Rect pixelRect, Vector4 win, float alpha, bool clear) {
            mat.SetVector(RectId, new Vector4(pixelRect.x / dst.width, pixelRect.y / dst.height, pixelRect.width / dst.width, pixelRect.height / dst.height));
            mat.SetVector(WinId, win);
            mat.SetVector(ParamsId, new Vector4(Flip, alpha, 0, 0));
            mat.SetVector(SensorId, new Vector4(0, 0, dst.width, dst.height));
            mat.SetTexture(TexId, src);
            Run(dst, 1, clear);
        }

        void Run(RenderTexture dst, int pass, bool clear) {
            cmd.Clear();
            cmd.SetRenderTarget(dst);
            if (clear) cmd.ClearRenderTarget(false, true, Color.black);
            cmd.DrawProcedural(Matrix4x4.identity, mat, pass, MeshTopology.Triangles, 6);
            Graphics.ExecuteCommandBuffer(cmd);
        }

        public void Dispose() { cmd.Release(); if (mat != null) UnityEngine.Object.DestroyImmediate(mat); }
    }

    /// <summary>One camera of the OAK-D S2 mounted on the prefab's sensor frame (rig.cameraMount: +Z optical axis).</summary>
    public sealed class OnboardCamera {
        public readonly Camera Cam;
        public readonly RenderTexture Rt;
        public readonly string Name;
        public readonly bool Mono;
        public OnboardCamera(string name, Transform mount, Vector3 localOffset, float hfov, int w, int h, bool mono) {
            Name = name; Mono = mono;
            var go = new GameObject(name);
            go.transform.SetParent(mount, false);
            go.transform.localPosition = localOffset; go.transform.localRotation = Quaternion.identity;
            Cam = go.AddComponent<Camera>();
            Cam.enabled = false;
            RenderProfiles.ConfigureOnboardCamera(Cam, hfov, w, h);
            Rt = new RenderTexture(w, h, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 1, name = name };
            Rt.Create();
            Cam.targetTexture = Rt;
        }
        public void Render() { Cam.Render(); }
        public void Dispose() { if (Rt != null) { Rt.Release(); UnityEngine.Object.DestroyImmediate(Rt); } if (Cam != null) UnityEngine.Object.DestroyImmediate(Cam.gameObject); }
    }
}
