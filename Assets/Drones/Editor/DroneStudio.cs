using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using UnityEngine.SceneManagement;

namespace Crane.Drones.EditorTools
{
    /// <summary>
    /// Lit HDRP turntable studio. Needs a GPU-backed Editor: with -nographics the device is Null and nothing is
    /// rendered. Run: unity-drones LOG -executeMethod Crane.Drones.EditorTools.DroneStudio.RenderTurntable -quit
    /// (from a wrapper without -nographics) -> Docs/DroneVisuals/model/hdrp_*.png
    /// </summary>
    public static class DroneStudio
    {
        [MenuItem("CRANE/Drones/Render Turntable")]
        public static void RenderTurntable()
        {
            if (SystemInfo.graphicsDeviceType == GraphicsDeviceType.Null)
            {
                Debug.LogWarning("[DroneStudio] Null graphics device (-nographics): turntable skipped");
                return;
            }
            string outDir = "Docs/DroneVisuals/model"; Directory.CreateDirectory(outDir);
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(BuildAOS7Quad.PrefabPath);
            var quad = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            quad.transform.position = new Vector3(0, 1f, 0);
            quad.GetComponent<DroneVisualRig>().SetRotorSpeeds(new float[] { 0, 0, 0, 0 });

            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane); floor.transform.localScale = Vector3.one * 2f;
            floor.GetComponent<MeshRenderer>().sharedMaterial = AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/DarkGrayMatt.mat");

            var vgo = new GameObject("StudioVolume"); var vol = vgo.AddComponent<Volume>(); vol.isGlobal = true;
            var prof = ScriptableObject.CreateInstance<VolumeProfile>(); vol.sharedProfile = prof;
            var exp = prof.Add<Exposure>(); exp.mode.Override(ExposureMode.Fixed); exp.fixedExposure.Override(11.5f);
            var env = prof.Add<VisualEnvironment>(); env.skyType.Override((int)SkyType.Gradient);
            var sky = prof.Add<GradientSky>(); sky.top.Override(new Color(0.55f, 0.65f, 0.85f)); sky.middle.Override(new Color(0.75f, 0.78f, 0.82f)); sky.bottom.Override(new Color(0.18f, 0.18f, 0.2f));
            var tm = prof.Add<Tonemapping>(); tm.mode.Override(TonemappingMode.ACES);

            var lgo = new GameObject("Key"); var l = lgo.AddComponent<Light>(); l.type = LightType.Directional; l.intensity = 12000f; l.color = new Color(1f, 0.96f, 0.9f);
            lgo.AddComponent<HDAdditionalLightData>(); lgo.transform.rotation = Quaternion.Euler(48, -35, 0);
            var fgo = new GameObject("Fill"); var f = fgo.AddComponent<Light>(); f.type = LightType.Directional; f.intensity = 3000f; f.color = new Color(0.7f, 0.8f, 1f);
            fgo.AddComponent<HDAdditionalLightData>(); fgo.transform.rotation = Quaternion.Euler(25, 140, 0);

            var cgo = new GameObject("Cam"); var cam = cgo.AddComponent<Camera>(); cgo.AddComponent<HDAdditionalCameraData>();
            cam.fieldOfView = 28f; cam.nearClipPlane = 0.02f; cam.farClipPlane = 20f; cam.clearFlags = CameraClearFlags.Skybox;
            var rt = new RenderTexture(1600, 1000, 24, RenderTextureFormat.ARGB32); cam.targetTexture = rt;

            string[] names = { "front34", "rear34", "top", "side" };
            Vector3[] dirs = { new Vector3(-0.55f, 0.45f, 0.7f), new Vector3(0.55f, 0.4f, -0.72f), new Vector3(0.0f, 1f, 0.01f), new Vector3(1f, 0.08f, 0f) };
            for (int pass = 0; pass < 2; pass++)
            {
                quad.GetComponent<DroneVisualRig>().SetRotorSpeeds(pass == 0 ? new float[] { 0, 0, 0, 0 } : new float[] { 420, 420, 420, 420 });
                for (int i = 0; i < names.Length; i++)
                {
                    cgo.transform.position = quad.transform.position + dirs[i].normalized * 0.9f; cgo.transform.LookAt(quad.transform.position + new Vector3(0, 0.02f, 0));
                    for (int w = 0; w < 3; w++) cam.Render();
                    RenderTexture.active = rt; var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
                    tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0); tex.Apply(); RenderTexture.active = null;
                    File.WriteAllBytes(outDir + "/hdrp_" + (pass == 0 ? "static_" : "spinning_") + names[i] + ".png", tex.EncodeToPNG());
                    Object.DestroyImmediate(tex);
                }
            }
            Debug.Log("[DroneStudio] wrote HDRP renders to " + outDir);
        }
    }
}
