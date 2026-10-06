using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Performance {
    /// <summary>Hide only the throw-away startup scene while --crane-scene loads.</summary>
    [DefaultExecutionOrder(32000)]
    public sealed class CraneStartupRenderGuard : MonoBehaviour {
        private string targetScene;
        private bool waitingForTarget;
        private readonly List<Camera> suppressed = new();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, "--crane-scene");
            if (index < 0 || index + 1 >= args.Length) return;
            var host = new GameObject("CRANE Startup Render Guard");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneStartupRenderGuard>().Configure(args[index + 1]);
        }

        public void Configure(string sceneName) {
            targetScene = sceneName;
            waitingForTarget = !string.IsNullOrWhiteSpace(sceneName);
            SceneManager.sceneLoaded += HandleSceneLoaded;
        }

        public void HandleSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!waitingForTarget) return;
            if (scene.name.Equals(targetScene, StringComparison.OrdinalIgnoreCase)) {
                // Surviving persistent cameras regain their original enabled state.
                // New target cameras have never been touched by the guard.
                foreach (Camera camera in suppressed)
                    if (camera != null && (camera.gameObject.scene == scene ||
                        camera.gameObject.scene.name == "DontDestroyOnLoad"))
                        camera.enabled = true;
                waitingForTarget = false;
                SceneManager.sceneLoaded -= HandleSceneLoaded;
                Debug.Log($"CRANE_STARTUP_RENDER_READY scene={scene.name} suppressed={suppressed.Count}");
                enabled = false;
                return;
            }
            SuppressOutgoingCameras();
        }

        // Recheck after ordinary Update/LateUpdate camera rigs: outgoing scene
        // Start methods may have created or re-enabled a screen camera.
        private void LateUpdate() {
            if (waitingForTarget) SuppressOutgoingCameras();
        }

        public void SuppressOutgoingCameras() {
            if (!waitingForTarget) return;
            foreach (Camera camera in FindObjectsByType<Camera>(FindObjectsInactive.Include)) {
                if (!camera.enabled || camera.targetTexture != null ||
                    camera.gameObject.scene.name.Equals(targetScene,
                        StringComparison.OrdinalIgnoreCase)) continue;
                camera.enabled = false;
                if (!suppressed.Contains(camera)) suppressed.Add(camera);
            }
        }

        private void OnDestroy() => SceneManager.sceneLoaded -= HandleSceneLoaded;
    }
}
