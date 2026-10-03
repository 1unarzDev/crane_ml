using System;
using System.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Visualization;

namespace Sim.Performance {
    /// <summary>
    /// Opt-in presentation harness for the real player process. It mounts the read-only
    /// Nav2 overlay on a detached spectator camera and captures the player window after
    /// ROS has had time to publish a plan/costmap. It never changes the controller.
    /// </summary>
    internal sealed class CraneEvidenceCaptureBootstrap : MonoBehaviour {
        private static bool enabledByArgs, firstDockCheckpoint;
        private static string capturePath;
        private static float captureDelay, captureDelay2, captureDelay3;
        private static string capturePath2, capturePath3;
        private Camera spectator;
        private Transform boat;
        private Nav2EvidenceOverlay overlay;
        private Bounds firstDockBounds, secondDockBounds;
        private bool haveDockBounds;
        private Vector3 spectatorOffset = new Vector3(5.5f, 12.8f, -13f);

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            enabledByArgs = Array.IndexOf(args, "--crane-evidence-overlay") >= 0;
            firstDockCheckpoint = Array.IndexOf(args, "--crane-evidence-first-dock") >= 0;
            if (!enabledByArgs) return;
            capturePath = ReadString(args, "--crane-evidence-capture", "artifacts/roboboat-player-capture.png");
            capturePath2 = ReadString(args, "--crane-evidence-capture-2", "");
            capturePath3 = ReadString(args, "--crane-evidence-capture-3", "");
            captureDelay = Mathf.Max(2f, ReadFloat(args, "--crane-evidence-capture-delay", 12f));
            captureDelay2 = Mathf.Max(captureDelay, ReadFloat(args, "--crane-evidence-capture-delay-2", captureDelay + 18f));
            captureDelay3 = Mathf.Max(captureDelay2, ReadFloat(args, "--crane-evidence-capture-delay-3", captureDelay2 + 18f));
            var host = new GameObject("CRANE Evidence Capture Bootstrap");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneEvidenceCaptureBootstrap>();
        }

        private void Awake() { SceneManager.sceneLoaded += OnSceneLoaded; }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabledByArgs || !scene.name.Equals("Roboboat Course", StringComparison.OrdinalIgnoreCase)) return;
            StartCoroutine(ConfigureAfterSceneReady());
        }

        private IEnumerator ConfigureAfterSceneReady() {
            // Let prefab-linked controllers, water, and ROS publishers finish Start().
            yield return null; yield return null; yield return new WaitForSeconds(0.5f);
            boat = FindBoat();
            haveDockBounds = TryDockBounds("Marina/Dock0", out firstDockBounds) && TryDockBounds("Marina/Dock1", out secondDockBounds);
            spectator = FindSpectator();
            if (spectator == null) {
                Debug.LogWarning("CRANE_EVIDENCE_CAPTURE_CAMERA_UNAVAILABLE");
                yield break;
            }
            spectator.transform.SetParent(null, true);
            spectator.enabled = true;
            spectator.targetTexture = null;
            spectator.rect = new Rect(0, 0, 1, 1);
            spectator.depth = 100;
            spectator.cullingMask |= 1 << Nav2EvidenceOverlay.VisualizationLayer;
            // Presentation must not become a depth/RGB sensor observation.
            foreach (Camera camera in FindObjectsByType<Camera>(FindObjectsInactive.Include))
                if (camera != spectator) camera.cullingMask &= ~(1 << Nav2EvidenceOverlay.VisualizationLayer);
            spectator.clearFlags = CameraClearFlags.Skybox;
            spectator.fieldOfView = 50f;
            if (boat != null) {
                Vector3 target = boat.position + Vector3.up * 0.2f;
                // Detached, elevated three-quarter view follows the local evidence field.
                spectator.transform.position = target + spectatorOffset;
                spectator.transform.LookAt(target + new Vector3(0f, 0.1f, 2.0f));
            }
            var overlayHost = new GameObject("CRANE Nav2 Evidence Overlay");
            overlay = overlayHost.AddComponent<Nav2EvidenceOverlay>();
            overlay.transform.position = Vector3.zero;
            Debug.Log($"CRANE_EVIDENCE_CAPTURE_READY camera={spectator.name} path={capturePath} delay={captureDelay:R}");
            float configuredAt = Time.time;
            if (firstDockCheckpoint) {
                while (Time.time - configuredAt < 45f &&
                    (!haveDockBounds || boat == null || !overlay.ReadyForCapture ||
                     Nav2EvidenceOverlay.HorizontalDistanceToBounds(firstDockBounds, boat.position) > 1.6f ||
                     Nav2EvidenceOverlay.HorizontalDistanceToBounds(firstDockBounds, boat.position) >=
                     Nav2EvidenceOverlay.HorizontalDistanceToBounds(secondDockBounds, boat.position))) yield return null;
                if (!haveDockBounds || boat == null || !overlay.ReadyForCapture ||
                    Nav2EvidenceOverlay.HorizontalDistanceToBounds(firstDockBounds, boat.position) > 1.6f) {
                    Debug.LogWarning("CRANE_FIRST_DOCK_CAPTURE_CHECKPOINT_NOT_REACHED"); yield break;
                }
            } else yield return new WaitForSeconds(captureDelay);
            Capture(capturePath);
            if (!string.IsNullOrEmpty(capturePath2)) {
                yield return new WaitForSeconds(Mathf.Max(0f, captureDelay2 - (Time.time - configuredAt)));
                Capture(capturePath2);
            }
            if (!string.IsNullOrEmpty(capturePath3)) {
                float waitStarted = Time.time;
                float timeout = captureDelay3 - (string.IsNullOrEmpty(capturePath2) ? captureDelay : captureDelay2);
                // Capture the actual frozen docking predicate while it is qualified. A later
                // timed photograph can conceal post-result drift or miss the qualified interval.
                while (!overlay.DockingQualified && Time.time - waitStarted < timeout) yield return null;
                Capture(capturePath3);
            }
        }

        private void LateUpdate() {
            if (spectator == null || boat == null) return;
            var target = boat.position + Vector3.up * .2f;
            spectator.transform.position = target + spectatorOffset;
            spectator.transform.LookAt(target + new Vector3(0f, .1f, 2f));
        }

        private void Capture(string path) {
            var directory = System.IO.Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory)) System.IO.Directory.CreateDirectory(directory);
            overlay.RecordCamera(spectator);
            if (haveDockBounds && boat != null)
                overlay.RecordDockProximity(firstDockBounds, secondDockBounds, boat.position,
                    firstDockCheckpoint && path == capturePath ? "first-dock-approach" : "navigation-validation");
            System.IO.File.WriteAllText(path + ".evidence.json", overlay.CaptureMetadataJson());
            ScreenCapture.CaptureScreenshot(path, 1);
            // CaptureScreenshot writes asynchronously at frame end; this records the request.
            Debug.Log($"CRANE_EVIDENCE_CAPTURE_REQUESTED {path} utc={DateTime.UtcNow:O} simulationTime={Time.fixedTimeAsDouble:R} cameraPosition={spectator.transform.position} boat={boat?.name}");
        }

        private static bool TryDockBounds(string path, out Bounds bounds) {
            bounds = default;
            var dock = GameObject.Find(path);
            if (dock == null) return false;
            var renderers = dock.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0) return false;
            bounds = renderers[0].bounds;
            for (int i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
            return true;
        }

        private static Transform FindBoat() {
            foreach (var component in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude))
                if (component.GetType().FullName == "Sim.Controllers.OmniXController")
                    return component.transform;
            return null;
        }

        private static Camera FindSpectator() {
            foreach (Camera c in FindObjectsByType<Camera>(FindObjectsInactive.Include)) {
                if (c.name.Equals("Camera", StringComparison.OrdinalIgnoreCase) ||
                    c.name.IndexOf("spectator", StringComparison.OrdinalIgnoreCase) >= 0) return c;
            }
            return Camera.main ?? FindAnyObjectByType<Camera>();
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int i = Array.IndexOf(args, key); return i >= 0 && i + 1 < args.Length ? args[i + 1] : fallback;
        }
        private static float ReadFloat(string[] args, string key, float fallback) {
            return float.TryParse(ReadString(args, key, null), out float v) ? v : fallback;
        }
        private void OnDestroy() { SceneManager.sceneLoaded -= OnSceneLoaded; }
    }
}
