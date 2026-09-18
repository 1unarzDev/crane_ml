using System;
using System.Collections;
using System.Linq;
using System.Reflection;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Performance {
    /// <summary>Applies per-process seed and transport settings before scene Start methods run.</summary>
    internal sealed class CraneWorkerOverrides : MonoBehaviour {
        private string rosIp;
        private int rosPort = -1;
        private int mavrosPort = -1;
        private int randomSeed = 1;
        private CraneRuntimeOptions runtimeOptions;
        private string requestedScene;
        private bool externalSceneLoader;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            if (args.Contains("--crane-list-profiles")) {
                Debug.Log($"CRANE_RUNTIME_PROFILE_CATALOG {CraneRuntimeOptions.CatalogJson()}");
                Application.Quit(0);
                return;
            }
            if (!args.Contains("--crane-worker") && !args.Contains("--crane-benchmark") &&
                !args.Contains("--crane-profile") && !args.Contains("--crane-replay")) return;
            var host = new GameObject("CRANE Worker Overrides");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneWorkerOverrides>();
        }

        private void Awake() {
            string[] args = Environment.GetCommandLineArgs();
            rosIp = ReadString(args, "--crane-ros-ip", null);
            rosPort = ReadInt(args, "--crane-ros-port", -1);
            mavrosPort = ReadInt(args, "--crane-mavros-port", -1);
            randomSeed = ReadInt(args, "--crane-seed", 1);
            requestedScene = ReadString(args, "--crane-scene", null);
            externalSceneLoader = args.Contains("--crane-benchmark") ||
                args.Contains("--crane-land-validation") ||
                args.Contains("--crane-aerial-validation") ||
                args.Contains("--crane-collision-validation") ||
                args.Contains("--crane-replay");
            runtimeOptions = CraneRuntimeOptions.Parse(args);
            UnityEngine.Random.InitState(randomSeed);
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private IEnumerator Start() {
            if (externalSceneLoader || string.IsNullOrEmpty(requestedScene) ||
                SceneManager.GetActiveScene().name.Equals(requestedScene,
                    StringComparison.OrdinalIgnoreCase)) yield break;
            AsyncOperation load = SceneManager.LoadSceneAsync(requestedScene, LoadSceneMode.Single);
            while (load != null && !load.isDone) yield return null;
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            UnityEngine.Random.InitState(randomSeed);
            // sceneLoaded normally precedes Start, but disabling ROSConnection here is not
            // sufficient on every player startup path: its ConnectOnStart flag can already be
            // observed by Start while the initial build scene is yielding to --crane-scene.
            // Clear the package-owned auto-connect switch before applying the general component
            // gates so --crane-disable-ros never starts a background reconnect loop.
            if (runtimeOptions.DisableRos) SuppressRosAutoConnect();
            runtimeOptions.Apply();
            foreach (MonoBehaviour component in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include)) {
                Type type = component.GetType();
                if (type.FullName == "Unity.Robotics.ROSTCPConnector.ROSConnection") {
                    if (!string.IsNullOrEmpty(rosIp)) type.GetProperty("RosIPAddress")?.SetValue(component, rosIp);
                    if (rosPort > 0) type.GetProperty("RosPort")?.SetValue(component, rosPort);
                }
                else if (type.FullName == "Sim.Sensors.Nav.MAVROSConnection" && mavrosPort > 0) {
                    type.GetField("localPort", BindingFlags.Instance | BindingFlags.NonPublic)
                        ?.SetValue(component, mavrosPort);
                }
            }
        }

        private static void SuppressRosAutoConnect() {
            foreach (MonoBehaviour component in FindObjectsByType<MonoBehaviour>(
                         FindObjectsInactive.Include)) {
                Type type = component.GetType();
                if (type.FullName != "Unity.Robotics.ROSTCPConnector.ROSConnection") continue;
                type.GetProperty("ConnectOnStart")?.SetValue(component, false);
                component.enabled = false;
            }
        }

        private void OnDestroy() => SceneManager.sceneLoaded -= OnSceneLoaded;

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : fallback;
        }

        private static int ReadInt(string[] args, string key, int fallback) =>
            int.TryParse(ReadString(args, key, null), out int value) ? value : fallback;
    }
}
