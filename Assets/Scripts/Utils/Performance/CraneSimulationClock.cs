using System;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Utils.Performance {
    /// <summary>
    /// Owns the authoritative fixed-step counter for normal runtime execution. The counter is
    /// independent of benchmark warmup/measurement state and advances exactly once per Unity
    /// FixedUpdate. Scene loads establish a new episode generation before scene Start methods.
    /// </summary>
    [DefaultExecutionOrder(-32700)]
    internal sealed class CraneSimulationClock : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            // The provenance fixture advances a synthetic clock explicitly and must remain
            // isolated from Unity lifecycle ticks.
            if (Array.IndexOf(Environment.GetCommandLineArgs(),
                    "--crane-action-validation") >= 0) return;
            var host = new GameObject("CRANE Simulation Clock");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneSimulationClock>();
        }

        private void Awake() => SceneManager.sceneLoaded += OnSceneLoaded;

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) =>
            CraneRuntimeMetrics.BeginEpisode();

        private void FixedUpdate() => CraneRuntimeMetrics.AdvanceSimulationTick();

        private void OnDestroy() => SceneManager.sceneLoaded -= OnSceneLoaded;
    }
}
