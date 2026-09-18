using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Physics.Water {
    /// <summary>
    /// Restores the authoritative HDRP spectral timeline for in-place reset. This does not claim
    /// to reset stateful deformers, foam, wakes, splashes, or temporal presentation history.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(WaterSurface))]
    internal sealed class CraneWaterEpisodeReset : MonoBehaviour, ICraneEpisodeResettable {
        private WaterSurface surface;
        private float initialSimulationTime;
        private float initialTimeMultiplier;
        public int ResetPriority => -60;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Install() {
            foreach (WaterSurface water in FindObjectsByType<WaterSurface>(
                         FindObjectsInactive.Include, FindObjectsSortMode.InstanceID))
                if (!water.TryGetComponent<CraneWaterEpisodeReset>(out _))
                    water.gameObject.AddComponent<CraneWaterEpisodeReset>();
        }

        private void Awake() => surface = GetComponent<WaterSurface>();

        public void CaptureEpisodeInitialState() {
            initialSimulationTime = surface.simulationTime;
            initialTimeMultiplier = surface.timeMultiplier;
        }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) {
                surface.timeMultiplier = 0;
                surface.simulationTime = initialSimulationTime;
            }
            else {
                // Reapply after body restoration because HDRP can allocate/rebind water resources
                // while transforms are synchronized.
                surface.simulationTime = initialSimulationTime;
                surface.timeMultiplier = initialTimeMultiplier;
            }
        }
    }
}
