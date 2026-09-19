using System;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Deactivates canonical blocker geometry at a deterministic simulation-time boundary.
    /// Detailed intervention timing is written only through the evaluator-owned callback.
    /// </summary>
    internal sealed class CraneTimedBlockerRemoval : MonoBehaviour {
        private GameObject target;
        private Action<double> onRemoved;

        public double ScheduledSimulationTime { get; private set; } = -1d;

        public void Configure(GameObject configuredTarget, float removeAfterSeconds,
            Action<double> removalCallback) {
            target = configuredTarget != null
                ? configuredTarget
                : throw new ArgumentNullException(nameof(configuredTarget));
            if (removeAfterSeconds < 0f)
                throw new ArgumentOutOfRangeException(nameof(removeAfterSeconds));
            onRemoved = removalCallback;
            ScheduledSimulationTime = Time.fixedTimeAsDouble + removeAfterSeconds;
        }

        private void FixedUpdate() {
            if (target == null || Time.fixedTimeAsDouble < ScheduledSimulationTime) return;
            double actualTime = Time.fixedTimeAsDouble;
            onRemoved?.Invoke(actualTime);
            target.SetActive(false);
            Debug.Log($"CRANE_LAND_BLOCKER_REMOVED simulationTime={actualTime:R}");
            enabled = false;
        }
    }
}
