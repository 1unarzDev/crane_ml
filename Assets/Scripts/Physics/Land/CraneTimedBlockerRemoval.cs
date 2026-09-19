using System;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Activates and/or deactivates canonical blocker geometry at deterministic simulation-time
    /// boundaries. Detailed intervention timing is written only through evaluator-owned callbacks.
    /// </summary>
    internal sealed class CraneTimedBlockerRemoval : MonoBehaviour {
        private GameObject target;
        private Action<double> onActivated;
        private Action<double> onRemoved;
        private bool activationComplete;

        public double ScheduledActivationSimulationTime { get; private set; } = -1d;
        public double ScheduledSimulationTime { get; private set; } = -1d;

        public void Configure(GameObject configuredTarget, float enableAfterSeconds,
            float removeAfterSeconds, Action<double> activationCallback,
            Action<double> removalCallback) {
            target = configuredTarget != null
                ? configuredTarget
                : throw new ArgumentNullException(nameof(configuredTarget));
            if (enableAfterSeconds < 0f && removeAfterSeconds < 0f)
                throw new ArgumentOutOfRangeException(nameof(enableAfterSeconds),
                    "At least one intervention boundary is required.");
            if (enableAfterSeconds >= 0f && removeAfterSeconds >= 0f &&
                removeAfterSeconds <= enableAfterSeconds)
                throw new ArgumentException(
                    "Blocker removal must occur after delayed activation.");
            onActivated = activationCallback;
            onRemoved = removalCallback;
            activationComplete = enableAfterSeconds < 0f;
            if (enableAfterSeconds >= 0f) {
                target.SetActive(false);
                ScheduledActivationSimulationTime =
                    Time.fixedTimeAsDouble + enableAfterSeconds;
            }
            if (removeAfterSeconds >= 0f)
                ScheduledSimulationTime = Time.fixedTimeAsDouble + removeAfterSeconds;
        }

        private void FixedUpdate() {
            if (target == null) return;
            double now = Time.fixedTimeAsDouble;
            if (!activationComplete && now >= ScheduledActivationSimulationTime) {
                target.SetActive(true);
                activationComplete = true;
                onActivated?.Invoke(now);
                Debug.Log($"CRANE_LAND_BLOCKER_ACTIVATED simulationTime={now:R}");
                if (ScheduledSimulationTime < 0d) enabled = false;
            }
            if (ScheduledSimulationTime < 0d || now < ScheduledSimulationTime) return;
            target.SetActive(false);
            onRemoved?.Invoke(now);
            Debug.Log($"CRANE_LAND_BLOCKER_REMOVED simulationTime={now:R}");
            enabled = false;
        }
    }
}
