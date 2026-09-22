using System;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Applies one manifest-declared warehouse obstacle state to the canonical and presentation
    /// layers at the same deterministic simulation-time boundary. Canonical collision remains the
    /// authority; the visual object only mirrors its active state.
    /// </summary>
    internal sealed class CraneTimedWarehouseObstacle : MonoBehaviour {
        private GameObject canonical;
        private GameObject visual;
        private Action<bool, double> onStateChanged;
        private bool activationComplete;

        public double ScheduledActivationSimulationTime { get; private set; } = -1d;
        public double ScheduledRemovalSimulationTime { get; private set; } = -1d;

        public void Configure(GameObject canonicalObject, GameObject visualObject,
            bool activeInitially, float activationAfterSeconds, float removalAfterSeconds,
            Action<bool, double> stateChanged) {
            canonical = canonicalObject != null
                ? canonicalObject
                : throw new ArgumentNullException(nameof(canonicalObject));
            visual = visualObject;
            onStateChanged = stateChanged;
            activationComplete = activeInitially;
            SetTargetsActive(activeInitially);

            if (!activeInitially) {
                if (activationAfterSeconds < 0f)
                    throw new ArgumentOutOfRangeException(nameof(activationAfterSeconds),
                        "An initially inactive obstacle requires an activation boundary.");
                ScheduledActivationSimulationTime =
                    Time.fixedTimeAsDouble + activationAfterSeconds;
            }
            if (removalAfterSeconds >= 0f)
                ScheduledRemovalSimulationTime = Time.fixedTimeAsDouble + removalAfterSeconds;
            if (ScheduledActivationSimulationTime >= 0d &&
                ScheduledRemovalSimulationTime >= 0d &&
                ScheduledRemovalSimulationTime <= ScheduledActivationSimulationTime)
                throw new ArgumentException(
                    "Warehouse obstacle removal must occur after activation.");
            if (ScheduledActivationSimulationTime < 0d &&
                ScheduledRemovalSimulationTime < 0d)
                enabled = false;
        }

        private void FixedUpdate() {
            if (canonical == null) return;
            double now = Time.fixedTimeAsDouble;
            if (!activationComplete && now >= ScheduledActivationSimulationTime) {
                activationComplete = true;
                SetTargetsActive(true);
                onStateChanged?.Invoke(true, now);
                Debug.Log($"CRANE_WAREHOUSE_OBSTACLE_ACTIVATED id={canonical.name} " +
                          $"simulationTime={now:R}");
                if (ScheduledRemovalSimulationTime < 0d) enabled = false;
            }
            if (ScheduledRemovalSimulationTime < 0d ||
                now < ScheduledRemovalSimulationTime) return;
            SetTargetsActive(false);
            onStateChanged?.Invoke(false, now);
            Debug.Log($"CRANE_WAREHOUSE_OBSTACLE_REMOVED id={canonical.name} " +
                      $"simulationTime={now:R}");
            enabled = false;
        }

        private void SetTargetsActive(bool active) {
            canonical.SetActive(active);
            if (visual != null) visual.SetActive(active);
        }
    }
}
