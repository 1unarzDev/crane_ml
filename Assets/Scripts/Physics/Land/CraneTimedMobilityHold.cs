using System;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Temporarily prevents planar rigid-body motion at deterministic simulation-time boundaries.
    /// The intervention is evaluator-owned; robot-visible evidence observes only its consequences.
    /// </summary>
    internal sealed class CraneTimedMobilityHold : MonoBehaviour {
        private Rigidbody target;
        private RigidbodyConstraints originalConstraints;
        private Action<double> onHeld;
        private Action<double> onReleased;
        private bool holdApplied;

        public double ScheduledHoldSimulationTime { get; private set; } = -1d;
        public double ScheduledReleaseSimulationTime { get; private set; } = -1d;

        public void Configure(Rigidbody configuredTarget, float holdAfterSeconds,
            float releaseAfterSeconds, Action<double> holdCallback,
            Action<double> releaseCallback) {
            target = configuredTarget != null
                ? configuredTarget
                : throw new ArgumentNullException(nameof(configuredTarget));
            if (holdAfterSeconds < 0f)
                throw new ArgumentOutOfRangeException(nameof(holdAfterSeconds));
            if (releaseAfterSeconds <= holdAfterSeconds)
                throw new ArgumentException(
                    "Mobility release must occur after the hold begins.");
            originalConstraints = target.constraints;
            onHeld = holdCallback;
            onReleased = releaseCallback;
            double now = Time.fixedTimeAsDouble;
            ScheduledHoldSimulationTime = now + holdAfterSeconds;
            ScheduledReleaseSimulationTime = now + releaseAfterSeconds;
        }

        private void FixedUpdate() {
            if (target == null) return;
            double now = Time.fixedTimeAsDouble;
            if (!holdApplied && now >= ScheduledHoldSimulationTime) {
                target.linearVelocity = Vector3.zero;
                target.angularVelocity = Vector3.zero;
                target.constraints = originalConstraints |
                    RigidbodyConstraints.FreezePositionX |
                    RigidbodyConstraints.FreezePositionZ |
                    RigidbodyConstraints.FreezeRotationY;
                holdApplied = true;
                onHeld?.Invoke(now);
                Debug.Log($"CRANE_LAND_MOBILITY_HELD simulationTime={now:R}");
            }
            if (!holdApplied || now < ScheduledReleaseSimulationTime) return;
            target.constraints = originalConstraints;
            target.linearVelocity = Vector3.zero;
            target.angularVelocity = Vector3.zero;
            onReleased?.Invoke(now);
            Debug.Log($"CRANE_LAND_MOBILITY_RELEASED simulationTime={now:R}");
            enabled = false;
        }
    }
}
