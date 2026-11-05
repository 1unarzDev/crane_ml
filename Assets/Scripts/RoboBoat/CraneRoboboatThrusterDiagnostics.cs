using System;
using System.Globalization;
using Sim.Actuators.Motors;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Opt-in runtime sampler for diagnosing the RoboBoat command-to-thruster boundary.
    /// This component observes existing commands and shaft velocities without modifying them.
    /// </summary>
    public sealed class CraneRoboboatThrusterDiagnostics : MonoBehaviour {
        private const double SamplePeriodSeconds = 0.1;

        private OmniXController controller;
        private double nextSampleTime;

        public void Initialize(OmniXController target) {
            controller = target != null ? target : throw new ArgumentNullException(nameof(target));
            nextSampleTime = 0.0;
        }

        private void FixedUpdate() {
            if (controller == null || Time.fixedTimeAsDouble + 1e-9 < nextSampleTime) return;
            do nextSampleTime += SamplePeriodSeconds;
            while (nextSampleTime <= Time.fixedTimeAsDouble + 1e-9);

            Thruster fl = controller.frontLeft;
            Thruster fr = controller.frontRight;
            Thruster rl = controller.rearLeft;
            Thruster rr = controller.rearRight;
            if (fl == null || fr == null || rl == null || rr == null) return;

            Debug.Log(string.Format(CultureInfo.InvariantCulture,
                "CRANE_ROBOBOAT_THRUSTER_SAMPLE t={0:R} " +
                "fl_command={1:R} fl_speed={2:R} " +
                "fr_command={3:R} fr_speed={4:R} " +
                "rl_command={5:R} rl_speed={6:R} " +
                "rr_command={7:R} rr_speed={8:R}",
                Time.fixedTimeAsDouble,
                fl.GetCommand(), fl.GetVelocity(),
                fr.GetCommand(), fr.GetVelocity(),
                rl.GetCommand(), rl.GetVelocity(),
                rr.GetCommand(), rr.GetVelocity()));
        }
    }

    internal static class CraneRoboboatThrusterDiagnosticsBootstrap {
        private static bool enabled;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            enabled = Array.IndexOf(Environment.GetCommandLineArgs(),
                "--crane-roboboat-thruster-diagnostics") >= 0;
            if (enabled) SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            if (controller == null) {
                Debug.LogWarning($"CRANE_ROBOBOAT_THRUSTER_DIAGNOSTICS_UNAVAILABLE " +
                                 $"scene={scene.name} reason=no-enabled-omni-controller");
                return;
            }

            var diagnostics = controller.GetComponent<CraneRoboboatThrusterDiagnostics>() ??
                              controller.gameObject.AddComponent<CraneRoboboatThrusterDiagnostics>();
            diagnostics.Initialize(controller);
            Debug.Log($"CRANE_ROBOBOAT_THRUSTER_DIAGNOSTICS_READY scene={scene.name}");
        }
    }
}
