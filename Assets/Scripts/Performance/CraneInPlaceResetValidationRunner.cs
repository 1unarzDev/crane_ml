using System;
using System.Collections;
using System.IO;
using System.Linq;
using Sim.Utils.Performance;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Performance {
    [Serializable]
    internal sealed class CraneInPlaceResetValidationResult {
        public bool valid;
        public long firstEpisodeId;
        public long resetEpisodeId;
        public int rigidbodyCount;
        public int articulationCount;
        public int resettableCount;
        public double resetWallMilliseconds;
        public float positionError;
        public float rotationErrorDegrees;
        public float linearVelocityError;
        public float angularVelocityError;
        public bool customStateRestored;
        public double episodeClockAfterReset;
        public long simulationTickAfterReset;
    }

    internal sealed class CraneResetValidationState : MonoBehaviour, ICraneEpisodeResettable {
        public int Value { get; set; } = 17;
        private int initialValue;
        public int ResetPriority => 0;
        public void CaptureEpisodeInitialState() => initialValue = Value;
        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.AfterPhysics) Value = initialValue;
        }
    }

    /// <summary>Executable A-B-A reset fixture enabled by --crane-in-place-reset-validation.</summary>
    internal sealed class CraneInPlaceResetValidationRunner : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains(
                    "--crane-in-place-reset-validation")) return;
            var host = new GameObject("CRANE In-Place Reset Validation");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneInPlaceResetValidationRunner>();
        }

        private IEnumerator Start() {
            if (SceneManager.GetActiveScene().name != "Aerial Vehicle Validation") {
                AsyncOperation load = SceneManager.LoadSceneAsync(
                    "Aerial Vehicle Validation", LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            // Allow scene Start methods and runtime profile application to complete before
            // capturing the authoritative A state.
            yield return null;
            long firstEpisode = CraneRuntimeMetrics.EpisodeId;
            var fixture = new GameObject("reset-rigidbody-fixture");
            fixture.transform.SetPositionAndRotation(new Vector3(1, 2, 3),
                Quaternion.Euler(4, 5, 6));
            Rigidbody body = fixture.AddComponent<Rigidbody>();
            body.useGravity = false;
            body.linearVelocity = new Vector3(0.25f, -0.5f, 0.75f);
            body.angularVelocity = new Vector3(-0.1f, 0.2f, -0.3f);
            CraneResetValidationState custom = fixture.AddComponent<CraneResetValidationState>();
            yield return new WaitForFixedUpdate();

            Vector3 expectedPosition = body.position;
            Quaternion expectedRotation = body.rotation;
            Vector3 expectedLinearVelocity = body.linearVelocity;
            Vector3 expectedAngularVelocity = body.angularVelocity;
            var coordinator = new CraneEpisodeResetCoordinator(
                SceneManager.GetActiveScene(), 424242);
            coordinator.Capture();

            body.position = new Vector3(-10, 12, -14);
            body.rotation = Quaternion.Euler(40, 50, 60);
            body.linearVelocity = Vector3.one * 9;
            body.angularVelocity = Vector3.one * -7;
            custom.Value = 99;
            CraneRuntimeMetrics.AdvanceSimulationTick();
            yield return new WaitForFixedUpdate();

            CraneEpisodeResetResult reset = coordinator.Reset();
            var result = new CraneInPlaceResetValidationResult {
                firstEpisodeId = firstEpisode,
                resetEpisodeId = reset.EpisodeId,
                rigidbodyCount = reset.RigidbodyCount,
                articulationCount = reset.ArticulationCount,
                resettableCount = reset.ResettableCount,
                resetWallMilliseconds = reset.WallMilliseconds,
                positionError = Vector3.Distance(expectedPosition, body.position),
                rotationErrorDegrees = Quaternion.Angle(expectedRotation, body.rotation),
                linearVelocityError = Vector3.Distance(expectedLinearVelocity,
                    body.linearVelocity),
                angularVelocityError = Vector3.Distance(expectedAngularVelocity,
                    body.angularVelocity),
                customStateRestored = custom.Value == 17,
                episodeClockAfterReset = Sim.Utils.ROS.Clock.time,
                simulationTickAfterReset = CraneRuntimeMetrics.SimulationTick
            };
            result.valid = reset.EpisodeId > firstEpisode &&
                result.positionError < 1e-5f && result.rotationErrorDegrees < 1e-4f &&
                result.linearVelocityError < 1e-5f && result.angularVelocityError < 1e-5f &&
                result.customStateRestored && result.episodeClockAfterReset < 0.1 &&
                result.simulationTickAfterReset == 0;

            string json = JsonUtility.ToJson(result, true);
            string output = ReadArgument("--crane-output") ??
                Path.Combine(Application.persistentDataPath,
                    "crane-in-place-reset-validation.json");
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)) ?? ".");
            File.WriteAllText(output, json + Environment.NewLine);
            Debug.Log($"CRANE_IN_PLACE_RESET_VALIDATION {json}");
            Application.Quit(result.valid ? 0 : 2);
        }

        private static string ReadArgument(string key) {
            string[] args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }
    }
}
