using System;
using System.Collections;
using System.IO;
using System.Linq;
using UnityEngine;

namespace Sim.Performance {
    [Serializable]
    internal sealed class ReplayOutcomeValidationResult {
        public string schema = "crane-replay-outcome-validation-v1";
        public string unityVersion;
        public bool playerFound;
        public int outcomeCount;
        public bool rewardSequenceValid;
        public bool terminationValid;
        public bool truncationValid;
        public bool valid;
    }

    internal sealed class CraneReplayOutcomeValidationRunner : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains(
                    "--crane-replay-outcome-validation")) return;
            var host = new GameObject("CRANE Replay Outcome Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneReplayOutcomeValidationRunner>();
        }

        private IEnumerator Start() {
            var result = new ReplayOutcomeValidationResult { unityVersion = Application.unityVersion };
            CraneEpisodePlayer player = null;
            float deadline = Time.realtimeSinceStartup + 10;
            while (Time.realtimeSinceStartup < deadline) {
                player = FindAnyObjectByType<CraneEpisodePlayer>();
                if (player != null && player.LastOutcomes.Count > 0) break;
                yield return null;
            }

            result.playerFound = player != null;
            if (player != null) {
                player.Pause(true);
                CraneReplayOutcome[] outcomes = player.LastOutcomes.ToArray();
                result.outcomeCount = outcomes.Length;
                result.rewardSequenceValid = outcomes.Length == 3 &&
                    outcomes[0].Sequence == 1 && outcomes[0].Reward == 0.25 &&
                    outcomes[0].CumulativeReward == 0.25 &&
                    outcomes[1].Sequence == 2 && outcomes[1].Reward == 1.75 &&
                    outcomes[1].CumulativeReward == 2.0;
                result.terminationValid = outcomes.Length == 3 && outcomes[1].Terminated &&
                    !outcomes[1].Truncated && outcomes[1].Reason == "goal";
                result.truncationValid = outcomes.Length == 3 && outcomes[2].Sequence == 1 &&
                    outcomes[2].CumulativeReward == -0.5 && !outcomes[2].Terminated &&
                    outcomes[2].Truncated && outcomes[2].Reason == "timeout" &&
                    outcomes[2].EpisodeId != outcomes[1].EpisodeId;
            }
            result.valid = result.playerFound && result.rewardSequenceValid &&
                result.terminationValid && result.truncationValid;

            string[] args = Environment.GetCommandLineArgs();
            string outputPath = CraneReplayFormat.ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-replay-outcome-validation.json");
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_REPLAY_OUTCOME_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            Application.Quit(result.valid ? 0 : 2);
        }
    }
}
