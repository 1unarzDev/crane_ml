using System;
using System.Collections;
using System.IO;
using System.Linq;
using Sim.Utils.Performance;
using UnityEngine;

namespace Sim.Performance {
    [Serializable]
    internal sealed class ActionValidationResult {
        public string schema = "crane-action-validation-v1";
        public string unityVersion;
        public bool boundedAccepted;
        public long boundedReceiveTick;
        public long boundedApplicationTick;
        public bool duplicateRejected;
        public bool staleKnownRejected;
        public bool staleUnknownRejected;
        public bool crossEpisodeRejected;
        public bool latestPolicyAcceptedOldAction;
        public bool latestPolicyTimingValid;
        public bool queuedPayloadHeldBeforeApply;
        public bool queuedPayloadApplied;
        public bool queuedLatestPayloadApplied;
        public bool queuedRejectedDidNotMutate;
        public bool acceptedPayloadRecorded;
        public bool floatPayloadEncodingValid;
        public bool floatArrayPayloadEncodingValid;
        public bool pwmPayloadEncodingValid;
        public int acceptedPayloadCount;
        public int outcomeEventCount;
        public bool outcomeSequenceValid;
        public bool cumulativeRewardValid;
        public bool terminationValid;
        public bool truncationValid;
        public bool postTerminalRejected;
        public long unknownSourceActionsBeforeReset;
        public long staleActionsBeforeReset;
        public bool valid;
    }

    internal sealed class CraneActionValidationRunner : MonoBehaviour {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains("--crane-action-validation")) return;
            var host = new GameObject("CRANE Action Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneActionValidationRunner>();
        }

        private IEnumerator Start() {
            // Give the replay recorder a fixed-update capture point before this fixture emits
            // actions. A second point before quit ensures the accepted payloads reach a frame.
            yield return new WaitForFixedUpdate();
            var result = new ActionValidationResult { unityVersion = Application.unityVersion };

            CraneActionGate.Configure(CraneActionPolicy.BoundedLag, 2);
            CraneRuntimeMetrics.BeginEpisode();
            CraneRuntimeMetrics.AdvanceSimulationTick();
            long knownTick = CraneRuntimeMetrics.SimulationTick;
            CraneActionReceipt accepted = CraneActionGate.Receive("validation:known", 1, knownTick);
            result.boundedReceiveTick = accepted.ReceiveTick;
            result.boundedAccepted = CraneActionGate.TryApply(accepted, out CraneActionRejection acceptedReason) &&
                acceptedReason == CraneActionRejection.None;
            result.boundedApplicationTick = CraneRuntimeMetrics.ActionApplicationTick;

            result.duplicateRejected = !CraneActionGate.TryApply(accepted,
                out CraneActionRejection duplicateReason) &&
                duplicateReason == CraneActionRejection.DuplicateOrOutOfOrder;

            CraneActionReceipt staleKnown = CraneActionGate.Receive("validation:known", 2, knownTick);
            CraneRuntimeMetrics.AdvanceSimulationTick();
            CraneRuntimeMetrics.AdvanceSimulationTick();
            CraneRuntimeMetrics.AdvanceSimulationTick();
            result.staleKnownRejected = !CraneActionGate.TryApply(staleKnown,
                out CraneActionRejection staleKnownReason) &&
                staleKnownReason == CraneActionRejection.Stale;

            CraneActionReceipt staleUnknown = CraneActionGate.Receive("validation:unknown", 1, -1);
            CraneRuntimeMetrics.AdvanceSimulationTick();
            CraneRuntimeMetrics.AdvanceSimulationTick();
            CraneRuntimeMetrics.AdvanceSimulationTick();
            result.staleUnknownRejected = !CraneActionGate.TryApply(staleUnknown,
                out CraneActionRejection staleUnknownReason) &&
                staleUnknownReason == CraneActionRejection.Stale;
            result.unknownSourceActionsBeforeReset = CraneRuntimeMetrics.UnknownSourceActions;
            result.staleActionsBeforeReset = CraneRuntimeMetrics.StaleActions;

            CraneActionReceipt oldEpisode = CraneActionGate.Receive("validation:episode", 1, -1);
            CraneRuntimeMetrics.BeginEpisode();
            result.crossEpisodeRejected = !CraneActionGate.TryApply(oldEpisode,
                out CraneActionRejection crossEpisodeReason) &&
                crossEpisodeReason == CraneActionRejection.CrossEpisode;

            CraneActionGate.Configure(CraneActionPolicy.LatestValid, 0);
            CraneActionReceipt latest = CraneActionGate.Receive("validation:latest", 1, 0);
            for (int i = 0; i < 10; i++) CraneRuntimeMetrics.AdvanceSimulationTick();
            result.latestPolicyAcceptedOldAction = CraneActionGate.TryApply(latest,
                out CraneActionRejection latestReason) && latestReason == CraneActionRejection.None;
            CraneRuntimeMetrics.ActionTimingSnapshot latestTiming =
                CraneRuntimeMetrics.GetActionTimingSnapshot();
            result.latestPolicyTimingValid = latestTiming.AcceptedActions == 1 &&
                latestTiming.KnownSourceActions == 1 &&
                latestTiming.SourceToApplicationTicks == 10 &&
                latestTiming.MaximumSourceToApplicationTicks == 10 &&
                latestTiming.ReceiveToApplicationTicks == 10 &&
                latestTiming.MaximumReceiveToApplicationTicks == 10 &&
                latestTiming.MaximumInterApplicationTicks == 0 &&
                latestTiming.CommandTimeouts == 0;

            CraneActionGate.Configure(CraneActionPolicy.BoundedLag, 2);
            CraneRuntimeMetrics.BeginEpisode();
            int appliedPayload = 0;
            CraneAcceptedAction recordedAction = default;
            Action<CraneAcceptedAction> observer = action => {
                recordedAction = action;
                result.acceptedPayloadCount++;
            };
            CraneActionGate.SubscribeAcceptedActions(observer);
            var queued = new CraneQueuedAction<int>(CraneActionPayloadEncoding.Int32);
            queued.Receive(11, "validation:queued", 1);
            result.queuedPayloadHeldBeforeApply = appliedPayload == 0;
            result.queuedPayloadApplied = queued.TryApply(value => appliedPayload = value, out _) &&
                appliedPayload == 11;
            queued.Receive(12, "validation:queued", 2);
            queued.Receive(13, "validation:queued", 3);
            result.queuedLatestPayloadApplied = queued.TryApply(value => appliedPayload = value, out _) &&
                appliedPayload == 13;
            queued.Receive(99, "validation:queued", 4);
            CraneRuntimeMetrics.BeginEpisode();
            result.queuedRejectedDidNotMutate = !queued.TryApply(value => appliedPayload = value,
                out CraneActionRejection queuedReason) &&
                queuedReason == CraneActionRejection.CrossEpisode && appliedPayload == 13;
            CraneActionGate.UnsubscribeAcceptedActions(observer);
            byte[] recordedBytes = recordedAction.Payload.Data;
            result.acceptedPayloadRecorded = result.acceptedPayloadCount == 2 &&
                recordedAction.Source == "validation:queued" && recordedAction.Sequence == 3 &&
                recordedAction.Payload.Encoding == "int32-le" && recordedBytes.Length == 4 &&
                recordedBytes[0] == 13 && recordedBytes[1] == 0 &&
                recordedBytes[2] == 0 && recordedBytes[3] == 0;
            CraneActionPayload floatPayload = CraneActionPayloadEncoding.Float32(1.5f);
            result.floatPayloadEncodingValid = floatPayload.Encoding == "float32-le" &&
                floatPayload.Data.Length == 4 && floatPayload.Data[0] == 0 &&
                floatPayload.Data[1] == 0 && floatPayload.Data[2] == 0xC0 &&
                floatPayload.Data[3] == 0x3F;
            CraneActionPayload floatArrayPayload = CraneActionPayloadEncoding.Float32Array(
                new[] { 1.5f, -2f });
            result.floatArrayPayloadEncodingValid =
                floatArrayPayload.Encoding == "float32-le[]" &&
                floatArrayPayload.Data.Length == 8 &&
                floatArrayPayload.Data[0] == 0 && floatArrayPayload.Data[1] == 0 &&
                floatArrayPayload.Data[2] == 0xC0 && floatArrayPayload.Data[3] == 0x3F &&
                floatArrayPayload.Data[4] == 0 && floatArrayPayload.Data[5] == 0 &&
                floatArrayPayload.Data[6] == 0 && floatArrayPayload.Data[7] == 0xC0;
            CraneActionPayload pwmPayload = CraneActionPayloadEncoding.UInt16Array(
                new ushort[] { 1000, 2000 });
            result.pwmPayloadEncodingValid = pwmPayload.Encoding == "uint16-le[]" &&
                pwmPayload.Data.Length == 4 && pwmPayload.Data[0] == 0xE8 &&
                pwmPayload.Data[1] == 0x03 && pwmPayload.Data[2] == 0xD0 &&
                pwmPayload.Data[3] == 0x07;

            CraneTaskOutcomeEvent lastOutcome = default;
            Action<CraneTaskOutcomeEvent> outcomeObserver = outcome => {
                lastOutcome = outcome;
                result.outcomeEventCount++;
            };
            CraneTaskOutcome.Subscribe(outcomeObserver);
            CraneTaskOutcomeEvent firstOutcome = CraneTaskOutcome.Report("validation:task", 0.25);
            CraneTaskOutcomeEvent terminalOutcome = CraneTaskOutcome.Report("validation:task", 1.75,
                terminated: true, reason: "goal");
            result.outcomeSequenceValid = firstOutcome.Sequence == 1 && terminalOutcome.Sequence == 2;
            result.cumulativeRewardValid = firstOutcome.CumulativeReward == 0.25 &&
                terminalOutcome.CumulativeReward == 2.0;
            result.terminationValid = terminalOutcome.Terminated && !terminalOutcome.Truncated &&
                terminalOutcome.Reason == "goal";
            try {
                CraneTaskOutcome.Report("validation:task", 10);
            }
            catch (InvalidOperationException) {
                result.postTerminalRejected = true;
            }
            CraneRuntimeMetrics.BeginEpisode();
            CraneTaskOutcomeEvent truncatedOutcome = CraneTaskOutcome.Report("validation:task", -0.5,
                truncated: true, reason: "timeout");
            result.truncationValid = truncatedOutcome.Sequence == 1 &&
                truncatedOutcome.CumulativeReward == -0.5 && !truncatedOutcome.Terminated &&
                truncatedOutcome.Truncated && truncatedOutcome.Reason == "timeout" &&
                lastOutcome.Sequence == truncatedOutcome.Sequence;
            CraneTaskOutcome.Unsubscribe(outcomeObserver);

            result.valid = result.boundedAccepted &&
                result.boundedReceiveTick == result.boundedApplicationTick &&
                result.duplicateRejected && result.staleKnownRejected &&
                result.staleUnknownRejected && result.crossEpisodeRejected &&
                result.latestPolicyAcceptedOldAction && result.latestPolicyTimingValid &&
                result.queuedPayloadHeldBeforeApply && result.queuedPayloadApplied &&
                result.queuedLatestPayloadApplied && result.queuedRejectedDidNotMutate &&
                result.acceptedPayloadRecorded && result.floatPayloadEncodingValid &&
                result.floatArrayPayloadEncodingValid &&
                result.pwmPayloadEncodingValid && result.outcomeEventCount == 3 &&
                result.outcomeSequenceValid && result.cumulativeRewardValid &&
                result.terminationValid && result.truncationValid &&
                result.postTerminalRejected &&
                result.unknownSourceActionsBeforeReset == 1 &&
                result.staleActionsBeforeReset == 2;

            string[] args = Environment.GetCommandLineArgs();
            string outputPath = ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-action-validation.json");
            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_ACTION_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            yield return new WaitForFixedUpdate();
            yield return null;
            Application.Quit(result.valid ? 0 : 2);
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }
    }
}
