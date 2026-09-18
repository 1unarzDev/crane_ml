using System;
using System.Collections.Generic;

namespace Sim.Utils.Performance {
    /// <summary>An immutable task decision result stamped on the authoritative simulation timeline.</summary>
    public readonly struct CraneTaskOutcomeEvent {
        public readonly string Source;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long Tick;
        public readonly double Reward;
        public readonly double CumulativeReward;
        public readonly bool Terminated;
        public readonly bool Truncated;
        public readonly string Reason;

        internal CraneTaskOutcomeEvent(string source, long episodeId, long sequence, long tick,
            double reward, double cumulativeReward, bool terminated, bool truncated, string reason) {
            Source = source;
            EpisodeId = episodeId;
            Sequence = sequence;
            Tick = tick;
            Reward = reward;
            CumulativeReward = cumulativeReward;
            Terminated = terminated;
            Truncated = truncated;
            Reason = reason;
        }
    }

    /// <summary>
    /// Task-outcome seam shared by training tasks and authoritative recording. It owns episode
    /// stamping, per-source ordering, cumulative reward, and terminal-state enforcement.
    /// </summary>
    public static class CraneTaskOutcome {
        private sealed class SourceState {
            public long EpisodeId = -1;
            public long Sequence;
            public double CumulativeReward;
            public bool Ended;
        }

        private static readonly object s_Lock = new();
        private static readonly Dictionary<string, SourceState> s_States = new();
        private static Action<CraneTaskOutcomeEvent> s_Observers;

        public static CraneTaskOutcomeEvent Report(string source, double reward,
            bool terminated = false, bool truncated = false, string reason = null) {
            if (double.IsNaN(reward) || double.IsInfinity(reward))
                throw new ArgumentOutOfRangeException(nameof(reward), "Reward must be finite.");
            if (terminated && truncated)
                throw new ArgumentException("An outcome cannot be both terminated and truncated.");

            string stableSource = string.IsNullOrWhiteSpace(source) ? "task:unknown" : source;
            long episodeId = CraneRuntimeMetrics.EpisodeId;
            CraneTaskOutcomeEvent outcome;
            Action<CraneTaskOutcomeEvent> observers;
            lock (s_Lock) {
                if (!s_States.TryGetValue(stableSource, out SourceState state)) {
                    state = new SourceState();
                    s_States.Add(stableSource, state);
                }
                if (state.EpisodeId != episodeId) {
                    state.EpisodeId = episodeId;
                    state.Sequence = 0;
                    state.CumulativeReward = 0;
                    state.Ended = false;
                }
                if (state.Ended)
                    throw new InvalidOperationException(
                        $"Task source '{stableSource}' already ended episode {episodeId}.");

                state.Sequence++;
                state.CumulativeReward += reward;
                state.Ended = terminated || truncated;
                outcome = new CraneTaskOutcomeEvent(stableSource, episodeId, state.Sequence,
                    CraneRuntimeMetrics.SimulationTick, reward, state.CumulativeReward,
                    terminated, truncated, reason ?? string.Empty);
                observers = s_Observers;
            }
            observers?.Invoke(outcome);
            return outcome;
        }

        public static void Subscribe(Action<CraneTaskOutcomeEvent> observer) {
            if (observer == null) throw new ArgumentNullException(nameof(observer));
            lock (s_Lock) s_Observers += observer;
        }

        public static void Unsubscribe(Action<CraneTaskOutcomeEvent> observer) {
            if (observer == null) return;
            lock (s_Lock) s_Observers -= observer;
        }
    }
}
