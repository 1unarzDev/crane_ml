using System;
using System.Collections.Generic;
using System.Globalization;

namespace Sim.Utils.Performance {
    public enum CraneActionPolicy {
        LatestValid,
        BoundedLag
    }

    public enum CraneActionRejection {
        None,
        CrossEpisode,
        DuplicateOrOutOfOrder,
        Stale
    }

    public readonly struct CraneActionReceipt {
        public readonly string Source;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long SourceObservationTick;
        public readonly long ReceiveTick;

        internal CraneActionReceipt(string source, long episodeId, long sequence,
            long sourceObservationTick, long receiveTick) {
            Source = source;
            EpisodeId = episodeId;
            Sequence = sequence;
            SourceObservationTick = sourceObservationTick;
            ReceiveTick = receiveTick;
        }
    }

    public readonly struct CraneActionPayload {
        public readonly string Encoding;
        public readonly byte[] Data;

        public CraneActionPayload(string encoding, byte[] data) {
            Encoding = encoding ?? string.Empty;
            Data = data ?? Array.Empty<byte>();
        }
    }

    public readonly struct CraneAcceptedAction {
        public readonly string Source;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long SourceObservationTick;
        public readonly long ReceiveTick;
        public readonly long ApplicationTick;
        public readonly CraneActionPayload Payload;

        internal CraneAcceptedAction(in CraneActionReceipt receipt, long applicationTick,
            in CraneActionPayload payload) {
            Source = receipt.Source;
            EpisodeId = receipt.EpisodeId;
            Sequence = receipt.Sequence;
            SourceObservationTick = receipt.SourceObservationTick;
            ReceiveTick = receipt.ReceiveTick;
            ApplicationTick = applicationTick;
            Payload = payload;
        }
    }

    public static class CraneActionPayloadEncoding {
        public static CraneActionPayload Float32(float value) {
            int bits = BitConverter.SingleToInt32Bits(value);
            return new CraneActionPayload("float32-le", new[] {
                (byte)bits, (byte)(bits >> 8), (byte)(bits >> 16), (byte)(bits >> 24)
            });
        }

        public static CraneActionPayload Int32(int value) =>
            new("int32-le", new[] {
                (byte)value, (byte)(value >> 8), (byte)(value >> 16), (byte)(value >> 24)
            });

        public static CraneActionPayload UInt16Array(ushort[] values) {
            byte[] bytes = new byte[values.Length * sizeof(ushort)];
            for (int i = 0; i < values.Length; i++) {
                bytes[i * 2] = (byte)values[i];
                bytes[i * 2 + 1] = (byte)(values[i] >> 8);
            }
            return new CraneActionPayload("uint16-le[]", bytes);
        }
    }

    /// <summary>
    /// Single-slot, latest-value mailbox for transport callbacks. Receive may run on a
    /// background thread; TryApply is called by the owning component from FixedUpdate.
    /// Replacing an unconsumed payload is intentional for latest-command control streams.
    /// </summary>
    public sealed class CraneQueuedAction<T> {
        private readonly object queueLock = new();
        private readonly Func<T, CraneActionPayload> payloadEncoder;
        private T pendingPayload;
        private CraneActionReceipt pendingReceipt;
        private bool hasPending;

        public CraneQueuedAction(Func<T, CraneActionPayload> payloadEncoder = null) {
            this.payloadEncoder = payloadEncoder;
        }

        public void Receive(T payload, string source, long sequence,
            long sourceObservationTick = -1) {
            CraneActionReceipt receipt = CraneActionGate.Receive(source, sequence,
                sourceObservationTick);
            lock (queueLock) {
                pendingPayload = payload;
                pendingReceipt = receipt;
                hasPending = true;
            }
        }

        public bool TryApply(Action<T> apply, out CraneActionRejection rejection) {
            T payload;
            CraneActionReceipt receipt;
            lock (queueLock) {
                if (!hasPending) {
                    rejection = CraneActionRejection.None;
                    return false;
                }
                payload = pendingPayload;
                receipt = pendingReceipt;
                pendingPayload = default;
                hasPending = false;
            }

            if (!CraneActionGate.TryApply(receipt, out rejection)) return false;
            apply(payload);
            if (payloadEncoder != null && CraneActionGate.HasAcceptedActionObservers)
                CraneActionGate.ReportAcceptedPayload(receipt, payloadEncoder(payload));
            return true;
        }

        public void Clear() {
            lock (queueLock) {
                pendingPayload = default;
                pendingReceipt = default;
                hasPending = false;
            }
        }
    }

    /// <summary>
    /// Thread-safe action provenance and acceptance module. Transport adapters receive an
    /// immutable receipt, then cross this seam once on the Unity fixed-update thread before
    /// mutating actuators.
    /// </summary>
    public static class CraneActionGate {
        private readonly struct AppliedSequence {
            public readonly long EpisodeId;
            public readonly long Sequence;
            public AppliedSequence(long episodeId, long sequence) {
                EpisodeId = episodeId;
                Sequence = sequence;
            }
        }

        private static readonly object s_Lock = new();
        private static readonly Dictionary<string, AppliedSequence> s_LastApplied = new();
        private static Action<CraneAcceptedAction> s_AcceptedActionObservers;
        private static CraneActionPolicy s_Policy;
        private static long s_MaxLagTicks;

        static CraneActionGate() {
            string[] args = Environment.GetCommandLineArgs();
            string policy = ReadArgument(args, "--crane-action-policy") ?? "latest";
            s_Policy = policy.Equals("bounded", StringComparison.OrdinalIgnoreCase) ?
                CraneActionPolicy.BoundedLag : CraneActionPolicy.LatestValid;
            s_MaxLagTicks = Math.Max(0, ReadLong(args, "--crane-max-action-lag-ticks", 5));
        }

        public static CraneActionPolicy Policy {
            get { lock (s_Lock) return s_Policy; }
        }
        public static long MaximumLagTicks {
            get { lock (s_Lock) return s_MaxLagTicks; }
        }
        internal static bool HasAcceptedActionObservers {
            get { lock (s_Lock) return s_AcceptedActionObservers != null; }
        }

        public static void SubscribeAcceptedActions(Action<CraneAcceptedAction> observer) {
            if (observer == null) throw new ArgumentNullException(nameof(observer));
            lock (s_Lock) s_AcceptedActionObservers += observer;
        }

        public static void UnsubscribeAcceptedActions(Action<CraneAcceptedAction> observer) {
            if (observer == null) return;
            lock (s_Lock) s_AcceptedActionObservers -= observer;
        }

        public static CraneActionReceipt Receive(string source, long sequence,
            long sourceObservationTick = -1) {
            var receipt = new CraneActionReceipt(source ?? "unknown",
                CraneRuntimeMetrics.EpisodeId, sequence, sourceObservationTick,
                CraneRuntimeMetrics.SimulationTick);
            CraneRuntimeMetrics.ReportActionReceived(receipt.ReceiveTick,
                sourceObservationTick < 0);
            return receipt;
        }

        public static bool TryApply(in CraneActionReceipt receipt,
            out CraneActionRejection rejection) {
            long episodeId = CraneRuntimeMetrics.EpisodeId;
            long applicationTick = CraneRuntimeMetrics.SimulationTick;
            lock (s_Lock) {
                if (receipt.EpisodeId != episodeId) {
                    rejection = CraneActionRejection.CrossEpisode;
                    CraneRuntimeMetrics.ReportRejectedAction(rejection);
                    return false;
                }

                if (s_LastApplied.TryGetValue(receipt.Source, out AppliedSequence last) &&
                    last.EpisodeId == episodeId && receipt.Sequence <= last.Sequence) {
                    rejection = CraneActionRejection.DuplicateOrOutOfOrder;
                    CraneRuntimeMetrics.ReportRejectedAction(rejection);
                    return false;
                }

                long provenanceTick = receipt.SourceObservationTick >= 0 ?
                    receipt.SourceObservationTick : receipt.ReceiveTick;
                if (s_Policy == CraneActionPolicy.BoundedLag &&
                    applicationTick - provenanceTick > s_MaxLagTicks) {
                    rejection = CraneActionRejection.Stale;
                    CraneRuntimeMetrics.ReportRejectedAction(rejection);
                    return false;
                }

                s_LastApplied[receipt.Source] = new AppliedSequence(episodeId, receipt.Sequence);
            }

            rejection = CraneActionRejection.None;
            CraneRuntimeMetrics.ReportAction(receipt.SourceObservationTick, receipt.ReceiveTick,
                applicationTick, receipt.Sequence);
            return true;
        }

        public static void Configure(CraneActionPolicy policy, long maximumLagTicks) {
            lock (s_Lock) {
                s_Policy = policy;
                s_MaxLagTicks = Math.Max(0, maximumLagTicks);
                s_LastApplied.Clear();
            }
        }

        internal static void ReportAcceptedPayload(in CraneActionReceipt receipt,
            in CraneActionPayload payload) {
            Action<CraneAcceptedAction> observers;
            lock (s_Lock) observers = s_AcceptedActionObservers;
            if (observers == null) return;
            var accepted = new CraneAcceptedAction(receipt,
                CraneRuntimeMetrics.ActionApplicationTick, payload);
            observers.Invoke(accepted);
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }

        private static long ReadLong(string[] args, string key, long fallback) =>
            long.TryParse(ReadArgument(args, key), NumberStyles.Integer,
                CultureInfo.InvariantCulture, out long value) ? value : fallback;
    }
}
