using System;

namespace Sim.Utils.Performance {
    /// <summary>
    /// Compact provenance for an acquired observation. Payload bytes are intentionally not held
    /// here: the replay recorder can subscribe without extending the lifetime of camera/cloud data.
    /// </summary>
    public readonly struct CraneObservationMetadata {
        public readonly string Source;
        public readonly string Topic;
        public readonly string FrameId;
        public readonly string MessageType;
        public readonly string Encoding;
        public readonly string PayloadPolicy;
        public readonly long EpisodeId;
        public readonly long Sequence;
        public readonly long AcquisitionTick;
        public readonly double AcquisitionTime;
        public readonly long CompletionTick;
        public readonly double CompletionTime;
        public readonly int Width;
        public readonly int Height;
        public readonly int RowStep;
        public readonly int ElementCount;
        public readonly long PayloadBytes;

        public CraneObservationMetadata(string source, string topic, string frameId,
            string messageType, string encoding, string payloadPolicy, long episodeId,
            long sequence, long acquisitionTick, double acquisitionTime, long completionTick,
            double completionTime, int width, int height, int rowStep, int elementCount,
            long payloadBytes) {
            Source = source ?? string.Empty;
            Topic = topic ?? string.Empty;
            FrameId = frameId ?? string.Empty;
            MessageType = messageType ?? string.Empty;
            Encoding = encoding ?? string.Empty;
            PayloadPolicy = payloadPolicy ?? string.Empty;
            EpisodeId = episodeId;
            Sequence = sequence;
            AcquisitionTick = acquisitionTick;
            AcquisitionTime = acquisitionTime;
            CompletionTick = completionTick;
            CompletionTime = completionTime;
            Width = width;
            Height = height;
            RowStep = rowStep;
            ElementCount = elementCount;
            PayloadBytes = payloadBytes;
        }
    }

    /// <summary>
    /// Process-local observation event stream. Subscribers must copy metadata synchronously and
    /// must never retain sensor payloads; this keeps recording memory bounded.
    /// </summary>
    public static class CraneObservationJournal {
        private static readonly object s_Lock = new();
        private static Action<CraneObservationMetadata> s_Handlers;

        public static void Subscribe(Action<CraneObservationMetadata> handler) {
            if (handler == null) return;
            lock (s_Lock) s_Handlers += handler;
        }

        public static void Unsubscribe(Action<CraneObservationMetadata> handler) {
            if (handler == null) return;
            lock (s_Lock) s_Handlers -= handler;
        }

        public static void Report(in CraneObservationMetadata metadata) {
            Action<CraneObservationMetadata> handlers;
            lock (s_Lock) handlers = s_Handlers;
            handlers?.Invoke(metadata);
        }
    }
}
