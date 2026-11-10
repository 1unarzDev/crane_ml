using System.Collections.Generic;

// Rover-facing API. Nothing in this namespace (or anywhere outside Crane.Fleet.Perception.Evaluator) may reference evaluator truth;
// TruthFirewallTests enforces that by reflection over the assembly (signatures, fields, base types and IL call/field tokens).
namespace Crane.Fleet.Perception.Rover
{
    /// <summary>What one rover can get from its sensors. Every method returns only measurements it could have on hardware.</summary>
    public interface IObservationProvider
    {
        string RoverId { get; }
        SensorSuite Suite { get; }
        /// <summary>Tag-derived relative poses (own camera, peers, fixed stations, overhead) delivered by time <paramref name="now"/> and not returned before.</summary>
        IReadOnlyList<RelPoseObservation> PollTagObservations(double now);
        IReadOnlyList<EncoderReading> PollEncoders(double now);
        IReadOnlyList<ImuReading> PollImu(double now);
        IReadOnlyList<RangeReading> PollTof(string mountName, double now);
        IReadOnlyList<LidarScan> PollLidar(double now);
        /// <summary>The rover's own localisation output; peers' shared observations are transformed through the latest estimate at or before acquisition.</summary>
        void ReportSelfEstimate(double stamp, Pose pose, Matrix covariance);
    }

    /// <summary>Transport between a rover handle and whatever produces measurements (evaluator simulator, or later hardware bridge).</summary>
    public interface IObservationBackend
    {
        IReadOnlyList<RelPoseObservation> PollTagObservations(string roverId, double now);
        IReadOnlyList<EncoderReading> PollEncoders(string roverId, double now);
        IReadOnlyList<ImuReading> PollImu(string roverId, double now);
        IReadOnlyList<RangeReading> PollTof(string roverId, string mountName, double now);
        IReadOnlyList<LidarScan> PollLidar(string roverId, double now);
        void ReportSelfEstimate(string roverId, double stamp, Pose pose, Matrix covariance);
        SensorSuite SuiteOf(string roverId);
    }

    /// <summary>The per-rover handle: knows its id and the backend interface, nothing else.</summary>
    public sealed class RoverObservationProvider : IObservationProvider
    {
        private readonly IObservationBackend _b;
        public string RoverId { get; }
        public SensorSuite Suite => _b.SuiteOf(RoverId);
        public RoverObservationProvider(IObservationBackend backend, string roverId) { _b = backend; RoverId = roverId; }
        public IReadOnlyList<RelPoseObservation> PollTagObservations(double now) => _b.PollTagObservations(RoverId, now);
        public IReadOnlyList<EncoderReading> PollEncoders(double now) => _b.PollEncoders(RoverId, now);
        public IReadOnlyList<ImuReading> PollImu(double now) => _b.PollImu(RoverId, now);
        public IReadOnlyList<RangeReading> PollTof(string mountName, double now) => _b.PollTof(RoverId, mountName, now);
        public IReadOnlyList<LidarScan> PollLidar(double now) => _b.PollLidar(RoverId, now);
        public void ReportSelfEstimate(double stamp, Pose pose, Matrix covariance) => _b.ReportSelfEstimate(RoverId, stamp, pose, covariance);
    }
}
