using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;

namespace Crane.Fleet
{
    // =====================================================================================================================
    // Integration surface. CRANE (or the headless harness) implements IClock, one IObservationSource and one IActuatorSink per
    // rover, and calls FleetAutonomy.Step(dt) once per control tick. Nothing here carries ground truth.
    // =====================================================================================================================

    /// <summary>Monotonic simulation (or hardware) clock in seconds. The same clock stamps every observation.</summary>
    public interface IClock
    {
        double Now { get; }
    }

    /// <summary>Quantised wheel encoder sample (counts since power-on), wheel order fl, fr, rl, rr.</summary>
    public struct EncoderSample
    {
        public double Stamp;
        public long Fl, Fr, Rl, Rr;
        public long this[int i] => i == 0 ? Fl : i == 1 ? Fr : i == 2 ? Rl : Rr;
        public EncoderSample(double stamp, long fl, long fr, long rl, long rr) { Stamp = stamp; Fl = fl; Fr = fr; Rl = rl; Rr = rr; }
    }

    /// <summary>Gyro z sample (rad/s, includes the sensor's own bias and noise).</summary>
    public struct GyroSample
    {
        public double Stamp, RateZ;
        public GyroSample(double stamp, double rate) { Stamp = stamp; RateZ = rate; }
    }

    /// <summary>What the connector electronics on one face report (hook sensors and the connector controller's state). The
    /// connector state machine (Crane.Fleet.Docking.ConnectorMachine) runs on the simulator side; the autonomy sees only this.</summary>
    public struct ConnectorReport
    {
        public bool Present;                 // false: no connector machine on this face (never docked)
        public ConnectorState State;
        public bool SensedA, SensedB;        // hook sensor reads
        public RequestedAction Action;       // action requested by the connector controller this tick (BackOffAndReapproach, Abandon, ...)
        public int FailedAttempts;
        public bool RetriesExhausted;
        public string PartnerRover, PartnerFace;   // filled once magnets captured (coupling identity is known from the mated face's ID chip)
    }

    /// <summary>Lift and gripper state as reported by their drivers.</summary>
    public struct LiftReport
    {
        public bool Present;
        public double Height;        // m, lift travel above its lowest position
        public bool GripperClosed;   // gripper reached the commanded closed position
        public bool HoldingObject;   // gripper force/position says an object is held
        /// <summary>Deck load cell / contact switches (cargo decks). DeckLoadValid = false when the rover has no such sensor.</summary>
        public bool DeckLoadValid;
        public double DeckLoadKg;
        /// <summary>The lift actuator is at its force/current limit while short of its height target (load heavier than it can raise).</summary>
        public bool ForceLimited;
        /// <summary>CRANE local patch: IMU tilt magnitude max(|pitch|, |roll|) in rad (0 when unknown); a lift that tips fails as Tipping.</summary>
        public double TiltRad;
    }

    /// <summary>Body-frame specific force in the plane (m/s^2, gravity removed, x forward, y left) from the rover's IMU accelerometer.</summary>
    public struct AccelSample
    {
        public double Stamp, Ax, Ay;
        public AccelSample(double stamp, double ax, double ay) { Stamp = stamp; Ax = ax; Ay = ay; }
    }

    /// <summary>OPTIONAL on an IObservationSource: accelerometer samples since the last call (enables impact/stall/shove detection).</summary>
    public interface IInertialSource
    {
        void ReadAccel(List<AccelSample> into);
    }

    /// <summary>An absolute fix of this rover's base pose in the world frame (LiDAR scan matching, known landmark, external
    /// tracker), with its 3x3 covariance (x, y, yaw; world axes), acquired at Stamp.</summary>
    public struct PoseFix
    {
        public double Stamp; public Pose2 Pose; public double[] Cov3; public string SourceId;
    }

    /// <summary>OPTIONAL on an IObservationSource: absolute pose fixes delivered since the last call.</summary>
    public interface IPoseFixSource
    {
        void ReadPoseFixes(List<PoseFix> into);
    }

    /// <summary>
    /// Per-rover measurement source (what CRANE must provide each tick). Every Read* call returns the measurements DELIVERED since
    /// the previous call (stamped with their acquisition time, so they can be older than Now) into a caller-owned buffer, which it
    /// clears first. Implementations must not block and must never return ground truth.
    /// </summary>
    public interface IObservationSource
    {
        string RoverId { get; }
        void ReadEncoders(List<EncoderSample> into);
        void ReadGyro(List<GyroSample> into);
        /// <summary>Tag-derived relative poses (own camera: parent = "&lt;id&gt;/base_link"; peers, stations, overhead: parent = "world").
        /// Child frames are "&lt;rover&gt;/base_link" or "&lt;object&gt;/base_link" for tagged objects.</summary>
        void ReadTagObservations(List<RelPoseObservation> into);
        /// <summary>Connector on face <paramref name="face"/> ("front", "rear", "left", "right").</summary>
        ConnectorReport ReadConnector(string face);
        LiftReport ReadLift();
    }

    /// <summary>Per-rover actuator sink. Called at most once per tick per channel; values are level-held until the next call.</summary>
    public interface IActuatorSink
    {
        string RoverId { get; }
        /// <summary>Wheel angular speed set-points (rad/s), order fl, fr, rl, rr. Already within the wheel limits.</summary>
        void SetWheelSpeeds(double fl, double fr, double rl, double rr);
        /// <summary>Connector command for one face. Commands are pulses: the autonomy sends a command for exactly one tick and
        /// DockingCommand.None otherwise, matching the edge-triggered ConnectorMachine.</summary>
        void CommandConnector(string face, DockingCommand command);
        /// <summary>Lift height target (m above lowest) and gripper close/open.</summary>
        void SetLift(double heightTarget, bool gripperClosed);
    }
}
