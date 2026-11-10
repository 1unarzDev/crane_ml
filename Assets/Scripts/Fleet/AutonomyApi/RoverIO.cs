using System.Collections.Generic;
using Crane.Fleet.Docking;
using Crane.Fleet.Perception;
using Crane.Fleet.Perception.Rover;

namespace Crane.Fleet.Autonomy {
    /// <summary>The rover's own pose estimate from its own sensors (CRANE wheel-encoder translation + gyro heading), never truth.</summary>
    public struct OdometryEstimate {
        public double T, X, Y, Yaw;
        public Twist Twist;
    }

    /// <summary>What the rover can sense about one of its docking faces (connector state machine output and hook switches).</summary>
    public struct ConnectorStatus {
        public string Face, Coupling, State;   // State: profile name (FREE, APPROACH, MAGNET_CAPTURED, ... LATCHED_VERIFIED, RELEASED)
        public bool SensedA, SensedB, Coupled;  // Coupled: the physical coupling exists
        public int FailedAttempts;
        public bool RetriesExhausted;
    }

    /// <summary>
    /// Everything an autonomy layer may read or command for one rover in CRANE. Engine-free and truth-free: observations come from the
    /// perception core (tags with noise/latency/dropout from the rover's own cameras, peers and stations; encoders, IMU, ToF), the
    /// odometry estimate from CRANE's own encoders and gyro, design knowledge (frames, limits) from the rover description. Commands go to
    /// the plant (body twist; limits, lag, slip and traction are the plant's), the lifter, the gripper and the connectors.
    /// </summary>
    public interface IRoverIO {
        string Id { get; }
        string Role { get; }
        RoverDescription Description { get; }
        IObservationProvider Sensors { get; }
        OdometryEstimate Odometry { get; }
        double LiftPosition { get; }
        double GripperOpening { get; }
        bool GripperHolding { get; }
        IReadOnlyList<string> Faces { get; }
        ConnectorStatus Connector(string face);

        void CommandTwist(Twist body);
        void CommandLift(double q);
        void CommandGripper(double opening);
        /// <summary>Approach (the simulator pairs the face with the nearest compatible facing face), EngageHooks, Verify, TestLoad, Release.</summary>
        void CommandConnector(string face, DockingCommand cmd);
        /// <summary>Free-text status shown in logs and still overlays ("which controller is in charge").</summary>
        void Report(string controller, string phase);
    }

    /// <summary>An autonomy layer for the whole fleet. CRANE calls Initialize once after settling, then Step before every physics step.</summary>
    public interface IFleetAutonomy {
        void Initialize(IReadOnlyList<IRoverIO> rovers, FleetSpec spec, JsonNode mission, double dt);
        void Step(double t);
        bool Done { get; }
        string Status { get; }
    }
}
