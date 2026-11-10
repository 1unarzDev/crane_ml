using System;
using System.Collections.Generic;
using Crane.Fleet.Perception;

namespace Crane.Fleet
{
    /// <summary>One docking face in the chassis frame (x forward, y left), from the profile's docking.face_frames.</summary>
    public sealed class FaceSpec
    {
        public string Name;
        public Pose2 InBody;
        public double Z;
    }

    /// <summary>Static description of one rover as the autonomy needs it. Every number comes from the shared fleet profile
    /// (fleet_profile_v0.json) via <see cref="RoverSpec.FromProfile"/>; nothing here is truth.</summary>
    public sealed class RoverSpec
    {
        public string Id, Role;
        public double WheelRadius, Lx, Ly, MaxWheelSpeed;
        public int CountsPerRev;
        public double MaxSpeedXy, MaxYawRate;
        public double BodyLength, BodyWidth, OverallWidth;
        public double Mass;
        /// <summary>Lateral efficiency of the nominal floor (profile floors.*.lateral_efficiency): the kinematic model the estimator uses.</summary>
        public double LateralEfficiency = 1.0;
        /// <summary>Per-wheel slip sigma as a fraction of wheel speed (profile floors.*.slip_sigma_frac).</summary>
        public double SlipSigmaFrac = 0.03;
        public double GyroNoiseDensity, GyroBiasInstability;
        public double EncoderRateHz, ImuRateHz;
        public readonly List<FaceSpec> Faces = new List<FaceSpec>();
        public bool HasDeck, HasLift;
        public double DeckLength, DeckWidth, DeckCapacity, LiftTravel, LiftCapacity;
        public bool HasFrontCamera, HasRearCamera;

        public FaceSpec Face(string name)
        {
            foreach (var f in Faces) if (f.Name == name) return f;
            throw new KeyNotFoundException("rover " + Id + " has no face " + name);
        }

        public static RoverSpec FromProfile(FleetProfile p, string roverId, string role, string floor = "hard_tile")
        {
            var s = new RoverSpec { Id = roverId, Role = role };
            s.WheelRadius = p.Num("chassis.wheel.radius");
            s.Lx = p.Num("chassis.wheel.half_wheelbase_lx");
            s.Ly = p.Num("chassis.wheel.half_track_ly");
            s.MaxWheelSpeed = p.Num("chassis.wheel.max_wheel_speed_rad_s");
            s.CountsPerRev = (int)p.Num("chassis.wheel.encoder_counts_per_rev");
            s.MaxSpeedXy = p.Num("chassis.max_speed_xy");
            s.MaxYawRate = p.Num("chassis.max_yaw_rate");
            var body = p.Vec("chassis.body_size_lwh");
            s.BodyLength = body[0]; s.BodyWidth = body[1];
            s.OverallWidth = p.Num("chassis.overall_width_with_wheels");
            s.Mass = p.Num("chassis.base_mass");
            s.LateralEfficiency = p.Num("floors." + floor + ".lateral_efficiency");
            s.SlipSigmaFrac = p.Num("floors." + floor + ".slip_sigma_frac");
            s.GyroNoiseDensity = p.Imu.GyroNoiseDensity; s.GyroBiasInstability = p.Imu.GyroBiasInstability;
            s.EncoderRateHz = p.Encoder.RateHz; s.ImuRateHz = p.Imu.RateHz;
            foreach (var kv in p.FaceFrames)
                s.Faces.Add(new FaceSpec { Name = kv.Key, InBody = new Pose2(kv.Value.X, kv.Value.Y, Se2.Wrap(kv.Value.Yaw)), Z = kv.Value.Z });
            if (role != null && p.Roles.TryGetValue(role, out var rs))
            {
                foreach (var t in rs.Sensors)
                    if (t.Kind == SensorKind.MonoCamera || t.Kind == SensorKind.StereoPair)
                    {
                        if (t.Facing == "rear") s.HasRearCamera = true; else s.HasFrontCamera = true;
                    }
                if (p.Leaves.ContainsKey("roles." + role + ".added_mass")) s.Mass += p.Num("roles." + role + ".added_mass");
                if (p.Leaves.ContainsKey("roles." + role + ".deck_size_lw"))
                {
                    var d = p.Vec("roles." + role + ".deck_size_lw");
                    s.HasDeck = true; s.DeckLength = d[0]; s.DeckWidth = d[1]; s.DeckCapacity = p.Num("roles." + role + ".deck_capacity");
                }
                if (p.Leaves.ContainsKey("roles." + role + ".lift_travel"))
                {
                    s.HasLift = true; s.LiftTravel = p.Num("roles." + role + ".lift_travel"); s.LiftCapacity = p.Num("roles." + role + ".lift_capacity");
                }
            }
            return s;
        }

        /// <summary>Standard 45-degree mecanum inverse kinematics row for wheel i (fl, fr, rl, rr) at a module-frame twist,
        /// identical to Crane.Fleet.Core MecanumGeometry.Inverse and Perception MecanumKinematics.</summary>
        public void WheelRow(int i, out double ax, out double ay, out double aw)
        {
            double k = Lx + Ly, r = WheelRadius;
            switch (i)
            {
                case 0: ax = 1 / r; ay = -1 / r; aw = -k / r; break;
                case 1: ax = 1 / r; ay = 1 / r; aw = k / r; break;
                case 2: ax = 1 / r; ay = 1 / r; aw = -k / r; break;
                default: ax = 1 / r; ay = -1 / r; aw = k / r; break;
            }
        }
    }

    /// <summary>
    /// Every constant the autonomy uses that the fleet profile does not define. All are ASSUMPTIONS (control/policy choices or
    /// stand-ins for measurements); <see cref="Registry"/> lists the measurement or decision that replaces each.
    /// </summary>
    public sealed class AutonomyAssumptions
    {
        // ---- estimation
        public double EncoderQuantFloorRadS = 0.02;      // encoder rate noise floor (1 count at 100 Hz ~ 0.044 rad/s; std of rounding ~ 0.013)
        public double SlipCorrelationInflation = 2.0;    // slip is time-correlated (CRANE overlay: 0.2 s); the white-noise EKF inflates its sigma
        public double EncoderMinInterval = 0.04;         // s: encoder counts are differenced over at least this interval (quantisation)
        public double GyroMinInterval = 0.01;            // s: gyro samples are averaged over this interval
        public double ZuptSigmaLin = 0.002, ZuptSigmaYaw = 0.0005; // zero-velocity update when no wheel count changed over the interval
        public double StartupStillTime = 1.5;            // s held still before the first task (gyro bias from zero-velocity updates)
        // ---- stall / slip / shove detection and relocalisation
        public double StallResidual = 0.07;              // m/s: |encoder velocity - inertial velocity| that flags a stall, shove or gross slip
        public double StallResidualTime = 0.15;          // s it must persist
        public double InertialLeakTime = 1.5;            // s: complementary-filter leak of the accelerometer-integrated velocity toward the encoder velocity
        public double StallYawResidual = 0.12;           // rad/s: |encoder yaw rate - gyro yaw rate| that flags slip (without accelerometer)
        public double StallPosInflation = 0.25, StallYawInflation = 0.08; // added to the pose covariance at detection
        public double StallBackoffDistance = 0.15, StallBackoffSpeed = 0.08, StallHoldTime = 2.0;
        public int MaxStallsPerTask = 3;
        public int RelocaliseStreak = 4;                 // consecutive gate-rejected fixes with mutually consistent innovations
        public double RelocaliseConsistency = 0.08;      // m between successive rejected innovations
        public double CrossFixSigmaRatio = 0.5;          // a rover's camera fixes another rover only if its own position sigma is below this fraction
        // ---- handoff
        public double HandoffPrePlaceBack = 0.25;        // m: lifter aligns on the receiver's tags this far behind the place pose
        public double HandoffAlignPosSigma = 0.008, HandoffAlignYawSigma = 0.02; // gate on the relative estimate before the blind final move
        public double HandoffAlignPosTol = 0.012, HandoffAlignYawTol = 0.02;
        public double HandoffConfirmTimeout = 6.0;       // s to confirm the payload rests on the decks after release
        public double AccelProcessSigma = 0.5;           // m/s^2 white acceleration (constant-velocity model)
        public double YawAccelProcessSigma = 0.5;        // rad/s^2 (large values let slip-corrupted encoder yaw rate override the gyro between samples)
        public double InitialPosSigma = 0.02, InitialYawSigma = 0.02, InitialBiasSigma = 0.005;
        public double TagGateChi2 = 16.3;                // 3 dof, p = 0.001 (q1 v2 innovation gate)
        public double AmbiguousInflation = 0.0;
        public double WorldAmbiguousInflation = 4.0;     // world-frame (overhead/peer) ambiguous PnP: planar part used with covariance x4; 0 discards          // 0: discard AmbiguousPose observations; >0: inflate covariance by this factor
        public double DelayBufferSeconds = 1.0;          // how far back late measurements are re-applied
        public double RelativeProcessPosPerMeter = 0.03; // relative-estimate growth per metre of docker motion (slip, 1 sigma fraction)
        public double RelativeProcessYawPerRad = 0.03;
        // ---- planning
        public double PlanCell = 0.10, PlanStep = 0.15;  // m
        public int PlanHeadings = 16;
        public double PlanMargin = 0.03;                 // m clearance kept from every obstacle
        public double TipTiltLimit = 0.30;               // rad: IMU tilt beyond which a lifting rover has tipped (CRANE local patch)
        public double PlanGoalPosTol = 0.06, PlanGoalYawTol = 0.10;
        public int PlanMaxExpansions = 60000;
        public int PlanMaxExpansionsReduced = 25000;   // reduced-mobility searches are greedier (heuristic weight 2) and bounded tighter
        public double CruiseFraction = 0.6;              // fraction of the twist-set bound used as cruise speed
        public double TransportCruiseFraction = 0.35;
        // ---- tracking
        public double TrackKpos = 1.5, TrackKyaw = 2.0, TrackLookahead = 0.12;
        public double MaxLinAccel = 0.5, MaxYawAccel = 1.5;
        public double ReplanDeviation = 0.20;            // m from the path triggers a replan
        public double ReducedMobilityExtraMargin = 0.06; // extra clearance when the assembly cannot command all three twist directions
        public double ReducedMobilityReplanDeviation = 0.06;
        public double GoalPosTol = 0.03, GoalYawTol = 0.04;
        // ---- docking
        public double PreDockStandoff = 0.50;            // m face gap of the pre-dock pose (wave-3 start range)
        public double BackoffStandoff = 0.30;
        public double DockCloseSpeedFar = 0.06, DockCloseSpeedNear = 0.03; // m/s (closing <= MaxClosingSpeed 0.15 of the docking core)
        public double DockNearGap = 0.12;
        public double DockKLat = 1.2, DockKYaw = 1.5;
        public double DockCommitGap = 0.14;              // below this gap the camera is near its blind range: the estimate must already be good
        public double DockSigmaConfidence = 2.5;         // n-sigma used in the capture-envelope test
        public double DockCaptureLateral = 0.010;        // m: guide half width (profile) used as the lateral capture budget
        public double DockCaptureYaw = 0.087;            // rad: guide acceptance half angle (profile)
        public double DockObserveTimeout = 4.0;          // s waiting for tags before declaring perception failure
        public double DockSettleTime = 0.6;              // s still before the re-observe window counts
        public double DockApproachCommandGap = 0.10;     // m: send connector Approach once the estimate is inside this gap
        public double DockPushSpeed = 0.02;              // m/s creep after the estimate says contact, until capture or jam
        public double DockPhaseTimeout = 40.0;
        public int DockMaxBackoffs = 4;
        // ---- separation
        public double SeparateClearDistance = 0.25;
        // ---- health identification
        public double HealthCmdThreshold = 1.5;          // rad/s commanded before a wheel is judged
        public double HealthWindow = 0.4;                // s of persistent mismatch
        public double HealthLockedFrac = 0.15, HealthWeakFrac = 0.75;

        public AutonomyAssumptions Clone() => (AutonomyAssumptions)MemberwiseClone();

        public sealed class Entry { public string Name, Default, ReplacedBy; public Entry(string n, string d, string r) { Name = n; Default = d; ReplacedBy = r; } }

        public static IReadOnlyList<Entry> Registry => new[]
        {
            new Entry("EncoderQuantFloorRadS", "0.02 rad/s", "encoder noise from a stationary/constant-speed bench log"),
            new Entry("AccelProcessSigma / YawAccelProcessSigma", "0.8 m/s^2 / 3 rad/s^2", "accelerations seen in logged CRANE/hardware runs"),
            new Entry("TagGateChi2", "16.3", "NEES distribution of real tag observations (perception calibration)"),
            new Entry("RelativeProcessPosPerMeter / YawPerRad", "3 % / 3 %", "odometry drift per metre on each floor (marked-floor course)"),
            new Entry("PlanCell / PlanStep / PlanHeadings", "0.10 m / 0.15 m / 16", "planner tuning; doorway sweep in CRANE"),
            new Entry("PlanMargin", "0.03 m", "measured localisation error at doorways + CAD collision envelope"),
            new Entry("TipTiltLimit", "0.30 rad", "static tip angle of the lifter with its rated load (CAD statics); CRANE evaluator uses 0.35 rad"),
            new Entry("CruiseFraction / TransportCruiseFraction", "0.6 / 0.35", "tracking error vs speed in CRANE; payload slide test"),
            new Entry("Track gains, accel limits", "1.5 / 2.0, 0.5 m/s^2, 1.5 rad/s^2", "closed-loop tuning in CRANE; traction-limited acceleration"),
            new Entry("PreDockStandoff / BackoffStandoff", "0.50 / 0.30 m", "operations policy; wave-3 start range"),
            new Entry("DockCloseSpeedFar / Near", "0.06 / 0.03 m/s", "impact test (MaxClosingSpeed) and capture rate vs speed"),
            new Entry("DockCommitGap / DockSigmaConfidence", "0.14 m / 2.5 sigma", "capture rate vs predicted-envelope margin in CRANE"),
            new Entry("DockCaptureLateral / DockCaptureYaw", "profile guide_half_width / guide_acceptance_half_angle", "CAD guide geometry"),
            new Entry("Health thresholds", "1.5 rad/s, 0.4 s, 15 %, 75 %", "motor-driver fault logs; bench locked/free/weak tests"),
        };

        /// <summary>Copies the capture budget from the docking profile (guide half width and acceptance angle) so it is not duplicated.</summary>
        public void BindDockingProfile(Profile.DockingProfile dp)
        {
            DockCaptureLateral = dp.GuideHalfWidth.Value;
            DockCaptureYaw = dp.GuideAcceptanceHalfAngle.Value;
        }
    }
}
