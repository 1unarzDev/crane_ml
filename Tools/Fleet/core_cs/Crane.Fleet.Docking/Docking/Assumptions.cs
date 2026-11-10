using System;
using System.Collections.Generic;

namespace Crane.Fleet.Docking
{
    /// <summary>
    /// EVERY model constant that is NOT in the fleet profile lives here, and nowhere else. All are ASSUMPTIONS
    /// (uncalibrated, no hardware exists). Defaults marked "py" are copied from Tools/Fleet/docking_mc/model.py
    /// (also assumptions there) so the two models start from the same numbers; <see cref="Registry"/> says what
    /// measurement would replace each one. Profile-derived quantities (hook spacing, tolerances, rated load,
    /// compliance, magnet range) are NOT repeated here; they are read from DockingProfile.
    /// </summary>
    public sealed class DockingAssumptions
    {
        // ---- guide / contact geometry
        /// <summary>Half length of the guide face along the lateral axis; yaw eats lateral acceptance by ell*|sin(yaw)|. (py ell)</summary>
        public double GuideFaceHalfLength = 0.03;
        /// <summary>Gap over which the funnel relaxes the error from full to residual.</summary>
        public double FunnelDepth = 0.02;
        /// <summary>Multiplier on profile connector_compliance_lateral for the stiction dead-band sd (py: band_y = compliance, so 1.0).</summary>
        public double StictionBandScale = 1.0;
        /// <summary>Height dead-band sd (m). (py band_h)</summary>
        public double HeightBandSd = 0.0005;
        /// <summary>Yaw dead-band sd (rad). (py band_psi = 0.3 deg)</summary>
        public double YawBandSd = 0.3 * Math.PI / 180.0;

        // ---- magnets
        /// <summary>Magnet force at the pull-in range edge as a fraction of hold force (exponential law).</summary>
        public double MagnetEdgeForceFraction = 0.05;
        /// <summary>Length scale (m) of lateral magnetic restoring: F_lat = F_normal * y / this.</summary>
        public double MagnetLateralScale = 0.02;

        // ---- hooks
        /// <summary>Hook deflection at rated load as a multiple of profile connector_compliance_lateral (sets hook stiffness).</summary>
        public double HookDeflectionAtRatedFactor = 1.0;
        /// <summary>Hook mechanically catches (seats) only within tol; beyond tol*this it jams short; in between it may close on nothing.</summary>
        public double ClosedOnNothingWindowFactor = 3.0;
        /// <summary>Probability a misaligned hook inside the window swings fully closed on nothing instead of stalling.</summary>
        public double ClosedOnNothingProbability = 0.5;
        /// <summary>Probability the closed-position sensor reads 'engaged' while the hook is not at its closed stop.</summary>
        public double SensorFalsePositiveProbability = 0.02;
        /// <summary>Probability a hook with GOOD geometry still closes on nothing (dirt, broken catch). Invisible to the geometry check; only the load check finds it.</summary>
        public double HookMissCatchProbability = 0.02;
        /// <summary>Closing speed above which the magnets cannot capture and the faces bounce (m/s). Python MC shows no speed effect up to 0.10.</summary>
        public double MaxClosingSpeed = 0.15;
        /// <summary>Time with only one hook (or none) sensed after the first one is sensed / after engage command, as multiple of hook actuation time.</summary>
        public double HookTimeoutFactor = 2.0;
        /// <summary>Time for release hooks to open before STUCK_HOOK, multiple of actuation time.</summary>
        public double ReleaseTimeoutFactor = 1.5;

        // ---- verification / load check
        public double PoseMeasureSdLateral = 0.0001;
        public double PoseMeasureSdYaw = 0.1 * Math.PI / 180.0;
        public double PoseMeasureSdHeight = 0.0001;
        /// <summary>Test-pull tension applied by LOAD check when the input supplies none (N).</summary>
        public double TestPullTension = 20.0;

        // ---- retries / obstruction
        public int MaxRetries = 3;
        /// <summary>Gap the rover must back off to before a re-approach (m).</summary>
        public double BackoffGap = 0.05;
        /// <summary>Obstruction flag must persist this long to declare OBSTRUCTED (s).</summary>
        public double ObstructionDebounce = 0.3;

        // ---- reciprocal (v1) connection-level observation; ignored in two-hooks-per-face (v0) mode
        /// <summary>Latency (s) of the connection-level report that carries the partner's hook state (and the partner close command) between the two rovers. Each rover senses only its own hook.</summary>
        public double PartnerLinkLatency = 0.1;
        /// <summary>If no partner report arrives for this long (s) the partner hook is treated as NOT engaged (fail-safe), and release of a verified connection is refused.</summary>
        public double PartnerObservationMaxAge = 0.5;

        // ---- misc
        public double Gravity = 9.81;
        /// <summary>Friction coefficient between mated faces; capacity of a magnet-only (unseated) side against shear = mu * hold force.</summary>
        public double FaceFriction = 0.3;
        /// <summary>Position at which a jammed-short hook stops (0 open, 1 closed stop).</summary>
        public double JamPosition = 0.6;
        /// <summary>Load check passes if measured connection stiffness >= this fraction of the two-hook stiffness.</summary>
        public double LoadConsistencyMinFraction = 0.75;
        /// <summary>Latched coupling reports VERIFICATION_LOST when a measured hook residual exceeds tolerance times this.</summary>
        public double VerificationLossFactor = 1.5;
        /// <summary>Deck contact points are the body-footprint corners inset by this (m).</summary>
        public double DeckContactInset = 0.03;
        /// <summary>Required clearance of payload COM inside the support hull (m).</summary>
        public double SupportMargin = 0.02;

        public DockingAssumptions Clone() => (DockingAssumptions)MemberwiseClone();

        /// <summary>Zero-noise, perfect-centring variant used to compare against the Python MC and for hand-computed tests.</summary>
        public DockingAssumptions Deterministic()
        {
            var a = Clone();
            a.StictionBandScale = 1e6; a.HeightBandSd = 1e6; a.YawBandSd = 1e6; // dead-bands so wide that nothing is centred: residual == terminal error
            a.PoseMeasureSdLateral = 0; a.PoseMeasureSdYaw = 0; a.PoseMeasureSdHeight = 0;
            a.SensorFalsePositiveProbability = 0; a.ClosedOnNothingProbability = 0; a.HookMissCatchProbability = 0;
            return a;
        }

        public sealed class Entry
        {
            public string Name, Default, ReplacedBy;
            public Entry(string n, string d, string r) { Name = n; Default = d; ReplacedBy = r; }
        }

        /// <summary>What each assumption is and which measurement replaces it (also in README / handoff).</summary>
        public static IReadOnlyList<Entry> Registry => new[]
        {
            new Entry(nameof(GuideFaceHalfLength), "0.03 m", "CAD: guide face length along lateral axis"),
            new Entry(nameof(FunnelDepth), "0.02 m", "CAD: guide taper depth"),
            new Entry(nameof(StictionBandScale), "1.0", "Bench: repeatability of settled position (sd of lateral residual), compare with profile connector_compliance_lateral"),
            new Entry(nameof(HeightBandSd), "0.5 mm", "Bench: vertical settled-position repeatability on flat floor"),
            new Entry(nameof(YawBandSd), "0.3 deg", "Bench: settled yaw repeatability"),
            new Entry(nameof(MagnetEdgeForceFraction), "0.05", "Force-gap curve of the magnet pair on a load cell (force at pull-in range / force at contact)"),
            new Entry(nameof(MagnetLateralScale), "0.02 m", "Lateral force vs offset at contact on a load cell"),
            new Entry(nameof(HookDeflectionAtRatedFactor), "1.0", "Pull a closed hook to rated load, measure deflection (gives hook stiffness)"),
            new Entry(nameof(ClosedOnNothingWindowFactor), "3.0", "Hook CAD: catch geometry; bench: how far off-seat a hook still closes"),
            new Entry(nameof(ClosedOnNothingProbability), "0.5", "Bench: fraction of misaligned closes that reach the closed stop without catching"),
            new Entry(nameof(SensorFalsePositiveProbability), "0.02", "Bench: hook-sensor reads vs ground truth over >=500 cycles incl. dirt/vibration"),
            new Entry(nameof(HookMissCatchProbability), "0.02", "Bench: fraction of well-aligned closes that fail to catch, over >=500 cycles"),
            new Entry(nameof(MaxClosingSpeed), "0.15 m/s", "Impact test: speed at which magnets stop capturing"),
            new Entry(nameof(HookTimeoutFactor), "2.0", "Firmware choice after measuring hook actuation time spread"),
            new Entry(nameof(ReleaseTimeoutFactor), "1.5", "Measured actuation-time spread under load"),
            new Entry(nameof(PoseMeasureSdLateral), "0.1 mm", "Verification pose sensor (tag/ToF) repeatability at contact"),
            new Entry(nameof(PoseMeasureSdYaw), "0.1 deg", "Same"),
            new Entry(nameof(PoseMeasureSdHeight), "0.1 mm", "Same"),
            new Entry(nameof(TestPullTension), "20 N", "Choose from measured hook rated load and magnet hold force"),
            new Entry(nameof(MaxRetries), "3", "Operations policy from measured per-attempt success rate"),
            new Entry(nameof(BackoffGap), "0.05 m", "Rover stopping/positioning accuracy"),
            new Entry(nameof(ObstructionDebounce), "0.3 s", "Obstruction sensor latency/noise"),
            new Entry(nameof(FaceFriction), "0.3", "Friction pull test of mated faces (magnets on, hooks open)"),
            new Entry(nameof(JamPosition), "0.6", "Hook CAD: where a mis-seated hook hits the face"),
            new Entry(nameof(LoadConsistencyMinFraction), "0.75", "Repeatability of measured connection stiffness over >=50 good latches"),
            new Entry(nameof(VerificationLossFactor), "1.5", "Noise/hysteresis of the pose monitor in the latched state"),
            new Entry(nameof(PartnerLinkLatency), "0.1 s", "Measured rover-to-rover (or fleet-hub) message latency for the connection-level hook report; v1 reciprocal mode only"),
            new Entry(nameof(PartnerObservationMaxAge), "0.5 s", "Link dropout statistics and the safety policy on stale partner reports; v1 reciprocal mode only"),
            new Entry(nameof(Gravity), "9.81 m/s^2", "Constant"),
            new Entry(nameof(DeckContactInset), "0.03 m", "CAD: where payload actually bears on the deck"),
            new Entry(nameof(SupportMargin), "0.02 m", "Payload slide/tip tests on deck; safety policy"),
        };
    }
}
