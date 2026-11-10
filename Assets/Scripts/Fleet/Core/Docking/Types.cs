using System;
using System.Collections.Generic;
using System.Text;

namespace Crane.Fleet.Docking
{
    /// <summary>States of one coupling. Names map 1:1 to the profile's docking.states and fault_states (FOO_BAR_BAZ -> FooBarBaz).</summary>
    public enum ConnectorState
    {
        // nominal
        Free, Approach, MagnetCaptured, HookAEngaged, HookBEngaged, LatchedVerified, LoadValidated, ReleaseRequested, Released,
        // faults
        Partial, FalseEngagement, Obstructed, VerificationLost, StuckHook,
    }

    public static class ConnectorStateNames
    {
        /// <summary>"HOOK_A_ENGAGED" -> "HookAEngaged"; null if the input has characters other than A-Z and '_'.</summary>
        public static string FromProfileName(string s)
        {
            if (string.IsNullOrEmpty(s)) return null;
            var sb = new StringBuilder();
            foreach (string part in s.Split('_'))
            {
                if (part.Length == 0) return null;
                foreach (char c in part) if (c < 'A' || c > 'Z') return null;
                sb.Append(part[0]).Append(part.Substring(1).ToLowerInvariant());
            }
            return sb.ToString();
        }

        /// <summary>"HookAEngaged" -> "HOOK_A_ENGAGED".</summary>
        public static string ToProfileName(ConnectorState st)
        {
            string n = st.ToString();
            var sb = new StringBuilder();
            for (int i = 0; i < n.Length; i++)
            {
                char c = n[i];
                if (i > 0 && char.IsUpper(c) && (char.IsLower(n[i - 1]) || (i + 1 < n.Length && char.IsLower(n[i + 1]) && char.IsUpper(n[i - 1]))))
                    sb.Append('_');
                sb.Append(char.ToUpperInvariant(c));
            }
            return sb.ToString();
        }

        public static bool IsFault(ConnectorState s) =>
            s == ConnectorState.Partial || s == ConnectorState.FalseEngagement || s == ConnectorState.Obstructed ||
            s == ConnectorState.VerificationLost || s == ConnectorState.StuckHook;

        /// <summary>True when the coupling may be carrying load as far as the module graph is concerned.</summary>
        public static bool IsVerified(ConnectorState s) => s == ConnectorState.LatchedVerified || s == ConnectorState.LoadValidated;
    }

    /// <summary>Events that drive the transition table (a transition is looked up by current state + trigger).</summary>
    public enum Trigger
    {
        CmdApproach, Captured, Jam, ObstructionDetected,
        HookASensed, HookBSensed, HookTimeout, HookSignalLost,
        VerifyPassed, FalseEngagementDetected,
        LoadTestPassed, LoadTestSlip, VerificationLostDetected,
        CmdRelease, ReleaseComplete, ReleaseTimeout,
    }

    public enum DockingCommand { None, Approach, EngageHooks, Verify, TestLoad, Release }

    /// <summary>Per-hook actuator condition reported by the caller (the actuator model is outside this library).</summary>
    public enum HookDrive
    {
        Normal,
        /// <summary>Actuator cannot move (jam / no power). Prevents closing and, more importantly, opening (STUCK_HOOK).</summary>
        Stalled,
        /// <summary>Hook is knocked/driven open (vibration, fault): its signal drops.</summary>
        ForcedOpen,
    }

    public enum RequestedAction { None, BackOffAndReapproach, Abandon, ReverifyOrReleaseWithSupport, StopApproach, RetryRelease }

    public enum EventKind
    {
        Transition, IllegalTransition, GuardRejected, Capture, Jam, HookSensed, FalsePositiveSensor, HookClosedOnNothing,
        HookOverload, VerifyFailed, RetriesExhausted, ReleaseRefused, UnsafeRelease, Released,
    }

    /// <summary>Typed, timestamped output of the machine and the module graph.</summary>
    public sealed class DockingEvent
    {
        public double Time { get; }
        public EventKind Kind { get; }
        public ConnectorState From { get; }
        public ConnectorState To { get; }
        public Trigger? Trigger { get; }
        public string Detail { get; }
        public string CouplingId { get; }
        public DockingEvent(double t, EventKind k, ConnectorState from, ConnectorState to, Trigger? trig, string detail, string couplingId = null)
        { Time = t; Kind = k; From = from; To = to; Trigger = trig; Detail = detail; CouplingId = couplingId; }
        public override string ToString() => $"[{Time:0.000}s] {Kind} {From}->{To} {Trigger} {Detail}";
    }

    /// <summary>Relative pose of face B w.r.t. face A (docking frame). Lateral/Yaw/Height are errors from perfect mating.</summary>
    public struct FacePose
    {
        public double Lateral;      // m, + = left of dock axis
        public double Gap;          // m, longitudinal distance between faces (0 = contact)
        public double Yaw;          // rad
        public double Height;       // m, total vertical mismatch (includes pitch*arm if the caller has it; see Pitch)
        public double Pitch;        // rad, enters the guide acceptance angle only
        public FacePose(double lateral, double gap, double yaw, double height = 0, double pitch = 0)
        { Lateral = lateral; Gap = gap; Yaw = yaw; Height = height; Pitch = pitch; }
    }

    /// <summary>Load on the coupling in the face frame. Tension: along the normal (+ pulls apart). Shear: in-plane (lateral or vertical).
    /// Moment: about the vertical axis through the face centre; + raises the load on hook A (the +lateral hook).</summary>
    public struct LoadCase
    {
        public double Tension, Shear, Moment;
        public LoadCase(double tension, double shear, double moment) { Tension = tension; Shear = shear; Moment = moment; }
        public bool IsZero => Tension == 0 && Shear == 0 && Moment == 0;
    }

    public sealed class ConnectorInputs
    {
        public FacePose Pose;
        /// <summary>Gap closing speed, m/s (+ = closing).</summary>
        public double ClosingSpeed;
        public double LateralSpeed;
        public DockingCommand Command = DockingCommand.None;
        public HookDrive HookA = HookDrive.Normal, HookB = HookDrive.Normal;
        public bool Obstruction;
        /// <summary>From ModuleGraph.CanRelease. Defaults to false so a forgotten wiring refuses to release verified couplings.</summary>
        public bool ReleaseAuthorized;
        public LoadCase AppliedLoad;
        /// <summary>External shift of the mated pose after capture (slip, creep); added to the settled residual.</summary>
        public FacePose Disturbance;
    }

    public sealed class StepResult
    {
        public ConnectorState State;
        public List<DockingEvent> Events = new List<DockingEvent>();
        public RequestedAction Action;
        public bool SensedA, SensedB;
        public double HookPosA, HookPosB;
        public int Attempt;
    }

    /// <summary>Shared outcome vocabulary for the fast Python models and the simulator. See OutcomeClassifier for exact definitions.</summary>
    public enum DockingOutcome
    {
        /// <summary>Never magnet-captured (guide jam, outside envelope, bounce) and not obstructed.</summary>
        Missed = 0,
        /// <summary>Faces mated by magnets, but no hook is mechanically seated (or nothing was demonstrated).</summary>
        MagnetCapturedOnly = 1,
        /// <summary>Exactly one hook mechanically seated.</summary>
        OneHook = 2,
        /// <summary>Both hooks seated AND verification (sensors + geometry) passed; no load test yet.</summary>
        BothHooksVerified = 3,
        /// <summary>Verified and the test pull / load-consistency check passed.</summary>
        LoadValidated = 4,
        /// <summary>At least one hook sensor said engaged (or verify claimed success) while that hook was not mechanically seated.</summary>
        FalseEngagement = 5,
        /// <summary>Something between the faces blocked capture; magnets never pulled in.</summary>
        Obstructed = 6,
    }
}
