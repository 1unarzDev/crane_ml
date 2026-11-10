using System;

namespace Crane.Fleet.Docking
{
    /// <summary>
    /// The shared outcome vocabulary (see <see cref="DockingOutcome"/>). Two entry points:
    ///  * <see cref="FromGeometry"/> for fast models that know only terminal geometry (assumes a perfect verification step);
    ///  * <see cref="Classify"/> for a finished machine attempt, using ground truth and what was actually sensed/verified.
    /// Precedence (first match wins): Obstructed, Missed, FalseEngagement, LoadValidated, BothHooksVerified, OneHook, MagnetCapturedOnly.
    /// </summary>
    public static class OutcomeClassifier
    {
        public static DockingOutcome Classify(AttemptSummary s)
        {
            if (s.EverObstructed && !s.EverCaptured) return DockingOutcome.Obstructed;
            if (!s.EverCaptured) return DockingOutcome.Missed;
            // a hook that reads engaged without being seated, or a declared false engagement, outranks everything else captured
            bool falseA = s.SensedA && !s.SeatedA, falseB = s.SensedB && !s.SeatedB;
            if (falseA || falseB || s.FalseEngagementDeclared) return DockingOutcome.FalseEngagement;
            int seated = (s.SeatedA ? 1 : 0) + (s.SeatedB ? 1 : 0);
            if (seated == 2 && s.Verified && s.LoadValidated) return DockingOutcome.LoadValidated;
            if (seated == 2 && s.Verified) return DockingOutcome.BothHooksVerified;
            if (seated == 1) return DockingOutcome.OneHook;
            return DockingOutcome.MagnetCapturedOnly;
        }

        /// <summary>Fast-model mapping: captured + number of mechanically seated hooks (0..2) [+ obstruction / load-validated flags].</summary>
        public static DockingOutcome FromGeometry(bool captured, int seatedHooks, bool obstructed = false, bool loadValidated = false)
        {
            if (obstructed) return DockingOutcome.Obstructed;
            if (!captured) return DockingOutcome.Missed;
            if (seatedHooks >= 2) return loadValidated ? DockingOutcome.LoadValidated : DockingOutcome.BothHooksVerified;
            if (seatedHooks == 1) return DockingOutcome.OneHook;
            return DockingOutcome.MagnetCapturedOnly;
        }

        /// <summary>Integer code of docking_mc/model.py classify(): 0 missed, 1 captured only, 2 one hook, 3 both hooks.</summary>
        public static DockingOutcome FromPythonCode(int code)
        {
            switch (code)
            {
                case 0: return DockingOutcome.Missed;
                case 1: return DockingOutcome.MagnetCapturedOnly;
                case 2: return DockingOutcome.OneHook;
                case 3: return DockingOutcome.BothHooksVerified;
                default: throw new ArgumentOutOfRangeException(nameof(code));
            }
        }
    }
}
