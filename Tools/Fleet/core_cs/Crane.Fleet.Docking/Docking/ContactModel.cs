using System;
using Crane.Fleet.Profile;

namespace Crane.Fleet.Docking
{
    /// <summary>Settled residual error after guide/magnet centring (lateral m, yaw rad, height m).</summary>
    public struct Residual
    {
        public double Lateral, Yaw, Height;
        public Residual(double lateral, double yaw, double height) { Lateral = lateral; Yaw = yaw; Height = height; }
    }

    /// <summary>Per-sample dead-band magnitudes (absolute, >= 0): largest error the faces keep after settling.</summary>
    public struct Bands
    {
        public double Lateral, Yaw, Height;
        public Bands(double lateral, double yaw, double height) { Lateral = lateral; Yaw = yaw; Height = height; }
    }

    public enum HookSeat { Seated, ClosedOnNothing, JammedShort }

    public struct HookGeometry
    {
        public double LateralAtHook, Height, Yaw;
        public bool WithinTolerance;
        public double Ratio;   // worst-of residual / tolerance, for diagnostics
    }

    public struct HookLoads
    {
        public double A, B;               // N, magnitude of hook load
        public double RatioA, RatioB;     // load / rated
        public bool OverloadA, OverloadB;
        public double MagnetSideTension;  // N carried by magnets on an unseated side (sum)
        public bool MagnetOverload;       // magnet/face-only side exceeded its capacity
        public bool AnyOverload => OverloadA || OverloadB || MagnetOverload;
    }

    /// <summary>
    /// SIMPLIFIED, UNCALIBRATED contact and force model for one face pair. Every number comes from the profile
    /// (guide, magnet, hooks, compliance) or from <see cref="DockingAssumptions"/>. Pure functions of its inputs.
    /// Two-hooks-per-face (v0): hook A sits at +hookHalfSpan on the lateral axis (left), hook B at -hookHalfSpan.
    /// Reciprocal (v1): HOOK_A is the initiating rover's own hook at -hookHalfSpan, HOOK_B the partner's hook at +hookHalfSpan
    /// (Rz(pi) maps each face's -y hook onto the partner's +y pin); see <see cref="HookASide"/>.
    /// </summary>
    public sealed class ContactModel
    {
        public double GuideHalfWidth { get; }
        public double AcceptanceHalfAngle { get; }
        public double MagnetRange { get; }
        public double MagnetHoldForce { get; }
        public double HookHalfSpan { get; }
        public double HookSpacing => 2 * HookHalfSpan;
        /// <summary>Lateral side of HOOK_A: +1 (v0) or -1 (reciprocal v1). HOOK_B is the opposite side.</summary>
        public double HookASide { get; }
        public bool Reciprocal => HookASide < 0;
        public double TolPosition { get; }
        public double TolYaw { get; }
        public double HookRatedLoad { get; }
        public double Compliance { get; }
        public double HookActuationTime { get; }
        public DockingAssumptions Assume { get; }
        /// <summary>N/m: hook stiffness = rated load / (deflection factor * profile compliance).</summary>
        public double HookStiffness { get; }

        public ContactModel(DockingProfile p, DockingAssumptions assume)
            : this(p.GuideHalfWidth, p.GuideAcceptanceHalfAngle, p.MagnetPullInRange, p.MagnetHoldForce, p.HookHalfSpan,
                   p.HookPositionTolerance, p.HookYawTolerance, p.HookRatedLoad, p.ConnectorComplianceLateral,
                   p.HookActuationTime, assume, p.HookASide) { }

        public ContactModel(double guideHalfWidth, double acceptHalfAngle, double magnetRange, double holdForce, double hookHalfSpan,
                            double tolPos, double tolYaw, double ratedLoad, double compliance, double actuationTime, DockingAssumptions assume,
                            double hookASide = +1.0)
        {
            HookASide = hookASide < 0 ? -1.0 : +1.0;
            GuideHalfWidth = guideHalfWidth; AcceptanceHalfAngle = acceptHalfAngle; MagnetRange = magnetRange; MagnetHoldForce = holdForce;
            HookHalfSpan = hookHalfSpan; TolPosition = tolPos; TolYaw = tolYaw; HookRatedLoad = ratedLoad; Compliance = compliance;
            HookActuationTime = actuationTime; Assume = assume;
            HookStiffness = ratedLoad / (assume.HookDeflectionAtRatedFactor * compliance);
        }

        // ------------------------------------------------------------------ guide funnel
        /// <summary>
        /// Lateral acceptance of the guide at yaw: g + r - ell*|sin(yaw)|. Same form as docking_mc/model.py classify().
        /// </summary>
        public double LateralAcceptance(double yaw) => GuideHalfWidth + MagnetRange - Assume.GuideFaceHalfLength * Math.Abs(Math.Sin(yaw));

        /// <summary>True if the terminal pose is inside the guide acceptance envelope (else the faces jam on the lips).
        /// Lateral and height combine as a norm, yaw and pitch likewise (py: y_eff = hypot(y,h), p_eff = hypot(psi,pitch)).</summary>
        public bool InsideEnvelope(double lateral, double yaw, double height, double pitch)
            => Math.Sqrt(lateral * lateral + height * height) <= LateralAcceptance(yaw)
               && Math.Sqrt(yaw * yaw + pitch * pitch) <= AcceptanceHalfAngle;

        /// <summary>
        /// Error as seen at remaining gap, once inside the envelope: funnels linearly relax |error| to the dead-band as the gap goes
        /// from FunnelDepth to 0. Outside the envelope the funnel does not act (returns the error unchanged: the faces jam).
        /// </summary>
        public double FunnelError(double error, double band, double gap, bool insideEnvelope)
        {
            if (!insideEnvelope) return error;
            double a = Math.Abs(error);
            if (a <= band) return error;
            double f = Math.Max(0, Math.Min(1, gap / Assume.FunnelDepth));
            return Math.Sign(error) * (band + (a - band) * f);
        }

        /// <summary>Residual at full closure (gap 0): each component limited to its dead-band (stiction after compliant centring).</summary>
        public Residual Settle(double lateral, double yaw, double height, Bands b)
            => new Residual(Clip(lateral, b.Lateral), Clip(yaw, b.Yaw), Clip(height, b.Height));

        private static double Clip(double v, double band) => Math.Sign(v) * Math.Min(Math.Abs(v), band);

        // ------------------------------------------------------------------ magnets
        /// <summary>Exponential pull-in law saturating at hold force: F = hold * exp(-gap/lambda), lambda from the edge fraction at the pull-in range. Zero beyond range.</summary>
        public double MagnetForce(double gap)
        {
            if (gap <= 0) return MagnetHoldForce;
            if (gap > MagnetRange) return 0;
            double lambda = MagnetRange / Math.Log(1.0 / Assume.MagnetEdgeForceFraction);
            return MagnetHoldForce * Math.Exp(-gap / lambda);
        }

        /// <summary>Lateral restoring force (N, sign opposes offset): normal force * offset / scale, limited to the friction-free cap of normal force.</summary>
        public double MagnetLateralForce(double gap, double lateral)
        {
            double fn = MagnetForce(gap);
            double f = fn * lateral / Assume.MagnetLateralScale;
            return -Math.Sign(f) * Math.Min(Math.Abs(f), fn);
        }

        // ------------------------------------------------------------------ hook seating
        /// <summary>v0 hook index sign: +1 for hook A (left), -1 for hook B (right). Use <see cref="SideOf"/> for the model's actual mode.</summary>
        public static double Side(int hook) => hook == 0 ? +1.0 : -1.0;

        /// <summary>Lateral side (+1 left, -1 right in the docking frame) of hook index 0 (A) or 1 (B) in this model's face mode.</summary>
        public double SideOf(int hook) => hook == 0 ? HookASide : -HookASide;

        /// <summary>Residual seen by one hook: lateral offset at the hook = lateral + side*halfSpan*sin(yaw). Seats iff position (lateral+height as norm) and yaw are within tolerance.</summary>
        public HookGeometry HookResidual(Residual r, int hook)
        {
            double lat = r.Lateral + SideOf(hook) * HookHalfSpan * Math.Sin(r.Yaw);
            double pos = Math.Sqrt(lat * lat + r.Height * r.Height);
            double ratio = Math.Max(pos / TolPosition, Math.Abs(r.Yaw) / TolYaw);
            return new HookGeometry { LateralAtHook = lat, Height = r.Height, Yaw = r.Yaw, Ratio = ratio, WithinTolerance = ratio <= 1.0 };
        }

        /// <summary>
        /// What a hook does when closed on this geometry. Within tolerance: seats (unless the catch randomly misses, uMiss &lt; HookMissCatchProbability).
        /// Within ClosedOnNothingWindowFactor*tolerance: with probability ClosedOnNothingProbability swings fully shut on nothing, otherwise jams short.
        /// Beyond: jams short.
        /// </summary>
        public HookSeat Seat(HookGeometry g, double uMiss)
        {
            if (g.WithinTolerance) return uMiss < Assume.HookMissCatchProbability ? HookSeat.ClosedOnNothing : HookSeat.Seated;
            if (g.Ratio <= Assume.ClosedOnNothingWindowFactor) return uMiss < Assume.ClosedOnNothingProbability ? HookSeat.ClosedOnNothing : HookSeat.JammedShort;
            return HookSeat.JammedShort;
        }

        // ------------------------------------------------------------------ load sharing
        private static double Mag(double a, double b) => Math.Sqrt(a * a + b * b);

        /// <summary>
        /// Statically determinate split. Tension T (+ pulls apart), shear V, moment M about the vertical axis (+ loads the hook at +lateral: A in v0, B in reciprocal mode).
        /// Tension on hook i = T/2 +- M/spacing (negative = bearing on the face, hook carries 0); shear V/2 each.
        /// Unseated side: carried by magnets only: tension capacity hold/2, shear capacity friction*hold (shared with any seated hook is NOT assumed).
        /// With one hook seated it takes all the shear and its tension share per the same couple formula.
        /// </summary>
        public HookLoads StaticSplit(LoadCase l, bool seatedA, bool seatedB)
        {
            double d = HookSpacing;
            double mA = HookASide * l.Moment / d;
            double tA = l.Tension / 2 + mA, tB = l.Tension / 2 - mA;
            return Assemble(l, seatedA, seatedB, tA, tB, seatedA && seatedB ? l.Shear / 2 : l.Shear, seatedA && seatedB ? l.Shear / 2 : l.Shear);
        }

        /// <summary>
        /// Compliant split when both hooks are engaged with unequal seating. Hook i has slack s_i = |lateral residual at hook i|
        /// (the displacement before it bears) and stiffness k. Under a symmetric load P (tension or shear) with s1 &lt;= s2:
        /// P &lt;= k(s2-s1): hook 1 takes all; else F1 = P/2 + k(s2-s1)/2, F2 = P/2 - k(s2-s1)/2. The moment is added as the rigid couple.
        /// </summary>
        public HookLoads CompliantSplit(LoadCase l, Residual r)
        {
            double sA = Math.Abs(HookResidual(r, 0).LateralAtHook), sB = Math.Abs(HookResidual(r, 1).LateralAtHook);
            Share(Math.Max(l.Tension, 0), sA, sB, out double tA, out double tB);
            Share(Math.Abs(l.Shear), sA, sB, out double vA, out double vB);
            double c = HookASide * l.Moment / HookSpacing;
            // negative tension (compression) is borne by the face, not the hooks
            return Assemble(l, true, true, tA + c, tB - c, vA, vB);
        }

        private void Share(double p, double sA, double sB, out double fA, out double fB)
        {
            double lo = Math.Min(sA, sB), hi = Math.Max(sA, sB);
            double diff = HookStiffness * (hi - lo);
            double fLo, fHi;
            if (p <= diff) { fLo = p; fHi = 0; }
            else { fLo = p / 2 + diff / 2; fHi = p / 2 - diff / 2; }
            if (sA <= sB) { fA = fLo; fB = fHi; } else { fA = fHi; fB = fLo; }
        }

        private HookLoads Assemble(LoadCase l, bool seatedA, bool seatedB, double tA, double tB, double vA, double vB)
        {
            var h = new HookLoads();
            double magCapT = MagnetHoldForce / 2;
            double magT = 0;
            if (seatedA) { h.A = Mag(Math.Max(tA, 0), vA); }
            else { magT += Math.Max(tA, 0); }
            if (seatedB) { h.B = Mag(Math.Max(tB, 0), vB); }
            else { magT += Math.Max(tB, 0); }
            h.MagnetSideTension = magT;
            // an unseated side only has the magnets (and friction for shear)
            bool magOver = (!seatedA && Math.Max(tA, 0) > magCapT) || (!seatedB && Math.Max(tB, 0) > magCapT);
            if (!seatedA && !seatedB && Math.Abs(l.Shear) > Assume.FaceFriction * MagnetHoldForce) magOver = true;
            h.MagnetOverload = magOver;
            h.RatioA = h.A / HookRatedLoad; h.RatioB = h.B / HookRatedLoad;
            h.OverloadA = seatedA && h.RatioA > 1.0; h.OverloadB = seatedB && h.RatioB > 1.0;
            return h;
        }
    }
}
