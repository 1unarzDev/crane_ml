using System;
using Crane.Fleet.Docking;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    public class ContactTests
    {
        [Fact]
        public void Magnet_force_saturates_at_hold_and_follows_the_exponential_law()
        {
            var m = Fx.Model(new DockingAssumptions());
            Assert.Equal(m.MagnetHoldForce, m.MagnetForce(0));
            Assert.Equal(m.MagnetHoldForce, m.MagnetForce(-0.001));
            Assert.Equal(m.MagnetHoldForce * m.Assume.MagnetEdgeForceFraction, m.MagnetForce(m.MagnetRange), 9);
            Assert.Equal(0, m.MagnetForce(m.MagnetRange * 1.01));
            double prev = double.MaxValue;
            for (double g = 0; g <= m.MagnetRange; g += m.MagnetRange / 20) { double f = m.MagnetForce(g); Assert.True(f <= prev); prev = f; }
        }

        [Fact]
        public void Magnet_lateral_force_restores_and_is_capped_by_normal_force()
        {
            var m = Fx.Model(new DockingAssumptions());
            Assert.True(m.MagnetLateralForce(0, 0.005) < 0);
            Assert.True(m.MagnetLateralForce(0, -0.005) > 0);
            Assert.Equal(-m.MagnetForce(0), m.MagnetLateralForce(0, 1.0), 9);
            Assert.Equal(0, m.MagnetLateralForce(0, 0), 12);
        }

        [Fact]
        public void Envelope_matches_guide_plus_magnet_reach_minus_yaw_loss()
        {
            var m = Fx.Model();
            double ell = m.Assume.GuideFaceHalfLength;
            Assert.True(m.InsideEnvelope(m.GuideHalfWidth + m.MagnetRange - 1e-6, 0, 0, 0));
            Assert.False(m.InsideEnvelope(m.GuideHalfWidth + m.MagnetRange + 1e-6, 0, 0, 0));
            double yaw = 0.05;
            Assert.Equal(m.GuideHalfWidth + m.MagnetRange - ell * Math.Sin(yaw), m.LateralAcceptance(yaw), 12);
            Assert.False(m.InsideEnvelope(0, m.AcceptanceHalfAngle + 1e-4, 0, 0));
            Assert.False(m.InsideEnvelope(0, 0.08, 0, 0.04)); // yaw and pitch combine
        }

        [Fact]
        public void Funnel_reduces_error_as_gap_closes_only_inside_the_envelope()
        {
            var m = Fx.Model(new DockingAssumptions());
            double band = 0.0005, e = 0.008;
            double far = m.FunnelError(e, band, m.Assume.FunnelDepth, true);
            double mid = m.FunnelError(e, band, m.Assume.FunnelDepth / 2, true);
            double near = m.FunnelError(e, band, 0, true);
            Assert.Equal(e, far, 12);
            Assert.True(far > mid && mid > near);
            Assert.Equal(band, near, 12);
            Assert.Equal(-band, m.FunnelError(-e, band, 0, true), 12);
            Assert.Equal(e, m.FunnelError(e, band, 0, false), 12);         // jammed: no help
            Assert.Equal(0.0002, m.FunnelError(0.0002, band, 0, true), 12); // already inside the dead-band
        }

        [Fact]
        public void Each_hook_sees_a_different_residual_under_yaw()
        {
            var m = Fx.Model();
            var r = new Residual(0.001, 0.025, 0);
            double arm = m.HookHalfSpan * Math.Sin(0.025);
            var a = m.HookResidual(r, 0); var b = m.HookResidual(r, 1);
            Assert.Equal(0.001 + arm, a.LateralAtHook, 12);
            Assert.Equal(0.001 - arm, b.LateralAtHook, 12);
            Assert.False(a.WithinTolerance);   // 2.5 mm > 2 mm
            Assert.True(b.WithinTolerance);    // ~0
        }

        [Fact]
        public void Seat_decision_by_geometry_window()
        {
            var a = new DockingAssumptions();
            var m = Fx.Model(a);
            var good = m.HookResidual(new Residual(0.0005, 0, 0), 0);
            var window = m.HookResidual(new Residual(m.TolPosition * 2, 0, 0), 0);
            var far = m.HookResidual(new Residual(m.TolPosition * 10, 0, 0), 0);
            Assert.Equal(HookSeat.Seated, m.Seat(good, 0.99));
            Assert.Equal(HookSeat.ClosedOnNothing, m.Seat(good, 0.0));          // catch misses
            Assert.Equal(HookSeat.ClosedOnNothing, m.Seat(window, 0.0));
            Assert.Equal(HookSeat.JammedShort, m.Seat(window, 0.99));
            Assert.Equal(HookSeat.JammedShort, m.Seat(far, 0.0));
        }

        [Fact]
        public void Yaw_tolerance_gates_seating_independently_of_lateral()
        {
            var m = Fx.Model();
            Assert.False(m.HookResidual(new Residual(0, m.TolYaw * 1.2, 0), 0).WithinTolerance);
            Assert.True(m.HookResidual(new Residual(0, 0, m.TolPosition * 0.99), 0).WithinTolerance);
            Assert.False(m.HookResidual(new Residual(0, 0, m.TolPosition * 1.01), 1).WithinTolerance); // height mismatch
        }

        [Fact]
        public void Static_split_hand_computed_symmetric()
        {
            var m = Fx.Model();
            var l = m.StaticSplit(new LoadCase(40, 20, 0), true, true);
            Assert.Equal(Math.Sqrt(20 * 20 + 10 * 10), l.A, 9);
            Assert.Equal(l.A, l.B, 12);
            Assert.Equal(l.A / 60.0, l.RatioA, 9);
            Assert.False(l.AnyOverload);
        }

        [Fact]
        public void Static_split_with_moment_couple()
        {
            var m = Fx.Model();
            // spacing 0.12 m: M = 1.2 N m => couple 10 N. T=40: hook A 30, hook B 10; no shear.
            var l = m.StaticSplit(new LoadCase(40, 0, 1.2), true, true);
            Assert.Equal(30, l.A, 9); Assert.Equal(10, l.B, 9);
            // bigger moment: hook B would go into compression => carried by the face, hook load 0
            var l2 = m.StaticSplit(new LoadCase(10, 0, 1.2), true, true);
            Assert.Equal(15, l2.A, 9); Assert.Equal(0, l2.B, 9);
        }

        [Fact]
        public void Static_split_overload_against_rated_load()
        {
            var m = Fx.Model();
            var l = m.StaticSplit(new LoadCase(130, 0, 0), true, true); // 65 N per hook vs 60 N
            Assert.True(l.OverloadA && l.OverloadB);
            Assert.Equal(65.0 / 60.0, l.RatioA, 9);
        }

        [Fact]
        public void One_hook_seated_takes_all_shear_and_magnets_carry_the_other_side()
        {
            var m = Fx.Model();
            var l = m.StaticSplit(new LoadCase(30, 10, 0), true, false);
            Assert.Equal(Math.Sqrt(15 * 15 + 10 * 10), l.A, 9);
            Assert.Equal(0, l.B);
            Assert.Equal(15, l.MagnetSideTension, 9);
            Assert.True(l.MagnetOverload);       // 15 N > hold/2 = 12.5 N
            Assert.False(m.StaticSplit(new LoadCase(20, 0, 0), true, false).MagnetOverload);
        }

        [Fact]
        public void No_hooks_only_friction_resists_shear()
        {
            var m = Fx.Model();
            double cap = m.Assume.FaceFriction * m.MagnetHoldForce; // 7.5 N
            Assert.False(m.StaticSplit(new LoadCase(0, cap * 0.9, 0), false, false).MagnetOverload);
            Assert.True(m.StaticSplit(new LoadCase(0, cap * 1.1, 0), false, false).MagnetOverload);
        }

        [Fact]
        public void Compliant_split_is_even_for_even_seating()
        {
            var m = Fx.Model();
            var l = m.CompliantSplit(new LoadCase(80, 0, 0), new Residual(0.001, 0, 0));
            Assert.Equal(40, l.A, 9); Assert.Equal(40, l.B, 9);
        }

        [Fact]
        public void Compliant_split_hand_computed_for_unequal_seating()
        {
            var m = Fx.Model();
            // k = rated / (1.0 * compliance) = 60 / 0.0005 = 120000 N/m
            Assert.Equal(120000, m.HookStiffness, 6);
            // residual at hooks: lateral 1.0 mm with yaw lever +0.5 mm => hook A 1.5 mm, hook B 0.5 mm; slack difference 1 mm => k*1mm = 120 N
            double yaw = Math.Asin(0.0005 / m.HookHalfSpan);
            var r = new Residual(0.001, yaw, 0);
            Assert.Equal(0.0015, m.HookResidual(r, 0).LateralAtHook, 12);
            // below 120 N the better-seated hook B carries everything
            var l = m.CompliantSplit(new LoadCase(100, 0, 0), r);
            Assert.Equal(0, l.A, 9); Assert.Equal(100, l.B, 9);
            // above: F_B = P/2 + 60, F_A = P/2 - 60.  P = 240 => B 180, A 60
            var l2 = m.CompliantSplit(new LoadCase(240, 0, 0), r);
            Assert.Equal(60, l2.A, 9); Assert.Equal(180, l2.B, 9);
            Assert.True(l2.OverloadB && !l2.OverloadA);
        }

        [Fact]
        public void Compliant_split_conserves_load_and_is_not_worse_than_one_hook_taking_all()
        {
            var m = Fx.Model();
            var r = new Residual(0.0008, 0.01, 0);
            for (double p = 10; p < 400; p += 37)
            {
                var l = m.CompliantSplit(new LoadCase(p, 0, 0), r);
                Assert.Equal(p, l.A + l.B, 9);
            }
        }

        [Fact]
        public void Compliant_split_applies_moment_couple_on_top()
        {
            var m = Fx.Model();
            var l = m.CompliantSplit(new LoadCase(40, 0, 1.2), new Residual(0.001, 0, 0));
            Assert.Equal(30, l.A, 9); Assert.Equal(10, l.B, 9);
        }

        [Fact]
        public void Settle_clips_each_component_to_its_dead_band()
        {
            var m = Fx.Model();
            var r = m.Settle(0.005, -0.02, 0.0004, new Bands(0.0005, 0.005, 0.001));
            Assert.Equal(0.0005, r.Lateral, 12); Assert.Equal(-0.005, r.Yaw, 12); Assert.Equal(0.0004, r.Height, 12);
        }
    }
}
