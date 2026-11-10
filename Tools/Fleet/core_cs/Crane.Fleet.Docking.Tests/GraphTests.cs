using System;
using System.Linq;
using Crane.Fleet.Docking;
using Crane.Fleet.Graph;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    public class GraphTests
    {
        private static ModuleGraph Line3(ConnectorState s = ConnectorState.LatchedVerified) => ModuleGraph.Line(Fx.Profile(), new DockingAssumptions(), 3, s);

        [Fact]
        public void Line_places_rovers_by_mating_faces()
        {
            var g = Line3();
            Assert.Equal(0.42, g.Center("r1").X, 6); Assert.Equal(0, g.Center("r1").Y, 6);
            Assert.Equal(0.84, g.Center("r2").X, 6);
            Assert.Equal(0, g.Rovers["r2"].Pose.Yaw, 6);
        }

        [Fact]
        public void Side_by_side_and_branched_use_the_side_faces()
        {
            var sb = ModuleGraph.SideBySide(Fx.Profile(), new DockingAssumptions());
            Assert.Equal(0, sb.Center("r1").X, 6); Assert.Equal(0.4, sb.Center("r1").Y, 6);
            var br = ModuleGraph.Branched(Fx.Profile(), new DockingAssumptions());
            Assert.Equal(-0.42, br.Center("r2").X, 6);
            Assert.Equal(0.4, br.Center("r3").Y, 6);
            Assert.Equal(3, br.Couplings.Count);
        }

        [Fact]
        public void Release_refused_when_payload_would_lose_its_verified_link()
        {
            var g = Line3();
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            var d = g.CanRelease("c0_1");
            Assert.False(d.Allowed);
            Assert.Contains("payload p", d.Reason);
        }

        [Fact]
        public void Release_allowed_for_a_coupling_the_payload_does_not_depend_on()
        {
            var g = Line3();
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            var d = g.CanRelease("c1_2");
            Assert.True(d.Allowed, d.Reason);
        }

        [Fact]
        public void Release_refused_when_another_support_coupling_is_unverified()
        {
            var g = Line3();
            g.Couplings["c0_1"].State = ConnectorState.MagnetCaptured;   // payload bridge only magnet-held
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            var d = g.CanRelease("c1_2");
            Assert.False(d.Allowed);
            Assert.Contains("not be joined by a verified coupling", d.Reason);
        }

        [Fact]
        public void Single_deck_payload_allows_release_if_com_inside_hull_with_margin()
        {
            var g = Line3();
            g.AddPayload("p", "box_small", new Vec2(0, 0), "r0");
            Assert.True(g.CanRelease("c0_1").Allowed);
        }

        [Fact]
        public void Payload_com_outside_support_hull_refuses_every_release()
        {
            var g = Line3();
            g.AddPayload("p", "box_small", new Vec2(0.2, 0), "r0");     // hull x in [-0.18, 0.18]
            var d = g.CanRelease("c1_2");
            Assert.False(d.Allowed);
            Assert.Contains("margin", d.Reason);
        }

        [Fact]
        public void Margin_requirement_is_enforced_near_the_hull_edge()
        {
            var g = Line3();
            double edge = 0.21 - g.Assume.DeckContactInset;   // 0.18
            g.AddPayload("p", "box_small", new Vec2(edge - g.Assume.SupportMargin + 0.005, 0), "r0");
            Assert.False(g.CanRelease("c1_2").Allowed);
            g.Payloads["p"].Com = new Vec2(edge - g.Assume.SupportMargin - 0.005, 0);
            Assert.True(g.CanRelease("c1_2").Allowed);
        }

        [Fact]
        public void Side_by_side_tray_needs_both_decks_and_their_coupling()
        {
            var g = ModuleGraph.SideBySide(Fx.Profile(), new DockingAssumptions());
            g.AddPayload("tray", "tray_wide", new Vec2(0, 0.2), "r0", "r1");
            var d = g.CanRelease("c0_1");
            Assert.False(d.Allowed);
            // the combined hull spans y in [-0.12, 0.52]; COM at y=0.2 is well inside once linked
            Assert.True(g.CheckSupport().Ok);
        }

        [Fact]
        public void Side_by_side_with_tray_on_one_deck_can_release()
        {
            var g = ModuleGraph.SideBySide(Fx.Profile(), new DockingAssumptions());
            g.AddPayload("small", "box_small", new Vec2(0, 0), "r0");
            Assert.True(g.CanRelease("c0_1").Allowed);
        }

        [Fact]
        public void Branched_only_the_load_bearing_leg_is_protected()
        {
            var g = ModuleGraph.Branched(Fx.Profile(), new DockingAssumptions());
            g.AddPayload("long", "box_long", new Vec2(0.21, 0), "r0", "r1");
            Assert.False(g.CanRelease("c0_1").Allowed);
            Assert.True(g.CanRelease("c0_2").Allowed);   // rear leg
            Assert.True(g.CanRelease("c0_3").Allowed);   // side leg
            // after the side leg is unverified the remaining graph is still fine for payload on r0+r1 but the rear leg must not be affected
            g.Couplings["c0_3"].State = ConnectorState.Partial;
            Assert.True(g.CanRelease("c0_2").Allowed);
        }

        [Fact]
        public void Branched_payload_spanning_side_leg_blocks_that_leg_only()
        {
            var g = ModuleGraph.Branched(Fx.Profile(), new DockingAssumptions());
            g.AddPayload("tray", "tray_wide", new Vec2(0, 0.2), "r0", "r3");
            Assert.False(g.CanRelease("c0_3").Allowed);
            Assert.True(g.CanRelease("c0_1").Allowed);
            Assert.True(g.CanRelease("c0_2").Allowed);
        }

        [Fact]
        public void Forced_unsafe_release_raises_UnsafeRelease_and_plain_release_raises_refused()
        {
            var g = Line3();
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            var refused = g.Release("c0_1");
            Assert.Equal(EventKind.ReleaseRefused, refused.Kind);
            Assert.False(g.Couplings["c0_1"].Removed);
            var unsafeEv = g.Release("c0_1", force: true);
            Assert.Equal(EventKind.UnsafeRelease, unsafeEv.Kind);
            Assert.True(g.Couplings["c0_1"].Removed);
            Assert.Equal("c0_1", unsafeEv.CouplingId);
            // the payload's deck link is gone, so nothing else may release now either
            Assert.Equal(EventKind.ReleaseRefused, g.Release("c1_2").Kind);
            var g2 = Line3();
            g2.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            Assert.Equal(EventKind.Released, g2.Release("c1_2").Kind);
        }

        [Fact]
        public void Poll_raises_UnsafeRelease_when_a_load_bearing_coupling_loses_verification()
        {
            var g = Line3();
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            Assert.Empty(g.Poll());
            g.Couplings["c1_2"].State = ConnectorState.VerificationLost;   // not load bearing: nothing
            Assert.Empty(g.Poll());
            g.Couplings["c0_1"].State = ConnectorState.VerificationLost;   // load bearing
            var ev = g.Poll();
            Assert.Single(ev);
            Assert.Equal(EventKind.UnsafeRelease, ev[0].Kind);
        }

        [Fact]
        public void Coupling_load_estimate_hand_computed_lever_rule()
        {
            var g = Line3();
            var p = g.AddPayload("p", "box_long", new Vec2(0.14, 0), "r0", "r1");   // 0.14 from r0, 0.28 from r1
            var s = g.DeckShares(p);
            Assert.Equal(2.0 / 3, s["r0"], 9); Assert.Equal(1.0 / 3, s["r1"], 9);
            double w = 6.0 * g.Assume.Gravity;
            var l = g.CouplingLoad("c0_1");
            Assert.Equal(w * (1.0 / 3) / 2 / 2, l.A, 9);   // V = W*|ds|/2, split over two hooks
            Assert.Equal(0, g.CouplingLoad("c1_2").A, 12);
        }

        [Fact]
        public void Overloaded_remaining_coupling_blocks_release()
        {
            var assume = new DockingAssumptions();
            var g = ModuleGraph.Line(Fx.Profile(), assume, 3);
            // heavy asymmetric load: build a payload spec-free by editing profile mass
            g.Profile.Payloads["box_long"] = new Crane.Fleet.Profile.PayloadSpec("box_long", g.Profile.Payloads["box_long"].SizeLwh,
                new Crane.Fleet.Profile.Param(400.0, Crane.Fleet.Profile.Provenance.Provisional), "test");
            g.AddPayload("p", "box_long", new Vec2(0.05, 0), "r0", "r1");
            var d = g.CanRelease("c1_2");
            Assert.False(d.Allowed);
            Assert.Contains("exceeds rated", d.Reason);
        }

        [Fact]
        public void Machine_release_flows_through_graph_authorisation()
        {
            var g = Line3();
            g.AddPayload("p", "box_long", new Vec2(0.21, 0), "r0", "r1");
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            g.Couplings["c0_1"].Machine = m;
            inp.Command = DockingCommand.Release;
            inp.ReleaseAuthorized = g.CanRelease("c0_1").Allowed;
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            Assert.Contains(res.Events, e => e.Kind == EventKind.ReleaseRefused);

            // the other leg may release
            var m2 = Fx.Machine(); var inp2 = Fx.Latch(m2);
            g.Couplings["c1_2"].Machine = m2;
            inp2.Command = DockingCommand.Release; inp2.ReleaseAuthorized = g.CanRelease("c1_2").Allowed;
            Fx.Run(m2, inp2, 1.2);
            Assert.Equal(ConnectorState.Released, m2.State);
        }

        [Fact]
        public void Convex_hull_and_margin()
        {
            var sq = Hull2.Convex(new[] { new Vec2(0, 0), new Vec2(1, 0), new Vec2(1, 1), new Vec2(0, 1), new Vec2(0.5, 0.5), new Vec2(0.5, 0) });
            Assert.Equal(4, sq.Count);
            Assert.Equal(0.5, Hull2.InsideMargin(sq, new Vec2(0.5, 0.5)), 12);
            Assert.True(Hull2.InsideMargin(sq, new Vec2(1.2, 0.5)) < 0);
            Assert.True(double.IsNegativeInfinity(Hull2.InsideMargin(new[] { new Vec2(0, 0), new Vec2(1, 0) }, new Vec2(0.5, 0))));
        }
    }

    public class OutcomeTests
    {
        [Theory]
        [InlineData(false, 0, DockingOutcome.Missed)]
        [InlineData(false, 2, DockingOutcome.Missed)]
        [InlineData(true, 0, DockingOutcome.MagnetCapturedOnly)]
        [InlineData(true, 1, DockingOutcome.OneHook)]
        [InlineData(true, 2, DockingOutcome.BothHooksVerified)]
        public void From_geometry(bool captured, int hooks, DockingOutcome expected)
            => Assert.Equal(expected, OutcomeClassifier.FromGeometry(captured, hooks));

        [Fact]
        public void From_geometry_flags()
        {
            Assert.Equal(DockingOutcome.Obstructed, OutcomeClassifier.FromGeometry(true, 2, obstructed: true));
            Assert.Equal(DockingOutcome.LoadValidated, OutcomeClassifier.FromGeometry(true, 2, loadValidated: true));
        }

        [Fact]
        public void Classify_attempt_precedence()
        {
            var s = new AttemptSummary { EverCaptured = true, SeatedA = true, SeatedB = true, SensedA = true, SensedB = true, Verified = true };
            Assert.Equal(DockingOutcome.BothHooksVerified, OutcomeClassifier.Classify(s));
            s.LoadValidated = true; Assert.Equal(DockingOutcome.LoadValidated, OutcomeClassifier.Classify(s));
            s.SeatedB = false; Assert.Equal(DockingOutcome.FalseEngagement, OutcomeClassifier.Classify(s));   // B sensed but not seated
            s = new AttemptSummary { EverCaptured = true, SeatedA = true, SensedA = true };
            Assert.Equal(DockingOutcome.OneHook, OutcomeClassifier.Classify(s));
            s = new AttemptSummary { EverCaptured = true };
            Assert.Equal(DockingOutcome.MagnetCapturedOnly, OutcomeClassifier.Classify(s));
            s = new AttemptSummary { EverObstructed = true };
            Assert.Equal(DockingOutcome.Obstructed, OutcomeClassifier.Classify(s));
            s = new AttemptSummary();
            Assert.Equal(DockingOutcome.Missed, OutcomeClassifier.Classify(s));
        }

        [Fact]
        public void Python_codes_map_to_shared_words()
        {
            Assert.Equal(DockingOutcome.Missed, OutcomeClassifier.FromPythonCode(0));
            Assert.Equal(DockingOutcome.BothHooksVerified, OutcomeClassifier.FromPythonCode(3));
            Assert.Throws<ArgumentOutOfRangeException>(() => OutcomeClassifier.FromPythonCode(4));
        }
    }
}
