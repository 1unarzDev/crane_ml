using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Docking;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    public class TransitionTableTests
    {
        public static IEnumerable<object[]> UnguardedRows() =>
            ConnectorMachine.Table.Where(r => r.Guard == null).Select(r => new object[] { r.From, r.Trigger, r.To });

        [Theory]
        [MemberData(nameof(UnguardedRows))]
        public void Every_unguarded_legal_transition_fires(ConnectorState from, Trigger trig, ConnectorState to)
        {
            var m = Fx.Machine();
            m.ForceState(from);
            var ev = m.Fire(trig, new ConnectorInputs());
            Assert.Equal(EventKind.Transition, ev.Kind);
            Assert.Equal(from, ev.From); Assert.Equal(to, ev.To); Assert.Equal(trig, ev.Trigger);
            Assert.Equal(to, m.State);
        }

        [Fact]
        public void Every_pair_not_in_the_table_is_illegal_and_leaves_state_unchanged()
        {
            int checkedPairs = 0;
            foreach (ConnectorState s in Enum.GetValues(typeof(ConnectorState)))
                foreach (Trigger t in Enum.GetValues(typeof(Trigger)))
                {
                    if (ConnectorMachine.Table.Any(r => r.From == s && r.Trigger == t)) continue;
                    var m = Fx.Machine(); m.ForceState(s);
                    var ev = m.Fire(t, new ConnectorInputs());
                    Assert.Equal(EventKind.IllegalTransition, ev.Kind);
                    Assert.Equal(s, m.State);
                    checkedPairs++;
                }
            Assert.True(checkedPairs > 100);
        }

        [Fact]
        public void Specific_illegal_examples()
        {
            var m = Fx.Machine();
            Assert.Equal(EventKind.IllegalTransition, m.Fire(Trigger.Captured).Kind);          // FREE cannot capture
            Assert.Equal(EventKind.IllegalTransition, m.Fire(Trigger.LoadTestPassed).Kind);    // FREE cannot validate load
            m.ForceState(ConnectorState.Released);
            Assert.Equal(EventKind.IllegalTransition, m.Fire(Trigger.CmdRelease).Kind);        // already released
            m.ForceState(ConnectorState.Obstructed);
            Assert.Equal(EventKind.IllegalTransition, m.Fire(Trigger.Captured).Kind);          // obstructed never captures
        }

        [Fact]
        public void Every_state_appears_in_the_table()
        {
            foreach (ConnectorState s in Enum.GetValues(typeof(ConnectorState)))
                Assert.Contains(ConnectorMachine.Table, r => r.From == s || r.To == s);
        }

        [Fact]
        public void Illegal_commands_through_Step_are_reported_not_applied()
        {
            var m = Fx.Machine();
            var inp = Fx.At(); inp.Command = DockingCommand.Verify;
            var res = m.Step(Fx.Dt, inp);
            Assert.Contains(res.Events, e => e.Kind == EventKind.IllegalTransition);
            Assert.Equal(ConnectorState.Free, m.State);
            inp.Command = DockingCommand.EngageHooks;
            res = m.Step(Fx.Dt, inp);
            Assert.Contains(res.Events, e => e.Kind == EventKind.IllegalTransition);
        }

        [Fact]
        public void Guarded_release_rejected_without_authorisation_and_allowed_with_it()
        {
            foreach (var s in new[] { ConnectorState.LatchedVerified, ConnectorState.LoadValidated, ConnectorState.VerificationLost })
            {
                var m = Fx.Machine(); m.ForceState(s);
                var ev = m.Fire(Trigger.CmdRelease, new ConnectorInputs { ReleaseAuthorized = false });
                Assert.Equal(EventKind.GuardRejected, ev.Kind);
                Assert.Equal(s, m.State);
                Assert.Contains(m.Log, e => e.Kind == EventKind.ReleaseRefused);
                Assert.Equal(EventKind.Transition, m.Fire(Trigger.CmdRelease, new ConnectorInputs { ReleaseAuthorized = true }).Kind);
                Assert.Equal(ConnectorState.ReleaseRequested, m.State);
            }
        }
    }

    public class ScenarioTests
    {
        [Fact]
        public void Happy_path_reaches_load_validated_and_release_with_typed_timestamped_events()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            inp.Command = DockingCommand.TestLoad;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LoadValidated, m.State);
            Assert.Equal(DockingOutcome.LoadValidated, OutcomeClassifier.Classify(m.Summary));

            inp.Command = DockingCommand.Release; inp.ReleaseAuthorized = true;
            Fx.Run(m, inp, 1.2);
            Assert.Equal(ConnectorState.Released, m.State);

            var seq = m.Log.Where(e => e.Kind == EventKind.Transition).Select(e => e.To).ToList();
            Assert.Equal(new[]
            {
                ConnectorState.Approach, ConnectorState.MagnetCaptured, ConnectorState.HookAEngaged, ConnectorState.HookAEngaged /* B sensed */, ConnectorState.LatchedVerified,
                ConnectorState.LoadValidated, ConnectorState.ReleaseRequested, ConnectorState.Released,
            }, seq);
            for (int i = 1; i < m.Log.Count; i++) Assert.True(m.Log[i].Time >= m.Log[i - 1].Time);
            Assert.All(m.Log, e => Assert.True(e.Time > 0));
        }

        [Fact]
        public void Capture_and_both_hooks_sensed_independently()
        {
            var m = Fx.Machine();
            Fx.Engage(m);
            Assert.True(m.SensedA && m.SensedB);
            Assert.Equal(HookSeat.Seated, m.MechA); Assert.Equal(HookSeat.Seated, m.MechB);
            Assert.Equal(ConnectorState.HookAEngaged, m.State); // second sensed hook keeps the state, flags carry the rest
        }

        [Fact]
        public void Hooks_take_the_profile_actuation_time()
        {
            var m = Fx.Machine();
            var inp = Fx.Capture(m);
            inp.Command = DockingCommand.EngageHooks;
            double t0 = m.Time; int steps = 0;
            while (!m.SensedA && steps++ < 100) m.Step(Fx.Dt, inp);
            Assert.InRange(m.Time - t0, m.Model.HookActuationTime - Fx.Dt, m.Model.HookActuationTime + Fx.Dt);
        }

        [Fact]
        public void Yaw_error_at_the_hook_spacing_gives_one_hook_engagement()
        {
            // lateral +1.5 mm and yaw 0.025 rad: lever arm 0.06*sin(0.025) = +1.5 mm => hook A sees 3.0 mm (> 2 mm), hook B sees ~0.
            var m = Fx.Machine();
            Fx.Engage(m, lat: 0.0015, yaw: 0.025, seconds: 3.0);
            Assert.Equal(HookSeat.JammedShort, m.MechA);
            Assert.Equal(HookSeat.Seated, m.MechB);
            Assert.False(m.SensedA); Assert.True(m.SensedB);
            Assert.Equal(ConnectorState.Partial, m.State);
            Assert.Equal(DockingOutcome.OneHook, OutcomeClassifier.Classify(m.Summary));
            Assert.Contains(m.Log, e => e.Kind == EventKind.Jam && e.Detail.Contains("hook A"));
        }

        [Fact]
        public void Mirror_pose_engages_the_other_hook()
        {
            var m = Fx.Machine();
            Fx.Engage(m, lat: 0.0015, yaw: -0.025, seconds: 3.0);
            Assert.Equal(HookSeat.Seated, m.MechA); Assert.Equal(HookSeat.JammedShort, m.MechB);
            Assert.Equal(ConnectorState.Partial, m.State);
        }

        [Fact]
        public void Yaw_error_alone_within_tolerance_still_seats_both()
        {
            var m = Fx.Machine();
            Fx.Engage(m, lat: 0, yaw: 0.03); // lever arm 1.8 mm < 2 mm each side
            Assert.Equal(HookSeat.Seated, m.MechA); Assert.Equal(HookSeat.Seated, m.MechB);
        }

        [Fact]
        public void Yaw_beyond_tolerance_seats_neither_hook()
        {
            var m = Fx.Machine();
            Fx.Engage(m, lat: 0, yaw: 0.05, seconds: 3.0);
            Assert.Equal(DockingOutcome.MagnetCapturedOnly, OutcomeClassifier.Classify(m.Summary));
            Assert.False(m.SensedA || m.SensedB);
            Assert.Equal(ConnectorState.Released, m.State);   // timeout with no hook sensed: released for a retry
            Assert.Equal(1, m.FailedAttempts);
        }

        [Fact]
        public void Obstruction_never_reaches_capture()
        {
            var m = Fx.Machine();
            var inp = Fx.At(0, 0, 0.1); inp.Command = DockingCommand.Approach;
            m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.None; inp.Obstruction = true;
            foreach (double gap in new[] { 0.05, 0.02, 0.008, 0.004, 0.0 })
            {
                inp.Pose.Gap = gap;
                Fx.Run(m, inp, 0.5);
            }
            Assert.Equal(ConnectorState.Obstructed, m.State);
            Assert.DoesNotContain(m.Log, e => e.Kind == EventKind.Capture);
            Assert.False(m.Captured);
            Assert.Equal(DockingOutcome.Obstructed, OutcomeClassifier.Classify(m.Summary));
        }

        [Fact]
        public void Brief_obstruction_below_debounce_is_ignored_and_clears()
        {
            var m = Fx.Machine();
            var inp = Fx.At(0, 0, 0.1); inp.Command = DockingCommand.Approach;
            m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.None; inp.Obstruction = true;
            m.Step(Fx.Dt, inp);
            inp.Obstruction = false; inp.Pose.Gap = 0.004;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.MagnetCaptured, m.State);
        }

        [Fact]
        public void Obstructed_can_reapproach_after_clear_but_not_while_blocked()
        {
            var m = Fx.Machine(); m.ForceState(ConnectorState.Obstructed);
            var ev = m.Fire(Trigger.CmdApproach, new ConnectorInputs { Obstruction = true });
            Assert.Equal(EventKind.GuardRejected, ev.Kind);
            ev = m.Fire(Trigger.CmdApproach, new ConnectorInputs { Obstruction = false });
            Assert.Equal(ConnectorState.Approach, m.State);
        }

        [Fact]
        public void Outside_guide_envelope_jams_and_requests_back_off_and_reapproach()
        {
            var m = Fx.Machine();
            var inp = Fx.At(0.03, 0, 0.1); inp.Command = DockingCommand.Approach;  // 30 mm > g + r = 18 mm
            m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.None; inp.Pose.Gap = 0.004;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.Approach, m.State); // not captured: the lip is not reached yet
            inp.Pose.Gap = 0.0;
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.Free, m.State);
            Assert.Equal(RequestedAction.BackOffAndReapproach, res.Action);
            Assert.Equal(1, m.FailedAttempts);
            Assert.Equal(DockingOutcome.Missed, OutcomeClassifier.Classify(m.Summary));
        }

        [Fact]
        public void Too_fast_closing_bounces()
        {
            var m = Fx.Machine();
            var inp = Fx.At(0, 0, 0.1); inp.Command = DockingCommand.Approach; inp.ClosingSpeed = 0.5;
            m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.None; inp.Pose.Gap = 0.0;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.Free, m.State);
            Assert.Contains(m.Log, e => e.Kind == EventKind.Jam && e.Detail.Contains("closing speed"));
        }

        [Fact]
        public void Retries_are_bounded_then_abandon()
        {
            var a = Fx.Det(); a.MaxRetries = 2;
            var m = Fx.Machine(a);
            var inp = Fx.At(0.03, 0, 0.1);
            RequestedAction last = RequestedAction.None;
            int attempts = 0;
            for (; attempts < 10 && !m.RetriesExhausted; attempts++)
            {
                inp.Command = DockingCommand.Approach; inp.Pose.Gap = 0.1; m.Step(Fx.Dt, inp);
                inp.Command = DockingCommand.None; inp.Pose.Gap = 0.0; last = m.Step(Fx.Dt, inp).Action;
            }
            Assert.Equal(3, attempts);                      // MaxRetries + 1 total attempts
            Assert.Equal(RequestedAction.Abandon, last);
            Assert.Contains(m.Log, e => e.Kind == EventKind.RetriesExhausted);
            // a further approach is refused
            inp.Command = DockingCommand.Approach; inp.Pose.Gap = 0.1;
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.Free, m.State);
            Assert.Equal(RequestedAction.Abandon, res.Action);
            Assert.Contains(res.Events, e => e.Kind == EventKind.GuardRejected);
        }

        [Fact]
        public void Hook_timeout_with_no_hook_sensed_releases_for_a_retry()
        {
            var a = Fx.Det();
            var m = Fx.Machine(a);
            var inp = Fx.Capture(m);
            inp.Command = DockingCommand.EngageHooks;
            inp.HookA = HookDrive.Stalled; inp.HookB = HookDrive.Stalled;
            var ev = Fx.Run(m, inp, 3.0);
            Assert.Contains(ev, e => e.Kind == EventKind.Transition && e.Trigger == Trigger.HookTimeout);
            Assert.Equal(ConnectorState.Released, m.State);   // nothing was closed, so the hooks are already open
            Assert.Equal(1, m.FailedAttempts);
        }

        [Fact]
        public void Closed_on_nothing_is_caught_by_the_verification_geometry()
        {
            var a = Fx.Det(); a.ClosedOnNothingProbability = 1.0;
            var m = Fx.Machine(a);
            Fx.Engage(m, lat: 0.003); // 3 mm = 1.5x tolerance on both hooks: inside the window, hooks swing shut on nothing
            Assert.Equal(HookSeat.ClosedOnNothing, m.MechA); Assert.Equal(HookSeat.ClosedOnNothing, m.MechB);
            Assert.True(m.SensedA && m.SensedB);                // the sensors lie
            var inp = Fx.At(0.003, 0, 0.004); inp.Command = DockingCommand.Verify;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
            Assert.Contains(m.Log, e => e.Kind == EventKind.VerifyFailed);
            Assert.Equal(DockingOutcome.FalseEngagement, OutcomeClassifier.Classify(m.Summary));
        }

        [Fact]
        public void Sensor_false_positive_on_a_jammed_hook_is_caught_by_verification()
        {
            var a = Fx.Det(); a.SensorFalsePositiveProbability = 1.0;
            var m = Fx.Machine(a);
            Fx.Engage(m, lat: 0.0, yaw: 0.05); // outside window (ratio 0.003/.002 = 1.5? yaw lever 3 mm) -> jam, FP makes sensors read engaged
            Assert.Contains(m.Log, e => e.Kind == EventKind.FalsePositiveSensor);
            Assert.True(m.SensedA && m.SensedB);
            Assert.NotEqual(HookSeat.Seated, m.MechA);
            var inp = Fx.At(0, 0.05, 0.004); inp.Command = DockingCommand.Verify;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
            Assert.Equal(DockingOutcome.FalseEngagement, OutcomeClassifier.Classify(m.Summary));
        }

        [Fact]
        public void Perfect_geometry_but_hook_misses_its_catch_passes_geometry_but_fails_the_load_check()
        {
            var a = Fx.Det(); a.HookMissCatchProbability = 1.0;
            var m = Fx.Machine(a);
            var inp = Fx.Latch(m);
            Assert.Equal(ConnectorState.LatchedVerified, m.State); // geometry alone cannot see it
            inp.Command = DockingCommand.TestLoad;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
        }

        [Fact]
        public void Load_check_with_a_test_load_above_rated_does_not_validate()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.Command = DockingCommand.TestLoad; inp.AppliedLoad = new LoadCase(250, 0, 0);
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
        }

        [Fact]
        public void Verification_lost_when_pose_residual_drifts_out()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.Disturbance.Lateral = 0.0025; // 1.25x tolerance: inside the 1.5x monitoring hysteresis
            Fx.Run(m, inp, 0.5);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            inp.Disturbance.Lateral = 0.004;  // 2x tolerance
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
            Assert.Equal(RequestedAction.ReverifyOrReleaseWithSupport, res.Action);
        }

        [Fact]
        public void Verification_lost_when_a_hook_signal_drops()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.HookB = HookDrive.ForcedOpen;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
            Assert.False(m.SensedB);
        }

        [Fact]
        public void Verification_lost_then_reverify_restores_latch()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.Disturbance.Lateral = 0.004; m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
            inp.Disturbance.Lateral = 0.0; inp.Command = DockingCommand.Verify; m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
        }

        [Fact]
        public void Overload_slips_the_hooks_and_loses_verification()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.AppliedLoad = new LoadCase(150, 0, 0); // 75 N per hook > 60 N rated
            m.Step(Fx.Dt, inp);
            Assert.Contains(m.Log, e => e.Kind == EventKind.HookOverload);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
        }

        [Fact]
        public void Within_rated_load_keeps_the_latch()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.AppliedLoad = new LoadCase(100, 0, 0); // 50 N per hook
            Fx.Run(m, inp, 1.0);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
        }

        [Fact]
        public void Stuck_hook_when_release_is_commanded_and_a_hook_does_not_open()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.ReleaseAuthorized = true; inp.Command = DockingCommand.Release; inp.HookA = HookDrive.Stalled;
            var res = Fx.Run(m, inp, 2.0);
            Assert.Equal(ConnectorState.StuckHook, m.State);
            Assert.Contains(res, e => e.Trigger == Trigger.ReleaseTimeout);
            // actuator recovers, release re-commanded
            inp.HookA = HookDrive.Normal; inp.Command = DockingCommand.None; m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.Release; Fx.Run(m, inp, 1.2);
            Assert.Equal(ConnectorState.Released, m.State);
        }

        [Fact]
        public void Release_retries_from_stuck_hook_are_bounded()
        {
            var a = Fx.Det(); a.MaxRetries = 1;
            var m = Fx.Machine(a);
            var inp = Fx.Latch(m);
            inp.ReleaseAuthorized = true; inp.HookA = HookDrive.Stalled;
            int rejected = 0;
            for (int i = 0; i < 5; i++)
            {
                inp.Command = DockingCommand.None; m.Step(Fx.Dt, inp);
                inp.Command = DockingCommand.Release;
                var r = m.Step(Fx.Dt, inp);
                if (r.Events.Any(e => e.Kind == EventKind.GuardRejected)) rejected++;
                Fx.Run(m, inp, 2.0);
            }
            Assert.True(rejected >= 1);
            Assert.Equal(ConnectorState.StuckHook, m.State);
        }

        [Fact]
        public void Release_requires_authorisation_in_the_stepped_machine()
        {
            var m = Fx.Machine();
            var inp = Fx.Latch(m);
            inp.Command = DockingCommand.Release; inp.ReleaseAuthorized = false;
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            Assert.Contains(res.Events, e => e.Kind == EventKind.ReleaseRefused);
        }

        [Fact]
        public void Unverified_states_can_release_without_graph_authorisation()
        {
            var m = Fx.Machine();
            var inp = Fx.Engage(m);                        // sensed but never verified
            inp.Command = DockingCommand.Release;
            Fx.Run(m, inp, 1.5);
            Assert.Equal(ConnectorState.Released, m.State);
        }

        [Fact]
        public void Determinism_same_seed_same_events_different_seed_differs()
        {
            string Trace(ulong seed)
            {
                var m = new ConnectorMachine(Fx.Model(new DockingAssumptions()), seed);
                var inp = Fx.Latch(m, lat: 0.0012, yaw: 0.02);
                inp.Command = DockingCommand.TestLoad; m.Step(Fx.Dt, inp);
                return string.Join("|", m.Log.Select(e => e.ToString())) + $"|{m.Settled.Lateral:R}|{m.Settled.Yaw:R}";
            }
            Assert.Equal(Trace(42), Trace(42));
            Assert.NotEqual(Trace(42), Trace(43));
        }

        [Fact]
        public void Rng_is_reproducible_and_roughly_standard_normal()
        {
            var r = new DeterministicRng(7); var r2 = new DeterministicRng(7);
            double s = 0, s2 = 0; int n = 20000;
            for (int i = 0; i < n; i++) { double g = r.NextGaussian(); Assert.Equal(g, r2.NextGaussian()); s += g; s2 += g * g; }
            Assert.InRange(s / n, -0.05, 0.05); Assert.InRange(s2 / n, 0.95, 1.05);
        }

        [Fact]
        public void Step_rejects_non_positive_dt()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => Fx.Machine().Step(0, new ConnectorInputs()));
        }
    }
}
