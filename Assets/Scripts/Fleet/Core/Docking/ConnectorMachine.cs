using System;
using System.Collections.Generic;
using Crane.Fleet.Profile;

namespace Crane.Fleet.Docking
{
    /// <summary>Everything a transition guard may look at.</summary>
    public sealed class GuardContext
    {
        public ConnectorMachine M;
        public ConnectorInputs In;
    }

    /// <summary>One row of the transition table.</summary>
    public sealed class TransitionRow
    {
        public ConnectorState From; public Trigger Trigger; public string GuardName; public Func<GuardContext, bool> Guard;
        public ConnectorState To; public RequestedAction Action; public bool CountsAsFailure;
        public override string ToString() => $"{From,-16} --{Trigger}[{GuardName ?? "-"}]--> {To}  action={Action}{(CountsAsFailure ? " (failed attempt)" : "")}";
    }

    /// <summary>Ground-truth and sensed summary of an attempt, input to <see cref="OutcomeClassifier"/>.</summary>
    public struct AttemptSummary
    {
        public bool EverCaptured, EverObstructed, Verified, LoadValidated;
        public bool SeatedA, SeatedB;          // ground truth: hook mechanically seated at last close
        public bool SensedA, SensedB;          // what the sensors say now
        public bool FalseEngagementDeclared;
        public ConnectorState FinalState;
    }

    /// <summary>
    /// Deterministic state machine for ONE coupling (face A on rover i with face B on rover j). Time advances only through
    /// <see cref="Step"/>. Transitions are rows in <see cref="Table"/>; rows with the same (state, trigger) are tried in order.
    /// Commands are edge-triggered: re-issue a command (via None) to repeat it.
    /// </summary>
    public sealed class ConnectorMachine
    {
        public ContactModel Model { get; }
        public DockingAssumptions A => Model.Assume;
        public string Id { get; }
        public ConnectorState State { get; private set; } = ConnectorState.Free;
        public double Time { get; private set; }
        public int FailedAttempts { get; private set; }
        public bool RetriesExhausted => FailedAttempts > A.MaxRetries;
        public int ReleaseAttempts { get; private set; }
        public double HookPosA { get; private set; }
        public double HookPosB { get; private set; }
        public HookSeat? MechA { get; private set; }
        public HookSeat? MechB { get; private set; }
        public bool SensedA { get; private set; }
        public bool SensedB { get; private set; }
        public Residual Settled { get; private set; }
        public bool Captured { get; private set; }
        /// <summary>Reciprocal mode: true while the partner-hook report is fresh (link up within PartnerObservationMaxAge). Always true in two-hooks-per-face mode.</summary>
        public bool PartnerObservationFresh => !Model.Reciprocal || Time - _lastLinkTime <= A.PartnerObservationMaxAge + 1e-9;
        public AttemptSummary Summary => BuildSummary();
        public IReadOnlyList<DockingEvent> Log => _log;

        private readonly DeterministicRng _rng;
        private readonly List<DockingEvent> _log = new List<DockingEvent>();
        private DockingCommand _lastCmd = DockingCommand.None;
        private double _engageTimer, _releaseTimer, _obstructionTimer;
        private bool _engaging, _everCaptured, _everObstructed, _verified, _loadValidated, _falseDeclared;
        private bool _seatedA, _seatedB;
        // reciprocal mode: connection-level observation of the partner's hook (HOOK_B) and the partner close command, delayed by A.PartnerLinkLatency
        private readonly List<KeyValuePair<double, bool>> _bLine = new List<KeyValuePair<double, bool>>();
        private bool _bHeld;
        private double _lastLinkTime, _partnerCmdAge;
        private StepResult _cur;
        private ConnectorInputs _in;

        public ConnectorMachine(ContactModel model, ulong seed, string id = "c0")
        {
            Model = model; _rng = new DeterministicRng(seed); Id = id;
        }

        // ================================================================= transition table
        public static readonly IReadOnlyList<TransitionRow> Table = BuildTable();

        private static TransitionRow R(ConnectorState from, Trigger t, ConnectorState to, RequestedAction act = RequestedAction.None,
            string gname = null, Func<GuardContext, bool> g = null, bool fail = false)
            => new TransitionRow { From = from, Trigger = t, To = to, Action = act, GuardName = gname, Guard = g, CountsAsFailure = fail };

        private static List<TransitionRow> BuildTable()
        {
            const ConnectorState Fr = ConnectorState.Free, Ap = ConnectorState.Approach, Mc = ConnectorState.MagnetCaptured,
                Ha = ConnectorState.HookAEngaged, Hb = ConnectorState.HookBEngaged, Lv = ConnectorState.LatchedVerified,
                Lo = ConnectorState.LoadValidated, Rr = ConnectorState.ReleaseRequested, Re = ConnectorState.Released,
                Pa = ConnectorState.Partial, Fe = ConnectorState.FalseEngagement, Ob = ConnectorState.Obstructed,
                Vl = ConnectorState.VerificationLost, Sh = ConnectorState.StuckHook;
            Func<GuardContext, bool> notExhausted = c => !c.M.RetriesExhausted;
            Func<GuardContext, bool> clear = c => !c.In.Obstruction && !c.M.RetriesExhausted;
            Func<GuardContext, bool> authorized = c => c.In.ReleaseAuthorized && c.M.PartnerObservationFresh;
            Func<GuardContext, bool> relBudget = c => c.M.ReleaseAttempts <= c.M.A.MaxRetries;
            Func<GuardContext, bool> anyHook = c => c.M.SensedA || c.M.SensedB;
            Func<GuardContext, bool> noHook = c => !c.M.SensedA && !c.M.SensedB;
            Func<GuardContext, bool> sensedAOnly = c => c.M.SensedA;
            Func<GuardContext, bool> bothSensed = c => c.M.SensedA && c.M.SensedB;
            var t = new List<TransitionRow>
            {
                // --- approach
                R(Fr, Trigger.CmdApproach, Ap, g: notExhausted, gname: "retries left"),
                R(Re, Trigger.CmdApproach, Ap, g: notExhausted, gname: "retries left"),
                R(Ob, Trigger.CmdApproach, Ap, g: clear, gname: "obstruction cleared, retries left"),
                R(Ob, Trigger.CmdRelease, Fr),
                R(Ap, Trigger.Captured, Mc),
                R(Ap, Trigger.Jam, Fr, RequestedAction.BackOffAndReapproach, fail: true),
                R(Ap, Trigger.ObstructionDetected, Ob, RequestedAction.StopApproach),
                R(Ap, Trigger.CmdRelease, Fr),
                // --- hooks
                R(Mc, Trigger.HookASensed, Ha),
                R(Mc, Trigger.HookBSensed, Hb),
                R(Mc, Trigger.HookTimeout, Rr, RequestedAction.BackOffAndReapproach, fail: true),
                R(Mc, Trigger.CmdRelease, Rr),
                R(Ha, Trigger.HookBSensed, Ha),   // second hook: stays in HOOK_A_ENGAGED with both flags set
                R(Hb, Trigger.HookASensed, Hb),
                R(Ha, Trigger.HookSignalLost, Mc, g: noHook, gname: "no hook sensed"),
                R(Ha, Trigger.HookSignalLost, Hb, g: c => c.M.SensedB && !c.M.SensedA, gname: "only B sensed"),
                R(Ha, Trigger.HookSignalLost, Ha, g: sensedAOnly, gname: "A still sensed"),
                R(Hb, Trigger.HookSignalLost, Mc, g: noHook, gname: "no hook sensed"),
                R(Hb, Trigger.HookSignalLost, Ha, g: c => c.M.SensedA && !c.M.SensedB, gname: "only A sensed"),
                R(Hb, Trigger.HookSignalLost, Hb, g: c => c.M.SensedB, gname: "B still sensed"),
                R(Ha, Trigger.HookTimeout, Pa, RequestedAction.BackOffAndReapproach, g: c => !(c.M.SensedA && c.M.SensedB), gname: "not both sensed", fail: true),
                R(Hb, Trigger.HookTimeout, Pa, RequestedAction.BackOffAndReapproach, g: c => !(c.M.SensedA && c.M.SensedB), gname: "not both sensed", fail: true),
                R(Pa, Trigger.HookASensed, Ha, g: bothSensed, gname: "second hook arrived"),
                R(Pa, Trigger.HookBSensed, Hb, g: bothSensed, gname: "second hook arrived"),
                R(Pa, Trigger.CmdRelease, Rr),
                R(Ha, Trigger.CmdRelease, Rr),
                R(Hb, Trigger.CmdRelease, Rr),
                // --- verification
                R(Ha, Trigger.VerifyPassed, Lv, g: bothSensed, gname: "both sensed, residual in tolerance"),
                R(Hb, Trigger.VerifyPassed, Lv, g: bothSensed, gname: "both sensed, residual in tolerance"),
                R(Ha, Trigger.FalseEngagementDetected, Fe, RequestedAction.BackOffAndReapproach, fail: true),
                R(Hb, Trigger.FalseEngagementDetected, Fe, RequestedAction.BackOffAndReapproach, fail: true),
                R(Vl, Trigger.VerifyPassed, Lv),
                R(Vl, Trigger.FalseEngagementDetected, Fe, RequestedAction.BackOffAndReapproach, fail: true),
                R(Fe, Trigger.CmdRelease, Rr),
                // --- load check and monitoring
                R(Lv, Trigger.LoadTestPassed, Lo),
                R(Lv, Trigger.LoadTestSlip, Fe, RequestedAction.BackOffAndReapproach, fail: true),
                R(Lo, Trigger.LoadTestPassed, Lo),
                R(Lo, Trigger.LoadTestSlip, Fe, RequestedAction.BackOffAndReapproach, fail: true),
                R(Lv, Trigger.VerificationLostDetected, Vl, RequestedAction.ReverifyOrReleaseWithSupport),
                R(Lo, Trigger.VerificationLostDetected, Vl, RequestedAction.ReverifyOrReleaseWithSupport),
                // --- release (verified states require an authorized release)
                R(Lv, Trigger.CmdRelease, Rr, g: authorized, gname: "release authorised (support verified)"),
                R(Lo, Trigger.CmdRelease, Rr, g: authorized, gname: "release authorised (support verified)"),
                R(Vl, Trigger.CmdRelease, Rr, g: authorized, gname: "release authorised (support verified)"),
                R(Rr, Trigger.ReleaseComplete, Re),
                R(Rr, Trigger.ReleaseTimeout, Sh, RequestedAction.RetryRelease),
                R(Sh, Trigger.CmdRelease, Rr, g: relBudget, gname: "release retries left"),
            };
            return t;
        }

        /// <summary>Sets the state directly (replay, tests, restoring a saved coupling). Does not emit events or touch hook actuators.</summary>
        public void ForceState(ConnectorState s) { State = s; }

        // ================================================================= firing
        /// <summary>Fires a trigger via the table. Public so tests (and hosts with external triggers) can drive transitions directly.</summary>
        public DockingEvent Fire(Trigger trig, ConnectorInputs inputs = null)
        {
            if (_cur == null) _cur = new StepResult();
            if (inputs != null) _in = inputs;
            if (_in == null) _in = new ConnectorInputs();
            return FireInternal(trig);
        }

        private DockingEvent FireInternal(Trigger trig)
        {
            var ctx = new GuardContext { M = this, In = _in };
            bool anyRow = false;
            foreach (var row in Table)
            {
                if (row.From != State || row.Trigger != trig) continue;
                anyRow = true;
                if (row.Guard != null && !row.Guard(ctx)) continue;
                return Apply(row, trig);
            }
            DockingEvent ev;
            if (anyRow)
            {
                ev = Emit(EventKind.GuardRejected, State, State, trig, "guard failed");
                if (trig == Trigger.CmdRelease && (ConnectorStateNames.IsVerified(State) || State == ConnectorState.VerificationLost))
                    Emit(EventKind.ReleaseRefused, State, State, trig, "release not authorised: support not verified on remaining graph");
                if (trig == Trigger.CmdApproach && RetriesExhausted)
                    _cur.Action = RequestedAction.Abandon;
            }
            else ev = Emit(EventKind.IllegalTransition, State, State, trig, "no transition from this state");
            return ev;
        }

        private DockingEvent Apply(TransitionRow row, Trigger trig)
        {
            var from = State;
            State = row.To;
            if (row.Action != RequestedAction.None) _cur.Action = row.Action;
            if (row.CountsAsFailure)
            {
                FailedAttempts++;
                if (RetriesExhausted)
                {
                    _cur.Action = RequestedAction.Abandon;
                    Emit(EventKind.RetriesExhausted, from, State, trig, $"failed attempts {FailedAttempts} > max retries {A.MaxRetries}");
                }
            }
            if (State == ConnectorState.ReleaseRequested) { _releaseTimer = 0; if (from == ConnectorState.StuckHook) ReleaseAttempts++; _engaging = false; }
            if (State == ConnectorState.Approach) { _obstructionTimer = 0; Captured = false; }
            if (State == ConnectorState.Free || State == ConnectorState.Released) Captured = false;
            if (State == ConnectorState.Released) ReleaseAttempts = 0;
            return Emit(EventKind.Transition, from, State, trig, row.GuardName == null ? "" : "guard: " + row.GuardName);
        }

        private DockingEvent Emit(EventKind k, ConnectorState from, ConnectorState to, Trigger? trig, string detail)
        {
            var e = new DockingEvent(Time, k, from, to, trig, detail, Id);
            _log.Add(e); _cur.Events.Add(e);
            return e;
        }

        // ================================================================= Step
        /// <summary>Advances time by dt seconds under the given inputs. Deterministic for a fixed seed and input sequence.</summary>
        public StepResult Step(double dt, ConnectorInputs inputs)
        {
            if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
            _in = inputs ?? new ConnectorInputs();
            _cur = new StepResult { Attempt = FailedAttempts };
            Time += dt;
            if (_in.PartnerLinkUp) _lastLinkTime = Time;

            // 1. commands (edge-triggered)
            var cmd = _in.Command;
            if (cmd != _lastCmd)
            {
                _lastCmd = cmd;
                switch (cmd)
                {
                    case DockingCommand.Approach: FireInternal(Trigger.CmdApproach); break;
                    case DockingCommand.Release: FireInternal(Trigger.CmdRelease); break;
                    case DockingCommand.EngageHooks: BeginEngage(); break;
                    case DockingCommand.Verify: DoVerify(); break;
                    case DockingCommand.TestLoad: DoLoadTest(); break;
                }
            }

            // 2. actuators
            StepHooks(dt);

            // 3. state-specific monitoring
            switch (State)
            {
                case ConnectorState.Approach: MonitorApproach(dt); break;
                case ConnectorState.MagnetCaptured:
                case ConnectorState.HookAEngaged:
                case ConnectorState.HookBEngaged:
                case ConnectorState.Partial:
                    MonitorHooks(dt); break;
                case ConnectorState.LatchedVerified:
                case ConnectorState.LoadValidated:
                case ConnectorState.VerificationLost:
                    MonitorLatched(); break;
                case ConnectorState.ReleaseRequested: MonitorRelease(dt); break;
            }

            _cur.State = State; _cur.SensedA = SensedA; _cur.SensedB = SensedB; _cur.HookPosA = HookPosA; _cur.HookPosB = HookPosB;
            _cur.Attempt = FailedAttempts;
            return _cur;
        }

        private void BeginEngage()
        {
            var s = State;
            if (s == ConnectorState.MagnetCaptured || s == ConnectorState.HookAEngaged || s == ConnectorState.HookBEngaged || s == ConnectorState.Partial)
            {
                if (!_engaging) { _engaging = true; _engageTimer = 0; }
            }
            else Emit(EventKind.IllegalTransition, State, State, null, "EngageHooks command not valid in this state");
        }

        // ------------------------------------------------------------------ approach / capture
        private void MonitorApproach(double dt)
        {
            if (_in.Obstruction)
            {
                _obstructionTimer += dt;
                if (_obstructionTimer >= A.ObstructionDebounce) { _everObstructed = true; FireInternal(Trigger.ObstructionDetected); }
                return; // magnets do not pull in past an obstruction
            }
            _obstructionTimer = 0;
            var p = _in.Pose;
            if (p.Gap > Model.MagnetRange) return;
            bool inside = Model.InsideEnvelope(p.Lateral, p.Yaw, p.Height, p.Pitch);
            if (inside && _in.ClosingSpeed <= A.MaxClosingSpeed)
            {
                var bands = new Bands(
                    Math.Abs(_rng.NextGaussian()) * Model.Compliance * A.StictionBandScale,
                    Math.Abs(_rng.NextGaussian()) * A.YawBandSd,
                    Math.Abs(_rng.NextGaussian()) * A.HeightBandSd);
                Settled = Model.Settle(p.Lateral, p.Yaw, p.Height, bands);
                Captured = _everCaptured = true;
                Emit(EventKind.Capture, State, ConnectorState.MagnetCaptured, Trigger.Captured, $"settled lat={Settled.Lateral:0.0000} yaw={Settled.Yaw:0.0000} h={Settled.Height:0.0000}");
                ClearHook(0); ClearHook(1);
                FireInternal(Trigger.Captured);
            }
            else if (p.Gap <= 0)
            {
                Emit(EventKind.Jam, State, ConnectorState.Free, Trigger.Jam, inside ? "closing speed too high: bounce" : "outside guide acceptance envelope");
                FireInternal(Trigger.Jam);
            }
        }

        // ------------------------------------------------------------------ hooks
        private Residual EffectiveResidual() => new Residual(
            Settled.Lateral + _in.Disturbance.Lateral, Settled.Yaw + _in.Disturbance.Yaw, Settled.Height + _in.Disturbance.Height);

        private readonly double[] _pos = new double[2];
        private readonly HookSeat?[] _plan = new HookSeat?[2];
        private readonly double[] _uFp = new double[2];

        private void StepHooks(double dt)
        {
            double rate = dt / Model.HookActuationTime;
            bool closeCmd = _engaging && (State == ConnectorState.MagnetCaptured || State == ConnectorState.HookAEngaged ||
                                           State == ConnectorState.HookBEngaged || State == ConnectorState.Partial);
            bool open = State == ConnectorState.ReleaseRequested;
            bool closeB = closeCmd;
            if (Model.Reciprocal)
            {
                // the partner's close command crosses the link: its hook starts latency seconds after ours
                _partnerCmdAge = closeCmd ? _partnerCmdAge + dt : 0;
                closeB = closeCmd && _partnerCmdAge >= A.PartnerLinkLatency;
            }
            MoveHook(0, _in.HookA, closeCmd, open, rate);
            MoveHook(1, _in.HookB, closeB, open, rate);
            HookPosA = _pos[0]; HookPosB = _pos[1];
            ObserveSensors();
        }

        /// <summary>Own hook (A) is read directly. In reciprocal mode HOOK_B is the partner's hook: its state is sampled each step, sent over the link
        /// (dropped while the link is down), and applied latency seconds later; a stale report reads as not engaged. Idempotent within a step.</summary>
        private void ObserveSensors()
        {
            SensedA = ReadSensor(0);
            bool raw = ReadSensor(1);
            if (!Model.Reciprocal) { SensedB = raw; return; }
            if (_in.PartnerLinkUp)
            {
                if (_bLine.Count > 0 && _bLine[_bLine.Count - 1].Key == Time) _bLine.RemoveAt(_bLine.Count - 1);
                _bLine.Add(new KeyValuePair<double, bool>(Time, raw));
            }
            while (_bLine.Count > 0 && _bLine[0].Key + A.PartnerLinkLatency <= Time + 1e-9) { _bHeld = _bLine[0].Value; _bLine.RemoveAt(0); }
            SensedB = _bHeld && PartnerObservationFresh;
        }

        private void MoveHook(int h, HookDrive drive, bool close, bool open, double rate)
        {
            if (drive == HookDrive.ForcedOpen) { ClearHook(h); _pos[h] = 0; return; }
            if (drive == HookDrive.Stalled) return;
            if (open)
            {
                if (_pos[h] > 0) { _pos[h] = Math.Max(0, _pos[h] - rate); if (_pos[h] < 1.0) DropMech(h); }
                _plan[h] = null; // a later close re-plans
                return;
            }
            if (!close || _evaluated[h]) return;
            if (_plan[h] == null)
            {
                // decide, once per close, what this hook will do given the geometry now (2 uniforms, always drawn)
                var g = Model.HookResidual(EffectiveResidual(), h);
                double uMiss = _rng.NextDouble(); _uFp[h] = _rng.NextDouble();
                _plan[h] = Model.Seat(g, uMiss);
                _planRatio[h] = g.Ratio;
            }
            double target = _plan[h] == HookSeat.JammedShort ? A.JamPosition : 1.0;
            _pos[h] = Math.Min(target, _pos[h] + rate);
            if (_pos[h] >= target - 1e-12) FinishClosing(h);
        }
        private readonly bool[] _evaluated = new bool[2];
        private readonly double[] _planRatio = new double[2];
        private readonly bool[] _fp = new bool[2];

        private void FinishClosing(int h)
        {
            var seat = _plan[h].Value;
            _evaluated[h] = true;
            if (h == 0) { MechA = seat; _seatedA = seat == HookSeat.Seated; } else { MechB = seat; _seatedB = seat == HookSeat.Seated; }
            _fp[h] = seat == HookSeat.JammedShort && _uFp[h] < A.SensorFalsePositiveProbability;
            string who = h == 0 ? "A" : "B";
            if (seat == HookSeat.ClosedOnNothing) Emit(EventKind.HookClosedOnNothing, State, State, null, $"hook {who} reached closed stop without seating (ratio {_planRatio[h]:0.00})");
            else if (seat == HookSeat.JammedShort) Emit(EventKind.Jam, State, State, null, $"hook {who} jammed short (ratio {_planRatio[h]:0.00})");
            if (_fp[h]) Emit(EventKind.FalsePositiveSensor, State, State, null, $"hook {who} sensor reads engaged while jammed short");
        }

        private void DropMech(int h)
        {
            if (h == 0) { MechA = null; _seatedA = false; } else { MechB = null; _seatedB = false; }
            _fp[h] = false; _evaluated[h] = false;
        }

        private void ClearHook(int h) { DropMech(h); _plan[h] = null; }

        private bool ReadSensor(int h)
        {
            var m = h == 0 ? MechA : MechB;
            if (m == HookSeat.Seated || m == HookSeat.ClosedOnNothing) return _pos[h] >= 1.0 - 1e-9;
            return _fp[h];
        }

        private void MonitorHooks(double dt)
        {
            // sensor edges -> triggers
            bool prevA = _prevSensedA, prevB = _prevSensedB;
            _prevSensedA = SensedA; _prevSensedB = SensedB;
            if (SensedA && !prevA) { Emit(EventKind.HookSensed, State, State, Trigger.HookASensed, "hook A sensed engaged"); FireInternal(Trigger.HookASensed); }
            if (SensedB && !prevB) { Emit(EventKind.HookSensed, State, State, Trigger.HookBSensed, "hook B sensed engaged"); FireInternal(Trigger.HookBSensed); }
            if ((!SensedA && prevA) || (!SensedB && prevB)) FireInternal(Trigger.HookSignalLost);
            if (_engaging && (State == ConnectorState.MagnetCaptured || State == ConnectorState.HookAEngaged || State == ConnectorState.HookBEngaged))
            {
                _engageTimer += dt;
                bool both = SensedA && SensedB;
                if (!both && _engageTimer >= Model.HookActuationTime * A.HookTimeoutFactor)
                {
                    _engaging = false;
                    FireInternal(Trigger.HookTimeout);
                }
            }
        }
        private bool _prevSensedA, _prevSensedB;

        // ------------------------------------------------------------------ verification
        private Residual Measure()
        {
            var r = EffectiveResidual();
            return new Residual(r.Lateral + _rng.NextGaussian() * A.PoseMeasureSdLateral,
                                r.Yaw + _rng.NextGaussian() * A.PoseMeasureSdYaw,
                                r.Height + _rng.NextGaussian() * A.PoseMeasureSdHeight);
        }

        private double WorstRatio(Residual r) => Math.Max(Model.HookResidual(r, 0).Ratio, Model.HookResidual(r, 1).Ratio);

        private void DoVerify()
        {
            if (State != ConnectorState.HookAEngaged && State != ConnectorState.HookBEngaged && State != ConnectorState.VerificationLost)
            { FireInternal(Trigger.VerifyPassed); return; }
            if (!(SensedA && SensedB))
            {
                Emit(EventKind.GuardRejected, State, State, Trigger.VerifyPassed, "verify requires both hook signals");
                return;
            }
            double ratio = WorstRatio(Measure());
            if (ratio <= 1.0) { _verified = true; FireInternal(Trigger.VerifyPassed); }
            else
            {
                _falseDeclared = true; _verified = false;
                FireInternal(Trigger.FalseEngagementDetected);
                Emit(EventKind.VerifyFailed, State, State, Trigger.FalseEngagementDetected, $"hook sensors say engaged but measured pose residual is {ratio:0.00}x tolerance");
            }
        }

        private void MonitorLatched()
        {
            if (State == ConnectorState.VerificationLost) return;
            // overload: any seated hook above rated slips out
            var loads = _in.AppliedLoad.IsZero ? default(HookLoads) : Model.CompliantOrStatic(_in.AppliedLoad, _seatedA, _seatedB, EffectiveResidual());
            if (!_in.AppliedLoad.IsZero)
            {
                if (loads.OverloadA) { Emit(EventKind.HookOverload, State, State, null, $"hook A load {loads.A:0.0} N > rated {Model.HookRatedLoad} N: slips"); SlipHook(0); }
                if (loads.OverloadB) { Emit(EventKind.HookOverload, State, State, null, $"hook B load {loads.B:0.0} N > rated {Model.HookRatedLoad} N: slips"); SlipHook(1); }
            }
            ObserveSensors();
            bool lost = !(SensedA && SensedB);
            if (!lost) lost = WorstRatio(Measure()) > A.VerificationLossFactor;
            if (lost) { _verified = false; _loadValidated = false; FireInternal(Trigger.VerificationLostDetected); }
        }

        private void SlipHook(int h) => DropMech(h);

        private void DoLoadTest()
        {
            if (State != ConnectorState.LatchedVerified && State != ConnectorState.LoadValidated) { FireInternal(Trigger.LoadTestPassed); return; }
            var load = _in.AppliedLoad.IsZero ? new LoadCase(A.TestPullTension, 0, 0) : _in.AppliedLoad;
            var loads = Model.CompliantOrStatic(load, _seatedA, _seatedB, EffectiveResidual());
            int n = (_seatedA ? 1 : 0) + (_seatedB ? 1 : 0);
            double stiffnessFraction = n / 2.0;
            if (loads.AnyOverload || stiffnessFraction < A.LoadConsistencyMinFraction)
            {
                _loadValidated = false;
                Emit(EventKind.HookOverload, State, State, null, $"load check failed: connection stiffness {stiffnessFraction:0.00} of nominal, overload={loads.AnyOverload}");
                FireInternal(Trigger.LoadTestSlip);
            }
            else { _loadValidated = true; FireInternal(Trigger.LoadTestPassed); }
        }

        // ------------------------------------------------------------------ release
        private void MonitorRelease(double dt)
        {
            _releaseTimer += dt;
            if (HookPosA <= 0 && HookPosB <= 0) { Emit(EventKind.Released, State, ConnectorState.Released, Trigger.ReleaseComplete, "both hooks open"); FireInternal(Trigger.ReleaseComplete); ResetAfterRelease(); }
            else if (_releaseTimer >= Model.HookActuationTime * A.ReleaseTimeoutFactor) FireInternal(Trigger.ReleaseTimeout);
        }

        private void ResetAfterRelease()
        {
            _prevSensedA = _prevSensedB = false; _engaging = false;
            _verified = false; _loadValidated = false;
            ClearHook(0); ClearHook(1);
            SensedA = SensedB = false; _bLine.Clear(); _bHeld = false;
        }
        // (hooks are already open when RELEASED is entered)

        // ------------------------------------------------------------------ summary
        private AttemptSummary BuildSummary() => new AttemptSummary
        {
            EverCaptured = _everCaptured, EverObstructed = _everObstructed, Verified = _verified || ConnectorStateNames.IsVerified(State),
            LoadValidated = _loadValidated, SeatedA = _seatedA, SeatedB = _seatedB, SensedA = SensedA, SensedB = SensedB,
            FalseEngagementDeclared = _falseDeclared || State == ConnectorState.FalseEngagement, FinalState = State,
        };
    }

    public static class ContactModelExtensions
    {
        /// <summary>Uses the compliant split when both hooks are seated, otherwise the statically determinate split.</summary>
        public static HookLoads CompliantOrStatic(this ContactModel m, LoadCase l, bool seatedA, bool seatedB, Residual r)
            => seatedA && seatedB ? m.CompliantSplit(l, r) : m.StaticSplit(l, seatedA, seatedB);
    }
}
