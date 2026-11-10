using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;

namespace Crane.Fleet
{
    public enum TaskState { Pending, Running, Succeeded, Failed, Skipped }

    /// <summary>Outcome of one task as reported by the executive.</summary>
    public sealed class TaskRecord
    {
        public TaskSpec Spec; public TaskState State; public int Attempts; public double Started = double.NaN, Ended = double.NaN;
        public FailureClass Class; public string Reason;
        public TaskBehavior Behavior;
    }

    /// <summary>Behaviour of one running task (a small hierarchical state machine). Tick returns Running, Succeeded or Failed.</summary>
    public abstract class TaskBehavior
    {
        public FailureClass FailClass; public string FailReason; public string Phase = "start";
        protected FleetAutonomy F; protected TaskSpec S;
        public void Bind(FleetAutonomy f, TaskSpec s) { F = f; S = s; }
        /// <summary>Rovers this task needs exclusively (resolved to their current agents each tick).</summary>
        public abstract IEnumerable<string> Rovers();
        public virtual void Start() { }
        public abstract TaskState Tick(double dt);
        public virtual void Abort() { }
        protected TaskState Fail(FailureClass c, string why) { FailClass = c; FailReason = why; return TaskState.Failed; }
        protected void SetPhase(string p) { if (p != Phase) { F.Log("phase", S.Id, null, Phase + " -> " + p); Phase = p; } }
    }

    /// <summary>
    /// Mission executive: runs the task list as a dependency DAG. A task starts when its "after" tasks succeeded and none of its
    /// rovers is used by another running task. A failed task is retried (fresh behaviour = replan) up to its retry budget; then it is
    /// reported with a failure class and every dependent task is skipped. The mission ends when no task can make progress.
    /// </summary>
    public sealed class MissionExecutive
    {
        private readonly FleetAutonomy _f;
        public readonly MissionSpec Mission;
        public readonly List<TaskRecord> Records = new List<TaskRecord>();
        public bool Done { get; private set; }
        /// <summary>True only after the host confirmed the outcome (<see cref="ConfirmByHost"/>): the library never reports mission
        /// success from its own belief alone.</summary>
        public bool Succeeded { get; private set; }
        /// <summary>Every task succeeded according to the library's own belief; the host must check the world and confirm or reject.</summary>
        public bool BelievedComplete { get; private set; }
        public bool HostConfirmed { get; private set; }
        public string HostVerdict { get; private set; }

        /// <summary>Host's verdict on a believed-complete mission (evaluator delivery check, operator). A rejection becomes the
        /// mission's failure with class HostRejected (a divergence between belief and world worth logging, promotion_v0 §3).</summary>
        public void ConfirmByHost(bool ok, string reason)
        {
            if (!BelievedComplete) throw new InvalidOperationException("ConfirmByHost: the mission is not believed complete");
            HostConfirmed = ok; Succeeded = ok; HostVerdict = reason;
            if (!ok) { FirstFailure = "host rejected: " + reason; FirstFailureClass = FailureClass.HostRejected; }
            _f.Log(ok ? "mission_confirmed" : "mission_rejected_by_host", null, null, reason ?? "");
        }
        public FailureClass FirstFailureClass { get; private set; }
        public string FirstFailure { get; private set; }

        public MissionExecutive(FleetAutonomy f, MissionSpec m)
        {
            _f = f; Mission = m;
            foreach (var t in m.Tasks) Records.Add(new TaskRecord { Spec = t, State = TaskState.Pending });
        }

        public TaskRecord Record(string id) { foreach (var r in Records) if (r.Spec.Id == id) return r; return null; }

        public static TaskBehavior Create(TaskSpec s)
        {
            switch (s.Type)
            {
                case "goto": return new GotoBehavior();
                case "explore": return new ExploreBehavior();
                case "lift": return new LiftBehavior();
                case "rendezvous": return new RendezvousBehavior();
                case "dock": return new DockBehavior();
                case "assemble": return new AssembleBehavior();
                case "transport": return new TransportBehavior();
                case "separate": return new SeparateBehavior();
                case "handoff": return new HandoffBehavior();
                case "wait": return new WaitBehavior();
                default: throw new ArgumentException("unknown task type " + s.Type);
            }
        }

        public void Tick(double dt)
        {
            if (Done) return;
            if (_f.Now < _f.A.StartupStillTime) return;   // hold still: zero-velocity updates calibrate the gyro biases
            // start eligible tasks
            foreach (var r in Records)
            {
                if (r.State != TaskState.Pending) continue;
                bool ready = true, blocked = false;
                foreach (var a in r.Spec.After)
                {
                    var d = Record(a);
                    if (d.State == TaskState.Failed || d.State == TaskState.Skipped) blocked = true;
                    if (d.State != TaskState.Succeeded) ready = false;
                }
                if (blocked) { r.State = TaskState.Skipped; r.Reason = "dependency failed"; _f.Log("task_skipped", r.Spec.Id, null, r.Reason); continue; }
                if (!ready) continue;
                var b = Create(r.Spec); b.Bind(_f, r.Spec);
                bool busy = false;
                foreach (var rv in b.Rovers()) { var ag = _f.AgentOf(rv); if (ag.OwnerTask != null) busy = true; }
                if (busy) continue;
                StartTask(r, b);
            }
            // tick running tasks
            foreach (var r in Records)
            {
                if (r.State != TaskState.Running) continue;
                foreach (var rv in r.Behavior.Rovers()) _f.AgentOf(rv).OwnerTask = r.Spec.Id;
                TaskState s;
                if (_f.Now - r.Started > r.Spec.Timeout) s = Fail(r, FailureClass.Timeout, "task timeout " + r.Spec.Timeout + " s");
                else s = r.Behavior.Tick(dt);
                if (s == TaskState.Succeeded)
                {
                    r.State = TaskState.Succeeded; r.Ended = _f.Now; Release(r);
                    _f.Log("task_succeeded", r.Spec.Id, null, r.Behavior.Phase);
                }
                else if (s == TaskState.Failed)
                {
                    r.Behavior.Abort();
                    _f.Log("task_attempt_failed", r.Spec.Id, null, $"{r.Behavior.FailClass}: {r.Behavior.FailReason} (attempt {r.Attempts})");
                    Release(r);
                    if (r.Attempts <= r.Spec.Retries && r.Behavior.FailClass != FailureClass.Configuration && r.Behavior.FailClass != FailureClass.Support)
                    {
                        var b = Create(r.Spec); b.Bind(_f, r.Spec); StartTask(r, b);
                    }
                    else
                    {
                        r.State = TaskState.Failed; r.Ended = _f.Now; r.Class = r.Behavior.FailClass; r.Reason = r.Behavior.FailReason;
                        if (FirstFailure == null) { FirstFailure = r.Spec.Id + ": " + r.Reason; FirstFailureClass = r.Class; }
                        _f.Log("task_failed", r.Spec.Id, null, $"{r.Class}: {r.Reason}");
                    }
                }
            }
            bool anyActive = false, allOk = true;
            foreach (var r in Records)
            {
                if (r.State == TaskState.Pending || r.State == TaskState.Running) anyActive = true;
                if (r.State != TaskState.Succeeded) allOk = false;
            }
            if (!anyActive)
            {
                Done = true; BelievedComplete = allOk; Succeeded = false;
                _f.Log(allOk ? "mission_believed_complete" : "mission_failed", null, null, FirstFailure ?? "awaiting host confirmation");
            }
        }

        private TaskState Fail(TaskRecord r, FailureClass c, string why) { r.Behavior.FailClass = c; r.Behavior.FailReason = why; r.Attempts = int.MaxValue / 2; return TaskState.Failed; }

        private void StartTask(TaskRecord r, TaskBehavior b)
        {
            r.Behavior = b; r.State = TaskState.Running; r.Attempts++; if (double.IsNaN(r.Started)) r.Started = _f.Now;
            foreach (var rv in b.Rovers()) _f.AgentOf(rv).OwnerTask = r.Spec.Id;
            _f.Log("task_started", r.Spec.Id, null, r.Spec.Type + " attempt " + r.Attempts);
            b.Start();
        }

        private void Release(TaskRecord r)
        {
            foreach (var ag in _f.Agents) if (ag.OwnerTask == r.Spec.Id) { ag.OwnerTask = null; ag.Tracker.Stop(); }
        }
    }
}
