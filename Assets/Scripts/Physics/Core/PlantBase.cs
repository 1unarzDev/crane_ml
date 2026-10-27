using System.Collections.Generic;
using Sim.Utils;
using Sim.Utils.Performance;
using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>Rate-limited plant events (non-finite input, out of envelope, ...), logged as
    /// <c>CRANE_PLANT_EVENT code=...</c>. Counts stay readable for tests.</summary>
    public static class PlantEvents {
        private const int LogLimit = 20;
        private static readonly Dictionary<string, int> counts = new();

        public static void Report(string code, string detail) {
            counts.TryGetValue(code, out int n);
            counts[code] = ++n;
            if (n <= LogLimit) Debug.LogWarning($"CRANE_PLANT_EVENT code={code} count={n} {detail}");
        }

        public static int Count(string code) => counts.TryGetValue(code, out int n) ? n : 0;
        public static void Clear() => counts.Clear();
    }

    /// <summary>
    /// One body's physics in a fixed order each tick: snapshot → actuators → force models →
    /// <see cref="IInertiaCoupling"/> → one wrench applied at the centre of mass. This replaces
    /// independent force MonoBehaviours whose result depended on script order. Runs after the
    /// default execution order so commands set in FixedUpdate this tick are used this tick.
    /// </summary>
    [DefaultExecutionOrder(100)]
    public abstract class PlantBase : MonoBehaviour, ICraneEpisodeResettable {
        protected IPhysicsBody body;
        protected IBodyFrame frame;
        protected IMedium medium;
        protected IInertiaCoupling coupling = new NoCoupling();
        [SerializeReference] protected List<IForceModel> forces = new();
        protected readonly List<IActuator> actuators = new();

        /// <summary>The wrench applied last tick (body FLU) and the state it came from.</summary>
        public Wrench LastWrench { get; private set; }
        public PlantState LastState { get; private set; }
        public IReadOnlyList<IActuator> Actuators => actuators;
        public IReadOnlyList<IForceModel> Forces => forces;

        public virtual int ResetPriority => -30;
        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context, CraneEpisodeResetPhase phase) {
            if (phase == CraneEpisodeResetPhase.BeforePhysics) ResetDynamics();
        }

        /// <summary>Clears actuator and force-model state and the last tick, for a body that was
        /// teleported to rest.</summary>
        public void ResetDynamics() {
            foreach (IActuator actuator in actuators) actuator.ResetState();
            foreach (IForceModel force in forces) force.ResetState();
            LastWrench = default;
            LastState = default;
        }

        public IActuator Actuator(string name) => actuators.Find(a => a.Name == name);

        /// <summary>The body's centre of mass, Unity world [m].</summary>
        protected virtual Vector3 CenterOfMass() => body.transform.TransformPoint(body.centerOfMass);

        protected virtual void FixedUpdate() {
            if (body == null || frame == null) return;
            Step(Time.fixedDeltaTime, Time.timeAsDouble);
        }

        /// <summary>One tick: returns false (and applies nothing) on a non-finite state or wrench.</summary>
        public bool Step(float dt, double time) {
            Vector3 cg = CenterOfMass();
            Vector3 rootCg = body.transform.TransformPoint(body.centerOfMass);
            Vector3 velocity = body.linearVelocity + Vector3.Cross(body.angularVelocity, cg - rootCg);
            Vector3 flow = medium?.FlowAt(cg, time) ?? Vector3.zero;
            var state = new PlantState(frame, cg, frame.BodyLinear(velocity),
                frame.BodyAngular(body.angularVelocity), frame.BodyLinear(flow), dt, time);
            if (!state.IsFinite) {
                PlantEvents.Report("NONFINITE_STATE", $"plant={name} v={velocity} w={body.angularVelocity}");
                LastWrench = default;
                return false;
            }
            Wrench hydro = default, direct = default, jacobian = default;
            foreach (IActuator actuator in actuators) actuator.Accumulate(state, ref hydro);
            foreach (IForceModel force in forces) {
                if (force.Route == WrenchRoute.Hydro) force.Accumulate(state, ref hydro);
                else force.Accumulate(state, ref direct);
                if (force is IDampingModel damping) damping.AccumulateJacobian(state, ref jacobian);
            }
            Wrench total = coupling.Map(state, hydro, jacobian) + direct;
            LastState = state;
            if (!total.IsFinite) {
                PlantEvents.Report("NONFINITE_WRENCH", $"plant={name} hydro={hydro} direct={direct}");
                LastWrench = default;
                return false;
            }
            LastWrench = total;
            body.AddForceAtPosition(frame.UnityLinear(total.Force), cg);
            body.AddTorque(frame.UnityAngular(total.Torque));
            return true;
        }
    }
}
