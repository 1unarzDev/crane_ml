using System;
using System.Collections.Generic;

namespace Sim.Utils.Performance {
    /// <summary>One timed event for the trajectory log (crane-traj/1): a kind (contact, gate,
    /// task_start, task_end, failure, note, ...) and flat string or number fields.</summary>
    public readonly struct CraneTrajEvent {
        public readonly string Kind;
        public readonly long Tick;
        public readonly IReadOnlyDictionary<string, object> Fields;

        internal CraneTrajEvent(string kind, long tick, IReadOnlyDictionary<string, object> fields) {
            Kind = kind;
            Tick = tick;
            Fields = fields;
        }
    }

    /// <summary>
    /// The seam through which any domain code reports events for the trajectory log, without
    /// knowing whether a recorder runs (<c>--crane-traj</c>): <c>CraneTrajEvents.Emit("gate",
    /// ("task", "entry"), ("index", 0))</c>. Stamped with the simulation tick.
    /// </summary>
    public static class CraneTrajEvents {
        private static readonly object s_Lock = new();
        private static Action<CraneTrajEvent> s_Observers;

        public static bool Recording {
            get { lock (s_Lock) return s_Observers != null; }
        }

        public static void Emit(string kind, params (string key, object value)[] fields) {
            Action<CraneTrajEvent> observers;
            lock (s_Lock) observers = s_Observers;
            if (observers == null || string.IsNullOrWhiteSpace(kind)) return;
            var map = new Dictionary<string, object>();
            foreach (var (key, value) in fields)
                if (!string.IsNullOrEmpty(key) && key != "t" && key != "kind") map[key] = value;
            observers(new CraneTrajEvent(kind, CraneRuntimeMetrics.SimulationTick, map));
        }

        public static void Subscribe(Action<CraneTrajEvent> observer) {
            if (observer == null) throw new ArgumentNullException(nameof(observer));
            lock (s_Lock) s_Observers += observer;
        }

        public static void Unsubscribe(Action<CraneTrajEvent> observer) {
            if (observer == null) return;
            lock (s_Lock) s_Observers -= observer;
        }
    }
}
