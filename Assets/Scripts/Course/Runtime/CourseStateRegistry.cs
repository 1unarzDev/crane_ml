using System;
using System.Collections.Generic;
using Sim.Utils.ROS;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Runs every state program against sim time (seconds since the course was ready),
    /// applies changes to the objects, and logs each change to states.jsonl {t_sim, id, value}.</summary>
    public sealed class CourseStateRegistry : MonoBehaviour {
        readonly List<(IStatefulCourseObject obj, JObj program)> items = new List<(IStatefulCourseObject, JObj)>();
        double start;
        string statesPath;
        public event Action<string, string, double> StateChanged;

        public double Now => Clock.FrameStartTimeInSeconds - start;

        public void Init(JObj truth, IReadOnlyDictionary<string, IStatefulCourseObject> objects, string path) {
            start = Clock.FrameStartTimeInSeconds;
            statesPath = path;
            foreach (JObj s in truth.Arr("states")) {
                if (!objects.TryGetValue(s.Str("id"), out var obj)) continue;
                items.Add((obj, s.Obj("program")));
                obj.Changed += o => {
                    CourseTruthWriter.AppendState(statesPath, Now, o.Id, o.State);
                    StateChanged?.Invoke(o.Id, o.State, Now);
                };
                obj.Apply(s.Str("initial"));
            }
        }

        void FixedUpdate() {
            double t = Now;
            foreach (var (obj, program) in items) obj.Apply(Programs.ValueAt(program, t));
        }

        public JObj Snapshot() {
            var states = new JObj();
            foreach (var (obj, _) in items) states[obj.Id] = obj.State;
            return new JObj().Set("t_sim", Math.Round(Now, 3)).Set("states", states);
        }
    }
}
