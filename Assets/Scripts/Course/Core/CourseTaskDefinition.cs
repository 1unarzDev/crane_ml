using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Marker for a task's parameter class: public <see cref="ParamRange"/> fields,
    /// each with a <see cref="CourseParamAttribute"/>.</summary>
    public interface ITaskParams { }

    /// <summary>Everything one task needs (§1): its placement rule, its layout, and which rows
    /// of the shared constraint spec judge it. A new year's task is one subclass plus an asset.</summary>
    public abstract class CourseTaskDefinition : ScriptableObject {
        [SerializeField, Tooltip("Unique within the family; defaults to the task type.")]
        string taskId = "";

        public string TaskId => string.IsNullOrEmpty(taskId) ? TaskType : taskId;
        public abstract string TaskType { get; }
        public abstract string Cite { get; }
        public abstract IReadOnlyList<ParamEntry> ParamEntries { get; }

        /// <summary>Where this task's anchor goes, relative to the tasks placed before it.</summary>
        public abstract Pose2 Place(PlacementContext c);

        /// <summary>The task's objects, gates, bays, states and semantics in the course frame.</summary>
        public abstract TaskLayout Generate(TaskContext c);

        /// <summary>The spec rows that judge this task (task classes reference rows by task,
        /// never re-implement them, so both evaluators share one meaning).</summary>
        public virtual IEnumerable<ConstraintRow> Constraints(ConstraintSpec spec) =>
            spec.Rows.Where(r => r.Task == TaskType && r.Predicate != "path");

        public virtual IEnumerable<ConstraintRow> PathChecks(ConstraintSpec spec, TaskLayout layout) =>
            spec.Rows.Where(r => r.Task == TaskType && r.Predicate == "path");

        internal void SetTaskId(string id) => taskId = id;
    }

    public abstract class CourseTaskDefinition<TParams> : CourseTaskDefinition
        where TParams : class, ITaskParams, new() {
        [SerializeField] TParams parameters;

        public TParams Params {
            get => parameters ?? (parameters = new TParams());
            set => parameters = value;
        }

        public override IReadOnlyList<ParamEntry> ParamEntries => ParamReflection.Entries(Params);
    }
}
