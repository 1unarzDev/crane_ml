using System.Collections.Generic;

namespace Sim.Course {
    /// <summary>One row of the shared constraint spec (§3): an id, the task type it judges,
    /// its source (H page or D), a predicate from the shared vocabulary with its args, and the
    /// limit {min, max, in} with a tolerance (negative tol makes a bound strict).</summary>
    public sealed class ConstraintRow {
        public string Id, Task, Cite, Tag, Predicate;
        public JObj Args, Limit, When;
        public double Tol;
        public bool Final;  // judged once the whole course exists (e.g. paths between tasks)
    }

    /// <summary>Tools/RoboBoat/constraints/&lt;family&gt;.json, the single source both the C#
    /// evaluator and validate.py read.</summary>
    public sealed class ConstraintSpec {
        public string Family;
        public readonly List<ConstraintRow> Rows = new List<ConstraintRow>();

        public static ConstraintSpec Parse(string json) {
            var doc = (JObj)Json.Parse(json);
            var spec = new ConstraintSpec { Family = doc.Str("family") };
            foreach (JObj r in doc.Arr("rows"))
                spec.Rows.Add(new ConstraintRow {
                    Id = r.Str("id"), Task = r.Str("task"), Cite = r.Str("cite") ?? "", Tag = r.Str("tag"),
                    Predicate = r.Str("predicate"), Args = r.Obj("args") ?? new JObj(),
                    Limit = r.Obj("limit") ?? new JObj(), When = r.Obj("when"),
                    Tol = r.Num("tol", 0), Final = r["final"] is bool f && f
                });
            return spec;
        }
    }

    /// <summary>A row's verdict, as written to truth <c>constraints[]</c>.</summary>
    public sealed class RowResult {
        public string Id, Tag;
        public bool Pass;
        public object Value;   // double, string, or null when the row does not apply
        public JObj Limit;

        public JObj ToJson() => new JObj().Set("id", Id).Set("tag", Tag).Set("pass", Pass)
            .Set("value", Value is double d ? System.Math.Round(d, 4) : Value).Set("limit", Limit);
    }
}
