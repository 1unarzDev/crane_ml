using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Course {
    /// <summary>Evaluates spec rows against a truth tree. Used by the generator as a self-check
    /// and by tests; Tools/RoboBoat/validate.py is the independent twin and must agree.</summary>
    public static class ConstraintEvaluator {
        public static List<RowResult> Evaluate(ConstraintSpec spec, JObj truth, Func<ConstraintRow, bool> which = null) =>
            spec.Rows.Where(r => which == null || which(r)).Select(r => Evaluate(r, truth)).ToList();

        public static RowResult Evaluate(ConstraintRow row, JObj truth) {
            var result = new RowResult { Id = row.Id, Tag = row.Tag, Limit = row.Limit };
            if (row.When != null && !Matches(truth, row.When)) { result.Pass = true; return result; }
            JObj limit = row.Limit;
            List<object> values;
            try { values = Values(row, truth, ref limit); }
            catch (KeyNotFoundException missing) { result.Value = "missing: " + missing.Message; return result; }
            result.Limit = limit;
            // reduce "min"|"max": judge only the smallest/largest value (e.g. at least one spine rooted).
            string reduce = row.Args.Str("reduce");
            if (reduce != null && values.Count > 0) {
                var nums = values.OfType<double>().ToList();
                values = new List<object> { reduce == "max" ? nums.Max() : nums.Min() };
            }
            if (values.Count == 0) { result.Pass = true; return result; }
            object worst = null;
            double worstMargin = double.MaxValue;
            foreach (object v in values) {
                double m = Margin(v, limit, row.Tol);
                if (m < worstMargin) { worstMargin = m; worst = v; }
            }
            result.Value = worst;
            result.Pass = worstMargin >= 0;
            return result;
        }

        /// <summary>How far inside the limit a value is (negative outside).</summary>
        public static double Margin(object v, JObj limit, double tol) {
            if (limit.Has("in")) return limit.Arr("in").Any(x => Same(x, v)) ? 1 : -1;
            if (!(v is double d)) return -1;
            double m = double.MaxValue;
            if (limit.Has("min")) m = Math.Min(m, d - limit.Num("min") + tol);
            if (limit.Has("max")) m = Math.Min(m, limit.Num("max") + tol - d);
            return m;
        }

        static List<object> Values(ConstraintRow row, JObj t, ref JObj limit) {
            JObj a = row.Args;
            var vessel = VesselProfile.FromJson(t.Obj("vessel"));
            if (a["plus_beam"] is bool pb && pb) limit = new JObj().Set("min", limit.Num("min") + vessel.Beam);
            switch (row.Predicate) {
                case "distance":
                    if (a.Has("select")) {
                        P2 from = Ref(t, a.Str("a")).At;
                        return Select(t, a).Select(e => (object)Surface(e, from)).ToList();
                    }
                    return One(P2.Dist(Ref(t, a.Str("a")).At, Ref(t, a.Str("b")).At));
                // Recomputed from the posts, never read from the gate record.
                case "gate_cc":
                    return Select(t, a).Select(g => (object)P2.Dist(Xy(Element(t, g.Str("port"))), Xy(Element(t, g.Str("starboard"))))).ToList();
                case "gate_clear":
                    return Select(t, a).Select(g => {
                        JObj p = Element(t, g.Str("port")), s = Element(t, g.Str("starboard"));
                        return (object)(P2.Dist(Xy(p), Xy(s)) - p.Num("radius") - s.Num("radius"));
                    }).ToList();
                case "gate_side":
                    return Select(t, a).Select(g => {
                        JObj port = Element(t, g.Str("port"));
                        double lateral = P2.Dot(Xy(port) - Xy(g), P2.Dir(g.Num("normal_yaw")).Left);
                        return (object)(port.Str("kind").Contains(a.Str("port_color")) ? lateral : -Math.Abs(lateral));
                    }).ToList();
                case "along": {
                    if (a.Has("select")) return Select(t, a).Select(e => (object)Along(t, a.Str("gate"), "elem:" + e.Str("id"))).ToList();
                    double s = Along(t, a.Str("gate"), a.Str("point"));
                    if (a.Has("relative_to")) s /= Along(t, a.Str("gate"), a.Str("relative_to"));
                    return One(s);
                }
                case "lateral": {
                    var g = Ref(t, a.Str("gate"));
                    return One(Math.Abs(P2.Dot(Ref(t, a.Str("point")).At - g.At, P2.Dir(g.Yaw).Left)));
                }
                case "count": return One(Select(t, a).Count);
                case "angle_between": {
                    double d = Math.Abs(CourseFrame.Wrap(Ref(t, a.Str("a")).Yaw - Ref(t, a.Str("b")).Yaw - (a["antiparallel"] is bool ap && ap ? Math.PI : 0)));
                    // axial: a box's yaw is an axis (yaw and yaw + pi are the same box)
                    if (a["axial"] is bool ax && ax) d = Math.Min(d, Math.PI - d);
                    return One(d * 180 / Math.PI);
                }
                case "inside": {
                    // edge "shore": signed distance to the shore line only (water side positive)
                    Func<P2, double> margin;
                    if (a.Str("edge") == "shore") {
                        var (s0, s1, sign) = Shore(t);
                        margin = q => sign * Geometry.Cross(s0, s1, q) / (s1 - s0).Length;
                    }
                    else {
                        var poly = Points(t.Arr("boundary"));
                        margin = q => Geometry.InsideMargin(poly, q);
                    }
                    return Select(t, a).Select(e => (object)ShapeInside(margin, e)).ToList();
                }
                // Angle between a gate's normal (from its posts) and the bearing from a point to the gate centre.
                case "facing": {
                    var (at, yaw) = GatePose(t, a.Str("gate"));
                    P2 from = Ref(t, a.Str("from")).At;
                    return One(Math.Abs(CourseFrame.Wrap(yaw - Math.Atan2(at.Y - from.Y, at.X - from.X))) * 180 / Math.PI);
                }
                // Curvature of the circle through consecutive gates tangent to their normals (poses from their
                // posts): 2 sin(turn / 2) / chord, in order.
                case "curvature": {
                    var gates = Select(t, a).OrderBy(g => g.Num("order")).Select(g => GatePose(t, "gate:" + g.Str("id"))).ToList();
                    var list = new List<object>();
                    for (int i = 0; i + 1 < gates.Count; i++)
                        list.Add(2 * Math.Sin(Math.Abs(CourseFrame.Wrap(gates[i + 1].yaw - gates[i].yaw)) / 2) / Math.Max(1e-9, P2.Dist(gates[i].at, gates[i + 1].at)));
                    return list;
                }
                // 0 when map[state of elem at t=0] equals path in every selected item, else 1.
                case "state_map": {
                    string state = Element(t, a.Str("elem")).Str("state") ?? throw new KeyNotFoundException(a.Str("elem") + " state");
                    JObj st = t.Arr("states").Cast<JObj>().FirstOrDefault(x => x.Str("id") == state) ?? throw new KeyNotFoundException(state);
                    object want = a.Obj("map")[st.Str("initial") ?? ""];
                    return Select(t, a).Select(x => (object)(want != null && Same(want, Json.Path(x, a.Str("path"))) ? 0.0 : 1.0)).ToList();
                }
                // Each selected element's centre distance to the nearest element of another task
                // (to "other_tasks", the default) or to the nearest other selected element ("selected").
                case "nearest": {
                    var items = Select(t, a);
                    var all = t.Arr("elements").Cast<JObj>().ToList();
                    bool among = a.Str("to") == "selected";
                    return items.Select(e => (object)(among ? items : all)
                        .Where(o => among ? !ReferenceEquals(o, e) : o.Str("task") != e.Str("task"))
                        .Select(o => P2.Dist(Xy(o), Xy(e))).DefaultIfEmpty(double.NaN).Min()).Where(v => !double.IsNaN((double)v)).ToList();
                }
                // Shortfall of distinct tasks owning the selected elements' nearest other-task objects
                // below min(min_tasks, number selected).
                case "spread_tasks": {
                    var items = Select(t, a);
                    var all = t.Arr("elements").Cast<JObj>().ToList();
                    var owners = new HashSet<string>();
                    foreach (var e in items) {
                        var near = all.Where(o => o.Str("task") != e.Str("task")).OrderBy(o => P2.Dist(Xy(o), Xy(e))).FirstOrDefault();
                        if (near != null) owners.Add(near.Str("task"));
                    }
                    return One(Math.Max(0, Math.Min((int)a.Num("min_tasks"), items.Count) - owners.Count));
                }
                // Least distance from a point (or from each selected element) to a polyline through the
                // given references, then through every gate of "through_gates" (a task) in order.
                case "polyline_distance": {
                    var pts = a.Arr("through").Select(r => Ref(t, (string)r).At).ToList();
                    if (a.Has("through_gates")) {
                        var ids = TaskIds(t, a.Str("through_gates"));
                        pts.AddRange(t.Arr("gates").Cast<JObj>().Where(g => ids.Contains(g.Str("task"))).OrderBy(g => g.Num("order")).Select(Xy));
                    }
                    double Least(P2 q) {
                        double best = pts.Count == 1 ? P2.Dist(q, pts[0]) : double.MaxValue;
                        for (int i = 0; i + 1 < pts.Count; i++) best = Math.Min(best, SegmentDistance(q, pts[i], pts[i + 1]));
                        return best;
                    }
                    if (a.Has("select")) return Select(t, a).Select(e => (object)Least(Xy(e))).ToList();
                    return One(Least(Ref(t, a.Str("point")).At));
                }
                case "clearance":
                    if (a.Has("gates")) return One(GateClearance(t, Select(t, a), Select(t, new JObj().Set("select", a.Obj("gates")))));
                    return One(MinClearance(Select(t, a), Strings(a["exclude_roles"])));
                case "no_overlap": {
                    var polys = t.Arr("tasks").Cast<JObj>().Select(x => Points(x.Arr("footprint"))).Where(p => p.Count > 2).ToList();
                    int n = 0;
                    for (int i = 0; i < polys.Count; i++)
                        for (int j = i + 1; j < polys.Count; j++)
                            if (Geometry.ConvexOverlap(polys[i], polys[j])) n++;
                    return One(n);
                }
                case "unique": {
                    var allowed = a.Has("allowed") ? a.Arr("allowed") : null;
                    int bad = 0;
                    foreach (var group in Select(t, a).GroupBy(x => Json.Write(Json.Path(x, a.Str("group") ?? ""))))
                        foreach (var same in group.GroupBy(x => Json.Write(Json.Path(x, a.Str("key"))))) {
                            bad += same.Count() - 1;
                            if (allowed != null && !allowed.Any(v => Same(v, Json.Path(same.First(), a.Str("key"))))) bad += same.Count();
                        }
                    return One(bad);
                }
                case "implies":
                    return One(Select(t, a).Count(x => Where(x, a.Obj("if")) && !Where(x, a.Obj("then"))));
                case "argmin_unique": {
                    var keys = Select(t, a).Select(x => Json.Path(x, a.Str("key"))).OfType<double>().ToList();
                    return One(keys.Count == 0 ? 0 : keys.Count(k => k == keys.Min()));
                }
                case "value": return Select(t, a).Select(x => Json.Path(x, a.Str("path"))).ToList();
                case "path": return One(PathCheck.Run(t, a) ? 1 : 0);
                // Selected elements whose centre lies inside any selected bay's rectangle (depth along its yaw).
                // beside: the bays next to a selected bay on the same pier (slip number +-1) instead.
                case "in_bay": {
                    var bays = Select(t, new JObj().Set("select", a.Obj("bays")));
                    if (a["beside"] is bool bs && bs) {
                        var all = new JObj();
                        foreach (var kv in a.Obj("bays")) if (kv.Key != "where") all[kv.Key] = kv.Value;
                        var chosen = bays;
                        bays = Select(t, new JObj().Set("select", all)).Where(b => chosen.Any(m =>
                            m.Str("pier") == b.Str("pier") && Math.Abs(m.Num("slip") - b.Num("slip")) == 1)).ToList();
                    }
                    return One(Select(t, a).Count(e => bays.Any(b =>
                        Geometry.BoxDistance(Xy(e), Xy(b), b.Num("yaw"), b.Num("depth"), b.Num("width")) <= 0)));
                }
                default: throw new ArgumentException($"unknown predicate {row.Predicate} in row {row.Id}");
            }
        }

        static List<object> One(double v) => new List<object> { v };

        // ---- selection -------------------------------------------------------------------

        /// <summary>Items of a collection ("elements" by default) filtered by task type, role,
        /// kind prefix and a where clause.</summary>
        public static List<JObj> Select(JObj t, JObj a) {
            JObj s = a.Obj("select") ?? new JObj();
            string collection = s.Str("collection") ?? "elements";
            var items = t.Arr(collection).Cast<JObj>();
            if (s.Has("task")) {
                var ids = TaskIds(t, s.Str("task"));
                items = items.Where(x => ids.Contains(collection == "tasks" ? x.Str("id") : x.Str("task")));
            }
            if (s.Has("role")) { var roles = Strings(s["role"]); items = items.Where(x => roles.Contains(x.Str("role"))); }
            if (s.Has("kind_prefix")) {
                var prefixes = Strings(s["kind_prefix"]);
                items = items.Where(x => prefixes.Any(p => (x.Str("kind") ?? "").StartsWith(p, StringComparison.Ordinal)));
            }
            if (s.Has("where")) items = items.Where(x => Where(x, s.Obj("where")));
            return items.ToList();
        }

        static HashSet<string> TaskIds(JObj t, string type) {
            var ids = new HashSet<string> { type };
            foreach (JObj x in t.Arr("tasks")) if (x.Str("type") == type) ids.Add(x.Str("id"));
            return ids;
        }

        public static bool Where(JObj item, JObj clause) {
            if (clause == null) return true;
            foreach (var kv in clause) {
                object v = Json.Path(item, kv.Key);
                if (kv.Value is List<object> options ? !options.Any(o => Same(o, v)) : !Same(kv.Value, v)) return false;
            }
            return true;
        }

        static bool Matches(JObj t, JObj when) =>
            Select(t, when).Any(x => Where(x, new JObj().Set(when.Str("path"), when["equals"])));

        public static bool Same(object a, object b) =>
            a is double x && b is double y ? Math.Abs(x - y) < 1e-9 : Equals(a, b);

        static HashSet<string> Strings(object v) =>
            v is List<object> l ? new HashSet<string>(l.Cast<string>()) : v is string s ? new HashSet<string> { s } : new HashSet<string>();

        // ---- references and shapes -------------------------------------------------------

        public readonly struct RefPoint {
            public readonly P2 At;
            public readonly double Yaw;
            public RefPoint(P2 at, double yaw) { At = at; Yaw = yaw; }
        }

        /// <summary>"elem:ID", "gate:ID", "bay:ID", "anchor:TASK.NAME" or "shore:normal" (the shore
        /// line's midpoint, yaw pointing into the course).</summary>
        public static RefPoint Ref(JObj t, string r) {
            int colon = r.IndexOf(':');
            string kind = r.Substring(0, colon), id = r.Substring(colon + 1);
            if (kind == "shore") {
                var (s0, s1, sign) = Shore(t);
                P2 n = (s1 - s0).Left * sign;
                return new RefPoint((s0 + s1) * 0.5, Math.Atan2(n.Y, n.X));
            }
            if (kind == "gate" && id.Contains("@")) {
                var ids = TaskIds(t, id.Substring(0, id.IndexOf('@')));
                var gates = t.Arr("gates").Cast<JObj>().Where(g => ids.Contains(g.Str("task"))).OrderBy(g => g.Num("order")).ToList();
                if (gates.Count == 0) throw new KeyNotFoundException(r);
                JObj g0 = id.EndsWith("@last", StringComparison.Ordinal) ? gates[gates.Count - 1] : gates[0];
                return new RefPoint(Xy(g0), g0.Num("normal_yaw"));
            }
            if (kind == "anchor") {
                int dot = id.IndexOf('.');
                var ids = TaskIds(t, id.Substring(0, dot));
                foreach (JObj task in t.Arr("tasks"))
                    if (ids.Contains(task.Str("id")) && task.Obj("anchors")?.Obj(id.Substring(dot + 1)) is JObj p)
                        return new RefPoint(Xy(p), double.NaN);
                throw new KeyNotFoundException(r);
            }
            string collection = kind == "elem" ? "elements" : kind == "gate" ? "gates" : "bays";
            foreach (JObj o in t.Arr(collection))
                if (o.Str("id") == id) return new RefPoint(Xy(o), o.Num(kind == "gate" ? "normal_yaw" : "yaw"));
            throw new KeyNotFoundException(r);
        }

        static double Along(JObj t, string gate, string point) {
            var g = Ref(t, gate);
            return P2.Dot(Ref(t, point).At - g.At, P2.Dir(g.Yaw));
        }

        static JObj Element(JObj t, string id) {
            foreach (JObj e in t.Arr("elements")) if (e.Str("id") == id) return e;
            throw new KeyNotFoundException(id);
        }

        public static P2 Xy(JObj o) => new P2(o.Num("x"), o.Num("y"));

        public static List<P2> Points(List<object> list) =>
            list.Select(p => { var xy = (List<object>)p; return new P2((double)xy[0], (double)xy[1]); }).ToList();

        public static bool IsBox(JObj e) => e.Has("length");

        /// <summary>Distance from p to an element's surface (box or circle).</summary>
        public static double Surface(JObj e, P2 p) => IsBox(e)
            ? Geometry.BoxDistance(p, Xy(e), e.Num("yaw"), e.Num("length"), e.Num("width"))
            : P2.Dist(p, Xy(e)) - e.Num("radius");

        /// <summary>The shore line and the sign that makes the course side positive.</summary>
        static (P2, P2, double) Shore(JObj t) {
            var shore = Points(t.Arr("shore"));
            var poly = Points(t.Arr("boundary"));
            P2 c = new P2(poly.Average(q => q.X), poly.Average(q => q.Y));
            return (shore[0], shore[1], Geometry.Cross(shore[0], shore[1], c) >= 0 ? 1.0 : -1.0);
        }

        /// <summary>Gate centre and normal recomputed from its posts (port on the left of travel).</summary>
        static (P2 at, double yaw) GatePose(JObj t, string gateRef) {
            string id = gateRef.Substring(gateRef.IndexOf(':') + 1);
            JObj g = t.Arr("gates").Cast<JObj>().FirstOrDefault(x => x.Str("id") == id) ?? throw new KeyNotFoundException(gateRef);
            P2 port = Xy(Element(t, g.Str("port"))), starboard = Xy(Element(t, g.Str("starboard"))), v = port - starboard;
            return ((port + starboard) * 0.5, Math.Atan2(-v.X, v.Y));
        }

        static double ShapeInside(Func<P2, double> margin, JObj e) {
            if (!IsBox(e)) return margin(Xy(e)) - e.Num("radius");
            double best = double.MaxValue;
            P2 c = Xy(e), f = P2.Dir(e.Num("yaw")), l = f.Left;
            foreach (var (u, v) in new[] { (1, 1), (1, -1), (-1, 1), (-1, -1) })
                best = Math.Min(best, margin(c + f * (u * e.Num("length") / 2) + l * (v * e.Num("width") / 2)));
            return best;
        }

        /// <summary>Least distance from an element's surface to a gate segment (post to post),
        /// ignoring the gate's own posts.</summary>
        static double GateClearance(JObj t, List<JObj> items, List<JObj> gates) {
            double best = 1e9;
            foreach (var g in gates) {
                P2 a = Xy(Element(t, g.Str("port"))), b = Xy(Element(t, g.Str("starboard")));
                foreach (var e in items) {
                    if (e.Str("id") == g.Str("port") || e.Str("id") == g.Str("starboard")) continue;
                    best = Math.Min(best, SegmentDistance(Xy(e), a, b) - (IsBox(e) ? 0 : e.Num("radius")));
                }
            }
            return best;
        }

        public static double SegmentDistance(P2 p, P2 a, P2 b) {
            P2 ab = b - a;
            double u = Math.Max(0, Math.Min(1, P2.Dot(p - a, ab) / P2.Dot(ab, ab)));
            return P2.Dist(p, a + ab * u);
        }

        static double MinClearance(List<JObj> items, HashSet<string> exclude) {
            var kept = items.Where(e => !exclude.Contains(e.Str("role"))).ToList();
            double best = 1e9;
            for (int i = 0; i < kept.Count; i++)
                for (int j = i + 1; j < kept.Count; j++) {
                    JObj a = kept[i], b = kept[j];
                    if (IsBox(a) && IsBox(b)) continue;
                    double d = IsBox(a) ? Surface(a, Xy(b)) - b.Num("radius")
                        : IsBox(b) ? Surface(b, Xy(a)) - a.Num("radius")
                        : P2.Dist(Xy(a), Xy(b)) - a.Num("radius") - b.Num("radius");
                    best = Math.Min(best, d);
                }
            return best;
        }
    }
}
