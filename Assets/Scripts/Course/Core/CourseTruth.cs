using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Course {
    /// <summary>One generated task, as the generator hands it to the truth builder.</summary>
    public sealed class GeneratedTask {
        public CourseTaskDefinition Definition;
        public ParamValues Params;
        public TaskLayout Layout;
        public int Attempts;
    }

    /// <summary>Ground truth <c>crane.course_truth/2</c> (§4), a superset of v1: the referee's
    /// root, element and bay keys are kept. Built in the course frame, hashed there, then moved
    /// to the world frame (/crane/odom) for the file.</summary>
    public static class CourseTruth {
        public const string SchemaV1 = "crane-roboboat-course-truth-v1";
        public const int SchemaVersion = 2;

        /// <summary>The course-frame content: everything the seed determines.</summary>
        public static JObj Core(string name, double side, CourseBoundary boundary, VesselProfile vessel,
            IReadOnlyList<GeneratedTask> tasks, CourseObjectCatalog catalog) {
            var elements = new List<object>();
            var gates = new List<object>();
            var bays = new List<object>();
            var taskList = new List<object>();
            var states = new List<object>();
            var byId = new Dictionary<string, PlacedObject>();
            var stateById = new Dictionary<string, StateSpec>();
            foreach (var t in tasks) {
                foreach (var o in t.Layout.Objects) byId[o.Id] = o;
                foreach (var s in t.Layout.States) stateById[s.Id] = s;
            }

            int dock = 0;
            foreach (var t in tasks) {
                string task = t.Definition.TaskId;
                foreach (var o in t.Layout.Objects) {
                    var entry = catalog.Get(o.Kind);
                    string state = o.StateId != null && stateById.TryGetValue(o.StateId, out var s) ? s.Initial : null;
                    var e = new JObj()
                        .Set("task", task).Set("label", catalog.Label(o.Kind, state, LabelMode.Oracle))
                        .Set("template", entry.template).Set("x", CourseFrame.Mm(o.At.X)).Set("y", CourseFrame.Mm(o.At.Y))
                        .Set("radius", o.IsBox ? CourseFrame.Mm(0.5 * Math.Sqrt(o.Length * o.Length + o.Width * o.Width)) : catalog.Radius(o.Kind))
                        .Set("id", o.Id).Set("role", o.Role).Set("kind", o.Kind).Set("z", CourseFrame.Mm(o.Z))
                        .Set("yaw", CourseFrame.Rad(o.Yaw)).Set("height", catalog.Height(o.Kind)).Set("state", o.StateId);
                    if (o.IsBox) e.Set("length", CourseFrame.Mm(o.Length)).Set("width", CourseFrame.Mm(o.Width));
                    elements.Add(e);
                }
                foreach (var g in t.Layout.Gates) {
                    PlacedObject p = byId[g.Port], sb = byId[g.Starboard];
                    double cc = P2.Dist(p.At, sb.At);
                    P2 mid = (p.At + sb.At) * 0.5;
                    gates.Add(new JObj().Set("id", g.Id).Set("task", task).Set("port", g.Port).Set("starboard", g.Starboard)
                        .Set("order", g.Order).Set("x", CourseFrame.Mm(mid.X)).Set("y", CourseFrame.Mm(mid.Y))
                        .Set("cc", CourseFrame.Mm(cc))
                        .Set("clear", CourseFrame.Mm(cc - catalog.Radius(p.Kind) - catalog.Radius(sb.Kind)))
                        .Set("normal_yaw", CourseFrame.Rad(g.NormalYaw)));
                }
                bool hasBays = t.Layout.Bays.Count > 0;
                foreach (var b in t.Layout.Bays) {
                    bays.Add(new JObj()
                        .Set("dock", (double)(dock + (b.Pier == "S" ? 1 : 0))).Set("index", (double)(b.Slip - 1))
                        .Set("x", CourseFrame.Mm(b.At.X)).Set("y", CourseFrame.Mm(b.At.Y)).Set("yaw", CourseFrame.Rad(b.Yaw))
                        .Set("width", CourseFrame.Mm(b.Width)).Set("depth", CourseFrame.Mm(b.Depth))
                        .Set("banner", $"banner_{b.Number}").Set("occupied", b.Occupied)
                        .Set("id", b.Id).Set("task", task).Set("pier", b.Pier).Set("slip", (double)b.Slip)
                        .Set("sign", new JObj().Set("id", b.SignId).Set("number", (double)b.Number))
                        .Set("indicator", new JObj().Set("id", b.IndicatorId).Set("color", b.IndicatorColor))
                        .Set("vessel", b.Vessel));
                }
                if (hasBays) dock += 2;
                foreach (var s in t.Layout.States)
                    states.Add(new JObj().Set("id", s.Id).Set("kind", s.Kind).Set("initial", s.Initial).Set("program", s.Program));
                var parameters = new JObj();
                foreach (var kv in t.Params.All) parameters[kv.Key] = kv.Value is long n ? (double)n : kv.Value is double d ? Math.Round(d, 4) : kv.Value;
                taskList.Add(new JObj().Set("id", task).Set("type", t.Definition.TaskType).Set("cite", t.Definition.Cite)
                    .Set("params", parameters).Set("anchors", RoundPoints(t.Layout.Anchors))
                    .Set("footprint", t.Layout.Footprint.Select(p => (object)Point(p)).ToList())
                    .Set("semantics", t.Layout.Semantics).Set("attempts", (double)t.Attempts));
            }

            return new JObj()
                .Set("boundary", boundary.Polygon.Select(p => (object)Point(p)).ToList())
                .Set("shore", boundary.Shore.Select(p => (object)Point(p)).ToList())
                .Set("side", side > 0 ? "port" : "starboard")
                .Set("vessel", vessel.ToJson())
                .Set("elements", elements).Set("gates", gates).Set("bays", bays)
                .Set("tasks", taskList).Set("states", states);
        }

        static List<object> Point(P2 p) => new List<object> { CourseFrame.Mm(p.X), CourseFrame.Mm(p.Y) };

        static JObj RoundPoints(JObj anchors) {
            var o = new JObj();
            foreach (var kv in anchors) {
                var a = (JObj)kv.Value;
                var r = new JObj().Set("x", CourseFrame.Mm(a.Num("x"))).Set("y", CourseFrame.Mm(a.Num("y")));
                if (a.Has("r")) r.Set("r", CourseFrame.Mm(a.Num("r")));
                o[kv.Key] = r;
            }
            return o;
        }

        /// <summary>The full document: v1 root keys, provenance, then the core. The sha256
        /// covers family, handbook, generator (without commit and sha) and the core, all in the
        /// course frame, so the headless CLI and the player agree.</summary>
        public static JObj Assemble(JObj core, string name, string family, string handbookId, string handbookSha,
            JObj generator, List<object> constraints) {
            var hashed = new JObj().Set("family", family)
                .Set("handbook", new JObj().Set("id", handbookId).Set("sha256", handbookSha))
                .Set("generator", generator);
            foreach (var kv in core) hashed[kv.Key] = kv.Value;
            hashed["constraints"] = constraints;
            string sha = CourseFamilyDefinition.Hex(Json.Write(hashed));
            generator.Set("crane_commit", "unknown").Set("truth_sha256", sha);

            var doc = new JObj().Set("schema", SchemaV1).Set("schema_version", (double)SchemaVersion)
                .Set("name", name).Set("frame", "world")
                .Set("startX", 0.0).Set("startY", 0.0).Set("startYaw", 0.0)
                .Set("currentSpeed", 0.0).Set("currentToward", 0.0)
                .Set("course_frame", new JObj().Set("x", 0.0).Set("y", 0.0).Set("yaw", 0.0))
                .Set("geodetic_origin", null).Set("label_mode", "oracle");
            foreach (var kv in hashed) doc[kv.Key] = kv.Value;
            return doc;
        }

        public static string Sha(JObj truth) => (Json.Path(truth, "generator.truth_sha256") as string) ?? "";

        /// <summary>Moves every position and heading from the course frame into the world frame
        /// whose origin is the boat start (in place). Distances and the hash are unchanged.</summary>
        public static void ToWorld(JObj truth, Pose2 start) {
            truth.Set("startX", CourseFrame.Mm(start.X)).Set("startY", CourseFrame.Mm(start.Y)).Set("startYaw", CourseFrame.Rad(start.Yaw));
            truth.Set("course_frame", new JObj().Set("x", CourseFrame.Mm(start.X)).Set("y", CourseFrame.Mm(start.Y)).Set("yaw", CourseFrame.Rad(start.Yaw)));
            foreach (var key in new[] { "elements", "gates", "bays" })
                foreach (JObj o in truth.Arr(key)) MovePose(o, start, key == "gates" ? "normal_yaw" : "yaw");
            foreach (var key in new[] { "boundary", "shore" }) truth[key] = MovePoints(truth.Arr(key), start);
            foreach (JObj t in truth.Arr("tasks")) {
                foreach (var kv in t.Obj("anchors")) MovePose((JObj)kv.Value, start, null);
                t["footprint"] = MovePoints(t.Arr("footprint"), start);
            }
        }

        static void MovePose(JObj o, Pose2 start, string yawKey) {
            P2 w = CourseFrame.ToWorld(start, new P2(o.Num("x"), o.Num("y")));
            o.Set("x", CourseFrame.Mm(w.X)).Set("y", CourseFrame.Mm(w.Y));
            if (yawKey != null && o.Has(yawKey)) o.Set(yawKey, CourseFrame.Rad(CourseFrame.Wrap(o.Num(yawKey) + start.Yaw)));
        }

        static List<object> MovePoints(List<object> points, Pose2 start) => points.Select(p => {
            var xy = (List<object>)p;
            P2 w = CourseFrame.ToWorld(start, new P2((double)xy[0], (double)xy[1]));
            return (object)new List<object> { CourseFrame.Mm(w.X), CourseFrame.Mm(w.Y) };
        }).ToList();

        /// <summary>The oracle-mode detection id: stable per seed and element. Oracle labels
        /// carry colour and numerals anyway, so this id may be recomputable.</summary>
        public static int OpaqueId(ulong seed, string elementId) =>
            (int)(CourseRng.Fnv1a64(seed, "opaque:" + elementId) & 0x7FFFFFFF);

        /// <summary>The camera-mode id salt: drawn from a cryptographic RNG at process start,
        /// never from the logged seed, so camera ids cannot be recomputed from the run's log.
        /// Tests pass an explicit salt instead.</summary>
        public static readonly ulong RunSalt = FreshSalt();

        static ulong FreshSalt() {
            var bytes = new byte[8];
            using (var rng = System.Security.Cryptography.RandomNumberGenerator.Create()) rng.GetBytes(bytes);
            return BitConverter.ToUInt64(bytes, 0);
        }

        /// <summary>A camera-mode id: FNV-1a of a colour-free key (task, role, ordinal), xored
        /// with the salt and run through the SplitMix64 finaliser.</summary>
        public static int CameraId(ulong salt, string key) {
            ulong z = CourseRng.Fnv1a64(0, "camera:" + key) ^ salt;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            z ^= z >> 31;
            return (int)(z & 0x7FFFFFFF);
        }

        /// <summary>The colour-free key of every element: task, role and the element's ordinal
        /// among that task's elements of the same role, in truth order. Never an element id
        /// (ids name colours, e.g. green_beacon1).</summary>
        public static Dictionary<string, string> CameraKeys(JObj truth) {
            var keys = new Dictionary<string, string>();
            var count = new Dictionary<string, int>();
            foreach (JObj e in truth.Arr("elements")) {
                string group = e.Str("task") + "/" + e.Str("role");
                count.TryGetValue(group, out int n);
                count[group] = n + 1;
                keys[e.Str("id")] = group + "/" + n;
            }
            return keys;
        }

        /// <summary>Detection id per element id, as perception truth and Detection3D both use:
        /// oracle mode the seed-stable <see cref="OpaqueId"/>, camera mode the salted
        /// <see cref="CameraId"/> (salt defaults to <see cref="RunSalt"/>). Not hashed into
        /// truth_sha256.</summary>
        public static Dictionary<string, int> DetectionIds(JObj truth, LabelMode mode, ulong seed, ulong? salt = null) {
            var ids = new Dictionary<string, int>();
            if (mode == LabelMode.Camera) {
                ulong s = salt ?? RunSalt;
                foreach (var kv in CameraKeys(truth)) ids[kv.Key] = CameraId(s, kv.Value);
            } else {
                foreach (JObj e in truth.Arr("elements")) ids[e.Str("id")] = OpaqueId(seed, e.Str("id"));
            }
            return ids;
        }

        /// <summary>The stripped perception truth for sim_backend (§4): x, y, z, label in the
        /// run's label mode, and an opaque id (<see cref="DetectionIds"/>); nothing else.</summary>
        public static JObj Perception(JObj truth, CourseObjectCatalog catalog, LabelMode mode, ulong seed,
            Func<string, string> stateNow = null, ulong? salt = null) {
            var ids = DetectionIds(truth, mode, seed, salt);
            var list = new List<(string id, JObj e)>();
            foreach (JObj e in truth.Arr("elements")) {
                string stateId = e.Str("state");
                string state = stateId == null ? null : stateNow?.Invoke(stateId) ?? Initial(truth, stateId);
                string label = catalog.Label(e.Str("kind"), state, mode);
                if (string.IsNullOrEmpty(label)) continue;
                list.Add((e.Str("id"), new JObj().Set("x", e.Num("x")).Set("y", e.Num("y")).Set("z", e.Num("z"))
                    .Set("label", label).Set("id", (double)ids[e.Str("id")])));
            }
            return new JObj().Set("schema", "crane.perception_truth/1").Set("frame", "world")
                .Set("label_mode", mode == LabelMode.Camera ? "camera" : "oracle")
                .Set("elements", list.OrderBy(x => (double)x.e["id"]).Select(x => (object)x.e).ToList());
        }

        public static string Initial(JObj truth, string stateId) {
            foreach (JObj s in truth.Arr("states")) if (s.Str("id") == stateId) return s.Str("initial");
            return null;
        }
    }
}
