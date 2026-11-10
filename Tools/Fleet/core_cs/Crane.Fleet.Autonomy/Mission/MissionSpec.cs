using System;
using System.Collections.Generic;
using System.Globalization;
using Crane.Fleet.Perception;

namespace Crane.Fleet
{
    /// <summary>
    /// A mission is DATA: a JSON task list (schema "crane.fleet_mission/v0") that the fleet game can generate (promotion_v0.md §2).
    /// Each task has an id, a type, the agents it uses, parameters, "after" dependencies and a retry budget.
    /// Types: goto, explore, lift, rendezvous, dock, assemble, transport, separate, handoff, wait.
    /// </summary>
    public sealed class MissionSpec
    {
        public const string Schema = "crane.fleet_mission/v0";
        public string MissionId;
        public readonly List<TaskSpec> Tasks = new List<TaskSpec>();
        /// <summary>Payloads already resting on decks at mission start: id, profile payload type, supporting rovers, COM in world.</summary>
        public readonly List<PayloadPlacement> InitialPayloads = new List<PayloadPlacement>();
        /// <summary>Couplings already latched at mission start (pre-assembled groups).</summary>
        public readonly List<CouplingSpec> InitialCouplings = new List<CouplingSpec>();

        public static MissionSpec Parse(string json)
        {
            var root = MiniJson.Parse(json) as Dictionary<string, object> ?? throw new FormatException("mission: root is not an object");
            if (!(root.TryGetValue("schema", out var sc) && sc as string == Schema)) throw new FormatException("mission: schema must be " + Schema);
            var m = new MissionSpec { MissionId = Str(root, "mission_id", "mission") };
            if (root.TryGetValue("initial_couplings", out var ic) && ic is List<object> icl)
                foreach (var o in icl) m.InitialCouplings.Add(CouplingSpec.From((Dictionary<string, object>)o));
            if (root.TryGetValue("initial_payloads", out var ip) && ip is List<object> ipl)
                foreach (var o in ipl)
                {
                    var d = (Dictionary<string, object>)o;
                    var com = Nums(d, "com");
                    m.InitialPayloads.Add(new PayloadPlacement { Id = Str(d, "id", "payload"), Type = Str(d, "type", "box_long"), Supporters = Strs(d, "supporters"), ComX = com[0], ComY = com[1] });
                }
            if (!(root.TryGetValue("tasks", out var t) && t is List<object> tl)) throw new FormatException("mission: tasks missing");
            var ids = new HashSet<string>();
            foreach (var o in tl)
            {
                var d = o as Dictionary<string, object> ?? throw new FormatException("mission: task is not an object");
                var ts = new TaskSpec { Id = Str(d, "id", null), Type = Str(d, "type", null) ?? Str(d, "kind", null), Raw = d };
                if (ts.Id == null || ts.Type == null) throw new FormatException("mission: every task needs id and type");
                if (ts.Type == "undock") ts.Type = "separate";
                if (!ids.Add(ts.Id)) throw new FormatException("mission: duplicate task id " + ts.Id);
                if (!TaskSpec.KnownTypes.Contains(ts.Type)) throw new FormatException("mission: unknown task type '" + ts.Type + "' (supported: " + string.Join(", ", TaskSpec.KnownTypes) + ")");
                ts.NormalizeAliases();
                ts.ExplicitAfter = d.ContainsKey("after");
                ts.After = Strs(d, "after");
                ts.Retries = d.TryGetValue("retries", out var r) && r is double rd ? (int)rd : 1;
                ts.Timeout = d.TryGetValue("timeout", out var to) && to is double tod ? tod : 300;
                m.Tasks.Add(ts);
            }
            // implicit dependencies: a task without "after" waits for the latest earlier task that uses one of its rovers
            for (int i = 0; i < m.Tasks.Count; i++)
            {
                var ts = m.Tasks[i];
                if (ts.ExplicitAfter) continue;
                var mine = ts.RoversMentioned();
                for (int j = i - 1; j >= 0; j--)
                {
                    bool share = false;
                    foreach (var r in m.Tasks[j].RoversMentioned()) if (mine.Contains(r)) share = true;
                    if (share) { ts.After.Add(m.Tasks[j].Id); break; }
                }
            }
            foreach (var ts in m.Tasks) foreach (var a in ts.After) if (!ids.Contains(a)) throw new FormatException($"mission: task {ts.Id} depends on unknown {a}");
            return m;
        }

        internal static string Str(Dictionary<string, object> d, string k, string def) => d.TryGetValue(k, out var v) && v is string s ? s : def;
        internal static List<string> Strs(Dictionary<string, object> d, string k)
        {
            var l = new List<string>();
            if (d.TryGetValue(k, out var v) && v is List<object> lo) foreach (var o in lo) if (o is string s) l.Add(s);
            return l;
        }
        internal static double[] Nums(Dictionary<string, object> d, string k)
        {
            if (!(d.TryGetValue(k, out var v) && v is List<object> lo)) return null;
            var r = new double[lo.Count];
            for (int i = 0; i < lo.Count; i++) r[i] = Convert.ToDouble(lo[i], CultureInfo.InvariantCulture);
            return r;
        }
        internal static double Num(Dictionary<string, object> d, string k, double def) => d.TryGetValue(k, out var v) && v is double x ? x : def;
    }

    public sealed class TaskSpec
    {
        public static readonly HashSet<string> KnownTypes = new HashSet<string> { "goto", "explore", "lift", "rendezvous", "dock", "assemble", "transport", "separate", "handoff", "wait" };
        public string Id, Type;
        public List<string> After = new List<string>();
        public int Retries = 1;
        public double Timeout = 300;
        public Dictionary<string, object> Raw;
        public bool ExplicitAfter;

        /// <summary>Accept the field names emitted by Tools/Fleet/harness/promote.py (kind, actors, reference_member, follower/leader,
        /// face_follower/face_leader, coupling, route_m) by copying them to the canonical names. Canonical names win when both exist.</summary>
        public void NormalizeAliases()
        {
            void Alias(string canon, string alias) { if (!Raw.ContainsKey(canon) && Raw.TryGetValue(alias, out var v)) Raw[canon] = v; }
            Alias("agent", "reference_member");
            if (!Raw.ContainsKey("agent") && Raw.TryGetValue("actors", out var ac) && ac is List<object> al && al.Count > 0) Raw["agent"] = al[0];
            Alias("docker", "follower"); Alias("target", "leader");
            Alias("docker_face", "face_follower"); Alias("target_face", "face_leader");
            Alias("route", "route_m");
            // coupling "target.face-docker.face" (harness convention: leader first)
            if (Raw.TryGetValue("coupling", out var cv) && cv is string cs && (!Raw.ContainsKey("docker") || !Raw.ContainsKey("target")))
            {
                var parts = cs.Split('-');
                if (parts.Length == 2)
                {
                    var a = parts[0].Split('.'); var b = parts[1].Split('.');
                    if (a.Length == 2 && b.Length == 2) { Raw["target"] = a[0]; Raw["target_face"] = a[1]; Raw["docker"] = b[0]; Raw["docker_face"] = b[1]; }
                }
            }
        }

        /// <summary>Every rover id named by the task (used for implicit ordering and exclusive use).</summary>
        public HashSet<string> RoversMentioned()
        {
            var h = new HashSet<string>();
            foreach (var k in new[] { "agent", "docker", "target", "from" }) { var v = Str(k); if (v != null) h.Add(v); }
            foreach (var k in new[] { "agents", "supporters", "actors" }) foreach (var v in Strs(k)) h.Add(v);
            foreach (var c in Couplings("sequence")) { h.Add(c.Docker); h.Add(c.Target); }
            return h;
        }
        public string Str(string k, string def = null) => MissionSpec.Str(Raw, k, def);
        public List<string> Strs(string k) => MissionSpec.Strs(Raw, k);
        public double[] Nums(string k) => MissionSpec.Nums(Raw, k);
        public double Num(string k, double def) => MissionSpec.Num(Raw, k, def);
        public Pose2 PoseOf(string k) { var v = Nums(k); if (v == null) throw new FormatException($"task {Id}: {k} missing"); return new Pose2(v[0], v[1], v.Length > 2 ? v[2] : 0); }
        public List<CouplingSpec> Couplings(string k)
        {
            var l = new List<CouplingSpec>();
            if (Raw.TryGetValue(k, out var v) && v is List<object> lo) foreach (var o in lo) l.Add(CouplingSpec.From((Dictionary<string, object>)o));
            return l;
        }
    }

    /// <summary>A face pair: docker (moving) rover and face, target (holding) rover and face.</summary>
    public sealed class CouplingSpec
    {
        public string Docker, DockerFace, Target, TargetFace;
        public static CouplingSpec From(Dictionary<string, object> d) => new CouplingSpec
        {
            Docker = MissionSpec.Str(d, "docker", null), DockerFace = MissionSpec.Str(d, "docker_face", "front"),
            Target = MissionSpec.Str(d, "target", null), TargetFace = MissionSpec.Str(d, "target_face", "rear"),
        };
        public string Id => Docker + "." + DockerFace + "-" + Target + "." + TargetFace;
    }

    public sealed class PayloadPlacement
    {
        public string Id, Type; public List<string> Supporters = new List<string>(); public double ComX, ComY;
    }
}
