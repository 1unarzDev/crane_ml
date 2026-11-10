using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Docking;
using Crane.Fleet.Profile;

namespace Crane.Fleet.Graph
{
    public sealed class Rover
    {
        public string Id { get; }
        public Pose2 Pose { get; set; }
        public Rover(string id, Pose2 pose) { Id = id; Pose = pose; }
    }

    public sealed class Payload
    {
        public string Id { get; }
        public double Mass { get; }
        /// <summary>Centre of mass in the world frame (planar).</summary>
        public Vec2 Com { get; set; }
        /// <summary>Rovers whose decks the payload rests on.</summary>
        public List<string> SupportingRovers { get; }
        public Payload(string id, double mass, Vec2 com, IEnumerable<string> supporters)
        { Id = id; Mass = mass; Com = com; SupportingRovers = new List<string>(supporters); }
    }

    /// <summary>One coupling edge: face A of rover A mated with face B of rover B.</summary>
    public sealed class Coupling
    {
        public string Id { get; }
        public string RoverA { get; }
        public string FaceA { get; }
        public string RoverB { get; }
        public string FaceB { get; }
        public ConnectorMachine Machine { get; set; }
        private ConnectorState _manual = ConnectorState.Free;
        /// <summary>Live machine state if one is attached; otherwise a manually set state (planning / tests).</summary>
        public ConnectorState State { get => Machine != null ? Machine.State : _manual; set => _manual = value; }
        public bool Removed { get; internal set; }
        internal bool WasVerified;
        public Coupling(string id, string ra, string fa, string rb, string fb) { Id = id; RoverA = ra; FaceA = fa; RoverB = rb; FaceB = fb; }
        public bool IsVerified => !Removed && ConnectorStateNames.IsVerified(State);
    }

    public struct ReleaseDecision
    {
        public bool Allowed;
        public string Reason;
        public override string ToString() => (Allowed ? "ALLOWED: " : "REFUSED: ") + Reason;
    }

    public struct SupportResult
    {
        public bool Ok;
        public string Reason;
        public double MinMargin;
    }

    /// <summary>
    /// Small graph of rovers, couplings and payloads with the release invariant:
    /// a coupling may release only when support is VERIFIED on the graph that would remain.
    /// "Verified" = every payload's supporting decks are joined by LATCHED_VERIFIED/LOAD_VALIDATED couplings, the payload COM is inside the
    /// convex hull of the supporting deck contact points by SupportMargin, and every remaining verified coupling is within rated load.
    /// </summary>
    public sealed class ModuleGraph
    {
        public DockingProfile Profile { get; }
        public DockingAssumptions Assume { get; }
        public ContactModel Contact { get; }
        public Dictionary<string, Rover> Rovers { get; } = new Dictionary<string, Rover>();
        public Dictionary<string, Coupling> Couplings { get; } = new Dictionary<string, Coupling>();
        public Dictionary<string, Payload> Payloads { get; } = new Dictionary<string, Payload>();
        public List<DockingEvent> Events { get; } = new List<DockingEvent>();
        public double Time { get; set; }

        public ModuleGraph(DockingProfile profile, DockingAssumptions assume)
        {
            Profile = profile; Assume = assume; Contact = new ContactModel(profile, assume);
        }

        // ------------------------------------------------------------ construction
        public Rover AddRover(string id, Pose2 pose) { var r = new Rover(id, pose); Rovers[id] = r; return r; }

        /// <summary>Adds rover <paramref name="newId"/> mated to <paramref name="existing"/> face <paramref name="faceExisting"/> using the newcomer's <paramref name="faceNew"/>.</summary>
        public Coupling MateNew(string couplingId, string existing, string faceExisting, string newId, string faceNew, ConnectorState state = ConnectorState.LatchedVerified)
        {
            var fa = Profile.FaceFrames[faceExisting].Pose;
            var fb = Profile.FaceFrames[faceNew].Pose;
            // faces mate when face B's pose equals face A's pose turned by pi: T_new = T_existing * F_A * Rot(pi) * F_B^-1
            var pose = Rovers[existing].Pose.Compose(fa).Compose(new Pose2(0, 0, Math.PI)).Compose(fb.Inverse());
            AddRover(newId, pose);
            return AddCoupling(couplingId, existing, faceExisting, newId, faceNew, state);
        }

        public Coupling AddCoupling(string id, string ra, string fa, string rb, string fb, ConnectorState state)
        {
            var c = new Coupling(id, ra, fa, rb, fb) { State = state };
            c.WasVerified = ConnectorStateNames.IsVerified(state);
            Couplings[id] = c; return c;
        }

        public Payload AddPayload(string id, string specName, Vec2 com, params string[] supporters)
        {
            var spec = Profile.Payloads[specName];
            var p = new Payload(id, spec.Mass, com, supporters);
            Payloads[id] = p; return p;
        }

        public Vec2 Center(string roverId) => Rovers[roverId].Pose.Position;

        public static ModuleGraph Line(DockingProfile profile, DockingAssumptions assume, int n, ConnectorState state = ConnectorState.LatchedVerified)
        {
            var g = new ModuleGraph(profile, assume);
            g.AddRover("r0", Pose2.Identity);
            for (int i = 1; i < n; i++) g.MateNew($"c{i - 1}_{i}", $"r{i - 1}", "front", $"r{i}", "rear", state);
            return g;
        }

        /// <summary>Two rovers side by side: r0 left face with r1 right face.</summary>
        public static ModuleGraph SideBySide(DockingProfile profile, DockingAssumptions assume, ConnectorState state = ConnectorState.LatchedVerified)
        {
            var g = new ModuleGraph(profile, assume);
            g.AddRover("r0", Pose2.Identity);
            g.MateNew("c0_1", "r0", "left", "r1", "right", state);
            return g;
        }

        /// <summary>T shape: r0 with r1 in front (front-rear), r2 behind (rear-front) and r3 on the left side (left-right).</summary>
        public static ModuleGraph Branched(DockingProfile profile, DockingAssumptions assume, ConnectorState state = ConnectorState.LatchedVerified)
        {
            var g = new ModuleGraph(profile, assume);
            g.AddRover("r0", Pose2.Identity);
            g.MateNew("c0_1", "r0", "front", "r1", "rear", state);
            g.MateNew("c0_2", "r0", "rear", "r2", "front", state);
            g.MateNew("c0_3", "r0", "left", "r3", "right", state);
            return g;
        }

        // ------------------------------------------------------------ support
        /// <summary>Deck contact points of a rover in the world frame: footprint corners inset by DeckContactInset.</summary>
        public List<Vec2> DeckContacts(string roverId)
        {
            var size = Profile.BodySizeLwh.Value;
            double hx = size[0] / 2 - Assume.DeckContactInset, hy = size[1] / 2 - Assume.DeckContactInset;
            var pose = Rovers[roverId].Pose;
            return new List<Vec2>
            {
                pose.TransformPoint(new Vec2(hx, hy)), pose.TransformPoint(new Vec2(-hx, hy)),
                pose.TransformPoint(new Vec2(-hx, -hy)), pose.TransformPoint(new Vec2(hx, -hy)),
            };
        }

        private Dictionary<string, string> Components(string excludeCoupling)
        {
            var parent = Rovers.Keys.ToDictionary(k => k, k => k);
            string Find(string x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
            foreach (var c in Couplings.Values)
            {
                if (c.Id == excludeCoupling || !c.IsVerified) continue;
                parent[Find(c.RoverA)] = Find(c.RoverB);
            }
            return Rovers.Keys.ToDictionary(k => k, Find);
        }

        /// <summary>Support check on the graph with <paramref name="excludeCoupling"/> removed (null = current graph).</summary>
        public SupportResult CheckSupport(string excludeCoupling = null)
        {
            var comp = Components(excludeCoupling);
            double minMargin = double.PositiveInfinity;
            foreach (var p in Payloads.Values)
            {
                foreach (string s in p.SupportingRovers)
                    if (!Rovers.ContainsKey(s)) return new SupportResult { Ok = false, Reason = $"payload {p.Id}: unknown supporting rover {s}" };
                var roots = p.SupportingRovers.Select(s => comp[s]).Distinct().ToList();
                if (roots.Count > 1)
                    return new SupportResult { Ok = false, Reason = $"payload {p.Id}: supporting decks {string.Join(",", p.SupportingRovers)} would not be joined by a verified coupling" };
                var pts = new List<Vec2>();
                foreach (string s in p.SupportingRovers) pts.AddRange(DeckContacts(s));
                double m = Hull2.InsideMargin(Hull2.Convex(pts), p.Com);
                minMargin = Math.Min(minMargin, m);
                if (m < Assume.SupportMargin)
                    return new SupportResult { Ok = false, MinMargin = m, Reason = $"payload {p.Id}: centre of mass margin {m:0.000} m inside support hull < required {Assume.SupportMargin:0.000} m" };
            }
            foreach (var c in Couplings.Values)
            {
                if (c.Id == excludeCoupling || !c.IsVerified) continue;
                var l = CouplingLoad(c.Id);
                if (l.RatioA > 1 || l.RatioB > 1)
                    return new SupportResult { Ok = false, MinMargin = minMargin, Reason = $"coupling {c.Id}: estimated hook load {Math.Max(l.A, l.B):0.0} N exceeds rated {Contact.HookRatedLoad} N" };
            }
            return new SupportResult { Ok = true, MinMargin = minMargin, Reason = "support verified" };
        }

        /// <summary>Weight share carried by each supporting deck: inverse-distance (lever rule for two decks), COM-to-deck-centre.</summary>
        public Dictionary<string, double> DeckShares(Payload p)
        {
            var w = new Dictionary<string, double>();
            double total = 0;
            foreach (string s in p.SupportingRovers)
            {
                double d = Math.Max((Center(s) - p.Com).Length, 1e-6);
                w[s] = 1.0 / d; total += 1.0 / d;
            }
            foreach (string k in w.Keys.ToList()) w[k] /= total;
            return w;
        }

        /// <summary>
        /// Crude per-coupling load estimate: for every payload resting on both ends of the coupling, the weight imbalance between the two decks,
        /// V = W*|share_a - share_b|/2, is transferred as shear (split between the two seated hooks). PLACEHOLDER pending a real rigid-body solve.
        /// </summary>
        public HookLoads CouplingLoad(string couplingId)
        {
            var c = Couplings[couplingId];
            double v = 0;
            foreach (var p in Payloads.Values)
            {
                if (!p.SupportingRovers.Contains(c.RoverA) || !p.SupportingRovers.Contains(c.RoverB)) continue;
                var sh = DeckShares(p);
                v += p.Mass * Assume.Gravity * Math.Abs(sh[c.RoverA] - sh[c.RoverB]) / 2;
            }
            return Contact.StaticSplit(new LoadCase(0, v, 0), true, true);
        }

        // ------------------------------------------------------------ release invariant
        public ReleaseDecision CanRelease(string couplingId)
        {
            if (!Couplings.TryGetValue(couplingId, out var c)) return new ReleaseDecision { Allowed = false, Reason = "unknown coupling " + couplingId };
            if (c.Removed) return new ReleaseDecision { Allowed = false, Reason = "coupling already released" };
            var s = CheckSupport(couplingId);
            return new ReleaseDecision { Allowed = s.Ok, Reason = s.Ok ? "support verified on the remaining graph" : s.Reason };
        }

        /// <summary>
        /// Releases a coupling. Without <paramref name="force"/> a refused release emits ReleaseRefused and does nothing.
        /// With force the coupling is removed anyway and an UnsafeRelease event records the violation.
        /// </summary>
        public DockingEvent Release(string couplingId, bool force = false)
        {
            var d = CanRelease(couplingId);
            var c = Couplings.ContainsKey(couplingId) ? Couplings[couplingId] : null;
            var st = c?.State ?? ConnectorState.Free;
            if (d.Allowed)
            {
                c.Removed = true;
                return Log(new DockingEvent(Time, EventKind.Released, st, ConnectorState.Released, null, d.Reason, couplingId));
            }
            if (!force || c == null || c.Removed)
                return Log(new DockingEvent(Time, EventKind.ReleaseRefused, st, st, null, d.Reason, couplingId));
            c.Removed = true;
            return Log(new DockingEvent(Time, EventKind.UnsafeRelease, st, ConnectorState.Released, null, d.Reason, couplingId));
        }

        /// <summary>
        /// Call after stepping connector machines. If a coupling that was verified is no longer verified (e.g. VERIFICATION_LOST, hook slipped)
        /// and support is not verified on the graph as it now stands, an UnsafeRelease event is raised for it.
        /// </summary>
        public List<DockingEvent> Poll()
        {
            var raised = new List<DockingEvent>();
            foreach (var c in Couplings.Values)
            {
                bool now = c.IsVerified;
                if (c.WasVerified && !now && !c.Removed)
                {
                    var s = CheckSupport(null);
                    if (!s.Ok && DependsOn(c))
                        raised.Add(Log(new DockingEvent(Time, EventKind.UnsafeRelease, ConnectorState.LatchedVerified, c.State, null,
                            "load-bearing coupling lost verification: " + s.Reason, c.Id)));
                }
                c.WasVerified = now;
            }
            return raised;
        }

        // a coupling matters if some payload rests on both of its ends
        private bool DependsOn(Coupling c) => Payloads.Values.Any(p => p.SupportingRovers.Contains(c.RoverA) && p.SupportingRovers.Contains(c.RoverB));

        private DockingEvent Log(DockingEvent e) { Events.Add(e); return e; }
    }
}
