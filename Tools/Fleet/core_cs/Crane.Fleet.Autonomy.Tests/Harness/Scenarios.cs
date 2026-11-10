using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using Crane.Fleet.Perception;
using DockProfile = Crane.Fleet.Profile.DockingProfile;

namespace Crane.Fleet.Autonomy.Tests.Harness
{
    public static class Env
    {
        public static string FleetDir
        {
            get
            {
                var d = new DirectoryInfo(AppContext.BaseDirectory);
                while (d != null && !File.Exists(Path.Combine(d.FullName, "spec", "fleet_profile_v0.json"))) d = d.Parent;
                if (d == null) throw new FileNotFoundException("Tools/Fleet/spec not found above " + AppContext.BaseDirectory);
                return d.FullName;
            }
        }
        public static string ProfileJson => File.ReadAllText(Path.Combine(FleetDir, "spec", "fleet_profile_v0.json"));
        private static FleetProfile _p, _pAll; private static DockProfile _dp;
        public static FleetProfile Profile => _p ?? (_p = FleetProfile.Parse(ProfileJson));
        /// <summary>Profile with tags on all four faces active (side-by-side and L docking need left/right face tags).</summary>
        public static FleetProfile ProfileAllFaces => _pAll ?? (_pAll = FleetProfile.Parse(ProfileJson.Replace("\"active_faces_default\": [\"front\", \"rear\"]", "\"active_faces_default\": [\"front\", \"rear\", \"left\", \"right\"]")));
        public static DockProfile Docking => _dp ?? (_dp = Crane.Fleet.Profile.ProfileLoader.LoadOrThrow(ProfileJson));
        public static RoverSpec Spec(string id, string role, FleetProfile p = null) => RoverSpec.FromProfile(p ?? Profile, id, role);
    }

    public sealed class RoverSetup { public string Id, Role; public Pose2 Truth, Believed; }

    public sealed class HarnessSetup
    {
        public ulong Seed = 1;
        public PlanMap Map;
        public readonly List<RoverSetup> Rovers = new List<RoverSetup>();
        public readonly List<string[]> Pairs = new List<string[]>();       // docker, face, target, face
        public readonly List<string[]> PreLatched = new List<string[]>();
        public string MissionJson;
        public double Dt = 0.02, MaxTime = 240;
        public bool Overhead;
        public Action<KinematicWorld, FleetAutonomy, double> Hook;
        public Action<KinematicWorld> WorldInit;
        public FleetProfile Profile;
        public bool IdentifyHealth = true;
        public double InitialPosSigma = 0.02, InitialYawSigma = 0.02;
        public double CameraRate = 20;
        public Action<FleetConfig> ConfigHook;
        public readonly List<(string name, Pose2 pose)> Landmarks = new List<(string, Pose2)>();
        /// <summary>Host (evaluator) check of a believed-complete mission from TRUTH; default: goals reached, payloads not dropped and on their decks.</summary>
        public Func<KinematicWorld, MissionSpec, string> HostCheck;
        public double HostGoalTol = 0.12;
        /// <summary>Boxes only the autonomy's planning map contains (CRANE adds the payload to the map; physically it is not structure).</summary>
        public readonly List<Box2> PlannerOnlyBoxes = new List<Box2>();
    }

    public sealed class HarnessResult
    {
        public bool Success, BelievedComplete; public FailureClass Class; public string Failure; public double Time;
        public List<string> Collisions; public int UnsafeReleases;
        public KinematicWorld World; public FleetAutonomy Autonomy;
        public string Category => Success ? (Collisions.Count > 0 ? "success_with_contact" : "success") : (Class + (Collisions.Count > 0 ? "+collision" : ""));
    }

    public static class Sim
    {
        public static PlanMap Warehouse(double doorWidth = 0.80, bool pallet = true)
        {
            var m = new PlanMap(0, 0, 10, 7);
            m.AddWallWithDoor("divider", 6.0, 0.15, 3.5, doorWidth);
            if (pallet) m.AddBox("pallet_stack", 4.6, 1.6, 1.0, 0.8);
            m.AddBox("shelf_a", 2.8, 5.4, 0.5, 2.4);
            return m;
        }

        public static HarnessResult Run(HarnessSetup s)
        {
            var prof = s.Profile ?? Env.Profile;
            var world = new KinematicWorld(prof, Env.Docking, s.Map, s.Seed) { CameraRateHz = s.CameraRate };
            var clock = new HarnessClock();
            var planMap = s.Map;
            if (s.PlannerOnlyBoxes.Count > 0) { planMap = new PlanMap(s.Map.MinX, s.Map.MinY, s.Map.MaxX, s.Map.MaxY); planMap.Obstacles.AddRange(s.Map.Obstacles); planMap.Obstacles.AddRange(s.PlannerOnlyBoxes); }
            var cfg = new FleetConfig { Map = planMap, DockingProfile = Env.Docking, IdentifyWheelHealth = s.IdentifyHealth, InitialPosSigma = s.InitialPosSigma, InitialYawSigma = s.InitialYawSigma };
            var srcs = new List<IObservationSource>(); var sinks = new List<IActuatorSink>();
            foreach (var r in s.Rovers)
            {
                var spec = RoverSpec.FromProfile(prof, r.Id, r.Role);
                var wr = world.AddRover(spec, r.Truth);
                cfg.Rovers.Add(spec); cfg.InitialPoses[r.Id] = r.Believed;
                srcs.Add(new HarnessSource(world, wr)); sinks.Add(new HarnessSink(wr));
            }
            foreach (var p in s.Pairs) world.AddPair(p[0], p[1], p[2], p[3]);
            foreach (var p in s.PreLatched) { var pr = world.AddPair(p[0], p[1], p[2], p[3]); world.PreLatch(pr); }
            if (s.Overhead)
            {
                world.Overhead = true;
                world.OverheadPose = CameraMount.LookAt(new Vec3(4.5, 3.0, 2.6), new Vec3(4.5, 3.4, 0));
                world.OverheadK = CameraIntrinsics.FromHfov(1920, 1080, prof.Overhead.Hfov);
                var rng = Rng.ForStream(s.Seed, "survey");
                world.OverheadSurveyError = new Pose2(0.004 * rng.Gaussian(), 0.004 * rng.Gaussian(), 0.0035 * rng.Gaussian());
            }
            foreach (var (n, p) in s.Landmarks) { world.AddLandmark(n, p); cfg.Landmarks[n] = p; }
            s.WorldInit?.Invoke(world);
            s.ConfigHook?.Invoke(cfg);
            // pre-latched groups: believed poses consistent with the mated geometry
            var fa = new FleetAutonomy(cfg, clock, srcs, sinks);
            var mission = MissionSpec.Parse(s.MissionJson);
            fa.LoadMission(mission);
            while (world.Time < s.MaxTime && !fa.Executive.Done)
            {
                world.Step(s.Dt);
                clock.Now = world.Time;
                s.Hook?.Invoke(world, fa, world.Time);
                fa.Step(s.Dt);
            }
            if (fa.Executive.BelievedComplete)
            {
                string verdict = (s.HostCheck ?? ((w, m) => DefaultHostCheck(w, m, s.HostGoalTol)))(world, mission);
                fa.Executive.ConfirmByHost(verdict == null, verdict ?? "evaluator: goals reached, payloads supported");
            }
            var res = new HarnessResult { World = world, Autonomy = fa, Time = world.Time, Collisions = world.Collisions, UnsafeReleases = world.UnsafeReleases };
            res.Success = fa.Executive.Done && fa.Executive.Succeeded;
            res.BelievedComplete = fa.Executive.BelievedComplete;
            if (!res.Success)
            {
                res.Class = fa.Executive.Done ? fa.Executive.FirstFailureClass : FailureClass.Timeout;
                res.Failure = fa.Executive.FirstFailure ?? "mission time limit";
            }
            return res;
        }

        /// <summary>Evaluator check from truth (what CRANE's harness must do): every goto/transport goal reached by the named rover, no payload
        /// dropped, every handoff payload resting on its supporters. Returns null when the outcome is confirmed, else the reason.</summary>
        public static string DefaultHostCheck(KinematicWorld w, MissionSpec m, double tol)
        {
            var last = new Dictionary<string, TaskSpec>();   // only the last goal of each rover must hold at the end
            foreach (var t in m.Tasks) if ((t.Type == "goto" || t.Type == "transport") && t.Nums("goal") != null) last[t.Str("agent")] = t;
            foreach (var t in m.Tasks)
            {
                if ((t.Type == "goto" || t.Type == "transport") && t.Nums("goal") != null && last[t.Str("agent")] == t)
                {
                    var g = t.PoseOf("goal"); var r = w.R(t.Str("agent"));
                    double tl = Math.Max(tol, t.Nums("tol") != null ? t.Nums("tol")[0] + 0.05 : 0);
                    double d = Se2.Distance(r.Pose, g);
                    if (d > tl) return $"task {t.Id}: {t.Str("agent")} is {d:0.000} m from its goal (truth)";
                }
                if (t.Type == "handoff")
                {
                    var p = w.Payloads.Find(x => x.Id == t.Str("payload"));
                    if (p == null || p.Dropped || p.CarriedBy == null || !t.Strs("supporters").Contains(p.CarriedBy)) return $"task {t.Id}: payload {t.Str("payload")} not on the decks (truth)";
                }
            }
            foreach (var p in w.Payloads) if (p.Dropped) return "payload " + p.Id + " fell to the floor (truth)";
            return null;
        }

        public static string Tally(IEnumerable<HarnessResult> rs, out int ok, out int n)
        {
            var d = new SortedDictionary<string, int>(); ok = 0; n = 0;
            foreach (var r in rs) { n++; if (r.Success && r.Collisions.Count == 0) ok++; d.TryGetValue(r.Category, out int c); d[r.Category] = c + 1; }
            var sb = new StringBuilder();
            foreach (var kv in d) sb.Append(kv.Key).Append('=').Append(kv.Value).Append(' ');
            return sb.ToString().Trim();
        }

        public static void Report(string name, IEnumerable<HarnessResult> rs, Func<HarnessResult, string> extra = null)
        {
            var list = new List<HarnessResult>(rs);
            string t = Tally(list, out int ok, out int n);
            var sb = new StringBuilder();
            sb.Append($"{name}: {ok}/{n} clean success; classes: {t}");
            foreach (var r in list) if (!r.Success || r.Collisions.Count > 0) sb.Append($"\n   - seed fail: {r.Failure} {(r.Collisions.Count > 0 ? r.Collisions[0] : "")}");
            if (extra != null) foreach (var r in list) { var e = extra(r); if (e != null) sb.Append("\n   ").Append(e); }
            string dir = Environment.GetEnvironmentVariable("CRANE_AUTONOMY_REPORT");
            if (!string.IsNullOrEmpty(dir)) { Directory.CreateDirectory(dir); File.AppendAllText(Path.Combine(dir, "harness_report.txt"), sb + "\n"); }
            Console.WriteLine(sb.ToString());
        }

        // ------------------------------------------------------------------------------------- missions
        public static string GotoMission(string agent, Pose2 goal) =>
            "{\"schema\":\"crane.fleet_mission/v0\",\"mission_id\":\"nav\",\"tasks\":[{\"id\":\"go\",\"type\":\"goto\",\"agent\":\"" + agent + "\",\"goal\":[" + F(goal.X) + "," + F(goal.Y) + "," + F(goal.Yaw) + "]}]}";

        public static string F(double x) => x.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
    }
}
