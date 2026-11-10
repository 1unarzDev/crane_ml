using System;
using Crane.Fleet.Autonomy.Tests.Harness;
using Xunit;
using Xunit.Abstractions;

namespace Crane.Fleet.Autonomy.Tests
{
    public class DebugRun
    {
        private readonly ITestOutputHelper _o;
        public DebugRun(ITestOutputHelper o) { _o = o; }

        /// <summary>Prints the event log of one scenario (CRANE_DEBUG_SCENARIO=nav|dock|line|crab|loaded|fail|sep|sepp, CRANE_DEBUG_SEED).</summary>
        [Fact]
        public void PrintScenario()
        {
            var name = Environment.GetEnvironmentVariable("CRANE_DEBUG_SCENARIO");
            if (string.IsNullOrEmpty(name)) return;
            ulong seed = ulong.Parse(Environment.GetEnvironmentVariable("CRANE_DEBUG_SEED") ?? "1");
            HarnessSetup s = name switch
            {
                "nav" => Missions.SingleNav(seed), "dock" => Missions.TwoRoverDock(seed), "line" => Missions.LineAssemblyTransport(seed),
                "crab" => Missions.CrabPair(seed), "loaded" => Missions.LoadedLine(seed, false), "fail" => Missions.LoadedLine(seed, true),
                "sep" => Missions.Separation(seed, false), "sepp" => Missions.Separation(seed, true), _ => throw new ArgumentException(name)
            };
            var r = Sim.Run(s);
            foreach (var e in r.Autonomy.Events) if (e.Kind != "connector_cmd" || true) _o.WriteLine(e.ToString());
            foreach (var c in r.Collisions) { _o.WriteLine("COLLISION " + c); break; }
            _o.WriteLine($"success={r.Success} class={r.Class} failure={r.Failure} t={r.Time:0.0} collisions={r.Collisions.Count}");
            foreach (var rv in r.World.Rovers) _o.WriteLine($"{rv.Spec.Id} truth={rv.Pose.X:0.000},{rv.Pose.Y:0.000},{rv.Pose.Yaw:0.000} est={rv.Estimate.X:0.000},{rv.Estimate.Y:0.000},{rv.Estimate.Yaw:0.000}");
            foreach (var p in r.World.Pairs) { _o.WriteLine($"pair {p.Machine.Id} {p.Machine.State} fails={p.Machine.FailedAttempts}"); foreach (var e in p.Machine.Log) _o.WriteLine("   " + e); }
        }
    }
}
