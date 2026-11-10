using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Crane.Fleet;
using Crane.Fleet.Docking;
using Crane.Fleet.Profile;

namespace Crane.Fleet.Docking.Tests
{
    public static class Fx
    {
        public static string ProfilePath()
        {
            string dir = AppContext.BaseDirectory;
            while (dir != null)
            {
                string p = Path.Combine(dir, "spec", "fleet_profile_v0.json");
                if (File.Exists(p)) return p;
                string q = Path.Combine(dir, "Tools", "Fleet", "spec", "fleet_profile_v0.json");
                if (File.Exists(q)) return q;
                dir = Path.GetDirectoryName(dir);
            }
            throw new FileNotFoundException("fleet_profile_v0.json not found above " + AppContext.BaseDirectory);
        }

        public static string ProfilePathV1()
        {
            string p0 = ProfilePath();
            return Path.Combine(Path.GetDirectoryName(p0), "fleet_profile_v1.json");
        }
        public static string ProfileJsonV1() => File.ReadAllText(ProfilePathV1());
        public static DockingProfile ProfileV1() => ProfileLoader.LoadOrThrow(ProfileJsonV1());
        public static ContactModel ModelV1(DockingAssumptions a = null) => new ContactModel(ProfileV1(), a ?? Det());
        public static ConnectorMachine MachineV1(DockingAssumptions a = null, ulong seed = 1) => new ConnectorMachine(ModelV1(a), seed);

        public static string ProfileJson() => File.ReadAllText(ProfilePath());
        public static DockingProfile Profile() => ProfileLoader.LoadOrThrow(ProfileJson());

        /// <summary>Noise-free, no-centring assumptions: the settled residual equals the terminal error and nothing random happens.</summary>
        public static DockingAssumptions Det() => new DockingAssumptions().Deterministic();

        public static ContactModel Model(DockingAssumptions a = null) => new ContactModel(Profile(), a ?? Det());
        public static ConnectorMachine Machine(DockingAssumptions a = null, ulong seed = 1) => new ConnectorMachine(Model(a), seed);

        public const double Dt = 0.05;

        public static List<DockingEvent> Run(ConnectorMachine m, ConnectorInputs inp, double seconds)
        {
            var ev = new List<DockingEvent>();
            int n = (int)Math.Round(seconds / Dt);
            for (int i = 0; i < n; i++) ev.AddRange(m.Step(Dt, inp).Events);
            return ev;
        }

        public static ConnectorInputs At(double lat = 0, double yaw = 0, double gap = 0.005, double height = 0)
            => new ConnectorInputs { Pose = new FacePose(lat, gap, yaw, height) };

        /// <summary>Approach, capture, then engage hooks and run until the hook phase is over (2 s).</summary>
        public static ConnectorInputs Capture(ConnectorMachine m, double lat = 0, double yaw = 0)
        {
            var inp = At(lat, yaw, 0.1);
            inp.Command = DockingCommand.Approach;
            m.Step(Dt, inp);
            inp.Command = DockingCommand.None; inp.Pose.Gap = 0.004;
            m.Step(Dt, inp);
            return inp;
        }

        public static ConnectorInputs Engage(ConnectorMachine m, double lat = 0, double yaw = 0, double seconds = 2.0)
        {
            var inp = Capture(m, lat, yaw);
            inp.Command = DockingCommand.EngageHooks;
            Run(m, inp, seconds);
            return inp;
        }

        public static ConnectorInputs Latch(ConnectorMachine m, double lat = 0, double yaw = 0)
        {
            var inp = Engage(m, lat, yaw);
            inp.Command = DockingCommand.Verify;
            m.Step(Dt, inp);
            inp.Command = DockingCommand.None;
            return inp;
        }
    }
}
