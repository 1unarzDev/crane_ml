using System.IO;
using Crane.Fleet.Autonomy.Tests.Harness;
using Crane.Fleet.Perception;
using Xunit;

namespace Crane.Fleet.Autonomy.Tests
{
    public class ProfileV1AutonomyTests
    {
        [Fact]
        public void RoverSpec_BuildsFromV1ForEveryRole()
        {
            var p = FleetProfile.Load(Path.Combine(Env.FleetDir, "spec", "fleet_profile_v1.json"));
            foreach (var m in p.Fleet)
            {
                var s = RoverSpec.FromProfile(p, m.Id, m.Role);
                Assert.Equal(2249, s.CountsPerRev); Assert.True(s.HasFrontCamera); Assert.False(s.HasRearCamera);
                Assert.Equal(m.Role == "lifter", s.HasLift);
                Assert.True(s.Mass > 4.2);
            }
        }
    }
}
