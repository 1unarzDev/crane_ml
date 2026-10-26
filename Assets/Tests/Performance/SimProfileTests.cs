using NUnit.Framework;
using Sim.Performance;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Tests.Performance {
    public class SimProfileTests {
        private static readonly string[] s_Tiers = { "Full", "Balanced", "Minimal" };

        private static CraneSimProfile Load(string name) {
            var profile = Resources.Load<CraneSimProfile>("SimProfiles/" + name);
            Assert.IsNotNull(profile, $"SimProfiles/{name} missing from Resources");
            return profile;
        }

        // Buoyancy queries HDRP water, so every tier must simulate it exactly like Full.
        [Test]
        public void EveryTierKeepsFullWaterPhysics() {
            var full = (HDRenderPipelineAsset)Load("Full").Pipeline;
            RenderPipelineSettings reference = full.currentPlatformRenderPipelineSettings;
            foreach (string name in s_Tiers) {
                var pipeline = Load(name).Pipeline as HDRenderPipelineAsset;
                Assert.IsNotNull(pipeline, $"{name} needs an HDRP asset");
                RenderPipelineSettings settings = pipeline.currentPlatformRenderPipelineSettings;
                Assert.IsTrue(settings.supportWater, $"{name} ({pipeline.name}) disables water");
                Assert.AreEqual(reference.waterSimulationResolution,
                    settings.waterSimulationResolution, name);
                Assert.AreEqual(reference.waterScriptInteractionsMode,
                    settings.waterScriptInteractionsMode, name);
                Assert.AreEqual(reference.waterFullCPUSimulation,
                    settings.waterFullCPUSimulation, name);
            }
        }

        // A lower tier never costs more: 0 Hz means "scene value", the most expensive choice.
        [Test]
        public void TiersOnlyGetCheaper() {
            static float Rate(float hz) => hz > 0f ? hz : float.PositiveInfinity;
            for (int i = 1; i < s_Tiers.Length; i++) {
                CraneSimProfile upper = Load(s_Tiers[i - 1]);
                CraneSimProfile lower = Load(s_Tiers[i]);
                string pair = $"{s_Tiers[i]} vs {s_Tiers[i - 1]}";
                Assert.LessOrEqual(lower.FrameRateCap, upper.FrameRateCap, pair);
                Assert.LessOrEqual(lower.CameraResolutionScale, upper.CameraResolutionScale, pair);
                Assert.LessOrEqual(Rate(lower.CameraHz), Rate(upper.CameraHz), pair);
                Assert.LessOrEqual(Rate(lower.DetectionHz), Rate(upper.DetectionHz), pair);
                Assert.LessOrEqual(Rate(lower.LidarHz), Rate(upper.LidarHz), pair);
                Assert.IsTrue(lower.ReducedEffects || !upper.ReducedEffects, pair);
                Assert.IsTrue(!lower.WaterDetail || upper.WaterDetail, pair);
                Assert.IsTrue(!lower.Spectator || upper.Spectator, pair);
                Assert.GreaterOrEqual(lower.FrameRateCap, Rate(lower.CameraHz) == float.PositiveInfinity
                    ? 0f : lower.CameraHz, $"{s_Tiers[i]} frame cap below its camera rate");
            }
        }
    }
}
