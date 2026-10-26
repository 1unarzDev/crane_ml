using System;
using System.IO;
using NUnit.Framework;
using Sim.Actuators.Motors;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Dynamics;
using UnityEngine;

namespace Sim.Tests.Physics {
    /// <summary>C#/Python parity: MarinePlanarModel + ThrusterModel against the Python twin's
    /// cases (Tools/Characterization/parity/cases.json, written by parity.py).</summary>
    public sealed class ParityTests {
        private const string Root = "Tools/Characterization/";

        [Serializable] private sealed class Case {
            public string name;
            public double dt;
            public int steps;
            public double currentX, currentY;
            public double[] initialState, pwm, expectedTimes, expected;
            public int[] expectedTicks;
        }

        [Serializable] private sealed class Cases {
            public string schema, profileFile, profileSha256;
            public double relTolerance, absTolerance;
            public Case[] cases;
        }

        [Test]
        public void CSharpMatchesThePythonTwin() {
            var file = JsonUtility.FromJson<Cases>(File.ReadAllText(Root + "parity/cases.json"));
            Assert.That(file.schema, Is.EqualTo("crane.parity/1"));
            string profileJson = File.ReadAllText(Root + file.profileFile);
            Assert.That(Sim.Physics.Core.ProfileBootstrap.Sha256(profileJson), Is.EqualTo(file.profileSha256),
                "cases.json is stale: rerun parity.py");
            var p = JsonUtility.FromJson<MarineSurfaceParameters>(profileJson);
            double worst = 0;
            foreach (Case c in file.cases) {
                MarinePlanarModel model = p.PlanarModel();
                var thrusters = Kit.Thrusters(p);
                var s = new MarinePlanarModel.State {
                    X = c.initialState[0], Y = c.initialState[1], Psi = c.initialState[2],
                    Vx = c.initialState[3], Vy = c.initialState[4], R = c.initialState[5]
                };
                int sample = 0;
                var pulses = new float[4];
                for (int k = 1; k <= c.steps && sample < c.expectedTicks.Length; k++) {
                    for (int i = 0; i < 4; i++) pulses[i] = (float)c.pwm[4 * (k - 1) + i];
                    model.Step(ref s, Kit.Thrust(thrusters, pulses, (float)c.dt), c.currentX, c.currentY, c.dt);
                    if (k != c.expectedTicks[sample]) continue;
                    double[] got = { s.X, s.Y, s.Psi, s.Vx, s.Vy, s.R };
                    for (int j = 0; j < 6; j++) {
                        double want = c.expected[6 * sample + j];
                        double error = Math.Abs(got[j] - want);
                        worst = Math.Max(worst, error / (file.absTolerance + file.relTolerance * Math.Abs(want)));
                        Assert.That(error, Is.LessThanOrEqualTo(file.absTolerance + file.relTolerance * Math.Abs(want)),
                            $"{c.name} t={c.expectedTimes[sample]} component {j}: {got[j]} vs {want}");
                    }
                    sample++;
                }
                Assert.That(sample, Is.EqualTo(c.expectedTicks.Length), c.name);
            }
            UnityEngine.Debug.Log($"PHYS_PARITY cases={file.cases.Length} worst_fraction_of_tolerance={worst:E3}");
        }
    }
}
