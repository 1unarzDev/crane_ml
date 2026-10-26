using System;
using System.Collections.Generic;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Boat;
using Sim.Physics.Water.Dynamics;
using Sim.Utils;
using UnityEngine;

namespace Sim.Tests.Physics {
    /// <summary>Shared fixtures: the Ideal profile's model, thrust sums and simulation loops.</summary>
    internal static class Kit {
        public static readonly double[] Dts = { 0.005, 0.01, 0.02, 0.04 };
        public const double ReferenceDt = 0.001;

        public static MarineSurfaceParameters Ideal() => new();
        public static MarinePlanarModel Model() => Ideal().PlanarModel();

        /// <summary>Four T200 at 45° along one axis: 4 · 51.5 · cos 45° ≈ 145.7 N.</summary>
        public static readonly double FullAxisThrust = 4 * 51.5 * Math.Sqrt(0.5);
        /// <summary>Full yaw: 4 · 51.5 · 0.61 · cos 45° ≈ 88.9 N m.</summary>
        public static readonly double FullYawMoment = 4 * 51.5 * (0.27 + 0.34) * Math.Sqrt(0.5);

        public static MarinePlanarModel.State Run(MarinePlanarModel m, MarinePlanarModel.State s,
            Func<double, Planar> thrust, double dt, double seconds, bool rk4 = false,
            double cx = 0, double cy = 0, Action<double, MarinePlanarModel.State> each = null) {
            int n = (int)Math.Round(seconds / dt);
            for (int k = 0; k < n; k++) {
                if (rk4) m.Rk4Step(ref s, thrust(k * dt), cx, cy, dt);
                else m.Step(ref s, thrust(k * dt), cx, cy, dt);
                each?.Invoke((k + 1) * dt, s);
            }
            return s;
        }

        /// <summary>Time for body axis <paramref name="j"/> to reach <paramref name="fraction"/>
        /// of <paramref name="target"/> from rest under a constant body thrust.</summary>
        public static double RiseTime(MarinePlanarModel m, Planar thrust, int j, double target, double fraction,
            double dt, bool rk4 = false, double limit = 120) {
            var s = new MarinePlanarModel.State();
            double t = 0;
            while (t < limit) {
                double before = s.Body[j];
                if (rk4) m.Rk4Step(ref s, thrust, 0, 0, dt);
                else m.Step(ref s, thrust, 0, 0, dt);
                t += dt;
                double after = s.Body[j];
                if (Math.Abs(after) >= fraction * Math.Abs(target))
                    return t - dt * (Math.Abs(after) - fraction * Math.Abs(target)) / Math.Max(1e-12, Math.Abs(after - before));
            }
            return double.NaN;
        }

        /// <summary>Body wrench of thruster models stepped with one pulse each (same math as ThrusterActuator).</summary>
        public static Planar Thrust(IReadOnlyList<ThrusterModel> models, float[] pulses, float dt) {
            double x = 0, y = 0, n = 0;
            for (int i = 0; i < models.Count; i++) {
                float t = models[i].Step(pulses[i], dt);
                Vector2 d = models[i].Spec.Direction;
                x += d.x * t;
                y += d.y * t;
                n += models[i].Spec.x * d.y * t - models[i].Spec.y * d.x * t;
            }
            return new Planar(x, y, n);
        }

        public static List<ThrusterModel> Thrusters(MarineSurfaceParameters p) =>
            p.thrusters.ConvertAll(spec => new ThrusterModel(spec.Clone()));

        /// <summary>Hardware mixer rows (FL, FR, RR, RL) of surge, sway, yaw → pulses.</summary>
        public static float[] Pulses(float surge, float sway, float yaw) {
            float[][] rows = { new[] { -1f, 1f, 1f }, new[] { -1f, -1f, -1f }, new[] { 1f, -1f, 1f }, new[] { 1f, 1f, -1f } };
            var p = new float[4];
            for (int i = 0; i < 4; i++)
                p[i] = 1500f + 400f * Mathf.Clamp(rows[i][0] * surge + rows[i][1] * sway + rows[i][2] * yaw, -1f, 1f);
            return p;
        }
    }

    /// <summary>A plant on a test articulation: given models, no scene.</summary>
    internal sealed class TestPlant : PlantBase {
        public void Setup(ArticulationBody root, IBodyFrame bodyFrame, IMedium water, IInertiaCoupling inertia,
            params IForceModel[] models) {
            body = new ArticulationBodyAdapter(root);
            frame = bodyFrame;
            medium = water;
            coupling = inertia;
            forces.Clear();
            forces.AddRange(models);
        }
    }

    /// <summary>A constant body-frame force model (a stand-in thrust).</summary>
    internal sealed class ConstantForce : IForceModel {
        public Wrench Value;
        public WrenchRoute Route => WrenchRoute.Hydro;
        public void Accumulate(in PlantState s, ref Wrench w) => w += Value;
    }

    internal sealed class FlowMedium : IMedium {
        public Vector3 Flow;
        public float Density => 997f;
        public Vector3 FlowAt(Vector3 p, double t) => Flow;
        public float SurfaceHeight(Vector3 p) => 0f;
    }
}
