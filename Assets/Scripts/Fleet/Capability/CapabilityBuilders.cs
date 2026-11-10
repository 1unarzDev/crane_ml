using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Convenience constructors for common modules and the two Gazebo reference platforms.</summary>
    public static class CapabilityBuilders
    {
        public static CapWheelKind ParseKind(string s)
        {
            switch (s)
            {
                case "wheel_mecanum": case "mecanum": return CapWheelKind.Mecanum;
                case "wheel_diff": case "wheel_conventional": case "conventional": case "diff": return CapWheelKind.Conventional;
                case "wheel_omni": case "omni": return CapWheelKind.Omni;
                case "wheel_caster": case "caster": return CapWheelKind.Caster;
                default: throw new ArgumentException("unknown wheel kind " + s);
            }
        }

        public static CapHealth ParseHealth(string mode, double gain)
        {
            switch (mode)
            {
                case "ok": return CapHealth.Ok;
                case "dead_free": case "dead": return CapHealth.DeadFree;
                case "dead_locked": case "locked": return CapHealth.DeadLocked;
                case "weak": return CapHealth.Weak(gain);
                case "intermittent": return CapHealth.Intermittent(gain);
                default: throw new ArgumentException("unknown health mode " + mode);
            }
        }

        /// <summary>Four mecanum wheels in the standard X layout (rollers +45/-45), half-lengths lx, ly, wheel radius r.</summary>
        public static CapModule MecanumRover(string id, double lx, double ly, double r, double maxSpeed, double x = 0, double y = 0, double yaw = 0)
        {
            double g = Math.PI / 4;
            var m = new CapModule { Id = id, X = x, Y = y, Yaw = yaw };
            m.Wheels.Add(new CapWheel { Id = id + ".fl", Kind = CapWheelKind.Mecanum, X = lx, Y = ly, Radius = r, MaxSpeed = maxSpeed, RollerAngle = -g });
            m.Wheels.Add(new CapWheel { Id = id + ".fr", Kind = CapWheelKind.Mecanum, X = lx, Y = -ly, Radius = r, MaxSpeed = maxSpeed, RollerAngle = g });
            m.Wheels.Add(new CapWheel { Id = id + ".rl", Kind = CapWheelKind.Mecanum, X = -lx, Y = ly, Radius = r, MaxSpeed = maxSpeed, RollerAngle = g });
            m.Wheels.Add(new CapWheel { Id = id + ".rr", Kind = CapWheelKind.Mecanum, X = -lx, Y = -ly, Radius = r, MaxSpeed = maxSpeed, RollerAngle = -g });
            return m;
        }

        /// <summary>Two-wheel differential pod as in the Gazebo model (wheel centres +-0.07 m, radius 0.055 m, 24 rad/s limit).</summary>
        public static CapModule DifferentialPod(string id, double x, double y, double yaw, CapLateralMode lateral = CapLateralMode.Hard,
            double halfTrack = 0.07, double r = 0.055, double maxSpeed = 24.0, double normalLoad = 0.0, double lateralMu = 0.0)
        {
            var m = new CapModule { Id = id, X = x, Y = y, Yaw = yaw };
            m.Wheels.Add(new CapWheel { Id = id + ".l", Kind = CapWheelKind.Conventional, X = 0, Y = halfTrack, Radius = r, MaxSpeed = maxSpeed, Lateral = lateral, NormalLoad = normalLoad, LateralFriction = lateralMu });
            m.Wheels.Add(new CapWheel { Id = id + ".r", Kind = CapWheelKind.Conventional, X = 0, Y = -halfTrack, Radius = r, MaxSpeed = maxSpeed, Lateral = lateral, NormalLoad = normalLoad, LateralFriction = lateralMu });
            return m;
        }

        /// <summary>Pod slots (core frame) of the Gazebo morphologies, from morphologies.yaml: compact_diff and narrow_tandem.</summary>
        public static readonly Dictionary<string, double[][]> GazeboSlots = new Dictionary<string, double[][]>
        {
            { "compact_diff", new[] { new[] { 0.23, 0.20 }, new[] { 0.23, -0.20 }, new[] { -0.23, 0.20 }, new[] { -0.23, -0.20 }, new[] { 0.0, 0.20 }, new[] { 0.0, -0.20 } } },
            { "narrow_tandem", new[] { new[] { 0.71, 0.20 }, new[] { 0.71, -0.20 }, new[] { -0.71, 0.20 }, new[] { -0.71, -0.20 }, new[] { 0.35, 0.20 }, new[] { -0.35, -0.20 } } },
        };

        /// <summary>The six-pod differential platform of the Gazebo benchmark. Gazebo wheels have anisotropic friction (rolling mu 1.2,
        /// lateral mu2 0.08), so the physically representative form is lateral = Skid with lateralMu = 0.08; Hard gives the textbook
        /// no-slip form, whose rank-2 lateral constraint set predicts that the platform cannot turn.</summary>
        public static CapAssembly GazeboSixPod(string morphology, CapLateralMode lateral, double totalMassKg = 5.0 + 6 * 2.1, double lateralMu = 0.08)
        {
            var slots = GazeboSlots[morphology];
            var a = new CapAssembly { Id = "gazebo_" + morphology };
            double loadPerWheel = totalMassKg * 9.81 / (slots.Length * 2);
            for (int i = 0; i < slots.Length; i++)
                a.Modules.Add(DifferentialPod("pod_" + i, slots[i][0], slots[i][1], 0.0, lateral, normalLoad: loadPerWheel, lateralMu: lateralMu));
            return a;
        }
    }
}
