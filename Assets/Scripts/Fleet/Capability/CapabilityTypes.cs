using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Wheel families of the capability descriptor (wheel_mecanum, wheel_diff/conventional, wheel_omni, wheel_caster).</summary>
    public enum CapWheelKind { Mecanum, Conventional, Omni, Caster }

    /// <summary>Actuator health modes of descriptor v0.1. Dead-free rolls freely; dead-locked cannot rotate.</summary>
    public enum CapHealthMode { Ok, DeadFree, DeadLocked, Weak, Intermittent }

    /// <summary>How a conventional wheel's lateral no-slip row is treated.
    /// Hard: kinematic constraint (Campion et al. 1996 C1 row). Skid: lateral scrub allowed at a Coulomb cost (skid-steer
    /// arrays such as the six-pod Gazebo platform; the row moves from the hard set to the soft set).</summary>
    public enum CapLateralMode { Hard, Skid }

    public readonly struct CapHealth
    {
        public readonly CapHealthMode Mode;
        /// <summary>Weak: speed/torque gain in [0,1]. Intermittent: availability probability p in [0,1]. Otherwise 1.</summary>
        public readonly double Gain;
        public CapHealth(CapHealthMode mode, double gain = 1.0) { Mode = mode; Gain = gain; }
        public static CapHealth Ok => new CapHealth(CapHealthMode.Ok);
        public static CapHealth DeadFree => new CapHealth(CapHealthMode.DeadFree);
        public static CapHealth DeadLocked => new CapHealth(CapHealthMode.DeadLocked);
        public static CapHealth Weak(double gain) => new CapHealth(CapHealthMode.Weak, gain);
        public static CapHealth Intermittent(double p) => new CapHealth(CapHealthMode.Intermittent, p);
    }

    /// <summary>One wheel. Pose and drive direction are in the owning module frame. Units: m, rad, rad/s.</summary>
    public sealed class CapWheel
    {
        public string Id;
        public CapWheelKind Kind = CapWheelKind.Conventional;
        public double X, Y;
        /// <summary>Angle of the wheel-plane rolling direction d from the module x axis (rad, CCW).</summary>
        public double DriveYaw;
        public double Radius = 0.05;
        /// <summary>Mecanum only: signed angle (rad) from d to the roller axis a (roller rolls freely perpendicular to a);
        /// handedness is the sign, typically +-pi/4.</summary>
        public double RollerAngle = Math.PI / 4;
        /// <summary>Wheel speed limit (rad/s) at full health.</summary>
        public double MaxSpeed = 20.0;
        /// <summary>False for a wheel that has no motor by design (it behaves like a dead-free wheel).</summary>
        public bool Driven = true;
        public CapHealth Health = CapHealth.Ok;
        public CapLateralMode Lateral = CapLateralMode.Hard;
        /// <summary>Normal load (N) and lateral friction coefficient, used only by the soft (skid) scrub model.</summary>
        public double NormalLoad = 0.0;
        public double LateralFriction = 0.0;

        public CapWheel Clone()
        {
            return (CapWheel)MemberwiseClone();
        }
    }

    /// <summary>A rigid module carrying wheels; pose in the assembly frame.</summary>
    public sealed class CapModule
    {
        public string Id;
        public double X, Y, Yaw;
        public List<CapWheel> Wheels = new List<CapWheel>();
    }

    /// <summary>Rigidly coupled modules. All wheels share one planar body twist (vx, vy, wz) expressed in the assembly frame
    /// at the assembly origin.</summary>
    public sealed class CapAssembly
    {
        public string Id = "assembly";
        public List<CapModule> Modules = new List<CapModule>();

        public CapWheel FindWheel(string id)
        {
            foreach (var m in Modules) foreach (var w in m.Wheels) if (w.Id == id) return w;
            return null;
        }

        public CapAssembly Clone()
        {
            var a = new CapAssembly { Id = Id };
            foreach (var m in Modules)
            {
                var c = new CapModule { Id = m.Id, X = m.X, Y = m.Y, Yaw = m.Yaw };
                foreach (var w in m.Wheels) c.Wheels.Add(w.Clone());
                a.Modules.Add(c);
            }
            return a;
        }

        /// <summary>Returns a copy with the given wheel's health replaced (fault injection / hypothesis evaluation).</summary>
        public CapAssembly WithHealth(string wheelId, CapHealth health)
        {
            var a = Clone();
            var w = a.FindWheel(wheelId);
            if (w == null) throw new ArgumentException("unknown wheel " + wheelId);
            w.Health = health;
            return a;
        }

        public IEnumerable<CapWheel> AllWheels()
        {
            foreach (var m in Modules) foreach (var w in m.Wheels) yield return w;
        }
    }

    public sealed class CapOptions
    {
        /// <summary>Relative eigenvalue threshold of the 3x3 Gram matrices used for rank decisions.</summary>
        public double RankTolerance = 1e-12;
        /// <summary>Intermittent(p): false = expected authority (limits scaled by p); true = worst case (treated as dead-free).</summary>
        public bool IntermittentWorstCase = false;
        public static CapOptions Default => new CapOptions();
    }
}
