using System;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Dynamics;
using UnityEngine;

namespace Sim.Physics.Water.Boat {
    /// <summary>
    /// Physical errors for held-out characterization tests, applied as a layer after the profile
    /// (never into it) and keyed by thruster name. Reads the calibration-fault JSON: arrays are in
    /// <see cref="names"/> order (default FL, FR, RR, RL); an absent field changes nothing. No
    /// controller or sensor is told the values.
    /// </summary>
    [Serializable]
    public sealed class ActuatorErrorProfile {
        public string[] names = { "front_left", "front_right", "rear_right", "rear_left" };
        public float[] strengths;   // force multipliers; 0 is a dead thruster
        public float[] angles;      // mount yaw error, deg, Unity sense (clockwise from above)
        public float[] reverse;     // reverse / forward thrust after strength
        public float[] deadband_us; // added ESC deadband [µs]
        public float[] offset_us;   // added ESC neutral offset [µs]
        public float[] deadtime_s;  // added per-thruster dead time [s]
        public int deadtime_ticks;  // added dead time on every thruster, physics ticks
        public float spinup = float.NaN;  // thrust lag τ_T [s] on every thruster
        public float[] drag;        // surge, sway, yaw multipliers (>= 1; extra drag only)
        public float[] dragOffset;  // bow, port [m] where the extra drag acts
        public float mass = float.NaN;    // kg, total
        public float izz = float.NaN;     // kg m² about the CG
        public float[] com;         // bow, port [m] CG shift

        public static ActuatorErrorProfile Parse(string json) {
            ActuatorErrorProfile e = JsonUtility.FromJson<ActuatorErrorProfile>(json);
            int n = e.names?.Length ?? 0;
            Check(e.strengths, n, 0, 4, "strengths");
            Check(e.angles, n, -45, 45, "angles");
            Check(e.reverse, n, 0.2f, 1.5f, "reverse");
            Check(e.deadband_us, n, 0, 200, "deadband_us");
            Check(e.offset_us, n, -100, 100, "offset_us");
            Check(e.deadtime_s, n, 0, 2, "deadtime_s");
            Check(e.drag, 3, 1, 20, "drag");
            Check(e.dragOffset, 2, -1, 1, "dragOffset");
            Check(e.com, 2, -0.3f, 0.3f, "com");
            Range(e.mass, 10, 80, "mass");
            Range(e.izz, 0.5f, 20, "izz");
            Range(e.spinup, 0.001f, 2, "spinup");
            Range(e.deadtime_ticks, 0, 100, "deadtime_ticks");
            return e;
        }

        private static void Check(float[] values, int length, float min, float max, string name) {
            if (values == null || values.Length == 0) return;
            if (values.Length != length) throw new ArgumentException($"{name} requires {length} values");
            foreach (float value in values) Range(value, min, max, name);
        }

        private static void Range(float value, float min, float max, string name) {
            if (float.IsNaN(value) && (name == "mass" || name == "izz" || name == "spinup")) return;
            if (!float.IsFinite(value) || value < min || value > max)
                throw new ArgumentException($"{name} outside [{min}, {max}]");
        }

        private static float At(float[] values, int i, float fallback) =>
            values != null && values.Length > i ? values[i] : fallback;

        /// <summary>The effective parameters: a copy of <paramref name="nominal"/> with the errors.</summary>
        public MarineSurfaceParameters Apply(MarineSurfaceParameters nominal, float dt) {
            MarineSurfaceParameters p = nominal.Clone();
            for (int i = 0; i < names.Length; i++) {
                ThrusterSpec t = p.thrusters.Find(s => s.name == names[i]) ??
                    throw new ArgumentException($"no thruster named {names[i]} in profile {p.name}");
                float strength = At(strengths, i, 1f);
                t.kForward *= strength;
                t.kReverse = reverse != null && reverse.Length > i ? t.kForward * reverse[i] : t.kReverse * strength;
                // Unity rotates about +y (clockwise seen from above); FLU yaw is counter-clockwise.
                t.alphaDeg -= At(angles, i, 0f);
                t.deadbandUs += At(deadband_us, i, 0f);
                t.offsetUs += At(offset_us, i, 0f);
                t.deadTime += At(deadtime_s, i, 0f) + deadtime_ticks * dt;
                if (float.IsFinite(spinup)) t.tauT = spinup;
            }
            if (float.IsFinite(mass)) p.mass = mass;
            if (float.IsFinite(izz)) p.izz = izz;
            if (com != null && com.Length == 2) {
                p.comBow += com[0];
                p.comPort += com[1];
            }
            return p;
        }

        /// <summary>The extra drag force model, or null when the drag multipliers are all 1.</summary>
        public IForceModel DragModel(MarinePlanarModel nominal) {
            if (drag == null || drag.Length != 3 || (drag[0] == 1 && drag[1] == 1 && drag[2] == 1)) return null;
            return new ErrorDrag(nominal, new Vector3(drag[0] - 1, drag[1] - 1, drag[2] - 1),
                new Vector2(At(dragOffset, 0, 0f), At(dragOffset, 1, 0f)));
        }
    }

    /// <summary>Extra hull drag (k − 1)·D at an offset point (bow, port), an error-layer force
    /// with its Jacobian so it steps implicitly like the nominal drag.</summary>
    public sealed class ErrorDrag : IForceModel, IDampingModel {
        private readonly MarinePlanarModel nominal;
        private readonly Vector3 extra;
        private readonly Vector2 offset;

        public ErrorDrag(MarinePlanarModel nominal, Vector3 extra, Vector2 offset) {
            this.nominal = nominal;
            this.extra = extra;
            this.offset = offset;
        }

        public WrenchRoute Route => WrenchRoute.Hydro;

        private Vector3 Local(in PlantState s) {
            Vector3 vr = s.RelativeLinear;
            float r = s.Angular.z;
            return new Vector3(vr.x - r * offset.y, vr.y + r * offset.x, r);
        }

        public void Accumulate(in PlantState s, ref Wrench w) {
            Vector3 l = Local(s);
            var force = new Vector3(-extra.x * (float)nominal.Surge.Force(l.x), -extra.y * (float)nominal.Sway.Force(l.y), 0f);
            w.AddAt(force, new Vector3(offset.x, offset.y, 0f));
            w.Torque.z -= extra.z * (float)nominal.Yaw.Force(l.z);
        }

        public void AccumulateJacobian(in PlantState s, ref Wrench j) {
            Vector3 l = Local(s);
            float ju = extra.x * (float)nominal.Surge.Jacobian(l.x), jv = extra.y * (float)nominal.Sway.Jacobian(l.y);
            j.Force.x += ju;
            j.Force.y += jv;
            j.Torque.z += extra.z * (float)nominal.Yaw.Jacobian(l.z) + ju * offset.y * offset.y + jv * offset.x * offset.x;
        }
    }
}
