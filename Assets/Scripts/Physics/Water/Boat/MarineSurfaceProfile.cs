using System;
using System.Collections.Generic;
using System.Linq;
using Sim.Actuators.Motors;
using Sim.Physics.Core;
using Sim.Physics.Water.Dynamics;
using UnityEngine;

namespace Sim.Physics.Water.Boat {
    [Serializable]
    public sealed class SignedDamping {
        [Tooltip("Linear term for positive / negative motion [N s/m or N m s/rad]")] public float d1Pos, d1Neg;
        [Tooltip("Quadratic term for positive / negative motion [N s²/m² or N m s²/rad²]")] public float d2Pos, d2Neg;
        public AxisDamping Axis => new(d1Pos, d1Neg, d2Pos, d2Neg);
    }

    [Serializable]
    public sealed class SymmetricDamping {
        public float d1, d2;
        public AxisDamping Axis => AxisDamping.Symmetric(d1, d2);
    }

    [Serializable]
    public sealed class AddedMassTerms {
        [Tooltip("Surge added mass [kg]")] public float a11;
        [Tooltip("Sway added mass [kg]")] public float a22;
        [Tooltip("Sway-yaw coupling [kg m]")] public float a26;
        [Tooltip("Yaw added inertia [kg m²]")] public float a66;
    }

    [Serializable]
    public sealed class ProfileEnvelope {
        [Tooltip("m/s")] public float surge, sway;
        [Tooltip("rad/s")] public float yaw;
    }

    /// <summary>
    /// The numbers of a marine surface vehicle, in the JSON schema Tools/Characterization emits
    /// (profiles/ideal.json). SI throughout; body FLU at the nominal CG.
    /// </summary>
    [Serializable]
    public sealed class MarineSurfaceParameters {
        public string name = "Ideal";
        [Tooltip("Physics step the plant sets [s]")] public float fixedDeltaTime = 0.02f;
        [Tooltip("Total mass [kg]")] public float mass = 30f;
        [Tooltip("Roll, pitch, yaw inertia about the CG [kg m²]")] public float ixx = 2.2f, iyy = 2f, izz = 4.1f;
        [Tooltip("CG offset from the nominal CG [m]")] public float comBow, comPort;
        public AddedMassTerms addedMass = new() { a11 = 1.5f, a22 = 15f, a26 = 0f, a66 = 1.23f };
        public SignedDamping surge = new() { d1Pos = 10f, d1Neg = 10f, d2Pos = 19.4f, d2Neg = 19.4f };
        public SignedDamping sway = new() { d1Pos = 15f, d1Neg = 15f, d2Pos = 41.7f, d2Neg = 41.7f };
        public SignedDamping yaw = new() { d1Pos = 10f, d1Neg = 10f, d2Pos = 19.3f, d2Neg = 19.3f };
        public SymmetricDamping heave = new() { d1 = 150f, d2 = 40f };
        public SymmetricDamping roll = new() { d1 = 30f, d2 = 10f };
        public SymmetricDamping pitch = new() { d1 = 30f, d2 = 10f };
        public List<ThrusterSpec> thrusters = IdealThrusters();
        public ProfileEnvelope envelope = new() { surge = 2.5f, sway = 1.7f, yaw = 1.9f };
        public Provenance provenance = new() { kind = "ideal", summary = "prior" };

        /// <summary>The mixer.tex geometry: FL(a,b) 135°, FR(a,−b) −135°, RR(−a,−b) −45°,
        /// RL(−a,b) 45°, a = 0.27 m, b = 0.34 m; T200 51.5 N both ways at 16 V.</summary>
        public static List<ThrusterSpec> IdealThrusters() => new() {
            Ideal("front_left", 0.27f, 0.34f, 135f), Ideal("front_right", 0.27f, -0.34f, -135f),
            Ideal("rear_right", -0.27f, -0.34f, -45f), Ideal("rear_left", -0.27f, 0.34f, 45f)
        };

        private static ThrusterSpec Ideal(string name, float x, float y, float alpha) =>
            new() { name = name, x = x, y = y, alphaDeg = alpha };

        public MarineSurfaceParameters Clone() =>
            JsonUtility.FromJson<MarineSurfaceParameters>(JsonUtility.ToJson(this));

        public MarinePlanarModel PlanarModel(float massOverride = float.NaN, float izzOverride = float.NaN) => new() {
            Mass = float.IsFinite(massOverride) ? massOverride : mass,
            Izz = float.IsFinite(izzOverride) ? izzOverride : izz,
            A11 = addedMass.a11, A22 = addedMass.a22, A26 = addedMass.a26, A66 = addedMass.a66,
            Surge = surge.Axis, Sway = sway.Axis, Yaw = yaw.Axis
        };

        /// <summary>Rejects unusable numbers; warns where the step loses accuracy
        /// (dt·J(1.5 v_max)/M_eff > 0.5) or nothing bounds the speed (no damping).</summary>
        public ProfileValidation Validate(float dt) {
            var v = new ProfileValidation();
            float[] all = { fixedDeltaTime, mass, ixx, iyy, izz, comBow, comPort, addedMass.a11, addedMass.a22,
                addedMass.a26, addedMass.a66, heave.d1, heave.d2, roll.d1, roll.d2, pitch.d1, pitch.d2 };
            v.Require(all.All(float.IsFinite), "non-finite parameter");
            v.Require(fixedDeltaTime > 0 && fixedDeltaTime <= 0.1f, $"fixedDeltaTime {fixedDeltaTime} outside (0, 0.1]");
            v.Require(mass > 0, $"mass {mass} <= 0");
            v.Require(ixx > 0 && iyy > 0 && izz > 0, "inertia <= 0");
            v.Require(addedMass.a11 >= 0 && addedMass.a22 >= 0 && addedMass.a66 >= 0, "negative added mass");
            v.Require((mass + addedMass.a22) * (izz + addedMass.a66) > addedMass.a26 * addedMass.a26,
                "M not positive definite (a26 too large)");
            foreach (var (axis, d) in new[] { ("surge", surge), ("sway", sway), ("yaw", yaw) }) {
                float[] c = { d.d1Pos, d.d1Neg, d.d2Pos, d.d2Neg };
                v.Require(c.All(float.IsFinite) && c.All(x => x >= 0), $"{axis} damping negative or non-finite");
                v.Warn(c.All(x => x == 0), $"{axis} has no damping; only maxLinearVelocity bounds it");
            }
            v.Require(new[] { heave, roll, pitch }.All(d => d.d1 >= 0 && d.d2 >= 0), "vertical damping negative");
            v.Require(thrusters != null && thrusters.Count > 0, "no thrusters");
            if (thrusters != null) {
                v.Require(thrusters.Select(t => t.name).Distinct().Count() == thrusters.Count, "duplicate thruster names");
                foreach (ThrusterSpec t in thrusters) {
                    float[] f = { t.x, t.y, t.z, t.alphaDeg, t.kForward, t.kReverse, t.vRef, t.nV, t.deadbandUs,
                        t.offsetUs, t.tauT, t.deadTime, t.submersionDepth };
                    v.Require(f.All(float.IsFinite), $"thruster {t.name} non-finite");
                    v.Require(t.kForward >= 0 && t.kReverse >= 0 && t.vRef > 0 && t.tauT >= 0 &&
                              t.deadTime >= 0 && t.deadbandUs >= 0 && t.submersionDepth > 0,
                        $"thruster {t.name} negative or zero spec");
                    v.Require(t.kappa == 0, $"thruster {t.name} kappa must be 0 until a bollard pull gives k");
                }
            }
            if (!v.Ok) return v;
            MarinePlanarModel model = PlanarModel();
            float[] vmax = { envelope.surge, envelope.sway, envelope.yaw };
            for (int j = 0; j < 3; j++) {
                AxisDamping d = model.Axis(j);
                double s = 1.5 * vmax[j];
                double ratio = dt * Math.Max(d.Jacobian(s), d.Jacobian(-s)) / model.EffectiveMass(j);
                v.Warn(ratio > 0.5, $"axis {j}: dt*J(1.5 v_max)/M_eff = {ratio:F2} > 0.5 (step accuracy limit)");
            }
            return v;
        }

        /// <summary>Per-axis dt·J(1.5 v_max)/M_eff for logs and tests.</summary>
        public double StepRatio(int axis, float dt) {
            MarinePlanarModel model = PlanarModel();
            float vmax = axis == 0 ? envelope.surge : axis == 1 ? envelope.sway : envelope.yaw;
            AxisDamping d = model.Axis(axis);
            return dt * Math.Max(d.Jacobian(1.5 * vmax), d.Jacobian(-1.5 * vmax)) / model.EffectiveMass(axis);
        }
    }

    /// <summary>A marine surface vehicle profile asset (Assets/Config/Boats/Resources/*.asset).
    /// Ideal assets hold priors; Characterized ones are generated by Tools/Characterization and
    /// imported with CraneBoatProfileImport, never hand-edited.</summary>
    [CreateAssetMenu(menuName = "Physics/MarineSurfaceProfile")]
    public sealed class MarineSurfaceProfile : VehicleProfile {
        [SerializeField] private MarineSurfaceParameters parameters = new();

        public MarineSurfaceParameters Parameters {
            get => parameters;
            set => parameters = value ?? throw new ArgumentNullException(nameof(value));
        }

        public override string ProfileName => string.IsNullOrEmpty(parameters.name) ? name : parameters.name;
        public override Provenance Provenance => parameters.provenance;
        public override string ToJson() => JsonUtility.ToJson(parameters, true);
        public override void FromJson(string json) => parameters = JsonUtility.FromJson<MarineSurfaceParameters>(json);
        public override ProfileValidation Validate(float dt) => parameters.Validate(dt);
    }
}
