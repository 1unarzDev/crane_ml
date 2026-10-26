using System;
using UnityEngine;

namespace Sim.Actuators.Motors {
    /// <summary>
    /// One thruster's physical spec, addressed by name. Field names are the profile JSON schema
    /// (Tools/Characterization/profiles/*.json). Thrust is a static map of the ESC effort at V_ref
    /// through a dead time and a first-order lag; mount pose is body FLU from the nominal CG.
    /// </summary>
    [Serializable]
    public sealed class ThrusterSpec {
        [Tooltip("Matches the thruster link: <name>_propeller_link")] public string name = "";
        [Tooltip("Mount position from the CG, body FLU [m]")] public float x, y, z;
        [Tooltip("Thrust direction for positive effort, deg CCW from bow")] public float alphaDeg;
        [Tooltip("Full forward thrust at V_ref [N]")] public float kForward = 51.5f;
        [Tooltip("Full reverse thrust magnitude at V_ref [N]")] public float kReverse = 51.5f;
        [Tooltip("Pack voltage of the bollard test [V]")] public float vRef = 16f;
        [Tooltip("Voltage exponent n_V of (V/V_ref)^n_V; 0 ignores voltage")] public float nV;
        [Tooltip("ESC deadband about neutral+offset [µs]")] public float deadbandUs;
        [Tooltip("ESC neutral offset from 1500 [µs]")] public float offsetUs;
        [Tooltip("First-order thrust lag τ_T [s]; 0 is instant")] public float tauT;
        [Tooltip("Dead time t_d [s], applied as round(t_d/dt) ticks")] public float deadTime;
        [Tooltip("Depth for full submersion [m]")] public float submersionDepth = 0.08f;
        [Tooltip("Advance/idle-prop loss κ; must be 0 until a bollard pull gives k")] public float kappa;

        public Vector2 Direction => new(Mathf.Cos(alphaDeg * Mathf.Deg2Rad), Mathf.Sin(alphaDeg * Mathf.Deg2Rad));
        public ThrusterSpec Clone() => (ThrusterSpec)MemberwiseClone();
    }
}
