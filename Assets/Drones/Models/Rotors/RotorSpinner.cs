using System;
using UnityEngine;

namespace Crane.Drones.Models
{
    /// <summary>
    /// Visual-only propeller spinner for URDF-imported multirotors (Assets/Robot Descriptions/aos7v5). It turns each rotor link's
    /// transform about the link's own shaft axis; it never touches physics: the aerial plant is kinematic, and the rotor links
    /// must NOT carry an ArticulationBody (the prefab adapter strips them). Feed it the plant's rotor speeds each frame:
    /// <c>SetRotorSpeeds(plant.RotorSpeeds)</c> (rad/s, index = PX4/CRANE motor-1: FR CCW, RL CCW, FL CW, RR CW).
    /// Optional motion blur: above <see cref="blurStart"/> rad/s the blades fade out and a translucent disc fades in.
    /// Spin sign: <c>spinSign = +1</c> is counter-clockwise seen from above. Unity rotates clockwise (seen from above) for a positive
    /// angle about +Y, so the applied angle is <c>-spinSign * omega * dt</c>.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RotorSpinner : MonoBehaviour
    {
        [Serializable]
        public class Rotor
        {
            public string link;                  // URDF link name, e.g. rotor_0
            public int motorIndex;               // 1-based PX4 / CRANE M1..M4
            public int spinSign = 1;             // +1 CCW seen from above, -1 CW
            public Transform hub;                // the rotor link transform (origin exactly on the shaft)
            public Vector3 axisLocal = Vector3.up;   // shaft axis in the hub's local frame (URDF +z -> Unity local up)
            public Renderer[] blades = new Renderer[0];   // optional: hidden above blurFull
            public Renderer blurDisc;            // optional translucent disc
            [NonSerialized] public float angleDeg;
            [NonSerialized] public float speed;
            [NonSerialized] public Quaternion rest = Quaternion.identity;
        }

        public Rotor[] rotors = new Rotor[0];
        [Tooltip("rad/s: below this only the solid blades are shown")] public float blurStart = 90f;
        [Tooltip("rad/s: at and above this the blades are hidden and the disc is at full alpha")] public float blurFull = 280f;
        [Range(0f, 1f)] public float maxDiscAlpha = 0.45f;
        public bool autoAdvance = true;

        static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        MaterialPropertyBlock _mpb;

        void Awake() { Capture(); }

        /// <summary>Remember each hub's rest rotation (call again after you re-parent or re-pose the rig).</summary>
        public void Capture()
        {
            foreach (var r in rotors)
            {
                if (r == null || r.hub == null) continue;
                r.rest = r.hub.localRotation;
                r.angleDeg = 0f;
                r.speed = 0f;
            }
        }

        public void SetRotorSpeeds(float[] radPerSec)
        {
            if (radPerSec == null) return;
            int n = Mathf.Min(radPerSec.Length, rotors.Length);
            for (int i = 0; i < n; i++)
            {
                rotors[i].speed = Mathf.Abs(radPerSec[i]);
                ApplyBlur(rotors[i]);
            }
        }

        /// <summary>Advance every rotor by dt seconds at its current speed (call from LateUpdate or the fixed step).</summary>
        public void AdvanceRotors(float dt)
        {
            foreach (var r in rotors)
            {
                if (r == null || r.hub == null) continue;
                r.angleDeg = Mathf.Repeat(r.angleDeg - r.spinSign * r.speed * Mathf.Rad2Deg * dt, 360f);
                r.hub.localRotation = r.rest * Quaternion.AngleAxis(r.angleDeg, r.axisLocal);
            }
        }

        void LateUpdate() { if (autoAdvance) AdvanceRotors(Time.deltaTime); }

        void ApplyBlur(Rotor r)
        {
            if (r.blades == null && r.blurDisc == null) return;
            float t = Mathf.Clamp01((r.speed - blurStart) / Mathf.Max(1e-3f, blurFull - blurStart));
            t = t * t * (3f - 2f * t);
            if (r.blades != null)
                foreach (var b in r.blades) if (b != null) b.enabled = t < 1f;
            if (r.blurDisc != null)
            {
                r.blurDisc.enabled = t > 0f;
                if (t > 0f && r.blurDisc.sharedMaterial != null && r.blurDisc.sharedMaterial.HasProperty(BaseColorId))
                {
                    if (_mpb == null) _mpb = new MaterialPropertyBlock();
                    r.blurDisc.GetPropertyBlock(_mpb);
                    Color c = r.blurDisc.sharedMaterial.GetColor(BaseColorId);
                    c.a = t * maxDiscAlpha;
                    _mpb.SetColor(BaseColorId, c);
                    r.blurDisc.SetPropertyBlock(_mpb);
                }
            }
        }
    }
}
