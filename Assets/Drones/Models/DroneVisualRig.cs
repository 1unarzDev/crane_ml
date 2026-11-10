using System;
using UnityEngine;

namespace Crane.Drones
{
    /// <summary>Rotor spin as seen from above (looking down -Y in Unity, i.e. from the FLU +Z axis).</summary>
    public enum RotorSpin { CW, CCW }

    /// <summary>
    /// Visual-only rig for the AOS 7 V5 research quad. The physics plant owns the dynamics; it only
    /// has to call <see cref="SetRotorSpeeds"/> every frame (or every physics step) with the four
    /// rotor speeds in rad/s, ordered like ArduPilot Quad-X: M1 FR(CCW), M2 RL(CCW), M3 FL(CW), M4 RR(CW).
    /// Frames: the prefab root is the body frame in Unity axes (+X right, +Y up, +Z forward). The
    /// FLU body frame (x forward, y left, z up) maps as unity = (-y_flu, z_flu, x_flu); use
    /// <see cref="FluToUnity"/> / <see cref="UnityToFlu"/>.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class DroneVisualRig : MonoBehaviour
    {
        [Serializable]
        public class Rotor
        {
            public string name;
            public int motorIndex;          // 1-based, ArduPilot Quad-X numbering
            public Transform hub;           // spin axis = hub local +Y; hub position = prop-plane center
            public RotorSpin spin;          // direction of positive thrust-producing rotation, from above
            public Renderer bladesOpaque;   // solid blades, shown at low rpm
            public Renderer bladesFade;     // same blades, alpha-blended during the crossfade
            public Renderer blurDisc;       // translucent motion-blur disc
            [NonSerialized] public float angleDeg;
            [NonSerialized] public float speed;
        }

        public Rotor[] rotors = new Rotor[0];
        public Transform comMarker;         // visual estimate of the center of mass; physics may override
        public Transform cameraMount;       // OAK-D S2 RGB optical center, +Z = optical axis (unity forward)
        public Transform lidarMount;        // Unitree L2 scan origin
        public Transform imuMount;          // Cube Orange IMU location
        public Renderer[] statusLeds = new Renderer[0];

        [Header("Prop motion blur (rad/s)")]
        public float blurStart = 90f;       // below: opaque blades only
        public float blurFull = 280f;       // above: faint ghost blades plus full blur disc
        [Range(0f, 1f)] public float ghostBladeAlpha = 0.12f;
        [Range(0f, 1f)] public float maxDiscAlpha = 0.62f;
        public bool autoAdvance = true;     // integrate the spin angle in LateUpdate

        static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        static readonly int EmissiveId = Shader.PropertyToID("_EmissiveColor");
        MaterialPropertyBlock _mpb;
        Color[] _fadeBase, _discBase;

        void Awake() { ResetVisuals(); }

        public static Vector3 FluToUnity(Vector3 flu) { return new Vector3(-flu.y, flu.z, flu.x); }
        public static Vector3 UnityToFlu(Vector3 u) { return new Vector3(u.z, -u.x, u.y); }

        /// <summary>Position of a child transform in the body FLU frame (meters).</summary>
        public Vector3 BodyFlu(Transform t) { return UnityToFlu(transform.InverseTransformPoint(t.position)); }

        public void ResetVisuals()
        {
            if (_mpb == null) _mpb = new MaterialPropertyBlock();
            _fadeBase = new Color[rotors.Length]; _discBase = new Color[rotors.Length];
            for (int i = 0; i < rotors.Length; i++)
            {
                var r = rotors[i];
                if (r.bladesFade != null && r.bladesFade.sharedMaterial != null) _fadeBase[i] = r.bladesFade.sharedMaterial.GetColor(BaseColorId);
                if (r.blurDisc != null && r.blurDisc.sharedMaterial != null) _discBase[i] = r.blurDisc.sharedMaterial.GetColor(BaseColorId);
                r.speed = 0f; r.angleDeg = 0f;
                Apply(i, 0f);
            }
        }

        /// <summary>Rotor speeds in rad/s (magnitude along the rotor's own spin direction), index = motor-1.</summary>
        public void SetRotorSpeeds(float[] radPerSec)
        {
            if (radPerSec == null) return;
            int n = Mathf.Min(radPerSec.Length, rotors.Length);
            for (int i = 0; i < n; i++)
            {
                rotors[i].speed = radPerSec[i];
                Apply(i, Mathf.Abs(radPerSec[i]));
            }
        }

        /// <summary>Advance the visual spin angle by dt seconds (use this from a fixed-step sim if autoAdvance is off).</summary>
        public void AdvanceRotors(float dt)
        {
            for (int i = 0; i < rotors.Length; i++)
            {
                var r = rotors[i];
                if (r.hub == null) continue;
                // Unity rotation about +Y is clockwise seen from above.
                float sign = r.spin == RotorSpin.CW ? 1f : -1f;
                r.angleDeg = Mathf.Repeat(r.angleDeg + sign * r.speed * Mathf.Rad2Deg * dt, 360f);
                r.hub.localRotation = Quaternion.Euler(0f, r.angleDeg, 0f);
            }
        }

        void LateUpdate() { if (autoAdvance) AdvanceRotors(Time.deltaTime); }

        /// <summary>Set a status LED (index into statusLeds) to a color and intensity multiplier (0 = off).</summary>
        public void SetStatusLed(int index, Color color, float intensity)
        {
            if (index < 0 || index >= statusLeds.Length || statusLeds[index] == null) return;
            if (_mpb == null) _mpb = new MaterialPropertyBlock();
            statusLeds[index].GetPropertyBlock(_mpb);
            _mpb.SetColor(EmissiveId, color * (Mathf.Max(0f, intensity) * 600f));
            _mpb.SetColor(BaseColorId, Color.Lerp(new Color(0.02f, 0.02f, 0.02f), color, Mathf.Clamp01(intensity)));
            statusLeds[index].SetPropertyBlock(_mpb);
        }

        void Apply(int i, float w)
        {
            var r = rotors[i];
            if (_mpb == null) _mpb = new MaterialPropertyBlock();
            float t = Mathf.Clamp01((w - blurStart) / Mathf.Max(1e-3f, blurFull - blurStart));
            t = t * t * (3f - 2f * t);
            bool solid = t <= 0f;
            if (r.bladesOpaque != null) r.bladesOpaque.enabled = solid;
            if (r.bladesFade != null)
            {
                r.bladesFade.enabled = !solid;
                if (!solid)
                {
                    r.bladesFade.GetPropertyBlock(_mpb);
                    Color c = _fadeBase != null && i < _fadeBase.Length ? _fadeBase[i] : Color.gray;
                    c.a = Mathf.Lerp(1f, ghostBladeAlpha, t);
                    _mpb.SetColor(BaseColorId, c);
                    r.bladesFade.SetPropertyBlock(_mpb);
                }
            }
            if (r.blurDisc != null)
            {
                r.blurDisc.enabled = !solid;
                if (!solid)
                {
                    r.blurDisc.GetPropertyBlock(_mpb);
                    Color c = _discBase != null && i < _discBase.Length ? _discBase[i] : Color.gray;
                    c.a = t * maxDiscAlpha;
                    _mpb.SetColor(BaseColorId, c);
                    r.blurDisc.SetPropertyBlock(_mpb);
                }
            }
        }
    }
}
