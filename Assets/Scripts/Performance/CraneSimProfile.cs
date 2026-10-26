using UnityEngine;
using UnityEngine.Rendering;

namespace Sim.Performance {
    /// <summary>
    /// One device tier: how much rendering and sensor work the sim spends per second. A profile
    /// never changes physics (0.02 s step, water simulation, colliders) or the points per LiDAR
    /// scan. Assets: Assets/Config/Performance/Resources/SimProfiles/{Full,Balanced,Minimal}.
    /// Selection and precedence: CraneSimProfileSelector; docs: PerformanceEngineering.md.
    /// </summary>
    [CreateAssetMenu(menuName = "Performance/SimProfile")]
    public sealed class CraneSimProfile : ScriptableObject {
        [field: SerializeField, Tooltip("HDRP asset to render with; it must keep water enabled " +
            "(buoyancy queries HDRP water). Empty keeps the project default.")]
        public RenderPipelineAsset Pipeline { get; private set; }

        [field: SerializeField, Tooltip("Frame rate cap. Every enabled camera, sensor cameras " +
            "included, renders once per frame, so this is the main GPU budget. Keep it at or " +
            "above CameraHz so each capture sees a new frame."), Range(5, 240)]
        public int FrameRateCap { get; private set; } = 60;

        [field: SerializeField, Tooltip("Scale of the robot camera RenderTextures (RGB, depth, " +
            "normals, processed). Camera info follows the texture size."), Range(0.125f, 1f)]
        public float CameraResolutionScale { get; private set; } = 1f;

        [field: SerializeField, Tooltip("RGB and depth capture rate in Hz; 0 keeps the scene " +
            "value."), Range(0f, 60f)]
        public float CameraHz { get; private set; }

        [field: SerializeField, Tooltip("Detection publish rate in Hz; 0 keeps the scene value."),
         Range(0f, 60f)]
        public float DetectionHz { get; private set; }

        [field: SerializeField, Tooltip("LiDAR scan rate in Hz; 0 keeps the scene value. Points " +
            "per scan never change."), Range(0f, 60f)]
        public float LidarHz { get; private set; }

        [field: SerializeField, Tooltip("Turn off bloom, depth of field, motion blur, SSAO, SSR, " +
            "SSGI, contact shadows and volumetrics on every camera.")]
        public bool ReducedEffects { get; private set; }

        [field: SerializeField, Tooltip("Water tessellation and caustics. Render-only: buoyancy " +
            "reads the same water simulation either way.")]
        public bool WaterDetail { get; private set; } = true;

        [field: SerializeField, Tooltip("Render the spectator (window) camera. Off leaves only the " +
            "robot sensor cameras rendering.")]
        public bool Spectator { get; private set; } = true;
    }
}
