using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sim.Utils.Viewer {
    public enum ViewerShotKind {
        /// <summary>Held relative to the robot's heading.</summary>
        Follow,
        /// <summary>Held over the robot with a world-aligned yaw (map up).</summary>
        FollowWorld,
        /// <summary>Frames the task area nearest the robot.</summary>
        Task,
        /// <summary>Frames the whole course.</summary>
        Course
    }

    /// <summary>One selectable view. Angles in degrees, lengths in metres.</summary>
    [Serializable]
    public sealed class ViewerPreset {
        public string name;
        public ViewerShotKind kind;
        [Tooltip("Input System binding path that selects this view, e.g. <Keyboard>/1")]
        public string key;
        [Tooltip("Follow: look yaw relative to the robot heading (0 chase, 90 from its left, 180 from ahead). " +
                 "FollowWorld, Task, Course: world yaw (0 looks along +Z)")]
        public float yaw;
        [Tooltip("Look pitch, positive down")]
        [Range(-10f, 89.5f)] public float pitch = 15f;
        [Tooltip("Follow kinds: camera distance from the focus")]
        [Min(0.5f)] public float distance = 8f;
        [Tooltip("Follow kinds: focus height above the robot origin")]
        public float height = 0.5f;
        [Tooltip("Task and Course: framing margin on the area's bounding sphere")]
        [Min(0.5f)] public float margin = 1.1f;

        public ViewerPreset() { }

        public ViewerPreset(string name, ViewerShotKind kind, string key, float yaw, float pitch,
            float distance, float height = 0.5f, float margin = 1.1f) {
            this.name = name;
            this.kind = kind;
            this.key = key;
            this.yaw = yaw;
            this.pitch = pitch;
            this.distance = distance;
            this.height = height;
            this.margin = margin;
        }

        public ViewerShot ToShot() {
            var follow = new FollowShot(yaw, pitch, distance, Vector3.up * height, kind == ViewerShotKind.FollowWorld);
            if (kind == ViewerShotKind.Follow || kind == ViewerShotKind.FollowWorld) return follow;
            // Without areas, frame the robot from above so the key still does something useful.
            var overhead = new FollowShot(yaw, pitch, Mathf.Max(distance, 20f), Vector3.up * height, true);
            return new AreaShot(kind == ViewerShotKind.Task, yaw, pitch, margin, overhead);
        }
    }

    /// <summary>
    /// The viewer camera's settings: follow damping, how much of the robot's roll and pitch it
    /// inherits, transitions, free-orbit input and the view presets. Saved values live in
    /// Assets/Config/Viewer/Resources/ViewerCamera.asset; <c>--crane-viewer-*</c> flags override
    /// a few at runtime (see <see cref="ViewerCameraRig"/>).
    /// </summary>
    [CreateAssetMenu(menuName = "Viewer/ViewerCameraConfig")]
    public sealed class ViewerCameraConfig : ScriptableObject, IViewerFollowSettings {
        public const string ResourceName = "ViewerCamera";

        [field: Header("Follow")]
        [field: SerializeField, Tooltip("Position time constant [s]; 0 is rigid"), Range(0f, 3f)]
        public float PositionTime { get; set; } = 0.35f;
        [field: SerializeField, Tooltip("Heading time constant [s]; 0 is rigid"), Range(0f, 3f)]
        public float YawTime { get; set; } = 0.6f;
        [field: SerializeField, Tooltip("Roll/pitch time constant [s]; 0 is rigid"), Range(0f, 3f)]
        public float TiltTime { get; set; } = 0.4f;
        [field: SerializeField, Tooltip("Fraction of the robot's roll and pitch the view inherits: 0 level, 1 attached"), Range(0f, 1f)]
        public float RollFraction { get; set; } = 0.2f;

        [field: Header("Transitions and free orbit")]
        [field: SerializeField, Tooltip("Preset transition time [s]"), Range(0f, 5f)]
        public float TransitionTime { get; set; } = 1.2f;
        [field: SerializeField, Tooltip("Orbit degrees per mouse pixel"), Range(0.01f, 1f)]
        public float OrbitSensitivity { get; set; } = 0.15f;
        [field: SerializeField, Tooltip("Log-zoom step per scroll notch"), Range(0.01f, 0.5f)]
        public float ZoomStep { get; set; } = 0.12f;
        [field: SerializeField, Tooltip("Lowest look pitch [deg] after orbiting"), Range(-30f, 30f)]
        public float MinPitch { get; set; } = 2f;
        [field: SerializeField, Tooltip("Highest look pitch [deg] after orbiting"), Range(30f, 89.5f)]
        public float MaxPitch { get; set; } = 89f;
        [field: SerializeField, Tooltip("Without course ground truth, root objects larger than this [m] are not task areas")]
        public float MaxAreaSize { get; set; } = 80f;

        [field: Header("Input (Input System binding paths)")]
        [field: SerializeField] public string NextKey { get; set; } = "<Keyboard>/v";
        [field: SerializeField] public string OrbitButton { get; set; } = "<Mouse>/rightButton";
        [field: SerializeField] public string OrbitDelta { get; set; } = "<Mouse>/delta";
        [field: SerializeField] public string ZoomAxis { get; set; } = "<Mouse>/scroll/y";

        [field: Header("Presets (the first is the start view unless --crane-viewer-preset)")]
        [field: SerializeField] public ViewerPreset[] Presets { get; set; } = DefaultPresets();

        public static ViewerPreset[] DefaultPresets() => new[] {
            new ViewerPreset("Chase", ViewerShotKind.Follow, "<Keyboard>/1", 0f, 14f, 9f, 0.8f),
            new ViewerPreset("Side", ViewerShotKind.Follow, "<Keyboard>/2", 90f, 8f, 8f),
            new ViewerPreset("Front", ViewerShotKind.Follow, "<Keyboard>/3", 180f, 10f, 8f),
            new ViewerPreset("Corner", ViewerShotKind.Follow, "<Keyboard>/4", 45f, 40f, 14f),
            new ViewerPreset("Overhead", ViewerShotKind.Follow, "<Keyboard>/5", 0f, 89f, 18f),
            new ViewerPreset("Map", ViewerShotKind.FollowWorld, "<Keyboard>/6", 0f, 89f, 35f),
            new ViewerPreset("Task", ViewerShotKind.Task, "<Keyboard>/7", 0f, 60f, 20f, 0.5f, 1.15f),
            new ViewerPreset("Course", ViewerShotKind.Course, "<Keyboard>/8", 0f, 65f, 40f, 0.5f, 1.05f),
        };

        /// <summary>The saved config as a fresh instance (the asset is never modified), or the
        /// code defaults when no asset exists.</summary>
        public static ViewerCameraConfig Load() {
            ViewerCameraConfig asset = Resources.Load<ViewerCameraConfig>(ResourceName);
            ViewerCameraConfig config = asset != null ? Instantiate(asset) : CreateInstance<ViewerCameraConfig>();
            config.name = ResourceName;
            return config;
        }

        /// <summary>Index of the preset named <paramref name="name"/> (case-insensitive), or -1.</summary>
        public int IndexOf(string name) =>
            Array.FindIndex(Presets, preset => string.Equals(preset.name, name, StringComparison.OrdinalIgnoreCase));

        public List<string> Validate() {
            var errors = new List<string>();
            if (Presets == null || Presets.Length == 0) errors.Add("no presets");
            var names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (ViewerPreset preset in Presets ?? Array.Empty<ViewerPreset>()) {
                if (string.IsNullOrWhiteSpace(preset.name) || !names.Add(preset.name))
                    errors.Add($"preset name '{preset.name}' empty or repeated");
                if (preset.distance <= 0f) errors.Add($"{preset.name}: distance must be positive");
            }
            if (MinPitch >= MaxPitch) errors.Add("MinPitch must be below MaxPitch");
            return errors;
        }
    }
}
