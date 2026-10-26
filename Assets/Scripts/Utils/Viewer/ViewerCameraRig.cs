using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

namespace Sim.Utils.Viewer {
    /// <summary>
    /// Drives the viewer's on-screen camera: a damped follow of whatever robot is spawned, with a
    /// fraction of its roll and pitch, eased transitions between the <see cref="ViewerCameraConfig"/>
    /// presets, and a mouse free orbit layered on the current view.
    ///
    /// The bootstrap adds one rig per scene. It drives only an enabled camera that renders to the
    /// screen (no target texture), so sensor cameras, the water update driver and runs that disable
    /// spectator cameras are untouched, and it never creates a camera. It stays off without
    /// graphics, in batch mode unless <c>--crane-viewer-preset</c> is given, under
    /// <c>--crane-evidence-overlay</c> (that harness holds its own fixed shot through this rig) and
    /// with <c>--crane-disable-viewer-camera</c>.
    ///
    /// Flags: <c>--crane-viewer-preset NAME</c> start view, <c>--crane-viewer-roll F</c> roll
    /// fraction, <c>--crane-viewer-target NAME</c> follow a named object instead of the robot found.
    /// </summary>
    [DefaultExecutionOrder(31000)]
    public sealed class ViewerCameraRig : MonoBehaviour {
        public const string PresetFlag = "--crane-viewer-preset";
        public const string RollFlag = "--crane-viewer-roll";
        public const string TargetFlag = "--crane-viewer-target";
        public const string DisableFlag = "--crane-disable-viewer-camera";
        private const float SearchInterval = 1f;

        private ViewerCameraConfig config;
        private Camera view;
        private Transform target;
        private string targetName;
        private Quaternion forwardFromBody = Quaternion.identity;
        private ViewerFrame frame;
        private bool snapped;
        private ViewerShot shot;
        private int preset = -1;
        private float yawOffset, pitchOffset, zoomOffset;
        private float nextSearch;
        private ViewerAreaList areas;
        private readonly List<InputAction> actions = new();
        private InputAction orbitButton, orbitDelta, zoomAxis;

        public Camera View => view;
        public Transform Target => target;
        public int Preset => preset;
        public ViewerCameraConfig Config => config;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            if (Array.IndexOf(args, DisableFlag) >= 0 || Array.IndexOf(args, "--crane-evidence-overlay") >= 0 ||
                SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null ||
                (Application.isBatchMode && Array.IndexOf(args, PresetFlag) < 0)) return;
            SceneManager.sceneLoaded += (_, mode) => {
                if (mode != LoadSceneMode.Single) return;
                var host = new GameObject("CRANE Viewer Camera Rig");
                ViewerCameraRig rig = host.AddComponent<ViewerCameraRig>();
                rig.config = ViewerCameraConfig.Load();
                if (float.TryParse(Read(args, RollFlag), System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out float roll))
                    rig.config.RollFraction = Mathf.Clamp01(roll);
                rig.targetName = Read(args, TargetFlag);
                string start = Read(args, PresetFlag);
                rig.preset = Mathf.Max(0, start == null ? 0 : rig.config.IndexOf(start));
                rig.BindInput();
            };
        }

        /// <summary>
        /// Drives <paramref name="camera"/> from code (no input, no bootstrap). The camera is
        /// detached from any parent; <paramref name="robot"/>'s +Z is its forward.
        /// </summary>
        public static ViewerCameraRig Drive(Camera camera, Transform robot, ViewerCameraConfig settings,
            ViewerShot startShot) {
            ViewerCameraRig rig = camera.gameObject.AddComponent<ViewerCameraRig>();
            rig.config = settings;
            rig.Bind(camera, robot, Quaternion.identity);
            rig.shot = startShot;
            return rig;
        }

        /// <summary>Eases to preset <paramref name="index"/>; reselecting clears the free orbit.</summary>
        public void Select(int index) {
            if (config == null || index < 0 || index >= config.Presets.Length) return;
            preset = index;
            Select(config.Presets[index].ToShot(), config.TransitionTime);
        }

        /// <summary>Eases to <paramref name="next"/>, folding the free-orbit offsets into the
        /// transition so they unwind with it.</summary>
        public void Select(ViewerShot next, float transition) {
            ViewerShot from = shot == null ? null :
                new OffsetShot(shot, yawOffset, pitchOffset, zoomOffset, config.MinPitch, config.MaxPitch);
            yawOffset = pitchOffset = zoomOffset = 0f;
            shot = from == null || transition <= 0f ? next : new ViewerBlend(from, next, transition);
        }

        private void LateUpdate() {
            if (view == null && !TryBind()) return;
            if (view == null || !view.enabled || view.targetTexture != null) return;
            if (target == null) {
                if (Time.unscaledTime < nextSearch) return;
                nextSearch = Time.unscaledTime + SearchInterval;
                target = FindRobot(targetName);
                snapped = false;
                areas = null;
                if (target == null) return;
            }
            float dt = Time.unscaledDeltaTime;
            Quaternion attitude = target.rotation * forwardFromBody;
            frame = snapped ? ViewerFollow.Step(frame, target.position, attitude, config, dt)
                            : ViewerFollow.Snap(target.position, attitude, config.RollFraction);
            if (shot == null) shot = config.Presets[Mathf.Clamp(preset, 0, config.Presets.Length - 1)].ToShot();
            else if (snapped) shot = shot.Advance(dt);
            snapped = true;
            ReadOrbitInput();
            areas ??= ViewerAreas.ForActiveScene(target, config.MaxAreaSize);
            var context = new ViewerContext(frame, view.fieldOfView, view.aspect, areas);
            ViewerOrbit held = shot.Evaluate(context);
            // Dragging past a pitch limit must not bank offset that has to be dragged back first.
            pitchOffset = Mathf.Clamp(pitchOffset, config.MinPitch - held.Pitch, config.MaxPitch - held.Pitch);
            ViewerOrbit orbit = OffsetShot.Apply(held, yawOffset, pitchOffset, zoomOffset,
                config.MinPitch, config.MaxPitch);
            Pose pose = orbit.ToPose();
            view.transform.SetPositionAndRotation(pose.position, pose.rotation);
        }

        /// <summary>Takes the screen camera once scene Start methods and runtime gates have run.</summary>
        private bool TryBind() {
            if (Time.unscaledTime < nextSearch) return false;
            nextSearch = Time.unscaledTime + SearchInterval;
            Camera chosen = null;
            foreach (Camera camera in FindObjectsByType<Camera>(FindObjectsInactive.Exclude)) {
                if (!camera.enabled || camera.targetTexture != null) continue;
                if (chosen == null || camera.CompareTag("MainCamera") ||
                    (!chosen.CompareTag("MainCamera") && camera.depth > chosen.depth)) chosen = camera;
            }
            if (chosen == null) return false;
            // A camera mounted on a robot names that robot, and its heading is the robot's forward.
            Transform robot = targetName != null ? FindRobot(targetName) : RobotAbove(chosen.transform);
            Quaternion forward = Quaternion.identity;
            if (robot != null && chosen.transform.IsChildOf(robot)) {
                Vector3 local = robot.InverseTransformDirection(chosen.transform.forward);
                local.y = 0f;
                if (local.sqrMagnitude > 1e-6f) forward = Quaternion.LookRotation(local);
            }
            Bind(chosen, robot, forward);
            Debug.Log($"CRANE_VIEWER_CAMERA camera={chosen.name} target={(robot != null ? robot.name : "searching")} " +
                      $"preset={config.Presets[Mathf.Clamp(preset, 0, config.Presets.Length - 1)].name} " +
                      $"roll_fraction={config.RollFraction:R}");
            return true;
        }

        private void Bind(Camera camera, Transform robot, Quaternion forward) {
            view = camera;
            target = robot;
            forwardFromBody = forward;
            snapped = false;
            nextSearch = 0f;
            camera.transform.SetParent(null, true);
        }

        private static Transform RobotAbove(Transform part) {
            ArticulationBody body = part.GetComponentInParent<ArticulationBody>();
            if (body != null) {
                while (!body.isRoot && body.transform.parent != null) {
                    ArticulationBody parent = body.transform.parent.GetComponentInParent<ArticulationBody>();
                    if (parent == null) break;
                    body = parent;
                }
                return body.transform;
            }
            Rigidbody rigidbody = part.GetComponentInParent<Rigidbody>();
            return rigidbody != null ? rigidbody.transform : null;
        }

        /// <summary>The named object, else the first articulated robot, else the heaviest
        /// dynamic rigidbody.</summary>
        private static Transform FindRobot(string name) {
            if (!string.IsNullOrEmpty(name)) return GameObject.Find(name)?.transform;
            foreach (ArticulationBody body in FindObjectsByType<ArticulationBody>(FindObjectsInactive.Exclude))
                if (body.isRoot && !body.immovable) return body.transform;
            Rigidbody heaviest = null;
            foreach (Rigidbody body in FindObjectsByType<Rigidbody>(FindObjectsInactive.Exclude))
                if (!body.isKinematic && (heaviest == null || body.mass > heaviest.mass)) heaviest = body;
            return heaviest != null ? heaviest.transform : null;
        }

        private void BindInput() {
            for (int i = 0; i < config.Presets.Length; i++) {
                int index = i;
                if (string.IsNullOrWhiteSpace(config.Presets[i].key)) continue;
                InputAction select = Add(new InputAction($"Viewer {config.Presets[i].name}", InputActionType.Button, config.Presets[i].key));
                select.performed += _ => Select(index);
            }
            InputAction next = Add(new InputAction("Viewer Next", InputActionType.Button, config.NextKey));
            next.performed += _ => Select((preset + 1) % config.Presets.Length);
            orbitButton = Add(new InputAction("Viewer Orbit", InputActionType.Button, config.OrbitButton));
            orbitDelta = Add(new InputAction("Viewer Orbit Delta", InputActionType.Value, config.OrbitDelta));
            zoomAxis = Add(new InputAction("Viewer Zoom", InputActionType.Value, config.ZoomAxis));
        }

        private InputAction Add(InputAction action) {
            action.Enable();
            actions.Add(action);
            return action;
        }

        private void ReadOrbitInput() {
            if (orbitButton == null) return;
            if (orbitButton.IsPressed()) {
                Vector2 delta = orbitDelta.ReadValue<Vector2>() * config.OrbitSensitivity;
                yawOffset += delta.x;
                pitchOffset -= delta.y;
            }
            // Scroll units differ per platform; one notch is one step either way.
            float scroll = zoomAxis.ReadValue<float>();
            if (Mathf.Abs(scroll) > 1e-3f) zoomOffset -= Mathf.Sign(scroll) * config.ZoomStep;
        }

        private void OnDestroy() {
            foreach (InputAction action in actions) action.Dispose();
            actions.Clear();
        }

        private static string Read(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("--") ? args[index + 1] : null;
        }
    }
}
