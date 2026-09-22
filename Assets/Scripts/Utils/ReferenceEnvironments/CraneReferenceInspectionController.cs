using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Sim.Utils.ReferenceEnvironments {
    /// <summary>
    /// Optional presentation-only inspection controls for reference environments. The controller
    /// moves only the spectator camera and draws transient overlays; it never changes canonical
    /// geometry, colliders, rigid bodies, sensors, or navigation state.
    /// </summary>
    public sealed class CraneReferenceInspectionController : MonoBehaviour {
        private enum ViewMode { Overview, Oblique, Follow }

        [SerializeField] private Camera inspectionCamera;
        [SerializeField] private Transform followTarget;
        [SerializeField] private string environmentId = string.Empty;
        [SerializeField] private string routeId = string.Empty;
        [SerializeField] private string[] relevantObstacleIds = Array.Empty<string>();
        [SerializeField] private Vector3 overviewCenter = new(0f, 0f, 13f);

        private readonly List<Vector3> trajectory = new();
        private CraneSemanticEvidenceHighlighter highlighter;
        private LineRenderer trajectoryRenderer;
        private GameObject colliderOverlayHost;
        private Material colliderOverlayMaterial;
        private ViewMode viewMode = ViewMode.Overview;
        private bool semanticHighlightEnabled;
        private bool colliderOverlayEnabled;
        private bool trajectoryEnabled = true;

        public void Configure(Camera camera, Transform target, string environment, string route,
            string[] obstacleIds, Vector3 center) {
            inspectionCamera = camera;
            followTarget = target;
            environmentId = environment ?? string.Empty;
            routeId = route ?? string.Empty;
            relevantObstacleIds = obstacleIds ?? Array.Empty<string>();
            overviewCenter = center;
        }

        private void Awake() {
            highlighter = gameObject.GetComponent<CraneSemanticEvidenceHighlighter>();
            if (highlighter == null)
                highlighter = gameObject.AddComponent<CraneSemanticEvidenceHighlighter>();
            CreateTrajectoryRenderer();
        }

        private void Start() {
            SetView(ViewMode.Overview);
            if (followTarget != null) AddTrajectoryPoint(followTarget.position);
            ApplyCommandLineOptions(Environment.GetCommandLineArgs());
        }

        private void Update() {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null) {
                if (keyboard.digit1Key.wasPressedThisFrame) SetView(ViewMode.Overview);
                if (keyboard.digit2Key.wasPressedThisFrame) SetView(ViewMode.Oblique);
                if (keyboard.digit3Key.wasPressedThisFrame) SetView(ViewMode.Follow);
                if (keyboard.hKey.wasPressedThisFrame) ToggleSemanticHighlight();
                if (keyboard.cKey.wasPressedThisFrame)
                    SetColliderOverlay(!colliderOverlayEnabled);
                if (keyboard.tKey.wasPressedThisFrame) {
                    trajectoryEnabled = !trajectoryEnabled;
                    if (trajectoryRenderer != null)
                        trajectoryRenderer.enabled = trajectoryEnabled;
                }
            }

            if (followTarget != null) {
                if (trajectory.Count == 0 ||
                    Vector3.Distance(trajectory[^1], followTarget.position) >= 0.15f)
                    AddTrajectoryPoint(followTarget.position);
                if (viewMode == ViewMode.Follow) UpdateFollowView();
            }

        }

        private void SetView(ViewMode mode) {
            if (inspectionCamera == null) return;
            viewMode = mode;
            inspectionCamera.orthographic = mode == ViewMode.Overview;
            switch (mode) {
                case ViewMode.Overview:
                    inspectionCamera.orthographicSize = 14.5f;
                    inspectionCamera.transform.position = overviewCenter + Vector3.up * 24f;
                    inspectionCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
                    break;
                case ViewMode.Oblique:
                    inspectionCamera.transform.position = overviewCenter + new Vector3(14f, 14f, -17f);
                    inspectionCamera.transform.LookAt(overviewCenter + Vector3.up * 0.5f);
                    break;
                case ViewMode.Follow:
                    UpdateFollowView();
                    break;
            }
        }

        private void UpdateFollowView() {
            if (inspectionCamera == null || followTarget == null) return;
            Vector3 lookAt = followTarget.position + Vector3.up * 0.25f;
            Vector3 position = ResolveFollowPosition(lookAt);
            inspectionCamera.transform.position = Vector3.Lerp(
                inspectionCamera.transform.position, position, 8f * Time.unscaledDeltaTime);
            inspectionCamera.transform.LookAt(lookAt);
        }

        private Vector3 ResolveFollowPosition(Vector3 lookAt) {
            Vector3 forward = Vector3.ProjectOnPlane(followTarget.forward, Vector3.up).normalized;
            if (forward.sqrMagnitude < 0.5f) forward = Vector3.forward;
            Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;
            const float distance = 1.8f;
            Vector3 lift = Vector3.up * 1.25f;
            Vector3 behind = followTarget.position - forward * distance + lift;
            if (IsFollowPositionClear(lookAt, behind)) return behind;

            // Near a boundary, a side view preserves both robot scale and environment context.
            Vector3 left = followTarget.position - right * distance + lift;
            Vector3 rightSide = followTarget.position + right * distance + lift;
            bool leftClear = IsFollowPositionClear(lookAt, left);
            bool rightClear = IsFollowPositionClear(lookAt, rightSide);
            if (leftClear && rightClear)
                return PlanarDistanceFromCenter(left) <= PlanarDistanceFromCenter(rightSide)
                    ? left : rightSide;
            if (leftClear) return left;
            if (rightClear) return rightSide;

            Vector3 ahead = followTarget.position + forward * distance + lift;
            return IsFollowPositionClear(lookAt, ahead)
                ? ahead
                : followTarget.position + Vector3.up * 4.5f;
        }

        private static bool IsFollowPositionClear(Vector3 lookAt, Vector3 candidate) {
            // Start above the robot so its own chassis cannot reject every camera position.
            Vector3 sightlineStart = lookAt + Vector3.up * 0.6f;
            return !Physics.Linecast(sightlineStart, candidate, out _,
                Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
        }

        private float PlanarDistanceFromCenter(Vector3 position) {
            Vector3 offsetFromCenter = position - overviewCenter;
            offsetFromCenter.y = 0f;
            return offsetFromCenter.sqrMagnitude;
        }

        private void ToggleSemanticHighlight() {
            semanticHighlightEnabled = !semanticHighlightEnabled;
            if (semanticHighlightEnabled) highlighter.Highlight(relevantObstacleIds);
            else highlighter.Clear();
        }

        private void ApplyCommandLineOptions(string[] args) {
            for (int index = 0; index < args.Length; index++) {
                if (args[index] == "--crane-inspection-view" && index + 1 < args.Length) {
                    string requested = args[++index];
                    if (Enum.TryParse(requested, true, out ViewMode parsed)) SetView(parsed);
                    else Debug.LogWarning($"Unknown reference inspection view '{requested}'.");
                }
                else if (args[index] == "--crane-inspection-semantic-overlay" &&
                         !semanticHighlightEnabled) {
                    ToggleSemanticHighlight();
                }
                else if (args[index] == "--crane-inspection-collider-overlay") {
                    SetColliderOverlay(true);
                }
                else if (args[index] == "--crane-inspection-no-trajectory") {
                    trajectoryEnabled = false;
                    if (trajectoryRenderer != null) trajectoryRenderer.enabled = false;
                }
            }
        }

        private void CreateTrajectoryRenderer() {
            var host = new GameObject("Reference Trajectory Overlay");
            host.transform.SetParent(transform, false);
            trajectoryRenderer = host.AddComponent<LineRenderer>();
            trajectoryRenderer.useWorldSpace = true;
            trajectoryRenderer.widthMultiplier = 0.055f;
            trajectoryRenderer.startColor = new Color(0.05f, 0.95f, 1f, 1f);
            trajectoryRenderer.endColor = new Color(0.1f, 0.45f, 1f, 1f);
            trajectoryRenderer.sharedMaterial = new Material(
                Shader.Find("HDRP/Unlit") ?? Shader.Find("Sprites/Default"));
            trajectoryRenderer.positionCount = 0;
        }

        private void AddTrajectoryPoint(Vector3 point) {
            point.y += 0.12f;
            trajectory.Add(point);
            trajectoryRenderer.positionCount = trajectory.Count;
            trajectoryRenderer.SetPosition(trajectory.Count - 1, point);
        }

        private void SetColliderOverlay(bool enabled) {
            colliderOverlayEnabled = enabled;
            if (colliderOverlayHost == null && enabled) CreateColliderOverlay();
            if (colliderOverlayHost != null) colliderOverlayHost.SetActive(enabled);
        }

        private void CreateColliderOverlay() {
            colliderOverlayHost = new GameObject("Reference Collider Overlay");
            colliderOverlayHost.transform.SetParent(transform, false);
            colliderOverlayMaterial = new Material(
                Shader.Find("HDRP/Unlit") ?? Shader.Find("Sprites/Default"));
            foreach (CraneSemanticIdentity identity in FindObjectsByType<CraneSemanticIdentity>(
                         FindObjectsInactive.Exclude, FindObjectsSortMode.None)) {
                BoxCollider box = identity.GetComponent<BoxCollider>();
                if (box == null || !box.enabled) continue;
                CreateBoxLine(box, identity.SemanticId);
            }
        }

        private void CreateBoxLine(BoxCollider box, string semanticId) {
            Vector3 half = box.size * 0.5f;
            Vector3[] local = {
                box.center + new Vector3(-half.x, -half.y, -half.z),
                box.center + new Vector3( half.x, -half.y, -half.z),
                box.center + new Vector3( half.x, -half.y,  half.z),
                box.center + new Vector3(-half.x, -half.y,  half.z),
                box.center + new Vector3(-half.x,  half.y, -half.z),
                box.center + new Vector3( half.x,  half.y, -half.z),
                box.center + new Vector3( half.x,  half.y,  half.z),
                box.center + new Vector3(-half.x,  half.y,  half.z)
            };
            for (int i = 0; i < local.Length; i++) local[i] = box.transform.TransformPoint(local[i]);
            int[] path = { 0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4 };
            var lineObject = new GameObject($"collider-{semanticId}");
            lineObject.transform.SetParent(colliderOverlayHost.transform, false);
            LineRenderer line = lineObject.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.widthMultiplier = 0.025f;
            line.startColor = new Color(0.05f, 1f, 0.2f, 1f);
            line.endColor = line.startColor;
            line.sharedMaterial = colliderOverlayMaterial;
            line.positionCount = path.Length;
            for (int index = 0; index < path.Length; index++)
                line.SetPosition(index, local[path[index]]);
        }

        private void OnGUI() {
            const float width = 500f;
            GUI.Box(new Rect(14f, 14f, width, 116f), GUIContent.none);
            GUI.Label(new Rect(28f, 24f, width - 24f, 24f),
                $"CRANE reference inspection — {environmentId}");
            GUI.Label(new Rect(28f, 47f, width - 24f, 24f), $"Route: {routeId}");
            GUI.Label(new Rect(28f, 70f, width - 24f, 24f),
                $"View: {viewMode} | semantic H: {State(semanticHighlightEnabled)} | " +
                $"colliders C: {State(colliderOverlayEnabled)} | trajectory T: {State(trajectoryEnabled)}");
            GUI.Label(new Rect(28f, 93f, width - 24f, 24f),
                "Views: 1 overview   2 oblique   3 robot follow");
        }

        private static string State(bool enabled) => enabled ? "ON" : "OFF";
    }
}
