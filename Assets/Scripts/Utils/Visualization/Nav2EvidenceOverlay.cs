using System;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Visualization
{
    /// <summary>
    /// Read-only Nav2 evidence overlay. It draws the received plan, odometry trail, and
    /// occupancy grid without changing navigation or physics. Topics are explicit so a capture
    /// manifest can record the exact interfaces used.
    /// </summary>
    public sealed class Nav2EvidenceOverlay : MonoBehaviour
    {
        [SerializeField] private string planTopic = "/plan";
        [SerializeField] private string odomTopic = "/crane/odom";
        [SerializeField] private string costmapTopic = "/local_costmap/costmap";
        [SerializeField] private string twistTopic = "/crane/cmd_vel_stamped";
        [SerializeField] private Material planMaterial;
        [SerializeField] private Material odomMaterial;
        [SerializeField] private Material costmapMaterial;
        [SerializeField] private float lineWidth = 0.035f;
        [SerializeField] private float costmapHeight = 0.12f;
        [SerializeField] private float maxCostmapRadius = 7.5f;
        [SerializeField] private bool renderUnknownCells = false;
        [SerializeField, Range(0.05f, 0.8f)] private float costmapOpacity = 0.28f;
        [SerializeField] private bool showCollisionMarkers = true;
        [SerializeField] private string responseText = "E1 plan/costmap; E2 command; E3 observed motion. Collision cause not established.";
        [SerializeField] private int maxOdomPoints = 2000;
        private LineRenderer planLine, odomLine;
        private LineRenderer twistArrow;
        private Vector3 latestPose;
        private bool havePose;
        private Quaternion latestOrientation = Quaternion.identity;
        private readonly Dictionary<int, Material> ownedMaterials = new();
        private const float EvidenceElevation = 0.20f;
        private TMPro.TextMeshProUGUI hudStatus;
        private float nextHudUpdate;
        private readonly List<Vector3> odom = new();
        private Transform costmapRoot;
        public const int VisualizationLayer = 31;
        private Mesh costmapMesh;
        private WaterSurface water;
        private float nextWaterSample;
        [SerializeField] private float waterClearance = .45f;

        private int renderedCellCount;


        [Serializable]
        private sealed class CaptureMetadata {
            public string schema = "crane-overlay-capture-v1";
            public string utc, planTopic, odomTopic, costmapTopic, twistTopic, dockingEvaluationJson, checkpointLabel;
            public string planFrame, odomFrame, costmapFrame, twistFrame;
            public double simulationTime, planStamp, odomStamp, costmapStamp, twistStamp;
            public int planMessages, odomMessages, costmapMessages, twistMessages;
            public int renderedCells, activeVisualColliders, costmapWidth, costmapHeight;
            public float costmapResolution, displayedHeightOffset, sampledMaximumWaterHeight, waterClearance;
            public int validWaterHeightProbes;
            public bool sensorCamerasExcludeOverlay;

            public Vector3 costmapOrigin;
            public Quaternion costmapOriginRotation;
            public double[] planXY;
            public string costmapDataBase64;
            public Vector3 observedPosition, desiredBodyVelocity, cameraPosition;
            public Quaternion cameraRotation;
            public float cameraFieldOfView, firstDockDistance, secondDockDistance;
            public Vector3 firstDockCenter, firstDockSize, secondDockCenter, secondDockSize;
            public Quaternion observedOrientation;
            public string boundary = "Received messages; not proof of internal Nav2 consumption. Screenshot requested at frame end.";
        }
        private readonly CaptureMetadata evidence = new CaptureMetadata();
        [Serializable] private sealed class DockState { public bool success; }
        public bool DockingQualified { get; private set; }


        public bool ReadyForCapture => evidence.planMessages > 0 && evidence.odomMessages > 0 && evidence.costmapMessages > 0 && evidence.twistMessages > 0;
        public void RecordDockProximity(Bounds first, Bounds second, Vector3 boat, string label) {
            evidence.firstDockCenter = first.center; evidence.firstDockSize = first.size;
            evidence.secondDockCenter = second.center; evidence.secondDockSize = second.size;
            evidence.firstDockDistance = HorizontalDistanceToBounds(first, boat);
            evidence.secondDockDistance = HorizontalDistanceToBounds(second, boat);
            evidence.checkpointLabel = label;
        }
        public static float HorizontalDistanceToBounds(Bounds bounds, Vector3 position) {
            var closest = bounds.ClosestPoint(position);
            return Vector2.Distance(new Vector2(position.x, position.z), new Vector2(closest.x, closest.z));
        }

        public void RecordCamera(Camera camera) {
            evidence.cameraPosition = camera.transform.position;
            evidence.cameraRotation = camera.transform.rotation;
            evidence.cameraFieldOfView = camera.fieldOfView;
        }

        public string CaptureMetadataJson() {
            evidence.utc = DateTime.UtcNow.ToString("O");
            evidence.simulationTime = Time.fixedTimeAsDouble;
            evidence.planTopic = planTopic; evidence.odomTopic = odomTopic;
            evidence.costmapTopic = costmapTopic; evidence.twistTopic = twistTopic;
            evidence.renderedCells = renderedCellCount;
            evidence.displayedHeightOffset = costmapRoot.position.y;
            evidence.waterClearance = waterClearance;
            evidence.sensorCamerasExcludeOverlay = true;
            foreach (var camera in FindObjectsByType<Camera>(FindObjectsInactive.Include))
                if (camera.targetTexture != null && (camera.cullingMask & (1 << VisualizationLayer)) != 0)
                    evidence.sensorCamerasExcludeOverlay = false;
            evidence.activeVisualColliders = 0;
            foreach (var collider in GetComponentsInChildren<Collider>())
                if (collider.enabled) evidence.activeVisualColliders++;
            return JsonUtility.ToJson(evidence, true);
        }

        private void Awake()
        {
            planLine = CreateLine("Nav2 plan (received)", planMaterial, Color.cyan);
            odomLine = CreateLine("Observed odometry", odomMaterial, Color.yellow);
            twistArrow = CreateLine("Published twist (command)", odomMaterial, Color.magenta);
            costmapRoot = new GameObject("Costmap (received)").transform;
            costmapRoot.SetParent(transform, false);
            costmapRoot.gameObject.layer = VisualizationLayer;
            water = FindAnyObjectByType<WaterSurface>();
            costmapRoot.position = Vector3.up * waterClearance;
            costmapMesh = new Mesh { name = "Received costmap tiles", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            costmapRoot.gameObject.AddComponent<MeshFilter>().sharedMesh = costmapMesh;
            costmapRoot.gameObject.AddComponent<MeshRenderer>().sharedMaterials = new[] {
                costmapMaterial != null ? costmapMaterial : GetMaterial(100, new Color(.85f, .18f, .12f, .46f)),
                costmapMaterial != null ? costmapMaterial : GetMaterial(80, new Color(.95f, .60f, .12f, costmapOpacity)),
                costmapMaterial != null ? costmapMaterial : GetMaterial(-1, new Color(.35f, .35f, .35f, .08f))
            };
            CreateHud();
            gameObject.AddComponent<ROSSubscriber>().Initialize<PathMsg>(planTopic, ReceivePlan);
            gameObject.AddComponent<ROSSubscriber>().Initialize<OdometryMsg>(odomTopic, ReceiveOdom);
            gameObject.AddComponent<ROSSubscriber>().Initialize<OccupancyGridMsg>(costmapTopic, ReceiveCostmap);
            gameObject.AddComponent<ROSSubscriber>().Initialize<TwistStampedMsg>(twistTopic, ReceiveTwist);
            gameObject.AddComponent<ROSSubscriber>().Initialize<RosMessageTypes.Std.StringMsg>("/crane/docking_evaluator", message => {
                evidence.dockingEvaluationJson = message.data;
                DockingQualified = JsonUtility.FromJson<DockState>(message.data)?.success == true;
            });
        }

        private void CreateHud() {
            var canvasHost = new GameObject("Nav2 Evidence HUD", typeof(RectTransform), typeof(Canvas), typeof(UnityEngine.UI.CanvasScaler));
            canvasHost.transform.SetParent(transform, false);
            var canvas = canvasHost.GetComponent<Canvas>(); canvas.renderMode = RenderMode.ScreenSpaceOverlay; canvas.sortingOrder = 100;
            var scaler = canvasHost.GetComponent<UnityEngine.UI.CanvasScaler>();
            scaler.uiScaleMode = UnityEngine.UI.CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1600, 900); scaler.matchWidthOrHeight = .5f;
            var panel = new GameObject("Evidence Legend", typeof(RectTransform), typeof(UnityEngine.UI.Image), typeof(UnityEngine.UI.VerticalLayoutGroup));
            panel.transform.SetParent(canvasHost.transform, false);
            var rect = panel.GetComponent<RectTransform>(); rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(0, 1);
            rect.anchoredPosition = new Vector2(28, -28); rect.sizeDelta = new Vector2(304, 216);
            var background = panel.GetComponent<UnityEngine.UI.Image>(); background.color = new Color(.035f, .075f, .10f, .90f); background.raycastTarget = false;
            var layout = panel.GetComponent<UnityEngine.UI.VerticalLayoutGroup>();
            layout.padding = new RectOffset(20, 20, 16, 16); layout.spacing = 6;
            layout.childControlWidth = true; layout.childControlHeight = true; layout.childForceExpandWidth = true; layout.childForceExpandHeight = false;
            AddHudText(panel.transform, "ROBOBOAT / NAV2", 20, Color.white, 28);
            AddHudText(panel.transform, "LIVE NAVIGATION", 11, new Color(.57f, .70f, .74f), 18);
            AddHudText(panel.transform, "<color=#45DDE8>—</color>  Planned route", 15, Color.white, 21);
            AddHudText(panel.transform, "<color=#F4D45F>—</color>  Observed motion", 15, Color.white, 21);
            AddHudText(panel.transform, "<color=#E77AE9>—</color>  Desired velocity", 15, Color.white, 21);
            AddHudText(panel.transform, "<color=#E58A5A>■</color>   Local costmap", 15, Color.white, 21);
            hudStatus = AddHudText(panel.transform, "Waiting for received evidence", 11, new Color(.67f, .77f, .79f), 18);
        }

        private static TMPro.TextMeshProUGUI AddHudText(Transform parent, string content, float size, Color color, float height) {
            var host = new GameObject("Legend text", typeof(RectTransform), typeof(TMPro.TextMeshProUGUI), typeof(UnityEngine.UI.LayoutElement));
            host.transform.SetParent(parent, false);
            var label = host.GetComponent<TMPro.TextMeshProUGUI>();
            label.font = Resources.Load<TMPro.TMP_FontAsset>("Fonts & Materials/LiberationSans SDF");
            label.text = content; label.fontSize = size; label.color = color; label.raycastTarget = false;
            host.GetComponent<UnityEngine.UI.LayoutElement>().preferredHeight = height;
            return label;
        }

        private void LateUpdate() {
            if (hudStatus != null && Time.unscaledTime >= nextHudUpdate) {
                nextHudUpdate = Time.unscaledTime + .25f;
                hudStatus.text = DockingQualified ? $"Dock qualified · {evidence.odomStamp:0.0} s" : $"Received evidence · {evidence.odomStamp:0.0} s · {renderedCellCount:N0} tiles";
            }
            if (!havePose || Time.unscaledTime < nextWaterSample) return;
            nextWaterSample = Time.unscaledTime + .2f;
            float maximum = water != null ? water.transform.position.y : 0f;
            evidence.validWaterHeightProbes = 0;
            if (water != null) {
                // Sample the displayed local window, including its edges. Moving one mesh
                // above the highest sampled wave keeps the received map geometry unchanged.
                for (int x = -1; x <= 1; x++) for (int z = -1; z <= 1; z++) {
                    Vector3 probe = latestPose + new Vector3(x * maxCostmapRadius, 0f, z * maxCostmapRadius);
                    var query = new WaterSearchParameters { startPositionWS = probe, targetPositionWS = probe,
                        error = .01f, maxIterations = 8, includeDeformation = true, excludeSimulation = false };
                    if (water.ProjectPointOnWaterSurface(query, out var result)) {
                        maximum = Mathf.Max(maximum, result.projectedPositionWS.y);
                        evidence.validWaterHeightProbes++;
                    }
                }
            }
            evidence.sampledMaximumWaterHeight = maximum;
            costmapRoot.position = Vector3.up * (maximum + waterClearance);
        }

        private LineRenderer CreateLine(string label, Material material, Color fallback)
        {
            var go = new GameObject(label); go.transform.SetParent(transform, false);
            go.layer = VisualizationLayer;
            var line = go.AddComponent<LineRenderer>();
            line.sharedMaterial = material != null ? material : GetMaterial(-10 - ownedMaterials.Count, fallback);
            line.startColor = line.endColor = fallback;
            line.startWidth = line.endWidth = lineWidth;
            line.useWorldSpace = true;
            return line;
        }

        private static Vector3 RosToUnity(double x, double y, double z = 0)
            => new((float)-y, (float)z, (float)x);

        // ROS FLU to Unity is a reflection, so quaternion vector components use -M(q.xyz).
        private static Quaternion RosToUnity(QuaternionMsg q)
            => q == null ? Quaternion.identity : new Quaternion((float)q.y, (float)-q.z,
                (float)-q.x, (float)q.w).normalized;

        private Material GetMaterial(int key, Color color)
        {
            if (ownedMaterials.TryGetValue(key, out var existing)) return existing;
            var shader = Shader.Find("HDRP/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
            if (shader == null) return null;
            var material = new Material(shader);
            material.color = color;
            if (material.HasProperty("_UnlitColor")) material.SetColor("_UnlitColor", color);
            if (material.HasProperty("_SurfaceType")) {
                material.SetFloat("_SurfaceType", 1f);
                material.SetFloat("_ZWrite", 0f);
                material.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
                material.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
                material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                material.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            }
            ownedMaterials[key] = material;
            return material;
        }

        private void OnDestroy()
        {
            foreach (var material in ownedMaterials.Values) Destroy(material);
            if (costmapMesh != null) Destroy(costmapMesh);
        }

        private void ReceivePlan(PathMsg msg)
        {
            if (msg?.poses == null) return;
            evidence.planMessages++; evidence.planFrame = msg.header.frame_id;
            evidence.planStamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
            evidence.planXY = new double[msg.poses.Length * 2];
            for (int i = 0; i < msg.poses.Length; i++) {
                evidence.planXY[i * 2] = msg.poses[i].pose.position.x;
                evidence.planXY[i * 2 + 1] = msg.poses[i].pose.position.y;
            }
            var points = new Vector3[msg.poses.Length];
            for (var i = 0; i < points.Length; i++)
                points[i] = RosToUnity(msg.poses[i].pose.position.x, msg.poses[i].pose.position.y) + Vector3.up * EvidenceElevation;
            planLine.positionCount = points.Length; planLine.SetPositions(points);
        }

        private void ReceiveOdom(OdometryMsg msg)
        {
            if (msg?.pose?.pose?.position == null) return;
            evidence.odomMessages++; evidence.odomFrame = msg.header.frame_id;
            evidence.odomStamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
            odom.Add(RosToUnity(msg.pose.pose.position.x, msg.pose.pose.position.y) + Vector3.up * EvidenceElevation);
            latestOrientation = RosToUnity(msg.pose.pose.orientation);
            latestPose = odom[odom.Count - 1]; havePose = true;
            evidence.observedPosition = latestPose; evidence.observedOrientation = latestOrientation;
            if (odom.Count > maxOdomPoints) odom.RemoveAt(0);
            odomLine.positionCount = odom.Count; odomLine.SetPositions(odom.ToArray());
        }

        private void ReceiveTwist(TwistStampedMsg msg)
        {
            if (msg?.twist?.linear == null || !havePose) return;
            evidence.twistMessages++; evidence.twistFrame = msg.header.frame_id;
            evidence.twistStamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
            evidence.desiredBodyVelocity = new Vector3((float)msg.twist.linear.x, (float)msg.twist.linear.y, (float)msg.twist.linear.z);
            // Arrow is a commanded body velocity. It is intentionally separate from odometry.
            var tip = latestPose + latestOrientation * RosToUnity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z) * 1.5f;
            twistArrow.positionCount = 2;
            twistArrow.SetPosition(0, latestPose + Vector3.up * .08f);
            twistArrow.SetPosition(1, tip + Vector3.up * .08f);
        }

        private void ReceiveCostmap(OccupancyGridMsg msg)
        {
            if (msg?.info == null || msg.data == null) return;
            var width = (int)msg.info.width; var height = (int)msg.info.height;
            var resolution = (float)msg.info.resolution;
            if (width <= 0 || height <= 0 || resolution <= 0 || (long)width * height > msg.data.Length) return;
            evidence.costmapMessages++; evidence.costmapFrame = msg.header.frame_id;
            evidence.costmapStamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
            evidence.costmapWidth = width; evidence.costmapHeight = height;
            evidence.costmapResolution = resolution;
            var bytes = new byte[msg.data.Length]; Buffer.BlockCopy(msg.data, 0, bytes, 0, bytes.Length);
            evidence.costmapDataBase64 = Convert.ToBase64String(bytes);
            Vector3 mapOrigin = RosToUnity(msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z);
            var mapRotation = RosToUnity(msg.info.origin.orientation);
            evidence.costmapOrigin = mapOrigin; evidence.costmapOriginRotation = mapRotation;
            var vertices = new List<Vector3>();
            var indices = new[] { new List<int>(), new List<int>(), new List<int>() };
            renderedCellCount = 0;
            for (var y = 0; y < height; y++) for (var x = 0; x < width; x++)
            {
                var value = msg.data[y * width + x];
                if (value < 65 && !(renderUnknownCells && value < 0)) continue;
                Vector3 cellCenter = mapOrigin + mapRotation * RosToUnity((x + .5f) * resolution,
                    (y + .5f) * resolution);
                if (havePose && Vector2.Distance(new Vector2(cellCenter.x, cellCenter.z),
                        new Vector2(latestPose.x, latestPose.z)) > maxCostmapRadius) continue;
                var cellHeight = Mathf.Max(0.018f, costmapHeight * 0.18f * Mathf.Max(.15f, value / 100f));
                var center = cellCenter + Vector3.up * (.018f + cellHeight * .5f);
                AddTile(vertices, indices[value >= 100 ? 0 : value < 0 ? 2 : 1],
                    center, mapRotation, resolution * .45f, cellHeight * .5f);
                renderedCellCount++;
            }
            // One nonphysical mesh, three material batches. Thousands of primitive objects
            // per update stalled rendering/sensor callbacks and caused stale observations.
            costmapMesh.Clear(); costmapMesh.SetVertices(vertices); costmapMesh.subMeshCount = 3;
            for (int bucket = 0; bucket < 3; bucket++) costmapMesh.SetTriangles(indices[bucket], bucket);
            costmapMesh.RecalculateNormals(); costmapMesh.RecalculateBounds();
        }

        private static void AddTile(List<Vector3> vertices, List<int> indices,
            Vector3 center, Quaternion rotation, float halfWidth, float halfHeight) {
            int first = vertices.Count;
            for (int i = 0; i < 8; i++)
                vertices.Add(center + rotation * new Vector3((i & 1) == 0 ? -halfWidth : halfWidth,
                    (i & 2) == 0 ? -halfHeight : halfHeight, (i & 4) == 0 ? -halfWidth : halfWidth));
            int[] faces = { 2, 6, 7, 2, 7, 3, 0, 1, 5, 0, 5, 4,
                0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6,
                0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5 };
            foreach (int index in faces) indices.Add(first + index);
        }

        private void OnCollisionEnter(Collision collision)
        {
            if (!showCollisionMarkers || collision.contactCount == 0) return;
            var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "Contact marker (recorded Unity collision)";
            marker.layer = VisualizationLayer;
            var markerCollider = marker.GetComponent<Collider>();
            if (markerCollider != null) { markerCollider.enabled = false; Destroy(markerCollider); }
            marker.transform.position = collision.GetContact(0).point;
            marker.transform.localScale = Vector3.one * .22f;
            marker.transform.SetParent(transform, true);
            var renderer = marker.GetComponent<Renderer>();
            renderer.sharedMaterial = GetMaterial(-3, new Color(1f, .1f, .05f, .85f));

        }
    }
}
