using System;
using RosMessageTypes.Sensor;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Sim.Sensors.Vision {
    /// <summary>
    /// Render-independent pinhole depth camera. Values are optical-axis Z in metres, stored as
    /// native-endian ROS 32FC1 rows from image top to bottom. A pixel with no physical collider in
    /// the configured near/far interval is quiet NaN.
    /// </summary>
    public sealed class GeometricDepthCamera : MonoBehaviour, IROSSensor<ImageMsg> {
        [SerializeField] private Camera sensorCamera;
        [SerializeField, Min(1)] private int width = 320;
        [SerializeField, Min(1)] private int height = 180;
        [SerializeField, Min(1)] private int batchSize = 64;
        [SerializeField] private LayerMask collisionMask = ~0;
        [SerializeField] private string topicName = "camera/depth/image_rect_raw";
        [SerializeField] private string frameId = "front_camera_link";
        [SerializeField, Min(0.01f)] private float Hz = 15f;

        public ROSPublisher publisher { get; set; }
        public int Width => width;
        public int Height => height;
        public float NearClip => sensorCamera != null ? sensorCamera.nearClipPlane : 0f;
        public float FarClip => sensorCamera != null ? sensorCamera.farClipPlane : 0f;

        private NativeArray<Vector3> localDirections;
        private NativeArray<float> opticalAxisFactors;
        private NativeArray<RaycastCommand> commands;
        private NativeArray<RaycastHit> hits;
        private NativeArray<byte> packedDepth;
        private byte[] depthData;
        private bool manualAcquisition;
        private float configuredVerticalFov;
        private float configuredAspect;

        [BurstCompile(FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.High)]
        private struct PopulateCommandsJob : IJobParallelFor {
            [ReadOnly] public NativeArray<Vector3> LocalDirections;
            [WriteOnly] public NativeArray<RaycastCommand> Commands;
            public Vector3 CameraPosition;
            public Quaternion CameraRotation;
            public float NearClip;
            public float FarClip;
            public int LayerMask;

            public void Execute(int index) {
                Vector3 localDirection = LocalDirections[index];
                float opticalFactor = math.max(1e-6f, localDirection.z);
                float radialNear = NearClip / opticalFactor;
                Vector3 worldDirection = CameraRotation * localDirection;
                QueryParameters query = QueryParameters.Default;
                query.layerMask = LayerMask;
                query.hitTriggers = QueryTriggerInteraction.Ignore;
                Commands[index] = new RaycastCommand(
                    CameraPosition + worldDirection * radialNear,
                    worldDirection, query, (FarClip - NearClip) / opticalFactor);
            }
        }

        [BurstCompile(FloatMode = FloatMode.Strict, FloatPrecision = FloatPrecision.High)]
        private struct PackDepthJob : IJobParallelFor {
            [ReadOnly] public NativeArray<RaycastHit> Hits;
            [ReadOnly] public NativeArray<float> OpticalAxisFactors;
            [WriteOnly, NativeDisableParallelForRestriction] public NativeArray<byte> Bytes;
            public float NearClip;

            public void Execute(int index) {
                float radialDistance = Hits[index].distance;
                float depth = radialDistance > 0f
                    ? NearClip + radialDistance * OpticalAxisFactors[index]
                    : float.NaN;
                uint bits = math.asuint(depth);
                int offset = index * sizeof(float);
                Bytes[offset] = (byte)bits;
                Bytes[offset + 1] = (byte)(bits >> 8);
                Bytes[offset + 2] = (byte)(bits >> 16);
                Bytes[offset + 3] = (byte)(bits >> 24);
            }
        }

        private void Awake() => publisher = gameObject.AddComponent<ROSPublisher>();

        /// <summary>Configures a runtime replacement before Start allocates native buffers.</summary>
        public void Configure(Camera camera, int imageWidth, int imageHeight, string topic,
            string opticalFrame, float rateHz, bool manual = false) {
            sensorCamera = camera;
            if (camera != null) {
                collisionMask = camera.cullingMask;
                // This component reads intrinsics only. Leaving the Camera enabled would silently
                // turn a geometric observation back into a rendered observation.
                camera.enabled = false;
            }
            width = Math.Max(1, imageWidth);
            height = Math.Max(1, imageHeight);
            topicName = string.IsNullOrWhiteSpace(topic) ? topicName : topic;
            frameId = string.IsNullOrWhiteSpace(opticalFrame) ? frameId : opticalFrame;
            Hz = Mathf.Max(0.01f, rateHz);
            manualAcquisition = manual;
            if (localDirections.IsCreated) RebuildProjection();
        }

        private void Start() {
            if (sensorCamera == null) {
                Debug.LogError("Geometric depth requires a Camera for pinhole intrinsics, but it " +
                               "does not require that Camera to be enabled or rendered.");
                enabled = false;
                return;
            }
            RebuildProjection();
            publisher.Initialize(topicName, frameId, CreateMessage, Hz, manualAcquisition);
        }

        private void FixedUpdate() {
            if (sensorCamera == null) return;
            float aspect = width / (float)height;
            if (!Mathf.Approximately(configuredVerticalFov, sensorCamera.fieldOfView) ||
                !Mathf.Approximately(configuredAspect, aspect))
                RebuildProjection();
        }

        public ImageMsg CreateMessage() {
            CaptureNow();
            var message = new ImageMsg(publisher.CreateHeader(), (uint)height, (uint)width,
                "32FC1", 0, (uint)(width * sizeof(float)), depthData);
            CraneRuntimeMetrics.ReportImage(true, width, height, depthData,
                CraneRuntimeMetrics.SimulationTick);
            CraneRuntimeMetrics.ReportObservation(CraneRuntimeMetrics.SimulationTick);
            return message;
        }

        /// <summary>Captures synchronously at the current physics state; primarily useful in tests.</summary>
        public void CaptureNow() {
            if (!localDirections.IsCreated) RebuildProjection();
            using var marker = CraneProfiler.DepthGeometric.Auto();
            Transform cameraTransform = sensorCamera.transform;
            int count = width * height;
            JobHandle populate = new PopulateCommandsJob {
                LocalDirections = localDirections,
                Commands = commands,
                CameraPosition = cameraTransform.position,
                CameraRotation = cameraTransform.rotation,
                NearClip = sensorCamera.nearClipPlane,
                FarClip = sensorCamera.farClipPlane,
                LayerMask = collisionMask.value
            }.Schedule(count, batchSize);
            JobHandle raycasts = RaycastCommand.ScheduleBatch(commands, hits, batchSize, 1,
                populate);
            JobHandle pack = new PackDepthJob {
                Hits = hits,
                OpticalAxisFactors = opticalAxisFactors,
                Bytes = packedDepth,
                NearClip = sensorCamera.nearClipPlane
            }.Schedule(count, batchSize, raycasts);
            pack.Complete();

            int byteCount = count * sizeof(float);
            if (!ROSPublisher.TransportSuppressed || depthData == null ||
                depthData.Length != byteCount)
                depthData = new byte[byteCount];
            packedDepth.CopyTo(depthData);
        }

        public float GetDepth(int x, int y) {
            if (depthData == null) throw new InvalidOperationException("CaptureNow must run first.");
            if ((uint)x >= width || (uint)y >= height)
                throw new ArgumentOutOfRangeException($"Pixel ({x}, {y}) is outside {width}x{height}.");
            return BitConverter.ToSingle(depthData, (y * width + x) * sizeof(float));
        }

        private void RebuildProjection() {
            if (sensorCamera == null) return;
            DisposeBuffers();
            int count = checked(width * height);
            localDirections = new NativeArray<Vector3>(count, Allocator.Persistent);
            opticalAxisFactors = new NativeArray<float>(count, Allocator.Persistent);
            commands = new NativeArray<RaycastCommand>(count, Allocator.Persistent);
            hits = new NativeArray<RaycastHit>(count, Allocator.Persistent);
            packedDepth = new NativeArray<byte>(checked(count * sizeof(float)),
                Allocator.Persistent);

            float tangent = Mathf.Tan(0.5f * sensorCamera.fieldOfView * Mathf.Deg2Rad);
            float aspect = width / (float)height;
            for (int y = 0; y < height; y++) {
                float localY = (1f - 2f * ((y + 0.5f) / height)) * tangent;
                for (int x = 0; x < width; x++) {
                    float localX = (2f * ((x + 0.5f) / width) - 1f) * tangent * aspect;
                    Vector3 direction = new Vector3(localX, localY, 1f).normalized;
                    int index = y * width + x;
                    localDirections[index] = direction;
                    opticalAxisFactors[index] = direction.z;
                }
            }
            configuredVerticalFov = sensorCamera.fieldOfView;
            configuredAspect = aspect;
        }

        private void OnDestroy() => DisposeBuffers();

        private void DisposeBuffers() {
            if (localDirections.IsCreated) localDirections.Dispose();
            if (opticalAxisFactors.IsCreated) opticalAxisFactors.Dispose();
            if (commands.IsCreated) commands.Dispose();
            if (hits.IsCreated) hits.Dispose();
            if (packedDepth.IsCreated) packedDepth.Dispose();
        }
    }
}
