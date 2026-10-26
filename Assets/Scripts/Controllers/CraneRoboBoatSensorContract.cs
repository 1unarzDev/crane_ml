using System;
using System.Globalization;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using Sim.Utils;
using Sim.Utils.Performance;
using Sim.Utils.ROS;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Controllers {
    /// <summary>
    /// Publishes what the boat's flight controller provides through MAVROS, so mhseals_nav's
    /// localization runs unchanged with no ArduPilot or MAVROS (odom.launch.py fcu:=false):
    /// /odom/mavros (body-frame velocity), /imu/raw (attitude, body rates, specific force) and
    /// /gps/fix. Frames follow REP 103 (world ENU, body FLU, frame_id base_link like MAVROS).
    /// </summary>
    public sealed class CraneRoboBoatSensorContract : MonoBehaviour, ICraneEpisodeResettable {
        public sealed class Settings {
            public string OdometryTopic = "/odom/mavros";
            public string ImuTopic = "/imu/raw";
            public string GpsTopic = "/gps/fix";
            public string WorldFrame = "odom";
            public string BaseFrame = "base_link";
            // The rates odom.launch.py requests from the real flight controller.
            public float OdometryHz = 20f;
            public float ImuHz = 20f;
            public float GpsHz = 5f;
            public double OriginLatitude = 27.3747;     // RoboBoat course, Sarasota FL
            public double OriginLongitude = -82.4518;
            public float GpsSigma;                      // m, horizontal
            public float VelocitySigma;                 // m/s, FCU velocity estimate
            public int Seed;
        }

        private const double EarthRadius = 6378137.0;
        // Covariance floors keep filters from treating exact simulated data as infinitely certain.
        private const double PositionVarianceFloor = 0.01;
        private const double VelocityVarianceFloor = 1e-4;
        private const double AttitudeVariance = 1e-4;

        private IPhysicsBody body;
        private ROSConnection ros;
        private Settings settings;
        private System.Random random;
        private Vector3 previousVelocity;
        private bool hasPreviousVelocity;
        private Vector3 specificForceWorld;
        private double nextOdometry, nextImu, nextGps;

        public int ResetPriority => -80;

        private bool topicsRegistered;
        private void RegisterTopics() {
            if (topicsRegistered) return;
            ros.RegisterPublisher<OdometryMsg>(settings.OdometryTopic);
            ros.RegisterPublisher<ImuMsg>(settings.ImuTopic);
            ros.RegisterPublisher<NavSatFixMsg>(settings.GpsTopic);
            topicsRegistered = true;
        }

        private void Start() {
            if (ros != null) RegisterTopics();
        }

        public void Initialize(Component target, Settings contractSettings) {
            body = target switch {
                Rigidbody rigidbody => new RigidbodyAdapter(rigidbody),
                ArticulationBody articulation => new ArticulationBodyAdapter(articulation),
                _ => throw new ArgumentException("The sensor contract requires a PhysX body.",
                    nameof(target))
            };
            settings = contractSettings ?? new Settings();
            random = new System.Random(settings.Seed);
            ros = ROSConnection.GetOrCreateInstance();
            // sceneLoaded for the initial build scene precedes generated message registry
            // initialization, so a registration there reaches the endpoint without a type and
            // is refused. Start runs after all runtime init methods (as in ROSPublisher).
            if (MessageRegistry.GetRosMessageName<OdometryMsg>() != null) RegisterTopics();
            Debug.Log($"CRANE_ROBOBOAT_CONTRACT_READY body={target.name} " +
                      $"odom={settings.OdometryTopic}@{settings.OdometryHz:R} " +
                      $"imu={settings.ImuTopic}@{settings.ImuHz:R} " +
                      $"gps={settings.GpsTopic}@{settings.GpsHz:R} " +
                      $"origin={settings.OriginLatitude:R},{settings.OriginLongitude:R} " +
                      $"gpsSigma={settings.GpsSigma:R} velocitySigma={settings.VelocitySigma:R}");
        }

        public void CaptureEpisodeInitialState() { }

        public void ResetEpisode(in CraneEpisodeResetContext context,
            CraneEpisodeResetPhase phase) {
            if (phase != CraneEpisodeResetPhase.BeforePhysics) return;
            hasPreviousVelocity = false;
            nextOdometry = nextImu = nextGps = 0;
        }

        private void FixedUpdate() {
            if (body == null) return;
            // Differentiate every physics step so the acceleration is exact at any publish rate.
            Vector3 velocity = body.linearVelocity;
            specificForceWorld = hasPreviousVelocity
                ? SpecificForce(velocity, previousVelocity, Time.fixedDeltaTime,
                    UnityEngine.Physics.gravity)
                : -UnityEngine.Physics.gravity;
            previousVelocity = velocity;
            hasPreviousVelocity = true;

            double now = Clock.time;
            if (Due(now, settings.OdometryHz, ref nextOdometry)) PublishOdometry(now);
            if (Due(now, settings.ImuHz, ref nextImu)) PublishImu(now);
            if (Due(now, settings.GpsHz, ref nextGps)) PublishGps(now);
        }

        /// <summary>Advances by whole periods, keeping fractional cadence at any rate.</summary>
        internal static bool Due(double now, float hz, ref double next) {
            if (now + 1e-9 < next) return false;
            double period = 1.0 / Math.Max(0.1f, hz);
            do next += period;
            while (next <= now + 1e-9);
            return true;
        }

        /// <summary>What an accelerometer measures: acceleration minus gravity (+g up at rest).</summary>
        internal static Vector3 SpecificForce(Vector3 velocity, Vector3 previous, float deltaTime,
            Vector3 gravity) => (velocity - previous) / Mathf.Max(1e-6f, deltaTime) - gravity;

        /// <summary>Local tangent-plane (east, north) metres to latitude/longitude degrees.</summary>
        internal static void EnuToGeodetic(double east, double north, double originLatitude,
            double originLongitude, out double latitude, out double longitude) {
            latitude = originLatitude + north / EarthRadius * (180.0 / Math.PI);
            longitude = originLongitude + east /
                (EarthRadius * Math.Cos(originLatitude * Math.PI / 180.0)) * (180.0 / Math.PI);
        }

        private Vector3 BodyVector(Vector3 world) =>
            RoboBoatRosFrame.ToRosLocalUnity(body.transform.InverseTransformDirection(world));

        private QuaternionMsg Orientation() => RoboBoatRosFrame.Rotation(body.rotation).To<FLU>();

        private double Gaussian(double sigma) {
            if (sigma <= 0) return 0;
            // Box-Muller; 1 - NextDouble() avoids log(0).
            double u1 = 1.0 - random.NextDouble(), u2 = random.NextDouble();
            return sigma * Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        private void PublishOdometry(double now) {
            Vector3Msg velocity = CraneROSNavigationState.ToRosVector(
                BodyVector(body.linearVelocity));
            velocity.x += Gaussian(settings.VelocitySigma);
            velocity.y += Gaussian(settings.VelocitySigma);
            var odometry = new OdometryMsg {
                header = CraneROSNavigationState.CreateHeader(now, settings.WorldFrame),
                child_frame_id = settings.BaseFrame
            };
            odometry.pose.pose.position = CraneROSNavigationState.ToRosPoint(body.position);
            odometry.pose.pose.orientation = Orientation();
            odometry.twist.twist.linear = velocity;
            odometry.twist.twist.angular = CraneROSNavigationState.ToRosAngularVector(
                BodyVector(body.angularVelocity));
            double variance = Math.Max(VelocityVarianceFloor,
                settings.VelocitySigma * settings.VelocitySigma);
            for (int i = 0; i < 6; i++) odometry.twist.covariance[i * 7] = variance;
            ros.Publish(settings.OdometryTopic, odometry);
        }

        private void PublishImu(double now) {
            var imu = new ImuMsg {
                header = CraneROSNavigationState.CreateHeader(now, settings.BaseFrame),
                orientation = Orientation(),
                angular_velocity = CraneROSNavigationState.ToRosAngularVector(
                    BodyVector(body.angularVelocity)),
                linear_acceleration = CraneROSNavigationState.ToRosVector(
                    BodyVector(specificForceWorld))
            };
            for (int i = 0; i < 3; i++) {
                imu.orientation_covariance[i * 4] = AttitudeVariance;
                imu.angular_velocity_covariance[i * 4] = AttitudeVariance;
                imu.linear_acceleration_covariance[i * 4] = AttitudeVariance;
            }
            ros.Publish(settings.ImuTopic, imu);
        }

        private void PublishGps(double now) {
            PointMsg position = CraneROSNavigationState.ToRosPoint(body.position);
            EnuToGeodetic(position.x + Gaussian(settings.GpsSigma),
                position.y + Gaussian(settings.GpsSigma),
                settings.OriginLatitude, settings.OriginLongitude,
                out double latitude, out double longitude);
            double horizontal = Math.Max(PositionVarianceFloor,
                settings.GpsSigma * settings.GpsSigma);
            ros.Publish(settings.GpsTopic, new NavSatFixMsg {
                header = CraneROSNavigationState.CreateHeader(now, settings.BaseFrame),
                status = new NavSatStatusMsg(NavSatStatusMsg.STATUS_FIX,
                    NavSatStatusMsg.SERVICE_GPS),
                latitude = latitude,
                longitude = longitude,
                altitude = position.z,
                position_covariance = new[] {
                    horizontal, 0, 0, 0, horizontal, 0, 0, 0, 4 * horizontal
                },
                position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN
            });
        }
    }

    internal static class CraneRoboBoatSensorContractBootstrap {
        internal const string Flag = "--crane-roboboat-contract";
        // The scene's own IMU/GPS publish on the same topics without the RoboBoat bow
        // correction and with an empty fix; the contract replaces them.
        private static readonly string[] ReplacedSensors = {
            "Sim.Sensors.Nav.Imu", "Sim.Sensors.Nav.GPS"
        };
        private static CraneRoboBoatSensorContract.Settings settings;
        private static bool enabled;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            string[] args = Environment.GetCommandLineArgs();
            enabled = Array.IndexOf(args, Flag) >= 0 && !ROSPublisher.TransportSuppressed;
            if (!enabled) return;
            settings = new CraneRoboBoatSensorContract.Settings {
                GpsSigma = Mathf.Max(0f, ReadFloat(args, "--crane-gps-sigma", 0f)),
                VelocitySigma = Mathf.Max(0f, ReadFloat(args, "--crane-fcu-velocity-sigma", 0f)),
                Seed = (int)ReadFloat(args, "--crane-contract-seed", 0f)
            };
            string origin = ReadString(args, "--crane-gps-origin", null);
            if (origin != null) {
                string[] parts = origin.Split(',');
                settings.OriginLatitude = double.Parse(parts[0], CultureInfo.InvariantCulture);
                settings.OriginLongitude = double.Parse(parts[1], CultureInfo.InvariantCulture);
            }
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode mode) {
            if (!enabled) return;
            OmniXController controller = UnityEngine.Object.FindAnyObjectByType<OmniXController>(
                FindObjectsInactive.Exclude);
            Component body = null;
            if (controller != null) {
                body = controller.GetComponent<ArticulationBody>();
                body ??= controller.GetComponentInParent<ArticulationBody>();
                body ??= controller.GetComponent<Rigidbody>();
                body ??= controller.GetComponentInParent<Rigidbody>();
            }
            if (body == null) {
                Debug.LogWarning($"CRANE_ROBOBOAT_CONTRACT_UNAVAILABLE scene={scene.name} " +
                                 "reason=no-enabled-omni-x-body");
                return;
            }
            // sceneLoaded runs before Start, so disabling here keeps them from ever publishing.
            foreach (MonoBehaviour component in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                         FindObjectsInactive.Include)) {
                if (Array.IndexOf(ReplacedSensors, component.GetType().FullName) < 0) continue;
                component.enabled = false;
                Debug.Log($"CRANE_ROBOBOAT_CONTRACT_REPLACED sensor={component.name} " +
                          $"type={component.GetType().Name}");
            }
            var contract = body.GetComponent<CraneRoboBoatSensorContract>() ??
                           body.gameObject.AddComponent<CraneRoboBoatSensorContract>();
            contract.Initialize(body, settings);
        }

        private static string ReadString(string[] args, string key, string fallback) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length && !args[index + 1].StartsWith("--")
                ? args[index + 1] : fallback;
        }

        private static float ReadFloat(string[] args, string key, float fallback) =>
            float.TryParse(ReadString(args, key, null), NumberStyles.Float,
                CultureInfo.InvariantCulture, out float value) ? value : fallback;
    }
}
