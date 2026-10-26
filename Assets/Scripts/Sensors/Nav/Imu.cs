using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Sim.Utils;
using Sim.Utils.ROS;

namespace Sim.Sensors.Nav {
    public class Imu : MonoBehaviour, IROSSensor<ImuMsg> {
        [SerializeField] private string topicName = "imu/raw";
        [SerializeField] private string frameId = "imu_link";
        [SerializeField] private float Hz = 50.0f;
        public ROSPublisher publisher { get; set; }

        public IPhysicsBody body;

        private void OnValidate() {
            if (GetComponent<Rigidbody>() == null && GetComponent<ArticulationBody>() == null)
                Debug.LogWarning($"{name} should have either a Rigidbody or an ArticulationBody attached.");
        }

        private void Awake() {
            var rb = GetComponent<Rigidbody>();
            var ab = GetComponent<ArticulationBody>();

            if (rb != null) body = new RigidbodyAdapter(rb);
            else if (ab != null) body = new ArticulationBodyAdapter(ab);
            else throw new MissingComponentException($"{name} requires a Rigidbody or ArticulationBody!");

            publisher = gameObject.AddComponent<ROSPublisher>();
        }

        public ImuMsg CreateMessage() {
            return new ImuMsg {
                orientation = body.transform.rotation.To<FLU>(),
                angular_velocity = AngularVelocity(body.transform.InverseTransformDirection(body.angularVelocity)),
                header = publisher.CreateHeader()
            };
        }

        /// <summary>Body-local rate in REP-103. Angular velocity is axial, so the Unity-to-FLU
        /// reflection flips its sign: the shared axial map -FLU(w), not the polar To&lt;FLU&gt;().</summary>
        public static Vector3Msg AngularVelocity(Vector3 bodyLocal) {
            Vector3 flu = FLU.ConvertAngularVelocityFromRUF(bodyLocal);
            return new Vector3Msg(flu.x, flu.y, flu.z);
        }

        private void Start() {
            publisher.Initialize(topicName, frameId, CreateMessage, Hz);
        }
    }
}
