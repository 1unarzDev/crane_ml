using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace Sim.Utils.ROS {
    /// <summary>One-shot latched std_msgs/String, for facts a late subscriber must still see
    /// (the resolved simulation profile). A no-op when ROS transport is suppressed.</summary>
    public static class ROSLatchedString {
        /// <summary>False when the message registry is not ready yet; call again next frame.</summary>
        public static bool TryPublish(string topic, string data) {
            if (ROSPublisher.TransportSuppressed) return true;
            if (MessageRegistry.GetRosMessageName<StringMsg>() == null) return false;
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<StringMsg>(topic, 1, true);
            ros.Publish(topic, new StringMsg(data));
            return true;
        }
    }
}
