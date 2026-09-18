using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace Sim.Utils.ROS {
    [Serializable]
    public class ROSSubscriber : MonoBehaviour {
        public string topicName;

        public void Initialize<T>(string topicName, Action<T> callback)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            this.topicName = topicName;
            if (ROSPublisher.TransportSuppressed) return;
            ROSConnection.GetOrCreateInstance().Subscribe(topicName, callback);
        }

        public void Initialize<T>(Action<T> callback)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            if (topicName == null) { Debug.LogError("No topic name set"); return; }
            if (ROSPublisher.TransportSuppressed) return;
            ROSConnection.GetOrCreateInstance().Subscribe(topicName, callback);
        }
    }
}
