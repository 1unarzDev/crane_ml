using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace Sim.Utils.ROS {
    [Serializable]
    public class ROSSubscriber : MonoBehaviour {
        public string topicName;
        private Action subscribeAfterRuntimeInitialization;

        public void Initialize<T>(string topicName, Action<T> callback)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            this.topicName = topicName;
            if (ROSPublisher.TransportSuppressed) return;
            SubscribeWhenReady(topicName, callback);
        }

        public void Initialize<T>(Action<T> callback)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            if (topicName == null) { Debug.LogError("No topic name set"); return; }
            if (ROSPublisher.TransportSuppressed) return;
            SubscribeWhenReady(topicName, callback);
        }

        private void SubscribeWhenReady<T>(string topic, Action<T> callback)
        where T : Unity.Robotics.ROSTCPConnector.MessageGeneration.Message {
            if (Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageRegistry
                    .GetRosMessageName<T>() == null)
                subscribeAfterRuntimeInitialization = () =>
                    ROSConnection.GetOrCreateInstance().Subscribe(topic, callback);
            else ROSConnection.GetOrCreateInstance().Subscribe(topic, callback);
        }

        private void Start() {
            subscribeAfterRuntimeInitialization?.Invoke();
            subscribeAfterRuntimeInitialization = null;
        }
    }
}
