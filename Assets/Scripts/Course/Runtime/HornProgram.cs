using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Sim.Course {
    /// <summary>The harbor-alert horn (H pp.56–57). Its state is "on:&lt;Hz&gt;" or "off"; truth-side
    /// it goes to /crane/truth/states and states.jsonl, boat-side to /crane/sim/horn
    /// {"on", "freq_hz"} as a stand-in microphone until a real audio sensor exists.</summary>
    public sealed class HornProgram : StatefulCourseObject {
        public const string Topic = "/crane/sim/horn";
        ROSConnection ros;
        public override string Kind => "horn";

        void Start() {
            if (FindAnyObjectByType<ROSConnection>() == null) return;
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<StringMsg>(Topic);
        }

        protected override void Render(string value) {
            if (ros == null) return;
            bool on = value.StartsWith("on:", System.StringComparison.Ordinal);
            string hz = on ? value.Substring(3) : "0";
            ros.Publish(Topic, new StringMsg($"{{\"on\":{(on ? "true" : "false")},\"freq_hz\":{hz}}}"));
        }
    }
}
