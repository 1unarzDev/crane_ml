using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace Sim.Course {
    /// <summary>ROS transport (§4, behind <c>--crane-ros-course-truth</c>): /crane/truth/course
    /// (std_msgs/String JSON) on build and every 2 s with the same sha; /crane/truth/states on
    /// each change plus a 1 Hz snapshot. For the referee, rosbag, agent tools and rviz only;
    /// the rig fails a run whose subscribers are not on Tools/RoboBoat/truth_allowlist.yaml.</summary>
    public sealed class CourseTruthPublisher : MonoBehaviour {
        public const string CourseTopic = "/crane/truth/course", StatesTopic = "/crane/truth/states";
        string courseJson;
        CourseStateRegistry registry;
        ROSConnection ros;
        float nextCourse, nextSnapshot;

        public void Init(JObj truth, CourseStateRegistry states) {
            courseJson = Json.Write(truth);
            registry = states;
        }

        // Start runs after the message registry is ready (see CraneRoboBoatSensorContract).
        void Start() {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<StringMsg>(CourseTopic);
            ros.RegisterPublisher<StringMsg>(StatesTopic);
            registry.StateChanged += (id, value, t) => ros.Publish(StatesTopic, new StringMsg(
                Json.Write(new JObj().Set("t_sim", System.Math.Round(t, 3)).Set("id", id).Set("value", value))));
        }

        void Update() {
            if (ros == null) return;
            if (Time.time >= nextCourse) { ros.Publish(CourseTopic, new StringMsg(courseJson)); nextCourse = Time.time + 2f; }
            if (Time.time >= nextSnapshot) { ros.Publish(StatesTopic, new StringMsg(Json.Write(registry.Snapshot()))); nextSnapshot = Time.time + 1f; }
        }
    }
}
