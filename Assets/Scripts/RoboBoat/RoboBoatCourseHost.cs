using System;
using Sim.Controllers;
using UnityEngine;

namespace Sim.Course.RoboBoat {
    /// <summary>
    /// The RoboBoat side of the generic course bootstrap: the scene boat's start pose, the
    /// <c>--crane-water-current</c>, the sim-only /crane/reset_to_start, the 2025 element-list
    /// spec (<see cref="JsonCourseSource"/>), and the flag and log spellings the RoboBoat tools
    /// still use (<c>--crane-roboboat-course[-truth]</c>, <c>CRANE_ROBOBOAT_COURSE_READY/FAILED</c>).
    /// </summary>
    public sealed class RoboBoatCourseHost : ICourseHost {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterAssembliesLoaded)]
        static void Register() => CourseBootstrap.Host = new RoboBoatCourseHost();

        public string Alias(string flag) => flag switch {
            "--crane-course-spec" => JsonCourseSource.Flag,
            "--crane-course-truth" => "--crane-roboboat-course-truth",
            _ => null
        };

        public bool Start(out Vector2 position, out float yaw) => RoboBoatCourse.BoatStart(out position, out yaw);

        public (float speed, float toward) Current() {
            RoboBoatCourse.ApplyCurrentFromArgs();
            return RoboBoatCourse.CurrentNow();
        }

        public bool BuildSpec(string path, string[] args) {
            RoboBoatCourse.ApplyCurrentFromArgs();
            if (!Start(out Vector2 start, out float yaw)) return false;
            JsonCourseSource.Build(path, args);  // laid out from this same start
            AddStartReset(null, start, yaw);
            return true;
        }

        public void Built(Transform root, JObj truth, Vector2 start, float yaw, string truthPath) {
            AddStartReset(root, start, yaw);
            Debug.Log($"CRANE_ROBOBOAT_COURSE_READY name={truth.Str("name")} elements={truth.Arr("elements").Count} " +
                      $"bays={truth.Arr("bays").Count} start=({start.x:R},{start.y:R},{yaw:R}) truth={truthPath ?? "none"}");
        }

        public void Failed(Exception error) => Debug.LogError($"CRANE_ROBOBOAT_COURSE_FAILED {error.Message}");

        /// <summary>The sim-only /crane/reset_to_start, on the course truth start pose.</summary>
        static void AddStartReset(Transform root, Vector2 start, float yaw) {
            var host = new GameObject("Course Start Reset");
            if (root != null) host.transform.SetParent(root, false);
            host.AddComponent<RoboBoatStartReset>().Capture(RoboBoatStartReset.FindBoatBody(), start, yaw);
        }
    }
}
