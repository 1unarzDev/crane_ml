using System;
using UnityEngine;

namespace Sim.Course {
    /// <summary>
    /// The robot's side of <see cref="CourseBootstrap"/>, so the generic course names no robot:
    /// where the robot starts, the water current, what to add once the course is built, and the
    /// robot's own spellings of course flags, spec files and log lines that its tools still use.
    /// A robot assembly sets <see cref="CourseBootstrap.Host"/> from a
    /// <c>[RuntimeInitializeOnLoadMethod(AfterAssembliesLoaded)]</c>. With none, the course is laid
    /// out from the world origin with no current.
    /// </summary>
    public interface ICourseHost {
        /// <summary>The robot's alias for a course flag such as <c>--crane-course-truth</c>, or null.</summary>
        string Alias(string flag);

        /// <summary>The robot's start pose in world-ROS (/crane/odom); false while the scene has
        /// no robot yet (a bootstrap scene).</summary>
        bool Start(out Vector2 position, out float yaw);

        /// <summary>The water current now: speed (m/s) and direction (rad, counter-clockwise
        /// from world-ROS x).</summary>
        (float speed, float toward) Current();

        /// <summary>Builds a course spec that names no family (a format of the robot's own);
        /// false while the scene has no robot yet.</summary>
        bool BuildSpec(string path, string[] args);

        /// <summary>The course is built (and its READY line logged) from this start pose.</summary>
        void Built(Transform root, JObj truth, Vector2 start, float yaw, string truthPath);

        /// <summary>Building the course threw.</summary>
        void Failed(Exception error);
    }
}
