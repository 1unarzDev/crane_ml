using UnityEngine;

namespace Sim.Utils.Viewer {
    /// <summary>
    /// Where the viewer thinks the robot is: a damped position and heading, plus a fraction of
    /// the robot's roll and pitch as a world-space tilt. Shots are composed in this frame.
    /// </summary>
    public readonly struct ViewerFrame {
        public readonly Vector3 Position;
        /// <summary>Heading [deg] about world up; 0 faces world +Z.</summary>
        public readonly float Yaw;
        /// <summary>World-space rotation that rocks a shot with the robot (identity = level).</summary>
        public readonly Quaternion Tilt;

        public ViewerFrame(Vector3 position, float yaw, Quaternion tilt) {
            Position = position;
            Yaw = yaw;
            Tilt = tilt;
        }

        public Quaternion Heading => Quaternion.Euler(0f, Yaw, 0f);
    }

    /// <summary>
    /// Follow math. Every damper is first-order exponential with a time constant, so it never
    /// overshoots a step, converges to 1/e of the error per time constant, and gives the same
    /// result for one long step as for many short ones (frame-rate independent).
    /// </summary>
    public static class ViewerFollow {
        /// <summary>Fraction of the remaining error removed over <paramref name="dt"/>.</summary>
        public static float Alpha(float timeConstant, float dt) =>
            timeConstant <= 0f ? 1f : 1f - Mathf.Exp(-Mathf.Max(0f, dt) / timeConstant);

        public static float Damp(float current, float target, float timeConstant, float dt) =>
            current + (target - current) * Alpha(timeConstant, dt);

        public static Vector3 Damp(Vector3 current, Vector3 target, float timeConstant, float dt) =>
            current + (target - current) * Alpha(timeConstant, dt);

        /// <summary>Damps along the shortest arc, so 350° → 10° passes through 0°.</summary>
        public static float DampAngle(float current, float target, float timeConstant, float dt) =>
            current + Mathf.DeltaAngle(current, target) * Alpha(timeConstant, dt);

        public static Quaternion Damp(Quaternion current, Quaternion target, float timeConstant, float dt) =>
            Quaternion.Slerp(current, target, Alpha(timeConstant, dt));

        /// <summary>
        /// Splits a robot attitude (+Z = its forward, +Y = its up) into heading [deg] and the
        /// world-space tilt scaled by <paramref name="fraction"/>: 0 keeps the shot level,
        /// 1 rocks it exactly with the robot.
        /// </summary>
        public static Quaternion PartialTilt(Quaternion attitude, float fraction, out float yaw) {
            Vector3 forward = attitude * Vector3.forward;
            Vector3 flat = new Vector3(forward.x, 0f, forward.z);
            // Pointing straight up or down has no heading; keep the attitude's own yaw.
            yaw = flat.sqrMagnitude > 1e-8f ? Quaternion.LookRotation(flat).eulerAngles.y : attitude.eulerAngles.y;
            Quaternion heading = Quaternion.Euler(0f, yaw, 0f);
            Quaternion local = Quaternion.Inverse(heading) * attitude;
            return heading * Scale(local, Mathf.Clamp01(fraction)) * Quaternion.Inverse(heading);
        }

        /// <summary>The rotation about the same axis by <paramref name="fraction"/> of the angle
        /// (exact; Quaternion.Slerp from identity is off by about 0.3 % at small angles).</summary>
        public static Quaternion Scale(Quaternion rotation, float fraction) {
            if (rotation.w < 0f) rotation = new Quaternion(-rotation.x, -rotation.y, -rotation.z, -rotation.w);
            Vector3 axis = new Vector3(rotation.x, rotation.y, rotation.z);
            float sinHalf = axis.magnitude;
            if (sinHalf < 1e-9f) return Quaternion.identity;
            float half = Mathf.Atan2(sinHalf, rotation.w) * fraction;
            axis *= Mathf.Sin(half) / sinHalf;
            return new Quaternion(axis.x, axis.y, axis.z, Mathf.Cos(half));
        }

        /// <summary>The frame sitting exactly on the robot.</summary>
        public static ViewerFrame Snap(Vector3 position, Quaternion attitude, float rollFraction) {
            Quaternion tilt = PartialTilt(attitude, rollFraction, out float yaw);
            return new ViewerFrame(position, yaw, tilt);
        }

        /// <summary>One damped step of <paramref name="frame"/> toward the robot.</summary>
        public static ViewerFrame Step(in ViewerFrame frame, Vector3 position, Quaternion attitude,
            IViewerFollowSettings settings, float dt) {
            Quaternion tilt = PartialTilt(attitude, settings.RollFraction, out float yaw);
            return new ViewerFrame(
                Damp(frame.Position, position, settings.PositionTime, dt),
                DampAngle(frame.Yaw, yaw, settings.YawTime, dt),
                Damp(frame.Tilt, tilt, settings.TiltTime, dt));
        }

        /// <summary>Zero velocity at both ends and monotonic in between: a transition never
        /// passes its endpoints.</summary>
        public static float Ease(float t) {
            t = Mathf.Clamp01(t);
            return t * t * (3f - 2f * t);
        }
    }

    /// <summary>The follow settings <see cref="ViewerFollow.Step"/> needs.</summary>
    public interface IViewerFollowSettings {
        /// <summary>Position time constant [s]; 0 is rigid.</summary>
        float PositionTime { get; }
        /// <summary>Heading time constant [s]; 0 is rigid.</summary>
        float YawTime { get; }
        /// <summary>Roll/pitch time constant [s]; 0 is rigid.</summary>
        float TiltTime { get; }
        /// <summary>Fraction [0, 1] of the robot's roll and pitch the shot inherits.</summary>
        float RollFraction { get; }
    }
}
