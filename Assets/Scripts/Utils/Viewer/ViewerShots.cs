using UnityEngine;

namespace Sim.Utils.Viewer {
    /// <summary>
    /// A camera described by what it looks at: a focus point, a look yaw and pitch [deg], a
    /// distance [m] and a tilt about the focus. Shots interpolate in these terms, so a
    /// transition swings around the subject instead of cutting across it.
    /// </summary>
    public struct ViewerOrbit {
        public Vector3 Focus;
        public float Yaw;
        /// <summary>Positive looks down.</summary>
        public float Pitch;
        public float Distance;
        public Quaternion Tilt;

        public ViewerOrbit(Vector3 focus, float yaw, float pitch, float distance, Quaternion tilt) {
            Focus = focus;
            Yaw = yaw;
            Pitch = pitch;
            Distance = distance;
            Tilt = tilt;
        }

        public Quaternion Rotation => Tilt * Quaternion.Euler(Pitch, Yaw, 0f);

        public Pose ToPose() {
            Quaternion rotation = Rotation;
            return new Pose(Focus - rotation * Vector3.forward * Distance, rotation);
        }

        /// <summary>Distance is interpolated in log space so a zoom feels uniform.</summary>
        public static ViewerOrbit Lerp(in ViewerOrbit a, in ViewerOrbit b, float t) => new(
            Vector3.Lerp(a.Focus, b.Focus, t),
            Mathf.LerpAngle(a.Yaw, b.Yaw, t),
            Mathf.Lerp(a.Pitch, b.Pitch, t),
            Mathf.Exp(Mathf.Lerp(Mathf.Log(Mathf.Max(a.Distance, 1e-3f)), Mathf.Log(Mathf.Max(b.Distance, 1e-3f)), t)),
            Quaternion.Slerp(a.Tilt, b.Tilt, t));

        /// <summary>Yaw, pitch and distance of a camera at <paramref name="cameraFromFocus"/>.</summary>
        public static void Angles(Vector3 cameraFromFocus, out float yaw, out float pitch, out float distance) {
            distance = cameraFromFocus.magnitude;
            Vector3 look = -cameraFromFocus / Mathf.Max(distance, 1e-6f);
            yaw = Mathf.Atan2(look.x, look.z) * Mathf.Rad2Deg;
            pitch = -Mathf.Asin(Mathf.Clamp(look.y, -1f, 1f)) * Mathf.Rad2Deg;
        }
    }

    /// <summary>Everything a shot may look at this frame.</summary>
    public readonly struct ViewerContext {
        public readonly ViewerFrame Frame;
        /// <summary>Vertical field of view [deg].</summary>
        public readonly float FieldOfView;
        public readonly float Aspect;
        /// <summary>Named world areas (course tasks), may be empty.</summary>
        public readonly IViewerAreas Areas;

        public ViewerContext(in ViewerFrame frame, float fieldOfView, float aspect, IViewerAreas areas) {
            Frame = frame;
            FieldOfView = fieldOfView;
            Aspect = aspect;
            Areas = areas;
        }
    }

    /// <summary>A camera placement, evaluated every frame against the moving robot.</summary>
    public abstract class ViewerShot {
        public abstract ViewerOrbit Evaluate(in ViewerContext context);

        /// <summary>Advances time-dependent shots; returns the shot to use from now on.</summary>
        public virtual ViewerShot Advance(float dt) => this;
    }

    /// <summary>
    /// A view held relative to the followed frame: <see cref="Yaw"/> is where the camera looks
    /// relative to the robot's heading (0 = chase from behind, 90 = from its left looking
    /// across, 180 = from ahead). With <see cref="WorldYaw"/> the yaw and focus offset are
    /// world-aligned instead (a map-up overhead). The shot rocks with the frame's tilt.
    /// </summary>
    public sealed class FollowShot : ViewerShot {
        public readonly float Yaw, Pitch, Distance;
        public readonly Vector3 FocusOffset;
        public readonly bool WorldYaw;

        public FollowShot(float yaw, float pitch, float distance, Vector3 focusOffset, bool worldYaw = false) {
            Yaw = yaw;
            Pitch = pitch;
            Distance = distance;
            FocusOffset = focusOffset;
            WorldYaw = worldYaw;
        }

        /// <summary>A shot that puts the camera at <paramref name="cameraFromFocus"/>.</summary>
        public static FollowShot Looking(Vector3 focusOffset, Vector3 cameraFromFocus, bool worldYaw) {
            ViewerOrbit.Angles(cameraFromFocus, out float yaw, out float pitch, out float distance);
            return new FollowShot(yaw, pitch, distance, focusOffset, worldYaw);
        }

        public override ViewerOrbit Evaluate(in ViewerContext context) {
            ViewerFrame frame = context.Frame;
            Quaternion axes = WorldYaw ? Quaternion.identity : frame.Heading;
            return new ViewerOrbit(frame.Position + frame.Tilt * (axes * FocusOffset),
                (WorldYaw ? 0f : frame.Yaw) + Yaw, Pitch, Distance, frame.Tilt);
        }
    }

    /// <summary>
    /// Frames a world area: the whole course, or the task area nearest the robot. Without any
    /// area it falls back to <see cref="Fallback"/>.
    /// </summary>
    public sealed class AreaShot : ViewerShot {
        public readonly bool NearestOnly;
        public readonly float Yaw, Pitch, Margin;
        public readonly ViewerShot Fallback;

        public AreaShot(bool nearestOnly, float yaw, float pitch, float margin, ViewerShot fallback) {
            NearestOnly = nearestOnly;
            Yaw = yaw;
            Pitch = pitch;
            Margin = margin;
            Fallback = fallback;
        }

        public override ViewerOrbit Evaluate(in ViewerContext context) {
            Bounds area = default;
            bool found = context.Areas != null && (NearestOnly
                ? context.Areas.TryNearest(context.Frame.Position, out area)
                : context.Areas.TryAll(out area));
            if (!found) return Fallback.Evaluate(context);
            return new ViewerOrbit(area.center, Yaw, Pitch,
                FramingDistance(area.extents.magnitude * Margin, context.FieldOfView, context.Aspect),
                Quaternion.identity);
        }

        /// <summary>Distance [m] at which a sphere of <paramref name="radius"/> fits the view.</summary>
        public static float FramingDistance(float radius, float verticalFov, float aspect) {
            float halfV = 0.5f * Mathf.Clamp(verticalFov, 1f, 179f) * Mathf.Deg2Rad;
            float halfH = Mathf.Atan(Mathf.Tan(halfV) * Mathf.Max(aspect, 1e-3f));
            return Mathf.Max(radius, 0.5f) / Mathf.Sin(Mathf.Min(halfV, halfH));
        }
    }

    /// <summary>Free-orbit offsets layered on a shot: yaw and pitch [deg] and a log zoom.</summary>
    public sealed class OffsetShot : ViewerShot {
        private ViewerShot inner;
        private readonly float yaw, pitch, zoom, minPitch, maxPitch;

        public OffsetShot(ViewerShot inner, float yaw, float pitch, float zoom, float minPitch, float maxPitch) {
            this.inner = inner;
            this.yaw = yaw;
            this.pitch = pitch;
            this.zoom = zoom;
            this.minPitch = minPitch;
            this.maxPitch = maxPitch;
        }

        public static ViewerOrbit Apply(ViewerOrbit orbit, float yaw, float pitch, float zoom,
            float minPitch, float maxPitch) {
            orbit.Yaw += yaw;
            orbit.Pitch = Mathf.Clamp(orbit.Pitch + pitch, minPitch, maxPitch);
            orbit.Distance *= Mathf.Exp(zoom);
            return orbit;
        }

        public override ViewerOrbit Evaluate(in ViewerContext context) =>
            Apply(inner.Evaluate(context), yaw, pitch, zoom, minPitch, maxPitch);

        public override ViewerShot Advance(float dt) {
            inner = inner.Advance(dt);
            return this;
        }
    }

    /// <summary>
    /// An eased transition. Both ends are evaluated live, so the camera keeps tracking the robot
    /// while it moves; a new transition started mid-way blends from this one, so nothing jumps.
    /// </summary>
    public sealed class ViewerBlend : ViewerShot {
        public ViewerShot From { get; private set; }
        public ViewerShot To { get; }
        public float Duration { get; }
        public float Elapsed { get; private set; }

        public ViewerBlend(ViewerShot from, ViewerShot to, float duration) {
            From = from;
            To = to;
            Duration = duration;
        }

        public float Progress => Duration <= 0f ? 1f : ViewerFollow.Ease(Elapsed / Duration);

        public override ViewerOrbit Evaluate(in ViewerContext context) =>
            ViewerOrbit.Lerp(From.Evaluate(context), To.Evaluate(context), Progress);

        public override ViewerShot Advance(float dt) {
            Elapsed += Mathf.Max(0f, dt);
            if (Elapsed >= Duration) return To.Advance(0f);
            From = From.Advance(dt);
            return this;
        }
    }
}
