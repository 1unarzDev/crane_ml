using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>The medium the body moves in. Only water is implemented; air and ground are
    /// deferred until they have a consumer.</summary>
    public interface IMedium {
        /// <summary>kg/m³.</summary>
        float Density { get; }
        /// <summary>Medium velocity at a Unity world point, Unity world [m/s].</summary>
        Vector3 FlowAt(Vector3 worldPosition, double time);
        /// <summary>Surface height at a Unity world point [m, Unity y].</summary>
        float SurfaceHeight(Vector3 worldPosition);
    }

    /// <summary>A still medium with no surface (tests, or a body with no water).</summary>
    public sealed class StillMedium : IMedium {
        public float Density { get; }
        public StillMedium(float density) => Density = density;
        public Vector3 FlowAt(Vector3 worldPosition, double time) => Vector3.zero;
        public float SurfaceHeight(Vector3 worldPosition) => float.PositiveInfinity;
    }
}
