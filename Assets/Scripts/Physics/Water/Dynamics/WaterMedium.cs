using System;
using Sim.Physics.Core;
using Sim.Physics.Processing;
using Sim.Utils;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace Sim.Physics.Water.Dynamics {
    /// <summary>
    /// The HDRP water surface as an <see cref="IMedium"/>: a uniform current of the surface's large
    /// current speed toward its large orientation (counter-clockwise from Unity +x, as the course's
    /// --crane-water-current sets it), scaled by <see cref="Ramp"/> (seconds since level load →
    /// [0,1]), and the surface height.
    /// </summary>
    public sealed class WaterMedium : IMedium {
        private readonly WaterSurface surface;
        public Func<float, float> Ramp = _ => 1f;

        public WaterMedium(WaterSurface surface) => this.surface = surface;

        public float Density => Constants.waterDensity;

        public Vector3 FlowAt(Vector3 worldPosition, double time) {
            if (surface == null) return Vector3.zero;
            float angle = surface.largeOrientationValue * Mathf.Deg2Rad;
            return surface.largeCurrentSpeedValue * Ramp(Time.timeSinceLevelLoad) *
                   new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
        }

        public float SurfaceHeight(Vector3 worldPosition) => surface == null
            ? float.PositiveInfinity
            : WaterUtils.Search(surface, worldPosition).projectedPositionWS.y;
    }
}
