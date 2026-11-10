// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using UnityEngine;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Sensor ray world backed by the scene's PhysX colliders (Drone Forest: terrain, trunk/branch MeshColliders, logs,
    /// rocks on layer Environment(9); foliage cards have no colliders). Frames: ENU (x, y, z) = Unity (x, z, y).
    /// The struck structure's diameter (thin-structure dropout) is looked up from the manifest capsule nearest to the hit
    /// point (&lt;= 10 cm); terrain and unmatched colliders count as large. Beam footprint: a sphere cast of radius beam x BeamRefRange (the analytic world grows it with range).
    /// Seeing through a dropped structure re-casts from just beyond it.
    /// </summary>
    public sealed class UnityColliderRayWorld : IRayWorld {
        public int LayerMask = 1 << 9;
        /// <summary>Range [m] at which the decimated-pixel footprint sets the sphere-cast radius (beam * BeamRefRange).</summary>
        public double BeamRefRange = 4.0;
        private readonly ForestManifestWorld manifest;
        public long Casts { get; private set; }

        public UnityColliderRayWorld(ForestManifestWorld manifestForIds) { manifest = manifestForIds; }

        public static Vector3 ToUnity(Vec3 p) => new Vector3((float)p.X, (float)p.Z, (float)p.Y);

        public bool Raycast(Vec3 o, Vec3 d, double maxRange, double beam, int ignore, out RayHit hit) {
            hit = new RayHit { Distance = double.PositiveInfinity, ObjectId = -2, Diameter = double.PositiveInfinity };
            var uo = ToUnity(o); var ud = ToUnity(d);
            double travelled = 0;
            for (int pass = 0; pass < 3; pass++) {
                Casts++;
                RaycastHit h; float len = (float)(maxRange - travelled);
                // beam footprint: a sphere of the footprint at BeamRefRange (decimated depth pixels); ideal ray otherwise
                bool any = beam > 0 ? Physics.SphereCast(uo, (float)(beam * BeamRefRange), ud, out h, len, LayerMask, QueryTriggerInteraction.Ignore)
                                    : Physics.Raycast(uo, ud, out h, len, LayerMask, QueryTriggerInteraction.Ignore);
                if (!any) return false;
                double dist = travelled + h.distance;
                int id = -1; double diam = double.PositiveInfinity;
                if (!(h.collider is TerrainCollider) && manifest != null) {
                    var p = new Vec3(h.point.x, h.point.z, h.point.y);
                    double c = manifest.Clearance(p, 0.2, false, out int cid, out double r);
                    if (cid >= 0 && c < 0.10) { id = cid; diam = 2.0 * r; }
                }
                if (id >= 0 && id == ignore) {   // the dropped structure itself: continue past it
                    double skip = Math.Min(Math.Max(diam, 0.01) + 0.01, 0.5);
                    travelled = dist + skip; uo = h.point + ud * (float)skip;
                    if (travelled >= maxRange) return false;
                    continue;
                }
                hit.Distance = dist; hit.ObjectId = id; hit.Diameter = diam;
                return true;
            }
            return false;
        }

        /// <summary>Terrain height (ENU z) at ENU (x, y) from the active Unity terrain, NaN if none covers it.</summary>
        public static double TerrainHeight(double x, double y) {
            var t = Terrain.activeTerrain;
            if (t == null) return double.NaN;
            var w = new Vector3((float)x, 0, (float)y);
            return t.SampleHeight(w) + t.transform.position.y;
        }
    }
}
