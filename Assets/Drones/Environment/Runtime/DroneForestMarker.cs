using UnityEngine;

namespace Crane.Drones.Environment
{
    /// <summary>Editor gizmo/label for spawn point, corridor waypoints and camera rigs.</summary>
    public class DroneForestMarker : MonoBehaviour
    {
        public enum Kind { Spawn, CorridorWaypoint, CameraRig, Target }
        public Kind kind;
        public float corridorHalfWidth = 2f;

        void OnDrawGizmos()
        {
            switch (kind)
            {
                case Kind.Spawn: Gizmos.color = Color.green; Gizmos.DrawWireSphere(transform.position + Vector3.up * 0.2f, 0.4f); Gizmos.DrawRay(transform.position, transform.forward * 2f); break;
                case Kind.CorridorWaypoint: Gizmos.color = Color.cyan; Gizmos.DrawWireSphere(transform.position, corridorHalfWidth); break;
                case Kind.CameraRig: Gizmos.color = Color.yellow; Gizmos.DrawFrustum(transform.position, 60f, 5f, 0.3f, 16f / 9f); break;
                case Kind.Target: Gizmos.color = Color.red; Gizmos.DrawWireCube(transform.position + Vector3.up * 0.5f, new Vector3(1.8f, 1f, 0.8f)); break;
            }
        }
    }
}
