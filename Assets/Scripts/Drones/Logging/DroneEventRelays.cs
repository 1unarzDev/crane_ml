using System;
using System.Collections.Generic;
using UnityEngine;

namespace Crane.Drones.Logging {
    /// <summary>Put on the drone body (the Rigidbody/collider owner). Forwards contacts to a callback.</summary>
    [ExecuteAlways] // contact messages must also fire when physics is stepped from the Editor (EditMode tests)
    public sealed class DroneCollisionRelay : MonoBehaviour {
        public Action<string, double, Vector3> OnContact; // other name, impact speed (m/s), contact normal (Unity world)
        private void OnCollisionEnter(Collision c) {
            Vector3 n = c.contactCount > 0 ? c.GetContact(0).normal : Vector3.zero;
            OnContact?.Invoke(c.gameObject.name, c.relativeVelocity.magnitude, n);
        }
    }

    /// <summary>Marks a search target the detector should report when it is visible in a camera frame.</summary>
    [ExecuteAlways] // OnEnable registration must also run in EditMode
    public sealed class DroneDetectableTarget : MonoBehaviour {
        public string targetId = "target";
        public static readonly List<DroneDetectableTarget> All = new();
        private void OnEnable() => All.Add(this);
        private void OnDisable() => All.Remove(this);
    }

    /// <summary>
    /// Ground-truth visibility: a target counts as detected when its centre is inside the camera frustum
    /// (with margin) within range and a line of sight is not blocked. Pure geometry: no ML.
    /// </summary>
    public static class DroneDetectionProbe {
        public static void Detect(Camera cam, double maxRange, int layerMask, List<Dictionary<string, object>> into) {
            into.Clear();
            foreach (var t in DroneDetectableTarget.All) {
                Vector3 vp = cam.WorldToViewportPoint(t.transform.position);
                if (vp.z <= 0 || vp.x < 0 || vp.x > 1 || vp.y < 0 || vp.y > 1 || vp.z > maxRange) continue;
                Vector3 o = cam.transform.position, d = t.transform.position - o;
                const float skip = 0.3f; // start outside the carrier's own collider
                if (d.magnitude > skip && Physics.Raycast(o + d.normalized * skip, d.normalized, out RaycastHit hit, d.magnitude - skip - 0.05f, layerMask,
                        QueryTriggerInteraction.Ignore)) continue;
                into.Add(new Dictionary<string, object> {
                    ["target"] = t.targetId, ["camera"] = cam.name, ["range_m"] = (double)d.magnitude,
                    ["u"] = (double)vp.x, ["v"] = (double)vp.y });
            }
        }
    }
}
