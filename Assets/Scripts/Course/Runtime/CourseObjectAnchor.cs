using System;
using UnityEngine;

namespace Sim.Course {
    /// <summary>On every course prefab root (stream A; design Appendix B): footprint, waterline
    /// and named mounts, so the builder places any prefab, or the placeholder primitive standing
    /// in for one, the same way.</summary>
    public sealed class CourseObjectAnchor : MonoBehaviour {
        [Serializable]
        public struct Mount {
            public string name;
            public Transform transform;
        }

        [SerializeField, Tooltip("Horizontal radius the generator keeps clear, metres.")] float footprintRadius = 0.2f;
        [SerializeField, Tooltip("Height of the root above the waterline, metres.")] float waterlineOffset;
        [SerializeField, Tooltip("Renderer recoloured by the object's state (lights, signs).")] Renderer stateSlot;
        [SerializeField] Mount[] mounts = Array.Empty<Mount>();
        [SerializeField, Tooltip("True on placeholder primitives the builder made; stream A's prefabs replace them.")] bool placeholder;

        public float FootprintRadius { get => footprintRadius; set => footprintRadius = value; }
        public float WaterlineOffset { get => waterlineOffset; set => waterlineOffset = value; }
        public Renderer StateSlot { get => stateSlot; set => stateSlot = value; }
        public bool Placeholder { get => placeholder; set => placeholder = value; }

        public Transform MountPoint(string mountName) {
            foreach (var m in mounts) if (m.name == mountName) return m.transform;
            return null;
        }
    }
}
