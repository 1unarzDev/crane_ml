using System;
using UnityEngine;

namespace Sim.Course {
    /// <summary>A course object whose state the registry drives from its program (§5): a
    /// light's colour, a sign's face, the horn. The state is a string ("red", "2", "on:800").</summary>
    public interface IStatefulCourseObject {
        string Id { get; }
        string Kind { get; }
        string State { get; }
        event Action<IStatefulCourseObject> Changed;
        void Apply(string value);
    }

    /// <summary>Shared plumbing: Apply sets the state, renders it and raises Changed.</summary>
    public abstract class StatefulCourseObject : MonoBehaviour, IStatefulCourseObject {
        [SerializeField] string id = "";
        [SerializeField] string state = "";
        public string Id { get => id; set => id = value; }
        public abstract string Kind { get; }
        public string State => state;
        public event Action<IStatefulCourseObject> Changed;

        public void Apply(string value) {
            if (value == state) return;
            state = value;
            Render(value);
            Changed?.Invoke(this);
        }

        protected abstract void Render(string value);

        /// <summary>The renderer that shows the state: the anchor's slot, else the first one.</summary>
        protected Renderer Slot() {
            var anchor = GetComponent<CourseObjectAnchor>();
            return anchor != null && anchor.StateSlot != null ? anchor.StateSlot : GetComponentInChildren<Renderer>();
        }

        protected static void Tint(Renderer r, Color color, bool emissive) {
            if (r == null) return;
            var block = new MaterialPropertyBlock();
            r.GetPropertyBlock(block);
            block.SetColor("_BaseColor", color);
            block.SetColor("_Color", color);
            if (emissive) block.SetColor("_EmissiveColor", color * 4f);
            r.SetPropertyBlock(block);
        }
    }
}
