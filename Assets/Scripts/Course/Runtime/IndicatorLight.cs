using UnityEngine;

namespace Sim.Course {
    public enum IndicatorColor { Off, Red, Green }

    /// <summary>The red/green colour indicator (L Dec 16): by default the non-emissive printed
    /// cylinder, recoloured on the anchor's slot; the emissive variant (H p.58 LED tube) is kept
    /// for older layouts. Serves the speed marker, the debris beacons and the slip indicators.</summary>
    public sealed class IndicatorLight : StatefulCourseObject {
        [SerializeField] bool emissive;
        public override string Kind => "indicator";
        public bool Emissive { get => emissive; set => emissive = value; }

        public static Color ColorOf(string value) =>
            value == "red" ? new Color(0.85f, 0.08f, 0.08f) : value == "green" ? new Color(0.08f, 0.7f, 0.2f) : new Color(0.15f, 0.15f, 0.15f);

        protected override void Render(string value) => Tint(Slot(), ColorOf(value), emissive);
    }
}
