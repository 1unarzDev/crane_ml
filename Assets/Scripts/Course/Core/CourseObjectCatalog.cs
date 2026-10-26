using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sim.Course {
    /// <summary>Label mode for detections and perception truth (§5): oracle exposes state
    /// colours and numerals in labels; camera does not.</summary>
    public enum LabelMode { Oracle, Camera }

    public enum PlaceholderShape { Cylinder, Box, Panel, Hull }

    /// <summary>Kind ("roboboat/a0_red") → prefab and the data the generator and labels need.
    /// Labels may name the object's state: "{state}_light_buoy", "banner_{state}".</summary>
    [CreateAssetMenu(menuName = "Course/Object Catalog")]
    public sealed class CourseObjectCatalog : ScriptableObject {
        [Serializable]
        public sealed class Entry {
            public string kind;
            [Tooltip("Stream A prefab; when empty the scene template, else a placeholder primitive.")]
            public GameObject prefab;
            public string template = "";
            public string oracleLabel = "";
            public string cameraLabel = "";
            public float radius = 0.2f;
            public float height = 0.3f;
            public string color = "";
            public PlaceholderShape placeholder = PlaceholderShape.Cylinder;
        }

        [SerializeField] List<Entry> entries = new List<Entry>();
        Dictionary<string, Entry> byKind;

        public IReadOnlyList<Entry> Entries => entries;

        public void SetEntries(IEnumerable<Entry> values) {
            entries = new List<Entry>(values);
            byKind = null;
        }

        public Entry Get(string kind) {
            if (byKind == null) {
                byKind = new Dictionary<string, Entry>();
                foreach (var e in entries) byKind[e.kind] = e;
            }
            return byKind.TryGetValue(kind, out var entry) ? entry
                : throw new ArgumentException($"course catalog has no kind {kind}");
        }

        public double Radius(string kind) => Math.Round(Get(kind).radius, 3);
        public double Height(string kind) => Math.Round(Get(kind).height, 3);

        public string Label(string kind, string state, LabelMode mode) {
            Entry e = Get(kind);
            string label = mode == LabelMode.Camera && e.cameraLabel.Length > 0 ? e.cameraLabel : e.oracleLabel;
            return label.Replace("{state}", state ?? "");
        }

        public JObj Describe() {
            var list = new List<object>();
            foreach (var e in entries)
                list.Add(new JObj().Set("kind", e.kind).Set("oracle", e.oracleLabel).Set("camera", e.cameraLabel)
                    .Set("radius", Radius(e.kind)).Set("height", Height(e.kind)));
            return new JObj().Set("entries", list);
        }
    }

    /// <summary>The boat the course is sized for (§1): clearances and path checks use it, and
    /// it is written to truth.</summary>
    public sealed class VesselProfile {
        public double Beam = 0.895, Length = 1.063, TurnRadius = 1.0;
        public string Source = "blastoise-bounds";

        public JObj ToJson() => new JObj().Set("beam", Beam).Set("length", Length)
            .Set("turn_radius", TurnRadius).Set("source", Source);

        public static VesselProfile FromJson(JObj o) => o == null ? new VesselProfile() : new VesselProfile {
            Beam = o.Num("beam"), Length = o.Num("length"), TurnRadius = o.Num("turn_radius"), Source = o.Str("source")
        };
    }
}
