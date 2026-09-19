using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sim.Utils.ReferenceEnvironments {
    /// <summary>
    /// Render-only evidence overlay keyed by stable environment semantic IDs. This deliberately
    /// does not enable, disable, add, or modify colliders, rigid bodies, or sensor layers.
    /// </summary>
    public sealed class CraneSemanticEvidenceHighlighter : MonoBehaviour {
        [SerializeField] private Color highlightColor = new(1f, 0.55f, 0.04f, 1f);

        private readonly Dictionary<Renderer, MaterialPropertyBlock> originalBlocks = new();
        private MaterialPropertyBlock workingBlock;

        private void Awake() {
            // MaterialPropertyBlock allocates a Unity native object and therefore cannot be
            // constructed by a MonoBehaviour field initializer.
            workingBlock = new MaterialPropertyBlock();
        }

        public int Highlight(IEnumerable<string> semanticIds) {
            Clear();
            if (semanticIds == null) return 0;
            var requested = new HashSet<string>(semanticIds, StringComparer.Ordinal);
            requested.RemoveWhere(string.IsNullOrWhiteSpace);
            if (requested.Count == 0) return 0;

            int highlighted = 0;
            foreach (Renderer renderer in FindObjectsByType<Renderer>(
                         FindObjectsInactive.Include, FindObjectsSortMode.None)) {
                string semanticId = ResolveSemanticId(renderer.transform, requested);
                if (semanticId == null) continue;

                var original = new MaterialPropertyBlock();
                renderer.GetPropertyBlock(original);
                originalBlocks.Add(renderer, original);

                renderer.GetPropertyBlock(workingBlock);
                workingBlock.SetColor("_BaseColor", highlightColor);
                workingBlock.SetColor("_Color", highlightColor);
                workingBlock.SetColor("_EmissiveColor", highlightColor);
                renderer.SetPropertyBlock(workingBlock);
                workingBlock.Clear();
                highlighted++;
            }
            return highlighted;
        }

        public void Clear() {
            foreach (KeyValuePair<Renderer, MaterialPropertyBlock> entry in originalBlocks) {
                if (entry.Key != null) entry.Key.SetPropertyBlock(entry.Value);
            }
            originalBlocks.Clear();
        }

        private static string ResolveSemanticId(Transform value, HashSet<string> requested) {
            for (Transform current = value; current != null; current = current.parent) {
                CraneSemanticIdentity identity = current.GetComponent<CraneSemanticIdentity>();
                if (identity != null && requested.Contains(identity.SemanticId))
                    return identity.SemanticId;
            }

            // Primitive reference scenes keep canonical collision and presentation as siblings.
            // Their visual names are deterministically derived from the canonical semantic ID.
            string name = value.name;
            const string suffix = "-visual";
            if (name.EndsWith(suffix, StringComparison.Ordinal))
                name = name[..^suffix.Length];
            return requested.Contains(name) ? name : null;
        }

        private void OnDisable() => Clear();
    }
}
