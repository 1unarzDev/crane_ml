using System;
using UnityEngine;

namespace Sim.Utils.ReferenceEnvironments {
    /// <summary>
    /// Stable identity attached to canonical environment objects. Explanation evidence refers to
    /// this identifier, never to a renderer name or an instance ID that changes between runs.
    /// </summary>
    public sealed class CraneSemanticIdentity : MonoBehaviour {
        [SerializeField] private string semanticId;
        [SerializeField] private string semanticRole;
        [SerializeField] private string environmentId;

        public string SemanticId => semanticId;
        public string SemanticRole => semanticRole;
        public string EnvironmentId => environmentId;

        public void Configure(string id, string role, string environment) {
            if (string.IsNullOrWhiteSpace(id))
                throw new ArgumentException("Semantic ID must be non-empty.", nameof(id));
            semanticId = id;
            semanticRole = role ?? string.Empty;
            environmentId = environment ?? string.Empty;
        }
    }
}
