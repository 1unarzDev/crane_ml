using UnityEngine;

namespace Sim.Utils.ReferenceEnvironments {
    /// <summary>Runtime provenance marker for an editor-generated offline SDF import.</summary>
    public sealed class CraneImportedReferenceEnvironment : MonoBehaviour {
        [SerializeField] private string environmentId;
        [SerializeField] private string manifestSha256;
        [SerializeField] private string sourceVersion;
        [SerializeField] private int sourceObjectCount;

        public string EnvironmentId => environmentId;
        public string ManifestSha256 => manifestSha256;
        public string SourceVersion => sourceVersion;
        public int SourceObjectCount => sourceObjectCount;

        public void Configure(string id, string manifestHash, string version, int objectCount) {
            environmentId = id;
            manifestSha256 = manifestHash;
            sourceVersion = version;
            sourceObjectCount = objectCount;
        }
    }
}
