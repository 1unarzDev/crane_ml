using System;
using System.Collections.Generic;

namespace Sim.Physics.Core {
    /// <summary>Where a profile's numbers came from. Written by Tools/Characterization
    /// (import_yaml.py); hand-edited only for priors.</summary>
    [Serializable]
    public sealed class Provenance {
        /// <summary>"ideal" or "characterized".</summary>
        public string kind = "ideal";
        public string source = "";
        /// <summary>One word for logs: prior, provisional, characterized.</summary>
        public string summary = "prior";
        public List<ProvenanceInput> inputs = new();
        /// <summary>Gate report JSON next to the profile, if any.</summary>
        public string gateReport = "";
        public List<ParameterStatus> parameters = new();
    }

    [Serializable]
    public sealed class ProvenanceInput {
        public string path = "";
        public string sha256 = "";
    }

    /// <summary>Per-parameter gate outcome: identified, provisional or prior (with the reason).</summary>
    [Serializable]
    public sealed class ParameterStatus {
        public string name = "";
        public string status = "prior";
        public string reason = "";
    }
}
