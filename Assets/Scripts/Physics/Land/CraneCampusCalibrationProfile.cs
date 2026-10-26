using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using UnityEngine;

namespace Sim.Physics.Land {
    /// <summary>
    /// Immutable, hashable calibration parameters loaded from an acquisition-aligned fit
    /// artifact. A synthetic fixture may exercise the loader, but it is never physical evidence.
    /// </summary>
    [Serializable] public sealed class CraneCampusCalibrationProfile {
        [Serializable] public sealed class LidarParameters {
            public float sigmaMeters;
            public float dropoutProbability;
            public int latencyScans;
            public string materialClass;
        }
        [Serializable] public sealed class Parameters {
            public float massKg;
            public float motorLagSeconds;
            public float accelerationMps2;
            public float angularAccelerationRadPerSec2;
            public float maxMotorTorqueNm;
            public float leftWheelScale = 1;
            public float rightWheelScale = 1;
            public LidarParameters lidar;
        }
        [Serializable] public sealed class FitProvenance {
            public string[] trialIds;
            public string manifestSha256;
            public string fitNote;
        }
        public string schema;
        public string id;
        public string version;
        public string sourceKind;
        public Parameters parameters;
        public FitProvenance fitProvenance;

        public string ArtifactPath { get; private set; }
        public string ArtifactSha256 { get; private set; }

        public static CraneCampusCalibrationProfile Load(string[] args) {
            int index = Array.IndexOf(args, "--crane-campus-calibration-profile");
            if (index < 0) return null;
            if (index + 1 >= args.Length || string.IsNullOrWhiteSpace(args[index + 1]))
                throw new ArgumentException("Calibration profile requires a JSON path");
            string path = Path.GetFullPath(args[index + 1]);
            if (!File.Exists(path)) throw new FileNotFoundException("Calibration profile", path);
            string json = File.ReadAllText(path, Encoding.UTF8);
            var profile = JsonUtility.FromJson<CraneCampusCalibrationProfile>(json);
            if (profile == null) throw new InvalidDataException("Calibration profile JSON is empty or invalid");
            profile.ArtifactPath = path;
            profile.ArtifactSha256 = Sha256(path);
            profile.Validate();
            return profile;
        }

        public void Validate() {
            if (schema != "crane-calibration-profile-v1") throw new InvalidDataException("Calibration profile schema");
            if (string.IsNullOrWhiteSpace(id) || string.IsNullOrWhiteSpace(version)) throw new InvalidDataException("Calibration profile identity");
            if (sourceKind != "physical" && sourceKind != "synthetic-fixture") throw new InvalidDataException("Calibration profile sourceKind");
            if (parameters == null) throw new InvalidDataException("Calibration profile parameters");
            FiniteRange(parameters.massKg, .5f, 5f, "massKg");
            FiniteRange(parameters.motorLagSeconds, .001f, 2f, "motorLagSeconds");
            FiniteRange(parameters.accelerationMps2, .0001f, 3f, "accelerationMps2");
            FiniteRange(parameters.angularAccelerationRadPerSec2, .0001f, 20f, "angularAccelerationRadPerSec2");
            FiniteRange(parameters.maxMotorTorqueNm, .0001f, 2f, "maxMotorTorqueNm");
            FiniteRange(parameters.leftWheelScale, .9f, 1.1f, "leftWheelScale");
            FiniteRange(parameters.rightWheelScale, .9f, 1.1f, "rightWheelScale");
            if (parameters.lidar == null) throw new InvalidDataException("Calibration profile lidar parameters");
            FiniteRange(parameters.lidar.sigmaMeters, 0, .1f, "lidar.sigmaMeters");
            FiniteRange(parameters.lidar.dropoutProbability, 0, .5f, "lidar.dropoutProbability");
            if (parameters.lidar.latencyScans < 0 || parameters.lidar.latencyScans > 3) throw new InvalidDataException("lidar.latencyScans");
            if (string.IsNullOrWhiteSpace(parameters.lidar.materialClass)) throw new InvalidDataException("lidar.materialClass");
            if (fitProvenance == null || fitProvenance.trialIds == null || fitProvenance.trialIds.Length == 0 ||
                string.IsNullOrWhiteSpace(fitProvenance.manifestSha256) || fitProvenance.manifestSha256.Length != 64 ||
                string.IsNullOrWhiteSpace(fitProvenance.fitNote))
                throw new InvalidDataException("Calibration profile fit provenance");
        }

        public string EventDescription() => string.Format(CultureInfo.InvariantCulture,
            "id={0}; version={1}; sourceKind={2}; sha256={3}; path={4}", id, version, sourceKind, ArtifactSha256, ArtifactPath);

        static void FiniteRange(float value, float min, float max, string name) {
            if (!float.IsFinite(value) || value < min || value > max) throw new ArgumentOutOfRangeException(name);
        }
        static string Sha256(string path) {
            using var sha = SHA256.Create();
            using var stream = File.OpenRead(path);
            return string.Concat(sha.ComputeHash(stream).Select(b => b.ToString("x2", CultureInfo.InvariantCulture)));
        }
    }
}
