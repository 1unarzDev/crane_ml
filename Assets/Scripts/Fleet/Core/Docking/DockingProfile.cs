using System;
using System.Collections.Generic;
using Crane.Fleet.Docking;

namespace Crane.Fleet.Profile
{
    /// <summary>Provenance label carried with every profile number (matches fleet_profile_v0.json provenance_levels).</summary>
    public enum Provenance { Provisional, Cad, Measured }

    /// <summary>A scalar from the profile: value in the file's units plus its provenance label.</summary>
    public sealed class Param
    {
        public double Value { get; }
        public Provenance Provenance { get; }
        public Param(double value, Provenance provenance) { Value = value; Provenance = provenance; }
        public static implicit operator double(Param p) => p.Value;
        public override string ToString() => $"{Value} [{Provenance}]";
    }

    /// <summary>Pose of a coupling face in the chassis frame (x forward, y left; yaw is the outward normal direction).</summary>
    public sealed class FaceFrame
    {
        public string Name { get; }
        public double X { get; }
        public double Y { get; }
        public double Z { get; }
        public double Yaw { get; }
        public Provenance Provenance { get; }
        public FaceFrame(string name, double x, double y, double z, double yaw, Provenance p)
        { Name = name; X = x; Y = y; Z = z; Yaw = yaw; Provenance = p; }
        public Pose2 Pose => new Pose2(X, Y, Yaw);
    }

    /// <summary>A vector-valued profile entry (e.g. body_size_lwh) with one provenance label.</summary>
    public sealed class VecParam
    {
        public double[] Value { get; }
        public Provenance Provenance { get; }
        public VecParam(double[] value, Provenance p) { Value = value; Provenance = p; }
    }

    public sealed class PayloadSpec
    {
        public string Name { get; }
        public VecParam SizeLwh { get; }
        public Param Mass { get; }
        public string Needs { get; }
        public PayloadSpec(string name, VecParam size, Param mass, string needs)
        { Name = name; SizeLwh = size; Mass = mass; Needs = needs; }
    }

    /// <summary>
    /// The 'docking' section (plus the chassis and payload entries the module graph needs) of the fleet profile.
    /// All values are read from the JSON; nothing here has a default. Mutable setters exist so tests and
    /// non-JSON hosts can build one, then call <see cref="Validate"/>.
    /// </summary>
    public sealed class DockingProfile
    {
        public string Schema { get; set; }
        public string ProfileId { get; set; }
        public Dictionary<string, string> Units { get; set; } = new Dictionary<string, string>();
        public Dictionary<string, FaceFrame> FaceFrames { get; set; } = new Dictionary<string, FaceFrame>();
        public List<string> ActiveFaces { get; set; } = new List<string>();
        public Param GuideHalfWidth { get; set; }
        public Param GuideAcceptanceHalfAngle { get; set; }
        public Param MagnetPullInRange { get; set; }
        public Param MagnetHoldForce { get; set; }
        public Param HookCount { get; set; }
        public Param HookLateralSpacing { get; set; }
        public Param HookPositionTolerance { get; set; }
        public Param HookYawTolerance { get; set; }
        public Param HookActuationTime { get; set; }
        public Param HookRatedLoad { get; set; }
        public Param ConnectorComplianceLateral { get; set; }
        public List<string> States { get; set; } = new List<string>();
        public List<string> FaultStates { get; set; } = new List<string>();
        public string ReleaseInvariant { get; set; }
        public VecParam BodySizeLwh { get; set; }
        public Param BaseMass { get; set; }
        public Dictionary<string, PayloadSpec> Payloads { get; set; } = new Dictionary<string, PayloadSpec>();

        /// <summary>Half the hook spacing; hook A sits at +this (left), hook B at -this.</summary>
        public double HookHalfSpan => HookLateralSpacing.Value * 0.5;

        /// <summary>Every provenance label in the profile, by path (for audit: how much of a result rests on provisional numbers).</summary>
        public IEnumerable<KeyValuePair<string, Provenance>> AllProvenance()
        {
            foreach (var kv in FaceFrames) yield return new KeyValuePair<string, Provenance>("docking.face_frames." + kv.Key, kv.Value.Provenance);
            var named = new (string, Param)[]
            {
                ("docking.guide_half_width", GuideHalfWidth), ("docking.guide_acceptance_half_angle", GuideAcceptanceHalfAngle),
                ("docking.magnet_pull_in_range", MagnetPullInRange), ("docking.magnet_hold_force", MagnetHoldForce),
                ("docking.hook_count", HookCount), ("docking.hook_lateral_spacing", HookLateralSpacing),
                ("docking.hook_position_tolerance", HookPositionTolerance), ("docking.hook_yaw_tolerance", HookYawTolerance),
                ("docking.hook_actuation_time", HookActuationTime), ("docking.hook_rated_load", HookRatedLoad),
                ("docking.connector_compliance_lateral", ConnectorComplianceLateral), ("chassis.base_mass", BaseMass),
            };
            foreach (var (n, p) in named) if (p != null) yield return new KeyValuePair<string, Provenance>(n, p.Provenance);
            if (BodySizeLwh != null) yield return new KeyValuePair<string, Provenance>("chassis.body_size_lwh", BodySizeLwh.Provenance);
            foreach (var kv in Payloads)
            {
                yield return new KeyValuePair<string, Provenance>("payloads." + kv.Key + ".size_lwh", kv.Value.SizeLwh.Provenance);
                yield return new KeyValuePair<string, Provenance>("payloads." + kv.Key + ".mass", kv.Value.Mass.Provenance);
            }
        }

        /// <summary>Value checks (non-positive tolerances, bad enums, unsupported hook count...). Empty list = valid.</summary>
        public List<ProfileError> Validate()
        {
            var e = new List<ProfileError>();
            void Req(Param p, string path, bool positive = true)
            {
                if (p == null) { e.Add(new ProfileError(path, "missing")); return; }
                if (double.IsNaN(p.Value) || double.IsInfinity(p.Value)) e.Add(new ProfileError(path, "not finite"));
                else if (positive && p.Value <= 0) e.Add(new ProfileError(path, $"must be > 0 (got {p.Value})"));
            }
            Req(GuideHalfWidth, "docking.guide_half_width");
            Req(GuideAcceptanceHalfAngle, "docking.guide_acceptance_half_angle");
            if (GuideAcceptanceHalfAngle != null && GuideAcceptanceHalfAngle.Value >= Math.PI / 2)
                e.Add(new ProfileError("docking.guide_acceptance_half_angle", "must be < pi/2 rad"));
            Req(MagnetPullInRange, "docking.magnet_pull_in_range");
            Req(MagnetHoldForce, "docking.magnet_hold_force");
            Req(HookCount, "docking.hook_count");
            if (HookCount != null && HookCount.Value != 2) e.Add(new ProfileError("docking.hook_count", "this library models exactly two hooks"));
            Req(HookLateralSpacing, "docking.hook_lateral_spacing");
            Req(HookPositionTolerance, "docking.hook_position_tolerance");
            Req(HookYawTolerance, "docking.hook_yaw_tolerance");
            Req(HookActuationTime, "docking.hook_actuation_time");
            Req(HookRatedLoad, "docking.hook_rated_load");
            Req(ConnectorComplianceLateral, "docking.connector_compliance_lateral");
            Req(BaseMass, "chassis.base_mass");
            if (BodySizeLwh == null || BodySizeLwh.Value.Length != 3) e.Add(new ProfileError("chassis.body_size_lwh", "missing or not length 3"));
            else foreach (double v in BodySizeLwh.Value) if (!(v > 0)) e.Add(new ProfileError("chassis.body_size_lwh", "entries must be > 0"));
            foreach (var kv in new[] { ("length", "m"), ("angle", "rad"), ("time", "s"), ("force", "N"), ("mass", "kg") })
            {
                if (!Units.TryGetValue(kv.Item1, out var u)) e.Add(new ProfileError("units." + kv.Item1, "missing"));
                else if (u != kv.Item2) e.Add(new ProfileError("units." + kv.Item1, $"expected '{kv.Item2}', got '{u}'"));
            }
            if (FaceFrames.Count == 0) e.Add(new ProfileError("docking.face_frames", "no faces"));
            foreach (string a in ActiveFaces)
                if (!FaceFrames.ContainsKey(a)) e.Add(new ProfileError("docking.active_faces_default", $"unknown face '{a}'"));
            if (ActiveFaces.Count == 0) e.Add(new ProfileError("docking.active_faces_default", "missing or empty"));
            CheckNames(States, typeof(ConnectorState), "docking.states", false, e);
            CheckNames(FaultStates, typeof(ConnectorState), "docking.fault_states", true, e);
            if (string.IsNullOrWhiteSpace(ReleaseInvariant)) e.Add(new ProfileError("docking.release_invariant", "missing"));
            return e;
        }

        private static void CheckNames(List<string> names, Type enumType, string path, bool fault, List<ProfileError> e)
        {
            if (names == null || names.Count == 0) { e.Add(new ProfileError(path, "missing or empty")); return; }
            foreach (string n in names)
            {
                string pascal = ConnectorStateNames.FromProfileName(n);
                if (pascal == null || !Enum.IsDefined(enumType, pascal))
                    e.Add(new ProfileError(path, $"state '{n}' is not known to the state machine"));
            }
        }
    }

    public sealed class ProfileError
    {
        public string Path { get; }
        public string Message { get; }
        public ProfileError(string path, string message) { Path = path; Message = message; }
        public override string ToString() => Path + ": " + Message;
    }

    public sealed class ProfileException : Exception
    {
        public IReadOnlyList<ProfileError> Errors { get; }
        public ProfileException(IReadOnlyList<ProfileError> errors)
            : base("Invalid fleet profile: " + string.Join("; ", errors)) { Errors = errors; }
    }
}
