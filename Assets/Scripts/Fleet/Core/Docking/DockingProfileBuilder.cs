using System;
using System.Collections.Generic;
using Crane.Fleet.Profile;

namespace Crane.Fleet.Docking {
    /// <summary>
    /// CRANE import decision (phase 2): the docking core's own System.Text.Json <c>ProfileLoader</c> is not imported. The single
    /// profile loader is <see cref="FleetSpec"/> (FleetJson + JsonNode); this builder fills the docking core's typed
    /// <see cref="DockingProfile"/> from it, keeping every provenance label, and runs the core's <c>Validate()</c>.
    /// </summary>
    public static class DockingProfileBuilder {
        public static DockingProfile Build(FleetSpec spec) {
            JsonNode root = spec.P, d = root["docking"];
            var p = new DockingProfile {
                Schema = root["schema"].String,
                ProfileId = root["profile_id"].String
            };
            foreach (var kv in root["units"].Members) p.Units[kv.Key] = kv.Value.String;
            foreach (var kv in d["face_frames"].Members) {
                if (kv.Key == "note") continue;
                double[] v = kv.Value.Doubles;
                p.FaceFrames[kv.Key] = new FaceFrame(kv.Key, v[0], v[1], v[2], v[3], Prov(kv.Value));
            }
            foreach (var n in d["active_faces_default"].Items) p.ActiveFaces.Add(n.String);
            p.GuideHalfWidth = Scalar(d, "guide_half_width");
            p.GuideAcceptanceHalfAngle = Scalar(d, "guide_acceptance_half_angle");
            p.MagnetPullInRange = Scalar(d, "magnet_pull_in_range");
            p.MagnetHoldForce = Scalar(d, "magnet_hold_force");
            p.HookCount = Scalar(d, "hook_count");
            p.HookLateralSpacing = Scalar(d, "hook_lateral_spacing");
            p.HookPositionTolerance = Scalar(d, "hook_position_tolerance");
            p.HookYawTolerance = Scalar(d, "hook_yaw_tolerance");
            p.HookActuationTime = Scalar(d, "hook_actuation_time");
            p.HookRatedLoad = Scalar(d, "hook_rated_load");
            p.ConnectorComplianceLateral = Scalar(d, "connector_compliance_lateral");
            foreach (var n in d["states"].Items) p.States.Add(n.String);
            foreach (var n in d["fault_states"].Items) p.FaultStates.Add(n.String);
            p.ReleaseInvariant = d["release_invariant"].String;
            p.BaseMass = Scalar(root["chassis"], "base_mass");
            p.BodySizeLwh = new VecParam(root["chassis.body_size_lwh"].Doubles, Prov(root["chassis.body_size_lwh"]));
            foreach (var kv in root["payloads"].Members) {
                var sz = kv.Value["size_lwh"];
                p.Payloads[kv.Key] = new PayloadSpec(kv.Key, new VecParam(sz.Doubles, Prov(sz)), Scalar(kv.Value, "mass"),
                    kv.Value.Has("needs") ? kv.Value["needs"].String : "");
            }
            List<ProfileError> errors = p.Validate();
            if (errors.Count > 0) throw new ProfileException(errors);
            return p;
        }

        static Param Scalar(JsonNode parent, string key) {
            var n = parent[key];
            return new Param(n.Double, Prov(n));
        }

        static Provenance Prov(JsonNode n) {
            string s = n.Provenance ?? throw new FormatException(n.Path + " has no provenance");
            if (!Enum.TryParse(s, true, out Provenance p)) throw new FormatException($"{n.Path}: unknown provenance '{s}'");
            return p;
        }
    }
}
