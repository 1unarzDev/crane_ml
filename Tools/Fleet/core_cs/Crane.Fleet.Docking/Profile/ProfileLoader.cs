using System;
using System.Collections.Generic;
using System.Text.Json;

namespace Crane.Fleet.Profile
{
    public sealed class ProfileLoadResult
    {
        public DockingProfile Profile { get; set; }
        public List<ProfileError> Errors { get; set; } = new List<ProfileError>();
        public bool Ok => Errors.Count == 0;
    }

    /// <summary>
    /// Loads fleet_profile_v0.json (System.Text.Json). The only file in the library with an external dependency;
    /// see README for the Unity options.
    /// </summary>
    public static class ProfileLoader
    {
        public static DockingProfile LoadOrThrow(string json)
        {
            var r = Load(json);
            if (!r.Ok) throw new ProfileException(r.Errors);
            return r.Profile;
        }

        public static ProfileLoadResult Load(string json)
        {
            var res = new ProfileLoadResult();
            JsonDocument doc;
            try { doc = JsonDocument.Parse(json); }
            catch (JsonException ex) { res.Errors.Add(new ProfileError("$", "invalid JSON: " + ex.Message)); return res; }
            using (doc)
            {
                var root = doc.RootElement;
                var errs = res.Errors;
                var p = new DockingProfile();
                if (root.ValueKind != JsonValueKind.Object) { errs.Add(new ProfileError("$", "root must be an object")); return res; }

                p.Schema = Str(root, "schema", "schema", errs);
                p.ProfileId = Str(root, "profile_id", "profile_id", errs);

                var levels = new HashSet<string>();
                if (root.TryGetProperty("provenance_levels", out var pl) && pl.ValueKind == JsonValueKind.Array)
                    foreach (var x in pl.EnumerateArray()) levels.Add(x.GetString());
                else errs.Add(new ProfileError("provenance_levels", "missing"));

                if (root.TryGetProperty("units", out var un) && un.ValueKind == JsonValueKind.Object)
                    foreach (var kv in un.EnumerateObject()) p.Units[kv.Name] = kv.Value.GetString();
                else errs.Add(new ProfileError("units", "missing"));

                if (!root.TryGetProperty("docking", out var d) || d.ValueKind != JsonValueKind.Object)
                { errs.Add(new ProfileError("docking", "missing")); return res; }

                if (d.TryGetProperty("face_frames", out var ff) && ff.ValueKind == JsonValueKind.Object)
                {
                    foreach (var f in ff.EnumerateObject())
                    {
                        if (f.Value.ValueKind != JsonValueKind.Object) continue; // 'note'
                        var v = Vec(f.Value, "docking.face_frames." + f.Name, 4, levels, errs, out var prov);
                        if (v != null) p.FaceFrames[f.Name] = new FaceFrame(f.Name, v[0], v[1], v[2], v[3], prov);
                    }
                }
                else errs.Add(new ProfileError("docking.face_frames", "missing"));

                if (d.TryGetProperty("active_faces_default", out var af) && af.ValueKind == JsonValueKind.Array)
                    foreach (var x in af.EnumerateArray()) p.ActiveFaces.Add(x.GetString());

                p.GuideHalfWidth = Scalar(d, "guide_half_width", "docking.", levels, errs);
                p.GuideAcceptanceHalfAngle = Scalar(d, "guide_acceptance_half_angle", "docking.", levels, errs);
                p.MagnetPullInRange = Scalar(d, "magnet_pull_in_range", "docking.", levels, errs);
                p.MagnetHoldForce = Scalar(d, "magnet_hold_force", "docking.", levels, errs);
                p.HookCount = Scalar(d, "hook_count", "docking.", levels, errs);
                if (d.TryGetProperty("hook_open_angle", out _)) p.HookOpenAngle = Scalar(d, "hook_open_angle", "docking.", levels, errs);
                if (d.TryGetProperty("magnet", out var mg)) p.Magnet = Magnet(mg, levels, errs, p.Metadata);
                foreach (string dn in new[] { "partner_acquisition", "states_mapping_v1", "partner_acquisition_proposal" })
                    if (d.TryGetProperty(dn, out var dv)) p.Metadata["docking." + dn] = Descriptive(dv);
                p.HookLateralSpacing = Scalar(d, "hook_lateral_spacing", "docking.", levels, errs);
                p.HookPositionTolerance = Scalar(d, "hook_position_tolerance", "docking.", levels, errs);
                p.HookYawTolerance = Scalar(d, "hook_yaw_tolerance", "docking.", levels, errs);
                p.HookActuationTime = Scalar(d, "hook_actuation_time", "docking.", levels, errs);
                p.HookRatedLoad = Scalar(d, "hook_rated_load", "docking.", levels, errs);
                p.ConnectorComplianceLateral = Scalar(d, "connector_compliance_lateral", "docking.", levels, errs);
                p.States = StrList(d, "states", "docking.states", errs);
                p.FaultStates = StrList(d, "fault_states", "docking.fault_states", errs);
                p.ReleaseInvariant = Str(d, "release_invariant", "docking.release_invariant", errs);

                if (root.TryGetProperty("roles", out var roles) && roles.ValueKind == JsonValueKind.Object)
                    foreach (var r in roles.EnumerateObject())
                        if (r.Value.ValueKind == JsonValueKind.Object && r.Value.TryGetProperty("active_faces", out var raf))
                            p.RoleActiveFaces[r.Name] = StrList(r.Value, "active_faces", "roles." + r.Name + ".active_faces", errs);

                if (root.TryGetProperty("chassis", out var ch) && ch.ValueKind == JsonValueKind.Object)
                {
                    p.BaseMass = Scalar(ch, "base_mass", "chassis.", levels, errs);
                    if (ch.TryGetProperty("body_size_lwh", out var bs))
                    {
                        var v = Vec(bs, "chassis.body_size_lwh", 3, levels, errs, out var prov);
                        if (v != null) p.BodySizeLwh = new VecParam(v, prov);
                    }
                    else errs.Add(new ProfileError("chassis.body_size_lwh", "missing"));
                }
                else errs.Add(new ProfileError("chassis", "missing"));

                if (root.TryGetProperty("payloads", out var pays) && pays.ValueKind == JsonValueKind.Object)
                {
                    foreach (var pe in pays.EnumerateObject())
                    {
                        string path = "payloads." + pe.Name;
                        if (!pe.Value.TryGetProperty("size_lwh", out var sz)) { errs.Add(new ProfileError(path + ".size_lwh", "missing")); continue; }
                        var v = Vec(sz, path + ".size_lwh", 3, levels, errs, out var prov);
                        var mass = Scalar(pe.Value, "mass", path + ".", levels, errs);
                        string needs = pe.Value.TryGetProperty("needs", out var nd) ? nd.GetString() : "";
                        if (v != null && mass != null) p.Payloads[pe.Name] = new PayloadSpec(pe.Name, new VecParam(v, prov), mass, needs);
                    }
                }

                res.Profile = p;
                if (errs.Count == 0) errs.AddRange(p.Validate());
            }
            return res;
        }

        private static string Str(JsonElement o, string name, string path, List<ProfileError> errs)
        {
            if (o.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String) return v.GetString();
            errs.Add(new ProfileError(path, "missing or not a string"));
            return null;
        }

        private static List<string> StrList(JsonElement o, string name, string path, List<ProfileError> errs)
        {
            var l = new List<string>();
            if (o.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.Array)
                foreach (var x in v.EnumerateArray()) l.Add(x.GetString());
            else errs.Add(new ProfileError(path, "missing or not an array"));
            return l;
        }

        private static bool Prov(JsonElement o, string path, HashSet<string> levels, List<ProfileError> errs, out Provenance prov)
        {
            prov = Provenance.Provisional;
            if (!o.TryGetProperty("provenance", out var pv) || pv.ValueKind != JsonValueKind.String)
            { errs.Add(new ProfileError(path + ".provenance", "missing")); return false; }
            string s = pv.GetString();
            if (!levels.Contains(s))
            { errs.Add(new ProfileError(path + ".provenance", $"unknown provenance '{s}' (profile provenance_levels: {string.Join("|", levels)})")); return false; }
            if (!Enum.TryParse(s, true, out prov) || !Enum.IsDefined(typeof(Provenance), prov))
            { errs.Add(new ProfileError(path + ".provenance", $"unknown provenance '{s}': listed in provenance_levels but not supported by this library")); return false; }
            return true;
        }

        private static Param Scalar(JsonElement o, string name, string prefix, HashSet<string> levels, List<ProfileError> errs)
        {
            string path = prefix + name;
            if (!o.TryGetProperty(name, out var e) || e.ValueKind != JsonValueKind.Object)
            { errs.Add(new ProfileError(path, "missing")); return null; }
            if (!e.TryGetProperty("value", out var v) || v.ValueKind != JsonValueKind.Number)
            { errs.Add(new ProfileError(path + ".value", "missing or not a number")); return null; }
            if (!Prov(e, path, levels, errs, out var prov)) return null;
            return new Param(v.GetDouble(), prov);
        }

        /// <summary>Descriptive leaf (string, or {"value": string, ...}): kept as text, never coerced.</summary>
        private static string Descriptive(JsonElement e)
        {
            if (e.ValueKind == JsonValueKind.Object && e.TryGetProperty("value", out var v)) e = v;
            return e.ValueKind == JsonValueKind.String ? e.GetString() : e.GetRawText();
        }

        private static MagnetGeometry Magnet(JsonElement e, HashSet<string> levels, List<ProfileError> errs, Dictionary<string, string> meta)
        {
            const string path = "docking.magnet";
            if (e.ValueKind != JsonValueKind.Object || !e.TryGetProperty("value", out var v) || v.ValueKind != JsonValueKind.Object)
            { errs.Add(new ProfileError(path + ".value", "missing or not an object")); return null; }
            var n = new double[4]; bool ok = true; int i = 0;
            foreach (string k in new[] { "diameter", "thickness", "face_y", "face_z" })
            {
                if (!v.TryGetProperty(k, out var x) || x.ValueKind != JsonValueKind.Number)
                { errs.Add(new ProfileError(path + ".value." + k, "missing or not a number")); ok = false; }
                else n[i] = x.GetDouble();
                i++;
            }
            if (v.TryGetProperty("polarity", out var pol)) meta[path + ".polarity"] = Descriptive(pol);
            if (!Prov(e, path, levels, errs, out var prov) || !ok) return null;
            return new MagnetGeometry(n[0], n[1], n[2], n[3], prov);
        }

        private static double[] Vec(JsonElement e, string path, int n, HashSet<string> levels, List<ProfileError> errs, out Provenance prov)
        {
            prov = Provenance.Provisional;
            if (e.ValueKind != JsonValueKind.Object || !e.TryGetProperty("value", out var v) || v.ValueKind != JsonValueKind.Array || v.GetArrayLength() != n)
            { errs.Add(new ProfileError(path + ".value", $"missing or not an array of {n}")); return null; }
            var r = new double[n]; int i = 0;
            foreach (var x in v.EnumerateArray())
            {
                if (x.ValueKind != JsonValueKind.Number) { errs.Add(new ProfileError(path + ".value", "non-numeric entry")); return null; }
                r[i++] = x.GetDouble();
            }
            if (!Prov(e, path, levels, errs, out prov)) return null;
            return r;
        }
    }
}
