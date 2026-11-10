using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Nodes;
using Crane.Fleet.Docking;
using Crane.Fleet.Graph;
using Crane.Fleet.Profile;
using Xunit;

namespace Crane.Fleet.Docking.Tests
{
    /// <summary>fleet_profile_v1.json loading and the reciprocal (one J-hook + one catch pin per face) connection model.</summary>
    public class ReciprocalProfileTests
    {
        private static ProfileLoadResult LoadMutated(Action<JsonNode> mutate)
        {
            var n = JsonNode.Parse(Fx.ProfileJsonV1());
            mutate(n);
            return ProfileLoader.Load(n.ToJsonString());
        }

        [Fact]
        public void V1_loads_and_selects_reciprocal_mode_from_hook_count()
        {
            var r = ProfileLoader.Load(Fx.ProfileJsonV1());
            Assert.True(r.Ok, string.Join("; ", r.Errors));
            var p = r.Profile;
            Assert.Equal(FaceMode.Reciprocal, p.Mode);
            Assert.Equal(1.0, p.HookCount.Value);
            Assert.Equal(Provenance.Concept, p.HookCount.Provenance);
            Assert.Equal(-1.0, p.HookASide);
            Assert.Equal(0.022, p.HookHalfSpan, 9);
        }

        [Fact]
        public void V1_numbers_come_from_the_file_and_descriptive_fields_are_metadata()
        {
            var d = JsonNode.Parse(Fx.ProfileJsonV1())["docking"];
            var p = Fx.ProfileV1();
            Assert.Equal((double)d["guide_half_width"]["value"], p.GuideHalfWidth.Value);
            Assert.Equal((double)d["hook_open_angle"]["value"], p.HookOpenAngle.Value);
            Assert.Equal((double)d["magnet"]["value"]["diameter"], p.Magnet.Diameter);
            Assert.Equal((double)d["magnet"]["value"]["thickness"], p.Magnet.Thickness);
            Assert.Equal((double)d["magnet"]["value"]["face_y"], p.Magnet.FaceY);
            Assert.Equal((double)d["magnet"]["value"]["face_z"], p.Magnet.FaceZ);
            Assert.Equal(Provenance.Concept, p.Magnet.Provenance);
            Assert.Equal((string)d["magnet"]["value"]["polarity"], p.Metadata["docking.magnet.polarity"]);
            Assert.Equal("bevel_tags_staged", p.Metadata["docking.partner_acquisition"]);
            Assert.Contains("HOOK_A", p.Metadata["docking.states_mapping_v1"]);
            var prov = p.AllProvenance().ToDictionary(k => k.Key, k => k.Value);
            Assert.Equal(Provenance.Concept, prov["docking.face_frames.front"]);
            Assert.Equal(Provenance.Concept, prov["docking.magnet"]);
        }

        [Fact]
        public void Per_role_active_faces_are_read_and_the_lifter_front_gripper_is_not_a_dock()
        {
            var p = Fx.ProfileV1();
            Assert.Equal(new[] { "front", "rear", "left", "right" }, p.RoleActiveFaces["scout"].ToArray());
            Assert.Equal(new[] { "front", "rear", "left", "right" }, p.RoleActiveFaces["cargo"].ToArray());
            Assert.Equal(new[] { "rear", "left", "right" }, p.RoleActiveFaces["lifter"].ToArray());
        }

        [Fact]
        public void Provenance_levels_are_read_from_the_profile()
        {
            // 'concept' and 'datasheet' are accepted when the profile lists them
            var r = LoadMutated(n => n["docking"]["hook_rated_load"]["provenance"] = "datasheet");
            Assert.True(r.Ok, string.Join("; ", r.Errors));
            Assert.Equal(Provenance.Datasheet, r.Profile.HookRatedLoad.Provenance);
            // not listed => rejected, naming the path and the profile's levels
            r = LoadMutated(n => ((JsonArray)n["provenance_levels"]).Remove(((JsonArray)n["provenance_levels"]).First(x => (string)x == "concept")));
            Assert.Contains(r.Errors, e => e.Path == "docking.guide_half_width.provenance" && e.Message.Contains("unknown provenance 'concept'"));
            // listed but the library has no label for it => its own message, not silently coerced
            r = LoadMutated(n => { ((JsonArray)n["provenance_levels"]).Add("vendor_claim"); n["docking"]["hook_rated_load"]["provenance"] = "vendor_claim"; });
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_rated_load.provenance" && e.Message.Contains("not supported by this library"));
        }

        [Fact]
        public void Malformed_numeric_fields_fail_loudly_with_their_path()
        {
            var r = LoadMutated(n => n["docking"]["magnet"]["value"]["diameter"] = "ten mm");
            Assert.Contains(r.Errors, e => e.Path == "docking.magnet.value.diameter");
            r = LoadMutated(n => n["docking"]["hook_count"]["value"] = "one");
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_count.value");
            r = LoadMutated(n => n["docking"]["hook_open_angle"]["value"] = "wide");
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_open_angle.value");
            r = LoadMutated(n => n["docking"]["hook_lateral_spacing"]["value"] = null);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_lateral_spacing.value");
            r = LoadMutated(n => n["docking"]["hook_count"]["value"] = 3);
            Assert.Contains(r.Errors, e => e.Path == "docking.hook_count");
            r = LoadMutated(n => ((JsonArray)n["roles"]["lifter"]["active_faces"]).Add("top"));
            Assert.Contains(r.Errors, e => e.Path == "roles.lifter.active_faces");
        }

        [Fact]
        public void A_descriptive_field_is_never_coerced()
        {
            var r = LoadMutated(n => n["docking"]["partner_acquisition"]["value"] = 5);
            Assert.True(r.Ok, string.Join("; ", r.Errors));
            Assert.Equal("5", r.Profile.Metadata["docking.partner_acquisition"]);
        }

        [Fact]
        public void V0_golden_values_are_unchanged()
        {
            var p = Fx.Profile();
            Assert.Equal("mecanum_fleet_provisional_v0", p.ProfileId);
            Assert.Equal(FaceMode.TwoHooksPerFace, p.Mode);
            Assert.Equal(+1.0, p.HookASide);
            Assert.Equal(2.0, p.HookCount.Value);
            Assert.Equal(0.01, p.GuideHalfWidth.Value); Assert.Equal(0.087, p.GuideAcceptanceHalfAngle.Value);
            Assert.Equal(0.008, p.MagnetPullInRange.Value); Assert.Equal(25.0, p.MagnetHoldForce.Value);
            Assert.Equal(0.12, p.HookLateralSpacing.Value); Assert.Equal(0.06, p.HookHalfSpan, 12);
            Assert.Equal(0.002, p.HookPositionTolerance.Value); Assert.Equal(0.035, p.HookYawTolerance.Value);
            Assert.Equal(0.8, p.HookActuationTime.Value); Assert.Equal(60.0, p.HookRatedLoad.Value);
            Assert.Equal(0.0005, p.ConnectorComplianceLateral.Value);
            Assert.Equal(6.0, p.BaseMass.Value); Assert.Equal(new[] { 0.42, 0.3, 0.1 }, p.BodySizeLwh.Value);
            Assert.Equal(new[] { "front", "rear" }, p.ActiveFaces);
            Assert.Equal(new[] { "box_long", "box_small", "tray_wide" }, p.Payloads.Keys.OrderBy(k => k).ToArray());
            Assert.Equal(5.0, p.Payloads["tray_wide"].Mass.Value);
            Assert.All(p.AllProvenance(), kv => Assert.Equal(Provenance.Provisional, kv.Value));
            Assert.Null(p.Magnet); Assert.Null(p.HookOpenAngle); Assert.Empty(p.Metadata);
            Assert.Equal(new[] { 0, 1, 2 }, new[] { (int)Provenance.Provisional, (int)Provenance.Cad, (int)Provenance.Measured });
            // contact model: identical hook geometry to the pre-v1 library
            var m = Fx.Model();
            Assert.Equal(+1.0, m.SideOf(0)); Assert.Equal(-1.0, m.SideOf(1));
            var h = m.HookResidual(new Residual(0, 0.01, 0), 0);
            Assert.Equal(0.06 * Math.Sin(0.01), h.LateralAtHook, 12);
            var l = m.StaticSplit(new LoadCase(0, 0, 1.2), true, true);
            Assert.Equal(1.2 / 0.12, l.A, 9); Assert.Equal(0, l.B, 9);
        }

        [Fact]
        public void V0_ignores_the_partner_link_entirely()
        {
            var a = Fx.Det(); a.PartnerLinkLatency = 5.0; a.PartnerObservationMaxAge = 0.0;
            var m = Fx.Machine(a);
            var inp = Fx.At(0, 0, 0.1); inp.Command = DockingCommand.Approach; inp.PartnerLinkUp = false;
            m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.None; inp.Pose.Gap = 0.004; m.Step(Fx.Dt, inp);
            inp.Command = DockingCommand.EngageHooks; Fx.Run(m, inp, 2.0);
            inp.Command = DockingCommand.Verify; m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            Assert.True(m.PartnerObservationFresh);
        }
    }

    public class ReciprocalMachineTests
    {
        private static ConnectorInputs Run2(ConnectorMachine m, ConnectorInputs inp, double s) { Fx.Run(m, inp, s); return inp; }

        [Fact]
        public void Both_hooks_engage_then_latched_verified_with_the_documented_hook_mapping()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Latch(m);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            Assert.Equal(HookSeat.Seated, m.MechA);   // initiator's own hook
            Assert.Equal(HookSeat.Seated, m.MechB);   // partner's hook
            Assert.True(m.SensedA && m.SensedB);
            Assert.Equal(DockingOutcome.BothHooksVerified, OutcomeClassifier.Classify(m.Summary));
            // the state walk is the same nominal sequence
            var path = m.Log.Where(e => e.Kind == EventKind.Transition).Select(e => e.To).Distinct().ToList();
            Assert.Equal(new[] { ConnectorState.Approach, ConnectorState.MagnetCaptured, ConnectorState.HookAEngaged, ConnectorState.LatchedVerified }, path);
            inp.Command = DockingCommand.TestLoad; m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LoadValidated, m.State);
        }

        [Fact]
        public void Partner_hook_report_arrives_after_the_link_latency()
        {
            var a = Fx.Det();
            double lat = a.PartnerLinkLatency;
            Assert.True(lat > 0);
            var m = Fx.MachineV1(a);
            var inp = Fx.Capture(m);
            inp.Command = DockingCommand.EngageHooks;
            double tA = -1, tB = -1;
            for (int i = 0; i < 80 && tB < 0; i++)
            {
                m.Step(Fx.Dt, inp);
                if (tA < 0 && m.SensedA) tA = m.Time;
                if (tB < 0 && m.SensedB) tB = m.Time;
            }
            Assert.True(tA > 0 && tB > 0);
            // partner closes `lat` later (command delay) and its report arrives `lat` later still
            Assert.InRange(tB - tA, 2 * lat - 2 * Fx.Dt, 2 * lat + 2 * Fx.Dt);
            // zero latency: no lag beyond one step
            var a0 = Fx.Det(); a0.PartnerLinkLatency = 0;
            var m0 = Fx.MachineV1(a0);
            var i0 = Fx.Capture(m0); i0.Command = DockingCommand.EngageHooks;
            double uA = -1, uB = -1;
            for (int i = 0; i < 80 && uB < 0; i++) { m0.Step(Fx.Dt, i0); if (uA < 0 && m0.SensedA) uA = m0.Time; if (uB < 0 && m0.SensedB) uB = m0.Time; }
            Assert.Equal(uA, uB, 6);
        }

        [Fact]
        public void A_stalled_partner_hook_gives_PARTIAL_and_never_verifies()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Capture(m);
            inp.HookB = HookDrive.Stalled; inp.Command = DockingCommand.EngageHooks;
            Fx.Run(m, inp, 3.0);
            Assert.Equal(ConnectorState.Partial, m.State);
            Assert.True(m.SensedA); Assert.False(m.SensedB);
            Assert.Equal(DockingOutcome.OneHook, OutcomeClassifier.Classify(m.Summary));
            inp.Command = DockingCommand.Verify; m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.Partial, m.State);
        }

        [Fact]
        public void A_stalled_own_hook_gives_PARTIAL_too()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Capture(m);
            inp.HookA = HookDrive.Stalled; inp.Command = DockingCommand.EngageHooks;
            Fx.Run(m, inp, 3.0);
            Assert.Equal(ConnectorState.Partial, m.State);
            Assert.False(m.SensedA); Assert.True(m.SensedB);
        }

        [Fact]
        public void Yaw_and_offset_load_the_two_hooks_with_the_reciprocal_sign()
        {
            // lat +1.5 mm with yaw 0.03 rad: hook at -s sees 1.5 - 0.66 = 0.84 mm (seats), hook at +s sees 2.16 mm (> 2 mm tol)
            var m = Fx.MachineV1();
            var inp = Fx.Engage(m, lat: 0.0015, yaw: 0.03);
            Assert.Equal(HookSeat.Seated, m.MechA);
            Assert.NotEqual(HookSeat.Seated, m.MechB);
            Assert.Equal(ConnectorState.Partial, m.State);
            // mirror offset (same yaw): the other hook is the one that fails
            var m2 = Fx.MachineV1();
            Fx.Engage(m2, lat: -0.0015, yaw: 0.03);
            Assert.NotEqual(HookSeat.Seated, m2.MechA);
            Assert.Equal(HookSeat.Seated, m2.MechB);
        }

        [Fact]
        public void Moment_loads_the_hook_at_plus_lateral_which_is_B_in_reciprocal_mode()
        {
            var m = Fx.ModelV1();
            var l = m.StaticSplit(new LoadCase(0, 0, 1.2), true, true);
            Assert.Equal(0, l.A, 9);
            Assert.Equal(1.2 / m.HookSpacing, l.B, 9);
            Assert.Equal(0.044, m.HookSpacing, 9);
        }

        [Fact]
        public void False_engagement_is_caught_by_the_pose_residual_check()
        {
            var a = Fx.Det(); a.ClosedOnNothingProbability = 1.0;
            var m = Fx.MachineV1(a);
            Fx.Engage(m, lat: 0.003);
            Assert.Equal(HookSeat.ClosedOnNothing, m.MechA); Assert.Equal(HookSeat.ClosedOnNothing, m.MechB);
            Assert.True(m.SensedA && m.SensedB);   // sensors say engaged
            var inp = Fx.At(0.003, 0, 0.004); inp.Command = DockingCommand.Verify;
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
            Assert.Equal(DockingOutcome.FalseEngagement, OutcomeClassifier.Classify(m.Summary));
        }

        [Fact]
        public void Latched_verified_needs_the_pose_residual_within_tolerance()
        {
            var m = Fx.MachineV1();
            Fx.Engage(m);
            var inp = Fx.At(0, 0, 0.004); inp.Disturbance.Lateral = 0.004; inp.Command = DockingCommand.Verify;   // 2x tolerance after both hooks sensed
            m.Step(Fx.Dt, inp);
            Assert.NotEqual(ConnectorState.LatchedVerified, m.State);
            Assert.Equal(ConnectorState.FalseEngagement, m.State);
        }

        [Fact]
        public void Verification_lost_under_load_when_hooks_slip()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Latch(m);
            inp.AppliedLoad = new LoadCase(150, 0, 0);   // 75 N per hook > rated
            m.Step(Fx.Dt, inp);
            Assert.Contains(m.Log, e => e.Kind == EventKind.HookOverload);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
        }

        [Fact]
        public void Within_rated_load_keeps_the_latch()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Latch(m);
            inp.AppliedLoad = new LoadCase(100, 0, 0);
            Fx.Run(m, inp, 1.0);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
        }

        [Fact]
        public void Partner_hook_loss_is_seen_only_after_the_link_latency_and_then_loses_verification()
        {
            var a = Fx.Det();
            var m = Fx.MachineV1(a);
            var inp = Fx.Latch(m);
            inp.HookB = HookDrive.ForcedOpen;           // the partner's hook is knocked open
            m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);   // not reported yet: the stale window
            Assert.True(m.SensedB);
            Fx.Run(m, inp, a.PartnerLinkLatency + 2 * Fx.Dt);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
            Assert.False(m.SensedB);
        }

        [Fact]
        public void A_stale_partner_report_fails_safe()
        {
            var a = Fx.Det();
            var m = Fx.MachineV1(a);
            var inp = Fx.Latch(m);
            inp.PartnerLinkUp = false;
            Fx.Run(m, inp, a.PartnerObservationMaxAge + 0.2);
            Assert.False(m.SensedB);
            Assert.False(m.PartnerObservationFresh);
            Assert.Equal(ConnectorState.VerificationLost, m.State);
        }

        [Fact]
        public void Release_of_a_verified_connection_needs_support_authorisation_and_a_fresh_partner_report()
        {
            var m = Fx.MachineV1();
            var inp = Fx.Latch(m);
            inp.Command = DockingCommand.Release; inp.ReleaseAuthorized = false;
            var res = m.Step(Fx.Dt, inp);
            Assert.Equal(ConnectorState.LatchedVerified, m.State);
            Assert.Contains(res.Events, e => e.Kind == EventKind.ReleaseRefused);

            // authorised by the graph but the partner report has gone stale: still refused
            var m3 = Fx.MachineV1();
            var in3 = Fx.Latch(m3);
            in3.PartnerLinkUp = false; in3.ReleaseAuthorized = true;
            Fx.Run(m3, in3, 0.6);                // verification lost, report stale
            in3.Command = DockingCommand.Release;
            var r3 = m3.Step(Fx.Dt, in3);
            Assert.Equal(ConnectorState.VerificationLost, m3.State);
            Assert.Contains(r3.Events, e => e.Kind == EventKind.ReleaseRefused);

            // authorised and fresh: releases
            var m4 = Fx.MachineV1();
            var in4 = Fx.Latch(m4);
            in4.Command = DockingCommand.Release; in4.ReleaseAuthorized = true;
            Fx.Run(m4, in4, 2.0);
            Assert.Equal(ConnectorState.Released, m4.State);
        }

        [Fact]
        public void Graph_refuses_a_release_that_would_lose_payload_support_on_v1_geometry()
        {
            var p = Fx.ProfileV1();
            var g = ModuleGraph.Line(p, new DockingAssumptions(), 3);
            double pitch = p.BodySizeLwh.Value[0];
            g.AddPayload("p", "box_long", new Vec2(pitch / 2, 0), "r0", "r1");
            var d = g.CanRelease("c0_1");
            Assert.False(d.Allowed);
            Assert.Contains("payload p", d.Reason);
        }

        [Fact]
        public void Capture_tolerances_are_the_v1_values()
        {
            var p = Fx.ProfileV1();
            var m = Fx.ModelV1();
            Assert.Equal(p.GuideHalfWidth.Value + p.MagnetPullInRange.Value, m.LateralAcceptance(0), 12);
            double edge = p.GuideHalfWidth.Value + p.MagnetPullInRange.Value;
            Func<double, double, ConnectorState> attempt = (lat, yaw) =>
            {
                var mm = Fx.MachineV1();
                var inp = Fx.At(lat, yaw, p.MagnetPullInRange.Value * 0.5); inp.Command = DockingCommand.Approach;
                mm.Step(Fx.Dt, inp); inp.Command = DockingCommand.None; inp.Pose.Gap = 0;
                mm.Step(Fx.Dt, inp);
                return mm.State;
            };
            Assert.Equal(ConnectorState.MagnetCaptured, attempt(edge * 0.95, 0));
            Assert.Equal(ConnectorState.Free, attempt(edge * 1.05, 0));
            Assert.Equal(ConnectorState.MagnetCaptured, attempt(0, p.GuideAcceptanceHalfAngle.Value * 0.9));
            Assert.Equal(ConnectorState.Free, attempt(0, p.GuideAcceptanceHalfAngle.Value * 1.1));
            // hook tolerance is the v1 number: 1.9 mm seats, 2.1 mm does not (both hooks see the same lateral offset at zero yaw)
            var s = Fx.MachineV1(); Fx.Engage(s, lat: p.HookPositionTolerance.Value * 0.95);
            Assert.Equal(HookSeat.Seated, s.MechA); Assert.Equal(HookSeat.Seated, s.MechB);
            var u = Fx.MachineV1(); Fx.Engage(u, lat: p.HookPositionTolerance.Value * 1.05);
            Assert.NotEqual(HookSeat.Seated, u.MechA); Assert.NotEqual(HookSeat.Seated, u.MechB);
        }
    }

    /// <summary>Hook engage point vs the partner's catch pin for mated v1 faces.</summary>
    public class ReciprocalGeometryTests
    {
        private static double Wrap(double a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }
        private static (double x, double y, double z) Rz(double a, (double x, double y, double z) v)
            => (Math.Cos(a) * v.x - Math.Sin(a) * v.y, Math.Sin(a) * v.x + Math.Cos(a) * v.y, v.z);
        private static (double x, double y, double z) Arr(JsonElement e) => (e[0].GetDouble(), e[1].GetDouble(), e[2].GetDouble());

        private static string ManifestPath(string role)
            => Path.Combine(Environment.GetEnvironmentVariable("HOME") ?? "", ".cache", "crane-fleet", "assets_v1", role, role + ".manifest.json");

        /// <summary>Hook engage point (closed, q = 0) and catch pin of a face, both in that face's frame, from a v1 manifest and the profile face frame.</summary>
        private static ((double x, double y, double z) hook, (double x, double y, double z) pin) FaceGeometry(JsonElement manifest, DockingProfile p, string face)
        {
            var att = manifest.GetProperty("attachments");
            var piv = att.GetProperty("hook_pivots").EnumerateArray().First(x => x.GetProperty("face").GetString() == face).GetProperty("origin");
            var o = Arr(piv.GetProperty("xyz")); double pyaw = piv.GetProperty("rpy")[2].GetDouble();
            var hookDef = manifest.GetProperty("actuated").GetProperty("hooks").EnumerateArray().First(x => x.GetProperty("face").GetString() == face);
            var eng = Arr(hookDef.GetProperty("engage_point"));
            var er = Rz(pyaw, eng);
            var baseP = (x: o.x + er.x, y: o.y + er.y, z: o.z + er.z);
            var f = p.FaceFrames[face];
            var d = Rz(-f.Yaw, (baseP.x - f.X, baseP.y - f.Y, baseP.z - f.Z));
            var pin = Arr(att.GetProperty("docking_faces").EnumerateArray().First(x => x.GetProperty("name").GetString() == face)
                .GetProperty("catch_pins")[0].GetProperty("position"));
            return (d, pin);
        }

        private static double Dist((double x, double y, double z) a, (double x, double y, double z) b)
            => Math.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));

        [Fact]
        public void Profile_hook_and_pin_sit_at_minus_and_plus_half_spacing_so_mated_faces_cross_over()
        {
            var p = Fx.ProfileV1();
            double s = p.HookHalfSpan;
            // own hook at face y = -s, own pin at +s; Rz(pi) maps the partner's pin (+s) to -s: exactly the own hook
            var partnerPinInOwnFrame = Rz(Math.PI, (0, +s, 0));
            Assert.Equal(-s, partnerPinInOwnFrame.y, 12);
            Assert.True(Math.Abs(partnerPinInOwnFrame.y - (-s)) <= p.HookPositionTolerance.Value);
            Assert.Equal(0.041, p.Magnet.FaceY, 9);
            Assert.True(p.Magnet.FaceY > s);
        }

        [Fact]
        public void Mated_faces_in_the_profile_have_opposite_outward_normals()
        {
            var p = Fx.ProfileV1();
            foreach (var (a, b) in new[] { ("front", "rear"), ("left", "right") })
                Assert.Equal(Math.PI, Math.Abs(Wrap(p.FaceFrames[a].Yaw - p.FaceFrames[b].Yaw)), 9);
        }

        [Fact]
        public void Hook_engage_point_is_within_tolerance_of_the_partner_pin_for_every_mated_v1_face_pair()
        {
            var p = Fx.ProfileV1();
            int checkedPairs = 0;
            var roles = new[] { "scout", "cargo", "lifter" };
            if (!roles.All(r => File.Exists(ManifestPath(r)))) return;   // asset cache not present: nothing to compare against
            var docs = roles.ToDictionary(r => r, r => JsonDocument.Parse(File.ReadAllText(ManifestPath(r))));
            try
            {
                double tol = p.HookPositionTolerance.Value;
                foreach (var ra in roles) foreach (var rb in roles)
                    foreach (string fa in p.RoleActiveFaces[ra]) foreach (string fb in p.RoleActiveFaces[rb])
                    {
                        var ga = FaceGeometry(docs[ra].RootElement, p, fa);
                        var gb = FaceGeometry(docs[rb].RootElement, p, fb);
                        // B's face frame = A's face frame * Rz(pi): a point at q in B's face frame is Rz(pi) q in A's
                        double e1 = Dist(ga.hook, Rz(Math.PI, gb.pin));    // A's hook on B's pin
                        double e2 = Dist(gb.hook, Rz(Math.PI, ga.pin));    // B's hook on A's pin
                        Assert.True(e1 <= tol, $"{ra}.{fa} hook -> {rb}.{fb} pin: {e1 * 1000:0.000} mm > {tol * 1000} mm");
                        Assert.True(e2 <= tol, $"{rb}.{fb} hook -> {ra}.{fa} pin: {e2 * 1000:0.000} mm > {tol * 1000} mm");
                        // the hook's lateral position is the profile's half-spacing
                        Assert.Equal(-p.HookHalfSpan, ga.hook.y, 6);
                        checkedPairs++;
                    }
                Assert.True(checkedPairs > 0);
            }
            finally { foreach (var d in docs.Values) d.Dispose(); }
        }

        [Fact]
        public void Lifter_front_is_marker_only_in_the_manifest_and_absent_from_its_dock_faces()
        {
            string path = ManifestPath("lifter");
            if (!File.Exists(path)) return;
            using var doc = JsonDocument.Parse(File.ReadAllText(path));
            var front = doc.RootElement.GetProperty("attachments").GetProperty("docking_faces").EnumerateArray().First(x => x.GetProperty("name").GetString() == "front");
            Assert.False(front.GetProperty("active").GetBoolean());
            Assert.DoesNotContain("front", Fx.ProfileV1().RoleActiveFaces["lifter"]);
        }
    }
}
