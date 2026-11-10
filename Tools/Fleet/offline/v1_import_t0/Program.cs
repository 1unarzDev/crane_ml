using System; using Crane.Fleet;
int fails = 0;
foreach (var role in new[] { "scout", "lifter", "cargo" }) {
  var a = AssetManifest.Load($"/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet/assets_v1/{role}/{role}.manifest.json", loadMeshes: false);
  Console.WriteLine($"{role}: tags {a.Tags.Count}, face tags [{string.Join(",", a.FaceTags)}]");
  foreach (var t in a.Tags) {
    var R = AssetManifest.OutwardRotation(t.Origin.Rpy);
    double[] z = { R[2], R[5], R[8] }, y = { R[1], R[4], R[7] };
    double dz = Math.Abs(z[0] - t.OutwardNormal[0]) + Math.Abs(z[1] - t.OutwardNormal[1]) + Math.Abs(z[2] - t.OutwardNormal[2]);
    // y must point "up the face": non-negative world z component (vertical/bevel tags: >0; top tags: horizontal)
    bool ok = dz < 1e-6 && y[2] > -1e-6;
    if (!ok) fails++;
    Console.WriteLine($"   {(ok ? "PASS" : "FAIL")} {t.Name} id {t.Id} size {t.Size} z_out ({z[0]:0.###},{z[1]:0.###},{z[2]:0.###}) vs normal ({t.OutwardNormal[0]:0.###},{t.OutwardNormal[1]:0.###},{t.OutwardNormal[2]:0.###}) y.z {y[2]:0.###}");
  }
}
// docking profile through CRANE's builder: v0 two hooks per face, v1 reciprocal with magnet geometry and per-role faces
foreach (var key in new[] { "v0", "v1" }) {
  var dspec = FleetSpec.LoadDirectory("/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet", FleetSpec.DefaultScenario, key);
  try {
    var dp = Crane.Fleet.Docking.DockingProfileBuilder.Build(dspec);
    bool ok = key == "v0" ? dp.Mode == Crane.Fleet.Profile.FaceMode.TwoHooksPerFace : dp.Mode == Crane.Fleet.Profile.FaceMode.Reciprocal && dp.Magnet != null && dp.RoleActiveFaces["lifter"].Count == 3 && dp.HookOpenAngle != null;
    if (!ok) fails++;
    Console.WriteLine($"{(ok ? "PASS" : "FAIL")} docking profile {key}: mode {dp.Mode}, magnet {(dp.Magnet != null ? dp.Magnet.FaceY.ToString() : "-")}, lifter faces {(dp.RoleActiveFaces.TryGetValue("lifter", out var lf) ? string.Join(",", lf) : "-")}");
  } catch (Exception e) { fails++; Console.WriteLine($"FAIL docking profile {key}: {e.Message}"); }
}
// perception and autonomy parse both profiles
foreach (var key in new[] { "v0", "v1" }) {
  var path = $"/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet/fleet_profile_{key}.json";
  try { var fp = Crane.Fleet.Perception.FleetProfile.Parse(System.IO.File.ReadAllText(path)); Console.WriteLine($"PASS perception parses {key} (face tag {fp.FaceTagSize})"); }
  catch (Exception e) { fails++; Console.WriteLine($"FAIL perception {key}: {e.Message}"); }
}
Console.WriteLine(fails == 0 ? "ALL PASS" : fails + " FAIL"); Environment.Exit(fails);
