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
Console.WriteLine(fails == 0 ? "ALL PASS" : fails + " FAIL"); Environment.Exit(fails);
