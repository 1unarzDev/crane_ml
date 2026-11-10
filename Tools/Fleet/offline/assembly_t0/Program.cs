using System; using System.Collections.Generic; using Crane.Fleet;
// T0 checks of Crane.Fleet.AssemblyDrive (M2 command side). Geometry: profile mecanum chassis (r 0.0485, lx 0.14, ly 0.175, body 0.42 x 0.40)
// and the capplan diff_carrier (conventional wheels at (0, +-0.175), casters at (+-0.15, 0)). Faces are (x, y, yaw) only: hardware-agnostic.
static class P {
  static int fails;
  static void Check(bool ok, string what) { Console.WriteLine((ok ? "PASS " : "FAIL ") + what); if (!ok) fails++; }
  static WheelUnit W(string n, WheelKind k, double x, double y, int hand = 0) => new WheelUnit { Name = n, Kind = k, X = x, Y = y, Hand = hand, Radius = 0.0485, MaxSpeed = 12, MaxTorque = 1.2 };
  static List<WheelUnit> Mec() => new List<WheelUnit> { W("fl", WheelKind.Mecanum, 0.14, 0.175, -1), W("fr", WheelKind.Mecanum, 0.14, -0.175, 1), W("rl", WheelKind.Mecanum, -0.14, 0.175, 1), W("rr", WheelKind.Mecanum, -0.14, -0.175, -1) };
  static List<WheelUnit> Diff() => new List<WheelUnit> { W("l", WheelKind.Conventional, 0, 0.175), W("r", WheelKind.Conventional, 0, -0.175), W("cf", WheelKind.Caster, 0.15, 0), W("cr", WheelKind.Caster, -0.15, 0) };
  static readonly (double, double, double) Front = (0.21, 0, 0), Rear = (-0.21, 0, Math.PI), Left = (0, 0.20, Math.PI / 2);
  // single-module reference IK of the member's own body twist: v_m = R(-yaw)(v + w x p), w
  static double[] Reference(AssemblyDrive.Member m, Twist g) {
    double px = m.InAssembly.X, py = m.InAssembly.Y, vx = g.Vx - g.Wz * py, vy = g.Vy + g.Wz * px, c = Math.Cos(m.InAssembly.Yaw), s = Math.Sin(m.InAssembly.Yaw);
    var t = new Twist(c * vx + s * vy, -s * vx + c * vy, g.Wz);
    var tp = new TractivePlant(m.Wheels, new TractionParams { MaxSpeedXy = 1e9, MaxYawRate = 1e9 });
    var full = new double[m.Wheels.Count]; tp.Inverse(t, full);
    var r = new double[m.Driven.Length]; for (int k = 0; k < r.Length; k++) r[k] = full[m.Driven[k]]; return r;
  }
  static double MaxDiffToReference(AssemblyDrive a, Twist g) {
    var w = a.Inverse(g); double d = 0;
    foreach (var m in a.Members) { var r = Reference(m, g); for (int k = 0; k < r.Length; k++) d = Math.Max(d, Math.Abs(r[k] - w[m.Id][k])); }
    return d;
  }
  static double Err(Twist a, Twist b) => Math.Abs(a.Vx - b.Vx) + Math.Abs(a.Vy - b.Vy) + Math.Abs(a.Wz - b.Wz);
  static void Main() {
    // inline cargo pair: cargo_2's front mated to cargo_1's rear
    var inline = new AssemblyDrive();
    var p1 = new Pose2(0, 0, 0); var p2 = AssemblyDrive.MatedPose(p1, Rear, Front);
    inline.Add("cargo_1", p1, Mec()); inline.Add("cargo_2", p2, Mec());
    Check(Math.Abs(p2.X + 0.42) < 1e-12 && Math.Abs(p2.Y) < 1e-12 && Math.Abs(p2.Yaw) < 1e-12, $"inline mated pose {p2.X:0.###},{p2.Y:0.###},{p2.Yaw:0.###} = (-0.42, 0, 0)");
    // pure translation: every member's wheels equal the single-rover IK of the same twist
    var tr = new Twist(0.2, 0.1, 0); var wi = inline.Inverse(tr);
    var single = new TractivePlant(Mec(), new TractionParams { MaxSpeedXy = 1e9, MaxYawRate = 1e9 }); var ws = new double[4]; single.Inverse(tr, ws);
    double dt = 0; foreach (var id in new[] { "cargo_1", "cargo_2" }) for (int k = 0; k < 4; k++) dt = Math.Max(dt, Math.Abs(wi[id][k] - ws[k]));
    Check(dt < 1e-12, $"inline pure translation = single-rover IK on both members (max diff {dt:e1})");
    // pure yaw about the assembly origin: cargo_2 also translates laterally (0, -0.42 wz) in its frame
    var yaw = new Twist(0, 0, 1.0);
    Check(MaxDiffToReference(inline, yaw) < 1e-12, "inline pure yaw = member IK of (R(-yaw)(w x p), w)");
    var ref2 = Reference(inline.Members[1], yaw); var w2 = new double[4]; single.Inverse(new Twist(0, -0.42, 1.0), w2);
    double dy = 0; for (int k = 0; k < 4; k++) dy = Math.Max(dy, Math.Abs(ref2[k] - w2[k]));
    Check(dy < 1e-12, "inline pure yaw: cargo_2 wheels = IK of (0, -0.42, 1) explicitly");
    // L: cargo_b's rear mated to cargo_a's left face (cargo_b at yaw +pi/2)
    var L = new AssemblyDrive();
    var pa = new Pose2(0, 0, 0); var pb = AssemblyDrive.MatedPose(pa, Left, Rear);
    L.Add("a", pa, Mec()); L.Add("b", pb, Mec());
    Check(Math.Abs(pb.X) < 1e-12 && Math.Abs(pb.Y - 0.41) < 1e-12 && Math.Abs(pb.Yaw - Math.PI / 2) < 1e-12, $"L mated pose {pb.X:0.###},{pb.Y:0.###},{pb.Yaw:0.###} = (0, 0.41, pi/2)");
    var wl = L.Inverse(new Twist(0.2, 0, 0)); var wbExp = new double[4]; single.Inverse(new Twist(0, -0.2, 0), wbExp);
    double dl = 0; for (int k = 0; k < 4; k++) dl = Math.Max(dl, Math.Abs(wl["b"][k] - wbExp[k]));
    Check(dl < 1e-12, "L pure translation +x: member b (yaw pi/2) strafes right, IK of (0, -0.2, 0)");
    Check(MaxDiffToReference(L, yaw) < 1e-12, "L pure yaw = member IK identity");
    // random twists: identity against the member reference, and B A = I for full-rank mecanum groups
    var rng = new FleetRng(3); double di = 0, dfk = 0;
    for (int i = 0; i < 200; i++) {
      var g = new Twist(rng.Uniform(-0.5, 0.5), rng.Uniform(-0.5, 0.5), rng.Uniform(-1.5, 1.5));
      di = Math.Max(di, Math.Max(MaxDiffToReference(inline, g), MaxDiffToReference(L, g)));
      dfk = Math.Max(dfk, Math.Max(Err(inline.Forward(inline.Inverse(g)), g), Err(L.Forward(L.Inverse(g)), g)));
    }
    Check(di < 1e-10, $"200 random twists: group IK = member IK (max {di:e1})");
    Check(dfk < 1e-6, $"200 random twists: Forward(Inverse(t)) = t for mecanum pair and L (max {dfk:e1})");
    Check(inline.Mobility() == 3 && L.Mobility() == 3, "mecanum assemblies: mobility 3 (no hard lateral rows)");
    // coupled diff + mecanum: diff_carrier front mated to mecanum_pod rear -> loses lateral motion
    var dm = new AssemblyDrive();
    var pd = new Pose2(0, 0, 0); var pm = AssemblyDrive.MatedPose(pd, Front, Rear);
    dm.Add("diff", pd, Diff()); dm.Add("mec", pm, Mec());
    Check(dm.Mobility() == 2, $"diff+mecanum: mobility {dm.Mobility()} = 2 (two conventional wheels on one axle give one hard row)");
    Check(!dm.Feasible(new Twist(0, 0.2, 0)), "diff+mecanum: pure lateral translation infeasible");
    Check(dm.Feasible(new Twist(0.2, 0, 0)), "diff+mecanum: forward feasible");
    Check(dm.Feasible(new Twist(0, 0, 1.0)), "diff+mecanum: yaw about the diff axle centre (assembly origin) feasible");
    Check(!dm.Feasible(new Twist(0, 0.42, 1.0)), "diff+mecanum: yaw about the mecanum pod centre infeasible (diff axle would slide)");
    Check(dm.DriveRank() == 3, "diff+mecanum: driven wheels still command 3 directions (lateral only through scrub)");
    // B A is a projector even when rank-deficient (diff alone: lateral unobservable)
    var dOnly = new AssemblyDrive(); dOnly.Add("diff", pd, Diff());
    double dp = 0;
    for (int i = 0; i < 50; i++) {
      var g = new Twist(rng.Uniform(-0.5, 0.5), rng.Uniform(-0.5, 0.5), rng.Uniform(-1.5, 1.5));
      var p = dOnly.Forward(dOnly.Inverse(g)); var pp = dOnly.Forward(dOnly.Inverse(p));
      dp = Math.Max(dp, Err(p, pp));
    }
    Check(dp < 1e-6 && dOnly.DriveRank() == 2, $"diff alone: rank 2 and B A idempotent (max {dp:e1})");
    // saturation keeps direction with one common scale over both members
    inline.MaxSpeedXy = 10; inline.MaxYawRate = 10;
    var big = new Twist(2.0, 0, 0); double s = inline.Saturate(ref big, out var sw); double mx = 0;
    foreach (var kv in sw) foreach (var x in kv.Value) mx = Math.Max(mx, Math.Abs(x));
    Check(s < 1 && Math.Abs(mx - 12) < 1e-9, $"common saturation: scale {s:0.###}, max wheel {mx:0.###} rad/s = limit 12");
    Console.WriteLine(fails == 0 ? "ALL PASS" : fails + " FAIL"); Environment.Exit(fails);
  }
}
