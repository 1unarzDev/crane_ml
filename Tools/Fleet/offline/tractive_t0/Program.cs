using System; using System.Collections.Generic; using Crane.Fleet;
static class P {
  static int fails = 0;
  static void Check(bool ok, string what) { Console.WriteLine((ok ? "PASS " : "FAIL ") + what); if (!ok) fails++; }
  static TractionParams Tp(double mass, double mu = 0.9) => new TractionParams { MuLong = mu, MuLatRatio = 0.9, SlipVelocity = 0.02, RollerDrag = 0.02, RollingResistance = 0.01,
      MotorGain = 0.5, MotorTimeConstant = 0.05, BackdriveTorque = 0.05, BackdriveSpeed = 0.5, Mass = mass, Izz = mass * 0.04, ComX = 0, ComY = 0, ComZ = 0.08, Gravity = 9.81, MaxSpeedXy = 0.5, MaxYawRate = 2.0 };
  static WheelUnit Wh(string n, WheelKind k, double x, double y, int hand = 0, double yaw = 0) => new WheelUnit { Name = n, Kind = k, X = x, Y = y, Yaw = yaw, Hand = hand, Radius = 0.04, MaxSpeed = 15, MaxTorque = 0.6 };
  static List<WheelUnit> Mec() => new List<WheelUnit> { Wh("fl", WheelKind.Mecanum, 0.1, 0.12, -1), Wh("fr", WheelKind.Mecanum, 0.1, -0.12, 1), Wh("rl", WheelKind.Mecanum, -0.1, 0.12, 1), Wh("rr", WheelKind.Mecanum, -0.1, -0.12, -1) };
  static List<WheelUnit> Diff() => new List<WheelUnit> { Wh("l", WheelKind.Conventional, 0, 0.12), Wh("r", WheelKind.Conventional, 0, -0.12), Wh("cf", WheelKind.Caster, 0.15, 0), Wh("cr", WheelKind.Caster, -0.15, 0) };
  // planar rigid body at the CoM in the body frame, explicit (the plant's forces are already implicit-corrected)
  static (Twist v, double x, double y, double yaw) Sim(TractivePlant p, Twist cmd, double T, double dt = 0.005) {
    double vx = 0, vy = 0, wz = 0, x = 0, y = 0, yaw = 0;
    for (int s = 0; s < (int)(T / dt); s++) {
      p.Step(cmd, new Twist(vx, vy, wz), dt);
      double Fx = 0, Fy = 0, M = 0;
      for (int i = 0; i < p.N; i++) { Fx += p.Fx[i]; Fy += p.Fy[i]; M += p.W[i].X * p.Fy[i] - p.W[i].Y * p.Fx[i]; }
      double ax = Fx / p.P.Mass + wz * vy, ay = Fy / p.P.Mass - wz * vx;
      vx += ax * dt; vy += ay * dt; wz += M / p.P.Izz * dt;
      x += (Math.Cos(yaw) * vx - Math.Sin(yaw) * vy) * dt; y += (Math.Sin(yaw) * vx + Math.Cos(yaw) * vy) * dt; yaw += wz * dt;
    }
    return (new Twist(vx, vy, wz), x, y, yaw);
  }
  static void Main() {
    // 1 kinematic identity with the 4-wheel mecanum geometry (no-slip limit)
    var g = new MecanumGeometry(0.04, 0.1, 0.12, 15, 0.6, 1000, 0.5, 2);
    var tp = new TractivePlant(Mec(), Tp(8));
    double maxd = 0; var rng = new FleetRng(7);
    for (int k = 0; k < 200; k++) { var t = new Twist(rng.Uniform(-1, 1), rng.Uniform(-1, 1), rng.Uniform(-2, 2)); var a = new double[4]; var b = new double[4]; g.Inverse(t, a); tp.Inverse(t, b);
      for (int i = 0; i < 4; i++) maxd = Math.Max(maxd, Math.Abs(a[i] - b[i])); var f = tp.Forward(b); maxd = Math.Max(maxd, Math.Abs(f.Vx - t.Vx) + Math.Abs(f.Vy - t.Vy) + Math.Abs(f.Wz - t.Wz)); }
    Check(maxd < 1e-6, $"mecanum IK/FK identical to MecanumGeometry (max diff {maxd:e2})");
    // 2 mecanum tracks forward, strafe, spin at high friction
    foreach (var c in new[] { new Twist(0.3, 0, 0), new Twist(0, 0.3, 0), new Twist(0, 0, 1.0), new Twist(0.2, 0.2, 0.5) }) {
      var r = Sim(new TractivePlant(Mec(), Tp(8)), c, 2.0);
      double e = Math.Abs(r.v.Vx - c.Vx) + Math.Abs(r.v.Vy - c.Vy) + 0.2 * Math.Abs(r.v.Wz - c.Wz);
      Check(e < 0.03, $"mecanum steady twist {c} -> {r.v} (err {e:0.000})");
    }
    // 3 diff pod: forward ok; lateral command gives no lateral motion; spin works (casters free)
    var df = Sim(new TractivePlant(Diff(), Tp(5)), new Twist(0.3, 0, 0), 2.0);
    Check(Math.Abs(df.v.Vx - 0.3) < 0.02 && Math.Abs(df.v.Vy) < 1e-3, $"diff forward {df.v}");
    var dl = Sim(new TractivePlant(Diff(), Tp(5)), new Twist(0, 0.3, 0), 2.0);
    Check(Math.Abs(dl.v.Vy) < 1e-3 && Math.Abs(dl.y) < 1e-3, $"diff lateral command -> no lateral motion {dl.v} y={dl.y:e2}");
    var ds = Sim(new TractivePlant(Diff(), Tp(5)), new Twist(0, 0, 1.0), 2.0);
    Check(Math.Abs(ds.v.Wz - 1.0) < 0.05, $"diff spin {ds.v}");
    // 4 a 2x2 parallel conventional array (skid steer) turns only by scrub: spin rate well below command, but nonzero
    var skid = new List<WheelUnit> { Wh("fl", WheelKind.Conventional, 0.1, 0.12), Wh("fr", WheelKind.Conventional, 0.1, -0.12), Wh("rl", WheelKind.Conventional, -0.1, 0.12), Wh("rr", WheelKind.Conventional, -0.1, -0.12) };
    var ss = Sim(new TractivePlant(skid, Tp(8)), new Twist(0, 0, 1.0), 2.0);
    Check(ss.v.Wz > 0.05 && ss.v.Wz < 0.95, $"skid-steer spins by scrub only partially {ss.v}");
    // 5 dead_free vs dead_locked mecanum (fl): strafe; locked drags, free does not stall the strafe as much
    var pf = new TractivePlant(Mec(), Tp(8)); pf.Health[0] = WheelHealth.DeadFree;
    var pl = new TractivePlant(Mec(), Tp(8)); pl.Health[0] = WheelHealth.DeadLocked;
    var rf = Sim(pf, new Twist(0, 0.3, 0), 2.0); var rl = Sim(pl, new Twist(0, 0.3, 0), 2.0);
    Check(Math.Abs(rf.v.Wz) + Math.Abs(rf.v.Vx) < Math.Abs(rl.v.Wz) + Math.Abs(rl.v.Vx) || Math.Abs(rf.v.Vy - 0.3) < Math.Abs(rl.v.Vy - 0.3),
      $"dead_free {rf.v} vs dead_locked {rl.v} strafes differ (locked worse)");
    // 6 weak wheel: weak(0.3) on all wheels caps speed at 0.3 x wheel limit
    var pw = new TractivePlant(Mec(), Tp(8)); for (int i = 0; i < 4; i++) { pw.Health[i] = WheelHealth.Weak; pw.Gain[i] = 0.3; }
    var rw = Sim(pw, new Twist(0.5, 0, 0), 2.0);
    Check(rw.v.Vx < 0.3 * 15 * 0.04 + 0.01 && rw.v.Vx > 0.1, $"weak(0.3) forward capped {rw.v}");
    // 7 casters only: no force at rest, small rolling drag when moving
    var cas = new TractivePlant(new List<WheelUnit> { Wh("a", WheelKind.Caster, 0.1, 0.1), Wh("b", WheelKind.Caster, -0.1, 0.1), Wh("c", WheelKind.Caster, 0, -0.1) }, Tp(3));
    cas.Step(new Twist(), new Twist(), 0.005); double f0 = 0; for (int i = 0; i < 3; i++) f0 += Math.Abs(cas.Fx[i]) + Math.Abs(cas.Fy[i]);
    cas.Step(new Twist(), new Twist(0.3, 0, 0), 0.005); double fm = 0; for (int i = 0; i < 3; i++) fm += cas.Fx[i];
    Check(f0 < 1e-12 && fm < 0 && fm > -0.01 * 3 * 9.81 * 1.01, $"casters: rest force {f0:e2}, rolling drag {fm:0.000} N");
    // 8 load split sums to weight; determinate 3-wheel split
    double sn = 0; for (int i = 0; i < 3; i++) sn += cas.NormalLoad[i];
    Check(Math.Abs(sn - 3 * 9.81) < 1e-6, $"loads sum to weight {sn:0.0000}");
    // 9 determinism
    var a1 = Sim(new TractivePlant(Mec(), Tp(8)), new Twist(0.2, 0.1, 0.3), 1.0); var a2 = Sim(new TractivePlant(Mec(), Tp(8)), new Twist(0.2, 0.1, 0.3), 1.0);
    Check(a1.x == a2.x && a1.yaw == a2.yaw, "bit determinism");
    // 10 friction limit: low mu, high accel demand -> traction scale < 1 reported and acceleration bounded by mu g
    var lo = new TractivePlant(Mec(), Tp(8, 0.1)); lo.Step(new Twist(0.5, 0, 0), new Twist(), 0.005);
    Check(lo.Ax <= 0.1 * 9.81 * 1.05, $"low mu accel {lo.Ax:0.000} <= mu g");
    // 11 linear DC motor curve (v1 datasheet motor): available torque falls to 0 at the no-load speed, so a full-speed command settles
    // below no-load (rolling resistance needs torque) while the clamp model reaches it; stall torque available from rest
    var v1w = new List<WheelUnit> { Wh("fl", WheelKind.Mecanum, 0.109, 0.104, -1), Wh("fr", WheelKind.Mecanum, 0.109, -0.104, 1), Wh("rl", WheelKind.Mecanum, -0.109, 0.104, 1), Wh("rr", WheelKind.Mecanum, -0.109, -0.104, -1) };
    foreach (var u in v1w) { u.MaxSpeed = 23; u.MaxTorque = 1.47; }
    TractionParams V1(string curve) { var t = Tp(5.26); t.MotorCurve = curve; t.MotorGain = 5.0; t.RollingResistance = 0.03; t.MaxSpeedXy = 2; return t; }
    var dcRun = Sim(new TractivePlant(v1w, V1("linear_dc")), new Twist(1.115, 0, 0), 4.0); var clRun = Sim(new TractivePlant(v1w, V1("clamp")), new Twist(1.115, 0, 0), 4.0);
    Check(dcRun.v.Vx < clRun.v.Vx - 0.01 && dcRun.v.Vx < 23 * 0.04 && dcRun.v.Vx > 0.5, $"linear_dc full-speed command settles below no-load ({dcRun.v.Vx:0.000} m/s) and below clamp ({clRun.v.Vx:0.000} m/s)");
    var dc0 = new TractivePlant(v1w, V1("linear_dc")); for (int i = 0; i < 40; i++) dc0.Step(new Twist(1.115, 0, 0), new Twist(), 0.005);
    double maxF = 0; for (int i = 0; i < 4; i++) maxF = Math.Max(maxF, Math.Abs(dc0.Fx[i]));
    Check(maxF > 0.5 * 1.47 / 0.04 / Math.Sqrt(2) || maxF > 0.9 * 0.9 * 5.26 * 9.81 / 4, $"linear_dc from rest: wheel force {maxF:0.0} N (stall- or traction-limited)");
    Console.WriteLine(fails == 0 ? "ALL PASS" : $"{fails} FAIL"); Environment.Exit(fails);
  }
}
