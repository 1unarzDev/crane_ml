using System; using System.Collections.Generic; using Crane.Fleet;
// Statics of the CAD v1 side-on handoff (iteration 7): box_small 2 kg placed with its centre 0.055 m toward the lifter side of cargo_1's deck.
// Numbers from the v1 spec through FleetSpec (masses, CoM, wheel positions, deck height, payload size); library acceleration limit 0.5 m/s^2.
int fails = 0;
void Check(bool ok, string what) { Console.WriteLine((ok ? "PASS " : "FAIL ") + what); if (!ok) fails++; }
var spec = FleetSpec.LoadDirectory("/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet", FleetSpec.DefaultScenario, "v1");
var c1 = spec.Describe("cargo_1", "cargo");
double g = spec.Gravity, deck = spec.P["roles.cargo.deck_height"].Double;
double[] box = spec.P["payloads.box_small.size_lwh"].Doubles; double boxM = spec.P["payloads.box_small.mass"].Double;
double accel = 0.5, offY = 0.055;
List<(double, double)> Wheels(RoverDescription d, double ox, double oy) { var l = new List<(double, double)>(); foreach (var w in d.Wheels) l.Add((ox + w.Axle.X, oy + w.Axle.Y)); return l; }
LoadStatics.Mass Body(RoverDescription d, double ox, double oy) => new LoadStatics.Mass(d.Mass, ox + d.ComOffset[0], oy + d.ComOffset[1], d.ComOffset[2] + d.OriginHeight);
var payload = new LoadStatics.Mass(boxM, 0, offY, deck + box[2] / 2);
// one deck
var single = LoadStatics.Evaluate(Wheels(c1, 0, 0), new[] { Body(c1, 0, 0), payload }, accel, g);
Console.WriteLine($"single: CoM ({single.ComX:0.0000}, {single.ComY:0.0000}, {single.ComZ:0.000}) margin {single.MarginStatic * 1000:0.0} mm rel {single.RelStatic:P0}, at {accel} m/s^2 rel {single.RelAtAccel:P0}, tip accel {single.TipAccel:0.00} m/s^2");
Check(single.RelAtAccel > 0.2, "single deck: margin under the 0.5 m/s^2 limit above 20 %");
// inline pair: cargo_2 front mated to cargo_1 rear (faces +-0.171 in v1)
double face = spec.P["docking.face_frames.front"].Doubles[0];
var c2 = spec.Describe("cargo_2", "cargo");
var contacts = Wheels(c1, 0, 0); contacts.AddRange(Wheels(c2, -2 * face, 0));
var pair = LoadStatics.Evaluate(contacts, new[] { Body(c1, 0, 0), Body(c2, -2 * face, 0), payload }, accel, g);
Console.WriteLine($"pair:   CoM ({pair.ComX:0.0000}, {pair.ComY:0.0000}, {pair.ComZ:0.000}) margin {pair.MarginStatic * 1000:0.0} mm rel {pair.RelStatic:P0}, at {accel} m/s^2 rel {pair.RelAtAccel:P0}, tip accel {pair.TipAccel:0.00} m/s^2");
Check(pair.RelAtAccel > 0.2, "inline pair: margin under the 0.5 m/s^2 limit above 20 %");
// v0 reference (informational): box_long 6 kg on the v0 inline pair, centre 0.17 m behind cargo_1's centre (mission v2 place_offset)
var s0 = FleetSpec.LoadDirectory("/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet");
var a1 = s0.Describe("cargo_1", "cargo"); var a2 = s0.Describe("cargo_2", "cargo");
double f0 = s0.P["docking.face_frames.front"].Doubles[0], d0 = s0.P["roles.cargo.deck_height"].Double; double[] bl = s0.P["payloads.box_long.size_lwh"].Doubles;
var c0 = Wheels(a1, 0, 0); c0.AddRange(Wheels(a2, -2 * f0, 0));
var v0 = LoadStatics.Evaluate(c0, new[] { Body(a1, 0, 0), Body(a2, -2 * f0, 0), new LoadStatics.Mass(s0.P["payloads.box_long.mass"].Double, -0.17, 0, d0 + bl[2] / 2) }, accel, g);
Console.WriteLine($"v0 pair box_long (info): rel {v0.RelStatic:P0}, at {accel} m/s^2 rel {v0.RelAtAccel:P0}, tip accel {v0.TipAccel:0.00} m/s^2");
// sanity: centred load, symmetric rover -> rel ~ 1; load beyond the wheel line -> negative
var centred = LoadStatics.Evaluate(new List<(double, double)> { (1, 1), (1, -1), (-1, 1), (-1, -1) }, new[] { new LoadStatics.Mass(1, 0, 0, 0.5) }, 0, 9.81);
Check(Math.Abs(centred.RelStatic - 1) < 1e-12 && Math.Abs(centred.TipAccel - 9.81 * 1 / 0.5) < 1e-9, "square support, centred CoM: rel 1, tip accel g d / h");
var outside = LoadStatics.Evaluate(new List<(double, double)> { (1, 1), (1, -1), (-1, 1), (-1, -1) }, new[] { new LoadStatics.Mass(1, 0, 1.2, 0.5) }, 0, 9.81);
Check(outside.RelStatic < 0, "CoM outside the support polygon: negative margin");
Console.WriteLine(fails == 0 ? "ALL PASS" : fails + " FAIL"); Environment.Exit(fails);
