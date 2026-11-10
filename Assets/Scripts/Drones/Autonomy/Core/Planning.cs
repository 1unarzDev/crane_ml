// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using Sim.Physics.Aerial;

namespace Crane.Drones.Autonomy {
    /// <summary>
    /// Stopping rule (NavPilot stacks/core.py StopRule): d_free &gt;= v tau_age + D_brake(v) + margin, with tau_age the
    /// rolling p99 of measured perception-to-command age (prior TauPrior until 20 samples) and D_brake(v) a table from a
    /// closed-loop simulation of the REAL plant and controller (zero-velocity command from steady cruise), so controller
    /// lag, tilt and rotor limits are included.
    /// </summary>
    public sealed class StopRule {
        public readonly double[] Vs, Ds;
        public double Tau, TauPrior, Margin;
        private readonly int window; private readonly double q;
        private readonly List<double> ages = new List<double>();

        public StopRule(double[] vs, double[] ds, double tauPrior, double margin, int window = 200, double q = 0.99) {
            Vs = vs; Ds = ds; Tau = TauPrior = tauPrior; Margin = margin; this.window = window; this.q = q;
        }

        public void ObserveAge(double age) {
            ages.Add(age); if (ages.Count > window) ages.RemoveAt(0);
            if (ages.Count >= 20) {
                var s = ages.ToArray(); Array.Sort(s);
                double pos = q * (s.Length - 1); int i = (int)Math.Floor(pos); double u = pos - i;
                double qv = i + 1 < s.Length ? s[i] * (1 - u) + s[i + 1] * u : s[i];
                Tau = Math.Max(TauPrior * 0.5, qv);
            }
        }

        public double Brake(double v) {
            if (v <= Vs[0]) return Ds[0];
            for (int i = 1; i < Vs.Length; i++) if (v <= Vs[i]) { double u = (v - Vs[i - 1]) / (Vs[i] - Vs[i - 1]); return Ds[i - 1] + u * (Ds[i] - Ds[i - 1]); }
            int n = Vs.Length; double slope = (Ds[n - 1] - Ds[n - 2]) / (Vs[n - 1] - Vs[n - 2]);
            return Ds[n - 1] + slope * (v - Vs[n - 1]);
        }

        public double Dist(double v, double extraMargin = 0) => v * Tau + Brake(v) + Margin + extraMargin;

        /// <summary>Brake table from the real plant: hover, track v along +x for 3 s, then command zero velocity; distance
        /// until |v| &lt; 0.05 m/s. Kinematic controller gains are used as-is (no force gain is touched).</summary>
        public static (double[] vs, double[] ds) SimulateBrakeTable(VehicleSpecV2 spec, double vmax = 12.0, double dv = 0.5) {
            int n = (int)Math.Round(vmax / dv) + 1; var vs = new double[n]; var ds = new double[n];
            for (int k = 0; k < n; k++) {
                double v0 = k * dv; vs[k] = v0; if (v0 <= 0) continue;
                var plant = new CraneAerialPlantCore(spec, 1000.0, 500.0);
                var ctl = new CraneMultirotorController(spec, plant.ControlDt); plant.CommandSource = ctl;
                var s0 = CraneAerialPlantCore.HoverState(spec, new[] { 0.0, 0.0, 50.0 }, 0.0);
                plant.Reset(s0, 1); ctl.ResetIntegrators();
                ctl.SetVelocity(new[] { v0, 0.0, 0.0 }, 0.0);
                for (int i = 0; i < 600; i++) plant.Advance(0.005);
                double x0 = plant.State.P[0];
                ctl.SetVelocity(new[] { 0.0, 0.0, 0.0 }, 0.0);
                for (int i = 0; i < 2000; i++) { plant.Advance(0.005); var V = plant.State.V; if (Math.Sqrt(V[0] * V[0] + V[1] * V[1]) < 0.05) break; }
                ds[k] = plant.State.P[0] - x0;
            }
            return (vs, ds);
        }
    }

    /// <summary>
    /// Global route (M4): mission corridor waypoints (global frame) mapped into the local frame with the believed
    /// alignment, plus a 2D wavefront cost-to-go (NavPilot cost_to_go, 0.5 m, 8-connected Dijkstra) toward the goal over
    /// the route's bounding region. Blocked = window voxels occupied inside the flight band around the route altitude,
    /// and (PriorUse route|belief) prior columns. Unknown is traversable (optimistic), as NavPilot.
    /// </summary>
    public sealed class CorridorRouter : IGlobalRouter {
        private readonly AutonomyConfig c; private readonly List<Vec3> routeGlobal; private readonly IPriorMap prior;
        private List<Vec3> routeLocal = new List<Vec3>();
        private double x0, y0, res; private int nx, ny; private double[] C; private bool[] blocked;
        private long lastNs; private bool fresh = true;
        public PlanningWindow Window;   // set by the stack before Update
        public IReadOnlyList<Vec3> Route => routeLocal;
        public Vec3 GoalLocal { get; private set; }
        public int Recomputes { get; private set; }

        public CorridorRouter(AutonomyConfig cfg, IList<Vec3> routeGlobal, IPriorMap prior) {
            c = cfg; this.routeGlobal = new List<Vec3>(routeGlobal); this.prior = prior; res = cfg.CtgRes;
            double ax = double.MaxValue, bx = double.MinValue, ay = double.MaxValue, by = double.MinValue;
            foreach (var p in routeGlobal) { ax = Math.Min(ax, p.X); bx = Math.Max(bx, p.X); ay = Math.Min(ay, p.Y); by = Math.Max(by, p.Y); }
            x0 = ax - 20; y0 = ay - 20; nx = (int)Math.Ceiling((bx - ax + 40) / res); ny = (int)Math.Ceiling((by - ay + 40) / res);
            C = new double[nx * ny]; blocked = new bool[nx * ny];
        }

        public void ForceRecompute() => fresh = true;

        /// <summary>Route altitude (local frame) at the projection of p onto the route polyline, and the along-route arc.</summary>
        public double RouteAltitude(Vec3 p, out double arc, out double lateral) {
            double best = double.MaxValue, z = p.Z, acc = 0; arc = 0; lateral = 0;
            for (int i = 0; i + 1 < routeLocal.Count; i++) {
                var a = routeLocal[i]; var b = routeLocal[i + 1]; var ab = b - a; double L2 = ab.X * ab.X + ab.Y * ab.Y; double L = Math.Sqrt(L2);
                double u = L2 > 1e-9 ? Math.Max(0, Math.Min(1, ((p.X - a.X) * ab.X + (p.Y - a.Y) * ab.Y) / L2)) : 0;
                double qx = a.X + u * ab.X - p.X, qy = a.Y + u * ab.Y - p.Y; double d = Math.Sqrt(qx * qx + qy * qy);
                if (d < best) { best = d; z = a.Z + u * ab.Z; arc = acc + u * L; lateral = d; }
                acc += L;
            }
            return z;
        }

        public double RouteLength { get { double s = 0; for (int i = 0; i + 1 < routeGlobal.Count; i++) s += (routeGlobal[i + 1] - routeGlobal[i]).NormXY; return s; } }

        public void Update(long tNs, StateEstimate est, MapAlignment align, IOccupancyMap map) {
            routeLocal.Clear(); foreach (var g in routeGlobal) routeLocal.Add(g - align.Offset);
            GoalLocal = routeLocal[routeLocal.Count - 1];
            if (!fresh && tNs - lastNs < (long)(c.CtgPeriodS * 1e9)) return;
            lastNs = tNs; fresh = false; Recomputes++;
            var off = align.Offset; double lx0 = x0 - off.X, ly0 = y0 - off.Y;
            Array.Clear(blocked, 0, blocked.Length);
            if (prior != null && c.PriorUse != "none")
                for (int i = 0; i < nx; i++) for (int j = 0; j < ny; j++)
                    if (prior.BlockedColumn(x0 + (i + 0.5) * res, y0 + (j + 0.5) * res)) blocked[i * ny + j] = true;
            var W = Window;
            if (W != null) {
                for (int i = 0; i < W.Nx; i++) for (int j = 0; j < W.Ny; j++) {
                    var p0 = W.CellCenter(i, j, 0);
                    int ci = (int)Math.Floor((p0.X - lx0) / res), cj = (int)Math.Floor((p0.Y - ly0) / res);
                    if (ci < 0 || cj < 0 || ci >= nx || cj >= ny) continue;
                    double zr = RouteAltitude(p0, out _, out _);
                    for (int k = 0; k < W.Nz; k++) {
                        if (!W.Occ[(i * W.Ny + j) * W.Nz + k]) continue;
                        double z = W.CellCenter(i, j, k).Z;
                        if (z >= zr - c.CtgBandDown && z <= zr + c.CtgBandUp) { blocked[ci * ny + cj] = true; break; }
                    }
                }
            }
            // Dijkstra from the goal cell
            for (int q = 0; q < C.Length; q++) C[q] = double.PositiveInfinity;
            int gi = (int)Math.Floor((GoalLocal.X - lx0) / res), gj = (int)Math.Floor((GoalLocal.Y - ly0) / res);
            gi = Math.Max(0, Math.Min(nx - 1, gi)); gj = Math.Max(0, Math.Min(ny - 1, gj));
            var pq = new MinHeap(); C[gi * ny + gj] = 0; pq.Push(0, gi * ny + gj);
            double s2 = Math.Sqrt(2) * res;
            while (pq.Count > 0) {
                pq.Pop(out double cost, out int id);
                if (cost > C[id]) continue;
                int i = id / ny, j = id % ny;
                for (int di = -1; di <= 1; di++) for (int dj = -1; dj <= 1; dj++) {
                    if (di == 0 && dj == 0) continue;
                    int a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
                    int nid = a * ny + b; if (blocked[nid]) continue;
                    double nc = cost + (di != 0 && dj != 0 ? s2 : res);
                    if (nc < C[nid]) { C[nid] = nc; pq.Push(nc, nid); }
                }
            }
            ctgOffX = lx0; ctgOffY = ly0;
            FiniteCells = 0; BlockedCells = 0;
            for (int q = 0; q < C.Length; q++) { if (!double.IsInfinity(C[q])) FiniteCells++; if (blocked[q]) BlockedCells++; }
        }
        public int FiniteCells, BlockedCells;
        public int TotalCells => C.Length;
        private double ctgOffX, ctgOffY;

        public double CostToGo(double x, double y) {
            int i = (int)Math.Floor((x - ctgOffX) / res), j = (int)Math.Floor((y - ctgOffY) / res);
            if (i < 0 || j < 0 || i >= nx || j >= ny) return double.PositiveInfinity;
            return C[i * ny + j];
        }

        /// <summary>NavPilot _ctg_soft: blocked cells take (neighbour min + 1 m); enclosed -> large finite.</summary>
        public double CostToGoSoft(double x, double y) {
            double v = CostToGo(x, y); if (!double.IsInfinity(v)) return v;
            int i = (int)Math.Floor((x - ctgOffX) / res), j = (int)Math.Floor((y - ctgOffY) / res); double best = double.PositiveInfinity;
            for (int a = -1; a <= 1; a++) for (int b = -1; b <= 1; b++) {
                int p = i + a, q = j + b; if (p < 0 || q < 0 || p >= nx || q >= ny) continue; best = Math.Min(best, C[p * ny + q]);
            }
            return double.IsInfinity(best) ? 1e4 : best + 1.0;
        }

        sealed class MinHeap {
            private readonly List<(double k, int v)> h = new List<(double, int)>();
            public int Count => h.Count;
            public void Push(double k, int v) { h.Add((k, v)); int i = h.Count - 1; while (i > 0) { int p = (i - 1) / 2; if (h[p].k <= h[i].k) break; (h[p], h[i]) = (h[i], h[p]); i = p; } }
            public void Pop(out double k, out int v) {
                k = h[0].k; v = h[0].v; var last = h[h.Count - 1]; h.RemoveAt(h.Count - 1);
                if (h.Count == 0) return; h[0] = last; int i = 0;
                while (true) { int l = 2 * i + 1, r = l + 1, m = i; if (l < h.Count && h[l].k < h[m].k) m = l; if (r < h.Count && h[r].k < h[m].k) m = r; if (m == i) break; (h[m], h[i]) = (h[i], h[m]); i = m; }
            }
        }
    }

    /// <summary>
    /// MAP_RHP (NavPilot stacks/policies.py MAP_RHP + core.evaluate) ported to 3D, and REACTIVE.
    /// Primitives: Headings x ClimbDeg x SpeedFractions velocity commands, each forward-simulated through a first-order
    /// velocity loop (TrackTau, |a| &lt;= 0.8 x tilt-limited accel) for T = clip(H / max(v, 0.5), 2, 8) s. d_free = arc to
    /// the first blocked sample (ESDF distance &lt; r_body + inflate, unknown, outside the window, or outside the altitude
    /// band), extended straight to H if the rollout ends free. Feasible iff StopRule.Dist(max(v_cmd, |v|)) &lt;= d_free.
    /// Score (NavPilot): v g + 0.15 v_cap g - w_turn |dheading| |v| + w_free min(d, H)/H - w_alt |dz_end| / 2, with g =
    /// cost-to-go progress per metre at s_eval (MAP_RHP) or cos(goal bearing) (REACTIVE). No feasible primitive -> v_cmd 0.
    /// </summary>
    public sealed class PrimitivePlanner : ILocalPlanner {
        private readonly AutonomyConfig c; private readonly bool reactive; private readonly double aMax;
        public PlanningWindow Window;     // the belief this cycle (built by the stack)
        public CorridorRouter Router;
        public string Name => reactive ? "reactive" : "map_rhp";
        public double LastStartDist { get; private set; }

        public PrimitivePlanner(AutonomyConfig cfg, bool reactive, double aMaxHorizontal) { c = cfg; this.reactive = reactive; aMax = aMaxHorizontal; }

        /// <summary>Blocked test for a sample (local frame). nearStart: NavPilot inflation-blob exception.</summary>
        bool Blocked(Vec3 p, double rInfl, double startDist, bool nearStart, bool startUnknown, double zRef, double groundBand) {
            if (Math.Abs(p.Z - zRef) > groundBand) return true;   // groundBand already widened to the current altitude error
            if (!Window.Index(p, out int w)) return true;
            double dist = Window.Dist[w];
            if (!Window.Obs[w] && !(nearStart && startUnknown)) return true;
            if (dist < rInfl) {
                if (nearStart && startDist < rInfl) return dist < startDist - 0.01;
                return true;
            }
            return false;
        }

        public PlanDecision Plan(long tNs, StateEstimate est, Vec3 goal, double vCap, IOccupancyMap map, IGlobalRouter router, StopRule rule, double zTarget) {
            double H = reactive ? c.ReactiveHorizon : c.Horizon;
            double rInfl = c.BodyRadius + c.InflateExtra + 0.5 * Window.Res;   // + half voxel for quantization
            var p = est.P; var v = est.V; double sp = v.NormXY;
            double gth = Math.Atan2(goal.Y - p.Y, goal.X - p.X);
            double vth = sp > 0.2 ? Math.Atan2(v.Y, v.X) : gth;
            Window.Index(p, out int w0);
            double startDist = w0 >= 0 ? Window.Dist[w0] : 0; bool startUnknown = w0 >= 0 && !Window.Obs[w0];
            LastStartDist = startDist;
            double c0 = reactive ? 0 : Router.CostToGoSoft(p.X, p.Y);
            double band = Math.Max(2.0, Math.Abs(p.Z - zTarget) + 0.1);   // never forbid the altitude we are at; only moving further out
            int nH = c.Headings, nC = reactive ? 1 : c.ClimbDeg.Length, nS = c.SpeedFractions.Length;
            var dec = new PlanDecision { TNs = tNs, Planner = Name, Candidates = nH * nC * nS };
            double bestScore = double.NegativeInfinity, bestFallback = double.NegativeInfinity;
            int feasCount = 0;
            double bTh = gth, bClimb = 0, bV = 0, bD = 0, bG = 0, fTh = gth, fClimb = 0, fD = 0;
            const int steps = 60;
            for (int hi = 0; hi < nH; hi++) {
                double th = gth - Math.PI + 2 * Math.PI * hi / nH;
                for (int ci = 0; ci < nC; ci++) {
                    double climb = (reactive ? 0 : c.ClimbDeg[ci]) * Math.PI / 180;
                    // altitude-holding climb for reactive: vertical command toward zTarget
                    for (int si = 0; si < nS; si++) {
                        double spd = Math.Max(vCap, 0.3) * c.SpeedFractions[si];
                        var cmd = new Vec3(spd * Math.Cos(climb) * Math.Cos(th), spd * Math.Cos(climb) * Math.Sin(th), spd * Math.Sin(climb));
                        // altitude hold toward the route altitude on top of the primitive's climb (both kinematic, m/s)
                        cmd.Z = Math.Max(-1.5, Math.Min(1.5, cmd.Z + 0.8 * (zTarget - p.Z)));
                        double T = Math.Max(2.0, Math.Min(8.0, H / Math.Max(spd, 0.5))), dt = T / steps;
                        var q = p; var u = v; double arc = 0, dfree = -1, sEvalArc = -1; Vec3 qEval = p; Vec3 last = p; Vec3 lastDir = cmd.Normalized;
                        for (int k = 0; k < steps; k++) {
                            var a = (cmd - u) * (1.0 / c.TrackTau); double an = a.Norm;
                            if (an > aMax) a = a * (aMax / an);
                            u = u + a * dt; var dq = u * dt; q = q + dq; arc += dq.Norm;
                            if (dfree < 0 && Blocked(q, rInfl, startDist, arc < 0.6, startUnknown, zTarget, band)) dfree = Math.Max(0, arc - dq.Norm);
                            if (sEvalArc < 0 && arc >= Math.Min(c.SEval, H)) { sEvalArc = arc; qEval = q; }
                            last = q; if (dq.Norm > 1e-6) lastDir = dq.Normalized;
                            if (arc >= H) break;
                        }
                        if (dfree < 0) {
                            dfree = Math.Min(arc, H);
                            for (double e = 0.2; arc + e <= H + 1e-9; e += 0.2) {
                                var qq = last + lastDir * e;
                                if (Blocked(qq, rInfl, startDist, false, startUnknown, zTarget, band)) { dfree = arc + e - 0.2; break; }
                                dfree = arc + e;
                            }
                            dfree = Math.Min(dfree, H);
                        }
                        double se = Math.Max(0.3, Math.Min(Math.Min(dfree, c.SEval), arc));
                        if (sEvalArc < 0 || sEvalArc > se) { qEval = Interp(p, v, cmd, se, steps, T); sEvalArc = se; }
                        double g;
                        if (reactive) { double dg = Math.Abs(Wrap(th - gth)); g = Math.Cos(dg); }
                        else g = Math.Max(-1, Math.Min(1, (c0 - Router.CostToGoSoft(qEval.X, qEval.Y)) / Math.Max(sEvalArc, 0.3)));
                        double dv = Math.Abs(Wrap(th - vth));
                        double dzEnd = Math.Abs(last.Z - zTarget);
                        double score = spd * g + 0.15 * vCap * g - c.WTurn * dv * sp + c.WFree * Math.Min(dfree, H) / H - (reactive ? 0 : c.WAlt * dzEnd / 2.0);
                        double vRule = Math.Max(spd, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                        bool feas = rule.Dist(vRule) <= dfree;
                        if (feas) { feasCount++; if (score > bestScore) { bestScore = score; bTh = th; bClimb = climb; bV = spd; bD = dfree; bG = g; dec.VelocityCmd = cmd; } }
                        double fb = Math.Min(dfree, 3.0) + 0.5 * g;
                        if (fb > bestFallback) { bestFallback = fb; fTh = th; fClimb = climb; fD = dfree; }
                    }
                }
            }
            dec.FeasibleAny = feasCount > 0; dec.FeasibleCount = feasCount;
            if (dec.FeasibleAny) { dec.Heading = bTh; dec.Climb = bClimb; dec.VCmd = bV; dec.DFree = bD; dec.Progress = bG; dec.StopDist = rule.Dist(Math.Max(bV, sp)); }
            else { dec.Heading = fTh; dec.Climb = fClimb; dec.VCmd = 0; dec.DFree = fD; dec.VelocityCmd = new Vec3(); dec.Note = "no_feasible_primitive"; }
            dec.TauAge = rule.Tau;
            return dec;
        }

        Vec3 Interp(Vec3 p, Vec3 v, Vec3 cmd, double sTarget, int steps, double T) {
            double dt = T / steps, arc = 0; var q = p; var u = v;
            for (int k = 0; k < steps; k++) {
                var a = (cmd - u) * (1.0 / c.TrackTau); double an = a.Norm; if (an > aMax) a = a * (aMax / an);
                u = u + a * dt; var dq = u * dt; q = q + dq; arc += dq.Norm; if (arc >= sTarget) break;
            }
            return q;
        }

        public static double Wrap(double a) { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; }
    }
}
