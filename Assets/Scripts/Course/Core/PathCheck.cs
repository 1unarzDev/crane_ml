using System;
using System.Collections.Generic;
using System.Linq;

namespace Sim.Course {
    /// <summary>Path rows (§3, D): can the boat get where a task needs it to? The boat is a
    /// disc of beam/2 + margin/2 (so a gate with clear ≥ beam + margin passes); every element
    /// is an obstacle; the boat stays inside the boundary. A straight segment is tried first,
    /// then A* on a 0.25 m grid. validate.py mirrors this exactly.</summary>
    public static class PathCheck {
        public const double Cell = 0.25, Step = 0.05, Window = 15.0;

        sealed class World {
            public List<JObj> Obstacles;
            public List<P2> Boundary;
            public double Inflate, Beam, Turn;

            public bool Free(P2 p) {
                if (Geometry.InsideMargin(Boundary, p) < Beam / 2) return false;
                foreach (var e in Obstacles) if (ConstraintEvaluator.Surface(e, p) <= Inflate) return false;
                return true;
            }

            public bool Segment(P2 a, P2 b) {
                int n = Math.Max(1, (int)Math.Ceiling(P2.Dist(a, b) / Step));
                for (int i = 0; i <= n; i++) if (!Free(a + (b - a) * ((double)i / n))) return false;
                return true;
            }
        }

        public static bool Run(JObj t, JObj a) {
            var vessel = VesselProfile.FromJson(t.Obj("vessel"));
            var w = new World {
                Obstacles = t.Arr("elements").Cast<JObj>()
                    .Where(e => !(a["ignore_roles"] is List<object> ignore && ignore.Contains(e.Str("role")))).ToList(),
                Boundary = ConstraintEvaluator.Points(t.Arr("boundary")),
                Beam = vessel.Beam, Turn = vessel.TurnRadius,
                Inflate = vessel.Beam / 2 + a.Num("margin", 0.3) / 2
            };
            switch (a.Str("kind")) {
                case "channel": return Channel(t, a, w);
                case "loop": return Loop(t, a, w);
                case "circle": return ConstraintEvaluator.Select(t, a).All(e => Circle(w, ConstraintEvaluator.Xy(e), w.Turn + w.Beam));
                case "slips": return Slips(t, a, w);
                case "anchors": return Anchors(t, a, w);
                default: throw new ArgumentException($"unknown path kind {a.Str("kind")}");
            }
        }

        /// <summary>P1: through every gate of the task in order (and so back out in reverse).</summary>
        static bool Channel(JObj t, JObj a, World w) {
            var gates = ConstraintEvaluator.Select(t, a).OrderBy(g => g.Num("order")).ToList();
            if (gates.Count == 0) return true;
            var pts = new List<P2> { ConstraintEvaluator.Xy(gates[0]) - P2.Dir(gates[0].Num("normal_yaw")) * 3 };
            pts.AddRange(gates.Select(ConstraintEvaluator.Xy));
            pts.Add(ConstraintEvaluator.Xy(gates[gates.Count - 1]) + P2.Dir(gates[gates.Count - 1].Num("normal_yaw")) * 3);
            return Chain(w, pts);
        }

        /// <summary>P2: gate → around the mark on a ring of max(turn radius, clearance) → gate.
        /// Paths are reversible, so this covers both circling directions.</summary>
        static bool Loop(JObj t, JObj a, World w) {
            JObj gate = ConstraintEvaluator.Select(t, a).FirstOrDefault();
            JObj mark = ConstraintEvaluator.Select(t, new JObj().Set("select", a.Obj("mark"))).FirstOrDefault();
            if (gate == null || mark == null) return false;
            P2 mid = ConstraintEvaluator.Xy(gate), c = ConstraintEvaluator.Xy(mark);
            double r = Math.Max(w.Turn, mark.Num("radius") + w.Inflate + 0.5);
            double start = Math.Atan2(mid.Y - c.Y, mid.X - c.X);
            var pts = new List<P2> { mid - P2.Dir(gate.Num("normal_yaw")) * 3, mid };
            for (int k = 0; k <= 8; k++) pts.Add(c + P2.Dir(start + 2 * Math.PI * k / 8) * r);
            pts.Add(mid);
            return Chain(w, pts);
        }

        /// <summary>P3: a full circle of radius turn radius + beam around a point is open water.</summary>
        static bool Circle(World w, P2 c, double r) {
            var pts = Enumerable.Range(0, 73).Select(k => c + P2.Dir(2 * Math.PI * k / 72) * r).ToList();
            for (int i = 0; i + 1 < pts.Count; i++) if (!w.Segment(pts[i], pts[i + 1])) return false;
            return true;
        }

        /// <summary>P4: each selected slip is entered straight from the aisle, and the aisle is
        /// reachable from the task's approach.</summary>
        static bool Slips(JObj t, JObj a, World w) {
            P2 approach = ConstraintEvaluator.Ref(t, a.Str("from")).At;
            foreach (var b in ConstraintEvaluator.Select(t, a)) {
                P2 center = ConstraintEvaluator.Xy(b);
                P2 aisle = center + P2.Dir(b.Num("yaw")) * (b.Num("depth") / 2 + 2);
                if (!w.Segment(aisle, center) || !Reachable(w, approach, aisle)) return false;
            }
            return true;
        }

        /// <summary>P5: from a start anchor to every task's named anchor.</summary>
        static bool Anchors(JObj t, JObj a, World w) {
            P2 from = ConstraintEvaluator.Ref(t, a.Str("from")).At;
            foreach (JObj task in t.Arr("tasks"))
                if (task.Obj("anchors")?.Obj(a.Str("anchor")) is JObj p && !Reachable(w, from, ConstraintEvaluator.Xy(p)))
                    return false;
            return true;
        }

        static bool Chain(World w, List<P2> pts) {
            for (int i = 0; i + 1 < pts.Count; i++) if (!Reachable(w, pts[i], pts[i + 1])) return false;
            return true;
        }

        static bool Reachable(World w, P2 a, P2 b) {
            if (!w.Free(a) || !w.Free(b)) return false;
            if (w.Segment(a, b)) return true;
            double x0 = Math.Min(a.X, b.X) - Window, y0 = Math.Min(a.Y, b.Y) - Window;
            int nx = (int)Math.Ceiling((Math.Abs(a.X - b.X) + 2 * Window) / Cell);
            int ny = (int)Math.Ceiling((Math.Abs(a.Y - b.Y) + 2 * Window) / Cell);
            // Cells are judged lazily (0 unknown, 1 free, 2 blocked); the answer is the same.
            var state = new byte[nx * ny];
            int si = (int)((a.X - x0) / Cell), sj = (int)((a.Y - y0) / Cell);
            int gi = (int)((b.X - x0) / Cell), gj = (int)((b.Y - y0) / Cell);
            state[si * ny + sj] = state[gi * ny + gj] = 1;
            bool Blocked(int i, int j) {
                int c = i * ny + j;
                if (state[c] == 0) state[c] = w.Free(new P2(x0 + (i + 0.5) * Cell, y0 + (j + 0.5) * Cell)) ? (byte)1 : (byte)2;
                return state[c] == 2;
            }
            // A* over 8-neighbours (no corner cutting), octile heuristic; only existence is used.
            var g = new double[nx * ny];
            for (int k = 0; k < g.Length; k++) g[k] = double.MaxValue;
            var heap = new List<(double f, int c)>();
            double H(int i, int j) {
                int dx = Math.Abs(i - gi), dy = Math.Abs(j - gj);
                return Math.Max(dx, dy) + (Math.Sqrt(2) - 1) * Math.Min(dx, dy);
            }
            g[si * ny + sj] = 0;
            Push(heap, (H(si, sj), si * ny + sj));
            while (heap.Count > 0) {
                var (f, c) = Pop(heap);
                int ci = c / ny, cj = c % ny;
                if (ci == gi && cj == gj) return true;
                if (f > g[c] + H(ci, cj) + 1e-9) continue;
                for (int di = -1; di <= 1; di++)
                    for (int dj = -1; dj <= 1; dj++) {
                        int ni = ci + di, nj = cj + dj;
                        if ((di == 0 && dj == 0) || ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
                        if (Blocked(ni, nj)) continue;
                        if (di != 0 && dj != 0 && (Blocked(ci, nj) || Blocked(ni, cj))) continue;
                        int n = ni * ny + nj;
                        double ng = g[c] + (di != 0 && dj != 0 ? Math.Sqrt(2) : 1);
                        if (ng >= g[n]) continue;
                        g[n] = ng;
                        Push(heap, (ng + H(ni, nj), n));
                    }
            }
            return false;
        }

        static void Push(List<(double f, int c)> h, (double f, int c) item) {
            h.Add(item);
            for (int i = h.Count - 1; i > 0;) {
                int p = (i - 1) / 2;
                if (h[p].f <= h[i].f) break;
                (h[p], h[i]) = (h[i], h[p]);
                i = p;
            }
        }

        static (double f, int c) Pop(List<(double f, int c)> h) {
            var top = h[0];
            h[0] = h[h.Count - 1];
            h.RemoveAt(h.Count - 1);
            for (int i = 0; ;) {
                int l = 2 * i + 1, r = l + 1, m = i;
                if (l < h.Count && h[l].f < h[m].f) m = l;
                if (r < h.Count && h[r].f < h[m].f) m = r;
                if (m == i) break;
                (h[m], h[i]) = (h[i], h[m]);
                i = m;
            }
            return top;
        }
    }
}
