using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>Static 2-D obstacle map: oriented boxes in the world frame plus rectangular bounds (the arena).</summary>
    public sealed class PlanMap
    {
        public double MinX, MinY, MaxX, MaxY;
        public readonly List<Box2> Obstacles = new List<Box2>();
        public PlanMap(double minX, double minY, double maxX, double maxY) { MinX = minX; MinY = minY; MaxX = maxX; MaxY = maxY; }

        public void AddBox(string id, double cx, double cy, double sx, double sy, double yaw = 0) => Obstacles.Add(new Box2(id, cx, cy, yaw, sx / 2, sy / 2));

        /// <summary>A wall along x = <paramref name="x"/> with a doorway centred at <paramref name="doorY"/> of width <paramref name="doorWidth"/>.</summary>
        public void AddWallWithDoor(string id, double x, double thickness, double doorY, double doorWidth)
        {
            double lo = doorY - doorWidth / 2, hi = doorY + doorWidth / 2;
            if (lo > MinY) AddBox(id + "_s", x, (MinY + lo) / 2, thickness, lo - MinY);
            if (hi < MaxY) AddBox(id + "_n", x, (hi + MaxY) / 2, thickness, MaxY - hi);
        }

        public bool InBounds(in Box2 b)
        {
            double c = Math.Abs(Math.Cos(b.Yaw)), s = Math.Abs(Math.Sin(b.Yaw));
            double ex = b.Hx * c + b.Hy * s, ey = b.Hx * s + b.Hy * c;
            return b.Cx - ex >= MinX && b.Cx + ex <= MaxX && b.Cy - ey >= MinY && b.Cy + ey <= MaxY;
        }

        /// <summary>True if footprint (assembly-frame boxes) at pose collides with any obstacle inflated by margin, an extra box, or leaves the bounds.</summary>
        public bool Collides(Pose2 pose, IReadOnlyList<Box2> footprint, double margin, IReadOnlyList<Box2> extra = null)
        {
            for (int i = 0; i < footprint.Count; i++)
            {
                var b = footprint[i].Transformed(pose);
                var bi = b.Inflated(margin);
                if (!InBounds(b)) return true;
                for (int k = 0; k < Obstacles.Count; k++) { var o = Obstacles[k]; if (Box2.Overlap(bi, o)) return true; }
                if (extra != null) for (int k = 0; k < extra.Count; k++) { var o = extra[k]; if (Box2.Overlap(bi, o)) return true; }
            }
            return false;
        }
    }

    public struct PathPoint
    {
        public Pose2 Pose;
        /// <summary>Planned body twist (assembly frame) on the segment leaving this point.</summary>
        public Twist2 Twist;
        public double T;     // planned time from start
        public double S;     // path length (m, translation only)
    }

    public sealed class PlannedPath
    {
        public bool Ok;
        public string Failure;              // null, "start_in_collision", "goal_in_collision", "no_path", "no_feasible_primitive"
        public readonly List<PathPoint> Points = new List<PathPoint>();
        public int Expansions;
        public double Duration => Points.Count == 0 ? 0 : Points[Points.Count - 1].T;
        public double Length => Points.Count == 0 ? 0 : Points[Points.Count - 1].S;
        public int CapabilityVersion;
        public Pose2 GoalUsed; public bool GoalAdjusted;
    }

    public sealed class PlanRequest
    {
        public Pose2 Start, Goal;
        public ICapability Capability;
        public double CruiseFraction = 0.6;
        public IReadOnlyList<Box2> ExtraObstacles;  // other rovers' estimated footprints (world)
        public double PosTol = double.NaN, YawTol = double.NaN;
        public double Margin = double.NaN;
        /// <summary>Map boxes (by id) that do not count as obstacles for this request, e.g. the object a lifter is about to grasp: its
        /// gripper envelope overlaps the object by design.</summary>
        public ICollection<string> IgnoreObstacleIds;
        /// <summary>If the goal is in collision, search nearby goals (backed off along the goal's -x, small lateral shifts and heading
        /// changes) before failing; the pose used is reported in <see cref="PlannedPath.GoalUsed"/>.</summary>
        public bool GoalSearch;
    }

    /// <summary>One motion primitive: a constant body twist held for a duration (an arc of the exponential map).</summary>
    public struct Primitive
    {
        public Twist2 Twist; public double Duration; public Pose2 Delta;
    }

    /// <summary>
    /// Hybrid-A* / state-lattice planner in SE(2). Its motion primitives are generated from the assembly's feasible twist set
    /// (ICapability: commandable subspace and wheel/body limits) and it collision-checks the assembly footprint, so one planner
    /// serves a single rover, a coupled line, a side-by-side pair (crabbing through a doorway when lateral twist is feasible)
    /// or an L, and a failure that removes a twist direction removes those primitives. Cost = planned time.
    /// </summary>
    public sealed class LatticePlanner
    {
        private readonly PlanMap _map;
        private readonly AutonomyAssumptions _a;
        public LatticePlanner(PlanMap map, AutonomyAssumptions a) { _map = map; _a = a; }

        /// <summary>Primitive set for a capability (exposed for tests: every primitive must be feasible and inside the subspace).</summary>
        public List<Primitive> Primitives(ICapability cap, double cruise)
        {
            double step = _a.PlanStep, dth = 2 * Math.PI / _a.PlanHeadings;
            var dirs = new List<Twist2>();
            for (int k = 0; k < 8; k++) { double ang = k * Math.PI / 4; dirs.Add(new Twist2(Math.Cos(ang), Math.Sin(ang), 0)); }
            dirs.Add(new Twist2(0, 0, 1)); dirs.Add(new Twist2(0, 0, -1));
            for (int k = 0; k < 4; k++)
            {
                double ang = k * Math.PI / 2;
                foreach (int sg in new[] { 1, -1 }) dirs.Add(new Twist2(Math.Cos(ang), Math.Sin(ang), sg * dth / step));
            }
            var cand = new List<Twist2>();
            if (cap.ActuatedRank >= 3) cand.AddRange(dirs);
            else
            {
                foreach (var d in dirs) cand.Add(cap.Project(d));
                if (cap.ActuatedRank == 2)
                {
                    var b0 = cap.Basis(0); var b1 = cap.Basis(1);
                    for (int k = 0; k < 16; k++) { double ang = k * Math.PI / 8; cand.Add(b0 * Math.Cos(ang) + b1 * Math.Sin(ang)); }
                }
                else if (cap.ActuatedRank == 1) { cand.Add(cap.Basis(0)); cand.Add(cap.Basis(0) * -1); }
            }
            var prims = new List<Primitive>();
            foreach (var u0 in cand)
            {
                double n = u0.Norm(1.0);
                if (n < 1e-6) continue;
                var u = u0 * (1.0 / n);
                bool dup = false;
                foreach (var p in prims)
                {
                    var t = p.Twist; double tn = t.Norm(1.0);
                    double cs = (t.Vx * u.Vx + t.Vy * u.Vy + t.Wz * u.Wz) / tn;
                    if (cs > 0.995) { dup = true; break; }
                }
                if (dup) continue;
                double smax = cap.MaxAlong(u);
                if (!(smax > 1e-4) || double.IsInfinity(smax)) continue;
                var xi = u * (smax * cruise);
                double lin = xi.LinearSpeed, w = Math.Abs(xi.Wz);
                double tau = Math.Max(lin * 1.0 / step, w / dth);
                if (tau <= 0) continue;
                double dur = 1.0 / tau;
                prims.Add(new Primitive { Twist = xi, Duration = dur, Delta = Se2.Exp(xi, dur) });
            }
            return prims;
        }

        // --------------------------------------------------------------------------------------------- search
        private struct Node { public Pose2 Pose; public double G; public int Parent; public int Prim; }

        public PlannedPath Plan(PlanRequest req)
        {
            if (req.IgnoreObstacleIds != null && req.IgnoreObstacleIds.Count > 0)
            {
                var m = new PlanMap(_map.MinX, _map.MinY, _map.MaxX, _map.MaxY);
                foreach (var o in _map.Obstacles) if (!req.IgnoreObstacleIds.Contains(o.Id)) m.Obstacles.Add(o);
                var sub = new PlanRequest { Start = req.Start, Goal = req.Goal, Capability = req.Capability, CruiseFraction = req.CruiseFraction, ExtraObstacles = req.ExtraObstacles, PosTol = req.PosTol, YawTol = req.YawTol, Margin = req.Margin, GoalSearch = req.GoalSearch };
                return new LatticePlanner(m, _a).Plan(sub);
            }
            if (req.GoalSearch)
            {
                double margin0 = double.IsNaN(req.Margin) ? _a.PlanMargin : req.Margin;
                if (req.Capability.ActuatedRank < 3) margin0 += _a.ReducedMobilityExtraMargin;
                if (_map.Collides(req.Goal, req.Capability.Footprint, margin0, req.ExtraObstacles))
                {
                    var best = FindFreeGoal(req.Goal, req.Capability.Footprint, margin0, req.ExtraObstacles, out bool found);
                    if (!found) return new PlannedPath { Failure = "goal_in_collision", CapabilityVersion = req.Capability.Version, GoalUsed = req.Goal };
                    var adj = new PlanRequest { Start = req.Start, Goal = best, Capability = req.Capability, CruiseFraction = req.CruiseFraction, ExtraObstacles = req.ExtraObstacles, PosTol = req.PosTol, YawTol = req.YawTol, Margin = req.Margin };
                    var r = Plan(adj); r.GoalAdjusted = true; r.GoalUsed = best; return r;
                }
            }
            var res0 = PlanCore(req); res0.GoalUsed = req.Goal; return res0;
        }

        /// <summary>Nearest collision-free variant of a goal: back-offs along its -x (up to 0.5 m), lateral shifts (10 cm) and heading
        /// changes (0.3 rad), cheapest first (back-off counts 1, lateral 2, heading 0.3 per rad... as metres).</summary>
        public Pose2 FindFreeGoal(Pose2 goal, IReadOnlyList<Box2> fp, double margin, IReadOnlyList<Box2> extra, out bool found)
        {
            var cands = new List<(double cost, Pose2 p)>();
            for (int b = 0; b <= 10; b++)
                foreach (double lat in new[] { 0.0, 0.05, -0.05, 0.10, -0.10 })
                    foreach (double dy in new[] { 0.0, 0.15, -0.15, 0.3, -0.3 })
                    {
                        var p = Se2.Compose(goal, new Pose2(-0.05 * b, lat, dy));
                        cands.Add((0.05 * b + 2 * Math.Abs(lat) + 0.3 * Math.Abs(dy), p));
                    }
            cands.Sort((x, y) => x.cost.CompareTo(y.cost));
            foreach (var c in cands) if (!_map.Collides(c.p, fp, margin, extra)) { found = true; return c.p; }
            found = false; return goal;
        }

        private PlannedPath PlanCore(PlanRequest req)
        {
            var res = new PlannedPath { CapabilityVersion = req.Capability.Version };
            var cap = req.Capability;
            double margin = double.IsNaN(req.Margin) ? _a.PlanMargin : req.Margin;
            if (req.Capability.ActuatedRank < 3) margin += _a.ReducedMobilityExtraMargin;   // errors outside the commandable subspace cannot be corrected
            double posTol = double.IsNaN(req.PosTol) ? _a.PlanGoalPosTol : req.PosTol;
            double yawTol = double.IsNaN(req.YawTol) ? _a.PlanGoalYawTol : req.YawTol;
            var fp = cap.Footprint;
            if (_map.Collides(req.Start, fp, 0.0, req.ExtraObstacles)) { res.Failure = "start_in_collision"; return res; }
            if (_map.Collides(req.Goal, fp, margin, req.ExtraObstacles)) { res.Failure = "goal_in_collision"; return res; }
            var prims = Primitives(cap, req.CruiseFraction);
            if (prims.Count == 0) { res.Failure = "no_feasible_primitive"; return res; }
            int maxExp = _a.PlanMaxExpansions; double wH = 1.2;
            if (cap.ActuatedRank < 3) { posTol = Math.Max(posTol, 0.10); yawTol = Math.Max(yawTol, 0.25); maxExp = _a.PlanMaxExpansionsReduced; wH = 2.0; }

            double vmax = 1e-6, wmax = 1e-6;
            foreach (var p in prims) { vmax = Math.Max(vmax, p.Twist.LinearSpeed); wmax = Math.Max(wmax, Math.Abs(p.Twist.Wz)); }
            double rfoot = 0; foreach (var b in fp) rfoot = Math.Max(rfoot, Math.Sqrt(b.Cx * b.Cx + b.Cy * b.Cy) + b.Radius);
            double inscribed = double.PositiveInfinity; foreach (var b in fp) inscribed = Math.Min(inscribed, Math.Min(b.Hx, b.Hy));

            var heur = new GridHeuristic(_map, req.Goal, _a.PlanCell, inscribed + margin * 0.5);
            double cell = _a.PlanCell; int H = cap.ActuatedRank >= 3 ? _a.PlanHeadings : 2 * _a.PlanHeadings;
            int nx = (int)Math.Ceiling((_map.MaxX - _map.MinX) / cell) + 1, ny = (int)Math.Ceiling((_map.MaxY - _map.MinY) / cell) + 1;
            var best = new Dictionary<long, double>();
            var closed = new HashSet<long>();
            var nodes = new List<Node>(4096);
            var open = new MinHeap();
            long Key(Pose2 p)
            {
                int ix = (int)Math.Floor((p.X - _map.MinX) / cell), iy = (int)Math.Floor((p.Y - _map.MinY) / cell);
                int ih = (int)Math.Floor(Se2.Wrap(p.Yaw + Math.PI / H) / (2 * Math.PI) * H + H) % H;
                return ((long)ix * ny + iy) * H + ih;
            }
            double Hfun(Pose2 p)
            {
                double d = heur.Distance(p.X, p.Y);
                if (double.IsPositiveInfinity(d)) return double.PositiveInfinity;
                return Math.Max(d / vmax, Math.Abs(Se2.Wrap(req.Goal.Yaw - p.Yaw)) / wmax);
            }
            nodes.Add(new Node { Pose = req.Start, G = 0, Parent = -1, Prim = -1 });
            double h0 = Hfun(req.Start);
            if (double.IsPositiveInfinity(h0)) { res.Failure = "no_path"; return res; }
            open.Push(0, wH * h0);
            best[Key(req.Start)] = 0;
            int goalNode = -1; List<PathPoint> shot = null;
            int exp = 0;
            const double switchPenalty = 0.05;
            while (open.Count > 0 && exp < maxExp)
            {
                int ni = open.Pop();
                var nd = nodes[ni];
                long key = Key(nd.Pose);
                if (closed.Contains(key)) continue;
                closed.Add(key);
                exp++;
                double dg = Se2.Distance(nd.Pose, req.Goal), dy = Math.Abs(Se2.Wrap(req.Goal.Yaw - nd.Pose.Yaw));
                if (dg <= posTol && dy <= yawTol && cap.ActuatedRank < 3) { goalNode = ni; break; }
                if (dg < 1.2 && (exp % 5 == 1 || dg < 0.3))
                {
                    var s = TryShot(nd.Pose, req.Goal, cap, req.CruiseFraction, margin, fp, req.ExtraObstacles, rfoot);
                    if (s != null) { goalNode = ni; shot = s; break; }
                }
                for (int pi = 0; pi < prims.Count; pi++)
                {
                    var pr = prims[pi];
                    var np = Se2.Compose(nd.Pose, pr.Delta);
                    if (!SegmentFree(nd.Pose, pr, fp, margin, req.ExtraObstacles, rfoot)) continue;
                    double g = nd.G + pr.Duration + (pi != nd.Prim && nd.Prim >= 0 ? switchPenalty : 0);
                    long k2 = Key(np);
                    if (closed.Contains(k2)) continue;
                    if (best.TryGetValue(k2, out double bg) && bg <= g) continue;
                    double h = Hfun(np);
                    if (double.IsPositiveInfinity(h)) continue;
                    best[k2] = g;
                    nodes.Add(new Node { Pose = np, G = g, Parent = ni, Prim = pi });
                    open.Push(nodes.Count - 1, g + wH * h);
                }
            }
            res.Expansions = exp;
            if (goalNode < 0) { res.Failure = "no_path"; return res; }
            // reconstruct
            var chain = new List<int>();
            for (int i = goalNode; i >= 0; i = nodes[i].Parent) chain.Add(i);
            chain.Reverse();
            double t = 0, sAcc = 0;
            for (int c = 0; c < chain.Count; c++)
            {
                var nd = nodes[chain[c]];
                if (c + 1 < chain.Count)
                {
                    var nx2 = nodes[chain[c + 1]]; var pr = prims[nx2.Prim];
                    int sub = Math.Max(1, (int)Math.Ceiling(Math.Max(pr.Twist.LinearSpeed * pr.Duration, Math.Abs(pr.Twist.Wz) * pr.Duration * 0.3) / 0.03));
                    for (int k = 0; k < sub; k++)
                    {
                        double tk = pr.Duration * k / sub;
                        res.Points.Add(new PathPoint { Pose = Se2.Advance(nd.Pose, pr.Twist, tk), Twist = pr.Twist, T = t + tk, S = sAcc + pr.Twist.LinearSpeed * tk });
                    }
                    t += pr.Duration; sAcc += pr.Twist.LinearSpeed * pr.Duration;
                }
                else if (shot != null)
                {
                    foreach (var sp in shot) { var q = sp; q.T += t; q.S += sAcc; res.Points.Add(q); }
                }
                else res.Points.Add(new PathPoint { Pose = nd.Pose, Twist = Twist2.Zero, T = t, S = sAcc });
            }
            res.Ok = true;
            return res;
        }

        private bool SegmentFree(Pose2 from, in Primitive pr, IReadOnlyList<Box2> fp, double margin, IReadOnlyList<Box2> extra, double rfoot)
        {
            double travel = pr.Twist.LinearSpeed * pr.Duration + Math.Abs(pr.Twist.Wz) * pr.Duration * rfoot;
            int n = Math.Max(1, (int)Math.Ceiling(travel / 0.03));
            for (int k = 1; k <= n; k++)
                if (_map.Collides(Se2.Advance(from, pr.Twist, pr.Duration * k / n), fp, margin, extra)) return false;
            return true;
        }

        /// <summary>Analytic connection to the goal. Holonomic: straight line with linearly interpolated yaw (body twist varies but is
        /// always in the twist set). Reduced mobility: a single constant body twist (log map), accepted only if it is commandable.</summary>
        private List<PathPoint> TryShot(Pose2 from, Pose2 goal, ICapability cap, double cruise, double margin, IReadOnlyList<Box2> fp, IReadOnlyList<Box2> extra, double rfoot)
        {
            var d = Se2.Between(from, goal);
            double dist = Math.Sqrt(d.X * d.X + d.Y * d.Y), dyaw = Math.Abs(d.Yaw);
            var pts = new List<PathPoint>();
            if (cap.ActuatedRank >= 3)
            {
                int n = Math.Max(2, (int)Math.Ceiling(Math.Max(dist, dyaw * rfoot) / 0.03));
                double dx = goal.X - from.X, dy = goal.Y - from.Y, dth = Se2.Wrap(goal.Yaw - from.Yaw);
                // speed: the slowest direction met along the line, at cruise
                double vmin = double.PositiveInfinity, T = 0;
                var poses = new Pose2[n + 1];
                for (int k = 0; k <= n; k++) poses[k] = new Pose2(from.X + dx * k / n, from.Y + dy * k / n, Se2.Wrap(from.Yaw + dth * k / n));
                for (int k = 1; k <= n; k++) if (_map.Collides(poses[k], fp, margin, extra)) return null;
                for (int k = 0; k < n; k++)
                {
                    var tw = Se2.Log(Se2.Between(poses[k], poses[k + 1]));   // per-unit-time twist of this sub-step
                    double m = cap.MaxAlong(tw);
                    if (!(m > 0)) return null;
                    double dt = 1.0 / (m * cruise);
                    var xi = tw * (1.0 / dt);
                    pts.Add(new PathPoint { Pose = poses[k], Twist = xi, T = T, S = dist * k / n });
                    T += dt; vmin = Math.Min(vmin, m);
                }
                pts.Add(new PathPoint { Pose = goal, Twist = Twist2.Zero, T = T, S = dist });
                return pts;
            }
            else
            {
                var tw = Se2.Log(d);
                var proj = cap.Project(tw);
                if ((tw - proj).Norm(1.0) > 1e-3 * Math.Max(1e-6, tw.Norm(1.0)) || dist + dyaw < 1e-9) return null;
                double m = cap.MaxAlong(tw);
                if (!(m > 0)) return null;
                double dur = 1.0 / (m * cruise);
                var xi = tw * (1.0 / dur);
                var pr = new Primitive { Twist = xi, Duration = dur, Delta = d };
                if (!SegmentFree(from, pr, fp, margin, extra, rfoot)) return null;
                int n = Math.Max(1, (int)Math.Ceiling(Math.Max(dist, dyaw * 0.3) / 0.03));
                for (int k = 0; k < n; k++) pts.Add(new PathPoint { Pose = Se2.Advance(from, xi, dur * k / n), Twist = xi, T = dur * k / n, S = dist * k / n });
                pts.Add(new PathPoint { Pose = goal, Twist = Twist2.Zero, T = dur, S = dist });
                return pts;
            }
        }

        // --------------------------------------------------------------------------------------------- helpers
        private sealed class MinHeap
        {
            private readonly List<int> _id = new List<int>(); private readonly List<double> _k = new List<double>();
            public int Count => _id.Count;
            public void Push(int id, double key)
            {
                _id.Add(id); _k.Add(key); int i = _id.Count - 1;
                while (i > 0) { int p = (i - 1) / 2; if (_k[p] < _k[i] || (_k[p] == _k[i] && _id[p] < _id[i])) break; Swap(i, p); i = p; }
            }
            public int Pop()
            {
                int top = _id[0]; int last = _id.Count - 1;
                _id[0] = _id[last]; _k[0] = _k[last]; _id.RemoveAt(last); _k.RemoveAt(last);
                int i = 0;
                while (true)
                {
                    int l = 2 * i + 1, r = l + 1, m = i;
                    if (l < _id.Count && Less(l, m)) m = l;
                    if (r < _id.Count && Less(r, m)) m = r;
                    if (m == i) break; Swap(i, m); i = m;
                }
                return top;
            }
            private bool Less(int a, int b) => _k[a] < _k[b] || (_k[a] == _k[b] && _id[a] < _id[b]);
            private void Swap(int a, int b) { int t = _id[a]; _id[a] = _id[b]; _id[b] = t; double k = _k[a]; _k[a] = _k[b]; _k[b] = k; }
        }

        /// <summary>8-connected Dijkstra distance-to-goal on a grid whose cells are blocked when within the footprint's inscribed radius of an obstacle.</summary>
        private sealed class GridHeuristic
        {
            private readonly double[] _d; private readonly int _nx, _ny; private readonly double _cell, _x0, _y0;
            public GridHeuristic(PlanMap map, Pose2 goal, double cell, double inflate)
            {
                _cell = cell; _x0 = map.MinX; _y0 = map.MinY;
                _nx = (int)Math.Ceiling((map.MaxX - map.MinX) / cell); _ny = (int)Math.Ceiling((map.MaxY - map.MinY) / cell);
                var blocked = new bool[_nx * _ny];
                for (int ix = 0; ix < _nx; ix++)
                    for (int iy = 0; iy < _ny; iy++)
                    {
                        double x = _x0 + (ix + 0.5) * cell, y = _y0 + (iy + 0.5) * cell;
                        var probe = new Box2("p", x, y, 0, 1e-4, 1e-4);
                        foreach (var o in map.Obstacles) if (Box2.Overlap(probe, o.Inflated(inflate))) { blocked[ix * _ny + iy] = true; break; }
                    }
                _d = new double[_nx * _ny];
                for (int i = 0; i < _d.Length; i++) _d[i] = double.PositiveInfinity;
                int gx = Clamp((int)((goal.X - _x0) / cell), _nx), gy = Clamp((int)((goal.Y - _y0) / cell), _ny);
                var heap = new MinHeap();
                _d[gx * _ny + gy] = 0; heap.Push(gx * _ny + gy, 0);
                var done = new bool[_nx * _ny];
                while (heap.Count > 0)
                {
                    int c = heap.Pop(); if (done[c]) continue; done[c] = true;
                    int cx = c / _ny, cy = c % _ny;
                    for (int ox = -1; ox <= 1; ox++)
                        for (int oy = -1; oy <= 1; oy++)
                        {
                            if (ox == 0 && oy == 0) continue;
                            int x2 = cx + ox, y2 = cy + oy;
                            if (x2 < 0 || y2 < 0 || x2 >= _nx || y2 >= _ny) continue;
                            int n = x2 * _ny + y2;
                            if (blocked[n] || done[n]) continue;
                            double nd = _d[c] + cell * (ox != 0 && oy != 0 ? 1.41421356 : 1.0);
                            if (nd < _d[n]) { _d[n] = nd; heap.Push(n, nd); }
                        }
                }
            }
            private static int Clamp(int v, int n) => v < 0 ? 0 : v >= n ? n - 1 : v;
            public double Distance(double x, double y)
            {
                int ix = Clamp((int)((x - _x0) / _cell), _nx), iy = Clamp((int)((y - _y0) / _cell), _ny);
                double d = _d[ix * _ny + iy];
                if (double.IsPositiveInfinity(d))
                {
                    // the pose may sit in an inflated cell next to free space (e.g. inside a doorway): use the best neighbour
                    for (int ox = -1; ox <= 1; ox++) for (int oy = -1; oy <= 1; oy++)
                    {
                        int x2 = ix + ox, y2 = iy + oy;
                        if (x2 < 0 || y2 < 0 || x2 >= _nx || y2 >= _ny) continue;
                        d = Math.Min(d, _d[x2 * _ny + y2] + _cell * 1.5);
                    }
                    if (double.IsPositiveInfinity(d)) d = 1e3; // keep searchable; the collision check is authoritative
                }
                return d;
            }
        }
    }
}
