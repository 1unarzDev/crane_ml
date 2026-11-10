// SPDX-License-Identifier: GPL-3.0-or-later
using System;
using System.Collections.Generic;
using Crane.Drones.Autonomy;

namespace Crane.Drones.Harness {
    /// <summary>In-memory event sink with optional streaming; the harness tap on IAutonomyRecorder.</summary>
    public sealed class RunRecorder : IAutonomyRecorder {
        public readonly List<(long t, string kind, Dictionary<string, object> f)> Events = new List<(long, string, Dictionary<string, object>)>();
        public string Drone = "d0";
        public void Event(long tNs, string kind, Dictionary<string, object> fields) => Events.Add((tNs, kind, fields));
        public Dictionary<string, object> Last(string kind) { for (int i = Events.Count - 1; i >= 0; i--) if (Events[i].kind == kind) return Events[i].f; return null; }
        public long LastT(string kind) { for (int i = Events.Count - 1; i >= 0; i--) if (Events[i].kind == kind) return Events[i].t; return -1; }
    }

    /// <summary>
    /// Pass-through IRayWorld that counts, per tick, how often sensor rays struck a watched set of capsules and how often
    /// the stereo model then saw through them (a re-cast with ignoreObject = that capsule is the dropout signature in
    /// DepthRaycastSensor). Results are identical to the wrapped world: it only observes.
    /// </summary>
    public sealed class ProbeRayWorld : IRayWorld {
        private readonly IRayWorld inner; public HashSet<int> Watch = new HashSet<int>();
        public long Tick; public double FirstViewS = double.PositiveInfinity, FirstDetectS = double.PositiveInfinity, LastDetectS = double.NegativeInfinity;
        public int DepthHits, DepthDrops, LidarHits; public double TickS;
        private int tickDepthHits, tickDepthDrops, tickLidarHits;
        public ProbeRayWorld(IRayWorld inner) { this.inner = inner; }
        public bool Raycast(Vec3 origin, Vec3 dirUnit, double maxRange, double beamHalfAngle, int ignoreObject, out RayHit hit) {
            bool r = inner.Raycast(origin, dirUnit, maxRange, beamHalfAngle, ignoreObject, out hit);
            if (Watch.Count == 0) return r;
            if (ignoreObject >= 0 && Watch.Contains(ignoreObject)) { DepthDrops++; tickDepthDrops++; }
            if (r && Watch.Contains(hit.ObjectId)) { if (beamHalfAngle > 0) { DepthHits++; tickDepthHits++; } else { LidarHits++; tickLidarHits++; } }
            return r;
        }
        /// <summary>Close the tick: a watched object was in view if any ray struck it; detected if a lidar return or a
        /// stereo hit that was not seen through.</summary>
        public void EndTick(double tS) {
            if (tickDepthHits + tickLidarHits > 0) FirstViewS = Math.Min(FirstViewS, tS);
            if (tickLidarHits > 0 || tickDepthHits > tickDepthDrops) { FirstDetectS = Math.Min(FirstDetectS, tS); LastDetectS = tS; }
            tickDepthHits = tickDepthDrops = tickLidarHits = 0;
        }
    }
}
