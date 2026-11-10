using System.Collections.Generic;

namespace Crane.Drones.Logging {
    /// <summary>
    /// Turns a per-tick nearest-obstacle distance into near-miss events: an event is emitted when the
    /// distance leaves the threshold zone, stamped at the exit time (events stay time-ordered) and carrying the minimum distance and t_min_ns.
    /// Flush() closes a still-open episode at the end of the run.
    /// </summary>
    public sealed class DroneClearanceTracker {
        public readonly double Threshold;
        private bool inside;
        private double min = double.PositiveInfinity;
        private long minT;
        private string minObstacle;

        public DroneClearanceTracker(double thresholdM = 0.5) { Threshold = thresholdM; }

        public void Update(long tNs, double distance, string obstacleId, DroneRunWriter w) {
            if (distance < Threshold) {
                inside = true;
                if (distance < min) { min = distance; minT = tNs; minObstacle = obstacleId; }
            }
            else if (inside) Emit(tNs, w);
        }

        public void Flush(long tNs, DroneRunWriter w) { if (inside) Emit(tNs, w); }

        private void Emit(long tNs, DroneRunWriter w) {
            w.WriteEvent(tNs, "near_miss", new Dictionary<string, object> {
                ["distance_m"] = min, ["t_min_ns"] = minT, ["obstacle"] = minObstacle, ["threshold_m"] = Threshold });
            inside = false; min = double.PositiveInfinity; minObstacle = null;
        }
    }
}
