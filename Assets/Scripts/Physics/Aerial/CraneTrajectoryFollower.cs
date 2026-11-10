using System;

namespace Sim.Physics.Aerial {
    /// <summary>
    /// Scripted trajectory for the demo director: straight segments between waypoints (world ENU),
    /// segment i flown at speeds[i] [m/s], speed changes rate-limited by MaxAccel, stopping at the
    /// last waypoint and holding it. Provides position, velocity and acceleration feed-forward plus
    /// yaw (along-track heading when FaceAlongTrack, else the yaw held at SetTrajectory).
    /// Pure and deterministic; sampled at controller ticks.
    /// </summary>
    public sealed class CraneTrajectoryFollower {
        public double MaxAccel = 2.5;          // [m/s^2] along-track
        public bool FaceAlongTrack = true;
        public bool Active { get; private set; }
        public bool Finished { get; private set; }
        private double[][] wp;
        private double[] speeds, segLen, cum;
        private double total, s, v, lastT = double.NaN, holdYaw;

        /// <param name="waypointsEnu">N &gt;= 2 points; the first should be the current position</param>
        /// <param name="segmentSpeeds">N-1 speeds (or 1, applied to all)</param>
        public void SetTrajectory(double[][] waypointsEnu, double[] segmentSpeeds, double initialYaw = 0.0) {
            if (waypointsEnu == null || waypointsEnu.Length < 2) throw new ArgumentException("need >= 2 waypoints");
            int m = waypointsEnu.Length - 1;
            if (segmentSpeeds == null || (segmentSpeeds.Length != m && segmentSpeeds.Length != 1))
                throw new ArgumentException("speeds must have N-1 entries or 1");
            wp = new double[waypointsEnu.Length][];
            for (int i = 0; i < wp.Length; i++) wp[i] = (double[])waypointsEnu[i].Clone();
            speeds = new double[m]; segLen = new double[m]; cum = new double[m + 1];
            for (int i = 0; i < m; i++) {
                speeds[i] = Math.Max(0.05, segmentSpeeds.Length == 1 ? segmentSpeeds[0] : segmentSpeeds[i]);
                segLen[i] = CraneRotorModelV2.Norm(CraneRotorModelV2.Sub(wp[i + 1], wp[i]));
                cum[i + 1] = cum[i] + segLen[i];
            }
            total = cum[m]; s = 0; v = 0; lastT = double.NaN; holdYaw = initialYaw;
            Active = true; Finished = false;
        }

        public void Stop() { Active = false; }

        private int Segment(double dist) {
            for (int i = 0; i < segLen.Length; i++) if (dist < cum[i + 1] || i == segLen.Length - 1) return i;
            return segLen.Length - 1;
        }

        public void Sample(double time, out double[] p, out double[] vel, out double[] acc, out double yaw) {
            double dt = double.IsNaN(lastT) ? 0.0 : Math.Max(0.0, time - lastT);
            lastT = time;
            int seg = Segment(s);
            double remaining = total - s;
            double vTarget = speeds[seg];
            double vStop = Math.Sqrt(2.0 * MaxAccel * Math.Max(0.0, remaining));   // brake to stop at the end
            vTarget = Math.Min(vTarget, vStop);
            double dv = Math.Max(-MaxAccel * dt, Math.Min(MaxAccel * dt, vTarget - v));
            double a = dt > 0 ? dv / dt : 0.0;
            v += dv;
            s = Math.Min(total, s + v * dt);
            if (s >= total - 1e-6 && v < 0.05) { s = total; v = 0; a = 0; Finished = true; }
            seg = Segment(s);
            double[] dir = segLen[seg] > 1e-9 ? CraneRotorModelV2.Scale(CraneRotorModelV2.Sub(wp[seg + 1], wp[seg]), 1.0 / segLen[seg])
                : new double[3];
            double u = segLen[seg] > 1e-9 ? (s - cum[seg]) / segLen[seg] : 1.0;
            p = new double[3];
            for (int i = 0; i < 3; i++) p[i] = wp[seg][i] + u * (wp[seg + 1][i] - wp[seg][i]);
            vel = CraneRotorModelV2.Scale(dir, v);
            acc = CraneRotorModelV2.Scale(dir, a);
            if (FaceAlongTrack && Math.Abs(dir[0]) + Math.Abs(dir[1]) > 1e-6) holdYaw = Math.Atan2(dir[1], dir[0]);
            yaw = holdYaw;
        }
    }
}
