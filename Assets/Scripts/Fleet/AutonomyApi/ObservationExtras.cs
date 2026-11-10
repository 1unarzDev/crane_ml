using System.Collections.Generic;

namespace Crane.Fleet.Autonomy {
    /// <summary>Wheel set-point (what the drive was told, after limits) and measured rate (from the quantised encoder counts) per wheel,
    /// at the encoder rate. A stall shows as commanded >> measured on a driven wheel; a body pushed while the wheels are held shows as
    /// IMU acceleration with measured rates near zero.</summary>
    public struct WheelRateSample {
        public double Stamp;
        public double[] Commanded, Measured;   // rad/s, fl fr rl rr
    }

    /// <summary>IMU linear acceleration in the body frame (x forward, y left), gravity removed, with sensor noise.</summary>
    public struct AccelSample {
        public double Stamp, Ax, Ay;
    }

    /// <summary>
    /// Extra rover-side channels CRANE provides for the autonomy library's stall/slip detection and support checks (phase 3). The library's
    /// IObservationSource stays unchanged; a source that also implements this interface offers the extras. Tag observations of decks
    /// (top tags), faces and payload end tags already arrive through IObservationSource.ReadTagObservations from every own camera
    /// (child "&lt;rover&gt;/base_link" or "&lt;object&gt;/base_link"); the payload's pose observation channel for the post-release support check is
    /// that same stream. Evaluator truth (stall/jam, contacts) is never offered here.
    /// </summary>
    public interface IObservationSourceExtras {
        void ReadWheelRates(List<WheelRateSample> into);
        void ReadAccel(List<AccelSample> into);
    }
}
