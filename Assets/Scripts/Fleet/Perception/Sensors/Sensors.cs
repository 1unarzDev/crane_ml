using System;
using System.Collections.Generic;
using System.Linq;

namespace Crane.Fleet.Perception
{
    // ------------------------------------------------------------------ mounts per role
    public sealed class CameraMount3
    {
        public string Name; public string Facing; public Pose OpticalInRover; public CameraIntrinsics K; public double RateHz, LatencyS;
        public string Provenance = "provisional";
    }

    public sealed class RangeMount { public string Name; public Pose InRover; public double MaxRange, ConeHalfAngle, RateHz; }
    public sealed class LidarMount { public string Name; public Pose InRover; public double MaxRange, RateHz; public int Points; }

    /// <summary>Which sensors a rover has, with frames. Mount poses are PROVISIONAL (profile has no mounting poses yet: open question for CAD).</summary>
    public sealed class SensorSuite
    {
        public string RoverId, Role;
        public bool HasEncoders, HasImu;
        public List<CameraMount3> Cameras = new List<CameraMount3>();
        public List<RangeMount> Tofs = new List<RangeMount>();
        public LidarMount Lidar;
        public double StereoBaseline;

        public static SensorSuite ForRover(FleetProfile p, string roverId)
        {
            var m = p.Fleet.FirstOrDefault(f => f.Id == roverId) ?? throw new KeyNotFoundException("unknown rover id " + roverId);
            var role = p.Roles[m.Role];
            var s = new SensorSuite { RoverId = roverId, Role = m.Role, StereoBaseline = p.StereoBaseline };
            double bodyH = p.BodySizeLwh[2], halfL = p.BodySizeLwh[0] / 2;
            var K = p.Camera.Intrinsics;
            foreach (var t in role.Sensors)
            {
                switch (t.Kind)
                {
                    case SensorKind.WheelEncoder: s.HasEncoders = true; break;
                    case SensorKind.Imu: s.HasImu = true; break;
                    case SensorKind.Lidar2D:
                        s.Lidar = new LidarMount { Name = "lidar", InRover = Pose.FromYaw(0.0, 0.0, bodyH + role.MastHeight, 0), MaxRange = p.Lidar.MaxRange, RateHz = p.Lidar.RateHz, Points = p.Lidar.Points };
                        break;
                    case SensorKind.StereoPair:
                        foreach (int side in new[] { +1, -1 })
                            s.Cameras.Add(Cam(p, K, side > 0 ? "stereo_left" : "stereo_right", "front", new Vec3(halfL - 0.02, side * p.StereoBaseline / 2, bodyH + role.MastHeight), 0));
                        break;
                    case SensorKind.MonoCamera:
                        bool rear = t.Facing == "rear";
                        s.Cameras.Add(Cam(p, K, "cam_" + t.Facing, t.Facing, new Vec3((rear ? -1 : 1) * (halfL - 0.01), 0, role.MastHeight > 0 ? bodyH + role.MastHeight : bodyH * 0.8), rear ? Math.PI : 0));
                        break;
                    case SensorKind.RangeTof:
                        bool r2 = t.Facing == "rear";
                        s.Tofs.Add(new RangeMount { Name = "tof_" + t.Facing, InRover = Pose.FromYaw((r2 ? -1 : 1) * halfL, 0, 0.06, r2 ? Math.PI : 0), MaxRange = p.Tof.MaxRange, ConeHalfAngle = p.Tof.Fov / 2, RateHz = p.Tof.RateHz });
                        break;
                }
            }
            return s;
        }

        private static CameraMount3 Cam(FleetProfile p, CameraIntrinsics K, string name, string facing, Vec3 pos, double yaw) => new CameraMount3
        {
            Name = name, Facing = facing, OpticalInRover = CameraMount.InRover(pos, yaw), K = K, RateHz = p.Camera.RateHz, LatencyS = p.Camera.Latency,
            Provenance = p.Camera.Provenance["hfov"] == "measured" ? "measured" : "provisional"
        };
    }

    // ------------------------------------------------------------------ encoders
    /// <summary>Mecanum wheel order: FL, FR, RL, RR. Wheel angular speed from body twist (vx forward, vy left, wz yaw rate).</summary>
    public static class MecanumKinematics
    {
        public static double[] WheelSpeeds(double vx, double vy, double wz, double lx, double ly, double r)
        {
            double k = lx + ly;
            return new[] { (vx - vy - k * wz) / r, (vx + vy + k * wz) / r, (vx + vy - k * wz) / r, (vx - vy + k * wz) / r };
        }
        public static void BodyTwist(double[] w, double lx, double ly, double r, out double vx, out double vy, out double wz)
        {
            double k = lx + ly;
            vx = r * (w[0] + w[1] + w[2] + w[3]) / 4; vy = r * (-w[0] + w[1] + w[2] - w[3]) / 4; wz = r * (-w[0] + w[1] - w[2] + w[3]) / (4 * k);
        }
    }

    public sealed class EncoderReading { public double Stamp; public long[] Counts = new long[4]; public double[] RatesRadS = new double[4]; }

    /// <summary>
    /// Quantised incremental encoders. True wheel angle (rad) is supplied by the evaluator; the model adds per-sample slip noise on the
    /// wheel increment (slip_sigma_frac of the increment, floor-dependent) and quantises to counts. Rate = finite difference of counts.
    /// </summary>
    public sealed class EncoderModel
    {
        private readonly int _cpr; private readonly double _slipFrac; private readonly Rng _rng;
        private readonly double[] _trueAngle = new double[4]; private readonly double[] _slipAngle = new double[4]; private readonly long[] _prevCounts = new long[4]; private double _prevT = double.NaN;
        public EncoderModel(int countsPerRev, double slipSigmaFrac, ulong seed, string id) { _cpr = countsPerRev; _slipFrac = slipSigmaFrac; _rng = Rng.ForStream(seed, "enc/" + id); }
        public double RadPerCount => 2 * Math.PI / _cpr;

        /// <summary>Advance with the true wheel angles at time t (monotone calls) and read counts.</summary>
        public EncoderReading Sample(double t, double[] trueWheelAngle)
        {
            var r = new EncoderReading { Stamp = t };
            for (int i = 0; i < 4; i++)
            {
                double inc = trueWheelAngle[i] - _trueAngle[i];
                _trueAngle[i] = trueWheelAngle[i];
                _slipAngle[i] += inc * (1 + _slipFrac * _rng.Gaussian());
                r.Counts[i] = (long)Math.Floor(_slipAngle[i] / RadPerCount);
                r.RatesRadS[i] = double.IsNaN(_prevT) || t <= _prevT ? 0 : (r.Counts[i] - _prevCounts[i]) * RadPerCount / (t - _prevT);
                _prevCounts[i] = r.Counts[i];
            }
            _prevT = t; return r;
        }
    }

    // ------------------------------------------------------------------ IMU
    public sealed class ImuReading { public double Stamp; public double GyroZ; }

    /// <summary>
    /// Gyro-z: measurement = true rate + bias + white noise with sigma = noise_density / sqrt(dt); bias is a random walk with
    /// sigma_step = bias_instability * sqrt(dt). Profile "gyro_bias_instability" is read as the bias random-walk density (rad/s/sqrt(s)): ASSUMPTION.
    /// The true bias is internal state (<c>TrueBias</c>, assembly-internal, visible to tests only) and never part of a reading.
    /// </summary>
    public sealed class ImuModel
    {
        private readonly double _nd, _bi; private readonly Rng _rng; private double _bias; private double _t = double.NaN;
        public ImuModel(double noiseDensity, double biasInstability, ulong seed, string id, double initialBiasSigma = 0.0)
        { _nd = noiseDensity; _bi = biasInstability; _rng = Rng.ForStream(seed, "imu/" + id); _bias = initialBiasSigma * _rng.Gaussian(); }
        internal double TrueBias => _bias;
        public ImuReading Sample(double t, double trueYawRate)
        {
            double dt = double.IsNaN(_t) ? 0 : t - _t; _t = t;
            if (dt > 0) _bias += _bi * Math.Sqrt(dt) * _rng.Gaussian();
            double sig = dt > 0 ? _nd / Math.Sqrt(dt) : 0;
            return new ImuReading { Stamp = t, GyroZ = trueYawRate + _bias + sig * _rng.Gaussian() };
        }
    }

    // ------------------------------------------------------------------ ToF
    public sealed class RangeReading { public double Stamp; public bool Valid; public double Range; }

    /// <summary>Single-cone ToF: nearest return over a fan of rays inside the cone (centre + rings), sigma = a + b r, quantised to 1 mm, optional dropout.</summary>
    public sealed class TofModel
    {
        public double SigmaBase = 0.005, SigmaPerMetre = 0.01, Quantum = 0.001, Dropout = 0.01; public int Rings = 2, RaysPerRing = 8;
        private readonly RangeMount _m;
        public TofModel(RangeMount m) { _m = m; }
        public RangeReading Sample(Pose roverWorld, IReadOnlyList<Obb> obstacles, double t, Rng rng)
        {
            Pose sw = roverWorld.Compose(_m.InRover);
            double best = double.PositiveInfinity;
            for (int ring = 0; ring <= Rings; ring++)
            {
                int n = ring == 0 ? 1 : RaysPerRing; double ang = _m.ConeHalfAngle * ring / Rings;
                for (int k = 0; k < n; k++)
                {
                    double az = 2 * Math.PI * k / n;
                    Vec3 dirLocal = new Vec3(Math.Cos(ang), Math.Sin(ang) * Math.Cos(az), Math.Sin(ang) * Math.Sin(az));
                    best = Math.Min(best, LidarModel.CastRay(sw.P, sw.R * dirLocal, _m.MaxRange, obstacles));
                }
            }
            double g = rng.Gaussian(), u = rng.NextDouble();
            var rd = new RangeReading { Stamp = t };
            if (double.IsInfinity(best) || u < Dropout) return rd;
            double r = best + (SigmaBase + SigmaPerMetre * best) * g;
            r = Math.Round(r / Quantum) * Quantum;
            if (r < 0 || r > _m.MaxRange) return rd;
            rd.Valid = true; rd.Range = r; return rd;
        }
    }

    // ------------------------------------------------------------------ LiDAR
    public sealed class LidarScan { public double Stamp; public double AngleMin, AngleIncrement, RangeMax; public double[] Ranges; }

    /// <summary>2-D LiDAR: horizontal fan at the mount height, ray cast against the 3-D scenario boxes (an elevated beam is only hit when the beam height is crossed).</summary>
    public sealed class LidarModel
    {
        public double SigmaBase = 0.01, SigmaPerMetre = 0.002, Dropout = 0.01;
        private readonly LidarMount _m;
        public LidarModel(LidarMount m) { _m = m; }

        public static double CastRay(Vec3 origin, Vec3 dir, double maxRange, IReadOnlyList<Obb> obstacles)
        {
            Vec3 end = origin + dir.Normalized() * maxRange; double best = double.PositiveInfinity;
            foreach (var o in obstacles)
            {
                double t = o.SegmentEntry(origin, end);
                if (t >= 0) best = Math.Min(best, t * maxRange);
            }
            return best;
        }

        public LidarScan Sample(Pose roverWorld, IReadOnlyList<Obb> obstacles, double t, Rng rng)
        {
            Pose sw = roverWorld.Compose(_m.InRover);
            var scan = new LidarScan { Stamp = t, AngleMin = -Math.PI, AngleIncrement = 2 * Math.PI / _m.Points, RangeMax = _m.MaxRange, Ranges = new double[_m.Points] };
            for (int i = 0; i < _m.Points; i++)
            {
                double a = scan.AngleMin + i * scan.AngleIncrement;
                double d = CastRay(sw.P, sw.R * new Vec3(Math.Cos(a), Math.Sin(a), 0), _m.MaxRange, obstacles);
                double g = rng.Gaussian(), u = rng.NextDouble();
                if (double.IsInfinity(d) || u < Dropout) { scan.Ranges[i] = double.PositiveInfinity; continue; }
                double r = d + (SigmaBase + SigmaPerMetre * d) * g;
                scan.Ranges[i] = r > _m.MaxRange || r < 0.05 ? double.PositiveInfinity : r;
            }
            return scan;
        }
    }
}
