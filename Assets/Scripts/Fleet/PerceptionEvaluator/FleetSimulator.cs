using System;
using System.Collections.Generic;
using System.Linq;
using Crane.Fleet.Perception.Rover;

namespace Crane.Fleet.Perception.Evaluator
{
    /// <summary>A fixed camera (survey-mounted station or scenario overhead camera). Pose is the TRUE pose; the system only ever sees the surveyed estimate.</summary>
    public sealed class StationSpec
    {
        public string Id; public ObservationSourceKind Kind = ObservationSourceKind.FixedStation;
        public Pose TruePose; public CameraIntrinsics K; public double RateHz, LatencyS, Phase;
        /// <summary>1-sigma survey error of the camera pose in the world frame (constant bias drawn once per seed): position m per axis, rotation rad per axis. ASSUMPTION.</summary>
        public double SurveySigmaPos = 0.004, SurveySigmaRot = 0.2 * Math.PI / 180.0;
    }

    public sealed class SimulatorConfig
    {
        public VisibilityParams Visibility = new VisibilityParams();
        public TagNoiseParams Noise = new TagNoiseParams();
        /// <summary>Probability (per overhead frame) that an 'occluded'-mode camera is blocked outright (observer_value oh_block_p = 0.60).</summary>
        public double OverheadBlockProbability = 0.60;
        /// <summary>Peer link: observer to consumer network delay added to the observer's own delivery stamp. ASSUMPTION.</summary>
        public double PeerLinkDelayS = 0.02;
        public double OverheadVisibilityMinPixels = double.NaN;   // NaN: use Visibility.MinPixels
        public int OverheadResolutionW = 1280, OverheadResolutionH = 800;
        public double FloorSlipSigmaFrac = double.NaN;            // NaN: profile hard_tile
    }

    /// <summary>
    /// Evaluator-side implementation of <see cref="IObservationBackend"/>. Holds ground truth, generates every rover's measurements from it and
    /// exposes only measurement types through the backend interface. Frame k of a source is drawn from the stream (seed, source, k, tag), so results
    /// are independent of polling cadence and of the number of consumers.
    /// </summary>
    public sealed class FleetSimulator : IObservationBackend
    {
        public readonly FleetProfile Profile; public readonly Scenario Scene; public readonly EvaluatorTruth Truth; public readonly ulong Seed; public readonly SimulatorConfig Config;
        private readonly Dictionary<string, SensorSuite> _suites = new Dictionary<string, SensorSuite>();
        private readonly Dictionary<string, RoverGeometry> _geom = new Dictionary<string, RoverGeometry>();
        private readonly Dictionary<string, List<(double t, Pose pose, Matrix cov)>> _est = new Dictionary<string, List<(double, Pose, Matrix)>>();
        private readonly List<StationSpec> _stations = new List<StationSpec>();
        private readonly Dictionary<string, Pose> _stationEst = new Dictionary<string, Pose>();
        private readonly Dictionary<string, Matrix> _stationCov = new Dictionary<string, Matrix>();
        private readonly Dictionary<string, List<RelPoseObservation>> _frameCache = new Dictionary<string, List<RelPoseObservation>>();
        private readonly Dictionary<string, long> _cursor = new Dictionary<string, long>();
        private readonly Dictionary<string, List<RelPoseObservation>> _queue = new Dictionary<string, List<RelPoseObservation>>();
        private readonly Dictionary<string, EncoderModel> _enc = new Dictionary<string, EncoderModel>();
        private readonly Dictionary<string, double[]> _wheelAngle = new Dictionary<string, double[]>();
        private readonly Dictionary<string, ImuModel> _imu = new Dictionary<string, ImuModel>();
        private readonly Dictionary<string, long> _scalarCursor = new Dictionary<string, long>();
        private readonly Dictionary<string, double> _lastEncT = new Dictionary<string, double>();

        public FleetSimulator(FleetProfile profile, Scenario scene, EvaluatorTruth truth, ulong seed, SimulatorConfig config = null)
        {
            Profile = profile; Scene = scene; Truth = truth; Seed = seed; Config = config ?? new SimulatorConfig();
            for (int i = 0; i < profile.Fleet.Count; i++)
            {
                var m = profile.Fleet[i];
                _suites[m.Id] = SensorSuite.ForRover(profile, m.Id);
                _geom[m.Id] = new RoverGeometry(profile, m.Id, i + 1);
                _est[m.Id] = new List<(double, Pose, Matrix)>();
                double slip = double.IsNaN(Config.FloorSlipSigmaFrac) ? profile.SlipSigmaFracHardTile : Config.FloorSlipSigmaFrac;
                if (_suites[m.Id].HasEncoders) { _enc[m.Id] = new EncoderModel(profile.Encoder.CountsPerRev, slip, seed, m.Id); _wheelAngle[m.Id] = new double[4]; }
                if (_suites[m.Id].HasImu) _imu[m.Id] = new ImuModel(profile.Imu.GyroNoiseDensity, profile.Imu.GyroBiasInstability, seed, m.Id);
            }
            // scenario overhead cameras (profile gives hfov, height, rate, latency; resolution assumed equal to OV9281 sensor)
            var ohK = CameraIntrinsics.FromHfov(Config.OverheadResolutionW, Config.OverheadResolutionH, profile.Overhead.Hfov);
            foreach (var c in scene.OverheadCameras)
                AddStation(new StationSpec { Id = c.Id, Kind = ObservationSourceKind.Overhead, TruePose = CameraMount.LookAt(c.Position, c.LookAt), K = ohK, RateHz = profile.Overhead.RateHz, LatencyS = profile.Overhead.Latency });
        }

        public void AddStation(StationSpec s)
        {
            _stations.Add(s);
            var rng = Rng.ForStream(Seed, "survey/" + s.Id);
            var cov = Matrix.Diag(s.SurveySigmaPos * s.SurveySigmaPos, s.SurveySigmaPos * s.SurveySigmaPos, s.SurveySigmaPos * s.SurveySigmaPos,
                s.SurveySigmaRot * s.SurveySigmaRot, s.SurveySigmaRot * s.SurveySigmaRot, s.SurveySigmaRot * s.SurveySigmaRot);
            var xi = rng.SampleGaussian(cov);
            _stationEst[s.Id] = s.TruePose.Perturbed(xi); _stationCov[s.Id] = cov;
        }

        /// <summary>Evaluator-only view of the surveyed (erroneous) pose a station's consumers will use.</summary>
        public Pose StationEstimate(string id) => _stationEst[id];
        public IObservationProvider ProviderFor(string roverId) { if (!_suites.ContainsKey(roverId)) throw new KeyNotFoundException(roverId); return new RoverObservationProvider(this, roverId); }
        public RoverGeometry GeometryOf(string roverId) => _geom[roverId];

        // ------------------------------------------------------------------ IObservationBackend
        public SensorSuite SuiteOf(string roverId) => _suites[roverId];

        public void ReportSelfEstimate(string roverId, double stamp, Pose pose, Matrix covariance)
        {
            var l = _est[roverId]; l.Add((stamp, pose, covariance)); l.Sort((a, b) => a.t.CompareTo(b.t));
        }

        private bool TryEstimateAt(string roverId, double t, out Pose pose, out Matrix cov)
        {
            pose = Pose.Identity; cov = null; bool found = false;
            foreach (var e in _est[roverId]) { if (e.t <= t + 1e-12) { pose = e.pose; cov = e.cov; found = true; } else break; }
            return found;
        }

        private List<Obb> Occluders(string exceptRover, double t)
        {
            var l = new List<Obb>(Scene.Boxes);
            foreach (var id in Truth.RoverIds)
                if (id != exceptRover) l.Add(_geom[id].BodyBox(Truth.At(id, t).Pose, id + "/body"));
            return l;
        }

        public IReadOnlyList<RelPoseObservation> PollTagObservations(string consumer, double now)
        {
            var result = new List<RelPoseObservation>();
            var queue = Queue(consumer);
            // own cameras
            foreach (var cam in _suites[consumer].Cameras)
                Advance(consumer, "own/" + consumer + "/" + cam.Name, new FrameSchedule(cam.RateHz, PhaseOf(consumer + cam.Name, cam.RateHz)), now, queue,
                    k => GenerateRoverCameraFrame(consumer, cam, k, ObservationSourceKind.OwnCamera, false));
            // peers
            foreach (var p in _suites.Keys.Where(x => x != consumer))
                foreach (var cam in _suites[p].Cameras)
                    Advance(consumer, "peer/" + p + "/" + cam.Name, new FrameSchedule(cam.RateHz, PhaseOf(p + cam.Name, cam.RateHz)), now, queue,
                        k => GenerateRoverCameraFrame(p, cam, k, ObservationSourceKind.PeerObserver, true), sharedKey: "peerframes/" + p + "/" + cam.Name);
            // stations and overhead
            foreach (var s in _stations)
                Advance(consumer, "station/" + s.Id, new FrameSchedule(s.RateHz, s.Phase), now, queue, k => GenerateStationFrame(s, k), sharedKey: "stationframes/" + s.Id);
            // deliver
            queue.Sort((a, b) => a.StampDelivered.CompareTo(b.StampDelivered));
            int n = 0; while (n < queue.Count && queue[n].StampDelivered <= now) n++;
            for (int i = 0; i < n; i++) { var o = queue[i].Clone(); o.Age = now - o.StampAcquired; result.Add(o); }
            queue.RemoveRange(0, n);
            return result;
        }

        private List<RelPoseObservation> Queue(string consumer) { if (!_queue.TryGetValue(consumer, out var q)) _queue[consumer] = q = new List<RelPoseObservation>(); return q; }
        private double PhaseOf(string key, double rate) => Rng.ForStream(Seed, "phase/" + key).NextDouble() / rate;

        private void Advance(string consumer, string feedKey, FrameSchedule sch, double now, List<RelPoseObservation> queue, Func<long, List<RelPoseObservation>> gen, string sharedKey = null)
        {
            string ck = consumer + "|" + feedKey;
            long next = _cursor.TryGetValue(ck, out var c) ? c : 0, last = sch.LastIndexAtOrBefore(now);
            for (long k = next; k <= last; k++)
            {
                string fk = (sharedKey ?? ck) + "#" + k;
                if (!_frameCache.TryGetValue(fk, out var obs)) _frameCache[fk] = obs = gen(k);
                queue.AddRange(obs);
            }
            if (last + 1 > next) _cursor[ck] = last + 1;
        }

        private List<RelPoseObservation> GenerateRoverCameraFrame(string observer, CameraMount3 cam, long k, ObservationSourceKind kind, bool shared)
        {
            var list = new List<RelPoseObservation>();
            double t = new FrameSchedule(cam.RateHz, PhaseOf(observer + cam.Name, cam.RateHz)).TimeOf(k);
            if (t < 0) return list;
            var obs = Truth.At(observer, t); Pose camWorld = obs.Pose.Compose(cam.OpticalInRover);
            var camMotion = new BodyMotion(obs.LinVelWorld, obs.AngVelWorld);
            var occ = Occluders(observer, t);
            Pose estPose = Pose.Identity; Matrix estCov = null;
            if (shared && !TryEstimateAt(observer, t, out estPose, out estCov)) return list;
            foreach (var target in Truth.RoverIds.Where(x => x != observer))
            {
                var tt = Truth.At(target, t); var tmo = new BodyMotion(tt.LinVelWorld, tt.AngVelWorld);
                foreach (var tag in _geom[target].Tags)
                {
                    var rng = Rng.ForStream(Seed, $"det/{observer}/{cam.Name}/{k}/{tag.Id}");
                    var det = TagObserver.Detect(camWorld, camMotion, cam.K, tt.Pose.Compose(tag.MountInOwner), tmo, tag, occ, Config.Visibility, Config.Noise, cam.LatencyS, t, rng);
                    if (!det.Detected) continue;
                    // measured T_base_tag and covariance in base axes
                    Pose baseTag = cam.OpticalInRover.Compose(det.CamTag);
                    Matrix covBaseTag = PoseUncertainty.RotateInto(cam.OpticalInRover.R, det.CovarianceCam);
                    Pose mInv = tag.MountInOwner.Inverse();
                    Pose baseTarget = baseTag.Compose(mInv);
                    Matrix cov = PoseUncertainty.RigidShift(baseTag, mInv, covBaseTag);
                    var o = new RelPoseObservation
                    {
                        StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = observer + "/" + cam.Name, SourceKind = kind, TagId = tag.Id, AmbiguousPose = det.Pnp.Ambiguous,
                        FrameParent = ObservationScorer.BaseFrame(observer), FrameChild = ObservationScorer.BaseFrame(target), Pose = baseTarget, Covariance = cov
                    };
                    if (shared)
                    {
                        // through the observer's own pose ESTIMATE: self-localisation error enters the shared-frame error
                        o.Covariance = PoseUncertainty.Compose(estPose, estCov, baseTarget, cov);
                        o.Pose = estPose.Compose(baseTarget);
                        o.FrameParent = ObservationScorer.WorldFrame;
                        o.StampDelivered += Config.PeerLinkDelayS;
                    }
                    list.Add(o);
                }
            }
            return list;
        }

        private List<RelPoseObservation> GenerateStationFrame(StationSpec s, long k)
        {
            var list = new List<RelPoseObservation>();
            double t = new FrameSchedule(s.RateHz, s.Phase).TimeOf(k);
            if (t < 0) return list;
            if (s.Kind == ObservationSourceKind.Overhead)
            {
                var mode = Truth.OverheadModeOf(s.Id);
                if (mode == OverheadMode.Disabled) return list;
                var br = Rng.ForStream(Seed, $"ohblock/{s.Id}/{k}");
                if (mode == OverheadMode.Occluded && br.NextDouble() < Config.OverheadBlockProbability) return list;
            }
            var occ = Occluders(null, t); Pose est = _stationEst[s.Id];
            var vp = Config.Visibility;
            foreach (var target in Truth.RoverIds)
            {
                var tt = Truth.At(target, t); var tmo = new BodyMotion(tt.LinVelWorld, tt.AngVelWorld);
                foreach (var tag in _geom[target].Tags)
                {
                    var rng = Rng.ForStream(Seed, $"det/{s.Id}/{k}/{tag.Id}");
                    var det = TagObserver.Detect(s.TruePose, BodyMotion.Still, s.K, tt.Pose.Compose(tag.MountInOwner), tmo, tag, occ, vp, Config.Noise, s.LatencyS, t, rng);
                    if (!det.Detected) continue;
                    Pose worldTag = est.Compose(det.CamTag);
                    Matrix cov = PoseUncertainty.Compose(est, _stationCov[s.Id], det.CamTag, det.CovarianceCam);
                    Pose mInv = tag.MountInOwner.Inverse();
                    list.Add(new RelPoseObservation
                    {
                        StampAcquired = t, StampDelivered = det.StampDelivered, SourceId = s.Id, SourceKind = s.Kind, TagId = tag.Id, AmbiguousPose = det.Pnp.Ambiguous,
                        FrameParent = ObservationScorer.WorldFrame, FrameChild = ObservationScorer.BaseFrame(target),
                        Pose = worldTag.Compose(mInv), Covariance = PoseUncertainty.RigidShift(worldTag, mInv, cov)
                    });
                }
            }
            return list;
        }

        // ------------------------------------------------------------------ proprioception
        private List<double> SampleTimes(string key, double rate, double now)
        {
            long next = _scalarCursor.TryGetValue(key, out var c) ? c : 0, last = new FrameSchedule(rate, 0).LastIndexAtOrBefore(now);
            var l = new List<double>(); var sch = new FrameSchedule(rate, 0);
            for (long k = next; k <= last; k++) l.Add(sch.TimeOf(k));
            if (last + 1 > next) _scalarCursor[key] = last + 1;
            return l;
        }

        public IReadOnlyList<EncoderReading> PollEncoders(string roverId, double now)
        {
            var res = new List<EncoderReading>();
            if (!_enc.ContainsKey(roverId)) return res;
            var p = Profile.Encoder;
            foreach (double t in SampleTimes("enc/" + roverId, p.RateHz, now))
            {
                double t0 = _lastEncT.TryGetValue(roverId, out var lt) ? lt : t;
                var st = Truth.At(roverId, 0.5 * (t0 + t));
                var w = MecanumKinematics.WheelSpeeds(st.Vx, st.Vy, st.Wz, p.HalfWheelbaseLx, p.HalfTrackLy, p.WheelRadius);
                for (int i = 0; i < 4; i++) _wheelAngle[roverId][i] += w[i] * (t - t0);
                _lastEncT[roverId] = t;
                res.Add(_enc[roverId].Sample(t, _wheelAngle[roverId]));
            }
            return res;
        }

        public IReadOnlyList<ImuReading> PollImu(string roverId, double now)
        {
            var res = new List<ImuReading>();
            if (!_imu.ContainsKey(roverId)) return res;
            foreach (double t in SampleTimes("imu/" + roverId, Profile.Imu.RateHz, now)) res.Add(_imu[roverId].Sample(t, Truth.At(roverId, t).Wz));
            return res;
        }

        public IReadOnlyList<RangeReading> PollTof(string roverId, string mountName, double now)
        {
            var res = new List<RangeReading>();
            var m = _suites[roverId].Tofs.FirstOrDefault(x => x.Name == mountName);
            if (m == null) return res;
            var model = new TofModel(m); long k0 = 0;
            foreach (double t in SampleTimes("tof/" + roverId + "/" + mountName, m.RateHz, now))
            {
                k0 = (long)Math.Round(t * m.RateHz);
                res.Add(model.Sample(Truth.At(roverId, t).Pose, Occluders(roverId, t), t, Rng.ForStream(Seed, $"tof/{roverId}/{mountName}/{k0}")));
            }
            return res;
        }

        public IReadOnlyList<LidarScan> PollLidar(string roverId, double now)
        {
            var res = new List<LidarScan>();
            var m = _suites[roverId].Lidar;
            if (m == null) return res;
            var model = new LidarModel(m);
            foreach (double t in SampleTimes("lidar/" + roverId, m.RateHz, now))
                res.Add(model.Sample(Truth.At(roverId, t).Pose, Occluders(roverId, t), t, Rng.ForStream(Seed, $"lidar/{roverId}/{(long)Math.Round(t * m.RateHz)}")));
            return res;
        }
    }
}
