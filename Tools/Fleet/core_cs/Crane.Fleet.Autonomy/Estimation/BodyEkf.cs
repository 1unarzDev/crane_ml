using System;
using System.Collections.Generic;

namespace Crane.Fleet
{
    /// <summary>
    /// EKF on SE(2) plus body velocity for one rigid assembly (a single rover is an assembly of one module).
    /// State x = [px, py, yaw, vx, vy, wz, b_1 .. b_G] (world pose of the assembly frame, body twist in the assembly frame,
    /// one gyro bias per module). Measurements:
    ///   wheel rates (per wheel, scalar): rate_i = row_i(module twist with vy / lateral_efficiency) + slip noise (sigma = slip_frac |rate| + quantisation);
    ///   gyro z of module m: wz + b_m;
    ///   world-frame pose of a member base (overhead, fixed station, peer-through-its-estimate) with full 3x3 covariance, gated.
    /// Delayed measurements: every processed measurement is logged with the post-update state; a measurement acquired before the
    /// filter time rolls the filter back to the last logged state at or before its stamp, applies it, and re-applies every later
    /// logged measurement (buffer and re-apply). Truth never enters.
    /// </summary>
    public sealed class BodyEkf
    {
        private enum Kind : byte { Wheel, Gyro, Pose, Zupt, GyroRest }
        private struct Entry { public double T; public Kind K; public int M; public int W; public double Z0, Z1, Z2; public double R0, R1, R2, R3, R4, R5, R6, R7, R8; }

        public int N { get; private set; }
        public double Time { get; private set; }
        public int Modules { get; private set; }
        public int RejectedPoseUpdates { get; private set; }
        public int AcceptedPoseUpdates { get; private set; }
        public int DelayedReapplied { get; private set; }
        public int DroppedTooOld { get; private set; }

        private readonly AutonomyAssumptions _a;
        private AssemblyCapability _cap;
        private AssemblyModel _model;
        private double[] _x, _p, _f, _scratch, _h, _k, _ph;
        private double[] _rowsMod; // per wheel: module-frame twist -> rate with lateral efficiency, mapped to assembly twist (3 per wheel)
        private double[] _slip;
        private bool[] _locked;
        // log + snapshots
        private readonly int _cap0;
        private Entry[] _log;
        private double[] _snapX, _snapP;
        private int _count;
        private double _logStartT;
        private double[] _baseX, _baseP; private double _baseT; // state just before the first logged entry

        public BodyEkf(AssemblyModel model, ICapability cap, AutonomyAssumptions a, Pose2 initial, double posSigma, double yawSigma, double t0, int logCapacity = 768)
        {
            _a = a; _cap0 = logCapacity;
            Allocate(model, (AssemblyCapability)cap);
            _x[0] = initial.X; _x[1] = initial.Y; _x[2] = Se2.Wrap(initial.Yaw);
            _p[0] = posSigma * posSigma; _p[N + 1] = posSigma * posSigma; _p[2 * N + 2] = yawSigma * yawSigma;
            for (int i = 3; i < 6; i++) _p[i * N + i] = 1e-4;
            for (int i = 6; i < N; i++) _p[i * N + i] = a.InitialBiasSigma * a.InitialBiasSigma;
            Time = t0; ResetLog();
        }

        private void Allocate(AssemblyModel model, AssemblyCapability cap)
        {
            _model = model; _cap = cap; Modules = model.Modules.Count; N = 6 + Modules;
            _x = new double[N]; _p = new double[N * N]; _f = new double[N * N]; _scratch = new double[N * N];
            _h = new double[N]; _k = new double[N]; _ph = new double[3 * N];
            BuildRows();
            _log = new Entry[_cap0]; _snapX = new double[_cap0 * N]; _snapP = new double[_cap0 * N * N];
            _baseX = new double[N]; _baseP = new double[N * N];
        }

        private void BuildRows()
        {
            int w = 4 * Modules;
            _rowsMod = new double[3 * w]; _slip = new double[w]; _locked = new bool[w];
            for (int m = 0; m < Modules; m++)
            {
                var mod = _model.Modules[m]; var s = mod.Spec;
                double c = Math.Cos(mod.InAssembly.Yaw), sn = Math.Sin(mod.InAssembly.Yaw), px = mod.InAssembly.X, py = mod.InAssembly.Y;
                for (int j = 0; j < 4; j++)
                {
                    s.WheelRow(j, out double ax, out double ay, out double aw);
                    ay /= Math.Max(1e-3, s.LateralEfficiency);   // ground lateral speed = LE x wheel-implied lateral speed
                    int i = 4 * m + j;
                    _rowsMod[3 * i] = ax * c - ay * sn;
                    _rowsMod[3 * i + 1] = ax * sn + ay * c;
                    _rowsMod[3 * i + 2] = ax * (-c * py + sn * px) + ay * (sn * py + c * px) + aw;
                    _slip[i] = s.SlipSigmaFrac;
                    _locked[i] = mod.Health[j].Mode == CapHealthMode.DeadLocked;
                }
            }
        }

        /// <summary>Re-bind after a capability change that keeps the module set (health update). State and covariance are kept.</summary>
        public void Rebind(AssemblyModel model, ICapability cap)
        {
            if (model.Modules.Count != Modules) throw new ArgumentException("module count changed; create a new filter (merge/split)");
            _model = model; _cap = (AssemblyCapability)cap; BuildRows();
        }

        // ------------------------------------------------------------------ accessors
        public Pose2 Pose => new Pose2(_x[0], _x[1], _x[2]);
        public Twist2 Velocity => new Twist2(_x[3], _x[4], _x[5]);
        public double GyroBias(int module) => _x[6 + module];
        public double Cov(int i, int j) => _p[i * N + j];
        /// <summary>3x3 pose covariance (x, y, yaw) in world axes, copied into <paramref name="into"/>.</summary>
        public void PoseCov(double[] into) { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) into[3 * i + j] = _p[i * N + j]; }
        public double PosSigma => Math.Sqrt(Math.Max(0, 0.5 * (_p[0] + _p[N + 1])));
        public double YawSigma => Math.Sqrt(Math.Max(0, _p[2 * N + 2]));

        /// <summary>World pose of member <paramref name="module"/>'s base.</summary>
        public Pose2 ModulePose(int module) => Se2.Compose(Pose, _model.Modules[module].InAssembly);

        /// <summary>Overwrite the pose part (used when an assembly forms or splits); velocity reset to zero.</summary>
        public void Reset(Pose2 pose, double[] cov3, double t, bool keepVelocity = false)
        {
            if (!keepVelocity) Array.Clear(_x, 3, 3);
            _x[0] = pose.X; _x[1] = pose.Y; _x[2] = Se2.Wrap(pose.Yaw);
            for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) if (i < 3 || j < 3) _p[i * N + j] = 0;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) _p[i * N + j] = cov3[3 * i + j];
            if (!keepVelocity) for (int i = 3; i < 6; i++) { for (int j = 0; j < N; j++) { _p[i * N + j] = 0; _p[j * N + i] = 0; } _p[i * N + i] = 1e-4; }
            Time = Math.Max(Time, t); ResetLog();
        }

        /// <summary>Add uncertainty to the pose block (stall, shove, unexplained motion): position sigma and yaw sigma in quadrature.
        /// The measurement log is restarted so a later re-application cannot undo the inflation.</summary>
        public void InflatePose(double posSigma, double yawSigma)
        {
            _p[0] += posSigma * posSigma; _p[N + 1] += posSigma * posSigma; _p[2 * N + 2] += yawSigma * yawSigma;
            ResetLog();
        }

        public void SetBias(int module, double b, double var) { _x[6 + module] = b; _p[(6 + module) * N + 6 + module] = var; }

        // ------------------------------------------------------------------ public measurement API
        public void PredictTo(double t) { if (t > Time) Predict(t - Time); Time = Math.Max(Time, t); }

        /// <summary>Wheel rate measurement (rad/s, mean over [t - dt, t]) of wheel <paramref name="wheel"/> (0..3) of module <paramref name="module"/>.</summary>
        public void WheelRate(int module, int wheel, double t, double rate, double quantSigma)
        {
            var e = new Entry { T = t, K = Kind.Wheel, M = module, W = wheel, Z0 = rate, Z1 = quantSigma };
            Ingest(ref e);
        }

        /// <summary>Zero-velocity update (all wheel counts unchanged over an interval): body twist ~ 0, which makes gyro biases observable.</summary>
        public void ZeroVelocity(double t, double sigmaLin, double sigmaYaw)
        {
            var e = new Entry { T = t, K = Kind.Zupt, Z0 = sigmaLin, Z1 = sigmaYaw };
            Ingest(ref e);
        }

        /// <summary>Gyro sample taken while the wheels report standstill: measures the bias directly (wz = 0).</summary>
        public void GyroAtRest(int module, double t, double rate, double sigma)
        {
            var e = new Entry { T = t, K = Kind.GyroRest, M = module, Z0 = rate, Z1 = sigma };
            Ingest(ref e);
        }

        public void Gyro(int module, double t, double rate, double sigma)
        {
            var e = new Entry { T = t, K = Kind.Gyro, M = module, Z0 = rate, Z1 = sigma };
            Ingest(ref e);
        }

        /// <summary>World-frame pose of member <paramref name="module"/>'s base acquired at <paramref name="t"/>; r3 = 3x3 covariance (x, y, yaw).</summary>
        public void WorldPose(int module, double t, Pose2 z, double[] r3)
        {
            var e = new Entry { T = t, K = Kind.Pose, M = module, Z0 = z.X, Z1 = z.Y, Z2 = z.Yaw, R0 = r3[0], R1 = r3[1], R2 = r3[2], R3 = r3[3], R4 = r3[4], R5 = r3[5], R6 = r3[6], R7 = r3[7], R8 = r3[8] };
            Ingest(ref e);
        }

        // ------------------------------------------------------------------ log and replay
        private void ResetLog() { _count = 0; Array.Copy(_x, _baseX, N); Array.Copy(_p, _baseP, N * N); _baseT = Time; _logStartT = Time; }

        private void Ingest(ref Entry e)
        {
            if (e.T >= Time - 1e-12)
            {
                PredictTo(e.T);
                Apply(ref e);
                Append(ref e);
                return;
            }
            // delayed: find last entry with T <= e.T
            if (e.T < _baseT) { DroppedTooOld++; return; }
            int j = _count - 1;
            while (j >= 0 && _log[j].T > e.T) j--;
            // restore state after entry j (or base)
            double tRestore;
            if (j >= 0) { Array.Copy(_snapX, j * N, _x, 0, N); Array.Copy(_snapP, j * N * N, _p, 0, N * N); tRestore = _log[j].T; }
            else { Array.Copy(_baseX, _x, N); Array.Copy(_baseP, _p, N * N); tRestore = _baseT; }
            double tEnd = Time;
            Time = tRestore;
            // insert e at j+1
            if (_count == _log.Length) DropOldest(_count / 4);
            int ins = Math.Min(j + 1, _count);
            if (j + 1 > _count) ins = _count;
            for (int k = _count; k > ins; k--) { _log[k] = _log[k - 1]; }
            _log[ins] = e; _count++;
            // replay from ins
            for (int k = ins; k < _count; k++)
            {
                PredictTo(_log[k].T);
                Apply(ref _log[k]);
                Array.Copy(_x, 0, _snapX, k * N, N); Array.Copy(_p, 0, _snapP, k * N * N, N * N);
            }
            PredictTo(tEnd);
            DelayedReapplied++;
        }

        private void Append(ref Entry e)
        {
            if (_count == _log.Length) DropOldest(_count / 4);
            _log[_count] = e;
            Array.Copy(_x, 0, _snapX, _count * N, N); Array.Copy(_p, 0, _snapP, _count * N * N, N * N);
            _count++;
            // trim the window
            int drop = 0;
            while (drop < _count && _log[drop].T < Time - _a.DelayBufferSeconds) drop++;
            if (drop > 64) DropOldest(drop);
        }

        private void DropOldest(int k)
        {
            if (k <= 0) return;
            Array.Copy(_snapX, (k - 1) * N, _baseX, 0, N); Array.Copy(_snapP, (k - 1) * N * N, _baseP, 0, N * N); _baseT = _log[k - 1].T;
            Array.Copy(_log, k, _log, 0, _count - k);
            Array.Copy(_snapX, k * N, _snapX, 0, (_count - k) * N);
            Array.Copy(_snapP, k * N * N, _snapP, 0, (_count - k) * N * N);
            _count -= k;
        }

        // ------------------------------------------------------------------ filter core
        private void Predict(double dt)
        {
            if (dt <= 0) return;
            double th = _x[2], c = Math.Cos(th), s = Math.Sin(th), vx = _x[3], vy = _x[4], wz = _x[5];
            _x[0] += (c * vx - s * vy) * dt;
            _x[1] += (s * vx + c * vy) * dt;
            _x[2] = Se2.Wrap(th + wz * dt);
            Array.Clear(_f, 0, N * N);
            for (int i = 0; i < N; i++) _f[i * N + i] = 1;
            _f[0 * N + 2] = (-s * vx - c * vy) * dt; _f[0 * N + 3] = c * dt; _f[0 * N + 4] = -s * dt;
            _f[1 * N + 2] = (c * vx - s * vy) * dt; _f[1 * N + 3] = s * dt; _f[1 * N + 4] = c * dt;
            _f[2 * N + 5] = dt;
            Lin.Congruence(_f, _p, _scratch, N);
            double qa = _a.AccelProcessSigma * _a.AccelProcessSigma * dt, qw = _a.YawAccelProcessSigma * _a.YawAccelProcessSigma * dt;
            _p[3 * N + 3] += qa; _p[4 * N + 4] += qa; _p[5 * N + 5] += qw;
            // position/yaw get the integrated velocity noise (dt^3/3 terms)
            _p[0] += qa * dt * dt / 3; _p[N + 1] += qa * dt * dt / 3; _p[2 * N + 2] += qw * dt * dt / 3;
            for (int m = 0; m < Modules; m++)
            {
                double bi = _model.Modules[m].Spec.GyroBiasInstability;
                _p[(6 + m) * N + 6 + m] += bi * bi * dt;
            }
        }

        private void Apply(ref Entry e)
        {
            switch (e.K)
            {
                case Kind.Wheel:
                {
                    int i = 4 * e.M + e.W;
                    if (_locked[i]) break;   // a locked wheel reads 0; its constraint is already in the other wheels' motion
                    Array.Clear(_h, 0, N);
                    _h[3] = _rowsMod[3 * i]; _h[4] = _rowsMod[3 * i + 1]; _h[5] = _rowsMod[3 * i + 2];
                    double pred = _h[3] * _x[3] + _h[4] * _x[4] + _h[5] * _x[5];
                    double sig = _a.SlipCorrelationInflation * _slip[i] * Math.Abs(pred) + e.Z1;
                    ScalarUpdate(e.Z0 - pred, sig * sig, considerBiases: true);
                    break;
                }
                case Kind.Gyro:
                {
                    Array.Clear(_h, 0, N);
                    _h[5] = 1; _h[6 + e.M] = 1;
                    double pred = _x[5] + _x[6 + e.M];
                    ScalarUpdate(e.Z0 - pred, e.Z1 * e.Z1);
                    break;
                }
                case Kind.Pose:
                    PoseUpdate(ref e);
                    break;
                case Kind.GyroRest:
                {
                    Array.Clear(_h, 0, N);
                    _h[6 + e.M] = 1;
                    ScalarUpdate(e.Z0 - _x[6 + e.M], e.Z1 * e.Z1);
                    break;
                }
                case Kind.Zupt:
                    for (int k = 3; k < 6; k++)
                    {
                        Array.Clear(_h, 0, N); _h[k] = 1;
                        double sg = k == 5 ? e.Z1 : e.Z0;
                        ScalarUpdate(-_x[k], sg * sg);
                    }
                    break;
            }
        }

        /// <summary>Scalar Kalman update. With <paramref name="considerBiases"/> the gyro biases are Schmidt "consider" states: their gain is
        /// zeroed (time-correlated wheel slip must not walk the biases; they are learned from gyro-vs-ZUPT at standstill) and the
        /// covariance is updated in Joseph form so it stays consistent with the suboptimal gain.</summary>
        private void ScalarUpdate(double innov, double r, bool considerBiases)
        {
            if (!considerBiases) { ScalarUpdate(innov, r); return; }
            double S = r;
            for (int i = 0; i < N; i++)
            {
                double ph = 0;
                for (int j = 0; j < N; j++) ph += _p[i * N + j] * _h[j];
                _k[i] = ph; S += _h[i] * ph;
            }
            if (S <= 0) return;
            for (int i = 0; i < N; i++) _k[i] = i >= 6 ? 0 : _k[i] / S;
            for (int i = 0; i < N; i++) _x[i] += _k[i] * innov;
            _x[2] = Se2.Wrap(_x[2]);
            // Joseph: P = (I - k h) P (I - k h)^T + k r k^T
            for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) _f[i * N + j] = (i == j ? 1 : 0) - _k[i] * _h[j];
            Lin.Congruence(_f, _p, _scratch, N);
            for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) _p[i * N + j] += _k[i] * _k[j] * r;
            Lin.Symmetrize(_p, N);
        }

        private void ScalarUpdate(double innov, double r)
        {
            // S = h P h^T + r ; K = P h^T / S
            double S = r;
            for (int i = 0; i < N; i++)
            {
                double ph = 0;
                for (int j = 0; j < N; j++) ph += _p[i * N + j] * _h[j];
                _k[i] = ph; S += _h[i] * ph;
            }
            if (S <= 0) return;
            for (int i = 0; i < N; i++) _k[i] /= S;
            for (int i = 0; i < N; i++) _x[i] += _k[i] * innov;
            _x[2] = Se2.Wrap(_x[2]);
            // P -= K (h P) = K K^T S
            for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) _p[i * N + j] -= _k[i] * _k[j] * S;
            Lin.Symmetrize(_p, N);
        }

        private readonly double[] _r9 = new double[9], _j = new double[9], _s3 = new double[9], _si = new double[9], _w6 = new double[18];

        private void PoseUpdate(ref Entry e)
        {
            // h(x) = T(x) * M  (M = module pose in assembly). Jacobian wrt (px, py, yaw) of T: JacComposeLeft(T, M)
            var mod = _model.Modules[e.M].InAssembly;
            var T = Pose;
            var pred = Se2.Compose(T, mod);
            Se2.JacComposeLeft(T, mod, _j);
            double i0 = e.Z0 - pred.X, i1 = e.Z1 - pred.Y, i2 = Se2.Wrap(e.Z2 - pred.Yaw);
            // H = [J 0]; PH^T = P[:,0:3] J^T  (N x 3)
            for (int r = 0; r < N; r++)
                for (int c = 0; c < 3; c++)
                {
                    double s = 0;
                    for (int t = 0; t < 3; t++) s += _p[r * N + t] * _j[3 * c + t];
                    _ph[r * 3 + c] = s;
                }
            var R = _r9; R[0] = e.R0; R[1] = e.R1; R[2] = e.R2; R[3] = e.R3; R[4] = e.R4; R[5] = e.R5; R[6] = e.R6; R[7] = e.R7; R[8] = e.R8;
            for (int a = 0; a < 3; a++)
                for (int b = 0; b < 3; b++)
                {
                    double s = R[3 * a + b];
                    for (int t = 0; t < 3; t++) s += _j[3 * a + t] * _ph[t * 3 + b];
                    _s3[3 * a + b] = s;
                }
            double d2 = Lin.Mahalanobis3(_s3, i0, i1, i2);
            if (!(d2 <= _a.TagGateChi2)) { RejectedPoseUpdates++; return; }
            if (!Lin.Invert(_s3, _si, 3, _w6)) { RejectedPoseUpdates++; return; }
            AcceptedPoseUpdates++;
            // K = PH^T S^-1 (N x 3)
            for (int r = 0; r < N; r++)
            {
                double k0 = 0, k1 = 0, k2 = 0;
                for (int t = 0; t < 3; t++) { double v = _ph[r * 3 + t]; k0 += v * _si[3 * t]; k1 += v * _si[3 * t + 1]; k2 += v * _si[3 * t + 2]; }
                _x[r] += k0 * i0 + k1 * i1 + k2 * i2;
                _k[r] = 0; // unused
                _scratch[r * 3] = k0; _scratch[r * 3 + 1] = k1; _scratch[r * 3 + 2] = k2;
            }
            _x[2] = Se2.Wrap(_x[2]);
            // P -= K (PH^T)^T
            for (int r = 0; r < N; r++)
                for (int c = 0; c < N; c++)
                    _p[r * N + c] -= _scratch[r * 3] * _ph[c * 3] + _scratch[r * 3 + 1] * _ph[c * 3 + 1] + _scratch[r * 3 + 2] * _ph[c * 3 + 2];
            Lin.Symmetrize(_p, N);
        }
    }
}
