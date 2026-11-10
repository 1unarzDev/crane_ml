using System;

namespace Crane.Fleet
{
    /// <summary>
    /// Relative-pose estimate between two docking faces, in the DOCKING FRAME = the active (docker) face A frame (x out of the face,
    /// y left). State D = pose of the target's mating frame (face B frame turned by pi) in face A's frame, so perfect mating is
    /// D = identity: Gap = D.x, Lateral = D.y, Yaw = D.yaw (the Crane.Fleet.Docking FacePose convention). Full 3x3 covariance.
    ///
    /// Sources fused (q1_observer_value_v2: anisotropic tag error, one fused estimate with full covariance):
    ///  * own camera: T(A base -> B base) with its 6x6 PnP covariance, marginalised to (x, y, yaw) and mapped through the face levers
    ///    (correlations kept, so a frontal view's good cross-range and poor yaw are both represented);
    ///  * world-frame observations of A and B (peer cameras through their own estimate, overhead): paired by source and stamp,
    ///    or B alone combined with A's own world estimate at the acquisition stamp.
    /// Prediction: the docker's own motion increments (from its EKF velocity) move D; the target holds still (docking protocol)
    /// unless its increments are supplied. Delayed measurements are transported to the current time through the buffered
    /// increments (exactly what re-applying them would give for a stationary target), with the accumulated process noise.
    /// </summary>
    public sealed class RelativePoseEstimator
    {
        public readonly Pose2 FaceA, MateB;    // F_A in A's base; F_B * Rot(pi) in B's base
        private readonly AutonomyAssumptions _a;
        private double _dx, _dy, _dth;
        private readonly double[] _p = new double[9];
        public bool Initialized { get; private set; }
        public double Time { get; private set; }
        public int Accepted, Rejected, Ambiguous;
        public double LastUpdateTime = double.NegativeInfinity;

        // increment ring: (t, increment G in face-A frame applied at t: D' = G * D, process var added)
        private const int Ring = 512;
        private readonly double[] _rt = new double[Ring], _rgx = new double[Ring], _rgy = new double[Ring], _rgth = new double[Ring], _rq = new double[Ring * 2];
        private int _rhead, _rcount;

        public RelativePoseEstimator(Pose2 faceAInA, Pose2 faceBInB, AutonomyAssumptions a)
        {
            FaceA = faceAInA; MateB = Se2.Compose(faceBInB, new Pose2(0, 0, Math.PI)); _a = a;
        }

        public Pose2 Estimate => new Pose2(_dx, _dy, _dth);
        public double Gap => _dx;
        public double Lateral => _dy;
        public double Yaw => _dth;
        public double SigmaGap => Math.Sqrt(Math.Max(0, _p[0]));
        public double SigmaLateral => Math.Sqrt(Math.Max(0, _p[4]));
        public double SigmaYaw => Math.Sqrt(Math.Max(0, _p[8]));
        public void Covariance(double[] into) { Array.Copy(_p, into, 9); }

        /// <summary>Docking-frame pose from a base-to-base relative pose T(A base -> B base).</summary>
        public Pose2 FromBaseRelative(Pose2 tRel) => Se2.Compose(Se2.Compose(Se2.Inverse(FaceA), tRel), MateB);

        /// <summary>Base-to-base relative pose giving docking-frame pose d.</summary>
        public Pose2 ToBaseRelative(Pose2 d) => Se2.Compose(Se2.Compose(FaceA, d), Se2.Inverse(MateB));

        private readonly double[] _wc = new double[9], _wb = new double[9], _si = new double[9], _w18 = new double[18], _kk = new double[9], _np = new double[9];
        private readonly double[] _j1 = new double[9], _j2 = new double[9], _jt = new double[9], _tmp = new double[9], _r = new double[9];

        /// <summary>Map a base-relative measurement covariance (A base axes) into docking-frame axes: J = JR(F_A^-1) * JL(T, MateB).</summary>
        private void MapCov(Pose2 tRel, double[] rIn, double[] rOut)
        {
            Se2.JacComposeLeft(tRel, MateB, _j1);
            Se2.JacComposeRight(Se2.Inverse(FaceA), _j2);
            Lin.Mul(_j2, _j1, _jt, 3, 3, 3);
            Lin.Mul(_jt, rIn, _tmp, 3, 3, 3);
            Lin.MulBt(_tmp, _jt, rOut, 3, 3, 3);
        }

        /// <summary>Initialise from a base-relative prior (e.g. both rovers' world estimates) with covariance in A-base axes.</summary>
        public void InitializeFromBaseRelative(Pose2 tRel, double[] covBaseAxes, double t)
        {
            var d = FromBaseRelative(tRel);
            _dx = d.X; _dy = d.Y; _dth = Se2.Wrap(d.Yaw);
            MapCov(tRel, covBaseAxes, _p);
            Initialized = true; Time = t; _rcount = 0;
        }

        /// <summary>Docker moved by body increment <paramref name="deltaA"/> (its base frame) during [t-dt, t]; travel = path length (m) for process noise.</summary>
        public void PredictDockerMotion(Pose2 deltaA, double t)
        {
            if (!Initialized) { Time = t; return; }
            // D' = F_A^-1 * delta^-1 * F_A * D
            var g = Se2.Compose(Se2.Compose(Se2.Inverse(FaceA), Se2.Inverse(deltaA)), FaceA);
            double dist = Math.Sqrt(deltaA.X * deltaA.X + deltaA.Y * deltaA.Y), rot = Math.Abs(deltaA.Yaw);
            double qp = Sq(_a.RelativeProcessPosPerMeter * dist) + Sq(0.21 * _a.RelativeProcessYawPerRad * rot), qy = Sq(_a.RelativeProcessYawPerRad * rot) + Sq(0.5 * _a.RelativeProcessPosPerMeter * dist);
            ApplyIncrement(g, qp, qy);
            _rt[_rhead] = t; _rgx[_rhead] = g.X; _rgy[_rhead] = g.Y; _rgth[_rhead] = g.Yaw; _rq[2 * _rhead] = qp; _rq[2 * _rhead + 1] = qy;
            _rhead = (_rhead + 1) % Ring; if (_rcount < Ring) _rcount++;
            Time = t;
        }

        private void ApplyIncrement(Pose2 g, double qp, double qy)
        {
            var d = Se2.Compose(g, new Pose2(_dx, _dy, _dth));
            _dx = d.X; _dy = d.Y; _dth = d.Yaw;
            Se2.JacComposeRight(g, _j1);
            Lin.Congruence(_j1, _p, _tmp, 3);
            _p[0] += qp; _p[4] += qp; _p[8] += qy;
        }

        /// <summary>Transport a docking-frame measurement acquired at tAcq to the current time through buffered increments.</summary>
        private Pose2 Transport(Pose2 z, double[] r, double tAcq)
        {
            // walk increments with t > tAcq in order
            int n = _rcount;
            int start = (_rhead - n + Ring) % Ring;
            for (int k = 0; k < n; k++)
            {
                int i = (start + k) % Ring;
                if (_rt[i] <= tAcq) continue;
                var g = new Pose2(_rgx[i], _rgy[i], _rgth[i]);
                z = Se2.Compose(g, z);
                Se2.JacComposeRight(g, _j1);
                Lin.Congruence(_j1, r, _tmp, 3);
                r[0] += _rq[2 * i]; r[4] += _rq[2 * i]; r[8] += _rq[2 * i + 1];
            }
            return z;
        }

        /// <summary>Own-camera (or any A-base-frame) observation of the target base: tRel = T(A base -> B base), cov3 in A-base axes.</summary>
        public bool UpdateBaseRelative(Pose2 tRel, double[] cov3, double tAcq)
        {
            var z = FromBaseRelative(tRel);
            MapCov(tRel, cov3, _r);
            if (!Initialized)
            {
                _dx = z.X; _dy = z.Y; _dth = z.Yaw; Array.Copy(_r, _p, 9); Initialized = true; Time = tAcq;
                // bring to now
                var zz = Transport(new Pose2(_dx, _dy, _dth), _p, tAcq);
                _dx = zz.X; _dy = zz.Y; _dth = zz.Yaw; Accepted++; LastUpdateTime = tAcq; return true;
            }
            z = Transport(z, _r, tAcq);
            return Update(z, _r, tAcq);
        }

        /// <summary>World-frame poses of A's base and B's base with their covariances (world axes), same acquisition stamp.</summary>
        public bool UpdateWorldPair(Pose2 wA, double[] covA, Pose2 wB, double[] covB, double tAcq)
        {
            var tRel = Se2.Between(wA, wB);
            double c = Math.Cos(wA.Yaw), s = Math.Sin(wA.Yaw);
            // J_A, J_B of Between(wA, wB) wrt additive world-axes perturbations
            double[] ja = _j1, jb = _j2;
            ja[0] = -c; ja[1] = -s; ja[2] = tRel.Y; ja[3] = s; ja[4] = -c; ja[5] = -tRel.X; ja[6] = 0; ja[7] = 0; ja[8] = -1;
            jb[0] = c; jb[1] = s; jb[2] = 0; jb[3] = -s; jb[4] = c; jb[5] = 0; jb[6] = 0; jb[7] = 0; jb[8] = 1;
            var cov = _wc; var cb = _wb;
            Lin.Mul(ja, covA, _tmp, 3, 3, 3); Lin.MulBt(_tmp, ja, cov, 3, 3, 3);
            Lin.Mul(jb, covB, _tmp, 3, 3, 3); Lin.MulBt(_tmp, jb, cb, 3, 3, 3);
            for (int i = 0; i < 9; i++) cov[i] += cb[i];
            return UpdateBaseRelative(tRel, cov, tAcq);
        }

        private bool Update(Pose2 z, double[] r, double tAcq)
        {
            double i0 = z.X - _dx, i1 = z.Y - _dy, i2 = Se2.Wrap(z.Yaw - _dth);
            var S = _tmp;
            for (int i = 0; i < 9; i++) S[i] = _p[i] + r[i];
            double d2 = Lin.Mahalanobis3(S, i0, i1, i2);
            if (!(d2 <= _a.TagGateChi2)) { Rejected++; return false; }
            var Si = _si; var w = _w18;
            if (!Lin.Invert(S, Si, 3, w)) { Rejected++; return false; }
            var K = _kk;
            Lin.Mul(_p, Si, K, 3, 3, 3);
            _dx += K[0] * i0 + K[1] * i1 + K[2] * i2;
            _dy += K[3] * i0 + K[4] * i1 + K[5] * i2;
            _dth = Se2.Wrap(_dth + K[6] * i0 + K[7] * i1 + K[8] * i2);
            // P = (I - K) P
            var np = _np;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
            {
                double s = _p[3 * i + j];
                for (int t = 0; t < 3; t++) s -= K[3 * i + t] * _p[3 * t + j];
                np[3 * i + j] = s;
            }
            Array.Copy(np, _p, 9); Lin.Symmetrize(_p, 3);
            Accepted++; LastUpdateTime = Math.Max(LastUpdateTime, tAcq);
            return true;
        }

        /// <summary>
        /// Conservative capture test against the guide envelope of the docking core: |lateral| + n sigma_lat + ell |sin(|yaw| + n sigma_yaw)|
        /// within the lateral budget and |yaw| + n sigma_yaw within the acceptance angle.
        /// </summary>
        public bool PredictsCapture(double nSigma, double lateralBudget, double yawBudget, double guideFaceHalfLength = 0.03)
        {
            double yw = Math.Abs(_dth) + nSigma * SigmaYaw;
            double lat = Math.Abs(_dy) + nSigma * SigmaLateral + guideFaceHalfLength * Math.Abs(Math.Sin(Math.Min(yw, 1.0)));
            return lat <= lateralBudget && yw <= yawBudget;
        }

        private static double Sq(double x) => x * x;
    }
}
