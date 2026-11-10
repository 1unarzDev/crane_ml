using System;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    public class ObservationModelTests
    {
        private static TagNoiseParams Clean(double sigma = 1.0) => new TagNoiseParams { CornerSigmaPx = sigma, BaseDropout = 0, RampMax = 0, LatencyJitterS = 0, ExposureS = 0, ObliquityNoiseGain = 0 };
        private static VisibilityParams Loose() => new VisibilityParams { MinPixels = 8, MaxViewAngle = 80 * Math.PI / 180 };

        private static TagSpec Tag(double size) => new TagSpec { Id = 1, Size = size };

        private static (TagDetection det, Pose trueCamTag) Shoot(double range, double viewDeg, ulong seed, int k, double size = 0.12, TagNoiseParams np = null, BodyMotion? camMotion = null)
        {
            Pose cam = TestEnv.CamAt(0, 0, 0.3), tag = TestEnv.TagFacingCamera(range, 0, 0.3, viewDeg * Math.PI / 180);
            var det = TagObserver.Detect(cam, camMotion ?? BodyMotion.Still, TestEnv.K(), tag, BodyMotion.Still, Tag(size), null, Loose(), np ?? Clean(), 0.03, 1.0, Rng.ForStream(seed, "t/" + k));
            return (det, cam.Inverse().Compose(tag));
        }

        private static (double pos, double rot) Err(TagDetection d, Pose truth)
        {
            var e = d.CamTag.ErrorFrom(truth);
            return (Math.Sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]), Math.Sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]));
        }

        private static double Median(IEnumerable<double> v) { var a = v.OrderBy(x => x).ToArray(); return a[a.Length / 2]; }

        [Theory]
        [InlineData(1.0, 0)]
        [InlineData(2.0, 30)]
        [InlineData(0.7, -45)]
        public void Pnp_NoiseFree_RecoversPoseExactly(double range, double viewDeg)
        {
            Pose cam = TestEnv.CamAt(0, 0, 0.3), tag = TestEnv.TagFacingCamera(range, 0.05, 0.28, viewDeg * Math.PI / 180);
            var truth = cam.Inverse().Compose(tag); var X = TagSpec.LocalCorners(0.12);
            var uv = PlanarPnP.Reproject(TestEnv.K(), X, truth);
            var res = PlanarPnP.Solve(TestEnv.K(), X, uv, 1.0);
            Assert.True(res.Ok);
            var e = res.CamTag.ErrorFrom(truth);
            Assert.True(e.Max(Math.Abs) < 1e-6, string.Join(",", e));
        }

        [Fact]
        public void AnalyticJacobian_MatchesFiniteDifferences()
        {
            var K = TestEnv.K(); var X = TagSpec.LocalCorners(0.12);
            Pose p = new Pose(Mat3.Rz(0.3) * Mat3.Rx(1.2), new Vec3(0.1, -0.05, 1.5));
            var J = PlanarPnP.Jacobian(K, X, p); var u0 = PlanarPnP.Reproject(K, X, p); double h = 1e-6;
            for (int c = 0; c < 6; c++)
            {
                var xi = new double[6]; xi[c] = h;
                var u1 = PlanarPnP.Reproject(K, X, p.Perturbed(xi));
                for (int r = 0; r < 8; r++) Assert.Equal((u1[r] - u0[r]) / h, J[r, c], 3);
            }
        }

        [Fact]
        public void PoseError_GrowsWithRange()
        {
            double[] ranges = { 0.8, 1.4, 2.2 }; var med = new List<double>();
            foreach (double r in ranges)
                med.Add(Median(Enumerable.Range(0, 300).Select(k => { var s = Shoot(r, 25, 7, k); return Err(s.det, s.trueCamTag).pos; })));
            Assert.True(med[0] < med[1] && med[1] < med[2], string.Join(", ", med));
            Assert.True(med[2] / med[0] > 2.0);   // depth error grows roughly with range squared
        }

        [Fact]
        public void Obliquity_WithNoiseGain_RaisesTranslationErrorMonotonically()
        {
            var np = Clean(); np.ObliquityNoiseGain = 0.5;
            var med = new[] { 20.0, 45.0, 65.0 }.Select(a => Median(Enumerable.Range(0, 500).Select(k => { var s = Shoot(1.2, a, 13, k, 0.12, np); return Err(s.det, s.trueCamTag).pos; }))).ToArray();
            Assert.True(med[0] < med[1] && med[1] < med[2], string.Join(", ", med));
        }

        [Fact]
        public void Obliquity_PureGeometry_TranslationErrorIsFlat_AndRotationErrorFalls()
        {
            var np = Clean(); np.ObliquityNoiseGain = 0;
            double[] angles = { 25, 45, 65 };
            var pos = angles.Select(a => Median(Enumerable.Range(0, 500).Select(k => { var s = Shoot(1.2, a, 14, k, 0.12, np); return Err(s.det, s.trueCamTag).pos; }))).ToArray();
            var rot = angles.Select(a => Median(Enumerable.Range(0, 500).Select(k => { var s = Shoot(1.2, a, 14, k, 0.12, np); return Err(s.det, s.trueCamTag).rot; }))).ToArray();
            Assert.InRange(pos[2] / pos[0], 0.9, 1.1);
            Assert.True(rot[0] > rot[1] && rot[1] > rot[2], string.Join(", ", rot));
        }

        [Fact]
        public void NearFrontalView_ShowsPlanarPoseAmbiguity_ThatSteeperViewsDoNot()
        {
            int Count(double deg, Func<TagDetection, Pose, bool> f) => Enumerable.Range(0, 600).Count(k => { var s = Shoot(1.2, deg, 21, k, 0.12, Clean(1.0)); return f(s.det, s.trueCamTag); });
            int ambF = Count(2, (d, t) => d.Pnp.Ambiguous), ambO = Count(50, (d, t) => d.Pnp.Ambiguous);
            int wrongF = Count(2, (d, t) => Err(d, t).rot > 15 * Math.PI / 180), wrongO = Count(50, (d, t) => Err(d, t).rot > 15 * Math.PI / 180);
            Assert.True(ambF > 2 * Math.Max(ambO, 1), $"flag frontal {ambF} oblique {ambO}");
            Assert.True(wrongF > 30, $"wrong-branch frontal {wrongF}/600");
            Assert.True(wrongF > 3 * Math.Max(wrongO, 1), $"wrong-branch frontal {wrongF} oblique {wrongO}");
        }

        [Fact]
        public void AmbiguityFlag_CatchesNearlyAllWrongBranchSolutions()
        {
            int wrong = 0, wrongUnflagged = 0;
            for (int k = 0; k < 3000; k++)
            {
                var s = Shoot(1.2, k % 2 == 0 ? 5 : 40, 22, k, 0.12);
                if (Err(s.det, s.trueCamTag).rot > 20 * Math.PI / 180) { wrong++; if (!s.det.Pnp.Ambiguous) wrongUnflagged++; }
            }
            Assert.True(wrong > 50); Assert.True(wrongUnflagged <= 0.02 * wrong, $"{wrongUnflagged}/{wrong} wrong-branch solutions unflagged");
        }

        [Fact]
        public void Covariance_IsCalibrated_NeesMeanNearSix()
        {
            double sum = 0; int n = 0;
            for (int k = 0; k < 3000; k++)
            {
                var s = Shoot(1.2, 40, 31, k, 0.12);
                if (s.det.Pnp.Ambiguous) continue;
                sum += s.det.CovarianceCam.Mahalanobis(s.det.CamTag.ErrorFrom(s.trueCamTag)); n++;
            }
            Assert.True(n > 1500);
            double mean = sum / n;
            Assert.InRange(mean, 5.5, 6.6);   // E = 6 for 6 dof; first-order covariance and nonlinear PnP leave a small excess
        }

        [Fact]
        public void Covariance_NormalisedErrorFractionWithin95PercentBound()
        {
            int inside = 0, n = 0;
            for (int k = 0; k < 2000; k++)
            {
                var s = Shoot(1.5, 35, 32, k, 0.12);
                if (s.det.Pnp.Ambiguous) continue;
                n++; if (s.det.CovarianceCam.Mahalanobis(s.det.CamTag.ErrorFrom(s.trueCamTag)) < 12.592) inside++;   // chi2(6) 95%
            }
            Assert.InRange(inside / (double)n, 0.93, 0.975);
        }

        [Fact]
        public void Covariance_IsCalibrated_AlsoWithObliquityGainAndBlur()
        {
            var np = Clean(); np.ObliquityNoiseGain = 0.5; np.ExposureS = 0.004; double sum = 0; int n = 0;
            for (int k = 0; k < 3000; k++)
            {
                var s = Shoot(1.0, 50, 33, k, 0.12, np, new BodyMotion(new Vec3(0, 0.3, 0), Vec3.Zero));
                if (s.det.Pnp.Ambiguous) continue;
                sum += s.det.CovarianceCam.Mahalanobis(s.det.CamTag.ErrorFrom(s.trueCamTag)); n++;
            }
            Assert.True(n > 1500); Assert.InRange(sum / n, 5.5, 6.6);
        }

        [Fact]
        public void Covariance_ScalesWithSigmaSquared()
        {
            Pose cam = TestEnv.CamAt(0, 0, 0.3), tag = TestEnv.TagFacingCamera(1.2, 0, 0.3, 0.5); var truth = cam.Inverse().Compose(tag); var X = TagSpec.LocalCorners(0.12);
            var a = PlanarPnP.Covariance(TestEnv.K(), X, truth, 1.0); var b = PlanarPnP.Covariance(TestEnv.K(), X, truth, 2.0);
            for (int i = 0; i < 6; i++) Assert.Equal(4.0, b[i, i] / a[i, i], 9);
        }

        [Fact]
        public void Covariance_PositionUncertaintyAlongSightlineExceedsCrossRange()
        {
            var c = Shoot(1.5, 20, 1, 0, 0.12).det.CovarianceCam;   // camera axes: z = line of sight
            Assert.True(c[2, 2] > 4 * c[0, 0]);
        }

        [Fact]
        public void MotionBlur_RaisesCornerNoiseInProportionToImageVelocityAndExposure()
        {
            var np = Clean(1.0); np.ExposureS = 0.004;
            var still = Shoot(1.0, 20, 1, 0, 0.12, np).det;
            var fast = Shoot(1.0, 20, 1, 0, 0.12, np, new BodyMotion(new Vec3(0, 0.5, 0), Vec3.Zero)).det;   // 0.5 m/s lateral at 1 m
            Assert.Equal(1.0, still.CornerSigmaPx, 6); Assert.True(fast.CornerSigmaPx > still.CornerSigmaPx);
            double expectBlur = TestEnv.K().Fx * 0.5 / 1.0 * 0.004;
            Assert.Equal(expectBlur, fast.BlurPx, 0);
            np.ExposureS = 0.008; var slow2 = Shoot(1.0, 20, 1, 0, 0.12, np, new BodyMotion(new Vec3(0, 0.5, 0), Vec3.Zero)).det;
            Assert.Equal(2.0, slow2.BlurPx / fast.BlurPx, 3);
        }

        [Fact]
        public void MotionBlur_IncreasesPoseCovariance()
        {
            var np = Clean(1.0); np.ExposureS = 0.004;
            var still = Shoot(1.0, 20, 1, 0, 0.12, np).det; var fast = Shoot(1.0, 20, 1, 0, 0.12, np, new BodyMotion(new Vec3(0, 0.8, 0), Vec3.Zero)).det;
            Assert.True(fast.CovarianceCam.Trace() > still.CovarianceCam.Trace());
        }

        [Fact]
        public void Dropout_RateMatchesBaseProbabilityForLargeTags_AndRisesNearPixelLimit()
        {
            var np = Clean(); np.BaseDropout = 0.10; np.RampMax = 0.4;
            int Drops(double size) => Enumerable.Range(0, 4000).Count(k => { var cam = TestEnv.CamAt(0, 0, 0.3); var tag = TestEnv.TagFacingCamera(1.0, 0, 0.3);
                var vp = new VisibilityParams { MinPixels = 32 };
                return TagObserver.Detect(cam, BodyMotion.Still, TestEnv.K(), tag, BodyMotion.Still, Tag(size), null, vp, np, 0.03, 0, Rng.ForStream(5, "d/" + k)).Dropped; });
            double big = Drops(0.12) / 4000.0, small = Drops(0.045) / 4000.0;   // 91 px vs 34 px at 1 m
            Assert.InRange(big, 0.085, 0.115); Assert.True(small > 0.3, small.ToString());
        }

        [Fact]
        public void Timestamps_AcquisitionIsSampleTime_DeliveryAddsCameraLatency()
        {
            var np = Clean(); np.LatencyJitterS = 0;
            var d = Shoot(1.0, 20, 1, 0, 0.12, np).det;
            Assert.Equal(1.0, d.StampAcquired, 12); Assert.Equal(1.03, d.StampDelivered, 9);
        }

        [Fact]
        public void LatencyJitter_HasConfiguredSpread_AndNeverDeliversBeforeAcquisition()
        {
            var np = Clean(); np.LatencyJitterS = 0.004;
            var lat = Enumerable.Range(0, 2000).Select(k => { var d = Shoot(1.0, 20, 3, k, 0.12, np).det; return d.StampDelivered - d.StampAcquired; }).ToArray();
            double mean = lat.Average(), sd = Math.Sqrt(lat.Select(x => (x - mean) * (x - mean)).Average());
            Assert.InRange(mean, 0.0295, 0.0305); Assert.InRange(sd, 0.0035, 0.0045); Assert.True(lat.All(x => x >= 0));
        }

        [Fact]
        public void FrameSchedule_RateLimitsAndIsMonotone()
        {
            var s = new FrameSchedule(60, 0.003);
            Assert.Equal(-1, s.LastIndexAtOrBefore(0.0));
            Assert.Equal(0, s.LastIndexAtOrBefore(0.003));
            long frames = s.LastIndexAtOrBefore(1.003) - s.LastIndexAtOrBefore(0.003);
            Assert.Equal(60, frames);
            Assert.Equal(0.003 + 5.0 / 60, s.TimeOf(5), 12);
        }

        [Fact]
        public void Detection_IsDeterministicUnderSeed_AndDiffersAcrossSeeds()
        {
            var a = Shoot(1.2, 30, 99, 4).det; var b = Shoot(1.2, 30, 99, 4).det; var c = Shoot(1.2, 30, 100, 4).det;
            Assert.Equal(a.CamTag.P.X, b.CamTag.P.X); Assert.Equal(a.CamTag.R.M02, b.CamTag.R.M02);
            Assert.NotEqual(a.CamTag.P.X, c.CamTag.P.X);
        }

        [Fact]
        public void Rng_StreamsAreIndependentAndStable()
        {
            var a = Rng.ForStream(1, "a"); var b = Rng.ForStream(1, "b"); var a2 = Rng.ForStream(1, "a");
            Assert.Equal(a.NextU64(), a2.NextU64()); Assert.NotEqual(Rng.ForStream(1, "a").NextU64(), b.NextU64());
            var g = Rng.ForStream(2, "g"); var xs = Enumerable.Range(0, 20000).Select(_ => g.Gaussian()).ToArray();
            Assert.InRange(xs.Average(), -0.03, 0.03); Assert.InRange(xs.Select(x => x * x).Average(), 0.96, 1.04);
        }

        [Fact]
        public void Aging_CovarianceGrowsLinearlyWithAge_AndAgeFieldIsSet()
        {
            var o = Fresh(); var drift = new DeadReckoningDrift(1e-4, 1e-5);
            var a1 = ObservationAging.Age(o, 10.5, drift); var a2 = ObservationAging.Age(o, 11.0, drift); var a0 = ObservationAging.Age(o, 10.0, drift);
            Assert.Equal(0, a0.Age); Assert.Equal(0.5, a1.Age, 12); Assert.Equal(1.0, a2.Age, 12);
            Assert.Equal(o.Covariance[0, 0], a0.Covariance[0, 0], 12);
            double g1 = a1.Covariance[0, 0] - o.Covariance[0, 0], g2 = a2.Covariance[0, 0] - o.Covariance[0, 0];
            Assert.Equal(2.0, g2 / g1, 9);
            Assert.True(a2.Covariance[5, 5] > o.Covariance[5, 5]);
            Assert.Equal(10.0, a2.StampAcquired);   // acquisition stamp never changes when ageing
        }

        [Fact]
        public void Aging_YawDrift_AddsLeverArmPositionVariance()
        {
            var o = Fresh(); o.Pose = new Pose(Mat3.Identity, new Vec3(0, 2.0, 0));   // lever 2 m along y
            var a = ObservationAging.Age(o, 12.0, new DeadReckoningDrift(0, 1e-4));
            // yaw variance 1e-4*2 -> x position variance (lever^2)*that
            Assert.Equal(o.Covariance[0, 0] + 4.0 * 2e-4, a.Covariance[0, 0], 9);
        }

        [Fact]
        public void Aging_ConsumerMotion_ReExpressesPose()
        {
            var o = Fresh(); o.Pose = new Pose(Mat3.Identity, new Vec3(2, 0, 0));
            var moved = Pose.FromYaw(0.5, 0, 0, 0);   // consumer drove 0.5 m forward
            var a = ObservationAging.Age(o, 10.2, new DeadReckoningDrift(0, 0), moved);
            Assert.Equal(1.5, a.Pose.P.X, 12);
        }

        [Fact]
        public void Aging_RejectsTimeBeforeAcquisition()
        {
            Assert.Throws<ArgumentException>(() => ObservationAging.Age(Fresh(), 9.0, new DeadReckoningDrift(1, 1)));
        }

        private static RelPoseObservation Fresh() => new RelPoseObservation
        {
            StampAcquired = 10.0, StampDelivered = 10.05, SourceId = "x", SourceKind = ObservationSourceKind.OwnCamera, FrameParent = "a/base_link", FrameChild = "b/base_link",
            Pose = new Pose(Mat3.Identity, new Vec3(1, 0, 0)), Covariance = Matrix.Diag(1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4)
        };
    }
}
