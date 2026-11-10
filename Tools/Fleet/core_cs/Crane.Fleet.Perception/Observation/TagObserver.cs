using System;
using System.Collections.Generic;

namespace Crane.Fleet.Perception
{
    /// <summary>Noise/latency parameters of the AprilTag detection chain. All values are ASSUMPTIONS unless noted (see README).</summary>
    public sealed class TagNoiseParams
    {
        /// <summary>Corner localisation noise (1 sigma, per axis) in pixels for a sharp, well-sized tag. 1.0 px reproduces the literature
        /// anchor "~2 cm at 2 m for a 16.4 cm tag" (arXiv 2502.03510) used by docking_mc to within a factor of about 1.5 on the dominant (range) axis.</summary>
        public double CornerSigmaPx = 1.0;
        /// <summary>
        /// Extra corner noise for foreshortened tags: sigma *= 1 + gain x (1/cos(view angle) - 1). Pure geometry makes translation error independent of obliquity and
        /// rotation error FALL with obliquity (tilt becomes observable); real detectors also localise corners worse on foreshortened, low-contrast edges.
        /// 0.5 is an ASSUMPTION (gives x1.7 at 65 degrees); set 0 for geometry only. Replace with the measured corner reprojection RMS versus view angle.
        /// </summary>
        public double ObliquityNoiseGain = 0.5;
        /// <summary>Exposure time for motion blur (global shutter: blur length = image speed x exposure; no row skew).</summary>
        public double ExposureS = 0.004;
        /// <summary>Extra corner sigma = coeff x blur length (px). ASSUMPTION; a blur of L px shifts an edge centroid by about L/sqrt(12) = 0.29 L.</summary>
        public double BlurCornerCoeff = 0.29;
        /// <summary>Per-frame probability that a geometrically visible tag is still missed (docking_mc tag_dropout = 0.10).</summary>
        public double BaseDropout = 0.10;
        /// <summary>Extra dropout ramp as the tag approaches the pixel-size limit: adds up to RampMax at PixelSize = MinPixels, linearly to 0 at RampStartFactor x MinPixels.</summary>
        public double RampMax = 0.4;
        public double RampStartFactor = 2.0;
        /// <summary>Camera-to-delivery latency (s) one sigma jitter on top of the profile latency.</summary>
        public double LatencyJitterS = 0.002;
        /// <summary>Additional transport delay (s) from the detector node to the consumer (ASSUMPTION; profile latency is camera-side).</summary>
        public double TransportDelayS = 0.0;

        public double DropoutProbability(VisibilityResult v, VisibilityParams vp)
        {
            double p = BaseDropout;
            double start = RampStartFactor * vp.MinPixels;
            if (v.PixelSize < start) p += RampMax * Math.Min(1.0, (start - v.PixelSize) / Math.Max(start - vp.MinPixels, 1e-9));
            return Math.Min(0.99, p);
        }
    }

    /// <summary>World-frame linear and angular velocity of a body (used only for motion blur).</summary>
    public readonly struct BodyMotion
    {
        public readonly Vec3 Lin, Ang;
        public BodyMotion(Vec3 lin, Vec3 ang) { Lin = lin; Ang = ang; }
        public static readonly BodyMotion Still = new BodyMotion(Vec3.Zero, Vec3.Zero);
        public Pose Advance(Pose p, double dt) => new Pose(Mat3.Exp(Ang * dt) * p.R, p.P + Lin * dt);
    }

    /// <summary>Periodic frame clock: frame k is acquired at phase + k / rate.</summary>
    public sealed class FrameSchedule
    {
        public readonly double RateHz, Phase;
        public FrameSchedule(double rateHz, double phase = 0) { if (rateHz <= 0) throw new ArgumentOutOfRangeException(nameof(rateHz)); RateHz = rateHz; Phase = phase; }
        public double TimeOf(long k) => Phase + k / RateHz;
        /// <summary>Index of the last frame acquired at or before t (may be negative before the first frame).</summary>
        public long LastIndexAtOrBefore(double t) => (long)Math.Floor((t - Phase) * RateHz + 1e-9);
    }

    public sealed class TagDetection
    {
        public int TagId; public VisibilityResult Visibility; public bool Dropped;
        public bool Detected => Visibility.Visible && !Dropped && Pnp != null && Pnp.Ok;
        public double[] NoisyCornersPx; public PnPResult Pnp;
        public double CornerSigmaPx; public double BlurPx;
        public Pose CamTag => Pnp.CamTag;                  // estimated T_cam_tag
        public Matrix CovarianceCam => Pnp.Covariance;     // 6x6 in camera axes
        public double StampAcquired, StampDelivered;
    }

    /// <summary>One camera's detection of one tag, from true geometry in, noisy estimate with covariance out.</summary>
    public static class TagObserver
    {
        public static TagDetection Detect(Pose camWorld, BodyMotion camMotion, CameraIntrinsics K, Pose tagWorld, BodyMotion tagMotion, TagSpec tag,
            IReadOnlyList<Obb> occluders, VisibilityParams vp, TagNoiseParams np, double cameraLatencyS, double tAcquire, Rng rng)
        {
            var det = new TagDetection { TagId = tag.Id, StampAcquired = tAcquire };
            det.Visibility = TagVisibility.Evaluate(camWorld, K, tagWorld, tag.Size, occluders, vp);
            double jitter = rng.Gaussian() * np.LatencyJitterS;       // always drawn: keeps the random stream aligned across outcomes
            det.StampDelivered = tAcquire + Math.Max(0, cameraLatencyS + jitter) + np.TransportDelayS;
            double uDrop = rng.NextDouble();
            if (!det.Visibility.Visible) return det;
            det.Dropped = uDrop < np.DropoutProbability(det.Visibility, vp);
            if (det.Dropped) return det;

            double blurPx = ImageSpeedPx(camWorld, camMotion, K, tagWorld, tagMotion, tag.Size) * np.ExposureS;
            double baseSigma = np.CornerSigmaPx * (1 + np.ObliquityNoiseGain * (1.0 / Math.Max(Math.Cos(det.Visibility.ViewAngle), 0.2) - 1));
            double sigma = Math.Sqrt(baseSigma * baseSigma + Math.Pow(np.BlurCornerCoeff * blurPx, 2));
            det.BlurPx = blurPx; det.CornerSigmaPx = sigma;
            var uv = (double[])det.Visibility.CornersPx.Clone();
            for (int i = 0; i < 8; i++) uv[i] += sigma * rng.Gaussian();
            det.NoisyCornersPx = uv;
            det.Pnp = PlanarPnP.Solve(K, TagSpec.LocalCorners(tag.Size), uv, sigma);
            return det;
        }

        /// <summary>Mean corner speed in the image (px/s) from finite differences of the relative motion.</summary>
        public static double ImageSpeedPx(Pose camWorld, BodyMotion camMotion, CameraIntrinsics K, Pose tagWorld, BodyMotion tagMotion, double tagSize)
        {
            const double dt = 1e-3;
            var X = TagSpec.LocalCorners(tagSize);
            Pose c0 = camWorld.Inverse(), c1 = camMotion.Advance(camWorld, dt).Inverse(), t1 = tagMotion.Advance(tagWorld, dt);
            double s = 0; int n = 0;
            for (int i = 0; i < 4; i++)
            {
                if (!K.Project(c0.Apply(tagWorld.Apply(X[i])), out double u0, out double v0)) continue;
                if (!K.Project(c1.Apply(t1.Apply(X[i])), out double u1, out double v1)) continue;
                s += Math.Sqrt((u1 - u0) * (u1 - u0) + (v1 - v0) * (v1 - v0)) / dt; n++;
            }
            return n == 0 ? 0 : s / n;
        }
    }
}
