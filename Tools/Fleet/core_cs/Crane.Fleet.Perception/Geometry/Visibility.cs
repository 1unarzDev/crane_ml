using System;
using System.Collections.Generic;

namespace Crane.Fleet.Perception
{
    public enum VisibilityReason { Visible, TooOblique, Behind, OutOfFov, TooClose, TooSmall, Occluded }

    /// <summary>Detector-capability parameters for deciding whether a tag can be decoded at all.</summary>
    public sealed class VisibilityParams
    {
        /// <summary>
        /// Minimum projected tag size in pixels (square root of the projected quad's area; for a frontal tag this is the
        /// edge of the black border) needed to decode tag36h11. Default 32 px = 4 px per cell x 8 cells (black border
        /// spans 8 cells of the 10-cell tag36h11 pattern). Source: a community field test (FRC forum, "Pixels for AprilTag
        /// detection", 4 px per cell was the reliability limit); other guides quote 6 px (48 px) or ~10 px per cell. The
        /// Wang-Olson AprilTag 2 / Olson 2011 papers give no explicit pixel minimum. Treated as an ASSUMPTION to be
        /// replaced by a bench measurement of detection rate versus pixel size with the real camera and detector settings.
        /// </summary>
        public double MinPixels = 32.0;
        /// <summary>Largest angle between the tag normal and the line of sight. ASSUMPTION (matches observer_value); measure detection rate vs. obliquity.</summary>
        public double MaxViewAngle = 65.0 * Math.PI / 180.0;
        /// <summary>Closest range (camera centre to tag centre), e.g. lens minimum focus. ASSUMPTION.</summary>
        public double MinRange = 0.10;
        public double NearPlane = 0.01;
        /// <summary>Last metres of every corner ray ignored: the tag sits on the surface of its owner.</summary>
        public double OcclusionEps = 0.006;
        public double BorderMarginPx = 0.0;
    }

    public sealed class VisibilityResult
    {
        public VisibilityReason Reason; public bool Visible => Reason == VisibilityReason.Visible;
        public double Range, ViewAngle, PixelSize;
        public double[] CornersPx = new double[8];   // u0,v0,...,u3,v3 (valid when all corners project)
        public bool[] CornerBlocked = new bool[4];
        public int BlockedCount;
        public bool PartiallyOccluded => Reason == VisibilityReason.Occluded && BlockedCount > 0 && BlockedCount < 4;
    }

    public static class TagVisibility
    {
        /// <summary>
        /// Evaluate whether a camera can detect a tag. Check order (also followed by the Python parity model):
        /// back-face / view-angle, behind camera, outside image (all four corners must be inside), minimum range,
        /// projected size, then line of sight on each of the four corners (any blocked corner means no detection).
        /// </summary>
        public static VisibilityResult Evaluate(Pose cameraWorld, CameraIntrinsics K, Pose tagWorld, double tagSize,
            IReadOnlyList<Obb> occluders, VisibilityParams p)
        {
            var r = new VisibilityResult();
            Vec3 centre = tagWorld.P, normal = tagWorld.R.Col(2);
            Vec3 toCam = cameraWorld.P - centre;
            r.Range = toCam.Norm;
            double cosv = r.Range > 0 ? normal.Dot(toCam) / r.Range : -1;
            r.ViewAngle = Math.Acos(Math.Max(-1, Math.Min(1, cosv)));
            if (cosv <= 0 || r.ViewAngle > p.MaxViewAngle) { r.Reason = VisibilityReason.TooOblique; return r; }

            var cw = TagSpec.WorldCorners(tagWorld, tagSize);
            Pose wc = cameraWorld.Inverse();
            var uv = r.CornersPx;
            for (int i = 0; i < 4; i++)
            {
                Vec3 pc = wc.Apply(cw[i]);
                if (pc.Z < p.NearPlane || !K.Project(pc, out uv[2 * i], out uv[2 * i + 1])) { r.Reason = VisibilityReason.Behind; return r; }
            }
            for (int i = 0; i < 4; i++)
                if (!K.InImage(uv[2 * i], uv[2 * i + 1], p.BorderMarginPx)) { r.Reason = VisibilityReason.OutOfFov; return r; }
            r.PixelSize = PixelSize(uv);
            if (r.Range < p.MinRange) { r.Reason = VisibilityReason.TooClose; return r; }
            if (r.PixelSize < p.MinPixels) { r.Reason = VisibilityReason.TooSmall; return r; }

            if (occluders != null)
                for (int i = 0; i < 4; i++)
                {
                    foreach (var o in occluders)
                        if (o.Blocks(cameraWorld.P, cw[i], p.OcclusionEps)) { r.CornerBlocked[i] = true; break; }
                    if (r.CornerBlocked[i]) r.BlockedCount++;
                }
            r.Reason = r.BlockedCount > 0 ? VisibilityReason.Occluded : VisibilityReason.Visible;
            return r;
        }

        /// <summary>Square root of the shoelace area of the projected quad (equals the edge length for a frontal tag).</summary>
        public static double PixelSize(double[] uv)
        {
            double a = 0;
            for (int i = 0; i < 4; i++) { int j = (i + 1) & 3; a += uv[2 * i] * uv[2 * j + 1] - uv[2 * j] * uv[2 * i + 1]; }
            return Math.Sqrt(Math.Abs(a) * 0.5);
        }
    }
}
