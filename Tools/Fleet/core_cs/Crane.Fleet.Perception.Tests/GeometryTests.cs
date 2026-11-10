using System;
using System.Collections.Generic;
using Xunit;

namespace Crane.Fleet.Perception.Tests
{
    public class GeometryTests
    {
        private static readonly VisibilityParams VP = new VisibilityParams();
        private static VisibilityResult Eval(Pose cam, Pose tag, double size, params Obb[] occ) => TagVisibility.Evaluate(cam, TestEnv.K(), tag, size, occ, VP);

        [Fact]
        public void Intrinsics_FromHfov_RoundTripsAndPrincipalPointIsCentre()
        {
            var K = CameraIntrinsics.FromHfov(1280, 800, 1.40);
            Assert.Equal(1.40, K.Hfov, 12);
            Assert.Equal(640, K.Cx); Assert.Equal(400, K.Cy);
            Assert.Equal(1280 / (2 * Math.Tan(0.7)), K.Fx, 9);
        }

        [Fact]
        public void Project_OnAxisPoint_LandsOnPrincipalPoint()
        {
            var K = TestEnv.K(); Assert.True(K.Project(new Vec3(0, 0, 3), out double u, out double v));
            Assert.Equal(K.Cx, u, 9); Assert.Equal(K.Cy, v, 9);
        }

        [Fact]
        public void Project_HalfFovRay_LandsOnImageEdge()
        {
            var K = TestEnv.K(); double z = 2; Assert.True(K.Project(new Vec3(z * Math.Tan(0.7), 0, z), out double u, out _));
            Assert.Equal(1280, u, 6);
        }

        [Fact]
        public void Project_BehindCamera_IsRejected()
        {
            Assert.False(TestEnv.K().Project(new Vec3(0, 0, -1), out _, out _));
        }

        [Fact]
        public void Project_UsesImageYDown()
        {
            var K = TestEnv.K(); K.Project(new Vec3(0, 0.1, 1), out _, out double v); Assert.True(v > K.Cy);
        }

        [Fact]
        public void RoverCameraMount_OpticalAxisPointsAlongRobotForward()
        {
            var cam = TestEnv.CamAt(0, 0, 0.3); Vec3 fwd = cam.R.Col(2); Assert.Equal(1, fwd.X, 12);
            Assert.Equal(-1, cam.R.Col(1).Z, 12);   // optical y points down
            Assert.Equal(-1, cam.R.Col(0).Y, 12);   // optical x points to the robot's right
        }

        [Fact]
        public void Visible_FrontalTagInFront()
        {
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05);
            Assert.Equal(VisibilityReason.Visible, r.Reason); Assert.Equal(1.0, r.Range, 9);
        }

        [Fact]
        public void PixelSize_FrontalTagMatchesFocalLengthTimesSizeOverRange()
        {
            var K = TestEnv.K(); var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05);
            Assert.Equal(K.Fx * 0.05 / 1.0, r.PixelSize, 6);
        }

        [Fact]
        public void TagBehindCamera_NotVisible()
        {
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(-1.0, 0, 0.3, 0), 0.05);
            Assert.False(r.Visible);
        }

        [Fact]
        public void TagOutsideFov_NotVisible()
        {
            // 1.0 m ahead, 1.2 m to the left: bearing 50 deg > 40 deg half-FOV
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 1.2, 0.3), 0.05);
            Assert.Equal(VisibilityReason.OutOfFov, r.Reason);
        }

        [Fact]
        public void TagStraddlingImageEdge_NotVisible_AllFourCornersMustBeInside()
        {
            var K = TestEnv.K(); double z = 1.0, yEdge = z * Math.Tan(0.7);   // left edge of the image: lateral = +yEdge (robot left)
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(z, yEdge, 0.3), 0.05);
            Assert.False(r.Visible);
            var inside = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(z, yEdge - 0.06, 0.3), 0.05);
            Assert.True(inside.Visible);
        }

        [Fact]
        public void MaxRange_FromPixelSize_IsAnalytic()
        {
            var K = TestEnv.K(); double rmax = K.MaxRangeFrontal(0.05, VP.MinPixels);
            Assert.True(Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(rmax * 0.99, 0, 0.3), 0.05).Visible);
            var far = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(rmax * 1.01, 0, 0.3), 0.05);
            Assert.Equal(VisibilityReason.TooSmall, far.Reason);
        }

        [Fact]
        public void MinPixelsParameter_ScalesMaximumRange()
        {
            var vp = new VisibilityParams { MinPixels = 64 };
            double rmax32 = TestEnv.K().MaxRangeFrontal(0.05, 32), rmax64 = TestEnv.K().MaxRangeFrontal(0.05, 64);
            Assert.Equal(2.0, rmax32 / rmax64, 9);
            var r = TagVisibility.Evaluate(TestEnv.CamAt(0, 0, 0.3), TestEnv.K(), TestEnv.TagFacingCamera(rmax64 * 1.2, 0, 0.3), 0.05, null, vp);
            Assert.Equal(VisibilityReason.TooSmall, r.Reason);
            Assert.True(Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(rmax64 * 1.2, 0, 0.3), 0.05).Visible);
        }

        [Fact]
        public void TopTag_DefaultRangeExceedsFaceTagRange()
        {
            var K = TestEnv.K();
            Assert.True(K.MaxRangeFrontal(0.12, VP.MinPixels) > 2 * K.MaxRangeFrontal(0.05, VP.MinPixels));
        }

        [Fact]
        public void MinimumRange_TooClose_NotVisible()
        {
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(0.08, 0, 0.3), 0.02);
            Assert.False(r.Visible);
        }

        [Theory]
        [InlineData(30, true)]
        [InlineData(60, true)]
        [InlineData(70, false)]
        [InlineData(85, false)]
        public void ViewingAngleLimit(double deg, bool visible)
        {
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3, deg * Math.PI / 180), 0.12);
            Assert.Equal(visible, r.Visible);
            Assert.Equal(deg * Math.PI / 180, r.ViewAngle, 6);
        }

        [Fact]
        public void BackFacingTag_NotVisible()
        {
            var tagAway = TagSpec.OnVerticalFace(new Vec3(1, 0, 0.3), 0.0);   // normal +x, away from camera
            Assert.False(Eval(TestEnv.CamAt(0, 0, 0.3), tagAway, 0.05).Visible);
        }

        [Fact]
        public void Occluder_BetweenCameraAndTag_Blocks()
        {
            var box = new Obb("b", new Vec3(0.5, 0, 0.3), new Vec3(0.05, 0.3, 0.3));
            var r = Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05, box);
            Assert.Equal(VisibilityReason.Occluded, r.Reason); Assert.Equal(4, r.BlockedCount);
        }

        [Fact]
        public void Occluder_BehindTag_DoesNotBlock()
        {
            var box = new Obb("b", new Vec3(1.5, 0, 0.3), new Vec3(0.05, 0.3, 0.3));
            Assert.True(Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05, box).Visible);
        }

        [Fact]
        public void PartialOcclusion_OneCornerBlocked_IsNoDetection()
        {
            // pillar at x = 0.5 covers the ray to exactly one corner (lower corner on the robot's left)
            var tag = TestEnv.TagFacingCamera(1.0, 0, 0.3);
            var corners = TagSpec.WorldCorners(tag, 0.1);
            Vec3 c = corners[0]; double s = 0.5 / c.X;   // point of that corner ray at x = 0.5
            Vec3 hit = new Vec3(0.5, c.Y * s, 0.3 + (c.Z - 0.3) * s);
            var pillar = new Obb("p", hit, new Vec3(0.01, 0.002, 0.002));
            var r = TagVisibility.Evaluate(TestEnv.CamAt(0, 0, 0.3), TestEnv.K(), tag, 0.1, new[] { pillar }, VP);
            Assert.Equal(1, r.BlockedCount); Assert.True(r.PartiallyOccluded); Assert.False(r.Visible);
        }

        [Fact]
        public void ElevatedBeam_BlocksRaysThroughItsHeightRange_NotRaysBelow()
        {
            var beam = new Obb("gantry_beam", new Vec3(4.5, 3.4, 2.3), new Vec3(1.2, 0.15, 0.15));
            var tagHigh = TestEnv.TagFacingCamera(6.5, 3.4, 2.3);
            var tagLow = TestEnv.TagFacingCamera(6.5, 3.4, 1.0);
            Assert.Equal(VisibilityReason.Occluded, Eval(TestEnv.CamAt(2.0, 3.4, 2.3), tagHigh, 0.4, beam).Reason);
            Assert.True(Eval(TestEnv.CamAt(2.0, 3.4, 1.0), tagLow, 0.4, beam).Visible);                       // under the beam
            Assert.True(Eval(TestEnv.CamAt(2.0, 3.4, 2.9), TestEnv.TagFacingCamera(6.5, 3.4, 2.9), 0.4, beam).Visible);   // above the beam
        }

        [Fact]
        public void ElevatedBeam_DoesNotBlockDriving_ButLidarBeamIsBelowIt()
        {
            var scene = TestEnv.Scene();
            Assert.Contains(scene.ElevatedOnly, b => b.Id == "gantry_beam");
            Assert.DoesNotContain(scene.ElevatedOnly, b => b.Id == "shelf_a");
        }

        [Fact]
        public void OrientedBox_BlocksAlongItsRotatedFootprint()
        {
            var slanted = new Obb("s", new Vec3(0.5, 0.0, 0.3), new Vec3(0.02, 0.5, 0.3), Math.PI / 4);
            Assert.Equal(VisibilityReason.Occluded, Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05, slanted).Reason);
            var off = new Obb("s", new Vec3(0.5, 1.5, 0.3), new Vec3(0.02, 0.5, 0.3), Math.PI / 4);
            Assert.True(Eval(TestEnv.CamAt(0, 0, 0.3), TestEnv.TagFacingCamera(1.0, 0, 0.3), 0.05, off).Visible);
        }

        [Fact]
        public void TagOnOwnerBodySurface_IsNotSelfOccluded_EvenAtMaxViewAngle()
        {
            var g = new RoverGeometry(TestEnv.Profile(), "r", 1);
            var body = g.BodyBox(Pose.Identity, "r/body");
            var face = g.Tags[0];   // front face tag
            Pose tagW = face.MountInOwner;
            // camera 0.6 m away at 60 degrees off the face normal
            double a = 60 * Math.PI / 180; Vec3 n = tagW.R.Col(2);
            Vec3 camPos = tagW.P + new Vec3(Math.Cos(a) * n.X, Math.Sin(a), 0) * 0.6 + new Vec3(0, 0, 0);
            var cam = CameraMount.LookAt(camPos, tagW.P);
            var r = TagVisibility.Evaluate(cam, TestEnv.K(), tagW, face.Size, new[] { body }, VP);
            Assert.Equal(VisibilityReason.Visible, r.Reason);
        }

        [Fact]
        public void RoverBody_BlocksViewOfTagOnItsFarSide()
        {
            var g = new RoverGeometry(TestEnv.Profile(), "r", 1); var body = g.BodyBox(Pose.Identity, "r/body");
            var rear = g.Tags[1].MountInOwner;   // rear face, normal -x
            var cam = CameraMount.LookAt(new Vec3(1.0, 0, 0.06), rear.P);
            var r = TagVisibility.Evaluate(cam, TestEnv.K(), rear, g.Tags[1].Size, new[] { body }, VP);
            Assert.False(r.Visible);
        }

        [Fact]
        public void LookAtCamera_PutsTargetOnPrincipalPoint()
        {
            var cam = CameraMount.LookAt(new Vec3(2, 3.5, 2.6), new Vec3(3.5, 3.5, 0));
            Assert.True(TestEnv.K().Project(cam.Inverse().Apply(new Vec3(3.5, 3.5, 0)), out double u, out double v));
            Assert.Equal(640, u, 6); Assert.Equal(400, v, 6);
        }

        [Fact]
        public void NoOccluders_NullList_Allowed()
        {
            Assert.True(TagVisibility.Evaluate(TestEnv.CamAt(0, 0, 0.3), TestEnv.K(), TestEnv.TagFacingCamera(1, 0, 0.3), 0.05, null, VP).Visible);
        }
    }
}
