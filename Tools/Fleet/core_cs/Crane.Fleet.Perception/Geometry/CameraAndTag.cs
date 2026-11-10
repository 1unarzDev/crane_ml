using System;
using System.Collections.Generic;

namespace Crane.Fleet.Perception
{
    /// <summary>
    /// Pinhole intrinsics. Optical frame: x right, y down, z forward. Pixel origin at the image's top-left
    /// corner (pixel centres at half integers), so the principal point is (W/2, H/2).
    /// </summary>
    public sealed class CameraIntrinsics
    {
        public int Width, Height; public double Fx, Fy, Cx, Cy;
        public static CameraIntrinsics FromHfov(int w, int h, double hfov)
        {
            if (hfov <= 0 || hfov >= Math.PI) throw new ArgumentOutOfRangeException(nameof(hfov));
            double f = w / (2 * Math.Tan(hfov / 2));
            return new CameraIntrinsics { Width = w, Height = h, Fx = f, Fy = f, Cx = w / 2.0, Cy = h / 2.0 };
        }
        public double Hfov => 2 * Math.Atan(Width / (2 * Fx));
        public double Vfov => 2 * Math.Atan(Height / (2 * Fy));
        /// <summary>Project a point given in the optical frame. Returns false when it is not in front of the camera.</summary>
        public bool Project(Vec3 pc, out double u, out double v)
        {
            if (pc.Z <= 1e-9) { u = v = double.NaN; return false; }
            u = Fx * pc.X / pc.Z + Cx; v = Fy * pc.Y / pc.Z + Cy; return true;
        }
        public bool InImage(double u, double v, double margin = 0) => u >= margin && u <= Width - margin && v >= margin && v <= Height - margin;
        /// <summary>Largest range at which a frontal tag of this size still spans <paramref name="minPixels"/> pixels (on-axis).</summary>
        public double MaxRangeFrontal(double tagSize, double minPixels) => Fx * tagSize / minPixels;
    }

    /// <summary>Camera optical-frame mounting helpers. Robot frame: x forward, y left, z up.</summary>
    public static class CameraMount
    {
        /// <summary>Optical axes (x right, y down, z forward) expressed in the robot frame for a camera looking along +x.</summary>
        public static readonly Mat3 OpticalFromRobot = Mat3.FromColumns(new Vec3(0, -1, 0), new Vec3(0, 0, -1), new Vec3(1, 0, 0));

        /// <summary>Pose of the optical frame in the rover base frame. Yaw about +z (left positive), pitchDown about the camera's lateral axis.</summary>
        public static Pose InRover(Vec3 position, double yaw, double pitchDown = 0)
            => new Pose(Mat3.Rz(yaw) * Mat3.Ry(pitchDown) * OpticalFromRobot, position);

        /// <summary>World-frame look-at camera with the world +z as "up" (stations, overhead cameras).</summary>
        public static Pose LookAt(Vec3 position, Vec3 target)
        {
            Vec3 fwd = (target - position).Normalized();
            Vec3 right = fwd.Cross(Vec3.UnitZ);
            if (right.Norm < 1e-6) right = Vec3.UnitX; else right = right.Normalized();
            Vec3 down = fwd.Cross(right);
            return new Pose(Mat3.FromColumns(right, down, fwd), position);
        }
    }

    /// <summary>A square AprilTag, defined by the edge of its black border ("tag size" in the AprilTag convention).</summary>
    public sealed class TagSpec
    {
        public int Id; public string OwnerId; public string Name; public double Size;
        /// <summary>Tag frame in the owner's base frame. Tag frame: x right, y up, z out of the visible face.</summary>
        public Pose MountInOwner;
        public string Provenance = "provisional";

        /// <summary>Corner order is the AprilTag one (counter-clockwise seen from the front, starting bottom-left).</summary>
        public static Vec3[] LocalCorners(double size)
        {
            double h = size / 2;
            return new[] { new Vec3(-h, -h, 0), new Vec3(h, -h, 0), new Vec3(h, h, 0), new Vec3(-h, h, 0) };
        }
        public static Vec3[] WorldCorners(Pose tagWorld, double size)
        {
            var l = LocalCorners(size); var w = new Vec3[4];
            for (int i = 0; i < 4; i++) w[i] = tagWorld.Apply(l[i]);
            return w;
        }

        /// <summary>Pose of a tag standing on a vertical face: normal along the horizontal heading, y up.</summary>
        public static Pose OnVerticalFace(Vec3 position, double normalYaw)
        {
            Vec3 n = new Vec3(Math.Cos(normalYaw), Math.Sin(normalYaw), 0), up = Vec3.UnitZ, x = up.Cross(n);
            return new Pose(Mat3.FromColumns(x, up, n), position);
        }
        /// <summary>Pose of a tag lying on a top face: normal +z, tag y along the owner's forward direction.</summary>
        public static Pose OnTopFace(Vec3 position, double forwardYaw)
        {
            Vec3 f = new Vec3(Math.Cos(forwardYaw), Math.Sin(forwardYaw), 0), z = Vec3.UnitZ, x = f.Cross(z);
            return new Pose(Mat3.FromColumns(x, f, z), position);
        }
    }

    /// <summary>Rover body and tag layout derived from the profile (mount poses are provisional until CAD).</summary>
    public sealed class RoverGeometry
    {
        public const double TagStandoff = 0.002;   // tag sits 2 mm proud of the face (occlusion epsilon must exceed standoff / cos(max view angle))
        public readonly Vec3 BodyHalf; public readonly double BodyCentreZ;
        public readonly List<TagSpec> Tags = new List<TagSpec>();

        public RoverGeometry(FleetProfile profile, string roverId, int roverIndex)
        {
            var b = profile.BodySizeLwh;
            BodyHalf = new Vec3(b[0] / 2, b[1] / 2, b[2] / 2); BodyCentreZ = b[2] / 2;   // base frame z = 0 on the floor, body from 0 to height
            int k = 0;
            foreach (var name in profile.ActiveFaces)
            {
                var f = profile.FaceFrames[name];
                double cx = Math.Cos(f.Yaw), sy = Math.Sin(f.Yaw);
                var pos = new Vec3(f.X + TagStandoff * cx, f.Y + TagStandoff * sy, f.Z);
                Tags.Add(new TagSpec { Id = 10 * roverIndex + k++, OwnerId = roverId, Name = "face_" + name, Size = profile.FaceTagSize, MountInOwner = TagSpec.OnVerticalFace(pos, f.Yaw), Provenance = f.Provenance });
            }
            Tags.Add(new TagSpec { Id = 10 * roverIndex + 9, OwnerId = roverId, Name = "top", Size = profile.TopTagSize, MountInOwner = TagSpec.OnTopFace(new Vec3(0, 0, b[2] + TagStandoff), 0) });
        }
        public Obb BodyBox(Pose roverWorld, string id) => new Obb(id, roverWorld.Apply(new Vec3(0, 0, BodyCentreZ)), BodyHalf, roverWorld.Yaw);
    }
}
