using System.Collections.Generic;
using System.Xml.Linq;
using NUnit.Framework;
using Sim.Physics.Core;
using UnityEngine;

namespace Crane.Urdf.Tests
{
    public class UrdfPlacementTests
    {
        // base_link -> mast (yawed 90 deg) -> lidar_link; base_link -> box_link (no geometry) -> gps_link.
        const string Urdf = @"<robot name='t'>
  <link name='base_link'><visual><geometry><box size='1 1 1'/></geometry></visual></link>
  <link name='mast'/><link name='lidar_link'/><link name='box_link'/><link name='gps_link'/>
  <joint name='a' type='fixed'><parent link='base_link'/><child link='mast'/><origin xyz='1 0 0' rpy='0 0 1.5707963'/></joint>
  <joint name='b' type='fixed'><parent link='mast'/><child link='lidar_link'/><origin xyz='1 0 0.5'/></joint>
  <joint name='c' type='fixed'><parent link='base_link'/><child link='box_link'/><origin xyz='0 0 -0.0126'/></joint>
  <joint name='d' type='fixed'><parent link='box_link'/><child link='gps_link'/><origin xyz='0.07 -0.12 0.108'/></joint>
</robot>";

        readonly List<GameObject> made = new List<GameObject>();

        [TearDown]
        public void Clean() { foreach (var g in made) Object.DestroyImmediate(g); made.Clear(); }

        static UrdfFrames Parse(string xml)
        {
            var errors = new List<string>();
            var f = UrdfFrames.Parse(XDocument.Parse(xml), "test", errors);
            Assert.That(errors, Is.Empty, string.Join("\n", errors));
            return f;
        }

        Transform Node(string name, Transform parent)
        {
            var t = new GameObject(name).transform;
            if (parent != null) t.SetParent(parent, false); else made.Add(t.gameObject);
            return t;
        }

        [Test]
        public void RosAxesAndRpyMapToUnity()
        {
            Assert.That(UrdfFrames.M(new Vector3(1, 2, 3)), Is.EqualTo(new Vector3(-2, 3, 1)));
            // ROS yaw +90 (bow to port) is a turn to the left about Unity up.
            Assert.That(Quaternion.Angle(UrdfFrames.M(UrdfFrames.Rpy(new Vector3(0, 0, Mathf.PI / 2))), Quaternion.Euler(0, -90, 0)), Is.LessThan(1e-3f));
            // ROS pitch +90 (nose down) is Unity's positive pitch about x.
            Assert.That(Quaternion.Angle(UrdfFrames.M(UrdfFrames.Rpy(new Vector3(0, Mathf.PI / 2, 0))), Quaternion.AngleAxis(90, Vector3.right)), Is.LessThan(1e-3f));
        }

        [Test]
        public void NestedRotatedChainComposes()
        {
            var f = Parse(Urdf);
            Assert.That(f.root, Is.EqualTo("base_link"));
            f.RootPose("lidar_link", Quaternion.identity, out var p, out var q);
            // ROS: (1,0,0) + yaw90 * (1,0,0.5) = (1,1,0.5) -> Unity (-1,0.5,1)
            Assert.That((p - new Vector3(-1, 0.5f, 1)).magnitude, Is.LessThan(1e-5f), p.ToString("F4"));
            Assert.That(Quaternion.Angle(q, Quaternion.Euler(0, -90, 0)), Is.LessThan(1e-3f));
            // The body yaw rotates every frame about the root alike.
            var y = UrdfFrames.BodyYaw(90);
            f.RootPose("lidar_link", y, out var py, out _);
            Assert.That((py - y * p).magnitude, Is.LessThan(1e-5f));
        }

        [Test]
        public void ApplyMovesReparentsAndCreatesFrames()
        {
            var robot = Node("robot", null);
            var b = Node("base_link", robot);
            var lidar = Node("lidar_link", Node("mast", b));
            var gps = Node("gps_link", b);
            gps.localPosition = new Vector3(5, 5, 5);
            var f = Parse(Urdf);
            var r = UrdfPlacement.Apply(f, robot, 90);
            Assert.That(r.errors, Is.Empty, string.Join("\n", r.errors));
            Assert.That(gps.parent.name, Is.EqualTo("box_link"));
            Assert.That(gps.parent.parent, Is.EqualTo(b));
            foreach (var t in new[] { lidar, gps })
            {
                f.RootPose(t.name, UrdfFrames.BodyYaw(90), out var p, out var q);
                Assert.That((b.InverseTransformPoint(t.position) - p).magnitude, Is.LessThan(1e-5f), t.name);
                Assert.That(Quaternion.Angle(Quaternion.Inverse(b.rotation) * t.rotation, q), Is.LessThan(1e-3f), t.name);
            }
            Assert.That(r.worstChangeM, Is.GreaterThan(5f));
            Assert.That(r.moved, Has.Some.StartsWith("link=box_link parent=base_link created"));
        }

        [Test]
        public void MissingSensorLinkOrOtherRootRefuses()
        {
            var robot = Node("robot", null);
            Node("mast", Node("base_link", robot));   // no lidar_link, no gps_link
            var r = UrdfPlacement.Apply(Parse(Urdf), robot, 90);
            Assert.That(r.errors, Has.Some.Contains("'lidar_link' is in the URDF but not in the robot"));
            Assert.That(r.errors, Has.Some.Contains("'gps_link' is in the URDF but not in the robot"));

            var other = Node("other", null);
            Node("hull", other);
            Assert.That(UrdfPlacement.Apply(Parse(Urdf), other, 90).errors, Has.Some.Contains("root link is 'base_link'"));
        }
    }
}
