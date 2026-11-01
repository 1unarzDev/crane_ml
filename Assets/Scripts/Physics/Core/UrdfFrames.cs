using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Xml.Linq;
using UnityEngine;

namespace Sim.Physics.Core {
    /// <summary>
    /// The link tree of an expanded URDF and its frames in Unity: one implementation for the
    /// editor sync (UrdfSync) and the runtime placement (<see cref="UrdfPlacement"/>).
    /// </summary>
    public sealed class UrdfFrames {
        public sealed class Joint {
            public string name, type, parent, child;
            public Vector3 xyz, axis = Vector3.right;
            public Quaternion rot = Quaternion.identity;
        }

        public string name, root;
        public readonly List<string> links = new List<string>();
        public readonly Dictionary<string, Joint> jointOf = new Dictionary<string, Joint>();   // child link -> joint
        public readonly HashSet<string> geometryFree = new HashSet<string>();                  // no visual or collision

        public static Vector3 V(string s, Vector3 dflt) {
            if (string.IsNullOrWhiteSpace(s)) return dflt;
            var p = s.Split((char[])null, StringSplitOptions.RemoveEmptyEntries)
                .Select(t => float.Parse(t, CultureInfo.InvariantCulture)).ToArray();
            return new Vector3(p[0], p[1], p[2]);
        }

        // ROS (x fwd, y left, z up) -> Unity (x right, y up, z fwd)
        public static Vector3 M(Vector3 p) => new Vector3(-p.y, p.z, p.x);
        public static Quaternion M(Quaternion q) => new Quaternion(q.y, -q.z, -q.x, q.w);
        public static Quaternion Rpy(Vector3 rpy) {
            Quaternion ax(Vector3 a, float r) => new Quaternion(a.x * Mathf.Sin(r / 2), a.y * Mathf.Sin(r / 2), a.z * Mathf.Sin(r / 2), Mathf.Cos(r / 2));
            return ax(Vector3.forward, rpy.z) * ax(Vector3.up, rpy.y) * ax(Vector3.right, rpy.x);   // ROS fixed-axis XYZ, in ROS axes
        }

        /// <summary>The body yaw Y every frame carries (RobotDescription.rootYawDeg): ROS yaw about z, in Unity.</summary>
        public static Quaternion BodyYaw(float rootYawDeg) => M(Quaternion.AngleAxis(rootYawDeg, Vector3.forward));

        /// <summary>Links and joints of an expanded URDF. Problems go to <paramref name="errors"/>
        /// as sentences; null when the tree is unusable.</summary>
        public static UrdfFrames Parse(XDocument doc, string source, List<string> errors) {
            var robot = doc.Root;
            if (robot == null || robot.Name.LocalName != "robot") { errors.Add($"'{source}' has no <robot> root element (is it still xacro? expand it with `xacro file.xacro > file.urdf`)."); return null; }
            if (robot.Descendants().Any(e => e.Name.NamespaceName.Contains("xacro"))) { errors.Add($"'{source}' still contains xacro elements; expand it first (`xacro file.xacro > file.urdf`)."); return null; }
            var f = new UrdfFrames { name = (string)robot.Attribute("name") };
            foreach (var l in robot.Elements("link")) {
                string n = (string)l.Attribute("name");
                if (f.links.Contains(n)) errors.Add($"link '{n}' is defined twice.");
                f.links.Add(n);
                if (!l.Elements("visual").Any() && !l.Elements("collision").Any()) f.geometryFree.Add(n);
            }
            foreach (var j in robot.Elements("joint")) {
                var o = j.Element("origin");
                var uj = new Joint {
                    name = (string)j.Attribute("name"), type = (string)j.Attribute("type"),
                    parent = (string)j.Element("parent")?.Attribute("link"),
                    child = (string)j.Element("child")?.Attribute("link"),
                    xyz = V((string)o?.Attribute("xyz"), Vector3.zero), rot = Rpy(V((string)o?.Attribute("rpy"), Vector3.zero)),
                    axis = V((string)j.Element("axis")?.Attribute("xyz"), Vector3.right),
                };
                if (!f.links.Contains(uj.child)) { errors.Add($"joint '{uj.name}' has child link '{uj.child}', which is not defined."); continue; }
                if (!f.links.Contains(uj.parent)) { errors.Add($"joint '{uj.name}' has parent link '{uj.parent}', which is not defined."); continue; }
                if (f.jointOf.ContainsKey(uj.child)) { errors.Add($"link '{uj.child}' is the child of two joints."); continue; }
                f.jointOf[uj.child] = uj;
            }
            var roots = f.links.Where(l => !f.jointOf.ContainsKey(l)).ToList();
            if (roots.Count != 1) { errors.Add($"the URDF needs exactly one root link; found {roots.Count}: {string.Join(", ", roots)}."); return null; }
            f.root = roots[0];
            return f;
        }

        /// <summary>Links parents before children, from the root.</summary>
        public List<string> TopDown() {
            var order = new List<string> { root };
            for (int i = 0; i < order.Count; i++) order.AddRange(links.Where(l => jointOf.TryGetValue(l, out var j) && j.parent == order[i]));
            return order;
        }

        public bool HasChildren(string link) => jointOf.Values.Any(j => j.parent == link);

        // Expected pose of a link relative to its parent link, in Unity, with the body yaw Y on every frame:
        // position Y·M(p), rotation Y·M(R)·Y⁻¹ (the frame carries the correction; geometry children get Y·M(R)).
        public void LocalPose(string link, Quaternion y, out Vector3 pos, out Quaternion rot) {
            var j = jointOf[link];
            pos = y * M(j.xyz);
            rot = y * M(j.rot) * Quaternion.Inverse(y);
        }

        /// <summary>Pose of a link relative to the root link.</summary>
        public void RootPose(string link, Quaternion y, out Vector3 pos, out Quaternion rot) {
            pos = Vector3.zero; rot = Quaternion.identity;
            var chain = new List<string>();
            for (string l = link; l != root; l = jointOf[l].parent) chain.Add(l);
            chain.Reverse();
            foreach (var l in chain) {
                LocalPose(l, y, out var lp, out var lr);
                pos += rot * lp; rot *= lr;
            }
        }
    }
}
