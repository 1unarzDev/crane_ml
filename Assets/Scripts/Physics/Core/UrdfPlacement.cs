using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Core {
    /// <summary>
    /// <c>--crane-urdf PATH</c> (expanded URDF): when a scene with a robot loads (after
    /// <see cref="RobotSelection"/>), every fixed joint of the URDF is applied to the robot's link
    /// GameObjects (parent, local pose, joint anchors), so sensors sit where the robot description
    /// puts them without a prefab sync or a player rebuild. Geometry-free intermediate frames the
    /// prefab lacks (box_link) are created. A URDF link that is missing otherwise, or a different
    /// root link, refuses the run. The prefab's own frames come from the same URDF via UrdfSync.
    /// </summary>
    public static class UrdfPlacement {
        public const string Flag = "--crane-urdf";
        public const float HeldPosM = 0.001f, HeldAngDeg = 0.1f;

        public sealed class Result {
            public readonly List<string> errors = new List<string>(), warnings = new List<string>(), moved = new List<string>();
            public readonly Dictionary<Transform, (Vector3 pos, Quaternion rot)> expected = new Dictionary<Transform, (Vector3, Quaternion)>();
            public int frames;
            public float worstChangeM, worstChangeDeg;
            public bool Ok => errors.Count == 0;
        }

        private static UrdfFrames frames;
        private static string path;
        private static readonly HashSet<RobotDescription> placed = new HashSet<RobotDescription>();

        // BeforeSceneLoad registers after RobotSelection (SubsystemRegistration), so the swap runs first.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            frames = null;
            placed.Clear();
            path = ProfileBootstrap.Read(Environment.GetCommandLineArgs(), Flag);
            if (string.IsNullOrEmpty(path)) return;
            var errors = new List<string>();
            try { frames = UrdfFrames.Parse(XDocument.Load(path), path, errors); }
            catch (Exception e) { errors.Add($"'{path}' is not readable XML: {e.Message}"); }
            if (frames == null || errors.Count > 0) { Refuse(errors); return; }
            SceneManager.sceneLoaded += (scene, _) => {
                foreach (var d in UnityEngine.Object.FindObjectsByType<RobotDescription>(FindObjectsInactive.Exclude)) {
                    if (!placed.Add(d)) continue;
                    var r = Apply(frames, d.transform, d.rootYawDeg);
                    foreach (var m in r.moved) Debug.Log("CRANE_URDF_FRAME " + m);
                    foreach (var w in r.warnings) Debug.LogWarning("CRANE_URDF_WARN " + w);
                    if (!r.Ok) { Refuse(r.errors); return; }
                    Debug.Log($"CRANE_URDF_PLACED robot={d.name} urdf={path} frames={r.frames} moved={r.moved.Count} " +
                              $"worst_change_mm={r.worstChangeM * 1000:F1} worst_change_deg={r.worstChangeDeg:F2}");
                    d.gameObject.AddComponent<HeldCheck>().result = r;
                }
            };
        }

        private static void Refuse(List<string> errors) {
            foreach (var e in errors) Debug.LogError("CRANE_URDF_FAILED " + e);
            if (!Application.isEditor) Application.Quit(3);   // a robot that does not match its description must not run
        }

        /// <summary>Applies every fixed joint of <paramref name="f"/> to the links under
        /// <paramref name="robot"/> (link GameObjects are named after URDF links).</summary>
        public static Result Apply(UrdfFrames f, Transform robot, float rootYawDeg) {
            var r = new Result();
            var have = new Dictionary<string, Transform>();
            foreach (var t in robot.GetComponentsInChildren<Transform>(true).Where(t => f.links.Contains(t.name))) {
                if (have.ContainsKey(t.name)) r.errors.Add($"the robot has two GameObjects named '{t.name}'.");
                else have[t.name] = t;
            }
            ArticulationBody rootBody = robot.GetComponentsInChildren<ArticulationBody>(true).FirstOrDefault(a => a.isRoot);
            string robotRoot = rootBody != null ? rootBody.name : null;
            if (!have.TryGetValue(f.root, out var rootLink) || (robotRoot != null && robotRoot != f.root)) {
                r.errors.Add($"the URDF's root link is '{f.root}', but the robot's is '{robotRoot ?? "(none)"}'.");
                return r;
            }
            foreach (var t in robot.GetComponentsInChildren<ArticulationBody>(true).Select(a => a.transform).Where(t => !f.links.Contains(t.name)))
                r.warnings.Add($"link '{t.name}' is in the robot but not in the URDF; left where it is.");
            var before = have.ToDictionary(kv => kv.Value, kv => RootRelative(rootLink, kv.Value));
            var y = UrdfFrames.BodyYaw(rootYawDeg);

            foreach (var n in f.TopDown().Skip(1)) {
                var j = f.jointOf[n];
                if (!have.TryGetValue(n, out var t)) {
                    if (!f.geometryFree.Contains(n) || !f.HasChildren(n)) {
                        r.errors.Add($"link '{n}' is in the URDF but not in the robot (only geometry-free intermediate frames are created; sync the prefab from this URDF).");
                        continue;
                    }
                    t = new GameObject(n).transform;
                    t.SetParent(have[j.parent], false);
                    have[n] = t;
                    r.moved.Add($"link={n} parent={j.parent} created");
                }
                // Moving joints (in the URDF or in the robot: the propellers spin) keep their pose and anchors.
                var body = t.GetComponent<ArticulationBody>();   // Unity's == null, not a pattern: a missing component is a fake null
                if (j.type != "fixed" || (body != null && !body.isRoot && body.jointType != ArticulationJointType.FixedJoint)) continue;
                bool reparented = t.parent != have[j.parent];
                if (reparented) t.SetParent(have[j.parent], false);
                f.LocalPose(n, y, out var p, out var q);
                t.localPosition = p; t.localRotation = q;
                r.frames++;
                if (reparented) r.moved.Add($"link={n} parent={j.parent} reparented");
            }
            if (!r.Ok) return r;

            foreach (var kv in have) {
                Transform t = kv.Value;
                var now = RootRelative(rootLink, t);
                r.expected[t] = now;
                if (!before.TryGetValue(t, out var was)) continue;
                float dp = (now.pos - was.pos).magnitude, da = Quaternion.Angle(now.rot, was.rot);
                r.worstChangeM = Mathf.Max(r.worstChangeM, dp); r.worstChangeDeg = Mathf.Max(r.worstChangeDeg, da);
                if (dp > 1e-5f || da > 1e-3f) r.moved.Add($"link={kv.Key} moved_mm={dp * 1000:F1} turned_deg={da:F2}");
            }
            // The solver holds each link at parent anchor ∘ child anchor⁻¹; put the parent anchor where the link now is.
            foreach (var ab in have.Values.Select(t => t.GetComponent<ArticulationBody>()).Where(a => a != null && !a.isRoot && a.jointType == ArticulationJointType.FixedJoint)) {
                var parentBody = ab.transform.parent.GetComponentInParent<ArticulationBody>();
                if (parentBody == null) continue;
                Vector3 relPos = parentBody.transform.InverseTransformPoint(ab.transform.position);   // links are unscaled
                Quaternion relRot = Quaternion.Inverse(parentBody.transform.rotation) * ab.transform.rotation;
                Vector3 anchorPos = relPos + relRot * ab.anchorPosition;
                Quaternion anchorRot = relRot * ab.anchorRotation;
                if (!ab.matchAnchors && (ab.parentAnchorPosition - anchorPos).sqrMagnitude < 1e-12f && Quaternion.Angle(ab.parentAnchorRotation, anchorRot) < 1e-4f) continue;
                ab.matchAnchors = false;
                ab.parentAnchorPosition = anchorPos;
                ab.parentAnchorRotation = anchorRot;
            }
            return r;
        }

        private static (Vector3 pos, Quaternion rot) RootRelative(Transform root, Transform t) =>
            (root.InverseTransformPoint(t.position), Quaternion.Inverse(root.rotation) * t.rotation);

        /// <summary>After the first physics steps, checks the solver kept every link where the
        /// placement put it (CRANE_URDF_HELD), then removes itself.</summary>
        private sealed class HeldCheck : MonoBehaviour {
            public Result result;

            private IEnumerator Start() {
                for (int i = 0; i < 3; i++) yield return new WaitForFixedUpdate();
                Transform root = result.expected.Keys.FirstOrDefault(t => t != null && t.GetComponent<ArticulationBody>() != null && t.GetComponent<ArticulationBody>().isRoot);
                if (root == null) { Destroy(this); yield break; }
                float worstM = 0, worstDeg = 0;
                string worst = "";
                foreach (var kv in result.expected) {
                    if (kv.Key == null) continue;
                    var ab = kv.Key.GetComponent<ArticulationBody>();
                    if (ab != null && ab.jointType != ArticulationJointType.FixedJoint && !ab.isRoot) continue;   // a spinning propeller moves
                    var now = RootRelative(root, kv.Key);
                    float dp = (now.pos - kv.Value.pos).magnitude, da = Quaternion.Angle(now.rot, kv.Value.rot);
                    if (dp > worstM || da > worstDeg) worst = kv.Key.name;
                    worstM = Mathf.Max(worstM, dp); worstDeg = Mathf.Max(worstDeg, da);
                }
                bool ok = worstM <= HeldPosM && worstDeg <= HeldAngDeg;
                string line = $"CRANE_URDF_HELD ok={ok.ToString().ToLowerInvariant()} worst_mm={worstM * 1000:F2} worst_deg={worstDeg:F3} link={worst}";
                if (ok) Debug.Log(line); else Debug.LogError(line);
                Destroy(this);
            }
        }
    }
}
