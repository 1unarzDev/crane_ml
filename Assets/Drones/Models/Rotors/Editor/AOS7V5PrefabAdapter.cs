using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Crane.Drones.Models
{
    /// <summary>
    /// Builds Assets/Drones/Models/AOS7V5Quad.prefab from the URDF-synced Assets/Robot Descriptions/aos7v5/aos7v5.prefab:
    /// strips ArticulationBody/UrdfJoint components (the aerial plant drives the pose; PhysX must not integrate the airframe),
    /// adds a <see cref="RotorSpinner"/> on rotor_0..3 (PX4 order, spin FR CCW, RL CCW, FL CW, RR CW) and CoM / sensor markers.
    /// Batch: unity-drones LOG -executeMethod Crane.Drones.Models.AOS7V5PrefabAdapter.Run   (exit 0 ok, 1 error)
    /// </summary>
    public static class AOS7V5PrefabAdapter
    {
        public const string SourcePrefab = "Assets/Robot Descriptions/aos7v5/aos7v5.prefab";
        public const string OutPrefab = "Assets/Drones/Models/AOS7V5Quad.prefab";
        static readonly string[] RotorLinks = { "rotor_0", "rotor_1", "rotor_2", "rotor_3" };
        static readonly int[] Spin = { +1, +1, -1, -1 };

        public static void Run()
        {
            try { Build(); Debug.Log("AOS7V5_ADAPTER ok " + OutPrefab); EditorApplication.Exit(0); }
            catch (Exception e) { Debug.LogError("AOS7V5_ADAPTER error " + e); EditorApplication.Exit(1); }
        }

        public static GameObject Build()
        {
            var src = AssetDatabase.LoadAssetAtPath<GameObject>(SourcePrefab);
            if (src == null) throw new InvalidOperationException(SourcePrefab + " missing: run Crane.Urdf.UrdfSync.Batch on aos7v5.urdf first");
            var root = (GameObject)PrefabUtility.InstantiatePrefab(src);
            try
            {
                PrefabUtility.UnpackPrefabInstance(root, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
                foreach (var ab in root.GetComponentsInChildren<ArticulationBody>(true)) UnityEngine.Object.DestroyImmediate(ab);
                foreach (var c in root.GetComponentsInChildren<Component>(true))
                    if (c != null && c.GetType().Name.StartsWith("Urdf") && c.GetType().Name.Contains("Joint")) UnityEngine.Object.DestroyImmediate(c);
                var sp = root.AddComponent<RotorSpinner>();
                sp.rotors = new RotorSpinner.Rotor[4];
                for (int i = 0; i < 4; i++)
                {
                    var hub = FindDeep(root.transform, RotorLinks[i]);
                    if (hub == null) throw new InvalidOperationException("link " + RotorLinks[i] + " not found");
                    float tilt = Vector3.Angle(hub.up, root.transform.up);
                    if (tilt > 0.1f) throw new InvalidOperationException($"{RotorLinks[i]} shaft is {tilt:F2} deg off body up; add a pivot child instead");
                    sp.rotors[i] = new RotorSpinner.Rotor { link = RotorLinks[i], motorIndex = i + 1, spinSign = Spin[i], hub = hub, axisLocal = Vector3.up };
                }
                sp.Capture();
                var saved = PrefabUtility.SaveAsPrefabAsset(root, OutPrefab);
                return saved;
            }
            finally { UnityEngine.Object.DestroyImmediate(root); }
        }

        static Transform FindDeep(Transform t, string name)
        {
            if (t.name == name) return t;
            for (int i = 0; i < t.childCount; i++) { var r = FindDeep(t.GetChild(i), name); if (r != null) return r; }
            return null;
        }
    }
}
