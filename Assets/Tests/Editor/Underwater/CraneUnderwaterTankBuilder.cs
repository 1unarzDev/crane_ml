using System.IO;
using Sim.Underwater;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Sim.Tests.Editor.Underwater {
    /// <summary>
    /// Regenerates the BlueROV2 proxy prefab and the "UW Tank" scene (no HDRP water, no camera/light, primitives only).
    /// Run: Unity -batchmode -nographics -projectPath . -executeMethod Sim.Tests.Editor.Underwater.CraneUnderwaterTankBuilder.Build -quit
    /// </summary>
    public static class CraneUnderwaterTankBuilder {
        public const string PrefabPath = "Assets/Robot Descriptions/bluerov2_heavy/BlueROV2Proxy.prefab";
        public const string ScenePath = "Assets/Scenes/Underwater/UW Tank.unity";

        [MenuItem("CRANE/Underwater/Build UW Tank")]
        public static void Build() {
            Directory.CreateDirectory(Path.GetDirectoryName(PrefabPath));
            Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));

            var root = new GameObject("BlueROV2Proxy");
            var rb = root.AddComponent<Rigidbody>();
            rb.useGravity = false; rb.linearDamping = 0f; rb.angularDamping = 0f;     // the adapter is the only hydrodynamic force
            rb.interpolation = RigidbodyInterpolation.None;
            rb.collisionDetectionMode = CollisionDetectionMode.Discrete;
            // Primitive colliders from published envelope (von Benzon Table 5: 0.46 x 0.38 x 0.58 m L x H x W). Mass/inertia come from the profile.
            var hull = root.AddComponent<BoxCollider>(); hull.size = new Vector3(0.58f, 0.20f, 0.46f); hull.center = new Vector3(0f, -0.09f, 0f);
            AddCapsule(root, "TubeLeft", new Vector3(-0.15f, 0.10f, 0f));
            AddCapsule(root, "TubeRight", new Vector3(0.15f, 0.10f, 0f));
            root.AddComponent<CraneUnderwaterBody>();
            PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            Object.DestroyImmediate(root);

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath));
            inst.transform.position = new Vector3(0f, -2f, 0f);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            Debug.Log("[CraneUnderwaterTankBuilder] wrote " + PrefabPath + " and " + ScenePath);
        }

        static void AddCapsule(GameObject root, string name, Vector3 localPos) {
            var child = new GameObject(name);
            child.transform.SetParent(root.transform, false);
            child.transform.localPosition = localPos;
            var c = child.AddComponent<CapsuleCollider>();
            c.direction = 2; c.radius = 0.06f; c.height = 0.44f;     // along the forward (Z) axis
        }
    }
}
