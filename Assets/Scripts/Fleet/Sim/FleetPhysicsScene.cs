using System;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Crane.Fleet.Sim {
    /// <summary>
    /// Each run steps its own PhysX scene, so a run is the same whatever ran before it in the process (phase 1 found that the shared
    /// default scene keeps history: a run after the CAD check differed by 1e-6 m). Player: a scene created with
    /// LocalPhysicsMode.Physics3D. Editor (edit mode, where SceneManager.CreateScene is not allowed): a preview scene from
    /// <see cref="EditorFactory"/>, which the editor assembly installs. Roots are moved in with MoveGameObjectToScene.
    /// </summary>
    public sealed class FleetPhysicsScene : IDisposable {
        public static Func<string, Scene> EditorFactory;
        public static Action<Scene> EditorClose;

        public readonly Scene Scene;
        public readonly PhysicsScene Physics;
        public readonly string Describe;
        readonly bool local;

        FleetPhysicsScene(Scene s, PhysicsScene p, bool local, string describe) { Scene = s; Physics = p; this.local = local; Describe = describe; }

        public static FleetPhysicsScene Create(string name) {
            if (Application.isPlaying) {
                var s = SceneManager.CreateScene(name, new CreateSceneParameters(LocalPhysicsMode.Physics3D));
                return new FleetPhysicsScene(s, s.GetPhysicsScene(), true, "local (CreateScene, LocalPhysicsMode.Physics3D)");
            }
            if (EditorFactory != null) {
                var s = EditorFactory(name);
                var ps = s.GetPhysicsScene();
                bool own = ps != UnityEngine.Physics.defaultPhysicsScene;
                return new FleetPhysicsScene(s, ps, true, own ? "local (editor preview scene)" : "editor preview scene sharing the default physics scene");
            }
            return new FleetPhysicsScene(default, UnityEngine.Physics.defaultPhysicsScene, false, "default");
        }

        public void Adopt(GameObject root) {
            if (local && root != null && root.transform.parent == null) SceneManager.MoveGameObjectToScene(root, Scene);
        }

        public void Simulate(float dt) => Physics.Simulate(dt);

        public bool Raycast(Vector3 origin, Vector3 dir, out RaycastHit hit, float dist) =>
            Physics.Raycast(origin, dir, out hit, dist, ~0, QueryTriggerInteraction.Ignore);

        public void Dispose() {
            if (!local || !Scene.IsValid()) return;
            if (Application.isPlaying) SceneManager.UnloadSceneAsync(Scene);
            else EditorClose?.Invoke(Scene);
        }
    }
}
