using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Sim.Utils.ROS;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace Sim.Utils.Performance {
    public enum CraneEpisodeResetPhase {
        BeforePhysics,
        AfterPhysics
    }

    public readonly struct CraneEpisodeResetContext {
        public readonly long EpisodeId;
        public readonly int Seed;

        internal CraneEpisodeResetContext(long episodeId, int seed) {
            EpisodeId = episodeId;
            Seed = seed;
        }
    }

    /// <summary>
    /// Explicit reset seam for state that Unity cannot infer from a Transform or physics body.
    /// Implementations must be idempotent and must not start asynchronous work while resetting.
    /// </summary>
    public interface ICraneEpisodeResettable {
        int ResetPriority { get; }
        void CaptureEpisodeInitialState();
        void ResetEpisode(in CraneEpisodeResetContext context, CraneEpisodeResetPhase phase);
    }

    public readonly struct CraneEpisodeResetResult {
        public readonly long EpisodeId;
        public readonly int RigidbodyCount;
        public readonly int ArticulationCount;
        public readonly int ResettableCount;
        public readonly double WallMilliseconds;

        internal CraneEpisodeResetResult(long episodeId, int rigidbodyCount,
            int articulationCount, int resettableCount, double wallMilliseconds) {
            EpisodeId = episodeId;
            RigidbodyCount = rigidbodyCount;
            ArticulationCount = articulationCount;
            ResettableCount = resettableCount;
            WallMilliseconds = wallMilliseconds;
        }
    }

    /// <summary>
    /// Captures and restores a single loaded scene without rebuilding it. Scene reload remains the
    /// reference reset until every stateful component in a production scene implements the reset
    /// seam and passes A-B-A equivalence.
    /// </summary>
    public sealed class CraneEpisodeResetCoordinator {
        private readonly struct RigidbodyState {
            public readonly Rigidbody Body;
            public readonly Vector3 Position;
            public readonly Quaternion Rotation;
            public readonly Vector3 LinearVelocity;
            public readonly Vector3 AngularVelocity;
            public readonly bool Sleeping;

            public RigidbodyState(Rigidbody body) {
                Body = body;
                Position = body.position;
                Rotation = body.rotation;
                LinearVelocity = body.linearVelocity;
                AngularVelocity = body.angularVelocity;
                Sleeping = body.IsSleeping();
            }

            public void Restore() {
                if (Body == null) return;
                Body.position = Position;
                Body.rotation = Rotation;
                Body.linearVelocity = LinearVelocity;
                Body.angularVelocity = AngularVelocity;
                if (Sleeping) Body.Sleep();
                else Body.WakeUp();
            }
        }

        private sealed class ArticulationState {
            private readonly ArticulationBody body;
            private readonly Vector3 position;
            private readonly Quaternion rotation;
            private readonly Vector3 linearVelocity;
            private readonly Vector3 angularVelocity;
            private readonly float[] jointPosition;
            private readonly float[] jointVelocity;
            private readonly float[] jointForce;
            private readonly bool sleeping;

            public ArticulationState(ArticulationBody body) {
                this.body = body;
                position = body.transform.position;
                rotation = body.transform.rotation;
                linearVelocity = body.linearVelocity;
                angularVelocity = body.angularVelocity;
                jointPosition = Copy(body.jointPosition);
                jointVelocity = Copy(body.jointVelocity);
                jointForce = Copy(body.jointForce);
                sleeping = body.IsSleeping();
            }

            public void Restore() {
                if (body == null) return;
                if (body.isRoot) body.TeleportRoot(position, rotation);
                body.jointPosition = Restore(body.jointPosition, jointPosition);
                body.jointVelocity = Restore(body.jointVelocity, jointVelocity);
                body.jointForce = Restore(body.jointForce, jointForce);
                body.linearVelocity = linearVelocity;
                body.angularVelocity = angularVelocity;
                if (sleeping) body.Sleep();
                else body.WakeUp();
            }

            private static float[] Copy(ArticulationReducedSpace values) {
                var copy = new float[values.dofCount];
                for (int i = 0; i < copy.Length; i++) copy[i] = values[i];
                return copy;
            }

            private static ArticulationReducedSpace Restore(ArticulationReducedSpace destination,
                float[] source) {
                ArticulationReducedSpace values = destination;
                int count = Math.Min(values.dofCount, source.Length);
                for (int i = 0; i < count; i++) values[i] = source[i];
                return values;
            }
        }

        private readonly List<RigidbodyState> rigidbodies = new();
        private readonly List<ArticulationState> articulations = new();
        private readonly List<ICraneEpisodeResettable> resettables = new();
        private readonly Scene scene;
        private readonly int seed;
        private bool captured;

        public CraneEpisodeResetCoordinator(Scene scene, int seed) {
            if (!scene.IsValid() || !scene.isLoaded)
                throw new ArgumentException("Reset scene must be valid and loaded.", nameof(scene));
            this.scene = scene;
            this.seed = seed;
        }

        public void Capture() {
            rigidbodies.Clear();
            articulations.Clear();
            resettables.Clear();

            foreach (Rigidbody body in UnityEngine.Object.FindObjectsByType<Rigidbody>(
                         FindObjectsInactive.Include, FindObjectsSortMode.InstanceID))
                if (body.gameObject.scene == scene) rigidbodies.Add(new RigidbodyState(body));

            foreach (ArticulationBody body in UnityEngine.Object.FindObjectsByType<ArticulationBody>(
                         FindObjectsInactive.Include, FindObjectsSortMode.InstanceID))
                if (body.gameObject.scene == scene) articulations.Add(new ArticulationState(body));

            resettables.AddRange(UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                    FindObjectsInactive.Include, FindObjectsSortMode.InstanceID)
                .Where(component => component.gameObject.scene == scene)
                .OfType<ICraneEpisodeResettable>()
                .OrderBy(component => component.ResetPriority));
            foreach (ICraneEpisodeResettable resettable in resettables)
                resettable.CaptureEpisodeInitialState();
            captured = true;
        }

        public CraneEpisodeResetResult Reset() {
            if (!captured) throw new InvalidOperationException("Capture must run before Reset.");
            using var marker = CraneProfiler.EpisodeReset.Auto();
            var stopwatch = Stopwatch.StartNew();

            // Drain callbacks while the old generation is still current, then advance the
            // generation so a late transport callback cannot affect the new episode.
            AsyncGPUReadback.WaitAllRequests();
            long episodeId = CraneRuntimeMetrics.BeginEpisode();
            CraneActionGate.BeginEpisode();
            UnityEngine.Random.InitState(seed);
            var context = new CraneEpisodeResetContext(episodeId, seed);

            foreach (ICraneEpisodeResettable resettable in resettables)
                resettable.ResetEpisode(context, CraneEpisodeResetPhase.BeforePhysics);
            foreach (RigidbodyState state in rigidbodies) state.Restore();
            foreach (ArticulationState state in articulations) state.Restore();
            Physics.SyncTransforms();
            foreach (ICraneEpisodeResettable resettable in resettables)
                resettable.ResetEpisode(context, CraneEpisodeResetPhase.AfterPhysics);

            stopwatch.Stop();
            return new CraneEpisodeResetResult(episodeId, rigidbodies.Count,
                articulations.Count, resettables.Count, stopwatch.Elapsed.TotalMilliseconds);
        }
    }
}
