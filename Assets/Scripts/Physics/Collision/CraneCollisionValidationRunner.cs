using System;
using System.Collections;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Contacts {
    [Serializable]
    internal sealed class CollisionValidationResult {
        public string schema = "crane-collision-validation-v1";
        public string unityVersion;
        public float fixedDeltaTime;
        public int hullDockContacts;
        public float hullFinalX;
        public int dynamicObstacleContacts;
        public float dynamicObstacleDisplacement;
        public int thinBarrierContacts;
        public float highSpeedBodyFinalX;
        public int triggerEnters;
        public float triggerBodyFinalX;
        public bool sensorQueryRayHit;
        public float sensorQueryBodyFinalX;
        public bool requiredMatrixValid;
        public bool excludedMatrixValid;
        public bool hullDockValid;
        public bool dynamicObstacleValid;
        public bool thinBarrierValid;
        public bool triggerValid;
        public bool sensorQueryValid;
        public bool valid;
    }

    internal sealed class CollisionProbe : MonoBehaviour {
        public int collisionEnters;
        public int triggerEnters;
        private void OnCollisionEnter(UnityEngine.Collision collision) => collisionEnters++;
        private void OnTriggerEnter(Collider other) => triggerEnters++;
    }

    internal sealed class CraneCollisionValidationRunner : MonoBehaviour {
        private string outputPath;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains("--crane-collision-validation")) return;
            var host = new GameObject("CRANE Collision Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneCollisionValidationRunner>();
        }

        private IEnumerator Start() {
            string[] args = Environment.GetCommandLineArgs();
            outputPath = ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-collision-validation.json");
            if (SceneManager.GetActiveScene().name != "Collision Validation") {
                AsyncOperation load = SceneManager.LoadSceneAsync("Collision Validation", LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            yield return RunValidation();
        }

        private IEnumerator RunValidation() {
            CreateGround();

            GameObject hull = CreateBody("Hull", new Vector3(-5f, 0.5f, 0f),
                Vector3.one, CraneCollisionLayers.Vehicle, new Vector3(6f, 0f, 0f), false, true);
            CollisionProbe hullProbe = hull.AddComponent<CollisionProbe>();
            CreateStatic("Dock", new Vector3(0f, 1f, 0f), new Vector3(1f, 2f, 5f),
                CraneCollisionLayers.Environment, false);

            GameObject vehicle = CreateBody("Vehicle Dynamic Test", new Vector3(-5f, 1f, 6f),
                Vector3.one, CraneCollisionLayers.Vehicle, new Vector3(6f, 0f, 0f), false, true);
            GameObject obstacle = CreateBody("Dynamic Obstacle", new Vector3(0f, 1f, 6f),
                Vector3.one, CraneCollisionLayers.DynamicObstacle, Vector3.zero, false, false);
            CollisionProbe obstacleProbe = obstacle.AddComponent<CollisionProbe>();
            Vector3 obstacleStart = obstacle.transform.position;

            GameObject fastBody = CreateBody("High Speed Body", new Vector3(-5f, 1f, 12f),
                Vector3.one * 0.5f, CraneCollisionLayers.Vehicle, new Vector3(40f, 0f, 0f), false, true);
            CollisionProbe fastProbe = fastBody.AddComponent<CollisionProbe>();
            CreateStatic("Thin Barrier", new Vector3(0f, 1f, 12f), new Vector3(0.05f, 3f, 4f),
                CraneCollisionLayers.Environment, false);

            GameObject trigger = CreateStatic("Simulation Trigger", new Vector3(0f, 1f, 18f),
                new Vector3(1f, 3f, 4f), CraneCollisionLayers.SimulationTrigger, true);
            CollisionProbe triggerProbe = trigger.AddComponent<CollisionProbe>();
            GameObject triggerBody = CreateBody("Trigger Body", new Vector3(-5f, 1f, 18f),
                Vector3.one, CraneCollisionLayers.Vehicle, new Vector3(6f, 0f, 0f), false, true);

            CreateStatic("Sensor Query Geometry", new Vector3(0f, 1f, 24f),
                new Vector3(1f, 3f, 4f), CraneCollisionLayers.SensorQuery, false);
            GameObject queryBody = CreateBody("Sensor Query Pass Body", new Vector3(-5f, 1f, 24f),
                Vector3.one, CraneCollisionLayers.Vehicle, new Vector3(6f, 0f, 0f), false, true);
            UnityEngine.Physics.SyncTransforms();
            bool queryHit = UnityEngine.Physics.Raycast(new Vector3(-5f, 1f, 24f), Vector3.right,
                10f, 1 << CraneCollisionLayers.SensorQuery, QueryTriggerInteraction.Ignore);

            yield return FixedSeconds(2f);

            var result = new CollisionValidationResult {
                unityVersion = Application.unityVersion,
                fixedDeltaTime = Time.fixedDeltaTime,
                hullDockContacts = hullProbe.collisionEnters,
                hullFinalX = hull.transform.position.x,
                dynamicObstacleContacts = obstacleProbe.collisionEnters,
                dynamicObstacleDisplacement = obstacle.transform.position.x - obstacleStart.x,
                thinBarrierContacts = fastProbe.collisionEnters,
                highSpeedBodyFinalX = fastBody.transform.position.x,
                triggerEnters = triggerProbe.triggerEnters,
                triggerBodyFinalX = triggerBody.transform.position.x,
                sensorQueryRayHit = queryHit,
                sensorQueryBodyFinalX = queryBody.transform.position.x
            };
            result.requiredMatrixValid = RequiredPairEnabled(CraneCollisionLayers.Vehicle,
                    CraneCollisionLayers.Environment) &&
                RequiredPairEnabled(CraneCollisionLayers.Vehicle, CraneCollisionLayers.DynamicObstacle) &&
                RequiredPairEnabled(CraneCollisionLayers.DynamicObstacle, CraneCollisionLayers.Environment) &&
                RequiredPairEnabled(CraneCollisionLayers.DynamicObstacle, CraneCollisionLayers.DynamicObstacle) &&
                RequiredPairEnabled(CraneCollisionLayers.Vehicle, CraneCollisionLayers.Vehicle) &&
                RequiredPairEnabled(CraneCollisionLayers.Vehicle, CraneCollisionLayers.SimulationTrigger);
            result.excludedMatrixValid = PairIgnored(CraneCollisionLayers.SensorQuery,
                    CraneCollisionLayers.Vehicle) &&
                PairIgnored(CraneCollisionLayers.SensorQuery, CraneCollisionLayers.Environment) &&
                PairIgnored(CraneCollisionLayers.SensorQuery, CraneCollisionLayers.DynamicObstacle) &&
                PairIgnored(CraneCollisionLayers.SensorQuery, CraneCollisionLayers.SensorQuery) &&
                PairIgnored(CraneCollisionLayers.SensorQuery, CraneCollisionLayers.SimulationTrigger) &&
                PairIgnored(CraneCollisionLayers.Environment, CraneCollisionLayers.Environment) &&
                PairIgnored(CraneCollisionLayers.Environment, 5) &&
                PairIgnored(CraneCollisionLayers.Vehicle, 5) &&
                PairIgnored(CraneCollisionLayers.DynamicObstacle, 5) &&
                PairIgnored(CraneCollisionLayers.SimulationTrigger, CraneCollisionLayers.Environment) &&
                PairIgnored(CraneCollisionLayers.SimulationTrigger,
                    CraneCollisionLayers.SimulationTrigger);
            result.hullDockValid = result.hullDockContacts > 0 && result.hullFinalX < 0f;
            result.dynamicObstacleValid = result.dynamicObstacleContacts > 0 &&
                result.dynamicObstacleDisplacement > 0.5f;
            result.thinBarrierValid = result.thinBarrierContacts > 0 && result.highSpeedBodyFinalX < 0.5f;
            result.triggerValid = result.triggerEnters > 0 && result.triggerBodyFinalX > 1f;
            result.sensorQueryValid = result.sensorQueryRayHit && result.sensorQueryBodyFinalX > 1f;
            result.valid = result.requiredMatrixValid && result.excludedMatrixValid &&
                result.hullDockValid && result.dynamicObstacleValid && result.thinBarrierValid &&
                result.triggerValid && result.sensorQueryValid;

            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_COLLISION_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            Application.Quit(result.valid ? 0 : 2);
        }

        private static void CreateGround() {
            CreateStatic("Collision Ground", new Vector3(0f, -0.5f, 12f),
                new Vector3(40f, 1f, 32f), CraneCollisionLayers.Environment, false);
        }

        private static GameObject CreateStatic(string name, Vector3 position, Vector3 scale,
            int layer, bool trigger) {
            GameObject value = GameObject.CreatePrimitive(PrimitiveType.Cube);
            value.name = name;
            value.layer = layer;
            value.transform.position = position;
            value.transform.localScale = scale;
            value.GetComponent<BoxCollider>().isTrigger = trigger;
            value.GetComponent<Renderer>().enabled = false;
            return value;
        }

        private static GameObject CreateBody(string name, Vector3 position, Vector3 scale,
            int layer, Vector3 velocity, bool gravity, bool continuous) {
            GameObject value = GameObject.CreatePrimitive(PrimitiveType.Cube);
            value.name = name;
            value.layer = layer;
            value.transform.position = position;
            value.transform.localScale = scale;
            value.GetComponent<Renderer>().enabled = false;
            Rigidbody body = value.AddComponent<Rigidbody>();
            body.useGravity = gravity;
            body.linearVelocity = velocity;
            body.collisionDetectionMode = continuous ? CollisionDetectionMode.ContinuousDynamic :
                CollisionDetectionMode.Discrete;
            return value;
        }

        private static bool RequiredPairEnabled(int first, int second) =>
            !UnityEngine.Physics.GetIgnoreLayerCollision(first, second);
        private static bool PairIgnored(int first, int second) =>
            UnityEngine.Physics.GetIgnoreLayerCollision(first, second);

        private static IEnumerator FixedSeconds(float seconds) {
            int steps = Mathf.CeilToInt(seconds / Time.fixedDeltaTime);
            for (int i = 0; i < steps; i++) yield return new WaitForFixedUpdate();
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }
    }
}
