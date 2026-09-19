using System;
using System.Collections;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Aerial {
    [Serializable]
    internal sealed class AerialValidationResult {
        public string schema = "crane-aerial-validation-v1";
        public string scene;
        public string referenceEnvironmentId;
        public int referenceWallCount;
        public bool referenceGeometryValid;
        public float referenceRaycastDistance;
        public float referenceCollisionStopX;
        public bool referenceRaycastValid;
        public bool referenceCollisionValid;
        public string unityVersion;
        public float fixedDeltaTime;
        public float hoverMotorSpeed;
        public float hoverAltitudeDrift;
        public float hoverVerticalSpeed;
        public float expectedVerticalAcceleration;
        public float measuredVerticalAcceleration;
        public float verticalAccelerationRelativeError;
        public float rollAngularSpeed;
        public float pitchAngularSpeed;
        public float yawAngularSpeed;
        public float windDisplacement;
        public float landingHeight;
        public float landingSpeed;
        public float saturatedMotorSpeed;
        public bool hoverValid;
        public bool verticalValid;
        public bool attitudeValid;
        public bool windValid;
        public bool landingValid;
        public bool saturationValid;
        public bool valid;
    }

    internal sealed class CraneAerialValidationRunner : MonoBehaviour {
        private MultirotorDynamics multirotor;
        private Rigidbody body;
        private string outputPath;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains("--crane-aerial-validation")) return;
            var host = new GameObject("CRANE Aerial Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneAerialValidationRunner>();
        }

        private IEnumerator Start() {
            string[] args = Environment.GetCommandLineArgs();
            outputPath = ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-aerial-validation.json");
            string requestedScene = ReadArgument(args, "--crane-aerial-scene") ??
                "Aerial Vehicle Validation";
            if (SceneManager.GetActiveScene().name != requestedScene) {
                AsyncOperation load = SceneManager.LoadSceneAsync(requestedScene, LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            multirotor = FindAnyObjectByType<MultirotorDynamics>();
            if (multirotor == null) throw new MissingReferenceException("Aerial validation multirotor is missing.");
            body = multirotor.GetComponent<Rigidbody>();
            yield return RunValidation();
        }

        private IEnumerator RunValidation() {
            var result = new AerialValidationResult {
                unityVersion = Application.unityVersion,
                fixedDeltaTime = Time.fixedDeltaTime,
                hoverMotorSpeed = multirotor.HoverMotorSpeed,
                scene = SceneManager.GetActiveScene().name,
                referenceGeometryValid = true
            };
            CraneReferencePx4Walls reference = FindAnyObjectByType<CraneReferencePx4Walls>();
            if (reference != null) {
                result.referenceEnvironmentId = CraneReferencePx4Walls.EnvironmentId;
                result.referenceWallCount = 4;
                result.referenceGeometryValid = reference.ValidateCanonicalGeometry(out string message);
                if (!result.referenceGeometryValid)
                    Debug.LogError($"CRANE_REFERENCE_GEOMETRY_INVALID {message}");
                result.referenceRaycastValid = ValidatePx4WallRaycast(result);
            }

            ResetPose(new Vector3(0f, 5f, 0f), Quaternion.identity, result.hoverMotorSpeed);
            float hoverStart = body.position.y;
            yield return FixedSeconds(3f);
            result.hoverAltitudeDrift = body.position.y - hoverStart;
            result.hoverVerticalSpeed = body.linearVelocity.y;

            ResetPose(new Vector3(0f, 5f, 0f), Quaternion.identity, result.hoverMotorSpeed);
            float climbCommand = Mathf.Min(0.95f, result.hoverMotorSpeed + 0.12f);
            multirotor.SetCommand(climbCommand, 0f, 0f, 0f);
            yield return FixedSeconds(0.5f);
            float velocityBefore = body.linearVelocity.y;
            float thrust = multirotor.TotalThrust;
            yield return FixedSeconds(0.1f);
            result.measuredVerticalAcceleration = (body.linearVelocity.y - velocityBefore) / 0.1f;
            result.expectedVerticalAcceleration = thrust / body.mass - UnityEngine.Physics.gravity.magnitude;
            result.verticalAccelerationRelativeError = Mathf.Abs(result.measuredVerticalAcceleration -
                result.expectedVerticalAcceleration) / Mathf.Max(0.1f, Mathf.Abs(result.expectedVerticalAcceleration));

            yield return MeasureAxisResponse(result, new Vector3(0f, 0f, 1f), 1);
            yield return MeasureAxisResponse(result, new Vector3(1f, 0f, 0f), 2);
            yield return MeasureAxisResponse(result, new Vector3(0f, 1f, 0f), 3);

            ResetPose(new Vector3(0f, 5f, 0f), Quaternion.identity, result.hoverMotorSpeed);
            multirotor.SetWind(new Vector3(4f, 0f, 0f), new Vector3(1f, 0f, 0f));
            Vector3 windStart = body.position;
            yield return FixedSeconds(2f);
            result.windDisplacement = Mathf.Abs(body.position.x - windStart.x);
            multirotor.SetWind(Vector3.zero, Vector3.zero);

            if (reference != null) {
                ResetPose(new Vector3(3.5f, 5f, 0f), Quaternion.identity,
                    result.hoverMotorSpeed);
                body.linearVelocity = new Vector3(4f, 0f, 0f);
                yield return FixedSeconds(0.8f);
                result.referenceCollisionStopX = body.position.x;
                result.referenceCollisionValid = body.position.x < 4.35f;
            } else {
                result.referenceRaycastValid = true;
                result.referenceCollisionValid = true;
            }

            ResetPose(new Vector3(0f, 1.5f, 0f), Quaternion.identity, 0f);
            multirotor.SetCommand(0f, 0f, 0f, 0f);
            yield return FixedSeconds(2f);
            result.landingHeight = body.position.y;
            result.landingSpeed = body.linearVelocity.magnitude;

            ResetPose(new Vector3(0f, 5f, 0f), Quaternion.identity, 0f);
            multirotor.SetCommand(2f, 0f, 0f, 0f);
            yield return FixedSeconds(0.5f);
            result.saturatedMotorSpeed = multirotor.AverageMotorSpeed;

            result.hoverValid = Mathf.Abs(result.hoverAltitudeDrift) < 0.15f &&
                Mathf.Abs(result.hoverVerticalSpeed) < 0.1f;
            result.verticalValid = result.measuredVerticalAcceleration > 1f &&
                result.verticalAccelerationRelativeError < 0.3f;
            result.attitudeValid = result.rollAngularSpeed > 0.1f &&
                result.pitchAngularSpeed > 0.1f && result.yawAngularSpeed > 0.1f;
            result.windValid = result.windDisplacement > 0.05f;
            result.landingValid = result.landingHeight > 0.06f && result.landingHeight < 0.12f &&
                result.landingSpeed < 0.1f;
            result.saturationValid = result.saturatedMotorSpeed > 0.99f &&
                result.saturatedMotorSpeed <= 1f;
            result.valid = result.hoverValid && result.verticalValid && result.attitudeValid &&
                result.windValid && result.landingValid && result.saturationValid &&
                result.referenceGeometryValid && result.referenceRaycastValid &&
                result.referenceCollisionValid;

            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_AERIAL_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            Application.Quit(result.valid ? 0 : 2);
        }

        private IEnumerator MeasureAxisResponse(AerialValidationResult result, Vector3 axis, int commandAxis) {
            ResetPose(new Vector3(0f, 5f, 0f), Quaternion.identity, result.hoverMotorSpeed);
            if (commandAxis == 1) multirotor.SetCommand(result.hoverMotorSpeed, 0.25f, 0f, 0f);
            else if (commandAxis == 2) multirotor.SetCommand(result.hoverMotorSpeed, 0f, 0.25f, 0f);
            else multirotor.SetCommand(result.hoverMotorSpeed, 0f, 0f, 0.25f);
            yield return FixedSeconds(0.4f);
            Vector3 localAngularVelocity = body.transform.InverseTransformDirection(body.angularVelocity);
            float response = Vector3.Dot(localAngularVelocity, axis);
            if (commandAxis == 1) result.rollAngularSpeed = response;
            else if (commandAxis == 2) result.pitchAngularSpeed = response;
            else result.yawAngularSpeed = response;
        }

        private static bool ValidatePx4WallRaycast(AerialValidationResult result) {
            if (!UnityEngine.Physics.Raycast(new Vector3(0f, 5f, 0f), Vector3.right,
                    out RaycastHit hit, 20f)) return false;
            result.referenceRaycastDistance = hit.distance;
            var identity = hit.collider.GetComponent<
                Sim.Utils.ReferenceEnvironments.CraneSemanticIdentity>();
            return identity != null && identity.SemanticId == "wall-box-01" &&
                Mathf.Abs(hit.distance - 4.5f) < 0.001f;
        }

        private void ResetPose(Vector3 position, Quaternion rotation, float motorSpeed) {
            body.position = position;
            body.rotation = rotation;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            multirotor.SetWind(Vector3.zero, Vector3.zero);
            multirotor.ResetActuators(motorSpeed);
            multirotor.SetCommand(motorSpeed, 0f, 0f, 0f);
            UnityEngine.Physics.SyncTransforms();
        }

        private IEnumerator FixedSeconds(float seconds) {
            int steps = Mathf.CeilToInt(seconds / Time.fixedDeltaTime);
            for (int i = 0; i < steps; i++) yield return new WaitForFixedUpdate();
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }
    }
}
