using System;
using System.Collections;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Sim.Physics.Land {
    [Serializable]
    internal sealed class LandValidationResult {
        public string schema = "crane-land-validation-v2";
        public string unityVersion;
        public float fixedDeltaTime;
        public float accelerationStartSpeed;
        public float accelerationEndSpeed;
        public float accelerationMetersPerSecondSquared;
        public float accelerationDisplacement;
        public int accelerationGroundedWheels;
        public float accelerationAverageWheelRpm;
        public float accelerationAppliedWheelTorque;
        public float accelerationBodySpeed;
        public float accelerationBodyHeight;
        public float accelerationUpAlignment;
        public float coastStartSpeed;
        public float coastEndSpeed;
        public float brakingStartSpeed;
        public float brakingEndSpeed;
        public float stoppingDistance;
        public float expectedTurnRadius;
        public float measuredTurnRadius;
        public float turnDistance;
        public float turnYawDegrees;
        public float turnRadiusRelativeError;
        public bool accelerationValid;
        public bool coastValid;
        public bool brakingValid;
        public bool turningValid;
        public bool valid;
    }

    /// <summary>Standalone analytic rover checks enabled by --crane-land-validation.</summary>
    internal sealed class CraneLandValidationRunner : MonoBehaviour {
        private string outputPath;
        private AckermannRoverDynamics rover;
        private Rigidbody body;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install() {
            if (!Environment.GetCommandLineArgs().Contains("--crane-land-validation")) return;
            var host = new GameObject("CRANE Land Validation Runner");
            DontDestroyOnLoad(host);
            host.AddComponent<CraneLandValidationRunner>();
        }

        private IEnumerator Start() {
            string[] args = Environment.GetCommandLineArgs();
            outputPath = ReadArgument(args, "--crane-output") ??
                Path.Combine(Application.persistentDataPath, "crane-land-validation.json");
            if (SceneManager.GetActiveScene().name != "Land Vehicle Validation") {
                AsyncOperation load = SceneManager.LoadSceneAsync("Land Vehicle Validation", LoadSceneMode.Single);
                while (load != null && !load.isDone) yield return null;
            }
            rover = FindFirstObjectByType<AckermannRoverDynamics>();
            if (rover == null) throw new MissingReferenceException("Land validation rover is missing.");
            body = rover.GetComponent<Rigidbody>();
            yield return RunValidation();
        }

        private IEnumerator RunValidation() {
            var result = new LandValidationResult {
                unityVersion = Application.unityVersion,
                fixedDeltaTime = Time.fixedDeltaTime
            };
            rover.SetCommand(0f, 0f, 1f);
            yield return FixedSeconds(1f);

            result.accelerationStartSpeed = Mathf.Abs(rover.Speed);
            Vector3 accelerationStart = body.position;
            rover.SetCommand(1f, 0f, 0f);
            yield return FixedSeconds(5f);
            result.accelerationEndSpeed = Mathf.Abs(rover.Speed);
            result.accelerationDisplacement = Vector3.Distance(accelerationStart, body.position);
            result.accelerationGroundedWheels = rover.GroundedWheelCount;
            result.accelerationAverageWheelRpm = rover.AverageWheelRpm;
            result.accelerationAppliedWheelTorque = rover.AppliedWheelTorque;
            result.accelerationBodySpeed = body.linearVelocity.magnitude;
            result.accelerationBodyHeight = body.position.y;
            result.accelerationUpAlignment = Vector3.Dot(body.rotation * Vector3.up, Vector3.up);
            result.accelerationMetersPerSecondSquared =
                (result.accelerationEndSpeed - result.accelerationStartSpeed) / 5f;

            result.brakingStartSpeed = Mathf.Abs(rover.Speed);
            Vector3 brakingStart = body.position;
            rover.SetCommand(0f, 0f, 1f);
            yield return FixedSeconds(3f);
            result.brakingEndSpeed = Mathf.Abs(rover.Speed);
            result.stoppingDistance = Vector3.Distance(brakingStart, body.position);

            ResetPose(new Vector3(0f, 0.65f, 0f), Quaternion.identity);
            yield return FixedSeconds(0.5f);
            rover.SetCommand(1f, 0f, 0f);
            yield return FixedSeconds(5f);
            result.coastStartSpeed = Mathf.Abs(rover.Speed);
            rover.SetCommand(0f, 0f, 0f);
            yield return FixedSeconds(4f);
            result.coastEndSpeed = Mathf.Abs(rover.Speed);

            ResetPose(new Vector3(0f, 0.65f, 0f), Quaternion.identity);
            yield return FixedSeconds(0.5f);
            rover.SetCommand(0.55f, 1f, 0f);
            Vector3 previousPosition = body.position;
            float traveled = 0f;
            float accumulatedYaw = 0f;
            float previousYaw = body.rotation.eulerAngles.y;
            int turnSteps = Mathf.CeilToInt(7f / Time.fixedDeltaTime);
            for (int i = 0; i < turnSteps; i++) {
                yield return new WaitForFixedUpdate();
                traveled += Vector3.Distance(previousPosition, body.position);
                previousPosition = body.position;
                float yaw = body.rotation.eulerAngles.y;
                accumulatedYaw += Mathf.Abs(Mathf.DeltaAngle(previousYaw, yaw));
                previousYaw = yaw;
            }
            result.expectedTurnRadius = rover.ExpectedCenterTurnRadius(1f);
            result.measuredTurnRadius = accumulatedYaw < 1f ? float.PositiveInfinity :
                traveled / (accumulatedYaw * Mathf.Deg2Rad);
            result.turnDistance = traveled;
            result.turnYawDegrees = accumulatedYaw;
            result.turnRadiusRelativeError = Mathf.Abs(result.measuredTurnRadius -
                result.expectedTurnRadius) / result.expectedTurnRadius;

            result.accelerationValid = result.accelerationEndSpeed > 2f &&
                result.accelerationMetersPerSecondSquared > 0.3f &&
                result.accelerationGroundedWheels >= 3 &&
                result.accelerationUpAlignment > 0.95f;
            result.coastValid = result.coastEndSpeed < result.coastStartSpeed &&
                result.coastEndSpeed > 0.1f;
            result.brakingValid = result.brakingEndSpeed < 0.3f && result.stoppingDistance < 8f;
            result.turningValid = !float.IsNaN(result.measuredTurnRadius) &&
                !float.IsInfinity(result.measuredTurnRadius) &&
                result.turnDistance > 2.5f && result.turnYawDegrees > 45f &&
                result.turnRadiusRelativeError < 0.35f;
            result.valid = result.accelerationValid && result.coastValid &&
                result.brakingValid && result.turningValid;

            string directory = Path.GetDirectoryName(outputPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(outputPath, JsonUtility.ToJson(result, true));
            Debug.Log($"CRANE_LAND_VALIDATION_COMPLETE {outputPath} valid={result.valid}");
            Application.Quit(result.valid ? 0 : 2);
        }

        private IEnumerator FixedSeconds(float seconds) {
            int steps = Mathf.CeilToInt(seconds / Time.fixedDeltaTime);
            for (int i = 0; i < steps; i++) yield return new WaitForFixedUpdate();
        }

        private void ResetPose(Vector3 position, Quaternion rotation) {
            rover.SetCommand(0f, 0f, 1f);
            rover.ResetActuators();
            body.position = position;
            body.rotation = rotation;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            UnityEngine.Physics.SyncTransforms();
        }

        private static string ReadArgument(string[] args, string key) {
            int index = Array.IndexOf(args, key);
            return index >= 0 && index + 1 < args.Length ? args[index + 1] : null;
        }
    }
}
