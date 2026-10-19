// EXPLORATORY bounded setup barrier; missing/ambiguous bindings fail, never fallback.
using System;
using System.Collections;
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Crane.MainVenueV8 {
    public sealed class RuntimeReadinessV8 : MonoBehaviour {
        public bool Attached { get; private set; }
        public string LastReason { get; private set; }
        public static bool TryResolve(FaultSpecV8 spec, Scene scene, out GameObject robot, out string reason) {
            robot = null; reason = "target path not present and active";
            foreach (var root in scene.GetRootGameObjects()) {
                foreach (var t in root.GetComponentsInChildren<Transform>(true)) {
                    if (!t.gameObject.activeInHierarchy || PathOf(t) != spec.target_path) continue;
                    if (robot != null) { robot = null; reason = "ambiguous target path"; return false; }
                    robot = t.gameObject;
                }
            }
            if (robot == null) return false;
            try {
                var body = PhysicalBodyV8.Resolve(robot);
                CraneFaultRuntimeV8.ResolveWheels(body.component, spec.encoders); CraneFaultRuntimeV8.ResolveMotors(body.component, spec);
            }
            catch (ArgumentException error) { reason = error.Message; return false; }
            reason = "bindings resolved"; return true;
        }
        public static string PathOf(Transform t) {
            string path = t.name;
            while (t.parent != null) { t = t.parent; path = t.name + "/" + path; }
            return path;
        }
        public void Begin(FaultSpecV8 spec, Scene scene, string sensorPath, string eventPath) {
            CraneFaultRuntimeV8.Validate(spec);
            StartCoroutine(WaitAndAttach(spec, scene, sensorPath, eventPath));
        }
        IEnumerator WaitAndAttach(FaultSpecV8 spec, Scene scene, string sensorPath, string eventPath) {
            double deadline = Time.realtimeSinceStartupAsDouble + 15;
            // Start() and late mounting must finish before a runtime origin is
            // frozen. Require bindings to survive two completed physics frames.
            int stableFrames = 0;
            GameObject robot = null;
            while (stableFrames < 2) {
                if (Time.realtimeSinceStartupAsDouble >= deadline) {
                    Debug.LogError("CRANE_V8_SETUP_FAILED " + LastReason);
                    enabled = false;
                    yield break;
                }
                string reason;
                if (TryResolve(spec, scene, out robot, out reason)) {
                    LastReason = reason; stableFrames++;
                    yield return new WaitForFixedUpdate();
                } else {
                    LastReason = reason; stableFrames = 0;
                    yield return null;
                }
            }
            string finalReason;
            if (!TryResolve(spec, scene, out robot, out finalReason)) {
                Debug.LogError("CRANE_V8_SETUP_FAILED " + finalReason); yield break;
            }
            if (robot.GetComponent<CraneFaultRuntimeV8>() != null || robot.GetComponent<CraneSensorRecorderV8>() != null)
                throw new InvalidOperationException("Episode runtime already attached; use a new process/output");
            if (File.Exists(sensorPath) || File.Exists(eventPath)) throw new IOException("Fresh sensor/private outputs required");
            var privateLog = new StreamWriter(new FileStream(eventPath, FileMode.CreateNew, FileAccess.Write));
            CraneFaultRuntimeV8 runtime = null;
            CraneSensorRecorderV8 sensors = null;
            try {
                runtime = robot.AddComponent<CraneFaultRuntimeV8>();
                runtime.Configure(spec, Time.fixedTimeAsDouble, line => { privateLog.WriteLine(line); privateLog.Flush(); });
                sensors = robot.AddComponent<CraneSensorRecorderV8>();
                sensors.Configure(spec, sensorPath, privateLog);
                Attached = true;
            } catch {
                if (runtime != null) { runtime.Stop(); runtime.enabled = false; }
                if (sensors != null) { sensors.Close(); sensors.enabled = false; }
                privateLog.Dispose(); throw;
            }
        }
    }
}
