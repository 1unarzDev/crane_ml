// EXPLORATORY opt-in fault actuation. Never infer that a requested fault occurred.
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Crane.MainVenueV8 {
    [Serializable] public sealed class FaultEventV8 {
        public string kind;
        public double start_s, end_s;
        public Vector3 position_local, direction_local, size_m;
        public float magnitude;
    }
    [Serializable] public sealed class EncoderBindingV8 {
        public string path, kind;
        public Vector3 axis_local;
        public float radius_m, polarity;
    }
    [Serializable] public sealed class FaultSpecV8 {
        public string schema, status, target_path, motor_component, scene_name;
        public MotorBindingV8[] motors = Array.Empty<MotorBindingV8>();
        public EncoderBindingV8[] encoders = Array.Empty<EncoderBindingV8>();
        public FaultEventV8[] events = Array.Empty<FaultEventV8>();
    }

    [DefaultExecutionOrder(-30000)]
    public sealed class CraneFaultRuntimeV8 : MonoBehaviour {
        sealed class FrictionState { public WheelFrictionCurve forward, sideways; }
        sealed class EventState {
            public FaultEventV8 spec;
            public bool begun, ended;
            public GameObject spawned;
            public Dictionary<Collider, PhysicsMaterial> materials = new Dictionary<Collider, PhysicsMaterial>();
            public PhysicsMaterial material;
            public readonly Dictionary<Behaviour, bool> motorEnabled = new Dictionary<Behaviour, bool>();
            public readonly Dictionary<WheelCollider, float> motorTorques = new Dictionary<WheelCollider, float>();
            public readonly Dictionary<WheelCollider, FrictionState> friction = new Dictionary<WheelCollider, FrictionState>();
        }
        Rigidbody target;
        Behaviour[] motors;
        EncoderBindingV8[] bindings;
        readonly List<EventState> events = new List<EventState>();
        public double StartedAt { get; private set; }
        Action<string> privateEvent;
        bool configured;

        public static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
        static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        public static void Validate(FaultSpecV8 spec) {
            if (spec == null || spec.schema != "crane-fault-runtime/v8" || spec.status != "EXPLORATORY_SIMULATION_ONLY" || string.IsNullOrWhiteSpace(spec.target_path))
                throw new ArgumentException("Explicit simulation-only spec required");
            if (spec.events == null || spec.encoders == null || spec.motors == null || spec.events.Length > 2)
                throw new ArgumentException("Null, single-fault or ambiguous two-fault episodes only");
            var kinds = new HashSet<string>();
            foreach (var e in spec.events) {
                if (e == null || !kinds.Add(e.kind) || !Finite(e.start_s) || !Finite(e.end_s) || e.start_s < 0 || e.end_s <= e.start_s || e.end_s > 120 || !Finite(e.position_local) || !Finite(e.direction_local) || !Finite(e.size_m) || !Finite(e.magnitude))
                    throw new ArgumentException("Invalid bounded event");
                if (e.kind != "slip" && e.kind != "collision" && e.kind != "motor_fault" && e.kind != "obstruction" && e.kind != "push")
                    throw new ArgumentException("Software faults require the separate ROS adapter");
                if (e.kind == "slip" && spec.encoders.Length < 2) throw new ArgumentException("Slip requires actual contact-driven wheel bindings");
                if (e.kind == "motor_fault" && string.IsNullOrWhiteSpace(spec.motor_component) && spec.motors.Length == 0) throw new ArgumentException("Motor actuator component binding required");
                if ((e.kind == "collision" || e.kind == "push") && (e.direction_local.sqrMagnitude < .99f || e.direction_local.sqrMagnitude > 1.01f || e.magnitude <= 0 || e.magnitude > 100))
                    throw new ArgumentException("Finite unit direction and bounded positive magnitude required");
                if ((e.kind == "collision" || e.kind == "obstruction") && (e.size_m.x <= 0 || e.size_m.y <= 0 || e.size_m.z <= 0 || e.size_m.sqrMagnitude > 100))
                    throw new ArgumentException("Physical obstacle dimensions required");
            }
            if (!string.IsNullOrWhiteSpace(spec.motor_component) && spec.motors.Length > 0) throw new ArgumentException("Choose legacy root motor or explicit motor group, not both");
            var motorKeys = new HashSet<string>();
            foreach (var m in spec.motors) {
                if (m == null || m.path == null || string.IsNullOrWhiteSpace(m.component_type) || !motorKeys.Add(m.path + "|" + m.component_type))
                    throw new ArgumentException("Unique explicit actuator group bindings required");
            }
            var paths = new HashSet<string>();
            foreach (var b in spec.encoders) {
                if (b == null || string.IsNullOrWhiteSpace(b.path) || !paths.Add(b.path) || !Finite(b.axis_local) || Math.Abs(b.axis_local.sqrMagnitude - 1) > .01 || !Finite(b.radius_m) || b.radius_m <= 0 || b.radius_m > 1 || (b.polarity != 1 && b.polarity != -1))
                    throw new ArgumentException("Explicit calibrated physical shaft binding required");
            }
        }

        public void Configure(FaultSpecV8 spec, double now, Action<string> privateLogger = null) {
            if (configured) throw new InvalidOperationException("No in-place episode reconfiguration");
            Validate(spec);
            if (!Finite(now) || now < 0) throw new ArgumentException("Finite nonnegative runtime origin required");
            target = GetComponent<Rigidbody>();
            if (target == null || target.isKinematic) throw new ArgumentException("Dynamic robot Rigidbody required");
            bindings = spec.encoders;
            motors = ResolveMotors(target, spec);
            // Fail before any actuation if an encoder is a virtual/reference shaft.
            ResolveWheels(target, bindings);
            foreach (var e in spec.events) events.Add(new EventState { spec = e });
            StartedAt = now;
            privateEvent = privateLogger;
            configured = true;
            privateEvent?.Invoke("{\"event\":\"runtime_ready\",\"simulation_origin_s\":" + now.ToString("R", CultureInfo.InvariantCulture) + "}");
        }

        public static EncoderChannelV8[] ResolveWheels(Rigidbody robot, EncoderBindingV8[] descriptors) {
            var wheels = new List<EncoderChannelV8>();
            foreach (var b in descriptors) wheels.Add(EncoderChannelV8.Resolve(robot, b));
            return wheels.ToArray();
        }
        public static Behaviour[] ResolveMotors(Rigidbody robot, FaultSpecV8 spec) {
            var result = new List<Behaviour>();
            var bindings = new List<MotorBindingV8>(spec.motors);
            if (!string.IsNullOrWhiteSpace(spec.motor_component)) bindings.Add(new MotorBindingV8 { path = "", component_type = spec.motor_component });
            foreach (var binding in bindings) {
                var owner = string.IsNullOrEmpty(binding.path) ? robot.transform : robot.transform.Find(binding.path);
                Behaviour selected = null;
                if (owner != null) foreach (var b in owner.GetComponents<Behaviour>()) if (b.GetType().FullName == binding.component_type) {
                    if (selected != null) throw new ArgumentException("Ambiguous motor component binding");
                    selected = b;
                }
                if (selected == null || selected is CraneFaultRuntimeV8 || selected is CraneSensorRecorderV8)
                    throw new ArgumentException("Bound drive actuator unavailable");
                result.Add(selected);
            }
            return result.ToArray();
        }

        void FixedUpdate() { if (configured) Tick(Time.fixedTimeAsDouble); }
        public void Tick(double now) {
            if (!configured || !Finite(now) || now < StartedAt) throw new InvalidOperationException("Invalid runtime time/state");
            double elapsed = now - StartedAt;
            foreach (var state in events) {
                var e = state.spec;
                if (!state.begun && elapsed >= e.end_s) throw new InvalidOperationException("Scheduled actuation window was missed");
                if (!state.begun && elapsed >= e.start_s && elapsed < e.end_s) Begin(state, elapsed);
                if (state.begun && !state.ended && elapsed >= e.end_s) End(state);
                if (state.begun && !state.ended && e.kind == "push")
                    target.AddForce(target.transform.TransformDirection(e.direction_local) * e.magnitude, ForceMode.Force);
            }
        }
        void Begin(EventState state, double elapsed) {
            var e = state.spec;
            state.begun = true;
            if (e.kind == "motor_fault") {
                foreach (var motor in motors) { state.motorEnabled.Add(motor, motor.enabled); motor.enabled = false; }
                // PhysX WheelColliders retain their last applied torque even when
                // a controller stops updating. A real output cut must zero it.
                foreach (var wheel in target.GetComponentsInChildren<WheelCollider>()) { state.motorTorques.Add(wheel, wheel.motorTorque); wheel.motorTorque = 0; }
            }
            else if (e.kind == "slip") {
                state.material = new PhysicsMaterial("v8-low-friction") { staticFriction = 0, dynamicFriction = 0, frictionCombine = PhysicsMaterialCombine.Minimum };
                foreach (var wheel in ResolveWheels(target, bindings)) {
                    if (wheel.wheel != null) {
                        state.friction.Add(wheel.wheel, new FrictionState { forward = wheel.wheel.forwardFriction, sideways = wheel.wheel.sidewaysFriction });
                        var forward = wheel.wheel.forwardFriction; var sideways = wheel.wheel.sidewaysFriction;
                        forward.stiffness = 0; sideways.stiffness = 0;
                        wheel.wheel.forwardFriction = forward; wheel.wheel.sidewaysFriction = sideways;
                    } else foreach (var c in wheel.shaft.GetComponents<Collider>()) { state.materials.Add(c, c.sharedMaterial); c.sharedMaterial = state.material; }
                }
            } else if (e.kind == "collision" || e.kind == "obstruction") {
                state.spawned = GameObject.CreatePrimitive(PrimitiveType.Cube);
                state.spawned.name = "v8-runtime-object";
                state.spawned.transform.position = target.position + target.rotation * e.position_local;
                state.spawned.transform.localScale = e.size_m;
                if (e.kind == "collision") {
                    var body = state.spawned.AddComponent<Rigidbody>(); body.mass = 1; body.useGravity = false;
                    body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                    body.linearVelocity = target.transform.TransformDirection(e.direction_local) * e.magnitude;
                }
            }
            privateEvent?.Invoke("{\"event\":\"actuator_started\",\"kind\":\"" + e.kind + "\",\"elapsed_s\":" + elapsed.ToString("R", CultureInfo.InvariantCulture) + "}");
        }
        void End(EventState state) {
            state.ended = true;
            foreach (var pair in state.motorEnabled) if (pair.Key != null) pair.Key.enabled = pair.Value;
            foreach (var pair in state.motorTorques) if (pair.Key != null) pair.Key.motorTorque = pair.Value;
            foreach (var pair in state.friction) if (pair.Key != null) { pair.Key.forwardFriction = pair.Value.forward; pair.Key.sidewaysFriction = pair.Value.sideways; }
            foreach (var pair in state.materials) if (pair.Key != null) pair.Key.sharedMaterial = pair.Value;
            if (state.spawned != null) { state.spawned.SetActive(false); Release(state.spawned); }
            if (state.material != null) Release(state.material);
            privateEvent?.Invoke("{\"event\":\"actuator_ended\",\"kind\":\"" + state.spec.kind + "\"}");
        }
        static void Release(UnityEngine.Object value) { if (Application.isPlaying) Destroy(value); else DestroyImmediate(value); }
        public void Stop() { foreach (var e in events) if (e.begun && !e.ended) End(e); configured = false; }
        void OnDisable() { Stop(); }
    }

    // No changes to existing scenes unless the explicit runtime arguments exist.
    public static class FaultBootstrapV8 {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void Install() {
            var args = Environment.GetCommandLineArgs();
            int index = Array.IndexOf(args, "--crane-upgrade-v8-spec");
            if (index < 0) return;
            int capture = Array.IndexOf(args, "--crane-upgrade-v8-sensors");
            int events = Array.IndexOf(args, "--crane-upgrade-v8-private-events");
            if (index + 1 >= args.Length || capture < 0 || capture + 1 >= args.Length || events < 0 || events + 1 >= args.Length)
                throw new ArgumentException("Separate spec, robot-sensor and private-event paths required");
            string specPath = Path.GetFullPath(args[index + 1]), sensorPath = Path.GetFullPath(args[capture + 1]), eventPath = Path.GetFullPath(args[events + 1]);
            if (specPath == sensorPath || specPath == eventPath || sensorPath == eventPath) throw new ArgumentException("Runtime paths must be distinct");
            FaultSpecV8 spec = JsonUtility.FromJson<FaultSpecV8>(File.ReadAllText(specPath));
            CraneFaultRuntimeV8.Validate(spec);
            if (string.IsNullOrWhiteSpace(spec.scene_name)) throw new ArgumentException("Explicit scene binding required at runtime bootstrap");
            SceneManager.sceneLoaded += (scene, _) => {
                if (scene.name != spec.scene_name) return;
                var host = new GameObject("CRANE V8 bounded readiness");
                host.AddComponent<RuntimeReadinessV8>().Begin(spec, scene, sensorPath, eventPath);
            };
        }
    }
}
