#if UNITY_EDITOR
using System;
using System.Linq.Expressions;
using System.Reflection;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Utils.ROS;

public static class CraneStartupDiagnostic {
    public static void Run() {
        string originalPath = SceneManager.GetActiveScene().path;
        Scene outgoing = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        outgoing.name = "Startup diagnostic outgoing";
        var host = new GameObject("Startup diagnostic");
        var screen = new GameObject("Outgoing screen").AddComponent<Camera>();
        SceneManager.MoveGameObjectToScene(screen.gameObject, outgoing);
        var sensor = new GameObject("Outgoing sensor").AddComponent<Camera>();
        var texture = new RenderTexture(8, 8, 0);
        sensor.targetTexture = texture;
        SceneManager.MoveGameObjectToScene(sensor.gameObject, outgoing);
        Type guardType = Type.GetType("Sim.Performance.CraneStartupRenderGuard, PerformanceAssembly", true);
        Component guard = host.AddComponent(guardType);
        try {
            guardType.GetMethod("Configure").Invoke(guard, new object[] { "Startup diagnostic target" });
            guardType.GetMethod("HandleSceneLoaded").Invoke(guard,
                new object[] { outgoing, LoadSceneMode.Single });
            Require(!screen.enabled && sensor.enabled, "outgoing screen hidden; sensor unchanged");
            screen.enabled = true;
            guardType.GetMethod("SuppressOutgoingCameras").Invoke(guard, null);
            Require(!screen.enabled, "late camera re-enable suppressed");
            // Rename this disposable scene to simulate the target event without
            // saving an untitled test scene into the repository.
            outgoing.name = "Startup diagnostic target";
            Scene target = SceneManager.GetActiveScene();
            var targetCamera = new GameObject("Target screen").AddComponent<Camera>();
            var disabledTarget = new GameObject("Target disabled").AddComponent<Camera>();
            disabledTarget.enabled = false;
            guardType.GetMethod("HandleSceneLoaded").Invoke(guard,
                new object[] { target, LoadSceneMode.Single });
            Require(targetCamera.enabled && !disabledTarget.enabled,
                "target camera enabled states preserved");
            Require(!((Behaviour)guard).enabled, "guard retires after target load");
            VerifyEarlyRegistration(host, "RosMessageTypes.Nav.OdometryMsg", true);
            VerifyEarlyRegistration(host, "RosMessageTypes.Geometry.TwistStampedMsg", false);
            Debug.Log("CRANE_STARTUP_DIAGNOSTIC_PASS outgoingScreenHidden=true targetStatesPreserved=true earlyPublisherReady=true earlySubscriberReady=true");
        }
        finally {
            UnityEngine.Object.DestroyImmediate(host);
            UnityEngine.Object.DestroyImmediate(texture);
            if (!string.IsNullOrEmpty(originalPath)) EditorSceneManager.OpenScene(originalPath, OpenSceneMode.Single);
            else EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        }
    }

    private static void VerifyEarlyRegistration(GameObject host, string messageName, bool publisher) {
        Type message = Type.GetType(messageName + ", Unity.Robotics.ROSTCPConnector.Messages", true);
        Type registry = Type.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageRegistry, Unity.Robotics.ROSTCPConnector.MessageGeneration", true);
        Type entry = registry.GetNestedType("RegistryEntry`1", BindingFlags.NonPublic).MakeGenericType(message);
        FieldInfo name = entry.GetField("s_RosMessageName", BindingFlags.Public | BindingFlags.Static);
        if (name.GetValue(null) == null) message.GetMethod("Register").Invoke(null, null);
        object registeredName = name.GetValue(null);
        name.SetValue(null, null); // Reproduce initial sceneLoaded before generated registration.
        string topic = publisher ? "/startup_diagnostic/odom" : "/startup_diagnostic/cmd";
        Component wrapper;
        try {
            if (publisher) {
                wrapper = host.AddComponent<ROSPublisher>();
                var factory = Expression.Lambda(typeof(Func<>).MakeGenericType(message),
                    Expression.New(message)).Compile();
                typeof(ROSPublisher).GetMethod("Initialize").MakeGenericMethod(message)
                    .Invoke(wrapper, new object[] { topic, "base_link", factory, 10f, false });
            } else {
                wrapper = host.AddComponent<ROSSubscriber>();
                var callback = Expression.Lambda(typeof(Action<>).MakeGenericType(message),
                    Expression.Empty(), Expression.Parameter(message, "msg")).Compile();
                MethodInfo initialize = Array.Find(typeof(ROSSubscriber).GetMethods(),
                    m => m.Name == "Initialize" && m.GetParameters().Length == 2);
                initialize.MakeGenericMethod(message).Invoke(wrapper, new object[] { topic, callback });
            }
            Require(GetTopic(topic) == null, "early registration deferred while message name missing");
            name.SetValue(null, registeredName);
            wrapper.GetType().GetMethod("Start", BindingFlags.Instance | BindingFlags.NonPublic)
                .Invoke(wrapper, null);
            object state = GetTopic(topic);
            Require(state != null && Equals(state.GetType().GetProperty("RosMessageName").GetValue(state), registeredName),
                "registration uses the initialized ROS message name");
        } finally { name.SetValue(null, registeredName); }
    }

    private static object GetTopic(string topic) {
        Type type = Type.GetType("Unity.Robotics.ROSTCPConnector.ROSConnection, Unity.Robotics.ROSTCPConnector", true);
        object connection = type.GetMethod("GetOrCreateInstance").Invoke(null, null);
        return type.GetMethod("GetTopic").Invoke(connection, new object[] { topic });
    }
    private static void Require(bool condition, string reason) {
        if (!condition) throw new Exception("CRANE startup: " + reason);
    }
}
#endif
