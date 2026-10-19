// Read-only scene structure inspection. Never saves/recreates a study scene.
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Crane.MainVenueV8;

public static class CourseInventoryV8 {
    [Serializable] public sealed class Motor { public string path, component_type; public bool enabled; }
    [Serializable] public sealed class Wheel { public string path, kind; public float radius; public Vector3 axis, scale; }
    [Serializable] public sealed class Body { public string path; public bool kinematic; public float mass; public List<Motor> actuators=new List<Motor>(); public List<Wheel> wheels=new List<Wheel>(); }
    [Serializable] public sealed class Course { public string scene_path, source_sha256; public List<Body> bodies=new List<Body>(); }
    [Serializable] public sealed class Report { public string status="EXPLORATORY_STATIC_SCENE_INVENTORY_NOT_RUNTIME_QUALIFICATION"; public List<Course> courses=new List<Course>(); }
    static string Hash(string path) { using(var h=SHA256.Create()) using(var f=File.OpenRead(path)) return BitConverter.ToString(h.ComputeHash(f)).Replace("-","").ToLowerInvariant(); }
    public static void Export() {
        var args=Environment.GetCommandLineArgs(); int outputIndex=Array.IndexOf(args,"--crane-v8-inventory-output");
        if(outputIndex<0||outputIndex+1>=args.Length)throw new ArgumentException("Fresh explicit inventory output required");
        string output=Path.GetFullPath(args[outputIndex+1]);
        if(File.Exists(output))throw new IOException("Inventory output already exists");
        string[] scenes={"Assets/Scenes/Land Vehicle Validation.unity","Assets/Scenes/TurtleBot3 Warehouse Validation.unity","Assets/Scenes/Roboboat Course.unity"};
        var report=new Report();
        foreach(string path in scenes) {
            string before=Hash(path); var scene=EditorSceneManager.OpenScene(path,OpenSceneMode.Single);
            var course=new Course {scene_path=path,source_sha256=before};
            foreach(var root in scene.GetRootGameObjects()) foreach(var body in root.GetComponentsInChildren<Rigidbody>(true)) {
                var item=new Body {path=RuntimeReadinessV8.PathOf(body.transform),kinematic=body.isKinematic,mass=body.mass};
                foreach(var actuator in body.GetComponentsInChildren<MonoBehaviour>(true)) {
                    if(actuator==null)throw new InvalidOperationException("Missing scene script");
                    string type=actuator.GetType().FullName;
                    if(type.EndsWith("Dynamics",StringComparison.Ordinal)||type.EndsWith("Thruster",StringComparison.Ordinal)||type.EndsWith("CraneCampusRobotModel",StringComparison.Ordinal))
                        item.actuators.Add(new Motor {path=Relative(body.transform,actuator.transform),component_type=type,enabled=actuator.enabled});
                }
                foreach(var wheel in body.GetComponentsInChildren<WheelCollider>(true)) if(wheel.attachedRigidbody==body)
                    item.wheels.Add(new Wheel {path=Relative(body.transform,wheel.transform),kind="wheel_collider",radius=wheel.radius,axis=Vector3.right,scale=wheel.transform.lossyScale});
                foreach(var joint in body.GetComponentsInChildren<HingeJoint>(true)) if(joint.connectedBody==body&&joint.GetComponent<Rigidbody>()!=null)
                    item.wheels.Add(new Wheel {path=Relative(body.transform,joint.transform),kind="hinge",radius=-1,axis=joint.axis,scale=joint.transform.lossyScale});
                course.bodies.Add(item);
            }
            if(Hash(path)!=before)throw new IOException("Scene changed during read-only inventory");
            report.courses.Add(course);
        }
        using(var stream=new StreamWriter(new FileStream(output,FileMode.CreateNew,FileAccess.Write))) stream.WriteLine(JsonUtility.ToJson(report,true));
        Debug.Log("CRANE_V8_SCENE_INVENTORY_WRITTEN "+output);
    }
    static string Relative(Transform parent,Transform child) {
        if(parent==child)return "";
        string prefix=RuntimeReadinessV8.PathOf(parent)+"/",path=RuntimeReadinessV8.PathOf(child);
        if(!path.StartsWith(prefix,StringComparison.Ordinal))throw new ArgumentException("Actuator is outside the bound robot");
        return path.Substring(prefix.Length);
    }
}
