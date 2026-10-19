// EXPLORATORY null binding validation only; no dynamics, ROS or episodes run.
using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using UnityEditor.SceneManagement;
using UnityEngine;
using Crane.MainVenueV8;

public static class CourseBindingCheckV8 {
    [Serializable] public sealed class Binding { public string scene_path,scene_sha256,body_kind;public FaultSpecV8 runtime_spec; }
    [Serializable] public sealed class Input { public Binding[] bindings; }
    [Serializable] public sealed class Verdict { public string scene_path,target_path,body_kind,reason;public bool resolved;public int motors,encoders; }
    [Serializable] public sealed class Report { public string status="EXPLORATORY_STATIC_BINDING_CHECK_NOT_NAV2_OR_SENSOR_QUALIFICATION";public List<Verdict> verdicts=new List<Verdict>(); }
    static string Hash(string path) {using(var h=SHA256.Create())using(var f=File.OpenRead(path))return BitConverter.ToString(h.ComputeHash(f)).Replace("-","").ToLowerInvariant();}
    static string Arg(string name) {var args=Environment.GetCommandLineArgs();int i=Array.IndexOf(args,name);if(i<0||i+1>=args.Length)throw new ArgumentException(name+" required");return Path.GetFullPath(args[i+1]);}
    public static void Export() {
        string output=Arg("--crane-v8-binding-output"),input=Arg("--crane-v8-binding-input");
        if(File.Exists(output))throw new IOException("Fresh binding output required");
        var bindings=JsonUtility.FromJson<Input>(File.ReadAllText(input));
        if(bindings.bindings==null||bindings.bindings.Length==0)throw new ArgumentException("Nonempty explicit bindings required");
        var report=new Report();bool passed=true;
        foreach(var binding in bindings.bindings) {
            if(Hash(binding.scene_path)!=binding.scene_sha256)throw new IOException("Scene hash differs from retained binding");
            CraneFaultRuntimeV8.Validate(binding.runtime_spec);
            if(binding.runtime_spec.events.Length!=0)throw new ArgumentException("Static checker accepts null specifications only");
            var scene=EditorSceneManager.OpenScene(binding.scene_path,OpenSceneMode.Single);
            if(scene.name!=binding.runtime_spec.scene_name)throw new ArgumentException("Scene name mismatch");
            GameObject robot;string reason;bool resolved=RuntimeReadinessV8.TryResolve(binding.runtime_spec,scene,out robot,out reason);
            var verdict=new Verdict {scene_path=binding.scene_path,target_path=binding.runtime_spec.target_path,body_kind=binding.body_kind,reason=reason,resolved=resolved};
            if(resolved) {
                var body=PhysicalBodyV8.Resolve(robot);
                string actual=body.rigid!=null?"Rigidbody":"ArticulationBody";
                if(actual!=binding.body_kind)throw new ArgumentException("Physical body type mismatch");
                verdict.motors=CraneFaultRuntimeV8.ResolveMotors(body.component,binding.runtime_spec).Length;
                verdict.encoders=CraneFaultRuntimeV8.ResolveWheels(body.component,binding.runtime_spec.encoders).Length;
            }
            passed&=resolved;report.verdicts.Add(verdict);
            if(Hash(binding.scene_path)!=binding.scene_sha256)throw new IOException("Static checker changed scene");
        }
        using(var stream=new StreamWriter(new FileStream(output,FileMode.CreateNew,FileAccess.Write)))stream.WriteLine(JsonUtility.ToJson(report,true));
        if(!passed)throw new InvalidOperationException("One or more course bindings failed");
        Debug.Log("CRANE_V8_COURSE_BINDINGS_RESOLVED "+output);
    }
}
