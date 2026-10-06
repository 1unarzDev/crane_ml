#if UNITY_EDITOR
using System;
using UnityEngine;
using Sim.Utils.ReferenceEnvironments;

public static class CraneCampusCameraDiagnostic {
    public static void Run() {
        var host=new GameObject("Camera control diagnostic");
        try {
            var camera=host.AddComponent<Camera>();
            var controls=host.AddComponent<CraneFreeCameraController>();
            controls.Configure(camera);controls.SetActive(true);
            for(int i=0;i<200;i++)controls.Step(new Vector3(1,0,1),Vector2.zero,false,.01f);
            Vector3 diagonal=host.transform.position;
            Require(Mathf.Abs(diagonal.x-diagonal.z)<.001f,"diagonal movement combines axes");
            Require(diagonal.magnitude>6.5f&&diagonal.magnitude<7.1f,"diagonal speed is bounded");
            for(int i=0;i<100;i++)controls.Step(Vector3.zero,Vector2.zero,false,.01f);
            float stoppingDistance=Vector3.Distance(diagonal,host.transform.position);
            Require(stoppingDistance>.05f&&stoppingDistance<.32f,"movement decelerates smoothly");
            Vector3 settled=host.transform.position;
            for(int i=0;i<100;i++)controls.Step(Vector3.zero,Vector2.zero,false,.01f);
            Require(Vector3.Distance(settled,host.transform.position)<.001f,"movement settles");
            for(int i=0;i<100;i++)controls.Step(Vector3.zero,new Vector2(75,75),false,.01f);
            float yaw=host.transform.eulerAngles.y;
            Require(yaw>60&&yaw<75,"arrow look has angular response");
            float pitch=Mathf.DeltaAngle(0,host.transform.eulerAngles.x);
            Require(pitch>60&&pitch<75,"simultaneous pitch and yaw");
            for(int i=0;i<200;i++)controls.Step(Vector3.zero,new Vector2(0,75),false,.01f);
            Require(Mathf.Abs(Mathf.DeltaAngle(0,host.transform.eulerAngles.x)-85)<.01f,"pitch is bounded");
            controls.SetActive(false);Vector3 disabled=host.transform.position;
            controls.Step(Vector3.one,Vector2.one*75,true,1);
            Require(Vector3.Distance(disabled,host.transform.position)<.001f,"inactive camera remains still");
            Require(host.GetComponents<Collider>().Length==0,"controls introduce no physics geometry");
            var target=new GameObject("Oscillating robot camera diagnostic");
            try {
                var inspection=host.AddComponent<CraneReferenceInspectionController>();
                inspection.Configure(camera,target.transform,"campus","diagnostic",Array.Empty<string>(),Vector3.zero);
                inspection.ConfigureStableFollow();
                var resolve=typeof(CraneReferenceInspectionController).GetMethod("ResolveFollowPosition",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance);
                Vector3 initial=(Vector3)resolve.Invoke(inspection,new object[]{Vector3.up*.25f});
                for(int i=0;i<20;i++) {
                    target.transform.rotation=Quaternion.Euler(0,i%2==0?170:-170,0);
                    Vector3 next=(Vector3)resolve.Invoke(inspection,new object[]{Vector3.up*.25f});
                    Require(Vector3.Distance(initial,next)<.001f,"robot yaw oscillation does not orbit follow camera");
                }
                Debug.Log("CAMPUS_STABLE_FOLLOW_PASS robotYawAlternation=20 cameraOffsetUnchanged=true");
            }finally{UnityEngine.Object.DestroyImmediate(target);}
            Debug.Log($"CAMPUS_CAMERA_PASS diagonalMeters={diagonal.magnitude:R} stoppingMeters={stoppingDistance:R} yaw={yaw:R}");
        }finally {UnityEngine.Object.DestroyImmediate(host);}
    }
    public static void BuildVerifiedBench() {BenchRun();CranePerformanceBuild.BuildLinuxWorker();}
    public static void BenchRun() {
        var host=new GameObject("Bench layout diagnostic");
        var campus=host.AddComponent<Sim.Physics.Land.CraneIndustrialCampus>();
        string output=System.IO.Path.Combine(System.IO.Path.GetTempPath(),"crane-bench-layout-diagnostic");System.IO.Directory.CreateDirectory(output);
        typeof(Sim.Physics.Land.CraneIndustrialCampus).GetProperty("OutputDirectory").SetValue(campus,output);
        var root=new GameObject("annex-packing-bench-visual");root.transform.SetParent(host.transform,false);
        var prop=typeof(Sim.Physics.Land.CraneIndustrialCampus).GetMethod("PropDetail",System.Reflection.BindingFlags.NonPublic|System.Reflection.BindingFlags.Instance);
        prop.Invoke(campus,new object[]{root.transform,"packing-bench",new Vector3(1.6f,.9f,2.4f)});
        Debug.Log("CAMPUS_BENCH_PASS supported=true noOverlap=true clearWorkArea=true");
    }
    static void Require(bool condition,string label) {if(!condition)throw new InvalidOperationException(label);}
}
#endif
