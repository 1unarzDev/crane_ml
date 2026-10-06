#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEditor.SceneManagement;
using UnityEngine;
using Sim.Physics.Land;

// Isolated reproduction using the production motor model, chassis and wheel joints.
public static class CraneCampusWheelDiagnostic {
    public static void Run() {
        float previousStep=Time.fixedDeltaTime;
        try {
        UnityEngine.Physics.simulationMode=SimulationMode.Script;
        float step=float.Parse(CraneIndustrialCampus.Arg(Environment.GetCommandLineArgs(),"--crane-wheel-diagnostic-step","0.01"),System.Globalization.CultureInfo.InvariantCulture);
        Time.fixedDeltaTime=step;
        foreach(float angularLimit in new[]{12f,100f,1000f}) {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
            var campus=new GameObject("campus").AddComponent<CraneIndustrialCampus>();
            string output=Path.GetFullPath("PerformanceResults/wheel-diagnostic");Directory.CreateDirectory(output);
            typeof(CraneIndustrialCampus).GetProperty("OutputDirectory").SetValue(campus,output);
            var surface=new CampusSurfaceProfile{id="normal-concrete",staticFriction=.85f,dynamicFriction=.7225f,rollingResistance=.025f};
            var profiles=(Dictionary<string,CampusSurfaceProfile>)typeof(CraneIndustrialCampus).GetField("surfaces",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(campus);
            profiles.Add(surface.id,surface);
            var floor=new GameObject("floor");floor.transform.position=new Vector3(0,-.1f,0);
            var fc=floor.AddComponent<BoxCollider>();fc.size=new Vector3(20,.2f,20);fc.contactOffset=.001f;
            fc.sharedMaterial=new PhysicsMaterial{staticFriction=.85f,dynamicFriction=.7225f,frictionCombine=PhysicsMaterialCombine.Minimum};
            floor.AddComponent<CraneCampusSurface>().Profile=surface;
            var robot=new GameObject("robot");robot.transform.position=new Vector3(0,.01f,0);
            var rb=robot.AddComponent<Rigidbody>();rb.linearDamping=.05f;rb.angularDamping=.15f;
            var box=robot.AddComponent<BoxCollider>();box.center=new Vector3(0,.047f,-.064f);box.size=new Vector3(.266f,.094f,.266f);
            box.sharedMaterial=new PhysicsMaterial{staticFriction=0,dynamicFriction=0,frictionCombine=PhysicsMaterialCombine.Minimum};
            var drive=robot.AddComponent<DifferentialDriveDynamics>();
            var model=robot.AddComponent<CraneCampusRobotModel>();
            var modelArgs=new List<string>{"--crane-campus-contact-wheels"};
            foreach(string flag in new[]{"--crane-campus-joint-motors","--crane-campus-convex-tires"})if(Array.Exists(Environment.GetCommandLineArgs(),x=>x==flag))modelArgs.Add(flag);
            model.Configure(campus,drive,modelArgs.ToArray());
            var actors=robot.GetComponentsInChildren<Rigidbody>();
            foreach(var actor in actors)if(actor!=rb){actor.maxAngularVelocity=angularLimit;
                if(Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-wheel-diagnostic-discrete"))actor.collisionDetectionMode=CollisionDetectionMode.Discrete;
                if(Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-wheel-diagnostic-unparent"))actor.transform.SetParent(null,true);
            }
            string velocityIterations=CraneIndustrialCampus.Arg(Environment.GetCommandLineArgs(),"--crane-wheel-diagnostic-velocity-iterations","");
            if(velocityIterations!="")foreach(var actor in actors)actor.solverVelocityIterations=int.Parse(velocityIterations);
            if(Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-wheel-diagnostic-mirror")) {
                var right=robot.transform.Find("campus-right-wheel-contact");right.localRotation=Quaternion.Euler(0,180,0);right.GetComponent<HingeJoint>().axis=Vector3.left;
            }
            UnityEngine.Physics.SyncTransforms();
            float steadySpeed=0;int steadySamples=0;
            for(int i=0;i<Mathf.RoundToInt(8/step);i++){model.Step(i<Mathf.RoundToInt(2/step)?0:.2f,0);UnityEngine.Physics.Simulate(step);if(i>=Mathf.RoundToInt(7/step)){steadySpeed+=rb.linearVelocity.z;steadySamples++;}}
            steadySpeed/=steadySamples;
            Debug.Log($"WHEEL_DIAGNOSTIC limit={angularLimit} speed={rb.linearVelocity.z} meanSpeed={steadySpeed} torque={model.LeftTorque} omega={model.LeftOmega} position={rb.position} lateral={rb.position.x:R} yaw={rb.rotation.eulerAngles.y} rightOmega={model.RightOmega} rightTorque={model.RightTorque}");
            foreach(var actor in actors)if(actor!=rb)Debug.Log($"WHEEL_DIAGNOSTIC inertia={actor.inertiaTensor} inertiaRotation={actor.inertiaTensorRotation} ccd={actor.collisionDetectionMode}");
            if(angularLimit==1000) {
                if(Math.Abs(steadySpeed-.2f)>.01f)throw new Exception("Production wheel velocity regression");
                if(Math.Abs(rb.position.x)>.01f)throw new Exception("Uncommanded straight-drive lateral drift");
                float meanYaw=0;int yawSamples=0;
                for(int i=0;i<Mathf.RoundToInt(3/step);i++){model.Step(0,.4f);UnityEngine.Physics.Simulate(step);if(i>=Mathf.RoundToInt(2/step)){meanYaw+=rb.angularVelocity.y;yawSamples++;}}
                meanYaw/=yawSamples;Debug.Log($"WHEEL_DIAGNOSTIC yawCommand=0.4 yawResponse={rb.angularVelocity.y} meanYaw={meanYaw}");
                if(Math.Abs(meanYaw-.4f)>.02f)throw new Exception("Production wheel yaw sign/response regression");
                Debug.Log($"WHEEL_DIAGNOSTIC yawCommand=0.4 yawResponse={rb.angularVelocity.y}");
            }
        }
        } finally {
            UnityEngine.Physics.simulationMode=SimulationMode.FixedUpdate;
            Time.fixedDeltaTime=previousStep;
        }
    }
    public static void RestoreProjectTiming(){Time.fixedDeltaTime=.02f;UnityEngine.Physics.simulationMode=SimulationMode.FixedUpdate;}
}
#endif
