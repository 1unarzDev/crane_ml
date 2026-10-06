using System;
using UnityEngine;
using UnityEngine.SceneManagement;
using Sim.Utils.ROS;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

namespace Sim.Physics.Land {
    /// <summary>Opt-in land-only HUD and live ROS evidence. All drawing is collider-free.</summary>
    public sealed class CraneCampusInspection:MonoBehaviour {
        CraneIndustrialCampus campus;Camera camera;CraneCampusRobotModel model;
        Sim.Utils.ReferenceEnvironments.CraneReferenceInspectionController legacyInspection;Transform roof;
        PathMsg pendingPlan;OccupancyGridMsg pendingGrid;LaserScanMsg scan;
        readonly object gate=new();LineRenderer line;Material routeMaterial,occupiedMaterial,inflatedMaterial;
        GameObject scanHost; Mesh scanMesh; GameObject gridHost;bool free,gridVisible=true,scanVisible;string status="waiting for ROS evidence";
        [Serializable] sealed class ActionState {public string state;public double rosTime;}
        string actionState="awaiting action status";double actionStateTime;
        bool materialAudit;int plans,grids,scans;Sim.Utils.ReferenceEnvironments.CraneFreeCameraController freeCamera;bool robotDetail;
        public void Configure(CraneIndustrialCampus owner,Camera spectator) {
            campus=owner;camera=spectator;ROSConnection.GetOrCreateInstance().ShowHud=false;roof=owner.VisualRoot.Find("warehouse-roof-visual");legacyInspection=spectator.GetComponent<Sim.Utils.ReferenceEnvironments.CraneReferenceInspectionController>();if(legacyInspection!=null)legacyInspection.HideHud=true;model=owner.Robot.GetComponent<CraneCampusRobotModel>();
            if(legacyInspection!=null){legacyInspection.ConfigureTrajectory(.01f,false,new Color(.05f,.65f,.7f));legacyInspection.ConfigureStableFollow();}
            freeCamera=spectator.gameObject.AddComponent<Sim.Utils.ReferenceEnvironments.CraneFreeCameraController>();freeCamera.Configure(spectator);
            freeCamera.ActiveChanged+=active=>{free=active;if(active)robotDetail=false;if(legacyInspection!=null)legacyInspection.enabled=!active&&!robotDetail;};
            gridVisible=!Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-hide-costmap");
            robotDetail=Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-robot-detail");if(robotDetail&&legacyInspection!=null)legacyInspection.enabled=false;
            routeMaterial=new Material(Shader.Find("HDRP/Unlit"));routeMaterial.SetColor("_UnlitColor",new Color(.05f,.8f,.9f));
            var route=new GameObject("Received Nav2 plan");route.transform.SetParent(owner.DebugRoot,false);route.layer=31;
            line=route.AddComponent<LineRenderer>();line.sharedMaterial=routeMaterial;line.widthMultiplier=.035f;
            occupiedMaterial=new Material(routeMaterial);occupiedMaterial.SetColor("_UnlitColor",new Color(.75f,.2f,.1f));
            inflatedMaterial=new Material(routeMaterial);inflatedMaterial.SetColor("_UnlitColor",new Color(.6f,.5f,.1f));
            if(!ROSPublisher.TransportSuppressed) {
                gameObject.AddComponent<ROSSubscriber>().Initialize<StringMsg>("/campus/nav2_state",m=>{var value=JsonUtility.FromJson<ActionState>(m.data);lock(gate){actionState=value.state;actionStateTime=value.rosTime;}});
                gameObject.AddComponent<ROSSubscriber>().Initialize<PathMsg>("/plan",m=>{lock(gate){pendingPlan=m;plans++;}});
                gameObject.AddComponent<ROSSubscriber>().Initialize<OccupancyGridMsg>("/local_costmap/costmap",m=>{lock(gate){pendingGrid=m;grids++;}});
                gameObject.AddComponent<ROSSubscriber>().Initialize<LaserScanMsg>("/scan",m=>{lock(gate){scan=m;scans++;}});
            }
        }
        void LateUpdate(){if(!robotDetail||camera==null)return;
            camera.orthographic=false;camera.fieldOfView=55;
            Vector3 target=campus.Robot.position+Vector3.up*.08f;
            Vector3 view=campus.Robot.position-campus.Robot.transform.forward*.72f-campus.Robot.transform.right*.45f+Vector3.up*.5f;
            float blend=1-Mathf.Exp(-12*Time.unscaledDeltaTime);
            camera.transform.position=Vector3.Distance(camera.transform.position,view)>5?view:Vector3.Lerp(camera.transform.position,view,blend);
            camera.transform.rotation=Quaternion.Slerp(camera.transform.rotation,Quaternion.LookRotation(target-camera.transform.position),blend);
        }
        void Update(){if(camera==null)return;
            if(!materialAudit&&Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-material-probe")) {
                var pipeline=UnityEngine.Rendering.RenderPipelineManager.currentPipeline;
                if(pipeline!=null){materialAudit=true;var debug=pipeline.GetType().GetProperty("debugDisplaySettings").GetValue(pipeline);var data=debug.GetType().GetProperty("data").GetValue(debug);var lighting=data.GetType().GetField("lightingDebugSettings").GetValue(data);var material=data.GetType().GetField("materialDebugSettings").GetValue(data);
                    campus.RecordEvent("material-debug-audit","HDRP",$"overrideAlbedo={lighting.GetType().GetField("overrideAlbedo").GetValue(lighting)}; lightingMode={lighting.GetType().GetField("debugLightingMode").GetValue(lighting)}; gbufferDebug={material.GetType().GetProperty("debugViewGBuffer").GetValue(material)}");
                }
            }
            if(roof!=null)roof.gameObject.SetActive(camera.transform.position.y<5.05f);

            PathMsg plan;OccupancyGridMsg grid;lock(gate){plan=pendingPlan;grid=pendingGrid;pendingPlan=null;pendingGrid=null;}
            if(plan!=null){line.positionCount=plan.poses.Length;for(int i=0;i<plan.poses.Length;i++){var p=plan.poses[i].pose.position;float x=-(float)p.y,z=(float)p.x;line.SetPosition(i,new Vector3(x,FloorHeight(x,z)+.06f,z));}status="live Nav2 plan received";}
            if(grid!=null&&gridVisible)DrawGrid(grid);
            DrawScan();
        }
        static float FloorHeight(float x,float z) {
            float elevation=0;
            foreach(var hit in UnityEngine.Physics.RaycastAll(new Vector3(x,3,z),Vector3.down,4)) {
                var identity=hit.collider.GetComponent<Sim.Utils.ReferenceEnvironments.CraneSemanticIdentity>();
                if(identity!=null&&(identity.SemanticRole=="floor"||identity.SemanticRole=="ramp"||identity.SemanticRole=="threshold"))elevation=Mathf.Max(elevation,hit.point.y);
            }
            return elevation;
        }
        void DrawGrid(OccupancyGridMsg grid){if(gridHost!=null){Destroy(gridHost.GetComponent<MeshFilter>().sharedMesh);Destroy(gridHost);}gridHost=new GameObject("Received costmap tiles");gridHost.transform.SetParent(campus.DebugRoot,false);gridHost.layer=31;
            var verts=new System.Collections.Generic.List<Vector3>();var a=new System.Collections.Generic.List<int>();var b=new System.Collections.Generic.List<int>();
            int width=(int)grid.info.width;float resolution=grid.info.resolution;
            for(int i=0;i<grid.data.Length;i++){int cost=grid.data[i];if(cost<=0)continue;float x=(float)grid.info.origin.position.x+(i%width+.5f)*resolution;float y=(float)grid.info.origin.position.y+(i/width+.5f)*resolution;
                // Incoming ROS cell orientation is retained, including a rotated grid origin.
                var q=grid.info.origin.orientation;var rotation=new Quaternion((float)q.x,(float)q.y,(float)q.z,(float)q.w);
                Vector3 local=rotation*new Vector3((i%width+.5f)*resolution,(i/width+.5f)*resolution,0);
                x=(float)grid.info.origin.position.x+local.x;y=(float)grid.info.origin.position.y+local.y;
                float elevation=FloorHeight(-y,x)+.045f;
                Vector3 p=new(-y,elevation,x);float h=resolution*.45f;int n=verts.Count;
                verts.Add(p+new Vector3(-h,0,-h));verts.Add(p+new Vector3(-h,0,h));verts.Add(p+new Vector3(h,0,h));verts.Add(p+new Vector3(h,0,-h));
                var indices=cost>=99?a:b;indices.AddRange(new[]{n,n+1,n+2,n,n+2,n+3});
            }
            var mesh=new Mesh();mesh.indexFormat=UnityEngine.Rendering.IndexFormat.UInt32;mesh.SetVertices(verts);mesh.subMeshCount=2;mesh.SetTriangles(a,0);mesh.SetTriangles(b,1);mesh.RecalculateBounds();
            gridHost.AddComponent<MeshFilter>().sharedMesh=mesh;gridHost.AddComponent<MeshRenderer>().sharedMaterials=new[]{occupiedMaterial,inflatedMaterial};
        }
        void OnGUI(){if(campus==null)return;var e=Event.current;
            if(e.type==EventType.KeyDown){
                if(e.keyCode==KeyCode.Alpha4){freeCamera.SetActive(false);robotDetail=!robotDetail;camera.orthographic=false;free=false;if(legacyInspection!=null)legacyInspection.enabled=!robotDetail;}
                if(e.keyCode==KeyCode.Alpha2||e.keyCode==KeyCode.Alpha3){freeCamera.SetActive(false);robotDetail=false;if(legacyInspection!=null){legacyInspection.enabled=true;legacyInspection.SelectView(e.keyCode==KeyCode.Alpha2?"Oblique":"Follow");}}
                if(e.keyCode==KeyCode.Alpha1){freeCamera.SetActive(false);robotDetail=false;if(legacyInspection!=null){legacyInspection.enabled=true;legacyInspection.SelectView("Overview");}camera.orthographic=true;camera.orthographicSize=36;camera.transform.position=new Vector3(12,80,31);camera.transform.rotation=Quaternion.Euler(90,0,0);}
                if(e.keyCode==KeyCode.F){robotDetail=false;freeCamera.SetActive(!freeCamera.Active);}
                if(e.keyCode==KeyCode.M){gridVisible=!gridVisible;if(gridHost!=null)gridHost.SetActive(gridVisible);}
                if(e.keyCode==KeyCode.L)scanVisible=!scanVisible;
                if(e.keyCode==KeyCode.R){campus.RecordEvent("operator-reset",campus.Scenario.id,"scene reload; active ROS task must be restarted by operator");SceneManager.LoadScene(SceneManager.GetActiveScene().name);}
            }
            var style=new GUIStyle(GUI.skin.label){fontSize=16,normal={textColor=new Color(.88f,.93f,.95f)}};
            Color old=GUI.color;GUI.color=new Color(.05f,.08f,.1f,.9f);GUI.DrawTexture(new Rect(18,18,600,135),Texture2D.whiteTexture);GUI.color=old;
            GUI.Label(new Rect(32,31,570,25),"INDUSTRIAL LOGISTICS CAMPUS  |  seed "+campus.Seed,style);
            GUI.Label(new Rect(32,58,570,25),campus.Scenario.id+"  |  Nav2 "+actionState,style);
            GUI.Label(new Rect(32,83,570,25),$"Plan {plans} · grid {grids} · scan {scans} · surface {model?.LeftSurface??"legacy"}",style);
            var controlsStyle=new GUIStyle(style){fontSize=13};
            GUI.Label(new Rect(32,109,570,25),"1/2/3 views · 4 robot · RMB/F free · WASD/QE + Shift · arrows look · M/L overlays",controlsStyle);

        }
        void DrawScan(){
            if(scanHost==null){scanHost=new GameObject("Received scan rays");scanHost.transform.SetParent(campus.DebugRoot,false);scanHost.layer=31;scanMesh=new Mesh();scanHost.AddComponent<MeshFilter>().sharedMesh=scanMesh;scanHost.AddComponent<MeshRenderer>().sharedMaterial=routeMaterial;}
            scanHost.SetActive(scanVisible);if(!scanVisible)return;
            LaserScanMsg acquired;lock(gate){acquired=scan;}if(acquired==null)return;
            var origin=campus.Robot.transform.Find("base_scan");if(origin==null)return;
            var vertices=new System.Collections.Generic.List<Vector3>();var indices=new System.Collections.Generic.List<int>();
            for(int i=0;i<acquired.ranges.Length;i+=4){float r=acquired.ranges[i];if(float.IsNaN(r)||float.IsInfinity(r))continue;float angle=acquired.angle_min+i*acquired.angle_increment;int n=vertices.Count;vertices.Add(origin.position);vertices.Add(origin.position+origin.rotation*new Vector3(-Mathf.Sin(angle),0,Mathf.Cos(angle))*r);indices.Add(n);indices.Add(n+1);}
            scanMesh.Clear();scanMesh.SetVertices(vertices);scanMesh.SetIndices(indices,MeshTopology.Lines,0);scanMesh.RecalculateBounds();
        }
        void OnDestroy(){if(freeCamera!=null)Destroy(freeCamera);if(scanMesh!=null)Destroy(scanMesh);if(gridHost!=null){var f=gridHost.GetComponent<MeshFilter>();if(f!=null)Destroy(f.sharedMesh);}if(routeMaterial!=null)Destroy(routeMaterial);if(occupiedMaterial!=null)Destroy(occupiedMaterial);if(inflatedMaterial!=null)Destroy(inflatedMaterial);}
    }
}
