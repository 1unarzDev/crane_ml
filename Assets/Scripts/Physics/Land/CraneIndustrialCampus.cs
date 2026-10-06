using System;
using System.Collections.Generic;
using System.IO;
using System.Globalization;
using Sim.Utils.ReferenceEnvironments;
using UnityEngine;

namespace Sim.Physics.Land {
    [Serializable] public sealed class CampusBox {
        public string id, kind, surface,sensorResponse;
        public float[] center, size, velocity;
        public float yaw,pitch, activate, remove;
    }
    [Serializable] public sealed class CampusSurfaceProfile {
        public string id, calibrationStatus,frictionCombine;
        public float staticFriction, dynamicFriction, rollingResistance, tractionMultiplier,
            lateralTraction, roughnessAcceleration;
    }
    [Serializable] public sealed class CampusScenario {
        public string id, environment, robot, routeName, expectedChallenge, expectedOutcome,
            surfaceConfiguration, sensorProfile;
        public int seed;
        public float[] start, goal;
        public float startYaw, goalYaw, approximateLengthMeters;
        public bool alternateRouteAvailable;
        public CampusBox[] obstacles;
        public string[] evidenceAnnotations;
    }
    [Serializable] public sealed class CampusManifest {
        public string schema, environmentId, generatorVersion;
        public CampusBox[] boxes;
        public CampusSurfaceProfile[] surfaceProfiles;
        public CampusScenario[] scenarios;
    }
    /// <summary>Extends the existing generated-land/semantic/inspection architecture.
    /// Opt-in runtime construction never imports project settings or changes aquatic scenes.</summary>
    public sealed class CraneIndustrialCampus : MonoBehaviour {
        public const string Resource = "ReferenceEnvironments/industrial_logistics_campus_v1";
        public CampusManifest Manifest { get; private set; }
        public CampusScenario Scenario { get; private set; }
        public Rigidbody Robot { get; private set; }
        public int Seed { get; private set; }
        public string OutputDirectory { get; private set; }
        public Transform VisualRoot, PhysicsRoot, SemanticRoot, ScenarioRoot, LightingRoot, DebugRoot;
        readonly Dictionary<string,Material> materials = new();
        readonly List<UnityEngine.Object> ownedResources=new();
        readonly Dictionary<Light,bool> priorDirectionalLights=new();
        readonly Dictionary<string,CampusSurfaceProfile> surfaces = new();
        readonly List<CraneCampusObstacle> obstacles = new();
        Mesh coneMesh,cylinderMesh;
        System.Random random;
        bool visuals;
        double started;
        float previousFixedStep,previousMaximumCatchup;
        Component diagnosticLidar;System.Reflection.MethodInfo lidarDiagnostics;double nextSensorEvidence;

        public static CraneIndustrialCampus Build(DifferentialDriveDynamics drive, string[] args) {
            var old = UnityEngine.Object.FindAnyObjectByType<CraneReferenceWarehouse>();
            if (old != null) old.gameObject.SetActive(false);
            var host = new GameObject("Industrial Logistics Campus");
            var campus = host.AddComponent<CraneIndustrialCampus>();
            campus.Initialize(drive,args);
            return campus;
        }
        public static string Arg(string[] args, string key, string fallback) {
            int i = Array.IndexOf(args,key);
            return i >= 0 && i+1 < args.Length ? args[i+1] : fallback;
        }
        void Initialize(DifferentialDriveDynamics drive,string[] args) {
            if(UnityEngine.Physics.simulationMode!=SimulationMode.FixedUpdate)
                throw new InvalidOperationException("Campus requires automatic FixedUpdate physics stepping; restore the project simulation mode before building.");
            var construction=System.Diagnostics.Stopwatch.StartNew();
            var asset = Resources.Load<TextAsset>(Resource);
            if (asset == null) throw new FileNotFoundException(Resource);
            Manifest = JsonUtility.FromJson<CampusManifest>(Array.IndexOf(args,"--crane-campus-manifest")>=0?File.ReadAllText(Arg(args,"--crane-campus-manifest","")):asset.text);
            if (Manifest.schema != "crane-industrial-campus-v1") throw new InvalidDataException("Campus schema");
            string id = Arg(args,"--crane-campus-scenario","warehouse_nominal");
            Scenario = Array.Find(Manifest.scenarios,s=>s.id == id) ?? throw new ArgumentException(id);
            Seed = int.Parse(Arg(args,"--crane-campus-seed",Scenario.seed.ToString()),CultureInfo.InvariantCulture);
            Scenario.seed = Seed;
            previousFixedStep=Time.fixedDeltaTime;
            float fixedStep=float.Parse(Arg(args,"--crane-campus-fixed-step","0.01"),CultureInfo.InvariantCulture);
            if(fixedStep<.005f||fixedStep>.04f)throw new ArgumentOutOfRangeException("campus fixed step");
            Time.fixedDeltaTime=fixedStep;
            previousMaximumCatchup=Time.maximumDeltaTime;
            // Keep a stalled graphical frame from advancing beyond the ROS command
            // freshness window before the next transport update. Fixed physics dt is
            // unchanged; wall throughput falls instead of skipping simulation steps.
            Time.maximumDeltaTime=Mathf.Max(fixedStep,Mathf.Min(previousMaximumCatchup,.1f));
            OutputDirectory = Arg(args,"--crane-campus-output","PerformanceResults/campus");
            Directory.CreateDirectory(OutputDirectory);
            random = new System.Random(Seed);
            if(Array.Exists(args,x=>x=="--crane-campus-randomize")) {
                var physicsRandom=new System.Random(Seed+23);
                foreach(var profile in Manifest.surfaceProfiles) {
                    float scale=1+(float)(physicsRandom.NextDouble()*.1-.05);
                    profile.staticFriction*=scale;profile.dynamicFriction*=scale;
                }
            }
            visuals = !Array.Exists(args,x=>x=="-nographics");
            foreach(string root in new[]{"VisualRoot","PhysicsRoot","SemanticRoot","ScenarioRoot","LightingRoot","DebugRoot"}) {
                var t=new GameObject(root).transform;t.SetParent(transform,false);
                switch(root) {case "VisualRoot":VisualRoot=t;break;case "PhysicsRoot":PhysicsRoot=t;break;
                    case "SemanticRoot":SemanticRoot=t;break;case "ScenarioRoot":ScenarioRoot=t;break;
                    case "LightingRoot":LightingRoot=t;break;default:DebugRoot=t;break;}
            }
            float thresholdHeight=float.Parse(Arg(args,"--crane-campus-threshold-height","-1"),CultureInfo.InvariantCulture);
            if(thresholdHeight!=-1) {
                if(!float.IsFinite(thresholdHeight)||thresholdHeight<0||thresholdHeight>.02f)throw new ArgumentOutOfRangeException("threshold height");
                var lip=Array.Find(Manifest.boxes,b=>b.id=="threshold");lip.size[1]=Mathf.Max(.0001f,thresholdHeight);lip.center[1]=lip.size[1]/2;
            }
            foreach(var profile in Manifest.surfaceProfiles) surfaces.Add(profile.id,profile);
            foreach(var box in Manifest.boxes) CreateBox(box,false);
            foreach(var box in Scenario.obstacles) {
                var obstacle=CreateBox(box,true).AddComponent<CraneCampusObstacle>();
                obstacle.Initialize(this,box);obstacles.Add(obstacle);
            }
            string calibration=Arg(args,"--crane-campus-calibration","");
            Robot=drive.GetComponent<Rigidbody>();
            Robot.position=calibration=="CAL-05"?new Vector3(24,.01f,6.4f):calibration=="CAL-06"?new Vector3(0,.01f,28.5f):new Vector3(Scenario.start[0],.01f,Scenario.start[1]);
            Robot.rotation=Quaternion.Euler(0,Scenario.startYaw,0);
            // Rigidbody pose assignment can precede transform publication in manual stepping.
            // Joint actors must be constructed from the same world pose as their chassis.
            Robot.transform.SetPositionAndRotation(Robot.position,Robot.rotation);
            UnityEngine.Physics.SyncTransforms();
            Robot.linearVelocity=Vector3.zero;Robot.angularVelocity=Vector3.zero;
            drive.SetCommand(0,0);
            // Rigidbody/planar architecture is preserved. Finite response is opt-in only here.
            if (Array.Exists(args,x=>x=="--crane-campus-model")) {
                Robot.gameObject.AddComponent<CraneCampusRobotModel>().Configure(this,drive,args);
            }
            var sensorType=Type.GetType("Sim.Sensors.Nav.CraneCampusSensorSuite, SensorsAssembly",true);
            var sensor=Robot.gameObject.AddComponent(sensorType);
            sensorType.GetMethod("Configure").Invoke(sensor,new object[]{Seed,Scenario.sensorProfile!="off"});
            var scanner=Robot.transform.Find("base_scan");
            if(scanner!=null) {
                scanner.localPosition=new Vector3(0,.122f,-.064f);
                var lidarType=Type.GetType("Sim.Sensors.Lidar.Lidar2D, SensorsAssembly",true);
                var lidar=scanner.GetComponent(lidarType);
                if(Scenario.sensorProfile!="off"||Array.Exists(args,x=>x.StartsWith("--crane-campus-lidar-"))) {
                    float sigma=float.Parse(Arg(args,"--crane-campus-lidar-sigma",Scenario.sensorProfile=="off"?"0":"0.005"),CultureInfo.InvariantCulture);
                    float dropout=float.Parse(Arg(args,"--crane-campus-lidar-dropout",Scenario.sensorProfile=="off"?"0":"0.01"),CultureInfo.InvariantCulture);
                    int latency=int.Parse(Arg(args,"--crane-campus-lidar-latency-scans","0"),CultureInfo.InvariantCulture);
                    lidarType.GetMethod("ConfigureErrors").Invoke(lidar,new object[]{Seed+313,sigma,dropout,latency,Scenario.sensorProfile});
                    diagnosticLidar=lidar;lidarDiagnostics=lidarType.GetMethod("ErrorDiagnosticsJson");
                    RecordEvent("lidar-profile","base_scan",$"sigmaMeters={sigma:R}; dropout={dropout:R}; latencyScans={latency}; materialClass={Scenario.sensorProfile}");
                }
            }
            var inspection=UnityEngine.Object.FindAnyObjectByType<CraneReferenceInspectionController>();
            if(inspection!=null) inspection.Configure(inspection.GetComponent<Camera>(),Robot.transform,
                Manifest.environmentId,Scenario.id,Scenario.evidenceAnnotations,
                    Scenario.id.StartsWith("yard_")?new Vector3(8,0,51):Scenario.id.StartsWith("proving_")?new Vector3(28,0,22):Scenario.id.StartsWith("dock_")?new Vector3(0,0,35):new Vector3(0,0,15));
            if(!string.IsNullOrEmpty(calibration))Robot.gameObject.AddComponent<CraneCampusCalibration>().Configure(this,drive,calibration);
            if(visuals) { AddPresentation();
                var cam=inspection!=null?inspection.GetComponent<Camera>():Camera.main;
                var dataType=Type.GetType("UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData, Unity.RenderPipelines.HighDefinition.Runtime",true);
                var data=cam.GetComponent(dataType)??cam.gameObject.AddComponent(dataType);dataType.GetField("customRenderingSettings").SetValue(data,false);cam.nearClipPlane=.03f;
                gameObject.AddComponent<CraneCampusInspection>().Configure(this,cam);
            }
            started=Time.fixedTimeAsDouble;
            var telemetry=Robot.gameObject.AddComponent<CraneCampusTelemetry>();telemetry.Configure(this,drive);
            File.WriteAllText(Path.Combine(OutputDirectory,"scenario.json"),JsonUtility.ToJson(Scenario,true));
            File.WriteAllText(Path.Combine(OutputDirectory,"manifest.json"),JsonUtility.ToJson(Manifest,true));
            construction.Stop();WriteRuntimeAudit(construction.Elapsed.TotalMilliseconds);
            Debug.Log($"CRANE_CAMPUS_READY scenario={Scenario.id} seed={Seed} boxes={Manifest.boxes.Length} fixedDt={Time.fixedDeltaTime:R}");
        }
        public double Elapsed => Time.fixedTimeAsDouble-started;
        public CampusSurfaceProfile Surface(string id) => surfaces[id];
        static Vector3 V(float[] p) => new Vector3(p[0],p[1],p[2]);
        GameObject CreateBox(CampusBox box,bool dynamic) {
            var pair=new GameObject(box.id);pair.transform.SetParent(dynamic?ScenarioRoot:PhysicsRoot,false);
            pair.transform.position=V(box.center);pair.transform.rotation=Quaternion.Euler(0,box.yaw,0);
            Collider collider;
            if(box.kind=="cone") {
                if(coneMesh==null){coneMesh=MakeCone();ownedResources.Add(coneMesh);}
                var shape=pair.AddComponent<MeshCollider>();shape.sharedMesh=coneMesh;shape.convex=true;pair.transform.localScale=V(box.size);collider=shape;
            } else if(box.kind=="pipe") {
                if(cylinderMesh==null){var prototype=GameObject.CreatePrimitive(PrimitiveType.Cylinder);prototype.GetComponent<Collider>().enabled=false;cylinderMesh=prototype.GetComponent<MeshFilter>().sharedMesh;Destroy(prototype);}
                collider=null;
                for(int i=0;i<3;i++) {
                    var tube=new GameObject("tube-"+i);tube.transform.SetParent(pair.transform,false);
                    tube.transform.localPosition=new Vector3((i-1)*box.size[0]/3,0,0);tube.transform.localRotation=Quaternion.Euler(90,0,0);
                    tube.transform.localScale=new Vector3(box.size[1],box.size[2]/2,box.size[1]);
                    var shape=tube.AddComponent<MeshCollider>();shape.sharedMesh=cylinderMesh;shape.convex=true;collider=shape;
                }
            } else {var shape=pair.AddComponent<BoxCollider>();shape.size=V(box.size);collider=shape;}
            // Ramps are authoritative oriented slabs, not merely tilted render meshes.
            if(box.kind=="ramp") pair.transform.rotation=Quaternion.Euler(box.pitch,box.yaw,0);
            var profile=surfaces[box.surface];
            var contactMaterial=new PhysicsMaterial(box.surface){staticFriction=profile.staticFriction,
                dynamicFriction=profile.dynamicFriction,frictionCombine=Enum.Parse<PhysicsMaterialCombine>(profile.frictionCombine)};
            ownedResources.Add(contactMaterial);foreach(var shape in pair.GetComponentsInChildren<Collider>()){shape.sharedMaterial=contactMaterial;shape.contactOffset=.001f;}
            pair.AddComponent<CraneSemanticIdentity>().Configure(box.id,box.kind,Manifest.environmentId);
            pair.AddComponent<CraneCampusSurface>().Profile=profile;
            if(!string.IsNullOrEmpty(box.sensorResponse)||box.kind=="container"||box.kind=="rack"||box.kind=="column") {
                var responseType=Type.GetType("Sim.Sensors.Lidar.CraneLidarResponse, SensorsAssembly",true);
                var response=pair.AddComponent(responseType);
                responseType.GetField("Class").SetValue(response,!string.IsNullOrEmpty(box.sensorResponse)?box.sensorResponse:box.kind=="rack"?"dark":"reflective");
            }
            var semantic=new GameObject(box.id);semantic.transform.SetParent(SemanticRoot,false);
            semantic.transform.position=pair.transform.position;
            semantic.AddComponent<CraneCampusSemanticRecord>().Configure(box.id,box.kind,box.surface,pair.transform);
            if(visuals) {
                var visual=new GameObject(box.id+"-visual");visual.transform.SetParent(VisualRoot,false);
                visual.transform.SetPositionAndRotation(pair.transform.position,pair.transform.rotation);
                visual.AddComponent<CraneSemanticIdentity>().Configure(box.id,box.kind,Manifest.environmentId);
                if(box.kind!="rack"&&box.kind!="cone"&&box.kind!="pipe"&&box.kind!="packing-bench"&&box.kind!="tote-cart"&&box.kind!="pallet"&&box.kind!="machine")Primitive(visual.transform,"body",V(box.size),Vector3.zero,Material(box.sensorResponse=="dark"?"dark-panel":box.sensorResponse=="reflective"?"reflective-panel":box.kind,box.surface));
                if(box.kind=="pallet")PalletDetail(visual.transform,V(box.size));
                if(box.kind=="packing-bench"||box.kind=="tote-cart"||box.kind=="machine")PropDetail(visual.transform,box.kind,V(box.size));
                if(box.kind=="cone") {
                    var shape=new GameObject("traffic-cone");shape.transform.SetParent(visual.transform,false);shape.transform.localScale=V(box.size);
                    shape.AddComponent<MeshFilter>().sharedMesh=coneMesh;shape.AddComponent<MeshRenderer>().sharedMaterial=Material("cone",box.surface);
                }
                if(box.kind=="pipe")for(int i=0;i<3;i++) {
                    var tube=new GameObject("stored-pipe-"+i);tube.transform.SetParent(visual.transform,false);tube.transform.localPosition=new Vector3((i-1)*box.size[0]/3,0,0);
                    tube.transform.localRotation=Quaternion.Euler(90,0,0);tube.transform.localScale=new Vector3(box.size[1],box.size[2]/2,box.size[1]);
                    tube.AddComponent<MeshFilter>().sharedMesh=cylinderMesh;tube.AddComponent<MeshRenderer>().sharedMaterial=Material("pipe",box.surface);
                }
                if(box.kind=="rack") RackDetail(visual.transform,V(box.size));
                if(box.kind=="container") ContainerDetail(visual.transform,V(box.size));
                if(dynamic) {var sync=pair.AddComponent<CraneCampusVisualSync>();sync.Target=visual.transform;}
            }
            return pair;
        }
        Material Material(string kind,string surface) {
            string key=kind+":"+surface;if(materials.TryGetValue(key,out var m))return m;
            var template=Resources.Load<Material>("CampusMaterials/Profiles/"+surface);
            m=template!=null?new Material(template):new Material(Shader.Find("HDRP/Lit"));ownedResources.Add(m);m.enableInstancing=true;
            m.SetTexture("_BaseColorMap",null);m.SetTexture("_NormalMap",null);
            Color color=kind switch {"floor"=>surface=="asphalt"?new Color(.12f,.13f,.14f):surface=="compacted-dirt"?new Color(.28f,.23f,.17f):surface=="rubber-mat"?new Color(.025f,.03f,.035f):surface=="painted-epoxy"?new Color(.2f,.3f,.32f):new Color(.46f,.46f,.43f),
                "cone"=>new Color(.72f,.23f,.055f),"dark-panel"=>new Color(.025f,.028f,.03f),"reflective-panel"=>new Color(.55f,.57f,.6f),"rack"=>new Color(.34f,.29f,.23f),"container"=>new Color(.22f,.28f,.29f),"pallet"=>new Color(.4f,.3f,.19f),
                "bollard"=>new Color(.63f,.46f,.10f),"pipe"=>new Color(.22f,.24f,.25f),_=>new Color(.48f,.48f,.45f)};
            float variation=(float)(random.NextDouble()*.06-.03);m.SetColor("_BaseColor",color*(1+variation));
            m.SetFloat("_Smoothness",kind=="reflective-panel"?.8f:surface=="polished-concrete"?.38f:surface=="metal-plate"?.5f:.18f);
            m.SetFloat("_Metallic",kind=="reflective-panel"||surface=="metal-plate"?1:kind=="container"||kind=="pipe"||kind=="bollard"?.65f:0);
            if((kind=="floor"&&surface!="metal-plate"&&surface!="rubber-mat"&&surface!="painted-epoxy")||kind=="container"||kind=="wall"||kind=="column"||kind=="utility") {
                string texture=kind=="container"?"rusty_corrugated_iron":kind!="floor"?"grey_plaster_02":surface=="asphalt"?"asphalt_02":surface=="compacted-dirt"?"gravel":"concrete_floor_02";
                var tex=Resources.Load<Texture2D>("CampusMaterials/"+texture+"_diff_1k");
                var normal=Resources.Load<Texture2D>("CampusMaterials/"+texture+"_nor_gl_1k");
                if(normal!=null){m.SetTexture("_NormalMap",normal);m.EnableKeyword("_NORMALMAP_TANGENT_SPACE");m.SetFloat("_NormalScale",.45f);}
                if(tex!=null){m.SetTexture("_BaseColorMap",tex);m.SetTextureScale("_BaseColorMap",Vector2.one);}
            }
            if(kind=="floor"&&(surface=="normal-concrete"||surface=="polished-concrete"||surface=="rough-concrete")) {
                var concrete=new Texture2D(256,256,TextureFormat.RGBA32,true){name="Seeded neutral concrete albedo",wrapMode=TextureWrapMode.Repeat};
                var pixels=new Color[256*256];float offset=Seed%997;
                for(int y=0;y<256;y++)for(int x=0;x<256;x++) {
                    float mottling=Mathf.PerlinNoise(x/36f+offset,y/36f+offset);
                    float fine=Mathf.PerlinNoise(x/2.8f+offset,y/2.8f+offset);
                    float shade=.72f+(mottling-.5f)*.065f+(fine-.5f)*.025f;
                    pixels[y*256+x]=new Color(shade,shade,shade);
                }
                concrete.SetPixels(pixels);concrete.Apply(true,false);ownedResources.Add(concrete);m.SetTexture("_BaseColorMap",concrete);
                m.SetColor("_BaseColor",new Color(.62f,.63f,.64f));m.SetFloat("_NormalScale",surface=="rough-concrete"?.12f:.055f);
            }
            ValidateVisualMaterial(m);materials.Add(key,m);return m;
        }
        public static void ValidateVisualMaterial(Material material) {
            var hdMaterial=Type.GetType("UnityEngine.Rendering.HighDefinition.HDMaterial, Unity.RenderPipelines.HighDefinition.Runtime",true);
            if(!(bool)hdMaterial.GetMethod("ValidateMaterial").Invoke(null,new object[]{material}))
                throw new InvalidOperationException("Unrecognized campus HDRP material: "+material.shader.name);
        }
        static Mesh MakeCone() {
            const int sides=12;var vertices=new List<Vector3>();var triangles=new List<int>();var uv=new List<Vector2>();
            for(int ring=0;ring<2;ring++)for(int i=0;i<sides;i++) {
                float angle=i*2*Mathf.PI/sides,radius=ring==0?.5f:.005f;vertices.Add(new Vector3(Mathf.Cos(angle)*radius,ring-.5f,Mathf.Sin(angle)*radius));uv.Add(new Vector2((float)i/sides,ring));
            }
            vertices.Add(new Vector3(0,-.5f,0));vertices.Add(new Vector3(0,.5f,0));uv.Add(Vector2.one*.5f);uv.Add(Vector2.one*.5f);
            for(int i=0;i<sides;i++){int j=(i+1)%sides;triangles.AddRange(new[]{i,sides+j,j,i,sides+i,sides+j,2*sides,i,j,2*sides+1,sides+j,sides+i});}
            var mesh=new Mesh{name="Campus traffic cone"};mesh.SetVertices(vertices);mesh.SetUVs(0,uv);mesh.SetTriangles(triangles,0);mesh.RecalculateNormals();mesh.RecalculateBounds();return mesh;
        }
        GameObject Primitive(Transform root,string name,Vector3 size,Vector3 pos,Material mat) {
            var g=GameObject.CreatePrimitive(PrimitiveType.Cube);g.name=name;g.transform.SetParent(root,false);
            g.transform.localPosition=pos;g.transform.localScale=size;
            var c=g.GetComponent<Collider>();c.enabled=false;Destroy(c);g.GetComponent<Renderer>().sharedMaterial=mat;
            if(mat.HasProperty("_BaseColorMap")&&mat.GetTexture("_BaseColorMap")!=null) {
                var filter=g.GetComponent<MeshFilter>();var mesh=Instantiate(filter.sharedMesh);ownedResources.Add(mesh);var uv=mesh.uv;var normals=mesh.normals;
                for(int i=0;i<uv.Length;i++) {
                    Vector3 n=normals[i];Vector2 dimensions=Mathf.Abs(n.y)>.5f?new Vector2(size.x,size.z):Mathf.Abs(n.x)>.5f?new Vector2(size.z,size.y):new Vector2(size.x,size.y);
                    uv[i]=Vector2.Scale(uv[i],dimensions*.5f); // common 2 m texture period on every face
                }
                mesh.uv=uv;mesh.RecalculateUVDistributionMetrics();filter.sharedMesh=mesh;
            }
            return g;
        }
        Material InventoryLabel() {
            const string key="inventory-label";
            if(materials.TryGetValue(key,out var existing))return existing;
            var m=PlainRobotMaterial();ownedResources.Add(m);m.SetColor("_BaseColor",new Color(.78f,.76f,.67f));m.SetFloat("_Smoothness",.12f);
            ValidateVisualMaterial(m);materials.Add(key,m);return m;
        }
        Material InventoryFinish(bool bin,int variant) {
            string key="inventory-"+bin+"-"+variant;
            if(materials.TryGetValue(key,out var existing))return existing;
            var m=PlainRobotMaterial();ownedResources.Add(m);m.enableInstancing=true;
            Color[] colors=bin?new[]{new Color(.12f,.23f,.28f),new Color(.24f,.29f,.25f),new Color(.28f,.29f,.31f),new Color(.19f,.2f,.22f)}:
                new[]{new Color(.53f,.39f,.25f),new Color(.63f,.49f,.32f),new Color(.46f,.34f,.23f),new Color(.64f,.55f,.38f)};
            var texture=new Texture2D(128,128,TextureFormat.RGBA32,true){name=key+" subtle surface grain",wrapMode=TextureWrapMode.Repeat};
            var pixels=new Color[128*128];var noise=new System.Random(Seed+variant*97+(bin?1103:2207));
            for(int y=0;y<128;y++)for(int x=0;x<128;x++) {
                float shade=1+(float)(noise.NextDouble()-.5)*(bin?.045f:.12f);
                if(!bin)shade+=(Mathf.PerlinNoise(x*.17f,y*.6f)-.5f)*.06f;
                pixels[y*128+x]=new Color(shade,shade,shade);
            }
            texture.SetPixels(pixels);texture.Apply();ownedResources.Add(texture);
            m.SetTexture("_BaseColorMap",texture);m.SetColor("_BaseColor",colors[variant]);m.SetFloat("_Smoothness",bin?.27f:.08f);
            ValidateVisualMaterial(m);materials.Add(key,m);return m;
        }
        void RackDetail(Transform root,Vector3 size) {
            var steel=Material("pipe","metal-plate");
            // Packed inventory volumes share a conservative solid collision proxy. No rack
            // interior is a traversable route. Render seams do not create contact edges.
            int bays=Mathf.Max(1,Mathf.RoundToInt(size.z/.85f));int levels=4;
            int cartons=0,bins=0,cases=0;
            for(int level=0;level<levels;level++) {
                float shelfY=-size.y/2+.09f+level*(size.y-.09f)/levels;
                // Continuous packing rather than one repeated object in each grid cell.
                float cursor=-size.z/2+.12f+(float)random.NextDouble()*.22f;
                while(cursor<size.z/2-.18f) {
                    float depth=.22f+(float)random.NextDouble()*.43f;
                    if(cursor+depth>size.z/2-.08f)break;
                    float width=size.x*(.52f+(float)random.NextDouble()*.32f);
                    float height=Mathf.Min(size.y/levels-.13f,.16f+(float)random.NextDouble()*.26f);
                    var position=new Vector3(((float)random.NextDouble()-.5f)*(size.x-width)*.65f,
                        shelfY+height/2,cursor+depth/2);
                    double choice=random.NextDouble();
                    if(choice<.10) {
                        string resource=random.NextDouble()<.8?"metal_toolbox":"metal_tool_chest";
                        FittedEquipmentModel(root,resource,position,new Vector3(width,height,depth),EquipmentTexturedFinish(resource),random.NextDouble()<.5);
                        cases++;
                    } else {
                        bool bin=choice<.47;
                        var item=new GameObject(bin?"Open spare-parts bin":"Shipping carton");item.transform.SetParent(root,false);item.transform.localPosition=position;
                        var finish=InventoryFinish(bin,random.Next(4));
                        if(bin) {
                            Primitive(item.transform,"Bin base",new Vector3(width,.025f,depth),new Vector3(0,-height/2+.0125f,0),finish);
                            for(int side=-1;side<=1;side+=2) {
                                Primitive(item.transform,"Bin side",new Vector3(.018f,height,depth),new Vector3(side*(width/2-.009f),0,0),finish);
                                Primitive(item.transform,"Bin end",new Vector3(width-.036f,height,.018f),new Vector3(0,0,side*(depth/2-.009f)),finish);
                            }
                            // A visible packed insert makes these read as parts storage.
                            Primitive(item.transform,"Bagged replacement components",new Vector3(width*.7f,height*.45f,depth*.65f),new Vector3(0,-height*.18f,0),InventoryFinish(true,3));bins++;
                        } else {
                            Primitive(item.transform,"Cardboard package",new Vector3(width,height,depth),Vector3.zero,finish);
                            Primitive(item.transform,"Packing tape",new Vector3(width*.16f,.003f,depth),new Vector3(0,height/2+.0015f,0),InventoryFinish(false,3));cartons++;
                        }
                        for(int side=-1;side<=1;side+=2) {
                            var label=Primitive(item.transform,"Inventory label",new Vector3(.002f,height*.27f,depth*.32f),new Vector3(side*(width/2+.001f),height*.08f,0),InventoryLabel());
                            for(int stripe=0;stripe<4;stripe++)Primitive(item.transform,"Label barcode",new Vector3(.003f,height*.12f,.004f),label.transform.localPosition+new Vector3(side*.001f,0,(stripe-1.5f)*.012f),steel);
                        }
                        // Small rotations bounded by clearance, with children moving together.
                        item.transform.localRotation=Quaternion.Euler(0,(float)(random.NextDouble()*8-4),0);
                    }
                    cursor+=depth+.045f+(float)random.NextDouble()*.18f;
                    if(random.NextDouble()<.23)cursor+=.20f+(float)random.NextDouble()*.30f;
                }
            }
            Debug.Log($"campus-inventory seed={Seed} rack={root.name} cartons={cartons} bins={bins} toolCases={cases}");
            for(int i=0;i<=bays;i++)for(int side=-1;side<=1;side+=2)
                Primitive(root,"rack-upright",new Vector3(.06f,size.y,.06f),new Vector3(side*(size.x/2-.03f),0,-size.z/2+.03f+i*(size.z-.06f)/bays),steel);
            for(int i=0;i<=levels;i++)Primitive(root,"shelf-beam",new Vector3(size.x,.09f,size.z),new Vector3(0,-size.y/2+.045f+i*(size.y-.09f)/levels),steel);
        }
        void PalletDetail(Transform root,Vector3 size) {
            var wood=Material("pallet","normal-concrete");
            Primitive(root,"Loaded pallet cargo",new Vector3(size.x-.04f,size.y-.12f,size.z-.04f),new Vector3(0,.06f,0),Material("rack","normal-concrete"));
            for(int i=0;i<5;i++)Primitive(root,"Timber pallet deck",new Vector3(size.x,.055f,size.z*.13f),new Vector3(0,-size.y/2+.055f,-size.z*.4f+i*size.z*.2f),wood);
            for(int i=-1;i<=1;i++)Primitive(root,"Pallet runner",new Vector3(.1f,.07f,size.z),new Vector3(i*size.x*.35f,-size.y/2+.035f,0),wood);
        }
        Material EquipmentFinish(string name,Color color,float metallic=0) {
            string key="equipment-flat:"+name;
            if(materials.TryGetValue(key,out var cached))return cached;
            var finish=PlainRobotMaterial();finish.name=name;finish.enableInstancing=true;ownedResources.Add(finish);materials.Add(key,finish);
            finish.SetColor("_BaseColor",color);finish.SetFloat("_Metallic",metallic);finish.SetFloat("_Smoothness",.28f);ValidateVisualMaterial(finish);return finish;
        }
        GameObject FittedEquipmentModel(Transform root,string resource,Vector3 center,Vector3 size,Material finish,bool quarterTurn=false) {
            var source=Resources.Load<GameObject>("CampusProps/"+resource);
            if(source==null)throw new MissingReferenceException(resource);
            var frame=new GameObject(resource);frame.transform.SetParent(root,false);
            var model=Instantiate(source,frame.transform);model.transform.localPosition=Vector3.zero;
            // FBX import axis conversion belongs to the asset: retain its root rotation.
            if(quarterTurn)model.transform.localRotation=Quaternion.Euler(0,90,0)*model.transform.localRotation;
            // The source wrench is standing on its end; tabletop tools lie flat.
            if(resource=="adjustable_wrench")model.transform.localRotation=Quaternion.Euler(90,0,0)*model.transform.localRotation;
            foreach(var collider in model.GetComponentsInChildren<Collider>()){collider.enabled=false;Destroy(collider);}
            Bounds bounds=new Bounds();bool first=true;
            foreach(var renderer in model.GetComponentsInChildren<Renderer>()) {
                var b=renderer.localBounds;for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2) {
                    Vector3 corner=frame.transform.InverseTransformPoint(renderer.localToWorldMatrix.MultiplyPoint3x4(b.center+Vector3.Scale(b.extents,new Vector3(x,y,z))));
                    if(first){bounds=new Bounds(corner,Vector3.zero);first=false;}else bounds.Encapsulate(corner);
                }
                var assigned=renderer.sharedMaterials;for(int i=0;i<assigned.Length;i++)assigned[i]=finish;renderer.sharedMaterials=assigned;
            }
            if(first||bounds.size.x<=0||bounds.size.y<=0||bounds.size.z<=0)throw new InvalidOperationException("Empty equipment bounds: "+resource);
            float scale=Mathf.Min(size.x/bounds.size.x,size.y/bounds.size.y,size.z/bounds.size.z);
            model.transform.localPosition-=bounds.center;
            frame.transform.localScale=Vector3.one*scale;
            // Uniform fitting must rest on the support plane, not float in the target box.
            frame.transform.localPosition=center+Vector3.up*((bounds.size.y*scale-size.y)/2);
            RecordEvent("equipment-fit",resource,$"source={bounds.size}; target={size}; uniformScale={scale}; supportY={center.y-size.y/2}");
            return frame;
        }
        Material EquipmentTexturedFinish(string resource) {
            string key="equipment-pbr:"+resource;
            if(materials.TryGetValue(key,out var cached))return cached;
            var template=Resources.Load<Material>("CampusProps/Materials/"+resource);
            if(template==null)throw new MissingReferenceException("Prepared equipment material: "+resource);
            var finish=new Material(template);finish.enableInstancing=true;ownedResources.Add(finish);
            var albedo=Resources.Load<Texture2D>("CampusProps/"+resource+"_diff_1k");
            var normal=Resources.Load<Texture2D>("CampusProps/"+resource+"_nor_gl_1k");
            var mask=Resources.Load<Texture2D>("CampusProps/"+resource+"_mask_1k");
            if(albedo!=null)finish.SetTexture("_BaseColorMap",albedo);
            if(normal!=null){finish.SetTexture("_NormalMap",normal);finish.SetFloat("_NormalScale",.55f);finish.EnableKeyword("_NORMALMAP_TANGENT_SPACE");}
            if(mask!=null){finish.SetTexture("_MaskMap",mask);finish.EnableKeyword("_MASKMAP");}
            ValidateVisualMaterial(finish);materials.Add(key,finish);return finish;
        }
        void PropDetail(Transform root,string kind,Vector3 size) {
            var steel=EquipmentFinish("Powder-coated equipment frame",new Color(.12f,.16f,.18f),.35f);
            var worktop=EquipmentFinish("Laminated worktop",new Color(.48f,.36f,.22f));
            var rubber=EquipmentFinish("Caster rubber",new Color(.025f,.03f,.035f));
            var orange=EquipmentFinish("Return tote orange",new Color(.62f,.25f,.08f));
            if(kind=="machine") {
                // Wall-side maintenance station: a floor-standing drill, with service
                // compressor alongside it rather than a distorted tank filling the room.
                FittedEquipmentModel(root,"drill_press_01",new Vector3(-size.x*.20f,-size.y/2+.6f,-size.z*.22f),new Vector3(size.x*.55f,1.2f,size.z*.45f),EquipmentTexturedFinish("drill_press_01"));
                FittedEquipmentModel(root,"old_military_compressor",new Vector3(size.x*.18f,-size.y/2+.45f,size.z*.23f),new Vector3(size.x*.58f,.9f,size.z*.43f),EquipmentTexturedFinish("old_military_compressor"),true);
                return;
            }
            if(kind=="tote-cart"&&root.name.Contains("tool-cart")) {
                FittedEquipmentModel(root,"portable_welding_cart",Vector3.zero,size,EquipmentTexturedFinish("portable_welding_cart"));
                return;
            }
            bool bench=kind=="packing-bench";
            float top=size.y/2-(bench?.14f:.035f);
            if(bench) {
                // A back-to-back drawer desk island keeps supplies at the workstation.
                for(int side=-1;side<=1;side+=2)FittedEquipmentModel(root,"metal_office_desk",new Vector3(side*size.x*.245f,-size.y/2+(size.y-.12f)/2,0),new Vector3(size.x*.49f,size.y-.12f,size.z*.98f),EquipmentTexturedFinish("metal_office_desk"),true);
                // Imported drawer units are storage inserts; a separate continuous
                // frame supports the assembly top even when native model proportions
                // leave clearance above the drawers.
                for(int x=-1;x<=1;x+=2)for(int z=-1;z<=1;z+=2)
                    Primitive(root,"Workbench structural leg",new Vector3(.045f,size.y-.14f,.045f),new Vector3(x*(size.x/2-.055f),-.07f,z*(size.z/2-.055f)),steel);
                for(int side=-1;side<=1;side+=2)
                    Primitive(root,"Workbench longitudinal rail",new Vector3(.045f,.055f,size.z-.08f),new Vector3(side*(size.x/2-.055f),top-.0475f,0),steel);
            }
            Primitive(root,bench?"Assembly work surface":"Mobile cart top tray",new Vector3(size.x,.04f,size.z),new Vector3(0,top,0),bench?worktop:steel);
            if(!bench) {
                for(int x=-1;x<=1;x+=2)for(int z=-1;z<=1;z+=2) {
                    Primitive(root,"Tubular support",new Vector3(.045f,size.y-.14f,.045f),new Vector3(x*(size.x/2-.04f),-.02f,z*(size.z/2-.04f)),steel);
                    var wheel=GameObject.CreatePrimitive(PrimitiveType.Cylinder);wheel.name="Swivel caster";wheel.transform.SetParent(root,false);
                    wheel.transform.localPosition=new Vector3(x*(size.x/2-.06f),-size.y/2+.055f,z*(size.z/2-.06f));
                    wheel.transform.localRotation=Quaternion.Euler(0,0,90);wheel.transform.localScale=new Vector3(.09f,.025f,.09f);
                    var collider=wheel.GetComponent<Collider>();collider.enabled=false;Destroy(collider);wheel.GetComponent<Renderer>().sharedMaterial=rubber;
                }
                Primitive(root,"Lower cart tray",new Vector3(size.x*.9f,.035f,size.z*.88f),new Vector3(0,-size.y/2+.13f,0),steel);
                for(int i=0;i<2;i++) {
                    var position=new Vector3(0,top-.20f,-size.z*.25f+i*size.z*.5f);
                    Primitive(root,"Returns tote",new Vector3(size.x*.68f,.28f,size.z/3),position,orange);
                    Primitive(root,"Tote rim",new Vector3(size.x*.70f,.025f,size.z/3+.02f),position+Vector3.up*.1275f,steel);
                }
                return;
            }
            // Operator stands on the +X aisle side. Keep an uncluttered central
            // electronics mat, storage along the -X back edge, and mechanical work
            // at the near corner rather than scattering tools across the work area.
            float support=top+.02f;
            var matCenter=new Vector3(size.x*.10f,support+.003f,0);
            var matSize=new Vector3(.60f,.006f,.65f);
            Primitive(root,"Clear electronics work area",matSize,matCenter,rubber);
            var tools=new[] {
                FittedEquipmentModel(root,"bench_vice_01",new Vector3(size.x*.34f,support+.09f,-size.z*.35f),new Vector3(.35f,.18f,.34f),EquipmentTexturedFinish("bench_vice_01"),true),
                FittedEquipmentModel(root,"metal_toolbox",new Vector3(-size.x*.28f,support+.09f,-size.z*.32f),new Vector3(.42f,.18f,.28f),EquipmentTexturedFinish("metal_toolbox"),true),
                FittedEquipmentModel(root,"retro_multimeter",new Vector3(-size.x*.19f,support+.065f,size.z*.17f),new Vector3(.20f,.13f,.23f),EquipmentTexturedFinish("retro_multimeter"),true),
                FittedEquipmentModel(root,"adjustable_wrench",new Vector3(size.x*.29f,support+.012f,size.z*.23f),new Vector3(.10f,.024f,.27f),EquipmentTexturedFinish("adjustable_wrench"))
            };
            ValidateBenchLayout(root,size,support,new Bounds(matCenter,matSize),tools);
        }
        void ValidateBenchLayout(Transform root,Vector3 size,float support,Bounds workArea,GameObject[] tools) {
            // Validate the instantiated render bounds after import rotation and uniform
            // fitting, not merely requested target sizes. These remain cosmetic items;
            // authoritative bench collision and the aisle envelope are unchanged.
            var footprints=new Bounds[tools.Length];
            for(int i=0;i<tools.Length;i++) {
                bool first=true;Bounds bounds=new Bounds();
                foreach(var renderer in tools[i].GetComponentsInChildren<Renderer>()) {
                    var b=renderer.localBounds;
                    for(int x=-1;x<=1;x+=2)for(int y=-1;y<=1;y+=2)for(int z=-1;z<=1;z+=2) {
                        var corner=root.InverseTransformPoint(renderer.localToWorldMatrix.MultiplyPoint3x4(b.center+Vector3.Scale(b.extents,new Vector3(x,y,z))));
                        if(first){bounds=new Bounds(corner,Vector3.zero);first=false;}else bounds.Encapsulate(corner);
                    }
                }
                if(first||Mathf.Abs(bounds.min.y-support)>.002f||bounds.min.x < -size.x/2||bounds.max.x > size.x/2||bounds.min.z < -size.z/2||bounds.max.z > size.z/2)
                    throw new InvalidOperationException("Unsupported or overhanging bench tool: "+tools[i].name+$" min={bounds.min:R} max={bounds.max:R} support={support:R} table={size:R}");
                footprints[i]=bounds;
                bool MatOverlap(Bounds a,Bounds b)=>a.min.x < b.max.x&&a.max.x > b.min.x&&a.min.z < b.max.z&&a.max.z > b.min.z;
                if(MatOverlap(bounds,workArea))throw new InvalidOperationException("Bench work area obstructed: "+tools[i].name);
                for(int j=0;j<i;j++)if(MatOverlap(bounds,footprints[j]))throw new InvalidOperationException("Overlapping bench tools: "+tools[i].name+" / "+tools[j].name);
                RecordEvent("bench-placement",tools[i].name,$"bench={root.name}; supportY={bounds.min.y}; footprint={bounds.size}; operatorSide=+X");
            }
            RecordEvent("bench-layout-pass",root.name,"tools=4; supported=true; noOverlap=true; clearWorkArea=.60x.65m; operatorSide=+X");
        }
        void ContainerDetail(Transform root,Vector3 size) {
            var steel=Material("pipe","metal-plate");
            for(int i=0;i<20;i++)for(int side=-1;side<=1;side+=2)
                Primitive(root,"corrugation",new Vector3(.025f,size.y,.035f),new Vector3(side*(size.x/2-.0125f),0,-size.z/2+i*size.z/20),steel);
        }
        void AddPresentation() {
            AddRobotPresentation();
            if(Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-material-probe")) {
                var colors=new[]{Color.red,Color.green,Color.blue};
                for(int i=0;i<3;i++){var test=PlainRobotMaterial();ownedResources.Add(test);var pixels=new Color[16];for(int j=0;j<pixels.Length;j++)pixels[j]=colors[i];var texture=new Texture2D(4,4,TextureFormat.RGBA32,false);texture.SetPixels(pixels);texture.Apply();ownedResources.Add(texture);test.SetTexture("_BaseColorMap",texture);test.SetColor("_BaseColor",Color.white);ValidateVisualMaterial(test);var probe=Primitive(DebugRoot,"Plain Lit color probe",new Vector3(.15f,.15f,.15f),Robot.position+new Vector3((i-1)*.2f,.2f,.6f),test);
                    var block=new MaterialPropertyBlock();block.SetColor("_BaseColor",Color.white);probe.GetComponent<Renderer>().SetPropertyBlock(block);RecordEvent("probe-renderer","RGB",$"renderingMask={probe.GetComponent<Renderer>().renderingLayerMask}");}
            }
            foreach(var renderer in Robot.GetComponentsInChildren<Renderer>()) {
                var assigned=renderer.sharedMaterials;
                for(int i=0;i<assigned.Length;i++)if(assigned[i]!=null&&assigned[i].shader.name=="HDRP/Lit") {
                    assigned[i]=new Material(assigned[i]);ownedResources.Add(assigned[i]);ValidateVisualMaterial(assigned[i]);
                }
                renderer.sharedMaterials=assigned;
                if(renderer.name.Contains("electronics")||renderer.name.Contains("identification"))foreach(var m in assigned)RecordEvent("robot-render-material",renderer.name,$"shader={m.shader.name}; color={m.GetColor("_BaseColor")}; surface={m.GetFloat("_SurfaceType")}; queue={m.renderQueue}; keywords={string.Join(",",m.shaderKeywords)}");
            }
            var stripe=Material("bollard","painted-epoxy");
            for(int z=0;z<30;z+=3)for(int side=-1;side<=1;side+=2)
                Primitive(VisualRoot,"aisle-paint",new Vector3(.07f,.002f,2f),new Vector3(side*2.8f,.002f,z+1.5f),stripe);
            for(int z=42;z<60;z+=3)Primitive(VisualRoot,"yard-lane-mark",new Vector3(.12f,.002f,1.8f),new Vector3(0,.002f,z),stripe);
            // Collider-free overhead structure leaves LiDAR and traversability authoritative.
            var steel=Material("pipe","metal-plate");
            var diffuser=new Material(Shader.Find("HDRP/Unlit"));ownedResources.Add(diffuser);diffuser.SetColor("_UnlitColor",Color.white);diffuser.SetColor("_EmissiveColor",new Color(4000,3840,3520));diffuser.SetFloat("_EmissiveExposureWeight",1);ValidateVisualMaterial(diffuser);
            // Local campus lighting never changes a shared profile or aquatic scene.
            foreach(var existing in UnityEngine.Object.FindObjectsByType<Light>(FindObjectsSortMode.None))
                if(existing.type==LightType.Directional&&existing.gameObject.scene==gameObject.scene&&!existing.transform.IsChildOf(transform)) {
                    priorDirectionalLights.Add(existing,existing.enabled);existing.enabled=false;
                }
            AddCampusLight("campus-daylight",Vector3.zero,LightType.Directional,35000,0,Quaternion.Euler(52,-32,0),new Color(1,.96f,.9f),1024);
            AddCampusExposure();
            if(Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-diagnostic-fill")) {
                AddCampusLight("diagnostic-unshadowed-fill",Robot.position+Vector3.up+Vector3.back*.3f,LightType.Point,2000,5,Quaternion.identity,Color.white,256);
                var fill=LightingRoot.Find("diagnostic-unshadowed-fill").GetComponent<Light>();fill.shadows=LightShadows.None;
            }
            for(int z=3;z<30;z+=6)Primitive(VisualRoot,"roof-truss",new Vector3(32,.15f,.15f),new Vector3(0,4.92f,z),steel);
            for(int x=-13;x<=11;x+=6)for(int z=3;z<30;z+=6) {
                // Suspension rods attach each fixture to the roof; no floating housings.
                Primitive(LightingRoot,"Ceiling suspension",new Vector3(.025f,.25f,.025f),new Vector3(x,4.925f,z),steel);
                var fixture=Primitive(LightingRoot,"warehouse-luminaire",new Vector3(2,.05f,.35f),new Vector3(x,4.8f,z),steel);
                var panel=Primitive(LightingRoot,"LED diffuser",new Vector3(1.85f,.008f,.28f),fixture.transform.position+Vector3.down*.032f,diffuser);panel.GetComponent<Renderer>().shadowCastingMode=UnityEngine.Rendering.ShadowCastingMode.Off;
                AddCampusLight("warehouse-bay-light",fixture.transform.position+Vector3.down*.065f,LightType.Spot,12000,12,Quaternion.Euler(90,0,0),new Color(1,.97f,.92f),512);
            }
            foreach(var box in Scenario.obstacles)if(box.kind=="packing-bench") {
                float surface=box.center[1]+box.size[1]/2-.14f;
                Vector3 center=new Vector3(box.center[0]-box.size[0]*.32f,surface+.23f,box.center[2]+box.size[2]*.22f);
                FittedEquipmentModel(LightingRoot,"desk_lamp_arm_01",center,new Vector3(.125f,.55f,.38f),steel);
                // Bulb remains below the shade. Small task lights use a broad cone;
                // bay spotlights provide the room's authoritative shadow treatment.
                AddCampusLight("Assembly task light",center+new Vector3(0,.17f,-.10f),LightType.Spot,350,2,Quaternion.Euler(100,0,0),new Color(.94f,.97f,1),256);
                LightingRoot.Find("Assembly task light").GetComponent<Light>().shadows=LightShadows.None;
            }
            if(Scenario.id.StartsWith("warehouse_compact")) {
                var boardFinish=EquipmentFinish("Service board",new Color(.23f,.27f,.29f));
                Primitive(VisualRoot,"Wall-mounted maintenance board",new Vector3(.025f,.85f,1.2f),new Vector3(-10.825f,1.4f,16.5f),boardFinish);
                for(int i=0;i<5;i++)Primitive(VisualRoot,"Tool board hooks",new Vector3(.04f,.018f,.018f),new Vector3(-10.86f,1.55f,16.05f+i*.20f),steel);
                FittedEquipmentModel(VisualRoot,"metal_toolbox",new Vector3(-15.4f,.93f,21.5f),new Vector3(.5f,.3f,.65f),boardFinish);
            }
            Primitive(DebugRoot,"start",new Vector3(.35f,.005f,.35f),new Vector3(Scenario.start[0],.02f,Scenario.start[1]),stripe);
            Primitive(DebugRoot,"goal",new Vector3(.35f,.005f,.35f),new Vector3(Scenario.goal[0],.02f,Scenario.goal[1]),stripe);
        }
        void AddCampusLight(string name,Vector3 position,LightType type,float intensity,float range,Quaternion rotation,Color color,int shadowResolution) {
            var host=new GameObject(name);host.SetActive(false);host.transform.SetParent(LightingRoot,false);host.transform.SetPositionAndRotation(position,rotation);
            var light=host.AddComponent<Light>();light.type=type;
            var data=host.AddComponent(Type.GetType("UnityEngine.Rendering.HighDefinition.HDAdditionalLightData, Unity.RenderPipelines.HighDefinition.Runtime",true));
            // HDRP 17 keeps a separate GPU light-layer mask. Writing the native
            // shadow mask alone does not synchronize the render database.
            var layerProperty=data.GetType().GetProperty("lightlayersMask");
            layerProperty.SetValue(data,Enum.ToObject(layerProperty.PropertyType,1));
            light.bakingOutput=new LightBakingOutput{isBaked=false,lightmapBakeType=LightmapBakeType.Realtime,occlusionMaskChannel=-1,probeOcclusionLightIndex=-1};
            light.type=type;light.renderingLayerMask=1;light.range=range;light.spotAngle=110;light.innerSpotAngle=90;light.color=color;light.shadows=type==LightType.Spot&&Array.Exists(Environment.GetCommandLineArgs(),x=>x=="--crane-campus-diagnostic-no-shadows")?LightShadows.None:LightShadows.Soft;
            light.lightUnit=type==LightType.Directional?UnityEngine.Rendering.LightUnit.Lux:UnityEngine.Rendering.LightUnit.Lumen;if(type==LightType.Spot||type==LightType.Point) {
                var units=Type.GetType("UnityEngine.Rendering.LightUnitUtils, Unity.RenderPipelines.Core.Runtime",true);
                var convert=units.GetMethod("ConvertIntensity",new[]{typeof(Light),typeof(float),typeof(UnityEngine.Rendering.LightUnit),typeof(UnityEngine.Rendering.LightUnit)});
                light.intensity=(float)convert.Invoke(null,new object[]{light,intensity,UnityEngine.Rendering.LightUnit.Lumen,UnityEngine.Rendering.LightUnit.Candela});
            }else light.intensity=intensity;
            host.SetActive(true);
            RecordEvent("presentation-light",name,$"type={type}; unit={light.lightUnit}; nativeIntensity={light.intensity:R}; shadows={light.shadows}; range={range:R}");
            data.GetType().GetMethod("SetShadowResolution").Invoke(data,new object[]{shadowResolution});
            var getLayers=data.GetType().GetMethod("GetLightLayers");RecordEvent("light-layers",name,$"nativeMask={light.renderingLayerMask}; position={host.transform.position}; forward={host.transform.forward}; effectiveMask={getLayers.Invoke(data,null)}; diffuse={data.GetType().GetProperty("affectDiffuse").GetValue(data)}");
        }
        void AddCampusExposure() {
            var volumeType=Type.GetType("UnityEngine.Rendering.Volume, Unity.RenderPipelines.Core.Runtime",true);
            var profileType=Type.GetType("UnityEngine.Rendering.VolumeProfile, Unity.RenderPipelines.Core.Runtime",true);
            var host=new GameObject("Campus exposure and contact shading");host.transform.SetParent(LightingRoot,false);
            var volume=host.AddComponent(volumeType);var profile=ScriptableObject.CreateInstance(profileType);ownedResources.Add(profile);
            volumeType.GetProperty("isGlobal").SetValue(volume,true);volumeType.GetField("priority").SetValue(volume,30f);volumeType.GetField("sharedProfile").SetValue(volume,profile);
            var add=profileType.GetMethod("Add",new[]{typeof(Type),typeof(bool)});
            var exposure=add.Invoke(profile,new object[]{Type.GetType("UnityEngine.Rendering.HighDefinition.Exposure, Unity.RenderPipelines.HighDefinition.Runtime",true),true});
            var mode=exposure.GetType().GetField("mode").GetValue(exposure);
            mode.GetType().GetProperty("value").SetValue(mode,Enum.Parse(mode.GetType().GetProperty("value").PropertyType,"Automatic"));
            SetVolumeFloat(exposure,"limitMin",5);SetVolumeFloat(exposure,"limitMax",16);
            var indirect=add.Invoke(profile,new object[]{Type.GetType("UnityEngine.Rendering.HighDefinition.IndirectLightingController, Unity.RenderPipelines.HighDefinition.Runtime",true),true});
            // An enclosed warehouse cannot reflect the outdoor sky at full intensity.
            SetVolumeFloat(indirect,"reflectionLightingMultiplier",.18f);
            var bloom=add.Invoke(profile,new object[]{Type.GetType("UnityEngine.Rendering.HighDefinition.Bloom, Unity.RenderPipelines.HighDefinition.Runtime",true),true});
            SetVolumeFloat(bloom,"threshold",1.1f);SetVolumeFloat(bloom,"intensity",.15f);SetVolumeFloat(bloom,"scatter",.5f);
            var ao=add.Invoke(profile,new object[]{Type.GetType("UnityEngine.Rendering.HighDefinition.ScreenSpaceAmbientOcclusion, Unity.RenderPipelines.HighDefinition.Runtime",true),true});
            SetVolumeFloat(ao,"intensity",.7f);SetVolumeFloat(ao,"radius",.5f);
        }
        static void SetVolumeFloat(object component,string field,float value) {
            var parameter=component.GetType().GetField(field).GetValue(component);parameter.GetType().GetProperty("value").SetValue(parameter,value);
        }
        void AddRobotPresentation() {
            var black=PlainRobotMaterial();ownedResources.Add(black);black.SetColor("_BaseColor",new Color(.018f,.022f,.026f));black.SetFloat("_Smoothness",.3f);ValidateVisualMaterial(black);
            var polymer=new Material(black);ownedResources.Add(polymer);polymer.SetColor("_BaseColor",new Color(.4f,.4f,.4f).linear);polymer.SetFloat("_Smoothness",.24f);
            RobotMesh(Robot.transform,"base",new Vector3(0,0,-.064f),polymer);
            foreach(string name in new[]{"base-visual","deck-visual"}){var visual=Robot.transform.Find(name);if(visual!=null)visual.gameObject.SetActive(false);}
            foreach(string side in new[]{"left","right"}) {
                var shaft=Robot.transform.Find("campus-"+side+"-wheel-contact");
                var old=Robot.transform.Find("wheel-"+side+"-visual");if(old==null&&shaft!=null)old=shaft.Find("wheel-"+side+"-visual");
                if(old!=null)old.gameObject.SetActive(false);
                if(shaft!=null){
                    var tire=RobotMesh(shaft,side=="left"?"wheel":"wheel_right",Vector3.zero,black);
                    var hub=new Material(polymer);ownedResources.Add(hub);hub.SetColor("_BaseColor",new Color(.36f,.39f,.42f));hub.SetFloat("_Metallic",.65f);hub.SetFloat("_Smoothness",.42f);ValidateVisualMaterial(hub);
                    var cap=GameObject.CreatePrimitive(PrimitiveType.Cylinder);cap.name="Wheel hub cap";cap.transform.SetParent(tire,false);cap.transform.localPosition=new Vector3(side=="left"?-.0095f:.0095f,0,0);cap.transform.localRotation=Quaternion.Euler(0,0,90);cap.transform.localScale=new Vector3(.029f,.001f,.029f);var collision=cap.GetComponent<Collider>();collision.enabled=false;Destroy(collision);cap.GetComponent<Renderer>().sharedMaterial=hub;
                    var witness=new Material(hub);ownedResources.Add(witness);witness.SetColor("_BaseColor",new Color(.92f,.87f,.63f));witness.SetFloat("_Metallic",0);ValidateVisualMaterial(witness);
                    Primitive(tire,"Wheel rotation witness spoke",new Vector3(.0015f,.005f,.028f),new Vector3(side=="left"?-.011f:.011f,0,0),witness);
                    var animation=tire.gameObject.AddComponent<CraneCampusWheelVisual>();animation.Model=Robot.GetComponent<CraneCampusRobotModel>();animation.Drive=Robot.GetComponent<DifferentialDriveDynamics>();animation.Left=side=="left";
                }
            }
            var scan=Robot.transform.Find("base_scan");
            if(scan!=null){var old=scan.Find("lidar-visual");if(old!=null)old.gameObject.SetActive(false);RobotMesh(scan,"lidar",Vector3.zero,black);}
        }
        Material PlainRobotMaterial(){var source=Resources.Load<Material>("CampusMaterials/Profiles/robot-opaque");if(source==null)throw new MissingReferenceException("Campus plain opaque material template");return new Material(source);}

        Mesh LidarColorRegions(Mesh source) {
            var colored=Instantiate(source);ownedResources.Add(colored);colored.name=source.name+" LDS color regions";
            var vertices=source.vertices;var indices=source.triangles;
            var housing=new List<int>();var crown=new List<int>();float split=source.bounds.min.y+source.bounds.size.y*.62f;
            for(int i=0;i<indices.Length;i+=3) {
                var region=(vertices[indices[i]].y+vertices[indices[i+1]].y+vertices[indices[i+2]].y)/3>=split?crown:housing;
                region.Add(indices[i]);region.Add(indices[i+1]);region.Add(indices[i+2]);
            }
            colored.subMeshCount=2;colored.SetTriangles(housing,0);colored.SetTriangles(crown,1);return colored;
        }
        Transform RobotMesh(Transform parent,string resource,Vector3 localPosition,Material material) {
            var mesh=Resources.Load<Mesh>("CampusRobot/"+resource);
            if(mesh==null)throw new MissingReferenceException("Campus robot render mesh: "+resource);
            var visual=new GameObject("ROBOTIS "+resource);visual.transform.SetParent(parent,false);visual.transform.localPosition=localPosition;
            Material[] finishes=new[]{material};
            if(resource=="lidar"){finishes=new[]{material,EquipmentFinish("LDS gold rotor crown",new Color(.85f,.56f,.035f),.3f)};mesh=LidarColorRegions(mesh);}
            visual.AddComponent<MeshFilter>().sharedMesh=mesh;var high=visual.AddComponent<MeshRenderer>();high.sharedMaterials=finishes;
            var coarse=Resources.Load<Mesh>("CampusRobot/"+resource+"_lod");
            if(coarse!=null) {
                var low=new GameObject("ROBOTIS "+resource+" LOD");low.transform.SetParent(visual.transform,false);
                if(resource=="lidar")coarse=LidarColorRegions(coarse);
                low.AddComponent<MeshFilter>().sharedMesh=coarse;var renderer=low.AddComponent<MeshRenderer>();renderer.sharedMaterials=finishes;
                var lod=visual.AddComponent<LODGroup>();lod.SetLODs(new[]{new LOD(.12f,new[]{high}),new LOD(.003f,new[]{renderer})});lod.RecalculateBounds();
            }
            return visual.transform;
        }
        [Serializable] sealed class RuntimeAudit {
            public string schema="crane-campus-runtime-structure-v1",state="STRUCTURAL_PASS";
            public int seed,physicalBoxes,scenarioBoxes,activeVisualColliders,missingRoots;
            public bool graphicsEnabled;public double constructionMilliseconds;
            public float mass,linearDamping,angularDamping,maxAngularVelocity,fixedStep,maximumCatchupSeconds;
            public Vector3 centerOfMass,inertiaTensor;public Quaternion inertiaTensorRotation;
            public string constraints;public RobotBodyAudit[] robotBodies;
        }
        [Serializable] sealed class RobotBodyAudit {
            public string name,constraints;public float mass,maxAngularVelocity;
            public Vector3 position,inertiaTensor,centerOfMass,scale;public Quaternion inertiaTensorRotation;
        }
        void WriteRuntimeAudit(double milliseconds) {
            var report=new RuntimeAudit{seed=Seed,physicalBoxes=PhysicsRoot.childCount,
                scenarioBoxes=ScenarioRoot.GetComponentsInChildren<BoxCollider>().Length,graphicsEnabled=visuals,
                constructionMilliseconds=milliseconds,mass=Robot.mass,centerOfMass=Robot.centerOfMass,
                inertiaTensor=Robot.inertiaTensor,inertiaTensorRotation=Robot.inertiaTensorRotation,
                linearDamping=Robot.linearDamping,angularDamping=Robot.angularDamping,maxAngularVelocity=Robot.maxAngularVelocity,
                fixedStep=Time.fixedDeltaTime,maximumCatchupSeconds=Time.maximumDeltaTime,constraints=Robot.constraints.ToString()};
            var actors=Robot.GetComponentsInChildren<Rigidbody>();report.robotBodies=new RobotBodyAudit[actors.Length];
            for(int i=0;i<actors.Length;i++){var actor=actors[i];report.robotBodies[i]=new RobotBodyAudit{name=actor.name,mass=actor.mass,position=actor.position,inertiaTensor=actor.inertiaTensor,inertiaTensorRotation=actor.inertiaTensorRotation,centerOfMass=actor.centerOfMass,scale=actor.transform.lossyScale,maxAngularVelocity=actor.maxAngularVelocity,constraints=actor.constraints.ToString()};}
            foreach(var c in VisualRoot.GetComponentsInChildren<Collider>())if(c.enabled)report.activeVisualColliders++;
            foreach(var t in new[]{VisualRoot,PhysicsRoot,SemanticRoot,ScenarioRoot,LightingRoot,DebugRoot})if(t==null)report.missingRoots++;
            if(report.activeVisualColliders>0||report.missingRoots>0||report.physicalBoxes!=Manifest.boxes.Length)report.state="PARTIAL";
            File.WriteAllText(Path.Combine(OutputDirectory,"runtime-structure.json"),JsonUtility.ToJson(report,true));
            if(report.state!="STRUCTURAL_PASS")throw new InvalidDataException("Campus runtime structure failed");
        }
        void OnDestroy(){if(previousFixedStep>0)Time.fixedDeltaTime=previousFixedStep;if(previousMaximumCatchup>0)Time.maximumDeltaTime=previousMaximumCatchup;foreach(var light in priorDirectionalLights)if(light.Key!=null)light.Key.enabled=light.Value;foreach(var resource in ownedResources)if(resource!=null)Destroy(resource);}
        public void RecordEvent(string type,string id,string detail) {
            File.AppendAllText(Path.Combine(OutputDirectory,"events.jsonl"),
                JsonUtility.ToJson(new CampusEvent{type=type,id=id,detail=detail,simulationTime=Time.fixedTimeAsDouble,rosTime=Sim.Utils.ROS.Clock.time})+"\n");
        }
        void FixedUpdate(){
            if(diagnosticLidar==null||lidarDiagnostics==null||Time.fixedTimeAsDouble<nextSensorEvidence)return;
            nextSensorEvidence=Time.fixedTimeAsDouble+1;
            RecordEvent("lidar-error-evidence","base_scan",(string)lidarDiagnostics.Invoke(diagnosticLidar,null));
        }
        [Serializable] sealed class CampusEvent {public string type,id,detail;public double simulationTime,rosTime;}
    }
    public sealed class CraneCampusSemanticRecord:MonoBehaviour {
        public string SemanticId,ObstacleClass,SurfaceClass;public Transform Authority;
        public Vector3 AuthoritativePosition=>Authority!=null?Authority.position:transform.position;
        public void Configure(string id,string role,string surface,Transform authority){SemanticId=id;ObstacleClass=role;SurfaceClass=surface;Authority=authority;}
    }
    public sealed class CraneCampusSurface:MonoBehaviour {public CampusSurfaceProfile Profile;}
    public sealed class CraneCampusVisualSync:MonoBehaviour {
        public Transform Target;
        void LateUpdate(){if(Target!=null){Target.SetPositionAndRotation(transform.position,transform.rotation);Target.gameObject.SetActive(GetComponent<Collider>().enabled);}}
    }
    public sealed class CraneCampusObstacle:MonoBehaviour {
        [Serializable] sealed class PoseEvidence {public Vector3 position;public bool colliderEnabled;}
        CraneIndustrialCampus campus;CampusBox spec;Collider shape;Rigidbody mover;bool active,removed;
        double nextPoseSample;
        public void Initialize(CraneIndustrialCampus owner,CampusBox box){campus=owner;spec=box;shape=GetComponent<Collider>();active=box.activate<=0;shape.enabled=active;
            if(box.velocity!=null&&Array.Exists(box.velocity,v=>v!=0)) {
                mover=gameObject.AddComponent<Rigidbody>();mover.isKinematic=true;mover.useGravity=false;
                mover.collisionDetectionMode=CollisionDetectionMode.ContinuousSpeculative;
            }
        }
        void FixedUpdate(){double t=campus.Elapsed;if(!active&&!removed&&t>=spec.activate){active=true;shape.enabled=true;campus.RecordEvent("obstacle-activated",spec.id,"authoritative collider and visual enabled");}
            if(active&&spec.remove>0&&t>=spec.remove){active=false;removed=true;shape.enabled=false;campus.RecordEvent("obstacle-removed",spec.id,"authoritative collider and visual disabled");}
            if(active&&mover!=null){
                // Capture the actual pre-step Rigidbody pose, matching body telemetry's
                // fixed-step acquisition convention; never substitute the commanded path.
                if(Time.fixedTimeAsDouble>=nextPoseSample){
                    campus.RecordEvent("obstacle-pose",spec.id,JsonUtility.ToJson(new PoseEvidence{position=mover.position,colliderEnabled=shape.enabled}));
                    do{nextPoseSample+=.1;}while(nextPoseSample<=Time.fixedTimeAsDouble);
                }
                mover.MovePosition(mover.position+new Vector3(spec.velocity[0],spec.velocity[1],spec.velocity[2])*Time.fixedDeltaTime);
            }
        }
    }
}
