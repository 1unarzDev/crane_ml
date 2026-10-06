using System;
using System.IO;
using System.Globalization;
using UnityEngine;
using Sim.Utils.ReferenceEnvironments;

namespace Sim.Physics.Land {
    /// <summary>Opt-in finite motor response for the existing force-driven Rigidbody drivetrain.
    /// The optional contact branch measures Rigidbody wheel joints; the force branch
    /// estimates virtual shafts. Both await physical calibration.</summary>
    public sealed class CraneCampusRobotModel:MonoBehaviour {
        CraneIndustrialCampus campus;Rigidbody body;DifferentialDriveDynamics drive;
        float motorLinear,motorAngular,leftScale=1,rightScale=1;
        Rigidbody leftWheel,rightWheel;float leftIntegral,rightIntegral;bool contactWheels,jointMotors;
        PhysicsMaterial wheelMaterial;
        Mesh wheelContactMesh;
        System.Random random;
        readonly RaycastHit[] groundHits = new RaycastHit[16];
        public bool ContactWheels=>contactWheels;
        public float ResponseSeconds=.15f,Acceleration=.6f,AngularAcceleration=2f;
        public string LeftSurface="unknown",RightSurface="unknown";
        public float SlipProxy,WheelDistance,WheelYaw,LastCommandLinear,LastCommandAngular;
        public Vector3 WheelPosition;
        public float MaximumMotorTorque=.35f;
        public float LeftTorque,RightTorque,LeftOmega,RightOmega,MotorLinearReference;
        public float WheelLinearSpeed,WheelAngularSpeed;
        public float LeftIntegral=>leftIntegral;public float RightIntegral=>rightIntegral;
        public void Configure(CraneIndustrialCampus owner,DifferentialDriveDynamics controller,string[] args) {
            campus=owner;drive=controller;body=GetComponent<Rigidbody>();random=new System.Random(owner.Seed+211);
            MaximumMotorTorque=Read(args,"--crane-campus-motor-torque",.35f);
            if(!float.IsFinite(MaximumMotorTorque)||MaximumMotorTorque<=0||MaximumMotorTorque>2)throw new ArgumentOutOfRangeException("motor torque");
            ResponseSeconds=Read(args,"--crane-campus-motor-lag",.15f);
            Acceleration=Read(args,"--crane-campus-acceleration",.6f);
            leftScale=Read(args,"--crane-campus-left-wheel-scale",1f);rightScale=Read(args,"--crane-campus-right-wheel-scale",1f);
            if(!float.IsFinite(ResponseSeconds)||ResponseSeconds<.001f||ResponseSeconds>2||
                !float.IsFinite(Acceleration)||Acceleration<=0||Acceleration>3||
                !float.IsFinite(leftScale)||!float.IsFinite(rightScale)||leftScale<.9f||leftScale>1.1f||rightScale<.9f||rightScale>1.1f)
                throw new ArgumentOutOfRangeException("Campus motor/encoder parameters");
            body.solverIterations=16;body.solverVelocityIterations=4;
            body.collisionDetectionMode=CollisionDetectionMode.ContinuousDynamic;
            // Preserve nominal reference mass and automatic collider inertia until a measured fit exists.
            // Compound round wheel/caster proxies allow measured small-threshold contact.
            // They are not WheelCollider-driven or articulated motor joints.
            var chassis=GetComponent<BoxCollider>();chassis.contactOffset=.001f;
            // Keep the original URDF-derived collision envelope and align the campus
            // visual body to it. Raising only the collider hid meaningful low contacts.
            var baseVisual=transform.Find("base-visual");
            if(baseVisual!=null){baseVisual.localPosition=chassis.center;baseVisual.localScale=chassis.size;}
            contactWheels=Array.Exists(args,x=>x=="--crane-campus-contact-wheels");
            jointMotors=Array.Exists(args,x=>x=="--crane-campus-joint-motors");
            wheelMaterial=contactWheels?new PhysicsMaterial("Campus wheel rubber"){staticFriction=.9f,dynamicFriction=.8f,frictionCombine=PhysicsMaterialCombine.Minimum}:null;
            bool convexTires=Array.Exists(args,x=>x=="--crane-campus-convex-tires");
            if(contactWheels&&convexTires)wheelContactMesh=WheelCylinder(controller.WheelRadius,.018f);
            foreach(float side in new[]{-1f,1f}) {
                var wheel=new GameObject(side<0?"campus-left-wheel-contact":"campus-right-wheel-contact");
                wheel.transform.SetParent(transform,false);wheel.transform.localPosition=new Vector3(side*controller.WheelSeparation/2,.023f,0);
                if(contactWheels){
                    if(convexTires){var c=wheel.AddComponent<MeshCollider>();c.sharedMesh=wheelContactMesh;c.convex=true;c.sharedMaterial=wheelMaterial;c.contactOffset=.001f;}
                    else{var c=wheel.AddComponent<SphereCollider>();c.radius=controller.WheelRadius;c.sharedMaterial=wheelMaterial;c.contactOffset=.001f;}
                }
                else{var c=wheel.AddComponent<SphereCollider>();c.radius=controller.WheelRadius;c.sharedMaterial=chassis.sharedMaterial;}
                if(contactWheels) {
                    // Numerical ceiling must allow the free torque integration before the
                    // contact solver transfers wheel momentum to the chassis. A 12 rad/s
                    // ceiling clipped that intermediate state and discarded motor impulse.
                    // Physical shaft targets remain bounded by MaximumLinearSpeed/radius.
                    var shaft=wheel.AddComponent<Rigidbody>();shaft.mass=.02849894f;shaft.angularDamping=.02f;shaft.maxAngularVelocity=1000;
                    shaft.centerOfMass=Vector3.zero;
                    float r2=controller.WheelRadius*controller.WheelRadius;float transverse=shaft.mass*(3*r2+.018f*.018f)/12;
                    shaft.inertiaTensorRotation=Quaternion.identity;shaft.inertiaTensor=new Vector3(shaft.mass*r2/2,transverse,transverse);
                    // Speculative angular contacts on the faceted convex tire create
                    // ghost friction. At 10 ms, nominal travel is only 2.6 mm/step;
                    // the chassis retains continuous collision detection.
                    shaft.solverIterations=24;shaft.solverVelocityIterations=8;shaft.collisionDetectionMode=CollisionDetectionMode.Discrete;
                    var joint=wheel.AddComponent<HingeJoint>();joint.connectedBody=body;joint.axis=Vector3.right;joint.anchor=Vector3.zero;
                    joint.autoConfigureConnectedAnchor=false;joint.connectedAnchor=new Vector3(side*controller.WheelSeparation/2,.023f,0);joint.enableCollision=false;
                    if(side<0)leftWheel=shaft;else rightWheel=shaft;
                    var bridge=wheel.AddComponent<CraneCampusWheelContact>();bridge.Owner=gameObject;
                    var visible=transform.Find(side<0?"wheel-left-visual":"wheel-right-visual");
                    if(visible!=null){visible.SetParent(wheel.transform,false);visible.localPosition=Vector3.zero;}
                }
            }
            foreach(float side in new[]{-1f,1f}) {
                var caster=new GameObject(side<0?"campus-left-caster-contact":"campus-right-caster-contact");caster.transform.SetParent(transform,false);
                caster.transform.localPosition=new Vector3(side*.064f,.003f,-.177f);
                var casterShape=caster.AddComponent<SphereCollider>();casterShape.radius=.013f;casterShape.contactOffset=.001f;casterShape.material=chassis.material;
            }
            // Unity rotation locks act in principal inertia space. Off-center compound
            // contacts tilt those axes; inherited locks then obstruct yaw against the floor.
            // Full campus contact dynamics allow pitch/roll and retain collider-derived inertia.
            body.constraints=RigidbodyConstraints.None;
            body.maxAngularVelocity=7;
            if(Array.Exists(args,x=>x=="--crane-campus-randomize")) {
                var parameters=new System.Random(owner.Seed+17);ResponseSeconds*=1+(float)(parameters.NextDouble()*.1-.05);
                leftScale*=1+(float)(parameters.NextDouble()*.006-.003);rightScale*=1+(float)(parameters.NextDouble()*.006-.003);
            }
            // Sum of published URDF inertial masses (base, 2 wheels, 2 casters, scanner).
            // Camera/payload not specified in URDF; physical measurements must replace this prior.
            body.mass=Read(args,"--crane-campus-mass",1.5539075f)-(contactWheels?2*.02849894f:0);
            if(!float.IsFinite(body.mass)||body.mass<.5f||body.mass>5)throw new ArgumentOutOfRangeException("Campus mass");
            body.centerOfMass=new Vector3(0,.009766f,-.005834f);
            drive.SetCampusModel(this);WheelPosition=body.position;WheelYaw=body.rotation.eulerAngles.y*Mathf.Deg2Rad;
            campus.RecordEvent("robot-model","turtlebot3-waffle-reference",$"mass={body.mass:R}; radius={drive.WheelRadius:R}; separation={drive.WheelSeparation:R}; tau={ResponseSeconds:R}; acceleration={Acceleration:R}; torqueLimit={MaximumMotorTorque:R}; contactWheels={contactWheels}; tireProxy={(convexTires?"experimental-rounded-convex":"spherical-contact-reference")}; planarPitch=false; inertia=collider-derived");
        }
        static Mesh WheelCylinder(float radius,float width) {
            // Rounded tire crown: nominal 18 mm width and 33 mm rolling radius.
            // Three rings keep the cooked convex hull below PhysX's polygon limit.
            const int sides=64;var vertices=new Vector3[sides*3];var triangles=new System.Collections.Generic.List<int>();
            for(int ring=0;ring<3;ring++)for(int i=0;i<sides;i++) {
                float angle=i*Mathf.PI*2/sides;float r=ring==1?radius:radius-.002f;
                vertices[ring*sides+i]=new Vector3((ring-1)*width/2,Mathf.Cos(angle)*r,Mathf.Sin(angle)*r);
            }
            for(int ring=0;ring<2;ring++)for(int i=0;i<sides;i++){int next=(i+1)%sides;int a=ring*sides;int b=(ring+1)*sides;triangles.AddRange(new[]{a+i,a+next,b+i,a+next,b+next,b+i});}
            for(int i=1;i<sides-1;i++)triangles.AddRange(new[]{0,i+1,i,2*sides,2*sides+i,2*sides+i+1});
            var mesh=new Mesh{name="Campus authoritative rounded tire"};mesh.vertices=vertices;mesh.triangles=triangles.ToArray();mesh.RecalculateNormals();mesh.RecalculateBounds();return mesh;
        }
        static float Read(string[] args,string key,float fallback)=>float.Parse(CraneIndustrialCampus.Arg(args,key,fallback.ToString(CultureInfo.InvariantCulture)),CultureInfo.InvariantCulture);
        CampusSurfaceProfile Sample(float side,out string name,out Vector3 normal) {
            Vector3 origin=body.position+transform.right*side+Vector3.up*.12f;
            name="unknown";normal=Vector3.up;
            int count=UnityEngine.Physics.RaycastNonAlloc(origin,Vector3.down,groundHits,.5f,
                UnityEngine.Physics.DefaultRaycastLayers,QueryTriggerInteraction.Ignore);
            CampusSurfaceProfile selected=null;float closest=float.PositiveInfinity;
            for(int i=0;i<count;i++) {
                var hit=groundHits[i];if(hit.collider.attachedRigidbody==body||(leftWheel!=null&&hit.collider.attachedRigidbody==leftWheel)||(rightWheel!=null&&hit.collider.attachedRigidbody==rightWheel))continue;
                var surface=hit.collider.GetComponentInParent<CraneCampusSurface>();
                if(surface!=null&&hit.distance<closest){selected=surface.Profile;closest=hit.distance;name=selected.id;normal=hit.normal;}
            }
            if(selected!=null)return selected;
            return campus.Surface("normal-concrete");
        }
        public void Step(float commandLinear,float commandAngular) {
            float dt=Time.fixedDeltaTime;LastCommandLinear=commandLinear;LastCommandAngular=commandAngular;
            var left=Sample(-drive.WheelSeparation/2,out LeftSurface,out var ln);
            var right=Sample(drive.WheelSeparation/2,out RightSurface,out var rn);
            float alpha=1-Mathf.Exp(-dt/Mathf.Max(.001f,ResponseSeconds));
            motorLinear=Mathf.MoveTowards(motorLinear,Mathf.Lerp(motorLinear,commandLinear,alpha),Acceleration*dt);
            MotorLinearReference=motorLinear;
            motorAngular=Mathf.MoveTowards(motorAngular,Mathf.Lerp(motorAngular,commandAngular,alpha),AngularAcceleration*dt);
            float vl=(motorLinear-motorAngular*drive.WheelSeparation/2)*leftScale;
            float vr=(motorLinear+motorAngular*drive.WheelSeparation/2)*rightScale;
            float shaftMax=drive.MaximumLinearSpeed;float shaftScale=Mathf.Max(1,Mathf.Max(Mathf.Abs(vl),Mathf.Abs(vr))/shaftMax);vl/=shaftScale;vr/=shaftScale;
            float desired=(vl+vr)/2,desiredYaw=(vr-vl)/drive.WheelSeparation;
            if(contactWheels) {
                // Positive Unity yaw turns right: the physical left wheel must run
                // faster. The legacy force model's virtual shaft convention below
                // does not determine the sign of actual wheel contact mechanics.
                float contactLeft=(motorLinear+motorAngular*drive.WheelSeparation/2)*leftScale;
                float contactRight=(motorLinear-motorAngular*drive.WheelSeparation/2)*rightScale;
                float contactScale=Mathf.Max(1,Mathf.Max(Mathf.Abs(contactLeft),Mathf.Abs(contactRight))/shaftMax);
                float actualLeft=DriveWheel(leftWheel,contactLeft/contactScale,ref leftIntegral);
                float actualRight=DriveWheel(rightWheel,contactRight/contactScale,ref rightIntegral);
                float measured=(actualLeft+actualRight)/2,measuredYaw=(actualLeft-actualRight)/drive.WheelSeparation;
                WheelLinearSpeed=measured;WheelAngularSpeed=measuredYaw;
                float speedAlong=Vector3.Dot(body.linearVelocity,transform.forward);
                float resistance=(left.rollingResistance+right.rollingResistance)/2*9.81f;
                body.AddForce(-transform.forward*Mathf.Sign(speedAlong)*Mathf.Min(Mathf.Abs(speedAlong)/dt,resistance),ForceMode.Acceleration);
                WheelDistance+=measured*dt;WheelYaw+=measuredYaw*dt;
                WheelPosition+=new Vector3(Mathf.Sin(WheelYaw),0,Mathf.Cos(WheelYaw))*measured*dt;
                SlipProxy=Mathf.Abs(measured-speedAlong)/Mathf.Max(.02f,Mathf.Abs(measured));return;
            }
            float traction=(left.tractionMultiplier+right.tractionMultiplier)/2;
            float roll=(left.rollingResistance+right.rollingResistance)/2;
            Vector3 normal=(ln+rn).normalized;
            Vector3 forward=Vector3.ProjectOnPlane(transform.forward,normal).normalized;
            float speed=Vector3.Dot(body.linearVelocity,forward);
            float frictionLimit=Mathf.Min(left.dynamicFriction,right.dynamicFriction)*9.81f;
            float torqueLimit=2*MaximumMotorTorque/(drive.WheelRadius*body.mass);
            float limit=Mathf.Min(Acceleration*traction,Mathf.Min(frictionLimit,torqueLimit));
            float response=Mathf.Clamp((desired-speed)/Mathf.Max(dt,ResponseSeconds),-limit,limit);
            float rollingAcceleration=Mathf.Sign(speed)*Mathf.Min(Mathf.Abs(speed)/dt,roll*9.81f);
            float rough=(left.roughnessAcceleration+right.roughnessAcceleration)/2;
            response-=rollingAcceleration;
            if(Mathf.Abs(speed)>.01f)response+=(float)(random.NextDouble()*2-1)*rough;
            body.AddForce(forward*response,ForceMode.Acceleration);
            body.AddTorque(Vector3.up*Mathf.Clamp((desiredYaw-body.angularVelocity.y)/Mathf.Max(dt,ResponseSeconds),-AngularAcceleration*traction,AngularAcceleration*traction),ForceMode.Acceleration);
            float lateral=Vector3.Dot(body.linearVelocity,transform.right);
            float lateralLimit=Mathf.Min(left.lateralTraction,right.lateralTraction)*frictionLimit;
            body.AddForce(transform.right*Mathf.Clamp(-lateral/Mathf.Max(dt,.1f),-lateralLimit,lateralLimit),ForceMode.Acceleration);
            WheelDistance+=desired*dt;WheelYaw+=desiredYaw*dt;
            WheelLinearSpeed=desired;WheelAngularSpeed=desiredYaw;
            WheelPosition+=new Vector3(Mathf.Sin(WheelYaw),0,Mathf.Cos(WheelYaw))*desired*dt;
            SlipProxy=Mathf.Abs(desired-speed)/Mathf.Max(.02f,Mathf.Abs(desired));
        }
        float DriveWheel(Rigidbody wheel,float targetVelocity,ref float integral) {
            Vector3 axis=transform.right;float omega=Vector3.Dot(wheel.angularVelocity-body.angularVelocity,axis);
            float target=targetVelocity/drive.WheelRadius;
            if(jointMotors) {
                var joint=wheel.GetComponent<HingeJoint>();
                integral=Mathf.Clamp(integral+(target-omega)*Time.fixedDeltaTime,-2f,2f);
                joint.motor=new JointMotor{targetVelocity=(target+integral)*Mathf.Rad2Deg,force=MaximumMotorTorque,freeSpin=false};joint.useMotor=true;
                float observed=Vector3.Dot(joint.currentTorque,axis);
                if(wheel==leftWheel){LeftTorque=observed;LeftOmega=omega;}else{RightTorque=observed;RightOmega=omega;}
                return omega*drive.WheelRadius;
            }
            // Reflected base inertia sets a velocity servo gain without treating tiny wheel
            // inertia as the whole driven load. Integral torque supplies sustained load response.
            float reflected=(body.mass/2+wheel.mass)*drive.WheelRadius*drive.WheelRadius;
            float error=target-omega;float tau=Mathf.Max(.05f,ResponseSeconds);
            float candidate=Mathf.Clamp(integral+error*Time.fixedDeltaTime,-MaximumMotorTorque*tau*tau/reflected,MaximumMotorTorque*tau*tau/reflected);
            float raw=reflected*(2*error/tau+candidate/(tau*tau));
            float torque=Mathf.Clamp(raw,-MaximumMotorTorque,MaximumMotorTorque);
            if(Mathf.Abs(raw)<=MaximumMotorTorque||Mathf.Sign(error)!=Mathf.Sign(raw))integral=candidate;
            if(wheel==leftWheel){LeftTorque=torque;LeftOmega=omega;}else{RightTorque=torque;RightOmega=omega;}
            wheel.AddTorque(axis*torque,ForceMode.Force);body.AddTorque(-axis*torque,ForceMode.Force);
            return omega*drive.WheelRadius;
        }
        void OnDestroy(){if(wheelMaterial!=null)Destroy(wheelMaterial);if(wheelContactMesh!=null)Destroy(wheelContactMesh);}
        public void ResetModel(){leftIntegral=rightIntegral=0;if(leftWheel!=null)leftWheel.angularVelocity=Vector3.zero;if(rightWheel!=null)rightWheel.angularVelocity=Vector3.zero;motorLinear=motorAngular=0;WheelDistance=0;WheelYaw=body.rotation.eulerAngles.y*Mathf.Deg2Rad;WheelPosition=body.position;}
    }
    public sealed class CraneCampusWheelContact:MonoBehaviour {
        public GameObject Owner;
        void OnCollisionEnter(Collision collision){Owner.GetComponent<CraneCampusTelemetry>()?.ContactEnter(collision);}
        void OnCollisionExit(Collision collision){Owner.GetComponent<CraneCampusTelemetry>()?.ContactExit(collision);}
    }
    /// <summary>Measured body/contact evidence; no causal labels inferred from a nearby obstacle.</summary>
    public sealed class CraneCampusTelemetry:MonoBehaviour {
        [Serializable] sealed class Sample {
            public double simulationTime,rosTime;public float fixedDeltaTime;
            public Vector3 position,velocity,angularVelocity,wheelOdometryPosition,rotationDegrees;
            public float yaw,commandLinear,commandAngular,slipProxy,minimumClearance,leftTorque,rightTorque,leftOmega,rightOmega,motorLinearReference,leftIntegral,rightIntegral;
            public string leftSurface,rightSurface;public int collisionCount,activeContacts;
        }
        CraneIndustrialCampus campus;DifferentialDriveDynamics drive;Rigidbody body;
        StreamWriter writer;double nextSample,samplePeriod=.1;int collisions,activeContacts;
        public void Configure(CraneIndustrialCampus owner,DifferentialDriveDynamics controller){
            campus=owner;drive=controller;body=GetComponent<Rigidbody>();
            string[] args=Environment.GetCommandLineArgs();int index=Array.IndexOf(args,"--crane-campus-telemetry-hz");
            if(index>=0){
                if(index+1>=args.Length)throw new ArgumentException("Telemetry frequency requires a value");
                float hz=float.Parse(args[index+1],CultureInfo.InvariantCulture);
                if(!float.IsFinite(hz)||hz<1||hz>200)throw new ArgumentOutOfRangeException("Telemetry frequency must be 1–200 Hz");
                samplePeriod=Math.Max(Time.fixedDeltaTime,1.0/hz);
            }
            writer=new StreamWriter(Path.Combine(owner.OutputDirectory,"telemetry.jsonl"));writer.AutoFlush=true;
            campus.RecordEvent("telemetry-profile","body",$"periodSeconds={samplePeriod.ToString("R",CultureInfo.InvariantCulture)}; fixedStepBound=true");
        }
        void FixedUpdate(){if(campus==null||Time.fixedTimeAsDouble<nextSample)return;
            // Advance the phase rather than restarting the period at every sample;
            // float fixed-step rounding must not turn a 10 Hz stream into 9 Hz.
            do{nextSample+=samplePeriod;}while(nextSample<=Time.fixedTimeAsDouble);
            var model=GetComponent<CraneCampusRobotModel>();float clearance=5;
            foreach(var c in UnityEngine.Physics.OverlapSphere(body.position,5)){
                if(c.attachedRigidbody==body||c.transform.IsChildOf(body.transform))continue;var s=c.GetComponentInParent<CraneSemanticIdentity>();
                if(s==null||s.SemanticRole=="floor"||s.SemanticRole=="ramp"||s.SemanticRole=="threshold")continue;
                clearance=Mathf.Min(clearance,Mathf.Max(0,Vector3.Distance(body.position,c.ClosestPoint(body.position))-.24f));
            }
            writer.WriteLine(JsonUtility.ToJson(new Sample{simulationTime=Time.fixedTimeAsDouble,rosTime=Sim.Utils.ROS.Clock.time,fixedDeltaTime=Time.fixedDeltaTime,
                position=body.position,rotationDegrees=body.rotation.eulerAngles,velocity=body.linearVelocity,angularVelocity=body.angularVelocity,yaw=body.rotation.eulerAngles.y,
                commandLinear=drive.CommandedLinear,commandAngular=drive.CommandedAngular,slipProxy=model?.SlipProxy??0,
                leftTorque=model?.LeftTorque??0,rightTorque=model?.RightTorque??0,leftOmega=model?.LeftOmega??0,rightOmega=model?.RightOmega??0,motorLinearReference=model?.MotorLinearReference??0,leftIntegral=model?.LeftIntegral??0,rightIntegral=model?.RightIntegral??0,minimumClearance=clearance,collisionCount=collisions,activeContacts=activeContacts,
                leftSurface=model?.LeftSurface??"legacy",rightSurface=model?.RightSurface??"legacy",wheelOdometryPosition=model?.WheelPosition??body.position}));
        }
        void OnCollisionEnter(Collision collision)=>ContactEnter(collision);
        public void ContactEnter(Collision collision){var id=collision.collider.GetComponentInParent<CraneSemanticIdentity>();if(id==null||id.SemanticRole=="floor")return;
            collisions++;activeContacts++;campus?.RecordEvent("contact-enter",id.SemanticId,$"impulse={collision.impulse.magnitude:R}; velocity={collision.relativeVelocity.magnitude:R}");}
        void OnCollisionExit(Collision collision)=>ContactExit(collision);
        public void ContactExit(Collision collision){var id=collision.collider.GetComponentInParent<CraneSemanticIdentity>();if(id==null||id.SemanticRole=="floor")return;activeContacts=Math.Max(0,activeContacts-1);campus?.RecordEvent("contact-exit",id.SemanticId,"contact ended");}
        void OnDestroy(){writer?.Dispose();}
    }
}
