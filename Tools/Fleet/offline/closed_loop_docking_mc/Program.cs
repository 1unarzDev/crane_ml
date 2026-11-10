using System; using System.Collections.Generic; using Crane.Fleet; using Crane.Fleet.Autonomy; using Crane.Fleet.Docking;
using Crane.Fleet.Perception; using Crane.Fleet.Perception.Evaluator; using Crane.Fleet.Perception.Rover;
class IO : IRoverIO {
  public string Id {get;set;} public string Role=>"cargo"; public RoverDescription Description {get;set;} public IObservationProvider Sensors {get;set;}
  public double X,Y,Yaw; public Twist Cmd; public double OdoErrScale=1.0;
  public OdometryEstimate Odometry => new OdometryEstimate{X=X+ox,Y=Y+oy,Yaw=Yaw+oyaw};
  public double ox,oy,oyaw;
  public double LiftPosition=>0; public double GripperOpening=>0; public bool GripperHolding=>false;
  public IReadOnlyList<string> Faces=>new[]{"front","rear"};
  public string State="FREE"; public DockingCommand Last; public bool Coupled;
  public ConnectorStatus Connector(string f)=> new ConnectorStatus{Face=f,State=State,SensedA=State=="HOOK_A_ENGAGED",SensedB=State=="HOOK_A_ENGAGED",Coupled=Coupled};
  public void CommandTwist(Twist t){Cmd=t;} public void CommandLift(double q){} public void CommandGripper(double o){}
  public void CommandConnector(string f, DockingCommand c){Last=c;} public void Report(string c,string p){}
}
class P { static void Main(string[] a) { int N=int.Parse(a.Length>0?a[0]:"1"); int ok=0, att=0; var lats=new List<double>(); for(int trial=0;trial<N;trial++){ var rr=new FleetRng((ulong)(1000+trial));
  string dir="/home/lunarz/worktrees/crane-fleet/Assets/StreamingAssets/Fleet"; var spec=FleetSpec.LoadDirectory(dir);
  var lead=spec.Describe("cargo_1","cargo"); var fol=spec.Describe("cargo_2","cargo");
  double lx=4.8+rr.Uniform(-0.05,0.05), ly=3.5+rr.Uniform(-0.05,0.05), lyaw=rr.Uniform(-0.05,0.05);   // leader actual parking (slightly off plan)
  var truth=new EvaluatorTruth(); var io=new IO{Id="cargo_2",Description=fol};
  io.X=4.8-0.42-0.7+rr.Uniform(-0.1,0.1); io.Y=3.5+rr.Uniform(-0.1,0.1); io.Yaw=rr.Uniform(-0.1,0.1); io.ox=rr.Uniform(-0.08,0.08); io.oy=rr.Uniform(-0.08,0.08); io.oyaw=0; // odometry error at hand-over
  truth.SetTrajectory("cargo_1", t=>new RoverTruthState{Pose=Pose.FromYaw(lx,ly,0,lyaw)});
  truth.SetTrajectory("cargo_2", t=>new RoverTruthState{Pose=Pose.FromYaw(io.X,io.Y,0,io.Yaw), Vx=io.Cmd.Vx, Vy=io.Cmd.Vy, Wz=io.Cmd.Wz});
  foreach (var id in new[]{"scout_1","lifter_1"}) truth.SetTrajectory(id, t=>new RoverTruthState{Pose=Pose.FromYaw(1,1,0,0)});
  var sim=new FleetSimulator(FleetProfile.Parse(System.IO.File.ReadAllText(dir+"/fleet_profile_v0.json")), Scenario.Parse(System.IO.File.ReadAllText(dir+"/scenarios/warehouse_a_v0.json")), truth, (ulong)(7+trial));
  io.Sensors=sim.ProviderFor("cargo_2");
  var ctl=new TagDockingController(io,"cargo_1",lead,"front","rear",spec.O["demo.cargo.tag_docking"]);
  double dt=0.005; double capT=-1;
  for(int i=0;i<12000;i++){ double t=i*dt;
    bool done=ctl.Step(t);
    // kinematic plant: body twist applied exactly; odometry error stays constant (no drift) in this check
    double c=Math.Cos(io.Yaw), s=Math.Sin(io.Yaw);
    io.X+=(c*io.Cmd.Vx - s*io.Cmd.Vy)*dt; io.Y+=(s*io.Cmd.Vx + c*io.Cmd.Vy)*dt; io.Yaw+=io.Cmd.Wz*dt;
    // face relative pose (truth): follower front face vs leader rear face
    double fx=io.X+Math.Cos(io.Yaw)*0.21, fy=io.Y+Math.Sin(io.Yaw)*0.21, rx=lx-Math.Cos(lyaw)*0.21, ry=ly-Math.Sin(lyaw)*0.21;
    double ca=Math.Cos(lyaw+Math.PI), sa=Math.Sin(lyaw+Math.PI); double dx=fx-rx, dy=fy-ry; double gap=ca*dx+sa*dy, lat=-sa*dx+ca*dy; double yerr=Angles.Wrap(io.Yaw-lyaw);
    if (io.Last==DockingCommand.Approach && io.State=="FREE") io.State="APPROACH";
    if (io.State=="APPROACH" && gap<=0.008) { bool inside=Math.Abs(lat)<=0.01 && Math.Abs(yerr)<=0.087; if(N==1)Console.WriteLine($"t={t:0.00} capture check gap={gap:0.0000} lat={lat:0.0000} yaw={yerr:0.0000} inside={inside}"); io.State=inside?"MAGNET_CAPTURED":"FREE"; if(!inside) io.Last=DockingCommand.None; capT=t; }
    if (io.State=="MAGNET_CAPTURED" && io.Last==DockingCommand.EngageHooks && t-capT>0.8) io.State="HOOK_A_ENGAGED";
    if (io.State=="HOOK_A_ENGAGED" && io.Last==DockingCommand.Verify) { io.State="LATCHED_VERIFIED"; io.Coupled=true; }
    if (gap<0) { io.X-= (c*io.Cmd.Vx)*dt; }  // contact: cannot pass through
    if (done) { if(ctl.State==TagDockingController.Phase.Latched){ok++; lats.Add(Math.Abs(lat));} att+=ctl.Attempts; Console.WriteLine($"trial {trial} done t={t:0.00} phase={ctl.State} obs={ctl.ObservationsUsed} rej={ctl.ObservationsRejected} attempts={ctl.Attempts} final lat={lat:0.0000} yaw={yerr:0.0000} est={ctl.TargetEstimate.X:0.000},{ctl.TargetEstimate.Y:0.000},{ctl.TargetEstimate.Yaw:0.000} (odo frame; truth {lx+io.ox:0.000},{ly+io.oy:0.000})"); break; }
  }
  }
  lats.Sort(); Console.WriteLine($"latched {ok}/{N}, attempts {att}, median |lat| at latch {(lats.Count>0?lats[lats.Count/2]:double.NaN):0.0000}, max {(lats.Count>0?lats[lats.Count-1]:double.NaN):0.0000}");
}}
