#!/usr/bin/env python3
"""Passive acquisition-stamped ROS evidence. Subscription never changes commands or Nav2; action state is relayed only to the presentation topic."""
import argparse,gzip,json,time,math,pathlib,signal
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan,Imu
from nav_msgs.msg import Odometry,OccupancyGrid,Path
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=230);args=p.parse_args()
rclpy.init();node=Node('crane_campus_passive_evidence');stamp=[0.];counts={};out=gzip.open(args.output,'wt');subs=[]
def clean(x):
 if isinstance(x,float) and not math.isfinite(x):return None
 if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [clean(v) for v in x]
 return x
status_publisher=node.create_publisher(String,'/campus/nav2_state',10);latest_state=[None]
def publish_state():
 if latest_state[0] is not None:status_publisher.publish(String(data=json.dumps(latest_state[0])))
node.create_timer(1.,publish_state)
node.create_subscription(Clock,'/clock',lambda m:stamp.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10)
for topic,typ in [('/scan',LaserScan),('/crane/odom',Odometry),('/crane/cmd_vel_stamped',TwistStamped),('/tf',TFMessage),('/plan',Path),('/local_costmap/costmap',OccupancyGrid),('/campus/imu',Imu),('/campus/wheel_odom',Odometry),('/navigate_to_pose/_action/status',GoalStatusArray)]:
 def record(m,t=topic):
  if t=='/navigate_to_pose/_action/status' and m.status_list:
   status=m.status_list[-1].status;latest_state[0]={'state':{0:'unknown',1:'accepted',2:'executing',3:'canceling',4:'succeeded',5:'canceled',6:'aborted'}.get(status,'unknown'),'rosTime':stamp[0]};publish_state()
  counts[t]=counts.get(t,0)+1;out.write(json.dumps({'topic':t,'receivedSimulationTime':stamp[0],'message':clean(message_to_ordereddict(m))},separators=(',',':'))+'\n')
 subs.append(node.create_subscription(typ,topic,record,qos_profile_sensor_data if typ in [LaserScan,Imu] else QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE) if typ is GoalStatusArray else 10))
running=[True]
signal.signal(signal.SIGTERM,lambda *_:running.__setitem__(0,False))
start=time.monotonic()
try:
 while running[0] and time.monotonic()-start<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
finally:
 out.close();pathlib.Path(args.output+'.counts.json').write_text(json.dumps(counts,indent=2));node.destroy_node();rclpy.shutdown()
