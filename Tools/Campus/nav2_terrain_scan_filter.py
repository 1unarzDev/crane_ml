#!/usr/bin/env python3
"""Optional registered-terrain prior for planar Nav2; retain unmodified /scan."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener,TransformException

from terrain_scan_geometry import filter_ranges


class TerrainFilter(Node):
    def __init__(self,manifest,output):
        super().__init__('crane_campus_terrain_scan_filter',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.manifest=manifest;self.output=output;self.surfaces=None;self.pending=deque()
        self.stats=dict(schema='crane-known-terrain-scan-v1',rawScans=0,publishedScans=0,removedGroundReturns=0,missingTransformDrops=0,queueDrops=0,
                        toleranceMeters=.008,source='Registered manifest terrain prior, measured raw ranges and acquisition-stamped TF; no collider hit metadata')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.publisher=self.create_publisher(LaserScan,'/campus/navigation_scan',qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.receive,qos_profile_sensor_data)
        self.create_timer(.01,self.process)

    def receive(self,message):
        self.stats['rawScans']+=1
        if len(self.pending)>=10:self.pending.popleft();self.stats['queueDrops']+=1
        self.pending.append((message,time.monotonic()))

    def process(self):
        if self.surfaces is None:
            if not self.manifest.exists():return
            try:
                raw=self.manifest.read_bytes();manifest=json.loads(raw)
            except (OSError,json.JSONDecodeError):
                # Unity may still be writing the resolved startup manifest.
                return
            self.surfaces=[b for b in manifest['boxes'] if b['kind'] in ('floor','ramp')]
            self.stats['manifestSha256']=hashlib.sha256(raw).hexdigest()
            self.stats['surfaceCount']=len(self.surfaces)
        while self.pending:
            message,received=self.pending[0]
            try:
                pose=self.buffer.lookup_transform('odom',message.header.frame_id,Time.from_msg(message.header.stamp)).transform
            except TransformException:
                if time.monotonic()-received>.5:
                    self.pending.popleft();self.stats['missingTransformDrops']+=1
                    self.get_logger().warning('Dropping scan without acquisition-time terrain transform')
                    continue
                return
            self.pending.popleft()
            translation=tuple(getattr(pose.translation,axis) for axis in 'xyz')
            quaternion=tuple(getattr(pose.rotation,axis) for axis in 'xyzw')
            ranges,removed=filter_ranges(message.ranges,message.angle_min,message.angle_increment,translation,quaternion,self.surfaces)
            filtered=copy.deepcopy(message);filtered.ranges=ranges
            self.publisher.publish(filtered)
            self.stats['publishedScans']+=1;self.stats['removedGroundReturns']+=removed
            self.output.write_text(json.dumps(self.stats,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();rclpy.init();node=TerrainFilter(args.manifest,args.output)
    try:rclpy.spin(node)
    finally:
        args.output.write_text(json.dumps(node.stats,indent=2)+'\n');node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
