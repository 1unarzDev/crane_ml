"""Thruster fault relay between thruster_pwm_node and crane_ml: plant variants without a rebuild.

thruster_pwm_node publishes to ``in`` (set ``pwm_topic`` in a copied sim hardware YAML); this node
republishes to ``out`` (/thrusters/pwm, which the player reads) after, per physical output:

    strength k (fwd, rev): thrust goes as e|e|, so e' = sign(e) sqrt(k) |e|, clipped to 1
    dead: a strength of 0
    deadband: |pulse - 1500| below it (us) is neutral, like the T200 ESC (25 us)
    dead_time: s of sim time from command to output (the boat's command-to-thrust latency)

Run: python3 pwm_relay.py <relay.yaml>   (positional only). Logs PWM_RELAY_APPLIED with the sha256.
"""
import collections
import hashlib
import json
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import yaml

DEFAULTS = {'in': '/thrusters/pwm_cmd', 'out': '/thrusters/pwm', 'fwd': [1.0] * 4, 'rev': [1.0] * 4,
            'deadband_us': 0.0, 'dead_time': 0.0, 'rate_hz': 50.0}


def transform(pulses, cfg):
    out = []
    for i, pulse in enumerate(pulses):
        e = (float(pulse) - 1500.0) / 400.0
        if abs(pulse - 1500.0) < cfg['deadband_us']:
            e = 0.0
        k = cfg['fwd'][i] if e >= 0 else cfg['rev'][i]
        e = math.copysign(min(1.0, math.sqrt(max(k, 0.0)) * abs(e)), e)
        out.append(int(round(1500 + 400 * e)))
    return out


class Relay(Node):
    def __init__(self, cfg, sha):
        super().__init__('pwm_relay', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=bool(cfg.get('use_sim_time', True)))])
        self.cfg, self.queue, self.last = cfg, collections.deque(), [1500] * 4
        self.pub = self.create_publisher(UInt16MultiArray, cfg['out'], 10)
        self.create_subscription(UInt16MultiArray, cfg['in'], self.received, 10)
        self.create_timer(1.0 / cfg['rate_hz'], self.tick)
        self.get_logger().info(f'PWM_RELAY_APPLIED sha={sha} cfg={json.dumps(cfg, sort_keys=True)}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def received(self, msg):
        self.queue.append((self.now() + self.cfg['dead_time'], transform(list(msg.data)[:4], self.cfg)))
        self.tick()

    def tick(self):
        now = self.now()
        while self.queue and self.queue[0][0] <= now:
            self.last = self.queue.popleft()[1]
        msg = UInt16MultiArray()
        msg.data = self.last
        self.pub.publish(msg)


def main():
    text = open(sys.argv[1]).read()
    cfg = {**DEFAULTS, **(yaml.safe_load(text) or {})}
    rclpy.init()
    node = Relay(cfg, hashlib.sha256(text.encode()).hexdigest()[:12])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
