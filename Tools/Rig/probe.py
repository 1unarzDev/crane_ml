"""In-run progress probe for rigq's watchdog: sim time, message counts, commanded motion, travel.

Usage: python3 probe.py OUT.json '{"clock_topic": ..., "odom_topic": ..., "command_topics": [...]}'
Runs inside the ROS container in the run's domain and rewrites OUT.json once a second:
  {"wall": unix s, "sim_time": s|null, "msgs": n, "odom": [x, y]|null,
   "cmd_active_s": s, "travel_since_cmd": m|null, "travel": m}
Tracker is pure (no rclpy) so the host tests it.
"""

import json
import math
import os
import sys
import time

CMD_EPS = 0.05     # |surge|, |sway| or |yaw| above this is a motion command
CMD_GAP_S = 0.5    # consecutive non-zero commands closer than this add to cmd_active_s


class Tracker:
    def __init__(self):
        self.sim_time = None
        self.msgs = 0
        self.start = self.odom = self.cmd_origin = None
        self.travel = 0.0
        self.travel_since_cmd = None
        self.cmd_active_s = 0.0
        self.last_cmd = None

    def on_clock(self, sim_time):
        self.msgs += 1
        self.sim_time = sim_time

    def on_odom(self, x, y):
        self.msgs += 1
        self.odom = (x, y)
        if self.start is None:
            self.start = self.odom
        self.travel = max(self.travel, math.dist(self.start, self.odom))
        if self.cmd_origin is not None:
            self.travel_since_cmd = max(self.travel_since_cmd or 0.0, math.dist(self.cmd_origin, self.odom))

    def on_command(self, now, surge, sway, yaw):
        if max(abs(surge), abs(sway), abs(yaw)) <= CMD_EPS:
            self.last_cmd = None
            return
        if self.cmd_origin is None and self.odom is not None:
            self.cmd_origin = self.odom
            self.travel_since_cmd = 0.0
        if self.last_cmd is not None and now - self.last_cmd < CMD_GAP_S:
            self.cmd_active_s += now - self.last_cmd
        self.last_cmd = now

    def snapshot(self, wall):
        return {'wall': wall, 'sim_time': self.sim_time, 'msgs': self.msgs,
                'odom': list(self.odom) if self.odom else None, 'travel': round(self.travel, 3),
                'cmd_active_s': round(self.cmd_active_s, 2),
                'travel_since_cmd': None if self.travel_since_cmd is None else round(self.travel_since_cmd, 3)}


def write(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as out:
        json.dump(data, out)
    os.replace(tmp, path)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    out, cfg = argv[0], json.loads(argv[1]) if len(argv) > 1 else {}

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node('simrig_probe')
    tracker = Tracker()
    qos = qos_profile_sensor_data   # best effort: matches reliable and best-effort publishers
    node.create_subscription(Clock, cfg.get('clock_topic', '/clock'),
                             lambda m: tracker.on_clock(m.clock.sec + m.clock.nanosec * 1e-9), qos)
    node.create_subscription(Odometry, cfg.get('odom_topic', '/crane/odom'),
                             lambda m: tracker.on_odom(m.pose.pose.position.x, m.pose.pose.position.y), qos)
    for topic in cfg.get('command_topics', []):
        node.create_subscription(Twist, topic, lambda m: tracker.on_command(
            time.monotonic(), m.linear.x, m.linear.y, m.angular.z), 10)
    node.create_timer(1.0, lambda: write(out, tracker.snapshot(time.time())))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        write(out, tracker.snapshot(time.time()))


if __name__ == '__main__':
    main()
