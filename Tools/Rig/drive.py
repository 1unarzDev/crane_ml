"""Open-loop effort schedule for a sim run: publish Twist segments, then neutral.

Usage: python3 drive.py TOPIC '[[seconds, surge, sway, yaw], ...]'
Runs inside the ROS container. Segments are wall-clock; the rig runs at 1x.
"""

import json
import sys
import time

RATE_HZ = 20


def segment_at(schedule, elapsed):
    """The (surge, sway, yaw) effort at ``elapsed`` seconds, or None after the end."""
    start = 0.0
    for seconds, *effort in schedule:
        if elapsed < start + seconds:
            return tuple(float(value) for value in effort)
        start += seconds
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    topic, schedule = argv[0], json.loads(argv[1])

    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node('simrig_drive')
    publisher = node.create_publisher(Twist, topic, 10)
    began = time.monotonic()
    try:
        while (effort := segment_at(schedule, time.monotonic() - began)) is not None:
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = effort
            publisher.publish(message)
            time.sleep(1 / RATE_HZ)
        publisher.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
