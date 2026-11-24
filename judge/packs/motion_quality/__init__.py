"""Motion quality of a recorded Nav2 run: yaw reversals and shake, command and thruster chatter, jerk,
spin in place, goal overshoot, path ratio, energy proxy and stalls, each against a threshold.

    python3 -m judge.packs.motion_quality BAG_DIR [thresholds.yaml]

``score(series, thresholds)`` is the ROS-free core (numpy); ``read_bag`` decodes the bag with
rosbag2_py (or inside the ROS container). Definitions and thresholds: ``motion_quality.yaml``.
"""

from judge.packs.motion_quality.motion_quality import DEFAULTS, read_bag, score

__all__ = ['DEFAULTS', 'read_bag', 'score']
