"""The T1 world as a ROS node: the plant card's ProfilePlant, stepped on /thrusters/pwm, publishing what the player
publishes (Docs/SimT1.md).

    python3 t1node.py RUN/t1.json        # inside the ROS container; crane sim up --tier T1 writes the JSON

It runs with the ROS python alone (rclpy, the stdlib): the plant comes from the file the JSON names
(Tools/Characterization/plant.py, stdlib only). ``Twin`` is the world without ROS (tests step it on the host);
``main`` wraps it in rclpy. The twin steps at the card's tick (dt); the loop paces the ticks at ``rate`` x real time
and publishes /clock every tick, so the stack's use_sim_time nodes run on the twin's time from 0.

Log lines: ``CRANE_T1_PLANT ...`` once the card is loaded, ``CRANE_T1_READY ...`` once the world steps; the state
dir's t1.alive is rewritten each wall second ({t_sim, wall, rate}).
"""

import importlib.util
import json
import math
import os
import random
import sys
import time

GRAVITY = 9.81
EARTH_RADIUS = 6378137.0
# Covariance floors of the player's sensor contract: exact simulated data is never infinitely certain.
POSITION_VARIANCE_FLOOR, VELOCITY_VARIANCE_FLOOR, ATTITUDE_VARIANCE = 0.01, 1e-4, 1e-4


def load_plant(path):
    """The plant module at PATH (a flat file, loaded by path)."""
    name = 'crane_t1_plant'
    if name in sys.modules and getattr(sys.modules[name], '__file__', None) == str(path):
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module            # dataclasses look their module up while the file runs
    spec.loader.exec_module(module)
    return module


def due(now, hz, nxt):
    """(publish now?, the next due time): whole periods from 0, as the player's sensors keep cadence."""
    if now + 1e-9 < nxt:
        return False, nxt
    period = 1.0 / max(0.1, hz)
    nxt += period
    while nxt <= now + 1e-9:
        nxt += period
    return True, nxt


class Twin:
    """The world without ROS: ProfilePlant on held PWM, the player's command timeout, the sensor quantities."""

    def __init__(self, cfg, plant):
        self.cfg = cfg
        self.dt = float(cfg['dt'])
        self.plant = plant.ProfilePlant(cfg['profile'], self.dt, tuple(cfg['current']))
        self.n = len(cfg['profile']['thrusters'])
        self.neutral = [plant.PWM_NEUTRAL] * self.n
        self.timeout = int(cfg['world']['command'].get('timeout_ticks', 25))
        self.lever = {k: tuple(v) for k, v in cfg['lever'].items()}
        self.rng = random.Random(int(cfg['world']['noise'].get('seed', 0)))
        self.reset()

    def reset(self):
        """The start pose at rest, thrusters neutral, no command; sim time keeps running."""
        x, y, yaw = self.cfg['start']
        k = getattr(self, 'k', 0)
        self.plant.reset((x, y, yaw, 0.0, 0.0, 0.0))
        self.plant.k, self.plant.t = k, k * self.dt
        self.k = k
        self.pwm, self.pending, self.last_applied = list(self.neutral), None, -1
        self.timed_out = False
        self.prev_imu = None
        self.specific_force = (0.0, 0.0, GRAVITY)

    def command(self, pulses):
        """A PWM message (us, FL,FR,RR,RL); dropped unless it has one pulse per thruster. Applied at the next tick."""
        pulses = list(pulses)
        if len(pulses) != self.n:
            return False
        self.pending = [float(p) for p in pulses]
        return True

    def step(self):
        """One tick: apply the pending command (or go neutral after the timeout), step the plant, differentiate."""
        if self.pending is not None:
            self.pwm, self.pending, self.last_applied, self.timed_out = self.pending, None, self.k, False
        elif not self.timed_out and self.last_applied >= 0 and self.k - self.last_applied > self.timeout:
            self.pwm, self.timed_out = list(self.neutral), True
        self.plant.step_pwm(self.pwm)
        self.k += 1
        self._differentiate()

    @property
    def t(self):
        return self.k * self.dt

    def pose(self):
        x, y, psi = self.plant.state[:3]
        return x, y, psi

    def body_twist(self):
        """(u, v, r): body FLU velocity and yaw rate."""
        return self.plant.body_velocity()

    def point(self, link):
        """World (x, y, z) of a link rigidly on the body."""
        x, y, psi = self.pose()
        lx, ly, lz = self.lever[link]
        c, s = math.cos(psi), math.sin(psi)
        return x + c * lx - s * ly, y + s * lx + c * ly, lz

    def _point_velocity(self, link):
        x, y, psi, vx, vy, r = self.plant.state
        lx, ly, _ = self.lever[link]
        c, s = math.cos(psi), math.sin(psi)
        rx, ry = c * lx - s * ly, s * lx + c * ly
        return vx - r * ry, vy + r * rx

    def _differentiate(self):
        """The specific force of the IMU point each tick (body frame, +g up at rest), as the player's contract."""
        v = self._point_velocity('imu')
        if self.prev_imu is None:
            ax = ay = 0.0
        else:
            ax, ay = (v[0] - self.prev_imu[0]) / self.dt, (v[1] - self.prev_imu[1]) / self.dt
        self.prev_imu = v
        psi = self.plant.state[2]
        c, s = math.cos(psi), math.sin(psi)
        self.specific_force = (c * ax + s * ay, -s * ax + c * ay, GRAVITY)

    def walls(self):
        """The pool's wall segments ((x0, y0), (x1, y1)) in the world, or [] without a pool."""
        pool = self.cfg.get('pool')
        if not pool:
            return []
        length, half = float(pool['length']), float(pool['width']) / 2.0
        corners = [(0.0, -half), (length, -half), (length, half), (0.0, half)]
        return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]

    def scan(self):
        """The lidar ring: (x, y, 0) in the lidar link (axes as base_link) where each beam meets a wall within the
        range; a 2-D slice at the mount height, beams evenly spaced from the bow counter-clockwise."""
        lidar, walls = self.cfg['world']['lidar'], self.walls()
        beams, reach = int(lidar.get('beams', 720)), float(lidar.get('range', 30.0))
        ox, oy, _ = self.point('lidar')
        psi = self.pose()[2]
        out = []
        for i in range(beams if walls else 0):
            a = 2.0 * math.pi * i / beams
            dx, dy = math.cos(psi + a), math.sin(psi + a)
            best = reach
            for (x0, y0), (x1, y1) in walls:
                ex, ey = x1 - x0, y1 - y0
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                t = ((x0 - ox) * ey - (y0 - oy) * ex) / den        # along the beam
                s = ((x0 - ox) * dy - (y0 - oy) * dx) / den        # along the wall
                if 0.0 < t < best and -1e-9 <= s <= 1.0 + 1e-9:
                    best = t
            if best < reach:
                out.append((best * math.cos(a), best * math.sin(a), 0.0))
        return out

    def gauss(self, sigma):
        return self.rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def fix(self):
        """(latitude, longitude, altitude, horizontal variance) of the GPS antenna, with the gps_sigma noise."""
        gps, noise = self.cfg['world']['gps'], self.cfg['world']['noise']
        sigma = float(noise.get('gps_sigma', 0.0))
        east, north, up = self.point('gps')
        east, north = east + self.gauss(sigma), north + self.gauss(sigma)
        lat0, lon0 = gps['origin']
        lat = lat0 + north / EARTH_RADIUS * (180.0 / math.pi)
        lon = lon0 + east / (EARTH_RADIUS * math.cos(math.radians(lat0))) * (180.0 / math.pi)
        return lat, lon, up, max(POSITION_VARIANCE_FLOOR, sigma * sigma)


# ---- ROS ----------------------------------------------------------------------------------------------------------

def _quat(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _stamp(t):
    from builtin_interfaces.msg import Time
    sec = int(math.floor(t))
    nanosec = int(round((t - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec, nanosec = sec + 1, 0
    return Time(sec=sec, nanosec=nanosec)


class Node:
    """The rclpy side: publishers per Docs/SimT1.md, the PWM subscription and the reset service."""

    def __init__(self, cfg, twin):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, NavSatFix
        from std_msgs.msg import UInt16MultiArray
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        self.rclpy, self.cfg, self.twin = rclpy, cfg, twin
        self.world, self.frames = cfg['world'], cfg['world']['frames']
        self.truth_mode = cfg['odom'] == 'truth'
        self.node = node = rclpy.create_node('crane_t1')
        w = self.world
        self.msgs = {'Odometry': Odometry, 'Imu': Imu, 'NavSatFix': NavSatFix, 'Clock': Clock, 'TFMessage': TFMessage}
        self.pub = {'clock': node.create_publisher(Clock, '/clock', 10),
                    'truth': node.create_publisher(Odometry, w['truth']['topic'], 10),
                    'imu': node.create_publisher(Imu, w['imu']['topic'], 10),
                    'gps': node.create_publisher(NavSatFix, w['gps']['topic'], 10)}
        if self.truth_mode:
            self.pub['local'] = node.create_publisher(Odometry, w['estimate']['local'], 10)
            self.pub['global'] = node.create_publisher(Odometry, w['estimate']['global'], 10)
            self.pub['tf'] = node.create_publisher(TFMessage, '/tf', 100)
            static = node.create_publisher(TFMessage, '/tf_static',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            fixed = [self._transform(0.0, self.frames['map'], self.frames['odom'], (0.0, 0.0, 0.0))]
            if twin.walls():                   # robot_state_publisher is off with localization: the lidar mount
                lx, ly, lz = twin.lever['lidar']
                mount = self._transform(0.0, self.frames['base'], w['lidar']['link'], (lx, ly, 0.0))
                mount.transform.translation.z = float(lz)
                fixed.append(mount)
            static.publish(TFMessage(transforms=fixed))
            self.pub['tf_static'] = static
        else:
            self.pub['fcu'] = node.create_publisher(Odometry, w['fcu']['topic'], 10)
        if twin.walls():
            from nav_msgs.msg import OccupancyGrid
            from sensor_msgs.msg import PointCloud2
            self.pub['lidar'] = node.create_publisher(PointCloud2, w['lidar']['topic'], 10)
            self.pub['costmap'] = node.create_publisher(
                OccupancyGrid, w['costmap']['topic'],
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.msgs.update(PointCloud2=PointCloud2, OccupancyGrid=OccupancyGrid)
        node.create_subscription(UInt16MultiArray, w['command']['topic'], lambda m: twin.command(m.data), 10)
        node.create_service(Trigger, w['reset'], self._reset)
        self.next = {k: 0.0 for k in ('truth', 'fcu', 'imu', 'gps', 'estimate', 'lidar')}

    def _reset(self, request, response):
        self.twin.reset()
        x, y, yaw = self.twin.pose()
        response.success, response.message = True, f'x={x:.3f} y={y:.3f} yaw={yaw:.4f}'
        return response

    def _transform(self, t, parent, child, pose):
        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = _stamp(t), parent, child
        tf.transform.translation.x, tf.transform.translation.y = float(pose[0]), float(pose[1])
        tf.transform.rotation = _quat(pose[2])
        return tf

    def _odometry(self, t, frame, twist_sigma=None):
        twin = self.twin
        x, y, psi = twin.pose()
        u, v, r = twin.body_twist()
        msg = self.msgs['Odometry']()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = _stamp(t), frame, self.frames['base']
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation = _quat(psi)
        if twist_sigma is not None:
            u, v = u + twin.gauss(twist_sigma), v + twin.gauss(twist_sigma)
            var = max(VELOCITY_VARIANCE_FLOOR, twist_sigma * twist_sigma)
            msg.twist.covariance = [var if i % 7 == 0 else 0.0 for i in range(36)]
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = u, v
        msg.twist.twist.angular.z = r
        return msg

    def _imu(self, t):
        twin = self.twin
        msg = self.msgs['Imu']()
        msg.header.stamp, msg.header.frame_id = _stamp(t), self.frames['base']
        msg.orientation = _quat(twin.pose()[2])
        msg.angular_velocity.z = twin.body_twist()[2]
        fx, fy, fz = twin.specific_force
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = fx, fy, fz
        cov = [ATTITUDE_VARIANCE if i % 4 == 0 else 0.0 for i in range(9)]
        msg.orientation_covariance = msg.angular_velocity_covariance = msg.linear_acceleration_covariance = cov
        return msg

    def _fix(self, t):
        from sensor_msgs.msg import NavSatStatus
        lat, lon, alt, h = self.twin.fix()
        msg = self.msgs['NavSatFix']()
        msg.header.stamp, msg.header.frame_id = _stamp(t), self.world['gps']['link']
        msg.status.status, msg.status.service = NavSatStatus.STATUS_FIX, NavSatStatus.SERVICE_GPS
        msg.latitude, msg.longitude, msg.altitude = float(lat), float(lon), float(alt)
        msg.position_covariance = [h, 0.0, 0.0, 0.0, h, 0.0, 0.0, 0.0, 4.0 * h]
        msg.position_covariance_type = self.msgs['NavSatFix'].COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg

    def _cloud(self, t, points):
        """PointCloud2 of (x, y, z) float32 in the lidar link."""
        import struct
        from sensor_msgs.msg import PointField
        msg = self.msgs['PointCloud2']()
        msg.header.stamp, msg.header.frame_id = _stamp(t), self.world['lidar']['link']
        msg.height, msg.width = 1, len(points)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(points), True
        msg.data = b''.join(struct.pack('<fff', *p) for p in points)
        return msg

    def _grid(self, t, points):
        """The perception costmap of these returns: lethal cells in a square about base_link (the costmap frame)."""
        spec = self.world['costmap']
        size, res = float(spec.get('size', 24.0)), float(spec.get('resolution', 0.15))
        n = int(round(size / res))
        lx, ly, _ = self.twin.lever['lidar']
        from array import array
        data = array('b', bytes(n * n))
        for x, y, _ in points:
            i, j = int((x + lx + size / 2) // res), int((y + ly + size / 2) // res)
            if 0 <= i < n and 0 <= j < n:
                data[j * n + i] = 100
        msg = self.msgs['OccupancyGrid']()
        msg.header.stamp, msg.header.frame_id = _stamp(t), spec.get('frame', self.frames['base'])
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution, msg.info.width, msg.info.height = res, n, n
        msg.info.origin.position.x = msg.info.origin.position.y = -size / 2
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def publish(self):
        """What is due at the twin's time: /clock every tick, the rest at their rates."""
        t, w, pub = self.twin.t, self.world, self.pub
        pub['clock'].publish(self.msgs['Clock'](clock=_stamp(t)))
        for key, hz, make in (('truth', w['truth']['hz'], lambda: self._odometry(t, self.frames['odom'])),
                              ('imu', w['imu']['hz'], lambda: self._imu(t)),
                              ('gps', w['gps']['hz'], lambda: self._fix(t))):
            go, self.next[key] = due(t, hz, self.next[key])
            if go:
                pub[key].publish(make())
        if 'lidar' in pub:
            go, self.next['lidar'] = due(t, w['lidar']['hz'], self.next['lidar'])
            if go:
                points = self.twin.scan()
                pub['lidar'].publish(self._cloud(t, points))
                pub['costmap'].publish(self._grid(t, points))
        if not self.truth_mode:
            go, self.next['fcu'] = due(t, w['fcu']['hz'], self.next['fcu'])
            if go:
                sigma = float(w['noise'].get('fcu_velocity_sigma', 0.0))
                pub['fcu'].publish(self._odometry(t, self.frames['odom'], twist_sigma=sigma))
            return
        go, self.next['estimate'] = due(t, w['estimate']['hz'], self.next['estimate'])
        if go:
            local = self._odometry(t, self.frames['odom'])
            pub['local'].publish(local)
            glob = self._odometry(t, self.frames['map'])
            pub['global'].publish(glob)
            pose = self.twin.pose()
            pub['tf'].publish(self.msgs['TFMessage'](
                transforms=[self._transform(t, self.frames['odom'], self.frames['base'], pose)]))

    def loop(self, heartbeat):
        """Step at rate x real time (as fast as it can when it cannot keep up), serving callbacks between ticks."""
        rclpy, node, twin = self.rclpy, self.node, self.twin
        period = twin.dt / float(self.cfg['rate'])
        wall0 = nxt = time.monotonic()
        beat, k0 = wall0, twin.k
        while rclpy.ok():
            now = time.monotonic()
            if now < nxt:
                rclpy.spin_once(node, timeout_sec=nxt - now)
                continue
            rclpy.spin_once(node, timeout_sec=0.0)
            twin.step()
            self.publish()
            nxt += period
            if nxt < now - 0.5:          # a stall (or a host that cannot keep up): no burst to catch up
                nxt = now
            if now - beat >= 1.0:
                heartbeat({'t_sim': round(twin.t, 3), 'wall': time.time(),
                           'rate': round((twin.k - k0) * twin.dt / (now - beat), 3)})
                beat, k0 = now, twin.k


def _heartbeat(path):
    def write(doc):
        tmp = f'{path}.tmp'
        with open(tmp, 'w') as f:
            json.dump(doc, f)
        os.replace(tmp, path)
    return write


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print('usage: t1node.py T1_JSON', file=sys.stderr)
        return 2
    with open(argv[0]) as f:
        cfg = json.load(f)
    plant = load_plant(cfg['plant'])
    twin = Twin(cfg, plant)
    print(f"CRANE_T1_PLANT profile={cfg['profile'].get('name', '?')} sha256={cfg['profile_sha256']} dt={twin.dt}",
          flush=True)
    node = Node(cfg, twin)
    beat = _heartbeat(cfg['alive'])
    beat({'t_sim': 0.0, 'wall': time.time(), 'rate': 0.0})
    x, y, yaw = cfg['start']
    print(f"CRANE_T1_READY course={cfg['course']} start=({x},{y},{yaw}) odom={cfg['odom']} rate={cfg['rate']}x "
          f"current=({cfg['current'][0]:.4f},{cfg['current'][1]:.4f})", flush=True)
    try:
        node.loop(beat)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
