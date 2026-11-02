"""Minimal reader for uncompressed rosbag2 MCAP files (stdlib only) and CDR decoders for the few
message types the plant card reads: sensor_msgs/NavSatFix, sensor_msgs/Imu, nav_msgs/Odometry,
std_msgs/UInt16MultiArray. Compressed chunks are refused (decode those through the dev container).
"""

import math
import struct


def _string(b, o):
    n = struct.unpack_from('<I', b, o)[0]
    return b[o + 4:o + 4 + n].decode('utf-8', 'replace'), o + 4 + n


def _records(b, start, end):
    pos = start
    while pos + 9 <= end:
        op = b[pos]
        n = struct.unpack_from('<Q', b, pos + 1)[0]
        yield op, pos + 9, n
        pos += 9 + n


def read(path, topics):
    """{topic: [(log_time_s, payload bytes)]} for the topics asked."""
    b = open(path, 'rb').read()
    if not b.startswith(b'\x89MCAP0\r\n'):
        raise ValueError(f'{path}: not an MCAP file')
    channels, out = {}, {t: [] for t in topics}

    def scan(buf, start, end):
        for op, o, n in _records(buf, start, end):
            if op == 4:      # channel: id u16, schema_id u16, topic str, encoding str, metadata
                cid = struct.unpack_from('<H', buf, o)[0]
                topic, _ = _string(buf, o + 4)
                channels[cid] = topic
            elif op == 5:    # message: channel u16, sequence u32, log_time u64, publish_time u64, data
                cid, _, log_t = struct.unpack_from('<HIQ', buf, o)
                topic = channels.get(cid)
                if topic in out:
                    out[topic].append((log_t * 1e-9, bytes(buf[o + 22:o + n])))
            elif op == 6:    # chunk: start u64, end u64, size u64, crc u32, compression str, records
                comp, p = _string(buf, o + 28)
                if comp:
                    raise ValueError(f'{path}: compressed chunk ({comp}) not supported')
                rl = struct.unpack_from('<Q', buf, p)[0]
                scan(buf, p + 8, p + 8 + rl)
            elif op == 15:   # footer
                return
    scan(b, 8, len(b))
    return out


class _CDR:
    def __init__(self, data):
        self.b, self.o = data, 4       # skip the encapsulation header; alignment is relative to it

    def align(self, n):
        r = (self.o - 4) % n
        if r:
            self.o += n - r

    def get(self, fmt, size):
        self.align(size)
        v = struct.unpack_from('<' + fmt, self.b, self.o)
        self.o += size * len(fmt)
        return v if len(v) > 1 else v[0]

    def string(self):
        n = self.get('I', 4)
        s = self.b[self.o:self.o + n]
        self.o += n
        return s.rstrip(b'\0').decode('utf-8', 'replace')

    def header(self):
        sec, nsec = self.get('i', 4), self.get('I', 4)
        frame = self.string()
        return sec + nsec * 1e-9, frame


def navsatfix(data):
    c = _CDR(data)
    stamp, frame = c.header()
    status = struct.unpack_from('<b', c.b, c.o)[0]
    c.o += 1
    c.get('H', 2)
    lat, lon, alt = c.get('d', 8), c.get('d', 8), c.get('d', 8)
    cov = [c.get('d', 8) for _ in range(9)]
    return {'stamp': stamp, 'status': status, 'lat': lat, 'lon': lon, 'alt': alt, 'cov': cov}


def imu(data):
    c = _CDR(data)
    stamp, frame = c.header()
    q = [c.get('d', 8) for _ in range(4)]
    [c.get('d', 8) for _ in range(9)]
    w = [c.get('d', 8) for _ in range(3)]
    [c.get('d', 8) for _ in range(9)]
    a = [c.get('d', 8) for _ in range(3)]
    return {'stamp': stamp, 'frame': frame, 'q': q, 'w': w, 'a': a}


def odometry(data):
    c = _CDR(data)
    stamp, frame = c.header()
    child = c.string()
    p = [c.get('d', 8) for _ in range(3)]
    qx, qy, qz, qw = [c.get('d', 8) for _ in range(4)]
    [c.get('d', 8) for _ in range(36)]
    lin = [c.get('d', 8) for _ in range(3)]
    ang = [c.get('d', 8) for _ in range(3)]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {'stamp': stamp, 'frame': frame, 'child': child, 'x': p[0], 'y': p[1], 'yaw': yaw,
            'vx': lin[0], 'vy': lin[1], 'wz': ang[2]}


def uint16_array(data):
    c = _CDR(data)
    ndim = c.get('I', 4)
    for _ in range(ndim):
        c.string()
        c.get('I', 4)
        c.get('I', 4)
    c.get('I', 4)                 # data_offset
    n = c.get('I', 4)
    return [c.get('H', 2) for _ in range(n)]
