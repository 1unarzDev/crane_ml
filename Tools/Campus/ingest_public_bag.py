#!/usr/bin/env python3
"""Audit a physical ROS bag offline without substituting bag time for acquisition.

Install rosbags==0.11.0 in a virtual environment. ROS itself is not required.
Outputs are a pilot ingestion report and stamped pose/command/scan summaries;
they are not a fitted CRANE profile or a physical sim-to-real result.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from heldout_calibration import sha256_file


def vector(value):
    return {key: float(getattr(value, key)) for key in 'xyz'}


def stamp_nanoseconds(message):
    header = getattr(message, 'header', None)
    if header is None:
        return None
    stamp = header.stamp
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def stamp_seconds(message):
    stamp = stamp_nanoseconds(message)
    return None if stamp is None else stamp*1e-9


def flu_pose(position, quaternion):
    """Fixed ROS FLU -> Unity RUF mapping; no trajectory registration."""
    x, y, z, w = [float(getattr(quaternion, key)) for key in 'xyzw']
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1) > .01:
        raise ValueError('Pose quaternion must be finite and normalized')
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return dict(position={'x': -float(position.y), 'y': float(position.z),
                          'z': float(position.x)}, yaw=-math.degrees(yaw))


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return dict(count=len(values), minimum=ordered[0], maximum=ordered[-1],
                mean=statistics.mean(values), median=statistics.median(values),
                p95=ordered[math.ceil(.95*len(ordered))-1])


def audit_bag(bag, output, source_url, license_name):
    from rosbags.highlevel import AnyReader
    output.mkdir(parents=True, exist_ok=True)
    stats = defaultdict(lambda: dict(count=0, headerCount=0, missingHeaderCount=0,
                                    zeroHeaderCount=0, nonIncreasingHeaderCount=0,
                                    periods=[], bagMinusHeader=[], frames=set()))
    last = {}
    names = ['poses', 'commands', 'scans', 'joints']
    if any((output/(name+'.jsonl')).exists() for name in names):
        raise ValueError('Choose a new output directory; existing pilot artifacts are immutable')
    writers = {name: (output / (name+'.jsonl')).open('x') for name in names}
    try:
        with AnyReader([bag]) as reader:
            for connection, record_ns, raw in reader.messages():
                msg = reader.deserialize(raw, connection.msgtype)
                topic = connection.topic
                stat = stats[topic]
                stat['type'] = connection.msgtype
                stat['count'] += 1
                acquired_ns = stamp_nanoseconds(msg)
                acquired = None if acquired_ns is None else acquired_ns*1e-9
                base = dict(topic=topic, bagRecordTimeNs=record_ns,
                            acquisitionTimeNs=acquired_ns, bagRecordTime=record_ns*1e-9,
                            acquisitionTime=acquired,
                            timestampMeaning='publisher header; hardware acquisition semantics unverified',
                            publicationTime=None, receptionTime=None, applicationTime=None)
                if acquired is None:
                    stat['missingHeaderCount'] += 1
                else:
                    stat['headerCount'] += 1
                    stat['zeroHeaderCount'] += int(acquired == 0)
                    stat['frames'].add(msg.header.frame_id)
                    stat['bagMinusHeader'].append((record_ns-acquired_ns)*1e-9)
                    if topic in last:
                        gap = (acquired_ns-last[topic])*1e-9
                        stat['nonIncreasingHeaderCount'] += int(gap <= 0)
                        if gap > 0:
                            stat['periods'].append(gap)
                    last[topic] = acquired_ns
                if connection.msgtype.endswith('/Odometry'):
                    base.update(flu_pose(msg.pose.pose.position, msg.pose.pose.orientation))
                    base.update(frameId=msg.header.frame_id, childFrameId=msg.child_frame_id,
                                twistFrame='childFrameId (body); not world-frame velocity',
                                bodyVelocityFLU=vector(msg.twist.twist.linear),
                                bodyAngularVelocityFLU=vector(msg.twist.twist.angular))
                    writers['poses'].write(json.dumps(base, allow_nan=False)+'\n')
                elif connection.msgtype.endswith('/Twist') or connection.msgtype.endswith('/TwistStamped'):
                    twist = getattr(msg, 'twist', msg)
                    base.update(linearFLU=vector(twist.linear), angularFLU=vector(twist.angular))
                    writers['commands'].write(json.dumps(base, allow_nan=False)+'\n')
                elif connection.msgtype.endswith('/JointState'):
                    base.update(names=list(msg.name), positions=[float(v) for v in msg.position],
                                velocities=[float(v) for v in msg.velocity],
                                efforts=[float(v) for v in msg.effort],
                                effortMeaning='ROS effort field; no electrical-current units inferred')
                    writers['joints'].write(json.dumps(base, allow_nan=False)+'\n')
                elif connection.msgtype.endswith('/LaserScan'):
                    finite = [float(r) for r in msg.ranges if math.isfinite(r)]
                    in_range = [r for r in finite if msg.range_min <= r <= msg.range_max]
                    base.update(rayCount=len(msg.ranges), finiteCount=len(finite),
                                inRangeCount=len(in_range),
                                invalidOrOutOfRangeCount=len(msg.ranges)-len(in_range),
                                rangeMin=float(msg.range_min), rangeMax=float(msg.range_max),
                                scanTime=float(msg.scan_time), timeIncrement=float(msg.time_increment),
                                rangeDistribution=distribution(in_range))
                    writers['scans'].write(json.dumps(base, allow_nan=False)+'\n')
            bag_duration = reader.duration*1e-9
    finally:
        for writer in writers.values():
            writer.close()
    topics = {}
    for topic, stat in stats.items():
        periods = stat.pop('periods')
        offsets = stat.pop('bagMinusHeader')
        frames = sorted(stat.pop('frames'))
        topics[topic] = dict(stat, frames=frames, headerPeriodSeconds=distribution(periods),
                            positiveHeaderRateHz=1/statistics.mean(periods) if periods else None,
                            bagRecordMinusHeaderSeconds=distribution(offsets))
    return dict(schema='crane-public-bag-pilot-v1', state='PUBLIC_DATA_INGESTED',
                dataSource='physical-public-external-platform', bagDurationSeconds=bag_duration,
                source=dict(url=source_url, license=license_name, path=str(bag.resolve()),
                            sha256=sha256_file(bag)), topics=topics,
                outputs={name: dict(path=str((output/(name+'.jsonl')).resolve()),
                                   sha256=sha256_file(output/(name+'.jsonl')))
                         for name in writers},
                boundaries=['Bag record minus header time is not measured transport latency.',
                            'Unstamped commands retain null acquisition times.',
                            'No plant/sensor fitting, repeatability, or closed-loop transfer is established.',
                            'External-platform evidence is not CRANE robot calibration.'])


def ingest_tum(path, output):
    """Preserve published truth frame/timestamps, with fixed FLU conversion."""
    from types import SimpleNamespace
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith('#'):
            continue
        values = [float(v) for v in line.split()]
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            raise ValueError('TUM reference requires eight finite fields per row')
        t, px, py, pz, qx, qy, qz, qw = values
        if rows and t <= rows[-1]['acquisitionTime']:
            raise ValueError('TUM timestamps must be strictly increasing')
        row = flu_pose(SimpleNamespace(x=px, y=py, z=pz),
                       SimpleNamespace(x=qx, y=qy, z=qz, w=qw))
        rows.append(dict(row, acquisitionTime=t,
                         measurementSource='published external reference trajectory',
                         coordinateFrame='unity-right-up-forward',
                         clockUncertaintySeconds=None))
    if len(rows) < 2:
        raise ValueError('Reference requires at least two samples')
    with output.open('x') as writer:
        for row in rows:
            writer.write(json.dumps(row, allow_nan=False)+'\n')
    path_length = sum(math.dist([a['position'][k] for k in 'xyz'],
                               [b['position'][k] for k in 'xyz']) for a,b in zip(rows,rows[1:]))
    return dict(samples=len(rows), durationSeconds=rows[-1]['acquisitionTime']-rows[0]['acquisitionTime'],
                sampledPathMeters=path_length, periodSeconds=distribution([
                    b['acquisitionTime']-a['acquisitionTime'] for a,b in zip(rows,rows[1:])]),
                sourceSha256=sha256_file(path), outputSha256=sha256_file(output),
                boundary='Reference poses are preserved; no velocity, clock uncertainty, or odom-to-reference registration is inferred.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--source-url', required=True)
    parser.add_argument('--license', required=True)
    parser.add_argument('--ground-truth-tum', type=Path)
    args = parser.parse_args()
    report = audit_bag(args.bag, args.output, args.source_url, args.license)
    if args.ground_truth_tum:
        report['externalReference'] = ingest_tum(args.ground_truth_tum, args.output/'reference.jsonl')
    (args.output/'pilot-report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
