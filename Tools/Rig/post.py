"""After a rig run: task events, referee verdict and task outcome from the session bag.

Usage (in the container, ROS sourced): python3 -P post.py SESSION_DIR TASK
Writes SESSION_DIR/events.jsonl (/task/events, each with 'time' = bag receive time, the clock
of the /crane/odom track) and SESSION_DIR/referee.json (analysis/referee.judge_task over
course_truth.json and the /crane/odom track, with the rubric config/missions/TASK.yaml), and
prints one JSON line with the task outcome for session.yaml.
"""
import json
import math
import sys
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ('/task/events', '/task/status', '/crane/odom')


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in TOPICS}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in types:
            yield topic, deserialize_message(data, types[topic]), stamp * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main(session, task):
    session = Path(session)
    events, track, status = [], [], {}
    for topic, msg, t in read(session / 'bag'):
        if topic == '/task/events':
            events.append(dict(json.loads(msg.data), time=t))
        elif topic == '/task/status':
            status = json.loads(msg.data)
        else:
            p = msg.pose.pose
            track.append((t, p.position.x, p.position.y, yaw(p.orientation)))
    (session / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
    started = next((e['time'] for e in events if e.get('event') == 'task_started'), None)
    finished = next((e for e in events if e.get('event') == 'task_finished'), None)
    outcome = dict(task=task, status=status.get('status'), events=len(events),
                   started=started is not None, finished=finished is not None,
                   elapsed_s=round(finished['time'] - started, 1) if finished and started else None)
    from mhseals_testing.analysis import referee
    from mhseals_testing.analysis.mission import score
    from mhseals_testing.profiles import PROFILE_DIR
    rubric_path = PROFILE_DIR.parent / 'missions' / f'{task}.yaml'
    truth_path = session / 'course_truth.json'
    if not rubric_path.exists() or not truth_path.exists() or not track:
        verdict = dict(task=task, verdict='N/A', reason='no rubric, course truth or /crane/odom track')
    else:
        rubric = yaml.safe_load(rubric_path.read_text())
        scored = score(events, rubric)
        outcome.update(outcome=scored['outcome'], score=scored['verdict'], missing=scored['missing'])
        verdict = referee.judge_task(task, json.loads(truth_path.read_text()), track, events, rubric)
    (session / 'referee.json').write_text(json.dumps(verdict, indent=1, default=str))
    outcome['referee'] = verdict.get('verdict')
    if 'stages' in verdict:
        outcome['stages'] = {name: stage.get('verdict') for name, stage in verdict['stages'].items()}
    print(json.dumps(outcome))


if __name__ == '__main__':
    main(*sys.argv[1:3])
