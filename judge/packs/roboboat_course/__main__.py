"""python3 -m judge.packs.roboboat_course SESSION_DIR [TASK...] [--rubric RUBRIC.json]: verdicts as JSON."""

import argparse
import json
import sys
from pathlib import Path

from judge.packs.roboboat_course.referee import judge_session, score


def main(argv=None):
    p = argparse.ArgumentParser(prog='python3 -m judge.packs.roboboat_course', description=__doc__)
    p.add_argument('session', help='directory with course_truth.json, track.jsonl, events.jsonl')
    p.add_argument('tasks', nargs='*', help='task ids or types (default: every task on the course), or mission')
    p.add_argument('--rubric', help='JSON rubric: tasks/required_stages for mission, required/points for the event score')
    args = p.parse_args(argv)
    rubric = json.loads(Path(args.rubric).read_text()) if args.rubric else None
    out = {'referee': judge_session(args.session, args.tasks or None, rubric)}
    if rubric and rubric.get('points'):
        events = [json.loads(line) for line in (Path(args.session) / 'events.jsonl').read_text().splitlines() if line.strip()]
        out['score'] = score(events, rubric)
    print(json.dumps(out, indent=1))
    verdicts = [out['referee']['verdict']] if 'verdict' in out['referee'] else [v['verdict'] for v in out['referee'].values()]
    return 1 if 'FAIL' in verdicts else 0


if __name__ == '__main__':
    sys.exit(main())
