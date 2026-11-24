"""RoboBoat course pack: the referee (per-task PASS/FAIL/N/A from the course truth and the true track)
and its judge Pack.

    python3 -m judge.packs.roboboat_course SESSION_DIR [TASK...]   # or TASK = mission (with --rubric)

``referee`` needs only the stdlib and ``Tools/RoboBoat/course_truth.py``; ``pack.PACK`` wraps it
for RunRecords (a ``pose.se2`` channel and the course truth in ``rec.truth``).
"""

from judge.packs.roboboat_course.referee import judge, judge_mission, judge_session, score

__all__ = ['judge', 'judge_mission', 'judge_session', 'score']
