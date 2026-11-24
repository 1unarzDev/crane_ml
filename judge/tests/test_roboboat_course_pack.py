"""The roboboat_course pack over a RunRecord built from a generated referee fixture."""

import json
from pathlib import Path

import numpy as np

from judge.pack import evaluate, outcome_of
from judge.packs.roboboat_course.pack import PACK
from judge.record import Header, Robot, RunRecord, se2_channel

CASES = Path(__file__).resolve().parents[2] / 'Tools' / 'RoboBoat' / 'tests' / 'referee'


def record(case):
    rows = [json.loads(line) for line in (CASES / case / 'track.jsonl').read_text().splitlines() if line.strip()]
    t, x, y, yaw = (np.array([r[k] for r in rows], float) for k in ('t', 'x', 'y', 'yaw'))
    header = Header(case, 'T2', 'fixture', 'roboboat', Robot('boat'), window=(float(t[0]), float(t[-1]), 'end'))
    truth = json.loads((CASES / case / 'course_truth.json').read_text())
    return RunRecord(header, {'pose.se2': se2_channel(t, x, y, yaw)}, truth=truth)


def test_pack_scores_tasks_and_contacts_and_names_the_outcome():
    clean, strike = record('entry_out_and_back'), record('contact_strike')
    v = evaluate(clean, packs=[PACK])
    assert v['course.tasks_passed'].extra['passed'] == ['entry_gates']
    assert v['course.contacts'].value == 0 and outcome_of(clean, v, [PACK]) == 'task_failed'
    v = evaluate(strike, packs=[PACK])
    assert v['course.contacts'].extra['contacts'] == ['entry_gates.gate1.port']
    assert outcome_of(strike, v, [PACK]) == 'contact'
