"""Puts Tools/Characterization on sys.path so `python3 -I -m pytest Tools/Characterization -q` from
the worktree root imports the flat modules (plant, gate, ...). No network, ROS or mhseals imports.

The ``accuracy`` fixture records measured errors; they are printed at the end of the run so the
numbers behind every accuracy check (including the ones past the design limit) are in the output."""
import os
import sys

import pytest

sys.dont_write_bytecode = True  # no __pycache__ for the test modules next to the Unity project
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

_RECORDS = []


@pytest.fixture
def accuracy():
    def record(check, dt, err, verdict, note=""):
        _RECORDS.append((check, dt, err, verdict, note))
    return record


def pytest_terminal_summary(terminalreporter):
    if not _RECORDS:
        return
    tr = terminalreporter
    tr.section("plant accuracy (relative error vs reference)")
    for check, dt, err, verdict, note in sorted(_RECORDS, key=lambda r: (r[0], r[1] or 0.0)):
        dts = "-" if dt is None else f"{dt:.3f}"
        tr.write_line(f"{check:<34} dt={dts:<6} err={err:+.2e}  {verdict:<9} {note}")
