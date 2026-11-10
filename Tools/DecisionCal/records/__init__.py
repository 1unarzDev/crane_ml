"""DecisionCal run-record writers (``crane.judge.run/2`` layout) for three backends.

  numpy_capture     the fast numpy plant (``mission.run`` unedited, plant instrumented by a subclass)
  lockstep_capture  the Unity player over the lockstep link (or unity_check's fake player)
  physical_pulse    the 2026-10-18 open-loop pulse trials (physical log replay, read-only)
  capture           CLI: ``python3 Tools/DecisionCal/records/capture.py numpy|lockstep|physical ... --out DIR``

All three build the same intermediate structure (``_common.Rec``) and write it through ``_compat``,
which uses ``judge.record``'s ``/2`` constructors when they exist and otherwise a deterministic
npz + json in the same field layout (``convert`` turns those into real records later).
"""

import sys
from pathlib import Path

_DC = str(Path(__file__).resolve().parents[1])     # Tools/DecisionCal (flat harness modules)
if _DC not in sys.path:
    sys.path.insert(0, _DC)

import _paths  # noqa: E402,F401  (puts the crane root on sys.path for ``judge`` and pins nlc)
