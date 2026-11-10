"""Puts this study directory and Tools/DecisionCal on sys.path for its tests."""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
for p in (HERE, os.path.abspath(os.path.join(HERE, '..', '..'))):
    if p not in sys.path:
        sys.path.insert(0, p)
