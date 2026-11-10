"""Puts Tools/DecisionCal on sys.path so `python3 -I -m pytest Tools/DecisionCal -q` imports the flat
modules. No network, ROS or mhseals imports (tests/test_no_mhseals.py enforces it)."""
import os
import sys

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
