"""Puts this directory on sys.path so `python -I -m pytest` can import crane_experiment."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
