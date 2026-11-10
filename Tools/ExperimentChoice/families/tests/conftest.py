"""Makes ``families`` and the flat parent modules importable under `python3 -I -m pytest`."""
import os
import sys

_EC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _EC not in sys.path:
    sys.path.insert(0, _EC)
