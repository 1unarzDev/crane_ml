"""Environment families, robot-configuration space, interaction features, held-out splits (EXPLORATORY).

New package next to the offset-doorway modules. It imports ``geometry``, ``mechanics``, ``course``
and ``robots`` from the parent directory and changes none of them. numpy only.
"""
import os
import sys

_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)

LABEL = "exploratory"
