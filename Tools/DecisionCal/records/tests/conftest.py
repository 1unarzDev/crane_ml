import sys
from pathlib import Path

sys.dont_write_bytecode = True
DC = str(Path(__file__).resolve().parents[2])        # Tools/DecisionCal
if DC not in sys.path:
    sys.path.insert(0, DC)
