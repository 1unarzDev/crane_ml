import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # the crane root
sys.dont_write_bytecode = True
for p in (ROOT, ROOT / 'Tools' / 'Characterization'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
