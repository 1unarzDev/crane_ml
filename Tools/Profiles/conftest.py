import sys
from pathlib import Path

sys.dont_write_bytecode = True  # never write __pycache__ into the read-only sibling tool directories
HERE = Path(__file__).resolve().parent
TOOLS = HERE.parent
for p in (HERE, HERE / "tests", TOOLS / "Underwater", TOOLS / "Aerial", TOOLS / "Terrain", TOOLS / "Campus"):
    if str(p) not in sys.path:
        sys.path.append(str(p))
