import sys
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))   # crane_archive
sys.path.insert(0, str(HERE.parents[2]))   # crane_cluster (miniyaml)
