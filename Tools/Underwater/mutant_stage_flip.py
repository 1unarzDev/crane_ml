"""Mutation check: reinsert the per-RK4-stage hemisphere flip and confirm the regression tests FAIL.

Usage: python -I Tools/Underwater/mutant_stage_flip.py
Exit 0 = mutant killed (tests failed as required); exit 1 = mutant survived.
"""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
GOOD = "quat_normalize(q_new) if canonical else quat_unit(q_new)"
BAD = "quat_normalize(q_new)"


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp) / "uw"
        shutil.copytree(HERE, work, ignore=shutil.ignore_patterns("__pycache__", "golden"))
        src = (work / "fossen6dof.py").read_text()
        assert GOOD in src, "fix not present"
        (work / "fossen6dof.py").write_text(src.replace(GOOD, BAD))
        r = subprocess.run([sys.executable, "-I", "-m", "pytest", "-q", "-k", "constant_rate or equivariance_holds", str(work)],
                           capture_output=True, text=True)
        print(r.stdout[-1500:])
        killed = r.returncode != 0
        print("MUTANT KILLED" if killed else "MUTANT SURVIVED")
        return 0 if killed else 1


if __name__ == "__main__":
    sys.exit(main())
