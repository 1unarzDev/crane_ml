import sys
from pathlib import Path

if __package__ in (None, ""):  # `python -I Tools/Profiles/crane_profiles`: -I drops PYTHONPATH and cwd
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from crane_profiles.cli import main
else:
    from .cli import main

sys.exit(main())
