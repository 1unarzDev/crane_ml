"""``python3 -m crane``: works from a bare checkout (the repo root on sys.path), no install needed."""

import sys

from crane.cli import main

sys.exit(main())
