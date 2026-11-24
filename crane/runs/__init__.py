"""The local run index, its query language and reproduction (Docs/Runs.md).

    from crane import runs
    runs.Index().update(['~/.cache/crane/runs'])              # incremental (mtime, then sha256)
    runs.query('outcome=TIMEOUT and tier=T1 and metric.time>120')   # -> [row dicts]
    runs.find('3fa2c1')                                       # a run dir or a run_id prefix -> one row

index.py owns the SQLite file and reading records, query.py the --where language and the verbs, repro.py
`crane repro`.
"""

from crane.runs.index import Index, RunNotFound, find, read_dir   # noqa: F401
from crane.runs.query import query  # noqa: F401
