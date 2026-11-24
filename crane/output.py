"""crane's output conventions: plain aligned text, one-line failures that name the fix, `next:` lines, --json.

    out = Output.of(args)              # --json, --quiet, --dry-run as the command was given them
    out.table([('python', 'ok', '3.12')])
    out.fail('run', 'no seeds given', 'crane run --domain toy --seeds 0:3')   # stderr, returns 1
    out.next('crane board ~/.cache/crane/runs')
    out.doc({'schema': 'crane.doctor/1', ...})                   # with --json: one document on stdout

Exit codes: 0 ok, 1 a check or run failed (judge FAIL included), 2 usage or config, 75 a bounded wait timed out.
"""

import json
import math
import sys

OK, FAILED, USAGE, TIMEOUT = 0, 1, 2, 75
GLOBALS = ('json', 'quiet', 'dry_run')


def _default(o):
    if hasattr(o, 'tolist'):
        return o.tolist()
    if hasattr(o, 'item'):
        return o.item()
    if hasattr(o, '__fspath__'):
        return str(o)
    if isinstance(o, (set, frozenset, tuple)):
        return list(o)
    return str(o)


def finite(o):
    """o with NaN/inf floats as None (strict JSON has neither)."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [finite(v) for v in o]
    return o


def dumps(doc, indent=1):
    return json.dumps(finite(doc), indent=indent, default=_default)


def table(rows, indent=2, gap=2):
    """Rows of cells as aligned lines (the last column is not padded)."""
    rows = [[str(c) for c in row] for row in rows]
    if not rows:
        return []
    widths = [max(len(r[i]) for r in rows if i < len(r)) for i in range(max(map(len, rows)))]
    pad = ' ' * gap
    return [' ' * indent + pad.join(c if i == len(r) - 1 else c.ljust(widths[i]) for i, c in enumerate(r)).rstrip()
            for r in rows]


def failure(what, cause, fix=None):
    """'crane NOUN VERB: cause; fix: CMD' (one line)."""
    return f'crane {what}: {cause}' + (f'; fix: {fix}' if fix else '')


class Output:
    """Where a command writes: text for people, or one JSON document (JSONL for streams) for machines."""

    def __init__(self, json=False, quiet=False, dry_run=False, stdout=None, stderr=None):
        self.json, self.quiet, self.dry_run = bool(json), bool(quiet), bool(dry_run)
        self._out, self._err = stdout, stderr

    @classmethod
    def of(cls, args):
        return cls(**{k: getattr(args, k, False) for k in GLOBALS})

    @property
    def stdout(self):
        return self._out or sys.stdout

    @property
    def stderr(self):
        return self._err or sys.stderr

    def line(self, text=''):
        """A text line (dropped with --json, where the document carries it, and with --quiet)."""
        if not self.json and not self.quiet:
            print(text, file=self.stdout)

    def say(self, text=''):
        """A text line kept under --quiet (a result or a verdict); dropped with --json."""
        if not self.json:
            print(text, file=self.stdout)

    def table(self, rows, indent=2):
        for line in table(rows, indent):
            self.line(line)

    def next(self, *cmds):
        """'next: CMD' (one command) or 'next:' and one indented command per line."""
        cmds = [c for c in cmds if c]
        if not cmds or self.json or self.quiet:
            return
        if len(cmds) == 1:
            print(f'next: {cmds[0]}', file=self.stdout)
        else:
            print('next:', file=self.stdout)
            for c in cmds:
                print(f'  {c}', file=self.stdout)

    def fail(self, what, cause, fix=None, code=FAILED):
        """The one-line failure on stderr (also under --json, whose stdout stays a document); returns code."""
        print(failure(what, cause, fix), file=self.stderr)
        return code

    def usage(self, what, cause, fix=None):
        return self.fail(what, cause, fix, USAGE)

    def doc(self, doc):
        """With --json, the one JSON document on stdout. Returns whether it was written."""
        if self.json:
            print(dumps(doc), file=self.stdout)
        return self.json

    def record(self, doc):
        """With --json, one JSONL record (a stream: one line per thing)."""
        if self.json:
            print(json.dumps(finite(doc), default=_default, separators=(',', ':')), file=self.stdout, flush=True)
        return self.json
