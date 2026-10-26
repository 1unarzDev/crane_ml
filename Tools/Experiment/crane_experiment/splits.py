"""Held-out splits are by whole site or whole session, never a later slice of a run."""
from .errors import SplitError

ALLOWED_BY = ("site", "session")


def make_split(contexts, by, heldout_groups):
    """Return (train, heldout) context lists, holding out whole `by` groups."""
    if by not in ALLOWED_BY:
        raise SplitError(f"split by {by!r} refused: only whole site/session splits "
                         "(time-slice splits leak the same run into held-out data)")
    held = set(heldout_groups)
    train = [c for c in contexts if c[by] not in held]
    test = [c for c in contexts if c[by] in held]
    validate_split(train, test)
    return train, test


def validate_split(train, heldout):
    if not train or not heldout:
        raise SplitError("need at least one train and one held-out context")
    for c in list(train) + list(heldout):
        if c.get("split_method", "site") not in ALLOWED_BY:
            raise SplitError(f"context {c['id']}: split_method {c.get('split_method')!r} "
                             "is not a whole-site/session split")
    for key in ("site", "session"):
        a = {c[key] for c in train}
        b = {c[key] for c in heldout}
        if a & b:
            raise SplitError(f"{key} overlap between train and held-out: {sorted(a & b)}")
    ids = [c["id"] for c in list(train) + list(heldout)]
    if len(ids) != len(set(ids)):
        raise SplitError("duplicate context ids")
