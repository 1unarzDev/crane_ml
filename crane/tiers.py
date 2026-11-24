"""The fidelity ladder: T0 abstract, T1 dynamic, T2 unity. One spelling in, one label out."""

NAMES = {'T0': 'abstract', 'T1': 'dynamic', 'T2': 'unity'}
_ALIASES = {**{k.lower(): k for k in NAMES}, **{v: k for k, v in NAMES.items()}, '0': 'T0', '1': 'T1', '2': 'T2'}


def normalise(tier):
    """'T1', 't1', '1' or 'dynamic' -> 'T1'. Anything else raises ValueError naming the choices."""
    key = str(tier).strip().lower()
    if key in _ALIASES:
        return _ALIASES[key]
    hint = ' (L0-L7 are the boat control layers, not tiers)' if key[:1] == 'l' and key[1:].isdigit() else ''
    choices = ', '.join(f'{k}|{v}' for k, v in NAMES.items())
    raise ValueError(f'unknown tier {tier!r}{hint}; choose one of {choices}')
