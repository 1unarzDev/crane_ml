"""Header-block accessor that works before and after ``crane.judge.run/2``: the header attribute when
the schema carries it, else ``provenance[name]`` (or ``provenance['v2'][name]``, the writers' /1
fallback). The rule lives in core (``judge.insight.layers.header_block``) because the divergence and
selection modules need it too; this module is the pack's name for it."""

from judge.insight.layers import header_block


def backend(rec):
    return header_block(rec, 'backend')


def controller(rec):
    return header_block(rec, 'controller')


def clock_mode(rec):
    return header_block(rec, 'clock_mode')


__all__ = ['backend', 'clock_mode', 'controller', 'header_block']
