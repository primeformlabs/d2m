"""Program mode: toolpaths without the per-pass material replay.

A price needs the toolpath of each operation and its time, not the material state after each
cutter pass. In program mode the milling operations keep their complete toolpath (the same moves
as the replay), but the material result of an operation is one cut of its target region, and no
pass states, pass reports or per-pass shape checks are made. D2O uses it for the program time of
a quote (D2O #729); the process review keeps the full replay.
"""
from contextlib import contextmanager
from contextvars import ContextVar

_PROGRAM_MODE: ContextVar[bool] = ContextVar("subcad_program_mode", default=False)


def is_program_mode() -> bool:
    return _PROGRAM_MODE.get()


@contextmanager
def program_mode(enabled: bool = True):
    """Make the toolpaths of the operations in this block without the per-pass replay."""
    token = _PROGRAM_MODE.set(enabled)
    try:
        yield
    finally:
        _PROGRAM_MODE.reset(token)


_FEEDS_SPEEDS: ContextVar = ContextVar("subcad_feeds_speeds_provider", default=None)


def feeds_speeds_provider():
    return _FEEDS_SPEEDS.get()


@contextmanager
def feeds_speeds(provider):
    """Use the caller's cutting data in this block.

    ``provider(tool_type, tool_diameter_mm, material)`` returns the ``get_feeds_speeds`` dict
    (``rpm``, ``feed_rate_mm_min`` and the other keys), or None for the built-in handbook."""
    token = _FEEDS_SPEEDS.set(provider)
    try:
        yield
    finally:
        _FEEDS_SPEEDS.reset(token)
