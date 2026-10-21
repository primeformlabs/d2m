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
