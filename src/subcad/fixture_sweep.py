"""Continuous cardinal tool-envelope checks against part-coordinate boxes.

The geometry is a declared cylindrical envelope, not an exact holder model.
The axial interval and radial rectangle distance are checked over the complete
linear segment. No sampling interval can hide a thin fixture zone.
"""

from __future__ import annotations

import math


FACE_AXES = {
    ">X": (1, 0, 0), "<X": (-1, 0, 0),
    ">Y": (0, 1, 0), "<Y": (0, -1, 0),
    ">Z": (0, 0, 1), "<Z": (0, 0, -1),
}


def cardinal_tool_axis(op: dict) -> tuple[int, int, int]:
    path = op.get("toolpath")
    metadata = path.get("metadata", {}) if isinstance(path, dict) else {}
    if not isinstance(metadata, dict):
        raise ValueError("The toolpath metadata is invalid.")
    frame = metadata.get("coordinate_system")
    if frame not in (None, "", "part_mm"):
        raise ValueError("Fixture checks require a toolpath in part_mm coordinates.")
    explicit = metadata.get("tool_axis")
    if explicit is not None:
        if not isinstance(explicit, (list, tuple)) or len(explicit) != 3:
            raise ValueError("The cutter axis is invalid.")
        try:
            axis = tuple(float(value) for value in explicit)
        except (TypeError, ValueError):
            raise ValueError("The cutter axis is invalid.") from None
        if axis not in FACE_AXES.values():
            raise ValueError("Fixture checks support only six cardinal cutter axes.")
        face_axis = FACE_AXES.get(op.get("face_selector"))
        if face_axis is not None and axis != face_axis:
            raise ValueError("The cutter axis does not match the operation face.")
        return tuple(int(value) for value in axis)
    face = op.get("face_selector", ">Z")
    if face not in FACE_AXES:
        raise ValueError("Fixture checks require a supported operation face.")
    return FACE_AXES[face]


def _radial_minimum(start, delta, ranges, low, high):
    """Minimize squared distance to a rectangle over a segment interval."""
    breaks = [low, high]
    for value, change, (minimum, maximum) in zip(start, delta, ranges):
        if change:
            for boundary in (minimum, maximum):
                t = (boundary - value) / change
                if low < t < high:
                    breaks.append(t)
    breaks = sorted(set(breaks))
    candidates = list(breaks)
    for a, b in zip(breaks, breaks[1:]):
        midpoint = (a + b) / 2
        quadratic = linear = 0.0
        for value, change, (minimum, maximum) in zip(start, delta, ranges):
            position = value + change * midpoint
            boundary = minimum if position < minimum else maximum if position > maximum else None
            if boundary is not None:
                quadratic += change * change
                linear += change * (value - boundary)
        if quadratic:
            candidates.append(max(a, min(b, -linear / quadratic)))

    def distance_squared(t):
        return sum(max(minimum - (value + change * t), 0.0,
                       value + change * t - maximum) ** 2
                   for value, change, (minimum, maximum) in zip(start, delta, ranges))

    best = min(candidates, key=distance_squared)
    return best, math.sqrt(distance_squared(best))


def segment_envelope_hit(start, end, axis, envelope, bounds, margin=0.0):
    """Return a possible collision with the declared cylinder, or None.

    Cutter axis points from the tip towards the holder. An isotropic margin
    expands both axial limits and the radial envelope conservatively.
    """
    axial = next(index for index, value in enumerate(axis) if value)
    sign = axis[axial]
    radial = [index for index in range(3) if index != axial]
    keys = ("x", "y", "z")
    offsets = sorted((sign * envelope["z_start_mm"], sign * envelope["z_end_mm"]))
    permitted_min = bounds[keys[axial] + "_min"] - offsets[1] - margin
    permitted_max = bounds[keys[axial] + "_max"] - offsets[0] + margin
    change = end[axial] - start[axial]
    low, high = 0.0, 1.0
    if change:
        first, last = sorted(((permitted_min - start[axial]) / change,
                              (permitted_max - start[axial]) / change))
        low, high = max(low, first), min(high, last)
        if low > high:
            return None
    elif not permitted_min <= start[axial] <= permitted_max:
        return None
    ranges = [(bounds[keys[index] + "_min"], bounds[keys[index] + "_max"])
              for index in radial]
    t, distance = _radial_minimum(
        [start[index] for index in radial],
        [end[index] - start[index] for index in radial], ranges, low, high,
    )
    clearance = distance - envelope["radius_mm"] - margin
    if clearance > 1e-9:
        return None
    point = [start[index] + (end[index] - start[index]) * t for index in range(3)]
    return {
        "point": [round(value, 6) for value in point],
        "segment_fraction": t,
        "clearance_mm": round(min(clearance, 0.0), 6),
        "component_axis_range_mm": [round(point[axial] + value, 6) for value in offsets],
    }
