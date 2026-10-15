"""Native drill cutters in stable part coordinates, including curved stock."""
import math
from typing import Any


def drill_cut(shape: Any, cx: float, cy: float, diameter: float, depth: float,
              through: bool = False, *, face_selector: str = ">Z") -> Any:
    import cadquery as cq

    values = {"cx": cx, "cy": cy, "diameter": diameter, "depth": depth}
    for name, value in values.items():
        if isinstance(value, bool) or not math.isfinite(float(value)):
            raise ValueError(f"The drill {name} must be finite.")
    if float(diameter) <= 0 or (not through and float(depth) <= 0) or float(depth) < 0:
        raise ValueError("The drill diameter and blind depth must be positive.")
    if not isinstance(through, bool):
        raise ValueError("The drill through flag must use a boolean value.")
    if face_selector not in {">X", "<X", ">Y", "<Y", ">Z", "<Z"}:
        raise ValueError("The drill needs a supported part access direction.")
    bounds = shape.val().BoundingBox()
    axis = face_selector[1].lower()
    lower, upper = getattr(bounds, axis + "min"), getattr(bounds, axis + "max")
    padding = 1.0
    if through:
        start, length = lower - padding, upper - lower + 2 * padding
    elif face_selector[0] == ">":
        start, length = upper - float(depth), float(depth) + padding
    else:
        start, length = lower - padding, float(depth) + padding
    if axis == "z":
        plane = cq.Workplane("XY", origin=(float(cx), float(cy), start))
    elif axis == "x":
        plane = cq.Workplane("YZ", origin=(start, float(cx), float(cy)))
    else:
        # XZ's normal is negative Y. Start at the upper end of the cutter.
        plane = cq.Workplane("XZ", origin=(float(cx), start + length, float(cy)))
    result = shape.cut(plane.circle(float(diameter) / 2).extrude(length))
    if not result.val().isValid() or not result.solids().vals():
        raise ValueError("The drill did not produce valid remaining material.")
    return result
