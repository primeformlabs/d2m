"""Native pocket profiles for machining review, independent of fillet fallbacks."""
import math
from typing import Any


def _dimension(value: Any, name: str, *, zero: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"The pocket {name} must be a finite dimension.")
    number = float(value)
    if not math.isfinite(number) or (number < 0 if zero else number <= 0):
        raise ValueError(f"The pocket {name} must be {'non-negative' if zero else 'positive'} and finite.")
    return number


def rounded_rectangle(workplane: Any, length: float, width: float, radius: float) -> Any:
    """Build true circular corner arcs before the cut; never fillet the result."""
    length = _dimension(length, "length")
    width = _dimension(width, "width")
    radius = _dimension(radius, "corner radius", zero=True)
    if radius > min(length, width) / 2:
        raise ValueError("The pocket corner radius exceeds half of the pocket opening.")
    if radius == 0:
        return workplane.rect(length, width)
    half_x, half_y = length / 2, width / 2
    diagonal = radius / math.sqrt(2)
    start = (-half_x + radius, -half_y)
    corners = [
        ((half_x - radius, -half_y), (half_x - radius + diagonal, -half_y + radius - diagonal), (half_x, -half_y + radius)),
        ((half_x, half_y - radius), (half_x - radius + diagonal, half_y - radius + diagonal), (half_x - radius, half_y)),
        ((-half_x + radius, half_y), (-half_x + radius - diagonal, half_y - radius + diagonal), (-half_x, half_y - radius)),
        ((-half_x, -half_y + radius), (-half_x + radius - diagonal, -half_y + radius - diagonal), start),
    ]
    profile = workplane.moveTo(*start)
    current = start
    for line_end, arc_middle, arc_end in corners:
        if math.dist(current, line_end) > 1e-9:
            profile = profile.lineTo(*line_end)
        profile = profile.threePointArc(arc_middle, arc_end)
        current = arc_end
    return profile.close()


def rectangular_pocket_cut(shape: Any, *, width: float, length: float, depth: float,
                           corner_radius: float = 0, cx: float = 0, cy: float = 0,
                           face_selector: str = ">Z", base_height: float | None = None) -> Any:
    """Cut a rounded prism in stable stock coordinates on any cardinal face.

    X-face coordinates are Y/Z; Y-face coordinates are X/Z; Z-face coordinates
    are X/Y. Previous cuts must not move the profile's coordinate origin.
    """
    import cadquery as cq

    width = _dimension(width, "width")
    length = _dimension(length, "length")
    depth = _dimension(depth, "depth")
    corner_radius = _dimension(corner_radius, "corner radius", zero=True)
    if not all(math.isfinite(float(value)) and not isinstance(value, bool) for value in (cx, cy)):
        raise ValueError("The pocket position must use finite coordinates.")
    bounds = shape.val().BoundingBox()
    if face_selector in {">Z", "<Z"}:
        origin = bounds.zmax - depth if face_selector == ">Z" else bounds.zmin
        if base_height is not None:
            if face_selector != ">Z":
                raise ValueError("A pocket base height requires the positive Z stock datum.")
            origin = bounds.zmin + _dimension(base_height, "base height", zero=True)
            depth = min(depth, bounds.zmax - origin)
            if depth <= 0:
                raise ValueError("The pocket base height is outside the remaining stock.")
        plane = cq.Workplane("XY", origin=(float(cx), float(cy), origin))
    elif base_height is not None:
        raise ValueError("A pocket base height requires the positive Z stock datum.")
    elif face_selector in {">X", "<X"}:
        origin = bounds.xmax - depth if face_selector == ">X" else bounds.xmin
        plane = cq.Workplane("YZ", origin=(origin, float(cx), float(cy)))
    elif face_selector in {">Y", "<Y"}:
        origin = bounds.ymax if face_selector == ">Y" else bounds.ymin + depth
        plane = cq.Workplane("XZ", origin=(float(cx), origin, float(cy)))
    else:
        raise ValueError("The pocket needs a supported stock face and datum.")
    cutter = rounded_rectangle(plane, length, width, corner_radius).extrude(depth)
    result = shape.cut(cutter)
    if not result.val().isValid() or not result.solids().vals():
        raise ValueError("The pocket cut did not produce valid remaining material.")
    return result


