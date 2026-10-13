"""Subtractive Turning geometry in an explicit spindle coordinate frame."""
from __future__ import annotations

import math
from typing import Any


def finite(value: Any, name: str, *, positive: bool = False, nonnegative: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"The Turning {name} must be a finite dimension.")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"The Turning {name} must be a finite dimension.") from error
    if not math.isfinite(result) or (positive and result <= 0) or (nonnegative and result < 0):
        raise ValueError(f"The Turning {name} is not a valid dimension.")
    return result


def frame(axis: str, origin: Any = (0, 0, 0)) -> tuple[str, tuple[float, float, float]]:
    axis = str(axis).upper().removeprefix("+")
    if axis not in {"X", "Y", "Z", "-X", "-Y", "-Z"}:
        raise ValueError("The Turning spindle axis must be X, Y, or Z, with an optional sign.")
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("The Turning datum requires three coordinates.")
    return axis, tuple(finite(value, "datum") for value in origin)


def to_spindle(shape: Any, axis: str = "Z", origin: Any = (0, 0, 0)) -> Any:
    axis, origin = frame(axis, origin)
    result = shape.translate(tuple(-value for value in origin))
    rotations = {"X": ((0, 1, 0), -90), "-X": ((0, 1, 0), 90),
        "Y": ((1, 0, 0), 90), "-Y": ((1, 0, 0), -90), "-Z": ((1, 0, 0), 180)}
    if axis in rotations:
        direction, degrees = rotations[axis]
        result = result.rotate((0, 0, 0), direction, degrees)
    return result


def from_spindle(shape: Any, axis: str = "Z", origin: Any = (0, 0, 0)) -> Any:
    axis, origin = frame(axis, origin)
    rotations = {"X": ((0, 1, 0), 90), "-X": ((0, 1, 0), -90),
        "Y": ((1, 0, 0), -90), "-Y": ((1, 0, 0), 90), "-Z": ((1, 0, 0), -180)}
    if axis in rotations:
        direction, degrees = rotations[axis]
        shape = shape.rotate((0, 0, 0), direction, degrees)
    return shape.translate(origin)


def spindle_point_to_world(radius: float, axial: float, axis: str = "Z", origin: Any = (0, 0, 0)) -> tuple:
    axis, origin = frame(axis, origin)
    points = {"Z": (radius, 0, axial), "X": (axial, 0, -radius), "Y": (radius, axial, 0),
        "-Z": (radius, 0, -axial), "-X": (-axial, 0, radius), "-Y": (radius, -axial, 0)}
    return tuple(value + offset for value, offset in zip(points[axis], origin))


def checked(shape: Any, previous: Any) -> Any:
    if not shape.solids().vals() or not shape.val().isValid() or shape.val().Volume() <= 1e-9:
        raise ValueError("The Turning operation leaves no valid material.")
    if not math.isfinite(shape.val().Volume()):
        raise ValueError("The Turning operation produced invalid material measurements.")
    if shape.val().Volume() > previous.val().Volume() + max(previous.val().Volume(), 1) * 1e-8:
        raise ValueError("A Turning operation cannot add material to the incoming stock.")
    return shape


def radial_extent(shape: Any) -> float:
    bounds = shape.val().BoundingBox()
    return math.hypot(max(abs(bounds.xmin), abs(bounds.xmax)), max(abs(bounds.ymin), abs(bounds.ymax)))


def axial_band(shape: Any, lower: float, upper: float) -> Any:
    import cadquery as cq

    if upper <= lower:
        raise ValueError("The Turning operation needs a positive axial span.")
    radius = radial_extent(shape) + 1
    return cq.Workplane("XY", origin=(0, 0, lower)).circle(radius).extrude(upper - lower)


def cylinder(radius: float, lower: float, upper: float) -> Any:
    import cadquery as cq

    return cq.Workplane("XY", origin=(0, 0, lower)).circle(radius).extrude(upper - lower)


def face_cut(shape: Any, z: float, *, axis: str = "Z", origin: Any = (0, 0, 0), side: str = "front") -> Any:
    local = to_spindle(shape, axis, origin)
    bounds = local.val().BoundingBox()
    z = finite(z, "face position")
    if side == "front":
        if z >= bounds.zmax:
            return shape
        cut = axial_band(local, z, bounds.zmax + 1)
    elif side == "back":
        if z <= bounds.zmin:
            return shape
        cut = axial_band(local, bounds.zmin - 1, z)
    else:
        raise ValueError("The Turning face must use the front or back spindle side.")
    return checked(from_spindle(local.cut(cut), axis, origin), shape)


def diameter_cut(shape: Any, diameter: float, length: float, *, axis: str = "Z", origin: Any = (0, 0, 0),
                 start_z: float | None = None, internal: bool = False) -> Any:
    diameter = finite(diameter, "diameter", positive=True)
    length = finite(length, "length", positive=True)
    local = to_spindle(shape, axis, origin)
    front = local.val().BoundingBox().zmax if start_z is None else finite(start_z, "start position")
    target = cylinder(diameter / 2, front - length, front)
    cut = target if internal else axial_band(local, front - length, front).cut(target)
    return checked(from_spindle(local.cut(cut), axis, origin), shape)


def section_radius(shape: Any, z: float) -> float:
    import cadquery as cq

    section = cq.Workplane("XY").newObject([shape.val()]).section(z)
    radii = [edge.radius() for edge in section.edges().vals()
        if edge.geomType() == "CIRCLE" and math.hypot(edge.arcCenter().x, edge.arcCenter().y) < 1e-6]
    if not radii:
        raise ValueError("The groove needs an explicit diameter for this non-circular material section.")
    return max(radii)


def groove_cut(shape: Any, width: float, depth: float, z: float, *, axis: str = "Z", origin: Any = (0, 0, 0),
               diameter: float | None = None, internal: bool = False) -> Any:
    width = finite(width, "groove width", positive=True)
    depth = finite(depth, "groove depth", positive=True)
    z = finite(z, "groove position")
    local = to_spindle(shape, axis, origin)
    if internal and diameter is None:
        raise ValueError("An internal groove needs the bore diameter.")
    radius = finite(diameter, "groove diameter", positive=True) / 2 if diameter is not None else section_radius(local, z)
    if internal:
        cut = cylinder(radius + depth, z - width / 2, z + width / 2).cut(cylinder(radius, z - width / 2, z + width / 2))
    else:
        if depth >= radius:
            raise ValueError("The external groove depth reaches the spindle centre.")
        cut = axial_band(local, z - width / 2, z + width / 2).cut(cylinder(radius - depth, z - width / 2, z + width / 2))
    return checked(from_spindle(local.cut(cut), axis, origin), shape)


def profile_points(profile: Any, bounds: Any) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Return radius/axial contour points; diameters are explicit in station dictionaries."""
    inner = []
    if isinstance(profile, dict) and profile.get("type") in {"tapered_cylinder", "tapered_ring", "frustum"}:
        height = finite(profile.get("height", profile.get("height_mm", bounds.zlen)), "profile height", positive=True)
        centre = finite(profile.get("z_center", (bounds.zmin + bounds.zmax) / 2), "profile centre")
        bottom = finite(profile.get("bottom_diameter", profile.get("bottom_diameter_mm")), "bottom diameter", positive=True)
        top = finite(profile.get("top_diameter", profile.get("top_diameter_mm", bottom)), "top diameter", positive=True)
        points = [(bottom / 2, centre - height / 2), (top / 2, centre + height / 2)]
        inner_bottom = finite(profile.get("inner_bottom_diameter", profile.get("inner_bottom_diameter_mm", 0)), "inner diameter", nonnegative=True)
        inner_top = finite(profile.get("inner_top_diameter", profile.get("inner_top_diameter_mm", inner_bottom)), "inner diameter", nonnegative=True)
        if inner_bottom or inner_top:
            if inner_bottom >= bottom or inner_top >= top:
                raise ValueError("The inner profile reaches the outside profile.")
            inner = [(inner_bottom / 2, points[0][1]), (inner_top / 2, points[1][1])]
    else:
        raw = profile.get("stations", profile.get("points")) if isinstance(profile, dict) else profile
        if not isinstance(raw, (list, tuple)) or len(raw) < 2:
            raise ValueError("The Turning profile needs at least two radius/axial stations.")
        points = []
        for item in raw:
            if isinstance(item, dict):
                radius = item.get("radius_mm", item.get("radius"))
                if radius is None and item.get("diameter_mm", item.get("diameter")) is not None:
                    radius = finite(item.get("diameter_mm", item.get("diameter")), "station diameter", nonnegative=True) / 2
                axial = item.get("z_mm", item.get("z"))
            elif isinstance(item, (list, tuple)) and len(item) == 2:
                radius, axial = item
            else:
                raise ValueError("The Turning profile has an invalid station.")
            point = (finite(radius, "station radius", nonnegative=True), finite(axial, "station position"))
            if not points or point != points[-1]:
                points.append(point)
    if len(points) < 2 or max(point[1] for point in points) <= min(point[1] for point in points):
        raise ValueError("The Turning profile needs a positive axial span.")
    if not any(radius > 0 for radius, _axial in points):
        raise ValueError("The Turning profile needs a positive radius.")
    directions = {math.copysign(1, b[1] - a[1]) for a, b in zip(points, points[1:]) if a[1] != b[1]}
    if len(directions) > 1:
        raise ValueError("The Turning contour must follow one axial direction.")
    return points, inner


def revolved_envelope(points: list[tuple[float, float]]) -> Any:
    import cadquery as cq

    contour = [*points]
    for point in ((0, points[-1][1]), (0, points[0][1])):
        if point != contour[-1] and point != contour[0]:
            contour.append(point)
    return cq.Workplane("XZ").polyline(contour).close().revolve(360, (0, 0), (0, 1))


def profile_cut(shape: Any, profile: Any, *, axis: str = "Z", origin: Any = (0, 0, 0)) -> Any:
    local = to_spindle(shape, axis, origin)
    outer, inner = profile_points(profile, local.val().BoundingBox())
    target = revolved_envelope(outer)
    if inner:
        target = target.cut(revolved_envelope(inner))
    lower, upper = min(point[1] for point in outer), max(point[1] for point in outer)
    cut = axial_band(local, lower, upper).cut(target)
    return checked(from_spindle(local.cut(cut), axis, origin), shape)


def thread_cut(shape: Any, diameter: float, pitch: float, length: float, *, axis: str = "Z", origin: Any = (0, 0, 0),
               start_z: float | None = None, internal: bool = False, depth: float | None = None,
               angle: float = 60, left_handed: bool = False) -> Any:
    """Cut a generic helical V profile; thread-standard verification is separate."""
    import cadquery as cq

    diameter = finite(diameter, "thread diameter", positive=True)
    pitch = finite(pitch, "thread pitch", positive=True)
    length = finite(length, "thread length", positive=True)
    if length / pitch > 500:
        raise ValueError("The thread geometry needs a segmented preview for more than 500 turns.")
    if not isinstance(internal, bool) or not isinstance(left_handed, bool):
        raise ValueError("The thread side and hand must use boolean values.")
    depth = finite(depth if depth is not None else pitch / 2, "thread depth", positive=True)
    angle = finite(angle, "thread angle", positive=True)
    if angle >= 180 or depth >= diameter / 2:
        raise ValueError("The thread profile is outside the available material.")
    half_width = depth * math.tan(math.radians(angle / 2))
    if 2 * half_width >= pitch:
        raise ValueError("The thread profile overlaps the next pitch.")
    local = to_spindle(shape, axis, origin)
    front = local.val().BoundingBox().zmax if start_z is None else finite(start_z, "thread start")
    lower = front - length
    major, minor = diameter / 2, diameter / 2 - depth
    band = axial_band(local, lower, front)
    if internal:
        bore_material = local.intersect(cylinder(minor - 1e-6, lower, front)).val().Volume()
        if bore_material > 1e-5:
            raise ValueError("Internal threading needs a preceding bore at the required minor diameter and depth.")
    elif local.intersect(band.cut(cylinder(major + 1e-6, lower, front))).val().Volume() > 1e-5:
        raise ValueError("External threading needs preceding outside-diameter machining.")
    padding = min(pitch * 0.02, 0.05)
    start = lower - pitch
    if internal:
        points = [(minor - padding, start - half_width), (major, start), (minor - padding, start + half_width)]
    else:
        points = [(major + padding, start - half_width), (minor, start), (major + padding, start + half_width)]
    helix = cq.Wire.makeHelix(pitch, length + 2 * pitch, (major + minor) / 2,
        center=cq.Vector(0, 0, start), lefthand=bool(left_handed))
    cutter = cq.Workplane("XZ").polyline(points).close().sweep(helix, isFrenet=True).intersect(band)
    return checked(from_spindle(local.cut(cutter), axis, origin), shape)
