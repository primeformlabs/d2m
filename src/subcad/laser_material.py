"""Material removal from compensated XY laser paths, without controller qualification."""
from dataclasses import dataclass
import math

from .milling_replay import _boolean, finite


@dataclass
class LaserMaterialPass:
    kind: str
    index: int
    points_mm: list
    before: object
    after: object
    removed_volume_mm3: float


@dataclass
class LaserMaterialReplay:
    material: object
    passes: list[LaserMaterialPass]
    report: dict


def _material(source, name="incoming material"):
    import cadquery as cq
    source = getattr(source, "native", source)
    source = getattr(source, "_shape", source)
    if isinstance(source, cq.Workplane):
        values = source.vals()
        if not values or any(not isinstance(value, cq.Shape) for value in values):
            raise ValueError(f"The {name} requires native material solids.")
        source = values[0] if len(values) == 1 else cq.Compound.makeCompound(values)
    if not isinstance(source, cq.Shape) or not source.isValid() or not source.Solids():
        raise ValueError(f"The {name} requires valid closed native material.")
    for solid in source.Solids():
        finite(solid.Volume(), f"{name} volume", positive=True)
    return source


def _points(path):
    if not isinstance(path, (list, tuple)) or not 2 <= len(path) <= 5001:
        raise ValueError("The laser path requires 2 to 5001 explicit XY points.")
    points = []
    for point in path:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("The laser path requires two millimetre coordinates per point.")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in point):
            raise ValueError("The laser path coordinates must be finite numbers.")
        parsed = tuple(finite(value, "laser path coordinate") for value in point)
        if points and math.dist(points[-1], parsed) <= 1e-9:
            raise ValueError("The laser path contains a repeated consecutive point.")
        points.append(parsed)
    return points


def _disk(point, radius, bottom, height):
    import cadquery as cq
    return cq.Solid.makeCylinder(radius, height, cq.Vector(point[0], point[1], bottom))


def _segment(first, second, radius, bottom, height):
    import cadquery as cq
    dx, dy = second[0] - first[0], second[1] - first[1]
    span = math.hypot(dx, dy)
    rectangle = cq.Solid.makeBox(span, radius * 2, height, cq.Vector(0, -radius, 0))
    rectangle = rectangle.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(dy, dx)))
    rectangle = rectangle.translate((first[0], first[1], bottom))
    return _boolean(rectangle, [_disk(first, radius, bottom, height),
        _disk(second, radius, bottom, height)], "fuse")


def laser_selection_region(outer_mm, holes_mm, *, thickness_mm, bottom_z_mm):
    """Build an explicit reference region for selecting a released material piece."""
    import cadquery as cq
    thickness = finite(thickness_mm, "selection thickness", positive=True)
    bottom = finite(bottom_z_mm, "selection bottom")

    def prism(ring):
        points = _points(ring)
        if points[-1] == points[0]:
            points = points[:-1]
        if len(points) < 3:
            raise ValueError("The selection region needs a closed polygon with at least three points.")
        native = cq.Workplane("XY").polyline(points).close().extrude(thickness).translate((0,0,bottom))
        return _material(native, "selection polygon")

    outer = region = prism(outer_mm)
    if not isinstance(holes_mm, (list, tuple)):
        raise ValueError("The selection holes must be an explicit list.")
    for ring in holes_mm:
        hole = prism(ring)
        common = _boolean(outer, [hole], "common")
        if abs(common.Volume() - hole.Volume()) > max(1e-7, hole.Volume() * 1e-10):
            raise ValueError("A selection hole lies outside the supplied outer polygon.")
        region = _material(_boolean(region, [hole], "cut"), "part selection region")
    return region


def replay_laser_material(source, path_mm, *, kerf_width_mm, units, coordinate_system):
    """Subtract the beam footprint along a caller-owned compensated path.

    Paths are not offset, closed, reordered, or replaced here. The caller owns
    compensation and its lead/pierce checks. Loose pieces remain until release.
    """
    if units != "mm" or coordinate_system != "stock_local_mm":
        raise ValueError("Laser replay requires millimetres in the declared stock-local frame.")
    if isinstance(kerf_width_mm, bool) or not isinstance(kerf_width_mm, (int, float)):
        raise ValueError("The laser kerf width must be a positive finite number.")
    kerf = finite(kerf_width_mm, "laser kerf width", positive=True)
    points = _points(path_mm)
    material = original = _material(source)
    bounds = material.BoundingBox()
    height = finite(bounds.zmax - bounds.zmin, "incoming material thickness", positive=True)
    # The overshoot reaches both material surfaces. It is not a part tolerance.
    bottom, through_height = bounds.zmin - 1e-4, height + 2e-4
    passes = []
    tools = [_disk(points[0], kerf / 2, bottom, through_height)]
    tools.extend(_segment(a, b, kerf / 2, bottom, through_height) for a, b in zip(points, points[1:]))
    for index, footprint in enumerate(tools):
        before = material
        material = _material(_boolean(before, [footprint], "cut"), "laser result")
        removed = before.Volume() - material.Volume()
        if removed < -max(1e-7, abs(before.Volume()) * 1e-12):
            raise ValueError("The laser operation added material. The native cut requires review.")
        passes.append(LaserMaterialPass("pierce" if index == 0 else "cut", index,
            [points[0]] if index == 0 else [points[index - 1], points[index]], before, material, max(removed, 0)))
    removed = original.Volume() - material.Volume()
    report = {
        "schema_version": "laser-material-replay/v1", "units": "mm", "coordinate_system": coordinate_system,
        "kerf_width_mm": kerf, "path_mm": [list(point) for point in points],
        "path_length_mm": sum(math.dist(a, b) for a, b in zip(points, points[1:])),
        "input_volume_mm3": original.Volume(), "output_volume_mm3": material.Volume(),
        "removed_volume_mm3": max(removed, 0), "piece_count": len(material.Solids()),
        "status": "material_removed" if removed > 1e-7 else "no_material_removed",
        "model": "constant_circular_kerf_through_material", "compensation_applied": False,
        "manufacturing_ready": False, "tolerance_verified": False,
        "reasons": ["The model does not verify heat, pierce quality, head clearance, or the required manufacturing tolerance."],
    }
    return LaserMaterialReplay(material, passes, report)


@dataclass
class LaserReleasedPart:
    part: object
    remainder: object | None
    report: dict


def release_laser_part(source, reference_region, *, envelope_tolerance_mm):
    """Select a detached material piece. The reference never replaces its geometry."""
    import cadquery as cq
    source = _material(source)
    region = _material(reference_region, "part selection reference")
    if len(region.Solids()) != 1:
        raise ValueError("Part selection requires one closed reference region.")
    tolerance = finite(envelope_tolerance_mm, "part selection envelope tolerance")
    if tolerance < 0:
        raise ValueError("The part selection envelope tolerance cannot be negative.")
    pieces = source.Solids()
    if len(pieces) < 2:
        raise ValueError("The laser cut did not prove a detached part and remaining material.")
    bounds = region.BoundingBox()
    candidates = []
    for index, piece in enumerate(pieces):
        box = piece.BoundingBox()
        if any(getattr(box, f"{axis}min") < getattr(bounds, f"{axis}min") - tolerance
            or getattr(box, f"{axis}max") > getattr(bounds, f"{axis}max") + tolerance for axis in "xyz"):
            continue
        common = _boolean(piece, [region], "common")
        overlap = common.Volume()
        if overlap > max(1e-7, piece.Volume() * 0.5):
            candidates.append((index, piece, overlap))
    if len(candidates) != 1:
        raise ValueError("The detached part selection is missing or ambiguous. Review the cut and part identity.")
    selected, part, overlap = candidates[0]
    waste = [piece for index, piece in enumerate(pieces) if index != selected]
    remainder = cq.Compound.makeCompound(waste) if waste else None
    return LaserReleasedPart(part, remainder, {
        "schema_version": "laser-part-release/v1", "selected_piece_index": selected,
        "input_volume_mm3": source.Volume(), "part_volume_mm3": part.Volume(),
        "remainder_volume_mm3": remainder.Volume() if remainder else 0,
        "reference_overlap_mm3": overlap, "envelope_tolerance_mm": tolerance,
        "geometry_source": "actual_cut_material_piece", "manufacturing_ready": False,
        "tolerance_verified": False,
    })
