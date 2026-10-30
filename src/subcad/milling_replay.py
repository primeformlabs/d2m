"""Native fixed-axis cutter removal from tool-tip moves in part millimetres.

This module does not use an operation's requested final shape. It subtracts
the selected cutter along cutting moves. Flat cutters use exact linear sweeps.
Unsupported variable-height ball-cutter sweeps use explicit bounded pose sampling.
"""
from __future__ import annotations

import math
from typing import Any

from .pocket_geometry import rounded_rectangle
from .toolpath import Toolpath, ToolpathMove


def _boolean(first, others, operation: str, *, tolerance: float | None = None):
    """Avoid kernel thread fan-out for the small solids in each surface pass."""
    import cadquery as cq
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
    from OCP.TopTools import TopTools_ListOfShape
    job = {"cut": BRepAlgoAPI_Cut, "fuse": BRepAlgoAPI_Fuse, "common": BRepAlgoAPI_Common}[operation]()
    arguments, tools = TopTools_ListOfShape(), TopTools_ListOfShape()
    arguments.Append(first.wrapped)
    for shape in others:
        tools.Append(shape.wrapped)
    job.SetArguments(arguments)
    job.SetTools(tools)
    job.SetRunParallel(False)
    job.SetNonDestructive(True)
    if tolerance is not None:
        job.SetFuzzyValue(tolerance)
    job.Build()
    if not job.IsDone() or job.Shape().IsNull():
        raise ValueError("The native Milling Boolean operation failed. The cutter geometry requires review.")
    return cq.Shape.cast(job.Shape())


def clean_flat_result(result):
    """Unify coplanar Boolean faces only when valid geometry and volume remain."""
    previous = result.val()
    cleaned = previous.clean()
    if cleaned.isValid() and abs(cleaned.Volume() - previous.Volume()) <= max(1e-6, abs(previous.Volume()) * 1e-9):
        return result.newObject([cleaned]), True
    return result, False


def finite(value: Any, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"The {name} must be finite.")
    try:
        number = float(value)
    except (ValueError, TypeError) as error:
        raise ValueError(f"The {name} must be finite.") from error
    if not math.isfinite(number) or (positive and number <= 0):
        raise ValueError(f"The {name} must be {'positive and ' if positive else ''}finite.")
    return number


def validate_cutter(tool, *, cutting_depth: float, cut_length: float | None = None) -> None:
    """``cutting_depth``: from the setup face to the deepest cut (``tool_selection.tool_reach``); ``cut_length``:
    the material height that the flutes cut (the default is the depth)."""
    from .tool_library import envelope_cutter
    tool = envelope_cutter(tool)
    if tool.tool_type not in {"flat_endmill", "ball_endmill"}:
        raise ValueError("Native surface replay requires a flat end mill or a ball end mill.")
    diameter = finite(tool.diameter, "cutter diameter", positive=True)
    if tool.tool_type == "flat_endmill" and finite(tool.corner_radius, "cutter corner radius") != 0:
        raise ValueError("Native flat end mill replay requires a zero corner radius. A bull nose cutter needs its own geometry model.")
    flute = finite(tool.flute_length, "flute length", positive=True)
    projection = finite(tool.stickout_mm, "cutter projection", positive=True)
    reach = finite(tool.reach_mm or flute, "cutter reach", positive=True)
    if tool.tool_type == "ball_endmill" and flute < diameter:
        raise ValueError("The ball end mill requires a flute length at least equal to its diameter.")
    cut = cutting_depth if cut_length is None else cut_length
    if cut > flute + 1e-8 or cutting_depth > min(projection, reach) + 1e-8:
        raise ValueError("The selected cutter cannot reach the required surface depth.")


def _cutter(tool):
    import cadquery as cq
    radius = tool.diameter / 2
    if tool.tool_type == "flat_endmill":
        return cq.Solid.makeCylinder(radius, tool.flute_length)
    # Revolve a single meridian instead of joining a sphere and cylinder at
    # their tangent equator. The lower hemisphere and upper flute are exact.
    diagonal = radius / math.sqrt(2)
    return (cq.Workplane("XZ").moveTo(0, 0)
        .threePointArc((diagonal, radius - diagonal), (radius, radius))
        .lineTo(radius, tool.flute_length).lineTo(0, tool.flute_length).close()
        .revolve(360, (0, 0), (0, 1)).val())


def _horizontal_sweep(tool, start, end):
    import cadquery as cq
    radius = tool.diameter / 2
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return _cutter(tool).translate(start)
    midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2, start[2])
    bottom = start[2] + (radius if tool.tool_type == "ball_endmill" else 0)
    profile = rounded_rectangle(cq.Workplane("XY"), length + 2 * radius, 2 * radius, radius)
    upper = profile.extrude(tool.flute_length - (radius if tool.tool_type == "ball_endmill" else 0)).val()
    upper = upper.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(dy, dx)))
    upper = upper.translate((midpoint[0], midpoint[1], bottom))
    if tool.tool_type == "flat_endmill":
        return upper
    center_start = cq.Vector(start[0], start[1], start[2] + radius)
    center_end = cq.Vector(end[0], end[1], end[2] + radius)
    bridge = cq.Solid.makeCylinder(radius, length, center_start, cq.Vector(dx / length, dy / length, 0))
    first = cq.Workplane("XY").sphere(radius).val().translate(center_start)
    last = cq.Workplane("XY").sphere(radius).val().translate(center_end)
    return _boolean(upper, [bridge, first, last], "fuse")


def _ball_sweep(radius, start, end):
    """Exact spherical sweep when the cutter's axial section clears the stock."""
    import cadquery as cq
    first_center = cq.Vector(start[0], start[1], start[2] + radius)
    last_center = cq.Vector(end[0], end[1], end[2] + radius)
    distance = math.dist(start, end)
    if distance <= 1e-9:
        return cq.Workplane("XY").sphere(radius).val().translate(first_center)
    direction = cq.Vector(*[(end[axis] - start[axis]) / distance for axis in range(3)])
    diagonal = radius / math.sqrt(2)
    capsule = (cq.Workplane("XZ").moveTo(0, -radius)
        .threePointArc((diagonal, -diagonal), (radius, 0)).lineTo(radius, distance)
        .threePointArc((diagonal, distance + diagonal), (0, distance + radius)).close()
        .revolve(360, (0, 0), (0, 1)).val())
    axis = cq.Vector(0, 0, 1).cross(direction)
    cosine = max(-1, min(1, direction.z))
    if axis.Length > 1e-12:
        capsule = capsule.rotate((0, 0, 0), axis.toTuple(), math.degrees(math.acos(cosine)))
    elif cosine < 0:
        capsule = capsule.rotate((0, 0, 0), (1, 0, 0), 180)
    return capsule.translate(first_center)


def _linear_flat_sweep(tool, start, end):
    """Sweep the actual cylinder's forward faces along a straight move."""
    import cadquery as cq
    from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
    from OCP.gp import gp_Vec

    delta = tuple(end[axis] - start[axis] for axis in range(3))
    horizontal = math.hypot(delta[0], delta[1])
    cutter = _cutter(tool)
    additions = []
    if horizontal > 0:
        # Only the leading half of the curved face generates added volume.
        # Its outward normals have a positive dot product with the move.
        half = cq.Solid.makeCylinder(tool.radius, tool.flute_length, angleDegrees=180)
        half = half.rotate((0, 0, 0), (0, 0, 1), math.degrees(math.atan2(delta[1], delta[0])) - 90)
        side = next(face for face in half.Faces() if face.geomType() == "CYLINDER")
        additions.append(cq.Shape.cast(BRepPrimAPI_MakePrism(side.wrapped, gp_Vec(*delta), True, True).Shape()))
    if delta[2] != 0:
        caps = [face for face in cutter.Faces() if face.geomType() == "PLANE"]
        cap = (max if delta[2] > 0 else min)(caps, key=lambda face: face.Center().z)
        additions.append(cq.Shape.cast(BRepPrimAPI_MakePrism(cap.wrapped, gp_Vec(*delta), True, True).Shape()))
    sweep = _boolean(cutter, additions, "fuse") if additions else cutter
    if not sweep.isValid():
        raise ValueError("The linear flat-cutter sweep is not a valid solid.")
    return sweep.translate(start)


def replay_pass(shape, path: Toolpath, tool, *, sample_interval_mm: float = 0.25,
                previous_sweep=None, return_sweep: bool = False):
    """Return remaining material and a geometric replay report for one pass."""
    import cadquery as cq
    interval = finite(sample_interval_mm, "replay sample interval", positive=True)
    if path.metadata.get("coordinate_system") != "part_mm":
        raise ValueError("Native surface replay requires tool-tip coordinates in part millimetres.")
    if path.metadata.get("tool_axis", [0, 0, 1]) != [0, 0, 1]:
        raise ValueError("Native surface replay requires the positive Z cutter axis.")
    if not path.moves:
        raise ValueError("The surface path has no tool moves.")
    if len(path.moves) > 10000:
        raise ValueError("The surface pass exceeds the native move limit. Divide the pass into smaller paths.")
    for move in path.moves:
        if not all(math.isfinite(value) for value in move.position):
            raise ValueError("The surface path contains a non-finite tool position.")
        if move.move_type == "arc":
            raise ValueError("Native surface replay requires linearized arc moves.")
    cutting = [move.position[2] for move in path.moves if move.move_type in {"feed", "plunge"}]
    if not cutting:
        raise ValueError("The surface path has no cutting moves.")
    validate_cutter(tool, cutting_depth=max(0, shape.val().BoundingBox().zmax - min(cutting)))
    cutter = _cutter(tool)
    result, previous = shape, None
    cutters = []
    sampled, exact, count = False, 0, 0
    for move in path.moves:
        if move.move_type in {"feed", "plunge"}:
            if previous is None:
                raise ValueError("A cutting move requires a preceding tool position.")
            start, end = previous.position, move.position
            if tool.tool_type == "ball_endmill" and min(start[2], end[2]) + tool.radius >= shape.val().BoundingBox().zmax - 1e-9:
                solids = [_ball_sweep(tool.radius, start, end)]
                exact += 1
            elif abs(start[2] - end[2]) <= 1e-9:
                solids = [_horizontal_sweep(tool, start, end)]
                exact += 1
            elif math.hypot(start[0] - end[0], start[1] - end[1]) <= 1e-9 and tool.tool_type == "flat_endmill":
                lower = min(start[2], end[2])
                solids = [cq.Solid.makeCylinder(tool.diameter / 2,
                    tool.flute_length + abs(start[2] - end[2]), cq.Vector(start[0], start[1], lower))]
                exact += 1
            elif tool.tool_type == "flat_endmill":
                solids = [_linear_flat_sweep(tool, start, end)]
                exact += 1
            else:
                steps = max(1, math.ceil(math.dist(start, end) / interval))
                if count + steps + 1 > 10000:
                    raise ValueError("The surface pass exceeds the native replay pose limit. Divide the pass into smaller paths.")
                solids = [cutter.translate(tuple(start[axis] + (end[axis] - start[axis]) * i / steps for axis in range(3)))
                    for i in range(steps + 1)]
                count += len(solids)
                sampled = True
            cutters.extend(solids)
        previous = move
    topology_repaired = False
    empty_contact_solids = 0
    if tool.tool_type == "ball_endmill":
        # Preserve each selected-cutter solid. Joining overlapping spherical
        # sweeps can create invalid topology or lose earlier removal regions.
        # Sequential set difference applies the same union without a fused tool.
        earlier = list(previous_sweep) if isinstance(previous_sweep, (list, tuple)) else (
            [previous_sweep] if previous_sweep is not None else [])
        united = tuple(earlier + cutters)
        for solid in united:
            if not solid.isValid():
                raise ValueError("The ball-cutter sweep is not a valid solid.")
            before = result.val()
            stock_box, tool_box = before.BoundingBox(), solid.BoundingBox()
            if any(min(getattr(stock_box, axis + "max"), getattr(tool_box, axis + "max"))
                   - max(getattr(stock_box, axis + "min"), getattr(tool_box, axis + "min")) <= 1e-9
                   for axis in ("x", "y", "z")):
                continue
            contact = _boolean(before, [solid], "common")
            if not contact.isValid():
                raise ValueError("The ball-cutter contact is not a valid solid.")
            if not contact.Solids() or contact.Volume() <= 1e-9:
                # A cutter can cross an already removed region. Re-cutting its
                # coincident spherical boundary can create a spurious sliver.
                empty_contact_solids += 1
                continue
            before_volume = before.Volume()
            remaining = _boolean(before, [solid], "cut")
            if not remaining.isValid():
                repaired = remaining.fix()
                if repaired.isValid() and abs(repaired.Volume() - remaining.Volume()) <= max(1e-6, abs(remaining.Volume()) * 1e-8):
                    remaining = repaired
                    topology_repaired = True
            if not remaining.Solids() or not remaining.isValid():
                raise ValueError("The ball-cutter move did not produce valid remaining material.")
            if remaining.Volume() > before_volume + max(1e-6, abs(before_volume) * 1e-8):
                raise ValueError("The cutter replay added material. The intermediate result is unavailable.")
            result = result.newObject([remaining])
    else:
        united = previous_sweep
        for offset in range(0, len(cutters), 16):
            group = cutters[offset:offset + 16]
            if united is None:
                united, group = group[0], group[1:]
            if group:
                united = _boolean(united, group, "fuse", tolerance=1e-7)
        if united is not None:
            united = united.clean()
            if not united.isValid():
                raise ValueError("The surface cutter sweep is not a valid solid.")
            result = shape.newObject([_boolean(shape.val(), [united], "cut")])
    topology_cleaned = False
    if tool.tool_type == "flat_endmill" and result.solids().vals() and result.val().isValid():
        result, topology_cleaned = clean_flat_result(result)
    if result.solids().vals() and not result.val().isValid():
        volume_before = result.val().Volume()
        repaired = result.val().fix()
        if repaired.isValid() and abs(repaired.Volume() - volume_before) <= max(1e-6, abs(volume_before) * 1e-8):
            result = result.newObject([repaired])
            topology_repaired = True
    if not result.solids().vals() or not result.val().isValid():
        raise ValueError("The surface path did not produce valid remaining material.")
    if result.val().Volume() > shape.val().Volume() + max(1e-6, abs(shape.val().Volume()) * 1e-8):
        raise ValueError("The cutter replay added material. The intermediate result is unavailable.")
    report = {"method": "selected_cutter_brep_replay", "coordinate_system": "part_mm",
        "tool_axis": [0, 0, 1], "geometry_fidelity": "sampled_cutter" if sampled else "exact_cutter_sweep",
        "sample_interval_mm": interval if sampled else None, "sampled_poses": count,
        "exact_segments": exact, "topology_repaired": topology_repaired, "topology_cleaned": topology_cleaned,
        "removal_union": "sequential_cutter_solids" if tool.tool_type == "ball_endmill" else "fused_cutter_solids",
        "empty_contact_solids": empty_contact_solids,
        "manufacturing_ready": False, "verification_status": "not_verified"}
    return (result, report, united) if return_sweep else (result, report)


def raster_paths(bounds, tip_height, *, tool, operation: str, tolerance_mm: float, feed: float, rpm: float | None):
    """Generate independent rows with safe entry and retract positions."""
    tolerance = finite(tolerance_mm, "surface tolerance", positive=True)
    radius = tool.diameter / 2
    step = min(radius, max(math.sqrt(4 * radius * tolerance), 0.1))
    sample = min(radius / 2, max(tolerance, 0.1))
    x0, x1, y0, y1 = bounds.xmin - radius, bounds.xmax + radius, bounds.ymin - radius, bounds.ymax + radius
    nx, ny = max(1, math.ceil((x1 - x0) / sample)), max(1, math.ceil((y1 - y0) / step))
    if nx * (ny + 1) > 10000 or ny > 200:
        raise ValueError("The surface raster exceeds the native replay limit. Divide the surface into smaller regions.")
    paths = []
    for row in range(ny + 1):
        y = y0 + (y1 - y0) * row / ny
        points = [(x0 + (x1 - x0) * column / nx, y) for column in range(nx + 1)]
        if row % 2:
            points.reverse()
        heights = [tip_height(x, y) for x, y in points]
        keep = [0] + [index for index in range(1, len(points) - 1)
            if abs(heights[index] - heights[index - 1] - (heights[index + 1] - heights[index])) > 1e-9] + [len(points) - 1]
        points = [points[index] for index in keep]
        path = Toolpath(operation, safe_z=bounds.zmax + max(2, radius), tool_type=tool.tool_type,
            tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
            metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1], "face_selector": ">Z", "pass_index": row + 1,
                "pass_role": "finishing", "sample_interval_mm": sample, "stepover_mm": (y1 - y0) / ny})
        x, y = points[0]
        path.append(ToolpathMove("rapid", (x, y, path.safe_z)))
        path.append(ToolpathMove("plunge", (x, y, tip_height(x, y)), feed=feed, spindle_rpm=rpm))
        for x, y in points[1:]:
            path.append(ToolpathMove("feed", (x, y, tip_height(x, y)), feed=feed, spindle_rpm=rpm))
        path.append(ToolpathMove("retract", (x, y, path.safe_z)))
        paths.append(path)
    return paths
