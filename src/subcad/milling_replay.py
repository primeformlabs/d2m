"""Native fixed-axis cutter removal from tool-tip moves in part millimetres.

This module does not use an operation's requested final shape. It subtracts
the selected cutter along cutting moves. Variable-height moves use bounded
pose sampling; the report keeps that approximation explicit.
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
    if tolerance is not None:
        job.SetFuzzyValue(tolerance)
    job.Build()
    if not job.IsDone() or job.Shape().IsNull():
        raise ValueError("The native Milling Boolean operation failed. The cutter geometry requires review.")
    return cq.Shape.cast(job.Shape())


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


def validate_cutter(tool, *, cutting_depth: float) -> None:
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
    if cutting_depth > min(flute, projection, reach) + 1e-8:
        raise ValueError("The selected cutter cannot reach the required surface depth.")


def _cutter(tool):
    import cadquery as cq
    radius = tool.diameter / 2
    if tool.tool_type == "flat_endmill":
        return cq.Solid.makeCylinder(radius, tool.flute_length)
    sphere = cq.Workplane("XY").sphere(radius).val().translate((0, 0, radius))
    shank = cq.Solid.makeCylinder(radius, tool.flute_length - radius, cq.Vector(0, 0, radius))
    return _boolean(sphere, [shank], "fuse")


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
    first = cq.Workplane("XY").sphere(radius).val().translate(first_center)
    distance = math.dist(start, end)
    if distance <= 1e-9:
        return first
    direction = cq.Vector(*[(end[axis] - start[axis]) / distance for axis in range(3)])
    bridge = cq.Solid.makeCylinder(radius, distance, first_center, direction)
    last = cq.Workplane("XY").sphere(radius).val().translate(last_center)
    return _boolean(first, [bridge, last], "fuse")


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
    # Unite overlapping sweep segments before the stock Boolean. A compound
    # of overlapping poses is not a valid Boolean tool. Repeated small cuts
    # also amplify tolerance errors at adjoining spherical sweeps.
    # Only material inside the incoming solid participates in the removal
    # union. Empty approach sweeps must not create tangent external shells.
    removals = []
    for solid in cutters:
        common = _boolean(solid, [shape.val()], "common") if tool.tool_type == "ball_endmill" else solid
        if common.Volume() > 1e-9:
            if not common.isValid():
                raise ValueError("The surface cutter intersection is not a valid solid.")
            removals.append(common)
    united = previous_sweep
    for offset in range(0, len(removals), 16):
        group = removals[offset:offset + 16]
        if united is None:
            united, group = group[0], group[1:]
        if group:
            united = _boolean(united, group, "fuse", tolerance=1e-7)
    if united is not None:
        # Same-domain cleaning can damage adjoining spherical pass faces.
        # Keep the valid Boolean topology for ball-cutter removal unions.
        if tool.tool_type == "flat_endmill":
            united = united.clean()
        if not united.isValid():
            raise ValueError("The surface cutter sweep is not a valid solid.")
        result = shape.newObject([_boolean(shape.val(), [united], "cut")])
    topology_repaired = False
    if result.solids().vals() and not result.val().isValid():
        volume_before = result.val().Volume()
        repaired = result.val().fix()
        if repaired.isValid() and abs(repaired.Volume() - volume_before) <= max(1e-6, abs(volume_before) * 1e-8):
            result = result.newObject([repaired])
            topology_repaired = True
    if not result.solids().vals() or not result.val().isValid():
        raise ValueError("The surface path did not produce valid remaining material.")
    report = {"method": "selected_cutter_brep_replay", "coordinate_system": "part_mm",
        "tool_axis": [0, 0, 1], "geometry_fidelity": "sampled_cutter" if sampled else "exact_cutter_sweep",
        "sample_interval_mm": interval if sampled else None, "sampled_poses": count,
        "exact_segments": exact, "topology_repaired": topology_repaired,
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
            metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1], "pass_index": row + 1,
                "pass_role": "finishing", "sample_interval_mm": sample, "stepover_mm": (y1 - y0) / ny})
        x, y = points[0]
        path.append(ToolpathMove("rapid", (x, y, path.safe_z)))
        path.append(ToolpathMove("plunge", (x, y, tip_height(x, y)), feed=feed, spindle_rpm=rpm))
        for x, y in points[1:]:
            path.append(ToolpathMove("feed", (x, y, tip_height(x, y)), feed=feed, spindle_rpm=rpm))
        path.append(ToolpathMove("retract", (x, y, path.safe_z)))
        paths.append(path)
    return paths
