"""Pocket clearing from compensated, clipped cutter paths in actual stock."""
from __future__ import annotations

from dataclasses import replace
import math

from .milling_replay import _boolean, finite, replay_pass, validate_cutter, _horizontal_sweep, _cutter, clean_flat_result
from .profile_milling import _profile_wire, _linearized_wire, _setup_plane, _replay_closed_contour
from .toolpath import Toolpath, ToolpathMove
from .program_mode import is_program_mode


def pocket_tool_limit(profile, islands=(), *, finish_allowance=0.1):
    """Limit the initial tool by the opening and positive island clearances."""
    wire, _ = _profile_wire(profile)
    bounds = wire.BoundingBox()
    maximum = min(bounds.xlen, bounds.ylen) * 0.8
    keep = [_profile_wire(item)[0] for item in islands or []]
    gaps = [edge.distance(island) for edge in wire.Edges() for island in keep]
    gaps.extend(first.distance(second) for index, first in enumerate(keep) for second in keep[index + 1:])
    positive = [gap for gap in gaps if gap > 1e-6]
    maximum = min(maximum, min(positive)) if positive else maximum
    allowance = finite(finish_allowance, "pocket finish allowance")
    if allowance < 0:
        raise ValueError("The pocket finish allowance must be non-negative.")
    lower, upper = 0.0, maximum / 2
    # Bounding dimensions do not prove that a cutter fits a triangular or
    # concave opening. Qualify its compensated center region too.
    for _ in range(18):
        radius = (lower + upper) / 2
        try:
            _center_region(wire, keep, radius + allowance)
            lower = radius
        except ValueError:
            upper = radius
    if lower <= 1e-6:
        raise ValueError("The pocket has no accessible cutter region with the requested finish allowance.")
    return 2 * lower


def _region(outer, islands):
    import cadquery as cq
    face = cq.Face.makeFromWires(outer)
    if islands:
        face = _boolean(face, [cq.Face.makeFromWires(wire) for wire in islands], "cut")
    if not face.isValid():
        raise ValueError("The pocket boundary and islands do not produce a valid machining region.")
    return face


def _center_region(outer, islands, radius):
    import cadquery as cq
    try:
        boundaries = outer.offset2D(-radius, kind="arc")
        if not boundaries:
            raise ValueError("The selected cutter does not fit the pocket opening.")
        faces = [cq.Face.makeFromWires(wire) for wire in boundaries]
        region = faces[0] if len(faces) == 1 else cq.Compound.makeCompound(faces)
        for island in islands:
            enlarged = island.offset2D(radius, kind="arc")
            if len(enlarged) != 1:
                raise ValueError("The pocket island has no continuous cutter clearance boundary.")
            region = _boolean(region, [cq.Face.makeFromWires(enlarged[0])], "cut")
    except Exception as error:
        reason = "The selected cutter cannot reach the pocket region between its boundary and islands." if islands else "The selected cutter does not fit the pocket opening or its accessible boundary."
        raise ValueError(reason) from error
    if not region.isValid() or not region.Faces() or sum(face.Area() for face in region.Faces()) <= 1e-8:
        raise ValueError("The selected cutter cannot reach the pocket region between its boundary and islands.")
    return region


def _path(points, z, safe, tool, feed, rpm, role, kind):
    path = Toolpath("profile_pocket", safe_z=safe, tool_type=tool.tool_type,
        tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
        metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1], "pass_role": role, "path_kind": kind})
    x, y = points[0]
    path.append(ToolpathMove("rapid", (x, y, safe)))
    path.append(ToolpathMove("plunge", (x, y, z), feed=feed, spindle_rpm=rpm))
    path.moves.extend(ToolpathMove("feed", (x, y, z), feed=feed, spindle_rpm=rpm) for x, y in points[1:])
    x, y = points[-1]
    path.append(ToolpathMove("retract", (x, y, safe)))
    return path


def _replay_raster(shape, path, tool):
    """The entry and horizontal line share one exact flat-cutter capsule."""
    cutting = [move for move in path.moves if move.move_type in {"plunge", "feed"}]
    if not cutting or len(cutting) > 2 or any(abs(move.position[2] - cutting[0].position[2]) > 1e-9 for move in cutting):
        raise ValueError("The pocket raster requires one horizontal cutting segment or one plunge.")
    start = cutting[0].position
    cutter = _horizontal_sweep(tool, start, cutting[-1].position) if len(cutting) == 2 else _cutter(tool).translate(start)
    result = shape.newObject([_boolean(shape.val(), [cutter], "cut")])
    if not result.solids().vals() or not result.val().isValid():
        raise ValueError("The pocket raster did not produce valid remaining material.")
    result, cleaned = clean_flat_result(result)
    return result, {"method": "selected_cutter_raster_sweep", "coordinate_system": "part_mm",
        "geometry_fidelity": "exact_cutter_sweep", "exact_segments": len(cutting),
        "sample_interval_mm": None, "sampled_poses": 0, "topology_repaired": False, "topology_cleaned": cleaned,
        "manufacturing_ready": False, "verification_status": "not_verified"}


def _replay_open_boundary(shape, path, tool):
    """Subtract each exact line sweep without fusing overlapping 3D tools."""
    cutting = [move for move in path.moves if move.move_type in {"plunge", "feed"}]
    result, cleaned = shape, False
    for first, last in zip(cutting, cutting[1:]):
        if abs(first.position[2] - last.position[2]) > 1e-9:
            raise ValueError("The pocket boundary requires horizontal cutting segments.")
        cutter = _horizontal_sweep(tool, first.position, last.position)
        result = result.newObject([_boolean(result.val(), [cutter], "cut")])
        if not result.solids().vals() or not result.val().isValid():
            raise ValueError("The pocket boundary did not produce valid remaining material.")
        result, cleaned = clean_flat_result(result)
    return result, {"method": "selected_cutter_boundary_segments", "coordinate_system": "part_mm",
        "geometry_fidelity": "exact_cutter_sweep", "exact_segments": len(cutting),
        "sample_interval_mm": None, "sampled_poses": 0, "topology_repaired": False, "topology_cleaned": cleaned,
        "manufacturing_ready": False, "verification_status": "not_verified"}


def _segment_distance(point, first, second):
    """Distance from a point to the segment first-second (2D)."""
    dx, dy = second[0] - first[0], second[1] - first[1]
    length = dx * dx + dy * dy
    t = 0.0 if length <= 0 else max(0.0, min(1.0, ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy) / length))
    return math.dist(point, (first[0] + t * dx, first[1] + t * dy))


def _clearing_paths(region, outer, islands, *, z, safe, tool, feed, rpm, role, allowance, chord):
    import cadquery as cq
    if not region.Faces():
        edges = region.Edges()
        if edges:
            edge = edges[0]
            points = [(edge.startPoint().x, edge.startPoint().y), (edge.endPoint().x, edge.endPoint().y)]
        else:
            point = region.Vertices()[0].Center()
            points = [(point.x, point.y)]
        return [_path(points, z, safe, tool, feed, rpm, role, "raster")]
    bounds = region.BoundingBox()
    step = min(tool.diameter * 0.4, finite((tool.safe_limits or {}).get("max_stepover_mm", tool.diameter * 0.4),
        "maximum pocket stepover", positive=True))
    rows = max(1, math.ceil(bounds.ylen / step))
    if rows > 500:
        raise ValueError("The pocket exceeds the raster row limit. Divide the operation into smaller regions.")
    paths = []
    levels = [bounds.ymin + bounds.ylen * row / rows for row in range(rows + 1)]
    # One boolean clips every raster line (not one boolean for each line); each clipped piece
    # keeps the y of its line.
    lines = [cq.Edge.makeLine(cq.Vector(bounds.xmin - 1, y, 0), cq.Vector(bounds.xmax + 1, y, 0)) for y in levels]
    clipped_rows: dict[int, list[tuple[float, float]]] = {}
    for edge in _boolean(region, [cq.Compound.makeCompound(lines)], "common").Edges():
        if edge.Length() <= 1e-7:
            continue
        mid = (edge.startPoint().y + edge.endPoint().y) / 2
        row = min(range(len(levels)), key=lambda index: abs(levels[index] - mid))
        clipped_rows.setdefault(row, []).append(
            (min(edge.startPoint().x, edge.endPoint().x), max(edge.startPoint().x, edge.endPoint().x)))
    for row, y in enumerate(levels):
        for start, end in sorted(clipped_rows.get(row, [])):
            points = [(start, y), (end, y)]
            if row % 2:
                points.reverse()
            paths.append(_path(points, z, safe, tool, feed, rpm, role, "raster"))
    # The boundary pass clears the material between raster endpoints, while
    # retaining corners that the selected cutter cannot reach.
    for wire, distance, is_island in [(outer, -(tool.radius + allowance), False)] + [
            (island, tool.radius + allowance, True) for island in islands]:
        for boundary in wire.offset2D(distance, kind="arc"):
            points, curved = _linearized_wire(boundary, min(chord, tool.radius / 4), tangent_curves=is_island)
            # Islands can meet the outer wall. A complete boundary loop would
            # then cut the protected island. Clip every boundary segment too.
            chains, current = [], []
            pairs = [(first, second) for first, second in zip(points, points[1:]) if math.dist(first, second) >= 1e-8]
            # One boolean clips every boundary segment; each clipped piece goes back to the segment
            # that it lies on (not one boolean for each segment).
            pieces: dict[int, list] = {}
            if pairs:
                edges = [cq.Edge.makeLine(cq.Vector(*first, 0), cq.Vector(*second, 0)) for first, second in pairs]
                for part in _boolean(region, [cq.Compound.makeCompound(edges)], "common").Edges():
                    if part.Length() <= 1e-7:
                        continue
                    centre = part.positionAt(0.5)
                    pieces.setdefault(min(range(len(pairs)), key=lambda index: _segment_distance(
                        (centre.x, centre.y), *pairs[index])), []).append(part)
            for index, (first, second) in enumerate(pairs):
                segments = []
                for part in pieces.get(index, []):
                    if part.Length() <= 1e-7:
                        continue
                    ends = [(part.startPoint().x, part.startPoint().y), (part.endPoint().x, part.endPoint().y)]
                    ends.sort(key=lambda point: math.dist(first, point))
                    segments.append(ends)
                segments.sort(key=lambda ends: math.dist(first, ends[0]))
                if not segments and current:
                    chains.append(current)
                    current = []
                for start, end in segments:
                    if current and math.dist(current[-1], start) > 1e-6:
                        chains.append(current)
                        current = []
                    current.extend([start, end] if not current else [end])
            if current:
                chains.append(current)
            for chain in chains:
                if math.dist(chain[0], chain[-1]) < 1e-6:
                    chain[-1] = chain[0]
                path = _path(chain, z, safe, tool, feed, rpm, role, "boundary")
                path.metadata["profile_fidelity"] = "linearized_curve" if curved else "linear_profile"
                paths.append(path)
    return paths


def _exact_width_center(profile, tool, mirror):
    """A matching circular cutter can have a line or point center region."""
    import cadquery as cq
    if not isinstance(profile, dict):
        return None
    kind = str(profile.get("type", profile.get("shape", ""))).lower()
    center = profile.get("center", (profile.get("cx", profile.get("x", 0)), profile.get("cy", profile.get("y", 0))))
    cx, cy = finite(center[0], "profile X position"), finite(center[1], "profile Y position")
    if kind in {"circle", "circular"}:
        diameter = 2 * finite(profile["radius"], "profile radius", positive=True) if "radius" in profile else finite(profile.get("diameter"), "profile diameter", positive=True)
        if abs(diameter - tool.diameter) < 1e-8:
            return cq.Vertex.makeVertex(cx, cy * mirror, 0)
    if kind in {"slot", "obround", "obround_slot", "slot2d"} and abs(finite(profile.get("width"), "slot width", positive=True) - tool.diameter) < 1e-8:
        span = (finite(profile.get("length"), "slot length", positive=True) - tool.diameter) / 2
        angle = math.radians(finite(profile.get("rotation_deg", profile.get("angle", 0)), "profile angle"))
        if span <= 1e-8:
            return cq.Vertex.makeVertex(cx, cy * mirror, 0)
        first = cq.Vector(cx - span * math.cos(angle), (cy - span * math.sin(angle)) * mirror, 0)
        last = cq.Vector(cx + span * math.cos(angle), (cy + span * math.sin(angle)) * mirror, 0)
        return cq.Edge.makeLine(first, last)
    return None


def run_profile_pocket(shape, profile, *, tool, depth, islands=None, through=False,
                       face_selector=">Z", stepdown=1, finish_allowance=0.1, chord_error=0.01,
                       feed=100, rpm=None, outside=False):
    """Return material, actual toolpath, retained pass states and target comparison."""
    import cadquery as cq
    if tool is None or tool.tool_type != "flat_endmill":
        raise ValueError("Native profile pocket replay requires a selected flat end mill.")
    depth = finite(depth, "profile pocket depth", positive=True)
    stepdown = finite(stepdown, "profile pocket stepdown", positive=True)
    allowance = finite(finish_allowance, "profile pocket finish allowance")
    chord = finite(chord_error, "profile pocket chord error", positive=True)
    if allowance < 0:
        raise ValueError("The pocket finish allowance must be non-negative.")
    if not isinstance(islands, (list, tuple, type(None))) or len(islands or []) > 100:
        raise ValueError("The pocket requires a list with at most 100 island boundaries.")
    plane, mirror = _setup_plane(face_selector)
    local = cq.Workplane("XY").newObject([plane.toLocalCoords(shape.val())])
    bounds = local.val().BoundingBox()
    depth = bounds.zlen if through else depth
    if depth > bounds.zlen + 1e-8:
        raise ValueError("The pocket depth exceeds the incoming material thickness along its cutter axis.")
    validate_cutter(tool, cutting_depth=depth)
    outer, sampled = _profile_wire(profile)
    keep = []
    for item in islands or []:
        wire, approximate = _profile_wire(item)
        sampled |= approximate
        keep.append(wire)
    if mirror < 0:
        outer, keep = outer.mirror("XZ"), [wire.mirror("XZ") for wire in keep]
    if outside:
        if keep:
            raise ValueError("The outside clearing operation requires one retained profile and no additional islands.")
        keep = [outer]
        margin = tool.diameter * 2
        outer = cq.Workplane("XY").rect(bounds.xlen + margin, bounds.ylen + margin).val().translate(
            ((bounds.xmin + bounds.xmax) / 2, (bounds.ymin + bounds.ymax) / 2, 0))
    target_region = _region(outer, keep)
    opening = outer.BoundingBox()
    open_stock_edges = (not outside and abs(cq.Face.makeFromWires(outer).Area() - bounds.xlen * bounds.ylen) < 1e-6
        and all(abs(first - second) < 1e-6 for first, second in [
            (opening.xmin, bounds.xmin), (opening.xmax, bounds.xmax), (opening.ymin, bounds.ymin), (opening.ymax, bounds.ymax)]))
    if open_stock_edges:
        outer = cq.Workplane("XY").rect(bounds.xlen + tool.diameter * 2, bounds.ylen + tool.diameter * 2).val().translate(
            ((bounds.xmin + bounds.xmax) / 2, (bounds.ymin + bounds.ymax) / 2, 0))
    center = _exact_width_center(profile, tool, mirror) if not outside and not keep else None
    if center is not None:
        if allowance != 0:
            raise ValueError("The matching cutter cannot leave radial finish allowance. Set the allowance to zero or select a smaller cutter.")
        rough_center = center
    else:
        center = _center_region(outer, keep, tool.radius)
        try:
            rough_center = _center_region(outer, keep, tool.radius + allowance)
        except ValueError:
            raise ValueError("The cutter and finish allowance do not fit the pocket region. Reduce the allowance or select another cutter.") from None
    maximum = finite((tool.safe_limits or {}).get("max_stepdown_mm", tool.diameter / 2), "maximum pocket stepdown", positive=True)
    stepdown = min(stepdown, maximum, tool.diameter / 2)
    count = max(1, math.ceil(depth / stepdown))
    if count > 100:
        raise ValueError("The pocket exceeds the depth-pass limit. Divide the operation into smaller regions.")
    safe = bounds.zmax + max(2, tool.radius)
    paths = []
    floor_allowance = min(allowance, stepdown, depth / 2)
    for level in range(1, count + 1):
        # Leave a floor allowance for the independently retained finish passes.
        z = bounds.zmax - min(depth * level / count, depth - floor_allowance)
        paths.extend(_clearing_paths(rough_center, outer, keep, z=z, safe=safe, tool=tool, feed=feed, rpm=rpm,
            role="roughing", allowance=allowance, chord=chord))
    paths.extend(_clearing_paths(center, outer, keep, z=bounds.zmax - depth, safe=safe, tool=tool, feed=feed, rpm=rpm,
        role="finishing", allowance=0, chord=chord))
    if not paths or len(paths) > 1000:
        raise ValueError("The pocket has no complete path within the retained pass limit. Divide the operation into smaller regions.")
    normal = list(plane.zDir.toTuple())
    aggregate = Toolpath("profile_cutout" if outside else "profile_pocket", safe_z=safe,
        tool_type=tool.tool_type, tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
        metadata={"coordinate_system": "part_mm", "tool_axis": normal, "face_selector": face_selector,
            "cutter_compensation": "geometry_applied", "maximum_stepdown_mm": stepdown,
            "finish_allowance_mm": allowance, "chord_error_mm": chord, "waste_discarded": False})
    aggregate.metadata["floor_allowance_mm"] = floor_allowance
    aggregate.metadata["open_stock_edges"] = open_stock_edges
    if is_program_mode():
        # The same moves, one cut of the target region, no pass states (program_mode.py).
        for path in paths:
            aggregate.moves.extend(replace(move, position=plane.toWorldCoords(move.position).toTuple()) for move in path.moves)
        solids = [cq.Solid.extrudeLinear(face.outerWire(), face.innerWires(), cq.Vector(0, 0, depth)).translate(
            (0, 0, bounds.zmax - depth)) for face in target_region.Faces()]
        cut = _boolean(local.val(), [cq.Compound.makeCompound(solids)], "cut")
        world = shape.newObject([cut.transformShape(plane.rG)])
        comparison = {"method": "program_mode_target_region", "tolerance_verified": False, "manufacturing_ready": False}
        aggregate.metadata.update(material_passes=[], operation_target_comparison=comparison, program_mode=True,
            pass_count=len(paths))
        state = {"shape": world, "pass_index": len(paths), "pass_role": "program", "label": "After the program operation",
            "geometry_replay": {"program_mode": True}}
        return world, aggregate, [state], [], comparison
    result, states, reports = local, [], []
    completed_paths = {}
    current_volume = local.val().Volume()
    for index, path in enumerate(paths, 1):
        before = current_volume
        cutting = [move for move in path.moves if move.move_type in {"plunge", "feed"}]
        geometry_key = tuple((move.move_type, tuple(move.position)) for move in path.moves)
        if geometry_key in completed_paths:
            # Only material subtraction occurs in this loop. A repeated cutter path
            # cannot remove material after its exact sweep was already subtracted.
            preceding_index, preceding_report = completed_paths[geometry_key]
            report = {**preceding_report, "repeated_pass_reused": True,
                "reused_from_pass_index": preceding_index, "reuse_proof": "set_difference_idempotence"}
        elif path.metadata["path_kind"] == "raster":
            result, report = _replay_raster(result, path, tool)
        elif math.dist(cutting[0].position, cutting[-1].position) < 1e-8:
            result, report = _replay_closed_contour(result, path, tool)
        else:
            result, report = _replay_open_boundary(result, path, tool)
        if geometry_key not in completed_paths:
            current_volume = result.val().Volume()
            completed_paths[geometry_key] = (index, dict(report))
        start = len(aggregate.moves)
        aggregate.moves.extend(replace(move, position=plane.toWorldCoords(move.position).toTuple()) for move in path.moves)
        report.update(pass_index=index, pass_role=path.metadata["pass_role"], path_kind=path.metadata["path_kind"],
            move_start_index=start, move_end_index=len(aggregate.moves), removed_volume_mm3=before - current_volume,
            tool_axis=normal, profile_fidelity=path.metadata.get("profile_fidelity", "linear_profile"), source_profile_sampled=sampled)
        reports.append(report)
        world = shape.newObject([result.val().transformShape(plane.rG)])
        states.append({"shape": world, "pass_index": index, "pass_role": path.metadata["pass_role"],
            "label": f"After profile pocket {path.metadata['pass_role']} pass {index}", "geometry_replay": report})
    if local.val().Volume() - result.val().Volume() <= 1e-8:
        raise ValueError("The pocket path does not remove material from the incoming stock.")
    solids = [cq.Solid.extrudeLinear(face.outerWire(), face.innerWires(), cq.Vector(0, 0, depth)).translate(
        (0, 0, bounds.zmax - depth)) for face in target_region.Faces()]
    target_removal = _boolean(local.val(), [cq.Compound.makeCompound(solids)], "common")
    actual_removal = _boolean(local.val(), [result.val()], "cut")
    remaining = _boolean(target_removal, [actual_removal], "cut").Volume()
    overcut = _boolean(actual_removal, [target_removal], "cut").Volume()
    comparison = {"method": "native_brep_operation_region", "remaining_target_material_mm3": max(0, remaining),
        "overcut_volume_mm3": max(0, overcut), "tolerance_verified": False, "manufacturing_ready": False}
    aggregate.metadata.update(material_passes=reports, operation_target_comparison=comparison)
    return states[-1]["shape"], aggregate, states, reports, comparison
