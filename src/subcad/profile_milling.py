"""Compensated profile contour paths and native cutter material states.

A contour cuts a cutter-width channel. It does not pocket the interior or
discard the exterior waste. Each path is replayed against incoming material.
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import Any

from .milling_replay import finite, _boolean, validate_cutter, clean_flat_result
from .pocket_geometry import rounded_rectangle
from .profiles import normalize_profile_points
from .toolpath import Toolpath, ToolpathMove
from .program_mode import is_program_mode
from .milling_layers import LayerCheckUnavailable, LayerReplay


def _profile_wire(profile: Any):
    import cadquery as cq

    kind = str(profile.get("type", profile.get("shape", ""))).lower() if isinstance(profile, dict) else "polyline"
    explicit_points = isinstance(profile, dict) and any(profile.get(key) for key in ("points", "vertices", "polyline", "control_points"))
    if isinstance(profile, dict) and not explicit_points and not profile.get("segments") and kind not in {
        "", "rect", "rectangle", "rib", "pad", "slot", "obround", "obround_slot", "slot2d", "circle", "circular", "polygon", "regular_polygon",
    }:
        raise ValueError("The contour profile type is unsupported. Supply its explicit closed boundary.")
    if isinstance(profile, dict) and kind in {"slot", "obround", "obround_slot", "slot2d"}:
        width = finite(profile.get("width"), "profile width", positive=True)
        length = finite(profile.get("length"), "profile length", positive=True)
        if length < width:
            raise ValueError("The slot profile length must be at least equal to its width.")
    try:
        points = normalize_profile_points(profile)
    except (ValueError, TypeError, IndexError, KeyError) as error:
        raise ValueError("The contour requires a finite closed profile with valid coordinates.") from error
    if len(points) > 2000:
        raise ValueError("The profile exceeds the contour vertex limit. Divide the operation into smaller regions.")
    if len(points) < 3 or any(len(point) != 2 or not all(math.isfinite(value) for value in point) for point in points):
        raise ValueError("The contour requires a finite closed profile with at least three points.")
    if points[0] == points[-1]:
        points = points[:-1]
    is_slot_circle = isinstance(profile, dict) and kind in {"slot", "obround", "obround_slot", "slot2d"} and length == width and not explicit_points
    if len(points) < 3 or (not is_slot_circle and any(math.dist(a, b) < 1e-8 for a, b in zip(points, points[1:] + points[:1]))):
        raise ValueError("The contour profile contains repeated or missing vertices.")
    plane = cq.Workplane("XY")
    sampled = isinstance(profile, dict) and any(str(segment.get("type", "")).lower() in {"arc", "curve_tessellated"} for segment in profile.get("segments", []))
    if isinstance(profile, dict) and not explicit_points and not profile.get("segments"):
        center = profile.get("center", (profile.get("cx", profile.get("x", 0)), profile.get("cy", profile.get("y", 0))))
        cx, cy = finite(center[0], "profile X coordinate"), finite(center[1], "profile Y coordinate")
        angle = finite(profile.get("rotation_deg", profile.get("angle", 0)), "profile rotation")
        if kind in {"circle", "circular"} or (not kind and ("radius" in profile or "diameter" in profile)):
            radius = finite(profile["radius"], "profile radius", positive=True) if "radius" in profile else finite(profile.get("diameter", 0), "profile diameter", positive=True) / 2
            wire = plane.circle(radius).val().translate((cx, cy, 0))
        elif kind in {"rect", "rectangle", "rib", "pad", "slot", "obround", "obround_slot", "slot2d"} or (not kind and "width" in profile):
            width = finite(profile.get("width"), "profile width", positive=True)
            length = finite(profile.get("length"), "profile length", positive=True)
            slot = kind in {"slot", "obround", "obround_slot", "slot2d"}
            radius = width / 2 if slot else finite(profile.get("corner_radius", 0), "profile corner radius")
            if slot and length < width:
                raise ValueError("The slot profile length must be at least equal to its width.")
            wire = plane.circle(radius).val() if length == width == 2 * radius else rounded_rectangle(plane, length, width, radius).val()
            wire = wire.rotate((0, 0, 0), (0, 0, 1), angle).translate((cx, cy, 0))
        else:
            wire = plane.polyline(points).close().val()
    else:
        wire = plane.polyline(points).close().val()
    try:
        face = cq.Face.makeFromWires(wire)
    except Exception as error:
        raise ValueError("The contour requires a valid closed profile without intersections.") from error
    if not wire.isValid() or not wire.IsClosed() or not face.isValid() or face.Area() <= 1e-8:
        raise ValueError("The contour requires a valid closed profile without intersections.")
    return wire, sampled


def _linearized_wire(wire, chord_error: float, *, tangent_curves: bool = False):
    """Traverse ordered edges. Keep line edges exact and bound circular chords."""
    import cadquery as cq
    from OCP.BRepTools import BRepTools_WireExplorer
    from OCP.TopAbs import TopAbs_REVERSED

    explorer, points, curved = BRepTools_WireExplorer(wire.wrapped), [], False
    while explorer.More():
        native = explorer.Current()
        edge = cq.Edge(native)
        kind = edge.geomType()
        if kind == "LINE":
            count = 1
        elif kind == "CIRCLE":
            radius = edge.radius()
            angle = 2 * math.acos(max(-1, min(1, 1 - chord_error / radius)))
            count = max(2, math.ceil(edge.Length() / max(radius * angle, 1e-8)))
            curved = True
        else:
            raise ValueError("The compensated contour contains an unsupported curve. Supply a qualified linear or circular profile.")
        if len(points) + count > 4000:
            raise ValueError("The contour exceeds the native path limit. Divide the operation into smaller regions.")
        positions = edge.positions([i / count for i in range(count + 1)])
        if native.Orientation() == TopAbs_REVERSED:
            positions.reverse()
        values = [(point.x, point.y) for point in positions]
        if kind == "CIRCLE" and tangent_curves:
            center = edge.arcCenter()
            vertices = []
            for first, second in zip(positions, positions[1:]):
                a, b = first - center, second - center
                denominator = radius * radius + a.dot(b)
                if denominator <= 1e-12:
                    raise ValueError("The contour arc requires smaller linear segments for cutter compensation.")
                point = center + (a + b) * (radius * radius / denominator)
                vertices.append((point.x, point.y))
            values = [values[0], *vertices, values[-1]]
        if points and math.dist(points[-1], values[0]) > 1e-6:
            raise ValueError("The compensated contour path is not continuous.")
        points.extend(values if not points else values[1:])
        explorer.Next()
    if not points or math.dist(points[0], points[-1]) > 1e-6:
        raise ValueError("The compensated contour path is not closed.")
    points[-1] = points[0]
    return points, curved


def _setup_plane(face_selector: str):
    import cadquery as cq

    frames = {
        ">Z": ((1, 0, 0), (0, 0, 1), 1),
        "<Z": ((1, 0, 0), (0, 0, -1), -1),
        ">X": ((0, 1, 0), (1, 0, 0), 1),
        "<X": ((0, 1, 0), (-1, 0, 0), -1),
        ">Y": ((1, 0, 0), (0, 1, 0), -1),
        "<Y": ((1, 0, 0), (0, -1, 0), 1),
    }
    if face_selector not in frames:
        raise ValueError("The profile contour requires one of the six cardinal setup faces.")
    xdir, normal, mirror = frames[face_selector]
    return cq.Plane((0, 0, 0), xDir=xdir, normal=normal), mirror


def _replay_closed_contour(shape, path, tool):
    """Exact flat-cutter sweep from the closed, linearized tool-tip path.

    A two-dimensional circular offset avoids hundreds of overlapping capsule
    Booleans. The extrusion is the union of those same horizontal sweeps.
    The vertical entry lies within that channel above its final cutting plane.
    """
    import cadquery as cq

    feed_moves = [move for move in path.moves if move.move_type == "feed"]
    entry = path.moves[1]
    points = [entry.position] + [move.position for move in feed_moves]
    if not feed_moves or math.dist(points[0], points[-1]) > 1e-8 or any(abs(point[2] - entry.position[2]) > 1e-8 for point in points):
        raise ValueError("Native contour replay requires a closed horizontal cutter path.")
    outline = [(point[0], point[1]) for point in points[:-1]]
    # Clipped paths can split an unchanged straight segment into short edges.
    # Remove those collinear vertices before the two-dimensional offset.
    changed = True
    while changed and len(outline) > 3:
        changed = False
        for index, middle in enumerate(outline):
            before, after = outline[index - 1], outline[(index + 1) % len(outline)]
            first = (middle[0] - before[0], middle[1] - before[1])
            second = (after[0] - middle[0], after[1] - middle[1])
            cross = first[0] * second[1] - first[1] * second[0]
            if abs(cross) <= 1e-9 * max(1, math.hypot(*first) * math.hypot(*second)) and first[0] * second[0] + first[1] * second[1] >= 0:
                outline.pop(index)
                changed = True
                break
    center = cq.Workplane("XY").polyline(outline).close().val()
    try:
        outer = center.offset2D(tool.radius, kind="arc")
        inner = center.offset2D(-tool.radius, kind="arc")
        if len(outer) != 1:
            raise ValueError("The contour sweep has a divided outer boundary.")
        region = cq.Face.makeFromWires(outer[0], inner)
        cutter = cq.Solid.extrudeLinear(outer[0], inner, cq.Vector(0, 0, tool.flute_length)).translate((0, 0, entry.position[2]))
    except Exception as error:
        raise ValueError("The selected cutter channel cannot produce a valid native contour sweep.") from error
    if not region.isValid() or not cutter.isValid():
        raise ValueError("The selected cutter channel cannot produce a valid native contour sweep.")
    result = shape.newObject([_boolean(shape.val(), [cutter], "cut")])
    if not result.solids().vals() or not result.val().isValid():
        raise ValueError("The contour cutter path did not produce valid remaining material.")
    result, cleaned = clean_flat_result(result)
    return result, {"method": "selected_cutter_closed_path_sweep", "coordinate_system": "part_mm",
        "geometry_fidelity": "exact_cutter_sweep", "exact_segments": len(points),
        "sample_interval_mm": None, "sampled_poses": 0, "topology_repaired": False, "topology_cleaned": cleaned,
        "manufacturing_ready": False, "verification_status": "not_verified"}


def _layer_contour_replay(shape, local, plane, built, tool, bounds, finish, states, reports):
    """2.5D layer replay of the contour levels: removed volumes from 2D areas; one Boolean for the result."""
    replay = LayerReplay(local, bounds.zmax, [path.moves[1].position[2] for _, _, path, _ in built], tool)
    for index, role, path, has_curve in built:
        reports.append(finish(index, role, path, has_curve, replay.cut(path, "selected_cutter_closed_path_sweep")))
    if sum(report["removed_volume_mm3"] for report in reports) <= 1e-8:
        raise ValueError("The compensated contour does not remove material from the incoming stock.")
    result, cleaned = clean_flat_result(local.newObject([replay.result_shape()]))
    reports[-1]["topology_cleaned"] = cleaned
    final = result.val()
    for position, (index, role, path, _) in enumerate(built):
        local_state = final if position == len(built) - 1 else replay.state_shape(final, position)
        states.append({"shape": shape.newObject([local_state.transformShape(plane.rG)]), "pass_index": index,
            "pass_role": role, "label": f"After profile contour {role} pass {index}", "geometry_replay": reports[position]})
    return True


def run_profile_contour(shape, profile, *, tool, depth: float, side: str = "outside",
                        face_selector: str = ">Z", stepdown: float = 1,
                        finish_allowance: float = 0.1, chord_error: float = 0.01,
                        feed: float = 100, rpm: float | None = None):
    """Return native result, world-coordinate path, retained states and reports."""
    import cadquery as cq

    if side not in {"inside", "outside"}:
        raise ValueError("The profile contour side must be inside or outside.")
    depth = finite(depth, "contour depth", positive=True)
    stepdown = finite(stepdown, "contour stepdown", positive=True)
    allowance = finite(finish_allowance, "contour finish allowance")
    if allowance < 0:
        raise ValueError("The contour finish allowance must be non-negative.")
    chord = finite(chord_error, "contour chord error", positive=True)
    if tool is None or tool.tool_type != "flat_endmill":
        raise ValueError("The profile contour requires a selected flat end mill.")
    validate_cutter(tool, cutting_depth=depth)
    maximum = finite((tool.safe_limits or {}).get("max_stepdown_mm", tool.diameter / 2), "maximum contour stepdown", positive=True)
    stepdown = min(stepdown, maximum, tool.diameter / 2)
    wire, sampled_profile = _profile_wire(profile)
    plane, mirror = _setup_plane(face_selector)
    if mirror < 0:
        wire = wire.mirror("XZ")
    local = cq.Workplane("XY").newObject([plane.toLocalCoords(shape.val())])
    bounds = local.val().BoundingBox()
    if depth > bounds.zlen + 1e-8:
        raise ValueError("The contour depth exceeds the incoming material thickness along its cutter axis.")
    count = math.ceil(depth / stepdown)
    if count > 100:
        raise ValueError("The contour exceeds the native depth-pass limit. Divide the operation into smaller regions.")
    levels = [(bounds.zmax - depth * index / count, "roughing", allowance) for index in range(1, count + 1)]
    levels.append((bounds.zmax - depth, "finishing", 0))
    normal = list(plane.zDir.toTuple())
    safe = bounds.zmax + max(2, tool.radius)
    aggregate = Toolpath("profile_contour", safe_z=safe, tool_type=tool.tool_type,
        tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
        metadata={"coordinate_system": "part_mm", "tool_axis": normal,
            "face_selector": face_selector, "side": side, "cutter_compensation": "geometry_applied",
            "safe_axis_position_mm": safe, "chord_error_mm": chord,
            "maximum_stepdown_mm": stepdown, "finish_allowance_mm": allowance,
            "waste_discarded": False})
    built, curved = [], False
    for index, (z, role, leave) in enumerate(levels, 1):
        try:
            offsets = wire.offset2D((tool.radius + leave) * (1 if side == "outside" else -1), kind="arc")
        except Exception as error:
            raise ValueError("The selected cutter cannot produce a continuous compensated contour for this profile.") from error
        if len(offsets) != 1 or not offsets[0].isValid():
            raise ValueError("The selected cutter does not fit the profile. Its compensated contour is empty or divided.")
        points, has_curve = _linearized_wire(offsets[0], min(chord, tool.radius / 4), tangent_curves=side == "outside")
        curved |= has_curve
        path = Toolpath("profile_contour", safe_z=safe, tool_type=tool.tool_type,
            tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
            metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1]})
        x, y = points[0]
        path.append(ToolpathMove("rapid", (x, y, safe)))
        path.append(ToolpathMove("plunge", (x, y, z), feed=feed, spindle_rpm=rpm))
        path.moves.extend(ToolpathMove("feed", (x, y, z), feed=feed, spindle_rpm=rpm) for x, y in points[1:])
        x, y = points[-1]
        path.append(ToolpathMove("retract", (x, y, safe)))
        built.append((index, role, path, has_curve))

    def finish(index, role, path, has_curve, report):
        start = len(aggregate.moves)
        aggregate.moves.extend(replace(move, position=plane.toWorldCoords(move.position).toTuple()) for move in path.moves)
        report.update(pass_index=index, pass_role=role, move_start_index=start,
            move_end_index=len(aggregate.moves),
            tool_axis=normal, profile_fidelity="linearized_curve" if has_curve or sampled_profile else "linear_profile",
            chord_error_mm=chord if has_curve else None, source_profile_sampled=sampled_profile)
        return report

    states, reports = [], []
    done = False
    if not is_program_mode():
        try:
            done = _layer_contour_replay(shape, local, plane, built, tool, bounds, finish, states, reports)
        except LayerCheckUnavailable:
            # The B-Rep replay below remains for material that the layers cannot represent.
            aggregate.moves.clear()
            states.clear()
            reports.clear()
    if not done:
        result = local
        for index, role, path, has_curve in built:
            before = result.val().Volume()
            if is_program_mode() and index < len(levels):
                # Program mode cuts only at the finish level: the same moves, one cut (program_mode.py).
                report = {"program_mode": True}
            else:
                result, report = _replay_closed_contour(result, path, tool)
            report = finish(index, role, path, has_curve, report)
            report["removed_volume_mm3"] = before - result.val().Volume()
            reports.append(report)
            world = shape.newObject([result.val().transformShape(plane.rG)])
            states.append({"shape": world, "pass_index": index, "pass_role": role,
                "label": f"After profile contour {role} pass {index}", "geometry_replay": report})
    if shape.val().Volume() - states[-1]["shape"].val().Volume() <= 1e-8:
        raise ValueError("The compensated contour does not remove material from the incoming stock.")
    aggregate.metadata.update(material_passes=reports, profile_fidelity="linearized_curve" if curved or sampled_profile else "linear_profile")
    return states[-1]["shape"], aggregate, states, reports
