"""Retained cutter clearing around explicit, nested ruled loft sections."""
from __future__ import annotations

import math

from .milling_replay import _boolean, finite, validate_cutter, replay_pass
from .profile_milling import _profile_wire
from .profile_pocket_milling import run_profile_pocket
from .toolpath import Toolpath, ToolpathMove


def loft_sections(profiles):
    if not isinstance(profiles, (list, tuple)) or not 2 <= len(profiles) <= 50:
        raise ValueError("Loft milling requires 2–50 explicit section profiles with z_mm positions.")
    sections = []
    for item in profiles:
        if not isinstance(item, dict) or not isinstance(item.get("type"), str) or item["type"] not in {"circle", "rectangle"}:
            raise ValueError("Loft milling requires circle or axis-aligned rectangle sections. Other sections need another machining plan.")
        kind = item["type"]
        section = {"type": kind, "z_mm": finite(item.get("z_mm"), "loft section Z"),
            "cx": finite(item.get("cx", 0), "loft section X"),
            "cy": finite(item.get("cy", 0), "loft section Y")}
        if item.get("angle", 0) != 0 or any(key in item for key in ("points", "segments", "vertices", "center")):
            raise ValueError("Loft sections require explicit dimensions and cx/cy coordinates without rotated or substituted boundaries.")
        keys = ("diameter",) if kind == "circle" else ("length", "width")
        if set(item) - {"type", "z_mm", "cx", "cy", "angle", *keys}:
            raise ValueError("The loft section contains unsupported geometry fields. Supply its explicit dimensions and cx/cy coordinates.")
        section.update({key: finite(item.get(key), f"loft section {key}", positive=True) for key in keys})
        sections.append(section)
    for low, high in zip(sections, sections[1:]):
        if high["z_mm"] <= low["z_mm"]:
            raise ValueError("Loft section Z positions must increase from the base to the top.")
        if high["type"] != low["type"] or high["cx"] != low["cx"] or high["cy"] != low["cy"]:
            raise ValueError("The current loft path requires concentric sections of one type. Shifted or mixed sections need another machining plan.")
        if any(high[key] > low[key] for key in keys):
            raise ValueError("The loft has an overhang. A positive Z cutter cannot reach that wall with this plan.")
    return sections


def _section_at(sections, z):
    low, high = next((low, high) for low, high in zip(sections, sections[1:]) if low["z_mm"] <= z <= high["z_mm"])
    fraction = (z - low["z_mm"]) / (high["z_mm"] - low["z_mm"])
    keys = ("diameter",) if low["type"] == "circle" else ("length", "width")
    return {**low, "z_mm": z, **{key: low[key] + fraction * (high[key] - low[key]) for key in keys}}


def _target(shape, sections):
    import cadquery as cq
    segments = [cq.Solid.makeLoft([_profile_wire(section)[0].translate((0, 0, section["z_mm"]))
        for section in (low, high)], ruled=True) for low, high in zip(sections, sections[1:])]
    bounds = shape.val().BoundingBox()
    below = cq.Solid.makeBox(bounds.xlen + 2, bounds.ylen + 2, sections[0]["z_mm"] - bounds.zmin,
        cq.Vector(bounds.xmin - 1, bounds.ymin - 1, bounds.zmin))
    keep = _boolean(below, segments, "fuse")
    return _boolean(shape.val(), [keep], "common")


def _wall_paths(sections, *, tool, tolerance, safe_z, feed, rpm):
    """Compensate the flat cutter's rim along each accessible wall generator."""
    interval = min(tool.diameter / 8, tolerance / 4)
    spacing = min(tool.radius / 2, math.sqrt(4 * tool.radius * tolerance))
    paths, poses = [], 0
    for section_index, (low, high) in enumerate(reversed(list(zip(sections, sections[1:]))), 1):
        if low["type"] == "circle":
            count = max(12, math.ceil(math.pi * low["diameter"] / spacing))
            if count > 200:
                raise ValueError("The loft wall exceeds the finishing-path limit. Divide the operation into smaller regions.")
            directions = [(math.cos(2 * math.pi * index / count), math.sin(2 * math.pi * index / count)) for index in range(count)]
            pairs = [((high["cx"] + (high["diameter"] / 2 + tool.radius) * x,
                       high["cy"] + (high["diameter"] / 2 + tool.radius) * y, high["z_mm"]),
                      (low["cx"] + (low["diameter"] / 2 + tool.radius) * x,
                       low["cy"] + (low["diameter"] / 2 + tool.radius) * y, low["z_mm"])) for x, y in directions]
        else:
            pairs = []
            for axis, cross in (("length", "width"), ("width", "length")):
                count = max(1, math.ceil(low[cross] / spacing))
                if count > 200:
                    raise ValueError("The loft wall exceeds the finishing-path limit. Divide the operation into smaller regions.")
                for side in (-1, 1):
                    for index in range(count + 1):
                        fraction = -1 + 2 * index / count
                        points = []
                        for section in (high, low):
                            along = side * (section[axis] / 2 + tool.radius)
                            across = fraction * section[cross] / 2
                            x, y = (along, across) if axis == "length" else (across, along)
                            points.append((section["cx"] + x, section["cy"] + y, section["z_mm"]))
                        pairs.append(tuple(points))
        for start, end in pairs:
            ratio = math.dist(start, end) / interval if interval > 0 else math.inf
            if not math.isfinite(ratio) or poses + math.ceil(ratio) + 2 > 10000 or len(paths) >= 200:
                raise ValueError("The loft wall exceeds the finishing-pose limit. Divide the operation into smaller regions.")
            poses += math.ceil(ratio) + 2
            path = Toolpath("loft_mill", safe_z=safe_z, tool_type=tool.tool_type,
                tool_diameter_mm=tool.diameter, feed_mm_min=feed, spindle_rpm=rpm,
                metadata={"coordinate_system":"part_mm", "tool_axis":[0,0,1], "pass_role":"finishing",
                    "path_kind":"loft_wall_generator", "wall_section_index":section_index,
                    "sample_interval_mm":interval, "generator_spacing_mm":spacing})
            path.append(ToolpathMove("rapid", (start[0], start[1], safe_z)))
            path.append(ToolpathMove("plunge", start, feed=feed, spindle_rpm=rpm))
            path.append(ToolpathMove("feed", end, feed=feed, spindle_rpm=rpm))
            path.append(ToolpathMove("retract", (end[0], end[1], safe_z)))
            paths.append(path)
    return paths


def run_loft_milling(shape, profiles, *, tool, tolerance_mm, stepdown, chord_error, feed, rpm, face_selector=">Z", finish_walls=True):
    if face_selector != ">Z":
        raise ValueError("Loft milling requires a positive Z setup. Another cutter axis needs an explicit loft transform.")
    sections = loft_sections(profiles)
    tolerance = finite(tolerance_mm, "loft preview tolerance", positive=True)
    step = finite(stepdown, "loft stepdown", positive=True)
    chord = finite(chord_error, "loft chord error", positive=True)
    if not isinstance(finish_walls, bool):
        raise ValueError("The loft wall-finishing selection must be true or false.")
    if tool is None or tool.tool_type != "flat_endmill":
        raise ValueError("Loft clearing requires a flat end mill with known cutter geometry.")
    bounds = shape.val().BoundingBox()
    if sections[0]["z_mm"] <= bounds.zmin:
        raise ValueError("The loft base must leave incoming material below it.")
    if abs(sections[-1]["z_mm"] - bounds.zmax) > 1e-7:
        raise ValueError("The incoming material top must match the loft top. Correct the preceding facing operation and datum.")
    footprint = _profile_wire(sections[0])[0].BoundingBox()
    if footprint.xmin < bounds.xmin - 1e-7 or footprint.xmax > bounds.xmax + 1e-7 or footprint.ymin < bounds.ymin - 1e-7 or footprint.ymax > bounds.ymax + 1e-7:
        raise ValueError("The complete loft base must lie inside the incoming material bounds.")
    depth = bounds.zmax - sections[0]["z_mm"]
    validate_cutter(tool, cutting_depth=depth)
    step = min(step, tool.diameter / 2,
        finite((tool.safe_limits or {}).get("max_stepdown_mm", tool.diameter / 2), "maximum loft stepdown", positive=True))
    levels = []
    for low, high in reversed(list(zip(sections, sections[1:]))):
        keys = ("diameter",) if low["type"] == "circle" else ("length", "width")
        slope = math.sqrt(sum(((low[key] - high[key]) / 2) ** 2 for key in keys)) / (high["z_mm"] - low["z_mm"])
        interval = min(step, tolerance / slope) if slope > 0 else step
        if interval <= 0 or not math.isfinite((high["z_mm"] - low["z_mm"]) / interval):
            raise ValueError("The loft preview interval exceeds the depth-pass limit. Divide the operation into smaller regions.")
        count = max(1, math.ceil((high["z_mm"] - low["z_mm"]) / interval))
        if len(levels) + count > 100:
            raise ValueError("The loft exceeds the depth-pass limit. Divide the operation into smaller regions.")
        levels.extend(low["z_mm"] if index == count else high["z_mm"] - (high["z_mm"] - low["z_mm"]) * index / count
            for index in range(1, count + 1))
    target = _target(shape, sections)
    finishing_paths = _wall_paths(sections, tool=tool, tolerance=tolerance,
        safe_z=bounds.zmax + max(2, tool.radius), feed=feed, rpm=rpm) if finish_walls else []
    aggregate = Toolpath("loft_mill", safe_z=bounds.zmax + max(2, tool.radius), tool_type=tool.tool_type, tool_diameter_mm=tool.diameter,
        feed_mm_min=feed, spindle_rpm=rpm,
        metadata={"coordinate_system": "part_mm", "tool_axis": [0, 0, 1], "face_selector": ">Z",
            "cutter_compensation": "geometry_applied", "sections": sections, "depth_levels_mm": levels})
    result, states, reports = shape, [], []
    for level_index, z in enumerate(levels, 1):
        profile = _section_at(sections, z)
        result, path, retained, checks, _ = run_profile_pocket(result, profile, tool=tool,
            depth=bounds.zmax - z, face_selector=">Z", stepdown=depth, finish_allowance=0,
            chord_error=chord, feed=feed, rpm=rpm, outside=True)
        if len(states) + len(retained) > 1000:
            raise ValueError("The loft exceeds the retained pass limit. Divide the operation into smaller regions.")
        offset = len(aggregate.moves)
        aggregate.moves.extend(path.moves)
        for state, report in zip(retained, checks):
            number = len(states) + 1
            role = "finishing" if not finish_walls and level_index == len(levels) else "roughing"
            check = {**report, "pass_index": number, "pass_role": role, "depth_level_index": level_index,
                "section_z_mm": z, "move_start_index": offset + report["move_start_index"],
                "move_end_index": offset + report["move_end_index"], "manufacturing_ready": False}
            reports.append(check)
            states.append({**state, "pass_index": number, "pass_role": role,
                "label": f"After loft {role} pass {number}", "geometry_replay": check})
    clearing_remaining = _boolean(result.val(), [target], "cut").Volume()
    for path in finishing_paths:
        if len(states) >= 1000:
            raise ValueError("The loft exceeds the retained pass limit. Divide the operation into smaller regions.")
        before = result.val().Volume()
        result, check = replay_pass(result, path, tool, sample_interval_mm=path.metadata["sample_interval_mm"])
        number, start = len(states) + 1, len(aggregate.moves)
        aggregate.moves.extend(path.moves)
        check.update(pass_index=number, pass_role="finishing", machining_phase="wall_finishing",
            removed_volume_mm3=before - result.val().Volume(), move_start_index=start,
            move_end_index=len(aggregate.moves), wall_section_index=path.metadata["wall_section_index"],
            generator_spacing_mm=path.metadata["generator_spacing_mm"])
        reports.append(check)
        states.append({"shape":result, "pass_index":number, "pass_role":"finishing",
            "label":f"After loft wall-finishing pass {number}", "geometry_replay":check})
    missing = _boolean(target, [result.val()], "cut").Volume()
    remaining = _boolean(result.val(), [target], "cut").Volume()
    comparison = {"method": "native_brep_ruled_loft", "overcut_volume_mm3": max(0, missing),
        "remaining_target_material_mm3": max(0, remaining), "clearing_remaining_material_mm3": max(0, clearing_remaining),
        "wall_finishing_enabled":finish_walls, "tolerance_verified": False, "manufacturing_ready": False}
    if missing > 1e-6:
        raise ValueError("The selected loft cutter removes required material. Review the cutter compensation and path.")
    if shape.val().Volume() - result.val().Volume() <= 1e-8:
        raise ValueError("The loft path does not remove material from the incoming stock.")
    aggregate.metadata.update(material_passes=reports, operation_target_comparison=comparison)
    return result, aggregate, states, reports, comparison, depth
