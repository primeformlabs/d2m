"""Fixed-axis drill envelopes and retained material for each axial plunge."""
from __future__ import annotations

from dataclasses import replace
import math

from .milling_replay import _boolean, finite, clean_flat_result
from .profile_milling import _setup_plane
from .toolpath import Toolpath, ToolpathMove


def point_geometry(tool):
    if tool is None or tool.tool_type not in {"drill", "reamer"}:
        raise ValueError("Axial cutter replay requires a selected drill or reamer with recorded point geometry.")
    diameter = finite(tool.diameter, "drill diameter", positive=True)
    angle = finite(tool.tip_angle, "drill point angle", positive=True)
    tip = finite(tool.tip_diameter, "drill tip diameter")
    if angle > 180 or tip < 0 or tip > diameter:
        raise ValueError("The drill point angle must not exceed 180 degrees. Its tip diameter must fit the cutter diameter.")
    if tool.tool_type == "reamer" and not 0 < tip < diameter:
        raise ValueError("Reaming requires a recorded lead tip diameter smaller than the finished hole and greater than zero.")
    height = 0 if angle == 180 else (diameter - tip) / (2 * math.tan(math.radians(angle / 2)))
    if finite(tool.flute_length, "drill flute length", positive=True) <= height:
        raise ValueError("The drill flute length does not contain its recorded point geometry.")
    return diameter / 2, tip / 2, height


def axial_sweep(tool, start, end):
    """Exact rotational envelope of one downward, fixed-axis drill plunge."""
    import cadquery as cq
    radius, tip, height = point_geometry(tool)
    if start[:2] != end[:2] or end[2] > start[2]:
        raise ValueError("Native drilling requires a downward axial plunge in its setup frame.")
    extent = tool.flute_length + start[2] - end[2]
    if height == 0:
        cutter = cq.Solid.makeCylinder(radius, extent)
    else:
        profile = cq.Workplane("XZ").moveTo(0, 0)
        if tip > 0:
            profile = profile.lineTo(tip, 0)
        cutter = profile.lineTo(radius, height).lineTo(radius, extent).lineTo(0, extent).close().revolve(360, (0, 0), (0, 1)).val()
    return cutter.translate(end)


def run_drilling(shape, *, tool, diameter, depth, cx=0, cy=0, face_selector=">Z",
                 through=False, spot=False, peck_step=None, feed=100, rpm=None, ream=False):
    """Blind depth is the full-diameter depth; spot depth is the tip depth."""
    import cadquery as cq
    requested = finite(diameter, "hole diameter", positive=True)
    cx, cy = finite(cx, "drill X position"), finite(cy, "drill Y position")
    depth = finite(depth, "drill depth")
    if not isinstance(through, bool) or not isinstance(spot, bool):
        raise ValueError("The drilling through and spot settings must be boolean values.")
    if depth < 0 or (not through and depth <= 0):
        raise ValueError("The drilling depth must be positive for a blind hole or a spot.")
    radius, tip, point_height = point_geometry(tool)
    if tool.tool_type != ("reamer" if ream else "drill"):
        raise ValueError("The selected axial cutter type does not match the drilling or reaming operation.")
    if not spot and abs(tool.diameter - requested) > 1e-8:
        raise ValueError("The selected drill diameter does not match the requested hole diameter.")
    plane, mirror = _setup_plane(face_selector)
    local = cq.Workplane("XY").newObject([plane.toLocalCoords(shape.val())])
    bounds = local.val().BoundingBox()
    full_depth = bounds.zlen if through else depth
    if full_depth > bounds.zlen + 1e-8:
        raise ValueError("The drilling depth exceeds the incoming material thickness along its cutter axis.")
    travel = full_depth if spot else full_depth + point_height
    if not through and not spot and travel > bounds.zlen + 1e-8:
        raise ValueError("The drill point passes through the incoming material. Use through drilling or change the blind depth.")
    if ream:
        # The complete lead must enter an existing hole. Test the actual
        # incoming material, including preceding cuts and the blind floor.
        pilot = cq.Solid.makeCylinder(tip, travel, cq.Vector(cx, cy * mirror, bounds.zmax - travel))
        contact = _boolean(local.val(), [pilot], "common")
        if not contact.isValid() or (contact.Solids() and contact.Volume() > 1e-8):
            raise ValueError("The reamer lead has no clear pilot hole through its complete travel. Drill a suitable pilot hole first.")
    limit = min(finite(tool.flute_length, "drill flute length", positive=True),
        finite(tool.stickout_mm, "drill projection", positive=True),
        finite(tool.reach_mm or tool.flute_length, "drill reach", positive=True))
    if travel > limit + 1e-8:
        raise ValueError("The selected drill cannot reach the required tip depth. Include its point length in the reach check.")
    step = travel if peck_step is None else finite(peck_step, "drill peck step", positive=True)
    count = max(1, math.ceil(travel / step))
    if count > 100:
        raise ValueError("The drill exceeds the retained peck limit. Divide the operation into smaller depth ranges.")
    operation = "ream" if ream else "spot_drill" if spot else "peck_drill" if peck_step is not None else "drill"
    safe = bounds.zmax + max(2, radius)
    path = Toolpath(operation, safe_z=safe, tool_type=tool.tool_type, tool_diameter_mm=tool.diameter,
        feed_mm_min=feed, spindle_rpm=rpm, metadata={"coordinate_system": "part_mm",
            "tool_axis": list(plane.zDir.toTuple()), "face_selector": face_selector,
            "point_angle_deg": tool.tip_angle, "tip_diameter_mm": tool.tip_diameter,
            "point_length_mm": point_height, "full_diameter_depth_mm": None if spot else full_depth,
            "tip_depth_mm": travel, "depth_reference": "tip" if spot else "full_diameter"})
    result, states, reports = local, [], []
    for index in range(1, count + 1):
        start = (cx, cy * mirror, safe)
        end = (cx, cy * mirror, bounds.zmax - min(index * step, travel))
        cutter = axial_sweep(tool, start, end)
        before = result.val().Volume()
        contact = _boolean(result.val(), [cutter], "common")
        if not contact.isValid():
            raise ValueError("The drill contact is invalid. Review the point geometry and incoming material.")
        if contact.Solids() and contact.Volume() > 1e-9:
            remaining = _boolean(result.val(), [cutter], "cut")
            if not remaining.isValid() or not remaining.Solids():
                raise ValueError("The drill pass does not produce valid remaining material.")
            if remaining.Volume() > before + max(1e-6, before * 1e-9):
                raise ValueError("The drill pass adds material. Its geometry is invalid.")
            result, _ = clean_flat_result(result.newObject([remaining]))
        first = len(path.moves)
        moves = [ToolpathMove("rapid", start), ToolpathMove("plunge", end, feed=feed, spindle_rpm=rpm),
            ToolpathMove("retract", start)]
        path.moves.extend(replace(move, position=plane.toWorldCoords(move.position).toTuple()) for move in moves)
        report = {"pass_index": index, "pass_role": "reaming" if ream else "spotting" if spot else "drilling", "path_kind": "axial_plunge",
            "move_start_index": first, "move_end_index": len(path.moves), "removed_volume_mm3": before - result.val().Volume(),
            "method": "selected_reamer_axial_sweep" if ream else "selected_drill_axial_sweep",
            "geometry_fidelity": "rotational_reamer_envelope" if ream else "rotational_drill_envelope",
            "coordinate_system": "part_mm", "tool_axis": list(plane.zDir.toTuple()),
            "tip_depth_mm": bounds.zmax - end[2], "manufacturing_ready": False, "verification_status": "not_verified"}
        reports.append(report)
        states.append({"shape": shape.newObject([result.val().transformShape(plane.rG)]), "pass_index": index,
            "pass_role": report["pass_role"], "label": f"After {operation.replace('_', ' ')} pass {index}", "geometry_replay": report})
    if local.val().Volume() - result.val().Volume() <= 1e-8:
        raise ValueError("The drill path removes no material from the incoming stock. Review the hole position and existing geometry.")
    path.metadata.update(material_passes=reports, pass_plan={"strategy": operation, "total_passes": count})
    return states[-1]["shape"], path, states, reports


def apply_drilling(operation, shape, *, spot=False, peck_step=None):
    from .operations import _feed_rate, _spindle_rpm, _tool_dict, _tool_selection_dict
    result, operation._drill_toolpath, operation._material_pass_states, operation._pass_reports = run_drilling(
        shape, tool=operation.tool, diameter=operation.hole_diameter if spot else operation.diameter,
        depth=operation.depth, cx=operation.cx, cy=operation.cy, face_selector=operation.face_selector,
        through=not spot and operation.through, spot=spot, peck_step=peck_step,
        feed=_feed_rate(operation.feeds_speeds), rpm=_spindle_rpm(operation.feeds_speeds))
    operation._drill_toolpath.metadata.update(tool=_tool_dict(operation.tool),
        tool_selection=_tool_selection_dict(operation.tool_selection))
    if not spot and operation.through:
        operation.depth = operation._drill_toolpath.metadata["full_diameter_depth_mm"]
    return result


def drilling_record(operation):
    reasons = ["The drill uses its recorded rotational cutting envelope. Flute, chisel edge, and runout details require verification.",
        "Drilling requires qualified entry, fixture, feed, spindle, and complete target verification."]
    if getattr(operation, "through", True) is False:
        reasons.append("The blind hole includes the drill point below its full-diameter depth. A flat floor needs another operation.")
    if (operation.tool.physical_assembly or {}).get("tip_diameter_source") == "nominal_zero":
        reasons.append("The drill point uses a nominal zero tip diameter. Its physical chisel edge geometry requires verification.")
    return {"material_passes": getattr(operation, "_pass_reports", []),
        "geometry_fidelity": "rotational_drill_envelope", "geometry_status": "requires_review",
        "verification_status": "not_verified", "manufacturing_ready": False,
        "review_reasons": reasons}
