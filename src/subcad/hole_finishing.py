"""Hole finishing from the selected cutter and the actual incoming material."""
from .milling_replay import finite


def prepare_reaming(stock, diameter, depth, *, cx, cy, through, face_selector, entry_plane_mm=None):
    """Select the actual reamer first, then preserve or drill its pilot hole."""
    import cadquery as cq
    from .drilling_replay import point_geometry
    from .milling_replay import _boolean
    from .operations import DrillOp, ReamOp
    from .profile_milling import _setup_plane
    from .tool_selection import ToolRequirement, ToolUnavailableError, select_tool
    if not isinstance(through, bool):
        raise ValueError("Reaming requires an explicit through setting.")
    setup = stock._setups.get(stock._active_setup) if stock._active_setup else None
    face = setup.face_selector if setup else face_selector
    plane, mirror = _setup_plane(face)
    local = plane.toLocalCoords(stock._shape.val())
    bounds = local.BoundingBox()
    from .feature_depth import material_span
    span = material_span(stock._shape, face, cx=finite(cx, "reaming X position"), cy=finite(cy, "reaming Y position"),
                         radius=finite(diameter, "finished hole diameter", positive=True) / 2) if through else None
    depth = (span[1] if span else bounds.zlen) if through else finite(depth, "reaming depth", positive=True)
    if not through and entry_plane_mm is not None:
        entry = finite(entry_plane_mm, "reaming entry plane")
        if not bounds.zmin <= entry <= bounds.zmax:
            raise ValueError("The recorded reaming entry plane lies outside the incoming material bounds.")
        depth += bounds.zmax - entry
    cx, cy = finite(cx, "reaming X position"), finite(cy, "reaming Y position")
    diameter = finite(diameter, "finished hole diameter", positive=True)
    operation = ReamOp(diameter=diameter, depth=depth, cx=cx, cy=cy, through=through, face_selector=face,
        material=stock._material)
    _, tip, lead = point_geometry(operation.tool)
    travel = depth + lead
    if not through and travel > bounds.zlen + 1e-8:
        raise ValueError("The reamer lead passes through the incoming material. Use through reaming or change the blind depth.")
    corridor = cq.Solid.makeCylinder(tip, travel, cq.Vector(cx, cy * mirror, bounds.zmax - travel))
    contact = _boolean(local, [corridor], "common")
    if not contact.isValid():
        raise ValueError("The reamer pilot clearance check produced invalid geometry.")
    prepared = stock
    if contact.Solids() and contact.Volume() > 1e-8:
        pilot_depth = depth if through else travel
        selection = select_tool(ToolRequirement("drill", min_diameter_mm=tip * 2 + 1e-6,
            max_diameter_mm=diameter - 1e-6, min_flute_length_mm=pilot_depth, require_point_geometry=True,
            include_point_length_in_reach=True, prefer_largest=True, feature="reaming_pilot",
            reason="the pilot drill must clear the recorded reamer lead and leave material for the finished diameter"))
        if not selection.ok:
            selection.errors.append("Reaming requires an available pilot drill larger than the lead tip and smaller than the finished hole.")
            raise ToolUnavailableError(selection)
        drill = selection.selected_tool
        prepared = stock._apply_op(DrillOp(diameter=drill.diameter, depth=pilot_depth, cx=cx, cy=cy,
            through=through, face_selector=face, spot_drill=False, tool=drill,
            tool_selection=selection, material=stock._material))
        operation._pilot_preparation = {"method": "selected_pilot_drill", "diameter_mm": drill.diameter,
            "tool_id": drill.catalog_id, "tool_selection": selection.to_dict(), "allowance_qualified": False,
            "point_extension_mm": point_geometry(drill)[2]}
    else:
        operation._pilot_preparation = {"method": "existing_material_hole", "allowance_qualified": False}
    return prepared._apply_op(operation)


def apply_reaming(operation, shape):
    from .drilling_replay import run_drilling
    from .operations import _feed_rate, _spindle_rpm, _tool_dict, _tool_selection_dict
    result, operation._ream_toolpath, operation._material_pass_states, operation._pass_reports = run_drilling(
        shape, tool=operation.tool, diameter=operation.diameter, depth=operation.depth,
        cx=operation.cx, cy=operation.cy, face_selector=operation.face_selector, through=operation.through,
        feed=_feed_rate(operation.feeds_speeds), rpm=_spindle_rpm(operation.feeds_speeds), ream=True)
    operation.depth = operation._ream_toolpath.metadata["full_diameter_depth_mm"]
    operation._ream_toolpath.metadata.update(tool=_tool_dict(operation.tool),
        tool_selection=_tool_selection_dict(operation.tool_selection), pilot_clearance="native_brep_checked")
    return result


def apply_circular_milling(operation, shape):
    from .profile_pocket_milling import run_profile_pocket
    from .profile_milling import _setup_plane
    from .operations import _feed_rate, _spindle_rpm, _tool_dict, _tool_selection_dict
    diameter = finite(operation.diameter, "circular milling diameter", positive=True)
    if operation.tool.tool_type != "flat_endmill" or operation.tool.diameter > diameter:
        raise ValueError("Circular milling requires a selected flat end mill that fits the requested hole diameter.")
    if operation.through:
        from .feature_depth import material_span
        span = material_span(shape, operation.face_selector, cx=operation.cx, cy=operation.cy, radius=diameter / 2)
        if span:
            operation.start_depth, operation.depth = span
        else:
            plane, _ = _setup_plane(operation.face_selector)
            operation.depth = plane.toLocalCoords(shape.val()).BoundingBox().zlen
    profile = {"type": "circle", "diameter": diameter, "cx": operation.cx, "cy": operation.cy}
    result, operation._bore_toolpath, operation._material_pass_states, operation._pass_reports, operation._target_comparison = run_profile_pocket(
        shape, profile, tool=operation.tool, depth=operation.depth, through=operation.through,
        face_selector=operation.face_selector, stepdown=operation.stepdown,
        finish_allowance=operation.finish_allowance, chord_error=operation.chord_error,
        feed=_feed_rate(operation.feeds_speeds), rpm=_spindle_rpm(operation.feeds_speeds),
        start_depth=getattr(operation, "start_depth", 0.0))
    operation._bore_toolpath.operation = "bore"
    operation._bore_toolpath.metadata.update(tool=_tool_dict(operation.tool),
        tool_selection=_tool_selection_dict(operation.tool_selection), hole_diameter_mm=diameter,
        strategy="circular_milling", pass_plan={"strategy": "circular_milling", "total_passes": len(operation._pass_reports)})
    for state in operation._material_pass_states:
        state["label"] = f"After circular milling {state['pass_role']} pass {state['pass_index']}"
    return result


def hole_finishing_record(operation, *, ream=False):
    passes = getattr(operation, "_pass_reports", [])
    comparison = getattr(operation, "_target_comparison", {})
    if ream:
        reasons = ["The reamer uses its recorded rotational lead envelope and an existing pilot hole.",
            "Reamer flutes, runout, finishing allowance, feeds, and the requested hole tolerance require qualification.",
            "Reaming requires a qualified fixture and complete target verification."]
        fidelity = "rotational_reamer_envelope"
        if not operation.through:
            reasons.append("The blind hole retains the reamer lead below the full-diameter depth. Its floor requires review.")
            if (getattr(operation, "_pilot_preparation", None) or {}).get("point_extension_mm", 0) > 0:
                reasons.append("The pilot drill point extends below the reamer lead. Review the required blind-hole floor.")
    else:
        reasons = ["The hole uses circular milling with the selected end mill.",
            "Circular milling requires qualified entry, fixture, feed, spindle, roundness, and complete target verification."]
        fidelity = "linearized_profile_cutter_replay" if any(row.get("profile_fidelity") == "linearized_curve" for row in passes) else "exact_cutter_sweep"
        if fidelity == "linearized_profile_cutter_replay":
            reasons.append("The circular path uses linear segments. The requested hole tolerance is not verified.")
        if comparison.get("remaining_target_material_mm3", 0) > 1e-6:
            reasons.append(f"After circular milling, {comparison['remaining_target_material_mm3']:.6g} mm³ of requested hole material remains.")
        if comparison.get("overcut_volume_mm3", 0) > 1e-6:
            reasons.append(f"Circular milling removes {comparison['overcut_volume_mm3']:.6g} mm³ outside the requested hole. Engineering review is required.")
    if not passes:
        reasons.append("The hole has no retained selected-cutter replay. Its material result and tool motion are unavailable.")
    if not (operation.tool.physical_assembly or {}).get("owned"):
        reasons.append("The hole requires an owned physical cutter assembly.")
    return {"material_passes": passes, "operation_target_comparison": comparison,
        "pilot_preparation": getattr(operation, "_pilot_preparation", None),
        "geometry_fidelity": fidelity if passes else "unavailable", "geometry_status": "requires_review",
        "toolpath_status": "unverified" if passes else "unavailable", "manufacturing_ready": False,
        "verification_status": "not_verified", "achieved_tolerance_estimate_mm": None, "review_reasons": reasons}
