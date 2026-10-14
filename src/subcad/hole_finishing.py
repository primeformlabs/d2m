"""Hole finishing from the selected cutter and the actual incoming material."""
from .milling_replay import finite


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
        plane, _ = _setup_plane(operation.face_selector)
        operation.depth = plane.toLocalCoords(shape.val()).BoundingBox().zlen
    profile = {"type": "circle", "diameter": diameter, "cx": operation.cx, "cy": operation.cy}
    result, operation._bore_toolpath, operation._material_pass_states, operation._pass_reports, operation._target_comparison = run_profile_pocket(
        shape, profile, tool=operation.tool, depth=operation.depth, through=operation.through,
        face_selector=operation.face_selector, stepdown=operation.stepdown,
        finish_allowance=operation.finish_allowance, chord_error=operation.chord_error,
        feed=_feed_rate(operation.feeds_speeds), rpm=_spindle_rpm(operation.feeds_speeds))
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
        "geometry_fidelity": fidelity if passes else "unavailable", "geometry_status": "requires_review",
        "toolpath_status": "unverified" if passes else "unavailable", "manufacturing_ready": False,
        "verification_status": "not_verified", "achieved_tolerance_estimate_mm": None, "review_reasons": reasons}
