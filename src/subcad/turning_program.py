"""Explicit CAM-intent spindle continuity for consecutive uses of one insert."""
from copy import deepcopy
import math

from .turning_geometry import finite, spindle_point_to_world
from .toolpath import ToolpathMove


def _identity(operation):
    tool = operation.get("tool") or {}
    assembly = tool.get("assembly") or {}
    geometry = assembly.get("cutting_geometry") or {}
    if not tool.get("catalog_id") or assembly.get("owned") is not True or not geometry.get("source_sha256"):
        raise ValueError("Continuous Turning phase needs an identified owned insert profile.")
    return tool["catalog_id"], tool.get("holder_id"), geometry["source_sha256"]


def _phase_path(path, start):
    meta, moves = path["metadata"], path["moves"]
    rpm = finite(path.get("spindle_rpm"), "spindle speed", positive=True)
    feed = finite(path.get("feed_mm_min"), "feed", positive=True)
    rapid = finite(path.get("rapid_rate_mm_min"), "rapid rate", positive=True)
    sign = meta.get("spindle_direction")
    if isinstance(sign, bool) or sign not in {-1, 1}:
        raise ValueError("Continuous Turning phase needs an explicit rotation direction.")
    phase, duration, previous = finite(start, "initial phase"), 0.0, None
    for record in moves:
        move = ToolpathMove.from_dict(record)
        if move.move_type not in {"rapid", "retract", "feed", "plunge", "dwell"} or not all(math.isfinite(v) for v in move.position):
            raise ValueError("Continuous Turning phase needs finite linear or dwell positions.")
        if move.spindle_rpm is not None and move.spindle_rpm != rpm:
            raise ValueError("A spindle speed change inside a path needs a recorded transition.")
        if move.feed is not None:
            finite(move.feed, "move feed", positive=True)
        if previous is not None:
            minutes = move.time_minutes_from(previous, feed, rapid)
            duration += minutes * 60
            phase += sign * rpm * math.tau * minutes
        record.setdefault("metadata", {})["spindle_phase_radians"] = phase
        previous = move
    return duration


def _transport(previous, current):
    """Author non-cutting transport through radial clearance in the same frame."""
    path, old_path = current["toolpath"], previous["toolpath"]
    meta = path["metadata"]
    first, last = path["moves"][0], old_path["moves"][-1]
    old = last["metadata"]
    new = first["metadata"]
    if first["type"] != "rapid":
        raise ValueError("A continuous Turning operation needs a recorded rapid start.")
    if any(record["type"] == "arc" for record in path["moves"]):
        raise ValueError("Continuous Turning phase needs supported linear or dwell moves.")
    bounds = meta["incoming_spindle_bounds_mm"]
    radius = math.hypot(max(abs(bounds["min"][0]), abs(bounds["max"][0])),
                        max(abs(bounds["min"][1]), abs(bounds["max"][1])))
    polygon = current["tool"]["assembly"]["cutting_geometry"]["polygon"]
    clearance = max(old["spindle_radius_mm"], new["spindle_radius_mm"], radius + 2 - min(p[1] for p in polygon))
    points = [(old["spindle_radius_mm"], old["spindle_z_mm"])]
    if math.dist(first["position"], last["position"]) > 1e-9:
        for point in [(clearance, old["spindle_z_mm"]), (clearance, new["spindle_z_mm"])]:
            if math.dist(point, points[-1]) > 1e-9:
                points.append(point)
    prefix = []
    if len(points) > 1 or math.dist(first["position"], last["position"]) > 1e-9:
        for radius, axial in points:
            prefix.append({"type": "rapid", "position": list(spindle_point_to_world(radius, axial,
                meta["spindle_axis"], meta["spindle_origin_mm"])), "spindle_rpm": path["spindle_rpm"],
                "metadata": {"spindle_radius_mm": radius, "spindle_z_mm": axial, "program_transition": True}})
        path["moves"] = prefix + path["moves"]
    meta.update(program_transition="radial_clearance_rapid" if prefix else "coincident_start",
        transition_verification="not_verified")


def continuous_turning_plan(plan: dict) -> dict:
    """Copy the source plan; preserve original operation-level phase exports."""
    result = deepcopy(plan)
    previous, blocked, duration, unavailable, count = None, False, 0.0, 0, 0
    for operation in result["operations"]:
        path = operation.get("toolpath")
        meta = path.get("metadata") if isinstance(path, dict) else None
        turning = str(operation.get("operation") or "").startswith("turn_")
        if not turning:
            previous, blocked = None, False
            continue
        count += 1
        try:
            if blocked:
                raise ValueError("An earlier Turning transition is unavailable in this program block.")
            if not isinstance(meta, dict) or meta.get("process") != "turn" or meta.get("spindle_phase_status") != "recorded":
                raise ValueError("The Turning program needs explicit phase, speed, feed, and rotation inputs.")
            if not path.get("moves"):
                raise ValueError("The Turning program has no recorded positions.")
            supplied = path["moves"][0]["metadata"]["spindle_phase_radians"]
            _identity(operation)
            phase = supplied
            if previous is not None:
                old_path = previous["toolpath"]
                old_meta = old_path["metadata"]
                if _identity(previous) != _identity(operation):
                    raise ValueError("The insert change needs a recorded tool-change transition.")
                if previous.get("setup") != operation.get("setup") or old_meta["stock_to_spindle_matrix"] != meta["stock_to_spindle_matrix"]:
                    raise ValueError("The Turning setup change needs a recorded spindle transition.")
                if old_path["spindle_rpm"] != path["spindle_rpm"] or old_meta["spindle_direction"] != meta["spindle_direction"]:
                    raise ValueError("The spindle speed or direction change needs a recorded transition.")
                phase = old_path["moves"][-1]["metadata"]["spindle_phase_radians"]
                _transport(previous, operation)
            duration += _phase_path(path, phase)
            meta.update(spindle_phase_scope="continuous_program", supplied_initial_phase_radians=supplied,
                source="cam_intent", transition_verification="not_verified")
            previous = operation
        except (KeyError, TypeError, ValueError) as error:
            unavailable += 1
            if isinstance(meta, dict):
                meta.update(spindle_phase_status="unavailable", spindle_phase_reason=str(error))
            operation.setdefault("review_reasons", []).append(str(error))
            previous, blocked = None, True
    result["turning_program_clock"] = {"source": "cam_intent", "duration_seconds": duration,
        "status": "partial" if unavailable else "ready" if count else "not_applicable",
        "transition_verification": "not_verified", "manufacturing_ready": False}
    return result
