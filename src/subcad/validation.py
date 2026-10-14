"""SubCAD validation -- pre-operation feasibility and DFM checks."""

from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class ValidationIssue:
    """Structured validation issue while preserving a string message."""

    severity: str
    code: str
    message: str
    operation_index: Optional[int] = None
    operation: str = ""
    context: dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        return self.message


@dataclass
class ValidationReport:
    """Structured validation result returned by validate_all(structured=True)."""

    issues: list[ValidationIssue] = field(default_factory=list)

    @property
    def warnings(self) -> list[str]:
        return [issue.message for issue in self.issues if issue.severity == "warning"]

    @property
    def errors(self) -> list[str]:
        return [issue.message for issue in self.issues if issue.severity == "error"]

    @property
    def messages(self) -> list[str]:
        return [issue.message for issue in self.issues]

    @property
    def ok(self) -> bool:
        return not self.errors

    def append(self, issue: ValidationIssue) -> None:
        self.issues.append(issue)

    def extend(self, issues: list[ValidationIssue]) -> None:
        self.issues.extend(issues)

    def __iter__(self):
        return iter(self.messages)

    def __len__(self) -> int:
        return len(self.issues)

    def __getitem__(self, index):
        return self.messages[index]


def _issue(severity: str, code: str, message: str,
           op_index: Optional[int] = None, op: Optional[dict] = None,
           **context) -> ValidationIssue:
    return ValidationIssue(
        severity=severity,
        code=code,
        message=message,
        operation_index=op_index,
        operation=(op or {}).get("operation", ""),
        context={k: v for k, v in context.items() if v is not None},
    )


def _as_float(value, default: Optional[float] = None) -> Optional[float]:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _op_name(op: dict) -> str:
    return str(op.get("operation", "")).lower()


def _position_key(op: dict):
    pos = op.get("position")
    if not isinstance(pos, (list, tuple)) or len(pos) < 2:
        return None
    x = _as_float(pos[0])
    y = _as_float(pos[1])
    if x is None or y is None:
        return None
    face = op.get("face_selector", ">Z")
    setup = op.get("setup", "")
    return (round(x, 3), round(y, 3), face, setup)


def _depth_mm(op: dict) -> float:
    dims = op.get("dimensions") if isinstance(op.get("dimensions"), dict) else {}
    for key in ("depth_mm", "depth", "counterbore_depth_mm"):
        depth = _as_float(op.get(key, dims.get(key)))
        if depth is not None:
            return abs(depth)
    return 0.0


def _tool_diameter_mm(op: dict) -> Optional[float]:
    for key in ("tool_diameter_mm", "tool_diameter", "diameter_mm"):
        value = _as_float(op.get(key))
        if value is not None:
            return value
    tool = op.get("tool")
    if isinstance(tool, dict):
        return _as_float(tool.get("diameter_mm", tool.get("tool_diameter_mm")))
    return None


def _toolpath_records(toolpath) -> Optional[list]:
    if isinstance(toolpath, dict):
        if isinstance(toolpath.get("moves"), list):
            return toolpath["moves"]
        if isinstance(toolpath.get("positions"), list):
            return toolpath["positions"]
        return [toolpath]
    if isinstance(toolpath, list):
        return toolpath
    return None


def _toolpath_xyz(record) -> Optional[tuple[float, float, float]]:
    if isinstance(record, dict):
        if all(k in record for k in ("x", "y", "z")):
            values = (record.get("x"), record.get("y"), record.get("z"))
        else:
            values = record.get("position")
            if values is None:
                return None
    elif isinstance(record, (list, tuple)) and len(record) >= 3:
        values = record[:3]
    else:
        return None

    try:
        return (float(values[0]), float(values[1]), float(values[2]))
    except (TypeError, ValueError, IndexError):
        return None


def _toolpath_move_type(record) -> str:
    if not isinstance(record, dict):
        return ""
    return str(record.get("type", record.get("move_type", ""))).lower()


def _is_cutting_operation(op: dict) -> bool:
    op_type = _op_name(op)
    non_cutting = {"", "inspect", "inspection", "setup", "fixture", "probe"}
    return op_type not in non_cutting


def _stock_limits(stock_dims: dict) -> dict[str, Optional[float]]:
    return {
        "length": _as_float(stock_dims.get("length", stock_dims.get("x"))),
        "width": _as_float(stock_dims.get("width", stock_dims.get("y"))),
        "height": _as_float(stock_dims.get("height", stock_dims.get("z"))),
    }


def _xy_inside_stock(x: float, y: float, length: Optional[float], width: Optional[float],
                     margin: float) -> bool:
    x_ok = True
    y_ok = True
    if length is not None:
        x_ok = (-margin <= x <= length + margin) or (
            -length / 2.0 - margin <= x <= length / 2.0 + margin
        )
    if width is not None:
        y_ok = (-margin <= y <= width + margin) or (
            -width / 2.0 - margin <= y <= width / 2.0 + margin
        )
    return x_ok and y_ok


def _tool_flute_length_mm(op: dict, tool_specs: Optional[list] = None) -> Optional[float]:
    for key in (
        "flute_length_mm",
        "tool_flute_length_mm",
        "tool_flute_length",
        "flute_length",
    ):
        value = _as_float(op.get(key))
        if value is not None:
            return value

    tool = op.get("tool")
    if isinstance(tool, dict):
        value = _as_float(tool.get("flute_length_mm", tool.get("flute_length")))
        if value is not None:
            return value

    if not tool_specs:
        return None

    op_type = op.get("tool_type")
    op_dia = _tool_diameter_mm(op)
    for tool in tool_specs:
        if isinstance(tool, dict):
            tool_type = tool.get("tool_type")
            tool_dia = _as_float(tool.get("tool_diameter_mm", tool.get("diameter")))
            flute = _as_float(tool.get("flute_length_mm", tool.get("flute_length")))
        else:
            tool_type = getattr(tool, "tool_type", None)
            tool_dia = _as_float(getattr(tool, "diameter", None))
            flute = _as_float(getattr(tool, "flute_length", None))
        if flute is None:
            continue
        type_matches = not op_type or not tool_type or op_type == tool_type
        dia_matches = op_dia is None or tool_dia is None or abs(op_dia - tool_dia) < 0.001
        if type_matches and dia_matches:
            return flute
    return None


def check_wall_thickness(shape, operation, min_wall_mm: float = 1.0) -> list[str]:
    """Check if an operation would leave walls thinner than min_wall_mm."""
    return []  # placeholder, returns empty for now


def check_tool_reach(operation, stock_dims: dict, tool) -> list[str]:
    """Check if the tool can physically reach the operation position."""
    warnings = []
    # Check if drill/pocket depth exceeds tool flute_length
    if isinstance(tool, dict):
        flute = _as_float(tool.get("flute_length_mm", tool.get("flute_length")), 50.0)
        reach = _as_float(tool.get("reach_mm", tool.get("tool_reach_mm")))
    else:
        flute = _as_float(getattr(tool, 'flute_length', 50.0), 50.0)
        reach = _as_float(getattr(tool, 'reach', None))
    if isinstance(operation, dict):
        depth = _depth_mm(operation)
        reach = _as_float(operation.get("tool_reach_mm", operation.get("reach_mm")), reach)
    else:
        depth = _as_float(getattr(operation, 'depth', 0.0), 0.0)
    if depth > flute:
        warnings.append(f"Depth ({depth}mm) exceeds tool flute length ({flute}mm).")
    if reach is not None and depth > reach:
        warnings.append(f"Depth ({depth}mm) exceeds tool reach ({reach}mm).")
    return warnings


def check_operation_order(operations: list[dict]) -> list[str]:
    """Validate operation ordering (face_mill first, chamfer last, etc.)."""
    return [issue.message for issue in _check_operation_order(operations)]


def _check_tool_reach_and_geometry(
    operations: list[dict],
    stock_dims: dict,
    tool_specs: Optional[list] = None,
) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    stock_height = _as_float(stock_dims.get("height") or stock_dims.get("z"))

    for i, op in enumerate(operations):
        op_type = _op_name(op)
        depth = _depth_mm(op)
        tool_dia = _tool_diameter_mm(op)
        flute = _tool_flute_length_mm(op, tool_specs)

        if flute is not None and depth > flute:
            issues.append(_issue(
                "warning", "tool_flute_length",
                f"{op_type or 'operation'} at position {i} depth ({depth:g}mm) "
                f"exceeds tool flute length ({flute:g}mm).",
                i, op, depth_mm=depth, flute_length_mm=flute,
            ))

        tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
        assembly = tool.get("assembly") if isinstance(tool.get("assembly"), dict) else {}
        stickout = _as_float(
            assembly.get("stickout_mm", tool.get("stickout_mm", op.get("tool_stickout_mm")))
        )
        holder_length = _as_float(assembly.get("holder_length_mm", tool.get("holder_length_mm")))
        holder_diameter = _as_float(assembly.get("holder_diameter_mm", tool.get("holder_diameter_mm")))
        if stickout is not None and depth > stickout:
            issues.append(_issue(
                "warning", "tool_holder_stickout",
                f"{op_type or 'operation'} at position {i} depth ({depth:g}mm) "
                f"exceeds tool stickout ({stickout:g}mm); holder collision is likely.",
                i, op, depth_mm=depth, stickout_mm=stickout,
            ))
        if holder_length and holder_diameter and depth > max(stickout or 0.0, flute or 0.0):
            issues.append(_issue(
                "warning", "tool_holder_envelope",
                f"{op_type or 'operation'} at position {i} may require holder clearance "
                f"(holder diameter {holder_diameter:g}mm).",
                i, op, holder_diameter_mm=holder_diameter,
            ))
        if isinstance(tool, dict) and tool.get("inventory_count") == 0:
            issues.append(_issue(
                "error", "tool_unavailable",
                f"{op_type or 'operation'} at position {i} uses unavailable tool "
                f"'{tool.get('catalog_id') or tool.get('label') or tool.get('tool_type')}'.",
                i, op,
            ))

        reach = _as_float(op.get("tool_reach_mm", op.get("reach_mm")))
        if reach is not None and depth > reach:
            issues.append(_issue(
                "warning", "tool_reach",
                f"{op_type or 'operation'} at position {i} depth ({depth:g}mm) "
                f"exceeds tool reach ({reach:g}mm).",
                i, op, depth_mm=depth, tool_reach_mm=reach,
            ))

        if op.get("through") and stock_height is not None and depth > stock_height:
            issues.append(_issue(
                "warning", "through_depth",
                f"{op_type or 'operation'} at position {i} depth ({depth:g}mm) "
                f"exceeds stock height ({stock_height:g}mm); verify through-cut clearance.",
                i, op, depth_mm=depth, stock_height_mm=stock_height,
            ))

        if tool_dia is None:
            continue

        slot_width = _as_float(op.get("slot_width_mm"))
        if slot_width is not None and tool_dia > slot_width:
            issues.append(_issue(
                "warning", "tool_vs_slot_width",
                f"{op_type or 'operation'} at position {i} uses a {tool_dia:g}mm tool "
                f"for a {slot_width:g}mm slot.",
                i, op, tool_diameter_mm=tool_dia, slot_width_mm=slot_width,
            ))

        dims = op.get("dimensions") if isinstance(op.get("dimensions"), dict) else {}
        pocket_width = _as_float(dims.get("width", op.get("pocket_width_mm")))
        pocket_length = _as_float(dims.get("length", op.get("pocket_length_mm")))
        if op_type in ("pocket", "rough_pocket", "finish_pocket") and pocket_width is not None:
            limiting_width = min(v for v in (pocket_width, pocket_length) if v is not None)
            if tool_dia > limiting_width:
                issues.append(_issue(
                    "warning", "tool_vs_pocket_width",
                    f"{op_type} at position {i} uses a {tool_dia:g}mm tool "
                    f"for a {limiting_width:g}mm pocket opening.",
                    i, op, tool_diameter_mm=tool_dia, pocket_width_mm=limiting_width,
                ))

        corner_radius = _as_float(op.get("corner_radius_mm"))
        if corner_radius is not None and corner_radius > 0 and tool_dia / 2.0 > corner_radius:
            if not op.get("has_finish_pass"):
                issues.append(_issue(
                    "warning", "tool_vs_internal_corner_radius",
                    f"{op_type or 'operation'} at position {i} has internal corner "
                    f"radius {corner_radius:g}mm smaller than tool radius {tool_dia / 2.0:g}mm.",
                    i, op, tool_radius_mm=tool_dia / 2.0, corner_radius_mm=corner_radius,
                ))

        pocket_dia = _as_float(
            op.get("pocket_diameter_mm",
                   op.get("counterbore_diameter_mm", op.get("spot_face_diameter_mm")))
        )
        if pocket_dia is not None and op_type in (
            "circular_pocket", "counterbore", "spot_face", "bore"
        ) and tool_dia >= pocket_dia:
            issues.append(_issue(
                "warning", "tool_vs_pocket_diameter",
                f"{op_type} at position {i} uses a {tool_dia:g}mm tool "
                f"for a {pocket_dia:g}mm circular pocket.",
                i, op, tool_diameter_mm=tool_dia, pocket_diameter_mm=pocket_dia,
            ))

        hole_dia = _as_float(op.get("hole_diameter_mm", op.get("thread_diameter_mm")))
        if hole_dia is not None and op_type in ("bore", "thread_mill") and tool_dia >= hole_dia:
            issues.append(_issue(
                "warning", "tool_vs_hole_diameter",
                f"{op_type} at position {i} uses a {tool_dia:g}mm tool "
                f"for a {hole_dia:g}mm hole.",
                i, op, tool_diameter_mm=tool_dia, hole_diameter_mm=hole_dia,
            ))

    return issues


def _check_toolpaths(operations: list[dict], stock_dims: dict) -> list[ValidationIssue]:
    """Validate authored toolpath completeness and approximate stock fit."""
    issues: list[ValidationIssue] = []
    limits = _stock_limits(stock_dims)
    margin = _as_float(stock_dims.get("toolpath_margin_mm"), 0.01) or 0.01

    for i, op in enumerate(operations):
        if "toolpath" not in op:
            continue

        op_type = _op_name(op) or "operation"
        records = _toolpath_records(op.get("toolpath"))
        records = records if records is not None else []
        points = [p for p in (_toolpath_xyz(record) for record in records) if p is not None]

        if _is_cutting_operation(op) and not records:
            issues.append(_issue(
                "warning", "empty_toolpath",
                f"{op_type} at position {i} has an empty authored toolpath.",
                i, op,
            ))
            continue

        if not points:
            continue

        toolpath = op.get("toolpath")
        safe_z = None
        if isinstance(toolpath, dict):
            safe_z = _as_float(toolpath.get("safe_z"))
        safe_z = _as_float(op.get("safe_z_mm", op.get("safe_z")), safe_z)
        if safe_z is None:
            positive_z = [z for _, _, z in points if z > 0]
            safe_z = max(positive_z) if positive_z else 0.0

        final_z = points[-1][2]
        has_retract = any(_toolpath_move_type(record) == "retract" for record in records)
        if _is_cutting_operation(op) and final_z < safe_z - 0.01 and not has_retract:
            issues.append(_issue(
                "warning", "missing_retract_safe_z",
                f"{op_type} at position {i} toolpath does not retract to safe Z "
                f"({safe_z:g}mm).",
                i, op, final_z_mm=final_z, safe_z_mm=safe_z,
            ))

        depth = _depth_mm(op)
        cutting_z = [z for _, _, z in points if z <= 0.0]
        if depth > 0 and cutting_z:
            min_z = min(cutting_z)
            required_z = -depth
            if min_z > required_z + 0.01:
                issues.append(_issue(
                    "warning", "toolpath_above_requested_depth",
                    f"{op_type} at position {i} toolpath reaches z={min_z:g}mm, "
                    f"above requested depth z={required_z:g}mm.",
                    i, op, reached_z_mm=min_z, requested_depth_mm=depth,
                ))

        x_bad = []
        y_bad = []
        z_bad = []
        for x, y, z in points:
            if not _xy_inside_stock(x, y, limits["length"], limits["width"], margin):
                x_bad.append(x)
                y_bad.append(y)
            if limits["height"] is not None and z < -limits["height"] - margin:
                z_bad.append(z)
        if x_bad or z_bad:
            context = {
                "bounds": {
                    "min": [min(x for x, _, _ in points),
                            min(y for _, y, _ in points),
                            min(z for _, _, z in points)],
                    "max": [max(x for x, _, _ in points),
                            max(y for _, y, _ in points),
                            max(z for _, _, z in points)],
                },
                "stock_dimensions": limits,
            }
            issues.append(_issue(
                "warning", "toolpath_stock_bounds",
                f"{op_type} at position {i} has toolpath points outside approximate "
                "stock bounds.",
                i, op, **context,
            ))

    return issues


def _check_operation_order(operations: list[dict]) -> list[ValidationIssue]:
    """Validate operation ordering with structured issue output."""
    issues: list[ValidationIssue] = []
    op_names = [op.get("operation", "") for op in operations]

    # Face mill should be first
    face_mill_order_issue_indices = set()
    if "face_mill" in op_names and op_names[0] != "face_mill":
        first_face = op_names.index("face_mill")
        face_mill_order_issue_indices.add(first_face)
        issues.append(_issue(
            "warning", "face_mill_not_first",
            "Face mill should be the first operation for proper datum.",
            first_face, operations[first_face],
        ))

    first_cut_by_face: dict[tuple[str, str], int] = {}
    first_face_mill_by_face: dict[tuple[str, str], int] = {}
    for i, op in enumerate(operations):
        key = (str(op.get("setup", "")), str(op.get("face_selector", ">Z")))
        if _op_name(op) == "face_mill":
            first_face_mill_by_face.setdefault(key, i)
        elif _op_name(op) not in ("spot_drill",):
            first_cut_by_face.setdefault(key, i)
    for key, cut_i in first_cut_by_face.items():
        face_i = first_face_mill_by_face.get(key)
        if face_i is not None and face_i > cut_i and face_i not in face_mill_order_issue_indices:
            issues.append(_issue(
                "warning", "face_mill_after_cut",
                f"Face mill for setup/face {key} occurs after another cutting operation; "
                "establish the datum before feature machining.",
                face_i, operations[face_i], setup=key[0], face_selector=key[1],
            ))

    # Tap after drill
    drilled_keys: set = set()
    any_drill_seen = False
    for i, name in enumerate(op_names):
        op = operations[i]
        op_type = _op_name(op)
        if op_type in ("drill", "peck_drill"):
            any_drill_seen = True
            key = _position_key(op)
            if key is not None:
                drilled_keys.add(key)
            continue

        if op_type in ("tap", "ream", "bore", "thread_mill"):
            key = _position_key(op)
            has_preceding_drill = key in drilled_keys if key is not None else any_drill_seen
            if not has_preceding_drill:
                issues.append(_issue(
                    "warning", f"{op_type}_before_drill",
                    f"{op_type} at position {i} has no preceding drill operation.",
                    i, op,
                ))

    return issues


def _check_setup_accessibility(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    operations = process_plan_dict.get("operations", [])
    fixture = process_plan_dict.get("fixture")
    setups = process_plan_dict.get("setups") or {}

    if fixture and not setups:
        issues.append(_issue(
            "warning", "fixture_without_setups",
            "Fixture metadata exists but no setups are defined.",
        ))

    setup_faces: dict[str, str] = {}
    for name, setup in setups.items():
        face = setup.get("face_selector") if isinstance(setup, dict) else None
        setup_faces[name] = face or ""
        if not face:
            issues.append(_issue(
                "warning", "setup_missing_face",
                f"Setup '{name}' is missing a face selector.",
                setup=name,
            ))
        if isinstance(setup, dict) and not setup.get("work_offset"):
            issues.append(_issue(
                "warning", "setup_missing_work_offset",
                f"Setup '{name}' is missing a work offset.",
                setup=name,
            ))
        if face and face not in {">Z", "<Z", ">X", "<X", ">Y", "<Y"}:
            issues.append(_issue(
                "warning", "setup_unsupported_face",
                f"Setup '{name}' uses unsupported face selector '{face}'.",
                setup=name, face_selector=face,
            ))

    referenced_faces = set()
    for i, op in enumerate(operations):
        setup_name = op.get("setup")
        face = op.get("face_selector")
        if fixture and setups and not setup_name:
            issues.append(_issue(
                "warning", "operation_missing_setup",
                f"{_op_name(op) or 'operation'} at position {i} has fixture/setup "
                "metadata available but no setup assignment.",
                i, op,
            ))
        if setup_name and setup_name not in setups:
            issues.append(_issue(
                "warning", "operation_unknown_setup",
                f"{_op_name(op) or 'operation'} at position {i} references unknown setup "
                f"'{setup_name}'.",
                i, op, setup=setup_name,
            ))
        if setup_name in setup_faces and face and setup_faces[setup_name] and face != setup_faces[setup_name]:
            issues.append(_issue(
                "warning", "setup_face_mismatch",
                f"{_op_name(op) or 'operation'} at position {i} uses face '{face}' "
                f"but setup '{setup_name}' is defined for '{setup_faces[setup_name]}'.",
                i, op, setup=setup_name, op_face=face, setup_face=setup_faces[setup_name],
            ))
        if face and face != ">Z":
            referenced_faces.add(face)
        if face and face not in {">Z", "<Z", ">X", "<X", ">Y", "<Y"}:
            issues.append(_issue(
                "warning", "operation_unsupported_face",
                f"{_op_name(op) or 'operation'} at position {i} uses unsupported "
                f"face selector '{face}'.",
                i, op, face_selector=face,
            ))

    if referenced_faces and not setups:
        issues.append(_issue(
            "warning", "multi_face_without_setups",
            "Operations reference non-top faces but no setup metadata is defined.",
            faces=sorted(referenced_faces),
        ))
    elif referenced_faces:
        for i, op in enumerate(operations):
            face = op.get("face_selector")
            if face and face != ">Z" and not op.get("setup"):
                issues.append(_issue(
                    "warning", "non_top_face_missing_setup",
                    f"{_op_name(op) or 'operation'} at position {i} machines face "
                    f"'{face}' but has no setup assignment.",
                    i, op, face_selector=face,
                ))

    return issues


def _zone_contains(zone: dict, x: float, y: float) -> bool:
    margin = _as_float(zone.get("margin_mm"), 2.0) or 0.0
    return (
        (_as_float(zone.get("x_min"), 0.0) or 0.0) - margin <= x <=
        (_as_float(zone.get("x_max"), 0.0) or 0.0) + margin
        and (_as_float(zone.get("y_min"), 0.0) or 0.0) - margin <= y <=
        (_as_float(zone.get("y_max"), 0.0) or 0.0) + margin
    )


def _zone_bounds(zone: dict) -> Optional[dict[str, float]]:
    keys = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
    values = {key: _as_float(zone.get(key)) for key in keys}
    if any(value is None or not math.isfinite(value) for value in values.values()):
        return None
    if any(values[axis + "_min"] > values[axis + "_max"] for axis in ("x", "y", "z")):
        return None
    return values


def _tool_id(op: dict) -> str:
    tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
    assembly = tool.get("assembly") if isinstance(tool.get("assembly"), dict) else {}
    for source in (assembly, tool, op):
        for key in ("tool_id", "catalog_id", "selected_tool_id", "tool_number", "label"):
            value = source.get(key) if isinstance(source, dict) else None
            if value not in (None, ""):
                return str(value)
    return ""


def _tool_assembly(op: dict) -> dict:
    tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
    assembly = tool.get("assembly") if isinstance(tool.get("assembly"), dict) else {}
    return {
        "tool_id": _tool_id(op),
        "cutter_diameter_mm": _as_float(
            assembly.get("cutter_diameter_mm",
                         tool.get("diameter_mm", tool.get("tool_diameter_mm", op.get("tool_diameter_mm")))),
            0.0,
        ) or 0.0,
        "flute_length_mm": _as_float(
            assembly.get("flute_length_mm",
                         tool.get("flute_length_mm", op.get("flute_length_mm"))),
            0.0,
        ) or 0.0,
        "shank_diameter_mm": _as_float(
            assembly.get("shank_diameter_mm",
                         tool.get("shank_diameter_mm", op.get("shank_diameter_mm"))),
            0.0,
        ) or 0.0,
        "stickout_mm": _as_float(
            assembly.get("stickout_mm", tool.get("stickout_mm", op.get("stickout_mm"))),
            0.0,
        ) or 0.0,
        "holder_id": str(assembly.get("holder_id", tool.get("holder_id", ""))),
        "holder_diameter_mm": _as_float(
            assembly.get("holder_diameter_mm", tool.get("holder_diameter_mm")),
            0.0,
        ) or 0.0,
        "holder_length_mm": _as_float(
            assembly.get("holder_length_mm", tool.get("holder_length_mm")),
            0.0,
        ) or 0.0,
    }


def _tool_envelopes(op: dict, *, include_cutter: bool = True) -> list[dict[str, Any]]:
    assembly = _tool_assembly(op)
    for key, value in assembly.items():
        if key.endswith("_mm") and (not math.isfinite(value) or value < 0):
            assembly[key] = 0.0
    cutter_dia = assembly["cutter_diameter_mm"]
    flute = assembly["flute_length_mm"]
    shank_dia = assembly["shank_diameter_mm"] or cutter_dia
    stickout = assembly["stickout_mm"] or flute
    holder_dia = assembly["holder_diameter_mm"]
    holder_len = assembly["holder_length_mm"]

    envelopes: list[dict[str, Any]] = []
    if include_cutter and cutter_dia > 0:
        envelopes.append({
            "component": "cutter",
            "radius_mm": cutter_dia / 2.0,
            "z_start_mm": 0.0,
            "z_end_mm": max(flute, 0.0),
        })
    elif include_cutter:
        envelopes.append({
            "component": "centerline",
            "radius_mm": 0.0,
            "z_start_mm": 0.0,
            "z_end_mm": 0.0,
        })
    if shank_dia > 0 and stickout > flute:
        envelopes.append({
            "component": "shank",
            "radius_mm": shank_dia / 2.0,
            "z_start_mm": max(flute, 0.0),
            "z_end_mm": stickout,
        })
    if holder_dia > 0 and holder_len > 0:
        envelopes.append({
            "component": "holder",
            "radius_mm": holder_dia / 2.0,
            "z_start_mm": max(stickout, flute, 0.0),
            "z_end_mm": max(stickout, flute, 0.0) + holder_len,
        })
    return envelopes


def _check_tool_envelope_against_zones(
    op: dict,
    op_index: int,
    zones: list[dict],
    *,
    zone_type: str,
    code: str,
    fixture_id: str,
    include_cutter: bool,
) -> list[ValidationIssue]:
    from .fixture_sweep import cardinal_tool_axis, segment_envelope_hit

    if not zones:
        return []
    def unavailable(message, **context):
        return _issue("warning", "fixture_clearance_unverified", message, op_index, op,
                      fixture_id=fixture_id, verification_status="unavailable",
                      manufacturing_ready=False, **context)

    records = _toolpath_records(op.get("toolpath"))
    if not records:
        return [unavailable("Fixture clearance is unverified because the operation has no toolpath.")]
    try:
        axis = cardinal_tool_axis(op)
    except ValueError as error:
        return [unavailable(str(error))]

    tool_id = _tool_id(op)
    envelopes = _tool_envelopes(op, include_cutter=include_cutter)
    issues: list[ValidationIssue] = []
    segments = []
    previous = None
    path = op.get("toolpath")
    metadata = path.get("metadata", {}) if isinstance(path, dict) else {}
    entry = metadata.get("pass_entry_position")
    if entry is not None:
        previous = _toolpath_xyz(entry)
        if previous is None or not all(math.isfinite(value) for value in previous):
            issues.append(unavailable("The toolpath entry position is invalid."))
            previous = None
    for move_index, record in enumerate(records):
        point = _toolpath_xyz(record)
        kind = str(record.get("type", record.get("kind", "feed"))).lower() if isinstance(record, dict) else "feed"
        if point is None or not all(math.isfinite(value) for value in point):
            if kind == "dwell" and previous is not None:
                continue
            issues.append(unavailable("A toolpath position is invalid. Fixture clearance is unverified.", move_index=move_index))
            previous = None
            continue
        if kind not in {"rapid", "feed", "plunge", "retract", "dwell", "linear"}:
            issues.append(unavailable("Fixture clearance is unverified for this toolpath move type.",
                                      move_index=move_index, move_type=kind))
            # Inspect the endpoint, but never replace an unsupported arc with a chord.
            segments.append((point, point, move_index))
        else:
            segments.append((previous or point, point, move_index))
        previous = point

    emitted: set[tuple[str, str]] = set()
    for zone in zones:
        if not isinstance(zone, dict):
            issues.append(unavailable("A fixture zone is invalid."))
            continue
        bounds = _zone_bounds(zone)
        label = str(zone.get("label") or zone_type)
        margin = _as_float(zone.get("margin_mm"), 0.0)
        if bounds is None:
            issues.append(unavailable("A fixture zone has invalid bounds.", zone=label))
            continue
        if margin is None or not math.isfinite(margin) or margin < 0:
            issues.append(unavailable("A fixture zone has an invalid clearance margin.", zone=label))
            continue
        severity = str(zone.get("severity", "warning")).lower()
        if severity not in ("warning", "error"):
            severity = "warning"

        best_by_component: dict[str, dict[str, Any]] = {}
        for start, end, move_index in segments:
            for envelope in envelopes:
                component = envelope["component"]
                hit = segment_envelope_hit(start, end, axis, envelope, bounds, margin)
                if hit is None:
                    continue
                previous = best_by_component.get(component)
                if previous is None or hit["clearance_mm"] < previous["clearance_mm"]:
                    best_by_component[component] = {
                        **hit, "move_index": move_index,
                        "radius_mm": envelope["radius_mm"],
                    }

        for component, hit in best_by_component.items():
            key = (label, component)
            if key in emitted:
                continue
            emitted.add(key)
            issues.append(_issue(
                severity,
                code,
                f"The {component} envelope can intersect {zone_type} '{label}' "
                f"during {_op_name(op) or 'operation'} with tool '{tool_id or 'unknown'}'.",
                op_index,
                op,
                fixture_id=fixture_id,
                tool_id=tool_id or None,
                zone=label,
                zone_type=zone_type,
                component=component,
                clearance_mm=hit["clearance_mm"],
                envelope_radius_mm=hit["radius_mm"],
                point=hit["point"],
                tool_axis=list(axis),
                move_index=hit["move_index"],
                segment_fraction=hit["segment_fraction"],
                component_axis_range_mm=hit["component_axis_range_mm"],
                method="continuous_cardinal_envelope",
                verification_status="possible_collision",
                manufacturing_ready=False,
                zone_bounds=bounds,
            ))

    return issues


def _check_fixture_clearance(process_plan_dict: dict) -> list[ValidationIssue]:
    fixture = process_plan_dict.get("fixture")
    setups = process_plan_dict.get("setups") or {}
    if not fixture and not setups:
        return []

    issues: list[ValidationIssue] = []
    stock_dims = process_plan_dict.get("stock_dimensions") or {}
    stock_length = _as_float(stock_dims.get("length"))
    stock_width = _as_float(stock_dims.get("width"))
    checked_setups = set()
    for i, op in enumerate(process_plan_dict.get("operations", [])):
        setup_name = op.get("setup")
        setup = setups.get(setup_name, {}) if setup_name else {}
        setup = setup if isinstance(setup, dict) else {}
        actual_fixture = setup.get("fixture_spec")
        if not isinstance(actual_fixture, dict):
            actual_fixture = setup.get("fixture") if isinstance(setup.get("fixture"), dict) else fixture
        actual_fixture = actual_fixture if isinstance(actual_fixture, dict) else {}
        fixture_id = str(actual_fixture.get("name") or actual_fixture.get("id") or "")
        referenced_fixture = setup.get("fixture")
        if isinstance(referenced_fixture, str) and referenced_fixture != fixture_id:
            issues.append(_issue(
                "warning", "fixture_clearance_unverified",
                "The setup fixture geometry is missing. The latest fixture cannot replace it.",
                i, op, setup=setup_name, fixture_id=referenced_fixture,
                verification_status="unavailable", manufacturing_ready=False,
            ))
            continue
        if setup_name and setup_name not in setups:
            issues.append(_issue(
                "warning", "fixture_clearance_unverified",
                "The operation setup is missing. Fixture clearance is unverified.",
                i, op, setup=setup_name, verification_status="unavailable", manufacturing_ready=False,
            ))
            continue
        setup_key = (setup_name, fixture_id)
        if setup_key not in checked_setups:
            checked_setups.add(setup_key)
            max_opening = _as_float(actual_fixture.get("max_opening_mm"))
            jaw_width = _as_float(actual_fixture.get("jaw_width_mm"))
            if max_opening and stock_length is not None and stock_length > max_opening:
                issues.append(_issue("warning", "fixture_stock_fit",
                    f"Stock length ({stock_length:g} mm) exceeds fixture opening ({max_opening:g} mm).",
                    i, op, setup=setup_name, fixture_id=fixture_id,
                    stock_length_mm=stock_length, max_opening_mm=max_opening))
            if jaw_width and stock_width is not None and stock_width > jaw_width:
                issues.append(_issue("warning", "fixture_jaw_width",
                    f"Stock width ({stock_width:g} mm) exceeds fixture jaw width ({jaw_width:g} mm).",
                    i, op, setup=setup_name, fixture_id=fixture_id,
                    stock_width_mm=stock_width, jaw_width_mm=jaw_width))
        frame = actual_fixture.get("coordinate_frame")
        if frame and frame not in ("part_mm", {"name": "part_mm"}, {"origin": "part_mm"}):
            issues.append(_issue("warning", "fixture_clearance_unverified",
                "Fixture zones require part_mm coordinates. The fixture coordinate transform is unverified.",
                i, op, setup=setup_name, fixture_id=fixture_id,
                verification_status="unavailable", manufacturing_ready=False))
            continue
        zones = setup.get("clamping_zones", actual_fixture.get("clamping_zones", []))
        clearance_zones = setup.get("clearance_zones", actual_fixture.get("clearance_zones", []))
        if not isinstance(zones, list) or not isinstance(clearance_zones, list):
            issues.append(_issue(
                "warning", "fixture_clearance_unverified", "The fixture zones are invalid.",
                i, op, fixture_id=fixture_id, verification_status="unavailable", manufacturing_ready=False,
            ))
            continue
        if not zones and not clearance_zones:
            issues.append(_issue(
                "warning", "fixture_clearance_unverified", "The fixture has no collision zones. Fixture clearance is unverified.",
                i, op, fixture_id=fixture_id, verification_status="unavailable", manufacturing_ready=False,
            ))

        assembly = _tool_assembly(op)
        required = ("cutter_diameter_mm", "flute_length_mm", "shank_diameter_mm",
                    "stickout_mm", "holder_diameter_mm", "holder_length_mm")
        missing = [key for key in required if not math.isfinite(assembly[key]) or assembly[key] <= 0]
        if assembly["stickout_mm"] < assembly["flute_length_mm"]:
            missing.append("stickout_at_least_flute_length")
        if missing:
            issues.append(_issue(
                "warning", "fixture_clearance_unverified",
                "The cutter assembly dimensions are incomplete or invalid. Complete fixture clearance is unverified.",
                i, op, fixture_id=fixture_id, tool_id=_tool_id(op), missing_dimensions=missing,
                verification_status="unavailable", manufacturing_ready=False,
            ))

        op_type = _op_name(op)
        pos = op.get("position")
        # A 2D feature position cannot override a valid 3D travel check.
        if not op.get("toolpath") and isinstance(pos, (list, tuple)) and len(pos) >= 2:
            x = _as_float(pos[0])
            y = _as_float(pos[1])
            if x is not None and y is not None:
                for zone in zones:
                    if isinstance(zone, dict) and _zone_contains(zone, x, y):
                        label = zone.get("label", "clamping zone")
                        issues.append(_issue(
                            "warning", "fixture_clearance",
                            f"{op_type or 'operation'} at position {i} intersects "
                            f"fixture clamping zone '{label}'.",
                            i, op, zone=label, x=x, y=y,
                        ))

        issues.extend(_check_tool_envelope_against_zones(
            {**op, "face_selector": op.get("face_selector", setup.get("face_selector", ">Z"))},
            i,
            zones,
            zone_type="clamping zone",
            code="fixture_toolpath_clearance",
            fixture_id=fixture_id,
            include_cutter=True,
        ))
        issues.extend(_check_tool_envelope_against_zones(
            {**op, "face_selector": op.get("face_selector", setup.get("face_selector", ">Z"))},
            i,
            clearance_zones,
            zone_type="clearance zone",
            code="fixture_toolpath_clearance_zone",
            fixture_id=fixture_id,
            include_cutter=False,
        ))

        if op_type in ("face_mill", "contour"):
            issues.append(_issue(
                "warning", "fixture_full_face_clearance",
                f"{op_type} at position {i} covers the full face; verify fixture "
                "clamps are clear of the toolpath.",
                i, op,
            ))

    return issues


def _check_requested_wall_thickness(process_plan_dict: dict) -> list[ValidationIssue]:
    request = process_plan_dict.get("check_wall_thickness")
    min_wall = _as_float(process_plan_dict.get("min_wall_mm"), 1.0) or 1.0
    if not request:
        return []
    return [_issue(
        "warning", "wall_thickness_placeholder",
        f"Wall thickness validation was requested (min {min_wall:g}mm), "
        "but geometric wall analysis is not implemented yet.",
        min_wall_mm=min_wall,
    )]


def _check_plan_metadata(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    if not process_plan_dict.get("material"):
        issues.append(_issue(
            "warning", "missing_material",
            "Process plan is missing material metadata.",
        ))
    schema = process_plan_dict.get("schema_version")
    if schema and schema != "subcad.shop_floor.v1":
        issues.append(_issue(
            "warning", "unexpected_schema_version",
            f"Process plan schema_version is '{schema}', expected 'subcad.shop_floor.v1'.",
            schema_version=schema,
        ))
    elif "schema_version" not in process_plan_dict:
        issues.append(_issue(
            "warning", "missing_schema_version",
            "Process plan is missing schema_version metadata.",
        ))
    return issues


def _check_tool_selection(operations: list[dict]) -> list[ValidationIssue]:
    """Surface inventory-selection failures from operation records."""
    issues: list[ValidationIssue] = []
    for i, op in enumerate(operations):
        selection = op.get("tool_selection")
        if not isinstance(selection, dict):
            selection = (op.get("tool") or {}).get("selection") if isinstance(op.get("tool"), dict) else None
        if not isinstance(selection, dict):
            continue

        requirement = selection.get("requirement") if isinstance(selection.get("requirement"), dict) else {}
        for message in selection.get("errors") or []:
            issues.append(_issue(
                "error",
                "tool_selection_unavailable",
                f"{_op_name(op) or 'operation'} at position {i} has no safe inventory tool: {message}",
                i,
                op,
                selected_tool_id=selection.get("selected_tool_id"),
                requirement=requirement,
                rejected_tools=selection.get("rejected_tools", []),
            ))
        for message in selection.get("warnings") or []:
            issues.append(_issue(
                "warning",
                "tool_selection_warning",
                f"{_op_name(op) or 'operation'} at position {i}: {message}",
                i,
                op,
                selected_tool_id=selection.get("selected_tool_id"),
                requirement=requirement,
            ))
    return issues


def _machine_processes(machine: dict) -> set[str]:
    capabilities = machine.get("capabilities") if isinstance(machine, dict) else {}
    processes = capabilities.get("processes") if isinstance(capabilities, dict) else None
    return {str(process).lower() for process in (processes or [])}


def _machine_capability_set(machine: dict, key: str, *, normalize=str.lower) -> set[str]:
    capabilities = machine.get("capabilities") if isinstance(machine, dict) else {}
    values = capabilities.get(key) if isinstance(capabilities, dict) else None
    if not values:
        return set()
    return {normalize(str(value)) for value in values}


def _machine_rotary_axes(machine: dict) -> list[str]:
    if not isinstance(machine, dict):
        return []
    axes = machine.get("rotary_axes")
    if isinstance(axes, list):
        return [str(axis).upper() for axis in axes]
    rotary_limits = machine.get("rotary_limits") or {}
    return [str(axis).upper() for axis in rotary_limits.keys()]


def _machine_linear_limits(machine: dict) -> dict[str, tuple[float, float]]:
    limits: dict[str, tuple[float, float]] = {}
    if not isinstance(machine, dict):
        return limits
    for axis, pair in (machine.get("linear_limits") or {}).items():
        if isinstance(pair, (list, tuple)) and len(pair) == 2:
            lo = _as_float(pair[0])
            hi = _as_float(pair[1])
            if lo is not None and hi is not None:
                limits[str(axis).upper()] = (lo, hi)
    return limits


def _required_process(op: dict) -> str:
    process = str(op.get("process") or "").lower()
    if process:
        return process
    name = _op_name(op)
    if name.startswith("turn_"):
        return "turn"
    if name in {"mill_turn_setup", "hollow_bore", "tube_profile"}:
        return "mill_turn"
    if name in {"surface_mill", "sweep_mill", "loft_mill", "dome_mill"}:
        return "5axis"
    if name.startswith("wire_cut"):
        return "wire"
    return "mill"


def _operation_feed_mm_min(op: dict) -> Optional[float]:
    feeds = op.get("feeds_speeds") if isinstance(op.get("feeds_speeds"), dict) else {}
    value = feeds.get("feed_rate_mm_per_min", feeds.get("feed_rate_mm_min"))
    if value is None:
        toolpath = op.get("toolpath") if isinstance(op.get("toolpath"), dict) else {}
        value = toolpath.get("feed_mm_min", toolpath.get("feed_rate_mm_min"))
    return _as_float(value)


def _operation_spindle_rpm(op: dict) -> Optional[float]:
    feeds = op.get("feeds_speeds") if isinstance(op.get("feeds_speeds"), dict) else {}
    value = feeds.get("spindle_rpm", feeds.get("rpm"))
    if value is None:
        toolpath = op.get("toolpath") if isinstance(op.get("toolpath"), dict) else {}
        value = toolpath.get("spindle_rpm", toolpath.get("rpm"))
    return _as_float(value)


def _operation_family(op: dict) -> str:
    return str(op.get("operation_family") or _op_name(op) or "").lower()


def _operation_controller_cycle(op: dict) -> str:
    for key in ("controller_cycle", "gcode_cycle", "cycle"):
        if op.get(key):
            return str(op[key]).upper()
    return ""


def _operation_coolant(op: dict) -> str:
    value = op.get("coolant")
    if value is None and isinstance(op.get("feeds_speeds"), dict):
        value = op["feeds_speeds"].get("coolant")
    if value in (None, False):
        return ""
    coolant = str(value).lower()
    return "" if coolant in {"", "none", "off", "false"} else coolant


def _operation_requires_probe(op: dict) -> bool:
    if op.get("requires_probe") or op.get("probing_required"):
        return True
    return _op_name(op) in {"probe", "probe_datum", "inspect_probe"}


def _operation_tool_interface(op: dict) -> str:
    tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
    assembly = tool.get("assembly") if isinstance(tool.get("assembly"), dict) else {}
    for source in (op, tool, assembly):
        for key in ("tool_interface", "interface", "holder_interface", "spindle_interface"):
            if source.get(key):
                return str(source[key])
    return ""


def _tool_key_from_op(op: dict) -> tuple:
    tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
    return (
        tool.get("catalog_id") or op.get("tool_catalog_id") or tool.get("label") or tool.get("tool_type") or op.get("tool_type"),
        tool.get("diameter_mm") or tool.get("tool_diameter_mm") or op.get("tool_diameter_mm"),
    )


def _check_machine_context(process_plan_dict: dict, stock_dims: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    machine = process_plan_dict.get("machine")
    if not machine:
        return issues
    if not isinstance(machine, dict):
        return [_issue(
            "error", "machine_context_invalid",
            "Process plan machine context must be a dictionary.",
        )]

    machine_name = machine.get("name") or machine.get("machine_id") or "machine"
    processes = _machine_processes(machine)
    operation_families = _machine_capability_set(machine, "operation_families")
    controller_cycles = _machine_capability_set(machine, "controller_cycles", normalize=str.upper)
    coolant_modes = _machine_capability_set(machine, "coolant")
    rotary_axes = _machine_rotary_axes(machine)
    linear_limits = _machine_linear_limits(machine)
    capabilities = machine.get("capabilities") if isinstance(machine.get("capabilities"), dict) else {}

    stock_checks = [
        ("X", "length", "machine_travel_x"),
        ("Y", "width", "machine_travel_y"),
        ("Z", "height", "machine_travel_z"),
    ]
    for axis, dim_key, code in stock_checks:
        if axis not in linear_limits:
            continue
        stock_value = _as_float(stock_dims.get(dim_key))
        if stock_value is None:
            continue
        lo, hi = linear_limits[axis]
        travel = abs(hi - lo)
        if stock_value > travel + 1e-6:
            issues.append(_issue(
                "error", code,
                f"Stock {dim_key} ({stock_value:g}mm) exceeds {machine_name} "
                f"{axis}-axis travel ({travel:g}mm).",
                machine=machine.get("machine_id"), axis=axis,
                stock_dimension_mm=stock_value, travel_mm=travel,
            ))

    spindle = machine.get("spindle") if isinstance(machine.get("spindle"), dict) else {}
    max_rpm = _as_float(spindle.get("max_rpm"))
    max_feed = None
    if linear_limits:
        feed_values = []
        axis_limits = machine.get("axis_limits")
        for axis in linear_limits:
            axis_info = axis_limits.get(axis, {}) if isinstance(axis_limits, dict) else {}
            feed = _as_float(axis_info.get("max_feed")) if isinstance(axis_info, dict) else None
            if feed is not None:
                feed_values.append(feed)
        max_feed = min(feed_values) if feed_values else None

    tool_capacity = _as_float(capabilities.get("tool_capacity"))
    tool_count = len(process_plan_dict.get("tools_used") or [])
    if not tool_count:
        tool_keys = {
            key for key in (
                _tool_key_from_op(op)
                for op in process_plan_dict.get("operations", [])
            )
            if key[0] or key[1]
        }
        tool_count = len(tool_keys)
    if tool_capacity is not None and tool_count > tool_capacity:
        issues.append(_issue(
            "error", "machine_tool_capacity_exceeded",
            f"Process plan uses {tool_count} tools, above {machine_name} "
            f"tool capacity {tool_capacity:g}.",
            machine=machine.get("machine_id"),
            tool_count=tool_count,
            tool_capacity=tool_capacity,
        ))

    for i, op in enumerate(process_plan_dict.get("operations", [])):
        family = _operation_family(op)
        if operation_families and family and family not in operation_families:
            issues.append(_issue(
                "error", "machine_operation_unsupported",
                f"{_op_name(op) or 'operation'} is not listed in {machine_name} "
                "supported operation families.",
                i, op, operation_family=family, machine=machine.get("machine_id"),
            ))

        required = _required_process(op)
        if required in {"turn", "mill_turn"} and required not in processes:
            issues.append(_issue(
                "error", "machine_process_unsupported",
                f"{_op_name(op) or 'operation'} requires {required} capability, "
                f"but {machine_name} supports {sorted(processes) or ['unspecified']}.",
                i, op, required_process=required, machine=machine.get("machine_id"),
            ))
        elif required == "5axis" and "5axis" not in processes:
            issues.append(_issue(
                "error", "machine_requires_5axis",
                f"{_op_name(op) or 'operation'} requires 5-axis capability, "
                f"but {machine_name} has rotary axes {rotary_axes or ['none']}.",
                i, op, rotary_axes=rotary_axes, machine=machine.get("machine_id"),
            ))
        elif required == "wire" and "wire" not in processes:
            issues.append(_issue(
                "error", "machine_process_unsupported",
                f"{_op_name(op) or 'operation'} requires wire EDM capability, "
                f"but {machine_name} supports {sorted(processes) or ['unspecified']}.",
                i, op, required_process=required, machine=machine.get("machine_id"),
            ))

        face = str(op.get("face_selector") or ">Z")
        if face not in {">Z", "<Z"} and not rotary_axes and not op.get("setup"):
            issues.append(_issue(
                "error", "machine_setup_required",
                f"{_op_name(op) or 'operation'} on face {face} requires an indexed setup "
                f"or rotary machine capability on {machine_name}.",
                i, op, face_selector=face, machine=machine.get("machine_id"),
            ))

        cycle = _operation_controller_cycle(op)
        if cycle and controller_cycles and cycle.upper() not in controller_cycles:
            issues.append(_issue(
                "error", "machine_controller_cycle_unsupported",
                f"{_op_name(op) or 'operation'} requests controller cycle {cycle}, "
                f"not declared for {machine_name}.",
                i, op, controller_cycle=cycle, machine=machine.get("machine_id"),
            ))

        coolant = _operation_coolant(op)
        if coolant and coolant_modes and coolant not in coolant_modes:
            issues.append(_issue(
                "error", "machine_coolant_unsupported",
                f"{_op_name(op) or 'operation'} requests coolant '{coolant}', "
                f"not declared for {machine_name}.",
                i, op, coolant=coolant, machine=machine.get("machine_id"),
            ))

        if _operation_requires_probe(op) and not bool(capabilities.get("probing")):
            issues.append(_issue(
                "error", "machine_probe_unsupported",
                f"{_op_name(op) or 'operation'} requires probing, but {machine_name} "
                "does not declare probing support.",
                i, op, machine=machine.get("machine_id"),
            ))

        required_interface = _operation_tool_interface(op)
        machine_interface = str(capabilities.get("tool_interface") or "")
        if required_interface and machine_interface and required_interface.upper() != machine_interface.upper():
            issues.append(_issue(
                "error", "machine_tool_interface_mismatch",
                f"{_op_name(op) or 'operation'} uses {required_interface} tooling, "
                f"but {machine_name} declares {machine_interface}.",
                i, op,
                required_tool_interface=required_interface,
                machine_tool_interface=machine_interface,
                machine=machine.get("machine_id"),
            ))

        rpm = _operation_spindle_rpm(op)
        if max_rpm is not None and rpm is not None and rpm > max_rpm + 1e-6:
            issues.append(_issue(
                "error", "machine_spindle_rpm_exceeded",
                f"{_op_name(op) or 'operation'} requests {rpm:g} RPM, "
                f"above {machine_name} spindle limit {max_rpm:g} RPM.",
                i, op, requested_rpm=rpm, max_rpm=max_rpm,
                machine=machine.get("machine_id"),
            ))

        feed = _operation_feed_mm_min(op)
        if max_feed is not None and feed is not None and feed > max_feed + 1e-6:
            issues.append(_issue(
                "error", "machine_feed_exceeded",
                f"{_op_name(op) or 'operation'} requests feed {feed:g} mm/min, "
                f"above {machine_name} axis limit {max_feed:g} mm/min.",
                i, op, requested_feed_mm_min=feed, max_feed_mm_min=max_feed,
                machine=machine.get("machine_id"),
            ))

    return issues


def _controller_capability_set(controller: dict, key: str, *, normalize=str.lower) -> set[str]:
    if not isinstance(controller, dict):
        return set()
    values = controller.get(key)
    supports = controller.get("supports") if isinstance(controller.get("supports"), dict) else {}
    capabilities = (
        controller.get("capabilities")
        if isinstance(controller.get("capabilities"), dict)
        else {}
    )
    if not values:
        values = supports.get(key)
    if not values:
        values = capabilities.get(key)
    if key == "cycles" and not values:
        values = controller.get("controller_cycles", capabilities.get("controller_cycles"))
    if key == "coolant_codes" and not values:
        values = controller.get(
            "coolant_modes",
            controller.get("coolant", capabilities.get("coolant")),
        )
    return {normalize(str(value)) for value in (values or [])}


def _controller_bool(controller: dict, key: str) -> bool:
    supports = controller.get("supports") if isinstance(controller.get("supports"), dict) else {}
    capabilities = (
        controller.get("capabilities")
        if isinstance(controller.get("capabilities"), dict)
        else {}
    )
    if key == "macros":
        return bool(controller.get(key, supports.get(key, capabilities.get("macro_support", False))))
    return bool(controller.get(key, supports.get(key, capabilities.get(key, False))))


def _operation_rotary_mode(op: dict) -> str:
    for key in ("rotary_mode", "rotary_strategy", "indexing_mode"):
        if op.get(key):
            return str(op[key]).lower()
    toolpath = op.get("toolpath") if isinstance(op.get("toolpath"), dict) else {}
    return str(toolpath.get("rotary_mode") or "").lower()


def _operation_work_offset(op: dict) -> str:
    return str(op.get("work_offset") or op.get("coordinate_system") or "").upper()


def _operation_requires_macro(op: dict) -> bool:
    if op.get("requires_macro") or op.get("uses_macro"):
        return True
    toolpath = op.get("toolpath") if isinstance(op.get("toolpath"), dict) else {}
    return bool(toolpath.get("requires_macro") or toolpath.get("uses_macro"))


def _operation_requires_rigid_tapping(op: dict) -> bool:
    if op.get("rigid_tapping") or op.get("requires_rigid_tapping"):
        return True
    return _operation_controller_cycle(op) == "G84" and _op_name(op) in {"tap", "threaded_hole"}


def _check_controller_context(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    controller = process_plan_dict.get("controller")
    if not controller:
        return issues
    if not isinstance(controller, dict):
        return [_issue(
            "error", "controller_context_invalid",
            "Process plan controller context must be a dictionary.",
        )]

    controller_name = controller.get("name") or controller.get("controller_id") or "controller"
    cycles = _controller_capability_set(controller, "cycles", normalize=str.upper)
    rotary_modes = _controller_capability_set(controller, "rotary_modes")
    coolant_codes = _controller_capability_set(controller, "coolant_codes")
    work_offsets = _controller_capability_set(controller, "coordinate_systems", normalize=str.upper)

    declared_offsets = set(process_plan_dict.get("work_offsets") or {})
    for setup in (process_plan_dict.get("setups") or {}).values():
        if isinstance(setup, dict) and setup.get("work_offset"):
            declared_offsets.add(str(setup["work_offset"]).upper())
    setup_intent = process_plan_dict.get("setup_intent")
    if isinstance(setup_intent, dict) and setup_intent.get("work_offset"):
        declared_offsets.add(str(setup_intent["work_offset"]).upper())
    for offset in sorted(declared_offsets):
        if work_offsets and offset not in work_offsets:
            issues.append(_issue(
                "error", "controller_work_offset_unsupported",
                f"Work offset {offset} is not declared for {controller_name}.",
                controller=controller.get("controller_id"), work_offset=offset,
            ))

    for i, op in enumerate(process_plan_dict.get("operations", [])):
        cycle = _operation_controller_cycle(op)
        if cycle and cycles and cycle not in cycles:
            issues.append(_issue(
                "error", "controller_cycle_unsupported",
                f"{_op_name(op) or 'operation'} requests controller cycle {cycle}, "
                f"not declared for {controller_name}.",
                i, op, controller_cycle=cycle, controller=controller.get("controller_id"),
            ))

        coolant = _operation_coolant(op)
        if coolant and coolant_codes and coolant not in coolant_codes:
            issues.append(_issue(
                "error", "controller_coolant_unsupported",
                f"{_op_name(op) or 'operation'} requests coolant '{coolant}', "
                f"not declared for {controller_name}.",
                i, op, coolant=coolant, controller=controller.get("controller_id"),
            ))

        rotary_mode = _operation_rotary_mode(op)
        if rotary_mode and rotary_modes and rotary_mode not in rotary_modes:
            issues.append(_issue(
                "error", "controller_rotary_mode_unsupported",
                f"{_op_name(op) or 'operation'} requests rotary mode '{rotary_mode}', "
                f"not declared for {controller_name}.",
                i, op, rotary_mode=rotary_mode, controller=controller.get("controller_id"),
            ))

        work_offset = _operation_work_offset(op)
        if work_offset and work_offsets and work_offset not in work_offsets:
            issues.append(_issue(
                "error", "controller_work_offset_unsupported",
                f"{_op_name(op) or 'operation'} requests work offset {work_offset}, "
                f"not declared for {controller_name}.",
                i, op, work_offset=work_offset, controller=controller.get("controller_id"),
            ))

        if _operation_requires_probe(op) and not _controller_bool(controller, "probing"):
            issues.append(_issue(
                "error", "controller_probe_unsupported",
                f"{_op_name(op) or 'operation'} requires probing, but {controller_name} "
                "does not declare probing support.",
                i, op, controller=controller.get("controller_id"),
            ))

        if _operation_requires_rigid_tapping(op) and not _controller_bool(controller, "rigid_tapping"):
            issues.append(_issue(
                "error", "controller_rigid_tapping_unsupported",
                f"{_op_name(op) or 'operation'} requires rigid tapping, but "
                f"{controller_name} does not declare it.",
                i, op, controller=controller.get("controller_id"),
            ))

        if _operation_requires_macro(op) and not _controller_bool(controller, "macros"):
            issues.append(_issue(
                "error", "controller_macro_unsupported",
                f"{_op_name(op) or 'operation'} requires controller macros, but "
                f"{controller_name} does not declare macro support.",
                i, op, controller=controller.get("controller_id"),
            ))

    return issues


def _check_setup_intent_context(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    setup_intent = process_plan_dict.get("setup_intent")
    if not setup_intent:
        return issues
    if not isinstance(setup_intent, dict):
        return [_issue(
            "error", "setup_intent_invalid",
            "Process plan setup intent must be a dictionary.",
        )]

    primary_face = str(setup_intent.get("primary_face") or setup_intent.get("face_selector") or ">Z")
    accessible_faces = {
        str(face)
        for face in (setup_intent.get("accessible_faces") or [primary_face])
    }
    allow_reorientation = bool(setup_intent.get("allow_reorientation"))
    setup_count = int(_as_float(setup_intent.get("setup_count"), 1) or 1)

    for face in accessible_faces | {primary_face}:
        if face not in {">Z", "<Z", ">X", "<X", ">Y", "<Y"}:
            issues.append(_issue(
                "warning", "setup_intent_unsupported_face",
                f"Setup intent references unsupported face selector '{face}'.",
                face_selector=face,
            ))

    if not setup_intent.get("workholding"):
        issues.append(_issue(
            "warning", "setup_intent_missing_workholding",
            "Setup intent is missing workholding/fixture details.",
        ))

    for i, op in enumerate(process_plan_dict.get("operations", [])):
        face = str(op.get("face_selector") or primary_face or ">Z")
        if face not in accessible_faces:
            issues.append(_issue(
                "error", "setup_face_not_accessible",
                f"{_op_name(op) or 'operation'} uses face {face}, not accessible "
                "from the declared setup intent.",
                i, op, face_selector=face, accessible_faces=sorted(accessible_faces),
            ))
        elif face != primary_face and not allow_reorientation and setup_count <= 1:
            issues.append(_issue(
                "error", "setup_reorientation_required",
                f"{_op_name(op) or 'operation'} uses face {face}, which requires "
                "another setup or explicit reorientation intent.",
                i, op, face_selector=face, primary_face=primary_face,
            ))

    return issues


def _tool_record_id(tool: dict) -> str:
    for key in ("tool_id", "id", "catalog_id", "label", "tool_number"):
        if tool.get(key) not in (None, ""):
            return str(tool[key])
    return ""


def _tool_record_type(tool: dict) -> str:
    return str(tool.get("tool_type") or tool.get("type") or "")


def _tool_record_diameter(tool: dict) -> Optional[float]:
    return _as_float(
        tool.get("diameter_mm", tool.get("tool_diameter_mm", tool.get("cutter_diameter_mm")))
    )


def _tooling_tool_records(tooling: dict) -> list[dict]:
    records = tooling.get("tools", tooling.get("assemblies", [])) if isinstance(tooling, dict) else []
    return [record for record in records if isinstance(record, dict)]


def _find_tool_record_for_op(op: dict, tools: list[dict]) -> Optional[dict]:
    tool_id = _tool_id(op)
    if tool_id:
        for tool in tools:
            if _tool_record_id(tool) == tool_id:
                return tool

    op_type = str(op.get("tool_type") or (op.get("tool") or {}).get("tool_type") or "")
    op_dia = _tool_diameter_mm(op)
    for tool in tools:
        if op_type and _tool_record_type(tool) != op_type:
            continue
        tool_dia = _tool_record_diameter(tool)
        if op_dia is None or tool_dia is None or abs(op_dia - tool_dia) < 0.001:
            return tool
    return None


def _tooling_magazine(tooling: dict) -> dict:
    return tooling.get("magazine") if isinstance(tooling.get("magazine"), dict) else {}


def _check_tooling_context(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    tooling = process_plan_dict.get("tooling")
    if not tooling:
        return issues
    if not isinstance(tooling, dict):
        return [_issue(
            "error", "tooling_context_invalid",
            "Process plan tooling context must be a dictionary.",
        )]

    tools = _tooling_tool_records(tooling)
    magazine = _tooling_magazine(tooling)
    capacity = _as_float(magazine.get("capacity", magazine.get("tool_capacity")))
    magazine_interface = str(magazine.get("interface") or magazine.get("tool_interface") or "")
    machine = process_plan_dict.get("machine") if isinstance(process_plan_dict.get("machine"), dict) else {}
    machine_caps = machine.get("capabilities") if isinstance(machine.get("capabilities"), dict) else {}
    machine_interface = str(machine_caps.get("tool_interface") or "")

    if capacity is not None and len(tools) > capacity:
        issues.append(_issue(
            "error", "tooling_magazine_capacity_exceeded",
            f"Tooling context defines {len(tools)} tools, above magazine capacity {capacity:g}.",
            tool_count=len(tools), magazine_capacity=capacity,
        ))

    if machine_interface and magazine_interface and machine_interface.upper() != magazine_interface.upper():
        issues.append(_issue(
            "error", "tooling_machine_interface_mismatch",
            f"Tooling magazine interface {magazine_interface} does not match "
            f"machine interface {machine_interface}.",
            magazine_interface=magazine_interface, machine_interface=machine_interface,
        ))

    if not tools:
        issues.append(_issue(
            "warning", "tooling_no_tools",
            "Tooling context is attached but contains no tool assemblies.",
        ))
        return issues

    used_tool_keys: set[str] = set()
    for i, op in enumerate(process_plan_dict.get("operations", [])):
        if not _is_cutting_operation(op):
            continue
        record = _find_tool_record_for_op(op, tools)
        if record is None:
            issues.append(_issue(
                "error", "tooling_tool_missing",
                f"{_op_name(op) or 'operation'} at position {i} uses a tool not "
                "available in the tooling context.",
                i, op, tool_id=_tool_id(op) or None, tool_type=op.get("tool_type"),
            ))
            continue

        used_tool_keys.add(_tool_record_id(record) or _tool_record_type(record))
        record_interface = str(record.get("interface") or record.get("tool_interface") or "")
        if magazine_interface and record_interface and magazine_interface.upper() != record_interface.upper():
            issues.append(_issue(
                "error", "tooling_interface_mismatch",
                f"Tool '{_tool_record_id(record) or _tool_record_type(record)}' uses "
                f"{record_interface}, but the magazine declares {magazine_interface}.",
                i, op, tool_interface=record_interface, magazine_interface=magazine_interface,
            ))

        coolant = _operation_coolant(op)
        if coolant in {"through_spindle", "tsc", "through_tool"} and not bool(record.get("coolant_through")):
            issues.append(_issue(
                "error", "tooling_through_coolant_unsupported",
                f"{_op_name(op) or 'operation'} requests through-tool coolant, but "
                f"tool '{_tool_record_id(record) or _tool_record_type(record)}' "
                "does not declare coolant-through support.",
                i, op, coolant=coolant, tool_id=_tool_record_id(record) or None,
            ))

        rpm = _operation_spindle_rpm(op)
        max_rpm = _as_float(record.get("max_rpm"))
        if rpm is not None and max_rpm is not None and rpm > max_rpm + 1e-6:
            issues.append(_issue(
                "error", "tooling_tool_rpm_exceeded",
                f"{_op_name(op) or 'operation'} requests {rpm:g} RPM, above "
                f"tool '{_tool_record_id(record) or _tool_record_type(record)}' "
                f"limit {max_rpm:g} RPM.",
                i, op, requested_rpm=rpm, max_rpm=max_rpm,
            ))

        feed = _operation_feed_mm_min(op)
        max_feed = _as_float(record.get("max_feed_mm_per_min", record.get("max_feed_rate_mm_min")))
        if feed is not None and max_feed is not None and feed > max_feed + 1e-6:
            issues.append(_issue(
                "error", "tooling_tool_feed_exceeded",
                f"{_op_name(op) or 'operation'} requests feed {feed:g} mm/min, above "
                f"tool '{_tool_record_id(record) or _tool_record_type(record)}' "
                f"limit {max_feed:g} mm/min.",
                i, op, requested_feed_mm_min=feed, max_feed_mm_min=max_feed,
            ))

    if capacity is not None and len(used_tool_keys) > capacity:
        issues.append(_issue(
            "error", "tooling_used_tool_capacity_exceeded",
            f"Process plan uses {len(used_tool_keys)} tools, above magazine capacity {capacity:g}.",
            used_tool_count=len(used_tool_keys), magazine_capacity=capacity,
        ))

    return issues


def _quality_context(process_plan_dict: dict):
    quality = process_plan_dict.get("quality")
    if not quality:
        return None
    try:
        from .quality_context import load_quality_context

        return load_quality_context(quality)
    except Exception:
        return None


def _machine_supports_probe(machine: dict) -> bool:
    capabilities = machine.get("capabilities") if isinstance(machine, dict) else {}
    if not isinstance(capabilities, dict):
        capabilities = {}
    return bool(capabilities.get("probing"))


def _controller_supports_probe(controller: dict) -> bool:
    return bool(controller) and _controller_bool(controller, "probing")


def _has_inspection_operation(operations: list[dict]) -> bool:
    return any(_op_name(op) in {"inspect", "inspection", "inspect_probe", "probe", "probe_datum"} for op in operations)


def _check_quality_context(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    quality_raw = process_plan_dict.get("quality")
    if not quality_raw:
        return issues
    if not isinstance(quality_raw, dict):
        return [_issue(
            "error", "quality_context_invalid",
            "Process plan quality context must be a dictionary.",
        )]

    quality = _quality_context(process_plan_dict)
    if quality is None:
        return [_issue(
            "error", "quality_context_invalid",
            "Process plan quality context could not be normalized.",
        )]

    operations = process_plan_dict.get("operations", [])
    inspection = quality_raw.get("inspection") if isinstance(quality_raw.get("inspection"), dict) else {}
    if inspection.get("required") and not quality.required_inspection_methods():
        issues.append(_issue(
            "warning", "quality_inspection_methods_missing",
            "Quality inspection is required but no inspection methods are declared.",
        ))
    if inspection.get("required") and not _has_inspection_operation(operations):
        issues.append(_issue(
            "warning", "quality_inspection_operation_missing",
            "Quality inspection is required but the process plan has no inspection/probe operation.",
        ))

    if quality.needs_probe():
        machine = process_plan_dict.get("machine") if isinstance(process_plan_dict.get("machine"), dict) else {}
        controller = (
            process_plan_dict.get("controller")
            if isinstance(process_plan_dict.get("controller"), dict)
            else {}
        )
        if machine or controller:
            if not _machine_supports_probe(machine) and not _controller_supports_probe(controller):
                issues.append(_issue(
                    "error", "quality_probe_unsupported",
                    "Quality requirements need probing/CMM-style measurement, but neither "
                    "the machine nor controller declares probing support.",
                ))
        else:
            issues.append(_issue(
                "warning", "quality_probe_context_missing",
                "Quality requirements need probing/CMM-style measurement, but no "
                "machine/controller context is attached.",
            ))

    tolerances = quality.tolerance_bands()
    default_band = tolerances.get("default") if isinstance(tolerances.get("default"), dict) else {}
    requested_tol = _as_float(default_band.get("linear_mm", default_band.get("tolerance_mm")))
    process_capability = quality_raw.get("process_capability") if isinstance(quality_raw.get("process_capability"), dict) else {}
    minimum_capability = _as_float(
        process_capability.get("minimum_tolerance_mm",
                               process_capability.get("capability_mm"))
    )
    if requested_tol is not None and minimum_capability is not None and requested_tol < minimum_capability:
        issues.append(_issue(
            "warning", "quality_tolerance_below_process_capability",
            f"Default tolerance ({requested_tol:g}mm) is tighter than declared "
            f"process capability ({minimum_capability:g}mm).",
            tolerance_mm=requested_tol, process_capability_mm=minimum_capability,
        ))

    return issues


def _normalized_collision_policy(process_plan_dict: dict) -> dict[str, Any]:
    policy = process_plan_dict.get("collision_policy")
    quality = process_plan_dict.get("quality") if isinstance(process_plan_dict.get("quality"), dict) else {}
    if not policy and isinstance(quality.get("collision_policy"), dict):
        policy = quality.get("collision_policy")
    if not isinstance(policy, dict):
        return {}
    try:
        from .quality_context import load_collision_policy

        return load_collision_policy(policy)
    except Exception:
        result = dict(policy)
        result.setdefault("enabled", bool(result.get("required")))
        result.setdefault("required", bool(result.get("enabled")))
        result.setdefault("toolpath_required", bool(result.get("required")))
        result.setdefault("holder_check", True)
        result.setdefault("fixture_check", True)
        result.setdefault("severity", "error" if result.get("required") else "warning")
        return result


def _collision_policy_severity(policy: dict) -> str:
    severity = str(policy.get("severity") or ("error" if policy.get("required") else "warning")).lower()
    return severity if severity in {"warning", "error"} else "warning"


def _check_collision_policy(process_plan_dict: dict) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    policy = _normalized_collision_policy(process_plan_dict)
    if not policy or not policy.get("enabled"):
        return issues

    severity = _collision_policy_severity(policy)
    operations = process_plan_dict.get("operations", [])
    fixture = process_plan_dict.get("fixture")
    setups = process_plan_dict.get("setups") or {}

    if policy.get("fixture_check") and not fixture and not setups:
        issues.append(_issue(
            "warning", "collision_fixture_context_missing",
            "Collision policy asks for fixture checks, but no fixture/setup context is attached.",
        ))

    for i, op in enumerate(operations):
        if not _is_cutting_operation(op):
            continue
        op_requires = (
            policy.get("required")
            or policy.get("toolpath_required")
            or op.get("requires_collision_check")
            or op.get("collision_check_required")
        )
        if op_requires and "toolpath" not in op:
            issues.append(_issue(
                severity, "collision_toolpath_missing",
                f"{_op_name(op) or 'operation'} at position {i} requires collision "
                "validation but has no authored toolpath.",
                i, op,
            ))

        if policy.get("holder_check"):
            envelopes = _tool_envelopes(op, include_cutter=False)
            has_holder = any(envelope.get("component") == "holder" for envelope in envelopes)
            if op_requires and not has_holder:
                issues.append(_issue(
                    severity, "collision_holder_metadata_missing",
                    f"{_op_name(op) or 'operation'} at position {i} requires holder "
                    "collision validation but has no holder envelope metadata.",
                    i, op,
                ))

    return issues


def validate_all(process_plan_dict: dict, stock_dims: dict,
                 tool_specs: Optional[list] = None,
                 structured: bool = False):
    """Run all validation checks against a complete process plan.

    By default this returns the legacy ``list[str]`` warning output. Pass
    ``structured=True`` to receive a ``ValidationReport`` with issue metadata.
    """
    report = ValidationReport()
    ops = process_plan_dict.get("operations", [])
    merged_stock = dict(stock_dims or {})
    merged_stock.update(process_plan_dict.get("stock_dimensions") or {})

    report.extend(_check_plan_metadata(process_plan_dict))
    report.extend(_check_machine_context(process_plan_dict, merged_stock))
    report.extend(_check_controller_context(process_plan_dict))
    report.extend(_check_setup_intent_context(process_plan_dict))
    report.extend(_check_tooling_context(process_plan_dict))
    report.extend(_check_quality_context(process_plan_dict))
    report.extend(_check_collision_policy(process_plan_dict))
    report.extend(_check_operation_order(ops))
    report.extend(_check_tool_selection(ops))
    report.extend(_check_tool_reach_and_geometry(ops, merged_stock, tool_specs))
    report.extend(_check_toolpaths(ops, merged_stock))
    report.extend(_check_setup_accessibility(process_plan_dict))
    report.extend(_check_fixture_clearance(process_plan_dict))
    report.extend(_check_requested_wall_thickness(process_plan_dict))

    if structured:
        return report
    return report.messages


# =============================================================================
#  Self-test
# =============================================================================

if __name__ == "__main__":
    print("=== SubCAD Validation Self-Test ===\n")
    passed = 0
    failed = 0

    def _check(label: str, condition: bool, detail: str = ""):
        global passed, failed
        if condition:
            passed += 1
            print(f"  PASS  {label}")
        else:
            failed += 1
            print(f"  FAIL  {label}  -- {detail}")

    # ------------------------------------------------------------------
    # 1. check_wall_thickness (placeholder)
    # ------------------------------------------------------------------
    print("1. check_wall_thickness ...")
    result = check_wall_thickness(None, None)
    _check("returns list", isinstance(result, list))
    _check("empty for now", len(result) == 0)
    print()

    # ------------------------------------------------------------------
    # 2. check_tool_reach
    # ------------------------------------------------------------------
    print("2. check_tool_reach ...")
    from dataclasses import dataclass

    @dataclass
    class FakeTool:
        flute_length: float = 50.0

    @dataclass
    class FakeOp:
        depth: float = 0.0

    tool = FakeTool(flute_length=30.0)
    shallow_op = FakeOp(depth=10.0)
    w = check_tool_reach(shallow_op, {}, tool)
    _check("shallow op OK", len(w) == 0, f"got {w}")

    deep_op = FakeOp(depth=40.0)
    w = check_tool_reach(deep_op, {}, tool)
    _check("deep op warns", len(w) == 1, f"got {len(w)}")
    if w:
        _check("mentions flute length", "flute" in w[0].lower())
    print()

    # ------------------------------------------------------------------
    # 3. check_operation_order
    # ------------------------------------------------------------------
    print("3. check_operation_order ...")

    # Correct order
    ops_ok = [
        {"operation": "face_mill"},
        {"operation": "drill"},
        {"operation": "tap"},
        {"operation": "chamfer"},
    ]
    w = check_operation_order(ops_ok)
    _check("correct order OK", len(w) == 0, f"got {w}")

    # Face mill not first
    ops_bad1 = [
        {"operation": "drill"},
        {"operation": "face_mill"},
    ]
    w = check_operation_order(ops_bad1)
    _check("face_mill not first warns", len(w) == 1, f"got {len(w)}")

    # Tap without drill
    ops_bad2 = [
        {"operation": "tap"},
    ]
    w = check_operation_order(ops_bad2)
    _check("tap without drill warns", len(w) == 1, f"got {len(w)}")
    print()

    # ------------------------------------------------------------------
    # 4. validate_all
    # ------------------------------------------------------------------
    print("4. validate_all ...")
    plan = {
        "schema_version": "subcad.shop_floor.v1",
        "material": "aluminum_6061",
        "operations": [
            {"operation": "face_mill"},
            {"operation": "drill"},
        ],
    }
    w = validate_all(plan, {})
    _check("returns list", isinstance(w, list))
    _check("no warnings for valid plan", len(w) == 0, f"got {w}")

    plan_bad = {
        "schema_version": "subcad.shop_floor.v1",
        "material": "aluminum_6061",
        "operations": [
            {"operation": "drill"},
            {"operation": "face_mill"},
        ],
    }
    w = validate_all(plan_bad, {})
    _check("warns for bad order", len(w) == 1, f"got {len(w)}")
    print()

    # ------------------------------------------------------------------
    # 5. SubCAD Shop-Floor v1 structured readiness checks
    # ------------------------------------------------------------------
    print("5. shop-floor readiness checks ...")
    plan_shop = {
        "operations": [
            {
                "operation": "slot",
                "tool_type": "flat_endmill",
                "tool_diameter_mm": 8.0,
                "slot_width_mm": 6.0,
                "depth_mm": 20.0,
                "position": [0, 0],
            },
            {
                "operation": "tap",
                "tool_type": "thread_mill",
                "tool_diameter_mm": 6.0,
                "thread_diameter_mm": 6.0,
                "depth_mm": 12.0,
                "position": [10, 10],
            },
            {
                "operation": "rough_pocket",
                "tool_type": "flat_endmill",
                "tool_diameter_mm": 10.0,
                "depth_mm": 3.0,
                "dimensions": {"width": 20.0, "length": 30.0, "depth": 3.0},
                "corner_radius_mm": 2.0,
                "position": [15, 15],
            },
            {
                "operation": "circular_pocket",
                "tool_type": "flat_endmill",
                "tool_diameter_mm": 12.0,
                "pocket_diameter_mm": 10.0,
                "depth_mm": 2.0,
                "position": [20, 20],
            },
        ],
    }
    report = validate_all(
        plan_shop,
        {},
        tool_specs=[{"tool_type": "flat_endmill", "diameter": 8.0, "flute_length_mm": 10.0}],
        structured=True,
    )
    codes = {issue.code for issue in report.issues}
    _check("structured report returned", isinstance(report, ValidationReport))
    _check("list-compatible iteration", list(report) == report.messages)
    _check("tool flute warning", "tool_flute_length" in codes, f"got {codes}")
    _check("slot width warning", "tool_vs_slot_width" in codes, f"got {codes}")
    _check("tap order warning", "tap_before_drill" in codes, f"got {codes}")
    _check("corner radius warning", "tool_vs_internal_corner_radius" in codes, f"got {codes}")
    _check("circular pocket warning", "tool_vs_pocket_diameter" in codes, f"got {codes}")

    plan_fixture = {
        "fixture": {
            "name": "test_fixture",
            "clamping_zones": [
                {"label": "left_clamp", "x_min": -5, "x_max": 5,
                 "y_min": -5, "y_max": 5, "z_min": 0, "z_max": 10,
                 "margin_mm": 0},
            ],
        },
        "setups": {
            "op1": {"name": "op1", "face_selector": ">Z", "work_offset": "G54"},
        },
        "operations": [
            {"operation": "drill", "position": [0, 0], "setup": "op1", "face_selector": ">Z"},
            {"operation": "drill", "position": [12, 0], "face_selector": ">Z"},
            {"operation": "drill", "position": [20, 0], "setup": "missing", "face_selector": ">Z"},
        ],
        "check_wall_thickness": True,
        "min_wall_mm": 1.5,
    }
    report = validate_all(plan_fixture, {}, structured=True)
    codes = {issue.code for issue in report.issues}
    _check("fixture clearance warning", "fixture_clearance" in codes, f"got {codes}")
    _check("missing setup assignment warning", "operation_missing_setup" in codes, f"got {codes}")
    _check("unknown setup warning", "operation_unknown_setup" in codes, f"got {codes}")
    _check("wall thickness placeholder warning", "wall_thickness_placeholder" in codes, f"got {codes}")

    plan_toolpaths = {
        "schema_version": "subcad.shop_floor.v1",
        "material": "aluminum_6061",
        "stock_dimensions": {"length": 50.0, "width": 40.0, "height": 10.0},
        "operations": [
            {
                "operation": "rough_pocket",
                "depth_mm": 5.0,
                "toolpath": [],
            },
            {
                "operation": "drill",
                "depth_mm": 5.0,
                "toolpath": {
                    "safe_z": 5.0,
                    "moves": [
                        {"type": "rapid", "position": [0, 0, 5.0]},
                        {"type": "plunge", "position": [60, 0, -2.0]},
                    ],
                },
            },
        ],
    }
    report = validate_all(plan_toolpaths, {}, structured=True)
    codes = {issue.code for issue in report.issues}
    legacy_messages = validate_all(plan_toolpaths, {})
    _check("empty toolpath warning", "empty_toolpath" in codes, f"got {codes}")
    _check("missing retract/safe-Z warning", "missing_retract_safe_z" in codes, f"got {codes}")
    _check("toolpath depth warning", "toolpath_above_requested_depth" in codes, f"got {codes}")
    _check("stock bounds warning", "toolpath_stock_bounds" in codes, f"got {codes}")
    _check("legacy validate_all remains list output", isinstance(legacy_messages, list))
    print()

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    total = passed + failed
    print(f"=== {'ALL TESTS PASSED' if failed == 0 else 'SOME TESTS FAILED'} "
          f"({passed}/{total} passed) ===")
    if failed > 0:
        import sys
        sys.exit(1)
