"""Browser visualization package export for SubCAD."""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path
from typing import Any


PACKAGE_SCHEMA_VERSION = "subcad.visualization_package.v1"


def export_visualization_package(
    stock,
    output_dir: str | Path,
    *,
    target: Any = None,
    tolerance_mm: float = 0.25,
    title: str = "SubCAD Visualization Session",
    include_diff_mesh: bool = True,
    economics: Any = None,
) -> dict:
    """Export a static directory of browser-friendly meshes and JSON."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    stock_path = out / "stock.stl"
    stock.to_stl(str(stock_path))

    plan = stock.process_plan()
    economics_report = _resolve_economics(stock, economics)
    stock_states = _export_stock_states(stock, out, plan)
    toolpath = _toolpath_payload(plan, economics_report=economics_report)
    toolpath_path = out / "toolpath.json"
    _write_json(toolpath_path, toolpath)
    validation_issues = _validation_issue_payloads(stock, plan)
    clearance_markers = _clearance_markers(
        validation_issues,
        plan,
        z_offset=_stock_z_offset(plan),
    )

    comparison = None
    target_path = None
    diff_path = None
    if target is not None:
        target_path = out / "target.stl"
        _export_target_mesh(target, target_path)

        comparison = stock.compare_to_target(target, tolerance_mm=tolerance_mm)
        comparison_path = out / "comparison.json"
        _write_json(comparison_path, comparison)

        if include_diff_mesh:
            diff_path = _try_export_diff_mesh(
                stock,
                target,
                out / "diff_mesh.ply",
                tolerance_mm=tolerance_mm,
            )

    assets = {
        "stock_mesh": {
            "path": stock_path.name,
            "format": "stl",
            "role": "stock",
            "color": "#8ca0b4",
        },
        "toolpath": {
            "path": toolpath_path.name,
            "format": "json",
            "operation_count": len(toolpath["operations"]),
            "move_count": toolpath["summary"]["move_count"],
        },
        "stock_states": stock_states,
    }
    fixtures = _fixture_payloads(stock, plan)
    if fixtures:
        assets["fixtures"] = {
            "format": "procedural",
            "fixture_count": len(fixtures),
            "roles": ["fixture_body", "clamping_zones", "setup_axes"],
        }
    if target_path is not None:
        assets["target_mesh"] = {
            "path": target_path.name,
            "format": "stl",
            "role": "target",
            "color": "#3cb44b",
            "opacity": 0.35,
        }
    if comparison is not None:
        assets["comparison"] = {
            "path": "comparison.json",
            "format": "json",
            "status": comparison.get("status"),
            "score": comparison.get("score"),
        }
    if diff_path is not None:
        assets["diff_mesh"] = {
            "path": diff_path.name,
            "format": diff_path.suffix.lstrip("."),
            "role": "deviation",
            "colors": {
                "overcut": "#d7191c",
                "within_tolerance": "#2ca25f",
                "undercut": "#2c7bb6",
            },
        }

    bbox = getattr(stock, "bounding_box", {})
    scene = {
        "title": title,
        "schema_version": PACKAGE_SCHEMA_VERSION,
        "units": "mm",
        "material": getattr(stock, "material", plan.get("material")),
        "stock_dimensions": plan.get("stock_dimensions", {}),
        "bounding_box": _round_mapping(bbox),
        "volume_mm3": _round_float(getattr(stock, "volume", None)),
        "operations": [
            {
                "sequence_number": op.get("sequence_number"),
                "operation": op.get("operation"),
                "setup": op.get("setup"),
                "face_selector": op.get("face_selector"),
                "tool": op.get("tool"),
                "tool_assembly": (op.get("tool") or {}).get("assembly"),
                "toolpath_summary": op.get("toolpath_summary"),
                "tool_selection": op.get("tool_selection"),
                "pass_plan": op.get("pass_plan"),
                "economics": _operation_economics(op.get("sequence_number"), economics_report),
                "validation": _operation_validation(op, validation_issues),
                "warnings": _operation_warnings(op, validation_issues),
            }
            for op in plan.get("operations", [])
        ],
        "clearance_markers": clearance_markers,
        "validation_issues": validation_issues,
        "economics": economics_report,
        "machine": plan.get("machine"),
        "fixtures": fixtures,
        "setups": plan.get("setups", {}),
        "assets": assets,
        "compare_status": comparison.get("status") if comparison else None,
        "view": _default_view(bbox),
        "viewer": {
            "entrypoint": "web/visualization.html",
            "recommended_url": "visualization.html?session=sessions/latest/scene.json",
        },
    }
    scene_path = out / "scene.json"
    _write_json(scene_path, scene)
    return scene


def _export_stock_states(stock: Any, out: Path, plan: dict) -> list[dict]:
    import hashlib
    from .geometry import export_stl

    operations_by_sequence = {
        op.get("sequence_number"): op
        for op in plan.get("operations", [])
    }
    history = list(getattr(stock, "_state_history", []) or [])

    if not history:
        history = [{
            "sequence_number": plan.get("total_operations", 0),
            "operation": "final_stock",
            "label": "Final stock",
            "shape": getattr(stock, "_shape", None),
        }]

    states_dir = out / "states"
    states_dir.mkdir(parents=True, exist_ok=True)
    states = []

    for index, state in enumerate(history):
        shape = state.get("shape")
        if shape is None:
            continue

        sequence_number = state.get("sequence_number", index)
        pass_index = state.get("pass_index")
        suffix = f"_pass_{int(pass_index):04d}" if pass_index is not None else ""
        filename = f"stock_state_{int(sequence_number):03d}{suffix}.stl"
        path = states_dir / filename
        export_stl(shape, str(path))

        op = operations_by_sequence.get(sequence_number, {})
        label = state.get("label") or (
            "Initial stock" if sequence_number == 0 else f"After OP {sequence_number}"
        )
        states.append({
            "index": len(states),
            "sequence_number": sequence_number,
            "pass_index": pass_index,
            "pass_role": state.get("pass_role"),
            "operation": op.get("operation") or state.get("operation"),
            "label": label,
            "path": f"states/{filename}",
            "format": "stl",
            "role": "stock_state",
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "volume_mm3": state["shape"].val().Volume(),
            "geometry_fidelity": op.get("geometry_fidelity", "unknown"),
            "verification_status": op.get("verification_status", "not_run"),
            "coordinate_frame": {"basis": "stock_world", "units": "mm", "spindle_axis": op.get("axis"),
                "spindle_origin_mm": op.get("spindle_origin_mm", [0, 0, 0])},
        })

    return states


def _toolpath_payload(plan: dict, economics_report: dict | None = None) -> dict:
    operations = []
    move_count = 0
    total_length = 0.0
    total_time = 0.0

    for op in plan.get("operations", []):
        moves = [_compact_move(move) for move in _toolpath_records(op.get("toolpath"))]
        moves = [move for move in moves if move is not None]
        move_count += len(moves)

        summary = dict(op.get("toolpath_summary") or {})
        length = _as_float(summary.get("length_mm"))
        if length is None:
            length = _polyline_length(moves)
            if length is not None:
                summary["length_mm"] = round(length, 6)
        if length is not None:
            total_length += length

        time_min = _as_float(
            summary.get("estimated_time_min")
            or summary.get("estimated_time_minutes")
        )
        if time_min is not None:
            total_time += time_min

        operations.append({
            "sequence_number": op.get("sequence_number"),
            "operation": op.get("operation"),
            "setup": op.get("setup"),
            "face_selector": op.get("face_selector"),
            "tool": op.get("tool"),
            "tool_assembly": (op.get("tool") or {}).get("assembly"),
            "tool_type": op.get("tool_type"),
            "tool_diameter_mm": op.get("tool_diameter_mm"),
            "tool_selection": op.get("tool_selection"),
            "pass_plan": op.get("pass_plan"),
            "economics": _operation_economics(op.get("sequence_number"), economics_report),
            "validation": op.get("validation_buckets"),
            "warnings": op.get("validation_buckets", {}).get("warnings", []),
            "summary": summary,
            "moves": moves,
        })

    return {
        "schema_version": PACKAGE_SCHEMA_VERSION,
        "units": "mm",
        "summary": {
            "operation_count": len(operations),
            "move_count": move_count,
            "length_mm": round(total_length, 6),
            "estimated_time_minutes": round(total_time, 6),
        },
        "operations": operations,
    }


def _fixture_payloads(stock: Any, plan: dict) -> list[dict]:
    fixture = getattr(stock, "_fixture", None)
    if fixture is None and isinstance(plan.get("fixture"), dict):
        return [{
            "name": plan["fixture"].get("name"),
            "fixture_type": plan["fixture"].get("fixture_type"),
            "description": plan["fixture"].get("description", ""),
            "clamping_zones": plan["fixture"].get("clamping_zones", []),
            "clearance_zones": plan["fixture"].get("clearance_zones", []),
            "body": {
                "type": plan["fixture"].get("fixture_type"),
                "base": {
                    "center": [0.0, 0.0, -float(plan["fixture"].get("base_height_mm") or 0.0) / 2.0],
                    "size": [
                        float((plan["fixture"].get("dimensions") or {}).get("length_mm", 100.0)),
                        float((plan["fixture"].get("dimensions") or {}).get("width_mm", 80.0)),
                        float((plan["fixture"].get("dimensions") or {}).get("height_mm", plan["fixture"].get("base_height_mm") or 10.0)),
                    ],
                },
            },
        }]
    if fixture is None:
        return []
    try:
        from .fixturing import fixture_visualization_payload

        payload = fixture_visualization_payload(fixture, plan.get("stock_dimensions", {}))
    except Exception:
        payload = fixture.to_dict() if hasattr(fixture, "to_dict") else {}
    payload["setups"] = plan.get("setups", {})
    return [payload]


def _resolve_economics(stock: Any, economics: Any) -> dict | None:
    if economics is None or economics is False:
        return None
    if isinstance(economics, dict):
        return economics
    if economics is True:
        try:
            return stock.estimate_cost()
        except Exception:
            return None
    return None


def _operation_economics(sequence_number: Any, economics_report: dict | None) -> dict | None:
    if not isinstance(economics_report, dict):
        return None
    for item in economics_report.get("operations", []):
        if item.get("sequence_number") == sequence_number:
            return item
    return None


def _validation_issue_payloads(stock: Any, plan: dict) -> list[dict]:
    try:
        report = stock.validate_shop_floor(structured=True)
        issues = getattr(report, "issues", []) or []
    except Exception:
        return []

    operations = plan.get("operations", [])
    payloads = []
    for issue in issues:
        op_index = getattr(issue, "operation_index", None)
        op = operations[op_index] if isinstance(op_index, int) and 0 <= op_index < len(operations) else {}
        payloads.append({
            "severity": getattr(issue, "severity", "warning"),
            "code": getattr(issue, "code", "validation"),
            "message": getattr(issue, "message", str(issue)),
            "operation_index": op_index,
            "sequence_number": op.get("sequence_number"),
            "operation": getattr(issue, "operation", "") or op.get("operation"),
            "context": getattr(issue, "context", {}),
        })
    return payloads


def _operation_validation(op: dict, issues: list[dict]) -> dict:
    selected = op.get("sequence_number")
    related = [issue for issue in issues if issue.get("sequence_number") == selected]
    return {
        "errors": [issue for issue in related if issue.get("severity") == "error"],
        "warnings": [issue for issue in related if issue.get("severity") != "error"],
    }


def _operation_warnings(op: dict, issues: list[dict]) -> list:
    selected = op.get("sequence_number")
    warnings = list((op.get("validation_buckets") or {}).get("warnings", []))
    warnings.extend(issue for issue in issues if issue.get("sequence_number") == selected)
    return warnings


def _clearance_markers(issues: list[dict], plan: dict, *, z_offset: float) -> list[dict]:
    markers = []
    for issue in issues:
        if not str(issue.get("code", "")).startswith("fixture_toolpath_clearance"):
            continue
        context = issue.get("context") or {}
        point = context.get("point")
        if not isinstance(point, list) or len(point) < 3:
            continue
        markers.append({
            "marker_type": "collision" if issue.get("severity") == "error" else "clearance",
            "severity": issue.get("severity", "warning"),
            "position": [
                _round_float(point[0]),
                _round_float(point[1]),
                _round_float(point[2] + z_offset),
            ],
            "sequence_number": issue.get("sequence_number"),
            "operation": issue.get("operation"),
            "message": issue.get("message"),
            "code": issue.get("code"),
            "fixture_id": context.get("fixture_id"),
            "tool_id": context.get("tool_id"),
            "zone": context.get("zone"),
            "component": context.get("component"),
            "clearance_mm": context.get("clearance_mm"),
            "radius_mm": max(abs(float(context.get("clearance_mm", 0.0) or 0.0)), 1.2),
        })
    return markers


def _stock_z_offset(plan: dict) -> float:
    stock = plan.get("stock_dimensions") or {}
    return float(stock.get("height") or stock.get("z") or 0.0) / 2.0


def _export_target_mesh(target: Any, path: Path) -> None:
    if hasattr(target, "to_stl"):
        target.to_stl(str(path))
        return

    if isinstance(target, (str, Path)):
        source = Path(target)
        suffix = source.suffix.lower()
        if suffix == ".stl":
            shutil.copyfile(source, path)
            return
        if suffix in {".step", ".stp"}:
            from .geometry import export_stl, import_step

            export_stl(import_step(str(source)), str(path))
            return

    mesh = _as_mesh(target)
    if mesh is None:
        raise ValueError("Target geometry could not be converted to an STL mesh")
    mesh.export(str(path))


def _try_export_diff_mesh(
    stock: Any,
    target: Any,
    path: Path,
    *,
    tolerance_mm: float,
) -> Path | None:
    try:
        import numpy as np

        try:
            from src.simulation.mesh_compare import compute_signed_deviation
        except ImportError:
            from simulation.mesh_compare import compute_signed_deviation
    except Exception:
        return None

    target_mesh = _as_mesh(target)
    stock_mesh = _as_mesh(stock)
    if target_mesh is None or stock_mesh is None:
        return None

    try:
        deviation = compute_signed_deviation(
            target_mesh,
            stock_mesh,
            sample_distance=max(float(tolerance_mm), 0.01),
        ).get("per_vertex_deviation")
        if deviation is None:
            return None

        colored = target_mesh.copy()
        colors = np.zeros((len(colored.vertices), 4), dtype=np.uint8)
        tol = max(float(tolerance_mm), 1e-9)
        for i, value in enumerate(np.asarray(deviation)[: len(colors)]):
            if value > tol:
                colors[i] = [215, 25, 28, 255]
            elif value < -tol:
                colors[i] = [44, 123, 182, 255]
            else:
                colors[i] = [44, 162, 95, 255]
        colored.visual.vertex_colors = colors
        colored.export(str(path))
        return path if path.exists() else None
    except Exception:
        return None


def _as_mesh(source: Any):
    from .target_compare import _as_mesh as target_as_mesh

    return target_as_mesh(source)


def _toolpath_records(toolpath: Any) -> list:
    if isinstance(toolpath, dict):
        moves = toolpath.get("moves")
        if isinstance(moves, list):
            return moves
        positions = toolpath.get("positions")
        if isinstance(positions, list):
            return positions
        if "position" in toolpath:
            return [toolpath]
        return []
    if isinstance(toolpath, list):
        return toolpath
    return []


def _compact_move(move: Any) -> dict | None:
    if isinstance(move, dict):
        position = _position_from(move)
        if position is None:
            return None
        compact = {
            "type": str(move.get("type") or move.get("move_type") or "feed"),
            "position": position,
        }
        for key in (
            "feed_mm_min",
            "spindle_rpm",
            "coolant",
            "dwell_seconds",
            "arc_center",
            "arc_direction",
        ):
            if key in move:
                compact[key] = _json_ready(move[key])
        return compact

    position = _position_from(move)
    if position is None:
        return None
    return {"type": "feed", "position": position}


def _position_from(value: Any) -> list[float] | None:
    if isinstance(value, dict):
        if "position" in value:
            return _position_from(value["position"])
        if all(k in value for k in ("x", "y", "z")):
            return [_round_float(value[k]) for k in ("x", "y", "z")]
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return [_round_float(v) for v in value[:3]]
    return None


def _polyline_length(moves: list[dict]) -> float | None:
    points = [move.get("position") for move in moves if move.get("position")]
    if len(points) < 2:
        return 0.0 if points else None
    return sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))


def _default_view(bbox: dict) -> dict:
    center = [
        _round_float((bbox.get("x_min", 0.0) + bbox.get("x_max", 0.0)) / 2.0),
        _round_float((bbox.get("y_min", 0.0) + bbox.get("y_max", 0.0)) / 2.0),
        _round_float((bbox.get("z_min", 0.0) + bbox.get("z_max", 0.0)) / 2.0),
    ]
    size = max(
        _as_float(bbox.get("length")) or 1.0,
        _as_float(bbox.get("width")) or 1.0,
        _as_float(bbox.get("height")) or 1.0,
    )
    return {
        "target": center,
        "camera_position": [
            _round_float(center[0] + size * 1.5),
            _round_float(center[1] - size * 1.8),
            _round_float(center[2] + size * 1.2),
        ],
        "up": [0.0, 0.0, 1.0],
    }


def _write_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as fh:
        json.dump(_json_ready(data), fh, indent=2)


def _json_ready(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _json_ready(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(v) for v in value]
    if hasattr(value, "tolist"):
        return _json_ready(value.tolist())
    if hasattr(value, "item"):
        return _json_ready(value.item())
    if isinstance(value, float):
        return _round_float(value)
    return value


def _round_mapping(data: dict) -> dict:
    return {
        str(key): _round_float(value) if isinstance(value, (int, float)) else value
        for key, value in dict(data or {}).items()
    }


def _round_float(value: Any) -> float | None:
    if value is None:
        return None
    return round(float(value), 6)


def _as_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
