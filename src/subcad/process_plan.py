"""SubCAD ProcessPlan: records every subtractive operation into a serializable plan.

A ProcessPlan is the output artifact that tells a machinist (or downstream
CAM software) exactly what operations to perform, in what order, with which
tools. It can be serialized to JSON for interchange.
"""

from __future__ import annotations

import json
import math
import tempfile
from dataclasses import dataclass, field
from typing import Optional


SCHEMA_VERSION = "subcad.shop_floor.v1"


def _jsonable(value):
    """Return *value* converted to plain JSON-compatible containers."""
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _normalize_feeds_speeds(feeds_speeds: Optional[dict]) -> Optional[dict]:
    """Normalize feed-rate aliases while preserving the original fields."""
    if feeds_speeds is None:
        return None

    feeds = dict(feeds_speeds)
    feed_rate = feeds.get("feed_rate_mm_per_min", feeds.get("feed_rate_mm_min"))
    if feed_rate is not None:
        feeds["feed_rate_mm_per_min"] = feed_rate
        feeds["feed_rate_mm_min"] = feed_rate
    return feeds


def _tool_metadata(op_dict: dict) -> dict:
    """Build structured tool metadata from the legacy flat operation fields."""
    tool_type = op_dict.get("tool_type", "")
    diameter = op_dict.get("tool_diameter_mm")
    tool: dict = {
        "tool_type": tool_type,
        "diameter_mm": diameter,
    }

    for old_key, new_key in (
        ("tool_tip_angle_deg", "tip_angle_deg"),
        ("tool_tip_diameter_mm", "tip_diameter_mm"),
        ("tool_flute_length_mm", "flute_length_mm"),
        ("tool_catalog_id", "catalog_id"),
        ("tool_number", "tool_number"),
        ("tool_holder_id", "holder_id"),
        ("tool_stickout_mm", "stickout_mm"),
        ("tool_shank_diameter_mm", "shank_diameter_mm"),
        ("tool_overall_length_mm", "overall_length_mm"),
        ("tool_reach_mm", "reach_mm"),
    ):
        if old_key in op_dict:
            tool[new_key] = op_dict[old_key]

    label_parts = [str(tool_type) or "tool"]
    if diameter not in (None, ""):
        label_parts.append(f"D{diameter}mm")
    tool["label"] = " ".join(label_parts)
    return tool


def _normalize_tool_entry(tool_dict: dict) -> dict:
    """Normalize legacy tool entries to structured shop-floor metadata."""
    tool = dict(tool_dict)
    if "diameter_mm" not in tool and "tool_diameter_mm" in tool:
        tool["diameter_mm"] = tool["tool_diameter_mm"]
    if "tool_diameter_mm" not in tool and "diameter_mm" in tool:
        tool["tool_diameter_mm"] = tool["diameter_mm"]
    if "label" not in tool:
        label_parts = [str(tool.get("tool_type", "")) or "tool"]
        if tool.get("diameter_mm") not in (None, ""):
            label_parts.append(f"D{tool.get('diameter_mm')}mm")
        tool["label"] = " ".join(label_parts)
    if "assembly" not in tool:
        tool["assembly"] = {
            "tool_id": tool.get("catalog_id", ""),
            "tool_type": tool.get("tool_type", ""),
            "cutter_diameter_mm": tool.get("diameter_mm", tool.get("tool_diameter_mm")),
            "flute_length_mm": tool.get("flute_length_mm", 50.0),
            "shank_diameter_mm": tool.get(
                "shank_diameter_mm",
                tool.get("diameter_mm", tool.get("tool_diameter_mm")),
            ),
            "stickout_mm": tool.get("stickout_mm", tool.get("flute_length_mm", 35.0)),
            "holder_id": tool.get("holder_id", "generic_er_collet"),
            "holder_diameter_mm": tool.get("holder_diameter_mm", 25.0),
            "holder_length_mm": tool.get("holder_length_mm", 35.0),
            "tool_number": tool.get("tool_number"),
        }
    return tool


def _toolpath_records(toolpath) -> Optional[list]:
    """Return serialized toolpath move/position records as a flat list."""
    if isinstance(toolpath, dict):
        if isinstance(toolpath.get("moves"), list):
            return toolpath["moves"]
        if isinstance(toolpath.get("positions"), list):
            return toolpath["positions"]
        return [toolpath]
    if isinstance(toolpath, list):
        return toolpath
    return None


def _toolpath_point_xyz(point):
    """Return an ``(x, y, z)`` tuple from supported serialized point records."""
    if isinstance(point, dict):
        if all(k in point for k in ("x", "y", "z")):
            return (point["x"], point["y"], point["z"])
        if "position" in point:
            return _toolpath_point_xyz(point["position"])
    if isinstance(point, (list, tuple)) and len(point) >= 3:
        return (point[0], point[1], point[2])
    return None


def _toolpath_length_mm(toolpath) -> Optional[float]:
    """Return authored toolpath length when available or computable."""
    if isinstance(toolpath, dict):
        for key in ("length_mm", "toolpath_length_mm"):
            length = toolpath.get(key)
            if length is not None:
                return float(length)

    records = _toolpath_records(toolpath)
    if records is None:
        return None
    xyz_points = [
        (float(p[0]), float(p[1]), float(p[2]))
        for p in (_toolpath_point_xyz(record) for record in records)
        if p is not None
    ]
    if len(xyz_points) < 2:
        return 0.0 if xyz_points else None
    return sum(math.dist(a, b) for a, b in zip(xyz_points[:-1], xyz_points[1:]))


def _toolpath_estimated_time_minutes(op: dict, feed_rate: Optional[float]) -> Optional[float]:
    """Return authored estimated time, or authored path length / feed rate."""
    toolpath = op.get("toolpath")
    if isinstance(toolpath, dict):
        for key in (
            "estimated_time_min",
            "estimated_time_minutes",
            "cycle_time_minutes",
            "time_minutes",
        ):
            value = toolpath.get(key)
            if value is not None:
                return float(value)

    summary = op.get("toolpath_summary")
    if isinstance(summary, dict):
        for key in ("estimated_time_min", "estimated_time_minutes"):
            value = summary.get(key)
            if value is not None:
                return float(value)

    if feed_rate:
        length = _toolpath_length_mm(toolpath)
        if length is not None:
            return length / float(feed_rate)
    return None


def _toolpath_summary(toolpath) -> Optional[dict]:
    """Summarize a serialized toolpath when an operation provides one."""
    records = _toolpath_records(toolpath)
    if records is None:
        return None

    summary = {"point_count": len(records)}
    if isinstance(toolpath, dict):
        for old_key, new_key in (
            ("length_mm", "length_mm"),
            ("toolpath_length_mm", "length_mm"),
            ("estimated_time_min", "estimated_time_min"),
            ("estimated_time_minutes", "estimated_time_minutes"),
        ):
            if old_key in toolpath:
                summary[new_key] = toolpath[old_key]
    if not records:
        return summary

    xyz_points = [p for p in (_toolpath_point_xyz(p) for p in records) if p is not None]
    if xyz_points:
        xs, ys, zs = zip(*xyz_points)
        summary["start"] = list(xyz_points[0])
        summary["end"] = list(xyz_points[-1])
        summary["bounds"] = {
            "min": [min(xs), min(ys), min(zs)],
            "max": [max(xs), max(ys), max(zs)],
        }
        if "length_mm" not in summary:
            summary["length_mm"] = round(_toolpath_length_mm(toolpath) or 0.0, 6)
    return summary


def _empty_validation_buckets() -> dict:
    return {"errors": [], "warnings": [], "notes": []}


def _append_bucket(buckets: dict, bucket_name: str, item) -> None:
    if item is None or item == "":
        return
    if isinstance(item, list):
        buckets.setdefault(bucket_name, []).extend(str(v) for v in item if v)
    else:
        buckets.setdefault(bucket_name, []).append(str(item))


def _merge_validation_buckets(buckets: dict, validation) -> None:
    if isinstance(validation, dict):
        for key in ("errors", "warnings", "notes"):
            _append_bucket(buckets, key, validation.get(key))
    elif isinstance(validation, list):
        _append_bucket(buckets, "warnings", validation)


def _operation_economics(sequence_number: int, economics) -> Optional[dict]:
    if not isinstance(economics, dict):
        return None
    for item in economics.get("operations", []):
        if item.get("sequence_number") == sequence_number:
            return item
    return None


def normalize_operation(op_dict: dict, sequence_number: int) -> dict:
    """Return an operation normalized to schema v1 with legacy fields intact."""
    op = _jsonable(dict(op_dict))
    op["sequence_number"] = int(op.get("sequence_number") or sequence_number)
    op["schema_version"] = SCHEMA_VERSION
    op["feeds_speeds"] = _normalize_feeds_speeds(op.get("feeds_speeds"))
    base_tool = _tool_metadata(op)
    existing_tool = op.get("tool") if isinstance(op.get("tool"), dict) else {}
    op["tool"] = _normalize_tool_entry({**base_tool, **existing_tool})

    if "toolpath_summary" not in op and "toolpath" in op:
        summary = _toolpath_summary(op.get("toolpath"))
        if summary is not None:
            op["toolpath_summary"] = summary
    if "pass_plan" not in op and isinstance(op.get("toolpath"), dict):
        metadata = op["toolpath"].get("metadata")
        if isinstance(metadata, dict) and isinstance(metadata.get("pass_plan"), dict):
            op["pass_plan"] = metadata["pass_plan"]
    if "tool_selection" not in op and isinstance(op.get("toolpath"), dict):
        metadata = op["toolpath"].get("metadata")
        if isinstance(metadata, dict) and isinstance(metadata.get("tool_selection"), dict):
            op["tool_selection"] = metadata["tool_selection"]

    buckets = _empty_validation_buckets()
    _merge_validation_buckets(buckets, op.get("validation"))
    _merge_validation_buckets(buckets, op.get("validation_buckets"))
    _merge_validation_buckets(buckets, op.get("tool_selection"))
    _append_bucket(buckets, "notes", op.get("notes"))
    op["validation_buckets"] = buckets
    return op


@dataclass
class OperationStep:
    """A single step in a subtractive machining process.

    Simplified copy from the rules.py pattern. Each step describes one
    tool engagement (face mill, pocket, drill, etc.) at a specific location
    with feeds/speeds data when available.
    """

    sequence_number: int
    operation: str  # "face_mill", "rough_pocket", "drill", etc.
    tool_type: str
    tool_diameter_mm: float
    depth_mm: float = 0.0
    position: Optional[tuple] = None  # (x, y) on workplane
    feeds_speeds: Optional[dict] = None
    notes: str = ""

    def to_dict(self) -> dict:
        """Return a JSON-serializable dictionary representation."""
        result: dict = {
            "sequence_number": self.sequence_number,
            "operation": self.operation,
            "tool_type": self.tool_type,
            "tool_diameter_mm": self.tool_diameter_mm,
            "depth_mm": self.depth_mm,
            "notes": self.notes,
        }
        if self.position is not None:
            result["position"] = list(self.position)
        if self.feeds_speeds is not None:
            result["feeds_speeds"] = self.feeds_speeds
        return result


@dataclass
class ProcessPlan:
    """Records a complete subtractive manufacturing process plan.

    Every operation in the SubCAD pipeline appends itself to this plan.
    The plan can be serialized to JSON for hand-off to a machinist or
    downstream CAM software.
    """

    material: str = ""
    stock_dimensions: dict = field(default_factory=dict)  # {"length": 100, "width": 50, "height": 20}
    operations: list[dict] = field(default_factory=list)
    tools_used: list[dict] = field(default_factory=list)
    schema_version: str = SCHEMA_VERSION

    # Fixturing / setup metadata
    fixture: Optional[dict] = None           # serialized FixtureSpec
    setups: dict = field(default_factory=dict)  # setup_name -> serialized Setup
    work_offsets: dict = field(default_factory=dict)  # e.g. {"G54": [300, 200, 50, 0, 0, 0]}
    machine: Optional[dict] = None           # serialized MachineContext
    controller: Optional[dict] = None        # serialized controller/post capability context
    setup_intent: Optional[dict] = None      # datum/workholding/orientation intent
    tooling: Optional[dict] = None           # tool assemblies and magazine context
    quality: Optional[dict] = None           # tolerances, inspection, finish requirements
    collision_policy: Optional[dict] = None  # collision/clearance validation policy

    def __post_init__(self) -> None:
        """Normalize any operations supplied at construction time."""
        self.operations = [
            normalize_operation(op, idx)
            for idx, op in enumerate(self.operations, start=1)
        ]
        self.tools_used = [_normalize_tool_entry(t) for t in self.tools_used]
        if not self.tools_used:
            self._rebuild_tools_used()

    # ------------------------------------------------------------------
    # Derived properties
    # ------------------------------------------------------------------

    @property
    def total_operations(self) -> int:
        """Return the total number of operations in the plan."""
        return len(self.operations)

    @property
    def estimated_time_minutes(self) -> float:
        """Estimate machining time from authored toolpaths or approximate dimensions.

        Authored toolpath estimates/lengths are preferred. Operations without
        usable path data fall back to the legacy dimension/feed approximation,
        then finally to 2 min per operation.
        """
        total = 0.0
        for op in self.operations:
            feeds = op.get("feeds_speeds")
            feed_rate = None
            if feeds:
                feed_rate = feeds.get("feed_rate_mm_per_min", feeds.get("feed_rate_mm_min"))

            authored_time = _toolpath_estimated_time_minutes(op, feed_rate)
            if authored_time is not None:
                total += authored_time
                continue

            if feed_rate:
                # Approximate toolpath length from op dimensions
                dims = op.get("dimensions", {})
                length = float(dims.get("length", 0))
                width = float(dims.get("width", 0))
                depth_pass = float(dims.get("depth", 0))
                # Simple perimeter-based estimate: 2*(L+W) per depth pass
                if length > 0 and width > 0:
                    perimeter = 2 * (length + width)
                    # Estimate number of passes from depth
                    step_down = feeds.get("step_down_mm", 1.0)
                    passes = max(1.0, depth_pass / step_down) if step_down > 0 else 1.0
                    toolpath_approx = perimeter * passes
                    total += toolpath_approx / feed_rate
                else:
                    total += 2.0  # fallback when dims are unavailable
            else:
                total += 2.0  # default 2 min per op
        return round(total, 2)

    @property
    def tool_change_count(self) -> int:
        """Count unique tool changes (when tool type or diameter changes between ops)."""
        changes = 0
        prev_key = None
        for op in self.operations:
            tool = op.get("tool", {})
            tool_type = tool.get("tool_type", op.get("tool_type", ""))
            tool_dia = tool.get("diameter_mm", op.get("tool_diameter_mm", 0))
            cur_key = (tool_type, tool_dia)
            if prev_key is not None and cur_key != prev_key:
                changes += 1
            prev_key = cur_key
        return changes

    # ------------------------------------------------------------------
    # Mutators
    # ------------------------------------------------------------------

    def add_operation(self, op_dict: dict) -> None:
        """Add an operation to the plan and track its tool."""
        op_dict = normalize_operation(op_dict, len(self.operations) + 1)
        self.operations.append(op_dict)
        # Register the tool if it hasn't been recorded yet
        tool_type = op_dict.get("tool_type", "")
        tool_dia = op_dict.get("tool_diameter_mm", 0)
        already_tracked = any(
            t.get("tool_type") == tool_type
            and t.get("diameter_mm", t.get("tool_diameter_mm")) == tool_dia
            for t in self.tools_used
        )
        if not already_tracked and tool_type:
            tool = dict(op_dict["tool"])
            tool["tool_diameter_mm"] = tool_dia
            self.tools_used.append(tool)

    def _rebuild_tools_used(self) -> None:
        """Rebuild structured unique-tool metadata from normalized operations."""
        self.tools_used = []
        for op in self.operations:
            tool = op.get("tool") or _tool_metadata(op)
            tool_type = tool.get("tool_type", "")
            tool_dia = tool.get("diameter_mm")
            if tool_type and not any(
                t.get("tool_type") == tool_type and t.get("diameter_mm") == tool_dia
                for t in self.tools_used
            ):
                self.tools_used.append(_normalize_tool_entry(tool))

    @property
    def validation_buckets(self) -> dict:
        """Return plan-level validation buckets for shop-floor consumers."""
        buckets = _empty_validation_buckets()

        try:
            from .validation import validate_all

            report = validate_all({
                    "schema_version": self.schema_version,
                    "material": self.material,
                    "stock_dimensions": self.stock_dimensions,
                    "operations": self.operations,
                    "fixture": self.fixture,
                    "setups": self.setups,
                    "work_offsets": self.work_offsets,
                    "machine": self.machine,
                    "controller": self.controller,
                    "setup_intent": self.setup_intent,
                    "tooling": self.tooling,
                    "quality": self.quality,
                    "collision_policy": self.collision_policy,
                }, self.stock_dimensions, structured=True)
            _append_bucket(buckets, "warnings", report.warnings)
            _append_bucket(buckets, "errors", report.errors)
        except Exception as error:
            _append_bucket(buckets, "errors", [f"Process validation failed: {error}"])

        for op in self.operations:
            _merge_validation_buckets(buckets, op.get("validation_buckets"))
        return buckets

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        """Return a full serializable dict representation of the plan."""
        return {
            "schema_version": self.schema_version,
            "units": {"linear": "mm", "time": "min", "angle": "deg"},
            "material": self.material,
            "stock_dimensions": self.stock_dimensions,
            "total_operations": self.total_operations,
            "estimated_time_minutes": self.estimated_time_minutes,
            "tool_change_count": self.tool_change_count,
            "tools_used": self.tools_used,
            "operations": self.operations,
            "validation": self.validation_buckets,
            "validation_buckets": self.validation_buckets,
            "fixture": self.fixture,
            "setups": self.setups,
            "work_offsets": self.work_offsets,
            "machine": self.machine,
            "controller": self.controller,
            "setup_intent": self.setup_intent,
            "tooling": self.tooling,
            "quality": self.quality,
            "collision_policy": self.collision_policy,
        }

    def to_json(self, indent: int = 2) -> str:
        """Serialize to a JSON string."""
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, path: str) -> None:
        """Save the plan as a JSON file to *path*."""
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.to_json())

    def estimate_cost(
        self,
        machine: str = "generic_3axis_mill",
        quantity: int = 1,
        currency: Optional[str] = None,
    ) -> dict:
        """Return an engineering time/cost estimate for this process plan."""
        from .economics import estimate_cost

        return estimate_cost(self, machine=machine, quantity=quantity, currency=currency)

    def _resolve_economics(self, economics):
        if economics is None or economics is False:
            return None
        if isinstance(economics, dict):
            return economics
        return self.estimate_cost()

    def setup_sheet_dict(self, economics=None) -> dict:
        """Return a shop-floor setup sheet as a structured JSON-ready dict."""
        economics_report = self._resolve_economics(economics)
        operations = [
            {
                "sequence_number": op.get("sequence_number", idx),
                "operation": op.get("operation", "?"),
                "setup": op.get("setup", ""),
                "face_selector": op.get("face_selector", ""),
                "tool": op.get("tool", _tool_metadata(op)),
                "feeds_speeds": op.get("feeds_speeds"),
                "toolpath_summary": op.get("toolpath_summary"),
                "pass_plan": op.get("pass_plan"),
                "tool_selection": op.get("tool_selection"),
                "economics": _operation_economics(op.get("sequence_number", idx), economics_report),
                "notes": op.get("notes", ""),
                "validation_buckets": op.get("validation_buckets", _empty_validation_buckets()),
            }
            for idx, op in enumerate(self.operations, start=1)
        ]
        operations_by_setup: dict[str, list[dict]] = {}
        for op in operations:
            setup_name = op.get("setup") or "unassigned"
            operations_by_setup.setdefault(setup_name, []).append(op)

        sheet = {
            "schema_version": self.schema_version,
            "units": {"linear": "mm", "time": "min", "angle": "deg"},
            "material": self.material or "unspecified",
            "stock_dimensions": self.stock_dimensions,
            "estimated_time_minutes": self.estimated_time_minutes,
            "tool_change_count": self.tool_change_count,
            "fixture": self.fixture,
            "setups": self.setups,
            "work_offsets": self.work_offsets,
            "machine": self.machine,
            "controller": self.controller,
            "setup_intent": self.setup_intent,
            "tooling_context": self.tooling,
            "quality": self.quality,
            "collision_policy": self.collision_policy,
            "tools": self.tools_used,
            "operations": operations,
            "operations_by_setup": operations_by_setup,
            "validation_buckets": self.validation_buckets,
            "gcode_status": "G-code export deferred; this setup sheet is neutral shop-floor intent.",
        }
        if economics_report is not None:
            sheet["economics"] = economics_report
            sheet["total_time_minutes"] = economics_report["time_breakdown"]["total_time_per_part_min"]
            sheet["total_cost"] = economics_report["cost_breakdown"]["total_cost"]
            sheet["currency"] = economics_report["currency"]
            sheet["machine_id"] = economics_report["machine_id"]
            sheet["economics_note"] = "Engineering estimate only; not a production quote."
        return sheet

    def setup_sheet_json(self, indent: int = 2, economics=None) -> str:
        """Serialize the setup sheet to JSON."""
        return json.dumps(self.setup_sheet_dict(economics=economics), indent=indent)

    def setup_sheet_markdown(self, economics=None) -> str:
        """Return a Markdown setup sheet for shop-floor review."""
        sheet = self.setup_sheet_dict(economics=economics)
        lines = [
            "# SubCAD Setup Sheet",
            "",
            f"- Schema: {sheet['schema_version']}",
            f"- Material: {sheet['material']}",
            f"- Stock: {sheet['stock_dimensions']}",
            f"- Estimated time: {sheet['estimated_time_minutes']} min",
            f"- Tool changes: {sheet['tool_change_count']}",
            f"- G-code: deferred; neutral shop-floor intent only",
        ]
        if sheet.get("economics"):
            econ = sheet["economics"]
            lines.extend([
                f"- Machine: {econ['machine_id']}",
                f"- Total time: {econ['time_breakdown']['total_time_per_part_min']} min/part",
                f"- Total cost: {econ['currency']} {econ['cost_breakdown']['total_cost']}",
                "- Economics: engineering estimate only; not a production quote",
            ])
        elif sheet.get("machine"):
            machine = sheet["machine"]
            lines.append(f"- Machine: {machine.get('name', machine.get('machine_id', '?'))}")
        if sheet.get("controller"):
            controller = sheet["controller"]
            lines.append(
                f"- Controller: {controller.get('name', controller.get('controller_id', '?'))}"
            )
        if sheet.get("setup_intent"):
            setup_intent = sheet["setup_intent"]
            primary = setup_intent.get("primary_face") or setup_intent.get("face_selector")
            if primary:
                lines.append(f"- Primary setup face: {primary}")
        if sheet.get("collision_policy"):
            policy = sheet["collision_policy"]
            if policy.get("required") or policy.get("enabled"):
                lines.append("- Collision validation: required")
        lines.extend(["", "## Tools"])
        if sheet["tools"]:
            for tool in sheet["tools"]:
                lines.append(f"- {tool.get('label', tool.get('tool_type', '?'))}")
        else:
            lines.append("- None")

        lines.extend(["", "## Operations"])
        if sheet.get("operations_by_setup"):
            for setup_name, ops in sheet["operations_by_setup"].items():
                lines.append(f"### Setup {setup_name}")
                for op in ops:
                    tool = op.get("tool") or {}
                    detail = f"{op['sequence_number']}. {op['operation']} - {tool.get('label', '?')}"
                    pass_plan = op.get("pass_plan") or {}
                    if pass_plan.get("total_passes"):
                        detail += f" - {pass_plan.get('total_passes')} passes"
                    if pass_plan.get("estimated_time_min") is not None:
                        detail += f" - {pass_plan.get('estimated_time_min')} min"
                    if op.get("face_selector"):
                        detail += f" - face {op['face_selector']}"
                    if op.get("notes"):
                        detail += f" - {op['notes']}"
                    lines.append(detail)
        else:
            for op in sheet["operations"]:
                tool = op.get("tool") or {}
                detail = f"{op['sequence_number']}. {op['operation']} - {tool.get('label', '?')}"
                if op.get("setup"):
                    detail += f" - setup {op['setup']}"
                if op.get("notes"):
                    detail += f" - {op['notes']}"
                lines.append(detail)
        lines.extend(["", "## Warnings"])
        warnings = sheet.get("validation_buckets", {}).get("warnings", [])
        if sheet.get("economics"):
            warnings = warnings + sheet["economics"].get("economics_warnings", [])
        if warnings:
            for warning in warnings:
                lines.append(f"- {warning}")
        else:
            lines.append("- No blocking warnings recorded.")
        return "\n".join(lines)

    def setup_sheet_text(self, economics=None) -> str:
        """Return a plain-text setup sheet."""
        lines = []
        for line in self.setup_sheet_markdown(economics=economics).splitlines():
            if line.startswith("## "):
                lines.append(line[3:])
            elif line.startswith("# "):
                lines.append(line[2:])
            else:
                lines.append(line)
        return "\n".join(lines)

    def to_setup_sheet_json(self, indent: int = 2, economics=None) -> str:
        """Compatibility alias for setup_sheet_json."""
        return self.setup_sheet_json(indent=indent, economics=economics)

    def to_setup_sheet_markdown(self, economics=None) -> str:
        """Compatibility alias for setup_sheet_markdown."""
        return self.setup_sheet_markdown(economics=economics)

    def to_setup_sheet_text(self, economics=None) -> str:
        """Compatibility alias for setup_sheet_text."""
        return self.setup_sheet_text(economics=economics)

    def gcode_preview(self) -> str:
        """Return preview-only G-code-like text from neutral toolpaths."""
        from .gcode_preview import gcode_preview

        return gcode_preview(self)

    def to_gcode_preview(self) -> str:
        """Compatibility alias for gcode_preview."""
        return self.gcode_preview()

    def postprocess(
        self,
        controller: str = "generic_3axis_debug",
        machine: str = "generic_3axis_mill",
        safe_mode: bool = True,
    ) -> str:
        """Return debug-only postprocessor output for this process plan."""
        from .postprocessor import postprocess

        return postprocess(self, controller=controller, machine=machine, safe_mode=safe_mode)

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def summary(self) -> str:
        """Return a human-readable summary listing operations in order."""
        lines = [
            f"ProcessPlan: {self.total_operations} operation(s) on {self.material or 'unspecified'} stock",
            f"  Estimated time: {self.estimated_time_minutes} min",
            f"  Tool changes:   {self.tool_change_count}",
            f"  Unique tools:   {len(self.tools_used)}",
        ]
        if self.fixture:
            lines.append(f"  Fixture:        {self.fixture.get('name', '?')}"
                         f" ({self.fixture.get('fixture_type', '?')})")
        if self.setups:
            lines.append("  Setups:")
            for sname, sdata in self.setups.items():
                face = sdata.get("face_selector", "?")
                woff = sdata.get("work_offset", "?")
                desc = sdata.get("description", "")
                lines.append(f"    {sname}: face={face} offset={woff}"
                             + (f" -- {desc}" if desc else ""))
        lines.append("")
        for i, op in enumerate(self.operations, start=1):
            tool = op.get("tool_type", "?")
            dia = op.get("tool_diameter_mm", "?")
            name = op.get("operation", "?")
            notes = op.get("notes", "")
            setup = op.get("setup", "")
            face = op.get("face_selector", "")
            extra = []
            if setup:
                extra.append(setup)
            if face and face != ">Z":
                extra.append(f"face={face}")
            if notes:
                extra.append(notes)
            suffix = " -- " + ", ".join(extra) if extra else ""
            lines.append(f"  {i}. [{tool} D={dia}mm] {name}{suffix}")
        return "\n".join(lines)


# ======================================================================
# Self-test – runs when the module is executed directly
# ======================================================================
if __name__ == "__main__":
    import sys

    passed = 0
    failed = 0

    def check(condition, label):
        global passed, failed
        if condition:
            passed += 1
            print(f"  PASS  {label}")
        else:
            failed += 1
            print(f"  FAIL  {label}")

    print("=== ProcessPlan self-tests ===\n")

    # --- Create a ProcessPlan and add operations ---
    plan = ProcessPlan(
        material="6061-T6 Aluminum",
        stock_dimensions={"length": 100, "width": 50, "height": 20},
    )

    plan.add_operation({
        "operation": "face_mill",
        "tool_type": "face_mill",
        "tool_diameter_mm": 50.0,
        "dimensions": {"length": 100, "width": 50, "depth": 1.0},
        "feeds_speeds": {"feed_rate_mm_per_min": 500, "step_down_mm": 1.0},
        "notes": "Face top surface",
    })

    plan.add_operation({
        "operation": "rough_pocket",
        "tool_type": "end_mill",
        "tool_diameter_mm": 10.0,
        "dimensions": {"length": 60, "width": 30, "depth": 10.0},
        "feeds_speeds": {"feed_rate_mm_per_min": 300, "step_down_mm": 2.0},
        "notes": "Rough pocket",
    })

    plan.add_operation({
        "operation": "drill",
        "tool_type": "drill",
        "tool_diameter_mm": 6.0,
        "notes": "Drill mounting holes",
    })

    print("1. Plan construction and add_operation")
    check(plan.total_operations == 3, "total_operations == 3")
    check(len(plan.tools_used) == 3, "3 unique tools tracked")
    print()

    # --- to_dict ---
    d = plan.to_dict()
    print("2. to_dict")
    check(isinstance(d, dict), "to_dict returns dict")
    check(d["material"] == "6061-T6 Aluminum", "material preserved")
    check(d["total_operations"] == 3, "total_operations in dict")
    check("operations" in d, "operations key present")
    check("tools_used" in d, "tools_used key present")
    check("estimated_time_minutes" in d, "estimated_time_minutes key present")
    check("tool_change_count" in d, "tool_change_count key present")
    check(d["schema_version"] == SCHEMA_VERSION, "schema_version in dict")
    check(d["operations"][0]["sequence_number"] == 1, "sequence number assigned")
    check(d["operations"][0]["tool"]["diameter_mm"] == 50.0, "structured tool metadata present")
    check("tool_diameter_mm" in d["operations"][0], "legacy tool_diameter_mm preserved")
    check(d["tools_used"][0]["tool_diameter_mm"] == 50.0, "legacy tool list diameter preserved")
    check("validation_buckets" in d, "plan validation_buckets present")
    print()

    # --- to_json ---
    json_str = plan.to_json()
    print("3. to_json")
    check(isinstance(json_str, str), "to_json returns str")
    parsed = json.loads(json_str)
    check(isinstance(parsed, dict), "to_json produces valid JSON")
    check(parsed["total_operations"] == 3, "JSON round-trips total_operations")
    print()

    # --- estimated_time_minutes ---
    est = plan.estimated_time_minutes
    print("4. estimated_time_minutes")
    check(isinstance(est, (int, float)) and est > 0, f"estimated_time > 0 (got {est})")
    # First op: perimeter=300, passes=1, time=300/500=0.6
    # Second op: perimeter=180, passes=5, toolpath=900, time=900/300=3.0
    # Third op: no feed data -> 2.0 fallback
    # Total = 0.6 + 3.0 + 2.0 = 5.6
    check(abs(est - 5.6) < 0.01, f"estimated_time ~ 5.6 (got {est})")
    print()

    # --- schema v1 normalization ---
    print("4b. schema v1 normalization")
    legacy_plan = ProcessPlan(
        operations=[
            {
                "operation": "slot",
                "tool_type": "flat_endmill",
                "tool_diameter_mm": 8.0,
                "dimensions": {"length": 20, "width": 8, "depth": 2.0},
                "feeds_speeds": {"feed_rate_mm_min": 280, "step_down_mm": 1.0},
                "toolpath": [[0, 0, 0], [10, 0, -1], [20, 0, -2]],
                "validation": {"warnings": ["Verify slot clearance"]},
            }
        ],
    )
    legacy_op = legacy_plan.operations[0]
    check(legacy_op["feeds_speeds"]["feed_rate_mm_per_min"] == 280,
          "feed_rate_mm_min normalized to feed_rate_mm_per_min")
    check(legacy_op["feeds_speeds"]["feed_rate_mm_min"] == 280,
          "feed_rate_mm_min preserved")
    check(legacy_op["toolpath_summary"]["point_count"] == 3,
          "toolpath summary generated")
    check("Verify slot clearance" in legacy_plan.validation_buckets["warnings"],
          "validation buckets merged")
    check(legacy_plan.estimated_time_minutes == 0.07,
          f"authored toolpath length used for estimate (got {legacy_plan.estimated_time_minutes})")

    authored_time_plan = ProcessPlan(
        operations=[
            {
                "operation": "drill",
                "tool_type": "drill",
                "tool_diameter_mm": 6.0,
                "dimensions": {"length": 200, "width": 200, "depth": 20},
                "feeds_speeds": {"feed_rate_mm_per_min": 100},
                "toolpath": {
                    "safe_z": 5.0,
                    "estimated_time_min": 1.25,
                    "length_mm": 42.0,
                    "moves": [
                        {"type": "rapid", "position": [0, 0, 5]},
                        {"type": "plunge", "position": [0, 0, -5]},
                        {"type": "retract", "position": [0, 0, 5]},
                    ],
                },
            }
        ],
    )
    check(authored_time_plan.operations[0]["toolpath_summary"]["length_mm"] == 42.0,
          "authored toolpath length summarized")
    check(authored_time_plan.estimated_time_minutes == 1.25,
          f"authored toolpath estimated time preferred "
          f"(got {authored_time_plan.estimated_time_minutes})")
    print()

    # --- tool_change_count ---
    print("5. tool_change_count")
    # face_mill/50 -> end_mill/10  (change)
    # end_mill/10  -> drill/6      (change)
    # = 2 changes
    check(plan.tool_change_count == 2, f"tool_change_count == 2 (got {plan.tool_change_count})")
    print()

    # --- save ---
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as tmp:
        tmp_path = tmp.name
    try:
        plan.save(tmp_path)
        with open(tmp_path, "r", encoding="utf-8") as fh:
            reloaded = json.load(fh)
        print("6. save")
        check(isinstance(reloaded, dict), "save writes valid JSON")
        check(reloaded["material"] == "6061-T6 Aluminum", "save preserves material")
        check(reloaded["total_operations"] == 3, "save preserves total_operations")
    finally:
        import os
        os.unlink(tmp_path)
    print()

    # --- summary ---
    print("7. summary (human-readable output)")
    print(plan.summary())
    print()

    # --- OperationStep ---
    print("8. OperationStep")
    step = OperationStep(
        sequence_number=1,
        operation="face_mill",
        tool_type="face_mill",
        tool_diameter_mm=50.0,
        depth_mm=1.0,
        position=(0, 0),
        feeds_speeds={"rpm": 3000, "feed_rate_mm_per_min": 500},
        notes="Face top surface",
    )
    sd = step.to_dict()
    check(sd["sequence_number"] == 1, "OperationStep sequence_number")
    check(sd["operation"] == "face_mill", "OperationStep operation")
    check(sd["position"] == [0, 0], "OperationStep position serialized as list")
    check("feeds_speeds" in sd, "OperationStep feeds_speeds present")
    print()

    # --- setup sheet exports ---
    print("9. setup sheet exports")
    sheet = plan.setup_sheet_dict()
    sheet_json = plan.setup_sheet_json()
    sheet_md = plan.setup_sheet_markdown()
    sheet_text = plan.setup_sheet_text()
    check(sheet["schema_version"] == SCHEMA_VERSION, "setup_sheet_dict schema_version")
    check(json.loads(sheet_json)["operations"][0]["operation"] == "face_mill",
          "setup_sheet_json valid")
    check("# SubCAD Setup Sheet" in sheet_md, "setup_sheet_markdown heading")
    check("SubCAD Setup Sheet" in sheet_text and "#" not in sheet_text.splitlines()[0],
          "setup_sheet_text plain heading")
    print()

    # --- Report ---
    print(f"=== Results: {passed} passed, {failed} failed ===")
    sys.exit(0 if failed == 0 else 1)
