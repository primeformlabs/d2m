"""SubCAD operations -- MachiningOperation ABC and Phase 1 concrete operations.

Each machining operation is BOTH a geometry modifier AND a manufacturing
instruction.  Operations modify a CadQuery B-Rep solid AND record themselves
into a ProcessPlan.

Phase 1 concrete operations (8 ops):
    FaceMillOp       -- face milling on top surface
    PocketOp         -- rectangular pocket (rough + optional finish)
    CircularPocketOp -- circular pocket
    DrillOp          -- drilling (with optional spot-drill)
    TapOp            -- thread tapping (geometry unchanged)
    ChamferOp        -- edge chamfering
    ContourOp        -- profile / contour milling
    SlotOp           -- slot milling

Plus the internal SpotDrillOp used by DrillOp when spot_drill=True.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import math
from typing import Any, Optional, Tuple

from .geometry import (
    face_mill_cut,
    rectangular_pocket_cut,
    sloped_rectangular_cut,
    circular_pocket_cut,
    drill_hole,
    countersink_cut,
    slot_cut,
    contour_cut_outer,
    chamfer_edges,
    fillet_edges,
    profile_pocket_cut,
    profile_cutout_cut,
    profile_contour_cut,
    machine_around_profile_cut,
    machine_around_profiles_cut,
    machine_around_cylinder_cut,
    thin_wall_pocket_cut,
    thin_wall_shell_cut,
    create_tapered_cylinder,
    create_axis_aligned_tube,
    create_axis_aligned_profile_tube,
    press_brake_bend as apply_press_brake_bend,
)
from .profiles import normalize_profile_points, profile_span_yx
from .tool_library import ToolSpec, ToolCatalog
from .tool_selection import ToolRequirement, ToolSelectionResult, select_or_fallback, select_tool
from .material import get_feeds_speeds
from .toolpath import Toolpath, ToolpathMove
from . import turning_geometry

# ---------------------------------------------------------------------------
# Tool-type mapping for feeds/speeds lookup
# ---------------------------------------------------------------------------
# ToolSpec.tool_type values (e.g. "flat_endmill") do not always match the
# keys expected by the knowledge-graph feeds/speeds engine (e.g. "end_mill").
# This mapping bridges the two.  An unrecognised type is passed through
# unchanged so that the callee can return its standard error dict.

_TOOL_TYPE_FS_MAP: dict[str, str] = {
    "flat_endmill": "end_mill",
    "ball_endmill": "ball_endmill",
    "drill":        "drill",
    "chamfer":      "chamfer",
    "thread_mill":  "tap",
    "bull_nose":    "end_mill",
    "dovetail":     "end_mill",
    "reamer":       "drill",
    "spot_drill":   "drill",
}


def _resolve_fs_tool_type(tool_type: str) -> str:
    """Map a ToolSpec tool_type to the key expected by get_feeds_speeds."""
    return _TOOL_TYPE_FS_MAP.get(tool_type, tool_type)


def _feed_rate(feeds_speeds: dict, fallback: float = 100.0) -> float:
    if "error" in feeds_speeds:
        return fallback
    return float(feeds_speeds.get("feed_rate_mm_min") or fallback)


def _spindle_rpm(feeds_speeds: dict) -> Optional[float]:
    if "error" in feeds_speeds:
        return None
    rpm = feeds_speeds.get("spindle_rpm", feeds_speeds.get("rpm"))
    return float(rpm) if rpm else None


def _new_toolpath(
    operation: str,
    tool: ToolSpec,
    feeds_speeds: dict,
    *,
    safe_z: float = 5.0,
    coolant: bool = True,
    metadata: Optional[dict] = None,
) -> Toolpath:
    return Toolpath(
        operation=operation,
        safe_z=safe_z,
        tool_type=tool.tool_type,
        tool_diameter_mm=tool.diameter,
        feed_mm_min=_feed_rate(feeds_speeds),
        spindle_rpm=_spindle_rpm(feeds_speeds),
        coolant=coolant,
        metadata={"tool": _tool_dict(tool), **(metadata or {})},
    )


def _tool_dict(tool: Optional[ToolSpec]) -> dict:
    """Return structured tool metadata for process-plan and viewer consumers."""
    if tool is None:
        return {}
    if hasattr(tool, "to_dict"):
        return tool.to_dict()
    return {
        "tool_type": getattr(tool, "tool_type", ""),
        "diameter_mm": getattr(tool, "diameter", None),
        "tool_diameter_mm": getattr(tool, "diameter", None),
    }


def _tool_selection_dict(result: Optional[ToolSelectionResult]) -> Optional[dict]:
    if result is None:
        return None
    return result.to_dict()


def _tool_safe_limit(tool: ToolSpec, key: str, default: float) -> float:
    value = (tool.safe_limits or {}).get(key)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _tool_stepdown(tool: ToolSpec, default_ratio: float = 0.5, minimum: float = 0.5) -> float:
    return max(
        min(
            tool.diameter * default_ratio,
            _tool_safe_limit(tool, "max_stepdown_mm", tool.diameter * default_ratio),
        ),
        minimum,
    )


def _tool_stepover(tool: ToolSpec, default_ratio: float = 0.5, minimum: float = 0.1) -> float:
    safe_ratio = _tool_safe_limit(tool, "max_stepover_ratio", default_ratio)
    return max(tool.diameter * min(default_ratio, safe_ratio), minimum)


def _pass_plan(
    *,
    strategy: str,
    tool: ToolSpec,
    depth_levels: list[float],
    lane_count: int = 1,
    pass_type: str = "roughing",
    rest_machining: bool = False,
    finish_passes: int = 0,
) -> dict:
    roughing_passes = max(len(depth_levels) * max(lane_count, 1), 0)
    total_passes = roughing_passes + max(finish_passes, 0)
    return {
        "strategy": strategy,
        "tool_id": tool.catalog_id,
        "tool_diameter_mm": tool.diameter,
        "roughing_passes": roughing_passes,
        "finishing_passes": max(finish_passes, 0),
        "rest_machining": bool(rest_machining),
        "total_passes": total_passes,
        "depth_levels_mm": [round(abs(z), 6) for z in depth_levels],
        "stepover_lanes": max(lane_count, 1),
        "pass_type": pass_type,
    }


def _attach_pass_plan(tp: Toolpath, pass_plan: dict) -> Toolpath:
    pass_plan = dict(pass_plan)
    total_passes = max(int(pass_plan.get("total_passes") or 0), 1)
    pass_plan["estimated_time_min"] = round(tp.estimated_time_min, 6)
    pass_plan["estimated_time_per_pass_min"] = round(tp.estimated_time_min / total_passes, 6)
    pass_plan["length_mm"] = round(tp.length_mm, 6)
    tp.metadata["pass_plan"] = pass_plan
    return tp


def _select_inventory_tool(
    requirement: ToolRequirement,
    fallback: ToolSpec,
) -> tuple[ToolSpec, ToolSelectionResult]:
    return select_or_fallback(requirement, fallback)


def _move(
    tp: Toolpath,
    move_type: str,
    x: float,
    y: float,
    z: float,
    *,
    feed: Optional[float] = None,
    dwell_seconds: float = 0.0,
    arc_center: Optional[Tuple[float, float, float]] = None,
    arc_direction: Optional[str] = None,
    metadata: Optional[dict] = None,
) -> None:
    tp.append(ToolpathMove(
        move_type=move_type,
        position=(x, y, z),
        feed=feed,
        spindle_rpm=tp.spindle_rpm,
        coolant=tp.coolant,
        dwell_seconds=dwell_seconds,
        arc_center=arc_center,
        arc_direction=arc_direction,
        metadata=metadata or {},
    ))


def _depth_levels(depth: float, stepdown: Optional[float]) -> list[float]:
    depth = abs(depth)
    if depth <= 0:
        return [0.0]
    if stepdown is None or stepdown <= 0:
        return [-depth]
    levels: list[float] = []
    current = min(stepdown, depth)
    while current < depth:
        levels.append(-current)
        current += stepdown
    levels.append(-depth)
    return levels


def _drill_toolpath(
    operation: str,
    tool: ToolSpec,
    feeds_speeds: dict,
    cx: float,
    cy: float,
    depth: float,
    *,
    peck_step: Optional[float] = None,
    dwell_seconds: float = 0.0,
    metadata: Optional[dict] = None,
) -> Toolpath:
    tp = _new_toolpath(
        operation,
        tool,
        feeds_speeds,
        metadata={"x": cx, "y": cy, **(metadata or {})},
    )
    feed = _feed_rate(feeds_speeds)
    _move(tp, "rapid", cx, cy, tp.safe_z)
    levels = _depth_levels(depth, peck_step if peck_step and peck_step > 0 else abs(depth))
    if dwell_seconds:
        _move(tp, "dwell", cx, cy, tp.safe_z, dwell_seconds=dwell_seconds)
    if peck_step and peck_step > 0:
        current = 0.0
        for level in levels:
            _move(tp, "plunge", cx, cy, level, feed=feed)
            current = level
            _move(tp, "retract", cx, cy, tp.safe_z)
            if level != -abs(depth):
                _move(tp, "rapid", cx, cy, min(current + peck_step * 0.25, -0.1))
    else:
        _move(tp, "plunge", cx, cy, -abs(depth), feed=feed)
        _move(tp, "retract", cx, cy, tp.safe_z)
    return _attach_pass_plan(tp, _pass_plan(
        strategy="peck_drill" if peck_step else "drill",
        tool=tool,
        depth_levels=levels,
        lane_count=1,
        pass_type="drilling",
    ))


def _slot_endpoints(cx: float, cy: float, length: float, angle_deg: float) -> tuple[tuple[float, float], tuple[float, float]]:
    theta = math.radians(angle_deg)
    half = length / 2.0
    dx = math.cos(theta) * half
    dy = math.sin(theta) * half
    return (cx - dx, cy - dy), (cx + dx, cy + dy)


def _linear_slot_toolpath(
    operation: str,
    tool: ToolSpec,
    feeds_speeds: dict,
    cx: float,
    cy: float,
    length: float,
    width: float,
    depth: float,
    angle: float,
    *,
    metadata: Optional[dict] = None,
) -> Toolpath:
    """Generate a conservative neutral slot/groove/form-cutter path."""
    stepdown = _tool_stepdown(tool, 0.5, 0.5)
    stepover = max(min(_tool_stepover(tool, 0.5, 0.1), max(width * 0.25, 0.1)), 0.1)
    (x0, y0), (x1, y1) = _slot_endpoints(cx, cy, length, angle)
    theta = math.radians(angle)
    nx = -math.sin(theta)
    ny = math.cos(theta)
    offsets = [0.0]
    side = stepover
    max_side = max(width / 2.0 - tool.radius, 0.0)
    while side <= max_side + 1e-9:
        offsets.extend([side, -side])
        side += stepover

    tp = _new_toolpath(
        operation,
        tool,
        feeds_speeds,
        metadata={
            "slot_length_mm": length,
            "slot_width_mm": width,
            "angle_deg": angle,
            **(metadata or {}),
        },
    )
    feed = _feed_rate(feeds_speeds)
    _move(tp, "rapid", x0, y0, tp.safe_z)
    direction = 1
    depth_levels = _depth_levels(depth, stepdown)
    for z in depth_levels:
        for offset in offsets:
            sx = x0 + nx * offset
            sy = y0 + ny * offset
            ex = x1 + nx * offset
            ey = y1 + ny * offset
            start = (sx, sy) if direction > 0 else (ex, ey)
            end = (ex, ey) if direction > 0 else (sx, sy)
            _move(tp, "rapid", start[0], start[1], tp.safe_z)
            _move(tp, "plunge", start[0], start[1], z, feed=feed)
            _move(tp, "feed", end[0], end[1], z, feed=feed)
            _move(tp, "retract", end[0], end[1], tp.safe_z)
            direction *= -1
    return _attach_pass_plan(tp, _pass_plan(
        strategy=operation,
        tool=tool,
        depth_levels=depth_levels,
        lane_count=len(offsets),
        pass_type="slotting",
    ))


def _rect_raster_toolpath(
    operation: str,
    tool: ToolSpec,
    feeds_speeds: dict,
    width: float,
    length: float,
    depth: float,
    stepover: float,
    *,
    metadata: Optional[dict] = None,
) -> Toolpath:
    """Generate a neutral raster finishing/deburring path around the origin."""
    stepover = max(stepover, 0.1)
    half_l = length / 2.0
    half_w = width / 2.0
    z = -abs(depth)
    tp = _new_toolpath(operation, tool, feeds_speeds, metadata=metadata or {})
    feed = _feed_rate(feeds_speeds)
    y = -half_w
    direction = 1
    lane_count = 0
    _move(tp, "rapid", -half_l, y, tp.safe_z)
    _move(tp, "plunge", -half_l, y, z, feed=feed)
    while y <= half_w + 1e-9:
        lane_count += 1
        x = half_l if direction > 0 else -half_l
        _move(tp, "feed", x, y, z, feed=feed)
        y += stepover
        if y <= half_w + 1e-9:
            _move(tp, "feed", x, y, z, feed=feed)
        direction *= -1
    last_x, last_y, _ = tp.moves[-1].position
    _move(tp, "retract", last_x, last_y, tp.safe_z)
    return _attach_pass_plan(tp, _pass_plan(
        strategy=operation,
        tool=tool,
        depth_levels=[z],
        lane_count=lane_count,
        pass_type="finishing",
    ))


def _profile_bounds(profile: Any) -> tuple[float, float]:
    """Best-effort profile bounding box for pure operation placeholders."""
    return profile_span_yx(profile)


def _profile_payload(profile: Any) -> Any:
    """JSON-friendly profile summary without importing opaque CAD geometry."""
    if isinstance(profile, (str, int, float, bool)) or profile is None:
        return profile
    if isinstance(profile, dict):
        return profile
    if isinstance(profile, (list, tuple)):
        return list(profile)
    return {"description": str(profile)}


def _pick_slot_tool(slot_width: float) -> ToolSpec:
    """Pick the best flat endmill that fits within *slot_width*.

    Tries to return a tool close to 80 % of the slot width.  If no catalog
    tool fits, falls back to the smallest available flat endmill and relies
    on the caller's validation note to warn.
    """
    try:
        return ToolCatalog.select(
            "flat_endmill",
            min_diameter=0.0,
            max_diameter=slot_width,
        )
    except ValueError:
        try:
            return ToolCatalog.select("flat_endmill")
        except ValueError:
            return ToolSpec("flat_endmill", max(slot_width * 0.8, 1.0))


def _pick_reamer(hole_diameter: float) -> ToolSpec:
    """Pick the closest reamer tool for a given hole diameter.

    Searches the catalog for tools whose name starts with ``REAM_`` and
    returns the smallest reamer whose diameter >= *hole_diameter*.  If no
    reamer is large enough, the largest available reamer is returned.
    Falls back to a generic drill-type tool with a 45-degree tip angle
    when the catalog contains no reamers at all.
    """
    ToolCatalog._ensure_initialised()
    reamers = sorted(
        (t for name, t in ToolCatalog._tools.items() if name.startswith("REAM_")),
        key=lambda t: t.diameter,
    )
    if reamers:
        for r in reamers:
            if r.diameter >= hole_diameter:
                return r
        return reamers[-1]
    # Fallback: no reamers in catalog
    return ToolSpec("drill", hole_diameter, tip_angle=45.0)


def _laser_tool(kerf_width: float) -> ToolSpec:
    return ToolSpec(
        "fiber_laser",
        max(float(kerf_width), 0.01),
        flute_length=100.0,
        material="laser",
        catalog_id="FIBER_LASER",
        reach_mm=100.0,
    )


def _profile_contour_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    closed = points + [points[0]]
    return sum(math.dist(a, b) for a, b in zip(closed[:-1], closed[1:]))


def _laser_profile_toolpath(
    operation: str,
    profile: Any,
    tool: ToolSpec,
    *,
    cut_speed_mm_min: float,
    kerf_width: float,
    pierce_time_s: float,
) -> Toolpath:
    points = normalize_profile_points(profile)
    tp = Toolpath(
        operation=operation,
        safe_z=5.0,
        tool_type=tool.tool_type,
        tool_diameter_mm=tool.diameter,
        feed_mm_min=float(cut_speed_mm_min),
        coolant=False,
        metadata={
            "process": "laser_cut",
            "kerf_width_mm": float(kerf_width),
            "pierce_time_s": float(pierce_time_s),
            "profile_point_count": len(points),
            "profile_length_mm": round(_profile_contour_length(points), 6),
        },
    )
    if not points:
        return tp
    first_x, first_y = points[0]
    _move(tp, "rapid", first_x, first_y, tp.safe_z)
    _move(tp, "plunge", first_x, first_y, 0.0, feed=cut_speed_mm_min)
    if pierce_time_s > 0:
        _move(tp, "dwell", first_x, first_y, 0.0, dwell_seconds=pierce_time_s)
    for x, y in points[1:]:
        _move(tp, "feed", x, y, 0.0, feed=cut_speed_mm_min)
    _move(tp, "feed", first_x, first_y, 0.0, feed=cut_speed_mm_min)
    _move(tp, "retract", first_x, first_y, tp.safe_z)
    return tp


# =============================================================================
#  Abstract base class
# =============================================================================

@dataclass
class MachiningOperation(ABC):
    """Abstract base for all subtractive machining operations.

    Attributes:
        sequence_number: Order within a ProcessPlan (assigned externally).
        tool: The ToolSpec chosen for this operation.
        feeds_speeds: dict with rpm, feed_rate_mm_min, etc.; empty if unknown.
        position: (x, y) on the top workplane (None for whole-face ops).
    """

    sequence_number: int = 0
    tool: Optional[ToolSpec] = None
    feeds_speeds: dict = field(default_factory=dict)
    position: Optional[Tuple[float, float]] = None
    face_selector: str = ">Z"
    tool_selection: Optional[ToolSelectionResult] = field(default=None, repr=False)

    # -- abstract interface --------------------------------------------------

    @abstractmethod
    def apply(self, shape):
        """Apply this operation to a CadQuery shape.  Returns modified shape."""
        ...

    @abstractmethod
    def to_dict(self) -> dict:
        """Serialize to dict for inclusion in a ProcessPlan."""
        ...

    @abstractmethod
    def to_toolpath(self):
        """Generate a neutral toolpath for simulation or shop-floor handoff.

        Implemented operations return a :class:`Toolpath`; operations that have
        not been mapped yet may still return an empty list.
        """
        ...


# =============================================================================
#  Phase 1 concrete operations
# =============================================================================

# ---------------------------------------------------------------------------
#  1. FaceMillOp
# ---------------------------------------------------------------------------

@dataclass
class FaceMillOp(MachiningOperation):
    """Face milling -- removes material from the entire top face.

    Auto-selects a flat endmill from the catalog; chooses ~50 mm (EM_50mm_FACE)
    for large parts and 10 mm for smaller parts when no tool is given.
    """

    depth: float = 1.0
    """Depth of cut from the top face (mm)."""

    stepover: Optional[float] = None
    """Width of each pass; defaults to tool_diameter * 0.7 when None."""

    finish_pass: bool = False
    """If True, record a finishing pass with lighter parameters."""

    tool: Optional[ToolSpec] = None
    """Face mill tool.  Auto-selected if None."""

    material: str = "aluminum_6061"
    """Workpiece material key."""

    def __post_init__(self):
        if self.tool is None:
            fallback = ToolCatalog.get("EM_50mm_FACE")
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    min_diameter_mm=10.0,
                    min_flute_length_mm=self.depth,
                    feature="face_mill",
                    reason="prefer widest available tool for facing",
                    prefer_largest=True,
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = None  # face mill covers entire surface

    # ------------------------------------------------------------------

    def apply(self, shape):
        return face_mill_cut(shape, self.depth, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "face_mill",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "stepover_mm": (
                self.stepover if self.stepover is not None
                else self.tool.diameter * 0.7
            ),
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "finish_pass": self.finish_pass,
            "face_selector": self.face_selector,
            "notes": "Finishing pass included" if self.finish_pass else "",
        }

    def to_toolpath(self) -> Toolpath:
        stepover = self.stepover if self.stepover is not None else _tool_stepover(self.tool, 0.7, 0.1)
        span_x = max(self.tool.diameter * 4.0, stepover * 4.0)
        span_y = max(self.tool.diameter * 2.0, stepover * 2.0)
        tp = _new_toolpath(
            "face_mill",
            self.tool,
            self.feeds_speeds,
            metadata={
                "nominal_span_x_mm": span_x,
                "nominal_span_y_mm": span_y,
                "tool_selection": _tool_selection_dict(self.tool_selection),
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        y = -span_y / 2.0
        direction = 1
        lane_count = 0
        _move(tp, "rapid", -span_x / 2.0, y, tp.safe_z)
        _move(tp, "plunge", -span_x / 2.0, y, -abs(self.depth), feed=feed)
        while y <= span_y / 2.0 + 1e-9:
            lane_count += 1
            x = span_x / 2.0 if direction > 0 else -span_x / 2.0
            _move(tp, "feed", x, y, -abs(self.depth), feed=feed)
            y += stepover
            if y <= span_y / 2.0 + 1e-9:
                _move(tp, "feed", x, y, -abs(self.depth), feed=feed)
            direction *= -1
        _move(tp, "retract", tp.moves[-1].position[0], tp.moves[-1].position[1], tp.safe_z)
        return _attach_pass_plan(tp, _pass_plan(
            strategy="face_mill",
            tool=self.tool,
            depth_levels=[-abs(self.depth)],
            lane_count=lane_count,
            pass_type="facing",
        ))


# ---------------------------------------------------------------------------
#  2. PocketOp
# ---------------------------------------------------------------------------

@dataclass
class PocketOp(MachiningOperation):
    """Rectangular pocket at a given position.

    If *corner_radius* > 0 and the auto-selected tool radius exceeds the
    corner radius, a secondary finish operation with a smaller tool is
    automatically created and stored in ``finish_op``.
    """

    cx: float = 0.0
    cy: float = 0.0
    width: float = 20.0
    length: float = 30.0
    depth: float = 5.0
    corner_radius: float = 0.0
    pass_role: str = "roughing"
    base_height: Optional[float] = None

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    finish_op: Optional["PocketOp"] = field(default=None, repr=False)
    """Auto-created finish operation for small corner radii (internal)."""

    def __post_init__(self):
        if self.tool is None:
            max_tool_dia = min(self.width, self.length) * 0.8
            fallback = ToolSpec("flat_endmill", max_tool_dia)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    max_diameter_mm=max_tool_dia,
                    min_flute_length_mm=self.depth,
                    feature="rectangular_pocket",
                    reason="largest available end mill that fits pocket opening",
                    prefer_largest=True,
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

        # Auto-finish: a large tool cannot reach into a tight corner radius
        if self.corner_radius > 0 and self.tool.radius > self.corner_radius:
            finish_tool_dia = self.corner_radius * 2.0
            finish_tool, finish_selection = _select_inventory_tool(
                ToolRequirement("flat_endmill", max_diameter_mm=finish_tool_dia,
                    min_flute_length_mm=self.depth, feature="pocket_corner_finish",
                    reason="end mill radius must fit the requested corner", prefer_largest=True),
                ToolSpec("flat_endmill", finish_tool_dia, inventory_count=0),
            )

            # Only create a finish op if the finish tool is actually smaller
            if finish_tool.diameter < self.tool.diameter:
                self.finish_op = PocketOp(
                    cx=self.cx,
                    cy=self.cy,
                    width=self.width,
                    length=self.length,
                    depth=self.depth,
                    corner_radius=self.corner_radius,
                    pass_role="finishing",
                    base_height=self.base_height,
                    tool=finish_tool,
                    material=self.material,
                    sequence_number=self.sequence_number + 1,
                )
                self.finish_op.tool_selection = finish_selection

    # ------------------------------------------------------------------

    def apply(self, shape):
        if self.tool.diameter <= 0 or self.tool.diameter > min(self.width, self.length):
            raise ValueError("The milling cutter does not fit the pocket opening.")
        result = rectangular_pocket_cut(
            shape,
            self.cx, self.cy,
            self.width, self.length,
            self.depth,
            max(self.corner_radius, self.tool.radius),
            face_selector=self.face_selector,
            base_height=self.base_height,
        )
        # Stock records and applies the separate finish operation after roughing.
        return result

    def to_dict(self) -> dict:
        d: dict = {
            "operation": "finish_pocket" if self.pass_role == "finishing" else "rough_pocket",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "dimensions": {
                "length": self.length,
                "width": self.width,
                "depth": self.depth,
            },
            "corner_radius_mm": self.corner_radius,
            "modeled_corner_radius_mm": max(self.corner_radius, self.tool.radius),
            "pass_role": self.pass_role,
            "verification_status": "not_verified",
            "geometry_status": "requires_review" if self.corner_radius == 0 else "modeled",
            "geometry_review_reasons": ["The milling cutter leaves rounded corners. The sharp inside corner needs another process or a design change."]
                if self.corner_radius == 0 else [],
            "base_height_mm": self.base_height,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "has_finish_pass": self.finish_op is not None,
            "notes": (
                "Finish pass available" if self.finish_op is not None
                else ""
            ),
        }
        return d

    def to_toolpath(self) -> Toolpath:
        stepover = _tool_stepover(self.tool, 0.5, 0.1)
        stepdown = _tool_stepdown(self.tool, 0.5, 0.5)
        half_w = max(self.width / 2.0 - self.tool.radius, 0.0)
        half_l = max(self.length / 2.0 - self.tool.radius, 0.0)
        tp = _new_toolpath(
            "finish_pocket" if self.pass_role == "finishing" else "rough_pocket",
            self.tool,
            self.feeds_speeds,
            metadata={
                "width_mm": self.width,
                "length_mm": self.length,
                "tool_selection": _tool_selection_dict(self.tool_selection),
                "following_finish_operation": self.finish_op is not None,
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        depth_levels = _depth_levels(self.depth, stepdown)
        max_lanes = 1
        for z in depth_levels:
            _move(tp, "plunge", self.cx, self.cy, z, feed=feed)
            inset = 0.0
            lane_count = 0
            while inset <= min(half_w, half_l) + 1e-9:
                lane_count += 1
                x0, x1 = self.cx - half_w + inset, self.cx + half_w - inset
                y0, y1 = self.cy - half_l + inset, self.cy + half_l - inset
                for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0), (x1, y0)):
                    _move(tp, "feed", x, y, z, feed=feed)
                inset += stepover
            max_lanes = max(max_lanes, lane_count)
            _move(tp, "feed", self.cx, self.cy, z, feed=feed)
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        pass_plan = _pass_plan(
            strategy="rectangular_pocket",
            tool=self.tool,
            depth_levels=depth_levels,
            lane_count=max_lanes,
            pass_type=self.pass_role,
            rest_machining=self.pass_role == "finishing",
            finish_passes=0,
        )
        if self.pass_role == "finishing":
            pass_plan["finishing_passes"] = pass_plan["roughing_passes"]
            pass_plan["roughing_passes"] = 0
        return _attach_pass_plan(tp, pass_plan)


@dataclass
class SlopedFloorOp(MachiningOperation):
    """Rectangular trough/pocket with an axis-aligned planar sloped floor."""

    cx: float = 0.0
    cy: float = 0.0
    width: float = 20.0
    length: float = 30.0
    start_depth: float = 1.0
    end_depth: float = 5.0
    slope_axis: str = "X"

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        self.slope_axis = str(self.slope_axis or "X").upper()
        if self.slope_axis == "LENGTH":
            self.slope_axis = "X"
        elif self.slope_axis == "WIDTH":
            self.slope_axis = "Y"
        if self.slope_axis not in {"X", "Y"}:
            raise ValueError("slope_axis must be 'X'/'length' or 'Y'/'width'")
        if self.width <= 0.0 or self.length <= 0.0:
            raise ValueError("sloped floor width and length must be positive")
        if self.start_depth < 0.0 or self.end_depth < 0.0:
            raise ValueError("sloped floor depths must be non-negative")

        if self.tool is None:
            max_tool_dia = max(min(self.width, self.length) * 0.8, 0.5)
            fallback = ToolSpec("flat_endmill", max_tool_dia)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    max_diameter_mm=max_tool_dia,
                    min_flute_length_mm=max(self.start_depth, self.end_depth),
                    feature="sloped_floor",
                    reason="largest available end mill that fits sloped rectangular trough",
                    prefer_largest=True,
                ),
                fallback,
            )

        self.feeds_speeds = get_feeds_speeds(
            _resolve_fs_tool_type(self.tool.tool_type), self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    @property
    def depth(self) -> float:
        return max(self.start_depth, self.end_depth)

    def apply(self, shape):
        return sloped_rectangular_cut(
            shape,
            self.cx,
            self.cy,
            self.width,
            self.length,
            self.start_depth,
            self.end_depth,
            slope_axis=self.slope_axis,
            face_selector=self.face_selector,
        )

    def to_dict(self) -> dict:
        return {
            "operation": "sloped_floor",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "start_depth_mm": self.start_depth,
            "end_depth_mm": self.end_depth,
            "slope_axis": self.slope_axis,
            "slope_depth_delta_mm": self.end_depth - self.start_depth,
            "dimensions": {
                "length": self.length,
                "width": self.width,
                "start_depth": self.start_depth,
                "end_depth": self.end_depth,
            },
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds if "error" not in self.feeds_speeds else None,
            "process": "mill",
            "coverage_family": "axis_aligned_sloped_prismatic_cut",
            "manufacturing_completeness": "pure_operation",
            "notes": "Planar sloped floor represented as a wedge cut",
        }

    def to_toolpath(self) -> Toolpath:
        stepover = _tool_stepover(self.tool, 0.45, 0.1)
        feed = _feed_rate(self.feeds_speeds)
        tp = _new_toolpath(
            "sloped_floor",
            self.tool,
            self.feeds_speeds,
            metadata={
                "width_mm": self.width,
                "length_mm": self.length,
                "start_depth_mm": self.start_depth,
                "end_depth_mm": self.end_depth,
                "slope_axis": self.slope_axis,
                "tool_selection": _tool_selection_dict(self.tool_selection),
            },
        )

        lanes = 0
        direction = 1
        if self.slope_axis == "X":
            x0, x1 = self.cx - self.length / 2.0, self.cx + self.length / 2.0
            y = self.cy - self.width / 2.0
            while y <= self.cy + self.width / 2.0 + 1e-9:
                lanes += 1
                start = (x0, y, -abs(self.start_depth))
                end = (x1, y, -abs(self.end_depth))
                a, b = (start, end) if direction > 0 else (end, start)
                _move(tp, "rapid", a[0], a[1], tp.safe_z)
                _move(tp, "plunge", a[0], a[1], a[2], feed=feed)
                _move(tp, "feed", b[0], b[1], b[2], feed=feed)
                y += stepover
                direction *= -1
        else:
            y0, y1 = self.cy - self.width / 2.0, self.cy + self.width / 2.0
            x = self.cx - self.length / 2.0
            while x <= self.cx + self.length / 2.0 + 1e-9:
                lanes += 1
                start = (x, y0, -abs(self.start_depth))
                end = (x, y1, -abs(self.end_depth))
                a, b = (start, end) if direction > 0 else (end, start)
                _move(tp, "rapid", a[0], a[1], tp.safe_z)
                _move(tp, "plunge", a[0], a[1], a[2], feed=feed)
                _move(tp, "feed", b[0], b[1], b[2], feed=feed)
                x += stepover
                direction *= -1

        if tp.moves:
            last_x, last_y, _ = tp.moves[-1].position
            _move(tp, "retract", last_x, last_y, tp.safe_z)
        return _attach_pass_plan(tp, _pass_plan(
            strategy="sloped_floor_raster",
            tool=self.tool,
            depth_levels=[-abs(self.start_depth), -abs(self.end_depth)],
            lane_count=max(lanes, 1),
            pass_type="finishing",
        ))


# ---------------------------------------------------------------------------
#  3. CircularPocketOp
# ---------------------------------------------------------------------------

@dataclass
class CircularPocketOp(MachiningOperation):
    """Circular pocket at a given position."""

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 20.0
    depth: float = 5.0
    base_height: Optional[float] = None

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            # Prefer a tool no larger than ~80% of the pocket diameter
            tool_dia = self.diameter * 0.8
            fallback = ToolSpec("flat_endmill", tool_dia)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    max_diameter_mm=tool_dia,
                    min_flute_length_mm=self.depth,
                    feature="circular_pocket",
                    reason="largest available end mill below circular pocket diameter",
                    prefer_largest=True,
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return circular_pocket_cut(shape, self.cx, self.cy, self.diameter, self.depth,
                                   face_selector=self.face_selector,
                                   base_height=self.base_height)

    def to_dict(self) -> dict:
        return {
            "operation": "circular_pocket",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "base_height_mm": self.base_height,
            "pocket_diameter_mm": self.diameter,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        stepdown = _tool_stepdown(self.tool, 0.5, 0.5)
        stepover = _tool_stepover(self.tool, 0.5, 0.1)
        radius = max(self.diameter / 2.0 - self.tool.radius, 0.0)
        tp = _new_toolpath(
            "circular_pocket",
            self.tool,
            self.feeds_speeds,
            metadata={
                "pocket_diameter_mm": self.diameter,
                "tool_selection": _tool_selection_dict(self.tool_selection),
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        depth_levels = _depth_levels(self.depth, stepdown)
        lane_count = max(1, math.ceil(radius / stepover)) if stepover else 1
        for z in depth_levels:
            _move(tp, "plunge", self.cx, self.cy, z, feed=feed)
            r = min(stepover, radius)
            while r <= radius + 1e-9:
                _move(tp, "feed", self.cx + r, self.cy, z, feed=feed)
                center = (self.cx, self.cy, z)
                for x, y in (
                    (self.cx, self.cy + r),
                    (self.cx - r, self.cy),
                    (self.cx, self.cy - r),
                    (self.cx + r, self.cy),
                ):
                    _move(tp, "arc", x, y, z, feed=feed, arc_center=center, arc_direction="ccw")
                r += stepover
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return _attach_pass_plan(tp, _pass_plan(
            strategy="circular_pocket",
            tool=self.tool,
            depth_levels=depth_levels,
            lane_count=lane_count,
            pass_type="roughing",
        ))


# ---------------------------------------------------------------------------
#  4. SpotDrillOp  (internal helper used by DrillOp)
# ---------------------------------------------------------------------------

@dataclass
class SpotDrillOp(MachiningOperation):
    """Spot-drilling operation -- creates a shallow conical start for a drill.

    This is normally created internally by ``DrillOp`` when
    ``spot_drill=True`` (the default).  Geometry is unchanged -- a spot
    drill is primarily a process-planning step.
    """

    cx: float = 0.0
    cy: float = 0.0
    hole_diameter: float = 6.0
    """Target hole diameter (the spot drill is sized slightly larger)."""

    material: str = "aluminum_6061"

    tool: Optional[ToolSpec] = None

    def __post_init__(self):
        if self.tool is None:
            # Pick a spot drill whose diameter covers the hole
            spot_dia = self.hole_diameter + 2.0  # slightly larger
            fallback = ToolSpec("drill", spot_dia, tip_angle=120.0)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "drill",
                    min_diameter_mm=spot_dia,
                    feature="spot_drill",
                    reason="spot drill diameter should cover the target hole",
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        # Spot-drill does not change the final hole geometry -- it is a
        # planning step that precedes the actual drill.
        return shape

    def to_dict(self) -> dict:
        return {
            "operation": "spot_drill",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": 1.0,  # spot-drill is shallow
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "Spot drill for hole alignment",
        }

    def to_toolpath(self) -> Toolpath:
        return _drill_toolpath(
            "spot_drill",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            1.0,
            dwell_seconds=0.2,
        )


# ---------------------------------------------------------------------------
#  5. DrillOp
# ---------------------------------------------------------------------------

@dataclass
class DrillOp(MachiningOperation):
    """Drilling operation -- creates a hole at a given position.

    When ``spot_drill=True`` (default) an internal ``SpotDrillOp`` is
    created and applied first (process-planning only -- the spot drill
    does not modify the B-Rep).

    When ``peck=True`` the feeds/speeds are adjusted for peck-drilling
    (reduced feed rate).
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 6.0
    depth: float = 10.0
    through: bool = False
    spot_drill: bool = True
    peck: bool = False

    tool: Optional[ToolSpec] = None
    """Drill bit.  Auto-selected to match hole diameter when None."""

    spot_op: Optional[SpotDrillOp] = field(default=None, repr=False)
    """Internal spot-drill operation created when spot_drill=True."""

    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            fallback = ToolSpec("drill", self.diameter, tip_angle=118.0)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "drill",
                    min_diameter_mm=self.diameter,
                    max_diameter_mm=self.diameter,
                    min_flute_length_mm=self.depth,
                    feature="drill",
                    reason="drill diameter must match requested hole diameter",
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )

        # Peck drilling adjusts feed rate downward
        if self.peck and "error" not in self.feeds_speeds:
            peck_fs = dict(self.feeds_speeds)
            orig = peck_fs.get("feed_rate_mm_min", 0)
            peck_fs["feed_rate_mm_min"] = orig * 0.5 if orig else 0
            peck_fs["peck_drilling"] = True
            self.feeds_speeds = peck_fs

        if self.spot_drill:
            self.spot_op = SpotDrillOp(
                cx=self.cx,
                cy=self.cy,
                hole_diameter=self.diameter,
                material=self.material,
            )

        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        if self.spot_op is not None:
            self.spot_op.face_selector = self.face_selector
            shape = self.spot_op.apply(shape)
        return drill_hole(shape, self.cx, self.cy, self.diameter, self.depth, self.through,
                          face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "drill",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "hole_diameter_mm": self.diameter,
            "through": self.through,
            "peck_drilling": self.peck,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "has_spot_drill": self.spot_op is not None,
            "notes": "Peck drilling" if self.peck else "",
        }

    def to_toolpath(self) -> Toolpath:
        tp = _drill_toolpath(
            "drill",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.depth,
            peck_step=(max(self.diameter, 1.0) if self.peck else None),
            metadata={"tool_selection": _tool_selection_dict(self.tool_selection)},
        )
        if self.spot_op is not None:
            tp.metadata["spot_toolpath"] = self.spot_op.to_toolpath().to_dict()
        return tp


# ---------------------------------------------------------------------------
#  6. TapOp
# ---------------------------------------------------------------------------

# Standard ISO metric coarse thread pitches (mm)
_METRIC_COARSE_PITCH: dict[float, float] = {
    1.0: 0.25, 1.2: 0.25, 1.6: 0.35,
    2.0: 0.4, 2.5: 0.45,
    3.0: 0.5, 4.0: 0.7, 5.0: 0.8,
    6.0: 1.0, 8.0: 1.25, 10.0: 1.5, 12.0: 1.75,
    14.0: 2.0, 16.0: 2.0, 18.0: 2.5, 20.0: 2.5,
    24.0: 3.0, 30.0: 3.5, 36.0: 4.0,
}


def _standard_pitch(diameter: float) -> float:
    """Return the standard ISO metric coarse pitch for *diameter*, or a
    rough estimate (diameter * 0.16) when the diameter is non-standard."""
    return _METRIC_COARSE_PITCH.get(diameter, round(diameter * 0.16, 2))


def _expected_tap_drill(diameter: float, pitch: float) -> float:
    """Standard metric tap-drill diameter = major_diameter - pitch."""
    return diameter - pitch


@dataclass
class TapOp(MachiningOperation):
    """Thread-tapping operation.

    Tapping does not appreciably change the B-Rep hole diameter; the
    operation is recorded in the process plan for downstream CAM.

    Validates that the *tap_drill_diameter* is within a reasonable range
    for the given thread diameter and pitch.
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 6.0
    """Thread major diameter (mm)."""

    pitch: float = 1.0
    """Thread pitch (mm).  Defaults to standard coarse for the diameter."""

    depth: float = 10.0
    """Tapping depth (mm)."""

    through: bool = False

    tap_drill_diameter: float = 5.0
    """Diameter of the hole to be tapped (should be ~diameter - pitch)."""

    tool: Optional[ToolSpec] = None
    """Tap tool.  Auto-selected from catalog by diameter."""

    material: str = "aluminum_6061"

    _validation_note: str = field(default="", repr=False)
    """Populated during __post_init__ with any tap-drill validation notes."""

    def __post_init__(self):
        # Auto-select pitch if not explicitly set (pitch=0 or default)
        if self.pitch <= 0:
            self.pitch = _standard_pitch(self.diameter)

        if self.tool is None:
            # Try named tap first, fall back to best thread_mill
            tap_name = f"TAP_M{int(self.diameter)}"
            try:
                self.tool = ToolCatalog.get(tap_name)
            except KeyError:
                try:
                    self.tool = ToolCatalog.get_best_for("thread_mill", self.diameter)
                except ValueError:
                    self.tool = ToolSpec("thread_mill", self.diameter)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )

        # Validate tap-drill diameter
        expected = _expected_tap_drill(self.diameter, self.pitch)
        tolerance = self.pitch * 0.3
        if abs(self.tap_drill_diameter - expected) > tolerance:
            self._validation_note = (
                f"Tap-drill {self.tap_drill_diameter:.2f} mm differs from "
                f"recommended {expected:.2f} mm (M{self.diameter:.1f}x{self.pitch})"
            )

        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        # Tapping does not change the B-Rep hole diameter appreciably
        return shape

    def to_dict(self) -> dict:
        return {
            "operation": "tap",
            "tool_type": self.tool.tool_type,
            "thread_diameter_mm": self.diameter,
            "thread_pitch_mm": self.pitch,
            "tap_drill_size_mm": self.tap_drill_diameter,
            "depth_mm": self.depth,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": self._validation_note,
        }

    def to_toolpath(self) -> list:
        return []


# ---------------------------------------------------------------------------
#  7. ChamferOp
# ---------------------------------------------------------------------------

@dataclass
class ChamferOp(MachiningOperation):
    """Edge chamfering -- bevels the top edges of the part.

    Auto-selects a 90-degree chamfer tool from the catalog.
    """

    width: float = 1.0
    """Chamfer width / depth (mm)."""

    tool: Optional[ToolSpec] = None
    """Chamfer tool.  Auto-selected (90-degree) when None."""

    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            fallback = ToolSpec("chamfer", 6.0, tip_angle=90.0, tip_diameter=1.0)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "chamfer",
                    min_diameter_mm=6.0,
                    feature="chamfer",
                    reason="standard 90 degree chamfer tool for edge break",
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = None  # applies to perimeter edges

    # ------------------------------------------------------------------

    def apply(self, shape):
        return chamfer_edges(shape, self.width, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "chamfer",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "chamfer_width_mm": self.width,
            "tool_tip_angle_deg": self.tool.tip_angle,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        span = max(self.tool.diameter * 8.0, 40.0)
        offset = self.width + self.tool.radius
        z = -abs(self.width)
        tp = _new_toolpath(
            "chamfer",
            self.tool,
            self.feeds_speeds,
            metadata={
                "nominal_square_mm": span,
                "chamfer_width_mm": self.width,
                "tool_selection": _tool_selection_dict(self.tool_selection),
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        x0, x1 = -span / 2.0 - offset, span / 2.0 + offset
        y0, y1 = -span / 2.0 - offset, span / 2.0 + offset
        _move(tp, "rapid", x0, y0, tp.safe_z)
        _move(tp, "plunge", x0, y0, z, feed=feed)
        for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
            _move(tp, "feed", x, y, z, feed=feed)
        _move(tp, "retract", x0, y0, tp.safe_z)
        return _attach_pass_plan(tp, _pass_plan(
            strategy="chamfer",
            tool=self.tool,
            depth_levels=[z],
            lane_count=4,
            pass_type="finishing",
        ))


# ---------------------------------------------------------------------------
#  8. ContourOp
# ---------------------------------------------------------------------------

@dataclass
class ContourOp(MachiningOperation):
    """Profile / contour milling -- trims the outer perimeter of the part.

    Supports multiple Z-level passes via the *stepdown* parameter.
    """

    depth: float = 5.0
    """Total depth of the contour cut (mm)."""

    stepdown: float = 1.0
    """Depth per Z-level pass.  Set to 0 or negative for single pass."""

    tool: Optional[ToolSpec] = None
    """End mill for contouring.  Auto-selected when None."""

    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            fallback = ToolSpec("flat_endmill", 10.0)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    min_diameter_mm=6.0,
                    min_flute_length_mm=self.depth,
                    feature="contour",
                    reason="rigid available end mill for profile contouring",
                    prefer_largest=True,
                ),
                fallback,
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = None  # contour follows the part perimeter

    # ------------------------------------------------------------------

    def apply(self, shape):
        return contour_cut_outer(shape, self.depth, self.stepdown,
                                 face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "contour",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "stepdown_mm": self.stepdown,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": f"Multi-pass (stepdown={self.stepdown} mm)" if self.stepdown > 0 else "",
        }

    def to_toolpath(self) -> Toolpath:
        span = max(self.tool.diameter * 8.0, 50.0)
        offset = self.tool.radius
        tp = _new_toolpath(
            "contour",
            self.tool,
            self.feeds_speeds,
            metadata={
                "nominal_square_mm": span,
                "offset_mm": offset,
                "tool_selection": _tool_selection_dict(self.tool_selection),
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        x0, x1 = -span / 2.0 - offset, span / 2.0 + offset
        y0, y1 = -span / 2.0 - offset, span / 2.0 + offset
        _move(tp, "rapid", x0, y0, tp.safe_z)
        depth_levels = _depth_levels(self.depth, self.stepdown)
        for z in depth_levels:
            _move(tp, "plunge", x0, y0, z, feed=feed)
            for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
                _move(tp, "feed", x, y, z, feed=feed)
        _move(tp, "retract", x0, y0, tp.safe_z)
        return _attach_pass_plan(tp, _pass_plan(
            strategy="contour",
            tool=self.tool,
            depth_levels=depth_levels,
            lane_count=1,
            pass_type="profiling",
        ))


# ---------------------------------------------------------------------------
#  9. SlotOp
# ---------------------------------------------------------------------------

@dataclass
class SlotOp(MachiningOperation):
    """Slot milling -- cuts an obround slot at a given position and angle.

    Validates that the selected tool diameter is not larger than the slot
    width.
    """

    cx: float = 0.0
    cy: float = 0.0
    length: float = 30.0
    width: float = 8.0
    depth: float = 5.0
    angle: float = 0.0
    """Rotation angle of the slot in degrees."""

    through: bool = False
    """If True, cut through entire part."""

    tool: Optional[ToolSpec] = None
    """End mill.  Auto-selected to fit within the slot width."""

    material: str = "aluminum_6061"

    _validation_note: str = field(default="", repr=False)

    def __post_init__(self):
        if self.tool is None:
            fallback = _pick_slot_tool(self.width)
            self.tool, self.tool_selection = _select_inventory_tool(
                ToolRequirement(
                    "flat_endmill",
                    max_diameter_mm=self.width,
                    min_flute_length_mm=self.depth,
                    feature="slot",
                    reason="largest available end mill that fits slot width",
                    prefer_largest=True,
                ),
                fallback,
            )

        # Validate: tool must not exceed slot width
        if self.tool.diameter > self.width:
            self._validation_note = (
                f"Tool diameter ({self.tool.diameter:.1f} mm) exceeds "
                f"slot width ({self.width:.1f} mm)"
            )

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return slot_cut(
            shape,
            self.cx, self.cy,
            self.length, self.width,
            self.depth,
            self.angle,
            self.through,
            face_selector=self.face_selector,
        )

    def to_dict(self) -> dict:
        return {
            "operation": "slot",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "slot_length_mm": self.length,
            "slot_width_mm": self.width,
            "slot_angle_deg": self.angle,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": self._validation_note,
        }

    def to_toolpath(self) -> Toolpath:
        return _linear_slot_toolpath(
            "slot",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.length,
            self.width,
            self.depth,
            self.angle,
            metadata={"tool_selection": _tool_selection_dict(self.tool_selection)},
        )


# =============================================================================
#  Phase 2 concrete operations
# =============================================================================

# ---------------------------------------------------------------------------
#  10. PeckDrillOp
# ---------------------------------------------------------------------------

@dataclass
class PeckDrillOp(MachiningOperation):
    """Deep-hole peck drilling -- like DrillOp but with peck always enabled.

    Uses the same ``drill_hole`` geometry as ``DrillOp``.  The feeds/speeds
    are adjusted with a reduced feed rate and a ``peck: True`` flag.
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 6.0
    depth: float = 20.0
    peck_step: float = 2.0
    """Retract step increment for peck drilling (mm)."""
    through: bool = False

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("drill", self.diameter)
            except ValueError:
                self.tool = ToolSpec("drill", self.diameter, tip_angle=118.0)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        if "error" not in self.feeds_speeds:
            self.feeds_speeds = dict(self.feeds_speeds)
            orig = self.feeds_speeds.get("feed_rate_mm_min", 0)
            self.feeds_speeds["feed_rate_mm_min"] = orig * 0.5 if orig else 0
            self.feeds_speeds["peck"] = True

        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return drill_hole(shape, self.cx, self.cy, self.diameter, self.depth,
                          self.through, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "peck_drill",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "hole_diameter_mm": self.diameter,
            "through": self.through,
            "peck_step_mm": self.peck_step,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": f"Peck drilling (step={self.peck_step}mm)",
        }

    def to_toolpath(self) -> Toolpath:
        return _drill_toolpath(
            "peck_drill",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.depth,
            peck_step=self.peck_step,
        )


# ---------------------------------------------------------------------------
#  11. ReamOp
# ---------------------------------------------------------------------------

@dataclass
class ReamOp(MachiningOperation):
    """Precision hole finishing with a reamer."""

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 6.0
    depth: float = 10.0
    through: bool = False

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            self.tool = _pick_reamer(self.diameter)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return drill_hole(shape, self.cx, self.cy, self.diameter, self.depth,
                          self.through, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "ream",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "hole_diameter_mm": self.diameter,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        return _drill_toolpath(
            "ream",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.depth,
            dwell_seconds=0.2,
        )


# ---------------------------------------------------------------------------
#  12. BoreOp
# ---------------------------------------------------------------------------

@dataclass
class BoreOp(MachiningOperation):
    """Boring operation for hole trueness and finish.

    Auto-selects a flat endmill approximately 90 % of the bore diameter.
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 20.0
    depth: float = 10.0
    through: bool = False

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            tool_dia = self.diameter * 0.9
            try:
                self.tool = ToolCatalog.get_best_for("flat_endmill", tool_dia)
            except ValueError:
                self.tool = ToolSpec("flat_endmill", tool_dia)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return drill_hole(shape, self.cx, self.cy, self.diameter, self.depth,
                          self.through, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "bore",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "hole_diameter_mm": self.diameter,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        stepdown = max(self.tool.diameter * 0.25, 0.5)
        radius = max(self.diameter / 2.0 - self.tool.radius, 0.0)
        tp = _new_toolpath(
            "bore",
            self.tool,
            self.feeds_speeds,
            metadata={"hole_diameter_mm": self.diameter},
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        for z in _depth_levels(self.depth, stepdown):
            _move(tp, "plunge", self.cx, self.cy, z, feed=feed)
            if radius > 0:
                _move(tp, "feed", self.cx + radius, self.cy, z, feed=feed)
                center = (self.cx, self.cy, z)
                for x, y in (
                    (self.cx, self.cy + radius),
                    (self.cx - radius, self.cy),
                    (self.cx, self.cy - radius),
                    (self.cx + radius, self.cy),
                ):
                    _move(tp, "arc", x, y, z, feed=feed, arc_center=center, arc_direction="ccw")
                _move(tp, "feed", self.cx, self.cy, z, feed=feed)
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return tp


# ---------------------------------------------------------------------------
#  13. CountersinkOp
# ---------------------------------------------------------------------------

@dataclass
class CountersinkOp(MachiningOperation):
    """Creates a conical countersink for flat-head screws.

    Uses *countersink_cut* which drills a through-hole and then applies a
    shallow circular pocket for the conical recess.
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 5.0
    """Through-hole diameter (mm)."""
    countersink_diameter: float = 10.0
    """Diameter of the countersink at the surface (mm)."""
    depth: float = 0
    """Depth of the countersink recess (mm).  0 = through-hole only."""
    countersink_angle: float = 82.0
    """Included angle of the countersink cone (degrees)."""

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("chamfer", 6.0)
            except ValueError:
                self.tool = ToolSpec("chamfer", 6.0, tip_angle=90.0, tip_diameter=1.0)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return countersink_cut(
            shape, self.cx, self.cy, self.diameter,
            self.countersink_diameter, self.depth,
            self.countersink_angle,
            face_selector=self.face_selector,
        )

    def to_dict(self) -> dict:
        return {
            "operation": "countersink",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "hole_diameter_mm": self.diameter,
            "countersink_diameter_mm": self.countersink_diameter,
            "countersink_angle_deg": self.countersink_angle,
            "depth_mm": self.depth,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        if self.depth > 0:
            sink_depth = self.depth
        else:
            angle_rad = math.radians(max(self.countersink_angle, 1.0) / 2.0)
            sink_depth = ((self.countersink_diameter - self.diameter) / 2.0) / math.tan(angle_rad)
            sink_depth = max(sink_depth, 0.0)

        tp = _new_toolpath(
            "countersink",
            self.tool,
            self.feeds_speeds,
            metadata={
                "hole_diameter_mm": self.diameter,
                "countersink_diameter_mm": self.countersink_diameter,
                "countersink_angle_deg": self.countersink_angle,
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        _move(tp, "plunge", self.cx, self.cy, -abs(sink_depth), feed=feed)
        _move(tp, "dwell", self.cx, self.cy, -abs(sink_depth), dwell_seconds=0.2)
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return tp


# ---------------------------------------------------------------------------
#  14. CounterboreOp
# ---------------------------------------------------------------------------

@dataclass
class CounterboreOp(MachiningOperation):
    """Creates a flat-bottom recess for socket-head cap screws.

    First drills the through-hole, then applies a circular pocket for the
    counterbore at shallow depth.
    """

    cx: float = 0.0
    cy: float = 0.0
    hole_diameter: float = 6.0
    """Diameter of the through-hole (mm)."""
    counterbore_diameter: float = 12.0
    """Diameter of the counterbore recess (mm)."""
    counterbore_depth: float = 6.0
    """Depth of the counterbore recess (mm)."""
    through: bool = False

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for(
                    "flat_endmill", self.counterbore_diameter * 0.8
                )
            except ValueError:
                self.tool = ToolSpec("flat_endmill", self.counterbore_diameter * 0.8)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        result = shape
        if self.through:
            result = drill_hole(
                result,
                self.cx,
                self.cy,
                self.hole_diameter,
                depth=0,
                through=True,
                face_selector=self.face_selector,
            )
        result = circular_pocket_cut(result, self.cx, self.cy,
                                     self.counterbore_diameter,
                                     self.counterbore_depth,
                                     face_selector=self.face_selector)
        return result

    def to_dict(self) -> dict:
        return {
            "operation": "counterbore",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "hole_diameter_mm": self.hole_diameter,
            "counterbore_diameter_mm": self.counterbore_diameter,
            "counterbore_depth_mm": self.counterbore_depth,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        stepdown = max(self.tool.diameter * 0.5, 0.5)
        stepover = max(self.tool.diameter * 0.5, 0.1)
        radius = max(self.counterbore_diameter / 2.0 - self.tool.radius, 0.0)
        tp = _new_toolpath(
            "counterbore",
            self.tool,
            self.feeds_speeds,
            metadata={
                "hole_diameter_mm": self.hole_diameter,
                "counterbore_diameter_mm": self.counterbore_diameter,
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        for z in _depth_levels(self.counterbore_depth, stepdown):
            _move(tp, "plunge", self.cx, self.cy, z, feed=feed)
            if radius > 0:
                r = min(stepover, radius)
                while r <= radius + 1e-9:
                    _move(tp, "feed", self.cx + r, self.cy, z, feed=feed)
                    center = (self.cx, self.cy, z)
                    for x, y in (
                        (self.cx, self.cy + r),
                        (self.cx - r, self.cy),
                        (self.cx, self.cy - r),
                        (self.cx + r, self.cy),
                    ):
                        _move(tp, "arc", x, y, z, feed=feed, arc_center=center, arc_direction="ccw")
                    r += stepover
            _move(tp, "feed", self.cx, self.cy, z, feed=feed)
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return tp


# ---------------------------------------------------------------------------
#  15. ThreadMillOp
# ---------------------------------------------------------------------------

@dataclass
class ThreadMillOp(MachiningOperation):
    """Thread milling using circular interpolation.

    Unlike ``TapOp`` which uses a tap tool, thread milling uses a thread-mill
    cutter that interpolates the thread form via helical motion.  Geometry is
    unchanged (thread form is too fine for B-Rep).
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 6.0
    """Thread major diameter (mm)."""
    pitch: float = 1.0
    """Thread pitch (mm).  Defaults to standard coarse for the diameter."""
    depth: float = 10.0
    """Thread depth (mm)."""
    through: bool = False

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.pitch <= 0:
            self.pitch = _standard_pitch(self.diameter)

        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("thread_mill", self.diameter)
            except ValueError:
                self.tool = ToolSpec("thread_mill", self.diameter)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        # Thread form is too fine for B-Rep CSG
        return shape

    def to_dict(self) -> dict:
        return {
            "operation": "thread_mill",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "thread_diameter_mm": self.diameter,
            "thread_pitch_mm": self.pitch,
            "depth_mm": self.depth,
            "through": self.through,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        radius = max(self.diameter / 2.0 - self.tool.radius, self.pitch * 0.25, 0.1)
        passes = max(1, math.ceil(abs(self.depth) / max(self.pitch, 0.1)))
        tp = _new_toolpath(
            "thread_mill",
            self.tool,
            self.feeds_speeds,
            metadata={
                "thread_diameter_mm": self.diameter,
                "thread_pitch_mm": self.pitch,
                "helix_passes": passes,
            },
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        _move(tp, "plunge", self.cx, self.cy, 0.0, feed=feed)
        _move(tp, "feed", self.cx + radius, self.cy, 0.0, feed=feed)
        for i in range(1, passes + 1):
            z = -min(i * self.pitch, abs(self.depth))
            center = (self.cx, self.cy, z)
            for x, y in (
                (self.cx, self.cy + radius),
                (self.cx - radius, self.cy),
                (self.cx, self.cy - radius),
                (self.cx + radius, self.cy),
            ):
                _move(tp, "arc", x, y, z, feed=feed, arc_center=center, arc_direction="ccw")
        _move(tp, "feed", self.cx, self.cy, -abs(self.depth), feed=feed)
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return tp


# ---------------------------------------------------------------------------
#  16. TSlotOp
# ---------------------------------------------------------------------------

@dataclass
class TSlotOp(MachiningOperation):
    """T-slot cutting (undercut slot wider at the bottom).

    Approximated as a rectangular slot -- full T-slot geometry requires a
    custom shape.  Auto-selects a dovetail / T-slot cutter.
    """

    cx: float = 0.0
    cy: float = 0.0
    length: float = 30.0
    width: float = 10.0
    depth: float = 8.0
    angle: float = 0.0

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("dovetail", self.width * 0.8)
            except ValueError:
                self.tool = ToolSpec("dovetail", self.width * 0.8)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return slot_cut(shape, self.cx, self.cy, self.length, self.width,
                        self.depth, self.angle, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "t_slot",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "slot_length_mm": self.length,
            "slot_width_mm": self.width,
            "slot_angle_deg": self.angle,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "T-slot approximated as rectangular slot",
        }

    def to_toolpath(self) -> Toolpath:
        return _linear_slot_toolpath(
            "t_slot",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.length,
            self.width,
            self.depth,
            self.angle,
            metadata={
                "form_tool": "t_slot",
                "approximation": "rectangular_slot",
            },
        )


# ---------------------------------------------------------------------------
#  17. DovetailOp
# ---------------------------------------------------------------------------

@dataclass
class DovetailOp(MachiningOperation):
    """Dovetail slot cutting.

    Approximated as a rectangular slot.  Auto-selects a dovetail cutter.
    """

    cx: float = 0.0
    cy: float = 0.0
    length: float = 30.0
    width: float = 10.0
    depth: float = 5.0
    angle: float = 45.0
    """Dovetail angle in degrees (default 45)."""

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("dovetail", self.width * 0.8)
            except ValueError:
                self.tool = ToolSpec("dovetail", self.width * 0.8)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return slot_cut(shape, self.cx, self.cy, self.length, self.width,
                        self.depth, self.angle, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "dovetail",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "slot_length_mm": self.length,
            "slot_width_mm": self.width,
            "dovetail_angle_deg": self.angle,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "Dovetail approximated as rectangular slot",
        }

    def to_toolpath(self) -> Toolpath:
        return _linear_slot_toolpath(
            "dovetail",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.length,
            self.width,
            self.depth,
            self.angle,
            metadata={
                "form_tool": "dovetail",
                "dovetail_angle_deg": self.angle,
                "approximation": "rectangular_slot",
            },
        )


# ---------------------------------------------------------------------------
#  18. GrooveOp
# ---------------------------------------------------------------------------

@dataclass
class GrooveOp(MachiningOperation):
    """Rectangular groove (open-ended slot).

    Like a slot but may extend to the part edge.  Auto-selects a flat endmill
    that fits within the groove width.
    """

    cx: float = 0.0
    cy: float = 0.0
    length: float = 30.0
    width: float = 8.0
    depth: float = 5.0
    angle: float = 0.0

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            self.tool = _pick_slot_tool(self.width)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return slot_cut(shape, self.cx, self.cy, self.length, self.width,
                        self.depth, self.angle, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "groove",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "groove_length_mm": self.length,
            "groove_width_mm": self.width,
            "groove_angle_deg": self.angle,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        return _linear_slot_toolpath(
            "groove",
            self.tool,
            self.feeds_speeds,
            self.cx,
            self.cy,
            self.length,
            self.width,
            self.depth,
            self.angle,
            metadata={"feature": "groove"},
        )


# ---------------------------------------------------------------------------
#  19. Surface3DOp
# ---------------------------------------------------------------------------

@dataclass
class Surface3DOp(MachiningOperation):
    """3D surface machining (parallel finishing).

    Auto-selects a ball endmill.  Geometry is unchanged for now -- 3D
    surface is too complex for simple CSG; this is a placeholder for
    Phase 3 tri-dexel surface machining.
    """

    stepover: float = 1.0
    """Stepover distance between parallel passes (mm)."""
    depth: float = 0.5
    """Depth of the surface finishing pass (mm)."""

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("ball_endmill", 6.0)
            except ValueError:
                self.tool = ToolSpec("ball_endmill", 6.0)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = None

    # ------------------------------------------------------------------

    def apply(self, shape):
        # 3D surface machining is a Phase 3 placeholder
        return shape

    def to_dict(self) -> dict:
        return {
            "operation": "surface_3d",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "stepover_mm": self.stepover,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "Placeholder for Phase 3 tri-dexel surface machining",
        }

    def to_toolpath(self) -> Toolpath:
        span = max(self.tool.diameter * 8.0, self.stepover * 8.0, 40.0)
        return _rect_raster_toolpath(
            "surface_3d",
            self.tool,
            self.feeds_speeds,
            width=span,
            length=span,
            depth=self.depth,
            stepover=self.stepover,
            metadata={
                "strategy": "parallel_finishing",
                "placeholder_geometry": True,
            },
        )


# ---------------------------------------------------------------------------
#  20. DeburrOp
# ---------------------------------------------------------------------------

@dataclass
class DeburrOp(MachiningOperation):
    """Deburring pass along part edges.

    Uses a very small chamfer (0.1 mm) as a proxy for deburring.
    Auto-selects a small chamfer tool.
    """

    edge_selection: str = "all"
    """Edge selection strategy (default: all top edges)."""
    tool_diameter: float = 3.0
    """Nominal tool diameter for the deburring tool (mm)."""

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("chamfer", self.tool_diameter)
            except ValueError:
                self.tool = ToolSpec("chamfer", self.tool_diameter,
                                     tip_angle=90.0, tip_diameter=0.5)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = None

    # ------------------------------------------------------------------

    def apply(self, shape):
        return chamfer_edges(shape, 0.1, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "deburr",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "edge_selection": self.edge_selection,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "Deburring pass (0.1 mm chamfer proxy)",
        }

    def to_toolpath(self) -> Toolpath:
        span = max(self.tool.diameter * 8.0, 40.0)
        return _rect_raster_toolpath(
            "deburr",
            self.tool,
            self.feeds_speeds,
            width=span,
            length=span,
            depth=0.1,
            stepover=max(self.tool.diameter, 1.0),
            metadata={
                "edge_selection": self.edge_selection,
                "proxy": "light_edge_break",
            },
        )


# ---------------------------------------------------------------------------
#  21. SpotFaceOp
# ---------------------------------------------------------------------------

@dataclass
class SpotFaceOp(MachiningOperation):
    """Spot-facing -- flat circular face for bolt head or washer.

    Uses ``circular_pocket_cut`` to create a shallow flat-bottom recess.
    """

    cx: float = 0.0
    cy: float = 0.0
    diameter: float = 20.0
    """Diameter of the spot-face (mm)."""
    depth: float = 0.5
    """Depth of the spot-face (mm)."""

    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("flat_endmill", self.diameter * 0.8)
            except ValueError:
                self.tool = ToolSpec("flat_endmill", self.diameter * 0.8)

        tool_type_fs = _resolve_fs_tool_type(self.tool.tool_type)
        self.feeds_speeds = get_feeds_speeds(
            tool_type_fs, self.tool.diameter, self.material
        )
        self.position = (self.cx, self.cy)

    # ------------------------------------------------------------------

    def apply(self, shape):
        return circular_pocket_cut(shape, self.cx, self.cy, self.diameter,
                                   self.depth, face_selector=self.face_selector)

    def to_dict(self) -> dict:
        return {
            "operation": "spot_face",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "depth_mm": self.depth,
            "spot_face_diameter_mm": self.diameter,
            "position": list(self.position) if self.position else None,
            "face_selector": self.face_selector,
            "feeds_speeds": self.feeds_speeds
            if "error" not in self.feeds_speeds
            else None,
            "notes": "",
        }

    def to_toolpath(self) -> Toolpath:
        stepdown = max(min(self.depth, self.tool.diameter * 0.5), 0.1)
        stepover = max(self.tool.diameter * 0.5, 0.1)
        radius = max(self.diameter / 2.0 - self.tool.radius, 0.0)
        tp = _new_toolpath(
            "spot_face",
            self.tool,
            self.feeds_speeds,
            metadata={"spot_face_diameter_mm": self.diameter},
        )
        feed = _feed_rate(self.feeds_speeds)
        _move(tp, "rapid", self.cx, self.cy, tp.safe_z)
        for z in _depth_levels(self.depth, stepdown):
            _move(tp, "plunge", self.cx, self.cy, z, feed=feed)
            r = min(stepover, radius)
            while r <= radius + 1e-9:
                _move(tp, "feed", self.cx + r, self.cy, z, feed=feed)
                center = (self.cx, self.cy, z)
                for x, y in (
                    (self.cx, self.cy + r),
                    (self.cx - r, self.cy),
                    (self.cx, self.cy - r),
                    (self.cx + r, self.cy),
                ):
                    _move(tp, "arc", x, y, z, feed=feed, arc_center=center, arc_direction="ccw")
                r += stepover
        _move(tp, "retract", self.cx, self.cy, tp.safe_z)
        return tp


# ---------------------------------------------------------------------------
#  Pure STEP coverage operations
# ---------------------------------------------------------------------------

@dataclass
class EdgeChamferOp(ChamferOp):
    """Selected-edge chamfer for pure STEP-to-SubCAD coverage."""

    selector: Any = "all"
    angle: float = 45.0

    def apply(self, shape):
        try:
            return chamfer_edges(
                shape,
                self.width,
                face_selector=self.face_selector,
                selector=self.selector,
            )
        except Exception:
            return shape

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "edge_chamfer",
            "selector": _profile_payload(self.selector),
            "angle_deg": self.angle,
            "process": "mill",
            "coverage_family": "selected_edge_treatment",
            "manufacturing_completeness": "pure_operation",
        })
        return data

    def to_toolpath(self) -> Toolpath:
        tp = super().to_toolpath()
        tp.operation = "edge_chamfer"
        tp.metadata.update({
            "selector": _profile_payload(self.selector),
            "angle_deg": self.angle,
            "process": "mill",
        })
        return tp


@dataclass
class EdgeFilletOp(MachiningOperation):
    """Selected-edge fillet represented as a tolerance finishing operation."""

    selector: Any = "all"
    radius: float = 1.0
    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            try:
                self.tool = ToolCatalog.get_best_for("ball_endmill", max(self.radius * 2.0, 2.0))
            except ValueError:
                self.tool = ToolSpec("ball_endmill", max(self.radius * 2.0, 2.0))
        self.feeds_speeds = get_feeds_speeds(
            _resolve_fs_tool_type(self.tool.tool_type), self.tool.diameter, self.material
        )

    def apply(self, shape):
        try:
            return fillet_edges(
                shape,
                self.radius,
                face_selector=self.face_selector,
                selector=self.selector,
            )
        except Exception:
            return shape

    def to_dict(self) -> dict:
        return {
            "operation": "edge_fillet",
            "selector": _profile_payload(self.selector),
            "radius_mm": self.radius,
            "process": "mill",
            "strategy": "ball_endmill_edge_blend",
            "target_tolerance_mm": 0.10,
            "achieved_tolerance_estimate_mm": 0.10,
            "sampling_resolution_mm": max(self.radius / 4.0, 0.05),
            "surface_strategy": "edge_blend",
            "verification_status": "requires_target_compare",
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "feeds_speeds": self.feeds_speeds if "error" not in self.feeds_speeds else None,
            "coverage_family": "selected_edge_treatment",
            "manufacturing_completeness": "pure_operation",
        }

    def to_toolpath(self) -> Toolpath:
        span = max(self.radius * 12.0, 20.0)
        return _rect_raster_toolpath(
            "edge_fillet",
            self.tool,
            self.feeds_speeds,
            width=span,
            length=span,
            depth=self.radius,
            stepover=max(self.radius / 2.0, 0.1),
            metadata={
                "selector": _profile_payload(self.selector),
                "process": "mill",
                "surface_strategy": "edge_blend",
                "target_tolerance_mm": 0.10,
            },
        )


@dataclass
class ProfilePocketOp(PocketOp):
    """Arbitrary profile pocket using profile bounds for current B-Rep cut."""

    profile: Any = field(default_factory=dict)
    islands: Optional[list[Any]] = None
    through: bool = False

    def __post_init__(self):
        if self.profile:
            width, length = _profile_bounds(self.profile)
            self.width = self.width or width
            self.length = self.length or length
        super().__post_init__()

    def apply(self, shape):
        return profile_pocket_cut(
            shape,
            self.profile or {"width": self.width, "length": self.length},
            self.depth,
            face_selector=self.face_selector,
            through=self.through,
            islands=self.islands,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "profile_pocket",
            "profile": _profile_payload(self.profile),
            "islands": _profile_payload(self.islands or []),
            "through": self.through,
            "process": "mill",
            "coverage_family": "arbitrary_profile_2_5d",
            "manufacturing_completeness": "pure_operation",
        })
        return data


@dataclass
class ProfileCutoutOp(ProfilePocketOp):
    """Arbitrary through/depth cutout operation."""

    through: bool = False

    def apply(self, shape):
        return profile_cutout_cut(
            shape,
            self.profile or {"width": self.width, "length": self.length},
            self.depth,
            face_selector=self.face_selector,
            through=self.through,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "profile_cutout",
            "through": self.through,
            "strategy": "profile_cutout",
        })
        return data


@dataclass
class ProfileContourOp(ContourOp):
    """Arbitrary profile contour operation."""

    profile: Any = field(default_factory=dict)
    side: str = "outside"

    def apply(self, shape):
        return profile_contour_cut(
            shape,
            self.profile,
            self.depth,
            side=self.side,
            face_selector=self.face_selector,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "profile_contour",
            "profile": _profile_payload(self.profile),
            "side": self.side,
            "process": "mill",
            "coverage_family": "arbitrary_profile_2_5d",
            "manufacturing_completeness": "pure_operation",
        })
        return data


@dataclass
class LaserCutInternalOp(MachiningOperation):
    """Laser-cut an internal through-profile in a sheet blank."""

    profile: Any = field(default_factory=dict)
    depth: float = 1.0
    kerf_width: float = 0.15
    assist_gas: str = "nitrogen"
    cut_speed_mm_min: float = 2400.0
    pierce_time_s: float = 0.15
    face_selector: str = ">Z"
    material: str = "steel_a36"
    operation_name: str = "laser_cut_internal"

    def __post_init__(self):
        self.tool = self.tool or _laser_tool(self.kerf_width)
        self.feeds_speeds = {
            "feed_rate_mm_min": float(self.cut_speed_mm_min),
            "feed_rate_mm_per_min": float(self.cut_speed_mm_min),
            "cut_speed_mm_min": float(self.cut_speed_mm_min),
            "pierce_time_s": float(self.pierce_time_s),
            "assist_gas": self.assist_gas,
        }
        self.position = None

    def apply(self, shape):
        return profile_pocket_cut(
            shape,
            self.profile,
            self.depth,
            face_selector=self.face_selector,
            through=True,
        )

    def to_dict(self) -> dict:
        width, length = _profile_bounds(self.profile)
        return {
            "operation": self.operation_name,
            "process": "laser_cut",
            "operation_family": "sheet_metal_cutting",
            "profile": _profile_payload(self.profile),
            "dimensions": {
                "length": length,
                "width": width,
                "depth": self.depth,
            },
            "depth_mm": self.depth,
            "through": True,
            "face_selector": self.face_selector,
            "kerf_width_mm": self.kerf_width,
            "assist_gas": self.assist_gas,
            "cut_speed_mm_min": self.cut_speed_mm_min,
            "pierce_time_s": self.pierce_time_s,
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "feeds_speeds": self.feeds_speeds,
            "coverage_family": "sheet_metal",
            "manufacturing_completeness": "pure_operation",
        }

    def to_toolpath(self) -> Toolpath:
        return _laser_profile_toolpath(
            self.operation_name,
            self.profile,
            self.tool,
            cut_speed_mm_min=self.cut_speed_mm_min,
            kerf_width=self.kerf_width,
            pierce_time_s=self.pierce_time_s,
        )


@dataclass
class LaserCutProfileOp(LaserCutInternalOp):
    """Laser-cut the outside blank profile and retain the requested outline."""

    operation_name: str = "laser_cut_profile"

    def apply(self, shape):
        return profile_cutout_cut(
            shape,
            self.profile,
            self.depth,
            face_selector=self.face_selector,
            through=True,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": self.operation_name,
            "strategy": "outside_profile_cut",
            "notes": "Retains the supplied closed profile as the flat sheet blank.",
        })
        return data


@dataclass
class PressBrakeBendOp(MachiningOperation):
    """Press-brake forming operation for a previously cut sheet blank."""

    axis: str = "X"
    bend_position: float = 0.0
    angle: float = 90.0
    side: str = "positive"
    radius: float = 1.0
    sheet_thickness: float = 1.0
    axis_z: Optional[float] = None
    tooling: str = "air_bend_v_die"
    material: str = "steel_a36"

    def __post_init__(self):
        self.tool = self.tool or ToolSpec(
            "press_brake_punch",
            max(float(self.radius) * 2.0, 1.0),
            material="tool_steel",
            catalog_id="PRESS_BRAKE_PUNCH",
        )
        self.feeds_speeds = {
            "bend_angle_deg": float(self.angle),
            "inside_radius_mm": float(self.radius),
            "sheet_thickness_mm": float(self.sheet_thickness),
            "tooling": self.tooling,
        }
        self.position = None

    def apply(self, shape):
        return apply_press_brake_bend(
            shape,
            axis=self.axis,
            position=self.bend_position,
            angle=self.angle,
            side=self.side,
            radius=self.radius,
            axis_z=self.axis_z,
        )

    def to_dict(self) -> dict:
        return {
            "operation": "press_brake_bend",
            "process": "press_brake",
            "operation_family": "sheet_metal_forming",
            "axis": str(self.axis).upper(),
            "bend_line_position_mm": self.bend_position,
            "side": self.side,
            "angle_deg": self.angle,
            "inside_radius_mm": self.radius,
            "sheet_thickness_mm": self.sheet_thickness,
            "axis_z_mm": self.axis_z,
            "tooling": self.tooling,
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "feeds_speeds": self.feeds_speeds,
            "coverage_family": "sheet_metal",
            "manufacturing_completeness": "pure_operation",
            "notes": "Geometry is an approximate split-and-rotate sheet bend; radius is recorded as manufacturing intent.",
        }

    def to_toolpath(self):
        return []


@dataclass
class MachineAroundProfileOp(ProfileCutoutOp):
    """Machine around retained profile/boss/pad material."""

    height: float = 5.0
    stock_envelope: Optional[dict] = None
    base_height: Optional[float] = None

    def apply(self, shape):
        return machine_around_profile_cut(
            shape,
            self.profile or {"width": self.width, "length": self.length},
            self.height,
            face_selector=self.face_selector,
            stock_envelope=self.stock_envelope,
            base_height=self.base_height,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "machine_around_profile",
            "height_mm": self.height,
            "stock_envelope": self.stock_envelope or {},
            "base_height_mm": self.base_height,
            "strategy": "machine_around_retained_island",
            "coverage_family": "retained_material",
        })
        return data


@dataclass
class MachineAroundProfilesOp(MachineAroundProfileOp):
    """Machine around multiple retained profile/boss/pad islands."""

    profiles: list[Any] = field(default_factory=list)

    def __post_init__(self):
        if not self.profiles and self.profile:
            self.profiles = list(self.profile) if isinstance(self.profile, list) else [self.profile]
        if self.profiles and not self.profile:
            self.profile = self.profiles
        saved_profile = self.profile
        self.profile = {}
        PocketOp.__post_init__(self)
        self.profile = saved_profile

    def apply(self, shape):
        return machine_around_profiles_cut(
            shape,
            self.profiles or [],
            self.height,
            face_selector=self.face_selector,
            stock_envelope=self.stock_envelope,
            base_height=self.base_height,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "machine_around_profiles",
            "profiles": _profile_payload(self.profiles),
            "island_count": len(self.profiles or []),
            "strategy": "machine_around_retained_islands",
        })
        return data


@dataclass
class MachineAroundCylinderOp(CircularPocketOp):
    """Machine around a retained cylindrical boss."""

    height: float = 5.0
    base_height: Optional[float] = None

    def apply(self, shape):
        return machine_around_cylinder_cut(
            shape,
            self.diameter,
            self.height,
            cx=self.cx,
            cy=self.cy,
            face_selector=self.face_selector,
            base_height=self.base_height,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "operation": "machine_around_cylinder",
            "height_mm": self.height,
            "base_height_mm": self.base_height,
            "strategy": "machine_around_retained_cylinder",
            "process": "mill",
            "coverage_family": "retained_material",
            "manufacturing_completeness": "pure_operation",
        })
        return data


@dataclass
class RibOp(MachineAroundProfileOp):
    """Retained rectangular rib."""

    angle: float = 0.0

    def __post_init__(self):
        self.profile = self.profile or {"width": self.width, "length": self.length, "type": "rib"}
        super().__post_init__()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"operation": "rib", "angle_deg": self.angle})
        return data


@dataclass
class PadOp(MachineAroundProfileOp):
    """Retained pad/boss profile."""

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"operation": "pad"})
        return data


@dataclass
class PureToleranceOp(MachiningOperation):
    """Base class for pure CNC tolerance-machined operation records."""

    operation_name: str = "tolerance_operation"
    process: str = "mill"
    profile: Any = field(default_factory=dict)
    path: Any = None
    tolerance_mm: float = 0.10
    strategy: str = "parallel"
    depth: float = 1.0
    tool: Optional[ToolSpec] = None
    material: str = "aluminum_6061"

    def __post_init__(self):
        if self.tool is None:
            tool_type = "ball_endmill" if self.process in {"4axis", "5axis", "mill_turn"} else "flat_endmill"
            try:
                self.tool = ToolCatalog.get_best_for(tool_type, 6.0)
            except ValueError:
                self.tool = ToolSpec(tool_type, 6.0)
        self.feeds_speeds = get_feeds_speeds(
            _resolve_fs_tool_type(self.tool.tool_type), self.tool.diameter, self.material
        )

    def apply(self, shape):
        return shape

    def to_dict(self) -> dict:
        return {
            "operation": self.operation_name,
            "process": self.process,
            "profile": _profile_payload(self.profile),
            "path": _profile_payload(self.path),
            "strategy": self.strategy,
            "target_tolerance_mm": self.tolerance_mm,
            "achieved_tolerance_estimate_mm": self.tolerance_mm,
            "sampling_resolution_mm": max(self.tolerance_mm / 2.0, 0.01),
            "surface_strategy": self.strategy,
            "verification_status": "requires_target_compare",
            "depth_mm": self.depth,
            "tool_type": self.tool.tool_type,
            "tool_diameter_mm": self.tool.diameter,
            "tool": _tool_dict(self.tool),
            "feeds_speeds": self.feeds_speeds if "error" not in self.feeds_speeds else None,
            "coverage_family": "pure_tolerance_machining",
            "manufacturing_completeness": "pure_operation",
        }

    def to_toolpath(self) -> Toolpath:
        span_w, span_l = _profile_bounds(self.profile or {"width": 40, "length": 40})
        return _rect_raster_toolpath(
            self.operation_name,
            self.tool,
            self.feeds_speeds,
            width=max(span_w, self.tool.diameter * 4),
            length=max(span_l, self.tool.diameter * 4),
            depth=max(self.depth, self.tolerance_mm),
            stepover=max(self.tool.diameter * 0.25, self.tolerance_mm),
            metadata={
                "process": self.process,
                "strategy": self.strategy,
                "target_tolerance_mm": self.tolerance_mm,
            },
        )


@dataclass
class TurningOp(PureToleranceOp):
    """Actual material geometry with an explicit, unreleased spindle path."""
    process: str = "turn"
    axis: str = "Z"
    origin: Any = (0, 0, 0)
    start_z: Optional[float] = None

    def __post_init__(self):
        self.axis, self.origin = turning_geometry.frame(self.axis, self.origin)
        self.requires_owned_tool = self.tool is None or not self.tool.catalog_id or self.tool.inventory_count <= 0
        if self.tool is None:
            self.tool = ToolSpec("turning_insert", 0.0, inventory_count=0)
        self.feeds_speeds = dict(self.tool.preferred_feeds_speeds or {})

    def _prepare(self, shape):
        local = turning_geometry.to_spindle(shape, self.axis, self.origin)
        self._incoming_bounds = local.val().BoundingBox()
        self._incoming_radius = turning_geometry.radial_extent(local)
        self._front = self._incoming_bounds.zmax if self.start_z is None else turning_geometry.finite(self.start_z, "start position")
        return local

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(axis=self.axis, spindle_origin_mm=list(self.origin), start_z_mm=self.start_z,
            achieved_tolerance_estimate_mm=None, verification_status="not_verified",
            geometry_status="modeled", geometry_fidelity="exact", manufacturing_completeness="requires_validation",
            toolpath_status="unverified", requires_owned_tool=self.requires_owned_tool,
            review_reasons=["The Turning toolpath needs feed, spindle, fixture, and tool verification."])
        if self.requires_owned_tool:
            data["review_reasons"].append("The Turning operation needs an owned tool assembly.")
        return data

    def _path(self, points: list[tuple[str, float, float]], **metadata) -> Toolpath:
        feed, rpm = self.feeds_speeds.get("feed_rate_mm_min"), self.feeds_speeds.get("rpm")
        if feed is not None:
            feed = turning_geometry.finite(feed, "feed", positive=True)
        if rpm is not None:
            rpm = turning_geometry.finite(rpm, "spindle speed", positive=True)
        if metadata.get("requires_spindle_phase") and rpm is not None:
            feed = turning_geometry.finite(metadata["pitch_mm"], "thread pitch", positive=True) * rpm
        path = Toolpath(self.operation_name, tool_type=self.tool.tool_type, tool_diameter_mm=self.tool.diameter,
            feed_mm_min=feed, spindle_rpm=rpm, metadata={"process": "turn", "spindle_axis": self.axis,
                "spindle_origin_mm": list(self.origin), "coordinate_system": "part_mm", "radius_mode": True,
                "verification_status": "not_verified", "requires_owned_tool": self.requires_owned_tool,
                "requires_feed_speed_release": feed is None or rpm is None,
                "tool_compensation": "not_applied", "roughing_passes": "not_generated", **metadata})
        for kind, radius, axial in points:
            path.append(ToolpathMove(kind, turning_geometry.spindle_point_to_world(radius, axial, self.axis, self.origin),
                feed=feed if kind == "feed" else None, spindle_rpm=rpm,
                metadata={"spindle_radius_mm": radius, "spindle_z_mm": axial}))
        return path


@dataclass
class TurnProfileOp(TurningOp):
    operation_name: str = "turn_profile"
    stock_diameter: Optional[float] = None

    def apply(self, shape):
        local = self._prepare(shape)
        self._contour, self._inner_contour = turning_geometry.profile_points(self.profile, local.val().BoundingBox())
        return turning_geometry.profile_cut(shape, self.profile, axis=self.axis, origin=self.origin)

    def to_toolpath(self) -> Toolpath:
        if not hasattr(self, "_contour"):
            raise ValueError("The Turning profile path needs the incoming material state.")
        contour = list(self._contour)
        if contour[-1][1] > contour[0][1]:
            contour.reverse()
        clearance = self._incoming_radius + 2
        points = [("rapid", clearance, contour[0][1] + 2), ("rapid", clearance, contour[0][1]),
            *(('feed', radius, axial) for radius, axial in contour), ("retract", clearance, contour[-1][1])]
        return self._path(points, strategy="outside_contour", inner_profile_requires_separate_boring=bool(self._inner_contour))

    def to_dict(self) -> dict:
        return {**super().to_dict(), "stock_diameter_mm": self.stock_diameter}


@dataclass
class TurnFaceOp(TurningOp):
    operation_name: str = "turn_face"
    z: float = 0.0
    side: str = "front"

    def apply(self, shape):
        self._prepare(shape)
        return turning_geometry.face_cut(shape, self.z, axis=self.axis, origin=self.origin, side=self.side)

    def to_toolpath(self) -> Toolpath:
        clearance = self._incoming_radius + 2
        outside = self._incoming_bounds.zmax + 2 if self.side == "front" else self._incoming_bounds.zmin - 2
        return self._path([("rapid", clearance, outside), ("rapid", clearance, self.z),
            ("feed", 0, self.z), ("retract", 0, outside)], strategy="facing")

    def to_dict(self) -> dict:
        return {**super().to_dict(), "z_mm": self.z, "side": self.side}


@dataclass
class TurnODOp(TurningOp):
    operation_name: str = "turn_od"
    diameter: float = 10.0
    length: float = 10.0

    def apply(self, shape):
        self._prepare(shape)
        return turning_geometry.diameter_cut(shape, self.diameter, self.length, axis=self.axis,
            origin=self.origin, start_z=self.start_z)

    def to_toolpath(self) -> Toolpath:
        radius, clearance = self.diameter / 2, self._incoming_radius + 2
        return self._path([("rapid", clearance, self._front + 2), ("rapid", radius, self._front + 2),
            ("feed", radius, self._front - self.length), ("retract", clearance, self._front - self.length)],
            strategy="outside_diameter")

    def to_dict(self) -> dict:
        return {**super().to_dict(), "diameter_mm": self.diameter, "length_mm": self.length}


@dataclass
class TurnIDOp(TurnODOp):
    operation_name: str = "turn_id"

    def apply(self, shape):
        self._prepare(shape)
        return turning_geometry.diameter_cut(shape, self.diameter, self.length, axis=self.axis,
            origin=self.origin, start_z=self.start_z, internal=True)

    def to_toolpath(self) -> Toolpath:
        radius = self.diameter / 2
        return self._path([("rapid", 0, self._front + 2), ("rapid", radius, self._front + 2),
            ("feed", radius, self._front - self.length), ("retract", 0, self._front - self.length),
            ("retract", 0, self._front + 2)], strategy="boring", requires_starting_bore_check=True)


@dataclass
class TurnGrooveOp(TurningOp):
    operation_name: str = "turn_groove"
    width: float = 2.0
    z: float = 0.0
    diameter: Optional[float] = None
    internal: bool = False

    def apply(self, shape):
        local = self._prepare(shape)
        self._groove_radius = self.diameter / 2 if self.diameter is not None else turning_geometry.section_radius(local, self.z)
        return turning_geometry.groove_cut(shape, self.width, self.depth, self.z, axis=self.axis,
            origin=self.origin, diameter=self.diameter, internal=self.internal)

    def to_toolpath(self) -> Toolpath:
        radius = self._groove_radius + self.depth if self.internal else self._groove_radius - self.depth
        clearance = max(0, self._groove_radius - 2) if self.internal else self._incoming_radius + 2
        return self._path([("rapid", clearance, self._front + 2), ("rapid", clearance, self.z),
            ("feed", radius, self.z), ("retract", clearance, self.z)], strategy="grooving",
            groove_width_mm=self.width, requires_groove_tool_width_check=True)

    def to_dict(self) -> dict:
        return {**super().to_dict(), "width_mm": self.width, "z_mm": self.z, "diameter_mm": self.diameter, "internal": self.internal}


@dataclass
class TurnThreadOp(TurningOp):
    operation_name: str = "turn_thread"
    diameter: float = 10.0
    pitch: float = 1.5
    length: float = 10.0
    internal: bool = False
    thread_depth: Optional[float] = None
    angle: float = 60.0
    left_handed: bool = False

    def apply(self, shape):
        self._prepare(shape)
        return turning_geometry.thread_cut(shape, self.diameter, self.pitch, self.length, axis=self.axis,
            origin=self.origin, start_z=self.start_z, internal=self.internal, depth=self.thread_depth,
            angle=self.angle, left_handed=self.left_handed)

    def to_toolpath(self) -> Toolpath:
        depth = self.thread_depth if self.thread_depth is not None else self.pitch / 2
        radius = self.diameter / 2 if self.internal else self.diameter / 2 - depth
        clearance = max(0, radius - depth - 2) if self.internal else self._incoming_radius + 2
        return self._path([("rapid", clearance, self._front + self.pitch), ("rapid", radius, self._front),
            ("feed", radius, self._front - self.length), ("retract", clearance, self._front - self.length)],
            strategy="synchronized_threading", pitch_mm=self.pitch, left_handed=self.left_handed, requires_spindle_phase=True)

    def to_dict(self) -> dict:
        return {**super().to_dict(), "diameter_mm": self.diameter, "pitch_mm": self.pitch, "length_mm": self.length,
            "internal": self.internal, "thread_depth_mm": self.thread_depth, "angle_deg": self.angle,
            "left_handed": self.left_handed, "geometry_fidelity": "approximate", "thread_profile": "generic_v",
            "thread_standard_verification": "not_run"}


@dataclass
class MillTurnSetupOp(PureToleranceOp):
    operation_name: str = "mill_turn_setup"
    process: str = "mill_turn"
    axis: str = "Z"
    work_offset: str = "G54"

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"axis": self.axis, "work_offset": self.work_offset})
        return data


@dataclass
class ThinWallPocketOp(PureToleranceOp):
    operation_name: str = "thin_wall_pocket"
    process: str = "mill"
    wall_thickness: float = 1.0
    open_faces: Optional[list[str]] = None

    def apply(self, shape):
        return thin_wall_pocket_cut(
            shape,
            self.profile,
            self.wall_thickness,
            self.depth,
            face_selector=self.face_selector,
        )

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"wall_thickness_mm": self.wall_thickness, "open_faces": self.open_faces or []})
        return data


@dataclass
class ThinWallShellOp(PureToleranceOp):
    operation_name: str = "thin_wall_shell"
    process: str = "mill"
    wall_thickness: float = 1.0

    def apply(self, shape):
        return thin_wall_shell_cut(shape, self.wall_thickness)

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "wall_thickness_mm": self.wall_thickness,
            "coverage_family": "thin_wall",
            "manufacturing_completeness": "pure_operation",
            "notes": "Offset shell of current machined shape",
        })
        return data


@dataclass
class HollowBoreOp(PureToleranceOp):
    operation_name: str = "hollow_bore"
    process: str = "mill_turn"
    inner_profile: Any = field(default_factory=dict)
    outer_profile: Any = field(default_factory=dict)
    access_face: str = ">Z"

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "inner_profile": _profile_payload(self.inner_profile),
            "outer_profile": _profile_payload(self.outer_profile),
            "access_face": self.access_face,
        })
        return data


@dataclass
class TubeProfileOp(HollowBoreOp):
    operation_name: str = "tube_profile"
    length: float = 10.0
    axis: str = "X"
    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0
    start: Optional[float] = None
    end: Optional[float] = None
    combine: str = "union"

    def apply(self, shape):
        if not isinstance(self.outer_profile, dict):
            return shape
        outer = (
            self.outer_profile.get("diameter")
            or self.outer_profile.get("diameter_mm")
            or self.outer_profile.get("outer_diameter")
            or self.outer_profile.get("outer_diameter_mm")
        )
        if outer is None:
            axis = self.outer_profile.get("axis", self.axis)
            start = self.outer_profile.get("start", self.outer_profile.get("start_mm", self.start))
            end = self.outer_profile.get("end", self.outer_profile.get("end_mm", self.end))
            tube = create_axis_aligned_profile_tube(
                self.outer_profile,
                self.inner_profile,
                self.length,
                axis=str(axis or "X"),
                cx=float(self.outer_profile.get("cx", self.cx)),
                cy=float(self.outer_profile.get("cy", self.cy)),
                cz=float(self.outer_profile.get("cz", self.outer_profile.get("z", self.cz))),
                start=start,
                end=end,
            )
            mode = str(self.combine or "union").lower()
            if mode == "replace":
                return tube
            if mode in {"intersect", "intersection"}:
                return shape.intersect(tube)
            return shape.union(tube)
        inner = 0.0
        if isinstance(self.inner_profile, dict):
            inner = (
                self.inner_profile.get("diameter")
                or self.inner_profile.get("diameter_mm")
                or self.inner_profile.get("inner_diameter")
                or self.inner_profile.get("inner_diameter_mm")
                or 0.0
            )
        axis = self.outer_profile.get("axis", self.axis)
        start = self.outer_profile.get("start", self.outer_profile.get("start_mm", self.start))
        end = self.outer_profile.get("end", self.outer_profile.get("end_mm", self.end))
        tube = create_axis_aligned_tube(
            float(outer),
            float(inner or 0.0),
            self.length,
            axis=str(axis or "X"),
            cx=float(self.outer_profile.get("cx", self.cx)),
            cy=float(self.outer_profile.get("cy", self.cy)),
            cz=float(self.outer_profile.get("cz", self.outer_profile.get("z", self.cz))),
            start=start,
            end=end,
        )
        mode = str(self.combine or "union").lower()
        if mode == "replace":
            return tube
        if mode in {"intersect", "intersection"}:
            return shape.intersect(tube)
        return shape.union(tube)

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "length_mm": self.length,
            "axis": self.axis,
            "position": [self.cx, self.cy, self.cz],
            "start_mm": self.start,
            "end_mm": self.end,
            "combine": self.combine,
            "process": self.process,
            "coverage_family": "thin_wall",
            "manufacturing_completeness": "pure_operation",
        })
        return data


@dataclass
class SurfaceMillOp(PureToleranceOp):
    operation_name: str = "surface_mill"
    process: str = "5axis"
    surface_ref: Any = field(default_factory=dict)

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"surface_ref": _profile_payload(self.surface_ref)})
        return data


@dataclass
class SweepMillOp(PureToleranceOp):
    operation_name: str = "sweep_mill"
    process: str = "5axis"


@dataclass
class LoftMillOp(PureToleranceOp):
    operation_name: str = "loft_mill"
    process: str = "5axis"
    profiles: Optional[list[Any]] = None

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"profiles": _profile_payload(self.profiles or [])})
        return data


@dataclass
class DomeMillOp(PureToleranceOp):
    operation_name: str = "dome_mill"
    process: str = "5axis"
    radius: float = 5.0
    height: float = 2.0
    cx: float = 0.0
    cy: float = 0.0

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({
            "radius_mm": self.radius,
            "height_mm": self.height,
            "position": [self.cx, self.cy],
        })
        return data


@dataclass
class WireCutProfileOp(PureToleranceOp):
    operation_name: str = "wire_cut_profile"
    process: str = "wire"
    thickness: float = 1.0
    taper: float = 0.0

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"thickness_mm": self.thickness, "taper_deg": self.taper})
        return data


@dataclass
class WireCutInternalOp(WireCutProfileOp):
    operation_name: str = "wire_cut_internal"
    start_hole: Any = None

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update({"start_hole": _profile_payload(self.start_hole)})
        return data


# =============================================================================
#  Convenience factory
# =============================================================================

_OP_REGISTRY: dict[str, type[MachiningOperation]] = {
    "face_mill":        FaceMillOp,
    "pocket":           PocketOp,
    "sloped_floor":     SlopedFloorOp,
    "slope_cut":        SlopedFloorOp,
    "circular_pocket":  CircularPocketOp,
    "drill":            DrillOp,
    "tap":              TapOp,
    "spot_drill":       SpotDrillOp,
    "chamfer":          ChamferOp,
    "contour":          ContourOp,
    "slot":             SlotOp,
    "peck_drill":       PeckDrillOp,
    "ream":             ReamOp,
    "bore":             BoreOp,
    "countersink":      CountersinkOp,
    "counterbore":      CounterboreOp,
    "thread_mill":      ThreadMillOp,
    "t_slot":           TSlotOp,
    "dovetail":         DovetailOp,
    "groove":           GrooveOp,
    "surface_3d":       Surface3DOp,
    "deburr":           DeburrOp,
    "spot_face":        SpotFaceOp,
    "edge_chamfer":     EdgeChamferOp,
    "edge_fillet":      EdgeFilletOp,
    "profile_pocket":   ProfilePocketOp,
    "profile_cutout":   ProfileCutoutOp,
    "profile_contour":  ProfileContourOp,
    "laser_cut_profile": LaserCutProfileOp,
    "laser_cut_internal": LaserCutInternalOp,
    "press_brake_bend": PressBrakeBendOp,
    "machine_around_profile": MachineAroundProfileOp,
    "machine_around_profiles": MachineAroundProfilesOp,
    "machine_around_cylinder": MachineAroundCylinderOp,
    "rib":              RibOp,
    "pad":              PadOp,
    "turn_profile":     TurnProfileOp,
    "turn_face":        TurnFaceOp,
    "turn_od":          TurnODOp,
    "turn_id":          TurnIDOp,
    "turn_groove":      TurnGrooveOp,
    "turn_thread":      TurnThreadOp,
    "mill_turn_setup":  MillTurnSetupOp,
    "thin_wall_pocket": ThinWallPocketOp,
    "thin_wall_shell":  ThinWallShellOp,
    "shell_wall":       ThinWallShellOp,
    "hollow_bore":      HollowBoreOp,
    "tube_profile":     TubeProfileOp,
    "surface_mill":     SurfaceMillOp,
    "sweep_mill":       SweepMillOp,
    "loft_mill":        LoftMillOp,
    "dome_mill":        DomeMillOp,
    "wire_cut_profile": WireCutProfileOp,
    "wire_cut_internal": WireCutInternalOp,
}


def create_operation(op_type: str, **kwargs) -> MachiningOperation:
    """Factory to create a MachiningOperation by name.

    Args:
        op_type: One of 'face_mill', 'pocket', 'circular_pocket', 'drill',
                 'tap', 'spot_drill', 'chamfer', 'contour', 'slot'.
        **kwargs: Forwarded to the operation class constructor.

    Returns:
        A concrete ``MachiningOperation`` instance.

    Raises:
        ValueError: If *op_type* is not a recognised operation name.
    """
    cls = _OP_REGISTRY.get(op_type)
    if cls is None:
        raise ValueError(
            f"Unknown operation type {op_type!r}. "
            f"Available: {sorted(_OP_REGISTRY.keys())}"
        )
    return cls(**kwargs)


# =============================================================================
#  Auto-operations helper
# =============================================================================

def threaded_hole_ops(
    diameter: float,
    depth: float,
    position: Tuple[float, float],
    material: str = "aluminum_6061",
    through: bool = False,
    face_selector: str = ">Z",
) -> list[MachiningOperation]:
    """Generate the full sequence for a threaded hole.

    Produces three operations in order:
        1. ``SpotDrillOp``  -- spot-drill for alignment
        2. ``DrillOp``      -- drill the tap-drill hole
        3. ``TapOp``        -- tap the threads

    The tap-drill diameter is computed using the standard ISO metric
    coarse formula: tap_drill = major_diameter - pitch.

    Args:
        diameter: Thread major diameter (mm), e.g. 6.0 for M6.
        depth: Hole / tap depth (mm).
        position: (cx, cy) on the top workplane.
        material: Workpiece material key.
        through: If True, drill and tap through the entire part.
        face_selector: CadQuery face selector (e.g. \">Z\", \"<Z\").

    Returns:
        A list of 3 ``MachiningOperation`` instances: [spot, drill, tap].
    """
    cx, cy = position
    pitch = _standard_pitch(diameter)
    tap_drill = _expected_tap_drill(diameter, pitch)

    spot = SpotDrillOp(
        cx=cx,
        cy=cy,
        hole_diameter=tap_drill,
        material=material,
        face_selector=face_selector,
    )

    drill = DrillOp(
        cx=cx,
        cy=cy,
        diameter=tap_drill,
        depth=depth,
        through=through,
        spot_drill=False,  # we already create a separate spot op
        material=material,
        face_selector=face_selector,
    )

    tap = TapOp(
        cx=cx,
        cy=cy,
        diameter=diameter,
        pitch=pitch,
        depth=depth,
        through=through,
        tap_drill_diameter=tap_drill,
        material=material,
        face_selector=face_selector,
    )

    return [spot, drill, tap]


# =============================================================================
#  Self-test
# =============================================================================

if __name__ == "__main__":
    import sys

    # ------------------------------------------------------------------
    # CadQuery guard
    # ------------------------------------------------------------------
    try:
        import cadquery as cq
        _HAS_CQ = True
    except ImportError:
        _HAS_CQ = False

    from .geometry import (
        create_rectangular_stock,
        compute_bounding_box,
        compute_volume,
    )
    from .tool_library import ToolCatalog

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

    # Ensure catalog is initialised
    ToolCatalog._ensure_initialised()

    print("=" * 60)
    print("SubCAD operations self-tests")
    print("=" * 60)

    # ==================================================================
    # Test 1: Each operation class instantiates with defaults
    # ==================================================================
    print("\n1. Instantiation with defaults ...")

    ops_default: list[MachiningOperation] = [
        FaceMillOp(),
        PocketOp(),
        CircularPocketOp(),
        DrillOp(),
        TapOp(),
        ChamferOp(),
        ContourOp(),
        SlotOp(),
    ]
    names = [
        "FaceMillOp", "PocketOp", "CircularPocketOp",
        "DrillOp", "TapOp", "ChamferOp", "ContourOp", "SlotOp",
    ]
    for op, name in zip(ops_default, names):
        check(isinstance(op, MachiningOperation), f"{name} is MachiningOperation")
        check(op.tool is not None, f"{name} auto-tool populated")
        check(isinstance(op.feeds_speeds, dict), f"{name} feeds_speeds is dict")

    # ==================================================================
    # Test 1b: DrillOp auto-creates spot_op when spot_drill=True
    # ==================================================================
    print("\n1b. DrillOp spot_drill ...")
    d_op = DrillOp(spot_drill=True)
    check(d_op.spot_op is not None, "DrillOp creates SpotDrillOp when spot_drill=True")
    check(isinstance(d_op.spot_op, SpotDrillOp), "spot_op is SpotDrillOp instance")

    d_op2 = DrillOp(spot_drill=False)
    check(d_op2.spot_op is None, "DrillOp without spot_drill has no spot_op")

    # ==================================================================
    # Test 1c: DrillOp peck drilling adjusts feeds
    # ==================================================================
    print("\n1c. DrillOp peck drilling ...")
    d_peck = DrillOp(peck=True)
    check("peck_drilling" in d_peck.feeds_speeds,
          "peck=True sets peck_drilling flag")
    check(d_peck.feeds_speeds.get("peck_drilling") is True,
          "peck_drilling value is True")

    # ==================================================================
    # Test 2: to_dict() returns correct structure
    # ==================================================================
    print("\n2. to_dict() structure ...")

    op_dicts: list[tuple[str, dict]] = [
        ("FaceMillOp", FaceMillOp().to_dict()),
        ("PocketOp", PocketOp().to_dict()),
        ("CircularPocketOp", CircularPocketOp().to_dict()),
        ("DrillOp", DrillOp().to_dict()),
        ("TapOp", TapOp().to_dict()),
        ("ChamferOp", ChamferOp().to_dict()),
        ("ContourOp", ContourOp().to_dict()),
        ("SlotOp", SlotOp().to_dict()),
        ("SpotDrillOp", SpotDrillOp().to_dict()),
    ]

    # Expected keys per operation type
    expected_keys = {
        "FaceMillOp":       ["operation", "tool_type", "tool_diameter_mm", "depth_mm", "feeds_speeds"],
        "PocketOp":         ["operation", "tool_type", "tool_diameter_mm", "depth_mm", "dimensions", "position"],
        "CircularPocketOp": ["operation", "tool_type", "tool_diameter_mm", "depth_mm", "pocket_diameter_mm"],
        "DrillOp":          ["operation", "tool_type", "tool_diameter_mm", "depth_mm", "hole_diameter_mm", "through"],
        "TapOp":            ["operation", "tool_type", "thread_diameter_mm", "thread_pitch_mm", "tap_drill_size_mm"],
        "ChamferOp":        ["operation", "tool_type", "chamfer_width_mm"],
        "ContourOp":        ["operation", "tool_type", "depth_mm", "stepdown_mm"],
        "SlotOp":           ["operation", "tool_type", "slot_length_mm", "slot_width_mm"],
        "SpotDrillOp":      ["operation", "tool_type", "depth_mm"],
    }

    for name, d in op_dicts:
        check(isinstance(d, dict), f"{name}.to_dict returns dict")
        for key in expected_keys.get(name, []):
            check(key in d, f"{name}.to_dict has key {key!r}")

    # ==================================================================
    # Test 3: Auto-tool selection works
    # ==================================================================
    print("\n3. Auto-tool selection ...")

    # FaceMillOp should pick a flat endmill
    fm = FaceMillOp()
    check(fm.tool.tool_type == "flat_endmill",
          f"FaceMillOp tool_type = {fm.tool.tool_type!r}")
    check(fm.tool.diameter >= 10.0,
          f"FaceMillOp diameter >= 10 mm (got {fm.tool.diameter})")

    # PocketOp should pick ~80% of pocket width
    p = PocketOp(width=20.0, length=30.0)
    expected_tool = 20.0 * 0.8  # 16.0
    check(p.tool.diameter >= expected_tool,
          f"PocketOp tool >= {expected_tool} mm (got {p.tool.diameter})")

    # DrillOp should match hole diameter
    d = DrillOp(diameter=8.0)
    check(d.tool.tool_type == "drill",
          f"DrillOp tool_type = {d.tool.tool_type!r}")
    check(d.tool.diameter >= 8.0,
          f"DrillOp diameter >= 8.0 mm (got {d.tool.diameter})")

    # TapOp should pick thread_mill
    t = TapOp(diameter=6.0)
    check(t.tool.tool_type == "thread_mill",
          f"TapOp tool_type = {t.tool.tool_type!r}")

    # ChamferOp should pick chamfer tool
    ch = ChamferOp()
    check(ch.tool.tool_type == "chamfer",
          f"ChamferOp tool_type = {ch.tool.tool_type!r}")

    # SlotOp should fit within slot width
    s = SlotOp(width=8.0)
    check(s.tool.diameter <= s.width,
          f"SlotOp tool diameter ({s.tool.diameter}) <= slot width ({s.width})")

    # ==================================================================
    # Test 4: create_operation factory
    # ==================================================================
    print("\n4. create_operation factory ...")

    for op_type in ["face_mill", "pocket", "circular_pocket", "drill",
                    "tap", "chamfer", "contour", "slot"]:
        try:
            op = create_operation(op_type)
            check(isinstance(op, MachiningOperation),
                  f"create_operation({op_type!r}) returns MachiningOperation")
        except Exception as exc:
            check(False, f"create_operation({op_type!r}) raised {exc}")

    # Unknown op_type should raise
    try:
        create_operation("laser_blast")
        check(False, "create_operation('laser_blast') should raise")
    except ValueError:
        check(True, "create_operation('laser_blast') raises ValueError")
    except Exception as exc:
        check(False, f"create_operation('laser_blast') wrong exception: {exc}")

    # ==================================================================
    # Test 5: threaded_hole_ops returns 3 operations
    # ==================================================================
    print("\n5. threaded_hole_ops ...")

    seq = threaded_hole_ops(diameter=6.0, depth=12.0, position=(10, 20),
                             material="aluminum_6061")
    check(len(seq) == 3,
          f"threaded_hole_ops returns 3 ops (got {len(seq)})")
    check(isinstance(seq[0], SpotDrillOp),
          f"seq[0] is SpotDrillOp (got {type(seq[0]).__name__})")
    check(isinstance(seq[1], DrillOp),
          f"seq[1] is DrillOp (got {type(seq[1]).__name__})")
    check(isinstance(seq[2], TapOp),
          f"seq[2] is TapOp (got {type(seq[2]).__name__})")

    # Spot drill should NOT have spot_drill=True (avoid infinite chain)
    d_op3 = seq[1]
    check(d_op3.spot_op is None,
          "threaded_hole_ops DrillOp has no spot_op (spot created separately)")

    # Tap should have correct drill size
    tap_op = seq[2]
    expected_td = 6.0 - 1.0  # M6 pitch = 1.0
    check(abs(tap_op.tap_drill_diameter - expected_td) < 0.01,
          f"tap_drill_diameter = {tap_op.tap_drill_diameter} "
          f"(expected {expected_td})")

    # ==================================================================
    # Test 6: apply() modifies geometry (requires CadQuery)
    # ==================================================================
    if _HAS_CQ:
        print("\n6. apply() modifies geometry ...")

        # 6a. FaceMillOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            fm_op = FaceMillOp(depth=3.0)
            result = fm_op.apply(stock)
            bb = compute_bounding_box(result)
            check(abs(bb["height"] - 17) < 0.5,
                  f"FaceMillOp reduces height to ~17 (got {bb['height']:.2f})")
        except Exception as exc:
            check(False, f"FaceMillOp.apply crashed: {exc}")

        # 6b. DrillOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            d_op6 = DrillOp(cx=0, cy=0, diameter=10.0, depth=0, through=True,
                            spot_drill=False)
            result = d_op6.apply(stock)
            vol_after = compute_volume(result)
            check(vol_after < compute_volume(stock),
                  f"DrillOp reduces volume")
        except Exception as exc:
            check(False, f"DrillOp.apply crashed: {exc}")

        # 6c. PocketOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            p_op = PocketOp(cx=0, cy=0, width=30, length=40, depth=5)
            result = p_op.apply(stock)
            vol_after = compute_volume(result)
            vol_expected = 100 * 60 * 20 - 30 * 40 * 5
            check(abs(vol_after - vol_expected) < 5.0,
                  f"PocketOp volume ~ {vol_expected} (got {vol_after:.1f})")
        except Exception as exc:
            check(False, f"PocketOp.apply crashed: {exc}")

        # 6d. ChamferOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            ch_op = ChamferOp(width=2.0)
            result = ch_op.apply(stock)
            vol_after = compute_volume(result)
            check(vol_after < compute_volume(stock),
                  "ChamferOp reduces volume")
        except Exception as exc:
            check(False, f"ChamferOp.apply crashed: {exc}")

        # 6e. TapOp geometry unchanged
        try:
            stock = create_rectangular_stock(100, 60, 20)
            t_op = TapOp()
            result = t_op.apply(stock)
            vol_before = compute_volume(stock)
            vol_after = compute_volume(result)
            check(abs(vol_after - vol_before) < 0.1,
                  f"TapOp does not change volume ({vol_before:.1f} vs {vol_after:.1f})")
        except Exception as exc:
            check(False, f"TapOp.apply crashed: {exc}")

        # 6f. SlotOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            s_op = SlotOp(cx=10, cy=0, length=30, width=8, depth=10)
            result = s_op.apply(stock)
            vol_after = compute_volume(result)
            check(vol_after < compute_volume(stock),
                  "SlotOp reduces volume")
        except Exception as exc:
            check(False, f"SlotOp.apply crashed: {exc}")

        # 6g. ContourOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            co_op = ContourOp(depth=5, stepdown=2.0)
            result = co_op.apply(stock)
            vol_after = compute_volume(result)
            check(vol_after < compute_volume(stock),
                  "ContourOp reduces volume")
        except Exception as exc:
            check(False, f"ContourOp.apply crashed: {exc}")

        # 6h. CircularPocketOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            cp_op = CircularPocketOp(cx=0, cy=0, diameter=30, depth=5)
            result = cp_op.apply(stock)
            vol_after = compute_volume(result)
            check(vol_after < compute_volume(stock),
                  "CircularPocketOp reduces volume")
        except Exception as exc:
            check(False, f"CircularPocketOp.apply crashed: {exc}")
    else:
        print("\n6. apply() geometry tests SKIPPED (CadQuery not available)")

    # ==================================================================
    # Test 7: PocketOp auto-finish
    # ==================================================================
    print("\n7. PocketOp auto-finish ...")
    # Corner radius >= tool radius -> no finish needed
    # For width=50 the tool is ~50 mm (radius ~25), so corner_radius=30 avoids finish
    p_no_finish = PocketOp(width=50, length=60, corner_radius=30.0)
    check(p_no_finish.finish_op is None,
          "No finish when corner_radius >= tool radius (got "
          f"tool_radius={p_no_finish.tool.radius:.1f}, corner_radius=30.0)")

    # Tight corner, large tool -> finish needed
    p_finish = PocketOp(width=20, length=30, corner_radius=1.0)
    # tool is ~16mm dia (80% of 20) -> radius 8 > corner 1
    if p_finish.tool.radius > 1.0:
        check(p_finish.finish_op is not None,
              "Finish op created when tool radius > corner_radius")
    else:
        # Tool happened to be smaller -- still a valid path
        check(True, "Tool happens to be small enough (no finish needed)")

    # ==================================================================
    # Test 8: TapOp validation note
    # ==================================================================
    print("\n8. TapOp validation ...")
    # Correct tap-drill: M6 coarse -> 5.0
    t_ok = TapOp(diameter=6.0, pitch=1.0, tap_drill_diameter=5.0)
    check(t_ok._validation_note == "",
          "Correct tap-drill yields empty validation note")

    # Wrong tap-drill
    t_bad = TapOp(diameter=6.0, pitch=1.0, tap_drill_diameter=3.0)
    check(t_bad._validation_note != "",
          "Incorrect tap-drill produces validation note")
    check("differs from recommended" in t_bad._validation_note,
          "Validation note mentions the discrepancy")

    # ==================================================================
    # Test 9: SlotOp width validation
    # ==================================================================
    print("\n9. SlotOp width validation ...")
    s_ok = SlotOp(width=20.0)
    check(s_ok._validation_note == "",
          "Tool fits in slot -> no validation note")

    s_narrow = SlotOp(width=2.0)
    if s_narrow.tool.diameter > 2.0:
        check(s_narrow._validation_note != "",
              "Tool too wide -> validation note populated")
    else:
        check(True, "Tool fits (unexpected but valid)")

    # ==================================================================
    # Test 10: core to_toolpath implementations
    # ==================================================================
    print("\n10. core to_toolpath implementations ...")
    core_toolpath_ops = (
        FaceMillOp(),
        PocketOp(),
        CircularPocketOp(),
        DrillOp(spot_drill=False),
        ChamferOp(),
        ContourOp(),
        SlotOp(),
    )
    for op in core_toolpath_ops:
        tp = op.to_toolpath()
        check(hasattr(tp, "to_dict"),
              f"{op.__class__.__name__}.to_toolpath returns Toolpath-like object")
        check(len(tp) > 0,
              f"{op.__class__.__name__}.to_toolpath is non-empty")

    tp = TapOp().to_toolpath()
    check(isinstance(tp, list), "TapOp.to_toolpath returns list")
    check(len(tp) == 0, "TapOp.to_toolpath remains empty")

    # ==================================================================
    # Test 11: Position tracking
    # ==================================================================
    print("\n11. Position tracking ...")
    check(FaceMillOp().position is None, "FaceMillOp position is None")
    check(ChamferOp().position is None, "ChamferOp position is None")
    check(ContourOp().position is None, "ContourOp position is None")

    d_pos = DrillOp(cx=12.5, cy=25.0)
    check(d_pos.position == (12.5, 25.0),
          f"DrillOp position = {d_pos.position}")

    p_pos = PocketOp(cx=-5.0, cy=10.0)
    check(p_pos.position == (-5.0, 10.0),
          f"PocketOp position = {p_pos.position}")

    s_pos = SlotOp(cx=30.0, cy=-15.0)
    check(s_pos.position == (30.0, -15.0),
          f"SlotOp position = {s_pos.position}")

    # ==================================================================
    # Test 12: face_selector in operations
    # ==================================================================
    print("\n12. face_selector ...")
    check(FaceMillOp().face_selector == ">Z", "default face_selector is >Z")
    check(FaceMillOp(face_selector="<Z").to_dict()["face_selector"] == "<Z",
          "to_dict includes face_selector")
    check(DrillOp(face_selector=">X").to_dict()["face_selector"] == ">X",
          "DrillOp face_selector in dict")
    check(ChamferOp(face_selector="<Z").to_dict()["face_selector"] == "<Z",
          "ChamferOp face_selector in dict")

    # threaded_hole_ops with face_selector
    seq = threaded_hole_ops(diameter=6, depth=10, position=(0, 0),
                            face_selector="<Z")
    for op in seq:
        check(op.face_selector == "<Z",
              f"{op.__class__.__name__} face_selector forwarded")

    if _HAS_CQ:
        try:
            stock = create_rectangular_stock(100, 60, 20)
            d_op = DrillOp(cx=0, cy=0, diameter=10, depth=0, through=True,
                          spot_drill=False, face_selector="<Z")
            result = d_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "DrillOp from bottom face reduces volume")
        except Exception as exc:
            check(False, f"DrillOp bottom face apply crashed: {exc}")

    # ==================================================================
    # Test 13: Phase 2 operations
    # ==================================================================
    print("\n13. Phase 2 operations ...")

    # 13a. Instantiate each new op with defaults
    p2_ops: list[MachiningOperation] = [
        PeckDrillOp(),
        ReamOp(),
        BoreOp(),
        CountersinkOp(),
        CounterboreOp(),
        ThreadMillOp(),
        TSlotOp(),
        DovetailOp(),
        GrooveOp(),
        Surface3DOp(),
        DeburrOp(),
        SpotFaceOp(),
    ]
    p2_names = [
        "PeckDrillOp", "ReamOp", "BoreOp", "CountersinkOp",
        "CounterboreOp", "ThreadMillOp", "TSlotOp", "DovetailOp",
        "GrooveOp", "Surface3DOp", "DeburrOp", "SpotFaceOp",
    ]
    for op, name in zip(p2_ops, p2_names):
        check(isinstance(op, MachiningOperation),
              f"{name} is MachiningOperation")
        check(op.tool is not None, f"{name} auto-tool populated")
        check(isinstance(op.feeds_speeds, dict), f"{name} feeds_speeds is dict")

    # 13b. to_dict() returns dict with "operation" key
    for op, name in zip(p2_ops, p2_names):
        d = op.to_dict()
        check(isinstance(d, dict), f"{name}.to_dict returns dict")
        check("operation" in d, f"{name}.to_dict has 'operation' key")

    # 13c. PeckDrillOp has peck flag in feeds_speeds
    pd = PeckDrillOp()
    check(pd.feeds_speeds.get("peck") is True,
          "PeckDrillOp sets peck=True in feeds_speeds")

    # 13d. ReamOp auto-selects a reamer tool
    rm = ReamOp(diameter=6.0)
    check(rm.tool.diameter >= 6.0,
          f"ReamOp tool diameter >= 6.0 (got {rm.tool.diameter})")

    # 13e. BoreOp uses flat_endmill at ~90% diameter
    bo = BoreOp(diameter=20.0)
    check(bo.tool.tool_type == "flat_endmill",
          f"BoreOp tool_type = {bo.tool.tool_type!r}")

    # 13f. CountersinkOp uses chamfer tool
    ck = CountersinkOp()
    check(ck.tool.tool_type == "chamfer",
          f"CountersinkOp tool_type = {ck.tool.tool_type!r}")

    # 13g. CounterboreOp uses flat_endmill
    cb = CounterboreOp()
    check(cb.tool.tool_type == "flat_endmill",
          f"CounterboreOp tool_type = {cb.tool.tool_type!r}")

    # 13h. ThreadMillOp uses thread_mill tool
    tm = ThreadMillOp()
    check(tm.tool.tool_type == "thread_mill",
          f"ThreadMillOp tool_type = {tm.tool.tool_type!r}")
    # Geometry unchanged
    check(tm.apply(None if not _HAS_CQ else create_rectangular_stock(100, 60, 20)) is not None
          if _HAS_CQ else True,
          "ThreadMillOp.apply returns a shape or is unchanged")

    # 13i. TSlotOp and DovetailOp use dovetail tool type
    ts = TSlotOp()
    check(ts.tool.tool_type == "dovetail",
          f"TSlotOp tool_type = {ts.tool.tool_type!r}")
    dv = DovetailOp()
    check(dv.tool.tool_type == "dovetail",
          f"DovetailOp tool_type = {dv.tool.tool_type!r}")

    # 13j. GrooveOp uses flat_endmill
    gv = GrooveOp()
    check(gv.tool.tool_type == "flat_endmill",
          f"GrooveOp tool_type = {gv.tool.tool_type!r}")

    # 13k. Surface3DOp uses ball_endmill
    s3 = Surface3DOp()
    check(s3.tool.tool_type == "ball_endmill",
          f"Surface3DOp tool_type = {s3.tool.tool_type!r}")
    check(s3.position is None, "Surface3DOp position is None")

    # 13l. DeburrOp uses chamfer tool
    db = DeburrOp()
    check(db.tool.tool_type == "chamfer",
          f"DeburrOp tool_type = {db.tool.tool_type!r}")

    # 13m. SpotFaceOp uses flat_endmill
    sf = SpotFaceOp()
    check(sf.tool.tool_type == "flat_endmill",
          f"SpotFaceOp tool_type = {sf.tool.tool_type!r}")

    # 13n. Position tracking for positioned ops
    check(PeckDrillOp(cx=10, cy=20).position == (10, 20),
          "PeckDrillOp position tracked")
    check(ReamOp(cx=5, cy=15).position == (5, 15),
          "ReamOp position tracked")
    check(BoreOp(cx=-5, cy=-10).position == (-5, -10),
          "BoreOp position tracked")
    check(CountersinkOp(cx=0, cy=0).position == (0, 0),
          "CountersinkOp position tracked")
    check(CounterboreOp(cx=50, cy=25).position == (50, 25),
          "CounterboreOp position tracked")
    check(ThreadMillOp(cx=30, cy=40).position == (30, 40),
          "ThreadMillOp position tracked")
    check(SpotFaceOp(cx=100, cy=50).position == (100, 50),
          "SpotFaceOp position tracked")
    # Ops with no position
    check(Surface3DOp().position is None,
          "Surface3DOp position is None")
    check(DeburrOp().position is None,
          "DeburrOp position is None")

    # 13o. to_toolpath behavior for new ops
    implemented_p2_toolpaths = {
        "PeckDrillOp", "ReamOp", "BoreOp", "CountersinkOp",
        "CounterboreOp", "ThreadMillOp", "TSlotOp", "DovetailOp",
        "GrooveOp", "Surface3DOp", "DeburrOp", "SpotFaceOp",
    }
    for op, name in zip(p2_ops, p2_names):
        tp = op.to_toolpath()
        if name in implemented_p2_toolpaths:
            check(hasattr(tp, "to_dict"),
                  f"{name}.to_toolpath returns Toolpath-like object")
            check(len(tp) > 0,
                  f"{name}.to_toolpath is non-empty")
        else:
            check(isinstance(tp, list),
                  f"{name}.to_toolpath returns list")
            check(len(tp) == 0,
                  f"{name}.to_toolpath is empty (future mapping)")

    # 13p. create_operation factory for new ops
    for op_type in ["peck_drill", "ream", "bore", "countersink",
                    "counterbore", "thread_mill", "t_slot", "dovetail",
                    "groove", "surface_3d", "deburr", "spot_face"]:
        try:
            op = create_operation(op_type)
            check(isinstance(op, MachiningOperation),
                  f"create_operation({op_type!r}) returns MachiningOperation")
        except Exception as exc:
            check(False, f"create_operation({op_type!r}) raised {exc}")

    # 13q. apply() tests for a few key ops (requires CadQuery)
    if _HAS_CQ:
        # PeckDrillOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            pd_op = PeckDrillOp(cx=0, cy=0, diameter=10, depth=0, through=True)
            result = pd_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "PeckDrillOp reduces volume")
        except Exception as exc:
            check(False, f"PeckDrillOp.apply crashed: {exc}")

        # BoreOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            bo_op = BoreOp(cx=0, cy=0, diameter=30, depth=0, through=True)
            result = bo_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "BoreOp reduces volume")
        except Exception as exc:
            check(False, f"BoreOp.apply crashed: {exc}")

        # CounterboreOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            cb_op = CounterboreOp(cx=0, cy=0, hole_diameter=6,
                                  counterbore_diameter=12, counterbore_depth=5,
                                  through=False)
            result = cb_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "CounterboreOp reduces volume")
        except Exception as exc:
            check(False, f"CounterboreOp.apply crashed: {exc}")

        # SpotFaceOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            sf_op = SpotFaceOp(cx=0, cy=0, diameter=30, depth=2)
            result = sf_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "SpotFaceOp reduces volume")
        except Exception as exc:
            check(False, f"SpotFaceOp.apply crashed: {exc}")

        # GrooveOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            g_op = GrooveOp(cx=10, cy=0, length=40, width=6, depth=8)
            result = g_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "GrooveOp reduces volume")
        except Exception as exc:
            check(False, f"GrooveOp.apply crashed: {exc}")

        # ThreadMillOp (geometry unchanged)
        try:
            stock = create_rectangular_stock(100, 60, 20)
            tm_op = ThreadMillOp()
            result = tm_op.apply(stock)
            check(abs(compute_volume(result) - compute_volume(stock)) < 0.1,
                  "ThreadMillOp does not change volume")
        except Exception as exc:
            check(False, f"ThreadMillOp.apply crashed: {exc}")

        # DeburrOp
        try:
            stock = create_rectangular_stock(100, 60, 20)
            db_op = DeburrOp()
            result = db_op.apply(stock)
            check(compute_volume(result) < compute_volume(stock),
                  "DeburrOp reduces volume")
        except Exception as exc:
            check(False, f"DeburrOp.apply crashed: {exc}")
    else:
        print("    Phase 2 apply() geometry tests SKIPPED (CadQuery not available)")

    # ==================================================================
    # Summary
    # ==================================================================
    total = passed + failed
    print(f"\n{'=' * 60}")
    print(f"Results: {passed}/{total} passed, {failed}/{total} failed")
    if failed > 0:
        sys.exit(1)
