"""Inventory-aware tool selection for SubCAD operations."""

from __future__ import annotations

from dataclasses import dataclass, field
from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
import math
from typing import Optional

from .tool_library import ToolCatalog, ToolSpec


@dataclass(frozen=True)
class _InventoryScope:
    tools: tuple[ToolSpec, ...]
    revision: str
    strict: bool
    selector: object = None


_inventory_scope: ContextVar[Optional[_InventoryScope]] = ContextVar("subcad_inventory", default=None)


@contextmanager
def tool_inventory(tools, *, revision: str = "", strict: bool = True, selector=None):
    """Use an isolated inventory snapshot without changing the shared catalog.

    Strict snapshots prohibit fallback tools and explicit tools with different
    specifications. The caller supplies qualified assemblies. An optional ``selector`` receives
    the requirement and its feasible candidates. It must return one of those candidates; it
    cannot bypass geometry, reach or availability checks. The policy is isolated to this context.
    """
    snapshot = tuple(deepcopy(tool) for tool in tools)
    ids = [tool.catalog_id for tool in snapshot]
    if any(not key for key in ids) or len(ids) != len(set(ids)):
        raise ValueError("Inventory tools require distinct catalog IDs.")
    token = _inventory_scope.set(_InventoryScope(snapshot, revision, strict, selector))
    try:
        yield
    finally:
        _inventory_scope.reset(token)


class ToolUnavailableError(ValueError):
    """An operation cannot use a tool from the current inventory snapshot."""

    def __init__(self, result: "ToolSelectionResult"):
        self.result = result
        super().__init__("; ".join(result.errors))


def tool_reach(tool: ToolSpec) -> float:
    """How deep the tool tip goes below the setup face: the projection from the holder, or a shorter recorded
    reach (a tool with no recorded reach reaches its flute length)."""
    return min(tool.stickout_mm, tool.reach_mm or tool.flute_length)


def validate_inventory_tool(tool: Optional[ToolSpec], *, depth: float = 0.0, cut_length: Optional[float] = None) -> None:
    """Reject explicit or retained cutters that bypass a strict snapshot.

    ``depth``: from the setup face to the deepest cut; ``cut_length``: the material height that the flutes cut
    (the tool goes through the air above the material start). The default is the depth."""
    scope = _inventory_scope.get()
    if scope is None or not scope.strict or tool is None:
        return
    matching = next((candidate for candidate in scope.tools if candidate.catalog_id == tool.catalog_id), None)
    if matching is None or matching.inventory_count <= 0 or matching.to_dict() != tool.to_dict():
        raise ValueError(f"Tool {tool.catalog_id or tool.tool_type} does not match the owned inventory snapshot.")
    cut = abs(depth) if cut_length is None else abs(cut_length)
    if cut > tool.flute_length + 1e-8 or abs(depth) > tool_reach(tool) + 1e-8:
        raise ValueError(f"Tool {tool.catalog_id} cannot reach the required cutting depth.")


@dataclass
class ToolRequirement:
    """Capability requested by an operation before a real tool is selected."""

    tool_type: str
    min_diameter_mm: float = 0.0
    max_diameter_mm: Optional[float] = None
    min_flute_length_mm: float = 0.0
    """The material height that the flutes cut in one pass."""
    min_reach_mm: Optional[float] = None
    """From the setup face to the deepest cut (``tool_reach``); None: the flute length (the cut starts at the face)."""
    feature: str = ""
    reason: str = ""
    prefer_largest: bool = False
    alternative_tool_types: tuple[str, ...] = ()
    require_point_geometry: bool = False
    include_point_length_in_reach: bool = False

    @property
    def reach_mm(self) -> float:
        return self.min_flute_length_mm if self.min_reach_mm is None else self.min_reach_mm

    def to_dict(self) -> dict:
        return {
            "tool_type": self.tool_type,
            "min_diameter_mm": self.min_diameter_mm,
            "max_diameter_mm": self.max_diameter_mm,
            "min_flute_length_mm": self.min_flute_length_mm,
            "min_reach_mm": self.reach_mm,
            "feature": self.feature,
            "reason": self.reason,
            "prefer_largest": self.prefer_largest,
            "alternative_tool_types": list(self.alternative_tool_types),
            "require_point_geometry": self.require_point_geometry,
            "include_point_length_in_reach": self.include_point_length_in_reach,
        }


@dataclass
class ToolSelectionResult:
    """Structured result of resolving a requested capability to inventory."""

    requirement: ToolRequirement
    selected_tool: Optional[ToolSpec] = None
    rejected_tools: list[dict] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    inventory_revision: str = ""

    @property
    def ok(self) -> bool:
        return self.selected_tool is not None and not self.errors

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "requirement": self.requirement.to_dict(),
            "selected_tool_id": (
                self.selected_tool.catalog_id if self.selected_tool else None
            ),
            "reason": self.requirement.reason,
            "rejected_tools": self.rejected_tools,
            "warnings": list(self.warnings),
            "errors": list(self.errors),
            "inventory_revision": self.inventory_revision,
        }


def select_tool(requirement: ToolRequirement) -> ToolSelectionResult:
    """Select an available catalog tool and record rejected candidates."""
    scope = _inventory_scope.get()
    if scope is None:
        ToolCatalog._ensure_initialised()
        inventory = list(ToolCatalog._tools.items())
    else:
        inventory = [(tool.catalog_id, tool) for tool in scope.tools]
    result = ToolSelectionResult(requirement=requirement, inventory_revision=scope.revision if scope else "")
    candidates: list[ToolSpec] = []

    for name, tool in sorted(inventory, key=lambda item: item[0]):
        reasons: list[str] = []
        if any(not math.isfinite(value) or value <= 0 for value in (tool.diameter, tool.flute_length)):
            reasons.append("invalid_cutting_dimensions")
        allowed_types = (requirement.tool_type, *requirement.alternative_tool_types)
        if tool.tool_type not in allowed_types:
            reasons.append("wrong_type")
        if tool.inventory_count <= 0:
            reasons.append("unavailable")
        if tool.diameter < requirement.min_diameter_mm:
            reasons.append("diameter_too_small")
        if (
            requirement.max_diameter_mm is not None
            and tool.diameter > requirement.max_diameter_mm
        ):
            reasons.append("diameter_too_large")
        if requirement.min_flute_length_mm and tool.flute_length < requirement.min_flute_length_mm:
            reasons.append("flute_length_too_short")
        if requirement.reach_mm and scope and scope.strict and tool_reach(tool) < requirement.reach_mm:
            reasons.append("reach_too_short")
        if requirement.require_point_geometry and tool.tool_type in allowed_types:
            try:
                from .drilling_replay import point_geometry
                _, _, point = point_geometry(tool)
                extra = point if requirement.include_point_length_in_reach else 0
                if (requirement.min_flute_length_mm + extra > tool.flute_length
                        or requirement.reach_mm + extra > tool_reach(tool)):
                    reasons.append("point_depth_exceeds_reach")
            except (ValueError, TypeError):
                reasons.append("missing_or_invalid_point_geometry")

        if reasons:
            if tool.tool_type in allowed_types:
                result.rejected_tools.append({
                    "tool_id": tool.catalog_id or name,
                    "diameter_mm": tool.diameter,
                    "flute_length_mm": tool.flute_length,
                    "reach_mm": tool.reach_mm or tool.flute_length,
                    "inventory_count": tool.inventory_count,
                    "reasons": reasons,
                })
            continue
        candidates.append(tool)

    if candidates:
        if scope is not None and scope.selector is not None:
            selected = scope.selector(requirement, tuple(deepcopy(tool) for tool in candidates))
            matching = next((tool for tool in candidates if selected is not None
                             and tool.to_dict() == selected.to_dict()), None)
            if matching is None:
                raise ValueError("The selection policy must return a feasible inventory candidate.")
            result.selected_tool = deepcopy(matching)
            return result
        candidates.sort(key=lambda tool: tool.diameter)
        result.selected_tool = deepcopy(candidates[-1] if requirement.prefer_largest else candidates[0])
        return result

    message = (
        f"No available {requirement.tool_type} satisfies "
        f"diameter {requirement.min_diameter_mm:g}"
    )
    if requirement.max_diameter_mm is not None:
        message += f"..{requirement.max_diameter_mm:g}"
    if requirement.min_flute_length_mm:
        message += f" and flute length {requirement.min_flute_length_mm:g} mm"
    if requirement.reach_mm > requirement.min_flute_length_mm + 1e-9:
        message += f" with reach {requirement.reach_mm:g} mm"
    result.errors.append(message)
    if requirement.require_point_geometry:
        result.errors.append("Reaming requires a recorded lead angle, a positive lead tip diameter, and complete lead reach." if requirement.tool_type == "reamer"
            else "Drilling requires a valid recorded point angle and enough reach for the complete drill point.")
    return result


def select_or_fallback(requirement: ToolRequirement, fallback: ToolSpec) -> tuple[ToolSpec, ToolSelectionResult]:
    """Return selected inventory, or a fallback tool with a structured error."""
    result = select_tool(requirement)
    if result.selected_tool is not None:
        return result.selected_tool, result
    scope = _inventory_scope.get()
    if scope is not None and scope.strict:
        raise ToolUnavailableError(result)
    result.selected_tool = fallback
    result.warnings.append("Using fallback tool so legacy geometry can still be generated.")
    return fallback, result
