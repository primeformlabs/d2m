"""Production tool selection must preserve each caller's physical inventory."""
from concurrent.futures import ThreadPoolExecutor
from threading import Barrier

import pytest

from src.subcad import Stock, ToolSpec, ToolRequirement, tool_inventory
from src.subcad.tool_selection import select_tool, select_or_fallback, ToolUnavailableError


def cutter(key="MILL-6", diameter=6, **kwargs):
    return ToolSpec("flat_endmill", diameter, catalog_id=key, flute_length=18,
        stickout_mm=24, reach_mm=18, **kwargs)


def test_empty_owned_inventory_never_uses_standard_catalog_or_fallback():
    with tool_inventory([], revision="empty"):
        result = select_tool(ToolRequirement("flat_endmill"))
        assert not result.ok
        assert result.selected_tool is None
        assert result.inventory_revision == "empty"
        with pytest.raises(ToolUnavailableError) as error:
            select_or_fallback(ToolRequirement("flat_endmill"), cutter())
        assert error.value.result.selected_tool is None
        with pytest.raises(ToolUnavailableError):
            Stock.rectangular(30, 30, 20).pocket(width=12, length=15, depth=3)


def test_owned_tool_and_actual_holder_are_retained_in_the_native_plan():
    tool = cutter(physical_assembly={"assembly_id": "MILL-6", "holder_id": "BT40-001",
        "holder_diameter_mm": 42, "holder_length_mm": 80, "owned": True,
        "geometry_status": "measured"})
    with tool_inventory([tool], revision="revision-1"):
        result = Stock.rectangular(30, 30, 20).pocket(width=12, length=15, depth=3, corner_radius=3)
    operation = result.process_plan()["operations"][0]
    assert operation["tool"]["catalog_id"] == "MILL-6"
    assert operation["tool"]["assembly"] == tool.physical_assembly
    assert operation["tool_selection"]["inventory_revision"] == "revision-1"
    assert result.volume < 30 * 30 * 20


def test_short_flutes_are_not_replaced_by_long_reach():
    tool = cutter()
    tool.flute_length = 3
    tool.reach_mm = 50
    with tool_inventory([tool]):
        result = select_tool(ToolRequirement("flat_endmill", min_flute_length_mm=5))
    assert not result.ok
    assert "flute_length_too_short" in result.rejected_tools[0]["reasons"]


def test_short_projection_cannot_reach_the_feature():
    tool = cutter()
    tool.stickout_mm = 2
    with tool_inventory([tool]):
        result = select_tool(ToolRequirement("flat_endmill", min_flute_length_mm=5))
    assert not result.ok
    assert "reach_too_short" in result.rejected_tools[0]["reasons"]


def test_explicit_tool_cannot_bypass_inventory_with_the_same_id():
    with tool_inventory([cutter()]):
        with pytest.raises(ValueError, match="does not match"):
            Stock.rectangular(30, 30, 20).pocket(width=12, length=15, depth=3,
                tool=cutter(diameter=4), corner_radius=3)


def test_missing_finish_cutter_stops_the_operation():
    with tool_inventory([cutter()]):
        with pytest.raises(ToolUnavailableError) as error:
            Stock.rectangular(30, 30, 20).pocket(width=12, length=15, depth=3, corner_radius=1)
    assert error.value.result.requirement.feature == "pocket_corner_finish"


def test_explicit_owned_cutter_still_requires_sufficient_depth():
    tool = cutter()
    tool.flute_length = 2
    with tool_inventory([tool]):
        with pytest.raises(ValueError, match="cutting depth"):
            Stock.rectangular(30, 30, 20).pocket(width=12, length=15, depth=3,
                tool=tool, corner_radius=3)


def test_face_mill_is_available_for_facing_but_not_for_a_pocket():
    tool = cutter()
    tool.tool_type = "face_mill"
    tool.diameter = 20
    with tool_inventory([tool]):
        result = Stock.rectangular(30, 30, 20).face_mill(depth=1)
        assert result.process_plan()["operations"][0]["tool"]["catalog_id"] == "MILL-6"
        assert not select_tool(ToolRequirement("flat_endmill")).ok


def test_nested_scopes_restore_inventory_after_an_exception():
    with tool_inventory([cutter("outer")], revision="outer"):
        with pytest.raises(RuntimeError):
            with tool_inventory([cutter("inner")], revision="inner"):
                assert select_tool(ToolRequirement("flat_endmill")).selected_tool.catalog_id == "inner"
                raise RuntimeError("stop")
        assert select_tool(ToolRequirement("flat_endmill")).selected_tool.catalog_id == "outer"


def test_two_workers_select_only_their_own_inventory():
    barrier = Barrier(2)
    def select(key):
        with tool_inventory([cutter(key)], revision=key):
            barrier.wait(timeout=5)
            result = select_tool(ToolRequirement("flat_endmill"))
            return result.selected_tool.catalog_id, result.inventory_revision
    with ThreadPoolExecutor(max_workers=2) as pool:
        left = pool.submit(select, "left")
        right = pool.submit(select, "right")
        assert left.result(timeout=10) == ("left", "left")
        assert right.result(timeout=10) == ("right", "right")


def test_scope_copies_tools_before_selection():
    original = cutter()
    with tool_inventory([original]):
        original.diameter = 100
        selected = select_tool(ToolRequirement("flat_endmill")).selected_tool
        assert selected.diameter == 6
        selected.diameter = 200
        assert select_tool(ToolRequirement("flat_endmill")).selected_tool.diameter == 6


@pytest.mark.parametrize("value", [float("nan"), float("inf"), 0, -1])
def test_invalid_owned_dimensions_never_select_a_cutter(value):
    with tool_inventory([cutter(diameter=value)]):
        result = select_tool(ToolRequirement("flat_endmill"))
    assert not result.ok
    assert "invalid_cutting_dimensions" in result.rejected_tools[0]["reasons"]


def test_duplicate_inventory_ids_fail_before_selection():
    with pytest.raises(ValueError, match="distinct catalog IDs"):
        with tool_inventory([cutter(), cutter()]):
            pass
