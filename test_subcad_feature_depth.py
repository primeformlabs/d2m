"""A feature cut starts where its material starts (feature_depth.py): the flutes cut the material height, the
reach covers the depth from the setup face, and a through feature ends at the bottom of its material column."""

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory
    from src.subcad.feature_depth import material_span
    from src.subcad.tool_selection import ToolRequirement, ToolUnavailableError, select_tool, tool_reach


def _drill(diameter, flute, stickout, catalog_id):
    return ToolSpec("drill", diameter, flute_length=flute, reach_mm=stickout, stickout_mm=stickout, tip_angle=118.0,
                    tip_diameter=0.0, num_flutes=2, catalog_id=catalog_id, physical_assembly={"owned": True})


def _mill(diameter, flute, stickout, catalog_id):
    return ToolSpec("flat_endmill", diameter, flute_length=flute, reach_mm=stickout, stickout_mm=stickout,
                    num_flutes=4, catalog_id=catalog_id, physical_assembly={"owned": True})


def _stepped():
    """A 60 x 40 x 30 block with its +X half 20 mm lower: a 10 mm section under the step."""
    with tool_inventory([_mill(10, 25, 40, "EM10")], revision="step", strict=True):
        return Stock.rectangular(60, 40, 30).profile_pocket(
            {"type": "rectangle", "length": 40, "width": 60, "cx": 20, "cy": 0}, 20, finish_allowance=0)


def test_the_span_is_the_material_inside_the_footprint():
    stepped = _stepped()
    assert material_span(stepped._shape, ">Z", cx=20, cy=0, radius=3) == pytest.approx((20, 30), abs=1e-6)
    assert material_span(stepped._shape, ">Z", cx=-20, cy=0, radius=3) == pytest.approx((0, 30), abs=1e-6)
    assert material_span(stepped._shape, ">Z", cx=200, cy=0, radius=3) is None


def test_a_hole_under_a_step_needs_a_short_flute_and_a_long_reach():
    stepped = _stepped()
    short_flute = _drill(6, 14, 45, "D6")
    with tool_inventory([short_flute], revision="step", strict=True):
        drilled = stepped.drill(6, through=True, cx=20, cy=0, spot_drill=False)
    operation = drilled.process_plan()["operations"][-1]
    assert operation["start_depth_mm"] == pytest.approx(20, abs=1e-6)
    assert operation["depth_mm"] == pytest.approx(30, abs=1e-6)
    removed = stepped._shape.val().Volume() - drilled._shape.val().Volume()
    assert removed == pytest.approx(3.14159265 * 9 * 10, rel=2e-3)
    # The same drill cannot drill the 30 mm section at the other end: its flutes are shorter than the material.
    with tool_inventory([short_flute], revision="step", strict=True), pytest.raises(ToolUnavailableError):
        stepped.drill(6, through=True, cx=-20, cy=0, spot_drill=False)


def test_a_short_tool_does_not_reach_the_low_step():
    stepped = _stepped()
    with tool_inventory([_drill(6, 20, 25, "D6S")], revision="step", strict=True), pytest.raises(ToolUnavailableError):
        stepped.drill(6, through=True, cx=20, cy=0, spot_drill=False)


def test_flute_and_reach_are_two_checks():
    tool = _mill(8, 20, 60, "EM8")
    with tool_inventory([tool], revision="rule", strict=True):
        assert select_tool(ToolRequirement("flat_endmill", max_diameter_mm=10, min_flute_length_mm=20,
                                           min_reach_mm=60)).ok
        assert not select_tool(ToolRequirement("flat_endmill", max_diameter_mm=10, min_flute_length_mm=21,
                                               min_reach_mm=30)).ok
        assert not select_tool(ToolRequirement("flat_endmill", max_diameter_mm=10, min_flute_length_mm=10,
                                               min_reach_mm=61)).ok
        # Without a reach the requirement keeps the old rule: the flute length is also the reach.
        assert select_tool(ToolRequirement("flat_endmill", max_diameter_mm=10, min_flute_length_mm=20)).ok
    assert tool_reach(tool) == 60
    assert tool_reach(ToolSpec("flat_endmill", 8, flute_length=20, stickout_mm=35)) == 20


def test_a_bore_under_a_step_starts_at_its_material():
    stepped = _stepped()
    with tool_inventory([_mill(10, 25, 40, "EM10"), _mill(8, 12, 40, "EM8")], revision="step", strict=True):
        bored = stepped.bore(16, through=True, cx=20, cy=0, face_selector=">Z")
    operation = bored.process_plan()["operations"][-1]
    assert operation["start_depth_mm"] == pytest.approx(20, abs=1e-6)
    removed = stepped._shape.val().Volume() - bored._shape.val().Volume()
    assert removed == pytest.approx(3.14159265 * 64 * 10, rel=5e-3)


def test_the_reach_is_measured_from_the_highest_material_near_the_hole():
    """Rajesh (2026-10-26): the reach starts at the highest material inside the region of the tool and its holder, not
    at the setup face. A 200 mm block whose +X 180 mm are 20 mm lower: a hole at x = 80 has no tall material within the
    holder (50 mm assumed) and the clearance, so a drill with 18 mm of projection drills its 10 mm section."""
    with tool_inventory([_mill(10, 25, 40, "EM10")], revision="local", strict=True):
        stepped = Stock.rectangular(200, 40, 30).profile_pocket(
            {"type": "rectangle", "length": 190, "width": 60, "cx": 15, "cy": 0}, 20, finish_allowance=0)
    short = _drill(6, 14, 18, "D6S")
    with tool_inventory([short], revision="local", strict=True):
        drilled = stepped.drill(6, through=True, cx=80, cy=0, spot_drill=False)
    assert drilled.process_plan()["operations"][-1]["start_depth_mm"] == pytest.approx(20, abs=1e-3)
    # Near the tall material the same drill does not reach: the holder region holds the 30 mm block end.
    with tool_inventory([short], revision="local", strict=True), pytest.raises(ToolUnavailableError):
        stepped.drill(6, through=True, cx=-70, cy=0, spot_drill=False)
