"""Insert cutters (d2o#878): each pass cuts at most the insert depth, the reach covers the wall.

An indexable end mill (90° inserts) roughs levels, faces and cuts the outside profile; a high-feed mill and a face mill
only face. A solid end mill cuts the whole wall with its flutes."""

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory
    from src.subcad.tool_library import envelope_cutter


def _tool(kind, diameter, depth, reach, catalog_id):
    """``depth``: the insert depth of an insert cutter, the flute length of a solid end mill."""
    return ToolSpec(kind, diameter, flute_length=depth, reach_mm=reach, stickout_mm=reach, num_flutes=3,
                    catalog_id=catalog_id, physical_assembly={"owned": True})


def _step_part():
    """A 60 x 40 x 30 block whose +X half is 20 mm lower."""
    high = cq.Workplane("XY").box(30, 40, 30).translate((-15, 0, 0))
    low = cq.Workplane("XY").box(30, 40, 10).translate((15, 0, -10))
    return high.union(low).val()


def _levels(record):
    return sorted({round(move["position"][2], 6) for move in record["toolpath"]["moves"] if move["type"] == "feed"})


def test_the_envelope_is_the_cutting_diameter_down_to_the_reach_in_passes_of_the_insert_depth():
    cutter = envelope_cutter(_tool("indexable_end_mill", 16, 10, 45, "IEM16"))
    assert (cutter.tool_type, cutter.corner_radius, cutter.flute_length) == ("flat_endmill", 0.0, 45)
    assert cutter.safe_limits["max_stepdown_mm"] == 10
    solid = _tool("flat_endmill", 16, 10, 45, "EM16")
    assert envelope_cutter(solid) is solid


def test_an_indexable_end_mill_roughs_a_step_deeper_than_its_inserts():
    with tool_inventory([_tool("indexable_end_mill", 16, 10, 45, "IEM16")], revision="insert", strict=True):
        roughed = Stock.rectangular(60, 40, 30).rough_levels(_step_part(), face_selector=">Z")
    record = roughed.process_plan()["operations"][-1]
    assert record["operation"] == "level_rough"
    assert record["tool"]["catalog_id"] == "IEM16"
    assert record["toolpath"]["tool_type"] == "indexable_end_mill"
    assert record["maximum_stepdown_mm"] <= 10
    assert 60 * 40 * 30 - roughed._shape.val().Volume() == pytest.approx(30 * 40 * 20, rel=2e-3)


def test_a_high_feed_mill_does_not_rough_levels():
    # Its effective corner radius would stay at the walls of the levels.
    with tool_inventory([_tool("high_feed_mill", 32, 1, 96, "HFM32")], revision="insert", strict=True):
        with pytest.raises(ValueError):
            Stock.rectangular(60, 40, 30).rough_levels(_step_part(), face_selector=">Z")


@pytest.mark.parametrize("kind", ["high_feed_mill", "face_mill", "indexable_end_mill"])
def test_facing_with_an_insert_cutter_cuts_in_passes_of_its_insert_depth(kind):
    with tool_inventory([_tool(kind, 32, 1, 96, "INSERT32")], revision="insert", strict=True):
        faced = Stock.rectangular(60, 40, 30).face_mill(2)
    record = faced.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "INSERT32"
    assert record["toolpath"]["tool_type"] == kind
    assert record["geometry_fidelity"] == "nominal_face_mill_envelope"
    # Two passes of 1 mm, the insert depth, not one of 2 mm.
    assert len(_levels(record)) == 2
    assert faced.volume == pytest.approx(60 * 40 * 28, rel=1e-6)


def test_the_outside_profile_with_an_indexable_end_mill_cuts_a_wall_deeper_than_its_inserts():
    profile = {"type": "rectangle", "length": 40, "width": 20}
    with tool_inventory([_tool("indexable_end_mill", 12, 10, 45, "IEM12")], revision="insert", strict=True):
        result = Stock.rectangular(60, 40, 30).profile_cutout(profile, depth=25, stepdown=5)
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "IEM12"
    assert record["toolpath"]["tool_type"] == "indexable_end_mill"
    assert Stock.rectangular(60, 40, 30).volume - result.volume == pytest.approx((60 * 40 - 40 * 20) * 25, rel=1e-4)


def test_a_solid_end_mill_needs_flutes_for_the_whole_outside_wall():
    profile = {"type": "rectangle", "length": 40, "width": 20}
    with tool_inventory([_tool("flat_endmill", 12, 10, 45, "EM12")], revision="solid", strict=True):
        with pytest.raises(ValueError):
            Stock.rectangular(60, 40, 30).profile_cutout(profile, depth=25, stepdown=5)


def test_a_pocket_is_not_cut_with_an_insert_cutter():
    with tool_inventory([_tool("indexable_end_mill", 16, 10, 45, "IEM16")], revision="insert", strict=True):
        with pytest.raises(ValueError):
            Stock.rectangular(60, 40, 30).profile_pocket({"type": "rectangle", "length": 40, "width": 30}, 5)
