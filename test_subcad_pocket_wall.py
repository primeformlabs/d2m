"""A pocket wall starts at the material beside the pocket (feature_depth.pocket_wall_start): the flutes cut the wall
below it, and the reach is measured from the highest material that the holder passes (PocketLocalReach)."""

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory
    from src.subcad.feature_depth import pocket_wall_start
    from src.subcad.tool_selection import ToolUnavailableError

OUTLINE = {"type": "rectangle", "length": 30, "width": 20, "cx": 0, "cy": 0}


def _mill(diameter, flute, stickout, catalog_id):
    return ToolSpec("flat_endmill", diameter, flute_length=flute, reach_mm=stickout, stickout_mm=stickout,
                    num_flutes=4, catalog_id=catalog_id, physical_assembly={"owned": True})


def _column():
    """An 80 x 60 x 60 block lowered by 30 mm around a 30 x 20 mm column: the material that level roughing keeps
    over a pocket (d2o#878, PT-002311)."""
    with tool_inventory([_mill(12, 40, 50, "EM12")], revision="column", strict=True):
        return Stock.rectangular(80, 60, 60).machine_around_profile(OUTLINE, 30)


def test_the_wall_starts_at_the_material_beside_the_outline():
    column = _column()
    points = [(-15, -10), (15, -10), (15, 10), (-15, 10)]
    assert pocket_wall_start(column._shape, ">Z", points, 60) == pytest.approx(30, abs=1e-6)
    assert pocket_wall_start(Stock.rectangular(80, 60, 60)._shape, ">Z", points, 60) == pytest.approx(0, abs=1e-6)


def test_a_pocket_under_a_level_needs_flutes_for_its_wall_only():
    column = _column()
    with tool_inventory([_mill(10, 32, 40, "EM10")], revision="pocket", strict=True):
        cut = column.profile_pocket(OUTLINE, 60, through=True, finish_allowance=0)
    operation = cut.process_plan()["operations"][-1]
    assert operation["tool"]["catalog_id"] == "EM10"
    # Before: the flutes had to cover the 60 mm from the stock face.
    with tool_inventory([_mill(10, 25, 40, "EM10S")], revision="short", strict=True), pytest.raises(ToolUnavailableError):
        column.profile_pocket(OUTLINE, 60, through=True, finish_allowance=0)


def test_a_pocket_in_a_flat_face_keeps_the_full_wall():
    with tool_inventory([_mill(10, 32, 40, "EM10")], revision="flat", strict=True), pytest.raises(ToolUnavailableError):
        Stock.rectangular(80, 60, 60).profile_pocket(OUTLINE, 40, finish_allowance=0)
