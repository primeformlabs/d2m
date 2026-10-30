"""Release program fixes (d2o#878): the countersink path in the part frame, and a flute length equal to the wall."""

import math

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory


def _chamfer():
    return ToolSpec("chamfer", 16.0, tip_angle=90.0, tip_diameter=2.0, flute_length=8.0, reach_mm=40.0, stickout_mm=40.0,
                    num_flutes=2, catalog_id="CH16", physical_assembly={"owned": True})


@pytest.mark.parametrize("face, axis", [(">Z", [0.0, 0.0, 1.0]), ("<Z", [0.0, 0.0, -1.0])])
def test_the_countersink_path_is_in_the_part_frame_with_the_tip_below_the_material_top(face, axis):
    # A 40 x 40 x 10 plate (Z -5 to 5); the cone of a 90° chamfer with a 2 mm tip is 12 mm wide 5 mm above its tip.
    with tool_inventory([_chamfer()], revision="countersink", strict=True):
        result = Stock.rectangular(40, 40, 10).countersink(6.0, 12.0, depth=3.0, countersink_angle=90.0, cx=5.0, cy=5.0,
                                                         face_selector=face)
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert path["metadata"]["coordinate_system"] == "part_mm"
    assert path["metadata"]["tool_axis"] == pytest.approx(axis)
    tip = (12.0 - 2.0) / 2 / math.tan(math.radians(45.0))
    plunge = next(move for move in path["moves"] if move["type"] == "plunge")
    assert plunge["position"][2] == pytest.approx((5.0 - tip) if face == ">Z" else (-5.0 + tip), abs=1e-6)
    assert plunge["position"][:2] == pytest.approx([5.0, 5.0], abs=1e-6)


def test_a_flute_length_equal_to_the_wall_cuts_it():
    mill = ToolSpec("flat_endmill", 8.0, flute_length=20.0, reach_mm=35.0, stickout_mm=35.0, num_flutes=4,
                    catalog_id="EM08", physical_assembly={"owned": True})
    with tool_inventory([mill], revision="flute", strict=True):
        result = Stock.rectangular(60, 40, 30).profile_pocket({"type": "rectangle", "length": 20, "width": 12}, 20.00004,
                                                               stepdown=5)
    assert result.process_plan()["operations"][-1]["tool"]["catalog_id"] == "EM08"


def test_an_insert_cutter_cuts_the_outside_wall_to_size_at_each_level():
    # Ø12 cutter, 40 x 20 outline: at the wall the cutter centre is 6 mm from the outline. A 10 mm insert cuts a 25 mm
    # wall in passes; each pass must reach the wall, not leave an allowance for one finishing pass at the bottom.
    tool = ToolSpec("indexable_end_mill", 12.0, flute_length=10.0, reach_mm=45.0, stickout_mm=45.0, num_flutes=2,
                    catalog_id="IEM12", physical_assembly={"owned": True})
    with tool_inventory([tool], revision="insert-wall", strict=True):
        result = Stock.rectangular(60, 40, 30).profile_cutout({"type": "rectangle", "length": 40, "width": 20}, depth=25,
                                                             stepdown=10)
    moves = [move for move in result.process_plan()["operations"][-1]["toolpath"]["moves"] if move["type"] == "feed"]
    levels: dict[float, float] = {}
    for move in moves:
        x, y, z = move["position"]
        gap = math.hypot(max(abs(x) - 20.0, 0.0), max(abs(y) - 10.0, 0.0))
        levels[round(z, 4)] = min(levels.get(round(z, 4), math.inf), gap)
    assert len(levels) >= 3
    assert all(value == pytest.approx(6.0, abs=1e-6) for value in levels.values())
