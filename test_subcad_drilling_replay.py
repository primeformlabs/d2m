import math

import cadquery as cq
import pytest

from src.subcad import Stock, ToolSpec, tool_inventory, replay_milling_independently
from src.subcad.operations import DrillOp
from src.subcad.toolpath import Toolpath


def drill(diameter=2, **params):
    return ToolSpec("drill", diameter, tip_angle=118, **params)


@pytest.mark.parametrize("face,axis,point", [
    (">Z", [0, 0, 1], lambda u, v, w: (u, v, w)),
    ("<Z", [0, 0, -1], lambda u, v, w: (u, v, -w)),
    (">X", [1, 0, 0], lambda u, v, w: (w, u, v)),
    ("<X", [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    (">Y", [0, 1, 0], lambda u, v, w: (u, w, v)),
    ("<Y", [0, -1, 0], lambda u, v, w: (u, -w, v)),
])
def test_blind_drill_point_and_pecks_follow_six_faces(face, axis, point):
    stock = Stock.rectangular(12, 12, 12)
    tool = drill()
    result = stock.peck_drill(2, 2, cx=1, cy=-1, peck_step=0.5, face_selector=face, tool=tool)
    point_length = 1 / math.tan(math.radians(59))
    assert stock.volume - result.volume == pytest.approx(math.pi * (2 + point_length / 3), abs=1e-6)
    assert not result._shape.val().isInside(cq.Vector(*point(1, -1, 5)))
    assert result._shape.val().isInside(cq.Vector(*point(1.9, -1, 3.7)))
    assert not result._shape.val().isInside(cq.Vector(*point(1, -1, 3.7)))
    record = result.process_plan()["operations"][-1]
    assert len(record["material_passes"]) == math.ceil((2 + point_length) / 0.5)
    assert record["toolpath"]["metadata"]["tool_axis"] == axis
    assert record["toolpath"]["metadata"]["full_diameter_depth_mm"] == 2
    previous = stock.volume
    for state, report in zip(result._state_history[1:], record["material_passes"]):
        assert state["shape"].val().isValid()
        assert state["shape"].val().Volume() <= previous + 1e-6
        assert report["removed_volume_mm3"] == pytest.approx(previous - state["shape"].val().Volume())
        previous = state["shape"].val().Volume()
    assert any("flat floor" in reason for reason in record["review_reasons"])
    assert record["manufacturing_ready"] is False


def test_spot_drill_changes_material_and_records_a_separate_cutter_before_drilling():
    spot = Stock.rectangular(12, 12, 8).spot_drill(2, 0.5, tool=drill(4))
    radius = 0.5 * math.tan(math.radians(59))
    assert 12 * 12 * 8 - spot.volume == pytest.approx(math.pi * radius ** 2 * 0.5 / 3, abs=1e-6)
    tools = [drill(2, catalog_id="OWNED-DRILL-2"), drill(4, catalog_id="OWNED-SPOT-4")]
    with tool_inventory(tools, strict=True):
        result = Stock.rectangular(12, 12, 8).drill(2, 2)
    records = result.process_plan()["operations"]
    assert [op["operation"] for op in records] == ["spot_drill", "drill"]
    assert [op["tool"]["catalog_id"] for op in records] == ["OWNED-SPOT-4", "OWNED-DRILL-2"]
    assert len(result._state_history) == 3


def test_direct_operation_spot_is_split_into_actual_stock_operations():
    result = Stock.rectangular(12, 12, 8)._apply_op(DrillOp(diameter=2, depth=2))
    assert [op["operation"] for op in result.process_plan()["operations"]] == ["spot_drill", "drill"]
    assert [op["sequence_number"] for op in result.process_plan()["operations"]] == [1, 2]


def test_through_drill_uses_axis_thickness_and_records_tip_breakout():
    stock = Stock.rectangular(10, 8, 6)
    result = stock.drill(2, through=True, face_selector=">X", spot_drill=False, tool=drill())
    assert stock.volume - result.volume == pytest.approx(math.pi * 10, abs=1e-6)
    record = result.process_plan()["operations"][-1]
    assert record["depth_mm"] == 10
    assert record["toolpath"]["metadata"]["tip_depth_mm"] > 10
    assert record["toolpath"]["metadata"]["tool_axis"] == [1, 0, 0]


def test_drill_preserves_translated_incoming_geometry_and_previous_hole():
    stock = Stock.rectangular(12, 12, 8)
    stock._shape = stock._shape.translate((20, -15, 10))
    stock._shape = stock._shape.cut(cq.Workplane("XY", origin=(17, -15, 5)).circle(0.5).extrude(10))
    result = stock.drill(2, 2, cx=22, cy=-15, spot_drill=False, tool=drill())
    assert not result._shape.val().isInside(cq.Vector(17, -15, 8))
    assert not result._shape.val().isInside(cq.Vector(22, -15, 13))
    assert result._shape.val().isInside(cq.Vector(20, -15, 13))
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert all(move["position"][:2] == [22, -15] for move in path["moves"])


def test_tool_selection_rejects_missing_point_and_includes_point_length_in_reach():
    unknown = ToolSpec("drill", 2, catalog_id="UNKNOWN")
    short = drill(2, catalog_id="SHORT", flute_length=2.2)
    long = drill(2, catalog_id="LONG")
    with tool_inventory([unknown, short, long], strict=True):
        result = Stock.rectangular(12, 12, 8).drill(2, 2, spot_drill=False)
    assert result.process_plan()["operations"][-1]["tool"]["catalog_id"] == "LONG"
    with tool_inventory([unknown], strict=True), pytest.raises(ValueError, match="point angle"):
        Stock.rectangular(12, 12, 8).drill(2, 2, spot_drill=False)


@pytest.mark.parametrize("tip", [0, 0.6])
def test_independent_replay_uses_the_recorded_point_and_real_incoming_material(tip):
    tool = drill(2, tip_diameter=tip)
    stock = Stock.rectangular(6, 6, 6)
    result = stock.peck_drill(2, 1, peck_step=0.5, tool=tool)
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=result)
    assert not model.get_material_at((0, 0, 2.5))
    assert model.get_material_at((2, 0, 2.5))
    assert model.get_material_at((0, 0, 0))
    assert report["target_comparison"]["samples"] > 0
    assert report["target_comparison"]["overcut_samples"] == 0
    assert report["manufacturing_ready"] is False


@pytest.mark.parametrize("params,tool,reason", [
    ({}, ToolSpec("drill", 2), "point angle"),
    ({}, drill(4), "does not match"),
    ({"depth": 3.6}, drill(), "passes through"),
    ({"depth": 2}, drill(2, flute_length=2.2), "cannot reach"),
    ({"cx": 100}, drill(), "no material"),
    ({"peck_step": 0}, drill(), "peck step"),
])
def test_invalid_drilling_cannot_return_an_ideal_hole(params, tool, reason):
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(12, 12, 4).peck_drill(**{"diameter": 2, "depth": 1, "tool": tool, **params})
