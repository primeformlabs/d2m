import math

import cadquery as cq
import pytest

from src.subcad import Stock, ToolSpec, tool_inventory
from src.subcad.operations import SlotOp


def inside(stock, point):
    return stock._shape.val().isInside(cq.Vector(*point))


@pytest.mark.parametrize("face,axis,point", [
    (">Z", [0, 0, 1], lambda u, v, w: (u, v, w)),
    ("<Z", [0, 0, -1], lambda u, v, w: (u, v, -w)),
    (">X", [1, 0, 0], lambda u, v, w: (w, u, v)),
    ("<X", [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    (">Y", [0, 1, 0], lambda u, v, w: (u, w, v)),
    ("<Y", [0, -1, 0], lambda u, v, w: (u, -w, v)),
])
def test_slot_retains_selected_cutter_passes_on_six_faces(face, axis, point):
    stock = Stock.rectangular(16, 16, 16)
    result = stock.slot(8, 2, 2, cx=1, cy=-1, angle=30,
        face_selector=face, tool=ToolSpec("flat_endmill", 2), stepdown=0.5)
    # The result arcs come back from polygon vertices on the cutter circle (integer nm scale).
    assert stock.volume - result.volume == pytest.approx((12 + math.pi) * 2, abs=1e-4)
    assert not inside(result, point(1, -1, 7))
    assert inside(result, point(1, -1, 5))
    assert inside(result, point(5.5, 3, 7))
    record = result.process_plan()["operations"][-1]
    assert record["operation"] == record["toolpath"]["operation"] == "slot"
    assert record["toolpath"]["metadata"]["tool_axis"] == axis
    assert record["toolpath"]["metadata"]["face_selector"] == face
    assert len(record["material_passes"]) == 5
    assert len(result._state_history) == 6
    previous_volume = stock.volume
    for state, report in zip(result._state_history[1:], record["material_passes"]):
        assert state["shape"].val().isValid()
        current = state["shape"].val().Volume()
        assert current <= previous_volume + 1e-6
        # Layer removal (2D chords within 0.25 um) against the shown state (docs/MILLING_LAYER_REPLAY.md).
        assert report["removed_volume_mm3"] == pytest.approx(previous_volume - current, rel=1e-3)
        assert report["move_end_index"] > report["move_start_index"]
        previous_volume = current
    assert record["operation_target_comparison"]["remaining_target_material_mm3"] < 1e-6
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert record["manufacturing_ready"] is False
    assert record["verification_status"] == "not_verified"
    assert record["review_reasons"]


def test_smaller_owned_cutter_uses_boundary_and_raster_passes_without_ideal_slot_substitution():
    tools = [ToolSpec("flat_endmill", 2, catalog_id="OWNED-SLOT-2", physical_assembly={"owned": True})]
    with tool_inventory(tools, revision="slot-owned", strict=True):
        result = Stock.rectangular(14, 14, 8).slot(8, 4, 1, angle=35, cx=1, cy=-1)
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED-SLOT-2"
    assert record["tool_selection"]["inventory_revision"] == "slot-owned"
    assert {row["path_kind"] for row in record["material_passes"]} == {"raster", "boundary"}
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert any("linear segments" in reason for reason in record["review_reasons"])
    assert not inside(result, (1, -1, 3.5))
    assert inside(result, (-5, 4, 3.5))


def test_slot_preserves_translated_incoming_material_and_previous_hole():
    stock = Stock.rectangular(16, 16, 8)
    stock._shape = stock._shape.translate((20, -15, 10))
    stock._shape = stock._shape.cut(cq.Workplane("XY", origin=(20, -15, 5)).circle(0.5).extrude(10))
    previous_volume = stock.volume
    result = stock.slot(8, 2, 2, cx=20, cy=-15, tool=ToolSpec("flat_endmill", 2))
    assert stock.volume == pytest.approx(previous_volume)
    assert not inside(result, (20, -15, 7))
    assert not inside(result, (22, -15, 13))
    assert inside(result, (25, -15, 13))
    assert min(move["position"][2] for move in result.process_plan()["operations"][-1]["toolpath"]["moves"]) == 12


def test_through_slot_uses_actual_axis_thickness_and_checks_cutter_reach():
    stock = Stock.rectangular(10, 8, 6)
    result = stock.slot(4, 2, 1, through=True, face_selector=">X", tool=ToolSpec("flat_endmill", 2))
    assert stock.volume - result.volume == pytest.approx((4 + math.pi) * 10, abs=1e-6)
    assert result.process_plan()["operations"][-1]["depth_mm"] == 10
    with pytest.raises(ValueError, match="cannot reach"):
        stock.slot(4, 2, 1, through=True, face_selector=">X", tool=ToolSpec("flat_endmill", 2, flute_length=8))


@pytest.mark.parametrize("params,reason", [
    ({"length": 1, "width": 2}, "length"),
    ({"width": float("nan")}, "finite"),
    ({"angle": float("inf")}, "finite"),
    ({"depth": 0}, "positive"),
    ({"stepdown": 0}, "positive"),
    ({"finish_allowance": -1}, "non-negative"),
])
def test_invalid_slot_parameters_cannot_return_a_requested_shape(params, reason):
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(12, 12, 8).slot(**{"length": 8, "width": 2, "depth": 1, **params})


def test_oversized_and_missing_cutters_stop_slot_replay():
    stock = Stock.rectangular(12, 12, 8)
    with pytest.raises(ValueError, match="exceeds the slot width"):
        stock.slot(8, 2, 1, tool=ToolSpec("flat_endmill", 4))
    with tool_inventory([], strict=True), pytest.raises(ValueError, match="flat_endmill"):
        stock.slot(8, 2, 1)


def test_unapplied_slot_remains_a_planning_path_without_material_proof():
    operation = SlotOp(length=8, width=4, depth=1, tool=ToolSpec("flat_endmill", 2))
    assert operation.to_dict()["geometry_fidelity"] == "unavailable"
    assert operation.to_toolpath().metadata["material_replay"] == "unavailable"
