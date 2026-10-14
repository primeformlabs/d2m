import math
import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec, tool_inventory


def test_facing_uses_actual_cutter_passes_and_matches_both_part_datums():
    stock = Stock.rectangular(12, 10, 12)
    tool = ToolSpec("flat_endmill", 4)
    first = stock.face_mill(2, tool=tool)
    final = first.face_mill(2, tool=tool, face_selector="<Z")
    assert final.volume == pytest.approx(12 * 10 * 8, abs=1e-5)
    assert final.bounding_box["z_min"] == pytest.approx(-4)
    assert final.bounding_box["z_max"] == pytest.approx(4)
    assert first.bounding_box["z_max"] == pytest.approx(4)
    record = final.process_plan()["operations"][-1]
    assert record["toolpath"]["metadata"]["tool_axis"] == [0, 0, -1]
    assert len(record["material_passes"]) > 2
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6


@pytest.mark.parametrize("face,key,positive", [(">X", "x", True), ("<X", "x", False),
    (">Y", "y", True), ("<Y", "y", False), (">Z", "z", True), ("<Z", "z", False)])
def test_cardinal_facing_preserves_axis_coordinates(face, key, positive):
    stock = Stock.rectangular(12, 10, 8)
    result = stock.face_mill(1, tool=ToolSpec("flat_endmill", 4), face_selector=face)
    bound = "max" if positive else "min"
    opposite = "min" if positive else "max"
    assert result.bounding_box[f"{key}_{bound}"] == pytest.approx(stock.bounding_box[f"{key}_{bound}"] + (-1 if positive else 1))
    assert result.bounding_box[f"{key}_{opposite}"] == pytest.approx(stock.bounding_box[f"{key}_{opposite}"])


def test_offset_hole_does_not_move_the_bottom_facing_origin_or_leave_a_strip():
    stock = Stock.rectangular(12, 10, 8)
    stock._shape = stock._shape.cut(cq.Workplane("XY", origin=(3, 2, -5)).circle(2).extrude(10))
    result = stock.face_mill(1, tool=ToolSpec("flat_endmill", 4), face_selector="<Z")
    assert result.volume == pytest.approx((120 - math.pi * 4) * 7, abs=1e-5)
    assert result.bounding_box["z_min"] == pytest.approx(-3)
    assert not result._shape.val().isInside(cq.Vector(3, 2, 0))


def test_owned_face_mill_retains_its_identity_and_declares_insert_envelope_limits():
    tool = ToolSpec("face_mill", 12, catalog_id="OWNED-FACE", physical_assembly={"owned": True})
    with tool_inventory([tool], revision="face-inventory", strict=True):
        result = Stock.rectangular(10, 10, 8).face_mill(1)
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED-FACE"
    assert record["toolpath"]["tool_type"] == "face_mill"
    assert record["geometry_fidelity"] == "nominal_face_mill_envelope"
    assert any("Insert geometry and runout" in reason for reason in record["review_reasons"])
    assert result.volume == pytest.approx(700, abs=1e-5)


def test_requested_facing_stepover_cannot_exceed_the_saved_cutter_limit():
    tool = ToolSpec("flat_endmill", 4, safe_limits={"max_stepover_mm": 1})
    result = Stock.rectangular(8, 8, 4).face_mill(1, tool=tool, stepover=3)
    path = result.process_plan()["operations"][-1]["toolpath"]
    rows = sorted({move["position"][1] for move in path["moves"] if move["type"] == "plunge"})
    assert len(rows) > 4
    assert max(after - before for before, after in zip(rows, rows[1:])) <= 1 + 1e-6
    assert result.volume == pytest.approx(8 * 8 * 3, abs=1e-5)
