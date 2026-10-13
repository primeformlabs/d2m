"""The rough pocket cannot contain material removal from a later finish tool."""
import math
import pytest

pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec


@pytest.mark.parametrize("face", [">X", "<X", ">Y", "<Y", ">Z", "<Z"])
def test_rough_and_finish_have_distinct_material_geometry_and_preserve_the_access_face(face):
    stock = Stock.rectangular(30, 30, 30)
    result = stock.pocket(width=10, length=12, depth=4, corner_radius=1,
        face_selector=face, tool=ToolSpec("flat_endmill", 6))
    history = result._state_history
    assert len(history) == 3
    rough_volume = history[1]["shape"].val().Volume()
    assert stock.volume - rough_volume == pytest.approx((120 - (4 - math.pi) * 3 ** 2) * 4)
    assert stock.volume - result.volume == pytest.approx((120 - (4 - math.pi)) * 4)
    assert result.volume < rough_volume < stock.volume
    operations = result.process_plan()["operations"]
    assert [op["operation"] for op in operations] == ["rough_pocket", "finish_pocket"]
    assert all(op["face_selector"] == face for op in operations)
    assert "finish_toolpath" not in operations[0]["toolpath"]["metadata"]
    assert operations[0]["toolpath"]["metadata"]["pass_plan"]["finishing_passes"] == 0
    assert operations[1]["toolpath"]["metadata"]["pass_plan"]["finishing_passes"] > 0
    assert operations[1]["toolpath"]["metadata"]["pass_plan"]["roughing_passes"] == 0


def test_milling_does_not_claim_a_sharp_internal_corner_from_a_round_cutter():
    stock = Stock.rectangular(30, 30, 20)
    result = stock.pocket(width=10, length=12, depth=4, corner_radius=0, tool=ToolSpec("flat_endmill", 6))
    assert stock.volume - result.volume == pytest.approx((120 - (4 - math.pi) * 3 ** 2) * 4)
    record = result.process_plan()["operations"][-1]
    assert record["geometry_status"] == "requires_review"
    assert record["modeled_corner_radius_mm"] == 3
    assert record["geometry_review_reasons"]


def test_a_smaller_required_finish_tool_is_not_replaced_with_an_oversized_tool():
    result = Stock.rectangular(30, 30, 20).pocket(width=10, length=12, depth=4, corner_radius=0.5,
        tool=ToolSpec("flat_endmill", 6))
    record = result.process_plan()["operations"][-1]
    assert record["tool_diameter_mm"] <= 1
    assert record["modeled_corner_radius_mm"] == pytest.approx(0.5)
