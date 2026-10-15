import math

import pytest

from src.subcad import Stock, ToolSpec, tool_inventory
from test_subcad_hole_finishing import FACES, pilot, reamer


def tools():
    return [reamer(catalog_id="OWNED-REAMER-4"), ToolSpec("drill", 3.8, tip_angle=118, catalog_id="OWNED-PILOT-3_8"),
        ToolSpec("drill", 4, tip_angle=118, catalog_id="FINAL-SIZE-DRILL")]


@pytest.mark.parametrize("face,axis", FACES)
def test_ream_hole_selects_actual_pilot_and_retains_each_tool_and_material(face, axis):
    initial = Stock.rectangular(10, 10, 10)
    with tool_inventory(tools(), revision="r1", strict=True):
        result = initial.ream_hole(4, through=True, cx=1, cy=-1, face_selector=face)
    records = result.process_plan()["operations"]
    assert [op["operation"] for op in records] == ["drill", "ream"]
    assert [op["tool"]["catalog_id"] for op in records] == ["OWNED-PILOT-3_8", "OWNED-REAMER-4"]
    assert all(op["tool_selection"]["ok"] and op["tool_selection"]["inventory_revision"] == "r1" for op in records)
    assert all(op["toolpath"]["metadata"]["tool_axis"] == axis for op in records)
    assert len(result._state_history) == 3
    assert initial.volume - result.volume == pytest.approx(math.pi * 4 * 10, abs=1e-6)
    assert records[1]["pilot_preparation"]["diameter_mm"] == 3.8
    assert records[1]["pilot_preparation"]["tool_selection"]["inventory_revision"] == "r1"
    assert records[1]["pilot_preparation"]["allowance_qualified"] is False


def test_existing_clear_pilot_needs_no_new_drill_and_preserves_its_bottom():
    initial = pilot(Stock.rectangular(12, 12, 12))
    with tool_inventory([reamer(catalog_id="REAM")], strict=True):
        result = initial.ream_hole(4, 2, cx=1, cy=-1)
    assert len(result.process_plan()["operations"]) == 2
    assert result.process_plan()["operations"][-1]["pilot_preparation"]["method"] == "existing_material_hole"
    assert initial.volume > result.volume


@pytest.mark.parametrize("available,reason", [
    ([tools()[0], tools()[2]], "available pilot drill"),
    ([tools()[1]], "available reamer"),
    ([ToolSpec("reamer", 4, catalog_id="UNKNOWN"), tools()[1]], "recorded lead angle"),
    ([tools()[0], ToolSpec("drill", 3.5, tip_angle=118, catalog_id="SMALL")], "available pilot drill"),
    ([tools()[0], ToolSpec("drill", 3.8, tip_angle=118, flute_length=10.1, catalog_id="SHORT")], "complete drill point"),
])
def test_missing_required_tool_or_lead_cannot_return_a_finished_hole(available, reason):
    initial = Stock.rectangular(10, 10, 10)
    original_volume = initial.volume
    with tool_inventory(available, strict=True), pytest.raises(ValueError, match=reason):
        initial.ream_hole(4, through=True)
    assert initial.volume == original_volume
    assert not initial.process_plan()["operations"]


def test_outside_material_position_cannot_become_a_finished_reaming_result():
    initial = Stock.rectangular(10, 10, 10)
    with tool_inventory(tools(), strict=True), pytest.raises(ValueError, match="no material"):
        initial.ream_hole(4, through=True, cx=100)


def test_blind_reaming_depth_uses_the_recorded_entry_plane_inside_stock_allowance():
    initial = Stock.rectangular(12, 12, 12)
    with tool_inventory(tools(), strict=True):
        result = initial.ream_hole(4, 2, entry_plane_mm=4)
    record = result.process_plan()["operations"][-1]
    assert record["depth_mm"] == 4
    assert record["toolpath"]["metadata"]["full_diameter_depth_mm"] == 4
    assert any("floor requires review" in reason for reason in record["review_reasons"])
    assert any("pilot drill point extends" in reason for reason in record["review_reasons"])
    with tool_inventory(tools(), strict=True), pytest.raises(ValueError, match="entry plane lies outside"):
        initial.ream_hole(4, 2, entry_plane_mm=7)
