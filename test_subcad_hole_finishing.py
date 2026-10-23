import math

import cadquery as cq
import pytest

from src.subcad import Stock, ToolSpec, tool_inventory, replay_milling_independently
from src.subcad.operations import BoreOp
from src.subcad.toolpath import Toolpath


FACES = [(">Z", [0, 0, 1]), ("<Z", [0, 0, -1]), (">X", [1, 0, 0]),
    ("<X", [-1, 0, 0]), (">Y", [0, 1, 0]), ("<Y", [0, -1, 0])]


def reamer(**params):
    return ToolSpec("reamer", 4, tip_angle=90, tip_diameter=3.6, **params)


def pilot(stock, face=">Z", *, diameter=3.8, depth=3, through=False):
    return stock.drill(diameter, depth, cx=1, cy=-1, through=through, face_selector=face,
        spot_drill=False, tool=ToolSpec("drill", diameter, tip_angle=118))


def assert_retained(result, before, operation, axis):
    record = result.process_plan()["operations"][-1]
    assert record["operation"] == operation
    assert record["toolpath"]["metadata"]["tool_axis"] == axis
    assert record["material_passes"]
    previous = before.volume
    states = result._state_history[-len(record["material_passes"]):]
    for state, report in zip(states, record["material_passes"]):
        shape = state["shape"].val()
        assert shape.isValid()
        assert shape.Volume() <= previous + 1e-6
        # Layer removal (2D chords within 0.25 um) against the shown state (docs/MILLING_LAYER_REPLAY.md).
        assert report["removed_volume_mm3"] == pytest.approx(previous - shape.Volume(), abs=1e-3)
        assert report["move_end_index"] > report["move_start_index"]
        previous = shape.Volume()
    assert record["manufacturing_ready"] is False
    assert record["review_reasons"]
    return record


@pytest.mark.parametrize("face,axis", FACES)
def test_reaming_uses_the_recorded_lead_and_pilot_on_six_faces(face, axis):
    before = pilot(Stock.rectangular(12, 12, 12), face)
    result = before.ream(4, 2, cx=1, cy=-1, face_selector=face, tool=reamer())
    record = assert_retained(result, before, "ream", axis)
    assert record["face_selector"] == face
    assert record["geometry_fidelity"] == "rotational_reamer_envelope"
    assert record["toolpath"]["metadata"]["tip_depth_mm"] == pytest.approx(2.2)
    assert record["toolpath"]["metadata"]["pilot_clearance"] == "native_brep_checked"
    assert result.volume < before.volume
    # The incoming pilot below the reamed depth remains in the result.
    from src.subcad.profile_milling import _setup_plane
    plane, mirror = _setup_plane(face)
    center = plane.toWorldCoords((1, -mirror, 3.5))
    assert not result._shape.val().isInside(center)


@pytest.mark.parametrize("face,axis", FACES)
def test_circular_milling_retains_actual_cutter_passes_on_six_faces(face, axis):
    stock = Stock.rectangular(12, 12, 12)
    before = pilot(stock, face, diameter=2, depth=3)
    result = before.bore(4, 2, cx=1, cy=-1, face_selector=face,
        tool=ToolSpec("flat_endmill", 2), stepdown=1)
    record = assert_retained(result, before, "bore", axis)
    assert record["face_selector"] == face
    assert record["strategy"] == "circular_milling"
    assert record["geometry_fidelity"] == "linearized_profile_cutter_replay"
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert result.volume < before.volume
    assert stock.volume == pytest.approx(1728)


def test_through_reamer_uses_actual_axis_thickness_and_preserves_an_unrelated_hole():
    stock = Stock.rectangular(10, 8, 6).drill(1, 1, cx=-3, cy=2, spot_drill=False,
        tool=ToolSpec("drill", 1, tip_angle=118))
    before = pilot(stock, ">X", through=True)
    result = before.ream(4, through=True, cx=1, cy=-1, face_selector=">X", tool=reamer())
    assert before.volume - result.volume == pytest.approx(math.pi * (4 ** 2 - 3.8 ** 2) / 4 * 10, abs=1e-6)
    assert not result._shape.val().isInside(cq.Vector(-3, 2, 2.5))
    assert result.process_plan()["operations"][-1]["depth_mm"] == 10


def test_through_circular_milling_selects_owned_tool_by_axis_thickness():
    short = ToolSpec("flat_endmill", 4, catalog_id="SHORT", flute_length=8, stickout_mm=8)
    correct = ToolSpec("flat_endmill", 3, catalog_id="OWNED", flute_length=15)
    with tool_inventory([short, correct], strict=True):
        result = Stock.rectangular(10, 8, 6).bore(4, through=True, face_selector=">X", stepdown=3)
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED"
    assert record["depth_mm"] == 10
    assert record["toolpath"]["metadata"]["tool_axis"] == [1, 0, 0]


@pytest.mark.parametrize("diameter,depth,reason", [(None, 3, "pilot hole"), (3, 3, "pilot hole"), (3.8, 1, "pilot hole")])
def test_reaming_requires_clear_pilot_for_its_complete_lead_travel(diameter, depth, reason):
    stock = Stock.rectangular(12, 12, 12)
    before = stock if diameter is None else pilot(stock, diameter=diameter, depth=depth)
    volume = before.volume
    with pytest.raises(ValueError, match=reason):
        before.ream(4, 2, cx=1, cy=-1, tool=reamer())
    assert before.volume == volume


@pytest.mark.parametrize("tool,reason", [
    (ToolSpec("drill", 4, tip_angle=118), "cutter type"),
    (ToolSpec("reamer", 4), "lead angle"),
    (ToolSpec("reamer", 4, tip_angle=90), "lead tip diameter"),
    (ToolSpec("reamer", 5, tip_angle=90, tip_diameter=3.6), "does not match"),
    (reamer(flute_length=2.1), "cannot reach"),
])
def test_invalid_reamer_cannot_return_an_ideal_finished_hole(tool, reason):
    before = pilot(Stock.rectangular(12, 12, 12))
    with pytest.raises(ValueError, match=reason):
        before.ream(4, 2, cx=1, cy=-1, tool=tool)


def test_reamer_selection_does_not_use_a_wrong_diameter_or_unknown_lead():
    wrong = ToolSpec("reamer", 5, tip_angle=90, tip_diameter=3.6, catalog_id="WRONG")
    unknown = ToolSpec("reamer", 4, catalog_id="UNKNOWN")
    valid = reamer(catalog_id="OWNED")
    before = pilot(Stock.rectangular(12, 12, 12))
    with tool_inventory([wrong, unknown, valid], strict=True):
        result = before.ream(4, 2, cx=1, cy=-1)
    assert result.process_plan()["operations"][-1]["tool"]["catalog_id"] == "OWNED"
    with tool_inventory([wrong, unknown], strict=True), pytest.raises(ValueError, match="recorded lead angle"):
        before.ream(4, 2, cx=1, cy=-1)


def test_independent_reamer_replay_uses_the_incoming_pilot():
    before = pilot(Stock.rectangular(6, 6, 6))
    tool = reamer()
    result = before.ream(4, 2, cx=1, cy=-1, tool=tool)
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    model, report = replay_milling_independently(before, path, tool, resolution_mm=0.5, target=result)
    assert not model.get_material_at((1, -1, 2))
    assert model.get_material_at((-2.5, 2.5, 2))
    assert report["target_comparison"]["overcut_samples"] == 0
    assert report["manufacturing_ready"] is False


def test_unapplied_bore_has_no_synthetic_toolpath_or_material_result():
    op = BoreOp(diameter=4, depth=2, tool=ToolSpec("flat_endmill", 2))
    with pytest.raises(ValueError, match="retained cutter path"):
        op.to_toolpath()
    assert op.to_dict()["geometry_fidelity"] == "unavailable"
