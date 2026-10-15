"""Native evidence for subtractive Turning and preservation of preceding features."""
import math

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock
from src.subcad.turning_geometry import from_spindle, to_spindle
from src.subcad.postprocessor import Generic3AxisDebugPost
from src.subcad.visualization import _export_stock_states
from src.subcad.operation_video import build_simulation_timeline, _operation_moves

AXES = ["X", "Y", "Z", "-X", "-Y", "-Z"]


@pytest.mark.parametrize("axis", AXES)
@pytest.mark.parametrize("origin", [(0, 0, 0), (7, -11, 23)])
def test_each_turning_operation_removes_the_expected_material_in_its_spindle_frame(axis, origin):
    stock = Stock.round_bar(30, 40, axis=axis, origin=origin)
    faced = stock.turn_face(10)
    outside = faced.turn_od(30, 25)
    bored = outside.turn_id(10, 10)
    grooved = bored.turn_groove(2, 1, -5)
    assert stock.volume == pytest.approx(math.pi * 20 ** 2 * 30)
    assert faced.volume == pytest.approx(math.pi * 20 ** 2 * 25)
    assert outside.volume == pytest.approx(math.pi * 15 ** 2 * 25)
    assert bored.volume == pytest.approx(math.pi * 15 ** 2 * 25 - math.pi * 5 ** 2 * 10)
    assert bored.volume - grooved.volume == pytest.approx(math.pi * (15 ** 2 - 14 ** 2) * 2)
    assert grooved._shape.val().isValid()
    local = to_spindle(bored._shape, axis, origin)
    assert not local.val().isInside(cq.Vector(0, 0, 5))
    assert local.val().isInside(cq.Vector(0, 0, -5))
    assert len(grooved._state_history) == 5
    assert [state["shape"].val().Volume() for state in grooved._state_history] == pytest.approx(
        [stock.volume, faced.volume, outside.volume, bored.volume, grooved.volume])


@pytest.mark.parametrize("axis", AXES)
def test_profile_cut_is_subtractive_and_does_not_replace_the_incoming_stock(axis):
    stock = Stock.round_bar(30, 20, axis=axis)
    result = stock.turn_profile({"type": "frustum", "bottom_diameter": 30, "top_diameter": 10, "height": 30})
    assert result.volume == pytest.approx(2375 * math.pi)
    assert result.volume < stock.volume
    local = to_spindle(result._shape, axis)
    assert not local.val().isInside(cq.Vector(12, 0, -10))


def test_turning_after_milling_preserves_the_actual_hole_and_each_previous_state():
    first_turn = Stock.round_bar(30, 40).turn_od(30, 30)
    milled = first_turn.drill(diameter=4, depth=30, through=True, cx=8, cy=0)
    final_turn = milled.turn_od(26, 30)
    assert final_turn.volume == pytest.approx(math.pi * (13 ** 2 - 2 ** 2) * 30)
    assert not final_turn._shape.val().isInside(cq.Vector(8, 0, 0))
    assert final_turn._shape.val().isInside(cq.Vector(0, 0, 0))
    assert len(final_turn._state_history) == len(milled._state_history) + 1
    assert final_turn._state_history[-2]["shape"].val().Volume() == pytest.approx(milled.volume)


def test_profile_band_preserves_material_outside_the_requested_axial_span():
    result = Stock.round_bar(30, 40).turn_profile({"stations": [
        {"z_mm": -5, "diameter_mm": 24}, {"z_mm": 5, "diameter_mm": 24}]})
    assert not result._shape.val().isInside(cq.Vector(19, 0, 0))
    assert result._shape.val().isInside(cq.Vector(19, 0, 10))
    assert result.volume == pytest.approx(math.pi * 20 ** 2 * 30 - math.pi * (20 ** 2 - 12 ** 2) * 10)


def test_internal_groove_preserves_the_preceding_bore():
    stock = Stock.round_bar(30, 40).turn_id(20, 30)
    result = stock.turn_groove(2, 1, 0, diameter=20, internal=True)
    assert stock.volume - result.volume == pytest.approx(math.pi * (11 ** 2 - 10 ** 2) * 2)
    assert not result._shape.val().isInside(cq.Vector(0, 0, 0))


@pytest.mark.parametrize("internal", [False, True])
@pytest.mark.parametrize("left_handed", [False, True])
def test_generic_threads_create_helical_material_removal_and_retain_their_limits(internal, left_handed):
    stock = Stock.round_bar(30, 40)
    if internal:
        stock = stock.turn_id(19, 10)
        diameter = 20
    else:
        stock = stock.turn_od(30, 30)
        diameter = 30
    result = stock.turn_thread(diameter, 2, 6, internal=internal, thread_depth=0.5, left_handed=left_handed)
    assert 0 < result.volume < stock.volume
    assert result._shape.val().isValid()
    record = result.process_plan()["operations"][-1]
    assert record["geometry_fidelity"] == "approximate"
    assert record["thread_standard_verification"] == "not_run"
    assert record["left_handed"] == left_handed


def test_thread_hand_changes_the_material_at_the_expected_helix_phase():
    stock = Stock.round_bar(30, 30)
    right = stock.turn_thread(30, 2, 6, thread_depth=0.5)
    left = stock.turn_thread(30, 2, 6, thread_depth=0.5, left_handed=True)
    point = cq.Vector(0, 14.7, 9.5)
    assert stock._shape.val().isInside(point)
    assert not right._shape.val().isInside(point)
    assert left._shape.val().isInside(point)


def test_threads_require_preceding_diameter_or_bore_operations():
    stock = Stock.round_bar(30, 40)
    with pytest.raises(ValueError, match="outside-diameter"):
        stock.turn_thread(30, 2, 6, thread_depth=0.5)
    with pytest.raises(ValueError, match="preceding bore"):
        stock.turn_thread(20, 2, 6, thread_depth=0.5, internal=True)


@pytest.mark.parametrize("operation", [
    lambda stock: stock.turn_face(float("nan")),
    lambda stock: stock.turn_face(-100),
    lambda stock: stock.turn_od(-1, 20),
    lambda stock: stock.turn_id(10, -1),
    lambda stock: stock.turn_groove(2, 100, 0),
    lambda stock: stock.turn_profile({}),
    lambda stock: stock.turn_profile([(5, 0), (5, 10), (5, 5)]),
    lambda stock: stock.turn_thread(40, 0, 10),
    lambda stock: stock.turn_thread(40, 0.001, 10),
])
def test_invalid_operations_cannot_return_an_unchanged_success_state(operation):
    with pytest.raises(ValueError):
        operation(Stock.round_bar(30, 40))


def test_operation_plans_use_spindle_paths_and_do_not_claim_measured_tolerance_or_owned_tools():
    result = Stock.round_bar(30, 40, axis="X").turn_od(30, 30)
    record = result.process_plan()["operations"][-1]
    assert record["axis"] == "X"
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["requires_owned_tool"]
    path = record["toolpath"]
    assert path["metadata"]["process"] == "turn"
    assert path["metadata"]["strategy"] == "outside_diameter"
    assert path["metadata"]["requires_feed_speed_release"]
    assert all(move["position"][1] == 0 for move in path["moves"])
    assert path["metadata"]["verification_status"] == "not_verified"


def test_step_profile_toolpath_preserves_the_order_at_an_axial_shoulder():
    profile = [(20, -10), (20, 0), (10, 0), (10, 10)]
    result = Stock.round_bar(30, 40).turn_profile(profile)
    moves = result.process_plan()["operations"][-1]["toolpath"]["moves"]
    cutting = [(move["metadata"]["spindle_radius_mm"], move["metadata"]["spindle_z_mm"])
        for move in moves if move["type"] == "feed"]
    assert cutting == list(reversed(profile))


def test_a_milling_post_cannot_emit_turning_moves():
    part = Stock.round_bar(30, 40).turn_od(30, 30)
    posted = Generic3AxisDebugPost().postprocess(part.process_plan())
    assert "Turning requires a lathe postprocessor" in posted
    assert "G1 " not in posted


def test_exported_material_states_have_real_mesh_identity_and_native_volume(tmp_path):
    import hashlib
    part = Stock.round_bar(30, 40).turn_face(10).turn_od(30, 25)
    states = _export_stock_states(part, tmp_path, part.process_plan())
    assert len(states) == 3
    assert [state["volume_mm3"] for state in states] == pytest.approx([
        math.pi * 400 * 30, math.pi * 400 * 25, math.pi * 225 * 25])
    for state in states:
        assert state["sha256"] == hashlib.sha256((tmp_path / state["path"]).read_bytes()).hexdigest()
        assert state["coordinate_frame"]["basis"] == "stock_world"
    assert states[-1]["verification_status"] == "not_verified"


def test_mixed_route_timeline_uses_each_native_material_result_without_claiming_tool_replay(tmp_path):
    import hashlib
    first = Stock.round_bar(30, 40).turn_od(30, 30)
    milled = first.drill(diameter=4, depth=30, through=True, cx=8, cy=0)
    final = milled.turn_od(26, 30)
    timeline = build_simulation_timeline(final, output_dir=tmp_path, max_keyframes=20)
    frames = timeline["stock_keyframes"]
    assert len(frames) == len(final._state_history)
    assert frames[-1]["volume_mm3"] == pytest.approx(final.volume)
    assert frames[-2]["volume_mm3"] == pytest.approx(milled.volume)
    assert all(frame["role"] == "native_operation_stock_state" for frame in frames)
    assert all(frame["sha256"] == hashlib.sha256((tmp_path / frame["path"]).read_bytes()).hexdigest() for frame in frames)
    assert timeline["fidelity"]["stock_geometry"] == "native_operation_geometry"
    assert timeline["fidelity"]["independent_tool_replay"] == "not_run"
    assert timeline["timeline"]["timing_status"] == "preview_estimate"


def test_turning_world_coordinates_do_not_receive_a_second_top_face_offset():
    part = Stock.round_bar(30, 40, axis="X").turn_od(30, 30)
    record = part.process_plan()["operations"][-1]
    moves = _operation_moves(record, {"z_max": 20})
    assert moves[0]["position"] == pytest.approx(record["toolpath"]["moves"][0]["position"], abs=1e-6)
