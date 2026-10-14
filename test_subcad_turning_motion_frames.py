"""Native spindle frames and explicitly supplied spindle phase remain traceable."""
import math
import numpy as np
import pytest

pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec
from src.subcad.turning_geometry import spindle_point_to_world, stock_to_spindle_matrix


@pytest.mark.parametrize("axis", ["X", "Y", "Z", "-X", "-Y", "-Z"])
def test_spindle_matrix_matches_native_points_and_has_no_reflection(axis):
    matrix = np.array(stock_to_spindle_matrix(axis, (3, -2, 7))).reshape(4, 4, order="F")
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1)
    for radius, axial in ((0, 0), (4, -8), (10, 12)):
        world = spindle_point_to_world(radius, axial, axis, (3, -2, 7))
        np.testing.assert_allclose(matrix @ [*world, 1], [radius, 0, axial, 1], atol=1e-9)


def tool(sign=1, **changes):
    fs = {"rpm": 200, "feed_rate_mm_min": 120, "spindle_direction": sign, "spindle_phase_start_radians": .3}
    fs.update(changes)
    return ToolSpec("turning_insert", 0, catalog_id="ENGINEERING-TEST-INSERT", inventory_count=1,
        preferred_feeds_speeds=fs, physical_assembly={"owned": True})


@pytest.mark.parametrize("axis", ["X", "Y", "Z", "-X", "-Y", "-Z"])
def test_phase_uses_the_authored_world_distance_feed_and_rapid_rate(axis):
    result = Stock.round_bar(20, 20, axis=axis).turn_od(16, 12, axis=axis, tool=tool())
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert path["metadata"]["spindle_phase_status"] == "recorded"
    assert path["metadata"]["roughing_passes"] == "not_generated"
    phase = .3
    for index, move in enumerate(path["moves"]):
        if index:
            rate = path["rapid_rate_mm_min"] if move["type"] in {"rapid", "retract"} else move["feed_mm_min"]
            phase += math.dist(path["moves"][index - 1]["position"], move["position"]) / rate * 200 * math.tau
        assert move["metadata"]["spindle_phase_radians"] == pytest.approx(phase)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("left", [False, True])
def test_thread_feed_matches_the_explicit_rotation_and_thread_hand(sign, left):
    result = Stock.round_bar(20, 20).turn_thread(20, 2, 8, thread_depth=.8, left_handed=left, tool=tool(sign))
    path = result.process_plan()["operations"][-1]["toolpath"]
    moves = path["moves"]
    for before, after in zip(moves, moves[1:]):
        if after["type"] != "feed":
            continue
        dz = after["metadata"]["spindle_z_mm"] - before["metadata"]["spindle_z_mm"]
        phase = after["metadata"]["spindle_phase_radians"] - before["metadata"]["spindle_phase_radians"]
        assert dz / phase == pytest.approx((1 if left else -1) * 2 / math.tau)


def test_missing_phase_does_not_create_a_default_spindle_angle():
    result = Stock.round_bar(20, 20).turn_od(16, 12, tool=tool(spindle_phase_start_radians=None))
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert path["metadata"]["spindle_phase_status"] == "unavailable"
    assert all("spindle_phase_radians" not in move["metadata"] for move in path["moves"])
