from copy import deepcopy
import math

import pytest
pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec
from src.subcad.turning_program import continuous_turning_plan


def tool(sign=1, **changes):
    inputs = {"rpm": 200, "feed_rate_mm_min": 120, "spindle_direction": sign, "spindle_phase_start_radians": .3}
    inputs.update(changes)
    return ToolSpec("turning_insert", 0, catalog_id="ENGINEERING-INSERT", inventory_count=1,
        preferred_feeds_speeds=inputs, physical_assembly={"owned": True, "cutting_geometry": {
            "source_sha256": "a" * 64, "polygon": [[-1, 0], [0, 0], [0, 20], [-1, 20]], "thickness_mm": .2}})


@pytest.mark.parametrize("axis", ["X", "Y", "Z", "-X", "-Y", "-Z"])
@pytest.mark.parametrize("sign", [-1, 1])
def test_consecutive_native_paths_keep_phase_and_record_transport(axis, sign):
    insert = tool(sign)
    first = Stock.round_bar(30, 20, axis=axis).turn_od(18, 30, axis=axis, tool=insert)
    final = first.turn_od(16, 30, axis=axis, tool=insert)
    before = deepcopy(final.process_plan())
    plan = final.process_plan(continuous_turning_phase=True)
    paths = [op["toolpath"] for op in plan["operations"]]
    assert plan["turning_program_clock"]["status"] == "ready"
    assert paths[1]["moves"][0]["position"] == paths[0]["moves"][-1]["position"]
    assert paths[1]["moves"][0]["metadata"]["spindle_phase_radians"] == paths[0]["moves"][-1]["metadata"]["spindle_phase_radians"]
    phase = .3
    for path in paths:
        for index, move in enumerate(path["moves"]):
            if index:
                previous = path["moves"][index - 1]
                rate = path["rapid_rate_mm_min"] if move["type"] in {"rapid", "retract"} else move["feed_mm_min"]
                phase += sign * 200 * math.tau * math.dist(move["position"], previous["position"]) / rate
            assert move["metadata"]["spindle_phase_radians"] == pytest.approx(phase)
    assert any(move["metadata"].get("program_transition") for move in paths[1]["moves"])
    assert final.process_plan() == before
    assert first.process_plan(continuous_turning_phase=True)["operations"] == plan["operations"][:1]


@pytest.mark.parametrize("change,reason", [
    (lambda op: op["tool"].update(catalog_id="OTHER"), "insert change"),
    (lambda op: op["toolpath"].update(spindle_rpm=300), "speed or direction"),
    (lambda op: op["toolpath"]["metadata"].update(spindle_direction=-1), "speed or direction"),
    (lambda op: op.update(setup="other"), "setup change"),
    (lambda op: op["toolpath"]["metadata"].update(spindle_phase_status="unavailable"), "explicit phase"),
])
def test_unknown_transitions_do_not_create_phase(change, reason):
    insert = tool()
    plan = Stock.round_bar(30, 20).turn_od(18, 30, tool=insert).turn_od(16, 30, tool=insert).process_plan()
    change(plan["operations"][1])
    output = continuous_turning_plan(plan)
    assert output["turning_program_clock"]["status"] == "partial"
    meta = output["operations"][1]["toolpath"]["metadata"]
    assert meta["spindle_phase_status"] == "unavailable"
    assert reason in meta["spindle_phase_reason"]


def test_missing_initial_phase_stays_unavailable():
    result = Stock.round_bar(30, 20).turn_od(18, 30, tool=tool(spindle_phase_start_radians=None))
    plan = result.process_plan(continuous_turning_phase=True)
    assert plan["turning_program_clock"]["status"] == "partial"
    assert all("spindle_phase_radians" not in move["metadata"] for move in plan["operations"][0]["toolpath"]["moves"])
