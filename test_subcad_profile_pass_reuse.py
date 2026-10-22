"""Repeated cutter paths retain motion and the same actual material difference."""
from copy import deepcopy

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import ToolSpec
from src.subcad.milling_replay import replay_pass
from src.subcad.profile_pocket_milling import run_profile_pocket
from src.subcad.toolpath import Toolpath


@pytest.mark.parametrize("center,with_hole", [((0, 0), False), ((20, -15), True)])
def test_each_retained_pass_matches_independent_selected_cutter_replay(center, with_hole):
    x, y = center
    incoming = cq.Workplane("XY").box(10, 10, 4).translate((x, y, 6))
    if with_hole:
        incoming = incoming.cut(cq.Workplane("XY", origin=(x, y, 3)).circle(0.5).extrude(6))
    original = incoming.val().Volume()
    tool = ToolSpec("flat_endmill", 2, flute_length=6, stickout_mm=8)
    result, path, states, reports, _ = run_profile_pocket(incoming,
        {"type": "rectangle", "length": 6, "width": 6, "cx": x, "cy": y},
        tool=tool, depth=1, stepdown=1, finish_allowance=0, chord_error=0.01, feed=100, rpm=1000)
    assert any(report.get("repeated_pass_reused") for report in reports)
    assert {report["pass_role"] for report in reports} == {"roughing", "finishing"}
    assert len(states) == len(reports)
    reference = incoming
    for state, report in zip(states, reports):
        moves = path.moves[report["move_start_index"]:report["move_end_index"]]
        independent = Toolpath("profile_pocket", moves=deepcopy(moves), metadata={
            "coordinate_system": "part_mm", "tool_axis": [0, 0, 1]})
        reference, _ = replay_pass(reference, independent, tool)
        if state is states[-1]:
            assert reference.val().cut(state["shape"].val()).Volume() < 1e-5
            assert state["shape"].val().cut(reference.val()).Volume() < 1e-5
        else:
            # An earlier state shows the result plus extruded layer polygons (docs/MILLING_LAYER_REPLAY.md):
            # its volume and extent match the B-Rep replay within the 2D chord tolerance.
            assert state["shape"].val().Volume() == pytest.approx(reference.val().Volume(), abs=5e-3)
            for axis in ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax"):
                assert getattr(state["shape"].val().BoundingBox(), axis) == pytest.approx(
                    getattr(reference.val().BoundingBox(), axis), abs=1e-3)
        if report.get("repeated_pass_reused"):
            assert report["reuse_proof"] == "set_difference_idempotence"
            assert report["reused_from_pass_index"] < report["pass_index"]
            assert report["removed_volume_mm3"] == 0
        assert report["manufacturing_ready"] is False
    # Layer removal: mesh chords of the incoming hole (1 um) and cutter chords (0.25 um), times their wall area.
    assert original - result.val().Volume() == pytest.approx(sum(row["removed_volume_mm3"] for row in reports), abs=5e-3)
    assert incoming.val().Volume() == original
    if with_hole:
        assert not result.val().isInside(cq.Vector(x, y, 5))
