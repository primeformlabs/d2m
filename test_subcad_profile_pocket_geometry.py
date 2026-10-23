"""Profile pockets must follow actual cutter paths instead of ideal cuts."""
import math
from copy import deepcopy

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec, tool_inventory, replay_milling_independently
from src.subcad.toolpath import Toolpath


def inside(stock, point):
    return stock._shape.val().isInside(cq.Vector(*point))


def test_rectangular_profile_leaves_selected_cutter_corner_material_and_retains_passes():
    stock = Stock.rectangular(20, 20, 8)
    result = stock.profile_pocket({"type": "rectangle", "length": 12, "width": 10}, 2,
        tool=ToolSpec("flat_endmill", 4))
    removed = (120 - (4 - math.pi) * 4) * 2
    assert stock.volume - result.volume == pytest.approx(removed, abs=1e-6)
    assert inside(result, (5.9, 4.9, 3))
    assert not inside(result, (0, 0, 3))
    record = result.process_plan()["operations"][-1]
    assert len(result._state_history) == len(record["material_passes"]) + 1
    assert {row["pass_role"] for row in record["material_passes"]} == {"roughing", "finishing"}
    # Layer removal: 2D chords within 0.25 um of the cutter arcs (docs/MILLING_LAYER_REPLAY.md).
    assert sum(row["removed_volume_mm3"] for row in record["material_passes"]) == pytest.approx(removed, rel=1e-4)
    # The layer check: cutter chords within 0.25 um leave a little more corner material (docs/MILLING_LAYER_REPLAY.md).
    assert record["operation_target_comparison"]["remaining_target_material_mm3"] == pytest.approx((4 - math.pi) * 8, rel=1e-3)
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["geometry_status"] == "requires_review"
    assert any("target cavity material" in reason for reason in record["review_reasons"])
    assert result._state_history[-1]["shape"].val().isSame(result._shape.val())


@pytest.mark.parametrize("profile,material,void", [
    ({"type": "circle", "diameter": 10}, (4.8, 4.8, 3), (0, 0, 3)),
    ({"type": "slot", "length": 12, "width": 6, "angle": 30}, (-5, 4, 3), (0, 0, 3)),
    ({"type": "polygon", "points": [[-5, -4], [5, -4], [0, 4]]}, (4, 3, 3), (0, -1, 3)),
    ({"type": "rectangle", "length": 12, "width": 10, "corner_radius": 2, "angle": 20}, (6, 5, 3), (0, 0, 3)),
])
def test_profile_geometry_and_paths_use_the_actual_boundary(profile, material, void):
    stock = Stock.rectangular(18, 18, 8)
    result = stock.profile_pocket(profile, 2, tool=ToolSpec("flat_endmill", 2))
    assert inside(result, material)
    assert not inside(result, void)
    record = result.process_plan()["operations"][-1]
    assert record["toolpath"]["metadata"]["coordinate_system"] == "part_mm"
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-5
    assert record["material_passes"]


def test_island_geometry_and_clipped_boundary_paths_do_not_cut_the_retained_island():
    stock = Stock.rectangular(20, 20, 8)
    result = stock.profile_pocket({"type": "rectangle", "length": 16, "width": 16}, 2,
        islands=[{"type": "rectangle", "length": 8, "width": 8}], tool=ToolSpec("flat_endmill", 2))
    assert inside(result, (0, 0, 3))
    assert inside(result, (3.99, 3.99, 3))
    assert not inside(result, (6, 0, 3))
    record = result.process_plan()["operations"][-1]
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert stock.volume - result.volume == pytest.approx((256 - 64 - (4 - math.pi)) * 2, abs=1e-5)


def test_island_can_meet_the_outer_boundary_without_an_unclipped_contour_cut():
    stock = Stock.rectangular(20, 20, 8)
    result = stock.profile_pocket({"type": "rectangle", "length": 16, "width": 16}, 1,
        islands=[{"type": "rectangle", "length": 6, "width": 16}], tool=ToolSpec("flat_endmill", 2))
    assert inside(result, (0, 7.5, 3.5))
    assert not inside(result, (5, 0, 3.5))
    assert result.process_plan()["operations"][-1]["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6


@pytest.mark.parametrize("face,axis,point", [
    (">Z", [0, 0, 1], lambda u, v, w: (u, v, w)),
    ("<Z", [0, 0, -1], lambda u, v, w: (u, v, -w)),
    (">X", [1, 0, 0], lambda u, v, w: (w, u, v)),
    ("<X", [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    (">Y", [0, 1, 0], lambda u, v, w: (u, w, v)),
    ("<Y", [0, -1, 0], lambda u, v, w: (u, -w, v)),
])
def test_six_faces_preserve_profile_coordinates_and_cutter_orientation(face, axis, point):
    stock = Stock.rectangular(12, 12, 12)
    result = stock.profile_pocket({"type": "rectangle", "length": 4, "width": 4, "cx": 1, "cy": -1}, 1,
        face_selector=face, tool=ToolSpec("flat_endmill", 2))
    assert not inside(result, point(1, -1, 5.5))
    assert inside(result, point(4, -1, 5.5))
    assert inside(result, point(1, -1, 4))
    assert result.process_plan()["operations"][-1]["toolpath"]["metadata"]["tool_axis"] == axis


@pytest.mark.parametrize("profile,expected", [
    ({"type": "circle", "diameter": 4}, math.pi * 4 * 2),
    ({"type": "slot", "length": 10, "width": 4, "angle": 35}, (24 + math.pi * 4) * 2),
])
def test_matching_cutter_can_follow_a_point_or_line_center_path(profile, expected):
    stock = Stock.rectangular(16, 16, 8)
    result = stock.profile_pocket(profile, 2, tool=ToolSpec("flat_endmill", 4), finish_allowance=0)
    # The result arcs come back from polygon vertices on the cutter circle (integer nm scale).
    assert stock.volume - result.volume == pytest.approx(expected, abs=1e-4)
    comparison = result.process_plan()["operations"][-1]["operation_target_comparison"]
    assert comparison["remaining_target_material_mm3"] < 1e-6
    assert comparison["overcut_volume_mm3"] < 1e-6
    assert not any("sharp inside corners" in reason for reason in result.process_plan()["operations"][-1]["review_reasons"])


def test_matching_cutter_cannot_claim_radial_finishing_stock():
    with pytest.raises(ValueError, match="cannot leave radial finish allowance"):
        Stock.rectangular(16, 16, 8).profile_pocket({"type": "slot", "length": 10, "width": 4}, 2,
            tool=ToolSpec("flat_endmill", 4))


def test_through_depth_uses_the_actual_selected_axis_instead_of_stock_height():
    stock = Stock.rectangular(14, 10, 8)
    result = stock.profile_pocket({"type": "circle", "diameter": 2}, 1, through=True,
        face_selector=">X", tool=ToolSpec("flat_endmill", 2), finish_allowance=0)
    assert stock.volume - result.volume == pytest.approx(math.pi * 14, abs=1e-6)
    assert result.process_plan()["operations"][-1]["depth_mm"] == 14
    with pytest.raises(ValueError, match="cannot reach"):
        stock.profile_pocket({"type": "circle", "diameter": 2}, 1, through=True,
            face_selector=">X", tool=ToolSpec("flat_endmill", 2, flute_length=10), finish_allowance=0)


def test_translated_stock_preserves_a_previous_hole_and_center_placement():
    stock = Stock.rectangular(18, 18, 8)
    stock._shape = stock._shape.translate((20, -15, 10))
    stock._shape = stock._shape.cut(cq.Workplane("XY", origin=(20, -15, 5)).circle(1).extrude(12))
    result = stock.profile_pocket({"type": "rectangle", "length": 8, "width": 8, "center": [0, 0]}, 2,
        cx=20, cy=-15, tool=ToolSpec("flat_endmill", 2))
    assert not inside(result, (20, -15, 8))
    assert not inside(result, (22, -15, 13))
    assert inside(result, (25, -15, 13))
    assert min(move["position"][2] for move in result.process_plan()["operations"][-1]["toolpath"]["moves"]) == 12


def test_pocket_preserves_previous_turning_material_and_state_history():
    stock = Stock.round_bar(16, 10).turn_face(4).turn_id(2, 6)
    previous = list(stock._state_history)
    result = stock.profile_pocket({"type": "rectangle", "length": 8, "width": 6}, 2, tool=ToolSpec("flat_endmill", 2))
    assert all(first["shape"] is second["shape"] for first, second in zip(previous, result._state_history))
    assert not inside(result, (0, 0, 0))
    assert not inside(result, (2, 0, 3))
    assert inside(result, (5, 0, 3))


def test_owned_tool_selection_respects_island_clearance_and_inventory_identity():
    tools = [ToolSpec("flat_endmill", diameter, catalog_id=f"OWNED-{diameter}", physical_assembly={"owned": True}) for diameter in (1, 4)]
    with tool_inventory(tools, revision="pocket-inventory", strict=True):
        result = Stock.rectangular(14, 14, 8).profile_pocket({"type": "rectangle", "length": 10, "width": 10}, 1,
            islands=[{"type": "rectangle", "length": 6, "width": 6}])
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED-1"
    assert record["tool_selection"]["inventory_revision"] == "pocket-inventory"
    with tool_inventory([], strict=True), pytest.raises(ValueError, match="flat_endmill"):
        Stock.rectangular(14, 14, 8).profile_pocket({"type": "rectangle", "length": 10, "width": 10}, 1)


def test_triangle_tool_selection_qualifies_the_actual_center_region():
    tools = [ToolSpec("flat_endmill", diameter, catalog_id=f"OWNED-{diameter}", physical_assembly={"owned": True}) for diameter in (4, 8)]
    with tool_inventory(tools, revision="triangle-inventory", strict=True):
        result = Stock.rectangular(20, 20, 8).profile_pocket({"type": "polygon", "points": [[-5, -5], [5, -5], [0, 5]]}, 1)
    assert result.process_plan()["operations"][-1]["tool"]["catalog_id"] == "OWNED-4"


def test_full_stock_opening_can_clear_the_open_edges_around_a_retained_rib():
    stock = Stock.rectangular(20, 12, 8)
    result = stock.profile_pocket({"type": "rectangle", "length": 20, "width": 12}, 2,
        islands=[{"type": "rectangle", "length": 8, "width": 12}], tool=ToolSpec("flat_endmill", 2))
    assert stock.volume - result.volume == pytest.approx((20 - 8) * 12 * 2, abs=1e-5)
    assert inside(result, (0, 5.5, 3))
    assert not inside(result, (9.5, 5.5, 3))
    record = result.process_plan()["operations"][-1]
    assert record["toolpath"]["metadata"]["open_stock_edges"] is True
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6


def test_outside_profile_clearing_replays_waste_removal_instead_of_intersecting_the_target():
    stock = Stock.rectangular(12, 12, 8)
    result = stock.profile_cutout({"type": "rectangle", "length": 6, "width": 6}, depth=2,
        tool=ToolSpec("flat_endmill", 2))
    assert inside(result, (0, 0, 3))
    assert not inside(result, (4, 0, 3))
    assert inside(result, (4, 0, 0))
    record = result.process_plan()["operations"][-1]
    assert stock.volume - result.volume == pytest.approx((144 - 36) * 2, abs=1e-6)
    assert record["material_passes"]
    assert record["operation_target_comparison"]["overcut_volume_mm3"] < 1e-6
    assert record["toolpath"]["metadata"]["waste_discarded"] is False


def test_independent_replay_uses_the_recorded_pocket_paths_and_actual_incoming_material():
    stock = Stock.rectangular(10, 10, 6)
    tool = ToolSpec("flat_endmill", 2)
    result = stock.profile_pocket({"type": "rectangle", "length": 6, "width": 6}, 1, tool=tool)
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=result)
    assert not model.get_material_at((0, 0, 2.5))
    assert model.get_material_at((4, 0, 2.5))
    assert model.get_material_at((0, 0, 0))
    assert report["target_comparison"]["samples"] > 0
    assert report["target_comparison"]["tolerance_verified"] is False
    assert report["manufacturing_ready"] is False


@pytest.mark.parametrize("profile,options,reason", [
    ({}, {}, "closed profile"),
    ({"type": "circle", "diameter": True}, {}, "profile diameter"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"tool": ToolSpec("flat_endmill", 6)}, "does not fit"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"depth": 10}, "material thickness"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"face_selector": "top"}, "six cardinal"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"stepdown": 0}, "stepdown"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"tool": ToolSpec("ball_endmill", 2)}, "selected flat"),
    ({"type": "rectangle", "length": 4, "width": 4}, {"islands": "missing"}, "list"),
])
def test_invalid_pockets_raise_specific_errors(profile, options, reason):
    parameters = {"depth": 1, "tool": ToolSpec("flat_endmill", 2), **options}
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(12, 12, 8).profile_pocket(profile, **parameters)
