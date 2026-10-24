"""A contour must remove its selected cutter channel on the correct side."""
import math

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec, tool_inventory
from src.subcad.operations import ProfileContourOp
from src.subcad.profile_milling import _profile_wire, _linearized_wire
from src.subcad.toolpath import Toolpath
from src.subcad.milling_verification import replay_milling_independently


PROFILE = {"type": "rect", "length": 20, "width": 16}


def material(stock, point):
    return stock._shape.val().isInside(cq.Vector(*point))


def test_outside_contour_keeps_the_profile_interior_and_uncut_exterior():
    stock = Stock.rectangular(40, 36, 10)
    result = stock.profile_contour(PROFILE, 2, tool=ToolSpec("flat_endmill", 2), finish_allowance=0)
    assert material(result, (0, 0, 4))
    assert material(result, (9.9, 0, 4))
    assert not material(result, (11, 0, 4))
    assert material(result, (15, 0, 4))
    assert material(result, (11, 0, 2))
    assert material(stock, (11, 0, 4))
    # The native path creates a two-millimetre channel around the outline.
    assert stock.volume - result.volume == pytest.approx((72 * 2 + math.pi * 4) * 2, abs=0.8)


def test_inside_contour_leaves_the_core_and_cutter_limited_corners():
    stock = Stock.rectangular(40, 36, 10)
    result = stock.profile_contour(PROFILE, 2, side="inside", tool=ToolSpec("flat_endmill", 2), finish_allowance=0)
    assert material(result, (0, 0, 4))
    assert not material(result, (9, 0, 4))
    assert material(result, (11, 0, 4))
    assert material(result, (9.9, 7.9, 4))
    assert stock.volume - result.volume == pytest.approx((320 - (4 - math.pi) - 192) * 2, abs=1e-6)
    assert any("inside corner" in reason for reason in result.process_plan()["operations"][-1]["review_reasons"])


def test_each_roughing_and_finishing_pass_has_its_actual_state_and_moves():
    stock = Stock.rectangular(24, 20, 10)
    result = stock.profile_contour({"length": 12, "width": 8}, 3, tool=ToolSpec("flat_endmill", 2), stepdown=0.75)
    record = result.process_plan()["operations"][-1]
    reports = record["material_passes"]
    assert len(reports) == 5
    assert [row["pass_role"] for row in reports] == ["roughing"] * 4 + ["finishing"]
    assert len(result._state_history) == 6
    assert all(row["removed_volume_mm3"] > 0 for row in reports)
    # Layer removal (2D chords within 0.25 um) against the B-Rep result (docs/MILLING_LAYER_REPLAY.md).
    assert sum(row["removed_volume_mm3"] for row in reports) == pytest.approx(stock.volume - result.volume, rel=1e-4)
    assert result._state_history[-1]["shape"].val().isSame(result._shape.val())
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["geometry_status"] == "requires_review"
    assert record["manufacturing_completeness"] == "requires_validation"
    moves = record["toolpath"]["moves"]
    for row in reports:
        segment = moves[row["move_start_index"]:row["move_end_index"]]
        assert segment[0]["type"] == "rapid"
        assert segment[-1]["type"] == "retract"
        assert all(move["position"][0] < 9 for move in segment)
    assert record["toolpath"]["metadata"]["waste_discarded"] is False


@pytest.mark.parametrize("face,axis,coordinates", [
    (">Z", [0, 0, 1], lambda u, v, w: (u, v, w)),
    ("<Z", [0, 0, -1], lambda u, v, w: (u, v, -w)),
    (">X", [1, 0, 0], lambda u, v, w: (w, u, v)),
    ("<X", [-1, 0, 0], lambda u, v, w: (-w, u, v)),
    (">Y", [0, 1, 0], lambda u, v, w: (u, w, v)),
    ("<Y", [0, -1, 0], lambda u, v, w: (u, -w, v)),
])
def test_cardinal_setups_use_stable_profile_coordinates_and_the_correct_cutter_axis(face, axis, coordinates):
    stock = Stock.rectangular(32, 32, 32)
    result = stock.profile_contour({"length": 12, "width": 8, "cx": 3, "cy": -2}, 2,
        face_selector=face, tool=ToolSpec("flat_endmill", 2), finish_allowance=0)
    assert material(result, coordinates(3, -2, 15))
    assert not material(result, coordinates(10, -2, 15))
    assert material(result, coordinates(10, -2, 12))
    path = result.process_plan()["operations"][-1]["toolpath"]
    assert path["metadata"]["tool_axis"] == axis
    assert result.volume < stock.volume
    assert path["metadata"]["coordinate_system"] == "part_mm"


def test_translated_stock_and_preceding_hole_survive_the_outside_cut():
    stock = Stock.rectangular(30, 26, 10)
    stock._shape = stock._shape.translate((20, -15, 10))
    hole = cq.Workplane("XY", origin=(20, -15, 4)).circle(1).extrude(12)
    stock._shape = stock._shape.cut(hole)
    result = stock.profile_contour({"length": 12, "width": 8}, 2, cx=20, cy=-15,
        tool=ToolSpec("flat_endmill", 2), finish_allowance=0)
    assert not material(result, (20, -15, 14))
    assert material(result, (22, -15, 14))
    assert not material(result, (27, -15, 14))
    assert material(result, (27, -15, 10))
    assert min(move["position"][2] for move in result.process_plan()["operations"][-1]["toolpath"]["moves"]) == 13


@pytest.mark.parametrize("profile", [
    {"type": "circle", "diameter": 12, "cx": 2, "cy": -1},
    {"type": "slot", "length": 14, "width": 6, "angle": 30},
    {"type": "slot", "length": 6, "width": 6},
    {"type": "polygon", "points": [(-5, -4), (5, -4), (3, 4), (-3, 4)]},
    {"type": "rectangle", "length": 14, "width": 10, "corner_radius": 2, "angle": 15},
])
def test_profiles_use_their_own_boundary_instead_of_a_square_substitute(profile):
    result = Stock.rectangular(28, 28, 8).profile_contour(profile, 1, tool=ToolSpec("flat_endmill", 2))
    assert material(result, (2, -1, 3.5))
    assert result.volume < 28 * 28 * 8
    record = result.process_plan()["operations"][-1]
    assert "nominal_square_mm" not in record["toolpath"]["metadata"]
    assert any("linear segments" in reason for reason in record["review_reasons"])


@pytest.mark.parametrize("profile,kwargs,reason", [
    ({}, {}, "closed profile"),
    ({"points": [(0, 0), (4, 4), (0, 4), (4, 0)]}, {}, "without intersections"),
    ({"points": [(0, 0), (4, 0), (4, 0), (0, 4)]}, {}, "repeated"),
    ({"points": [(0, 0), (math.nan, 4), (4, 0)]}, {}, "finite closed"),
    ({"type": "circle"}, {}, "profile diameter"),
    ({"type": "unknown", "length": 10, "width": 8}, {}, "unsupported"),
    ({"type": "slot", "length": 4, "width": 6}, {}, "at least equal"),
    ({"length": 4, "width": 4}, {"side": "inside", "tool": ToolSpec("flat_endmill", 6)}, "does not fit"),
    (PROFILE, {"depth": 11}, "material thickness"),
    (PROFILE, {"side": "center"}, "inside or outside"),
    (PROFILE, {"stepdown": 0}, "stepdown"),
    (PROFILE, {"finish_allowance": -1}, "non-negative"),
    (PROFILE, {"face_selector": "top"}, "six cardinal"),
    (PROFILE, {"tool": ToolSpec("ball_endmill", 2)}, "selected flat"),
    (PROFILE, {"tool": ToolSpec("flat_endmill", 2, flute_length=0.5)}, "cannot reach"),
])
def test_invalid_or_unreachable_contours_cannot_return_machined_material(profile, kwargs, reason):
    options = {"depth": 1, "tool": ToolSpec("flat_endmill", 2), **kwargs}
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(30, 30, 10).profile_contour(profile, **options)


def test_owned_tool_scope_selects_its_actual_cutter_and_retains_inventory_identity():
    tool = ToolSpec("flat_endmill", 2, catalog_id="OWNED-CONTOUR", physical_assembly={"owned": True})
    with tool_inventory([tool], revision="contour-inventory", strict=True):
        result = Stock.rectangular(30, 30, 8).profile_contour(PROFILE, 1, side="inside")
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED-CONTOUR"
    assert record["tool_selection"]["inventory_revision"] == "contour-inventory"
    assert not any("owned physical" in reason for reason in record["review_reasons"])
    with tool_inventory([], strict=True), pytest.raises(ValueError, match="flat_endmill"):
        Stock.rectangular(30, 30, 8).profile_contour(PROFILE, 1)


def test_unapplied_contour_cannot_export_the_previous_nominal_square_path():
    with pytest.raises(ValueError, match="incoming material"):
        ProfileContourOp(profile=PROFILE, tool=ToolSpec("flat_endmill", 2)).to_toolpath()


def test_contour_preserves_preceding_turning_material_and_history():
    stock = Stock.round_bar(24, 14).turn_face(5).turn_id(4, 8)
    previous = list(stock._state_history)
    result = stock.profile_contour({"length": 10, "width": 8}, 2, tool=ToolSpec("flat_endmill", 2))
    assert all(first["shape"] is second["shape"] for first, second in zip(previous, result._state_history))
    assert not material(result, (0, 0, 4))
    assert material(result, (3, 0, 4))
    assert result.volume < stock.volume
    assert result.process_plan()["operations"][-1]["material_passes"]


def test_offset_path_is_ordered_for_concave_boundaries():
    wire, _ = _profile_wire({"points": [(-6, -6), (6, -6), (6, -2), (-2, -2), (-2, 6), (-6, 6)]})
    points, curved = _linearized_wire(wire.offset2D(1)[0], 0.01)
    assert curved
    assert points[0] == points[-1]
    assert max(math.dist(a, b) for a, b in zip(points, points[1:])) < 15


def test_independent_engine_replays_the_actual_contour_channel():
    stock = Stock.rectangular(12, 12, 6)
    tool = ToolSpec("flat_endmill", 2)
    result = stock.profile_contour({"length": 6, "width": 6}, 2, side="inside", tool=tool, finish_allowance=0)
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=result)
    assert model.get_material_at((0, 0, 2))
    assert not model.get_material_at((2, 0, 2))
    assert report["removed_volume_mm3"] > 0
    assert report["target_comparison"]["samples"] > 0
    assert report["target_comparison"]["tolerance_verified"] is False
    assert report["manufacturing_ready"] is False


@pytest.mark.parametrize("axis,point", [
    ([0, 0, 1], lambda u, v, w: (u, v, w)),
    ([0, 0, -1], lambda u, v, w: (u, v, -w)),
    ([1, 0, 0], lambda u, v, w: (w, u, v)),
    ([-1, 0, 0], lambda u, v, w: (-w, u, v)),
    ([0, 1, 0], lambda u, v, w: (u, w, v)),
    ([0, -1, 0], lambda u, v, w: (u, -w, v)),
])
def test_independent_replay_uses_cardinal_cutter_orientation_in_part_coordinates(axis, point):
    from src.subcad.toolpath import ToolpathMove
    tool = ToolSpec("flat_endmill", 2)
    stock = Stock.rectangular(8, 8, 8)
    path = Toolpath("profile_contour", tool_type="flat_endmill", tool_diameter_mm=2,
        metadata={"coordinate_system": "part_mm", "tool_axis": axis}, moves=[
            ToolpathMove("rapid", point(-2, 1, 6)),
            ToolpathMove("plunge", point(-2, 1, 2)),
            ToolpathMove("feed", point(2, 1, 2)),
            ToolpathMove("retract", point(2, 1, 6)),
        ])
    model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5)
    assert not model.get_material_at(point(0, 1, 3))
    assert model.get_material_at(point(0, -2, 3))
    assert model.get_material_at(point(0, 1, 0))
    assert report["tool_axis"] == axis
    assert report["coordinate_system"] == "part_mm"
    assert report["removed_volume_mm3"] > 0


@pytest.mark.parametrize("axis", [[True, 0, 0], [math.nan, 0, 0], [[0], 0, 1], [0.5, 0, 0.5]])
def test_independent_replay_rejects_invalid_or_noncardinal_axes(axis):
    with pytest.raises(ValueError, match="cardinal cutter axis"):
        replay_milling_independently(Stock.rectangular(8, 8, 8),
            Toolpath("profile_contour", metadata={"coordinate_system": "part_mm", "tool_axis": axis}), ToolSpec("flat_endmill", 2))


def test_a_contour_cut_on_a_split_floor_keeps_the_material():
    """TP-0127 (D2O): after a 6 mm corner rest pass the pocket floor is split into coplanar faces at the end level
    of the D22 contour channel; the plain cut gave no solid. cut_material unifies the faces and cuts again."""
    from pathlib import Path
    from src.subcad.milling_replay import _boolean, cut_material
    folder = Path(__file__).parent / "test_fixtures" / "contour_coplanar_floor"
    material = cq.Shape.importBrep(str(folder / "material.brep"))
    cutter = cq.Shape.importBrep(str(folder / "cutter.brep"))
    assert not _boolean(material, [cutter], "cut").Solids()
    result = cut_material(material, cutter)
    assert len(result.Solids()) == 1 and result.isValid()
    removed = material.Volume() - result.Volume()
    assert 0 < removed < cutter.Volume() + 1e-6


def test_a_raster_cut_along_a_channel_cut_before_keeps_valid_material():
    """TP-0127 (D2O): the D22 raster capsule touches the walls of its contour channel within a micrometre; the plain
    cut gave an invalid solid. cut_material cuts with a 1 micrometre fuzzy value."""
    from pathlib import Path
    from src.subcad.milling_replay import _boolean, cut_material
    folder = Path(__file__).parent / "test_fixtures" / "raster_touching_channel"
    material = cq.Shape.importBrep(str(folder / "material.brep"))
    cutter = cq.Shape.importBrep(str(folder / "cutter.brep"))
    assert not _boolean(material, [cutter], "cut").isValid()
    result = cut_material(material, cutter)
    assert len(result.Solids()) == 1 and result.isValid()
    removed = material.Volume() - result.Volume()
    assert 0 < removed < cutter.Volume() + 1e-6
