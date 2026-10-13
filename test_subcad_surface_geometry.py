"""Native material states must follow cutter motion, including previous holes."""
import math

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock, ToolSpec, tool_inventory
from src.subcad.milling_replay import replay_pass
from src.subcad.toolpath import Toolpath, ToolpathMove


def test_plane_removal_uses_the_absolute_stock_datum_and_retains_each_row():
    stock = Stock.rectangular(12, 10, 8)
    result = stock.surface_mill({"type": "plane", "height_mm": 1}, tool=ToolSpec("flat_endmill", 4), tolerance_mm=0.25)
    assert result.volume == pytest.approx(12 * 10 * 5)
    assert stock.volume == pytest.approx(12 * 10 * 8)
    assert result._shape.val().BoundingBox().zmax == pytest.approx(1)
    record = result.process_plan()["operations"][-1]
    assert record["geometry_fidelity"] == "exact_cutter_sweep"
    assert record["achieved_tolerance_estimate_mm"] is None
    assert record["depth_mm"] == pytest.approx(3)
    assert len(result._state_history) == len(record["material_passes"]) + 1
    volumes = [state["shape"].val().Volume() for state in result._state_history]
    assert all(after <= before + 1e-7 for before, after in zip(volumes, volumes[1:]))
    assert len({round(volume, 6) for volume in volumes}) > 3
    assert {row["pass_role"] for row in record["material_passes"]} == {"roughing", "finishing"}


def test_surface_cut_preserves_a_previous_drill_hole_and_translated_coordinates():
    stock = Stock.rectangular(10, 10, 8)
    stock._shape = stock._shape.translate((20, -15, 10))
    # A geometric hole with known volume avoids a second operation's path approximation.
    hole = cq.Workplane("XY", origin=(21, -14, 5)).circle(1).extrude(10)
    stock._shape = stock._shape.cut(hole)
    result = stock.surface_mill({"type": "plane", "height_mm": 11}, tool=ToolSpec("flat_endmill", 4), tolerance_mm=0.25)
    assert result.volume == pytest.approx((100 - math.pi) * 5)
    assert not result._shape.val().isInside(cq.Vector(21, -14, 10))
    assert result._shape.val().isInside(cq.Vector(23, -14, 10))
    moves = result.process_plan()["operations"][-1]["toolpath"]["moves"]
    assert min(move["position"][0] for move in moves) >= 13
    assert min(move["position"][1] for move in moves) >= -22


def test_native_replay_does_not_cut_during_a_rapid_move():
    stock = Stock.rectangular(12, 12, 8)
    tool = ToolSpec("flat_endmill", 2)
    path = Toolpath("slot", metadata={"coordinate_system": "part_mm"}, moves=[
        ToolpathMove("rapid", (-5, -4, 8)), ToolpathMove("rapid", (5, -4, 8)),
        ToolpathMove("rapid", (-4, 4, 8)), ToolpathMove("plunge", (-4, 4, 2)),
        ToolpathMove("feed", (4, 4, 2)), ToolpathMove("retract", (4, 4, 8)),
    ])
    result, report = replay_pass(stock._shape, path, tool)
    assert result.val().isInside(cq.Vector(0, -4, 3))
    assert not result.val().isInside(cq.Vector(0, 4, 3))
    assert stock.volume - result.val().Volume() == pytest.approx((16 + math.pi) * 2)
    assert report["geometry_fidelity"] == "exact_cutter_sweep"


@pytest.mark.parametrize("operation,reason", [
    (lambda stock: stock.surface_mill({"id": "missing_patch"}), "explicit geometry"),
    (lambda stock: stock.sweep_mill({"width": 4, "length": 8}, {"points": [(0, 0), (5, 0)]}), "no native material-removal"),
    (lambda stock: stock.loft_mill([{"diameter": 8}, {"diameter": 4}]), "no native material-removal"),
])
def test_missing_surface_geometry_never_returns_unchanged_machined_material(operation, reason):
    with pytest.raises(ValueError, match=reason):
        operation(Stock.rectangular(12, 12, 8))


def test_surface_tool_must_reach_the_actual_cutting_depth():
    stock = Stock.rectangular(12, 12, 10)
    path = Toolpath("surface_mill", metadata={"coordinate_system": "part_mm"}, moves=[
        ToolpathMove("rapid", (0, 0, 8)), ToolpathMove("plunge", (0, 0, -1)),
    ])
    with pytest.raises(ValueError, match="cannot reach"):
        replay_pass(stock._shape, path, ToolSpec("flat_endmill", 2, flute_length=2, stickout_mm=2, reach_mm=2))


def test_surface_selection_uses_only_the_owned_inventory_scope():
    tool = ToolSpec("flat_endmill", 4, catalog_id="OWNED-4", physical_assembly={"owned": True})
    with tool_inventory([tool], revision="physical-inventory", strict=True):
        result = Stock.rectangular(8, 8, 6).surface_mill({"type": "plane", "height_mm": 1}, tolerance_mm=0.25)
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["catalog_id"] == "OWNED-4"
    assert record["tool_selection"]["inventory_revision"] == "physical-inventory"
    with tool_inventory([], strict=True), pytest.raises(ValueError, match="flat_endmill"):
        Stock.rectangular(8, 8, 6).surface_mill({"type": "plane", "height_mm": 1})


def test_surface_uses_shallow_owned_cutter_passes_instead_of_an_invented_initial_depth():
    tool = ToolSpec("flat_endmill", 1, flute_length=0.5, stickout_mm=2, reach_mm=0.5,
        catalog_id="OWNED-SHALLOW", physical_assembly={"owned": True})
    with tool_inventory([tool], strict=True):
        result = Stock.rectangular(4, 4, 4).surface_mill({"type": "plane", "height_mm": 1}, tolerance_mm=0.3)
    assert result.volume == pytest.approx(48)
    assert result.process_plan()["operations"][-1]["depth_mm"] == pytest.approx(1)


@pytest.mark.parametrize("tool,reason", [
    (ToolSpec("flat_endmill", float("nan")), "cutter diameter"),
    (ToolSpec("flat_endmill", 2, flute_length=0), "flute length"),
    (ToolSpec("flat_endmill", 2, corner_radius=0.5), "bull nose"),
])
def test_invalid_cutter_values_have_specific_reasons_before_path_generation(tool, reason):
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(8, 8, 6).surface_mill({"type": "plane", "height_mm": 1}, tool=tool)


def test_dome_cuts_native_material_with_the_ball_cutter_and_retains_a_previous_hole():
    stock = Stock.rectangular(8, 8, 8)
    hole = cq.Workplane("XY", origin=(0, 0, -5)).circle(0.5).extrude(10)
    stock._shape = stock._shape.cut(hole)
    result = stock.dome_mill(2.5, 1, tool=ToolSpec("ball_endmill", 2,
        safe_limits={"max_stepdown_mm": 0.5}), tolerance_mm=0.3)
    assert result.volume < stock.volume
    assert result._shape.val().isInside(cq.Vector(1, 0, 3.75))
    assert not result._shape.val().isInside(cq.Vector(3.5, 3.5, 3.6))
    assert not result._shape.val().isInside(cq.Vector(0, 0, 2))
    record = result.process_plan()["operations"][-1]
    assert record["tool"]["tool_type"] == "ball_endmill"
    assert record["geometry_fidelity"] == "exact_cutter_sweep"
    assert record["cutter_compensation"] == "geometry_applied"
    assert record["verification_status"] == "not_verified"
    assert len(result._state_history) == len(record["material_passes"]) + 1
    assert {row["pass_role"] for row in record["material_passes"]} == {"roughing", "finishing"}
    assert all(not row["manufacturing_ready"] for row in record["material_passes"])


@pytest.mark.parametrize("radius,height,tool,reason", [
    (2, 3, ToolSpec("ball_endmill", 2), "overhang"),
    (2, 1, ToolSpec("flat_endmill", 2), "ball end mill"),
    (float("nan"), 1, ToolSpec("ball_endmill", 2), "finite"),
    (20, 1, ToolSpec("ball_endmill", 2), "footprint"),
])
def test_dome_rejects_unsupported_or_invalid_geometry(radius, height, tool, reason):
    with pytest.raises(ValueError, match=reason):
        Stock.rectangular(8, 8, 8).dome_mill(radius, height, tool=tool)


def test_independent_simulation_keeps_part_coordinates_and_cuts_at_positive_Z():
    from src.subcad.stock import _toolpath_from_serialized
    from src.subcad.operation_video import _operation_moves, _is_cutting_move
    path = Toolpath("surface_mill", metadata={"coordinate_system": "part_mm"}, moves=[
        ToolpathMove("rapid", (-4, 0, 8)), ToolpathMove("plunge", (-4, 0, 1)),
        ToolpathMove("feed", (4, 0, 1)),
    ]).to_dict()
    poses = _toolpath_from_serialized(path, z_offset=4)
    assert poses[-1].position.tolist() == [4, 0, 1]
    moves = _operation_moves({"operation": "surface_mill", "toolpath": path}, {"z_max": 4})
    assert moves[-1]["position"] == [4, 0, 1]
    assert _is_cutting_move(moves[-1], moves[-1]["position"])
    assert not _is_cutting_move(moves[0], moves[0]["position"])


def test_each_retained_surface_pass_exports_its_own_mesh_and_content_identity(tmp_path):
    import hashlib
    from src.subcad.visualization import _export_stock_states
    result = Stock.rectangular(8, 8, 6).surface_mill({"type": "plane", "height_mm": 1},
        tool=ToolSpec("flat_endmill", 4), tolerance_mm=0.3)
    states = _export_stock_states(result, tmp_path, result.process_plan())
    assert len(states) == len(result._state_history)
    assert len({state["path"] for state in states}) == len(states)
    assert len({state["sha256"] for state in states}) > 3
    for state in states:
        assert hashlib.sha256((tmp_path / state["path"]).read_bytes()).hexdigest() == state["sha256"]


def test_independent_tool_replay_compares_the_material_without_using_the_target_for_removal():
    from src.subcad import replay_milling_independently
    stock = Stock.rectangular(8, 8, 6)
    tool = ToolSpec("flat_endmill", 4)
    result = stock.surface_mill({"type": "plane", "height_mm": 1}, tool=tool, tolerance_mm=0.3)
    path = Toolpath.from_dict(result.process_plan()["operations"][-1]["toolpath"])
    model, report = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=result, tolerance_mm=0.1)
    assert model.get_material_at(__import__("numpy").array([0.25, 0.25, 0.25]))
    assert not model.get_material_at(__import__("numpy").array([0.25, 0.25, 2.25]))
    assert report["target_comparison"]["status"] == "sampled_match"
    assert not report["target_comparison"]["tolerance_verified"]
    assert report["status"] == "requires_review"
    assert not report["manufacturing_ready"]
    _, wrong_target = replay_milling_independently(stock, path, tool, resolution_mm=0.5, target=stock)
    assert wrong_target["target_comparison"]["overcut_samples"] > 0
