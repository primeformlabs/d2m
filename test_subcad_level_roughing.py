"""Level roughing (level_roughing.py): the waste above each planar level of a setup, inside the part silhouette,
never into the part, holes or closed pockets."""

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory
    from src.subcad.level_roughing import level_plan


def _mill(diameter, flute, stickout, catalog_id):
    return ToolSpec("flat_endmill", diameter, flute_length=flute, reach_mm=stickout, stickout_mm=stickout,
                    num_flutes=4, catalog_id=catalog_id, physical_assembly={"owned": True})


def _drill(diameter, flute, stickout, catalog_id):
    return ToolSpec("drill", diameter, flute_length=flute, reach_mm=stickout, stickout_mm=stickout, tip_angle=118.0,
                    tip_diameter=0.0, num_flutes=2, catalog_id=catalog_id, physical_assembly={"owned": True})


def _part():
    """A 60 x 40 x 30 block whose +X half is 20 mm lower, with a 10 mm closed pocket in the high half and a hole."""
    high = cq.Workplane("XY").box(30, 40, 30).translate((-15, 0, 0))
    low = cq.Workplane("XY").box(30, 40, 10).translate((15, 0, -10))
    part = high.union(low)
    pocket = cq.Workplane("XY").box(10, 10, 5).translate((-15, 0, 12.5))
    hole = cq.Workplane("XY").circle(3).extrude(10).translate((15, 0, -15))
    return part.cut(pocket).cut(hole).val()


def test_the_plan_roughs_the_step_and_keeps_the_pocket_and_the_hole():
    part = _part()
    stock = cq.Workplane("XY").box(60, 40, 30).val()
    plan = level_plan(stock, part, ">Z", radius=5, reach=40)
    assert len(plan.bands) == 1
    low, high, region, _ = plan.bands[0]
    assert (low, high) == pytest.approx((-5, 15), abs=1e-6)
    # The 30 x 40 step, less the cutter radius in its inside corner line (a closing of a straight wall leaves none).
    assert plan.volume() == pytest.approx(30 * 40 * 20, rel=1e-3)


def test_level_roughing_then_a_drill_from_the_low_level():
    part = _part()
    tools = [_mill(10, 25, 45, "EM10"), _mill(16, 12, 30, "EM16S"), _drill(6, 14, 45, "D6")]
    with tool_inventory(tools, revision="levels", strict=True):
        roughed = Stock.rectangular(60, 40, 30).rough_levels(part, face_selector=">Z")
        operation = roughed.process_plan()["operations"][-1]
        assert operation["operation"] == "level_rough"
        # The largest end mill that reaches the 20 mm step floor (its 30 mm projection), not the longest one.
        assert operation["tool"]["catalog_id"] == "EM16S"
        removed = 60 * 40 * 30 - roughed._shape.val().Volume()
        assert removed == pytest.approx(30 * 40 * 20, rel=2e-3)
        drilled = roughed.drill(6, through=True, cx=15, cy=0, spot_drill=False)
    hole = drilled.process_plan()["operations"][-1]
    assert hole["start_depth_mm"] == pytest.approx(20, abs=1e-3)


def test_no_waste_above_a_level_gives_no_operation():
    with tool_inventory([_mill(10, 25, 45, "EM10")], revision="levels", strict=True):
        stock = Stock.rectangular(60, 40, 30)
        assert stock.rough_levels(cq.Workplane("XY").box(60, 40, 30)) is stock


def test_a_kept_profile_stays_for_its_own_operation():
    part = _part()
    stock = cq.Workplane("XY").box(60, 40, 30).val()
    full = level_plan(stock, part, ">Z", radius=5, reach=40).volume()
    kept = level_plan(stock, part, ">Z", radius=5, reach=40,
                      keep=[{"type": "rect", "length": 10, "width": 10, "cx": 20, "cy": 0}]).volume()
    # The 10 mm square, 15 mm from the step wall (a gap of 10 mm or less would close for the 10 mm cutter).
    assert full - kept == pytest.approx(10 * 10 * 20, rel=1e-2)


def test_a_large_pocket_path_set_replays_by_level(monkeypatch):
    from src.subcad import profile_pocket_milling
    tools = [_mill(6, 20, 35, "EM6")]
    profile = {"type": "rectangle", "length": 20, "width": 14}
    with tool_inventory(tools, revision="levels", strict=True):
        per_pass = Stock.rectangular(40, 30, 12).profile_pocket(profile, 4, finish_allowance=0)
        monkeypatch.setattr(profile_pocket_milling, "PER_PASS_PATHS", 1)
        by_level = Stock.rectangular(40, 30, 12).profile_pocket(profile, 4, finish_allowance=0)
    passes = by_level.process_plan()["operations"][-1]["material_passes"]
    assert all(row["path_kind"] == "level" for row in passes)
    assert by_level._shape.val().Volume() == pytest.approx(per_pass._shape.val().Volume(), rel=1e-4)
