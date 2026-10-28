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


def test_later_roughing_reads_the_ledger_instead_of_the_display_mesh(monkeypatch):
    import numpy as np
    from src.subcad import layers
    from src.subcad.material_ledger import MaterialLedger, Cut

    stock = cq.Workplane('XY').box(60, 40, 30)
    base = stock.val()
    # A previous operation removed half the waste above the lower step.
    region = layers.to_paths([[(0, -20), (15, -20), (15, 20), (0, 20)]])
    ledger = MaterialLedger(base).with_cuts([Cut((np.eye(3, dtype=np.int64), np.zeros(3)), -5, 15, region)])
    display = cq.Workplane('XY').box(60, 40, 30)
    ledger.attach(display)
    original = layers.SolidMesh.of.__func__
    def mesh(cls, shape, *args, **kwargs):
        assert shape is not display.val(), 'The display mesh cannot supply the material ledger.'
        return original(cls, shape, *args, **kwargs)
    monkeypatch.setattr(layers.SolidMesh, 'of', classmethod(mesh))
    plan = level_plan(display, _part(), '>Z', radius=5, reach=40)
    assert plan.volume() == pytest.approx(15 * 40 * 20, rel=1e-3)


def test_stock_roughing_keeps_the_ledger_through_selection_and_replay(monkeypatch):
    import numpy as np
    from src.subcad import layers
    from src.subcad.material_ledger import MaterialLedger, Cut
    stock = Stock.rectangular(60, 40, 30)
    base = stock._shape.val()
    region = layers.to_paths([[(0, -20), (15, -20), (15, 20), (0, 20)]])
    removed = cq.Workplane('XY').box(15, 40, 20).translate((7.5, 0, 5))
    incoming = stock._shape.cut(removed)
    MaterialLedger(base).with_cuts([Cut((np.eye(3, dtype=np.int64), np.zeros(3)), -5, 15, region)]).attach(incoming)
    stock._shape = incoming
    original = layers.SolidMesh.of.__func__
    def mesh(cls, shape, *args, **kwargs):
        assert shape is not incoming.val(), 'Roughing must read the ledger instead of its display solid.'
        return original(cls, shape, *args, **kwargs)
    monkeypatch.setattr(layers.SolidMesh, 'of', classmethod(mesh))
    with tool_inventory([_mill(10, 25, 45, 'EM10')], revision='ledger', strict=True):
        result = stock.rough_levels(_part())
    assert incoming.val().Volume() - result._shape.val().Volume() == pytest.approx(15 * 40 * 20, rel=2e-3)


def test_player_roughing_builds_the_display_from_ledger_layers(monkeypatch):
    import numpy as np
    from src.subcad import milling_layers, milling_replay, layers as L
    from src.subcad.material_ledger import MaterialLedger, Cut
    from src.subcad.milling_opening import Setup
    monkeypatch.setattr(milling_layers, 'PLAYER_BREP_STATES', False)
    stock = Stock.rectangular(60, 40, 30)
    base = stock._shape.val()
    removed = cq.Workplane('XY').box(15, 40, 20).translate((7.5, 0, 5))
    stock._shape = stock._shape.cut(removed)
    region = L.to_paths([[(-20, -5), (20, -5), (20, 15), (-20, 15)]])
    MaterialLedger(base).with_cuts([Cut((Setup.of('>X').rotation, np.zeros(3)), 0, 15, region)]).attach(stock._shape)
    def no_boolean(*args, **kwargs):
        raise AssertionError('The layer-player preview does not need a display Boolean.')
    monkeypatch.setattr(milling_replay, '_boolean', no_boolean)
    monkeypatch.setattr(L, 'cutter_arcs', no_boolean)
    with tool_inventory([_mill(10, 25, 45, 'EM10')], revision='player-ledger', strict=True):
        result = stock.rough_levels(_part())
    assert result._shape.val().Volume() == pytest.approx(60*40*30 - 30*40*20, rel=2e-3)
    assert result._shape._subcad_material_ledger.brep_from_layers


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


def _turned(profile):
    return cq.Workplane("XZ").polyline(profile).close().revolve(360, (0, 0, 0), (0, 1, 0))


def test_spanner_flats_under_a_cone_are_roughed():
    # d2o#1322: a turned shaft, Ø10 to a 30 degree cone and Ø8 above it, with two flats 1 mm deep under the cone. The
    # quick probe takes one section between the flat floor and the top (Ø8 there: no waste) and roughed nothing.
    # The depth limit keeps the probe to the flat level (a deeper reach finds sliver bands below the flats).
    from src.subcad.process_plan import ProcessPlan
    turned = _turned([(0, 0), (5, 0), (5, 13.27), (4, 15), (4, 30), (0, 30)])
    part = turned.cut(cq.Workplane("XY").box(2, 6, 5).translate((5, 0, 12.5))) \
                 .cut(cq.Workplane("XY").box(2, 6, 5).translate((-5, 0, 12.5))).val()
    waste = turned.val().Volume() - part.Volume()
    dims = {"length": 10, "width": 10, "height": 30}
    stock = Stock._from_parts(turned, "steel_a36", dims, ProcessPlan(material="steel_a36", stock_dimensions=dims), 1)
    with tool_inventory([_mill(4, 11, 30, "EM4"), _mill(8, 19, 40, "EM8")], revision="flats"):
        roughed = stock.rough_levels(part, face_selector=">Z", max_depth=20.5)
    operation = roughed.process_plan()["operations"][-1]
    assert operation["operation"] == "level_rough"
    assert operation["removed_volume_mm3"] > 0.9 * waste
