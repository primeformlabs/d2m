"""Ball cutter geometry and retained material must follow the selected tool."""
import math
from unittest.mock import patch

import pytest

cq = pytest.importorskip('cadquery')
from src.subcad import Stock, ToolSpec
from src.subcad.milling_replay import _ball_sweep, _cutter, replay_pass
from src.subcad.toolpath import Toolpath, ToolpathMove


@pytest.mark.parametrize('end', [(8, 0, 0), (-8, 0, 0), (0, 8, 0), (0, -8, 0),
                               (0, 0, 8), (0, 0, -8), (3, -4, 2), (0, 0, 0)])
def test_spherical_sweep_has_exact_capsule_volume_bounds_and_endpoint_balls(end):
    radius = 3
    start = (0, 0, 0)
    solid = _ball_sweep(radius, start, end)
    assert solid.isValid() and len(solid.Solids()) == 1
    assert solid.Volume() == pytest.approx(math.pi * radius ** 2 * math.dist(start, end)
                                          + 4 * math.pi * radius ** 3 / 3, abs=1e-6)
    bounds = solid.BoundingBox()
    for index, axis in enumerate(('x', 'y', 'z')):
        offset = radius if axis == 'z' else 0
        assert getattr(bounds, axis + 'min') == pytest.approx(min(start[index], end[index]) + offset - radius, abs=1e-6)
        assert getattr(bounds, axis + 'max') == pytest.approx(max(start[index], end[index]) + offset + radius, abs=1e-6)
    for tip in (start, end):
        center = cq.Vector(tip[0], tip[1], tip[2] + radius)
        assert solid.isInside(center)
        assert solid.isInside(center + cq.Vector(0.2, 0.2, 0.2))


def test_ball_cutter_has_the_declared_hemisphere_flute_volume_and_tip():
    tool = ToolSpec('ball_endmill', 6, flute_length=12)
    solid = _cutter(tool)
    assert solid.isValid() and len(solid.Solids()) == 1
    assert solid.Volume() == pytest.approx(math.pi * 3 ** 2 * (12 - 3) + 2 * math.pi * 3 ** 3 / 3)
    assert solid.BoundingBox().zmin == pytest.approx(0)
    assert solid.BoundingBox().zmax == pytest.approx(12)
    assert solid.isInside(cq.Vector(0.5, 0, 0.1))
    assert not solid.isInside(cq.Vector(2.5, 0, 0.1))
    assert solid.isInside(cq.Vector(2.5, 0, 10))


def test_a_valid_but_material_adding_boolean_cannot_become_a_pass_result():
    stock = Stock.rectangular(8, 8, 8)
    path = Toolpath('dome', metadata={'coordinate_system': 'part_mm', 'tool_axis': [0, 0, 1]}, moves=[
        ToolpathMove('rapid', (-4, 0, 7)), ToolpathMove('plunge', (-4, 0, 3.1)),
        ToolpathMove('feed', (4, 0, 3.1)), ToolpathMove('retract', (4, 0, 7))])
    enlarged = cq.Workplane('XY').box(9, 9, 9).val()
    with patch('src.subcad.milling_replay._boolean', return_value=enlarged):
        with pytest.raises(ValueError, match='added material'):
            replay_pass(stock._shape, path, ToolSpec('ball_endmill', 6))


def test_empty_contact_never_runs_a_coincident_boundary_cut():
    stock = Stock.rectangular(8, 8, 8)
    path = Toolpath('dome', metadata={'coordinate_system': 'part_mm'}, moves=[
        ToolpathMove('rapid', (-4, 0, 7)), ToolpathMove('plunge', (-4, 0, 3.1)),
        ToolpathMove('feed', (4, 0, 3.1))])
    empty = cq.Compound.makeCompound([])
    def boolean(_first, _others, operation, **_kwargs):
        assert operation == 'common', 'An empty contact must not run a cut.'
        return empty
    with patch('src.subcad.milling_replay._boolean', side_effect=boolean):
        result, report = replay_pass(stock._shape, path, ToolSpec('ball_endmill', 6))
    assert result.val().isSame(stock._shape.val())
    assert report['empty_contact_solids'] > 0


def test_default_six_mm_ball_dome_retains_valid_monotone_material_and_target():
    stock = Stock.rectangular(8, 8, 8)
    result = stock.dome_mill(2.5, 1, tolerance_mm=0.3)
    operation = result._plan.operations[-1]
    assert operation['tool']['diameter_mm'] == 6
    assert len(result._state_history) == len(operation['material_passes']) + 1
    for previous, current in zip(result._state_history, result._state_history[1:]):
        before, after = previous['shape'].val(), current['shape'].val()
        assert before.isValid() and after.isValid()
        assert after.Volume() <= before.Volume() + 1e-6
    # The spherical cap and lower plate are independent target geometry.
    sphere_radius = (2.5 ** 2 + 1) / 2
    sphere = cq.Workplane('XY').sphere(sphere_radius).translate((0, 0, 4 - sphere_radius))
    plate = cq.Workplane('XY').box(8, 8, 7).translate((0, 0, -0.5))
    cap = sphere.intersect(cq.Workplane('XY').box(8, 8, 1, centered=(True, True, False)).translate((0, 0, 3)))
    target = plate.union(cap)
    assert target.cut(result._shape).val().Volume() <= 1e-5
    assert target.val().Volume() <= result.volume <= target.val().Volume() + 64 * 0.3
    assert result._shape.val().isInside(cq.Vector(0, 0, 3.95))
    assert result._shape.val().isInside(cq.Vector(3.5, 3.5, 2.95))
    assert not result._shape.val().isInside(cq.Vector(3.5, 3.5, 3.6))
    assert stock.volume == pytest.approx(512)
    assert all(row['removal_union'] == 'sequential_cutter_solids' for row in operation['material_passes'])
    assert operation['achieved_tolerance_estimate_mm'] is None
    assert operation['verification_status'] == 'not_verified'


def test_ball_surface_with_axial_contact_preserves_translated_previous_hole():
    stock = Stock.rectangular(6, 6, 6)
    stock._shape = stock._shape.translate((15, -10, 8))
    hole = cq.Workplane('XY', origin=(15, -10, 4)).circle(0.5).extrude(10)
    stock._shape = stock._shape.cut(hole)
    tool = ToolSpec('ball_endmill', 2, flute_length=12, stickout_mm=15,
                    safe_limits={'max_stepdown_mm': 1})
    result = stock.surface_mill({'type': 'plane', 'height_mm': 8.5}, tool=tool, tolerance_mm=0.3)
    assert result.volume < stock.volume
    assert result._shape.val().isInside(cq.Vector(16.5, -10, 8))
    assert not result._shape.val().isInside(cq.Vector(15, -10, 8))
    assert not result._shape.val().isInside(cq.Vector(16.5, -10, 10.5))
    for previous, current in zip(result._state_history, result._state_history[1:]):
        assert current['shape'].val().isValid()
        assert current['shape'].val().Volume() <= previous['shape'].val().Volume() + 1e-6
