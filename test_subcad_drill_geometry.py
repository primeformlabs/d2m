"""Prove drill placement without planar face selection or shifted origins."""
import math

import pytest

cq = pytest.importorskip("cadquery")
from src.subcad import Stock
from src.subcad.drill_geometry import drill_cut


@pytest.mark.parametrize("face", [">X", "<X", ">Y", "<Y", ">Z", "<Z"])
@pytest.mark.parametrize("through", [True, False])
def test_drill_has_the_correct_volume_for_every_access_direction(face, through):
    initial = cq.Workplane("XY").box(40, 40, 40)
    result = drill_cut(initial, 3, 4, 4, 5, through, face_selector=face)
    assert initial.val().Volume() - result.val().Volume() == pytest.approx(math.pi * 2 ** 2 * (40 if through else 5))
    assert initial.val().isValid() and result.val().isValid()


def test_drill_keeps_its_coordinates_after_an_asymmetric_pocket():
    initial = Stock.rectangular(40, 40, 20).pocket(width=6, length=8, depth=5, cx=-12, cy=-10, corner_radius=1)
    result = initial.drill(diameter=4, depth=20, through=True, cx=12, cy=10, spot_drill=False)
    assert initial._shape.val().isInside(cq.Vector(12, 10, 0))
    assert not result._shape.val().isInside(cq.Vector(12, 10, 0))
    assert result._shape.val().isInside(cq.Vector(0, 0, 0))


@pytest.mark.parametrize("axis,face", [("X", ">Z"), ("Y", ">X"), ("Z", ">Y")])
def test_drill_cuts_curved_turning_stock_and_survives_later_turning(axis, face):
    initial = Stock.round_bar(40, 20, axis=axis).turn_od(18, 40)
    drilled = initial.drill(diameter=4, depth=20, through=True, face_selector=face, spot_drill=False)
    result = drilled.turn_od(16, 40)
    assert result._shape.val().Volume() < math.pi * 8 ** 2 * 40
    assert not result._shape.val().isInside(cq.Vector(0, 0, 0))
    assert result._shape.val().isValid()
    assert len(result._state_history) == len(initial._state_history) + 2


@pytest.mark.parametrize("changes", [{"diameter": 0}, {"depth": -1}, {"depth": 0},
    {"diameter": math.nan}, {"cx": math.inf}, {"through": "true"}, {"diameter": True}, {"face_selector": "unknown"}])
def test_invalid_drill_intent_never_returns_an_unchanged_success(changes):
    initial = cq.Workplane("XY").box(40, 40, 20)
    values = {"cx": 0, "cy": 0, "diameter": 4, "depth": 5, **changes}
    with pytest.raises((ValueError, TypeError)):
        drill_cut(initial, **values)
