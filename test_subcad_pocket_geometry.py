"""Native B-rep evidence for pocket dimensions and stable stock coordinates."""
import math

import pytest

cq = pytest.importorskip("cadquery")

from src.subcad.pocket_geometry import rectangular_pocket_cut


@pytest.mark.parametrize("face", [">X", "<X", ">Y", "<Y", ">Z", "<Z"])
def test_rounded_pocket_preserves_corners_on_all_stock_faces(face):
    stock = cq.Workplane("XY").box(60, 60, 60)
    result = rectangular_pocket_cut(stock, width=20, length=20, depth=5, corner_radius=2,
        face_selector=face, cx=3, cy=4)
    removed = stock.val().Volume() - result.val().Volume()
    expected = (20 * 20 - (4 - math.pi) * 2 ** 2) * 5
    assert removed == pytest.approx(expected, abs=1e-6)
    assert result.val().isValid()
    assert len(result.solids().vals()) == 1


def test_later_pocket_keeps_its_stock_position_after_an_asymmetric_cut():
    stock = cq.Workplane("XY").box(60, 60, 30)
    first = rectangular_pocket_cut(stock, width=8, length=12, depth=5, corner_radius=1, cx=-15, cy=-10)
    second = rectangular_pocket_cut(first, width=8, length=12, depth=5, corner_radius=1, cx=15, cy=10)
    assert first.val().isInside(cq.Vector(15, 10, 13))
    assert not second.val().isInside(cq.Vector(15, 10, 13))
    assert not second.val().isInside(cq.Vector(-15, -10, 13))
    assert second.val().isInside(cq.Vector(0, 0, 13))


def test_base_height_retains_the_required_floor():
    stock = cq.Workplane("XY").box(30, 30, 20)
    result = rectangular_pocket_cut(stock, width=12, length=16, depth=5, corner_radius=2, base_height=15)
    assert result.val().isInside(cq.Vector(0, 0, 4))
    assert not result.val().isInside(cq.Vector(0, 0, 8))
    assert stock.val().Volume() - result.val().Volume() == pytest.approx((192 - (4 - math.pi) * 4) * 5)


@pytest.mark.parametrize("radius", [0, 5])
def test_zero_radius_and_half_width_radius_are_valid_profiles(radius):
    stock = cq.Workplane("XY").box(30, 30, 20)
    result = rectangular_pocket_cut(stock, width=10, length=20, depth=3, corner_radius=radius)
    assert stock.val().Volume() - result.val().Volume() == pytest.approx((200 - (4 - math.pi) * radius ** 2) * 3)


@pytest.mark.parametrize("changes", [{"corner_radius": 11}, {"depth": -1}, {"width": float("nan")},
    {"cx": float("inf")}, {"depth": True}, {"base_height": 50}, {"face_selector": "unknown"}])
def test_invalid_geometry_does_not_return_a_square_or_unchanged_pocket(changes):
    stock = cq.Workplane("XY").box(30, 30, 20)
    params = {"width": 20, "length": 20, "depth": 5, "corner_radius": 2, **changes}
    with pytest.raises(ValueError):
        rectangular_pocket_cut(stock, **params)
