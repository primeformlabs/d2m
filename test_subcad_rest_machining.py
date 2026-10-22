"""Rest machining of a profile pocket: a smaller cutter cuts only what a larger cutter left (D2O #748)."""

import math

import pytest

from src.subcad import Stock, ToolSpec, tool_inventory

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None


pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

BIG = ToolSpec("flat_endmill", 16, catalog_id="EM16", physical_assembly={"owned": True})
SMALL = ToolSpec("flat_endmill", 4, catalog_id="EM4", physical_assembly={"owned": True})
POCKET = {"type": "rectangle", "length": 44, "width": 24}


def _path_length(operation) -> float:
    moves = (operation.get("toolpath") or {}).get("moves") or []
    return sum(math.dist(first["position"], second["position"])
               for first, second in zip(moves, moves[1:]) if second["type"] in {"feed", "plunge"})


def _last(stock: Stock):
    return stock.process_plan()["operations"][-1]


def test_rest_pass_cuts_the_corner_that_the_larger_cutter_left() -> None:
    with tool_inventory([BIG, SMALL], revision="rest-test", strict=True):
        stock = Stock.rectangular(60, 40, 20)
        top = stock._shape.val().BoundingBox().zmax
        corner = cq.Vector(-21.2, -11.2, top - 3)  # 0.8 mm from both walls of the pocket corner
        roughed = stock.profile_pocket(POCKET, 6, finish_allowance=0)
        assert _last(roughed)["tool"]["diameter_mm"] == 16
        assert roughed._shape.val().isInside(corner)

        rested = roughed.profile_pocket(POCKET, 6, finish_allowance=0, max_tool_diameter=4, rest_from_diameter=16)
        assert _last(rested)["tool"]["diameter_mm"] == 4
        assert not rested._shape.val().isInside(corner)
        # The corner keeps the 4 mm cutter radius: a point 0.3 mm from both walls stays material.
        assert rested._shape.val().isInside(cq.Vector(-21.7, -11.7, top - 3))


def test_rest_pass_is_much_shorter_than_a_full_clear_with_the_small_cutter() -> None:
    with tool_inventory([BIG, SMALL], revision="rest-test", strict=True):
        stock = Stock.rectangular(60, 40, 20)
        rest = _last(stock.profile_pocket(POCKET, 6, finish_allowance=0)
                     .profile_pocket(POCKET, 6, finish_allowance=0, max_tool_diameter=4, rest_from_diameter=16))
        full = _last(stock.profile_pocket(POCKET, 6, finish_allowance=0, max_tool_diameter=4))
        assert full["tool"]["diameter_mm"] == 4
        assert 0 < _path_length(rest) < 0.35 * _path_length(full)


def test_rest_pass_with_a_larger_cutter_is_refused() -> None:
    with tool_inventory([BIG, SMALL], revision="rest-test", strict=True):
        roughed = Stock.rectangular(60, 40, 20).profile_pocket(POCKET, 6, finish_allowance=0, max_tool_diameter=4)
        with pytest.raises(ValueError, match="smaller than the previous cutter"):
            roughed.profile_pocket(POCKET, 6, finish_allowance=0, tool=BIG, rest_from_diameter=4)


def test_rest_pass_with_no_material_left_has_no_moves() -> None:
    circle = {"type": "circle", "diameter": 30}
    with tool_inventory([BIG, SMALL], revision="rest-test", strict=True):
        roughed = Stock.rectangular(60, 40, 20).profile_pocket(circle, 6, finish_allowance=0)
        rested = roughed.profile_pocket(circle, 6, finish_allowance=0, max_tool_diameter=4, rest_from_diameter=16)
        rest = _last(rested)
        assert rest["tool"]["diameter_mm"] == 4
        assert rest["material_passes"] == []
        assert _path_length(rest) == 0
        assert rested.volume == pytest.approx(roughed.volume, rel=1e-9)


L_PROFILE = {"type": "polygon", "points": [[-30, -20], [30, -20], [30, 0], [0, 0], [0, 20], [-30, 20]]}


def test_outside_rest_pass_cuts_the_concave_corner_of_the_outer_profile() -> None:
    with tool_inventory([BIG, SMALL], revision="rest-test", strict=True):
        stock = Stock.rectangular(60, 40, 10)
        middle = stock._shape.val().BoundingBox().center.z
        roughed = stock.profile_cutout(L_PROFILE, through=True, finish_allowance=0, max_tool_diameter=16)
        assert _last(roughed)["tool"]["diameter_mm"] == 16
        assert roughed._shape.val().isInside(cq.Vector(1, 1, middle))
        assert not roughed._shape.val().isInside(cq.Vector(20, 10, middle))

        rested = roughed.profile_cutout(L_PROFILE, through=True, finish_allowance=0, max_tool_diameter=4,
                                        rest_from_diameter=16)
        rest = _last(rested)
        assert rest["tool"]["diameter_mm"] == 4
        assert not rested._shape.val().isInside(cq.Vector(1, 1, middle))
        # The corner keeps the 4 mm cutter radius.
        assert rested._shape.val().isInside(cq.Vector(0.3, 0.3, middle))
        # Only the corner: no pass along the whole outline, and no cut in the air outside the stock.
        full = _last(stock.profile_cutout(L_PROFILE, through=True, finish_allowance=0, max_tool_diameter=4))
        assert 0 < _path_length(rest) < 0.35 * _path_length(full)
        assert all(abs(move["position"][0]) <= 30 + 2 + 1e-6 and abs(move["position"][1]) <= 20 + 2 + 1e-6
                   for move in rest["toolpath"]["moves"] if move["type"] in {"feed", "plunge"})
