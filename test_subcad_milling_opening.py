"""The milling verdict by the morphological opening (milling_opening.py) and the player bands (layer_player.py)."""
import math

import cadquery as cq
import pytest

from subcad import layers as L
from subcad import milling_opening as M

STOCK = cq.Workplane().box(100, 60, 20).val()


def flat(diameter, flute=30.0, **values):
    return {"tool_type": "flat_endmill", "diameter_mm": diameter, "flute_length_mm": flute, **values}


def corner(radius):
    return radius * radius * (1 - math.pi / 4)


def left(part, setups, cutters):
    return M.part_verdict(STOCK, part, setups, cutters)


@pytest.mark.parametrize("diameter", [10.0, 6.0, 3.0])
def test_flat_cutter_leaves_its_corners_and_narrow_slots(diameter):
    part = (cq.Workplane().box(100, 60, 20).faces(">Z").workplane().rect(40, 30).cutBlind(-8)
            .faces(">Z").workplane().center(35, 0).rect(4, 40).cutBlind(-5)).val()
    result = left(part, [">Z"], [flat(diameter)])
    r = diameter / 2
    slot = 4 * 40 * 5 if diameter > 4 else 4 * corner(r) * 5
    expected = 4 * corner(r) * 8 + slot
    assert result["setups"][0]["material_to_remove_mm3"] == pytest.approx(STOCK.Volume() - part.Volume(), rel=1e-6)
    # The cutter centre fits 2 µm and the disk reaches the 1 µm mesh chord: the corners are 3 µm smaller.
    assert result["left_after_all_setups_mm3"] == pytest.approx(expected, rel=5e-3)


def test_ball_and_bull_nose_leave_their_floor_fillets():
    part = cq.Workplane().box(100, 60, 20).faces(">Z").workplane().rect(40, 30).cutBlind(-8).val()
    r = 3.0
    straight = 2 * (40 + 30) - 8 * r
    # Ball: wall fillet c²(1 - π/4) along the straight walls, the vertical corners above c, and at each corner
    # the cube corner r³ less one eighth of the ball.
    ball = corner(r) * straight + 4 * corner(r) * (8 - r) + 4 * (r ** 3 - math.pi * r ** 3 / 6)
    result = left(part, [">Z"], [{"tool_type": "ball_endmill", "diameter_mm": 6.0, "flute_length_mm": 30}])
    assert result["left_after_all_setups_mm3"] == pytest.approx(ball, rel=0.03)
    # Bull nose r 3, corner 1: the corner at height h has the radius 2 + sqrt(1 - (1 - h)²).
    c = 1.0
    integral = 4 + math.pi + 2 / 3  # ∫ ρ(h)² dh from 0 to 1
    bull = corner(c) * straight + 4 * corner(r) * (8 - c) + 4 * (r * r * c - math.pi / 4 * integral)
    result = left(part, [">Z"], [{"tool_type": "bull_nose", "diameter_mm": 6.0, "corner_radius_mm": 1.0,
                                  "flute_length_mm": 30}])
    assert result["left_after_all_setups_mm3"] == pytest.approx(bull, rel=0.03)


def test_a_cutter_reaches_a_hole_of_its_own_size_and_not_a_smaller_one():
    part = cq.Workplane().box(100, 60, 20).faces(">Z").workplane().hole(8).val()
    assert left(part, [">Z", "<Z"], [flat(8.0)])["left_after_all_setups_mm3"] == pytest.approx(0, abs=1e-3)
    hole = math.pi * 16 * 20
    assert left(part, [">Z", "<Z"], [flat(10.0)])["left_after_all_setups_mm3"] == pytest.approx(hole, rel=1e-3)


def test_reach_depth_and_the_second_setup():
    # A 12 mm deep pocket: a cutter with 10 mm of flutes leaves the bottom 2 mm from the top setup.
    part = cq.Workplane().box(100, 60, 20).faces(">Z").workplane().rect(40, 30).cutBlind(-12).val()
    result = left(part, [">Z"], [flat(6.0, flute=10.0, stickout=40.0)])
    assert result["left_after_all_setups_mm3"] == pytest.approx(40 * 30 * 2 + 4 * corner(3.0) * 10, rel=1e-3)
    # An undercut: a step under an overhang is left from >Z and reached from <Z.
    under = (cq.Workplane().box(100, 60, 20)
             .cut(cq.Workplane().box(20, 60, 10).translate((40, 0, -5)))).val()
    from_top = left(under, [">Z"], [flat(10.0)])["left_after_all_setups_mm3"]
    both = left(under, [">Z", "<Z"], [flat(10.0)])["left_after_all_setups_mm3"]
    assert from_top == pytest.approx(20 * 60 * 10, rel=1e-3)
    assert both == pytest.approx(0, abs=1e-3)


def test_a_side_setup_combines_with_the_top_setup():
    # A slot through the part along X, 4 mm wide: D10 cannot reach it from >Z; from >X it can (4 mm deep, 10 high).
    part = (cq.Workplane().box(100, 60, 20).cut(cq.Workplane().box(100, 4, 10).translate((0, 0, 5)))).val()
    top = left(part, [">Z"], [flat(10.0)])["left_after_all_setups_mm3"]
    # At each open end, the cutter outside the stock reaches into the slot by the disk segment past the corners.
    steps = 20000
    segment = sum(math.sqrt(25 - y * y) - math.sqrt(21) for y in
                  (-2 + 4 * (k + 0.5) / steps for k in range(steps))) * 4 / steps
    assert top == pytest.approx(100 * 4 * 10 - 2 * segment * 10, rel=1e-4)
    both = left(part, [">Z", ">X"], [flat(4.0, flute=120.0, stickout=120.0)])
    assert both["left_after_all_setups_mm3"] == pytest.approx(0, abs=1e-2)


def test_player_bands_hold_the_removed_volume_of_each_pass():
    from subcad import Stock, milling_layers
    from subcad.layer_player import UNITS_PER_MM
    saved = milling_layers.PLAYER_BREP_STATES
    milling_layers.PLAYER_BREP_STATES = False
    try:
        stock = Stock.rectangular(80, 60, 20).profile_pocket({"type": "rectangle", "length": 40, "width": 30,
                                                              "corner_radius": 5}, 6)
    finally:
        milling_layers.PLAYER_BREP_STATES = saved
    passes = [state for state in stock._state_history if state.get("player_layers")]
    assert passes and passes[-1]["shape"] is not None
    assert all(state["shape"] is None for state in passes[:-1])
    scale = L.SCALE // UNITS_PER_MM
    total = reported = 0.0
    for state in passes:
        volume = 0.0
        for band in state["player_layers"]["removed"]:
            paths = [[(loop[i] * scale, loop[i + 1] * scale) for i in range(0, len(loop), 2)] for loop in band["loops"]]
            volume += L.area(paths) * (band["z"][1] - band["z"][0])
        # Integer µm coordinates and the 2 µm outline tolerance change a thin finishing crescent by a few percent.
        assert volume == pytest.approx(state["geometry_replay"]["removed_volume_mm3"], rel=3e-2, abs=1e-3)
        total += volume
        reported += state["geometry_replay"]["removed_volume_mm3"]
    assert total == pytest.approx(reported, rel=2e-3)


def test_drill_plunges_make_an_open_groove_and_leave_the_cusps():
    # A half-round groove D6.5 through the part at its edge x = 50, and a slot of two plunges 5 mm apart.
    groove = cq.Workplane().box(100, 60, 20).cut(cq.Workplane().cylinder(30, 3.25).translate((50, 0, 0)))
    part = groove.cut(cq.Workplane().cylinder(30, 3.25).translate((-20, 0, 0)))
    part = part.cut(cq.Workplane().cylinder(30, 3.25).translate((-15, 0, 0)))
    part = part.cut(cq.Workplane().box(5, 6.5, 30).translate((-17.5, 0, 0))).val()
    drill = {"tool_type": "drill", "diameter_mm": 6.5, "tip_angle_deg": 118, "flute_length_mm": 50, "stickout_mm": 50}
    groove_volume = math.pi * 3.25 ** 2 * 20 / 2
    slot_volume = (math.pi * 3.25 ** 2 + 5 * 6.5) * 20
    # Without plunge points a drill makes only closed holes of its size: the open groove and the slot stay.
    plain = left(part, [">Z"], [drill])["left_after_all_setups_mm3"]
    assert plain == pytest.approx(groove_volume + slot_volume, rel=1e-3)
    # With the plunge points of the plan: the groove is made; the slot keeps the two cusps between the plunges.
    plunged = left(part, [">Z"], [{**drill, "plunges": [[50, 0, 0], [-20, 0, 0], [-15, 0, 0]]}])
    half = 2.5
    # The slot region between the plunge axes (x from -20 to -15) less the two disks: two cusps at y = ±3.25.
    between = 5 * 6.5 - (math.pi * 3.25 ** 2 - 2 * (3.25 ** 2 * math.acos(half / 3.25) - half * math.sqrt(3.25 ** 2 - half ** 2)))
    assert plunged["left_after_all_setups_mm3"] == pytest.approx(between * 20, rel=2e-2)
