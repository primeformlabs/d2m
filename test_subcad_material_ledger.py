"""The material ledger (material_ledger.py): the layers of each operation come from 2D cuts, not from the B-Rep chain."""

import math

import pytest

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - environment dependent
    cq = None

pytestmark = pytest.mark.skipif(cq is None, reason="CadQuery is required")

if cq is not None:
    from src.subcad import Stock, ToolSpec, tool_inventory
    from src.subcad import material_ledger as ML
    from src.subcad import milling_replay
    from src.subcad.profile_milling import _setup_plane


def _owned(diameter, flute, catalog_id):
    return ToolSpec("flat_endmill", diameter, flute_length=flute, reach_mm=flute + 15, stickout_mm=flute + 15,
                    num_flutes=4, catalog_id=catalog_id, physical_assembly={"owned": True})


TOOLS = [_owned(6, 20, "EM6"), _owned(4, 12, "EM4")]


def _ledger_volume(stock):
    ledger = ML.MaterialLedger.of(stock._shape)
    volume, _ = ledger.volume_in(ML.plane_frame(_setup_plane(">Z")[0]))
    return ledger, volume


def _passes(stock):
    return stock.process_plan()["operations"][-1]["material_passes"]


def test_the_ledger_holds_the_cuts_of_every_setup_frame():
    with tool_inventory(TOOLS, revision="ledger-test", strict=True):
        stock = Stock.rectangular(40, 30, 12)
        steps = [
            lambda s: s.profile_pocket({"type": "rectangle", "length": 20, "width": 14}, 4, finish_allowance=0),
            lambda s: s.bore(cx=8, cy=0, diameter=10, depth=0, through=True, face_selector="<Z"),
            # A crossing tool axis: the section of a >X cut in a >Z layer is a set of rectangles.
            lambda s: s.profile_pocket({"type": "rectangle", "length": 20, "width": 6}, 3, face_selector=">X",
                                       finish_allowance=0),
        ]
        cuts = 0
        for step in steps:
            stock = step(stock)
            ledger, volume = _ledger_volume(stock)
            assert len(ledger.cuts) > cuts
            cuts = len(ledger.cuts)
            assert volume == pytest.approx(stock._shape.val().Volume(), rel=2e-5)


def test_a_cut_that_the_brep_cannot_take_keeps_the_layer_material(monkeypatch):
    """The chained Boolean fails (as on coincident walls of two cutter sweeps): the operation keeps its layer
    result, the B-Rep comes from the ledger layers and the next operation needs no Boolean on it."""
    with tool_inventory(TOOLS, revision="ledger-test", strict=True):
        roughed = Stock.rectangular(40, 30, 12).profile_pocket({"type": "rectangle", "length": 20, "width": 14}, 4,
                                                                finish_allowance=0)

        def refused(*args, **kwargs):
            raise ValueError("The native Milling Boolean operation failed.")
        monkeypatch.setattr(milling_replay, "_boolean", refused)
        holed = roughed.bore(cx=0, cy=0, diameter=10, depth=0, through=True, face_selector="<Z")
        passes = _passes(holed)
        assert passes[-1]["material_brep"] == "layer_prisms"
        ledger, volume = _ledger_volume(holed)
        assert ledger.brep_from_layers
        removed = sum(row["removed_volume_mm3"] for row in passes)
        # The circular path has chords (0.01 mm): a thin crescent stays on the hole wall.
        assert removed == pytest.approx(math.pi * 25 * 8, rel=5e-3)
        assert holed._shape.val().Volume() == pytest.approx(volume, rel=1e-4)
        # The layer solid has the 2 µm outline of the player (layer_player.DISPLAY_TOLERANCE_MM).
        assert roughed._shape.val().Volume() - holed._shape.val().Volume() == pytest.approx(removed, rel=3e-3)

        slotted = holed.profile_pocket({"type": "rectangle", "length": 6, "width": 6, "cx": 14, "cy": 0}, 2,
                                       finish_allowance=0)
        assert _passes(slotted)[-1]["material_brep"] == "layer_prisms"
        _, after = _ledger_volume(slotted)
        # The D4 cutter leaves its radius in the four sharp corners.
        assert volume - after == pytest.approx(36 * 2 - 4 * 4 * (1 - math.pi / 4) * 2, rel=1e-3)
